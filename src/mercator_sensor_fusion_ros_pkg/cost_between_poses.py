#!/usr/bin/env python

import rospy
import rosbag
from geometry_msgs.msg import PoseArray
from scipy.optimize import linear_sum_assignment
import numpy as np
import threading
import matplotlib.pyplot as plt


class CostBetweenPosesNode:
    def __init__(self, experiment_bag_path='experiment_bag_path', ground_truth_bag_path='ground_truth_bag_path'):
        rospy.init_node('cost_between_poses_node', anonymous=True)

        # Get rosbag paths from ROS parameters
        self.experiment_bag_path = rospy.get_param('~experiment_bag_path', experiment_bag_path)
        self.ground_truth_bag_path = rospy.get_param('~ground_truth_bag_path', ground_truth_bag_path)
        self.fused_poses_topic = rospy.get_param('~fused_poses_topic', '/fused_posessss')
        rospy.loginfo(self.fused_poses_topic)

        # Buffers for poses and ground truth
        self.experiment_buffer = []
        self.cam_buffer = []
        self.lidar_buffer = []
        self.ground_truth_buffer = []
        self.ground_truth_buffer_for_experiment = []
        self.ground_truth_buffer_for_cam = []
        self.ground_truth_buffer_for_lidar = []

        self.max_shift_messages = rospy.get_param('~max_shift_messages', 0)  # Max shift in number of messages
        self.shift_step = rospy.get_param('~shift_step', 1)
        self.time_tolerance = rospy.get_param('~time_tolerance', 0.2)  # Tolerance in seconds for syncing messages

        self.experiment_timestamp_x_y_deviation = np.array([]).reshape(0, 3)
        self.cam_timestamp_x_y_deviation = np.array([]).reshape(0, 3)
        self.lidar_timestamp_x_y_deviation = np.array([]).reshape(0, 3)

        self.read_bags()

    def read_bags(self):
        # Open rosbag files
        experiment_bag = rosbag.Bag(self.experiment_bag_path, 'r')
        ground_truth_bag = rosbag.Bag(self.ground_truth_bag_path, 'r')

        # Read all messages from the bags
        experiment_messages = list(experiment_bag.read_messages(topics=[self.fused_poses_topic, '/cam_poses_transformed', '/lidar_poses_transformed']))
        ground_truth_messages = list(ground_truth_bag.read_messages(topics=['/ground_truth_poses']))

        first_experiment_time = None
        first_ground_truth_time = None

        # Get the first timestamps
        rospy.loginfo(len(experiment_messages))
        rospy.loginfo(len(ground_truth_messages))
        first_experiment_time = experiment_messages[0][2].to_sec()
        first_ground_truth_time = ground_truth_messages[0][2].to_sec()
        rospy.loginfo(first_experiment_time)
        rospy.loginfo(first_ground_truth_time)

        # Compute the time offset between the two bags
        time_offset = first_experiment_time - first_ground_truth_time

        for topic, msg, t in ground_truth_messages:
            self.ground_truth_callback(msg, t.to_sec())

        # Process each message in order of timestamp
        for topic, msg, t in experiment_messages:
            adjusted_time = t.to_sec() - time_offset
            if topic == self.fused_poses_topic:
                self.experiment_callback(msg, adjusted_time)
            elif topic == '/cam_poses_transformed':
                self.cam_callback(msg, adjusted_time)
            elif topic == '/lidar_poses_transformed':
                self.lidar_callback(msg, adjusted_time)

        experiment_bag.close()
        ground_truth_bag.close()

        # After processing all messages, compute costs
        self.compute_costs()

    def experiment_callback(self, msg, timestamp):
        gt_pose = self.find_closest_ground_truth(timestamp, self.ground_truth_buffer)
        if gt_pose:
            self.experiment_buffer.append((timestamp, msg.poses))
            self.ground_truth_buffer_for_experiment.append((timestamp, gt_pose))

    def cam_callback(self, msg, timestamp):
        gt_pose = self.find_closest_ground_truth(timestamp, self.ground_truth_buffer)
        if gt_pose:
            self.cam_buffer.append((timestamp, msg.poses))
            self.ground_truth_buffer_for_cam.append((timestamp, gt_pose))

    def lidar_callback(self, msg, timestamp):
        gt_pose = self.find_closest_ground_truth(timestamp, self.ground_truth_buffer)
        if gt_pose:
            self.lidar_buffer.append((timestamp, msg.poses))
            self.ground_truth_buffer_for_lidar.append((timestamp, gt_pose))

    def ground_truth_callback(self, msg, timestamp):
        # Append ground truth poses without filtering
        self.ground_truth_buffer.append((timestamp, msg.poses))

    def find_closest_ground_truth(self, timestamp, ground_truth_buffer):
        closest_pose = None
        min_time_diff = float('inf')

        for gt_time, gt_pose in ground_truth_buffer:
            time_diff = abs(gt_time - timestamp)
            if time_diff < self.time_tolerance and time_diff < min_time_diff:
                min_time_diff = time_diff
                closest_pose = gt_pose

        return closest_pose


    def compute_costs(self):
        # Compute costs for each pair of buffers
        rospy.loginfo(len(self.experiment_buffer))
        rospy.loginfo(len(self.cam_buffer))
        rospy.loginfo(len(self.lidar_buffer))
        rospy.loginfo(len(self.ground_truth_buffer_for_experiment))
        rospy.loginfo(len(self.ground_truth_buffer_for_cam))
        rospy.loginfo(len(self.ground_truth_buffer_for_lidar))

        self.compute_cost_for_pair(self.experiment_buffer, self.ground_truth_buffer_for_experiment, "/fused_poses")
        self.compute_cost_for_pair(self.cam_buffer, self.ground_truth_buffer_for_cam, "/cam_poses_transformed")
        self.compute_cost_for_pair(self.lidar_buffer, self.ground_truth_buffer_for_lidar, "/lidar_poses_transformed")

        self.plot_x_y_deviations_for_all()

    def plot_x_y_deviations_for_all(self):
        # Offset timestamps to start at 0
        self.experiment_timestamp_x_y_deviation[:, 2] -= self.experiment_timestamp_x_y_deviation[0, 2]
        self.cam_timestamp_x_y_deviation[:, 2] -= self.cam_timestamp_x_y_deviation[0, 2]
        self.lidar_timestamp_x_y_deviation[:, 2] -= self.lidar_timestamp_x_y_deviation[0, 2]
        self.plot_x_and_y_deviation_for_all()

    def plot_x_and_y_deviation_for_all(self):
        # Create a figure with two subplots (1 row, 2 columns)
        fig, axs = plt.subplots(2, 1, figsize=(10, 8))
        fig.suptitle('X and Y Deviation for all sensors in function of time')

        # Plot X deviations
        axs[0].plot(self.experiment_timestamp_x_y_deviation[:, 2], self.experiment_timestamp_x_y_deviation[:, 0], label='Fused Poses')
        axs[0].plot(self.cam_timestamp_x_y_deviation[:, 2], self.cam_timestamp_x_y_deviation[:, 0], label='Cam Poses')
        axs[0].plot(self.lidar_timestamp_x_y_deviation[:, 2], self.lidar_timestamp_x_y_deviation[:, 0], label='Lidar Poses')
        axs[0].set(xlabel='Time (s)', ylabel='X Deviation (m)')
        axs[0].legend()

        # Plot Y deviations
        axs[1].plot(self.experiment_timestamp_x_y_deviation[:, 2], self.experiment_timestamp_x_y_deviation[:, 1], label='Fused Poses')
        axs[1].plot(self.cam_timestamp_x_y_deviation[:, 2], self.cam_timestamp_x_y_deviation[:, 1], label='Cam Poses')
        axs[1].plot(self.lidar_timestamp_x_y_deviation[:, 2], self.lidar_timestamp_x_y_deviation[:, 1], label='Lidar Poses')
        axs[1].set(xlabel='Time (s)', ylabel='Y Deviation (m)')
        axs[1].legend()

        plt.tight_layout(rect=[0, 0, 1, 0.96])  # Adjust layout to fit the title

        # Save the figure
        bag_name = self.experiment_bag_path.split('/')[-1].split('.')[0]
        plt.savefig(f'{bag_name}_xy_deviation_all.png')

        plt.show()

    def compute_cost_for_pair(self, buffer_a, buffer_b, label):
        min_rmse = float('inf')
        original_rmse = float('inf')
        min_avg_error = float('inf')
        original_avg_error = float('inf')
        original_flag = False
        optimal_shift = 0
        perfect_delay = 0
        
        kept_total_poses_with_cost_array = np.array([]).reshape(0, 6)

        for shift in range(-self.max_shift_messages, self.max_shift_messages + 1, self.shift_step):
            if shift < 0:
                shifted_buffer_a = buffer_a[-shift:]
                shifted_buffer_b = buffer_b[:shift]
            elif shift > 0:
                shifted_buffer_a = buffer_a[:-shift]
                shifted_buffer_b = buffer_b[shift:]
            else:
                shifted_buffer_a = buffer_a
                shifted_buffer_b = buffer_b
                original_flag = True

            total_poses_with_cost_array = np.array([]).reshape(0, 6)

            for a_msg, b_msg in zip(shifted_buffer_a, shifted_buffer_b):
                poses_with_cost_array = self.calculate_cost(a_msg[1], b_msg[1])
                if poses_with_cost_array is not None:
                    poses_with_cost_array = np.hstack((poses_with_cost_array, np.full((poses_with_cost_array.shape[0], 1), a_msg[0])))
                    total_poses_with_cost_array = np.vstack((total_poses_with_cost_array, poses_with_cost_array))


            if len(total_poses_with_cost_array) > 0:
                # Compute Root Mean Squared Error
                rmse = np.sqrt(np.mean(np.square(total_poses_with_cost_array[:, 2])))    
                # Compute Average Error
                avg_error = np.mean(total_poses_with_cost_array[:, 2])        

                if rmse < min_rmse:
                    min_rmse = rmse
                    optimal_shift = shift
                    perfect_delay = shifted_buffer_a[0][0] - shifted_buffer_b[0][0]

                    min_avg_error = avg_error

                    kept_total_poses_with_cost_array = total_poses_with_cost_array

                if original_flag:
                    original_rmse = rmse
                    original_avg_error = avg_error
                    original_flag = False

        rospy.loginfo("[%s] Minimum RMSE: %f at shift: %d and delay %f", label, min_rmse, optimal_shift, perfect_delay)
        rospy.loginfo("[%s] Original RMSE: %f", label, original_rmse)
        rospy.loginfo("[%s] Minimum Average Error: %f", label, min_avg_error)
        rospy.loginfo("[%s] Original Average Error: %f", label, original_avg_error)

        if label == "/fused_poses":
            self.experiment_timestamp_x_y_deviation = kept_total_poses_with_cost_array[:, 3:]
        elif label == "/cam_poses_transformed":
            self.cam_timestamp_x_y_deviation = kept_total_poses_with_cost_array[:, 3:]
        elif label == "/lidar_poses_transformed":
            self.lidar_timestamp_x_y_deviation = kept_total_poses_with_cost_array[:, 3:]

        rospy.loginfo(kept_total_poses_with_cost_array.shape)
        polar_poses_with_cost = self.cartesian_to_polar_poses_with_cost(kept_total_poses_with_cost_array[:, :3])
        rospy.loginfo(polar_poses_with_cost.shape)

        # Plot all errors
        self.plot_all_same_plot(polar_poses_with_cost, label)

    def calculate_cost(self, poses_a, poses_b):
        poses_a = np.array([[pose.position.x, pose.position.y] for pose in poses_a])
        poses_b = np.array([[pose.position.x, pose.position.y] for pose in poses_b])

        if poses_a.shape[0] == 0 or poses_b.shape[0] == 0:
            return None

        cost_matrix = np.linalg.norm(poses_a[:, np.newaxis] - poses_b, axis=2)
        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        # for i in range(len(row_ind)):
        #     rospy.loginfo(poses_a[row_ind[i]])
        #     rospy.loginfo(cost_matrix[row_ind[i], col_ind[i]])
        poses_b_with_cost = np.array([[poses_b[col_ind[i]][0], poses_b[col_ind[i]][1], cost_matrix[row_ind[i], col_ind[i]]] for i in range(len(row_ind))])

        # Compute x and y deviations between poses
        deviations = poses_a[row_ind] - poses_b[col_ind]

        # Add deviation to poses_b_with_cost
        poses_b_with_cost = np.hstack((poses_b_with_cost, deviations))

        return poses_b_with_cost

    def cartesian_to_polar_poses_with_cost(self, poses_with_cost):
        polar_poses_with_cost = np.zeros((poses_with_cost.shape[0], 3))
        for i, pose in enumerate(poses_with_cost):
            x = pose[0]
            y = pose[1]
            cost = pose[2]
            r = np.sqrt(x**2 + y**2)
            theta = np.arctan2(y, x)
            # 0 radian angle is at the y-axis
            theta = theta - np.pi / 2
            if theta < -np.pi:
                theta += 2 * np.pi
            if theta < 0:
                theta = -theta
            polar_poses_with_cost[i] = [r, theta, cost]

        return polar_poses_with_cost

    def plot_all_same_plot(self, polar_poses_with_cost, topic_name):
        topic_name = topic_name[1:]

        bag_name = self.experiment_bag_path.split('/')[-1].split('.')[0]
        fig, axs = plt.subplots(3)
        fig.suptitle(f'Error on {topic_name} poses')

        # Compute cumulative histogram of errors values (on y-axis) on percentage of poses (on x-axis)
        axs[0].hist(polar_poses_with_cost[:, 2], bins=100, cumulative=True, density=True)
        axs[0].set(xlabel='Error (m)', ylabel='Percentage of poses')

        # Draw line at 67% of poses and display the corresponding error
        error_threshold = np.percentile(polar_poses_with_cost[:, 2], 67)
        axs[0].axvline(x=error_threshold, color='r', linestyle='--')
        axs[0].text(error_threshold, 0.5, f'67% < {error_threshold:.2f} m')

        # Separate distance values on bins of 0.1 m and plot the average error on each bin
        bins = np.arange(0, np.ceil(polar_poses_with_cost[:, 0].max()), 0.1)
        bin_indices = np.digitize(polar_poses_with_cost[:, 0], bins)
        bin_avg_errors = np.zeros(len(bins) - 1)
        for i in range(1, len(bins)):
            bin_avg_errors[i - 1] = np.mean(polar_poses_with_cost[bin_indices == i, 2])

        # Use vlines to create the stick plot for distance bins
        axs[1].vlines(bins[:-1], 0, bin_avg_errors, colors='b', lw=2, label='Error') 
        axs[1].set(xlabel='Distance (m)', ylabel='Average error (m)')
        axs[1].grid()

        # Separate angle values on bins of 10 degrees and plot the average error on each bin
        bins = np.arange(0, 180, 5)
        bin_indices = np.digitize(np.degrees(polar_poses_with_cost[:, 1]), bins)
        bin_avg_errors = np.zeros(len(bins) - 1)
        for i in range(1, len(bins)):
            bin_avg_errors[i - 1] = np.mean(polar_poses_with_cost[bin_indices == i, 2])

        # Use vlines for the stick plot for angle bins
        axs[2].vlines(bins[:-1], 0, bin_avg_errors, colors='b', lw=2, label='Error')
        axs[2].set(xlabel='Angle (degrees)', ylabel='Average error (m)')
        axs[2].grid()

        plt.show()

        # Save the plot
        plt.savefig(f'{bag_name}_{topic_name}_error_all.png')
        


if __name__ == '__main__':
    # Parse command line arguments
    import argparse
    parser = argparse.ArgumentParser(description='Compute cost between poses')
    parser.add_argument('experiment_bag_path', type=str, help='Path to the rosbag file with experiment poses')
    parser.add_argument('ground_truth_bag_path', type=str, help='Path to the rosbag file with ground truth poses')
    args = parser.parse_args()
    node = CostBetweenPosesNode(args.experiment_bag_path, args.ground_truth_bag_path)
    node.run()
