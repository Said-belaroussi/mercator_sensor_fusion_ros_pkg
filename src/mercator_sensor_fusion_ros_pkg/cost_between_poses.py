#!/usr/bin/env python

import rospy
import rosbag
from geometry_msgs.msg import PoseArray
from scipy.optimize import linear_sum_assignment
import numpy as np
import threading
import matplotlib.pyplot as plt
import os


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

        self.max_shift_messages = rospy.get_param('~max_shift_messages', 20)  # Max shift in number of messages
        self.shift_step = rospy.get_param('~shift_step', 1)
        self.time_tolerance = rospy.get_param('~time_tolerance', 0.2)  # Tolerance in seconds for syncing messages

        self.experiment_timestamp_x_y_deviation = np.array([]).reshape(0, 3)
        self.cam_timestamp_x_y_deviation = np.array([]).reshape(0, 3)
        self.lidar_timestamp_x_y_deviation = np.array([]).reshape(0, 3)

        self.experiment_polar_poses_with_cost = np.array([]).reshape(0, 3)
        self.cam_polar_poses_with_cost = np.array([]).reshape(0, 3)
        self.lidar_polar_poses_with_cost = np.array([]).reshape(0, 3)

        self.read_bags()

    def read_bags(self):
        # Open rosbag files
        experiment_bag = rosbag.Bag(self.experiment_bag_path, 'r')
        ground_truth_bag = rosbag.Bag(self.ground_truth_bag_path, 'r')

        # Read all messages from the bags
        experiment_messages = list(experiment_bag.read_messages(topics=[self.fused_poses_topic, '/cam_poses_transformed', '/lidar_poses_transformed']))
        ground_truth_messages = list(ground_truth_bag.read_messages(topics=['/ground_truth_poses']))


        experiment_messages_to_time = list(experiment_bag.read_messages(topics=[self.fused_poses_topic]))
        cam_messages_to_time = list(experiment_bag.read_messages(topics=['/cam_poses_transformed']))
        lidar_messages_to_time = list(experiment_bag.read_messages(topics=['/lidar_poses_transformed']))

        first_experiment_time = None
        first_ground_truth_time = None

        # Get the first timestamps
        rospy.loginfo(len(experiment_messages))
        rospy.loginfo(len(ground_truth_messages))
        first_experiment_time = experiment_messages_to_time[0][2].to_sec()
        first_cam_time = cam_messages_to_time[0][2].to_sec()
        first_lidar_time = lidar_messages_to_time[0][2].to_sec()

        first_ground_truth_time = ground_truth_messages[0][2].to_sec()
        rospy.loginfo(first_experiment_time)
        rospy.loginfo(first_ground_truth_time)

        # Compute the time offset between the two bags
        time_offset_experiment = first_experiment_time - first_ground_truth_time
        time_offset_cam = first_cam_time - first_ground_truth_time
        time_offset_lidar = first_lidar_time - first_ground_truth_time


        for topic, msg, t in ground_truth_messages:
            self.ground_truth_callback(msg, t.to_sec())

        # Process each message in order of timestamp
        for topic, msg, t in experiment_messages:
            if topic == self.fused_poses_topic:
                adjusted_time = t.to_sec() - time_offset_experiment
                self.experiment_callback(msg, adjusted_time)
            elif topic == '/cam_poses_transformed':
                adjusted_time = t.to_sec() - time_offset_cam
                self.cam_callback(msg, adjusted_time)
            elif topic == '/lidar_poses_transformed':
                adjusted_time = t.to_sec() - time_offset_lidar
                self.lidar_callback(msg, adjusted_time)

        experiment_bag.close()
        ground_truth_bag.close()

        # After processing all messages, compute costs
        self.compute_costs()

    def experiment_callback(self, msg, timestamp):
        gt_pose = self.find_closest_ground_truth(timestamp, self.ground_truth_buffer)
        if gt_pose and msg.header.frame_id == 'robot':
            self.experiment_buffer.append((timestamp, msg.poses))
            self.ground_truth_buffer_for_experiment.append((timestamp, gt_pose))

    def cam_callback(self, msg, timestamp):
        gt_pose = self.find_closest_ground_truth(timestamp, self.ground_truth_buffer)
        if gt_pose and msg.header.frame_id == 'robot':
            self.cam_buffer.append((timestamp, msg.poses))
            self.ground_truth_buffer_for_cam.append((timestamp, gt_pose))

    def lidar_callback(self, msg, timestamp):
        gt_pose = self.find_closest_ground_truth(timestamp, self.ground_truth_buffer)
        if gt_pose and msg.header.frame_id == 'robot':
            self.lidar_buffer.append((timestamp, msg.poses))
            self.ground_truth_buffer_for_lidar.append((timestamp, gt_pose))

            # Write both lidar and ground truth poses to a csv file
            with open('../lidar_poses.csv', 'a') as f:
                for pose in msg.poses:
                    f.write(f'{timestamp},{pose.position.x},{pose.position.y}\n')
            with open('../ground_truth_poses.csv', 'a') as f:
                for pose in gt_pose:
                    f.write(f'{timestamp},{pose.position.x},{pose.position.y}\n')

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
        self.plot_all_in_one_figure()

    def plot_all_in_one_figure(self):
        # Create a single figure with 5 subplots (2 rows, 3 columns)
        fig, axs = plt.subplots(2, 3, figsize=(18, 10))

        # Define the topics and data to be plotted
        topics_and_data = [
            (self.experiment_polar_poses_with_cost, 'Fused Poses'),
            (self.cam_polar_poses_with_cost, 'Cam Poses'),
            (self.lidar_polar_poses_with_cost, 'Lidar Poses')
        ]

        colors = ['blue', 'orange', 'green']  # Colors for each topic
        offsets = [-0.02, 0, 0.02]  # Small offsets for each topic to separate the sticks

        # Plot cumulative histograms of errors (first row, first three columns)
        for i, (polar_poses_with_cost, label) in enumerate(topics_and_data):
            col = i  # Determine the column index (0, 1, or 2)
            
            # Cumulative histogram
            axs[0, col].hist(polar_poses_with_cost[:, 2], bins=100, cumulative=True, density=True)
            axs[0, col].set(xlabel='Error (m)', ylabel='Cumulative frequency', title=f'Cumulative histogram of {label} errors')
            
            # Draw line at 67% of poses and display the corresponding error
            error_threshold = np.percentile(polar_poses_with_cost[:, 2], 67)
            axs[0, col].axvline(x=error_threshold, color='r', linestyle='--')
            axs[0, col].text(error_threshold, 0.5, f'67% < {error_threshold:.2f} m')

        # Plot errors in function of distance (second row, first column)
        for i, (polar_poses_with_cost, label) in enumerate(topics_and_data):
            # Separate distance values into bins of 0.1 m and calculate the average error in each bin
            bins = np.arange(0, np.ceil(polar_poses_with_cost[:, 0].max()), 0.1)
            bin_indices = np.digitize(polar_poses_with_cost[:, 0], bins)
            bin_avg_errors = np.zeros(len(bins) - 1)
            for j in range(1, len(bins)):
                bin_avg_errors[j - 1] = np.mean(polar_poses_with_cost[bin_indices == j, 2])

            # Plot the average error as a function of distance with a small offset
            axs[1, 0].vlines(bins[:-1] + offsets[i], 0, bin_avg_errors, colors=colors[i], lw=2, label=label)
        
        axs[1, 0].set(title='Average error over distance to target', xlabel='Distance (m)', ylabel='Average error (m)')
        axs[1, 0].grid()
        axs[1, 0].legend()

        # Plot errors in function of angle (second row, second column)
        for i, (polar_poses_with_cost, label) in enumerate(topics_and_data):
            # Separate angle values into bins of 5 degrees and calculate the average error in each bin
            bins = np.arange(-180, 180, 5)
            bin_indices = np.digitize(np.degrees(polar_poses_with_cost[:, 1]), bins)
            bin_avg_errors = np.zeros(len(bins) - 1)
            for j in range(1, len(bins)):
                bin_avg_errors[j - 1] = np.mean(polar_poses_with_cost[bin_indices == j, 2])

            # Plot the average error as a function of angle with a small offset
            axs[1, 1].vlines(bins[:-1] + offsets[i]*40, 0, bin_avg_errors, colors=colors[i], lw=2, label=label)
        
        axs[1, 1].set(title='Average error over angle to target',xlabel='Angle (degrees)', ylabel='Average error (m)')
        axs[1, 1].grid()
        axs[1, 1].legend()

        # Remove the last subplot (second row, third column)
        fig.delaxes(axs[1, 2])

        # Adjust layout to provide more space between plots
        plt.subplots_adjust(hspace=0.4, wspace=0.2)

        # Save the plot
        bag_name = self.experiment_bag_path.split('/')[-1].split('.')[0]
        plt.savefig(f'../01result_images/{bag_name}_error_analysis.png', format="png", bbox_inches='tight')

        rospy.loginfo(os.getcwd())

        # Show the figure
        plt.show()

        plt.close()

    def plot_x_y_deviations_for_all(self):
        # Offset timestamps to start at 0
        self.experiment_timestamp_x_y_deviation[:, 2] -= self.experiment_timestamp_x_y_deviation[0, 2]
        self.cam_timestamp_x_y_deviation[:, 2] -= self.cam_timestamp_x_y_deviation[0, 2]
        self.lidar_timestamp_x_y_deviation[:, 2] -= self.lidar_timestamp_x_y_deviation[0, 2]
        self.plot_x_and_y_deviation_for_all()


    def plot_x_and_y_deviation_for_all(self):
        # Define the maximum time gap (in seconds) that you consider to be continuous
        max_time_gap = 0.5  # Adjust this threshold based on your specific data

        # Create a figure with 8 subplots (4 rows, 2 columns)
        fig, axs = plt.subplots(4, 2, figsize=(14, 16))

        def plot_with_gaps(ax, times, deviations, color, label):
            # Identify where the time gaps are larger than the threshold
            time_diffs = np.diff(times)
            gaps = np.where(time_diffs > max_time_gap)[0]

            # Split the data into segments based on identified gaps
            segments = np.split(np.arange(len(times)), gaps + 1)

            for segment in segments:
                ax.plot(times[segment], deviations[segment], color=color)
            
            # Add a single invisible plot to handle the legend
            ax.plot([], [], color=color, label=label)

        # Plot X deviations with gaps (first plot)
        plot_with_gaps(axs[0, 0], self.experiment_timestamp_x_y_deviation[:, 2], self.experiment_timestamp_x_y_deviation[:, 0], color='blue', label='Fused Poses')
        plot_with_gaps(axs[0, 0], self.cam_timestamp_x_y_deviation[:, 2], self.cam_timestamp_x_y_deviation[:, 0], color='orange', label='Cam Poses')
        plot_with_gaps(axs[0, 0], self.lidar_timestamp_x_y_deviation[:, 2], self.lidar_timestamp_x_y_deviation[:, 0], color='green', label='Lidar Poses')
        axs[0, 0].set(title='X Deviation over time for fused and sensors poses', xlabel='Time (s)', ylabel='X Deviation (m)')
        axs[0, 0].legend(fontsize=5)

        # Plot Y deviations with gaps (second plot)
        plot_with_gaps(axs[0, 1], self.experiment_timestamp_x_y_deviation[:, 2], self.experiment_timestamp_x_y_deviation[:, 1], color='blue', label='Fused Poses')
        plot_with_gaps(axs[0, 1], self.cam_timestamp_x_y_deviation[:, 2], self.cam_timestamp_x_y_deviation[:, 1], color='orange', label='Cam Poses')
        plot_with_gaps(axs[0, 1], self.lidar_timestamp_x_y_deviation[:, 2], self.lidar_timestamp_x_y_deviation[:, 1], color='green', label='Lidar Poses')
        axs[0, 1].set(title='Y Deviation over time for fused and sensors poses', xlabel='Time (s)', ylabel='Y Deviation (m)')
        axs[0, 1].legend(fontsize=5)

        # Define colors for each topic
        colors = ['blue', 'orange', 'green']

        # Plot histograms for X deviations (third row)
        data_sets = [
            (self.experiment_timestamp_x_y_deviation[:, 0], 'Fused Poses'),
            (self.cam_timestamp_x_y_deviation[:, 0], 'Cam Poses'),
            (self.lidar_timestamp_x_y_deviation[:, 0], 'Lidar Poses')
        ]

        # Set a symmetric bin range around 0 for X and Y deviations
        bin_range = np.linspace(-np.ceil(max(abs(self.experiment_timestamp_x_y_deviation[:, 0].max()), abs(self.experiment_timestamp_x_y_deviation[:, 0].min()))), 
                                np.ceil(max(abs(self.experiment_timestamp_x_y_deviation[:, 0].max()), abs(self.experiment_timestamp_x_y_deviation[:, 0].min()))), 
                                50)

        for i, (data, label) in enumerate(data_sets):
            counts, _ = np.histogram(data, bins=bin_range)
            probabilities = counts / np.sum(counts)  # Normalize to get probabilities
            axs[i+1, 0].bar(bin_range[:-1], probabilities, width=np.diff(bin_range), color=colors[i], alpha=0.7, edgecolor='black')
            mean = np.mean(data)
            std_dev = np.std(data)
            axs[i+1, 0].axvline(mean, color='red', linestyle='dashed', linewidth=2)
            axs[i+1, 0].text(mean + 0.05, np.max(probabilities) * 0.5, f'Mean: {mean:.2f}\nSD: {std_dev:.2f}', color='red')
            axs[i+1, 0].set(title=f'X Deviation Distribution - {label}', xlabel='X Deviation (m)', ylabel='Frequency')
            axs[i+1, 0].set_xlim([bin_range.min(), bin_range.max()])  # Center the histogram around 0

        # Plot histograms for Y deviations (fourth row)
        data_sets = [
            (self.experiment_timestamp_x_y_deviation[:, 1], 'Fused Poses'),
            (self.cam_timestamp_x_y_deviation[:, 1], 'Cam Poses'),
            (self.lidar_timestamp_x_y_deviation[:, 1], 'Lidar Poses')
        ]

        # Set a symmetric bin range around 0 for Y deviations
        bin_range = np.linspace(-np.ceil(max(abs(self.experiment_timestamp_x_y_deviation[:, 1].max()), abs(self.experiment_timestamp_x_y_deviation[:, 1].min()))), 
                                np.ceil(max(abs(self.experiment_timestamp_x_y_deviation[:, 1].max()), abs(self.experiment_timestamp_x_y_deviation[:, 1].min()))), 
                                50)

        for i, (data, label) in enumerate(data_sets):
            counts, _ = np.histogram(data, bins=bin_range)
            probabilities = counts / np.sum(counts)  # Normalize to get probabilities
            axs[i+1, 1].bar(bin_range[:-1], probabilities, width=np.diff(bin_range), color=colors[i], alpha=0.7, edgecolor='black')
            mean = np.mean(data)
            std_dev = np.std(data)
            axs[i+1, 1].axvline(mean, color='red', linestyle='dashed', linewidth=2)
            axs[i+1, 1].text(mean + 0.05, np.max(probabilities) * 0.5, f'Mean: {mean:.2f}\nSD: {std_dev:.2f}', color='red')
            axs[i+1, 1].set(title=f'Y Deviation Distribution - {label}', xlabel='Y Deviation (m)', ylabel='Frequency')
            axs[i+1, 1].set_xlim([bin_range.min(), bin_range.max()])  # Center the histogram around 0

        # Adjust layout to provide more space between plots
        plt.subplots_adjust(hspace=0.6, wspace=0.2)

        # Save the figure before displaying it
        bag_name = self.experiment_bag_path.split('/')[-1].split('.')[0]
        plt.savefig(f'../01result_images/{bag_name}_xy_deviation_analysis.png', format="png", bbox_inches='tight')

        # Show the figure
        plt.show()

        plt.close()

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

                if avg_error < min_avg_error:
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

        if label == "/fused_poses":
            self.experiment_polar_poses_with_cost = polar_poses_with_cost
        elif label == "/cam_poses_transformed":
            self.cam_polar_poses_with_cost = polar_poses_with_cost
        elif label == "/lidar_poses_transformed":
            self.lidar_polar_poses_with_cost = polar_poses_with_cost

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
            # if theta < 0:
            #     theta = -theta
            polar_poses_with_cost[i] = [r, theta, cost]

        return polar_poses_with_cost

if __name__ == '__main__':
    # Parse command line arguments
    import argparse
    parser = argparse.ArgumentParser(description='Compute cost between poses')
    parser.add_argument('experiment_bag_path', type=str, help='Path to the rosbag file with experiment poses')
    parser.add_argument('ground_truth_bag_path', type=str, help='Path to the rosbag file with ground truth poses')
    args = parser.parse_args()
    node = CostBetweenPosesNode(args.experiment_bag_path, args.ground_truth_bag_path)
    node.run()
