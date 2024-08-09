#!/usr/bin/env python

import rospy
import rosbag
from geometry_msgs.msg import PoseArray
from scipy.optimize import linear_sum_assignment
import numpy as np
import threading

class CostBetweenPosesNode:
    def __init__(self, experiment_bag_path='experiment_bag_path', ground_truth_bag_path='ground_truth_bag_path'):
        rospy.init_node('cost_between_poses_node', anonymous=True)

        # Get rosbag paths from ROS parameters
        self.experiment_bag_path = rospy.get_param('~experiment_bag_path', experiment_bag_path)
        self.ground_truth_bag_path = rospy.get_param('~ground_truth_bag_path', ground_truth_bag_path)

        # Buffers for poses and ground truth
        self.experiment_buffer = []
        self.cam_buffer = []
        self.lidar_buffer = []
        self.ground_truth_buffer_for_experiment = []
        self.ground_truth_buffer_for_cam = []
        self.ground_truth_buffer_for_lidar = []

        self.max_shift_messages = rospy.get_param('~max_shift_messages', 20)  # Max shift in number of messages
        self.shift_step = rospy.get_param('~shift_step', 1)
        self.time_tolerance = rospy.get_param('~time_tolerance', 0.05)  # Tolerance in seconds for syncing messages

        # Thread to read and process rosbag data
        self.read_bag_thread = threading.Thread(target=self.read_bags)
        self.read_bag_thread.start()

        self.run()

    def read_bags(self):
        # Open rosbag files
        experiment_bag = rosbag.Bag(self.experiment_bag_path, 'r')
        ground_truth_bag = rosbag.Bag(self.ground_truth_bag_path, 'r')

        # Read all messages from the bags
        experiment_messages = list(experiment_bag.read_messages(topics=['fused_poses_odom', 'cam_poses_transformed', 'lidar_poses_transformed']))
        ground_truth_messages = list(ground_truth_bag.read_messages(topics=['ground_truth_poses']))

        # Get the first timestamps
        first_experiment_time = experiment_messages[0][2].to_sec()
        first_ground_truth_time = ground_truth_messages[0][2].to_sec()

        # Compute the time offset between the two bags
        time_offset = first_experiment_time - first_ground_truth_time

        # Process each message in order of timestamp
        for topic, msg, t in experiment_messages:
            adjusted_time = t.to_sec() - time_offset
            if topic == 'fused_poses_odom':
                self.experiment_callback(msg, adjusted_time)
            elif topic == 'cam_poses_transformed':
                self.cam_callback(msg, adjusted_time)
            elif topic == 'lidar_poses_transformed':
                self.lidar_callback(msg, adjusted_time)

        for topic, msg, t in ground_truth_messages:
            self.ground_truth_callback(msg, t.to_sec())

        experiment_bag.close()
        ground_truth_bag.close()

        # After processing all messages, compute costs
        self.compute_costs()

    def experiment_callback(self, msg, timestamp):
        gt_pose = self.find_closest_ground_truth(timestamp, self.ground_truth_buffer_for_experiment)
        if gt_pose:
            self.experiment_buffer.append((timestamp, msg.poses))
            self.ground_truth_buffer_for_experiment.append((timestamp, gt_pose))

    def cam_callback(self, msg, timestamp):
        gt_pose = self.find_closest_ground_truth(timestamp, self.ground_truth_buffer_for_cam)
        if gt_pose:
            self.cam_buffer.append((timestamp, msg.poses))
            self.ground_truth_buffer_for_cam.append((timestamp, gt_pose))

    def lidar_callback(self, msg, timestamp):
        gt_pose = self.find_closest_ground_truth(timestamp, self.ground_truth_buffer_for_lidar)
        if gt_pose:
            self.lidar_buffer.append((timestamp, msg.poses))
            self.ground_truth_buffer_for_lidar.append((timestamp, gt_pose))

    def ground_truth_callback(self, msg, timestamp):
        # Append ground truth poses without filtering
        self.ground_truth_buffer_for_experiment.append((timestamp, msg.poses))
        self.ground_truth_buffer_for_cam.append((timestamp, msg.poses))
        self.ground_truth_buffer_for_lidar.append((timestamp, msg.poses))

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
        self.compute_cost_for_pair(self.experiment_buffer, self.ground_truth_buffer_for_experiment, "fused_poses_odom")
        self.compute_cost_for_pair(self.cam_buffer, self.ground_truth_buffer_for_cam, "cam_poses_transformed")
        self.compute_cost_for_pair(self.lidar_buffer, self.ground_truth_buffer_for_lidar, "lidar_poses_transformed")

    def compute_cost_for_pair(self, buffer_a, buffer_b, label):
        min_average_cost = float('inf')
        original_min_average_cost = float('inf')
        original_flag = False
        optimal_shift = 0
        perfect_delay = 0

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

            total_cost = 0
            count = 0

            for a_msg, b_msg in zip(shifted_buffer_a, shifted_buffer_b):
                cost = self.calculate_cost(a_msg[1], b_msg[1])
                if cost is not None:
                    total_cost += cost
                    count += 1

            if count > 0:
                average_cost = total_cost / count
                if average_cost < min_average_cost:
                    min_average_cost = average_cost
                    optimal_shift = shift
                    perfect_delay = shifted_buffer_a[0][0] - shifted_buffer_b[0][0]

                if original_flag:
                    original_min_average_cost = average_cost
                    original_flag = False

        rospy.loginfo("[%s] Minimum average cost: %f at shift: %d and delay %f", label, min_average_cost, optimal_shift, perfect_delay)
        rospy.loginfo("[%s] Original minimum average cost: %f", label, original_min_average_cost)

    def calculate_cost(self, poses_a, poses_b):
        poses_a = np.array([[pose.position.x, pose.position.y] for pose in poses_a])
        poses_b = np.array([[pose.position.x, pose.position.y] for pose in poses_b])

        if poses_a.shape[0] == 0 or poses_b.shape[0] == 0:
            return None

        cost_matrix = np.linalg.norm(poses_a[:, np.newaxis] - poses_b, axis=2)
        row_ind, col_ind = linear_sum_assignment(cost_matrix)

        # Compute the average cost
        total_cost = cost_matrix[row_ind, col_ind].sum() / len(row_ind)

        return total_cost

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    # Parse command line arguments
    import argparse
    parser = argparse.ArgumentParser(description='Compute cost between poses')
    parser.add_argument('experiment_bag_path', type=str, help='Path to the rosbag file with experiment poses')
    parser.add_argument('ground_truth_bag_path', type=str, help='Path to the rosbag file with ground truth poses')
    args = parser.parse_args()
    node = CostBetweenPosesNode(args.experiment_bag_path, args.ground_truth_bag_path)
    node.run()
