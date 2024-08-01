#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseArray
from scipy.optimize import linear_sum_assignment
import numpy as np
import time

class CostBetweenPosesNode:
    def __init__(self):
        rospy.init_node('cost_between_poses_node', anonymous=True)
        self.experiment_buffer = []
        self.ground_truth_buffer = []
        self.latest_experiment_poses = None
        self.buffer_duration = rospy.get_param('~buffer_duration', 20)  # Buffer duration in seconds
        self.max_shift_messages = rospy.get_param('~max_shift_messages', 20)  # Max shift in number of messages
        self.shift_step = rospy.get_param('~shift_step', 2)
        self.timer = rospy.Timer(rospy.Duration(self.buffer_duration), self.compute_costs)

        self.experiment_sub = rospy.Subscriber('experiment_poses', PoseArray, self.experiment_callback)
        self.ground_truth_sub = rospy.Subscriber('ground_truth_poses', PoseArray, self.ground_truth_callback)

        self.run()

    def experiment_callback(self, msg):
        if self.latest_experiment_poses:
            current_time = time.time()
            self.ground_truth_buffer.append((current_time, self.latest_experiment_poses))
            self.experiment_buffer.append((current_time, msg.poses))

    def ground_truth_callback(self, msg):
        self.latest_experiment_poses = msg.poses

    def compute_costs(self, event):
        current_time = time.time()
        self.experiment_buffer = [msg for msg in self.experiment_buffer if current_time - msg[0] <= self.buffer_duration]
        self.ground_truth_buffer = [msg for msg in self.ground_truth_buffer if current_time - msg[0] <= self.buffer_duration]

        if not self.experiment_buffer or not self.ground_truth_buffer:
            rospy.loginfo("Buffers are empty or have insufficient data")
            return

        min_average_cost = float('inf')
        optimal_shift = 0
        perfect_delay = 0

        for shift in range(-self.max_shift_messages, self.max_shift_messages + 1, self.shift_step):
            if shift < 0:
                shifted_experiment_buffer = self.experiment_buffer[-shift:]
                shifted_ground_truth_buffer = self.ground_truth_buffer[:shift]
            elif shift > 0:
                shifted_experiment_buffer = self.experiment_buffer[:-shift]
                shifted_ground_truth_buffer = self.ground_truth_buffer[shift:]
            else:
                shifted_experiment_buffer = self.experiment_buffer
                shifted_ground_truth_buffer = self.ground_truth_buffer

            total_cost = 0
            count = 0

            for exp_msg, gt_msg in zip(shifted_experiment_buffer, shifted_ground_truth_buffer):
                cost = self.calculate_cost(exp_msg[1], gt_msg[1])
                if cost != None:
                    total_cost += cost
                    count += 1

            if count > 0:
                average_cost = total_cost / count
                if average_cost < min_average_cost:
                    min_average_cost = average_cost
                    optimal_shift = shift
                    perfect_delay = shifted_experiment_buffer[0][0] - shifted_ground_truth_buffer[0][0]

        rospy.loginfo("Minimum average cost: %f at shift: %d and delay %f", min_average_cost, optimal_shift, perfect_delay)

    def calculate_cost(self, experiment_poses, ground_truth_poses):
        experiment_poses = np.array([[pose.position.x, pose.position.y] for pose in experiment_poses])
        ground_truth_poses = np.array([[pose.position.x, pose.position.y] for pose in ground_truth_poses])

        if experiment_poses.shape[0] == 0 or ground_truth_poses.shape[0] == 0:
            return None

        cost_matrix = np.linalg.norm(experiment_poses[:, np.newaxis] - ground_truth_poses, axis=2)
        row_ind, col_ind = linear_sum_assignment(cost_matrix)

        # Compute the average cost
        total_cost = cost_matrix[row_ind, col_ind].sum() / len(row_ind)

        # TODO: Compute if the cost is more along the x or y axis or z ???

        return total_cost

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = CostBetweenPosesNode()
    node.run()
