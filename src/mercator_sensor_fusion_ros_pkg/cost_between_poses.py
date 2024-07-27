#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseArray
from scipy.optimize import linear_sum_assignment
import numpy as np

class CostBetweenPosesNode:
    def __init__(self):
        rospy.init_node('cost_between_poses_node', anonymous=True)
        self.experiment_poses = None
        self.ground_truth_poses = None
        self.average_cost = 0.0
        self.iterations = 0
        self.experiment_sub = rospy.Subscriber('experiment_poses', PoseArray, self.experiment_callback)
        self.ground_truth_sub = rospy.Subscriber('ground_truth_poses', PoseArray, self.ground_truth_callback)

        self.run()

    def experiment_callback(self, msg):
        self.experiment_poses = msg.poses
        if self.ground_truth_poses:
            self.match_poses()

    def ground_truth_callback(self, msg):
        self.ground_truth_poses = msg.poses

    def match_poses(self):
        experiment_poses = np.array([[pose.position.x, pose.position.y] for pose in self.experiment_poses])
        ground_truth_poses = np.array([[pose.position.x, pose.position.y] for pose in self.ground_truth_poses])

        cost_matrix = np.linalg.norm(experiment_poses[:, np.newaxis] - ground_truth_poses, axis=2)
        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        total_cost = cost_matrix[row_ind, col_ind].sum()
        self.average_cost = (self.average_cost * self.iterations + total_cost) / (self.iterations + 1)
        self.iterations += 1

        rospy.loginfo("Total cost of optimal matching: %f", total_cost)
        rospy.loginfo("Average cost since the beginning: %f", self.average_cost)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    main()