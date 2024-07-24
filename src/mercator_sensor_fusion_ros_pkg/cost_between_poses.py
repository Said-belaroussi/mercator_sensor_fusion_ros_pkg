#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseArray
from scipy.optimize import linear_sum_assignment
import numpy as np

class PoseMatcher:
    def __init__(self):
        self.cam_poses = None
        self.lidar_poses = None
        self.average_cost = 0.0
        self.iterations = 0
        self.cam_sub = rospy.Subscriber('cam_poses', PoseArray, self.cam_callback)
        self.lidar_sub = rospy.Subscriber('lidar_poses', PoseArray, self.lidar_callback)

    def cam_callback(self, msg):
        self.cam_poses = msg.poses
        if self.lidar_poses:
            self.match_poses()

    def lidar_callback(self, msg):
        self.lidar_poses = msg.poses

    def match_poses(self):
        cam_poses = np.array([[pose.position.x, pose.position.y] for pose in self.cam_poses])
        lidar_poses = np.array([[pose.position.x, pose.position.y] for pose in self.lidar_poses])

        cost_matrix = np.linalg.norm(cam_poses[:, np.newaxis] - lidar_poses, axis=2)
        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        total_cost = cost_matrix[row_ind, col_ind].sum()
        self.average_cost = (self.average_cost * self.iterations + total_cost) / (self.iterations + 1)
        self.iterations += 1

        rospy.loginfo("Total cost of optimal matching: %f", total_cost)
        rospy.loginfo("Average cost since the beginning: %f", self.average_cost)

def main():
    rospy.init_node('pose_matcher')
    matcher = PoseMatcher()
    rospy.spin()

if __name__ == '__main__':
    main()