#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseArray
import threading
import time

class PoseSyncNode:
    def __init__(self):
        rospy.init_node('pose_sync_node')

        self.cam_pose_sub = rospy.Subscriber('cam_poses', PoseArray, self.cam_pose_callback)
        self.ground_truth_pose_sub = rospy.Subscriber('ground_truth_poses', PoseArray, self.ground_truth_pose_callback)
        
        self.synced_cam_pub = rospy.Publisher('synced_cam_poses', PoseArray, queue_size=10)
        self.synced_ground_truth_pub = rospy.Publisher('synced_ground_truth_poses', PoseArray, queue_size=10)

        self.cam_pose_data = None
        self.prev_cam_pose_data = None
        self.ground_truth_pose_data = None
        self.prev_ground_truth_pose_data = None

        self.cam_start_time = None
        self.ground_truth_start_time = None
        self.delay = None

        self.lock_cam = threading.Lock()
        self.lock_ground = threading.Lock()

        self.last_cam_pose_time = time.time()
        self.last_ground_truth_pose_time = time.time()
        self.timeout = 5.0  # 5 seconds timeout

    def cam_pose_callback(self, msg):
        rospy.loginfo("-------------------------")
        with self.lock_cam:
            if self.prev_cam_pose_data is None:
                self.prev_cam_pose_data = msg
            else:
                rospy.loginfo("-------------------------")
                self.cam_pose_data = msg
                self.prev_cam_pose_data = msg
                self.cam_start_time = time.time()
                if len(self.prev_cam_pose_data.poses) > 0 and len(msg.poses) > 0:
                    if self.detect_significant_movement(self.prev_cam_pose_data.poses[0], msg.poses[0]):
                        rospy.loginfo("cam detected significant")
                        if self.ground_truth_start_time:
                            self.delay = self.ground_truth_start_time - self.cam_start_time
                            rospy.loginfo(self.delay)
            self.last_cam_pose_time = time.time()

    def ground_truth_pose_callback(self, msg):
        rospy.loginfo("+++++++++++++++++++++++++++")
        with self.lock_ground:
            if self.prev_ground_truth_pose_data is None:
                self.prev_ground_truth_pose_data = msg
            else:
                rospy.loginfo("+++++++++++++++++++++++++++")
                self.ground_truth_pose_data = msg
                self.prev_ground_truth_pose_data = msg
                self.ground_truth_start_time = time.time()
                if len(self.prev_ground_truth_pose_data.poses) > 0 and len(msg.poses) > 0:
                    if self.detect_significant_movement(self.prev_ground_truth_pose_data.poses[0], msg.poses[0]):
                        rospy.loginfo("ground_truth detected significant")
                        self.ground_truth_pose_data = msg
                        self.prev_ground_truth_pose_data = msg
                        self.ground_truth_start_time = time.time()
                        if self.cam_start_time:
                            self.delay = self.cam_start_time - self.ground_truth_start_time
                            rospy.loginfo(self.delay)
            self.last_ground_truth_pose_time = time.time()

    def detect_significant_movement(self, pose1, pose2):
        threshold = 0.1
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        dz = pose1.position.z - pose2.position.z
        return (dx**2 + dy**2 + dz**2) > threshold**2

    def sync_and_publish(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            with self.lock:
                current_time = time.time()
                if self.cam_pose_data and self.ground_truth_pose_data and self.delay is not None:
                    while not rospy.is_shutdown():
                        rospy.loginfo("entered here")
                        with self.lock:
                            current_time = time.time()
                            adjusted_cam_time = self.cam_start_time + (current_time - self.cam_start_time)
                            adjusted_ground_truth_time = self.ground_truth_start_time + (current_time - self.ground_truth_start_time) + self.delay

                            if adjusted_cam_time <= current_time:
                                self.synced_cam_pub.publish(self.cam_pose_data)
                            if adjusted_ground_truth_time <= current_time:
                                self.synced_ground_truth_pub.publish(self.ground_truth_pose_data)

                        if (current_time - self.last_cam_pose_time > self.timeout and
                            current_time - self.last_ground_truth_pose_time > self.timeout):
                            rospy.signal_shutdown('No new messages received, shutting down.')
                            break

                        rate.sleep()

if __name__ == '__main__':
    node = PoseSyncNode()
    node.sync_and_publish()
    rospy.spin()
