#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseArray

class PoseArrayRepublisherNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('pose_array_republisher_node', anonymous=True)

        # Define the publisher
        self.pub = rospy.Publisher('cam_poses_base_link', PoseArray, queue_size=10)

        # Define the subscriber and its callback
        self.sub = rospy.Subscriber('cam_poses', PoseArray, self.callback)
        
    def callback(self, pose_array_msg):
        # Change the frame_id to "base_link"
        pose_array_msg.header.frame_id = "base_link"
        
        # Publish the modified message
        self.pub.publish(pose_array_msg)

    def run(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    republisher = PoseArrayRepublisher()
    republisher.run()
