#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseArray, Pose
import numpy as np

class PoseArrayRepublisherNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('pose_array_republisher_node', anonymous=True)

        # Read the transformation matrix from ROS parameters
        self.transform_matrix = rospy.get_param('~transform_matrix', [
            [0.99675996, 0.08043372, -0.05389885],
            [-0.08043372, 0.99675996, 0.16321373],
            [0.0, 0.0, 1.0]
        ])
        self.transform_matrix = np.array(self.transform_matrix)

        # Define the publisher
        self.pub = rospy.Publisher('cam_poses_base_link', PoseArray, queue_size=10)

        # Define the subscriber and its callback
        self.sub = rospy.Subscriber('cam_poses', PoseArray, self.callback)

        self.run()
        
    def callback(self, pose_array_msg):
        # Change the frame_id to "base_link_40"
        pose_array_msg.header.frame_id = "base_link_40"
        
        # Transform each pose in the PoseArray
        for pose in pose_array_msg.poses:
            # Extract the x, y position
            x = pose.position.x
            y = pose.position.y
            
            # Create the homogeneous coordinates vector
            position_homogeneous = np.array([x, y, 1.0])
            
            # Apply the transformation
            transformed_position = self.transform_matrix @ position_homogeneous
            
            # Update the pose with the transformed position
            pose.position.x = transformed_position[0]
            pose.position.y = transformed_position[1]
            # Z remains unchanged in this 2D transformation
            
        # Publish the modified message
        self.pub.publish(pose_array_msg)

    def run(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    republisher = PoseArrayRepublisherNode()
    republisher.run()
