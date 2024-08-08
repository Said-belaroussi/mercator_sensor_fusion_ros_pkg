#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseArray, Pose
import numpy as np

class PoseArrayRepublisherNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('pose_array_republisher_node', anonymous=True)

        # Read the transformation matrix from ROS parameters
        self.transform_matrix_cam = rospy.get_param('~transform_matrix_cam', 
        "[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]")
        self.transform_matrix_lidar = rospy.get_param('~transform_matrix_lidar',
         "[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [ 0.0, 0.0, 1.0]]")

        self.frame_id = rospy.get_param('~frame_id', 'base_link_40')

        self.fov_range = rospy.get_param('~fov_range', 42.5)

        # Convert the string to a numpy array
        self.transform_matrix_cam = self.string_to_2d_array(self.transform_matrix_cam)
        self.transform_matrix_lidar = self.string_to_2d_array(self.transform_matrix_lidar)
        # Define the publisher
        self.pub_cam = rospy.Publisher('cam_poses_transformed', PoseArray, queue_size=3)
        self.pub_lidar = rospy.Publisher('lidar_poses_transformed', PoseArray, queue_size=3)

        # Define the subscriber and its callback
        self.sub_cam = rospy.Subscriber('cam_poses', PoseArray, self.callback_cam, queue_size=3)
        self.sub_lidar = rospy.Subscriber('lidar_poses', PoseArray, self.callback_lidar, queue_size=3)

        self.run()

    def string_to_2d_array(self, string):
        """
        Converts a string representation of a 2D array to a NumPy array.
        Args:
            string: The string representation of the 2D array.
        Returns:
            A 2D NumPy array representing the input string.
        Raises:
            ValueError: If the string format is invalid.
        """
        try:
            # Use eval to convert the string to a nested list
            array_list = eval(string)
        except (NameError, SyntaxError):
            raise ValueError("Invalid string format. Please use proper list of lists syntax.")

        # Convert the nested list to a NumPy array
        return np.array(array_list)

    def callback_cam(self, pose_array_msg):
        # filtered_pose_array_msg = self.filter_poses_by_fov(pose_array_msg)
        transformed_pose_array_msg = self.transform_poses(pose_array_msg, self.transform_matrix_cam)
        self.pub_cam.publish(transformed_pose_array_msg)


    def callback_lidar(self, pose_array_msg):
        transformed_pose_array_msg = self.transform_poses(pose_array_msg, self.transform_matrix_lidar)
        self.pub_lidar.publish(transformed_pose_array_msg)

    # def filter_poses_by_fov(self, pose_array_msg):
    #     # Create a new PoseArray to store filtered poses
    #     filtered_pose_array_msg = PoseArray()
    #     filtered_pose_array_msg.header = pose_array_msg.header

    #     # Filter the poses within the FOV
    #     for pose in pose_array_msg.poses:
    #         x = pose.position.x
    #         y = pose.position.y

    #         # Calculate the angle with the y-axis (in radians)
    #         angle = np.arctan2(x, y)

    #         # Convert the angle to degrees
    #         angle_deg = np.degrees(angle)
    #         rospy.loginfo(f"%f, %f, %f", x, y, angle_deg)

    #         # Check if the pose is within the FOV
    #         if -self.fov_range <= angle_deg <= self.fov_range:
    #             filtered_pose_array_msg.poses.append(pose)

    #     return filtered_pose_array_msg

    def transform_poses(self, pose_array_msg, transform_matrix):
        # Change the frame_id to "base_link_40"
        pose_array_msg.header.frame_id = pose_array_msg.header.frame_id
        
        # Transform each pose in the PoseArray
        for pose in pose_array_msg.poses:
            # Extract the x, y position
            x = pose.position.x
            y = pose.position.y
            
            # Create the homogeneous coordinates vector
            position_homogeneous = np.array([x, y, 1.0])
            
            # Apply the transformation
            transformed_position = transform_matrix @ position_homogeneous
            
            # Update the pose with the transformed position
            pose.position.x = transformed_position[0]
            pose.position.y = transformed_position[1]
            # Z remains unchanged in this 2D transformation
            
        return pose_array_msg

    def run(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    republisher = PoseArrayRepublisherNode()
    republisher.run()
