#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseArray, Pose
from vision_msgs.msg import Detection2DArray
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

        # Define the publishers
        self.pub_cam = rospy.Publisher('cam_poses_transformed', PoseArray, queue_size=3)
        self.pub_lidar = rospy.Publisher('lidar_poses_transformed', PoseArray, queue_size=3)

        # Define the subscribers and their callbacks
        self.sub_cam = rospy.Subscriber('cam_poses', PoseArray, self.callback_cam, queue_size=3)
        self.sub_lidar = rospy.Subscriber('lidar_poses', PoseArray, self.callback_lidar, queue_size=3)
        self.sub_oak = rospy.Subscriber('/oak', Detection2DArray, self.callback_oak, queue_size=3)

        # Variable to store the filtered detections
        self.filtered_detections = None

        self.run()

    def string_to_2d_array(self, string):
        try:
            # Use eval to convert the string to a nested list
            array_list = eval(string)
        except (NameError, SyntaxError):
            raise ValueError("Invalid string format. Please use proper list of lists syntax.")

        # Convert the nested list to a NumPy array
        return np.array(array_list)

    def callback_oak(self, detection_array_msg):
        # Filter detections based on confidence threshold
        self.filtered_detections = []
        for detection in detection_array_msg.detections:
            if detection.bbox.center.theta < 0.5 or detection.bbox.size_x > 160 or detection.bbox.size_y > 80:
                self.filtered_detections.append(detection)

        # Only keep the filtered detections if there are any
        if len(self.filtered_detections) > 0:
            self.filtered_detections = None
        else:
            self.filtered_detections = [1]

    def callback_cam(self, pose_array_msg):
        if self.filtered_detections:
            transformed_pose_array_msg = self.transform_poses(pose_array_msg, self.transform_matrix_cam)
            self.pub_cam.publish(transformed_pose_array_msg)
            self.filtered_detections = None

    def callback_lidar(self, pose_array_msg):
        transformed_pose_array_msg = self.transform_poses(pose_array_msg, self.transform_matrix_lidar)
        self.pub_lidar.publish(transformed_pose_array_msg)

    def transform_poses(self, pose_array_msg, transform_matrix):
        # Change the frame_id to "base_link_40"
        pose_array_msg.header.frame_id = self.frame_id
        
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
