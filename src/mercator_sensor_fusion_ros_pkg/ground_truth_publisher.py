#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, PoseArray
from tf import transformations
import numpy as np
from tf2_msgs.msg import TFMessage

class GroundTruthPublisherNode:
    def __init__(self, robot_names=["base_link_38"], reference_robot_name='base_link_0'):
        rospy.init_node('ground_truth_publisher_node')

        # Get list of robot names and reference robot name from parameter server
        robot_names = rospy.get_param('~robot_names', ["base_link_38"])
        reference_robot_name = rospy.get_param('~reference_robot_name', 'base_link_0')

        rospy.loginfo(robot_names)
        rospy.loginfo(reference_robot_name)

        if not robot_names or not reference_robot_name:
            rospy.logerr("Robot names or reference robot name not provided. Exiting...")
            return

        self.robot_names = robot_names
        self.reference_robot_name = reference_robot_name
        self.robot_poses = {}
        self.ground_truth_pub = rospy.Publisher('ground_truth_poses', PoseArray, queue_size=10)
        self.tf_sub = rospy.Subscriber('/tf', TFMessage, self.tf_callback)

        self.run()

    def tf_callback(self, tf_msg):
        for transform in tf_msg.transforms:
            child_frame_id = transform.child_frame_id
            
            if child_frame_id in self.robot_names:
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                pose = Pose(position=translation, orientation=rotation)
                self.robot_poses[child_frame_id] = pose

        if set(self.robot_poses.keys()) == set(self.robot_names):
            
            ref_pose = self.robot_poses.get(self.reference_robot_name)
            
            if ref_pose:

                ref_position = np.array([ref_pose.position.x, ref_pose.position.y, ref_pose.position.z, 1])
                ref_orientation = np.array([ref_pose.orientation.x, ref_pose.orientation.y, ref_pose.orientation.z, ref_pose.orientation.w])
                # Add +90 degrees rotation around z-axis to align with the map
                ref_orientation = transformations.quaternion_multiply(ref_orientation, transformations.quaternion_from_euler(0, 0, np.pi/2))
                ref_matrix = transformations.quaternion_matrix(ref_orientation)
                ref_matrix[0:3, 3] = ref_position[0:3]

                inverse_ref_matrix = np.linalg.inv(ref_matrix)

                pose_array_msg = PoseArray()
                pose_array_msg.header.stamp = rospy.Time.now()
                pose_array_msg.header.frame_id = "base_link_40"

                for robot_name in self.robot_names:
                    if robot_name != self.reference_robot_name:
                        robot_pose = self.robot_poses[robot_name]
                        robot_position = np.array([robot_pose.position.x, robot_pose.position.y, robot_pose.position.z, 1])
                        robot_matrix = transformations.quaternion_matrix(np.array([
                            robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w]))
                        robot_matrix[0:3, 3] = robot_position[0:3]

                        # Transform robot position to the reference frame
                        transformed_matrix = np.dot(inverse_ref_matrix, robot_matrix)
                        transformed_position = transformed_matrix[0:3, 3]

                        # New pose of the robot relative to the reference
                        new_pose = Pose()
                        new_pose.position.x = transformed_position[0]
                        new_pose.position.y = transformed_position[1]
                        new_pose.position.z = transformed_position[2]
                        new_pose.orientation = robot_pose.orientation  # Keeping original orientation for simplicity
                        pose_array_msg.poses.append(new_pose)

                self.ground_truth_pub.publish(pose_array_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    GroundTruthPublisherNode()