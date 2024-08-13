#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, PoseArray
from tf.transformations import quaternion_from_euler, quaternion_matrix, quaternion_multiply, inverse_matrix
from numpy import array, dot
from tf2_msgs.msg import TFMessage
from ast import literal_eval
import time
import rosbag

class GroundTruthBaggerNode:
    def __init__(self):
        rospy.init_node('ground_truth_bagger_node')

        # Get list of robot names and reference robot name from parameter server
        robot_names = rospy.get_param('~robot_names', "['base_link_31', 'base_link_22']")
        reference_robot_name = rospy.get_param('~reference_robot_name', 'base_link_31')
        self.reference_robot_rotation = rospy.get_param('~reference_robot_rotation', -90)
        # convert the rotation to radians
        self.reference_robot_rotation = -(self.reference_robot_rotation * 3.14159 / 180)
        self.new_frame_id = rospy.get_param('~new_frame_id', 'odom')

        rospy.loginfo(robot_names)
        rospy.loginfo(reference_robot_name)
        rospy.loginfo(self.reference_robot_rotation)

        if not robot_names or not reference_robot_name:
            rospy.logerr("Robot names or reference robot name not provided. Exiting...")
            return

        self.robot_names = self.string_to_list(robot_names)
        self.reference_robot_name = reference_robot_name
        self.robot_poses = {}

        # Rosbag handling
        self.input_bag_file = rospy.get_param('~input_bag_file')  # Get input bag file path from parameter
        self.output_bag_file = self.input_bag_file.replace('.bag', '_ground_truth.bag')  # Output bag file path

        if not self.input_bag_file or not self.output_bag_file:
            rospy.logerr("Input or output bag file not provided. Exiting...")
            return

        self.in_bag = rosbag.Bag(self.input_bag_file, 'r')
        self.out_bag = rosbag.Bag(self.output_bag_file, 'w')

        self.process_bag()

        self.in_bag.close()
        self.out_bag.close()

    def string_to_list(self, string):
        try:
            return literal_eval(string)
        except (ValueError, SyntaxError):
            raise ValueError("Invalid string format. Please use proper list of lists syntax.")

    def process_bag(self):
        for topic, msg, t in self.in_bag.read_messages(topics=['/tf']):
            # Process tf messages as before
            global_start_time = time.time()

            for transform in msg.transforms:
                child_frame_id = transform.child_frame_id

                if child_frame_id in self.robot_names:
                    translation = transform.transform.translation
                    rotation = transform.transform.rotation
                    self.robot_poses[child_frame_id] = Pose(
                        position=translation, orientation=rotation
                    )

            if set(self.robot_poses.keys()) == set(self.robot_names) and child_frame_id == self.reference_robot_name:
                ref_pose = self.robot_poses.get(self.reference_robot_name)

                if ref_pose:
                    start_time = time.time()

                    ref_position = array([ref_pose.position.x, ref_pose.position.y, 1])
                    ref_orientation = array([ref_pose.orientation.x, ref_pose.orientation.y, ref_pose.orientation.z, ref_pose.orientation.w])
                    ref_orientation = quaternion_multiply(quaternion_from_euler(0, 0, self.reference_robot_rotation), ref_orientation)
                    ref_matrix = quaternion_matrix(ref_orientation)
                    ref_matrix[:2, 3] = ref_position[:2]

                    inverse_ref_matrix = inverse_matrix(ref_matrix)

                    end_time = time.time()
                    # rospy.loginfo(f"Matrix inversion and reference pose transformation took {end_time - start_time:.6f} seconds")

                    pose_array_msg = PoseArray()
                    pose_array_msg.header.stamp = rospy.Time.now()  # Or use 't' from the bag message for original timestamp
                    pose_array_msg.header.frame_id = self.new_frame_id

                    for robot_name in self.robot_names:
                        if robot_name != self.reference_robot_name:
                            robot_pose = self.robot_poses[robot_name]
                            robot_position = array([robot_pose.position.x, robot_pose.position.y, 1])
                            robot_matrix = quaternion_matrix(array([
                                robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w
                            ]))
                            robot_matrix[:2, 3] = robot_position[:2]

                            start_time = time.time()

                            transformed_matrix = dot(inverse_ref_matrix, robot_matrix)
                            transformed_position = transformed_matrix[:2, 3]

                            end_time = time.time()
                            # rospy.loginfo(f"Robot {robot_name} transformation took {end_time - start_time:.6f} seconds")

                            new_pose = Pose()
                            new_pose.position.x = transformed_position[0]
                            new_pose.position.y = transformed_position[1]
                            new_pose.position.z = 0  # Set z to a constant value (e.g., 0) as we are ignoring it
                            new_pose.orientation = robot_pose.orientation

                            pose_array_msg.poses.append(new_pose)
                    self.out_bag.write('/ground_truth_poses', pose_array_msg, t)  # Use 't' for consistent timestamps

            # Write both the original tf message and the computed ground_truth_poses message to the output bag
            self.out_bag.write('/tf', msg, t)

            global_end_time = time.time()
            # rospy.loginfo(f"Total processing time for the message: {global_end_time - global_start_time:.6f} seconds")

if __name__ == '__main__':
    GroundTruthPublisherNode()
