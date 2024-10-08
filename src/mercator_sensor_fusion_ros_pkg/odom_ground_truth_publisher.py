#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, quaternion_matrix, quaternion_multiply, inverse_matrix
from numpy import array, dot
from ast import literal_eval
import time

class GroundTruthPublisherNode:
    def __init__(self):
        rospy.init_node('ground_truth_publisher_node')

        # Get list of robot names and reference robot name from parameter server
        robot_names = rospy.get_param('~robot_names', "['base_link_31', 'base_link_22']")
        reference_robot_name = rospy.get_param('~reference_robot_name', 'base_link_31')
        self.new_frame_id = rospy.get_param('~new_frame_id', 'odom')
        
        # Get odometry topic names
        odom_topics = rospy.get_param('~odom_topics', "['/epuck_22/odometry/filtered', '/epuck_31/odometry/filtered']")

        rospy.loginfo(robot_names)
        rospy.loginfo(reference_robot_name)

        self.robot_names = self.string_to_list(robot_names)
        self.reference_robot_name = reference_robot_name
        self.odom_topics = self.string_to_list(odom_topics)

        if not robot_names or not reference_robot_name or len(self.odom_topics) != len(self.robot_names):
            rospy.logerr("Robot names, reference robot name or odometry topics not provided correctly. Exiting...")
            return


        self.robot_poses = {}
        self.ground_truth_pub = rospy.Publisher('/ground_truth_poses', PoseArray, queue_size=10)
        
        # Initialize subscribers for each odometry topic
        self.odom_subs = [
            rospy.Subscriber(topic, Odometry, self.odom_callback, callback_args=(name, index))
            for index, (topic, name) in enumerate(zip(self.odom_topics, self.robot_names))
        ]

        self.run()

    def string_to_list(self, string):
        try:
            return literal_eval(string)
        except (ValueError, SyntaxError):
            raise ValueError("Invalid string format. Please use proper list of lists syntax.")

    def odom_callback(self, odom_msg, args):
        robot_name, index = args
        self.store_pose(odom_msg, robot_name)
        
        # Only publish ground truth poses when the first odometry topic callback is called
        if index == 0:
            self.publish_ground_truth()

    def store_pose(self, odom_msg, robot_name):
        translation = odom_msg.pose.pose.position
        rotation = odom_msg.pose.pose.orientation
        self.robot_poses[robot_name] = Pose(
            position=translation, orientation=rotation
        )

    def publish_ground_truth(self):
        global_start_time = time.time()

        if set(self.robot_poses.keys()) == set(self.robot_names):
            ref_pose = self.robot_poses.get(self.reference_robot_name)
            
            if ref_pose:
                start_time = time.time()
                
                ref_position = array([ref_pose.position.x, ref_pose.position.y, 1])
                ref_orientation = array([ref_pose.orientation.x, ref_pose.orientation.y, ref_pose.orientation.z, ref_pose.orientation.w])
                ref_orientation = quaternion_multiply(ref_orientation, quaternion_from_euler(0, 0, 0))
                ref_matrix = quaternion_matrix(ref_orientation)
                ref_matrix[:2, 3] = ref_position[:2]

                inverse_ref_matrix = inverse_matrix(ref_matrix)
                
                end_time = time.time()
                # rospy.loginfo(f"Matrix inversion and reference pose transformation took {end_time - start_time:.6f} seconds")

                pose_array_msg = PoseArray()
                pose_array_msg.header.stamp = rospy.Time.now()
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

                self.ground_truth_pub.publish(pose_array_msg)
        global_end_time = time.time()
        # rospy.loginfo(f"Total processing time for the message: {global_end_time - global_start_time:.6f} seconds")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    GroundTruthPublisherNode()
