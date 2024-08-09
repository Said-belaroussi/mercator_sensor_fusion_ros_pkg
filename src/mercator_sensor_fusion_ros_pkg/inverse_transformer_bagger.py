#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseArray, PoseStamped, TransformStamped
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from tf2_geometry_msgs import do_transform_pose
from tf2_msgs.msg import TFMessage
import tf2_py as tf2
import rosbag

class InverseTransformerBaggerNode:
    def __init__(self):
        rospy.init_node('inverse_transformer_bagger_node', anonymous=True)

        self.tf_buffer = Buffer()

        self.new_poses_frame_id = rospy.get_param('~new_poses_frame_id', 'odom')
        self.robot_odom_frame_id = rospy.get_param('~robot_odom_frame_id', 'robot_odom')
        self.robot_frame_id = rospy.get_param('~robot_frame_id', 'robot')

        # Rosbag handling
        self.input_bag_file = rospy.get_param('~input_bag_file')  # Get input bag file path from parameter
        self.output_bag_file = self.input_bag_file.replace('.bag', '_transformed.bag')  # Output bag file path

        if not self.input_bag_file or not self.output_bag_file:
            rospy.logerr("Input or output bag file not provided. Exiting...")
            return

        self.in_bag = rosbag.Bag(self.input_bag_file, 'r')
        self.out_bag = rosbag.Bag(self.output_bag_file, 'w')

        self.process_bag()

        self.in_bag.close()
        self.out_bag.close()

    def process_bag(self):
        for topic, msg, t in self.in_bag.read_messages():
            # If it's a 'tf' message, buffer it
            if topic == '/tf':
                for transform in msg.transforms:
                    self.tf_buffer.set_transform(transform, "bag")  # Authority is 'bag' to avoid conflicts

            # If it's a 'poses' message, transform it
            if topic == 'poses':
                transformed_poses = self.transform_poses(msg)
                self.out_bag.write('poses', transformed_poses, t)  # Write transformed poses

            # Write all messages (including /tf) to the output bag
            self.out_bag.write(topic, msg, t)

    def transform_poses(self, msg):
        transformed_poses = PoseArray()
        transformed_poses.header = msg.header 
        transformed_poses.header.stamp = rospy.Time.now() # Or use 't' from the bag message for original timestamp
        transformed_poses.header.frame_id = self.new_poses_frame_id

        for pose in msg.poses:
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.pose = pose

            try:
                # Lookup transform from robot_odom to robot and invert it
                robot_odom_to_robot_transform = self.tf_buffer.lookup_transform(self.robot_odom_frame_id, self.robot_frame_id, msg.header.stamp)  # Use timestamp from 'poses' message
                robot_to_robot_odom_transform = self.invert_transform(robot_odom_to_robot_transform)

                # Apply the inverted transform
                transformed_pose = do_transform_pose(pose_stamped, robot_to_robot_odom_transform)

                transformed_poses.poses.append(transformed_pose.pose)
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                rospy.logwarn(f"Problem looking up transform: {e}")
            except Exception as e:
                rospy.logerr(f"Error transforming pose: {e}")

        return transformed_poses

    def invert_transform(self, transform):
        inverted_transform = TransformStamped()

        inverted_transform.header = transform.header
        inverted_transform.child_frame_id = transform.child_frame_id

        # Invert translation
        inverted_transform.transform.translation.x = -transform.transform.translation.x
        inverted_transform.transform.translation.y = -transform.transform.translation.y
        inverted_transform.transform.translation.z = -transform.transform.translation.z

        # Invert rotation (quaternion)
        inverted_transform.transform.rotation.x = -transform.transform.rotation.x
        inverted_transform.transform.rotation.y = -transform.transform.rotation.y
        inverted_transform.transform.rotation.z = -transform.transform.rotation.z
        inverted_transform.transform.rotation.w = transform.transform.rotation.w

        return inverted_transform


if __name__ == '__main__':
    OdomToBaseLinkPoseTransformerNode()

