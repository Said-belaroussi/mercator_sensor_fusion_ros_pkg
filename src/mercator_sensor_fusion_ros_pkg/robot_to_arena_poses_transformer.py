#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseArray, PoseStamped, TransformStamped
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from tf2_geometry_msgs import do_transform_pose
from tf2_msgs.msg import TFMessage
import tf2_py as tf2

class RobotToArenaPosesTransformerNode:
    def __init__(self):
        rospy.init_node('robot_to_arena_poses_transformer_node', anonymous=True)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        self.frame_id = rospy.get_param('~frame_id', 'odom')
        self.child_frame_id = rospy.get_param('~child_frame_id', 'base_link')
        self.new_poses_frame_id = rospy.get_param('~new_poses_frame_id', 'odom')
        self.robot_odom_fram_id = rospy.get_param('~robot_odom_frame_id', 'robot_odom')
        self.robot_frame_id = rospy.get_param('~robot_frame_id', 'robot')

        self.cam_poses_sub = rospy.Subscriber('cam_poses', PoseArray, self.cam_poses_callback)
        self.lidar_poses_sub = rospy.Subscriber('lidar_poses', PoseArray, self.lidar_poses_callback)
        self.tf_sub = rospy.Subscriber('tf', TFMessage, self.tf_callback)

        self.cam_poses_pub = rospy.Publisher('cam_poses_odom', PoseArray, queue_size=10)
        self.lidar_poses_pub = rospy.Publisher('lidar_poses_odom', PoseArray, queue_size=10)

        self.transform = None

        self.run()

    def tf_callback(self, msg):
        for transform in msg.transforms:
            if transform.child_frame_id == self.child_frame_id and transform.header.frame_id == self.frame_id:
                self.transform = transform

    def cam_poses_callback(self, msg):
        self.publish_transformed_poses(msg, self.cam_poses_pub)

    def lidar_poses_callback(self, msg):
        self.publish_transformed_poses(msg, self.lidar_poses_pub)

    def publish_transformed_poses(self, msg, publisher):
        if self.transform is None:
            rospy.logwarn("Transform from odom to base_link not yet received")
            return

        transformed_poses = PoseArray()
        transformed_poses.header = msg.header # Keep the same frame_id and timestamp
        transformed_poses.header.stamp = rospy.Time.now()
        transformed_poses.header.frame_id = self.new_poses_frame_id

        for pose in msg.poses:
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.pose = pose

            try:
                # Lookup transform from robot_odom to robot and invert it
                robot_odom_to_robot_transform = self.tf_buffer.lookup_transform(self.robot_odom_fram_id, self.robot_frame_id, rospy.Time(0))
                robot_to_robot_odom_transform = self.invert_transform(robot_odom_to_robot_transform)

                # Apply the inverted transform first
                transformed_pose = do_transform_pose(pose_stamped, robot_to_robot_odom_transform)
                
                # Then apply the existing transform
                transformed_pose = do_transform_pose(transformed_pose, self.transform)

                transformed_poses.poses.append(transformed_pose.pose)
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                rospy.logwarn(f"Problem looking up transform: {e}")
            except Exception as e:
                rospy.logerr(f"Error transforming pose: {e}")

        publisher.publish(transformed_poses)

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

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        PoseTransformerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
