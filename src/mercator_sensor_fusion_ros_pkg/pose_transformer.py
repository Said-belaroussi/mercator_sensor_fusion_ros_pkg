#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseArray, PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
from tf2_msgs.msg import TFMessage

class PoseTransformerNode:
    def __init__(self):
        rospy.init_node('pose_transformer_node', anonymous=True)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        self.cam_poses_sub = rospy.Subscriber('cam_poses', PoseArray, self.cam_poses_callback)
        self.lidar_poses_sub = rospy.Subscriber('lidar_poses', PoseArray, self.lidar_poses_callback)
        self.tf_sub = rospy.Subscriber('tf', TFMessage, self.tf_callback)

        self.cam_poses_pub = rospy.Publisher('cam_poses_odom', PoseArray, queue_size=10)
        self.lidar_poses_pub = rospy.Publisher('lidar_poses_odom', PoseArray, queue_size=10)

        self.transform = None

        self.run()

    def tf_callback(self, msg):
        for transform in msg.transforms:
            if transform.child_frame_id == "base_link" and transform.header.frame_id == "odom":
                self.transform = transform

    def cam_poses_callback(self, msg):
        self.publish_transformed_poses(msg, self.cam_poses_pub, self.transformation_matrix_ground_truth)

    def lidar_poses_callback(self, msg):
        self.publish_transformed_poses(msg, self.lidar_poses_pub, self.transformation_matrix_ground_truth)

    def publish_transformed_poses(self, msg, publisher, transformation_matrix):
        if self.transform is None:
            rospy.logwarn("Transform from odom to base_link not yet received")
            return

        transformed_poses = PoseArray()
        transformed_poses.header = msg.header
        transformed_poses.header.frame_id = "odom"

        for pose in msg.poses:
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.pose = pose

            try:
                # Transform using tf
                transformed_pose = do_transform_pose(pose_stamped, self.transform)

                if transformation_matrix is not None:
                    # Transform using additional transformation matrix
                    pose_np = np.array([transformed_pose.pose.position.x,
                                        transformed_pose.pose.position.y,
                                        1.0])  # Homogeneous coordinates

                    transformed_pose_np = transformation_matrix @ pose_np
                else:
                    transformed_pose_np = np.array([transformed_pose.pose.position.x,
                                                    transformed_pose.pose.position.y])

                transformed_pose.pose.position.x = transformed_pose_np[0]
                transformed_pose.pose.position.y = transformed_pose_np[1]

                transformed_poses.poses.append(transformed_pose.pose)
            except Exception as e:
                rospy.logerr(f"Error transforming pose: {e}")

        publisher.publish(transformed_poses)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        PoseTransformerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
