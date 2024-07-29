#!/usr/bin/env python

import rospy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseArray, Pose
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import tf2_ros
import tf2_geometry_msgs

class ArenaPosesPublisherNode:
    def __init__(self):
        # Initialize the node
        rospy.init_node('arena_poses_publisher_node', anonymous=True)

        self.frame_id = rospy.get_param('~frame_id', 'odom')
        # Get the list of child_frame_ids from the parameter server
        self.child_frame_ids = rospy.get_param('~child_frame_ids', ["base_link_38"])
        if not self.child_frame_ids:
            rospy.logerr("No child_frame_ids specified. Shutting down node.")
            rospy.signal_shutdown("No child_frame_ids specified.")
            return

        # Convert the string to a list
        self.child_frame_ids = self.string_to_list(self.child_frame_ids)

        self.tf_data = {child_frame_id: None for child_frame_id in self.child_frame_ids}

        # Subscriber for the tf topic
        rospy.Subscriber('tf', TFMessage, self.tf_callback)

        # Publisher for the arena_poses topic
        self.arena_poses_pub = rospy.Publisher('arena_poses', PoseArray, queue_size=10)

        # Timer to regularly publish the poses
        rospy.Timer(rospy.Duration(0.1), self.publish_arena_poses)

        rospy.loginfo("TF Listener initialized and listening for frames: {}".format(self.child_frame_ids))

        self.run()

    def string_to_list(self, string):
        try:
            # Use eval to convert the string to a list
            list = eval(string)
        except (NameError, SyntaxError):
            raise ValueError("Invalid string format. Please use proper list of lists syntax.")

        return list
    
    def tf_callback(self, msg):
        for transform in msg.transforms:
            child_frame_id = transform.child_frame_id
            if child_frame_id in self.child_frame_ids:
                self.tf_data[child_frame_id] = transform

    def publish_arena_poses(self, event):
        # rospy.loginfo("publish_arena_poses")
        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = self.frame_id

        for child_frame_id, transform in self.tf_data.items():
            if transform:
                pose = Pose()
                pose.position.x = transform.transform.translation.x
                pose.position.y = transform.transform.translation.y
                pose.position.z = transform.transform.translation.z

                pose.orientation = transform.transform.rotation

                pose_array.poses.append(pose)

        self.arena_poses_pub.publish(pose_array)

    def run(self):
        rospy.spin()
