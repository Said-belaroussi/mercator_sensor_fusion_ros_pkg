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

        # Publisher for the filtered poses
        self.pose_pub = rospy.Publisher('arena_poses', PoseArray, queue_size=10)

        # tf2 listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Main loop frequency
        self.rate = rospy.Rate(30)  # 30 Hz

        rospy.loginfo("TF Listener initialized and listening for frames: {}".format(self.child_frame_ids))

    def string_to_list(self, string):
        try:
            # Use eval to convert the string to a list
            list = eval(string)
        except (NameError, SyntaxError):
            raise ValueError("Invalid string format. Please use proper list of lists syntax.")

        return list

    def run(self):
        while not rospy.is_shutdown():
            # Create a PoseArray message
            pose_array = PoseArray()
            pose_array.header.stamp = rospy.Time.now()
            pose_array.header.frame_id = self.frame_id

            # Check for the latest transforms of interest
            for child_frame_id in self.child_frame_ids:
                try:
                    transform = self.tf_buffer.lookup_transform(self.frame_id, child_frame_id, rospy.Time(0), rospy.Duration(1.0))
                    pose = Pose()
                    pose.position.x = transform.transform.translation.x
                    pose.position.y = transform.transform.translation.y
                    pose.position.z = transform.transform.translation.z
                    pose.orientation = transform.transform.rotation
                    pose_array.poses.append(pose)
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    rospy.logwarn("Could not transform {}: {}".format(child_frame_id, e))

            # Publish the PoseArray
            self.pose_pub.publish(pose_array)

            # Sleep for the remaining time to maintain the loop rate
            self.rate.sleep()
