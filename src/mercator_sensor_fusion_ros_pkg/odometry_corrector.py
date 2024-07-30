#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import sin, cos, atan2
import tf

class OdometryCorrectorNode:
    def __init__(self):
        rospy.init_node('odometry_corrector_node', anonymous=True)
        self.frame_id = rospy.get_param('~frame_id', 'odom')
        self.child_frame_id = rospy.get_param('~child_frame_id', 'base_link')
        self.imu_subscriber = rospy.Subscriber('rvr/imu', Imu, self.imu_callback)
        self.imu_publisher = rospy.Publisher('imu', Imu, queue_size=10)
        self.odom_subscriber = rospy.Subscriber('/rvr/odom', Odometry, self.odom_callback)
        self.odom_publisher = rospy.Publisher('/rvr/odom/correct', Odometry, queue_size=10)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.initial_orientation = None
        self.initial_yaw = None
        self.initial_x = None
        self.initial_y = None

        self.last_orientation = None

        self.run()

    def imu_callback(self, imu_msg):
        # Modify the frame_id field to "base_link"
        imu_msg.header.frame_id = self.child_frame_id
        # Publish the modified IMU message
        self.imu_publisher.publish(imu_msg)

        # Store the initial orientation
        if self.initial_orientation is None:
            self.initial_orientation = imu_msg.orientation

        self.last_orientation = imu_msg.orientation

    def odom_callback(self, odom_msg):
        if self.initial_orientation is not None:
                # Modify the frame_id and child_frame_id fields
                odom_msg.header.frame_id = self.frame_id
                odom_msg.child_frame_id = self.child_frame_id
                # Publish the modified odometry message
                # Publish the odometry message with IMU orientation mixed in
                odom_msg.pose.pose.orientation = self.last_orientation
                self.odom_publisher.publish(odom_msg)

                # Broadcast the transform
                self.tf_broadcaster.sendTransform(
                    (odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, 0),
                    (
                        self.last_orientation.x,
                        self.last_orientation.y,
                        self.last_orientation.z,
                        self.last_orientation.w,
                    ),
                    rospy.Time.now(),
                    self.child_frame_id,
                    self.frame_id
                )

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        message_republisher = OdometryCorrectorNode()
        message_republisher.run()
    except rospy.ROSInterruptException:
        pass
