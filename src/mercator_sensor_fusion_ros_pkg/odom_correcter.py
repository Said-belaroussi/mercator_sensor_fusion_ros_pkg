#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import sin, cos, atan2

class MessageRepublisher:
    def __init__(self):
        rospy.init_node('message_republisher', anonymous=True)
        self.imu_subscriber = rospy.Subscriber('rvr/imu', Imu, self.imu_callback)
        self.imu_publisher = rospy.Publisher('imu', Imu, queue_size=10)
        self.odom_subscriber = rospy.Subscriber('/rvr/odom', Odometry, self.odom_callback)
        self.odom_publisher = rospy.Publisher('/rvr/odom/correct', Odometry, queue_size=10)
        # self.odom_imu_mixed_publisher = rospy.Publisher('/rvr/odom/odom_imu_mixed', Odometry, queue_size=10)
        self.initial_orientation = None
        self.initial_yaw = None
        self.initial_x = None
        self.initial_y = None

        self.last_orientation = None

    def imu_callback(self, imu_msg):
        # Modify the frame_id field to "base_link"
        imu_msg.header.frame_id = "base_link"
        # Publish the modified IMU message
        self.imu_publisher.publish(imu_msg)

        # Store the initial orientation
        if self.initial_orientation is None:
            self.initial_orientation = imu_msg.orientation

        self.last_orientation = imu_msg.orientation

    def odom_callback(self, odom_msg):
        if self.initial_orientation is not None:
            if self.initial_yaw is None:
                if self.initial_x is None and self.initial_y is None:
                    self.initial_x = odom_msg.pose.pose.position.x
                    self.initial_y = odom_msg.pose.pose.position.y
                else:
                    delta_x = odom_msg.pose.pose.position.x - self.initial_x
                    delta_y = odom_msg.pose.pose.position.y - self.initial_y
                    yaw1 = atan2(delta_y, delta_x)
                    (roll, pitch, yaw) = euler_from_quaternion([
                    self.initial_orientation.x,
                    self.initial_orientation.y,
                    self.initial_orientation.z,
                    self.initial_orientation.w
                    ])
                    print("yaw1 : ", yaw1)
                    print("yaw : ", yaw)
                    if (delta_x != 0 and delta_y != 0): 
                        self.initial_yaw = yaw - yaw1
            else:
                # Calculate new x, y positions based on initial orientation
                x = odom_msg.pose.pose.position.x
                y = odom_msg.pose.pose.position.y
                new_x = x * cos(self.initial_yaw) - y * sin(self.initial_yaw)
                new_y = x * sin(self.initial_yaw) + y * cos(self.initial_yaw)
                # Modify the frame_id and child_frame_id fields
                odom_msg.header.frame_id = "odom"
                odom_msg.child_frame_id = "base_link"
                # Update the x, y positions
                odom_msg.pose.pose.position.x = new_x
                odom_msg.pose.pose.position.y = new_y
                # Publish the modified odometry message
                # Publish the odometry message with IMU orientation mixed in
                odom_msg.pose.pose.orientation = self.last_orientation
                self.odom_publisher.publish(odom_msg)

                # # Publish the odometry message with IMU orientation mixed in
                # odom_msg.pose.pose.orientation = self.last_orientation
                # self.odom_imu_mixed_publisher.publish(odom_msg)



    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        message_republisher = MessageRepublisher()
        message_republisher.run()
    except rospy.ROSInterruptException:
        pass