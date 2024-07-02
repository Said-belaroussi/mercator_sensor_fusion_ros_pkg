#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

class MessageRepublisher:
    def __init__(self):
        rospy.init_node('message_republisher', anonymous=True)
        self.imu_subscriber = rospy.Subscriber('rvr/imu', Imu, self.imu_callback)
        self.imu_publisher = rospy.Publisher('imu', Imu, queue_size=10)
        self.odom_subscriber = rospy.Subscriber('/rvr/odom', Odometry, self.odom_callback)
        self.odom_publisher = rospy.Publisher('/rvr/odom/correct', Odometry, queue_size=10)

    def imu_callback(self, imu_msg):
        # Modify the frame_id field to "base_link"
        imu_msg.header.frame_id = "base_link"
        # Publish the modified IMU message
        self.imu_publisher.publish(imu_msg)

    def odom_callback(self, odom_msg):
        # Modify the frame_id and child_frame_id fields
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        # Publish the modified odometry message
        self.odom_publisher.publish(odom_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        message_republisher = MessageRepublisher()
        message_republisher.run()
    except rospy.ROSInterruptException:
        pass

