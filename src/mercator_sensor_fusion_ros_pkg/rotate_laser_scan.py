#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

class RotateLaserScanNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('rotate_laserscan_node')

        # Create a publisher for the rotated LaserScan messages
        self.rotated_scan_pub = rospy.Publisher('/rotated_scan', LaserScan, queue_size=10)

        self.rotation_angle_in_radians = rospy.get_param('~rotation_angle_in_radians', 3.14159)

        # Subscribe to the /scan topic
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        self.run()

    def rotate_laserscan(self, scan_msg):
        """
        Rotate the LaserScan data by 180 degrees by modifying angle_min and angle_max.
        """
        rotated_scan = LaserScan()
        rotated_scan.header = scan_msg.header
        rotated_scan.header.stamp = rospy.Time.now()
        rotated_scan.angle_min = scan_msg.angle_min + self.rotation_angle_in_radians  # Add angle (in radians)
        rotated_scan.angle_max = scan_msg.angle_max + self.rotation_angle_in_radians  # Add angle (in radians)
        rotated_scan.angle_increment = scan_msg.angle_increment
        rotated_scan.time_increment = scan_msg.time_increment
        rotated_scan.scan_time = scan_msg.scan_time
        rotated_scan.range_min = scan_msg.range_min
        rotated_scan.range_max = scan_msg.range_max

        rotated_scan.ranges = scan_msg.ranges
        rotated_scan.intensities = scan_msg.intensities

        return rotated_scan

    def scan_callback(self, scan_msg):
        """
        Callback function for the /scan topic. It rotates the LaserScan data
        and republishes it to the /rotated_scan topic.
        """
        rotated_scan = self.rotate_laserscan(scan_msg)
        self.rotated_scan_pub.publish(rotated_scan)

    def run(self):
        # Spin to keep the script running and processing callbacks
        rospy.spin()

if __name__ == '__main__':
    node = RotateLaserScanNode()
    node.spin()
