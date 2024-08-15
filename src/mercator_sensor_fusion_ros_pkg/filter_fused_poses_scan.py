#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray
import numpy as np

class FilterFusedPosesScanNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('filter_fused_poses_scan_node')

        # Create a publisher for the rotated LaserScan messages
        self.rotated_scan_pub = rospy.Publisher('/rotated_scan', LaserScan, queue_size=10)

        self.rotation_angle_in_radians = rospy.get_param('~rotation_angle_in_radians', 3.14159)

        # Subscribe to the /scan topic
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # Subscribe to the /fused_poses topic
        self.last_pose_array = None
        self.last_pose_array_time = None
        rospy.Subscriber('/fused_poses', PoseArray, self.pose_array_callback)

        self.run()

    def pose_array_callback(self, pose_array_msg):
        """
        Callback function for the /fused_poses topic.
        Stores the most recent PoseArray message and its timestamp.
        """
        self.last_pose_array = pose_array_msg
        self.last_pose_array_time = rospy.Time.now()

    def rotate_laserscan(self, scan_msg):
        """
        Rotate the LaserScan data by the specified angle.
        """
        # Modify the LaserScan ranges based on the PoseArray
        if self.is_pose_array_recent():
            self.modify_scan_based_on_poses(scan_msg)

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

        rotated_scan.ranges = list(scan_msg.ranges)
        rotated_scan.intensities = scan_msg.intensities

        return rotated_scan

    def is_pose_array_recent(self):
        """
        Checks if the last received PoseArray message is recent (within 0.4 seconds).
        """
        if self.last_pose_array_time:
            time_diff = rospy.Time.now() - self.last_pose_array_time
            return time_diff.to_sec() <= 0.4
        return False

    def modify_scan_based_on_poses(self, scan_msg):
        """
        Modify the LaserScan ranges based on the most recent PoseArray message.
        Any LaserScan readings that intersect with a pose within a 0.12m radius
        are set to max_range.
        """
        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment
        ranges = scan_msg.ranges
        ranges = list(ranges)
        # if type(ranges) != list:
        #     rospy.loginfo(ranges)

        for pose in self.last_pose_array.poses:
            pose_x = pose.position.x
            pose_y = pose.position.y

            for i in range(len(ranges)):
                # Adjust the angle calculation considering 0 angle is along Y-axis
                angle = angle_min + i * angle_increment + np.pi / 2  # Adjusting for Y-axis as 0 angle
                scan_x = ranges[i] * np.cos(angle)
                scan_y = ranges[i] * np.sin(angle)

                distance = np.sqrt((pose_x - scan_x)**2 + (pose_y - scan_y)**2)
                if distance <= 0.12:
                    ranges[i] = scan_msg.range_max

        scan_msg.ranges = tuple(ranges)

    def scan_callback(self, scan_msg):
        """
        Callback function for the /scan topic. It rotates the LaserScan data,
        modifies it based on the latest PoseArray, and republishes it.
        """
        rotated_scan = self.rotate_laserscan(scan_msg)
        self.rotated_scan_pub.publish(rotated_scan)

    def run(self):
        # Spin to keep the script running and processing callbacks
        rospy.spin()

if __name__ == '__main__':
    node = RotateLaserScanNode()
    node.run()
