#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension

class MercatorRwNode:
    def __init__(self):
        rospy.init_node('mercator_rw_node', anonymous=True)

        # Retrieve parameters from the parameter server
        self.speed = rospy.get_param('~speed', 0.25)
        self.min_dist_threshold = rospy.get_param('~min_dist_threshold', 0.5)
        self.sensor_angles = rospy.get_param('~sensor_angles', "[-150, -80, -20, -10, 10, 20, 80, 150]")
        self.sensor_angles = self.string_to_list(self.sensor_angles)
        self.dodge_angle_range = rospy.get_param('~dodge_angle_range', 45)  # range of angles to dodge an obstacle

        self.pub = rospy.Publisher('/rvr/wheels_speed', Float32MultiArray, queue_size=1000)
        rospy.Subscriber("/ranges", Range, self.callback)

        self.run()

    def string_to_list(self, string):
        try:
            # Use eval to convert the string to a list
            list = eval(string)
        except (NameError, SyntaxError):
            raise ValueError("Invalid string format. Please use proper list of lists syntax.")

        return list

    def is_within_dodge_range(self, angle):
        return (-self.dodge_angle_range <= angle <= self.dodge_angle_range)

    def callback(self, data):
        ranges = data.ranges
        # min_dist = float('inf')
        # min_idx = 0

        # for i, range_msg in enumerate(data.ranges):
        #     if range_msg.range < min_dist and range_msg.range > 0:
        #         min_dist = range_msg.range
        #         min_idx = i
        self.obstacle_detection(ranges)

    def go_straight(self):
        left = self.speed
        right = self.speed

        return left, right

    def go_right(self):
        left = self.speed
        right = -self.speed

        return left, right

    def go_left(self):
        left = -self.speed
        right = self.speed

        return left, right

    def obstacle_detection(self, ranges):
        data_to_send = Float32MultiArray()

        min_dist = float('inf')
        min_idx = 0

        for i, range_msg in enumerate(ranges):
            angle = self.sensor_angles[i]
            if -self.dodge_angle_range <= angle <= self.dodge_angle_range:
                if range_msg.range < min_dist and range_msg.range > 0:
                    min_dist = range_msg.range
                    min_idx = i
        
        self.obstacle_avoidance(min_idx, min_dist)
    
    def obstacle_avoidance(self, idx, dist):
        if dist > self.min_dist_threshold:
            left, right = self.go_straight()
        else:
            angle = self.sensor_angles[idx]
            if -self.dodge_angle_range <= angle <= 0:
                left, right = self.go_right()
            else:
                left, right = self.go_left()

        data_to_send.data = [left, right]
        data_to_send.layout.dim.append(MultiArrayDimension())
        data_to_send.layout.dim[0].label = 'wheel_vel'
        data_to_send.layout.dim[0].size = 2
        data_to_send.layout.dim[0].stride = 1
        data_to_send.layout.data_offset = 0
        self.pub.publish(data_to_send)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = MercatorRwNode()
