#!/usr/bin/env python

import rospy
import random
import time
from sensor_msgs.msg import Range
from teraranger_array.msg import RangeArray
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension

class MercatorRwNode:
    def __init__(self):
        rospy.init_node('mercator_rw_node', anonymous=True)

        # Retrieve parameters from the parameter server
        self.speed = rospy.get_param('~speed', 0.3)
        self.min_dist_threshold = rospy.get_param('~min_dist_threshold', 0.4)
        self.sensor_angles = rospy.get_param('~sensor_angles', "[-150, -80, -20, -10, 10, 20, 80, 150]")
        self.sensor_angles = self.string_to_list(self.sensor_angles)
        self.dodge_angle_range = rospy.get_param('~dodge_angle_range', 45)  # range of angles to dodge an obstacle

        self.pub = rospy.Publisher('/rvr/wheels_speed', Float32MultiArray, queue_size=1)
        rospy.Subscriber("/ranges", RangeArray, self.callback, queue_size=1)

        self.message_counter = 0  # Counter to track messages

        self.straight_counter = 0  # Counter to track consecutive straight movements
        self.last_stop_time = 0  # Time when the last stop occurred
        self.can_stop_again = True  # Flag to control if the robot can stop again

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
        self.message_counter += 1

        # Ignore every other message
        if self.message_counter % 2 == 0:
            rospy.loginfo("Message ignored")
            return

        ranges = data.ranges
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
        min_dist = float('inf')
        second_min_dist = float('inf')
        min_idx = 0
        second_min_idx = 0

        for i, range_msg in enumerate(ranges):
            angle = self.sensor_angles[i]
            if -self.dodge_angle_range <= angle <= self.dodge_angle_range:
                if 0 < range_msg.range < min_dist:
                    # second_min_dist = min_dist
                    # second_min_idx = min_idx
                    min_dist = range_msg.range
                    min_idx = i
                # elif min_dist < range_msg.range < second_min_dist:
                #     second_min_dist = range_msg.range
                #     second_min_idx = i
        
        rospy.loginfo("Second min dist: %f", min_dist)
        rospy.loginfo("Second min idx: %d", min_idx)
        self.obstacle_avoidance(ranges, min_idx, min_dist)
    
    def obstacle_avoidance(self, ranges, idx, dist):
        data_to_send = Float32MultiArray()
        data_to_send.layout.dim.append(MultiArrayDimension())
        data_to_send.layout.dim[0].label = 'wheel_vel'
        data_to_send.layout.dim[0].size = 2
        data_to_send.layout.dim[0].stride = 1
        data_to_send.layout.data_offset = 0

        if dist > self.min_dist_threshold:
            left, right = self.go_straight()
            data_to_send.data = [left, right]
            self.pub.publish(data_to_send)
            rospy.loginfo("Going straight")
            self.straight_counter += 1
        else:
            self.straight_counter = 0  # Reset straight counter if not going straight

            # Check all ranges for specific angle conditions
            left_turn = False
            right_turn = False

            for i, range_msg in enumerate(ranges):
                angle = self.sensor_angles[i]
                if range_msg.range < self.min_dist_threshold:
                    if 45 <= angle <= 90:
                        right_turn = True
                    elif -90 <= angle <= -45:
                        left_turn = True

            if left_turn:
                left, right = self.go_left()
            elif right_turn:
                left, right = self.go_right()
            else:
                # Randomly choose left or right
                if random.choice([True, False]):
                    left, right = self.go_left()
                else:
                    left, right = self.go_right()
            
            # Randomly choose a time duration between 0 and 1 second
            duration = (0.25/self.speed)*random.uniform(0.2, 0.8)
            rospy.loginfo("Avoiding obstacle: Turning for %f seconds", duration)
            
            # Send the chosen direction
            data_to_send.data = [left, right]
            self.pub.publish(data_to_send)

            # Sleep for the duration
            time.sleep(duration)
            
            # Stop after the duration
            left, right = 0, 0

        # Check if the robot needs to stop
        current_time = time.time()
        if self.straight_counter >= 3 and self.can_stop_again:
            rospy.loginfo("Stopping robot for a random amount of time")
            left, right = 0, 0

            data_to_send.data = [left, right]
            self.pub.publish(data_to_send)

            # stop_duration = random.uniform(1, 3)
            # time.sleep(stop_duration)

            self.straight_counter = 0
            self.last_stop_time = time.time()
            self.can_stop_again = False

        # Check if the stop cooldown period has passed
        if not self.can_stop_again and (current_time - self.last_stop_time) > 15:
            self.can_stop_again = True

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = MercatorRwNode()
