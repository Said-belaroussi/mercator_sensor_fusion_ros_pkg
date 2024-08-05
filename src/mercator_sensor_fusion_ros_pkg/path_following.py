#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension

class PathFollowingNode:
    def __init__(self):
        rospy.init_node('path_following_node', anonymous=True)

        self.speed = rospy.get_param('~speed', 0.3)
        self.pub = rospy.Publisher('/rvr/wheels_speed', Float32MultiArray, queue_size=1)

        self.run()

    def move_straight(self, distance):
        data_to_send = Float32MultiArray()
        data_to_send.layout.dim.append(MultiArrayDimension())
        data_to_send.layout.dim[0].label = 'wheel_vel'
        data_to_send.layout.dim[0].size = 2
        data_to_send.layout.dim[0].stride = 1
        data_to_send.layout.data_offset = 0

        left = self.speed
        right = self.speed
        data_to_send.data = [left, right]
        self.pub.publish(data_to_send)

        # Calculate the time to move the given distance
        duration = distance / self.speed
        rospy.loginfo("Moving straight for %f seconds", duration)
        time.sleep(duration)

        # Stop the robot
        data_to_send.data = [0, 0]
        self.pub.publish(data_to_send)

    def turn(self, direction, angle=90):
        data_to_send = Float32MultiArray()
        data_to_send.layout.dim.append(MultiArrayDimension())
        data_to_send.layout.dim[0].label = 'wheel_vel'
        data_to_send.layout.dim[0].size = 2
        data_to_send.layout.dim[0].stride = 1
        data_to_send.layout.data_offset = 0

        # Calculate the time to turn the given angle
        turn_speed = self.speed
        duration = angle / (self.speed * 1300)  # Assuming it takes 1 second to turn 45 degrees

        if direction == 'left':
            left = -turn_speed
            right = turn_speed
        elif direction == 'right':
            left = turn_speed
            right = -turn_speed

        data_to_send.data = [left, right]
        self.pub.publish(data_to_send)

        rospy.loginfo("Turning %s for %f seconds", direction, duration)
        time.sleep(duration)

        # Stop the robot
        data_to_send.data = [0, 0]
        self.pub.publish(data_to_send)

    def run(self):
        while not rospy.is_shutdown():
            distances = [1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1]

            for distance in distances:
                self.move_straight(distance)
                self.turn('left')
                self.move_straight(0.1)
                self.turn('left')

            # Return to the starting position
            self.turn('left')
            self.turn('left')

if __name__ == '__main__':
    try:
        node = PathFollowingNode()
    except rospy.ROSInterruptException:
        pass
