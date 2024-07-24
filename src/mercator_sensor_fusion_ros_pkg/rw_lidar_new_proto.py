#!/usr/bin/env python

# license removed for brevity

import rospy
import geometry_msgs.msg as gmsg
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from math import cos, sin
from nav_msgs.msg import Odometry
import tf
import tf2_ros
import tf2_geometry_msgs

TH = -3.14
SPEED = 0.5
X = 1.85
Y = -3.07

rospy.init_node('publisher', anonymous=True)
last_time = rospy.Time.now()
config_pub = rospy.Publisher('/corrected_scan', LaserScan, queue_size=1000)
pub = rospy.Publisher('/rvr/wheels_speed', Float32MultiArray, queue_size=1000)
odom_pub = rospy.Publisher('odom', Odometry, queue_size=1000)

def callback(data):
    values = list(data.ranges)
    min_dist = 1000
    idx = 0
    for i in range(136, 368): # for i in range(262, 494):
        if values[i] < min_dist and values[i] > 0:
            min_dist = values[i]
            idx = i
    obstacle_avoidance(idx, min_dist)

def callback_wheels(data):
    rospy.loginfo(data.layout)
    rospy.loginfo(type(data.layout))
    rospy.loginfo(data.layout.dim)
    rospy.loginfo(data.layout.data_offset)
    rospy.loginfo(data.data)

def obstacle_avoidance(idx, dist):
    data_to_send = Float32MultiArray()
    if idx >= 505:
        idx -= 505
    if dist > 0.9: # 0.37
        left = SPEED
        right = SPEED
    else:
        if 368 >= idx >= 252: #if 484 >= idx >= 378:
            left = SPEED
            right = -SPEED
        elif 252 > idx >= 136: #elif 378 > idx >= 272:
            left = -SPEED
            right = SPEED
        else:
            left = SPEED
            right = SPEED
    data_to_send.data = (left, right)
    data_to_send.layout.dim.append(MultiArrayDimension())
    data_to_send.layout.dim[0].label = 'wheel_vel'
    data_to_send.layout.dim[0].size = 2
    data_to_send.layout.dim[0].stride = 1
    data_to_send.layout.data_offset = 0
    pub.publish(data_to_send)
    send_odom(left, right)

def send_odom(left, right):
    global TH, X, Y, last_time
    if left == right:
        vx = left * cos(TH)
        vy = left * sin(TH)
        vth = 0
    else:
        vx = 0
        vy = 0
        vth = (right - left) / 0.26

    current_time = rospy.Time.now()
    dt = (current_time - last_time).to_sec()
    delta_x = vx * dt
    delta_y = vy * dt
    delta_th = vth * dt

    X += delta_x
    Y += delta_y
    TH += delta_th

    q = tf.transformations.quaternion_from_euler(0, 0, TH)
    odom_quat = gmsg.Quaternion(*q)
    br = tf2_ros.TransformBroadcaster()
    odom_trans = gmsg.TransformStamped()
    odom_trans.header.stamp = current_time
    odom_trans.header.frame_id = "odom"
    odom_trans.child_frame_id = "base_link"

    odom_trans.transform.translation.x = X
    odom_trans.transform.translation.y = Y
    odom_trans.transform.translation.z = 0.0
    odom_trans.transform.rotation = odom_quat

    br.sendTransform(odom_trans)

    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"
    odom.pose.pose.position.x = X
    odom.pose.pose.position.y = Y
    odom.pose.pose.position.z = 0.0
    odom.pose.pose.orientation = odom_quat

    odom.child_frame_id = "base_link"
    odom.twist.twist.linear.x = vx
    odom.twist.twist.linear.y = vy
    odom.twist.twist.angular.z = vth

    odom_pub.publish(odom)
    last_time = current_time

def listener():
    rospy.Subscriber("scan", LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
