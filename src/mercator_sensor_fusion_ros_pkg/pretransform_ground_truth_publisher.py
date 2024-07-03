#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, PoseArray
from tf import TransformListener
from tf2_msgs.msg import TFMessage

class TFPoseSubscriber:
    def __init__(self, robot_names):
        self.robot_names = robot_names
        self.tf_listener = TransformListener()
        self.robot_poses = {}
        self.ground_truth_pub = rospy.Publisher('ground_truth_poses', PoseArray, queue_size=10)
        self.tf_sub = rospy.Subscriber('/tf', TFMessage, self.tf_callback)

    def tf_callback(self, tf_msg):
        for transform in tf_msg.transforms:
            child_frame_id = transform.child_frame_id
            if child_frame_id in self.robot_names:
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                pose = Pose(position=translation, orientation=rotation)
                self.robot_poses[child_frame_id] = pose

        if set(self.robot_poses.keys()) == set(self.robot_names):
            pose_array_msg = PoseArray()
            for robot_name in self.robot_names:
                pose_array_msg.poses.append(self.robot_poses[robot_name])
            self.ground_truth_pub.publish(pose_array_msg)

def main():
    rospy.init_node('tf_pose_subscriber')

    # Get list of robot names from parameter server
    robot_names = rospy.get_param('~robot_names', ["base_link_38"])

    if not robot_names:
        rospy.logerr("List of robot names not provided. Exiting...")
        return

    tf_pose_subscriber = TFPoseSubscriber(robot_names)
    rospy.spin()

if __name__ == '__main__':
    main()
