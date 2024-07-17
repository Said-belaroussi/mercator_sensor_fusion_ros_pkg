import rosbag
import numpy as np
from geometry_msgs.msg import PoseArray
from tf2_msgs.msg import TFMessage
import os

def extract_positions_from_cam_poses(bag):
    positions = []
    timestamps = []
    for topic, msg, t in bag.read_messages(topics=['/cam_poses']):
        for pose in msg.poses:
            position = (pose.position.x, pose.position.y, pose.position.z)
            positions.append(position)
            timestamps.append(t.to_sec())
    return np.array(positions), np.array(timestamps)

def extract_positions_from_tf(bag):
    positions = []
    timestamps = []
    for topic, msg, t in bag.read_messages(topics=['/tf']):
        for transform in msg.transforms:
            if transform.child_frame_id == "base_link_23":
                position = (transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z)
                positions.append(position)
                timestamps.append(t.to_sec())
    return np.array(positions), np.array(timestamps)

def detect_significant_movement(positions, threshold=0.03):
    diffs = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    movement_indices = np.where(diffs > threshold)[0]
    if movement_indices.size == 0:
        return None
    return movement_indices[0]

def sync_rosbags(bag1_path, bag2_path, output_bag1_path, output_bag2_path):
    bag1 = rosbag.Bag(bag1_path)
    bag2 = rosbag.Bag(bag2_path)

    positions1, timestamps1 = extract_positions_from_cam_poses(bag1)
    positions2, timestamps2 = extract_positions_from_tf(bag2)

    move_index1 = detect_significant_movement(positions1)
    move_index2 = detect_significant_movement(positions2)

    if move_index1 is None or move_index2 is None:
        raise Exception("No significant movement detected in one of the bags")

    start_time1 = timestamps1[move_index1]
    start_time2 = timestamps2[move_index2]

    bag1.close()
    bag2.close()

    with rosbag.Bag(output_bag1_path, 'w') as outbag1:
        for topic, msg, t in rosbag.Bag(bag1_path).read_messages():
            if t.to_sec() >= start_time1:
                outbag1.write(topic, msg, t)

    with rosbag.Bag(output_bag2_path, 'w') as outbag2:
        for topic, msg, t in rosbag.Bag(bag2_path).read_messages():
            if t.to_sec() >= start_time2:
                outbag2.write(topic, msg, t)

if __name__ == "__main__":
    bag1_path = 'rvr_40_calib.bag'
    bag2_path = 'rvr_40_tycho_calibration.bag'
    output_bag1_path = 'robot_synced.bag'
    output_bag2_path = 'tracking_system_synced.bag'

    sync_rosbags(bag1_path, bag2_path, output_bag1_path, output_bag2_path)
    print("Rosbags have been synced and saved as robot_synced.bag and tracking_system_synced.bag")