import rosbag
import numpy as np
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
import rospy

class RosbagSyncerNode:
    def __init__(self):
        rospy.init_node('rosbag_syncer', anonymous=True)
        self.bag1_path = rospy.get_param('~bag1_path', 'rvr_40_calib.bag')
        self.bag2_path = rospy.get_param('~bag2_path', 'rvr_40_tycho_calibration.bag')
        self.bag1_topic = rospy.get_param('~bag1_topic', '/rvr/odom')
        self.bag2_topic = rospy.get_param('~bag2_topic', '/tf')
        self.frame_id = rospy.get_param('~frame_id', 'base_link_23')

        self.output_bag1_path = self.bag1_path.replace('.bag', '_synced.bag')
        self.output_bag2_path = self.bag2_path.replace('.bag', '_synced.bag')

        self.run()

    def extract_and_print_topics(self, bag):
        rospy.loginfo("Topics in bag:")
        rospy.loginfo(bag.get_type_and_topic_info())

    def extract_positions_from_odometry(self, bag):
        positions = []
        timestamps = []
        rospy.loginfo(self.bag1_topic)

        for topic, msg, t in bag.read_messages(topics=[self.bag1_topic]):
            position = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
            positions.append(position)
            timestamps.append(t.to_sec())

        return np.array(positions), np.array(timestamps)

    def extract_positions_from_tf(self, bag):
        positions = []
        timestamps = []
        rospy.loginfo(self.bag2_topic)
        for topic, msg, t in bag.read_messages(topics=[self.bag2_topic]):
            for transform in msg.transforms:
                if transform.child_frame_id == self.frame_id:
                    position = (transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z)
                    positions.append(position)
                    timestamps.append(t.to_sec())
        return np.array(positions), np.array(timestamps)

    def detect_significant_movement(self, positions, threshold=0.01):
        initial_position = positions[0]
        distances = np.linalg.norm(positions - initial_position, axis=1)
        movement_indices = np.where(distances > threshold)[0]
        if movement_indices.size == 0:
            return None
        return movement_indices[0]

    def sync_rosbags(self):
        with rosbag.Bag(self.bag1_path) as bag1, rosbag.Bag(self.bag2_path) as bag2:
            rospy.loginfo("Extracting topics from bags")
            rospy.loginfo(self.bag1_path)
            self.extract_and_print_topics(bag1)
            rospy.loginfo(self.bag2_path)
            self.extract_and_print_topics(bag2)
            positions1, timestamps1 = self.extract_positions_from_odometry(bag1)
            positions2, timestamps2 = self.extract_positions_from_tf(bag2)

            rospy.loginfo(positions1)
            rospy.loginfo(positions2)
            move_index1 = self.detect_significant_movement(positions1)
            move_index2 = self.detect_significant_movement(positions2)

            if move_index1 is None or move_index2 is None:
                rospy.loginfo(f"move_index1: {move_index1}")
                rospy.loginfo(f"move_index2: {move_index2}")
                raise Exception("No significant movement detected in one of the bags")

            start_time1 = timestamps1[move_index1]
            start_time2 = timestamps2[move_index2]

            start_time1 -= timestamps1[0]
            start_time2 -= timestamps2[0]
            rospy.loginfo(f"start_time1: {start_time1}")
            rospy.loginfo(f"start_time2: {start_time2}")

            # Find the earlier start time
            if start_time1 < start_time2:
                self._write_synced_bag(self.bag1_path, self.output_bag1_path, timestamps1[0])
                self._write_synced_bag(self.bag2_path, self.output_bag2_path, start_time2 - start_time1 + timestamps2[0])
            else:
                self._write_synced_bag(self.bag2_path, self.output_bag2_path, timestamps2[0])
                self._write_synced_bag(self.bag1_path, self.output_bag1_path, start_time1 - start_time2 + timestamps1[0])
            # self._write_synced_bag(self.bag1_path, self.output_bag1_path, start_time1)
            # self._write_synced_bag(self.bag2_path, self.output_bag2_path, start_time2)

    def _write_synced_bag(self, input_bag_path, output_bag_path, start_time):
        with rosbag.Bag(input_bag_path) as inbag, rosbag.Bag(output_bag_path, 'w') as outbag:
            for topic, msg, t in inbag.read_messages():
                if t.to_sec() >= start_time:
                    outbag.write(topic, msg, t)

    def run(self):
        self.sync_rosbags()
        rospy.loginfo("Rosbags have been synced and saved")

if __name__ == "__main__":
    syncer = RosbagSyncerNode()
    syncer.sync_rosbags()
    print("Rosbags have been synced and saved as robot_synced.bag and tracking_system_synced.bag")
