#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import PoseArray
from threading import Lock
from time import time

class CalibrationCamLidarNode:
    def __init__(self):
        rospy.init_node('calibration_cam_lidar_node')

        # Data storage
        self.data_for_cam_calibration = {
            "cam_poses": [],
            "ground_truth_poses": []
        }

        self.data_for_lidar_calibration = {
            "lidar_poses": [],
            "ground_truth_poses": []
        }

        self.last_data = {
            "cam_poses": None,
            "lidar_poses": None,
            "ground_truth_poses": None
        }

        self.locks = {
            "cam_poses": Lock(),
            "lidar_poses": Lock(),
            "ground_truth_poses": Lock()
        }

        # Subscribe to topics
        self.subscribers = {
            "cam_poses": rospy.Subscriber("cam_poses", PoseArray, self.callback, callback_args="cam_poses"),
            "lidar_poses": rospy.Subscriber("lidar_poses", PoseArray, self.callback, callback_args="lidar_poses"),
            "ground_truth_poses": rospy.Subscriber("ground_truth_poses", PoseArray, self.callback, callback_args="ground_truth_poses")
        }

        # Timing for 60 seconds collection
        self.collect_time = rospy.get_param("~collect_time", 60.0)
        self.data_max_size = rospy.get_param("~data_max_size", 50)
        self.start_time = time()

        self.run()

    def callback(self, data, args):
        topic_key = args
        current_time = time()
        if current_time - self.start_time < self.collect_time and
            (len(self.data_for_cam_calibration["cam_poses"]) < self.data_max_size or 
            len(self.data_for_lidar_calibration["lidar_poses"]) < self.data_max_size):
            with self.locks[topic_key]:
                self.last_data[topic_key] = data
            if self.last_data["ground_truth_poses"] is None:
                return
            if topic_key == "lidar_poses":
                with self.locks[topic_key]:
                    self.data_for_lidar_calibration[topic_key].append(data)
                with self.locks["ground_truth_poses"]:
                    self.data_for_lidar_calibration["ground_truth_poses"].append(self.last_data["ground_truth_poses"])
            elif topic_key == "cam_poses":
                with self.locks[topic_key]:
                    self.data_for_cam_calibration[topic_key].append(data)
                with self.locks["ground_truth_poses"]:
                    self.data_for_cam_calibration["ground_truth_poses"].append(self.last_data["ground_truth_poses"])
        else:
            # Unregister to stop listening after the time limit
            self.subscribers[topic_key].unregister()

    def compute_calibration_matrices(self):
        # Process the data into numpy arrays for easier handling
        rospy.loginfo(self.data_for_lidar_calibration.items())
        processed_data_for_lidar_calibration = {key: self.convert_poses_to_np_array(poses) for key, poses in self.data_for_lidar_calibration.items()}
        processed_data_for_cam_calibration = {key: self.convert_poses_to_np_array(poses) for key, poses in self.data_for_cam_calibration.items()}

        # Calculate transformations
        # Assuming ground_truth_poses as the reference
        if "ground_truth_poses" in processed_data_for_lidar_calibration:
            ground_truth = processed_data_for_lidar_calibration["ground_truth_poses"]
            for key in ["lidar_poses"]:
                if key in processed_data_for_lidar_calibration:
                    transformation_matrix = self.calculate_transformation(ground_truth, processed_data_for_lidar_calibration[key])
                    print(f"Transformation matrix for {key} to ground_truth:")
                    print(transformation_matrix)
        
        if "ground_truth_poses" in processed_data_for_cam_calibration:
            ground_truth = processed_data_for_cam_calibration["ground_truth_poses"]
            for key in ["cam_poses"]:
                if key in processed_data_for_cam_calibration:
                    transformation_matrix = self.calculate_transformation(ground_truth, processed_data_for_cam_calibration[key])
                    print(f"Transformation matrix for {key} to ground_truth:")
                    print(transformation_matrix)

        # transformation_matrix = self.calculate_transformation(processed_data_for_cam_calibration["lidar_poses"],
        #                                                      processed_data_for_cam_calibration["cam_poses"])
        # print(f"Transformation matrix for cam_poses to lidar_poses:")
        # print(transformation_matrix)

    def convert_poses_to_np_array(self, pose_arrays):
        """Convert PoseArray to numpy array of [x, y] coordinates."""
        points = []
        for pose_array in pose_arrays:
            for pose in pose_array.poses:
                points.append([pose.position.x, pose.position.y])
        return np.array(points)

    def calculate_transformation(self, reference, data):
        """Calculate transformation matrix to align data to reference."""
        assert len(reference) >= 2 and len(data) >= 2, "Need at least two points to calculate transformation"
        
        # Compute centroids
        centroid_ref = np.mean(reference, axis=0)
        centroid_data = np.mean(data, axis=0)
        
        # Centralize data
        ref_centralized = reference - centroid_ref
        data_centralized = data - centroid_data
        
        # Compute rotation
        H = np.dot(data_centralized.T, ref_centralized)
        U, S, Vt = np.linalg.svd(H)
        R = np.dot(Vt.T, U.T)
        
        # Compute translation
        t = centroid_ref - np.dot(R, centroid_data)
        
        # Combine into affine transformation matrix
        transformation_matrix = np.eye(3)
        transformation_matrix[:2, :2] = R
        transformation_matrix[:2, 2] = t
        
        return transformation_matrix
    
    def run(self):
        rospy.sleep(self.collect_time + 0.5)  # Ensure all data is collected
        self.compute_calibration_matrices()
        rospy.spin()  # Keep node running if not all topics are unregistered


if __name__ == '__main__':
    collector = CalibrationCamLidarNode()
    rospy.sleep(collector.collect_time + 0.5)  # Ensure all data is collected
    collector.compute_calibration_matrices()
    rospy.spin()  # Keep node running if not all topics are unregistered
