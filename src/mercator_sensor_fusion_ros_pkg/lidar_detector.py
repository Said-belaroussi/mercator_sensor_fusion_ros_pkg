#!/usr/bin/env python

import rospy
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
import math
import numpy as np
from scipy.spatial import KDTree
from sklearn.cluster import DBSCAN
from visualization_msgs.msg import Marker
import threading

def polar_to_cartesian(range_data, angle_min, angle_increment):
    """Convert polar coordinates from the LiDAR data to Cartesian coordinates with 0 degrees at the Y axis."""
    angles = angle_min + np.arange(len(range_data)) * angle_increment
    adjusted_angles = angles + np.pi / 2
    return np.vstack((range_data * np.cos(adjusted_angles), range_data * np.sin(adjusted_angles))).T

def euclidean_clustering(points, distance_threshold, min_points, max_points):
    """Cluster points based on Euclidean distance."""
    tree = KDTree(points)
    unvisited = set(range(len(points)))
    clusters = []

    while unvisited:
        current_point = unvisited.pop()
        current_cluster = [current_point]
        search_queue = [current_point]

        while search_queue:
            point = search_queue.pop(0)
            neighbors = tree.query_ball_point(points[point], distance_threshold)

            for neighbor in neighbors:
                if neighbor in unvisited:
                    unvisited.remove(neighbor)
                    current_cluster.append(neighbor)
                    search_queue.append(neighbor)

        if min_points <= len(current_cluster) <= max_points:
            clusters.append([points[i] for i in current_cluster])

    return clusters

def dbscan_clustering(points, eps=0.1, min_samples=2):
    """Cluster points using the DBSCAN algorithm."""
    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(points)
    labels = clustering.labels_
    unique_labels = set(labels)
    clusters = [points[labels == label] for label in unique_labels if label != -1]
    return clusters

class LidarDetectorNode:
    def __init__(self):
        rospy.init_node('object_depth_finder_node', anonymous=True)

        # Parameters
        self.robot_name = rospy.get_param("~robot_name", "robot")  # Default robot name is robot
        self.camera_fov_deg = rospy.get_param("~camera_fov_deg", 95)  # Default camera FOV is 95 degrees
        self.camera_resolution = rospy.get_param("~camera_resolution", 256)  # Default camera resolution is 256 pixels
        self.detection_method = rospy.get_param("~detection_method", "max_strictly_below_median")  # Default detection method is max_strictly_below_median
        self.visualize = rospy.get_param("~visualize", True)  # Default visualization is enabled

        # Clustering parameters are adjusted for best results
        self.clustering_distance_threshold = rospy.get_param("~clustering_distance_threshold", 0.15)  # Default clustering distance threshold is 0.15 meters
        self.clustering_min_points = rospy.get_param("~clustering_min_points", 2)  # Default clustering min points is 2
        self.clustering_max_points = rospy.get_param("~clustering_max_points", 30)  # Default clustering max points is 30
        
        self.fully_assisted_detection_distance_threshold = rospy.get_param("~fully_assisted_detection_distance_threshold", 0.3)  # Default fully assisted detection distance threshold is 0.1 meters
        
        # Variable to store last detections
        self.last_detections = None
        self.last_detections_lock = threading.Lock()

        # Variable to store last camera poses
        self.last_camera_poses = None
        self.last_camera_poses_lock = threading.Lock()

        # Calculate the start angle of the Lidar scan based on camera FOV
        self.start_angle = -(self.camera_fov_deg / 2) * (math.pi / 180) # TODO: remove the hardcoded 90 degrees one robot is finished

        # Subscribers
        self.oak_sub = rospy.Subscriber('oak', Detection2DArray, self.oak_callback)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)


        if self.detection_method == "fully_assisted_detection":
            # Fully assisted detection requires the position from the camera in the LiDAR frame of the object
            self.camera_sub = rospy.Subscriber('cam_poses', PoseArray, self.camera_callback)

        # Publisher
        self.positions_pub = rospy.Publisher('lidar_poses', PoseArray, queue_size=10)

        if self.visualize:
            self.marker_pub = rospy.Publisher('detected_robots', Marker, queue_size=10)

        self.run()

    def visualize_detected_robot(self, positions):
        marker = Marker()
        marker.header = Header()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.robot_name
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        for position in positions.poses:
            marker.points.append(position.position)

        self.marker_pub.publish(marker)

    def min_max_angles(self, bbox):
        min_x = bbox.center.x - bbox.size_x / 2
        max_x = bbox.center.x + bbox.size_x / 2
        min_y = bbox.center.y - bbox.size_y / 2
        max_y = bbox.center.y + bbox.size_y / 2

        # Convert the coordinates to angles
        # The Lidar gives angles positive counterclockwise with 0° at its head (where motor is located) and ranges from -180° to 180°
        # 0 values are given when measurement failed (most of the time cuz of dark surface)
        min_angle = self.start_angle + (((self.camera_resolution - max_x)/self.camera_resolution)*self.camera_fov_deg* (math.pi / 180))
        max_angle = self.start_angle + (((self.camera_resolution - min_x)/self.camera_resolution)*self.camera_fov_deg* (math.pi / 180))
        # min_angle = - math.pi
        # max_angle = math.pi

        return min_angle, max_angle

    def detect_robots(self, data):
        if self.detection_method == "standalone_euclidean_clustering":
            return self.standalone_euclidean_clustering(data)
        elif self.detection_method == "standalone_dbscan_clustering":
            return self.standalone_dbscan_clustering(data)
        elif self.detection_method == "partially_assisted_euclidean_clustering":
            return self.process_detection(data, self.partially_assisted_euclidean_clustering)
        elif self.detection_method == "partially_assisted_dbscan_clustering":
            return self.process_detection(data, self.partially_assisted_dbscan_clustering)
        elif self.detection_method == "minimum_distance_within_bbox":
            return self.process_detection(data, self.minimum_distance_within_bbox_depth)
        elif self.detection_method == "fully_assisted_detection":
            return self.fully_assisted_detection(data)
        # elif self.detection_method == "max_strictly_below_median":
        #     return self.process_detection(data, self.max_strictly_below_median_depth)
        # elif self.detection_method == "split_from_background":
        #     return self.process_detection(data, self.split_from_background_depth)
        else:
            rospy.loginfo("deeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee")
            rospy.loginfo(self.detection_method)
            raise ValueError("Invalid detection method")
        
    def fully_assisted_detection(self, data):
        
        positions = PoseArray()

        with self.last_camera_poses_lock:
            camera_poses = self.last_camera_poses
        
        if camera_poses is None:
            rospy.logwarn("No camera poses received yet.")
            return positions

        positions.header.frame_id = camera_poses.header.frame_id 
        ranges = np.array(data.ranges)
        angles = np.linspace(data.angle_min, data.angle_min + len(ranges) * data.angle_increment, num=len(ranges), endpoint=False)
        
        # Camera poses in polar coordinates
        camera_polar = [(np.sqrt(pose.position.x**2 + pose.position.y**2),
                        np.arctan2(pose.position.y, pose.position.x) - np.pi/2) # - np.pi/2 to align with the Lidar frame
                        for pose in camera_poses.poses]

        # Define angular threshold for filtering (e.g., +/- 50 degrees)
        angle_threshold = np.radians(50)

        for radius, angle in camera_polar:
            # Filter LIDAR points based on angular proximity
            relevant_indices = np.where((angles >= (angle - angle_threshold)) & (angles <= (angle + angle_threshold)))[0]
            if relevant_indices.size == 0:
                continue

            # Convert relevant polar coordinates to Cartesian coordinates
            relevant_ranges = ranges[relevant_indices]
            relevant_angles = angles[relevant_indices]
            x_coords = relevant_ranges * np.cos(relevant_angles)
            y_coords = relevant_ranges * np.sin(relevant_angles)
            relevant_points = np.vstack((x_coords, y_coords)).T

            # Find the closest point within the filtered subset
            closest_point = relevant_points[np.argmin(np.linalg.norm(relevant_points - np.array([radius * np.cos(angle), radius * np.sin(angle)]), axis=1))]
            
            # Check if closest point is within the distance threshold
            if np.linalg.norm(closest_point - np.array([radius * np.cos(angle), radius * np.sin(angle )])) > self.fully_assisted_detection_distance_threshold:
                continue
            
            # Rotate closest point to Camera frame -90 degrees
            closest_point = np.array([-closest_point[1], closest_point[0]])

            position = Pose()
            position.position = Point(closest_point[0], closest_point[1], 0)
            positions.poses.append(position)

        return positions

    def standalone_euclidean_clustering(self, data):
        # Convert scan message to Cartesian coordinates
        points = polar_to_cartesian(np.array(data.ranges), data.angle_min, data.angle_increment)
        # Filter out invalid measurements
        points = points[np.isfinite(points).all(axis=1)]

        # Filter out 0 values
        points = points[np.all(points != 0, axis=1)]

        # Cluster the points
        clusters = euclidean_clustering(points, self.clustering_distance_threshold, 
                    self.clustering_min_points, self.clustering_max_points)

        positions = PoseArray()
        positions.header.frame_id = self.robot_name
        for cluster in clusters:
            center = np.mean(cluster, axis=0)
            position = Pose()
            position.position = Point(center[0], center[1], 0)
            positions.poses.append(position)

        return positions

    def standalone_dbscan_clustering(self, data):
        # Convert scan message to Cartesian coordinates
        points = polar_to_cartesian(np.array(data.ranges), data.angle_min, data.angle_increment)
        # Filter out invalid measurements
        points = points[np.isfinite(points).all(axis=1)]

        # Filter out 0 values
        points = points[np.all(points != 0, axis=1)]

        # Cluster the points
        clusters = dbscan_clustering(points, self.clustering_distance_threshold, self.clustering_min_points)

        positions = PoseArray()
        positions.header.frame_id = self.robot_name
        for cluster in clusters:
            center = np.mean(cluster, axis=0)
            position = Pose()
            position.position = Point(center[0], center[1], 0)
            positions.poses.append(position)

        return positions

    def process_detection(self, data, detection_function):
        """
        Process the detection using the provided detection function
        """

        with self.last_detections_lock:
            last_detections_copy = self.last_detections

            # self.last_detections = None

        # Initialize an array to store the positions
        positions = PoseArray()

        if last_detections_copy is None:
            rospy.logwarn("No detections received yet.")
            return positions

        positions.header.frame_id = last_detections_copy.header.frame_id

        for detection in last_detections_copy.detections:
            bbox = detection.bbox
            
            min_angle, max_angle = self.min_max_angles(bbox)
            
            # Extract the relevant part of the Lidar scan based on the angles within the bounding box
            zone_of_interest = [[data.ranges[i], data.angle_min + i * data.angle_increment]  for i in range(len(data.ranges)) if min_angle <= (data.angle_min + i * data.angle_increment) <= max_angle]
            
            # Calculate the depth, angle and convert it to cartesian using the provided detection function
            detected_point = detection_function(zone_of_interest)
            
            # # Calculate the middle angle of the bounding box
            # middle_angle = (min_angle + max_angle) / 2
            
            # # Convert depth and middle angle to x, y coordinates
            # x, y = self.depth_angle_to_two_d(depth, middle_angle)
            
            # Create a Pose object and set its position
            if detected_point is not None:
                position = Pose()
                position.position = Point(detected_point[0], detected_point[1], 0)
                
                positions.poses.append(position)
            
        return positions

    def closest_position(self, positions):
        """
        Return the position closest to the Lidar
        """

        if len(positions) > 0:
            distances = [math.sqrt(position[0]**2 + position[1]**2) for position in positions]
            min_position = positions[distances.index(min(distances))]
        else:
            min_position = None

        return min_position


    def partially_assisted_euclidean_clustering(self, data):
        # Filter out 0 values
        data = [pair for pair in data if pair[0] != 0]

        if len(data) == 0:
            closest_position = None
        else:
            # Convert the data to Cartesian coordinates
            points = [self.depth_angle_to_two_d(depth, angle) for depth, angle in data]

            rospy.loginfo(points)
            # Cluster the points
            clusters = euclidean_clustering(points, self.clustering_distance_threshold, 
                        self.clustering_min_points, self.clustering_max_points)

            positions = [np.mean(cluster, axis=0) for cluster in clusters]

            closest_position = self.closest_position(positions)

        return closest_position

    def partially_assisted_dbscan_clustering(self, data):
        # Filter out 0 values
        data = [pair for pair in data if pair[0] != 0]

        # Convert the data to Cartesian coordinates
        points = [self.depth_angle_to_two_d(depth, angle) for depth, angle in data]

        # Cluster the points
        clusters = dbscan_clustering(points, self.clustering_distance_threshold, self.clustering_min_points)

        positions = [np.mean(cluster, axis=0) for cluster in clusters]

        closest_position = self.closest_position(positions)

        return closest_position
        
    def minimum_distance_within_bbox_depth(self, data):
        """
        Remove all zero values from the data list (pairs of depth, angle) and return the minimum value with the angle
        """
        data = [pair for pair in data if pair[0] != 0]
        if len(data) == 0:
            return (0, 0)

        minimum_polar = min(data, key=lambda x: x[0])
        
        minimum_cartesian = self.depth_angle_to_two_d(minimum_polar[0], minimum_polar[1])

        return minimum_cartesian

    # def split_from_background_depth(self, data):
    #     """
    #     Split the data into two parts based on their proximity to the average value and find the median of the data closest to the minimum value
    #     """
    #     # Remove all zero values from the data list
    #     data = [value for value in data if value != 0]
        
    #     min_value, max_value = min(data, default=0), max(data, default=0)
    #     min_max_average = (max_value + min_value) / 2
        
    #     # Split the data into two parts based on their proximity to the average value
    #     data_closest_to_max = [value for value in data if value > min_max_average]
    #     data_closest_to_min = [value for value in data if value <= min_max_average]
        
    #     # Sort the data closest to the minimum value and find the median
    #     data_closest_to_min.sort()
    #     median = data_closest_to_min[len(data_closest_to_min) // 2]
        
    #     return median

    # def max_strictly_below_median_depth(self, data):
    #     """
    #     Find the median of the data and return the maximum value strictly below the median
    #     """
    #     # Remove all zero values from the data list
    #     data = [value for value in data if value != 0]
        
    #     # Sort the data and find the median
    #     if len(data) == 0:
    #         return 0
    #     data.sort()
    #     median = data[len(data) // 2] - data[len(data) // 2] * 0.05
        
    #     # Find the maximum value strictly below the median
    #     values_below_median = [value for value in data if value < median]
    #     if len(values_below_median) == 0:
    #         return 0
    #     max_value = max(values_below_median)
                
    #     return max_value


    def depth_angle_to_two_d(self, depth, angle):
        adjusted_angle = angle + np.pi / 2
        x = depth * math.cos(adjusted_angle)
        y = depth * math.sin(adjusted_angle)
        return x, y

    def publish_positions(self, positions):
        self.positions_pub.publish(positions)

    def oak_callback(self, data):
        # Store the last detections
        with self.last_detections_lock:
            self.last_detections = data

    def scan_callback(self, data):

        positions = self.detect_robots(data)
        positions.header.stamp = rospy.Time.now()
        # positions.header.frame_id = self.robot_name
        if positions is not None:
            self.publish_positions(positions)

            if self.visualize:
                self.visualize_detected_robot(positions)
        else:
            rospy.loginfo("Error: positions were not calculated")


    def camera_callback(self, data):
        # Store the last camera poses
        with self.last_camera_poses_lock:
            self.last_camera_poses = data

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        object_depth_finder = LidarDetectorNode()
        object_depth_finder.run()
    except rospy.ROSInterruptException:
        print("Process LidarDetectorNode interrupted")
