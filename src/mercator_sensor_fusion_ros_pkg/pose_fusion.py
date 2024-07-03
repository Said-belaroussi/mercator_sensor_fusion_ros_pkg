#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseArray
from scipy.optimize import linear_sum_assignment
from filterpy.kalman import KalmanFilter
from sensor_msgs.msg import LaserScan
import numpy as np


class kalman_filter:
    def __init__(self, id, sensors_number=2, inital_velocity=0.0, initial_position=(0.0, 0.0), frequency=25.0,
        process_noise_factor=0.0001, initial_sensors_variances=None, robot_average_radius=0.15):

        if initial_sensors_variances is None:
            initial_sensors_variances = [0.0256, 0.0196] # 4% error with 4 meters depths, 3.5% error with 4 meter depths

        if len(initial_sensors_variances) != sensors_number:
            raise ValueError("The number of sensors variances should be equal to the number of sensors")

        
        self.id = id
        self.number_of_times_no_update = 0
        self.number_of_times_diverged_only_lidar = 0

        self.number_of_times_no_update_threshold = 10
        self.number_of_times_diverged_only_lidar_threshold = 10

        delta_t = 1.0/frequency

        self.kf = KalmanFilter(dim_x=4, dim_z=sensors_number*2)
        self.kf.x = np.array([[initial_position[0]],
                              [initial_position[1]],
                              [inital_velocity],
                              [inital_velocity]])
        self.kf.F = np.array([[1, 0, delta_t, 0],
                              [0, 1, 0, delta_t],
                              [0, 0, 1, 0],
                              [0, 0, 0, 1]])

        self.kf.H = np.zeros((sensors_number*2, 4))
        for i in range(sensors_number):
            self.kf.H[i*2, 0] = 1
            self.kf.H[i*2+1, 1] = 1

        # self.kf.H = np.array([[1, 0, 0, 0],
        #                       [0, 1, 0, 0],
        #                       [1, 0, 0, 0],
        #                       [0, 1, 0, 0]])

        process_noise_variance_position = process_noise_factor*0.5*delta_t**2
        process_noise_variance_velocity = process_noise_factor*delta_t
        self.kf.Q = np.array([[process_noise_variance_position, 0, 0, 0],
                              [0, process_noise_variance_position, 0, 0],
                              [0, 0, process_noise_variance_velocity, 0],
                              [0, 0, 0, process_noise_variance_velocity]])

        self.kf.R = np.zeros((sensors_number*2, sensors_number*2))

        for i in range(sensors_number):
            self.kf.R[i*2, i*2] = initial_sensors_variances[i]
            self.kf.R[i*2+1, i*2+1] = initial_sensors_variances[i]

        # self.kf.R = np.array([[initial_sensors_variances[0], 0, 0, 0],
        #                       [0, initial_sensors_variances[0], 0, 0], 
        #                       [0, 0, initial_sensors_variances[1], 0],
        #                       [0, 0, 0, initial_sensors_variances[1]]]) # The measurement covariance noise matrice is update dynamically
        state_noise_velocity = robot_average_radius/delta_t
        self.kf.P = [[robot_average_radius, 0, 0, 0],
                        [0, robot_average_radius, 0, 0],
                        [0, 0, state_noise_velocity, 0],
                        [0, 0, 0, state_noise_velocity]]

    def predict(self):
        self.kf.predict()

    def update(self, measurement):
        self.kf.update(measurement)

    def update_measurement_covariance(self, measurement_covariance):
        self.kf.R = measurement_covariance

    def get_state(self):
        print("self.kf.x\n", self.kf.x)
        print("self.kf.Q\n", self.kf.Q)
        print("self.kf.R\n", self.kf.R)
        print("self.kf.P\n", self.kf.P)
        return [self.kf.x[0][0], self.kf.x[1][0]]

    def get_id(self):
        return self.id
    
    def increment_no_update(self):
        self.number_of_times_no_update += 1

    def get_no_update_threshold_reached(self):
        if self.number_of_times_no_update > self.number_of_times_no_update_threshold:
            return True
        return False
    
    def reset_no_update(self):
        self.number_of_times_no_update = 0

    def increment_diverged_only_lidar(self):
        self.number_of_times_diverged_only_lidar += 1

    def get_diverged_only_lidar(self):
        if self.number_of_times_diverged_only_lidar > self.number_of_times_diverged_only_lidar_threshold:
            return True
        return False

    def reset_diverged_only_lidar(self):
        self.number_of_times_diverged_only_lidar = 0


class PoseFusionNode:
    def __init__(self):
        rospy.init_node('pose_fusion_node', anonymous=True)

        self.cam_poses = None
        self.lidar_poses = None

        self.cam_sub = rospy.Subscriber('cam_poses', PoseArray, self.cam_callback)
        self.lidar_sub = rospy.Subscriber('lidar_poses', PoseArray, self.lidar_callback)
        self.fused_pub = rospy.Publisher('fused_poses', PoseArray, queue_size=10)

        self.sensors_topics = rospy.get_param('~sensors_topics', ['cam_poses', 'lidar_poses'])
        self.keep_tracking_with_only_lidar = rospy.get_param('~keep_tracking_with_only_lidar', False)
        self.sensors_number = rospy.get_param('~sensors_number', 2)
        self.process_noise_factor = rospy.get_param('~process_noise_factor', 0.0001)
        self.frequency = rospy.get_param('~frequency', 25.0)
        self.initial_sensors_variances = rospy.get_param('~initial_sensors_variances', [0.0256, 0.0196])
        self.robot_average_radius = rospy.get_param('~robot_average_radius', 0.15) # in meters
        self.divergence_only_lidar_threshold = robot_average_radius*2
        self.initial_velocity = rospy.get_param('~initial_velocity', 0.0)
        self.initial_position = rospy.get_param('~initial_position', 0.0)

        if sensors_number != len(sensors_topics):
            raise ValueError("The number of sensors topics should be equal to the number of sensors")

        self.lidar_scan_sub = rospy.Subscriber('scan', LaserScan, self.lidar_scan_callback)
        self.lidar_scan = None
        self.non_matched_kf_ids = []
        self.lidar_scan_only_poses = PoseArray()

        self.last_id = 0
        self.kalman_filters = dict()

        self.run()

    def generate_new_id(self):
        self.last_id += 1
        return self.last_id

    def initialize_kalman_filter(self, id, sensors_number=2, inital_velocity=0.0, initial_position=(0.0, 0.0), frequency=25.0,
        process_noise_factor=0.0001, initial_sensors_variances=None, robot_average_radius=0.15):
        return kalman_filter(id, sensors_number=sensors_number, inital_velocity=inital_velocity, initial_position=initial_position,
                            frequency=frequency, process_noise_factor=process_noise_factor, initial_sensors_variances=initial_sensors_variances,
                            robot_average_radius=robot_average_radius)

    def cam_absolute_error(self, x, y):
        """
        Returns the relative error of the position measured from the camera based on its distance
        from the object.
        https://docs.luxonis.com/projects/hardware/en/latest/pages/BW1098OAK/#stereo-depth-perception
        """
        distance = np.linalg.norm(x - y)
        relative_error = None
        if distance < 4:
            relative_error = 0.02
        elif distance < 7:
            relative_error = 0.04
        else:
            relative_error = 0.06
        
        return relative_error*distance

    def lidar_absolute_error(self, x, y):
        """
        Returns the relative error of the position measured from the lidar based on its distance
        from the object.
        https://www.ydlidar.com/dowfile.html?cid=5&type=1
        """
        distance = np.linalg.norm(x - y)

        absolute_error = None

        if distance <= 1:
            absolute_error = 0.02
        else:
            absolute_error = 0.035*distance
        
        return absolute_error

    def compute_measurement_covariance(self, measurements):
        """
        Computes the measurement covariance based on the positions given by the camera and lidar measurements.
        """
        cam_absolute_error = self.cam_absolute_error(measurements[0], measurements[1])
        lidar_absolute_error = self.lidar_absolute_error(measurements[2], measurements[3])

        cam_variance = cam_absolute_error**2
        lidar_variance = lidar_absolute_error**2

        covariance_noise_matrix = np.array([[cam_variance, 0, 0, 0],
                                            [0, cam_variance, 0, 0],
                                            [0, 0, lidar_variance, 0],
                                            [0, 0, 0, lidar_variance]])

        return covariance_noise_matrix

    def compute_measurement_covariance_only_cam(self, measurements):
        """
        Computes the measurement covariance based on the positions given by only the camera measurements.
        """
        cam_absolute_error = self.cam_absolute_error(measurements[0], measurements[1])

        cam_variance = cam_absolute_error**2

        covariance_noise_matrix = np.array([[cam_variance, 0, 0, 0],
                                            [0, cam_variance, 0, 0],
                                            [0, 0, cam_variance, 0],
                                            [0, 0, 0, cam_variance]])

        return covariance_noise_matrix

    def compute_measurement_covariance_only_lidar(self, measurements):
        """
        Computes the measurement covariance based on the positions given by only the lidar measurements.
        """
        lidar_absolute_error = self.lidar_absolute_error(measurements[0], measurements[1])

        lidar_variance = lidar_absolute_error**2

        covariance_noise_matrix = np.array([[lidar_variance, 0, 0, 0],
                                            [0, lidar_variance, 0, 0],
                                            [0, 0, lidar_variance, 0],
                                            [0, 0, 0, lidar_variance]])

        return covariance_noise_matrix

    def compute_measurement_covariance_single_sensor(self, measurements):
        """
        Computes the measurement covariance based on the positions given by only one sensor measurements.
        """
        if sensors_topics[0] == 'cam_poses':
            cam_absolute_error = self.cam_absolute_error(measurements[0], measurements[1])
            cam_variance = cam_absolute_error**2
            covariance_noise_matrix = np.array([[cam_variance, 0],
                                                [0, cam_variance])
        elif sensors_topics[0] == 'lidar_poses':
            lidar_absolute_error = self.lidar_absolute_error(measurements[0], measurements[1])
            lidar_variance = lidar_absolute_error**2
            covariance_noise_matrix = np.array([[lidar_variance, 0],
                                                [0, lidar_variance])
        else:
            raise ValueError("The sensor topic is not recognized")

        return covariance_noise_matrix
            
    def cam_callback(self, data):
        self.cam_poses = data

        if sensors_number == 1 and sensors_topics[0] == 'cam_poses':
            self.single_sensor_tracking()
        elif sensors_number == 2 and sensors_topics[0] == 'cam_poses':
            self.match_and_fuse()

    def lidar_callback(self, data):
        self.lidar_poses = data
        
        if sensors_number == 1 and sensors_topics[0] == 'lidar_poses':
            self.single_sensor_tracking()
        # self.match_and_fuse()

    def lidar_scan_callback(self, data):
        self.lidar_scan = data

    def search_for_closest_lidar_scan(self, pose):
        if self.lidar_scan is None:
            return None
        min_distance = float('inf')
        min_idx = -1
        min_lidar_pose = None
        for i, angle in enumerate(self.lidar_scan.angle_min + np.arange(len(self.lidar_scan.ranges)) * self.lidar_scan.angle_increment):
            lidar_pose = np.array([self.lidar_scan.ranges[i] * np.cos(angle), self.lidar_scan.ranges[i] * np.sin(angle)])
            distance = np.linalg.norm(np.array([pose.position.x, pose.position.y]) - lidar_pose)
            if distance < min_distance:
                min_distance = distance
                min_idx = i
                min_lidar_pose = lidar_pose
        return min_lidar_pose, min_distance

    def build_cost_matrix(self):
        cam_poses_xy = np.array([[pose.position.x, pose.position.y] for pose in self.cam_poses.poses])
        lidar_poses_xy = np.array([[pose.position.x, pose.position.y] for pose in self.lidar_poses.poses])
        print(cam_poses_xy)
        print(lidar_poses_xy)
        cost_matrix = np.linalg.norm(cam_poses_xy[:, None] - lidar_poses_xy, axis=2)
        print(cost_matrix)
        return cost_matrix

    def match_kf_poses(self, poses):
        poses_xy = np.array([[pose.position.x, pose.position.y] for pose in poses])
        kf_keys_list = list(self.kalman_filters.keys())
        kf_poses_xy = np.array([self.kalman_filters[key].get_state() for key in kf_keys_list])

        print("poses_xy", poses_xy)
        print("kf_poses_xy", kf_poses_xy)
        cost_matrix = np.linalg.norm(poses_xy[:, None] - kf_poses_xy, axis=2)
        cam_indices, kf_indices = linear_sum_assignment(cost_matrix)

        corresponding_kf_ids = [0 for i in range(len(poses_xy))]
        print("len(poses_xy)", len(poses_xy))
        print("cam_indices", cam_indices)
        print("kf_indices", kf_indices)
        print("kf_keys_list", kf_keys_list)
        for cam_idx, kf_idx in zip(cam_indices, kf_indices):
            corresponding_kf_ids[cam_idx] = kf_keys_list[kf_idx]
        print("corresponding_kf_ids ", corresponding_kf_ids )

        if (self.keep_tracking_with_only_lidar):
            self.non_matched_kf_ids = [key for key in self.kalman_filters.keys() if key not in corresponding_kf_ids]
            for key in non_matched_kf_ids:
        for i in range(len(poses_xy)):
            if i not in cam_indices:
                new_id = self.generate_new_id()
                self.kalman_filters[new_id] = self.initialize_kalman_filter(new_id, sensors_number=self.sensors_number,
                                                inital_velocity=self.initial_velocity, 
                                                initial_position=(poses[i].position.x, poses[i].position.y),
                                                frequency=self.frequency, process_noise_factor=self.process_noise_factor, 
                                                initial_sensors_variances=self.initial_sensors_variances, 
                                                robot_average_radius=self.robot_average_radius)
                corresponding_kf_ids[i] = new_id
        
        kfs_to_delete = []
        for key in self.kalman_filters.keys():
            if key not in corresponding_kf_ids:
                self.kalman_filters[key].increment_no_update()
                if self.kalman_filters[key].get_no_update_threshold_reached():
                    kfs_to_delete.append(key)
            else:
                self.kalman_filters[key].reset_no_update()
        
        for key in kfs_to_delete:
            del self.kalman_filters[key]

        return corresponding_kf_ids



    def match_and_fuse(self):
        if self.cam_poses is None or self.lidar_poses is None:
            return
        # Perform matching between poses using the Hungarian algorithm

        cost_matrix = self.build_cost_matrix()
        cam_indices, lidar_indices = linear_sum_assignment(cost_matrix)
        print("cam_indices: ", cam_indices)
        print("lidar_indices: ", lidar_indices)
        
        # Initialize Kalman filters if not already initialized
        if not self.kalman_filters:
            for i in range(len(self.cam_poses.poses)):
                new_id = self.generate_new_id()
                print("new_id", new_id)
                self.kalman_filters[new_id] = self.initialize_kalman_filter(new_id, sensors_number=self.sensors_number,
                                                inital_velocity=self.initial_velocity, 
                                                initial_position=(cam_poses.poses[i].position.x, cam_poses.poses[i].position.y),
                                                frequency=self.frequency, process_noise_factor=self.process_noise_factor, 
                                                initial_sensors_variances=self.initial_sensors_variances, 
                                                robot_average_radius=self.robot_average_radius) 

        # Match kf poses with cam poses
        corresponding_kf_ids = self.match_kf_poses(self.cam_poses.poses)

        # Publish fused poses
        fused_poses = PoseArray()
        fused_poses.header.stamp = rospy.Time.now()
        fused_poses.header.frame_id = 'fused_frame'

        for cam_idx, lidar_idx in zip(cam_indices, lidar_indices):
            cam_pose = self.cam_poses.poses[cam_idx]
            lidar_pose = self.lidar_poses.poses[lidar_idx]

            # Update Kalman filter
            # cam_measurement = np.array([cam_pose.position.x, cam_pose.position.y])
            # lidar_measurement = np.array([lidar_pose.position.x, lidar_pose.position.y])
            measurements = np.array([cam_pose.position.x, cam_pose.position.y, lidar_pose.position.x, lidar_pose.position.y])

            measurement_covariance = self.compute_measurement_covariance(measurements)
            self.kalman_filters[corresponding_kf_ids[cam_idx]].update_measurement_covariance(measurement_covariance)

            self.kalman_filters[corresponding_kf_ids[cam_idx]].predict()
            self.kalman_filters[corresponding_kf_ids[cam_idx]].update(measurements)

            # Get fused pose
            fused_pose = self.kalman_filters[corresponding_kf_ids[cam_idx]].get_state()

            # Create Pose message
            pose_msg = cam_pose
            pose_msg.position.x = fused_pose[0]
            pose_msg.position.y = fused_pose[1]
            fused_poses.poses.append(pose_msg)

        if(self.keep_tracking_with_only_lidar):
            kfs_to_delete = []
            for key in self.non_matched_kf_ids:
                lidar_pose, distance = self.search_for_closest_lidar_scan(self.kalman_filters[key].get_state())

                if lidar_pose is None:
                    continue

                if distance > self.divergence_only_lidar_threshold:
                    self.kalman_filters[key].increment_diverged_only_lidar()
                    if self.kalman_filters[key].get_diverged_only_lidar():
                        kfs_to_delete.append(key)
                else:
                    self.kalman_filters[key].reset_diverged_only_lidar()

                measurements = np.array([lidar_pose[0], lidar_pose[1], lidar_pose[0], lidar_pose[1]])
                measurement_covariance = self.compute_measurement_covariance_only_lidar(measurements)
                self.kalman_filters[key].update_measurement_covariance(measurement_covariance)

                self.kalman_filters[key].predict()
                self.kalman_filters[key].update(measurements)

                fused_pose = self.kalman_filters[key].get_state()
                pose_msg = self.cam_poses.poses[0]
                pose_msg.position.x = fused_pose[0]
                pose_msg.position.y = fused_pose[1]
                fused_poses.poses.append(pose_msg)

            for key in kfs_to_delete:
                del self.kalman_filters[key]

        self.fused_pub.publish(fused_poses)

    def single_sensor_tracking(self):
        if len(sensors_topics) != 1:
            raise ValueError("The number of sensors topics should be equal to 1"
            "when using the single sensor tracking mode")
        
        poses = None
        if sensors_topics[0] == 'cam_poses':
            poses = self.cam_poses
        elif sensors_topics[0] == 'lidar_poses':
            poses = self.lidar_poses
        else:
            raise ValueError("The sensor topic is not recognized")

        if poses is None:
            return

        if not self.kalman_filters:
            new_id = self.generate_new_id()
            self.kalman_filters[new_id] = self.initialize_kalman_filter(new_id, sensors_number=self.sensors_number,
                                                inital_velocity=self.initial_velocity, 
                                                initial_position=(poses.poses[0].position.x, poses.poses[0].position.y),
                                                frequency=self.frequency, process_noise_factor=self.process_noise_factor, 
                                                initial_sensors_variances=self.initial_sensors_variances, 
                                                robot_average_radius=self.robot_average_radius)

        # Match kf poses with poses
        corresponding_kf_ids = self.match_kf_poses(poses.poses)

        # Publish fused poses
        fused_poses = PoseArray()
        fused_poses.header.stamp = rospy.Time.now()
        fused_poses.header.frame_id = 'fused_frame'

        for i in range(len(poses.poses)):
            pose = poses.poses[i]

            # Update Kalman filter
            measurements = np.array([pose.position.x, pose.position.y, pose.position.x, pose.position.y])

            measurement_covariance = self.compute_measurement_covariance_single_sensor(measurements)

            self.kalman_filters[corresponding_kf_ids[i]].update_measurement_covariance(measurement_covariance)

            self.kalman_filters[corresponding_kf_ids[i]].predict()
            self.kalman_filters[corresponding_kf_ids[i]].update(measurements)

            # Get fused pose
            fused_pose = self.kalman_filters[corresponding_kf_ids[i]].get_state()

            # Create Pose message
            pose_msg = pose
            pose_msg.position.x = fused_pose[0]
            pose_msg.position.y = fused_pose[1]
            fused_poses.poses.append(pose_msg)

        self.fused_pub.publish(fused_poses)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = PoseFusionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
