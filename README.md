## Mercator sensor fusion ROS package

To install this ROS package, setup a catkin workspace and clone it in your_catkin_workspace/src/

For catkin workspace setup, refer to: http://wiki.ros.org/catkin/Tutorials/create_a_workspace

## Quick Setup that should be sufficient on the robots at Iridia for the camera:

python3 -m pip install --trusted-host pypi.org --trusted-host pypi.python.org --trusted-host files.pythonhosted.org depthai

sudo apt install ros-noetic-vision-msgs

Set the udev rules : 

echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules

sudo udevadm control --reload-rules && sudo udevadm trigger

Unplug and plug camera if necessary.

export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1 (if cannot allocate memory in static TLS block appears)


## Main software launch:

roslaunch mercator_sensor_fusion_ros_pkg fusion.launch
roslaunch mercator_sensor_fusion_ros_pkg camera_all_detector.launch

## Experiment assessment:

roslaunch mercator_sensor_fusion_ros_pkg sync_rosbag.launch
roslaunch mercator_sensor_fusion_ros_pkg ground_truth_bagger.launch
roslaunch mercator_sensor_fusion_ros_pkg cost_between_poses.launch

## Evaluate experiments from multiple rosbags at same time:

roslaunch mercator_sensor_fusion_ros_pkg all_results_generator.launch


