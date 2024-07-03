#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
import depthai as dai
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def publish_depth(depth_data):
    """
    Function to publish the depth image
    """
    header = rospy.Header()
    header.stamp = rospy.Time.now()

    # Convert depth data to uint16 format (mm)
    depth_data_uint16 = (depth_data * 1000).astype(np.uint16)

    # Create Image message
    depth_image_msg = Image()
    depth_image_msg.header = header
    depth_image_msg.height = depth_data.shape[0]
    depth_image_msg.width = depth_data.shape[1]
    depth_image_msg.encoding = '16UC1'  # 16-bit unsigned integer, 1 channel
    depth_image_msg.step = depth_image_msg.width * 2  # 16-bit depth data
    depth_image_msg.data = depth_data_uint16.tobytes()

    pub_depth.publish(depth_image_msg)

def start_oak_camera():
    # Define camera pipeline and its components
    pipeline = dai.Pipeline()

    # Define needed components
    monoLeft = pipeline.create(dai.node.MonoCamera)
    monoRight = pipeline.create(dai.node.MonoCamera)
    stereo = pipeline.create(dai.node.StereoDepth)
    xoutDepth = pipeline.create(dai.node.XLinkOut)

    xoutDepth.setStreamName("depth")

    # Setting resolution for depth
    monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

    stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
    stereo.setOutputSize(640, 400)

    monoLeft.out.link(stereo.left)
    monoRight.out.link(stereo.right)
    stereo.depth.link(xoutDepth.input)

    # Connect to device and start processing
    with dai.Device(pipeline) as device:
        depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)

        while True:
            depth_data = depthQueue.get().getFrame()
            publish_depth(depth_data)

if __name__ == "__main__":
    rospy.init_node("oak_depth_publisher", anonymous=True)
    pub_depth = rospy.Publisher("oak_depth", Image, queue_size=1)
    start_oak_camera()
