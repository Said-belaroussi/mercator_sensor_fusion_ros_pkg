import os.path
import json
import sys
import cv2
import depthai as dai
import numpy as np
import time

import rospy
from std_msgs.msg import Header
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import Detection2D
from vision_msgs.msg import BoundingBox2D
from geometry_msgs.msg import Pose2D

import rospkg


def publisher_bbox(i):
    """
    Function to publish the bounding box information as a Detection2DArray message
    """
    msg = Detection2DArray()
    header = Header()
    header.stamp = rospy.Time.now()
    msg.header = header
    detection_2d_list = []

    detection_2d_msg = Detection2D()
    detection_2d_msg.header = header
    bounding_box_2d = BoundingBox2D()
    pose_2d = Pose2D()
    pose_2d.x = i["x"]
    pose_2d.y = i["y"]
    pose_2d.theta = i["confidence"]
    bounding_box_2d.center = pose_2d
    bounding_box_2d.size_x = i["width"]
    bounding_box_2d.size_y = i["height"]
    detection_2d_msg.bbox = bounding_box_2d
    detection_2d_list.append(detection_2d_msg)

    msg.detections = detection_2d_list
    pub_bbox.publish(msg)

def get_config_from_json(json_filename):
    """
    Function to read configuration parameters from a JSON file
    """
    f = open(json_filename)
    jsonData = json.load(f)
    f.close()
    confidenceThreshold = jsonData["nn_config"]["NN_specific_metadata"]["confidence_threshold"]
    numClasses = jsonData["nn_config"]["NN_specific_metadata"]["classes"]
    anchors = jsonData["nn_config"]["NN_specific_metadata"]["anchors"]
    anchorMasks = jsonData["nn_config"]["NN_specific_metadata"]["anchor_masks"]
    coordinateSize = jsonData["nn_config"]["NN_specific_metadata"]["coordinates"]
    iouThreshold = jsonData["nn_config"]["NN_specific_metadata"]["iou_threshold"]
    inputSize = jsonData["nn_config"]["input_size"]
    inputSizeX, inputSizeY = inputSize.split("x")
    inputSizeX = int(inputSizeX)
    inputSizeY = int(inputSizeY)
    labelMap = jsonData["mappings"]["labels"]

    return confidenceThreshold, numClasses, anchors, anchorMasks, \
        coordinateSize, iouThreshold, inputSizeX, inputSizeY, labelMap

def start_oak_camera(blob_filename, json_filename, compressed=True):

    (confidenceThreshold, numClasses, anchors, anchorMasks,
     coordinateSize, iouThreshold, inputSizeX, inputSizeY, labelMap) = get_config_from_json(json_filename)

    # Define camera pipeline and its components
    pipeline = dai.Pipeline()

    # Define needed components
    camRgb = pipeline.create(dai.node.ColorCamera)
    yolo_detection_network = pipeline.create(dai.node.YoloDetectionNetwork)

    xoutRgb = pipeline.create(dai.node.XLinkOut)
    xoutNN = pipeline.create(dai.node.XLinkOut)

    xoutRgb.setStreamName("rgb")
    xoutNN.setStreamName("detections")

    # RGB camera setup
    camRgb.setPreviewSize(inputSizeX, inputSizeY)
    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    camRgb.setInterleaved(False)
    camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

    # If NN take horizontally compressed images as input. Increases FOV
    if compressed:
        camRgb.setPreviewKeepAspectRatio(False)

    yolo_detection_network.setBlobPath(blob_filename)
    yolo_detection_network.setConfidenceThreshold(confidenceThreshold)
    yolo_detection_network.input.setBlocking(False)

    yolo_detection_network.setNumClasses(numClasses)
    yolo_detection_network.setCoordinateSize(coordinateSize)
    yolo_detection_network.setAnchors(anchors)
    yolo_detection_network.setAnchorMasks(anchorMasks)
    yolo_detection_network.setIouThreshold(iouThreshold)

    camRgb.preview.link(yolo_detection_network.input)
    yolo_detection_network.passthrough.link(xoutRgb.input)
    yolo_detection_network.out.link(xoutNN.input)

    # Connect to device and start processing
    with dai.Device(pipeline) as device:
        previewQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        detectionNNQueue = device.getOutputQueue(name="detections", maxSize=4, blocking=False)

        while True:
            inPreview = previewQueue.get()
            inDet = detectionNNQueue.get()

            frame = inPreview.getCvFrame()
            detections = inDet.detections

            height, width = frame.shape[0], frame.shape[1]

            for detection in detections:
                bbox_data = dict()

                x1 = int(detection.xmin * width)
                x2 = int(detection.xmax * width)
                y1 = int(detection.ymin * height)
                y2 = int(detection.ymax * height)

                # Bounding box info for detection ros msg
                bbox_data["x"] = (x1 + x2) / 2
                bbox_data["y"] = (y1 + y2) / 2
                bbox_data["width"] = x2 - x1
                bbox_data["height"] = y2 - y1
                bbox_data["confidence"] = detection.confidence

                publisher_bbox(bbox_data)  # Publish the bounding box data


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("usage: /path/oak_detector.py /path/blob_file_name /path/json_file_name")
        exit(1)
    else:
        blob_filename = sys.argv[1]
        json_filename = sys.argv[2]

    # Set up ROS publisher for bounding boxes
    rospy.init_node("oak_detector", anonymous=True)
    pub_bbox = rospy.Publisher("oak", Detection2DArray, queue_size=1)

    start_oak_camera(blob_filename, json_filename)
