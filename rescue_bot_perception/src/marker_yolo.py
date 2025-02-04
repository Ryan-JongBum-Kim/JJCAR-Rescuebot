#!/usr/bin/env python

import sys
import argparse
import rospy

from math import isnan, isinf

import cv2
from cv_bridge import CvBridge

from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_msgs.msg import BoundingBox

from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker

from typing import List, Tuple

# global variables
detections : BoundingBoxes = None

cameraInfoMsg = None
depthMsg = None

marker_count : int = 0
last_marker_time : float = 0
prev_seq : int = 0

def detection_callback(msg : BoundingBoxes):
    global detections
    detections = msg

def camera_info_callback(msg : CameraInfo) -> None:
    global cameraInfoMsg
    cameraInfoMsg = msg

def depth_callback(msg : Image) -> None:
    global depthMsg
    depthMsg = msg

def detector_listener(detection_threshold : float = 0.7):

    global marker_count
    global last_marker_time
    global prev_seq

    # Initialise ros node
    rospy.init_node('dectector_listener_node', anonymous=True)

    # Subscribes to YOLO bounding boxes
    rospy.Subscriber(name='/darknet_ros/bounding_boxes', data_class=BoundingBoxes, callback=detection_callback)

    # Subscribes to the Camera image topics
    rospy.Subscriber(name='/camera/rgb/camera_info', data_class=CameraInfo, callback=camera_info_callback)
    rospy.Subscriber(name='/camera/depth/image_raw', data_class=Image, callback=depth_callback)

    # Publisher of rescue markers
    marker_pub = rospy.Publisher("/rescue_markers", Marker, queue_size = 1000)

    if CameraInfo is None:
        print('Warning! No camera info msgs')
        return
    
    if depthMsg is None:
        print('Warning! No depth image msgs')
        return

    if detections is None:
        return
    
    bridge = CvBridge()

    depthImg = bridge.imgmsg_to_cv2(depthMsg, desired_encoding='passthrough')

    detected_people = []

    if detections.header.seq == prev_seq:
        return

    for d in detections.bounding_boxes:
        detected : BoundingBox = d
        
        if detected.Class != "person":
            continue

        if detected.probability <= detection_threshold:
            continue


        detected_people.append(detected)
        print(f'Object: {detected.Class}, Probability: {detected.probability}')
        
        # calculate centroid
        xc = ((detected.xmax - detected.xmin)/2) + detected.xmin
        yc = ((detected.ymax - detected.ymin)/2) + detected.ymin

        obj_pos = get_object_position(centroid=(int(xc), int(yc)), cameraInfo=cameraInfoMsg, depthImg=depthImg)

        if obj_pos is None:
            continue

        cooldown = rospy.get_time() - last_marker_time
        if cooldown <= 3:
            print(f'marker placing cooldown active... {cooldown:.2f}s left')
            continue

        marker_count = add_marker(marker_count=marker_count, point=obj_pos, frame='base_footprint', publisher=marker_pub)
        last_marker_time = rospy.get_time()

    prev_seq = detections.header.seq

def get_object_position(centroid : Tuple[float, float], cameraInfo : CameraInfo, depthImg) -> Tuple[float, float, float]:
    fx = cameraInfo.K[0]
    fy = cameraInfo.K[4]

    cx = cameraInfo.K[2]
    cy = cameraInfo.K[5]
    
    ix = centroid[0]
    iy = centroid[1]

    depth = depthImg[iy, ix]

    print(f'DEPTH: {depth}')
    # Skip if the depth is a number and is greater then the max distance
    if isnan(depth) or isinf(depth):
        print('Not close enough to object to determine position, skipping...')
        return

    x = depth
    y = (-(ix - cx) * depth) / fx
    z = ((iy - cy) * depth) / fy

    return (x, y, z)

def add_marker(marker_count : int, point : Tuple[float, float, float], frame : str, publisher) -> int:
    marker = Marker()

    marker.header.frame_id = frame
    marker.header.stamp = rospy.Time.now()

    # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
    marker.type = 1

    marker.id = marker_count

    # Set the scale of the marker
    marker.scale.x = 0.4
    marker.scale.y = 0.4
    marker.scale.z = 0.8

    # Set the color
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    # Set the pose of the marker
    marker.pose.position.x = point[0] + 0.5
    marker.pose.position.y = point[1]
    marker.pose.position.z = marker.scale.z / 2
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    publisher.publish(marker)
    print('marker published')

    return marker_count + 1

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--detect_threshold', help="detection confidence threshold", type=float)
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    if args.detect_threshold:
        print(f"detection_threshold set to {args.detect_threshold * 100}%")
        detection_threshold = args.detect_threshold
    else:
        print(f"detection_threshold set to default of 70%")
        detection_threshold = 0.7

    print(f'Detection Probability set to: {detection_threshold}')
    while not rospy.is_shutdown():        
        detector_listener(detection_threshold)

        rospy.sleep(0.01)

if __name__ == '__main__':
    main()
