#!/usr/bin/env python

import argparse
import rospy

from math import isnan, isinf
import imutils

import cv2
from cv_bridge import CvBridge
import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import Marker

from typing import List, Tuple
# from numpy.typing import NDArray

# global variables
cameraInfoMsg = None
colMsg = None
depthMsg = None
poseMsg = None

# markers : List[Tuple[float, float, float]] = []
last_marker_time = 0
marker_count = 0

def camera_info_callback(msg : CameraInfo) -> None:
    global cameraInfoMsg
    cameraInfoMsg = msg

def image_callback(msg : Image) -> None:
    global colMsg
    colMsg = msg

def depth_callback(msg : Image) -> None:
    global depthMsg
    depthMsg = msg

def pose_callback(msg : TransformStamped) -> None:
    global poseMsg
    poseMsg = msg

def peeper(confidence_threshold : float = 1.5, max_detect_distance : float = 2):

    global CameraInfo
    global colImg
    global depthImg
    
    global last_marker_time
    global marker_count

    rospy.init_node('peeper_node', anonymous=True)
    
    rospy.Subscriber(name='/camera/rgb/camera_info', data_class=CameraInfo, callback=camera_info_callback)
    rospy.Subscriber(name='/camera/rgb/image_raw', data_class=Image, callback=image_callback)
    rospy.Subscriber(name='/camera/depth/image_raw', data_class=Image, callback=depth_callback)

    marker_pub = rospy.Publisher("/rescue_markers", Marker, queue_size = 2)

    if CameraInfo is None:
        print('Warning! No camera info msgs')
        return

    if colMsg is None:
        print('Warning! No colour image msgs')
        return
    
    if depthMsg is None:
        print('Warning! No depth image msgs')
        return

    bridge = CvBridge()
    colImg = bridge.imgmsg_to_cv2(colMsg, desired_encoding='bgr8')
    depthImg = bridge.imgmsg_to_cv2(depthMsg, desired_encoding='passthrough')
    
    centroids = detect(colImg, confidence_threshold)

    objects = get_object_position(centroids=centroids, 
                                  cameraInfo=cameraInfoMsg, 
                                  depthImg=depthImg, 
                                  max_detect_distance=max_detect_distance)

    if objects is None:
        print('No objects detected!')
        return

    for p in objects:
        cooldown = rospy.get_time() - last_marker_time 
        if cooldown <= 5:
            print(f'place marker cooldown active... {cooldown} left')
            break

        last_marker_time = rospy.get_time()
        marker_count = add_marker(marker_count=marker_count, point=p, frame='base_footprint', publisher=marker_pub)

def detect(imageArray, confidence_threshold : float) -> List[Tuple[float, float, float]]:
    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    # read the colMsg
    image = imutils.resize(imageArray, width=min(320, imageArray.shape[1]))

    (rects, weights) = hog.detectMultiScale(image, winStride=(4, 4), padding=(8, 8), scale=1.05)

    centroids : List[Tuple[float, float, float]] = []

    # draw bounding box
    for (x, y, w, h), weight in zip(rects, weights):

        dx = w / 2
        dy = h / 2

        if weight < confidence_threshold:
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
            centroid_point = (int(x + dx), int(y + dy))
            cv2.circle(image, centroid_point, 2, (0, 0, 255), -1)
            cv2.putText(image, f'c_r: {weight:.2f}', (x + 5, y + 10), cv2.FONT_HERSHEY_SIMPLEX , 0.3, (0, 0, 255), 1, cv2.LINE_AA)
            continue

        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        centroid_point = (int(x + dx), int(y + dy))
        cv2.circle(image, centroid_point, 2, (0, 255, 0), -1)
        centroids.append((centroid_point[0], centroid_point[1], weight))
        cv2.putText(image, f'c_r: {weight:.2f}', (x + 5, y + 10), cv2.FONT_HERSHEY_SIMPLEX , 0.3, (0, 255, 0), 1, cv2.LINE_AA)

    # show the output images
    cv2.imshow("Detected", image)
    cv2.waitKey(1)

    return centroids

def get_object_position(centroids : List[Tuple[float, float]], cameraInfo : CameraInfo, depthImg : Image, max_detect_distance : float) -> List[Tuple[float, float, float]]:
    fx = cameraInfo.K[0]
    fy = cameraInfo.K[4]

    cx = cameraInfo.K[2]
    cy = cameraInfo.K[5]
    
    rel_points = []

    for c in centroids:
        ix = c[0] * 2
        iy = c[1] * 2

        depth = depthImg[iy, ix]

        if isnan(depth) or isinf(depth) or depth > max_detect_distance:
            print('Not close enough to object to determine position, skipping...')
            continue

        x = depth
        y = (-(ix - cx) * depth) / fx
        z = ((iy - cy) * depth) / fy

        rel_points.append((x, y, z))

    return rel_points

def transform_point(point : Tuple[float, float, float]):
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)

    transform = buffer.lookup_transform('map', 'base_footprint', rospy.Time(), rospy.Duration(1))
    
    point_stamped = PointStamped()
    point_stamped.header.frame_id = 'base_footprint'
    point_stamped.point.x = point[0]
    point_stamped.point.y = point[1]
    point_stamped.point.z = point[2]

    try:
        # Transform the point to the target frame
        transformed_point : PointStamped = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
        return (transformed_point.point.x, transformed_point.point.y, transformed_point.point.z)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print("Transformation error")

def get_pose() -> Tuple[float, float, float]:
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)

    transform : TransformStamped = buffer.lookup_transform('map', 'base_footprint', rospy.Time(), rospy.Duration(1.0))

    try:
        t = transform.transform.translation
        return (t.x, t.y, t.z)

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print("Transformation error")

def add_marker(marker_count : int, point : Tuple[float, float, float], frame : str, publisher) -> Tuple[float, float, float]:
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

    # # # Set the pose of the marker
    # global_pos = transform_point((point[0] + 0.5, point[1], marker.scale.z / 2))
    # marker.pose.position.x = global_pos[0]
    # marker.pose.position.y = global_pos[1]
    # marker.pose.position.z = global_pos[2]

    # Set the pose of the marker
    marker.pose.position.x = point[0] + 0.5
    marker.pose.position.y = point[1]
    marker.pose.position.z = marker.scale.z / 2
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    # rospy.loginfo('Added marker')
    publisher.publish(marker)

    # return (marker.pose.position.x, marker.pose.position.y, marker.pose.position.z)
    return marker_count + 1

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--detect_threshold', help="detection confidence threshold")
    parser.add_argument('-md', '--max_detect', help="maximum detection distance")
    args = parser.parse_args()

    if args.detect_threshold:
        print(f"detection_threshold set to {args.detect_threshold}")
        detection_threshold = args.detect_threshold
    else:
        print(f"detection_threshold set to default of 1.5")
        detection_threshold = 1.5

    if args.max_detect:
        print(f"max_detect_distance set to {args.max_detect}")
        max_detect_distance = args.detect_threshold
    else:
        print(f"max_detect_distance set to default of 2")
        max_detect_distance = 2

    # print(f'Detection Threshold set to: {d_threshold}')
    while not rospy.is_shutdown():        
        peeper(detection_threshold, max_detect_distance)

        rospy.sleep(0.01)

if __name__ == '__main__':
    main()
