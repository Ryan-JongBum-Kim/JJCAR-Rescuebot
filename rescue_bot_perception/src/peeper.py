#!/usr/bin/env python

import sys
import argparse
import rospy

from math import isnan, isinf
import imutils

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import Marker

from typing import List, Tuple

Point = Tuple[float, float, float]

class Peeper:

    def __init__(self,
                 detect_threshold : float,
                 max_detect_distance : float,
                 image_scale : float) -> None:
        """Peeper Class Constructor, initialises all variables and ROS node
        
        ### Parameters
            - detect_threshold - threshold for object detection
            - max_detect_distance - maximum object detection distance
        """
        self.detect_threshold       : float = detect_threshold
        self.max_detect_distance    : float = max_detect_distance
    
        self.image_scale            : float = image_scale

        self.camera_info            : CameraInfo = None
        self.col_msg                : Image = None
        self.depth_msg              : Image = None

        self.last_marker_time       : float = 0.0
        self.marker_count           : int = 0

        rospy.init_node('peeper', anonymous=True)

        # Publisher of rescue markers for detected objects
        self.marker_pub = rospy.Publisher('/rescue_markers', Marker, queue_size=1000)

        # Subscribes to the camera_info ROS topic
        rospy.Subscriber(name='/camera/rgb/camera_info', data_class=CameraInfo, callback=self.camera_info_callback)

        # Subscribes to the rgb image_raw ROS topic
        rospy.Subscriber(name='/camera/rgb/image_raw', data_class=Image, callback=self.col_image_callback)

        # Subscribes to the depth image_raw ROS topic
        rospy.Subscriber(name='/camera/depth/image_raw', data_class=Image, callback=self.depth_callback)


    def camera_info_callback(self, msg) -> None:
        """Camera Info callback function"""
        self.camera_info = msg
    
    def col_image_callback(self, msg) -> None:
        """Colour Image callback function"""
        self.col_msg = msg
    
    def depth_callback(self, msg) -> None:
        """Depth Image callback function"""
        self.depth_msg = msg

    def peeping(self):
        if self.camera_info is None:
            print('Warning! No camera info msgs')
            return

        if self.col_msg is None:
            print('Warning! No colour image msgs')
            return
        
        if self.depth_msg is None:
            print('Warning! No depth image msgs')
            return
        
        # Initialise cv_bridge object
        bridge = CvBridge()

        # Convert camera images to an numpy NDArray for openCV
        colour_image = bridge.imgmsg_to_cv2(self.col_msg, desired_encoding='bgr8')
        depth_image = bridge.imgmsg_to_cv2(self.depth_msg, desired_encoding='passthrough')

        # Look and detect for people
        centroids = self.detect(colour_image)

        # Get the people's position relative to the robot
        people = self.get_relative_positions(centroids=centroids, depth_image=depth_image)

        # Check if there are people
        if people is None:
            print('No people detected!')
            return

        # Loop through all the people and place a marker for each
        for person_position in people:
            # Set marker placing cooldown
            cooldown = rospy.get_time() - self.last_marker_time

            # Don't place a marker if 5s have not passed since the last
            if cooldown <= 1:
                print(f'marker placing cooldown active... {cooldown:.2f}s left')
                break

            self.last_marker_time = rospy.get_time()

            # Place a marker at the object position
            self.add_marker(point=person_position, frame='base_footprint')

    def detect(self, image_array) -> List[Point]:
        """Detects people from an colour image array
        
        ### Parameters
            - image_array - NDArray of the colour image
            - threshold - object detection confidence threshold

        ### Returns
        A list of points
        """

        # Initialise openCV HOG machine learning
        hog = cv2.HOGDescriptor()
        hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        # Resize the image to improve performance at the cost of information
        image = imutils.resize(image_array, width=min(int(640 * self.image_scale), image_array.shape[1]))

        # Detect people
        (rects, weights) = hog.detectMultiScale(image, winStride=(4, 4), padding=(8, 8), scale=1.05)

        centroids : List[Point] = []

        # Draw bounding box of detected persons
        for (x, y, w, h), weight in zip(rects, weights):

            dx = w / 2
            dy = h / 2

            print(f'weight: {weight}')

            # Check if the detected object is above the confidence threshold
            if weight < self.detect_threshold:
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
                centroid_point = (int(x + dx), int(y + dy))
                cv2.circle(image, centroid_point, 2, (0, 0, 255), -1)
                cv2.putText(image, f'c_r: {weight}', (x + 5, y + 10), cv2.FONT_HERSHEY_SIMPLEX , 0.3, (0, 0, 255), 1, cv2.LINE_AA)
                continue

            # Draw a stats box on top of the object
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            centroid_point = (int(x + dx), int(y + dy))
            cv2.circle(image, centroid_point, 2, (0, 255, 0), -1)
            cv2.putText(image, f'c_r: {weight}', (x + 5, y + 10), cv2.FONT_HERSHEY_SIMPLEX , 0.3, (0, 255, 0), 1, cv2.LINE_AA)
            
            # Add person to the list of centroid points
            centroids.append((centroid_point[0], centroid_point[1], weight))

        # Show the output images
        cv2.imshow("Detected", image)
        cv2.waitKey(1)

        return centroids

    def get_relative_positions(self, centroids : List[Point], depth_image) -> List[Point]:
        """Gets the relative 3D position of a list of centroids.
        
        ### Parameters
            - centroids - A list of 2D centroid points
            - depth_image - The camera depth image

        ### Returns
        A list of converted 3D points relative to the robot
        """
        # Focal Length
        fx : float = self.camera_info.K[0]
        fy : float = self.camera_info.K[4]

        # Principal Point
        cx : float = self.camera_info.K[2]
        cy : float = self.camera_info.K[5]
        
        rel_points : List[Point] = []

        for centroid in centroids:
            # Get image coordinates of centroid, needs to be scaled back up
            ix : int = int(centroid[0] / self.image_scale)
            iy : int = int(centroid[1] / self.image_scale)

            # Get the depth value at those image coordinates
            depth = depth_image[iy, ix]
            
            print(f'DEPTH: {depth}')
            # Skip if the depth is a number and is greater then the max distance
            if isnan(depth) or isinf(depth) or depth > self.max_detect_distance:
                print('Not close enough to object to determine position, skipping...')
                continue

            # Calculate the 3D relative position
            x = depth
            y = (-(ix - cx) * depth) / fx
            z = ((iy - cy) * depth) / fy

            rel_points.append((x, y, z))

        return rel_points

    def add_marker(self, point : Point, frame : str) -> None:
        """Adds rescue marker at a point from the specified frame.
        
        ### Parameters
            - point - A 3D point
            - frame - The reference transform frame
        """
        marker = Marker()

        marker.header.frame_id = frame
        marker.header.stamp = rospy.Time.now()

        # Set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = 1

        marker.id = self.marker_count

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

        # Publish the rescue marker
        self.marker_pub.publish(marker)

        # Increase current marker ID
        self.marker_count = self.marker_count + 1

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--detect_threshold', help="detection confidence threshold", type=float)
    parser.add_argument('-md', '--max_detect', help="maximum detection distance", type=float)
    parser.add_argument('-s', '--image_scale', help="scales image for detection (lower res images, better performance, lower accuracy)", type=float)
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    if args.detect_threshold:
        print(f"detection_threshold set to {args.detect_threshold}")
        detection_threshold = args.detect_threshold
    else:
        print(f"detection_threshold set to default of 1.5")
        detection_threshold = 1.5

    if args.max_detect:
        print(f"max_detect_distance set to {args.max_detect}")
        max_detect_distance = args.max_detect
    else:
        print(f"max_detect_distance set to default of 2")
        max_detect_distance = 2

    if args.image_scale:
        print(f"image_scale set to {args.image_scale}")
        image_scale = args.image_scale
    else:
        print(f"image_scale set to default of 3")
        image_scale = 3
        
    # Instantiate Peeper Object
    detector = Peeper(detect_threshold=detection_threshold,
                      max_detect_distance=max_detect_distance,
                      image_scale=image_scale)

    while not rospy.is_shutdown():
        detector.peeping()

        rospy.sleep(0.01)