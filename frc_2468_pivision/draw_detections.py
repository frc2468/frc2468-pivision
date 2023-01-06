import rclpy
from rclpy.node import Node

from apriltag_msgs.msg import AprilTagDetectionArray
from apriltag_msgs.msg import AprilTagDetection
from std_msgs.msg import String
from sensor_msgs.msg import Image

from cv_bridge import CvBridge

import math
import numpy as np
import cv2
import matplotlib.pyplot as plt
import time


def annotate_image(image, corners):
    # cv2.rectangle(image, (corners[0].x, corners[0].y), (corners[2].x, corners[2].y), (255, 255, 0))

    start_point = (5, 5)

    # Ending coordinate, here (220, 220)
    # represents the bottom right corner of rectangle
    end_point = (220, 220)

    # Blue color in BGR
    color = (255, 0, 0)

    # Line thickness of 2 px
    thickness = 2

    # Using cv2.rectangle() method
    # Draw a rectangle with blue line borders of thickness of 2 px
    image = cv2.rectangle(image, start_point, end_point, color, thickness)

    for corner in corners:
        cv2.circle(image, (corner.x, corner.y), 3, (255, 0, 0))
    return image


class DetectionAnnotater(Node):

    def __init__(self):
        super().__init__('frc2468_apriltag_detection_annotater')
        self.get_logger().info('Annotation Node Started')
        self.apriltag_subscriber = self.create_subscription(AprilTagDetectionArray, '/apriltag/detections',
                                                            self.apriltag_callback, 10)
        self.image_subscriber = self.create_subscription(Image, '/image_raw', self.image_callback, 10)

        self.image_publisher = self.create_publisher(Image, '/detections_image', 10)

        self.bridge = CvBridge()

        self.detection_found = False
        self.detection = AprilTagDetection()
        self.camera_image = Image()
        self.annotated_image = Image()

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def apriltag_callback(self, msg):
        if msg.detections:
            self.detection_found = True
            self.detection = msg.detections[0]
        self.detection_found = False

    def image_callback(self, msg):
        self.camera_image = msg

    def timer_callback(self):
        if self.detection_found:
            image = self.bridge.imgmsg_to_cv2(self.camera_image, desired_encoding='passthrough')
            print('hi')
            print(np.max(image))
            image = annotate_image(image, self.detection.corners)
            image_msg = self.bridge.cv2_to_imgmsg(image, desired_encoding='passthrough')
            self.image_publisher.publish(image_msg)

def main(args=None):
    rclpy.init(args=args)

    annotater = DetectionAnnotater()

    rclpy.spin(annotater)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    annotater.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
