import rclpy
from rclpy.node import Node

from apriltag_msgs.msg import AprilTagDetectionArray
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg import String
from sensor_msgs.msg import Image

from cv_bridge import CvBridge

from transforms3d.quaternions import mat2quat
from transforms3d.euler import mat2euler
from transforms3d.axangles import mat2axangle

import math
import numpy as np
import matplotlib.pyplot as plt
import time

plt.ion()
figure, ax = plt.subplots(figsize=(20, 20))


def decompose_homography(H, K, return_degrees=True):
    print("Homography " + str(H))
    # Decomposing the Homography Matrix into Pose Matrix
    H_prime = np.dot(np.linalg.inv(K), H)

    norm1 = np.linalg.norm(H_prime[:, 0])
    norm2 = np.linalg.norm(H_prime[:, 1])
    tnorm = (norm1 + norm2) / 2

    # Extract Rotation Matrix
    R = np.array(H_prime)
    R[:, 0] /= norm1
    R[:, 1] /= norm2
    R[:, 2] = np.cross(R[:, 0], R[:, 1])

    # U, S, V = np.linalg.svd(R, full_matrices=True)

    # U = np.matrix(U)
    # V = np.matrix(V)
    # R = np.dot(U, V).T

    t = H_prime[:, 2] / tnorm
    print("Uncorrected " + str(t * 2.5))
    print("Rotation Matrix " + str(R))
    t = np.dot(R.T, t) * 2.58

    e_angles = np.rad2deg(mat2euler(R))

    print("Corrected T " + str(t))
    print("Euler Angles " + str(e_angles))

    return R, t, e_angles


class Localizer(Node):

    def __init__(self):
        super().__init__('frc2468_apriltag_homography_localizer')
        self.apriltag_subscriber = self.create_subscription(AprilTagDetectionArray, '/apriltag/detections',
                                                            self.apriltag_callback, 10)
        self.camera_intrinsics_subscriber = self.create_subscription(CameraInfo, '/camera_info',
                                                                     self.camera_intrinsics_callback, 10)

        self.pose_publisher = self.create_publisher(PoseStamped, '/apriltag_pose', 10)

        # Initializing running variables
        self.camera_intrinsics = CameraInfo()
        self.camera_intrinsics.k = np.eye(3)
        self.pose = PoseStamped()
        self.pose.header.frame_id = "map"

        self.get_logger().info('Localizer Node Started')
        self.prev_time = time.time()

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.pose_publisher.publish(self.pose)

    def camera_intrinsics_callback(self, msg):
        self.camera_intrinsics = msg

    def apriltag_callback(self, msg):
        if msg.detections:
            print("---------------------------")
            detection = msg.detections[0]

            # Extract and invert homography matrix to get pose of the camera relative to the image
            H = detection.homography.reshape(3, 3)

            # Extract camera intrinsics matrix
            K = self.camera_intrinsics.k.reshape(3, 3)

            R, t, e_angles = decompose_homography(H, K, False)

            ax.clear()
            t[0] = -1 * t[0]
            e_angles[1] = e_angles[1] * -1
            ax.annotate("X: " + str(round(t[0], 3)) + " Y: " + str(round(t[2], 3)) + " Theta: " + str(round(e_angles[1], 3)), xy=(-45, 5), fontsize=30)
            e_angles = np.deg2rad(e_angles)
            ax.annotate("",
                        xy=(t[0] + 4 * math.sin(e_angles[1]), t[2] - 4 * math.cos(e_angles[1])), xycoords='data',
                        xytext=(t[0], t[2]), textcoords='data',
                        arrowprops=dict(arrowstyle="simple",
                                        connectionstyle="arc3",
                                        lw=4),
                        )
            plt.xlim(-50, 50)
            plt.ylim(70, 0)
            figure.canvas.draw()
            figure.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)

    localizer = Localizer()

    rclpy.spin(localizer)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    localizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
