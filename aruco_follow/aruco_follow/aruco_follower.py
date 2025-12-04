#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np

class ArucoFollower(Node):
    def __init__(self):
        super().__init__('aruco_follower')

        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)

        # PUBLISH TwistStamped instead of Twist
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        self.mode = "STOP"
        self.gesture_sub = self.create_subscription(
            String,
            '/gesture_command',
            self.gesture_callback,
            10
        )

        self.bridge = CvBridge()

        # ArUco Setup
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.dictionary, self.parameters)

        # Low-pass filtering
        self.filtered_error_x = 0.0
        self.filtered_size_error = 0.0
        self.alpha = 0.2

        # Safe speed limits
        self.max_lin = 0.15
        self.max_ang = 1.5

        # Target marker pixel size
        self.desired_marker_px = 120

    def gesture_callback(self, msg: String):
        self.mode = msg.data.strip().upper()
        self.get_logger().info(f"Gesture command set to: {self.mode}")

    def image_callback(self, msg):
        # If HOME: stop controlling /cmd_vel at all
        if self.mode == "HOME":
            # Do NOT publish cmd_vel so another node can use the topic
            self.get_logger().debug("HOME mode → not publishing cmd_vel")
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = self.detector.detectMarkers(gray)

        # Create stamped cmd_vel
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"

        # If STOP: always publish zero velocity and return
        if self.mode == "STOP":
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            self.get_logger().info("STOP mode → publishing zero cmd_vel")
            return

        # FOLLOW
        if ids is not None:
            c = corners[0][0]

            # Compute center
            marker_x = int((c[0][0] + c[2][0]) / 2)
            img_center_x = frame.shape[1] // 2
            error_x = marker_x - img_center_x

            # Marker size estimation
            marker_size_px = np.linalg.norm(c[0] - c[1])
            size_error = self.desired_marker_px - marker_size_px

            # Low-pass filtering
            self.filtered_error_x = (
                self.alpha * error_x + (1 - self.alpha) * self.filtered_error_x
            )
            self.filtered_size_error = (
                self.alpha * size_error + (1 - self.alpha) * self.filtered_size_error
            )

            # Gains
            K_turn = 0.0025
            K_forward = 0.003

            # Control law
            cmd.twist.angular.z = -self.filtered_error_x * K_turn
            cmd.twist.linear.x = self.filtered_size_error * K_forward

            # Clamp speeds
            cmd.twist.angular.z = max(min(cmd.twist.angular.z, self.max_ang), -self.max_ang)
            cmd.twist.linear.x = max(min(cmd.twist.linear.x, self.max_lin), -self.max_lin)

            # Safety: don't drive forward while turning sharply
            if abs(cmd.twist.angular.z) > 0.8:
                cmd.twist.linear.x = 0.0

            self.get_logger().info(
                f"[MARKER] ID={ids[0][0]} | err_x={error_x:.1f} | "
                f"filt_err={self.filtered_error_x:.1f} | size={marker_size_px:.1f}"
            )

        else:
            # Search pattern
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0
            self.get_logger().info("Marker lost → searching...")

        # Publish stamped twist (FOLLOW mode)
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
