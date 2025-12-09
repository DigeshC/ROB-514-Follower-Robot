#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile


def quat_to_yaw(qz, qw):
    return math.atan2(2.0 * qz * qw, qw*qw - qz*qz)


class OdomGoHomeTriggered(Node):
    def __init__(self):
        super().__init__('odom_go_home_triggered')

        qos = QoSProfile(depth=10)

        # Publish TwistStamped (required by turtlebot3_node)
        self.cmd_pub = self.create_publisher(TwistStamped, "/cmd_vel", qos)

        # Subscribe to odom
        self.create_subscription(Odometry, "/odom", self.odom_callback, qos)

        # Subscribe to trigger (from aruco_follower_node.cpp)
        self.create_subscription(Bool, "/go_home", self.trigger_callback, qos)

        # Internal state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.home_recorded = False
        self.should_go_home = False

        # Timer
        self.timer = self.create_timer(0.05, self.update)

        # Safety shutdown
        rclpy.get_default_context().on_shutdown(self.safe_stop)

        self.get_logger().info("Go-Home Triggered Node Running...")

    # ------------------------------------------------------
    # SAFETY STOP
    # ------------------------------------------------------
    def safe_stop(self):
        stop = TwistStamped()
        stop.header.stamp = self.get_clock().now().to_msg()
        stop.header.frame_id = "base_link"

        for _ in range(5):  # ensure stop delivered
            self.cmd_pub.publish(stop)

        self.get_logger().warn("SAFETY STOP — Robot Halted")

    # ------------------------------------------------------
    # ODOM CALLBACK
    # ------------------------------------------------------
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.current_yaw = quat_to_yaw(qz, qw)

        if not self.home_recorded:
            self.home_x = self.current_x
            self.home_y = self.current_y
            self.home_yaw = self.current_yaw
            self.home_recorded = True

            self.get_logger().info(
                f"Home saved at x={self.home_x:.2f}, y={self.home_y:.2f}, yaw={self.home_yaw:.2f}"
            )

    # ------------------------------------------------------
    # TRIGGER CALLBACK
    # ------------------------------------------------------
    def trigger_callback(self, msg):
        if msg.data:
            self.get_logger().info("🚀 Return Home Triggered")
            self.should_go_home = True
        else:
            # Received false - stop all home movement (from aruco_follower_node.cpp)
            self.get_logger().info("Stop Home command received → exiting HOME mode")
            self.should_go_home = False
            # Publish zero velocity to stop any ongoing home movement
            stop_cmd = TwistStamped()
            stop_cmd.header.stamp = self.get_clock().now().to_msg()
            stop_cmd.header.frame_id = "base_link"
            stop_cmd.twist.linear.x = 0.0
            stop_cmd.twist.angular.z = 0.0
            self.cmd_pub.publish(stop_cmd)

    # ------------------------------------------------------
    # CONTROL LOOP — POSITION ONLY
    # ------------------------------------------------------
    def update(self):

        if not self.should_go_home or not self.home_recorded:
            return

        # Vector to home
        dx = self.home_x - self.current_x
        dy = self.home_y - self.current_y
        distance = math.sqrt(dx * dx + dy * dy)

        # If close enough, stop
        if distance < 0.05:
            self.get_logger().info("🏠 Arrived home (position only).")
            self.safe_stop()
            self.should_go_home = False
            return

        # Heading to home
        angle_to_home = math.atan2(dy, dx)
        yaw_error = self.normalize(angle_to_home - self.current_yaw)

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"

        # Rotate first
        if abs(yaw_error) > 0.18:
            cmd.twist.angular.z = 0.5 * np.sign(yaw_error)
        else:
            # Then move forward with small correction
            cmd.twist.linear.x = 0.8
            cmd.twist.angular.z = 0.2 * yaw_error  # Reduced from 0.4 to turn less

        self.cmd_pub.publish(cmd)

    # ------------------------------------------------------
    def normalize(self, a):
        return math.atan2(math.sin(a), math.cos(a))

# ------------------------------------------------------
# MAIN
# ------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = OdomGoHomeTriggered()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("CTRL-C — stopping robot.")
    finally:
        node.safe_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#trigger signal = ros2 topic pub /go_home std_msgs/Bool "data: true"
#manually kill all motion ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped "twist: {linear: {x: 0.0}, angular: {z: 0.0}}" -1