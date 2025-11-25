#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import math


class MoveTurnMove(Node):
    def __init__(self):
        super().__init__('move_turn_move')

        # ===== User settings =====
        self.forward_distance = 0.6
        self.backward_distance = 0.6
        self.rotate_degrees = 360   # degrees to rotate (positive = CCW)

        # Publisher
        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # Subscribe to odometry
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # Timer for publishing velocity
        self.timer = self.create_timer(0.05, self.timer_callback)

        # Internal state
        self.start_x = None
        self.start_y = None
        self.start_yaw = None

        self.phase = "forward"        # forward → rotate → backward → done
        self.reached_goal = False

        self.get_logger().info("Sequence started: forward 0.6m → rotate 360° → backward 0.6m")

    # ------------------------------------------------------
    # Extract yaw (rotation around Z) from odom quaternion
    # ------------------------------------------------------
    def get_yaw_from_quaternion(self, q):
        # ZYX Euler conversion
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    # ------------------------------------------------------
    # Odom callback: measures position and rotation
    # ------------------------------------------------------
    def odom_callback(self, msg):

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = self.get_yaw_from_quaternion(q)

        # Capture starting point and yaw for each phase
        if self.start_x is None:
            self.start_x = x
            self.start_y = y
            self.start_yaw = yaw
            return

        # ---------------- Phase 1: Move Forward ----------------
        if self.phase == "forward":
            dist = math.sqrt((x - self.start_x)**2 + (y - self.start_y)**2)
            if dist >= self.forward_distance:
                self.get_logger().info(f"Forward distance {dist:.2f}m reached → rotating")
                self.stop_robot()
                self.phase = "rotate"
                self.start_yaw = yaw   # reset rotation starting angle

        # ---------------- Phase 2: Rotate 360 ----------------
        elif self.phase == "rotate":
            angle = abs(self.normalize_angle(yaw - self.start_yaw))
            if angle >= math.radians(self.rotate_degrees):
                self.get_logger().info("Rotation completed → moving backward")
                self.stop_robot()
                self.phase = "backward"
                self.start_x = x
                self.start_y = y

        # ---------------- Phase 3: Move Backward ----------------
        elif self.phase == "backward":
            dist = math.sqrt((x - self.start_x)**2 + (y - self.start_y)**2)
            if dist >= self.backward_distance:
                self.get_logger().info(f"Backward {dist:.2f}m reached → sequence done")
                self.stop_robot()
                self.phase = "done"
                rclpy.shutdown()

    # ------------------------------------------------------
    # Normalize angle to [-pi, pi]
    # ------------------------------------------------------
    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    # ------------------------------------------------------
    # Timer: publish velocity commands for each phase
    # ------------------------------------------------------
    def timer_callback(self):
        if self.phase == "done":
            return

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        if self.phase == "forward":
            msg.twist.linear.x = 0.12
        elif self.phase == "rotate":
            msg.twist.angular.z = 0.8
        elif self.phase == "backward":
            msg.twist.linear.x = -0.12
        else:
            return

        self.publisher_.publish(msg)

    # ------------------------------------------------------
    # Stops robot immediately
    # ------------------------------------------------------
    def stop_robot(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MoveTurnMove()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
