#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped

from rclpy.qos import qos_profile_sensor_data


class LidarFollower(Node):
    def __init__(self):
        super().__init__('lidar_follower')

        # Parameters
        self.declare_parameter('target_distance', 0.6)
        self.declare_parameter('distance_tolerance', 0.1)
        self.declare_parameter('max_linear_speed', 0.25)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('front_angle_width', 60.0)
        self.declare_parameter('rate', 10.0)

        self.target_distance = self.get_parameter('target_distance').get_parameter_value().double_value
        self.distance_tolerance = self.get_parameter('distance_tolerance').get_parameter_value().double_value
        self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        self.front_angle_width = math.radians(
            self.get_parameter('front_angle_width').get_parameter_value().double_value
        )

        self.current_scan = None

        # Subscriber
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        # Publisher
        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )

        # Timer
        rate = self.get_parameter('rate').get_parameter_value().double_value
        self.control_timer = self.create_timer(1.0 / rate, self.control_loop)

        self.get_logger().info('LidarFollower node initialized.')

    def scan_callback(self, msg):
        self.current_scan = msg
        self.get_logger().info("✔ Receiving /scan messages.")

    def control_loop(self):
        if self.current_scan is None:
            self.get_logger().warn("⚠ No scan data yet.")
            return

        scan = self.current_scan
        valid_points = []

        # Parse scan data
        for i, r in enumerate(scan.ranges):
            print(f"Index: {i}, Range: {r}")
            if math.isfinite(r):
                angle = scan.angle_min + i * scan.angle_increment
                valid_points.append((angle, r))

        self.get_logger().debug(f"Valid points: {len(valid_points)}")

        if len(valid_points) == 0:
            self.get_logger().warn("⚠ No valid scan points.")
            self._publish_cmd(0.0, 0.0)
            return

        # Front sector filtering
        half_width = self.front_angle_width / 2.0
        front = [(a, r) for (a, r) in valid_points if -half_width <= a <= half_width]

        self.get_logger().debug(f"Front sector points: {len(front)}")

        if not front:
            self.get_logger().warn("⚠ No object in front sector. Rotating to search...")
            self._publish_cmd(0.0, 0.3)
            return

        # Find closest front object
        closest_angle, closest_range = min(front, key=lambda x: x[1])

        self.get_logger().info(
            f"Closest object — angle: {closest_angle:.2f}, distance: {closest_range:.2f}"
        )

        # Distance control
        error = closest_range - self.target_distance

        if abs(error) < self.distance_tolerance:
            linear_x = 0.0
        else:
            linear_x = 0.8 * error
            linear_x = max(-self.max_linear_speed, min(self.max_linear_speed, linear_x))

        # Angle control
        angular_z = -1.5 * closest_angle
        angular_z = max(-self.max_angular_speed, min(self.max_angular_speed, angular_z))

        self.get_logger().info(
            f"CMD → linear: {linear_x:.2f}, angular: {angular_z:.2f}"
        )

        # self._publish_cmd(linear_x, angular_z)

    def _publish_cmd(self, linear_x, angular_z):
        msg = TwistStamped()

        # Fill header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"   # or "odom" or empty ""

        # Fill twist
        msg.twist.linear.x = float(linear_x)
        msg.twist.angular.z = float(angular_z)

        self.cmd_pub.publish(msg)

        self.get_logger().info(
            f"[PUB] TwistStamped → linear_x={linear_x:.3f}, angular_z={angular_z:.3f}"
        )



def main(args=None):
    rclpy.init(args=args)
    node = LidarFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
