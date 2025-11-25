#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose


class GoHome(Node):
    def __init__(self):
        super().__init__('go_home')

        # ===== USER SET HOME LOCATION HERE =====
        self.home_x = 1.0      # meters
        self.home_y = -0.5     # meters
        self.home_yaw = 3.14   # radians (pi = face backwards)

        # Nav2 action client for navigation
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Publisher for safety stop
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # Safety hook (runs when node shuts down)
        self.add_on_shutdown(self.safe_stop)

        # Try sending goal once Nav2 is ready
        self.goal_sent = False
        self.timer = self.create_timer(1.0, self.send_home_goal)

        self.get_logger().info("GoHome node started. Will send robot to HOME pose.")

    # ---------------------------------------------------------
    # SAFETY STOP (runs on ANY shutdown path)
    # ---------------------------------------------------------
    def safe_stop(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0
        self.cmd_pub.publish(msg)
        self.get_logger().warn("SAFETY STOP: Robot stopped due to node shutdown.")

    # ---------------------------------------------------------
    # Create a PoseStamped message for Nav2
    # ---------------------------------------------------------
    def build_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        # Yaw → quaternion (2D rotation)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        return pose

    # ---------------------------------------------------------
    # Timer tries to send the goal exactly once
    # ---------------------------------------------------------
    def send_home_goal(self):
        if self.goal_sent:
            return

        if not self._client.wait_for_server(timeout_sec=0.5):
            self.get_logger().info("Waiting for Nav2 action server...")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.build_pose(self.home_x, self.home_y, self.home_yaw)

        self.get_logger().info(
            f"Sending HOME goal: ({self.home_x:.2f}, {self.home_y:.2f}, yaw={self.home_yaw:.2f})"
        )

        # Send goal asynchronously
        self._client.send_goal_async(goal_msg)
        self.goal_sent = True


# ============================================================
# MAIN
# ============================================================
def main(args=None):
    rclpy.init(args=args)
    node = GoHome()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("CTRL+C detected — performing safety stop.")
        node.safe_stop()

    except Exception as e:
        node.get_logger().error(f"ERROR: {e}")
        node.safe_stop()
        raise

    finally:
        node.safe_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
