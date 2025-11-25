#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile

#Convert from quaternion to yaw. Only need yaw for turtlebot
def quat_to_yaw(qz, qw):
    return math.atan2(2.0 * qz * qw, qw*qw - qz*qz)

#Define ROS node
class OdomGoHomeTriggered(Node):
    def __init__(self):
        super().__init__('odom_go_home_triggered')
        #keep up to 10 msgs in buffer
        qos = QoSProfile(depth=10)

        # Create publisher that sends out TwistStamped msgs to topic /cmd_vel
        self.cmd_pub = self.create_publisher(TwistStamped, "/cmd_vel", qos)

        # Create odometry subscriber to topic /odom, function odom_callback processes odom data
        self.create_subscription(Odometry, "/odom", self.odom_callback, qos)

        # Create trigger subscriber to topic /go_home which listens for home trigger
        # ros2 topic pub /go_home std_msgs/Bool "data: true"
        self.create_subscription(Bool, "/go_home", self.trigger_callback, qos)

        # Variables to store current state
        self.current_x = 0.0 # robot's current x pos
        self.current_y = 0.0 # robot's current y pos
        self.current_yaw = 0.0 # robot's current yaw pos
        self.home_recorded = False # variable to store if home position has already been saved
        self.should_go_home = False #set to true when /go_home is triggered

        # Calls self.update every .05s
        self.timer = self.create_timer(0.05, self.update)

        # Safety shutdown
        rclpy.get_default_context().on_shutdown(self.safe_stop)

        #Log to useer when node is triggerd (i.e recieve go home trigger)
        self.get_logger().info("Go-Home Triggered Node Running...")

   #Create stop Twist
    def safe_stop(self):
        stop = TwistStamped() #new TwistStamped msg called stop
        stop.header.stamp = self.get_clock().now().to_msg() # curernt time from clock
        stop.header.frame_id = "base_link" #send stop command to base_link

        for _ in range(5):  # publish stop 5 times to /cmd_vel
            self.cmd_pub.publish(stop)

        #Log when safety stop is implemented
        self.get_logger().warn("SAFETY STOP — Robot Halted")

    #Odom callback function which processes /odom data
    #input to function -> self, and /odom msgs. Triggered whenever new odom msg arrives
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x #store current x position
        self.current_y = msg.pose.pose.position.y #store current y position

        qz = msg.pose.pose.orientation.z #store quaternion qz
        qw = msg.pose.pose.orientation.w #store quaternion qw
        self.current_yaw = quat_to_yaw(qz, qw) # convert quaternion to yaw and store

        #Set current starting position as home
        if not self.home_recorded:
            self.home_x = self.current_x
            self.home_y = self.current_y
            self.home_yaw = self.current_yaw
            self.home_recorded = True
        #log msg that we recorded current pos as home pos
            self.get_logger().info(
                f"Home saved at x={self.home_x:.2f}, y={self.home_y:.2f}, yaw={self.home_yaw:.2f}"
            )

    #Function which is callled when we get /go_home msg
    def trigger_callback(self, msg):
        if msg.data:
            self.get_logger().info("Return Home Triggered") #log that we are returning home
            self.should_go_home = True


    def update(self):
        #if we dont have go home trigger and haven't recorded home pos do nothing
        if not self.should_go_home or not self.home_recorded:
            return

        # x,y vector to home
        dx = self.home_x - self.current_x
        dy = self.home_y - self.current_y
        distance = math.sqrt(dx * dx + dy * dy)

        # stop moving if within 5 cm of home position
        if distance < 0.05:
            self.get_logger().info("Arrived home (position only).")
            self.safe_stop()
            self.should_go_home = False
            return

        # Calculate angle to turn to go back home
        angle_to_home = math.atan2(dy, dx) #compute where robot should be pointing
        yaw_error = self.normalize(angle_to_home - self.current_yaw) #normalize to +/- pi
        #build velocity command TwistStamped msg
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"

        # Rotate first
        if abs(yaw_error) > 0.01:
            cmd.twist.angular.z = 0.5 * np.sign(yaw_error)
            cmd.twist.linear.x = 0.0
        else:
            # Then move forward
            cmd.twist.linear.x = 0.12
            cmd.twist.angular.z = 0.0

        self.cmd_pub.publish(cmd)

    # Normalize angle to +/- pi
    def normalize(self, a):
        return math.atan2(math.sin(a), math.cos(a))

#main function
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
