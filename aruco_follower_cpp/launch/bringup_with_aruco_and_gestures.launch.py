#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ------------------------------------------------------------
    # 1. TurtleBot3 bringup
    # ------------------------------------------------------------
    turtlebot3_bringup_dir = get_package_share_directory("turtlebot3_bringup")
    turtlebot3_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_bringup_dir, "launch", "robot.launch.py")
        )
    )

    # ------------------------------------------------------------
    # 2. v4l2_camera node
    # ------------------------------------------------------------
    v4l2_camera_node = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="v4l2_camera_node",
        output="screen",
        parameters=[
            {"image_size": [640, 480]},
            {"time_per_frame": [1, 15]},  # ~15 FPS
        ],
    )

    # ------------------------------------------------------------
    # 3. Gesture command node
    # ------------------------------------------------------------
    gesture_node = Node(
        package="gesture_commands",
        executable="hand_gesture_command",
        name="hand_gesture_command",
        output="screen",
    )

    # ------------------------------------------------------------
    # 4. ArUco follower node (your C++ package)
    # ------------------------------------------------------------
    aruco_follower_node = Node(
        package="aruco_follower_cpp",
        executable="aruco_follower_node",
        name="aruco_follower_node",
        output="screen",
        parameters=[
            {"image_topic": "/image_raw"},
            {"desired_marker_size_px": 80.0},
            {"k_linear": 0.5},
            {"k_angular": 1.0},
        ],
    )

    send_home_node = Node(
        package="driver_package",
        executable="send_home",
        name="send_home",
        output="screen",
    )

    # ------------------------------------------------------------
    # LaunchDescription
    # ------------------------------------------------------------
    return LaunchDescription([
        turtlebot3_launch,
        v4l2_camera_node,
        gesture_node,
        aruco_follower_node,
        send_home_node
    ])
