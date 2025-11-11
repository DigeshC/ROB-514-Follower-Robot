# Overview
This repository is for ROB 514

The code here has been taken from multiple sources and modified to fit our use case, a follower robot

```bash
ros2 run gesture_commands hand_gesture_command 
```

Original repository: https://github.com/patience60-svg/gesture-control-ros2

# Gesture Control ROS2 🤖👋

A ROS2 package for controlling robots using intuitive hand gestures. This project enables natural human-robot interaction by translating hand gestures into velocity commands for mobile robots.

![ROS2](https://img.shields.io/badge/ROS2-Jazzy-brightgreen)
![Python](https://img.shields.io/badge/Python-3.8%2B-blue)
![OpenCV](https://img.shields.io/badge/OpenCV-4.5%2B-orange)
![MediaPipe](https://img.shields.io/badge/MediaPipe-Latest-yellow)

## 🎯 Features

- **Natural Gesture Control**: Use intuitive hand gestures to control robot movement
- **Real-time Processing**: Low-latency hand tracking with MediaPipe
- **ROS2 Integration**: Seamlessly integrates with ROS2 robotic systems
- **Multiple Gesture Support**: 
  - ☝️**POINTING** - Move Forward
  - 👍 **THUMBS UP** - Move Backward  
  - ✌️ **VICTORY** - Turn Left
  - 🤘 **ROCK SIGN** - Turn Right
  - 🖐️ **OPEN HAND** - Stop

## 🚀 Quick Start
 [![Demo Video](https://github.com/patience60-svg/gesture-control-ros2//tree/main/raw/thumbnail.png)](https://github.com/patience60-svg/gesture-control-ros2//tree/main/raw/demo.mp4) 

### Prerequisites
- ROS2 Jazzy (or compatible distribution)
- Python 3.8+
- Webcam
- TurtleBot3 packages (for simulation)

### Installation

1. **Clone the repository**
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/yourusername/gesture-control-ros2.git
2. **Run packages**
   ```bash
   ros2 launch turtlebot3_gazebo empty_world.launch.py #launch turtlebot3 in gazebo
   
   ros2 run gesture_commands hand_gesture_controller
   
   #optional(ros2 run gesture_commands finger_detector)
