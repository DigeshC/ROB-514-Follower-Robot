#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image

from cv_bridge import CvBridge

import cv2
import mediapipe as mp
from math import sqrt
from collections import deque


class HandGestureCommand(Node):
    def __init__(self):
        super().__init__('hand_gesture_command')

        # Publisher for robot commands
        self.cmd = self.create_publisher(String, '/gesture_command', 10)

        # Subscriber to image topic
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10)

        # MediaPipe setup
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        # self.mp_draw = mp.solutions.drawing_utils
        # self.mp_drawing_styles = mp.solutions.drawing_styles

        # Gesture history for stabilization (last 5 gestures)
        self.gesture_history = deque(maxlen=5)
        self.last_published_gesture = None 
        self.previous_cmd = String()
        self.previous_cmd.data = "STOP"

        self.get_logger().info("Hand Gesture Controller Started")

    # -------------------------------------------------------------------------
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        self.process_frame(frame)


    def is_finger_extended(self, tip, pip, wrist):
        # Check if a finger is extended
        # For most fingers: tip should be above PIP joint
        return tip.y < pip.y

    def is_thumb_extended(self, tip, ip, wrist):
        # Check if thumb is extended (different logic)
        # For thumb: check if it's away from the hand
        return tip.x < ip.x if tip.x < wrist.x else tip.x > ip.x

    # -------------------------------------------------------------------------
    def detect_gesture(self, hand_landmarks):
        try:
            wrist = hand_landmarks.landmark[0]
            
            # Finger tips and PIP joints
            thumb_tip = hand_landmarks.landmark[4]
            thumb_ip = hand_landmarks.landmark[3]
            
            index_tip = hand_landmarks.landmark[8]
            index_pip = hand_landmarks.landmark[6]
            
            middle_tip = hand_landmarks.landmark[12]
            middle_pip = hand_landmarks.landmark[10]
            
            ring_tip = hand_landmarks.landmark[16]
            ring_pip = hand_landmarks.landmark[14]
            
            pinky_tip = hand_landmarks.landmark[20]
            pinky_pip = hand_landmarks.landmark[18]
            
            # Check which fingers are extended
            thumb_ext = self.is_thumb_extended(thumb_tip, thumb_ip, wrist)
            index_ext = self.is_finger_extended(index_tip, index_pip, wrist)
            middle_ext = self.is_finger_extended(middle_tip, middle_pip, wrist)
            ring_ext = self.is_finger_extended(ring_tip, ring_pip, wrist)
            pinky_ext = self.is_finger_extended(pinky_tip, pinky_pip, wrist)
            
            # Count extended fingers
            extended_fingers = [thumb_ext, index_ext, middle_ext, ring_ext, pinky_ext]
            count_extended = sum(extended_fingers)
            
            self.get_logger().debug(f"Fingers extended: {count_extended} - Thumb:{thumb_ext}, Index:{index_ext}, Middle:{middle_ext}, Ring:{ring_ext}, Pinky:{pinky_ext}")
            
            # Gesture 1: OPEN HAND (all fingers extended) - STOP
            if count_extended >= 4:
                return "STOP"
            elif index_ext and not middle_ext and not ring_ext and not pinky_ext:
                return "FOLLOW"
            elif index_ext and middle_ext and not ring_ext and not pinky_ext:
                return "HOME"

            return "UNKNOWN"

        except Exception as e:
            self.get_logger().error(f"Gesture detection error: {e}")
            return "ERROR"

    # -------------------------------------------------------------------------
    def process_frame(self, frame):
        frame = cv2.flip(frame, 1)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        results = self.hands.process(rgb_frame)

        if len(self.gesture_history) == 0:
            current_gesture = "STOP"
        else:
            # Find most frequent gesture in history
            current_gesture = max(set(self.gesture_history), key=self.gesture_history.count)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # self.mp_draw.draw_landmarks(
                #     frame,
                #     hand_landmarks,
                #     self.mp_hands.HAND_CONNECTIONS,
                #     self.mp_drawing_styles.get_default_hand_landmarks_style(),
                #     self.mp_drawing_styles.get_default_hand_connections_style()
                # )

                current_gesture = self.detect_gesture(hand_landmarks)
                self.get_logger().debug(f"Detected gesture (raw): {current_gesture}")

        # If detected gesture is UNKNOWN or ERROR, fallback to history
        if current_gesture not in ("FOLLOW", "HOME", "STOP"):
            if len(self.gesture_history) > 0:
                current_gesture = self.last_published_gesture
                self.get_logger().debug(
                    f"Gesture UNKNOWN/ERROR - fallback to last published: {current_gesture}"
                )
            else:
                current_gesture = "STOP"
                self.get_logger().debug("Gesture UNKNOWN/ERROR - no history, using STOP")


        self.gesture_history.append(current_gesture)

        self.get_logger().debug(
            f"Gesture history: {list(self.gesture_history)} "
            f"(last published: {self.last_published_gesture})"
        )

        # Only act if we have 10 samples
        if len(self.gesture_history) == 10:
            # Check if last 10 gestures are the same
            if len(set(self.gesture_history)) == 1:
                stable_gesture = self.gesture_history[0]

                # Only publish if this stable gesture changed from last published
                if stable_gesture != self.last_published_gesture:
                    cmd = String()

                    if stable_gesture == "FOLLOW":
                        cmd.data = "FOLLOW"
                    elif stable_gesture == "HOME":
                        cmd.data = "HOME"
                    else:
                        cmd.data = "STOP"

                    self.cmd.publish(cmd)
                    self.previous_cmd.data = cmd.data
                    self.last_published_gesture = stable_gesture

                    self.get_logger().info(
                        f"PUBLISHING - Stable gesture: {stable_gesture}, Command: {cmd.data}"
                    )
    # --------------------------------------------------------------------------

        # Optional OpenCV window
        # cv2.imshow("Hand Gesture Controller", frame)
        # if cv2.waitKey(1) & 0xFF == ord("q"):
        #     self.cleanup()

    # -------------------------------------------------------------------------
    def cleanup(self):
        # cv2.destroyAllWindows()
        self.cmd.publish(String(data="STOP"))
        self.get_logger().info("Hand Gesture Controller Shutting Down")
        rclpy.shutdown()

def main():
    rclpy.init()
    node = HandGestureCommand()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()

if __name__ == '__main__':
    main()