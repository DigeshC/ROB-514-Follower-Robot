#-------------------------------------------------------------------------------------
#-------------------------------------------------------------------------------------
#!/usr/bin/env python3
#-------------------------------------------------------------------------------------
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2
import mediapipe as mp
import numpy as np
from math import sqrt
#-------------------------------------------------------------------------------------
class HandGestureController(Node):
    def __init__(self):
        super().__init__('hand_gesture_controller')
        
        # Publisher for robot velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        #-----------------------------------------------------------------------------
        # MediaPipe hands setup
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.5,  # Lowered for better detection
            min_tracking_confidence=0.5
        )
        self.mp_draw = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        #-----------------------------------------------------------------------------
        # Camera setup
        self.cap = cv2.VideoCapture(0)
        self.cap.set(3, 640)  # Width
        self.cap.set(4, 480)  # Height
        #-----------------------------------------------------------------------------
        # Control parameters
        self.max_speed = 0.2
        self.max_angular = 0.8
        #-----------------------------------------------------------------------------
        # Hand state
        self.previous_gesture = "NO_HAND"
        self.gesture_stable_count = 0
        
        self.get_logger().info("Hand Gesture Controller Started")
        
        # Timer for continuous processing
        self.timer = self.create_timer(0.03, self.process_frame)
    #---------------------------------------------------------------------------------
    def calculate_distance(self, point1, point2):
        # Calculate distance between two landmarks
        return sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)
    #---------------------------------------------------------------------------------
    def is_finger_extended(self, tip, pip, wrist):
        # Check if a finger is extended
        # For most fingers: tip should be above PIP joint
        return tip.y < pip.y
    #---------------------------------------------------------------------------------
    def is_thumb_extended(self, tip, ip, wrist):
        # Check if thumb is extended (different logic)
        # For thumb: check if it's away from the hand
        return tip.x < ip.x if tip.x < wrist.x else tip.x > ip.x
    #---------------------------------------------------------------------------------
    def detect_gesture(self, hand_landmarks):
        # Detect hand gesture for robot control
        try:
            # Get key landmarks
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
            thumb_extended = self.is_thumb_extended(thumb_tip, thumb_ip, wrist)
            index_extended = self.is_finger_extended(index_tip, index_pip, wrist)
            middle_extended = self.is_finger_extended(middle_tip, middle_pip, wrist)
            ring_extended = self.is_finger_extended(ring_tip, ring_pip, wrist)
            pinky_extended = self.is_finger_extended(pinky_tip, pinky_pip, wrist)
            
            # Count extended fingers
            extended_fingers = [thumb_extended, index_extended, middle_extended, ring_extended, pinky_extended]
            count_extended = sum(extended_fingers)
            
            self.get_logger().info(f"Fingers extended: {count_extended} - Thumb:{thumb_extended}, Index:{index_extended}, Middle:{middle_extended}, Ring:{ring_extended}, Pinky:{pinky_extended}")
            
            # Gesture 1: OPEN HAND (all fingers extended) - STOP
            if count_extended >= 4:
                return "STOP"
            
            # Gesture 2: POINTING (only index extended) - FORWARD
            elif index_extended and not middle_extended and not ring_extended and not pinky_extended:
                return "FORWARD"
            
            # Gesture 3: THUMBS UP (only thumb extended) - BACKWARD
            elif thumb_extended and not index_extended and not middle_extended and not ring_extended and not pinky_extended:
                return "BACKWARD"
            
            # Gesture 4: VICTORY (index + middle extended) - LEFT
            elif index_extended and middle_extended and not ring_extended and not pinky_extended:
                return "LEFT"
            
            # Gesture 5: ROCK (thumb + pinky extended) - RIGHT
            elif thumb_extended and pinky_extended and not index_extended and not middle_extended and not ring_extended:
                return "RIGHT"
            
            # Default - no clear gesture
            return "UNKNOWN"
        #-----------------------------------------------------------------------------    
        except Exception as e:
            self.get_logger().error(f"Error in gesture detection: {e}")
            return "ERROR"
    #---------------------------------------------------------------------------------
    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to read frame from camera")
            return
        
        # Flip frame for mirror effect
        frame = cv2.flip(frame, 1)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Process with MediaPipe
        results = self.hands.process(rgb_frame)
        
        cmd_vel = Twist()
        current_gesture = "NO_HAND"
        #-----------------------------------------------------------------------------
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Draw hand landmarks
                self.mp_draw.draw_landmarks(
                    frame,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS,
                    self.mp_drawing_styles.get_default_hand_landmarks_style(),
                    self.mp_drawing_styles.get_default_hand_connections_style())
                
                # Detect gesture
                current_gesture = self.detect_gesture(hand_landmarks)
                self.get_logger().info(f"Detected gesture: {current_gesture}")
                
                # Apply gesture-based control
                if current_gesture == "FORWARD":
                    cmd_vel.linear.x = self.max_speed
                    self.get_logger().info("Sending FORWARD command")
                elif current_gesture == "BACKWARD":
                    cmd_vel.linear.x = -self.max_speed * 0.7
                    self.get_logger().info("Sending BACKWARD command")
                elif current_gesture == "LEFT":
                    cmd_vel.angular.z = self.max_angular
                    self.get_logger().info("Sending LEFT command")
                elif current_gesture == "RIGHT":
                    cmd_vel.angular.z = -self.max_angular
                    self.get_logger().info("Sending RIGHT command")
                elif current_gesture == "STOP":
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = 0.0
                    self.get_logger().info("Sending STOP command")
                else:
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = 0.0
                
                # Draw palm center for reference
                wrist = hand_landmarks.landmark[0]
                h, w, _ = frame.shape
                palm_x = int(wrist.x * w)
                palm_y = int(wrist.y * h)
                cv2.circle(frame, (palm_x, palm_y), 8, (255, 0, 0), -1)
        #-----------------------------------------------------------------------------
        # Add gesture stabilization to prevent flickering
        if current_gesture == self.previous_gesture:
            self.gesture_stable_count += 1
        else:
            self.gesture_stable_count = 0
        
        # Only publish if gesture is stable for a few frames
        if self.gesture_stable_count > 2:  # Reduced for more responsive control
            self.cmd_vel_pub.publish(cmd_vel)
            self.get_logger().info(f"PUBLISHING - Linear: {cmd_vel.linear.x:.2f}, Angular: {cmd_vel.angular.z:.2f}")
        
        self.previous_gesture = current_gesture
        
        # Display information
        self.draw_control_info(frame, current_gesture, cmd_vel)
        
        cv2.imshow('Hand Gesture Controller', frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.cleanup()
        elif key == ord(' '):  # Space bar to force stop
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            self.get_logger().info("Force STOP published")
    #---------------------------------------------------------------------------------
    def draw_control_info(self, frame, gesture, cmd_vel):
        # Draw control information on the frame
        # Gesture info
        cv2.putText(frame, f'Gesture: {gesture}', 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Control values
        cv2.putText(frame, f'Linear: {cmd_vel.linear.x:.2f}', 
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(frame, f'Angular: {cmd_vel.angular.z:.2f}', 
                   (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Publishing status
        status = "PUBLISHING" if self.gesture_stable_count > 2 else "WAITING_STABLE"
        cv2.putText(frame, f'Status: {status}', 
                   (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        # Control guide
        guide_text = [
            "Controls:",
            "OPEN HAND (4-5 fingers) - STOP",
            "POINT (index only) - FORWARD", 
            "THUMBS UP - BACKWARD",
            "V-SIGN (index+middle) - LEFT",
            "ROCK (thumb+pinky) - RIGHT",
            "Press SPACE to force stop",
            "Press Q to quit"
        ]
        
        for i, text in enumerate(guide_text):
            cv2.putText(frame, text, (10, 140 + i * 25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
    #---------------------------------------------------------------------------------
    def cleanup(self):
        # Cleanup resources
        self.cap.release()
        cv2.destroyAllWindows()
        # Stop the robot
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        self.get_logger().info("Hand Gesture Controller Shutting Down")
        rclpy.shutdown()
    #---------------------------------------------------------------------------------
#-------------------------------------------------------------------------------------
def main():
    rclpy.init()
    node = HandGestureController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
#-------------------------------------------------------------------------------------
if __name__ == '__main__':
    main()
#-------------------------------------------------------------------------------------
#-------------------------------------------------------------------------------------