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
#-------------------------------------------------------------------------------------v
class FingerFollower(Node):
    def __init__(self):
        super().__init__('finger_detector')
        
        # Publisher for robot velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        #-----------------------------------------------------------------------------
        # MediaPipe hands setup
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        self.mp_draw = mp.solutions.drawing_utils
        #-----------------------------------------------------------------------------
        # Camera setup
        self.cap = cv2.VideoCapture(0)
        self.cap.set(3, 640)  # Width
        self.cap.set(4, 480)  # Height
        #-----------------------------------------------------------------------------
        # Control parameters
        self.target_height = 0.3  # Target finger height (normalized)
        self.target_x = 0.5       # Target x position (center)
        self.dead_zone = 0.1      # Dead zone to prevent oscillations
        
        self.get_logger().info("Finger Follower Node Started")
        
        # Timer for continuous processing
        self.timer = self.create_timer(0.03, self.process_frame)  # ~30fps
    #---------------------------------------------------------------------------------
    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        
        # Flip frame for mirror effect
        frame = cv2.flip(frame, 1)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Process with MediaPipe
        results = self.hands.process(rgb_frame)
        
        cmd_vel = Twist()
        #-----------------------------------------------------------------------------
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Draw hand landmarks
                self.mp_draw.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                
                # Get index finger tip (landmark 8)
                finger_tip = hand_landmarks.landmark[8]
                
                # Convert to pixel coordinates
                h, w, _ = frame.shape
                x_pixel = int(finger_tip.x * w)
                y_pixel = int(finger_tip.y * h)
                
                # Draw finger tip
                cv2.circle(frame, (x_pixel, y_pixel), 10, (0, 255, 0), -1)
                
                # Calculate control signals
                self.control_robot(finger_tip, cmd_vel)
                
                # Display coordinates
                cv2.putText(frame, f'X: {finger_tip.x:.2f}, Y: {finger_tip.y:.2f}', 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        #-----------------------------------------------------------------------------
        # Display control info
        cv2.putText(frame, f'Linear: {cmd_vel.linear.x:.2f}, Angular: {cmd_vel.angular.z:.2f}', 
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        cv2.imshow('Finger Follower', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cleanup()
    #---------------------------------------------------------------------------------
    def control_robot(self, finger_tip, cmd_vel):
        """
        Control robot based on finger position:
        - Y position (height) controls linear velocity (forward/backward)
        - X position controls angular velocity (left/right)
        """
        # Linear velocity control (Y-axis)
        if finger_tip.y < self.target_height - self.dead_zone:
            # Finger up - move forward
            cmd_vel.linear.x = 0.2
        elif finger_tip.y > self.target_height + self.dead_zone:
            # Finger down - move backward
            cmd_vel.linear.x = -0.2
        else:
            cmd_vel.linear.x = 0.0
        
        # Angular velocity control (X-axis)
        if finger_tip.x < self.target_x - self.dead_zone:
            # Finger left - turn left
            cmd_vel.angular.z = 0.5
        elif finger_tip.x > self.target_x + self.dead_zone:
            # Finger right - turn right
            cmd_vel.angular.z = -0.5
        else:
            cmd_vel.angular.z = 0.0
        
        # Publish velocity command
        self.cmd_vel_pub.publish(cmd_vel)
    #---------------------------------------------------------------------------------
    def cleanup(self):
        # Cleanup resources
        self.cap.release()
        cv2.destroyAllWindows()
        # Stop the robot
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        self.get_logger().info("Finger Follower Node Shutting Down")
        rclpy.shutdown()
    #---------------------------------------------------------------------------------
#-------------------------------------------------------------------------------------
def main():
    rclpy.init()
    node = FingerFollower()
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