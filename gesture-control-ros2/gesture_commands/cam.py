#-------------------------------------------------------------------------------------
#-------------------------------------------------------------------------------------
#!/usr/bin/env python3
#-------------------------------------------------------------
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
#-------------------------------------------------------------
from cv_bridge import CvBridge
import numpy as np
import cv2
import os
#-------------------------------------------------------------------------------------
#-------------------------------------------------------------------------------------
class CvNode(Node):
    def __init__(self):
        super().__init__('cam')

        self.pub = self.create_publisher(Image, 'cam_fd', 10)
        #-----------------------------------------------------------------------------
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.cam_data)

        #cv2.setUseOptimized(True)
        #cv2.setNumThreads(0)
        #-----------------------------------------------------------------------------    
        self.cap = cv2.VideoCapture(0)
        #ur_cam = 'http://192.168.140.250:8080/video'
        #self.cap = cv2.VideoCapture(ur_cam)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        #-----------------------------------------------------------------------------    
        self.br = CvBridge()
    #---------------------------------------------------------------------------------
    def cam_data(self):
        ret, frame = self.cap.read()
          
        if ret == True:
            #frame = cv2.UMat(frame)
            self.pub.publish(self.br.cv2_to_imgmsg(frame))
        
            self.get_logger().info('Publishing video frame')

        #cv2.imshow('Object Detection', frame)
        #cv2.waitKey(1)
    #---------------------------------------------------------------------------------  
#-------------------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = CvNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
#-------------------------------------------------------------------------------------
if __name__ == '__main__':
    main()
#-------------------------------------------------------------------------------------
#-------------------------------------------------------------------------------------
#ros2 run gesture_commands cam