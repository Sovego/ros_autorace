import time
from typing import Literal
from collections import deque

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, Quaternion
from tf_transformations import euler_from_quaternion
from cv_bridge import CvBridge,CvBridgeError
import cv2
import math
import numpy as np
from std_msgs.msg import String

from .roadsign_detection import SignDetector
from .mask_search import MaskSerach


class Camera_node(Node):

    def __init__(self): # 0.2 linear 2.15 angular
        super().__init__("Camera_node")
        # Создание подписчика на данные о положении
        # Создание подписчика на изображение с камеры
        self._robot_Ccamera_sub = self.create_subscription(Image, "/color/image", self.camera_callback, 1)
        self.mission_sub = self.create_subscription(String, "/mission", self.mission_callback, 1)
        self.state_pub = self.create_publisher(String,"/state",10)
        self.timer = self.create_timer(0.2, self.go_forward)
        self.detect_timer = self.create_timer(0.1, self.detect_callback)
        self.depth_camera = self.create_subscription(Image,"/depth/image",self.depth_callback,1)
        self._cv_bridge = CvBridge()
        self.last_position = Point()
        self.total_distance = 0.0
        self.is_first_message = True
        self.sign_type = -1
        self.distance_to_obstacle = 99999 
        self.frame = None
        self.d_frame = None
        self.detector = SignDetector(path_to_signs_imgs="./signs_images", debug_mode=True)
        self.on_mission = 0
        self.intersection_founded = False
        self.started = 0
        self.mask_searcher = MaskSerach()

    def mission_callback(self,data):
        self.on_mission = int(data.data)

    def camera_callback(self,data):
        try:
            cv_image = self._cv_bridge.imgmsg_to_cv2(data, desired_encoding=data.encoding)
            self.frame = cv_image
            self.frame = cv2.cvtColor(self.frame, cv2.COLOR_RGB2BGR)
            
        except CvBridgeError as e:
            self.get_logger().info(f'Error converting image: {str(e)}')
            return
        
    def depth_callback(self, data):
        try:
            cv_image = self._cv_bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
            self.d_frame = cv_image
        except CvBridgeError as e:
            self.get_logger().info(f'Error converting image: {str(e)}')
            return

    def go_forward(self):
        
        if self.started==1:
            self.started = 2
            msg = String()
            msg.data = "7"
            self.state_pub.publish(msg)
            
        
        if self.sign_type != -1 and self.sign_type is not None:
            msg = String()
            msg.data = str(self.sign_type)
            
            match self.sign_type:
                case 0:
                    pass

                case 1:
                    pass
                
                case 2:
                    if not self.intersection_founded:
                        self.detector.set_dst_treshhold(0.8)
                        self.intersection_founded = True
                    
                case 3 | 5:
                    self.detector.set_dst_treshhold(0.3)
                    
                case 4:
                    pass
                    
                case 6:
                    pass
            
            self.state_pub.publish(msg)
            self.sign_type=-1


    def detect_callback(self):
        
        if self.frame is not None and self.d_frame is not None and self.on_mission==0:
            #print(self.frame.shape)
            #print(self.d_frame.shape)
            #self.mask_searcher(self.frame)
            #cv2.imshow('Image', self.frame)
            #cv2.imshow('Depth Image', self.d_frame)
            #cv2.waitKey(0) 
            #time.sleep(10)
            if self.started==0:
                self.started = self.detector.greenlight_detect(self.frame)   
                                
            detect_res = self.detector(self.frame,self.d_frame)
            self.sign_type = detect_res[1]

def main():
    rclpy.init()
    FTN = Camera_node()
    rclpy.spin(FTN)
    FTN.destroy_node()
    rclpy.shutdown()