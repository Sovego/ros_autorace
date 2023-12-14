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



class Camera_node(Node):

    def __init__(self): # 0.2 linear 2.15 angular
        super().__init__("Camera_node")
        # Создание подписчика на данные о положении
        self._pose_sub = self.create_subscription(Odometry, '/odom', self.pose_callback, 10)
        # Создание подписчика на изображение с камеры
        self._robot_Ccamera_sub = self.create_subscription(Image, "/color/image", self.camera_callback, 3)
        self.state_pub = self.create_publisher(String,"/state",10)
        self.timer = self.create_timer(0.2, self.go_forward)
        self.detect_timer = self.create_timer(0.033, self.detect_callback)
        self.depth_camera = self.create_subscription(Image,"/depth/image",self.depth_callback,10)
        self._cv_bridge = CvBridge()
        self.last_position = Point()
        self.total_distance = 0.0
        self.is_first_message = True
        self.sign_type = 1
        self.distance_to_obstacle = 99999 
        self.frame = None
        self.d_frame = None
        self.detector = SignDetector(path_to_signs_imgs="signs_images", debug_mode=False)


    def pose_callback(self, data):
        position = data.pose.pose.position

        if self.is_first_message:
            self.last_position = position
            self.is_first_message = False
            return

        delta_x = position.x - self.last_position.x
        delta_y = position.y - self.last_position.y
        distance = math.sqrt(delta_x**2 + delta_y**2)
        self.total_distance += distance
        self.last_position = position

        #print(f"Текущее расстояние: {self.total_distance} метров")
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        
        # Преобразование радиан в градусы, если нужно
        yaw_degree = yaw * 180 / math.pi

        #print(f"Угол поворота: {yaw_degree} градусов")    
    def camera_callback(self,data):
        try:
            cv_image = self._cv_bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
            self.frame = cv_image
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

        # Обработка изображения глубины для нахождения препятствий
        self.distance_to_obstacle = self.process_depth_image(cv_image)
        print(self.distance_to_obstacle)

    def process_depth_image(self, cv_image):
        height, width = cv_image.shape
        if width!=0:
            center_x, center_y = width // 2, height // 2
            num_points = 40
            points = []
            for i in range(num_points):
                angle = 2 * np.pi * i / num_points
                x = int(center_x + 0.8 * center_x * np.cos(angle))
                y = int(center_y + 0.8 * center_y * np.sin(angle))
                points.append((x, y))
            average_point = (int(np.mean([point[0] for point in points])), int(np.mean([point[1] for point in points])))
            pixel_values = [cv_image[point[1], point[0]] for point in points]
            average_pixel_value = np.mean(pixel_values, axis=0)
        print(average_pixel_value)
        return average_pixel_value
    
    
    def go_forward(self):
        if self.sign_type != -1:
            msg = String()
            msg.data = str(self.sign_type)
            self.state_pub.publish(msg)
            self.sign_type=-1


    def detect_callback(self):
        if self.frame is not None and self.d_frame is not None:
            detect_res = self.detector(self.frame,self.d_frame)
            self.sign_type = detect_res[1]

def main():
    rclpy.init()
    FTN = Camera_node()
    rclpy.spin(FTN)
    FTN.destroy_node()
    rclpy.shutdown()