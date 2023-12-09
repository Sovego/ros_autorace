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
from std_msgs.msg import String
import cv2
import math
import numpy as np

OFFSET_BTW_CENTERS = 3.5 # 4 5 
"""
Уровни дебага
0: нет его
1: выводим данные с перспективы
2: выводим данные с камеры
3: выводим маски слоев
4: машинка не поедет никуда
"""
DEBUG_LEVEL : Literal[0, 1, 2, 3, 4] = 2

# Начальные значения параметров
SOME_THRESHOLD = 0.1
MAX_DISTANCE_BETWEEN_CENTERS = 10

# Если потерял линию то стараться повернуть к ней?
# или наоборот держаться той линии что осталась, но на каком-то растоянии? (среднем за предыдущие время от этой линии)
# Что-то сделать со скоростями, PID регулятор?
class Follow_Trace_Node(Node):

    def __init__(self, linear_speed = 0.0, angular_speed=1.0, linear_slow_speed=None): # 0.2 linear 2.15 angular
        super().__init__("Follow_Trace_Node")
        # Создание подписчика на данные о положении
        self._pose_sub = self.create_subscription(Odometry, '/odom', self.pose_callback, 10)
        # Создание подписчика на изображение с камеры
        self._robot_Ccamera_sub = self.create_subscription(Image, "/color/image", self.camera_callback, 3)
        # Создание издателя для управления движением робота
        self._robot_cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.camera_sub = self.create_subscription(String,"/state",self.state_callback,10)
        # Инициализация объекта CvBridge для конвертации изображений ROS в OpenCV
        self._cv_bridge = CvBridge()
        self.timer = self.create_timer(0.5,self.timer_callback)
        self._linear_speed = linear_speed
        self.angular_speed = angular_speed
        self._linear_slow_speed = linear_slow_speed
        self.last_position = Point()
        self.total_distance = 0.0
        self.is_first_message = True
        self.state = 1
        '''
            Уровни состояния
            0 - стоим на месте горит красный
            1 - едем по обеим полосам
            2 - едем по белой
            3 - едем по желтой
            4 - змейка
            5 - парковка
            6 - пешеход
            7 - тоннель
            8 - перекресток
        '''
        self.yaw_degree = 0
        # Поправленное имя переменной
        self._direction_prevs = deque(maxlen=10)

        if self._linear_slow_speed is None:
            self._linear_slow_speed = self._linear_speed / 5

        self.yellow_prevs = deque(maxlen=10)
        self.white_prevs  = deque(maxlen=10)
        self.yellow_prevs.append(0)
        self.white_prevs.append(0)
        self.do_rotate=0
        self.pose = Odometry()
        self.Kp = 3.0 # 0.0025 # 1.5 # 2.0 при 0.225 и огранке 1.0
        self.Ki = 0.1 # 0.2 # 0.1 при скорости 0.1
        self.Kd = 0.25 # 0.007 # 0.25
        self.dt = 1
        self.old_e = 0
        self.E = 0
        self.start_angle = -999999
        self.do_forward=0
        self.zmeika_state=-1
        self.start_distance= -999999
    def turn_robot(self,publisher, angle):
        twist = Twist()
        twist.angular.z = angle # Угловая скорость: положительная для поворота налево, отрицательная для поворота направо
        if (self.start_angle==-999999):
            self.start_angle = self.yaw_degree
        start_time = self.get_clock().now()
        publisher.publish(twist)
        self.do_rotate=1

    def move_robot(self,publisher, distance):
        twist = Twist()
        if (self.start_distance==-999999):
            self.start_distance = self.total_distance
        twist.linear.x = distance # Линейная скорость
        start_time = self.get_clock().now()
        publisher.publish(twist)
        self.do_forward=1   
        
    def zmeika(self):

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self._robot_cmd_vel_pub.publish(twist)
        cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        
            # Поворот налево на 90 градусов
        if(self.zmeika_state==0):
            self.turn_robot(cmd_vel_publisher, 0.25) # Угловая скорость для поворота налево
        elif(self.zmeika_state==1):
            # Движение на 25 метров
            self.move_robot(cmd_vel_publisher, 0.1) # Линейная скорость для движения вперед
        elif(self.zmeika_state==2):
            # Поворот направо на 90 градусов
            self.turn_robot(cmd_vel_publisher, -0.25) # Угловая скорость для поворота направо
        elif(self.zmeika_state==3):
            # Движение на еще 50 метров
            self.move_robot(cmd_vel_publisher, 0.1) # Линейная скорость для движения вперед
        elif self.zmeika_state==4:
            self.turn_robot(cmd_vel_publisher, -0.25) # Угловая скорость для поворота направо
        elif self.zmeika_state==5:
            self.move_robot(cmd_vel_publisher, 0.1) # Линейная скорость для движения вперед
        elif self.zmeika_state==6:
            self.turn_robot(cmd_vel_publisher, 0.25) # Угловая скорость для поворота налево
        elif self.zmeika_state==7:
            self.state=2    
    def state_callback(self,data):
        self.state=int(data.data)
    def timer_callback(self):
        print(self.state)
        if (self.state==99):
            return
        if(self.state==4):
            if self.zmeika_state==-1:
                self.zmeika_state+=1
            self.zmeika()
            
        elif(self.state==5):
            parking()
            self.state=1
        elif(self.state==6):
            pedestant()
            self.state=6
        elif(self.state==8):
            cross()
    # Обратный вызов для получения данных о положении
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

        print(f"Текущее расстояние: {self.total_distance} метров")
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        
        # Преобразование радиан в градусы, если нужно
        self.yaw_degree = yaw * 180 / math.pi

        #print(f"Угол поворота: {self.yaw_degree} градусов")
        if (self.do_rotate==1):
            if abs(self.start_angle-self.yaw_degree)>=88:
                twist = Twist()
                twist.angular.z = 0.0 # Остановить поворот
                self._robot_cmd_vel_pub.publish(twist)
                self.do_rotate=0
                self.zmeika_state+=1
                self.start_angle=-999999
        if (self.do_forward==1 and (self.zmeika_state==1 or self.zmeika_state==5 )):
            if abs(self.start_distance-self.total_distance)>=0.25:
                twist = Twist()
                twist.linear.x = 0.0 # Остановить поворот
                self._robot_cmd_vel_pub.publish(twist)
                self.do_forward=0
                self.zmeika_state+=1
                self.start_distance=-999999
        if (self.do_forward==1 and self.zmeika_state==3):
            print(self.zmeika_state)
            if abs(self.start_distance-self.total_distance)>=0.40:
                twist = Twist()
                twist.linear.x = 0.0 # Остановить поворот
                self._robot_cmd_vel_pub.publish(twist)
                self.do_forward=0
                self.zmeika_state+=1
                self.start_distance=-999999
    # Получение угла поворота из данных о положении
    def get_angle(self):
        quaternion = (self.pose.pose.pose.orientation.x, self.pose.pose.pose.orientation.y, self.pose.pose.pose.orientation.z,self.pose.pose.pose.orientation.w) 
        euler = euler_from_quaternion(quaternion) 
        return euler[2]
        
    # Преобразование перспективы изображения
    def Perspective_warp(self, cvImg):
        h, w, _ = cvImg.shape
        top_x_offset = 50

        pts1 = np.float32([[0, 480], [w, 480], [top_x_offset, 300], [w-top_x_offset, 300]])
        result_img_width = np.int32(abs(pts1[0][0] - pts1[1][0])) 
        result_img_height = np.int32(abs(pts1[0][1] - pts1[2][0])) 

        pts2 = np.float32([[0, 0], [result_img_width,0], [0, result_img_height], [result_img_width, result_img_height]])

        M = cv2.getPerspectiveTransform(pts1, pts2)
        dst = cv2.warpPerspective(cvImg, M, (result_img_width, result_img_height))

        if(DEBUG_LEVEL >= 2):
            for pt in pts1:
                cvImg = cv2.rectangle(cvImg, np.int32(pt), np.int32(pt), (255, 0, 0), 5)
            cv2.imshow("orig", cvImg)
        
        return cv2.flip(dst, 0)
    

    def yellow_line(self, perspectiveImg,middle_h):
        h, w, _ = perspectiveImg.shape
        perspectiveImg= perspectiveImg[:,:w//2,:]
        yellow_mask = cv2.inRange(perspectiveImg, (0, 240, 255), (0, 255, 255))
        yellow_mask = cv2.dilate(yellow_mask, np.ones((2, 2)), iterations=4)
        middle_row = yellow_mask[middle_h]
        try:
            first_notYellow = np.int32(np.where(middle_row == 255))[0][-1]
            self.yellow_prevs.append(first_notYellow)
        except: 
            first_notYellow = sum(self.yellow_prevs)//len(self.yellow_prevs)

        return (first_notYellow)


    def white_line(self, perspectiveImg,middle_h):
        h, w, _ = perspectiveImg.shape
        #perspectiveImg= perspectiveImg[:,w//2:, :]
        #tmp = w//2
        white_mask = cv2.inRange(perspectiveImg, (250, 250, 250), (255, 255, 255))

        middle_row = white_mask[middle_h]
        try:
            first_white = np.int32(np.where(middle_row == 255))[0][0]
            self.white_prevs.append(first_white)
        except: 
            first_white = sum(self.white_prevs)//len(self.white_prevs)
            
        return (first_white)
    
    # Расчет новой угловой скорости с использованием PID-регулятора
    def PID(self, target):
        err = target
        e = np.arctan2(np.sin(err), np.cos(err))
        e_P = e
        e_I = self.E + e
        e_D = e - self.old_e
        
        angular_speed = self.Kp*e_P + self.Ki*e_I + self.Kd*e_D

        angular_speed = np.arctan2(np.sin(angular_speed), np.cos(angular_speed))

        self.E = self.E + e
        self.old_e = e
        return angular_speed #max(angular_speed, -self.angular_speed) if angular_speed < 0 else min(angular_speed, self.angular_speed) 

    # Обратный вызов для обработки данных с камеры
    def camera_callback(self, msg: Image):
        if (self.state==1):
            emptyTwist = Twist()
            emptyTwist.linear.x = self._linear_speed

            cvImg = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding=msg.encoding)
            cvImg = cv2.cvtColor(cvImg, cv2.COLOR_RGB2BGR)

            perspective = self.Perspective_warp(cvImg)
            h, w, _ = perspective.shape
            hLine=int(h*(3/4))
            # Получаем координаты края желтой линии и белой
            endYellow = self.yellow_line(perspective,hLine) #self._find_yellow_line(perspective,hLine) # 180
            startWhite = self.white_line(perspective,hLine) #610 #self.white_line(perspective,hLine)

            

            middle_btw_lines = (startWhite + endYellow) // 2

            center_crds = (w // 2, hLine)
            lines_center_crds = (middle_btw_lines, hLine)

    
            if abs(center_crds[0] - lines_center_crds[0]) > OFFSET_BTW_CENTERS:
                direction = center_crds[0] - lines_center_crds[0] 
                angle = math.atan2(direction,215)
                angular_v = self.PID(angle)
                emptyTwist.angular.z = 0.0 #angular_v
                adaptive_speed = abs(self._linear_speed * (1 - min(abs(angular_v) / self.angular_speed, 1)))
                emptyTwist.linear.x = adaptive_speed
                #self.get_logger().info(f"Angle Speed: {angular_v} Linear: {adaptive_speed}")
                #self.get_logger().info("----------------------------")
                
            else:

                emptyTwist.linear.x = self._linear_speed
                emptyTwist.angular.z = 0.0

            if DEBUG_LEVEL >= 1:
                # # рисуем точки
                # persective_drawed = cv2.rectangle(perspective, center_crds, center_crds, (0, 255, 0), 5)  # Центр изо
                # persective_drawed = cv2.rectangle(persective_drawed, lines_center_crds, lines_center_crds, (0, 0, 255), 5)  # центр точки между линиями
                # cv2.imshow("img", persective_drawed)
                # cv2.waitKey(1)

                # рисуем точки
                persective_drawed = cv2.rectangle(perspective, center_crds, center_crds, (0, 255, 0), 10)  # Центр изо
                persective_drawed = cv2.rectangle(persective_drawed, lines_center_crds, lines_center_crds, (0, 0, 255), 10)  # центр точки между линиями

                point = (10, 10)
                persective_drawed = cv2.circle(persective_drawed, point, 10, (0, 255, 0), -1)


                persective_drawed = cv2.rectangle(persective_drawed, lines_center_crds, lines_center_crds, (0, 0, 255), 10)  # центр точки между линиями

                # Выделяем желтую линию красным цветом
                persective_drawed = cv2.line(persective_drawed, (endYellow, hLine), (endYellow + 10, hLine), (0, 0, 255), 10)

                # Выделяем белую линию синим цветом
                persective_drawed = cv2.line(persective_drawed, (startWhite, hLine), (startWhite + 10, hLine), (255, 0, 0), 10)

                cv2.imshow("img", persective_drawed)
                cv2.waitKey(1)
            self._robot_cmd_vel_pub.publish(emptyTwist)
def main():
    rclpy.init()
    FTN = Follow_Trace_Node()
    rclpy.spin(FTN)
    FTN.destroy_node()
    rclpy.shutdown()
