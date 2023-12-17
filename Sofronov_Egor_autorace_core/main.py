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
from sensor_msgs.msg import LaserScan
OFFSET_BTW_CENTERS = 5 # 4 5 
"""
Уровни дебага
0: нет его
1: выводим данные с перспективы
2: выводим данные с камеры
3: выводим маски слоев
4: машинка не поедет никуда
"""
DEBUG_LEVEL : Literal[0, 1, 2, 3, 4] = 0


class Follow_Trace_Node(Node):
    """
    Follow_Trace_Node is a class that represents a node for following a trace using camera data and robot movement control.

    Args:
        linear_speed: The linear speed of the robot (default: 0.0).
        angular_speed: The angular speed of the robot (default: 1.0).
        linear_slow_speed: The slow linear speed of the robot (default: None).

    Explanation:
        This class initializes the necessary subscribers, publishers, and variables for controlling the robot's movement and processing camera data. It provides methods for turning the robot, moving it forward, performing a specific pattern of movements, and handling different states.

    Raises:
        None.
    """

    def __init__(self, linear_speed = 0.2, angular_speed=1.1, linear_slow_speed=None):# 0.2 linear 2.15 angular
        """
        __init__ is the constructor method of the Follow_Trace_Node class.

        Args:
            linear_speed: The linear speed of the robot (default: 0.0).
            angular_speed: The angular speed of the robot (default: 1.0).
            linear_slow_speed: The slow linear speed of the robot (default: None).

        Explanation:
            This method initializes the Follow_Trace_Node object by setting up subscribers, publishers, and variables for controlling the robot's movement and processing camera data. It also initializes various parameters and states used for tracking the robot's position and performing specific actions.

        Raises:
            None.
        """
        super().__init__("Follow_Trace_Node")
        # Создание подписчика на данные о положении
        self._pose_sub = self.create_subscription(Odometry, '/odom', self.pose_callback, 1)
        # Создание подписчика на изображение с камеры
        self._robot_Ccamera_sub = self.create_subscription(Image, "/color/image", self.camera_callback, 3)
        self._robot_depth_camera_sub = self.create_subscription(Image, "/depth/image", self.depth_callback, 3)
        # Создание издателя для управления движением робота
        self._robot_cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.misson_pub = self.create_publisher(String, "/mission", 10)
        self.camera_sub = self.create_subscription(String,"/state",self.state_callback,10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 1)
        # Инициализация объекта CvBridge для конвертации изображений ROS в OpenCV
        self._cv_bridge = CvBridge()
        self.timer = self.create_timer(0.1,self.timer_callback)
        self._linear_speed = linear_speed
        self.angular_speed = angular_speed
        self._linear_slow_speed = linear_slow_speed
        self.last_position = Point()
        self.lidar_msg = LaserScan()
        self.total_distance = 0.0
        self.is_first_message = True
        self.state = 8
        self.ped = 0
        '''
            Уровни состояния
            0 - стоим на месте горит красный
            1 - едем по обеим полосам
            2 - едем по белой
            3 - едем по желтой
            4 - змейка
            5 - парковка
            5.5 - едем до парковки
            6 - пешеход
            6.5 ждем пешехода
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
        self.target_yaw = -999999
        self.do_forward=0
        self.zmeika_state=-1
        self.start_distance= -999999
        self.parking_dst = -999999
        self.on_mission = 0
        self.parking_direction = -1
        self.parking_state= -1
    def lidar_callback(self,msg):
        self.lidar_msg = msg
    def depth_callback(self,msg):
        pass
        # try:
        #     # Convert ROS Image message to OpenCV image
        #     depth_image = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        # except Exception as e:
        #     self.get_logger().error(f'Error converting depth image: {str(e)}')
        #     return
        # print(depth_image.shape)
        # # Process depth image as needed (e.g., display or save)
        # cv2.imshow('Depth Image', depth_image)
        # cv2.waitKey(1)  # Update display
    def turn_robot(self,publisher, angle):
        """
        turn_robot publishes a Twist message with a specified angular velocity to turn the robot.

        Args:
            publisher: The publisher to publish the Twist message.
            angle: The angular velocity: positive for turning left, negative for turning right.

        Returns:
            None.

        Raises:
            None.
        """

        if ( self.target_yaw==-999999):
            #start_angle = self.yaw_degree
            if angle >0:
                self.target_yaw = (self.yaw_degree + 90) % 360
            else: # Поворот вправо
                self.target_yaw = (self.yaw_degree - 90) % 360
            # if angle >0:
            #     self.target_yaw = math.atan2(self.yaw_degree,90)
            # else:
            #     self.target_yaw = math.atan2(self.yaw_degree,-90)
            print(f"Целевой угол {self.target_yaw}")
            print(f"Текущий угол {self.yaw_degree}")
            twist = Twist()
            twist.angular.z = angle # Угловая скорость: положительная для поворота налево, отрицательная для поворота направо
            publisher.publish(twist)
            self.do_rotate=1
    def mission_upd(self,msg_s):
        msg = String()
        msg.data=msg_s
        self.misson_pub.publish(msg)
    def move_robot(self,publisher, distance):
        """
        move_robot publishes a Twist message with a specified linear velocity to move the robot forward.

        Args:
            publisher: The publisher to publish the Twist message.
            distance: The distance to move the robot forward.

        Returns:
            None.

        Raises:
            None.
        """

        
        if (self.start_distance==-999999):
            self.start_distance = self.total_distance
            twist = Twist()
            twist.linear.x = distance # Линейная скорость
            publisher.publish(twist)
            self.do_forward=1   
        
    def zmeika(self):
        """
        zmeika performs a sequence of movements and turns to navigate the robot in a specific pattern.

        Args:
            self: The instance of the class.

        Returns:
            None.

        Raises:
            None.
        """
        scan_data = self.lidar_msg.ranges
        # print(f'перед {min(scan_data[:10] + scan_data[349:359])}')
        # print(f'право {min(scan_data[260:300])}')
        # print(f"сзади { min(scan_data[120:240])}")
        # print(f"сектор справа спереди {min(scan_data[300:359])}")
        if self.state==1:
            if min(scan_data[:10] + scan_data[349:359]) < 0.3:
                    self.state+=0.5
                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self._robot_cmd_vel_pub.publish(twist)
        elif (self.zmeika_state==0):
            self.turn_robot(self._robot_cmd_vel_pub, 0.5) # Угловая скорость для поворота налево
        elif (self.zmeika_state==1):
            # Движение на 25 метров
            self.move_robot(self._robot_cmd_vel_pub, 0.2) # Линейная скорость для движения вперед
        elif (self.zmeika_state==2):
            # Поворот направо на 90 градусов
            self.turn_robot(self._robot_cmd_vel_pub, -0.5) # Угловая скорость для поворота направо
        elif (self.zmeika_state==3):
            # Движение на еще 50 метров
            self.move_robot(self._robot_cmd_vel_pub, 0.2) # Линейная скорость для движения вперед
        elif self.zmeika_state==4:
            self.turn_robot(self._robot_cmd_vel_pub, -0.5) # Угловая скорость для поворота направо
        elif self.zmeika_state==5:
            self.move_robot(self._robot_cmd_vel_pub, 0.2) # Линейная скорость для движения вперед
        elif self.zmeika_state==6:
            self.turn_robot(self._robot_cmd_vel_pub, 0.5) # Угловая скорость для поворота налево
        elif self.zmeika_state==7:
            # self.turn_robot(self._robot_cmd_vel_pub, 0.5) # Угловая скорость для поворота направо
            self.state=1
            self.on_mission==0
            self.mission_upd("0")
        # elif self.zmeika_state==8:
            
            
    def state_callback(self,data):
        """
        state_callback is a callback function that updates the state based on the received data.

        Args:
            self: The instance of the class.
            data: The data received.

        Returns:
            None.

        Raises:
            None.
        """
        tmp_state = int(data.data)
        print(f"Полученный знак: {tmp_state}")
        if (self.state==8 and tmp_state==7):
            self.state=tmp_state
        if (self.state==7 and tmp_state==2):
            self.state=tmp_state
        elif self.state == 2 and tmp_state in {3, 5}:
            self.state=tmp_state
        elif self.state in [3.5, 5.5] and tmp_state == 1:
            self.state=tmp_state
        elif self.state in [1,1.5] and tmp_state==4:
            self.state=10
        elif self.state== 10 and tmp_state== 0:
            self.state=tmp_state
        elif self.state== 0 and tmp_state== 6:
            self.state=tmp_state
    def parking(self):
        scan_data = self.lidar_msg.ranges
        #print(f'перед {min(scan_data[:10] + scan_data[349:359])}')
        #print(f'право {min(scan_data[260:300])}')
        #print(f"сектор слева спереди { min(scan_data[30:90])}")
        #print(f"сектор справа спереди {min(scan_data[300:339])}")
        if min(scan_data[30:90]) <= 0.30 and self.state==4 and abs(self.parking_dst-self.total_distance)>=0.25:
            print("Начинаю парковку")
            self.state+=0.5
            self.parking_state=1
            self.parking_direction=1
            self.do_then_parking_find(0.22,-0.9,True)
        elif min(scan_data[270:339]) <=0.30 and self.state==4 and abs(self.parking_dst-self.total_distance)>=0.25:
            print("Начинаю парковку")
            self.state+=0.5
            self.parking_state=1
            self.do_then_parking_find(0.22,0.9,True)
        elif self.parking_state==2:
            if self.parking_direction==1:
                self.do_then_parking_find(-0.15,-0.9,True)
            else:
                self.do_then_parking_find(-0.15,0.9,True)
            self.parking_state+=1
        elif self.parking_state==4:
            if self.parking_direction==1:
                self.do_then_parking_find(0.2,0.1,False)
            else:
                self.do_then_parking_find(0.2,-0.1,False)
            self.parking_state+=1
            
        
    def do_then_parking_find(self,linear, angular_speed, need_90):
        if angular_speed > 0 and need_90:
            self.target_yaw = (self.yaw_degree + 90) % 360
        elif angular_speed < 0 and need_90: # Поворот вправо
            self.target_yaw = (self.yaw_degree - 90) % 360
        elif need_90==False:
            self.start_distance = self.total_distance
        #self.target_yaw = (self.yaw_degree + 90) % 360
        #print(f"Целевой угол {self.target_yaw}")
        twist = Twist()
        twist.angular.z = angular_speed # Угловая скорость: положительная для поворота налево, отрицательная для поворота направо
        twist.linear.x=linear
        self._robot_cmd_vel_pub.publish(twist)
        self.do_rotate=1
        self.do_forward=1
        #self.turn_robot(self._robot_cmd_vel_pub, angular_speed)
        
        
    def pedestant(self):
        max_speed = 0.2
        scan_data = self.lidar_msg.ranges
        
        # Количество точек прошлого раза будем хранить в self.ped
        # Параметры diapason и threshold зависят от расстояния до перекрестка, чем ближе, тем меньше
        diapason = 22
        degree = min(self.yaw_degree, diapason) # учесть угол относительно перехода
        front = scan_data[359-diapason - math.floor(degree):359] + scan_data[:diapason - math.ceil(degree)]
        print(f"Угол поворота: {self.yaw_degree} градусов")
        # Подсчитать количество точек, где значение < 0.7
        close_points = [i for i in front if i < 0.7]
        dots_num = len(close_points)

        # Если объект удаляется от нас, порог нужен меньше
        threshold = 6 if dots_num > self.ped else 7

        self.get_logger().info('Dots: ' + str(dots_num))
        self.get_logger().info('Threshold: ' + str(threshold))
        if len(close_points) >= threshold:
            self.get_logger().info('Obstacle founded in distance: ' + str(min(close_points)))
            self._linear_speed = 0.0
        # Закомментить для подбора параметров
        else:  
            self._linear_speed = max_speed
            self.state = 9
        self.ped = dots_num
        
        
        
    def timer_callback(self):
        """
        timer_callback is a callback function that performs different actions based on the current state.

        Args:
            self: The instance of the class.

        Returns:
            None.

        Raises:
            None.
        """

        #print(self.state)
        if (self.state==99):
            return
        if(self.state==1 or self.state==1.5):
            if (self.on_mission==0):
                self.on_mission=1
                self.mission_upd("1")
            if self.zmeika_state==-1:
                self.zmeika_state+=1
            self.zmeika()
        elif(self.state==4 or self.state==4.5 or self.state == 109):
            if (self.on_mission==0):
                self.on_mission=1
                self.mission_upd("1")
            if self.parking_dst==-999999:
                self.parking_dst = self.total_distance
                print(f"Начало парковки расстояние: {self.total_distance} метров")
            self.parking()
        elif(self.state==0):
            if (self.on_mission==0):
                self.on_mission=1
                self.mission_upd("1")
            self.pedestant()
        elif ((self.state==3 or self.state==5) and self.on_mission==0):
            print(f"начало перекрестка: {self.total_distance}")
            self.start_distance=self.total_distance
            self.on_mission=1
            self.mission_upd("1")
    # Обратный вызов для получения данных о положении
    def pose_callback(self, data):
        """
        pose_callback is a callback function that processes the pose data received.

        Args:
            self: The instance of the class.
            data: The pose data received.

        Returns:
            None.

        Raises:
            None.
        """

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
        self.yaw_degree = yaw * 180 / math.pi
        if self.yaw_degree < 0:
            self.yaw_degree += 360

        #print(f"Угол поворота: {self.yaw_degree} градусов")
        if self.do_rotate==1 and abs(self.target_yaw-self.yaw_degree)<=1 and self.zmeika_state in [0, 2, 4, 6, 7]:
            twist = Twist()
            print(f"Угол поворота в конце: {self.yaw_degree} градусов")
            twist.angular.z = 0.0 # Остановить поворот
            self._robot_cmd_vel_pub.publish(twist)
            self.do_rotate=0
            self.zmeika_state+=1
            self.target_yaw=-999999
        if (
            self.do_forward == 1
            and self.zmeika_state in [1, 5]
            and abs(self.start_distance - self.total_distance) >= 0.25
        ):
            twist = Twist()
            twist.linear.x = 0.0 # Остановить поворот
            self._robot_cmd_vel_pub.publish(twist)
            self.do_forward=0
            self.zmeika_state+=1
            self.start_distance=-999999
        if self.do_forward==1 and self.zmeika_state==3 and abs(self.start_distance-self.total_distance)>=0.45:
            twist = Twist()
            twist.linear.x = 0.0 # Остановить поворот
            self._robot_cmd_vel_pub.publish(twist)
            self.do_forward=0
            self.zmeika_state+=1
            self.start_distance=-999999
        if (
            self.on_mission == 1
            and self.state in [3, 5]
            and abs(self.start_distance - self.total_distance) >= 1.7
        ):
            self.state+=0.5
            #print(self.total_distance)
            self.start_distance=-999999
            self.on_mission=0
            self.mission_upd("0")
        if self.on_mission==1 and self.parking_state in [1,3] and self.do_rotate==1 and abs(self.target_yaw-self.yaw_degree)<=2:
            print(f"Угол поворота в конце: {self.yaw_degree} градусов")
            twist = Twist()
            twist.linear.x = 0.0 # Остановить поворот
            twist.angular.z = 0.0
            self._robot_cmd_vel_pub.publish(twist)
            
            self.mission_upd("0")
            self.start_distance=-999999
            self.target_yaw=-999999
            self.do_forward=0
            self.do_rotate=0
            time.sleep(4)
            self.parking_state+=1
        if self.do_forward==1 and self.parking_state==5 and abs(self.start_distance-self.total_distance)>=0.38:
            twist = Twist()
            twist.linear.x = 0.0 # Остановить поворот
            self._robot_cmd_vel_pub.publish(twist)
            self.do_forward=0
            self.start_distance=self.total_distance
            self.on_mission=0
            self.parking_state=-1
            self.state=4.8
    # Получение угла поворота из данных о положении
    def get_angle(self):
        """
        get_angle returns the angle calculated from the quaternion representation of the pose.

        Args:
            self: The instance of the class.

        Returns:
            The calculated angle.

        Raises:
            None.
        """

        quaternion = (self.pose.pose.pose.orientation.x, self.pose.pose.pose.orientation.y, self.pose.pose.pose.orientation.z,self.pose.pose.pose.orientation.w) 
        euler = euler_from_quaternion(quaternion) 
        return euler[2]
        
    # Преобразование перспективы изображения
    def Perspective_warp(self, cvImg):
        """
        Perspective_warp applies a perspective transformation to the input image.

        Args:
            cvImg: The input image to be transformed.

        Returns:
            The transformed image.

        Raises:
            None.

        Examples:
            cvImg = cv2.imread('image.jpg')
            transformed_img = Perspective_warp(cvImg)
        """
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
        """
        yellow_line calculates the position of the yellow line in the perspective image.

        Args:
            self: The instance of the class.
            perspectiveImg: The perspective-transformed image.
            middle_h: The middle row of the image.

        Returns:
            The position of the yellow line.

        Raises:
            None.
        """
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
        """
        white_line calculates the position of the white line in the perspective image.

        Args:
            self: The instance of the class.
            perspectiveImg: The perspective-transformed image.
            middle_h: The middle row of the image.

        Returns:
            None.

        Raises:
            None.
        """
        h, w, _ = perspectiveImg.shape
        perspectiveImg= perspectiveImg[:,w//2:, :]
        tmp = w//2
        white_mask = cv2.inRange(perspectiveImg, (250, 250, 250), (255, 255, 255))

        middle_row = white_mask[middle_h]
        try:
            first_white = np.int32(np.where(middle_row == 255))[0][0]
            self.white_prevs.append(first_white)
        except: 
            first_white = sum(self.white_prevs)//len(self.white_prevs)
            
        return (first_white+ tmp)
    
    # Расчет новой угловой скорости с использованием PID-регулятора
    def PID(self, target):
        """
        PID calculates the new angular speed using a PID controller.

        Args:
            self: The instance of the class.
            target: The target value for the controller.

        Returns:
            The calculated angular speed.

        Raises:
            None.
        """
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
        """
            camera_callback is a callback function that processes the camera data and controls the robot's movement.

            Args:
                self: The instance of the class.
                msg: The camera data received.

            Returns:
                None.

            Raises:
                None.
            """
        if self.state not in (1.5,109,4.5,4.6,4.7,8): 
            
            #if self.state != 1:
            #    return
            emptyTwist = Twist()
            emptyTwist.linear.x = self._linear_speed

            cvImg = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding=msg.encoding)
            cvImg = cv2.cvtColor(cvImg, cv2.COLOR_RGB2BGR)

            perspective = self.Perspective_warp(cvImg)
            h, w, _ = perspective.shape
            hLine=int(h*(3/4))
            if self.state in [5,5.5,1,9,10]:
            # Получаем координаты края желтой линии и белой
                endYellow = 180 #self.yellow_line(perspective,hLine) # 180
                startWhite = self.white_line(perspective,hLine) #610 #self.white_line(perspective,hLine)
            elif self.state in [3,3.5,4,4.8]:
                endYellow = self.yellow_line(perspective,hLine) #self.yellow_line(perspective,hLine) # 180
                startWhite = 610 #610 #self.white_line(perspective,hLine)
            else:
                endYellow = self.yellow_line(perspective,hLine) #self.yellow_line(perspective,hLine) # 180
                startWhite = self.white_line(perspective,hLine) #610 #self.white_line(perspective,hLine)


            middle_btw_lines = (startWhite + endYellow) // 2

            center_crds = (w // 2, hLine)
            lines_center_crds = (middle_btw_lines, hLine)


            if abs(center_crds[0] - lines_center_crds[0]) > OFFSET_BTW_CENTERS:
                direction = center_crds[0] - lines_center_crds[0]
                angle = math.atan2(direction,215)
                angular_v = self.PID(angle)
                adaptive_speed = abs(self._linear_speed * (1 - min(abs(angular_v) / self.angular_speed, 1)))
                emptyTwist.linear.x = adaptive_speed
                emptyTwist.angular.z = angular_v
                #self.get_logger().info(f"Angle Speed: {angular_v} Linear: {adaptive_speed}")
                #self.get_logger().info("----------------------------")
            else:
                emptyTwist.linear.x = self._linear_speed
                emptyTwist.angular.z = 0.0 #angular_v
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
    """
    main is the entry point of the program.

    Args:
        None.

    Returns:
        None.

    Raises:
        None.
    """
    rclpy.init()
    FTN = Follow_Trace_Node()
    rclpy.spin(FTN)
    FTN.destroy_node()
    rclpy.shutdown()
