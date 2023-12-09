import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

class LineFollower(Node):
    def __init__(self):
        # Инициализация узла ROS
        super().__init__('line_follower')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image,"/color/image_projected_compensated", self.camera_callback,1)
        self.cmd_vel_pub = self.create_publisher(Twist,"/cmd_vel", 10)
        # Параметры PID
        self.kp = 0.0025
        self.ki = 0.1
        self.kd = 0.007
        self.prev_error = 0
        self.integral = 0
        # Скорости
        self.linear_speed = 0.1  # Линейная скорость
        self.angular_speed_limit = 2.0  # Максимальная угловая скорость

    def camera_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        #transformed_image = self.apply_perspective_transform(cv_image)
        processed_image, error = self.process_image(cv_image)
        self.control_robot(error)
        
        cv2.imshow("Image window", processed_image)
        cv2.waitKey(3)

    


    def process_image(self,frame):
        # Преобразование в градации серого
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Улучшение контрастности с помощью CLAHE
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        contrast_enhanced = clahe.apply(gray)

        # Адаптивный порог для выделения белой линии
        white_mask = cv2.adaptiveThreshold(contrast_enhanced, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                        cv2.THRESH_BINARY, 11, 2)

        # Преобразование цветового пространства в HSV для выделения желтой линии
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        yellow_lower = np.array([20, 100, 100])
        yellow_upper = np.array([30, 255, 255])
        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)

        # Морфологическое преобразование для улучшения маски
        kernel = np.ones((5, 5), np.uint8)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)

        # Объединение масок
        combined_mask = cv2.bitwise_or(yellow_mask, white_mask)

        # Детектирование краев (опционально)
        edges = cv2.Canny(combined_mask, 50, 150)

        # Поиск контуров (опционально)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Вычисление ошибки на основе положения контуров
        error = 0
        if contours:
            # Найти самый большой контур
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                error = cx - frame.shape[1]//2

        return combined_mask, error




    def control_robot(self, error):
        # PID-регулятор
        proportional = error
        self.integral += error
        derivative = error - self.prev_error

        angular_z = self.kp * error + self.kd * (error - self.prev_error)
        angular = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)

        # Создание сообщения Twist
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = angular  # Знак зависит от ориентации камеры/робота
        self.get_logger().info(f"{twist}")
        # Публикация сообщения
        self.cmd_vel_pub.publish(twist)
        self.prev_error = error

def main():
    rclpy.init()
    lf = LineFollower()
    try:
        rclpy.spin(lf)
    except rclpy.ROSInterruptException:
        pass
