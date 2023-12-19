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

# TODO: Починить парковку в левый карман и включить парковку в состояниях
class Line(Node):
    """
    Line is a class that represents a node for following a trace using camera data and robot movement control.

    Args:
        linear_speed: The linear speed of the robot (default: 0.0).
        angular_speed: The angular speed of the robot (default: 1.0).
        linear_slow_speed: The slow linear speed of the robot (default: None).

    Explanation:
        This class initializes the necessary subscribers, publishers, and variables for controlling the robot's movement and processing camera data. It provides methods for turning the robot, moving it forward, performing a specific pattern of movements, and handling different states.

    Raises:
        None.
    """
    def __init__(self, linear_speed = 0.15, angular_speed=1.0, linear_slow_speed=None):# 0.2 linear 2.15 angular
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
        super().__init__("Line")
        # Create a subscriber for pose data
        self._pose_sub = self.create_subscription(Odometry, '/odom', self.pose_callback, 1)
        # Create a subscriber for the camera image
        self._robot_Ccamera_sub = self.create_subscription(Image, "/color/image", self.camera_callback, 3)
        self._robot_depth_camera_sub = self.create_subscription(Image, "/depth/image", self.depth_callback, 3)
        # Create a publisher for controlling robot movement
        self._robot_cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.misson_pub = self.create_publisher(String, "/mission", 10)
        self.camera_sub = self.create_subscription(String,"/state",self.state_callback,10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 1)
        self.finish_pub=self.create_publisher(String, "/robot_finish", 10)
        # Initialize CvBridge object to convert ROS images to OpenCV
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
            State levels
            0 - stand still, red light is on
            1 - drive on both lanes
            2 - drive on white
            3 - drive on yellow
            4 - snake
            5 - parking
            5.5 - drive to parking
            6 - pedestrian
            6.5 wait for pedestrian
            7 - tunnel
            8 - intersection
        '''
        self.yaw_degree = 0
        self._direction_prevs = deque(maxlen=10)

        if self._linear_slow_speed is None:
            self._linear_slow_speed = self._linear_speed / 5

        self.yellow_prevs = deque(maxlen=10)
        self.white_prevs  = deque(maxlen=10)
        self.yellow_prevs.append(0)
        self.white_prevs.append(0)
        self.do_rotate=0
        self.pose = Odometry()
        self.Kp = 2.5 
        self.Ki = 0.1 
        self.Kd = 0.25 
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
        self.go_from_parking = 0
    def lidar_callback(self,msg):
        """
        This method is a callback that is called when a new lidar message is received.

        Parameters:
        msg (LaserScan): The lidar data received from the sensor.

        Returns:
        None
        """
        self.lidar_msg = msg
    def depth_callback(self,msg):
        pass
        
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
            if angle >0:
                # If the angle is positive, rotate left by 90 degrees
                self.target_yaw = (self.yaw_degree + 90) % 360
            else: 
                # If the angle is negative or zero, rotate right by 90 degrees
                self.target_yaw = (self.yaw_degree - 90) % 360

            # Log the target and current angles
            self.get_logger().info(f"Target angle {self.target_yaw}")
            self.get_logger().info(f"Current angle {self.yaw_degree}")
            twist = Twist()
            # Set the angular velocity: positive for left turn, negative for right turn
            twist.angular.z = angle 
            publisher.publish(twist)
            # Set the flag to indicate that the robot is in rotation
            self.do_rotate=1
    def mission_upd(self, msg_s):
        """
        This method updates the mission status and publishes it.

        Parameters:
        msg_s (str): The message string representing the mission status.

        Returns:
        None
        """
        # Create a new String message
        msg = String()
        # Assign the input string to the data field of the message
        msg.data = msg_s
        # Log the mission status that is being sent
        self.get_logger().info(f"Mission status sent: {msg_s}")
        # Publish the message
        self.misson_pub.publish(msg)
    def move_robot(self, publisher, distance):
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
        # Check if the start distance has not been set
        if (self.start_distance == -999999):
            # Set the start distance to the current total distance
            self.start_distance = self.total_distance
            # Create a new Twist message
            twist = Twist()
            # Set the linear velocity to the specified distance
            twist.linear.x = distance # Linear velocity
            # Publish the Twist message
            publisher.publish(twist)
            # Set the flag to indicate that the robot is moving forward
            self.do_forward = 1
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
        # Get the lidar scan data
        scan_data = self.lidar_msg.ranges

        # If the robot is in state 1 and an obstacle is detected within 0.27 units
        if self.state==1:
            if min(scan_data[:10] + scan_data[349:359]) < 0.27:
                # Transition to the next state
                self.state+=0.5
                # Stop the robot
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self._robot_cmd_vel_pub.publish(twist)
        # If the robot is in the first state of the pattern
        elif (self.zmeika_state==0):
            # Turn the robot to the right
            self.turn_robot(self._robot_cmd_vel_pub, 0.3) 
        # If the robot is in the second state of the pattern
        elif (self.zmeika_state==1):
            # Move the robot forward
            self.move_robot(self._robot_cmd_vel_pub, 0.15) 
        # If the robot is in the third state of the pattern
        elif (self.zmeika_state==2):
            # Turn the robot to the left
            self.turn_robot(self._robot_cmd_vel_pub, -0.3) 
        # If the robot is in the fourth state of the pattern
        elif (self.zmeika_state==3):
            # Move the robot forward
            self.move_robot(self._robot_cmd_vel_pub, 0.15) 
        # If the robot is in the fifth state of the pattern
        elif self.zmeika_state==4:
            # Turn the robot to the left
            self.turn_robot(self._robot_cmd_vel_pub, -0.3) 
        # If the robot is in the sixth state of the pattern
        elif self.zmeika_state==5:
            # Move the robot forward
            self.move_robot(self._robot_cmd_vel_pub, 0.15) 
        # If the robot is in the seventh state of the pattern
        elif self.zmeika_state==6:
            # Turn the robot to the right
            self.turn_robot(self._robot_cmd_vel_pub, 0.3) 
        # If the robot has completed the pattern
        elif self.zmeika_state==7:
            # Transition to the next state
            self.state=1.8
            # Indicate that the robot is not on a mission
            self.on_mission=0
            # Update the mission status
            self.mission_upd("0")

            
            
    def state_callback(self, data):
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
        # Convert the received data to an integer
        tmp_state = int(data.data)

        # If the current state is 8 and the received state is 7, update the state
        if (self.state == 8 and tmp_state == 7):
            self.state = tmp_state

        # If the current state is 7 and the received state is 2, update the state
        if (self.state == 7 and tmp_state == 2):
            self.state = tmp_state

        # If the current state is 2 and the received state is 3 or 5, update the state
        elif self.state == 2 and tmp_state in {3, 5}:
            self.state = tmp_state

        # If the current state is 3.5 or 5.5 and the received state is 1, update the state
        elif self.state in [3.5, 5.5] and tmp_state == 1:
            self.state = tmp_state

        # If the current state is 1.8 and the received state is 4, update the state
        elif self.state == 1.8 and tmp_state == 4:
            self.state = tmp_state

        # If the current state is 4.9 and the received state is 0, update the state
        # TODO: change to parking
        elif self.state == 4.9 and tmp_state == 0:
            self.state = tmp_state

        # If the current state is 9 and the received state is 6, update the state
        elif self.state == 9 and tmp_state == 6:
            self.state = tmp_state
            
            

    def parking(self):
        """
        This method handles the parking logic of the vehicle. It uses lidar data to find a parking spot and 
        maneuvers the vehicle into the spot. The method checks for parking spots on both sides of the vehicle 
        and adjusts the parking direction accordingly. It also handles different stages of parking.

        The method uses the following instance variables:
        - self.lidar_msg.ranges: The lidar data used to find parking spots.
        - self.state: The current state of the vehicle.
        - self.parking_dst: The target parking distance.
        - self.total_distance: The total distance travelled by the vehicle.
        - self.parking_state: The current state of the parking process.
        - self.parking_direction: The direction of the parking spot (1 for right, -1 for left).

        The method does not take any parameters and does not return any value.
        """
        # Get the lidar scan data
        scan_data = self.lidar_msg.ranges

        # If a parking spot is detected on the right and the vehicle is in state 4 and not yet at the target parking distance
        if min(scan_data[30:90]) <= 0.30 and self.state==4 and abs(self.parking_dst-self.total_distance)>=0.25:
            # Log the start of parking
            self.get_logger().info("Starting parking")
            # Transition to the next state
            self.state+=0.5
            # Set the parking state to 1
            self.parking_state=1
            # Set the parking direction to right
            self.parking_direction=1
            # Start the parking maneuver
            self.do_then_parking_find(0.165,-0.6,True)
        # If a parking spot is detected on the left and the vehicle is in state 4 and not yet at the target parking distance
        elif min(scan_data[270:339]) <=0.35 and self.state==4 and abs(self.parking_dst-self.total_distance)>=0.25:
            # Log the start of parking
            self.get_logger().info("Starting parking")
            # Transition to the next state
            self.state+=0.5
            # Set the parking state to 1
            self.parking_state=1
            # Set the parking direction to left
            self.do_then_parking_find(0.15,0.6,True)
        # If the vehicle is in the second state of the parking process
        elif self.parking_state==2:
            # If the parking direction is right
            if self.parking_direction==1:
                # Continue the parking maneuver
                self.do_then_parking_find(-0.09,-0.35,True)
            else:
                # Continue the parking maneuver
                self.do_then_parking_find(-0.09,0.35,True)
            # Transition to the next parking state
            self.parking_state+=1
        # If the vehicle is in the fourth state of the parking process
        elif self.parking_state==4:
            # If the parking direction is right
            if self.parking_direction==1:
                # Continue the parking maneuver
                self.do_then_parking_find(0.2,0.0,False)
                # Transition to the next parking state
                self.parking_state+=1
            else:
                # Continue the parking maneuver
                self.do_then_parking_find(0.2,0.1,False)
                # Transition to the next parking state
                self.parking_state+=1.5
            
        
    def do_then_parking_find(self, linear, angular_speed, need_90):
        """
        This method handles the movement of the vehicle during the parking finding process. It sets the target yaw 
        based on the current yaw and the direction of rotation. It also sets the start distance for forward movement. 
        It then publishes a Twist message to control the vehicle's movement.

        Parameters:
        linear (float): The linear speed of the vehicle.
        angular_speed (float): The angular speed of the vehicle. Positive for left turn, negative for right turn.
        need_90 (bool): A flag indicating whether a 90 degree rotation is needed.

        The method uses the following instance variables:
        - self.yaw_degree: The current yaw of the vehicle in degrees.
        - self.target_yaw: The target yaw of the vehicle in degrees.
        - self.total_distance: The total distance travelled by the vehicle.
        - self.start_distance: The start distance for forward movement.
        - self._robot_cmd_vel_pub: The ROS publisher for the vehicle's command velocity.
        - self.do_rotate: A flag indicating whether the vehicle should rotate.
        - self.do_forward: A flag indicating whether the vehicle should move forward.

        The method does not return any value.
        """
        # If a left turn is needed and a 90 degree rotation is needed
        if angular_speed > 0 and need_90:
            # Set the target yaw to the current yaw plus 90 degrees (mod 360 to keep it within 0-359)
            self.target_yaw = (self.yaw_degree + 90) % 360
        # If a right turn is needed and a 90 degree rotation is needed
        elif angular_speed < 0 and need_90:
            # Set the target yaw to the current yaw minus 90 degrees (mod 360 to keep it within 0-359)
            self.target_yaw = (self.yaw_degree - 90) % 360
        # If a 90 degree rotation is not needed
        elif need_90 == False:
            # Set the start distance to the current total distance
            self.start_distance = self.total_distance

        # Create a new Twist message
        twist = Twist()
        # Set the angular velocity to the specified angular speed
        twist.angular.z = angular_speed  # Angular velocity: positive for left turn, negative for right turn
        # Set the linear velocity to the specified linear speed
        twist.linear.x = linear
        # Publish the Twist message
        self._robot_cmd_vel_pub.publish(twist)
        # Set the flag to indicate that the vehicle should rotate
        self.do_rotate = 1
        # Set the flag to indicate that the vehicle should move forward
        self.do_forward = 1

        
        
    def pedestant(self):
        """
        This method handles the detection of pedestrians or obstacles in front of the vehicle using lidar data. 
        It calculates the number of close points in the lidar data and adjusts the vehicle's speed accordingly. 
        If an obstacle is detected, the vehicle's speed is set to 0. Otherwise, the vehicle's speed is set to 
        its maximum speed and its state is updated.

        The method uses the following instance variables:
        - self.lidar_msg.ranges: The lidar data used to detect obstacles.
        - self.yaw_degree: The current yaw of the vehicle in degrees.
        - self._linear_speed: The linear speed of the vehicle.
        - self.state: The current state of the vehicle.
        - self.on_mission: A flag indicating whether the vehicle is on a mission.
        - self.ped: The number of close points in the lidar data from the previous check.

        The method does not take any parameters and does not return any value.
        """
        max_speed = 0.2
        scan_data = self.lidar_msg.ranges

        # We will store the number of points from the last time in self.ped
        # The parameters diapason and threshold depend on the distance to the intersection, the closer, the less
        diapason = 22

        # Convert the angle so that it is in the range from -180 to 180 degrees
        yaw_degree_adjusted = (self.yaw_degree + 180) % 360 - 180

        degree = min(yaw_degree_adjusted, diapason) # consider the angle relative to the transition

        front = scan_data[359-diapason - math.floor(degree):359] + scan_data[:diapason - math.ceil(degree)]
        # Count the number of points where the value is < 0.7
        close_points = [i for i in front if i < 0.7]
        dots_num = len(close_points)

        # If the object is moving away from us, we need a lower threshold
        threshold = 6 if dots_num > self.ped else 7

        self.get_logger().info('Dots: ' + str(dots_num))
        self.get_logger().info('Threshold: ' + str(threshold))
        self.get_logger().info(f"Current state: {self.state}")
        if len(close_points) >= threshold:
            self.get_logger().info('Obstacle detected at distance: ' + str(min(close_points)))
            self._linear_speed = 0.0
        else:  
            self._linear_speed = max_speed
            self.state = 9
            self.on_mission=0
            self.mission_upd("0")
        self.ped = dots_num
        
        
        
    def timer_callback(self):
        """
        timer_callback is a callback function that performs different actions based on the current state of the vehicle. 
        It handles the following states:
        - State 1 or 1.5: The vehicle is in the "snake" maneuver.
        - State 4, 4.5, or 109: The vehicle is in the parking process.
        - State 0: The vehicle is in the pedestrian detection mode.
        - State 3 or 5: The vehicle is at an intersection.

        The method uses the following instance variables:
        - self.state: The current state of the vehicle.
        - self.on_mission: A flag indicating whether the vehicle is on a mission.
        - self.zmeika_state: The current state of the "snake" maneuver.
        - self.parking_dst: The target parking distance.
        - self.total_distance: The total distance travelled by the vehicle.
        - self.start_distance: The start distance for forward movement.

        The method does not take any parameters and does not return any value.
        """
        # If the vehicle is in state 99, do nothing and return
        if (self.state == 99):
            return
        # If the vehicle is in state 1 or 1.5 (snake maneuver)
        if (self.state == 1 or self.state == 1.5):
            # If the vehicle is not on a mission
            if (self.on_mission == 0):
                # Log the current state
                self.get_logger().info(f"Current state: {self.state}")
                # Set the flag to indicate that the vehicle is on a mission
                self.on_mission = 1
                # Update the mission status
                self.mission_upd("1")
            # If the snake maneuver has not started
            if self.zmeika_state == -1:
                # Start the snake maneuver
                self.zmeika_state += 1
            # Perform the snake maneuver
            self.zmeika()
        # If the vehicle is in state 4, 4.5, or 109 (parking process)
        elif (self.state == 4 or self.state == 4.5 or self.state == 109):
            # If the vehicle is not on a mission
            if (self.on_mission == 0):
                # Log the current state
                self.get_logger().info(f"Current state: {self.state}")
                # Set the flag to indicate that the vehicle is on a mission
                self.on_mission = 1
                # Update the mission status
                self.mission_upd("1")
            # If the parking process has not started
            if self.parking_dst == -999999:
                # Set the start distance for parking
                self.parking_dst = self.total_distance
                # Log the start of parking
                self.get_logger().info(f"Start of parking distance: {self.total_distance} meters")
            # Perform the parking process
            self.parking()
        # If the vehicle is in state 0 (pedestrian detection mode)
        elif (self.state == 0):
            # If the vehicle is not on a mission
            if (self.on_mission == 0):
                # Set the flag to indicate that the vehicle is on a mission
                self.on_mission = 1
                # Update the mission status
                self.mission_upd("1")
            # Perform pedestrian detection
            self.pedestant()
        # If the vehicle is in state 3 or 5 (at an intersection) and not on a mission
        elif ((self.state == 3 or self.state == 5) and self.on_mission == 0):
            # Log the current state
            self.get_logger().info(f"Current state: {self.state}")
            # Log the start of the intersection
            self.get_logger().info(f"Start of intersection: {self.total_distance}")
            # Set the start distance for the intersection
            self.start_distance = self.total_distance
            # Set the flag to indicate that the vehicle is on a mission
            self.on_mission = 1
            # Update the mission status
            self.mission_upd("1")

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

        # Extract the position from the pose data
        position = data.pose.pose.position

        # If this is the first message, store the position and return
        if self.is_first_message:
            self.last_position = position
            self.is_first_message = False
            return

        # Calculate the distance traveled since the last message
        delta_x = position.x - self.last_position.x
        delta_y = position.y - self.last_position.y
        distance = math.sqrt(delta_x**2 + delta_y**2)
        self.total_distance += distance
        self.last_position = position
        #self.get_logger().info(f"Distance: {self.total_distance}")
        # Extract the orientation from the pose data and convert it to Euler angles
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        # Convert the yaw angle from radians to degrees
        self.yaw_degree = yaw * 180 / math.pi
        if self.yaw_degree < 0:
            self.yaw_degree += 360

        # If the robot is in the process of rotating and has reached the target yaw, stop rotating
        if self.do_rotate==1 and abs(self.target_yaw-self.yaw_degree)<=1 and self.zmeika_state in [0, 2, 4, 6, 7]:
            twist = Twist()
            twist.angular.z = 0.0 # Stop rotation
            self._robot_cmd_vel_pub.publish(twist)
            self.do_rotate=0
            self.zmeika_state+=1
            self.target_yaw=-999999

        # If the robot is in the process of moving forward and has reached the target distance, stop moving
        if (
            self.do_forward == 1
            and self.zmeika_state in [1, 5]
            and abs(self.start_distance - self.total_distance) >= 0.26
        ):
            twist = Twist()
            twist.linear.x = 0.0 # Stop forward movement
            self._robot_cmd_vel_pub.publish(twist)
            self.do_forward=0
            self.zmeika_state+=1
            self.start_distance=-999999
        # If the robot is on a mission and has reached the target distance, update the state and stop the mission
        if (
            self.on_mission == 1
            and self.state in [3, 5]
            and abs(self.start_distance - self.total_distance) >= 1.7
        ):
            self.state+=0.5
            self.start_distance=-999999
            self.on_mission=0
            self.mission_upd("0")
        # If the robot is moving forward and in state 3, and has reached the target distance
        if self.do_forward==1 and self.zmeika_state==3 and abs(self.start_distance-self.total_distance)>=0.45:
            twist = Twist()
            twist.linear.x = 0.0 # Stop rotation
            self._robot_cmd_vel_pub.publish(twist)
            self.do_forward=0
            self.zmeika_state+=1
            self.start_distance=-999999

        # If the robot is on a mission and in state 3 or 5, and has reached the target distance
        if (
            self.on_mission == 1
            and self.state in [3, 5]
            and abs(self.start_distance - self.total_distance) >= 1.7
        ):
            self.state+=0.5
            self.start_distance=-999999
            self.on_mission=0
            self.mission_upd("0")

        # If the robot is on a mission and in parking state 1 or 3, and is rotating, and has reached the target yaw
        if self.on_mission==1 and self.parking_state in [1,3] and self.do_rotate==1 and abs(self.target_yaw-self.yaw_degree)<=2:
            self.get_logger().info(f"Final rotation angle: {self.yaw_degree} degrees")
            twist = Twist()
            twist.linear.x = 0.0 # Stop rotation
            twist.angular.z = 0.0
            self._robot_cmd_vel_pub.publish(twist)
            self.start_distance=-999999
            self.target_yaw=-999999
            self.do_forward=0
            self.do_rotate=0
            time.sleep(1)
            self.parking_state+=1

        # If the robot is moving forward and in parking state 5, and has reached the target distance
        if self.do_forward==1 and self.parking_state==5 and abs(self.start_distance-self.total_distance)>=0.38:
            twist = Twist()
            twist.linear.x = 0.0 # Stop rotation
            self._robot_cmd_vel_pub.publish(twist)
            self.do_forward=0
            self.start_distance=self.total_distance
            self.go_from_parking=1
            self.parking_state=-1
            self.state=4.8

        # If the robot is moving forward and in parking state 5.5, and has reached the target distance
        if self.do_forward==1 and self.parking_state==5.5 and abs(self.start_distance-self.total_distance)>=0.38:
            twist = Twist()
            twist.linear.x = 0.0 # Stop rotation
            self._robot_cmd_vel_pub.publish(twist)
            self.do_forward=0
            self.start_distance=self.total_distance
            self.go_from_parking=1
            self.parking_state=-1
            self.state=4.8

        # If the robot is moving from parking and has reached the target distance
        if self.go_from_parking==1 and abs(self.start_distance-self.total_distance)>=0.85:
            self.state=4.9
            self.go_from_parking=0
            self.start_distance=-999999
            self.on_mission=0
            self.mission_upd("0")
        
        if self.total_distance>=17.9:
            self.state=8
            twist = Twist()
            twist.linear.x = 0.0 
            twist.angular.z = 0.0 # Stop rotation
            self._robot_cmd_vel_pub.publish(twist)
            msg = String()
            msg.data="BAGodelnya"
            self.finish_pub.publish(msg)
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

        # Get the height and width of the image
        h, w, _ = cvImg.shape

        # Define the offset for the top x-coordinate
        top_x_offset = 50

        # Define the source points for the perspective transformation
        # These points define a trapezoid shape
        pts1 = np.float32([[0, 480], [w, 480], [top_x_offset, 300], [w-top_x_offset, 300]])

        # Calculate the width and height of the resulting image
        result_img_width = np.int32(abs(pts1[0][0] - pts1[1][0])) 
        result_img_height = np.int32(abs(pts1[0][1] - pts1[2][0])) 

        # Define the destination points for the perspective transformation
        # These points define a rectangle shape
        pts2 = np.float32([[0, 0], [result_img_width,0], [0, result_img_height], [result_img_width, result_img_height]])

        # Compute the perspective transformation matrix
        M = cv2.getPerspectiveTransform(pts1, pts2)

        # Apply the perspective transformation to the image
        dst = cv2.warpPerspective(cvImg, M, (result_img_width, result_img_height))

        # If the debug level is 2 or higher, draw the source points on the original image and display it
        if(DEBUG_LEVEL >= 2):
            for pt in pts1:
                cvImg = cv2.rectangle(cvImg, np.int32(pt), np.int32(pt), (255, 0, 0), 5)
            cv2.imshow("orig", cvImg)
        
        # Flip the transformed image vertically and return it
        return cv2.flip(dst, 0)
    

    def yellow_line(self, perspectiveImg, middle_h):
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

        # Get the height and width of the image
        h, w, _ = perspectiveImg.shape

        # Crop the image to only include the left half
        perspectiveImg = perspectiveImg[:, :w//2, :]

        # Create a mask that isolates the yellow pixels in the image
        yellow_mask = cv2.inRange(perspectiveImg, (0, 240, 255), (0, 255, 255))

        # Dilate the mask to fill in gaps between yellow pixels
        yellow_mask = cv2.dilate(yellow_mask, np.ones((2, 2)), iterations=4)

        # Extract the middle row from the mask
        middle_row = yellow_mask[middle_h]

        try:
            # Find the last yellow pixel in the middle row
            first_notYellow = np.int32(np.where(middle_row == 255))[0][-1]

            # Add this position to the list of previous yellow line positions
            self.yellow_prevs.append(first_notYellow)
        except:
            # If no yellow pixel was found, estimate the position of the yellow line
            # based on the average of the previous positions
            first_notYellow = sum(self.yellow_prevs) // len(self.yellow_prevs)

        # Return the position of the yellow line
        return first_notYellow


    def white_line(self, perspectiveImg, middle_h):
        """
        white_line calculates the position of the white line in the perspective image.

        Args:
            self: The instance of the class.
            perspectiveImg: The perspective-transformed image.
            middle_h: The middle row of the image.

        Returns:
            The position of the white line.

        Raises:
            None.
        """

        # Get the height and width of the image
        h, w, _ = perspectiveImg.shape

        # Crop the image to only include the right half
        perspectiveImg = perspectiveImg[:, w//2:, :]

        # Store the width of the cropped image
        tmp = w//2

        # Create a mask that isolates the white pixels in the image
        white_mask = cv2.inRange(perspectiveImg, (250, 250, 250), (255, 255, 255))

        # Extract the middle row from the mask
        middle_row = white_mask[middle_h]

        try:
            # Find the first white pixel in the middle row
            first_white = np.int32(np.where(middle_row == 255))[0][0]

            # Add this position to the list of previous white line positions
            self.white_prevs.append(first_white)
        except:
            # If no white pixel was found, estimate the position of the white line
            # based on the average of the previous positions
            first_white = sum(self.white_prevs) // len(self.white_prevs)

        # Return the position of the white line, adjusted for the cropping of the image
        return (first_white + tmp)
    
    
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

        # Calculate the error between the target and the current state
        err = target

        # Normalize the error to the range [-pi, pi]
        e = np.arctan2(np.sin(err), np.cos(err))

        # Calculate the proportional, integral, and derivative terms
        e_P = e
        e_I = self.E + e
        e_D = e - self.old_e

        # Calculate the new angular speed using the PID controller formula
        angular_speed = self.Kp*e_P + self.Ki*e_I + self.Kd*e_D

        # Normalize the angular speed to the range [-pi, pi]
        angular_speed = np.arctan2(np.sin(angular_speed), np.cos(angular_speed))

        # Update the integral and previous error terms
        self.E = self.E + e
        self.old_e = e

        # Return the calculated angular speed
        return angular_speed

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
        # Check if the current state is not in the specified list
        if self.state not in (1.5,4.5,4.6,4.7,8): 

            # Initialize a Twist message to control the robot's movement
            emptyTwist = Twist()
            emptyTwist.linear.x = self._linear_speed

            # Convert the camera data to an OpenCV image
            cvImg = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding=msg.encoding)
            cvImg = cv2.cvtColor(cvImg, cv2.COLOR_RGB2BGR)

            # Apply a perspective transform to the image
            perspective = self.Perspective_warp(cvImg)
            h, w, _ = perspective.shape
            hLine=int(h*(3/4))

            # Depending on the current state, calculate the positions of the yellow and white lines
            if self.state in [5,5.5,1,9,4.9,1.8]:
                endYellow = 180
                startWhite = self.white_line(perspective,hLine)
            elif self.state in [3,3.5,4,4.8]:
                endYellow = self.yellow_line(perspective,hLine)
                startWhite = 610
            else:
                endYellow = self.yellow_line(perspective,hLine)
                startWhite = self.white_line(perspective,hLine)

            # Calculate the middle point between the yellow and white lines
            middle_btw_lines = (startWhite + endYellow) // 2

            # Define the coordinates of the center of the image and the center between the lines
            center_crds = (w // 2, hLine)
            lines_center_crds = (middle_btw_lines, hLine)

            # If the distance between the two centers is greater than a threshold, calculate a new angular speed
            if abs(center_crds[0] - lines_center_crds[0]) > OFFSET_BTW_CENTERS:
                direction = center_crds[0] - lines_center_crds[0]
                angle = math.atan2(direction,215)
                angular_v = self.PID(angle)
                adaptive_speed = abs(self._linear_speed * (1 - min(abs(angular_v) / self.angular_speed, 1)))
                emptyTwist.linear.x = adaptive_speed
                emptyTwist.angular.z = angular_v
            else:
                emptyTwist.linear.x = self._linear_speed
                emptyTwist.angular.z = 0.0

            # If the debug level is set to 1 or higher, draw the centers and lines on the image
            if DEBUG_LEVEL >= 1:
                persective_drawed = cv2.rectangle(perspective, center_crds, center_crds, (0, 255, 0), 10)  # Center of image
                persective_drawed = cv2.rectangle(persective_drawed, lines_center_crds, lines_center_crds, (0, 0, 255), 10)  # Center between lines

                point = (10, 10)
                persective_drawed = cv2.circle(persective_drawed, point, 10, (0, 255, 0), -1)

                persective_drawed = cv2.rectangle(persective_drawed, lines_center_crds, lines_center_crds, (0, 0, 255), 10)  # Center between lines

                # Highlight the yellow line in red
                #persective_drawed = cv2.line(persective_drawed, (endYellow, hLine), (endYellow + 10, hLine), (0, 0, 255), 10)

                # Highlight the white line in blue
                #persective_drawed = cv2.line(persective_drawed, (startWhite, hLine), (startWhite + 10, hLine), (255, 0, 0), 10)

                cv2.imshow("img", persective_drawed)
                cv2.waitKey(1)

            # Publish the Twist message to control the robot's movement
            self._robot_cmd_vel_pub.publish(emptyTwist)
def main():
    """
    main is the entry point of the program. It initializes the ROS client library,
    creates an instance of the Line class, spins the ROS node to process callbacks,
    and then cleans up by destroying the node and shutting down the client library.

    Args:
        None.

    Returns:
        None.

    Raises:
        None.
    """

    # Initialize the ROS client library
    rclpy.init()

    # Create an instance of the Line class
    line = Line()

    # Spin the ROS node to process callbacks
    rclpy.spin(line)

    # Destroy the node to free up resources
    line.destroy_node()

    # Shutdown the ROS client library
    rclpy.shutdown()