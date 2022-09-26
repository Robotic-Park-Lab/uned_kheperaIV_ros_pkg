from cmath import sqrt
from rosgraph_msgs.msg import Clock
import rclpy
import os
import math
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from math import atan2, cos, sin, degrees, radians, pi
import sys
import tf_transformations
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped



class KheperaWebotsDriver:
    def init(self, webots_node, properties):
        
        self.robot = webots_node.robot
        timestep = int(self.robot.getBasicTimeStep())

        ## Initialize motors
        self.motor_left = self.robot.getDevice("left wheel motor")
        self.motor_left.setPosition(float('inf'))
        self.motor_left.setVelocity(0)
        self.motor_right = self.robot.getDevice("right wheel motor")
        self.motor_right.setPosition(float('inf'))
        self.motor_right.setVelocity(0)

        self.target_twist = Twist()

        ## Init Pose
        self.global_x = 0.0
        self.global_y = 0.0
        self.global_yaw = 0.0
        self.init_pose = False

        self.target_pose = Pose()
        self.target_pose.position.x = 0.0
        self.target_pose.position.y= 0.0

        ## Initialize Sensors
        self.gyro = self.robot.getDevice("gyro")
        self.gyro.enable(timestep)
        self.gps = self.robot.getDevice("gps")
        self.gps.enable(timestep)
        self.imu = self.robot.getDevice("inertial unit")
        self.imu.enable(timestep)

        self.range_left = self.robot.getDevice("left ultrasonic sensor")
        self.range_left.enable(timestep)
        self.range_frontleft = self.robot.getDevice("front left ultrasonic sensor")
        self.range_frontleft.enable(timestep)
        self.range_front = self.robot.getDevice("front ultrasonic sensor")
        self.range_front.enable(timestep)
        self.range_frontright = self.robot.getDevice("front right ultrasonic sensor")
        self.range_frontright.enable(timestep)
        self.range_right = self.robot.getDevice("right ultrasonic sensor")
        self.range_right.enable(timestep)

        ## Intialize Variables
        self.past_time = self.robot.getTime()

        ## ROS2 Environment
        name_value = os.environ['WEBOTS_ROBOT_NAME']
        rclpy.init(args=None)
        self.node = rclpy.create_node(name_value+'_driver')
        self.node.get_logger().info('Webots_Node::inicialize() ok. %s' % (str(name_value)))
        self.node.create_subscription(Twist, name_value+'/cmd_vel', self.cmd_vel_callback, 1)
        self.node.create_subscription(Pose, name_value+'/goal_pose', self.goal_pose_callback, 1)
        self.laser_publisher = self.node.create_publisher(LaserScan, name_value+'/scan', 10)
        self.msg_laser = LaserScan()
        self.node.create_timer(1.0/30.0, self.publish_laserscan_data)

        self.tfbr = TransformBroadcaster(self.node)
    
        

    def publish_laserscan_data(self):
        left_range = self.range_left.getValue()
        frontleft_range = self.range_frontleft.getValue()
        front_range = self.range_front.getValue()
        frontright_range = self.range_frontright.getValue()
        right_range = self.range_right.getValue()
        max_range = 2.5
        if left_range > max_range:
            left_range = float("inf")
        if frontleft_range > max_range:
            frontleft_range = float("inf")
        if front_range > max_range:
            front_range = float("inf")
        if frontright_range > max_range:
            frontright_range = float("inf")
        if right_range > max_range:
            right_range = float("inf")
 
        self.msg_laser = LaserScan()
        self.msg_laser.header.stamp = Time(seconds=self.robot.getTime()).to_msg()
        self.msg_laser.header.frame_id = 'base_link'
        self.msg_laser.range_min = 0.25
        self.msg_laser.range_max = max_range
        self.msg_laser.ranges = [left_range, frontleft_range, front_range, frontright_range, right_range]
        self.msg_laser.angle_min = -0.5 * pi
        self.msg_laser.angle_max =  0.5 * pi
        self.msg_laser.angle_increment = pi/4
        self.laser_publisher.publish(self.msg_laser)

    def cmd_vel_callback(self, twist):
        self.target_twist = twist

    def goal_pose_callback(self, pose):
        self.target_pose = pose

    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)

        dt = self.robot.getTime() - self.past_time

        # Get pose
        self.global_x = self.gps.getValues()[0]
        self.global_y = self.gps.getValues()[1]
        self.global_yaw = self.imu.getRollPitchYaw()[2]
        
        if not self.init_pose:
            self.target_pose.position.x = self.global_x
            self.target_pose.position.y= self.global_y
            self.init_pose = True
        
        # TO-DO: Position Controller
        distance_error = sqrt(pow(self.target_pose.position.x-self.global_x,2)+pow(self.target_pose.position.y-self.global_y,2))
        angle_error = -self.global_yaw+atan2(self.target_pose.position.y-self.global_y,self.target_pose.position.x-self.global_x)
        
        if distance_error.real>0.01:
            self.target_twist.linear.x = distance_error.real*cos(angle_error)
        else:
            self.target_twist.linear.x = 0.0
        self.target_twist.angular.z = sin(angle_error)*10

        # Velocity [rad/s]
        self.motor_left.setVelocity((self.target_twist.linear.x/0.042-0.10540*self.target_twist.angular.z))
        self.motor_right.setVelocity((self.target_twist.linear.x/0.042+0.10540*self.target_twist.angular.z))

        # TO-DO: Introudcir Modelo Cinemático Diferencial Directo -> /odom

        self.past_time = self.robot.getTime()

        # DEBUG
        # self.node.get_logger().info('Get Pose3D: X:%f Y:%f yaw:%f' % (self.gps.getValues()[0],self.gps.getValues()[1],self.imu.getRollPitchYaw()[2]))
        # self.node.get_logger().info('Target Pose3D: X:%f Y:%f' % (self.target_pose.position.x,self.target_pose.position.y))
        # self.node.get_logger().info('Distance:%f Angle:%f' % (distance_error.real,angle_error))