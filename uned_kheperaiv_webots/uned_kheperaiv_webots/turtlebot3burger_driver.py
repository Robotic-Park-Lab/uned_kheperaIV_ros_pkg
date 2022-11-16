from rosgraph_msgs.msg import Clock
import rclpy
import os
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import Odometry

from math import atan2, cos, sin, sqrt, degrees, radians, pi
import sys
import tf_transformations
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class PIDController():
    def __init__(self, Kp, Ki, Kd, Td, Nd, UpperLimit, LowerLimit, ai, co):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Td = Td
        self.Nd = Nd
        self.UpperLimit = UpperLimit
        self.LowerLimit = LowerLimit
        self.integral = 0
        self.derivative = 0
        self.error = [0.0, 0.0]
        self.trigger_ai = ai
        self.trigger_co = co
        self.trigger_last_signal = 0.0
        self.noise = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.past_time = 0.0
        self.last_value = 0.0
        self.th = 0.0

    def update(self, dt):
        P = self.Kp * self.error[0]
        self.integral = self.integral + self.Ki*self.error[1]*dt
        self.derivative = (self.Td/(self.Td+self.Nd+dt))*self.derivative+(self.Kd*self.Nd/(self.Td+self.Nd*dt))*(self.error[0]-self.error[1])
        out = P + self.integral + self.derivative
        
        if not self.UpperLimit==0.0:
            # out_i = out
            if out>self.UpperLimit:
                out = self.UpperLimit
            if out<self.LowerLimit:
                out = self.LowerLimit

            # self.integral = self.integral - (out-out_i) * sqrt(self.Kp/self.Ki)
        
        self.error[1] = self.error[0]

        self.last_value = out
        
        return out

    def eval_threshold(self, error):
        # a
        a = self.trigger_ai * abs(error)
        if a > self.trigger_ai:
            a = self.trigger_ai

        # Threshold
        self.th = self.trigger_co + a
        self.inc = abs(abs(error) - self.trigger_last_signal) 
        # Delta Error
        if (self.inc >= abs(self.th)):
            self.trigger_last_signal = abs(error)
            return True

        return False

class TurtlebotWebotsDriver:
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

        ## Initialize Pose
        self.global_x = 0.0
        self.global_y = 0.0
        self.global_yaw = 0.0
        self.init_pose = False

        self.goal_pose = Pose()
        self.goal_pose.position.x = 0.0
        self.goal_pose.position.y= 0.0

        ## Initialize Sensors
        self.past_time = self.robot.getTime()
        self.gyro = self.robot.getDevice("gyro")
        self.gyro.enable(timestep)
        self.gps = self.robot.getDevice("gps")
        self.gps.enable(timestep)
        self.imu = self.robot.getDevice("inertial_unit")
        self.imu.enable(timestep)
        self.lidar = self.robot.getDevice("LDS-01")
        self.lidar.enable(timestep)

        ## Intialize Variables
        ## Intialize Controllers
        self.continuous = False
        # Position
        self.linear_controller = PIDController(1.0, 0.0, 0.0, 0.0, 100, 1.0, -1.0, 0.05, 0.01)
        # Angle
        self.angular_controller = PIDController(2.0, 0.0, 0.0, 0.0, 100, 0.0, 0.0, 0.1, 0.1)
        
        ## ROS2 Environment
        self.name_value = os.environ['WEBOTS_ROBOT_NAME']
        rclpy.init(args=None)
        self.node = rclpy.create_node(self.name_value+'_driver')
        self.node.get_logger().info('Webots_Node::inicialize() ok. %s' % (str(self.name_value)))
        # Subscription
        self.node.create_subscription(Twist, self.name_value+'/cmd_vel', self.cmd_vel_callback, 1)
        self.node.create_subscription(Pose, self.name_value+'/goal_pose', self.goal_pose_callback, 1)
        # Publisher
        self.pose_publisher_ = self.node.create_publisher(Pose, self.name_value+'/local_pose', 10)
        self.laser_publisher_ = self.node.create_publisher(LaserScan, self.name_value+'/scan', 10)
        
        self.tfbr = TransformBroadcaster(self.node)

        self.node.create_timer(0.2, self.publish_laserscan_data)
    
    def publish_laserscan_data(self):
        ranges = self.lidar.getLayerRangeImage(0)
        if ranges:
            self.msg_laser = LaserScan()
            self.msg_laser.header.stamp = Time(seconds=self.robot.getTime()).to_msg()
            self.msg_laser.header.frame_id = self.name_value
            self.msg_laser.angle_min = -self.lidar.getFov()/2.0
            self.msg_laser.angle_max = self.lidar.getFov()/2.0
            self.msg_laser.angle_increment = self.lidar.getFov()/self.lidar.getHorizontalResolution()
            self.msg_laser.time_increment = float(self.lidar.getSamplingPeriod()) / (1000.0 * self.lidar.getHorizontalResolution())
            self.msg_laser.scan_time = float(self.lidar.getSamplingPeriod()) / 1000.0
            self.msg_laser.range_min = self.lidar.getMinRange()
            self.msg_laser.range_max = self.lidar.getMaxRange()
            self.msg_laser.ranges = ranges
            self.laser_publisher_.publish(self.msg_laser)

    def cmd_vel_callback(self, twist):
        self.target_twist = twist

    def goal_pose_callback(self, pose):
        self.goal_pose = pose

    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)

        dt = self.robot.getTime() - self.past_time

        # Get pose
        self.global_x = self.gps.getValues()[0]
        self.global_y = self.gps.getValues()[1]
        self.global_yaw = self.imu.getRollPitchYaw()[2]
        
        q = tf_transformations.quaternion_from_euler(0.0, 0.0, self.global_yaw)
        self.gt_pose = Pose()
        self.gt_pose.position.x = self.global_x
        self.gt_pose.position.y = self.global_y
        self.gt_pose.position.z = 0.15
        self.gt_pose.orientation.x = q[0]
        self.gt_pose.orientation.y = q[1]
        self.gt_pose.orientation.z = q[2]
        self.gt_pose.orientation.w = q[3]
        self.pose_publisher_.publish(self.gt_pose)

        q = tf_transformations.quaternion_from_euler(pi, 0.0, self.global_yaw)
        t_base = TransformStamped()
        t_base.header.stamp = Time(seconds=self.robot.getTime()).to_msg()
        t_base.header.frame_id = 'map'
        t_base.child_frame_id = self.name_value
        t_base.transform.translation.x = self.global_x
        t_base.transform.translation.y = self.global_y
        t_base.transform.translation.z = 0.15
        t_base.transform.rotation.x = q[0]
        t_base.transform.rotation.y = q[1]
        t_base.transform.rotation.z = q[2]
        t_base.transform.rotation.w = q[3]
        self.tfbr.sendTransform(t_base)

        if not self.init_pose:
            self.goal_pose.position.x = self.global_x
            self.goal_pose.position.y= self.global_y
            self.init_pose = True

        # Errors
        angle = atan2(self.goal_pose.position.y-self.global_y, self.goal_pose.position.x-self.global_x)
        distance = sqrt(pow(self.goal_pose.position.x-self.global_x,2)+pow(self.goal_pose.position.y-self.global_y,2))
        # Controller "A Khepera IV library for robotic control education using V-REP"
        if(distance>0.01):
            self.linear_controller.error[0] = distance*cos(angle-self.global_yaw)
            self.target_twist.linear.x = self.linear_controller.update(dt)
            self.angular_controller.error[0] = angle-self.global_yaw
            self.target_twist.angular.z = self.angular_controller.update(dt)
        else:
            self.target_twist.linear.x = 0.0
            self.target_twist.angular.z = 0.0
        

        self.node.get_logger().debug('Yaw: %f Goal: %f Error: %f CMD: %f' % (self.global_yaw, angle, self.angular_controller.error[0], self.target_twist.angular.z))
        # self.target_twist = self.forces_field()
        
        # Velocity [rad/s]
        self.motor_left.setVelocity((self.target_twist.linear.x/0.1-self.target_twist.angular.z))
        self.motor_right.setVelocity((self.target_twist.linear.x/0.1+self.target_twist.angular.z))

        # TO-DO: Introducir Modelo Cinemático Diferencial Directo -> /odom
        self.past_time = self.robot.getTime()

        self.node.get_logger().debug('Target: X:%f Y:%f' % (self.goal_pose.position.x,self.goal_pose.position.y))
        self.node.get_logger().debug('PID cmd: vX:%f vZ:%f' % (self.target_twist.linear.x,self.target_twist.angular.z))
        self.node.get_logger().debug('Distance:%f Angle:%f' % (distance,angle))


    def forces_field(self):
        Ka = -1.0
        Kr = 150

        # Attractive force
        error_x = (self.goal_pose.position.x-self.global_x)*cos(self.global_yaw)+(self.goal_pose.position.y-self.global_y)*sin(self.global_yaw)
        error_y = -(self.goal_pose.position.x-self.global_x)*sin(self.global_yaw)+(self.goal_pose.position.y-self.global_y)*cos(self.global_yaw)

        Fa_x = Ka * error_x
        Fa_y = Ka * error_y

        Fa = sqrt(pow(Fa_x,2)+pow(Fa_y,2))
        self.node.get_logger().debug('Fa: %.2f Fa_x %.2f Fa_y %.2f ' % (Fa.real, Fa_x, Fa_y))

        # TO-DO: Reactive force
        Fr_x = Fr_y = 0
        Fr = sqrt(pow(Fr_x,2)+pow(Fr_y,2))
        self.node.get_logger().debug('Fr: %.2f Fr_x %.3f Fr_y %.3f' % (Fr.real, Fr_x, Fr_y))

        # Gloabal force
        F = sqrt(pow(Fa,2)+pow(Fr,2))
        # Movement
        vmax = 1 # [m/s]
        wmax = 1.5 # [rad/s]
        vx = vy = 0
        if F.real>0.01:
            vx = -(Fa_x+Fr_x)/F.real
            vy = -(Fa_y+Fr_y)/F.real

        if Fr_x>0:
            vx = 0

        v = sqrt(pow(vx,2)+pow(vy,2))
        w = wmax*sin(atan2(vy,vx))*10

        self.node.get_logger().debug('V: %.2f Vx: %.2f Vy: %.2f W: %.2f' % (v.real, vx, vy, w))

        ## Cmd_Vel
        out = Twist()
        if F.real>0.01:
            out.linear.x = -(Fa_x+Fr_x)
        else:
            out.linear.x = 0.0

        out.angular.z = w

        return out 





