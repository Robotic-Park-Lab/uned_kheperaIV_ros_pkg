from rosgraph_msgs.msg import Clock
import rclpy
import os
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan, Range
from nav_msgs.msg import Odometry

from math import atan2, cos, sin, sqrt, radians, pi
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

        ## Initialize Pose
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
        self.name_value = os.environ['WEBOTS_ROBOT_NAME']
        rclpy.init(args=None)
        self.node = rclpy.create_node(self.name_value+'_driver')
        self.node.get_logger().info('Webots_Node::inicialize() ok. %s' % (str(self.name_value)))
        self.pose_publisher = self.node.create_publisher(Pose, self.name_value+'/local_pose', 10)
        self.range_publisher = self.node.create_publisher(Range, self.name_value+'/range0', 10)
        self.node.create_subscription(Twist, self.name_value+'/cmd_vel', self.cmd_vel_callback, 1)
        self.node.create_subscription(Pose, self.name_value+'/goal_pose', self.goal_pose_callback, 1)
        self.tfbr = TransformBroadcaster(self.node)

        ## Intialize Controllers
        self.node.get_logger().info('Webots_Node::Param. %s' % (str(properties.get("controller"))))
        if properties.get("controller") == 'IPC':
            self.controller_IPC = True
            self.eomas = 3.14
            self.node.get_logger().info('%s::IPC Controller' % (str(self.name_value)))
        else:
            self.controller_IPC = False
            self.node.get_logger().info('%s::Force Field Controller' % (str(self.name_value)))
        self.continuous = False
        # Position
        self.linear_controller = PIDController(1.0, 0.0, 0.0, 0.0, 100, 1.0, -1.0, 0.05, 0.01)
        # Angle
        self.angular_controller = PIDController(1.0, 0.0, 0.0, 0.0, 100, 0.0, 0.0, 0.1, 0.1)
    
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
        self.node.get_logger().debug('Target: X:%f Y:%f' % (self.target_pose.position.x,self.target_pose.position.y))

    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)

        dt = self.robot.getTime() - self.past_time

        # Get pose
        self.global_x = self.gps.getValues()[0]
        self.global_y = self.gps.getValues()[1]
        roll = self.imu.getRollPitchYaw()[0]
        pitch =  self.imu.getRollPitchYaw()[1]
        self.global_yaw = self.imu.getRollPitchYaw()[2]
        
        q = tf_transformations.quaternion_from_euler(roll, pitch, self.global_yaw)
        self.gt_pose = Pose()
        self.gt_pose.position.x = self.global_x
        self.gt_pose.position.y = self.global_y
        self.gt_pose.position.z = 0.0
        self.gt_pose.orientation.x = q[0]
        self.gt_pose.orientation.y = q[1]
        self.gt_pose.orientation.z = q[2]
        self.gt_pose.orientation.w = q[3]
        self.pose_publisher.publish(self.gt_pose)
        
        t_base = TransformStamped()
        t_base.header.stamp = Time(seconds=self.robot.getTime()).to_msg()
        t_base.header.frame_id = 'map'
        t_base.child_frame_id = self.name_value
        t_base.transform.translation.x = self.global_x
        t_base.transform.translation.y = self.global_y
        t_base.transform.translation.z = self.gps.getValues()[2]
        t_base.transform.rotation.x = q[0]
        t_base.transform.rotation.y = q[1]
        t_base.transform.rotation.z = q[2]
        t_base.transform.rotation.w = q[3]
        self.tfbr.sendTransform(t_base)
        
        if not self.init_pose:
            self.target_pose.position.x = self.global_x
            self.target_pose.position.y= self.global_y
            self.init_pose = True
        
        # TO-DO: Position Controller
        if self.controller_IPC:
            self.target_twist = self.IPC_controller()
            self.node.get_logger().info('IPC: vX:%f vZ:%f' % (self.target_twist.linear.x,self.target_twist.angular.z))
        else:
            # Force Field
            distance_error = sqrt(pow(self.target_pose.position.x-self.global_x,2)+pow(self.target_pose.position.y-self.global_y,2))
            angle_error = -self.global_yaw+atan2(self.target_pose.position.y-self.global_y,self.target_pose.position.x-self.global_x)
            self.target_twist = self.forces_field(dt, distance_error, angle_error)
            
            self.node.get_logger().debug('Force Field: vX:%f vZ:%f' % (self.target_twist.linear.x,self.target_twist.angular.z))
            self.node.get_logger().debug('Distance:%f Angle:%f' % (distance_error.real,angle_error))
        
        # PID
        # self.linear_controller.error[0] = distance_error
        # self.angular_controller.error[0] = angle_error
        # self.target_twist.linear.x = self.linear_controller.update(dt)
        # self.target_twist.angular.z = self.angular_controller.update(dt)

        # Velocity [rad/s]
        # self.motor_left.setVelocity((self.target_twist.linear.x/0.042-0.10540*self.target_twist.angular.z))
        # self.motor_right.setVelocity((self.target_twist.linear.x/0.042+0.10540*self.target_twist.angular.z))

        # TO-DO: Introducir Modelo Cinemático Diferencial Directo -> /odom

        self.past_time = self.robot.getTime()

        self.node.get_logger().debug('Pose3D: X:%f Y:%f yaw:%f' % (self.global_x,self.global_y,self.global_yaw))
        self.node.get_logger().debug('PID cmd: vX:%f vZ:%f' % (self.target_twist.linear.x,self.target_twist.angular.z))
        

    
    def forces_field(self,dt, error, angle_error):
        Ka = -1.0
        Kr = 150

        # Attractive force
        error_x = (self.target_pose.position.x-self.global_x)*cos(self.global_yaw)+(self.target_pose.position.y-self.global_y)*sin(self.global_yaw)
        error_y = -(self.target_pose.position.x-self.global_x)*sin(self.global_yaw)+(self.target_pose.position.y-self.global_y)*cos(self.global_yaw)

        Fa_x = Ka * error_x
        Fa_y = Ka * error_y

        Fa = sqrt(pow(Fa_x,2)+pow(Fa_y,2))
        self.node.get_logger().debug('Fa: %.2f Fa_x %.2f Fa_y %.2f ' % (Fa.real, Fa_x, Fa_y))

        # Reactive force
        msg = Range()
        msg.radiation_type = 0
        msg.min_range = 0.1
        msg.max_range = 3.5
        msg.header.stamp = Time(seconds=self.robot.getTime()).to_msg()
        msg.header.frame_id = self.name_value

        limit_distance = 30
        Fr_x = Fr_y = 0
        sensor_value = self.range_left.getValue() * 100
        if sensor_value<limit_distance:
            Fr_x += Kr * ((1/sensor_value)-(1/limit_distance))*(1/pow(sensor_value,2))*cos(self.global_yaw+radians(90))
            Fr_y += Kr * ((1/sensor_value)-(1/limit_distance))*(1/pow(sensor_value,2))*sin(self.global_yaw+radians(90))
            # self.node.get_logger().debug('Sensor 1: %.2f Fr_x %.2f Fr_y %.2f ' % (sensor_value,  Fr_x, Fr_y))
        sensor_value = self.range_frontleft.getValue() * 100
        if sensor_value<limit_distance:
            Fr_x += Kr * ((1/sensor_value)-(1/limit_distance))*(1/pow(sensor_value,2))*cos(self.global_yaw+radians(45))
            Fr_y += Kr * ((1/sensor_value)-(1/limit_distance))*(1/pow(sensor_value,2))*sin(self.global_yaw+radians(45))
            # self.node.get_logger().debug('Sensor 2: %.2f Fr_x %.2f Fr_y %.2f ' % (sensor_value,  Fr_x, Fr_y))
        sensor_value = self.range_front.getValue() * 100
        msg.range = self.range_front.getValue()
        self.range_publisher.publish(msg)
        if sensor_value<limit_distance:
            Fr_x += Kr * ((1/sensor_value)-(1/limit_distance))*(1/pow(sensor_value,2))*cos(self.global_yaw)
            Fr_y += Kr * ((1/sensor_value)-(1/limit_distance))*(1/pow(sensor_value,2))*sin(self.global_yaw)
            # self.node.get_logger().debug('Sensor 3: %.2f Fr_x %.2f Fr_y %.2f ' % (sensor_value,  Fr_x, Fr_y))
        sensor_value = self.range_frontright.getValue() * 100
        if sensor_value<limit_distance:
            Fr_x += Kr * ((1/sensor_value)-(1/limit_distance))*(1/pow(sensor_value,2))*cos(self.global_yaw+radians(-45))
            Fr_y += Kr * ((1/sensor_value)-(1/limit_distance))*(1/pow(sensor_value,2))*sin(self.global_yaw+radians(-45))
            # self.node.get_logger().debug('Sensor 4: %.2f Fr_x %.2f Fr_y %.2f ' % (sensor_value,  Fr_x, Fr_y))
        sensor_value = self.range_right.getValue() * 100
        if sensor_value<limit_distance:
            Fr_x += Kr * ((1/sensor_value)-(1/limit_distance))*(1/pow(sensor_value,2))*cos(self.global_yaw+radians(-90))
            Fr_y += Kr * ((1/sensor_value)-(1/limit_distance))*(1/pow(sensor_value,2))*sin(self.global_yaw+radians(-90))
            # self.node.get_logger().debug('Sensor 5: %.2f Fr_x %.2f Fr_y %.2f ' % (sensor_value,  Fr_x, Fr_y))

        if Fr_x>600:
            Fr_x = -600
        if Fr_y > 600:
            Fr_y = 600
        
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

        ## Next Pose:
        # out = Pose()
        # out.position.x = - dt * vmax * (Fa_x+Fr_x)/F.real
        # out.position.y = - dt * vmax * (Fa_y+Fr_y)/F.real

        ## Cmd_Vel
        out = Twist()
        if F.real>0.001:
            out.linear.x = -(Fa_x+Fr_x)
        else:
            out.linear.x = 0.0

        out.angular.z = w

        return out 

    def IPC_controller(self):
        L = 0.10540
        Vmax = 1.0
        K1 = 0.1
        Kp = 1.5
        Ki = 0.008

        d = sqrt(pow(self.target_pose.position.x-self.global_x,2)+pow(self.target_pose.position.y-self.global_y,2))*100
        alpha = atan2(self.target_pose.position.y-self.global_y,self.target_pose.position.x-self.global_x)
        oc = alpha - self.global_yaw
        eo = atan2(sin(oc),cos(oc))
        p = (3.14-abs(eo))/3.14
        V = min(K1*d*p,Vmax)

        self.eomas = eo+self.eomas
        w = Kp*sin(eo) + Ki*self.eomas*0.003

        ## Cmd_Vel
        out = Twist()
        out.linear.x = V
        out.angular.z = w

        self.motor_left.setVelocity((V-(w*L*100)/2))
        self.motor_right.setVelocity((V+(2*V+w*L*100)/2))

        return out 
        '''
        Position_control=function(xp,yp,xc,yc,gamma,Vmax,Wmax,L,Kr)
            d = math.sqrt(((xp-xc)^2)+((yp-yc)^2))*100      
            alpha = math.atan2(yp-yc,xp-xc)
            Oc = alpha-gamma
            eo= math.atan2((math.sin(Oc)),(math.cos(Oc)))
            % w=(Wmax*math.sin(eo))
            p=(3.14-math.abs(eo))/3.14
            V=math.min(K1*d*p,Vmax)
   
            eomas=eo+eomas
            w=Kp*math.sin(eo)+Ki*eomas*0.003

            % result=simxAddStatusbarMessage(,oemas,simx_opmode_oneshot)

            % if (d>Kr) then--(d>=0.005 or Oc>=0.087)) then
            %    w=(Wmax*math.sin(Oc))
            %    V=(Kp*d)
            %    if V>Vmax then
            %        V=Vmax
            %    end
            %    Vr=(2*V+w*L)/2    
            %    Vl=(2*V-w*L)/2    
            %    Flag=0
            % else
            %    V=d*(Vmax/Kr)
            %    Vl=0
            %    Vr=0
                Flag=1
            % end

            Vr=((2*V+w*L*100)/2)/100    
            Vl=((2*V-w*L*100)/2)/100
   

            return Flag,Vr,Vl
        '''

