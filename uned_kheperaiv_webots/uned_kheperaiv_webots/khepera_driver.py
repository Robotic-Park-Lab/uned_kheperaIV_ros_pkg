import rclpy
from rclpy.time import Time
import yaml

from std_msgs.msg import String, Float64, Bool
from geometry_msgs.msg import Twist, Pose, Point, PoseStamped, Vector3, TransformStamped
from sensor_msgs.msg import LaserScan, Range
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker

from math import atan2, cos, sin, sqrt, radians, pi
import tf_transformations
import numpy as np
from tf2_ros import TransformBroadcaster

class Agent():
    def __init__(self, parent, id, x = None, y = None, z = None, d = None, point = None, vector = None):
        self.id = id
        self.pose = Pose()
        self.parent = parent
        if not id.find("line") == -1:
            self.distance_bool = True
            self.d = 0
            self.point = point
            self.vector = vector
            self.mod=pow(vector.x,2)+pow(vector.y,2)+pow(vector.z,2)
            self.k = 4.0
        else:
            if d == None:
                self.distance_bool = False
                self.x = x
                self.y = y
                self.z = z
            else:
                self.distance_bool = True
                self.d = d
            
            if self.id == 'origin':
                self.pose.position.x = 0.0
                self.pose.position.y = 0.0
                self.pose.position.z = 0.0
                self.k = 2.0
                self.sub_pose = self.parent.node.create_subscription(PoseStamped, self.id + '/local_pose', self.gtpose_callback, 10)
            else:
                self.k = 1.0
                self.sub_pose = self.parent.node.create_subscription(PoseStamped, self.id + '/local_pose', self.gtpose_callback, 10)
        if not self.parent.digital_twin:
            self.sub_d_ = self.parent.node.create_subscription(Float64, '/' + self.id + '/d', self.d_callback, 10)
            self.publisher_data_ = self.parent.node.create_publisher(Float64, self.parent.name_value + '/' + self.id + '/data', 10)
            self.publisher_marker_ = self.parent.node.create_publisher(Marker, self.parent.name_value + '/' + self.id + '/marker', 10)

    def str_(self):
        return ('ID: ' + str(self.id) + ' X: ' + str(self.x) +
                ' Y: ' + str(self.y)+' Z: ' + str(self.z))

    def d_callback(self, msg):
        self.d = msg.data

    def gtpose_callback(self, msg):
        self.pose = msg.pose
        if not self.parent.digital_twin:
            line = Marker()
            p0 = Point()
            p0.x = self.parent.gt_pose.position.x
            p0.y = self.parent.gt_pose.position.y
            p0.z = self.parent.gt_pose.position.z

            p1 = Point()
            p1.x = self.pose.position.x
            p1.y = self.pose.position.y
            p1.z = self.pose.position.z
            # self.parent.distance_formation_bool = True

            distance = sqrt(pow(p0.x-p1.x,2)+pow(p0.y-p1.y,2)+pow(p0.z-p1.z,2))
        
            line.header.frame_id = 'map'
            line.header.stamp = self.parent.node.get_clock().now().to_msg()
            line.id = 1
            line.type = 5
            line.action = 0
            line.scale.x = 0.01
            line.scale.y = 0.01
            line.scale.z = 0.01

            if self.distance_bool:
                distance = sqrt(pow(p0.x-p1.x,2)+pow(p0.y-p1.y,2)+pow(p0.z-p1.z,2))
                e = abs(distance - self.d)
            else:
                e = sqrt(pow(self.x-(p0.x-p1.x),2)+pow(self.y-(p0.y-p1.y),2)+pow(self.z-(p0.z-p1.z),2))

            if e > 0.05:
                line.color.r = 1.0
            else:
                if e > 0.025:
                    line.color.r = 1.0
                    line.color.g = 0.5
                else:
                    line.color.g = 1.0

            line.color.a = 1.0
            line.points.append(p1)
            line.points.append(p0)

            self.publisher_marker_.publish(line)


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

    def eval_threshold(self, signal, ref):
        # Noise (Cn)
        mean = signal/len(self.noise)
        for i in range(0,len(self.noise)-2):
            self.noise[i] = self.noise[i+1]
            mean += self.noise[i]/len(self.noise)
        
        self.noise[len(self.noise)-1] = signal

        trigger_cn = 0.0
        for i in range(0,len(self.noise)-1):
            if abs(self.noise[i]-mean) > trigger_cn:
                trigger_cn = self.noise[i]-mean
        trigger_cn = 0.0
        # a
        a = self.trigger_ai * abs(signal - ref)
        if a > self.trigger_ai:
            a = self.trigger_ai

        # Threshold
        self.th = self.trigger_co + a + trigger_cn
        self.inc = abs(abs(ref-signal) - self.trigger_last_signal) 
        # Delta Error
        if (self.inc >= abs(self.th)):
            self.trigger_last_signal = abs(ref-signal)
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
        self.last_pose = Pose()

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
        self.gt_pose = Pose()
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = "map"
        self.target_pose.pose.position.z = 0.0
        self.target_pose.pose.position.x = 0.0
        self.target_pose.pose.position.y= 0.0

        # Position
        self.linear_controller = PIDController(1.0, 0.0, 0.0, 0.0, 100, 1.0, -1.0, 0.05, 0.01)
        # Angle
        self.angular_controller = PIDController(1.0, 0.0, 0.0, 0.0, 100, 0.0, 0.0, 0.1, 0.1)

        ## ROS2 Environment
        self.name_value = properties.get("name")
        rclpy.init(args=None)
        self.node = rclpy.create_node(self.name_value+'_driver')
        self.digital_twin = properties.get("type") == 'digital_twin'

        # Subscription
        self.node.create_subscription(Twist, self.name_value+'/cmd_vel', self.cmd_vel_callback, 1)
        self.sub_goalpose = self.node.create_subscription(PoseStamped, self.name_value+'/goal_pose', self.goal_pose_callback, 1)
        self.node.create_subscription(String, self.name_value+'/order', self.order_callback, 1)
        self.node.create_subscription(String, 'swarm/order', self.order_callback, 1)

        # Publisher
        self.laser_publisher = self.node.create_publisher(LaserScan, self.name_value+'/scan', 10)
        self.range_publisher = self.node.create_publisher(Range, self.name_value+'/range0', 10)
        if self.digital_twin:
            self.node.create_subscription(PoseStamped, self.name_value+'/local_pose', self.dt_pose_callback, 1)
            pose_name = self.name_value+'/dt_pose'
        else:
            pose_name = self.name_value+'/local_pose'
        self.pose_publisher = self.node.create_publisher(PoseStamped, pose_name, 10)
        self.path_publisher = self.node.create_publisher(Path, self.name_value+'/path', 10)
        
        self.continuous = False
        self.tfbr = TransformBroadcaster(self.node)
        self.config_file = properties.get("config_file")
        self.position_controller_IPC = False
        self.path = Path()
        self.path.header.frame_id = "map"
        self.t_event = self.robot.getTime()
        self.continuous = False
        self.trigger_ai = 0.01
        self.trigger_co = 0.1
        self.trigger_last_signal = 0.0
        self.formation_bool = False
        self.initialize()

    def initialize(self):
        self.node.get_logger().info('Webots_Node::inicialize() ok. %s' % (str(self.name_value)))
        # Read Params
        with open(self.config_file, 'r') as file:
            documents = yaml.safe_load(file)
        self.config = documents[self.name_value]

        ## Intialize Controllers
        if self.config['controller']['enable']:
            self.position_controller = True
            if self.config['controller']['type'] == 'ipc':
                self.position_controller_IPC = True
                self.eomas = 3.14
                self.node.get_logger().info('%s::IPC Controller' % (str(self.name_value)))
            else:
                self.position_controller_IPC = False
                self.node.get_logger().info('%s::Force Field Controller' % (str(self.name_value)))
        else:
            self.position_controller = False
            self.node.get_logger().info('%s::Open Loop' % (str(self.name_value)))
            
        # Set Formation
        if self.config['task']['enable']:
            self.node.destroy_subscription(self.sub_goalpose)
            self.publisher_goalpose = self.node.create_publisher(PoseStamped, self.name_value + '/goal_pose', 10)
        
        if self.config['task']['enable'] and not self.config['task']['Onboard']:
            self.node.get_logger().info('Task %s by %s' % (self.config['task']['type'], self.config['task']['role']))
            self.controller = self.config['task']['controller']
            self.controller_type = self.controller['type']
            self.k = self.controller['gain']
            self.ul = self.controller['upperLimit']
            self.ll = self.controller['lowerLimit']
            self.continuous = self.controller['protocol'] == 'Continuous'
            self.event_x = self.node.create_publisher(Bool, self.name_value + '/event_x', 10)
            self.event_y = self.node.create_publisher(Bool, self.name_value + '/event_y', 10)
            self.event_z = self.node.create_publisher(Bool, self.name_value + '/event_z', 10)

            if not self.continuous:
                self.trigger_ai = self.controller['threshold']['ai']
                self.trigger_co = self.controller['threshold']['co']

            if self.controller_type == 'pid':
                self.formation_x_controller = PIDController(self.k, 0.0, 0.0, 0.0, 100, self.ul, self.ll, self.trigger_ai, self.trigger_co)
                self.formation_y_controller = PIDController(self.k, 0.0, 0.0, 0.0, 100, self.ul, self.ll, self.trigger_ai, self.trigger_co)
                self.formation_z_controller = PIDController(self.k, 0.0, 0.0, 0.0, 100, self.ul, self.ll, self.trigger_ai, self.trigger_co)

            self.agent_list = list()
            aux = self.config['task']['relationship']
            self.relationship = aux.split(', ')
            if self.config['task']['type'] == 'distance':
                if self.controller_type == 'gradient':
                    self.node.create_timer(self.controller['period'], self.distance_gradient_controller)
                elif self.controller_type == 'pid':
                    self.node.create_timer(self.controller['period'], self.distance_pid_controller)
                for rel in self.relationship:
                    aux = rel.split('_')
                    aux = rel.split('_')
                    id = aux[0]

                    if not id.find("line") == -1:
                        p = Point()
                        p.x = float(aux[1])
                        p.y = float(aux[2])
                        p.z = float(aux[3])
                        u = Vector3()
                        u.x = float(aux[4])
                        u.y = float(aux[5])
                        u.z = float(aux[6])
                        robot = Agent(self, id, point = p, vector = u)
                        self.node.get_logger().info('Agent: %s: Neighbour: %s ::: Px: %s Py: %s Pz: %s' % (self.name_value, id, aux[1], aux[2], aux[3]))
                    else:
                        robot = Agent(self, aux[0], d = float(aux[1]))
                        self.node.get_logger().info('Agent: %s: Neighbour: %s \td: %s' % (self.name_value, aux[0], aux[1]))
                    self.agent_list.append(robot)
            elif self.config['task']['type'] == 'pose':
                if self.controller_type == 'gradient':
                    self.node.create_timer(self.controller['period'], self.pose_gradient_controller)
                elif self.controller_type == 'pid':
                    self.node.create_timer(self.controller['period'], self.pose_pid_controller)
                for rel in self.relationship:
                    aux = rel.split('_')
                    robot = Agent(self, aux[0], x = float(aux[1]), y = float(aux[2]), z = float(aux[3]))
                    self.node.get_logger().info('Agent: %s. Neighbour %s ::: x: %s \ty: %s \tz: %s' % (self.name_value, aux[0], aux[1], aux[2], aux[3]))
                    self.agent_list.append(robot)
        
        self.path_enable = self.config['local_pose']['path']
        self.communication = (self.config['communication']['type'] == 'Continuous')
        if not self.communication:
            self.threshold = self.config['communication']['threshold']['co']
        else:
            self.threshold = 0.001

        self.timer = self.node.create_timer(0.1, self.publish_laserscan_data)

    def order_callback(self, msg):
        self.node.get_logger().info('Order: "%s"' % msg.data)
        if msg.data == 'formation_run':
            if self.config['task']['enable']:
                self.formation_bool = True
        elif msg.data == 'formation_stop':
            self.formation_bool = False
        else:
            self.node.get_logger().error('"%s": Unknown order' % (msg.data))
            
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
        self.msg_laser.header.frame_id = self.name_value+'/base_link'
        self.msg_laser.range_min = 0.25
        self.msg_laser.range_max = max_range
        self.msg_laser.ranges = [right_range, frontright_range, front_range, frontleft_range, left_range]
        self.msg_laser.angle_min = -0.5 * pi
        self.msg_laser.angle_max =  0.5 * pi
        self.msg_laser.angle_increment = pi/4
        self.laser_publisher.publish(self.msg_laser)

    def dt_pose_callback(self, pose):
        self.node.get_logger().debug('DT Pose: X:%f Y:%f' % (pose.pose.position.x,pose.pose.position.y))
        delta = np.array([self.gt_pose.position.x-pose.pose.position.x,self.gt_pose.position.y-pose.pose.position.y,self.gt_pose.position.z-pose.pose.position.z])
        
        if np.linalg.norm(delta)>0.05 and False:
            self.node.get_logger().debug('DT Pose: X:%f Y:%f' % (pose.pose.position.x,pose.pose.position.y))
            self.robot.getSelf().getField("translation").setSFVec3f([pose.pose.position.x, pose.pose.position.y, 0.015])
            # self.robot.getSelf().getField("rotation").setSFVec3f([0.0, 0.0, 0.0])

    def cmd_vel_callback(self, twist):
        self.target_twist = twist

    def goal_pose_callback(self, pose):
        self.target_pose = pose
        self.node.get_logger().debug('Target: X:%f Y:%f' % (self.target_pose.pose.position.x,self.target_pose.pose.position.y))

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
        
        t_base = TransformStamped()
        t_base.header.stamp = Time(seconds=self.robot.getTime()).to_msg()
        t_base.header.frame_id = 'map'
        if self.digital_twin:
            base_name = self.name_value+'_dt/base_link'
        else:
            base_name = self.name_value+'/base_link'
        t_base.child_frame_id = base_name
        t_base.transform.translation.x = self.global_x
        t_base.transform.translation.y = self.global_y
        t_base.transform.translation.z = self.gps.getValues()[2]
        t_base.transform.rotation.x = q[0]
        t_base.transform.rotation.y = q[1]
        t_base.transform.rotation.z = q[2]
        t_base.transform.rotation.w = q[3]
        self.tfbr.sendTransform(t_base)
        
        if not self.init_pose:
            self.target_pose.pose.position.x = self.global_x
            self.target_pose.pose.position.x = self.gt_pose.position.x
            self.target_pose.pose.position.y = self.gt_pose.position.y
            self.target_pose.pose.position.z = self.gt_pose.position.z
            self.target_pose.pose.orientation.x = self.gt_pose.orientation.x
            self.target_pose.pose.orientation.y = self.gt_pose.orientation.y
            self.target_pose.pose.orientation.z = self.gt_pose.orientation.z
            self.target_pose.pose.orientation.w = self.gt_pose.orientation.w
            self.last_pose = self.gt_pose
            self.init_pose = True
        
        delta = np.array([self.gt_pose.position.x-self.last_pose.position.x,self.gt_pose.position.y-self.last_pose.position.y,self.gt_pose.position.z-self.last_pose.position.z])
        dt = self.robot.getTime() - self.t_event
        if np.linalg.norm(delta) > 0.01 or self.communication or dt>2.0:
            self.t_event = self.robot.getTime()
            PoseStamp = PoseStamped()
            PoseStamp.header.frame_id = "map"
            PoseStamp.pose.position.x = self.gt_pose.position.x
            PoseStamp.pose.position.y = self.gt_pose.position.y
            PoseStamp.pose.position.z = self.gt_pose.position.z
            PoseStamp.pose.orientation.x = self.gt_pose.orientation.x
            PoseStamp.pose.orientation.y = self.gt_pose.orientation.y
            PoseStamp.pose.orientation.z = self.gt_pose.orientation.z
            PoseStamp.pose.orientation.w = self.gt_pose.orientation.w
            PoseStamp.header.stamp = self.node.get_clock().now().to_msg()
            if self.path_enable:
                self.path.header.stamp = self.node.get_clock().now().to_msg()
                self.path.poses.append(PoseStamp)
                self.path_publisher.publish(self.path)
            self.pose_publisher.publish(PoseStamp)
            self.last_pose.position.x = self.gt_pose.position.x
            self.last_pose.position.y = self.gt_pose.position.y
            self.last_pose.position.z = self.gt_pose.position.z
        
        ## Formation Control
        # if self.formation_control_bool:
        #     if self.distance_formation_bool:
        #         self.distance_formation_control()
                # self.distance_formation_bool = False
        #     elif self.pose_formation_bool:
        #         self.pose_formation_control()

        # Position Controller
        if self.position_controller:
            if self.position_controller_IPC:
                self.target_twist = self.IPC_controller()
                self.node.get_logger().debug('IPC: vX:%f vZ:%f' % (self.target_twist.linear.x,self.target_twist.angular.z))
            else:
                # Force Field
                distance_error = sqrt(pow(self.target_pose.pose.position.x-self.global_x,2)+pow(self.target_pose.pose.position.y-self.global_y,2))
                angle_error = -self.global_yaw+atan2(self.target_pose.pose.position.y-self.global_y,self.target_pose.pose.position.x-self.global_x)
                self.target_twist = self.forces_field(dt, distance_error, angle_error)
                
                self.node.get_logger().debug('Force Field: vX:%f vZ:%f' % (self.target_twist.linear.x,self.target_twist.angular.z))
                self.node.get_logger().debug('Distance:%f Angle:%f' % (distance_error.real,angle_error))
        else:
            self.motor_right.setVelocity((self.target_twist.linear.x/2+(self.target_twist.angular.z*0.10540*100)/2))
            self.motor_left.setVelocity(( self.target_twist.linear.x/2-(self.target_twist.angular.z*0.10540*100)/2))
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
        
    def distance_gradient_controller(self):
        if self.formation_bool:
            dx = dy = dz = 0
            for agent in self.agent_list:
                if not agent.id.find("line") == -1:
                    nearest = PoseStamped()
                    nearest.header.frame_id = "map"
                    gamma = -np.dot([agent.point.x-self.gt_pose.position.x, agent.point.y-self.gt_pose.position.y, agent.point.z-self.gt_pose.position.z],[agent.vector.x, agent.vector.y, agent.vector.z])/agent.mod
                    nearest.pose.position.x = agent.point.x + gamma * agent.vector.x
                    nearest.pose.position.y = agent.point.y + gamma * agent.vector.y
                    nearest.pose.position.z = agent.point.z + gamma * agent.vector.z
                    agent.gtpose_callback(nearest)
                error_x = self.gt_pose.position.x - agent.pose.position.x
                error_y = self.gt_pose.position.y - agent.pose.position.y
                error_z = self.gt_pose.position.z - agent.pose.position.z
                distance = pow(error_x,2)+pow(error_y,2)+pow(error_z,2)
                d = sqrt(distance)
                dx += self.k * agent.k * (pow(agent.d,2) - distance) * error_x/d
                dy += self.k * agent.k * (pow(agent.d,2) - distance) * error_y/d
                dz += self.k * agent.k * (pow(agent.d,2) - distance) * error_z/d
                
                if not self.digital_twin:
                    msg_data = Float64()
                    msg_data.data = abs(agent.d - d)
                    agent.publisher_data_.publish(msg_data)
                    self.node.get_logger().debug('Agent %s: D: %.2f dx: %.2f dy: %.2f dz: %.2f ' % (agent.id, msg_data.data, dx, dy, dz)) 
            
            if not self.continuous:
                delta=sqrt(pow(dx,2)+pow(dy,2)+pow(dz,2))
                if not self.eval_threshold(0.0, delta):
                    return
            
            msg = Bool()
            msg.data = True
            self.event_x.publish(msg)

            if dx > self.ul:
                dx = self.ul
            if dx < self.ll:
                dx = self.ll
            if dy > self.ul:
                dy = self.ul
            if dy < self.ll:
                dy = self.ll

            self.target_pose.pose.position.x = self.gt_pose.position.x + dx
            self.target_pose.pose.position.y = self.gt_pose.position.y + dy
            self.target_pose.pose.position.z = self.gt_pose.position.z
        
            delta=sqrt(pow(dx,2)+pow(dy,2)+pow(dz,2))
            angles = tf_transformations.euler_from_quaternion((self.gt_pose.orientation.x, self.gt_pose.orientation.y, self.gt_pose.orientation.z, self.gt_pose.orientation.w))
                    
            if delta<0.05:
                roll = angles[0]
                pitch = angles[1]
                yaw = angles[2]
            else:
                h = sqrt(pow(dx,2)+pow(dy,2))
                roll = 0.0
                pitch = -atan2(dz,h)
                yaw = atan2(dy,dx)

            q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
            self.target_pose.pose.orientation.x = q[0]
            self.target_pose.pose.orientation.y = q[1]
            self.target_pose.pose.orientation.z = q[2]
            self.target_pose.pose.orientation.w = q[3]
            self.target_pose.header.stamp = self.node.get_clock().now().to_msg()
            self.publisher_goalpose.publish(self.target_pose)

    def distance_pid_controller(self):
        if self.formation_bool:
            ex = ey = ez = 0
            for agent in self.agent_list:
                if not agent.id.find("line") == -1:
                    nearest = PoseStamped()
                    nearest.header.frame_id = "map"
                    gamma = -np.dot([agent.point.x-self.gt_pose.position.x, agent.point.y-self.gt_pose.position.y, agent.point.z-self.gt_pose.position.z],[agent.vector.x, agent.vector.y, agent.vector.z])/agent.mod
                    nearest.pose.position.x = agent.point.x + gamma * agent.vector.x
                    nearest.pose.position.y = agent.point.y + gamma * agent.vector.y
                    nearest.pose.position.z = agent.point.z + gamma * agent.vector.z
                    agent.gtpose_callback(nearest)
                error_x = self.gt_pose.position.x - agent.pose.position.x
                error_y = self.gt_pose.position.y - agent.pose.position.y
                error_z = self.gt_pose.position.z - agent.pose.position.z
                distance = sqrt(pow(error_x,2)+pow(error_y,2)+pow(error_z,2))
                e = agent.d - distance
                ex += agent.k * e * (error_x/distance)
                ey += agent.k * e * (error_y/distance)
                ez += agent.k * e * (error_z/distance)
                
                if not self.digital_twin:
                    msg_data = Float64()
                    msg_data.data = e
                    agent.publisher_data_.publish(msg_data)

            aux = self.node.get_clock().now().to_msg()
            time = aux.sec + aux.nanosec*1e-9
                
            if not (self.formation_x_controller.eval_threshold(0.0, ex) or self.formation_y_controller.eval_threshold(0.0, ey) or self.formation_z_controller.eval_threshold(0.0, ez) or self.continuous):
                return
            
            msg = Bool()
            msg.data = True
            self.event_x.publish(msg)
            # X Controller
            self.formation_x_controller.error[0] = ex
            dtx = time - self.formation_x_controller.past_time
            dx = self.formation_x_controller.update(dtx)
            self.formation_x_controller.past_time = time

            # Y Controller
            self.formation_y_controller.error[0] = ey
            dty = time - self.formation_y_controller.past_time
            dy = self.formation_y_controller.update(dty)
            self.formation_y_controller.past_time = time

            # Z Controller
            self.formation_z_controller.error[0] = ez
            dtz = time - self.formation_z_controller.past_time
            dz = self.formation_z_controller.update(dtz)
            self.formation_z_controller.past_time = time

            self.target_pose.pose.position.x = self.gt_pose.position.x + dx
            self.target_pose.pose.position.y = self.gt_pose.position.y + dy
            self.target_pose.pose.position.z = self.gt_pose.position.z + dz

            delta=sqrt(pow(dx,2)+pow(dy,2)+pow(dz,2))
            angles = tf_transformations.euler_from_quaternion((self.gt_pose.orientation.x, self.gt_pose.orientation.y, self.gt_pose.orientation.z, self.gt_pose.orientation.w))
                    
            if delta<0.05:
                roll = angles[0]
                pitch = angles[1]
                yaw = angles[2]
            else:
                h = sqrt(pow(dx,2)+pow(dy,2))
                roll = 0.0
                pitch = -atan2(dz,h)
                yaw = atan2(dy,dx)

            q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
            self.target_pose.pose.orientation.x = q[0]
            self.target_pose.pose.orientation.y = q[1]
            self.target_pose.pose.orientation.z = q[2]
            self.target_pose.pose.orientation.w = q[3]
            self.target_pose.header.stamp = self.node.get_clock().now().to_msg()
            self.publisher_goalpose.publish(self.target_pose)

    def pose_gradient_controller(self):
        if self.formation_bool:
            dx = dy = dz = 0
            for agent in self.agent_list:
                error_x = self.gt_pose.position.x - agent.pose.position.x
                error_y = self.gt_pose.position.y - agent.pose.position.y
                distance = pow(error_x,2)+pow(error_y,2)
                d = sqrt(distance)
                dx += self.k * agent.k * (pow(agent.x,2) - pow(error_x,2)) * error_x/d
                dy += self.k * agent.k * (pow(agent.y,2) - pow(error_y,2)) * error_y/d
                        
                msg_data = Float64()
                msg_data.data = sqrt(pow(agent.x-error_x,2) + pow(agent.y-error_y,2))
                agent.publisher_data_.publish(msg_data)
            
            if not self.continuous:
                delta=sqrt(pow(dx,2)+pow(dy,2))
                if not self.eval_threshold(0.0, delta):
                    return

            msg = Bool()
            msg.data = True
            self.event_x.publish(msg)

            if dx > self.ul:
                dx = self.ul
            if dx < self.ll:
                dx = self.ll
            if dy > self.ul:
                dy = self.ul
            if dy < self.ll:
                dy = self.ll

            self.target_pose.pose.position.x = self.gt_pose.position.x + dx
            self.target_pose.pose.position.y = self.gt_pose.position.y + dy
            self.target_pose.pose.position.z = self.gt_pose.position.z
        
            delta=sqrt(pow(dx,2)+pow(dy,2)+pow(dz,2))
            angles = tf_transformations.euler_from_quaternion((self.gt_pose.orientation.x, self.gt_pose.orientation.y, self.gt_pose.orientation.z, self.gt_pose.orientation.w))
                    
            if delta<0.05:
                roll = angles[0]
                pitch = angles[1]
                yaw = angles[2]
            else:
                h = sqrt(pow(dx,2)+pow(dy,2))
                roll = 0.0
                pitch = -atan2(dz,h)
                yaw = atan2(dy,dx)

            q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
            self.target_pose.pose.orientation.x = q[0]
            self.target_pose.pose.orientation.y = q[1]
            self.target_pose.pose.orientation.z = q[2]
            self.target_pose.pose.orientation.w = q[3]
            self.target_pose.header.stamp = self.node.get_clock().now().to_msg()
            self.publisher_goalpose.publish(self.target_pose)

    def pose_pid_controller(self):
        if self.formation_bool:
            ex = ey = ez = 0
            for agent in self.agent_list:
                error_x = agent.x - (self.gt_pose.position.x - agent.pose.position.x)
                error_y = agent.y - (self.gt_pose.position.y - agent.pose.position.y)
                ex += agent.k * error_x
                ey += agent.k * error_y
                        
                msg_data = Float64()
                msg_data.data = sqrt(pow(error_x,2)+pow(error_y,2))
                agent.publisher_data_.publish(msg_data)

            aux = self.node.get_clock().now().to_msg()
            time = aux.sec + aux.nanosec*1e-9
            
            msg = Bool()
            msg.data = True
            dx = dy = dz = 0
            # X Controller
            if self.formation_x_controller.eval_threshold(0.0, ex) or self.continuous:
                self.formation_x_controller.error[0] = ex
                dtx = time - self.formation_x_controller.past_time
                dx = self.formation_x_controller.update(dtx)
                self.formation_x_controller.past_time = time
                self.event_x.publish(msg)
            
            # Y Controller
            if self.formation_y_controller.eval_threshold(0.0, ey) or self.continuous:
                self.formation_y_controller.error[0] = ey
                dty = time - self.formation_y_controller.past_time
                dy = self.formation_y_controller.update(dty)
                self.formation_y_controller.past_time = time
                self.event_y.publish(msg)

            self.target_pose.pose.position.x = self.gt_pose.position.x + dx
            self.target_pose.pose.position.y = self.gt_pose.position.y + dy

            delta=sqrt(pow(dx,2)+pow(dy,2)+pow(dz,2))
            angles = tf_transformations.euler_from_quaternion((self.gt_pose.orientation.x, self.gt_pose.orientation.y, self.gt_pose.orientation.z, self.gt_pose.orientation.w))
                    
            if delta<0.05:
                roll = angles[0]
                pitch = angles[1]
                yaw = angles[2]
            else:
                h = sqrt(pow(dx,2)+pow(dy,2))
                roll = 0.0
                pitch = -atan2(dz,h)
                yaw = atan2(dy,dx)

            q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
            self.target_pose.pose.orientation.x = q[0]
            self.target_pose.pose.orientation.y = q[1]
            self.target_pose.pose.orientation.z = q[2]
            self.target_pose.pose.orientation.w = q[3]
            self.target_pose.header.stamp = self.node.get_clock().now().to_msg()
            self.publisher_goalpose.publish(self.target_pose)

    def eval_threshold(self, signal, ref):
        '''
        # Noise (Cn)
        mean = signal/len(self.noise)
        for i in range(0,len(self.noise)-2):
            self.noise[i] = self.noise[i+1]
            mean += self.noise[i]/len(self.noise)
        
        self.noise[len(self.noise)-1] = signal

        trigger_cn = 0.0
        for i in range(0,len(self.noise)-1):
            if abs(self.noise[i]-mean) > trigger_cn:
                trigger_cn = self.noise[i]-mean
        '''
        trigger_cn = 0.0

        # a
        a = self.trigger_ai * abs(signal - ref)
        if a > self.trigger_ai:
            a = self.trigger_ai

        # Threshold
        self.th = self.trigger_co + a + trigger_cn
        self.inc = abs(abs(ref-signal) - self.trigger_last_signal) 
        # Delta Error
        if (self.inc >= abs(self.th)):
            self.trigger_last_signal = abs(ref-signal)
            return True

        return False
    
    def forces_field(self,dt, error, angle_error):
        Ka = -1.0
        Kr = 150

        # Attractive force
        error_x = (self.target_pose.pose.position.x-self.global_x)*cos(self.global_yaw)+(self.target_pose.pose.position.y-self.global_y)*sin(self.global_yaw)
        error_y = -(self.target_pose.pose.position.x-self.global_x)*sin(self.global_yaw)+(self.target_pose.pose.position.y-self.global_y)*cos(self.global_yaw)

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
        Vmax = 10.0
        K1 = 1.0
        Kp = 1.5
        Ki = 0.008

        d = sqrt(pow(self.target_pose.pose.position.x-self.global_x,2)+pow(self.target_pose.pose.position.y-self.global_y,2))*100
        if d<1.0:
            V = 0.0
            angles = tf_transformations.euler_from_quaternion((self.target_pose.pose.orientation.x, self.target_pose.pose.orientation.y, self.target_pose.pose.orientation.z, self.target_pose.pose.orientation.w))
            oc = angles[2] - self.global_yaw
            if abs(oc)>0.01:
                eo = atan2(sin(oc),cos(oc))
                self.eomas = eo+self.eomas
                w = Kp*sin(eo) + Ki*self.eomas*0.003
            else:
                w = 0.0
        else:
            alpha = atan2(self.target_pose.pose.position.y-self.global_y,self.target_pose.pose.position.x-self.global_x)
            oc = alpha - self.global_yaw
            eo = atan2(sin(oc),cos(oc))
            p = (3.14-abs(eo))/3.14
            self.eomas = eo+self.eomas
            w = Kp*sin(eo) + Ki*self.eomas*0.003
            V = min(K1*d*p,Vmax)

        ## Cmd_Vel
        out = Twist()
        out.linear.x = V
        out.angular.z = w

        self.motor_right.setVelocity((V+(w*L*100)/2))
        self.motor_left.setVelocity(( V-(w*L*100)/2))
        

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

