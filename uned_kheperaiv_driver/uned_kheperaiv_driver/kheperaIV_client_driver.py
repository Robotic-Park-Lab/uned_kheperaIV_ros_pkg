import rclpy
import socket
import numpy as np
import yaml
import time
import rclpy.qos
from rclpy.node import Node
from std_msgs.msg import String, Bool, UInt16MultiArray, Float64, Float64MultiArray
from geometry_msgs.msg import Twist, Pose, TransformStamped, Point, PoseStamped, Vector3, Quaternion
from sensor_msgs.msg import Imu
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from math import sqrt, cos, sin, atan2
from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from builtin_interfaces.msg import Time

class Agent():
    def __init__(self, parent, id, x = None, y = None, z = None, d = None, point = None, vector = None):
        self.id = id
        self.idn = len(parent.agent_list)
        self.distance = False
        self.parent = parent
        self.pose = Pose()
        if not id.find("line") == -1:
            self.distance = True
            self.d = 0
            self.point = point
            self.vector = vector
            self.mod=pow(vector.x,2)+pow(vector.y,2)+pow(vector.z,2)
            self.k = 4.0
        else:
            if d == None:
                self.x = x
                self.y = y
                self.z = z
            else:
                self.d = d
                self.distance = True
                self.sub_d_ = self.parent.create_subscription(Float64, '/' + self.id + '/d', self.d_callback, 10)
            if self.id == 'origin':
                self.pose.position.x = 0.0
                self.pose.position.y = 0.0
                self.pose.position.z = 0.0
                self.k = 2.0
            else:
                self.k = 1.0
                self.sub_pose_ = self.parent.create_subscription(PoseStamped, '/' + self.id + '/local_pose', self.gtpose_callback, 10)
            if self.parent.onboard:
                command = "n " + str(self.idn) + " " + str(self.d) + " " + str(self.k)
                self.parent.sock.sendall(bytes(command, 'utf-8'))
                self.parent.get_logger().info('New Agent: %s' % command)
            
        self.publisher_data_ = self.parent.create_publisher(Float64, self.id + '/data', 10)
        self.publisher_marker_ = self.parent.create_publisher(Marker, self.id + '/marker', 10)

    def str_(self):
        return ('ID: ' + str(self.id) + ' X: ' + str(self.x) +
                ' Y: ' + str(self.y)+' Z: ' + str(self.z))
    
    def str_distance_(self):
        return ('ID: ' + str(self.id) + ' Distance: ' + str(self.d))

    def d_callback(self, msg):
        self.d = msg.data

    def gtpose_callback(self, msg):
        self.pose = msg.pose
        self.neighbour_update = True
        if self.parent.onboard and False:
            self.parent.get_logger().debug('Update Neighbour pose')
            command = "m " + str(self.idn) + " " + str(round(self.pose.position.x,3)) + " " + str(round(self.pose.position.y,3)) + " " + str(round(self.pose.position.z,3))
            self.parent.sock.sendall(bytes(command, 'utf-8'))
        
        line = Marker()
        p0 = Point()
        p0.x = self.parent.pose.pose.position.x
        p0.y = self.parent.pose.pose.position.y
        p0.z = self.parent.pose.pose.position.z

        p1 = Point()
        p1.x = self.pose.position.x
        p1.y = self.pose.position.y
        p1.z = self.pose.position.z

        line.header.frame_id = 'map'
        line.header.stamp = self.parent.get_clock().now().to_msg()
        line.id = 1
        line.type = 5
        line.action = 0
        line.scale.x = 0.01
        line.scale.y = 0.01
        line.scale.z = 0.01
        
        if self.distance:
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


class KheperaIVDriver(Node):
    def __init__(self):
        super().__init__('driver')

        # ======================== ROS 2 ENVIROMENT ==================================
        # Configuración de QoS para mejor desempeño
        qos_profile = rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value
        # =========================== PARÁMETROS =====================================
        self.declare_parameter('config', 'file_path.yaml')
        self.declare_parameter('id', 'khepera01')

        # ========================== PUBLICADORES ====================================
        self.status_pub = self.create_publisher(String,'status', qos_profile)
        self.pose_pub = self.create_publisher(PoseStamped,'local_pose', qos_profile)
        self.path_pub = self.create_publisher(Path, 'path', qos_profile)
        self.imu_pub = self.create_publisher(Imu, 'imu', qos_profile)
        # ========================== SUSCRIPTORES ====================================
        self.create_subscription(PoseStamped, 'pose', self.pose_callback, qos_profile)
        self.create_subscription(String, 'cmd', self.cmd_callback, qos_profile)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, qos_profile)
        self.create_subscription(String, '/swarm/status', self.order_callback, qos_profile)
        self.create_subscription(String, '/swarm/order', self.order_callback, qos_profile)
        self.create_subscription(Time, '/swarm/time', self.time_callback, qos_profile)

        # ======================= VARIABLES GLOBALES =================================
        self.tfbr = TransformBroadcaster(self)
        self.pose = PoseStamped()
        self.pose.header.frame_id = "map"
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = "map"
        self.path = Path()
        self.path.header.frame_id = "map"
        self.max_path_length = 500  # Límite de puntos almacenados
        self.time = Time()
        self.target_twist = Twist()
        self.formation_bool = False
        self.first_goal_pose = False
        self.neighbour_update = False
        self.init_pose = False
        self.last_time = 0.0
        self.rele = False

        self.initialize()

    def initialize(self):
        self.get_logger().info('KheperaIVDriver::inicialize() ok.')
        
        # ======================= CONFIG PARAMETERS =================================
        self.id = self.get_parameter('id').get_parameter_value().string_value
        config_file = self.get_parameter('config').get_parameter_value().string_value
        with open(config_file, 'r') as file:
            documents = yaml.safe_load(file)
        self.config = documents[self.id]

        # Configuración de conexión
        self.robot_ip = self.config['agent_ip']
        self.robot_port = self.config['port_number']
        self.control_rate = 0.5
        self.get_logger().info('%s:: IP: %s;Port %s.' % (self.id, self.robot_ip, str(self.robot_port)))
        self.connected = False

        # Set Init Pose TO-DO: Improve
        self.pose.pose.position.x = self.config['pose']['x']
        self.pose.pose.position.y = self.config['pose']['y']
        self.theta = self.config['init_theta']
        self.theta_vicon = self.theta
        self.timer_pose = self.create_timer(self.config['local_pose']['T']/100, self.get_pose)
        self.path_enable = self.config['local_pose']['path']
        self.communication = (self.config['communication']['type'] == 'Continuous')
        if not self.communication:
            self.threshold = self.config['communication']['threshold']['co']
        else:
            self.threshold = 0.001
        self.positioning = self.config['positioning']
        # if self.positioning == 'Intern':
        #     self.pose_callback(self.pose)

        # Set Formation TO-DO: Improve
        if self.config['task']['enable']:
            self.publisher_goalpose = self.create_publisher(PoseStamped, 'goal_pose', 10)
        else:
            self.create_subscription(PoseStamped, 'goal_pose', self.goalpose_callback, 1)

        self.onboard = self.config['task']['Onboard']
        if self.config['task']['enable'] and not self.onboard:
            self.get_logger().info('Task %s by %s' % (self.config['task']['type'], self.config['task']['role']))
            self.controller = self.config['task']['controller']
            self.controller_type = self.controller['type']
            self.k = self.controller['gain']
            self.ul = self.controller['upperLimit']
            self.ll = self.controller['lowerLimit']
            self.continuous = self.controller['protocol'] == 'Continuous'
            self.event_x = self.create_publisher(Bool, '/'+self.id+'/event_x', 10)
            self.event_y = self.create_publisher(Bool, '/'+self.id+'/event_y', 10)
            self.event_z = self.create_publisher(Bool, '/'+self.id+'/event_z', 10)

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
                if self.onboard:
                    self.create_timer(self.controller['period'], self.task_formation_info)
                elif self.controller_type == 'gradient':
                    self.create_timer(self.controller['period'], self.distance_gradient_controller)
                elif self.controller_type == 'pid':
                    self.create_timer(self.controller['period'], self.distance_pid_controller)
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
                        self.get_logger().info('Agent: %s: Neighbour: %s ::: Px: %s Py: %s Pz: %s' % (self.id, id, aux[1], aux[2], aux[3]))
                    else:
                        robot = Agent(self, aux[0], d = float(aux[1]))
                        self.get_logger().info('Agent: %s: Neighbour: %s \td: %s' % (self.id, aux[0], aux[1]))
                    self.agent_list.append(robot)
            elif self.config['task']['type'] == 'pose':
                if self.controller_type == 'gradient':
                    self.create_timer(self.controller['period'], self.pose_gradient_controller)
                elif self.controller_type == 'pid':
                    self.create_timer(self.controller['period'], self.pose_pid_controller)
                for rel in self.relationship:
                    aux = rel.split('_')
                    robot = Agent(self, aux[0], x = float(aux[1]), y = float(aux[2]), z = float(aux[3]))
                    self.get_logger().info('Agent: %s. Neighbour %s ::: x: %s \ty: %s \tz: %s' % (self.id, aux[0], aux[1], aux[2], aux[3]))
                    self.agent_list.append(robot)
        
        self.connect_to_robot()
        self.imu_timer = self.create_timer(1.0/self.control_rate, self.get_imu)
        self.ultrasound_timer = self.create_timer(1.0/self.control_rate, self.get_us)

    def connect_to_robot(self):
        """Establece conexión TCP con el robot"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(2.0)
            self.sock.connect((self.robot_ip, self.robot_port))
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.connected = True
            self.get_logger().info(f"Conexión establecida con {self.robot_ip}:{self.robot_port}")
            if self.positioning == 'Intern':
                self.pose_callback(self.pose)
        except Exception as e:
            self.get_logger().error(f"Error de conexión: {str(e)}")
            self.connected = False
            self.reconnect_timer = self.create_timer(5.0, self.reconnect_attempt)

    def reconnect_attempt(self):
        """Intenta reconectar al robot"""
        if not self.connected:
            self.get_logger().info("Intentando reconectar...")
            self.connect_to_robot()

    def send_command(self, command):
        """Envía comando al robot con manejo de errores"""
        self.get_logger().debug('command:%s' % command)
        if self.connected:
            try:
                self.sock.sendall(command.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f"Error enviando comando: {str(e)}")
                self.connected = False
                self.sock.close()
                self.reconnect_attempt()

    def cmd_callback(self, msg):
        # Read a command
        command = msg.data

        # Send the command to the server
        # self.sock.sendall(bytes(command, 'utf-8'))
        self.send_command(command)

        if command == "get_data":
            data = self.sock.recv(1024).decode('utf-8')
            self.get_logger().info('Khepera IV Driver: sensors: %s' % data)
        if command == "p":
            data = self.sock.recv(1024).decode('utf-8')

    def cmd_vel_callback(self, msg):
        self.target_twist = msg
        command = f"d {msg.linear.x:.3f} {msg.angular.z:.3f}\n"
        self.send_command(command)

    def goalpose_callback(self, msg):
        self.target_pose = msg
        command = f"g {msg.pose.position.x:.3f} {msg.pose.position.y:.3f}\n"
        self.send_command(command)
    
    def time_callback(self,msg):
        self.time = msg.sec + (msg.nanosec/1000000000)
        command = f"t {self.time:.3f}\n"
        self.send_command(command)
    
    def pose_callback(self, msg):
        if not self.init_pose:
            self.pose = msg
            self.init_pose = True
            self.target_pose = self.pose
            [roll, pitch, theta_vicon] = euler_from_quaternion([self.pose.pose.orientation.x, self.pose.pose.orientation.y, self.pose.pose.orientation.z, self.pose.pose.orientation.w])
            self.get_logger().info('Init pose::: X: %s Y: %s Z: %s' % (str(round(msg.pose.position.x,3)), str(round(msg.pose.position.y,3)), str(round(self.theta,3))))
            self.theta = theta_vicon
            command = f"i {msg.pose.position.x:.3f} {msg.pose.position.y:.3f}\n"
            self.send_command(command)
        else:
            time = self.get_clock().now().to_msg()
            delta_t = (time.sec + time.nanosec*1e-9) - self.last_time
            if not self.positioning == "Intern" and delta_t>0.1: # TO-DO
                self.last_time = time.sec + time.nanosec*1e-9
                # if not(np.isnan(msg.position.x) or np.isnan(msg.position.y) or np.isnan(msg.position.z) or np.isnan(msg.orientation.x) or np.isnan(msg.orientation.y) or np.isnan(msg.orientation.z)  or np.isnan(msg.orientation.w)) and not (msg.position.x == 0.0 and msg.position.y == 0.0 and msg.position.z == 0.0):
                delta = sqrt(pow(self.pose.pose.position.x-msg.pose.position.x,2)+pow(self.pose.pose.position.y-msg.pose.position.y,2))
                if self.path_enable:
                    self.path.header.stamp = self.get_clock().now().to_msg()
                    PoseStamp = PoseStamped()
                    PoseStamp.header.frame_id = "map"
                    PoseStamp.pose.position.x = self.pose.pose.position.x
                    PoseStamp.pose.position.y = self.pose.pose.position.y
                    PoseStamp.pose.position.z = self.pose.pose.position.z
                    PoseStamp.pose.orientation.x = self.pose.pose.orientation.x
                    PoseStamp.pose.orientation.y = self.pose.pose.orientation.y
                    PoseStamp.pose.orientation.z = self.pose.pose.orientation.z
                    PoseStamp.pose.orientation.w = self.pose.pose.orientation.w
                    PoseStamp.header.stamp = self.get_clock().now().to_msg()
                    self.path.poses.append(PoseStamp)
                    self.path_pub.publish(self.path)

                if (np.linalg.norm(delta)>self.threshold and np.linalg.norm(delta)<0.2) or self.communication:
                    self.get_logger().debug('Delta %.3f' % np.linalg.norm(delta))
                    self.pose = msg
                    self.pose.pose.position.z = 0.00
                    #command = "i " + str(round(msg.pose.position.x,3)) + " " + str(round(msg.pose.position.y,3))+ " " + str(round(self.theta,3))
                    command = f"i {msg.pose.position.x:.3f} {msg.pose.position.x:.3f}\n"
                    self.send_command(command)
                    [roll, pitch, theta_vicon] = euler_from_quaternion([self.pose.pose.orientation.x, self.pose.pose.orientation.y, self.pose.pose.orientation.z, self.pose.pose.orientation.w])
                    if ((theta_vicon-self.theta) < 0.3) or abs(theta_vicon-self.theta)>4.6:
                        self.theta = theta_vicon
                    q = quaternion_from_euler(0.0, 0.0, self.theta)
                    self.pose.pose.orientation.x = q[0]
                    self.pose.pose.orientation.y = q[1]
                    self.pose.pose.orientation.z = q[2]
                    self.pose.pose.orientation.w = q[3]
                            
                    self.pose_pub.publish(self.pose)
                    t_base = TransformStamped()
                    t_base.header.stamp = self.get_clock().now().to_msg()
                    t_base.header.frame_id = 'map'
                    t_base.child_frame_id = self.id+'/base_link'
                    t_base.transform.translation.x = self.pose.pose.position.x
                    t_base.transform.translation.y = self.pose.pose.position.y
                    t_base.transform.translation.z = self.pose.pose.position.z
                    t_base.transform.rotation.x = self.pose.pose.orientation.x
                    t_base.transform.rotation.y = self.pose.pose.orientation.y
                    t_base.transform.rotation.z = self.pose.pose.orientation.z
                    t_base.transform.rotation.w = self.pose.pose.orientation.w
                    self.tfbr.sendTransform(t_base)

                    self.get_logger().debug('Pose X: %.3f Y: %.3f Yaw: %.3f theta_vicon %.3f' % (self.pose.pose.position.x, self.pose.pose.position.y, self.theta, self.theta_vicon))

    def get_pose(self):
        if self.init_pose and self.connected:
            try:
                self.send_command("p")
                data = self.sock.recv(1024).decode('utf-8').strip()
                if data:
                    x, y, theta = map(float, data.split(','))
                    # self.update_pose(x, y, theta)
            except Exception as e:
                self.get_logger().warn(f"Error obteniendo pose: {str(e)}")
                self.connected = False
            try:
                delta = sqrt((pow(self.pose.pose.position.x-x,2)+pow(self.pose.pose.position.y-y,2)))
                self.get_logger().debug('X: %.2f Y: %.2f x: %.2f y: %.2f DELTA: %.2f' % (self.pose.pose.position.x, self.pose.pose.position.y, x, y, delta))
                if (self.communication or delta>self.threshold) and delta<0.2:
                    self.pose.pose.position = Point(x=x, y=y, z=0.0)
                    q = quaternion_from_euler(0, 0, theta)
                    self.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
                    self.pose.header.stamp = self.get_clock().now().to_msg()
                    # self.pose.pose.orientation.z = np.sin(theta/2)
                    # self.pose.pose.orientation.w = np.cos(theta/2)
                    self.get_logger().debug('Xp: %.3f, Yp: %.3f' % (self.pose.pose.position.x, self.pose.pose.position.y))
                    self.pose_pub.publish(self.pose)
                    
                    t_base = TransformStamped()
                    t_base.header = self.pose.header
                    t_base.child_frame_id = self.id+'/base_link'
                    t_base.transform.translation.x = self.pose.pose.position.x
                    t_base.transform.translation.y = self.pose.pose.position.y
                    t_base.transform.translation.z = 0.0
                    t_base.transform.rotation = self.pose.pose.orientation
                    self.tfbr.sendTransform(t_base)

                    if self.path_enable:
                        self.path.header.stamp = self.get_clock().now().to_msg()
                        self.path.poses.append(self.pose)
                        # Limitar tamaño para eficiencia
                        if len(self.path.poses) > self.max_path_length:
                            self.path.poses.pop(0)  # Eliminar el punto más antiguo
                        self.path_pub.publish(self.path)
            except:
                pass

            # TO-DO
            # if self.neighbour_update and self.onboard:
            #     self.neighbour_update = False
            #     for agent in self.agent_list:
            #         command = "m " + str(agent.idn) + " " + str(round(agent.pose.pose.position.x,3)) + " " + str(round(agent.pose.pose.position.y,3)) + " " + str(round(agent.pose.pose.position.z,3))
            #         self.sock.sendall(bytes(command, 'utf-8'))
            #         time.sleep(0.05)
       
    def order_callback(self, msg):
        self.get_logger().debug('Order: "%s"' % msg.data)
        if msg.data == 'formation_run' and self.onboard:
            if self.config['task']['enable']:
                self.formation_bool = True
                if self.onboard:
                    command = b"f \n"
                    self.send_command(command)
        elif msg.data == 'formation_stop':
            self.formation_bool = False
            if self.onboard:
                command = b"f \n"
                self.send_command(command)
        elif msg.data == 'rele':
            if self.rele:
                self.rele = False
            else:
                self.rele = True
            command = b"r \n"
            self.send_command(command)
        else:
            self.get_logger().error('"%s": Unknown order' % (msg.data))
    
    def get_data(self):
        command = 'get_data'
        try:
            self.sock.sendall(bytes(command, 'utf-8'))

            data = self.sock.recv(1024).decode('utf-8')
            self.get_logger().info('Khepera IV Driver: sensors: %s' % data)
        except:
            self.get_logger().error('Fail get_data()')
            pass

    def get_imu(self):
        if self.connected:
            try:
                self.send_command("s")
                data = self.sock.recv(1024).decode('utf-8').strip()
                if data:
                    ax, ay, az, wx, wy, wz = map(float, data.split(','))
                msg = Imu()
                msg.linear_acceleration = Vector3(x=ax, y=ay, z=az)
                msg.angular_velocity = Vector3(x=wx, y=wy, z=wz)
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.id+'/base_link'
                self.imu_pub.publish(msg)
                # self.get_logger().info('Sensors: %s' % data)
            except Exception as e:
                self.get_logger().warn(f"Error obteniendo imu: {str(e)}")
                self.connected = False

    def get_us(self):
        if self.connected and False:
            try:
                self.send_command("u")
                data = self.sock.recv(1024).decode('utf-8').strip()
                if data:
                    ax, ay, az, wx, wy, wz = map(float, data.split(','))
                msg = Imu()
                msg.linear_acceleration = Vector3(x=ax, y=ay, z=az)
                msg.angular_velocity = Vector3(x=wx, y=wy, z=wz)
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.id+'/base_link'
                self.imu_pub.publish(msg)
                self.get_logger().info('Sensors: %s' % data)
            except Exception as e:
                self.get_logger().warn(f"Error obteniendo US: {str(e)}")
                self.connected = False

    ###############
    #    Tasks    # TO-DO
    ###############
    def distance_gradient_controller(self):
        if self.formation_bool:
            dx = dy = dz = 0
            for agent in self.agent_list:
                if not agent.id.find("line") == -1:
                    nearest = PoseStamped()
                    nearest.header.frame_id = "map"
                    gamma = -np.dot([agent.point.x-self.pose.pose.position.x, agent.point.y-self.pose.pose.position.y, agent.point.z-self.pose.pose.position.z],[agent.vector.x, agent.vector.y, agent.vector.z])/agent.mod
                    nearest.pose.position.x = agent.point.x + gamma * agent.vector.x
                    nearest.pose.position.y = agent.point.y + gamma * agent.vector.y
                    nearest.pose.position.z = agent.point.z + gamma * agent.vector.z
                    agent.gtpose_callback(nearest)
                error_x = self.pose.pose.position.x - agent.pose.position.x
                error_y = self.pose.pose.position.y - agent.pose.position.y
                error_z = self.pose.pose.position.z - agent.pose.position.z
                distance = pow(error_x,2)+pow(error_y,2)+pow(error_z,2)
                d = sqrt(distance)
                dx += self.k * agent.k * (pow(agent.d,2) - distance) * error_x/d
                dy += self.k * agent.k * (pow(agent.d,2) - distance) * error_y/d
                dz += self.k * agent.k * (pow(agent.d,2) - distance) * error_z/d
                
                msg_data = Float64()
                msg_data.data = abs(agent.d - d)
                agent.publisher_data_.publish(msg_data)
                self.get_logger().debug('Agent %s: D: %.2f dx: %.2f dy: %.2f dz: %.2f ' % (agent.id, msg_data.data, dx, dy, dz)) 
            
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

            self.target_pose.pose.position.x = self.pose.pose.position.x + dx
            self.target_pose.pose.position.y = self.pose.pose.position.y + dy
            self.target_pose.pose.position.z = self.pose.pose.position.z
        
            delta=sqrt(pow(dx,2)+pow(dy,2)+pow(dz,2))
            angles = euler_from_quaternion((self.pose.pose.orientation.x, self.pose.pose.orientation.y, self.pose.pose.orientation.z, self.pose.pose.orientation.w))
                    
            if delta<0.05:
                roll = angles[0]
                pitch = angles[1]
                yaw = angles[2]
            else:
                h = sqrt(pow(dx,2)+pow(dy,2))
                roll = 0.0
                pitch = -atan2(dz,h)
                yaw = atan2(dy,dx)

            q = quaternion_from_euler(roll, pitch, yaw)
            self.target_pose.pose.orientation.x = q[0]
            self.target_pose.pose.orientation.y = q[1]
            self.target_pose.pose.orientation.z = q[2]
            self.target_pose.pose.orientation.w = q[3]
            self.target_pose.header.stamp = self.get_clock().now().to_msg()
            self.publisher_goalpose.publish(self.target_pose)

    def distance_pid_controller(self):
        if self.formation_bool:
            ex = ey = ez = 0
            for agent in self.agent_list:
                if not agent.id.find("line") == -1:
                    nearest = PoseStamped()
                    nearest.header.frame_id = "map"
                    gamma = -np.dot([agent.point.x-self.pose.pose.position.x, agent.point.y-self.pose.pose.position.y, agent.point.z-self.pose.pose.position.z],[agent.vector.x, agent.vector.y, agent.vector.z])/agent.mod
                    nearest.pose.position.x = agent.point.x + gamma * agent.vector.x
                    nearest.pose.position.y = agent.point.y + gamma * agent.vector.y
                    nearest.pose.position.z = agent.point.z + gamma * agent.vector.z
                    agent.gtpose_callback(nearest)
                error_x = self.pose.pose.position.x - agent.pose.position.x
                error_y = self.pose.pose.position.y - agent.pose.position.y
                error_z = self.pose.pose.position.z - agent.pose.position.z
                distance = sqrt(pow(error_x,2)+pow(error_y,2)+pow(error_z,2))
                e = agent.d - distance
                ex += agent.k * e * (error_x/distance)
                ey += agent.k * e * (error_y/distance)
                ez += agent.k * e * (error_z/distance)
                
                msg_data = Float64()
                msg_data.data = e
                agent.publisher_data_.publish(msg_data)

            aux = self.get_clock().now().to_msg()
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

            self.target_pose.pose.position.x = self.pose.pose.position.x + dx
            self.target_pose.pose.position.y = self.pose.pose.position.y + dy
            self.target_pose.pose.position.z = self.pose.pose.position.z + dz

            delta=sqrt(pow(dx,2)+pow(dy,2)+pow(dz,2))
            angles = euler_from_quaternion((self.pose.pose.orientation.x, self.pose.pose.orientation.y, self.pose.pose.orientation.z, self.pose.pose.orientation.w))
                    
            if delta<0.05:
                roll = angles[0]
                pitch = angles[1]
                yaw = angles[2]
            else:
                h = sqrt(pow(dx,2)+pow(dy,2))
                roll = 0.0
                pitch = -atan2(dz,h)
                yaw = atan2(dy,dx)

            q = quaternion_from_euler(roll, pitch, yaw)
            self.target_pose.pose.orientation.x = q[0]
            self.target_pose.pose.orientation.y = q[1]
            self.target_pose.pose.orientation.z = q[2]
            self.target_pose.pose.orientation.w = q[3]
            self.target_pose.header.stamp = self.get_clock().now().to_msg()
            self.publisher_goalpose.publish(self.target_pose)

    def pose_gradient_controller(self):
        if self.formation_bool:
            dx = dy = dz = 0
            for agent in self.agent_list:
                error_x = self.pose.pose.position.x - agent.pose.position.x
                error_y = self.pose.pose.position.y - agent.pose.position.y
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

            self.target_pose.pose.position.x = self.pose.pose.position.x + dx
            self.target_pose.pose.position.y = self.pose.pose.position.y + dy
            self.target_pose.pose.position.z = self.pose.pose.position.z
        
            delta=sqrt(pow(dx,2)+pow(dy,2)+pow(dz,2))
            angles = euler_from_quaternion((self.pose.pose.orientation.x, self.pose.pose.orientation.y, self.pose.pose.orientation.z, self.pose.pose.orientation.w))
                    
            if delta<0.05:
                roll = angles[0]
                pitch = angles[1]
                yaw = angles[2]
            else:
                h = sqrt(pow(dx,2)+pow(dy,2))
                roll = 0.0
                pitch = -atan2(dz,h)
                yaw = atan2(dy,dx)

            q = quaternion_from_euler(roll, pitch, yaw)
            self.target_pose.pose.orientation.x = q[0]
            self.target_pose.pose.orientation.y = q[1]
            self.target_pose.pose.orientation.z = q[2]
            self.target_pose.pose.orientation.w = q[3]
            self.target_pose.header.stamp = self.get_clock().now().to_msg()
            self.publisher_goalpose.publish(self.target_pose)

    def pose_pid_controller(self):
        if self.formation_bool:
            ex = ey = ez = 0
            for agent in self.agent_list:
                error_x = agent.x - (self.pose.pose.position.x - agent.pose.position.x)
                error_y = agent.y - (self.pose.pose.position.y - agent.pose.position.y)
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

            self.target_pose.pose.position.x = self.pose.pose.position.x + dx
            self.target_pose.pose.position.y = self.pose.pose.position.y + dy

            delta=sqrt(pow(dx,2)+pow(dy,2)+pow(dz,2))
            angles = euler_from_quaternion((self.pose.pose.orientation.x, self.pose.pose.orientation.y, self.pose.pose.orientation.z, self.pose.pose.orientation.w))
                    
            if delta<0.05:
                roll = angles[0]
                pitch = angles[1]
                yaw = angles[2]
            else:
                h = sqrt(pow(dx,2)+pow(dy,2))
                roll = 0.0
                pitch = -atan2(dz,h)
                yaw = atan2(dy,dx)

            q = quaternion_from_euler(roll, pitch, yaw)
            self.target_pose.pose.orientation.x = q[0]
            self.target_pose.pose.orientation.y = q[1]
            self.target_pose.pose.orientation.z = q[2]
            self.target_pose.pose.orientation.w = q[3]
            self.target_pose.header.stamp = self.get_clock().now().to_msg()
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
    
    def task_formation_info(self):
        if self.formation_bool:
            for agent in self.agent_list:
                if agent.id == 'origin':
                    distance = sqrt((pow(self.pose.pose.position.x,2)+pow(self.pose.pose.position.y,2)+pow(self.pose.pose.position.z,2)))
                else:
                    distance = sqrt(pow(self.pose.pose.position.x-agent.pose.position.x,2)+pow(self.pose.pose.position.y-agent.pose.position.y,2)+pow(self.pose.pose.position.z-agent.pose.position.z,2))
                msg_data = Float64()
                msg_data.data = abs(agent.d - distance)
                agent.publisher_data_.publish(msg_data)


def main(args=None):
    rclpy.init(args=args)
    khepera_driver = KheperaIVDriver()

    try:
        rclpy.spin(khepera_driver)
    except KeyboardInterrupt:
        pass
    finally:
        khepera_driver.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()
