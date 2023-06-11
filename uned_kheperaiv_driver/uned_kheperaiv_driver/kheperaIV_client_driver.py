import rclpy
import socket
import numpy as np
import yaml

from rclpy.node import Node
from std_msgs.msg import String, UInt16, UInt16MultiArray, Float64, Float64MultiArray
from geometry_msgs.msg import Twist, Pose, TransformStamped, Point
from visualization_msgs.msg import Marker
from math import sqrt, cos, sin, atan2
from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion

class Agent():
    def __init__(self, parent, id, x = None, y = None, z = None, d = None):
        self.id = id
        self.distance = False
        self.parent = parent
        self.k = 1.0
        if d == None:
            self.x = x
            self.y = y
            self.z = z
            self.parent.get_logger().info('Agent: %s' % self.str_())
        else:
            self.d = d
            self.distance = True
            self.parent.get_logger().info('Agent: %s' % self.str_distance_())
        self.pose = Pose()
        if self.id == 'origin':
            self.pose.position.x = 0.0
            self.pose.position.y = 0.0
            self.pose.position.z = 0.0
        else:
            self.sub_pose_ = self.parent.create_subscription(Pose, '/' + self.id + '/local_pose', self.gtpose_callback, 10)
        if self.parent.config['task']['Onboard']:
            self.parent.get_logger().info('TO DO')
        self.sub_d_ = self.parent.create_subscription(Float64, '/' + self.id + '/d', self.d_callback, 10)
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
        self.pose = msg
        if self.parent.config['task']['Onboard']:
            self.parent.get_logger().debug('Update Neighbour pose')
        self.parent.get_logger().debug('Agent: X: %.2f Y: %.2f Z: %.2f' % (msg.position.x, msg.position.y, msg.position.z))

        line = Marker()
        p0 = Point()
        p0.x = self.parent.pose.position.x
        p0.y = self.parent.pose.position.y
        p0.z = self.parent.pose.position.z

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
            if abs(distance - self.d) > 0.05:
                line.color.r = 1.0
            else:
                if abs(distance - self.d) > 0.025:
                    line.color.r = 1.0
                    line.color.g = 0.5
                else:
                    line.color.g = 1.0
        else:
            dx = p0.x-p1.x
            dy = p0.y-p1.y
            dz = p0.z-p1.z
            if abs(dx) > 0.05 or abs(dy) > 0.05 or abs(dz) > 0.05:
                line.color.r = 1.0
            else:
                if abs(dx) > 0.025 or abs(dy) > 0.025 or abs(dz) > 0.025:
                    line.color.r = 1.0
                    line.color.g = 0.5
                else:
                    line.color.g = 1.0
        line.color.a = 1.0
        line.points.append(p1)
        line.points.append(p0)

        self.publisher_marker_.publish(line)


class KheperaIVDriver(Node):
    def __init__(self):
        super().__init__('driver')

        ## ROS2 Environment
        # Params
        self.declare_parameter('config', 'file_path.yaml')
        self.declare_parameter('id', 'khepera01')

        # Publisher
        self.publisher_status = self.create_publisher(String,'status', 10)
        self.pub_pose_ = self.create_publisher(Pose,'local_pose', 10)
        # Subscription
        self.create_subscription(Pose, 'pose', self.pose_callback, 1)
        self.create_subscription(Pose, 'goal_pose', self.goalpose_callback, 10)
        self.create_subscription(String, 'cmd', self.cmd_callback, 1)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 1)
        self.create_subscription(String, '/swarm/status', self.order_callback, 10)
        self.create_subscription(String, '/swarm/order', self.order_callback, 1)

        self.initialize()

        # Variables
        self.tfbr = TransformBroadcaster(self)

        self.timer_task = self.create_timer(0.1, self.get_pose)
        # self.timer_sensor = self.create_timer(0.5, self.get_data)
        # self.timer_iterate = self.create_timer(1.0, self.iterate)

    def initialize(self):
        self.get_logger().info('KheperaIVDriver::inicialize() ok.')
        self.tfbr = TransformBroadcaster(self)
        self.distance_formation_bool = False
        self.first_goal_pose = False
        # Read Params
        self.id = self.get_parameter('id').get_parameter_value().string_value
        config_file = self.get_parameter('config').get_parameter_value().string_value
        with open(config_file, 'r') as file:
            documents = yaml.safe_load(file)
            
        config = documents[self.id]
        robot_port = config['port_number']
        self.get_logger().info('KheperaIVDriver::Port %s.' % str(robot_port))
        self.pose = Pose()
        self.init_pose = False
        self.pose.position.x = config['pose']['x']
        self.pose.position.y = config['pose']['y']
        
        robot_ip = config['agent_ip']
         
        self.theta = config['init_theta']
        self.theta_vicon = self.theta

        self.communication = (config['communication']['type'] == 'Continuous')
        if not self.communication:
            self.threshold = config['communication']['threshold']['co']
        else:
            self.threshold = 0.01
        
        # Open a socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(2)

        # Set the server address structure
        server_address = (robot_ip, robot_port)

        self.get_logger().info('KheperaIVDriver::IP %s.' % robot_ip)
        
        self.get_logger().info('KheperaIVDriver::Yaw %f.' % self.theta)
        # Connect to the server
        self.sock.connect(server_address)

        # Formation Init
        if config['task']['enable']:
            self.get_logger().info('Task %s' % config['task']['type'])
            self.agent_list = list()
            aux = config['task']['relationship']
            self.relationship = aux.split(', ')
            if config['task']['type'] == 'distance':
                if self.config['task']['Onboard']:
                    self.timer_task = self.parent.create_timer(self.config['task']['T']/1000, self.task_formation_info)
                else:
                    self.timer_task = self.parent.create_timer(self.config['task']['T']/1000, self.task_formation_distance)
                
                for rel in self.relationship:
                    aux = rel.split('_')
                    robot = Agent(self, aux[0], d = float(aux[1]))
                    self.agent_list.append(robot)
            if config['task']['type'] == 'relative_pose':
                self.timer_task = self.create_timer(0.1, self.task_formation_pose)
                for rel in self.relationship:
                    aux = rel.split('_')
                    rel_pose = aux[1].split('/')
                    robot = Agent(self, aux[0], x = float(rel_pose[0]), y = float(rel_pose[1]), z = float(rel_pose[2]))
                    self.agent_list.append(robot)

        self.pose_callback(self.pose)

    def cmd_callback(self, msg):
        # Read a command
        command = msg.data

        # Send the command to the server
        self.sock.sendall(bytes(command, 'utf-8'))

        if command == "get_data":
            data = self.sock.recv(1024).decode('utf-8')
            self.get_logger().info('Khepera IV Driver: sensors: %s' % data)

    def cmd_vel_callback(self, msg):
        command = "d " + str(round(msg.linear.x,3)) + " " + str(round(msg.angular.z,3))
        self.sock.sendall(bytes(command, 'utf-8'))

    def order_callback(self, msg):
        self.get_logger().info('KheperaIVDriver::New Order')
        if msg.data == 'distance_formation_run':
            self.distance_formation_bool = True

    def pose_callback(self, msg):
        if abs(msg.position.x)<2 and abs(msg.position.y)<2:
            if not self.init_pose:
                self.pose = msg
                self.init_pose = True
                self.goal_pose = self.pose
                command = "i " + str(round(msg.position.x,3)) + " " + str(round(msg.position.y,3))+ " " + str(round(self.theta,3))
                self.sock.sendall(bytes(command, 'utf-8'))
            else:
                if not(np.isnan(msg.position.x) or np.isnan(msg.position.y) or np.isnan(msg.position.z) or np.isnan(msg.orientation.x) or np.isnan(msg.orientation.y) or np.isnan(msg.orientation.z)  or np.isnan(msg.orientation.w)) and not (msg.position.x == 0.0 and msg.position.y == 0.0 and msg.position.z == 0.0):
                    delta = np.array([self.pose.position.x-msg.position.x,self.pose.position.y-msg.position.y,self.pose.position.z-msg.position.z])
                    
                    if (np.linalg.norm(delta)>self.threshold and np.linalg.norm(delta)<0.2) or self.distance_formation_bool or self.communication:
                        if self.distance_formation_bool:
                            self.distance_formation_bool = False
                        self.pose = msg
                        self.get_logger().debug('Delta %.3f' % np.linalg.norm(delta))
                        self.get_logger().debug('New Local pose')
                        self.pose = msg
                        self.pose.position.z = 0.00
                        [roll, pitch, theta_vicon] = euler_from_quaternion([self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w])
                        if ((theta_vicon-self.theta) < 0.3) or abs(theta_vicon-self.theta)>4.6:
                            self.theta = theta_vicon
                        self.pose.orientation.x = 0.0
                        self.pose.orientation.y = 0.0
                        self.pose.orientation.z = np.sin(self.theta /2)
                        self.pose.orientation.w = np.cos(self.theta /2)
                        
                        self.pub_pose_.publish(self.pose)
                        t_base = TransformStamped()
                        t_base.header.stamp = self.get_clock().now().to_msg()
                        t_base.header.frame_id = 'map'
                        t_base.child_frame_id = self.id
                        t_base.transform.translation.x = self.pose.position.x
                        t_base.transform.translation.y = self.pose.position.y
                        t_base.transform.translation.z = self.pose.position.z
                        t_base.transform.rotation.x = self.pose.orientation.x
                        t_base.transform.rotation.y = self.pose.orientation.y
                        t_base.transform.rotation.z = self.pose.orientation.z
                        t_base.transform.rotation.w = self.pose.orientation.w
                        self.tfbr.sendTransform(t_base)
                        
                        self.get_logger().debug('Pose X: %.3f Y: %.3f Yaw: %.3f theta_vicon %.3f' % (self.pose.position.x, self.pose.position.y, self.theta, self.theta_vicon))

    def goalpose_callback(self, msg):
        if not self.first_goal_pose:
            self.first_goal_pose = True
        self.get_logger().debug('New Goal pose: %.2f, %.2f' % (msg.position.x, msg.position.y))
        command = "g " + str(round(msg.position.x,3)) + " " + str(round(msg.position.y,3))
        self.sock.sendall(bytes(command, 'utf-8'))
        self.goal_pose = msg
    
    def get_pose(self):
        if self.init_pose:
            # Read a command
            command = 'p'
            # Send the command to the server
            try:
                self.sock.sendall(bytes(command, 'utf-8'))

                data = self.sock.recv(1024).decode('utf-8')
                value = data.split(',')
                try:
                    '''
                    d = float(value[0])
                    self.theta = float(value[1])
                    if self.init_pose:
                        self.theta = float(value[1])
                    self.pose.position.x += d * cos(self.theta)
                    self.pose.position.y += d * sin(self.theta)
                    self.pose.orientation.x = 0.0
                    self.pose.orientation.y = 0.0
                    self.pose.orientation.z = np.sin(self.theta/2)
                    self.pose.orientation.w = np.cos(self.theta/2)
                    '''
                    self.pose.position.x = float(value[0])
                    self.pose.position.y = float(value[1])
                    self.pose.orientation.x = 0.0
                    self.pose.orientation.y = 0.0
                    self.pose.orientation.z = np.sin(float(value[2])/2)
                    self.pose.orientation.w = np.cos(float(value[2])/2)
                    self.get_logger().debug('Xp: %.3f, Yp: %.3f' % (self.pose.position.x, self.pose.position.y))
                    self.pub_pose_.publish(self.pose)
                    t_base = TransformStamped()
                    t_base.header.stamp = self.get_clock().now().to_msg()
                    t_base.header.frame_id = 'map'
                    t_base.child_frame_id = self.id+'/base_link'
                    t_base.transform.translation.x = self.pose.position.x
                    t_base.transform.translation.y = self.pose.position.y
                    t_base.transform.translation.z = 0.05
                    t_base.transform.rotation.x = 0.0
                    t_base.transform.rotation.y = 0.0
                    t_base.transform.rotation.z = self.pose.orientation.z
                    t_base.transform.rotation.w = self.pose.orientation.w 
                    self.tfbr.sendTransform(t_base)
                except:
                    pass
            except:
                self.get_logger().error('Fail get_pose()')
                pass
        
    def get_data(self):
        command = 'get_data'
        try:
            self.sock.sendall(bytes(command, 'utf-8'))

            data = self.sock.recv(1024).decode('utf-8')
            self.get_logger().info('Khepera IV Driver: sensors: %s' % data)
        except:
            self.get_logger().error('Fail get_data()')
            pass

    def iterate(self):
        command = "i " + str(round(self.pose.position.x,3)) + " " + str(round(self.pose.position.y,3))+ " " + str(round(self.theta ,3))
        self.sock.sendall(bytes(command, 'utf-8'))

    ###############
    #    Tasks    #
    ###############
    def task_formation_distance(self):
        if self.distance_formation_bool:
            # self.distance_formation_bool = False
            dx = dy = 0
            target_pose = Pose()
            for agent in self.agent_list:
                # self.parent.get_logger().info('Agent: %s' % agent.id)
                error_x = self.pose.position.x - agent.pose.position.x
                error_y = self.pose.position.y - agent.pose.position.y
                error_z = self.pose.position.z - agent.pose.position.z
                distance = pow(error_x,2)+pow(error_y,2)+pow(error_z,2)
                if agent.id == 'origin':
                    dx += 2 * (error_x * self.pose.position.x)
                    dy += 2 * (error_y * self.pose.position.y)
                else:
                    dx += (pow(agent.d,2) - distance) * error_x
                    dy += (pow(agent.d,2) - distance) * error_y

                msg_data = Float64()
                msg_data.data = abs(agent.d - sqrt(distance))
                agent.publisher_data_.publish(msg_data)


            if dx > 1.0:
                dx = 1.0
            if dx < -1.0:
                dx = -1.0
            if dy > 1.0:
                dy = 1.0
            if dy < -1.0:
                dy = -1.0
   
            target_pose.position.x = self.pose.position.x + dx/4
            target_pose.position.y = self.pose.position.y + dy/4
            
            # self.parent.get_logger().debug('Formation: X: %.2f->%.2f Y: %.2f->%.2f Z: %.2f->%.2f' % (self.pose.position.x, target_pose.position.x, self.pose.position.y, target_pose.position.y, self.pose.position.z, target_pose.position.z)) 

            self.goalpose_callback(target_pose)

            # self.parent.get_logger().debug('Distance: %.4f eX: %.2f eY: %.2f eZ: %.2f' % (sqrt(distance), error_x, error_y, error_z))
            # self.parent.get_logger().debug('Delta: %.4f X: %.2f Y: %.2f Z: %.2f' % (delta, dx, dy, dz))
            # self.parent.get_logger().debug('Target: X: %.2f Y: %.2f Z: %.2f' % (target_pose.position.x, target_pose.position.y, target_pose.position.z))

    def task_formation_pose(self):
        if self.ready:
            # TO-DO
            dx = dy = dz = 0


    def position_controller(self):
        Kp = 10
        Kw = 1
        Vmax = 10
        Wmax = 2.5/2

        d = sqrt(pow(self.goal_pose.position.x-self.pose.position.x,2)+pow(self.goal_pose.position.y-self.pose.position.y,2))

        # Attractive force
        error_x = (self.goal_pose.position.x-self.pose.position.x)*cos(self.theta)+(self.goal_pose.position.y-self.pose.position.y)*sin(self.theta)
        error_y = -(self.goal_pose.position.x-self.pose.position.x)*sin(self.theta)+(self.goal_pose.position.y-self.pose.position.y)*cos(self.theta)

        alfa = atan2(self.goal_pose.position.y-self.pose.position.y,self.goal_pose.position.x-self.pose.position.x)
        # [roll, pitch, self.theta] = euler_from_quaternion([self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w])
        oc = alfa - self.theta
        w = Kw * sin(oc)
        w = 0.0
        v = error_x * Kp
        '''
        if v > 0:
            v = v + 0.2
        if v < 0:
            v = v - 0.2
        '''
        if abs(d)<0.001:
            v = 0.0
            w = 0.0
        else:
            self.get_logger().info('alfa: %.2f theta: %.2f oc: %.2f e_x: %.3f e_y: %.3f' % (alfa, self.theta, oc, error_x, error_y))

        
        ## Cmd_Vel
        out = Twist()
        out.linear.x = v
        out.angular.z = w

        return out 

def main(args=None):
    rclpy.init(args=args)
    khepera_driver = KheperaIVDriver()
    rclpy.spin(khepera_driver)

    khepera_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
