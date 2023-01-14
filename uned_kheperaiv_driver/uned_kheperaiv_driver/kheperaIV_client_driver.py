import rclpy
import socket

from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose, TransformStamped
from math import sqrt, cos, sin, atan2
from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion

class KheperaIVDriver(Node):
    def __init__(self):
        super().__init__('khepera_iv_driver')

        ## ROS2 Environment
        # Params
        self.declare_parameter('agent_ip', '192.168.0.21')
        self.declare_parameter('port_number', 50000)
        self.declare_parameter('id', 'khepera01')
        # Publisher
        self.publisher_status = self.create_publisher(String,'status', 10)
        self.pub_pose_ = self.create_publisher(Pose,'local_pose', 10)
        # Subscription
        self.create_subscription(Pose, 'pose', self.pose_callback, 1)
        self.create_subscription(Pose, 'goal_pose', self.goalpose_callback, 10)
        self.create_subscription(String, 'cmd', self.cmd_callback, 1)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 1)

        self.initialize()

        # self.timer_task = self.create_timer(0.5, self.get_pose)
        self.timer_iterate = self.create_timer(0.01, self.iterate)

    def initialize(self):
        self.get_logger().info('KheperaIVDriver::inicialize() ok.')
        self.tfbr = TransformBroadcaster(self)
        self.init_pose = False
        self.pose = Pose()
        self.pose.position.x = 0.0
        self.pose.position.y = 0.0
        self.theta = 0.0
        # Read Params
        robot_ip = self.get_parameter('agent_ip').get_parameter_value().string_value
        robot_port = 50000 # self.get_parameter('port_number').get_parameter_value().integer_value
        self.id = self.get_parameter('id').get_parameter_value().string_value

        # Open a socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Set the server address structure
        server_address = (robot_ip, robot_port)

        self.get_logger().info('KheperaIVDriver::test() IP %s.' % robot_ip)
        self.get_logger().info('KheperaIVDriver::test() Port %s.' % robot_port)
        # Connect to the server
        self.sock.connect(server_address)

    def cmd_callback(self, msg):
        # Read a command
        command = msg.data

        # Send the command to the server
        self.sock.sendall(bytes(command, 'utf-8'))

        if command == "get_data":
            data = self.sock.recv(1024).decode('utf-8')
            self.get_logger().info('Khepera IV Driver: sensors: %s' % data)

    def cmd_vel_callback(self, msg):
        vel = msg.linear.x * 100
        w = msg.angular.z * 100
        command = "d " + str(round(msg.linear.x,3)) + " " + str(round(msg.angular.z,3))
        self.sock.sendall(bytes(command, 'utf-8'))

    def pose_callback(self, msg):
        self.pose = msg
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
        
        self.get_logger().debug('Pose X: %f Y: %f Yaw: %f' % (self.pose.position.x, self.pose.position.y, self.theta))
        if not self.init_pose:
            self.init_pose = True
            self.goal_pose = self.pose

    def goalpose_callback(self, msg):
        self.get_logger().debug('New Goal pose: %.2f, %.2f' % (msg.position.x, msg.position.y))
        self.goal_pose = msg
    
    def get_pose(self):
        '''
        # Read a command
        # command = 'p'
        # Send the command to the server
        # self.sock.sendall(bytes(command, 'utf-8'))

        # data = self.sock.recv(1024).decode('utf-8')
        # value = data.split(',')
        try:
            d = float(value[0])
            self.theta += float(value[1])
            self.pose.position.x += d * cos(self.theta)
            self.pose.position.y += d * sin(self.theta)
            self.pub_pose_.publish(self.pose)
        except:
            pass
        '''

    def iterate(self):
        if self.init_pose:
            cmd_vel = Twist()
            cmd_vel = self.position_controller()
            self.get_logger().debug('Pose Vx: %s W: %s' % (str(round(cmd_vel.linear.x,3)), str(round(cmd_vel.angular.z,3))))
            # cmd_vel.angular.z = 0.0
            command = "d " + str(round(cmd_vel.linear.x,4)) + " " + str(round(cmd_vel.angular.z,4))
            self.sock.sendall(bytes(command, 'utf-8'))


    def position_controller(self):
        Kp = 15
        Kw = 1
        Vmax = 10
        Wmax = 2.5/2

        d = sqrt(pow(self.goal_pose.position.x-self.pose.position.x,2)+pow(self.goal_pose.position.y-self.pose.position.y,2))

        # Attractive force
        error_x = (self.goal_pose.position.x-self.pose.position.x)*cos(self.theta)+(self.goal_pose.position.y-self.pose.position.y)*sin(self.theta)
        error_y = -(self.goal_pose.position.x-self.pose.position.x)*sin(self.theta)+(self.goal_pose.position.y-self.pose.position.y)*cos(self.theta)

        alfa = atan2(self.goal_pose.position.y-self.pose.position.y,self.goal_pose.position.x-self.pose.position.x)
        [roll, pitch, self.theta] = euler_from_quaternion([self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w])
        oc = alfa - self.theta
        w = Kw * sin(oc)

        v = error_x * Kp

        if v > 0:
            v = v + 0.2
        if v < 0:
            v = v - 0.2

        if abs(d)<0.001:
            v = 0.0
            w = 0.0
        else:
            self.get_logger().debug('alfa: %.2f theta: %.2f oc: %.2f e_x: %.3f e_y: %.3f' % (alfa, self.theta, oc, error_x, error_y))

        
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
