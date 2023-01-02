import rclpy
import socket

from rclpy.node import Node
from std_msgs.msg import String, UInt16, Float64

class KheperaIVDriver(Node):
    def __init__(self):
        super().__init__('khepera_iv_driver')

        ## ROS2 Environment
        # Params
        self.declare_parameter('agent_ip', '192.168.0.21')
        self.declare_parameter('port_number', '50000')
        # Publisher
        self.publisher_status = self.create_publisher(String,'/status', 10)
        # Subscription
        self.create_subscription(String, '/cmd', self.cmd_callback, 1)

        self.initialize()

    def initialize(self):
        self.get_logger().info('KheperaIVDriver::inicialize() ok.')
        # Read Params
        robot_ip = self.get_parameter('agent_ip').get_parameter_value().string_value
        robot_port = 50000 # self.get_parameter('port_number').get_parameter_value().integer_value

        # Open a socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Set the server address structure
        server_address = (robot_ip, robot_port)

        self.get_logger().info('KheperaIVDriver::test() IP %s.' % robot_ip)
        self.get_logger().info('KheperaIVDriver::test() Port %s.' % robot_port)
        # Connect to the server
        self.sock.connect(server_address)

    def cmd_callback(self, msg):
        self.get_logger().info('Khepera IV Driver: cmd_callback() In progress')
        # Read a command
        command = msg.data

        # Send the command to the server
        self.sock.sendall(bytes(command, 'utf-8'))

        if command == "get_data":
            data = self.sock.recv(1024).decode('utf-8')
            self.get_logger().info('Khepera IV Driver: sensors: %s' % data)


def main(args=None):
    rclpy.init(args=args)
    khepera_driver = KheperaIVDriver()
    rclpy.spin(khepera_driver)

    khepera_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
