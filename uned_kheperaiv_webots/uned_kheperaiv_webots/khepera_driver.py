from rosgraph_msgs.msg import Clock
import rclpy
import os
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from math import cos, sin, degrees, radians, pi
import sys
import tf_transformations
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped



class KheperaWebotsDriver:
     # The `init` method is called only once the driver is initialized.
    # You will always get two arguments in the `init` method.
    # - The `webots_node` argument contains a reference on a Supervisor instance.
    # - The `properties` argument is a dictionary created from the XML tags.
    def init(self, webots_node, properties):
        # Unfortunately, we cannot get an instance of the parent ROS node.
        # However, we can create a new one.
        rclpy.init(args=None)
        self.__node = rclpy.create_node('plugin_node_example')
        
        

        # Create a simple publisher, subscriber and "Clock" variable.
        self.__node.create_subscription(Clock, 'clock', self.__clock_callback, 1)
        self.__publisher = self.__node.create_publisher(Clock, 'custom_clock', 1)
        self.__clock = Clock()

    def __clock_callback(self, msg):
        self.__clock = msg

    # The `step` method is called at every step.
    def step(self):
        # The self.__node has to be spinned once in order to execute callback functions.
        rclpy.spin_once(self.__node, timeout_sec=0)

        self.__publisher.publish(self.__clock)