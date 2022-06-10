import logging
import time
import rclpy
from threading import Timer
import numpy as np
import math

from rclpy.node import Node
from std_msgs.msg import String, UInt16, UInt16MultiArray, Float64, Float64MultiArray
from geometry_msgs.msg import Pose, Twist, PoseWithCovariance
from nav_msgs.msg import Odometry

class KheperaIVDriver(Node):
    def __init__(self):
        super().__init__('formation_control')
        # Params
        self.declare_parameter('config_file', 'path')

        # Subscription
        self.gt_pose = self.create_subscription(Odometry, 'ground_truth', self.gtpose_callback, 10)
        self.agent_gt0 = self.create_subscription(Odometry, '/agent_gt', self.agent_pose_callback, 10)
        # Publisher
        self.ref_pose = self.create_publisher(Pose, 'goal_pose', 10)

        self.initialize()

    def initialize(self):
        self.get_logger().info('Formation Control::inicialize() ok.')
        # Read Params
        self.yaml_file = self.get_parameter('config_file').get_parameter_value().string_value
        self.groundtruth = Pose()
        self.agent_pose = Pose()
        self.init = False

    def update_ref_pose(self):
        msg = Pose()
        msg.position.x = (self.agent_pose.position.x - self.groundtruth.position.x) + 0.2
        msg.position.y = (self.agent_pose.position.y - self.groundtruth.position.y) + 0.2
        self.ref_pose.publish(msg)

    def gtpose_callback(self, msg):
        msg_aux = PoseWithCovariance()
        msg_aux = msg.pose
        self.groundtruth = msg_aux.pose
        self.init = True

    def agent_pose_callback(self, msg):
        msg_aux = PoseWithCovariance()
        msg_aux = msg.pose
        self.agent_pose = msg_aux.pose
        if self.init:
            self.update_ref_pose()

def main(args=None):
    print('Hi from uned_kheperaIV_task.')
    rclpy.init(args=args)
    formation_control = KheperaIVDriver()
    rclpy.spin(formation_control)

    formation_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
