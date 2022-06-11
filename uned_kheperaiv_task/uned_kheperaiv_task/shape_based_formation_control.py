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

agent_list = list()

class Agent():
    def __init__(self, parent, x, y, id):
        self.id = id
        self.x = x
        self.y = y
        self.pose = Pose()
        self.parent = parent
        print(self.id + '/ground_truth')
        self.sub_pose = self.parent.create_subscription(Odometry, self.id + '/ground_truth', self.gtpose_callback, 10)
        # self.str_()

    def str_(self):
        print(self.x)
        print(self.y)

    def gtpose_callback(self, msg):
        msg_aux = PoseWithCovariance()
        msg_aux = msg.pose
        self.pose = msg_aux.pose
    
class KheperaIVDriver(Node):
    def __init__(self):
        super().__init__('formation_control')
        # Params
        self.declare_parameter('config_file', 'path')
        self.declare_parameter('agents', 'khepera01')
        self.declare_parameter('agent_x', '0.2')
        self.declare_parameter('agent_y', '0.2')

        # Subscription
        self.gt_pose = self.create_subscription(Odometry, 'ground_truth', self.gtpose_callback, 10)
        # Publisher
        self.ref_pose = self.create_publisher(Pose, 'goal_pose', 10)

        self.initialize()

        self.timer = self.create_timer(0.02, self.update_ref_pose)

        

    def initialize(self):
        self.get_logger().info('Formation Control::inicialize() ok.')
        # Read Params
        self.yaml_file = self.get_parameter('config_file').get_parameter_value().string_value
        aux = self.get_parameter('agents').get_parameter_value().string_value
        id_array =  aux.split(', ')
        aux = self.get_parameter('agent_x').get_parameter_value().string_value
        x_array = aux.split(', ')
        aux = self.get_parameter('agent_y').get_parameter_value().string_value
        y_array = aux.split(', ')
        for i in range(int(len(id_array)),0,-1):
            agent_str = id_array[i-1]
            robot = Agent(self, float(x_array[i-1]), float(y_array[i-1]), agent_str)
            agent_list.append(robot)
            print(agent_list)

        self.groundtruth = Pose()
        self.init = False

    def gtpose_callback(self, msg):
        msg_aux = PoseWithCovariance()
        msg_aux = msg.pose
        self.groundtruth = msg_aux.pose
        self.init = True

    def update_ref_pose(self):
        if self.init:
            msg = Pose()
            for robot in agent_list:
                msg.position.x = msg.position.x + (robot.pose.position.x - self.groundtruth.position.x) + robot.x
                msg.position.y = msg.position.y + (robot.pose.position.y - self.groundtruth.position.y) + robot.y

            self.ref_pose.publish(msg)

def main(args=None):
    print('Hi from uned_kheperaIV_task.')
    rclpy.init(args=args)
    formation_control = KheperaIVDriver()
    rclpy.spin(formation_control)

    formation_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
