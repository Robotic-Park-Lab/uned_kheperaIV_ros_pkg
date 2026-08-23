# Copyright 2026 Robotic Park Lab
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Robotic Park Lab nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rclpy
from math import sqrt
from rclpy.node import Node
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
import yaml

from uned_kheperaiv_task.formation_marker import build_distance_marker

agent_list = list()


class Agent():
    def __init__(self, parent, distance, id):
        self.id = id
        self.distance = distance
        self.pose = Pose()
        self.parent = parent
        self.sub_pose = self.parent.create_subscription(
            Pose, '/' + self.id + '/local_pose', self.gtpose_callback, 10)
        self.publisher_data_ = self.parent.create_publisher(Float64, self.id + '/data', 10)
        self.publisher_marker = self.parent.create_publisher(Marker, self.id + '/marker', 10)

    def gtpose_callback(self, msg):
        if abs(msg.position.x) < 1.15 and abs(msg.position.y) < 1.15:
            self.pose = msg
            self.parent.distance_formation_bool = True
        self.parent.get_logger().debug('Formation Control::New local pose.')
        line = build_distance_marker(
            self.parent.groundtruth.position, self.pose.position, self.distance,
            self.parent.get_clock().now().to_msg())
        self.publisher_marker.publish(line)


class KheperaIVDriver(Node):
    def __init__(self):
        super().__init__('formation_control')
        # Params
        self.declare_parameter('config_file', 'file_path.yaml')
        self.declare_parameter('robot', 'khepera01')

        # Subscription
        self.gt_pose_ = self.create_subscription(Pose, 'local_pose', self.gtpose_callback, 10)
        self.sub_status_ = self.create_subscription(
            String, '/swarm/status', self.order_callback, 10)
        self.sub_order_ = self.create_subscription(String, '/swarm/order', self.order_callback, 1)
        self.sub_targetpose_ = self.create_subscription(
            Pose, 'target_pose', self.targetpose_callback, 10)
        self.sub_swarmgoalpose_ = self.create_subscription(
            Pose, '/swarm/goal_pose', self.swarm_goalpose_callback, 1)
        # Publisher
        self.pub_goalpose_ = self.create_publisher(Pose, 'goal_pose', 10)

        self.initialize()
        self.timer = self.create_timer(0.1, self.task_manager)

    def initialize(self):
        self.get_logger().info('Formation Control::inicialize() ok.')
        # Read Params
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        self.id = self.get_parameter('robot').get_parameter_value().string_value

        with open(config_file, 'r') as file:
            documents = yaml.safe_load(file)

        self.config = documents[self.id]

        if self.config['task']['enable']:
            self.agent_list = list()
            aux = self.config['task']['relationship']
            self.relationship = aux.split(', ')
            if self.config['task']['type'] == 'distance':
                for rel in self.relationship:
                    aux = rel.split('_')
                    robot = Agent(self, float(aux[1]), aux[0])
                    agent_list.append(robot)

        self.groundtruth = Pose()
        self.distance_formation_bool = False
        self.formation_bool = False
        self.leader = False
        self.centroid_leader = False
        self.leader_cmd = Pose()
        self.x_error = 0
        self.y_error = 0
        self.integral_x = 0
        self.integral_y = 0
        self.get_logger().info('Formation Control::inicialized.')

    def gtpose_callback(self, msg):
        self.groundtruth = msg

    def targetpose_callback(self, msg):
        self.target_pose = msg
        self.leader = True

    def order_callback(self, msg):
        self.get_logger().info('Order: "%s"' % msg.data)
        if msg.data == 'distance_formation_run':
            self.formation_bool = True
        elif msg.data == 'formation_stop':
            self.formation_bool = False
        elif msg.data == 'Ready':
            self.formation_bool = True
        else:
            self.get_logger().error('"%s": Unknown order' % (msg.data))

    def swarm_goalpose_callback(self, msg):
        if not self.centroid_leader:
            self.get_logger().info('Formation Control::Leader-> Centroid.')
        self.centroid_leader = True
        self.leader_cmd = msg

    def task_manager(self):
        if self.formation_bool:  # and self.distance_formation_bool:
            msg = Pose()
            dx = dy = 0
            for robot in agent_list:
                error_x = self.groundtruth.position.x - robot.pose.position.x
                error_y = self.groundtruth.position.y - robot.pose.position.y
                error_z = self.groundtruth.position.z - robot.pose.position.z
                distance = pow(error_x, 2) + pow(error_y, 2) + pow(error_z, 2)
                dx += (1 / 4) * (pow(robot.distance, 2) - distance) * error_x
                dy += (1 / 4) * (pow(robot.distance, 2) - distance) * error_y

                msg_data = Float64()
                msg_data.data = abs(robot.distance - sqrt(distance))
                robot.publisher_data_.publish(msg_data)

            if self.leader:
                ex = self.groundtruth.position.x - self.target_pose.position.x
                ey = self.groundtruth.position.y - self.target_pose.position.y
                msg.position.x += (1 / 4) * (-pow(ex, 2)) * ex
                msg.position.y += (1 / 4) * (-pow(ey, 2)) * ey
                self.get_logger().info(
                    'Target-> X: %.3f Y: %.3f \tCMD-> X: %.3f Y: %.3f \tPose-> X: %.3f Y: %.3f' %
                    (self.target_pose.position.x,
                     self.target_pose.position.y,
                     msg.position.x,
                     msg.position.y,
                     self.groundtruth.position.x,
                     self.groundtruth.position.y))

            if self.centroid_leader:
                msg.position.x += self.leader_cmd.position.x
                msg.position.y += self.leader_cmd.position.y
                msg.position.z += self.leader_cmd.position.z

            msg.position.x = self.groundtruth.position.x + dx / 4
            msg.position.y = self.groundtruth.position.y + dy / 4

            self.pub_goalpose_.publish(msg)
            self.distance_formation_bool = False


def main(args=None):
    rclpy.init(args=args)
    formation_control = KheperaIVDriver()
    rclpy.spin(formation_control)

    formation_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
