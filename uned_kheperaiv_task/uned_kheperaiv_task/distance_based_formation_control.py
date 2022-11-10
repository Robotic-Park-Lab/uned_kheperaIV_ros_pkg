from math import atan2, cos, sin
import rclpy
from math import sqrt
from rclpy.node import Node
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Pose, Point
from visualization_msgs.msg import Marker

agent_list = list()

class Agent():
    def __init__(self, parent, distance, id):
        self.id = id
        self.distance = distance
        self.pose = Pose()
        self.parent = parent
        self.sub_pose = self.parent.create_subscription(Pose, self.id + '/pose', self.gtpose_callback, 10)
        self.publisher_data = self.parent.create_publisher(Float64, self.id + '/data', 10)
        self.publisher_marker = self.parent.create_publisher(Marker, self.id + '/marker', 10)

    def gtpose_callback(self, msg):
        self.pose = msg

        line = Marker()
        p0 = Point()
        p0.x = self.parent.groundtruth.position.x
        p0.y = self.parent.groundtruth.position.y
        p0.z = self.parent.groundtruth.position.z

        p1 = Point()
        p1.x = self.pose.position.x
        p1.y = self.pose.position.y
        p1.z = self.pose.position.z

        distance = sqrt(pow(p0.x-p1.x,2)+pow(p0.y-p1.y,2)+pow(p0.z-p1.z,2))

        line.header.frame_id = 'map'
        line.header.stamp = self.parent.get_clock().now().to_msg()
        line.id = 1
        line.type = 5
        line.action = 0
        line.scale.x = 0.01
        line.scale.y = 0.01
        line.scale.z = 0.01
        if abs(distance - self.distance ) > 0.05:
            line.color.r = 1.0
        else:
            if abs(distance - self.distance ) > 0.025:
                line.color.r = 1.0
                line.color.g = 0.5
            else:
                line.color.g = 1.0
        line.color.a = 1.0
        line.points.append(p1)
        line.points.append(p0)

        self.publisher_marker.publish(line)
    
class KheperaIVDriver(Node):
    def __init__(self):
        super().__init__('formation_control')
        # Params
        self.declare_parameter('config_file', 'path')
        self.declare_parameter('agents', 'khepera01')
        self.declare_parameter('distance', '0.2')
        self.declare_parameter('robot', 'khepera01')

        # Subscription
        self.gt_pose = self.create_subscription(Pose, 'pose', self.gtpose_callback, 10)
        self.sub_order = self.create_subscription(String, 'swarm/status', self.order_callback, 10)
        # Publisher
        self.ref_pose = self.create_publisher(Pose, 'goal_pose', 10)

        self.initialize()
        self.timer = self.create_timer(0.02, self.task_manager)
        
    def initialize(self):
        self.get_logger().info('Formation Control::inicialize() ok.')
        # Read Params
        self.yaml_file = self.get_parameter('config_file').get_parameter_value().string_value
        self.id = self.get_parameter('robot').get_parameter_value().string_value
        aux = self.get_parameter('agents').get_parameter_value().string_value
        id_array =  aux.split(', ')
        aux = self.get_parameter('distance').get_parameter_value().string_value
        distance_array = aux.split(', ')
        for i in range(int(len(id_array)),0,-1):
            agent_str = id_array[i-1]
            robot = Agent(self, float(distance_array[i-1]), agent_str)
            agent_list.append(robot)

        self.groundtruth = Pose()
        self.init = False
        self.x_error = 0
        self.y_error = 0
        self.integral_x = 0
        self.integral_y = 0
        self.get_logger().info('Formation Control::inicialized.')

    def gtpose_callback(self, msg):
        self.groundtruth = msg

    def order_callback(self, msg):
        self.init = True

    def task_manager(self):
        if self.init:
            msg = Pose()
            msg.position.x = self.groundtruth.position.x
            msg.position.y = self.groundtruth.position.y
            dx = dy = dz = 0
            for robot in agent_list:
                error_x = self.groundtruth.position.x - robot.pose.position.x
                error_y = self.groundtruth.position.y - robot.pose.position.y
                error_z = self.groundtruth.position.z - robot.pose.position.z
                distance = pow(error_x,2)+pow(error_y,2)+pow(error_z,2)
                dx += (pow(robot.distance,2) - distance) * error_x
                dy += (pow(robot.distance,2) - distance) * error_y
                
                msg_data = Float64()
                msg_data.data = robot.distance - distance
                robot.publisher_data.publish(msg_data)
                # self.get_logger().warn('Agent %s: D: %.2f X: %.3f Y: %.3f Alfa: %.3f' % (robot.id, distance.real, error_x, error_y, alfa))
            msg.position.x += dx/4
            msg.position.y += dy/4

            self.ref_pose.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    formation_control = KheperaIVDriver()
    rclpy.spin(formation_control)

    formation_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
