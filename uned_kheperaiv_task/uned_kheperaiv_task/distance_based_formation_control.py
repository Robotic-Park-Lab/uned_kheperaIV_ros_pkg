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
        self.sub_pose = self.parent.create_subscription(Pose, self.id + '/local_pose', self.gtpose_callback, 10)
        self.publisher_data_ = self.parent.create_publisher(Float64, self.id + '/data', 10)
        self.publisher_marker = self.parent.create_publisher(Marker, self.id + '/marker', 10)

    def gtpose_callback(self, msg):
        if abs(msg.position.x)<1.15 and abs(msg.position.y)<1.15:
            self.pose = msg
            self.parent.distance_formation_bool = True

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
        self.gt_pose_ = self.create_subscription(Pose, 'local_pose', self.gtpose_callback, 10)
        self.sub_status_ = self.create_subscription(String, 'swarm/status', self.order_callback, 10)
        self.sub_order_ = self.create_subscription(String, 'swarm/order', self.order_callback, 1)
        self.sub_targetpose_ = self.create_subscription(Pose, 'target_pose', self.targetpose_callback, 10)
        self.sub_swarmgoalpose_ = self.create_subscription(Pose, 'swarm/goal_pose', self.swarm_goalpose_callback, 1)
        # Publisher
        self.pub_goalpose_ = self.create_publisher(Pose, 'goal_pose', 10)

        self.initialize()
        self.timer = self.create_timer(0.1, self.task_manager)
        
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
        if self.formation_bool: # and self.distance_formation_bool:
            msg = Pose()
            dx = dy = 0
            for robot in agent_list:
                error_x = self.groundtruth.position.x - robot.pose.position.x
                error_y = self.groundtruth.position.y - robot.pose.position.y
                error_z = self.groundtruth.position.z - robot.pose.position.z
                distance = pow(error_x,2)+pow(error_y,2)+pow(error_z,2)
                dx += (1/4) * (pow(robot.distance,2) - distance) * error_x
                dy += (1/4) * (pow(robot.distance,2) - distance) * error_y
                
                msg_data = Float64()
                msg_data.data = robot.distance - sqrt(distance)
                robot.publisher_data_.publish(msg_data)

            if self.leader:
                msg.position.x += (1/4) * (-pow(self.groundtruth.position.x - self.target_pose.position.x,2)) * (self.groundtruth.position.x - self.target_pose.position.x)
                msg.position.y += (1/4) * (-pow(self.groundtruth.position.y - self.target_pose.position.y,2)) * (self.groundtruth.position.y - self.target_pose.position.y)
                self.get_logger().info('Target-> X: %.3f Y: %.3f \tCMD-> X: %.3f Y: %.3f \tPose-> X: %.3f Y: %.3f' % (self.target_pose.position.x , self.target_pose.position.y, msg.position.x, msg.position.y, self.groundtruth.position.x, self.groundtruth.position.y))

            if self.centroid_leader:
                msg.position.x += self.leader_cmd.position.x
                msg.position.y += self.leader_cmd.position.y
                msg.position.z += self.leader_cmd.position.z

            msg.position.x = self.groundtruth.position.x + dx/4
            msg.position.y = self.groundtruth.position.y + dy/4

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
