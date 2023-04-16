import rclpy
from math import sqrt
from rclpy.node import Node
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Pose, Point, Twist, PoseWithCovariance
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
import yaml
from math import atan2, cos, sin, sqrt, radians, pi
import tf_transformations
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

agent_list = list()

class Agent():
    def __init__(self, parent, id, x = None, y = None, z = None, d = None):
        self.id = id
        if d == None:
            self.x = x
            self.y = y
            self.z = z
        else:
            self.d = d
        self.pose = Pose()
        self.parent = parent
        self.sub_pose = self.parent.create_subscription(Pose, '/' + self.id + '/local_pose', self.gtpose_callback, 10)
        if not self.parent.digital_twin:
            self.publisher_data_ = self.parent.create_publisher(Float64, self.id + '/data', 10)
            self.publisher_marker = self.parent.create_publisher(Marker, self.id + '/marker', 10)

    def str_(self):
        return ('ID: ' + str(self.id) + ' X: ' + str(self.x) +
                ' Y: ' + str(self.y)+' Z: ' + str(self.z))

    def gtpose_callback(self, msg):
        self.pose = msg
        if not self.parent.digital_twin:
            line = Marker()
            p0 = Point()
            p0.x = self.parent.gt_pose.position.x
            p0.y = self.parent.gt_pose.position.y
            p0.z = self.parent.gt_pose.position.z

            p1 = Point()
            p1.x = self.pose.position.x
            p1.y = self.pose.position.y
            p1.z = self.pose.position.z
            # self.parent.distance_formation_bool = True

            distance = sqrt(pow(p0.x-p1.x,2)+pow(p0.y-p1.y,2)+pow(p0.z-p1.z,2))
        
            line.header.frame_id = 'map'
            line.header.stamp = self.parent.get_clock().now().to_msg()
            line.id = 1
            line.type = 5
            line.action = 0
            line.scale.x = 0.01
            line.scale.y = 0.01
            line.scale.z = 0.01

            if abs(distance - self.d) > 0.05:
                line.color.r = 1.0
            else:
                if abs(distance - self.d) > 0.025:
                    line.color.r = 1.0
                    line.color.g = 0.5
                else:
                    line.color.g = 1.0
            line.color.a = 1.0
            line.points.append(p1)
            line.points.append(p0)

            self.publisher_marker.publish(line)


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

    def eval_threshold(self, error):
        # a
        a = self.trigger_ai * abs(error)
        if a > self.trigger_ai:
            a = self.trigger_ai

        # Threshold
        self.th = self.trigger_co + a
        self.inc = abs(abs(error) - self.trigger_last_signal) 
        # Delta Error
        if (self.inc >= abs(self.th)):
            self.trigger_last_signal = abs(error)
            return True

        return False

    
class KheperaIVDriver(Node):
    def __init__(self):
        super().__init__('formation_control')
        # Params
        self.declare_parameter('config_file', 'file_path.yaml')
        self.declare_parameter('robot', 'khepera01')
        self.declare_parameter('type', 'virtual')

        # Subscription
        self.gt_pose_ = self.create_subscription(Odometry, 'ground_truth', self.gtpose_callback, 10)
        self.sub_status_ = self.create_subscription(String, '/swarm/status', self.order_callback, 10)
        self.sub_order_ = self.create_subscription(String, '/swarm/order', self.order_callback, 1)
        self.sub_targetpose_ = self.create_subscription(Pose, 'target_pose', self.targetpose_callback, 10)
        self.sub_swarmgoalpose_ = self.create_subscription(Pose, '/swarm/goal_pose', self.swarm_goalpose_callback, 1)
        # Publisher
        self.pub_goalpose_ = self.create_publisher(Pose, 'goal_pose', 10)
        self.pub_cmd_vel_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.initialize()
        self.tfbr = TransformBroadcaster(self)

    def initialize(self):
        self.get_logger().info('Formation Control::inicialize() ok.')
        # Read Params
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        self.id = self.get_parameter('robot').get_parameter_value().string_value
        self.type = self.get_parameter('type').get_parameter_value().string_value
        self.digital_twin = self.type == 'digital_twin'
        if self.digital_twin:
            self.create_subscription(Pose, 'local_pose', self.dt_pose_callback, 1)
            pose_name = 'dt_pose'
        else:
            pose_name = 'local_pose'
        self.pose_publisher = self.create_publisher(Pose, pose_name, 10)

        with open(config_file, 'r') as file:
            documents = yaml.safe_load(file)

        self.config = documents[self.id]

        # Init relationship
        if self.config['task']['enable']:
            self.get_logger().info('Task %s' % self.config['task']['type'])
            self.agent_list = list()
            aux = self.config['task']['relationship']
            self.relationship = aux.split(', ')
            if self.config['task']['type'] == 'distance':
                for rel in self.relationship:
                    aux = rel.split('_')
                    robot = Agent(self, aux[0], d = float(aux[1]))
                    self.get_logger().info('Khepera: %s: Agent: %s \td: %s' % (self.id, aux[0], aux[1]))
                    self.agent_list.append(robot)

        self.communication = (self.config['communication']['type'] == 'Continuous')
        if not self.communication:
            self.threshold = self.config['communication']['threshold']['co']
        else:
            self.threshold = 0.001
        
        ## Intialize Variables
        self.target_twist = Twist()
        ## Initialize Pose
        self.init_pose = False
        self.last_pose = Pose()
        self.gt_pose = Pose()
        self.target_pose = Pose()
        self.eomas = 3.14
        self.target_pose.position.x = 0.0
        self.target_pose.position.y= 0.0
        self.distance_formation_bool = False
        self.continuous = False
        # Position
        self.linear_controller = PIDController(1.0, 0.0, 0.0, 0.0, 100, 1.0, -1.0, 0.05, 0.01)
        # Angle
        self.angular_controller = PIDController(1.0, 0.0, 0.0, 0.0, 100, 0.0, 0.0, 0.1, 0.1)

        self.leader = False
        self.centroid_leader = False
        self.leader_cmd = Pose()
        self.x_error = 0
        self.y_error = 0
        self.integral_x = 0
        self.integral_y = 0

        self.timer = self.create_timer(0.1, self.iterate)
        self.get_logger().info('Formation Control::inicialized.')

    def order_callback(self, msg):
        self.get_logger().info('Order: "%s"' % msg.data)
        if msg.data == 'distance_formation_run':
            self.distance_formation_bool = True
        elif msg.data == 'formation_stop':
            self.distance_formation_bool = False
        elif msg.data == 'Ready':
            self.distance_formation_bool = True
        else:
            self.get_logger().error('"%s": Unknown order' % (msg.data))

    def dt_pose_callback(self, pose):
        self.get_logger().debug('DT Pose: X:%f Y:%f' % (pose.position.x,pose.position.y))
        delta = np.array([self.gt_pose.position.x-pose.position.x,self.gt_pose.position.y-pose.position.y,self.gt_pose.position.z-pose.position.z])
    
    def cmd_vel_callback(self, twist):
        self.target_twist = twist
    
    def gtpose_callback(self, msg):
        self.gt_pose = Pose()
        self.gt_pose.position.x = msg.pose.pose.position.x
        self.gt_pose.position.y = msg.pose.pose.position.y
        self.gt_pose.position.z = msg.pose.pose.position.z
        self.gt_pose.orientation.x = msg.pose.pose.orientation.x
        self.gt_pose.orientation.y = msg.pose.pose.orientation.y
        self.gt_pose.orientation.z = msg.pose.pose.orientation.z
        self.gt_pose.orientation.w = msg.pose.pose.orientation.w
        
        siny_cosp = 2 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z + msg.pose.pose.orientation.x * msg.pose.pose.orientation.y)
        cosy_cosp = 1 - 2 * (msg.pose.pose.orientation.y * msg.pose.pose.orientation.y + msg.pose.pose.orientation.z * msg.pose.pose.orientation.z)
        self.global_yaw = atan2(siny_cosp, cosy_cosp)

        t_base = TransformStamped()
        t_base.header.stamp = self.get_clock().now().to_msg()
        t_base.header.frame_id = 'map'
        if self.digital_twin:
            base_name = self.id+'_dt/base_link'
        else:
            base_name = self.id+'/base_link'
        t_base.child_frame_id = base_name
        t_base.transform.translation.x = msg.pose.pose.position.x
        t_base.transform.translation.y = msg.pose.pose.position.y
        t_base.transform.translation.z = msg.pose.pose.position.z
        t_base.transform.rotation.x = msg.pose.pose.orientation.x
        t_base.transform.rotation.y = msg.pose.pose.orientation.y
        t_base.transform.rotation.z = msg.pose.pose.orientation.z
        t_base.transform.rotation.w = msg.pose.pose.orientation.w
        self.tfbr.sendTransform(t_base)

        if not self.init_pose:

            self.target_pose.position.x = msg.pose.pose.position.x
            self.target_pose.position.y= msg.pose.pose.position.y
            self.last_pose = self.gt_pose
            self.init_pose = True

        delta = np.array([self.gt_pose.position.x-self.last_pose.position.x,self.gt_pose.position.y-self.last_pose.position.y,self.gt_pose.position.z-self.last_pose.position.z])
        if np.linalg.norm(delta) > 0.01 or True:
            self.pose_publisher.publish(self.gt_pose)
            self.last_pose = self.gt_pose

        self.get_logger().debug('Pose: %.2f %.2f %.2f' % (self.gt_pose.position.x, self.gt_pose.position.y, self.global_yaw))

    def targetpose_callback(self, msg):
        self.target_pose = msg
        self.leader = True

    def swarm_goalpose_callback(self, msg):
        if not self.centroid_leader:
            self.get_logger().info('Formation Control::Leader-> Centroid.')
        self.centroid_leader = True
        self.leader_cmd = msg

    def iterate(self):
        ## Formation Control
        if self.distance_formation_bool:
            self.distance_formation_control()
            # self.distance_formation_bool = False

        ## Position Controller
        self.target_twist = self.IPC_controller()
        angle = atan2(self.target_pose.position.y-self.gt_pose.position.y, self.target_pose.position.x-self.gt_pose.position.x)
        distance = sqrt(pow(self.target_pose.position.x-self.gt_pose.position.x,2)+pow(self.target_pose.position.y-self.gt_pose.position.y,2))

        # Controller "A Khepera IV library for robotic control education using V-REP"
        if(distance>0.05):
            self.angular_controller.error[0] = sin(angle-self.global_yaw)
            self.target_twist.angular.z = self.angular_controller.update(0.1)
            self.linear_controller.error[0] = distance*cos(angle-self.global_yaw)
            self.target_twist.linear.x = self.linear_controller.update(0.1)
        else:
            self.target_twist.angular.z = 0.0
            self.target_twist.linear.x = 0.0

        self.get_logger().debug('IPC: vX:%f vZ:%f' % (self.target_twist.linear.x,self.target_twist.angular.z))
        self.pub_cmd_vel_.publish(self.target_twist)
            
    def distance_formation_control(self):
        dx = dy = dz = 0
        for agent in self.agent_list:
            error_x = self.gt_pose.position.x - agent.pose.position.x
            error_y = self.gt_pose.position.y - agent.pose.position.y
            error_z = self.gt_pose.position.z - agent.pose.position.z
            distance = pow(error_x,2)+pow(error_y,2)+pow(error_z,2)
            dx += (pow(agent.d,2) - distance) * error_x
            dy += (pow(agent.d,2) - distance) * error_y

            if not self.digital_twin:
                msg_data = Float64()
                msg_data.data = abs(agent.d - sqrt(distance))
                agent.publisher_data_.publish(msg_data)
                self.get_logger().debug('Agent %s: D: %.2f dx: %.2f dy: %.2f dz: %.2f ' % (agent.id, msg_data.data, dx, dy, dz)) 

        error_r = pow(1.0,2) - (pow(self.gt_pose.position.x,2)+pow(self.gt_pose.position.y,2))
        dx += 2 * (error_r *self.gt_pose.position.x)
        dy += 2 * (error_r * self.gt_pose.position.y)
        
        if dx > 0.32:
            dx = 0.32
        if dx < -0.32:
            dx = -0.32
        if dy > 0.32:
            dy = 0.32
        if dy < -0.32:
            dy = -0.32

        self.target_pose.position.x = self.gt_pose.position.x + dx/4
        self.target_pose.position.y = self.gt_pose.position.y + dy/4

        self.get_logger().debug('Formation: X: %.2f->%.2f Y: %.2f->%.2f Z: %.2f->%.2f' % (self.gt_pose.position.x, self.target_pose.position.x, self.gt_pose.position.y, self.target_pose.position.y, self.gt_pose.position.z, self.target_pose.position.z)) 

    def IPC_controller(self):
        L = 0.10540
        Vmax = 10.0
        K1 = 1.0
        Kp = 1.5
        Ki = 0.008

        d = sqrt(pow(self.target_pose.position.x-self.gt_pose.position.x,2)+pow(self.target_pose.position.y-self.gt_pose.position.y,2))*100
        if d<5:
            V = 0.0
            w = 0.0
        else:
            alpha = atan2(self.target_pose.position.y-self.gt_pose.position.y,self.target_pose.position.x-self.gt_pose.position.x)
            oc = alpha - self.global_yaw
            eo = atan2(sin(oc),cos(oc))
            p = (3.14-abs(eo))/3.14
            V = min(K1*d*p,Vmax)

            self.eomas = eo+self.eomas
            w = Kp*sin(eo) + Ki*self.eomas*0.003

        ## Cmd_Vel
        out = Twist()
        out.linear.x = V
        out.angular.z = w

        return out 
    
def main(args=None):
    rclpy.init(args=args)
    robot_driver = KheperaIVDriver()
    rclpy.spin(robot_driver)

    robot_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
