#include "uned_kheperaiv_controllers/KheperaPositionController.hpp"
using std::placeholders::_1;

bool PositionController::initialize(){
  RCLCPP_INFO_ONCE(this->get_logger(),"PositionController::inicialize() ok.");

  // Lectura de parámetros
	this->get_parameter("ROBOT_ID", m_robot_id);
  this->get_parameter("Relative_pose", m_rel_pose);

  // Linear Controller
  this->get_parameter("LKp", Kp);
  this->get_parameter("LKi", Ki);
  this->get_parameter("LKd", Kd);
  this->get_parameter("LTd", Td);
  l_controller = init_controller("L", Kp, Ki, Kd, Td, 100, 2.0, -1.0);

  // Angular Controller
  this->get_parameter("WKp", Kp);
  this->get_parameter("WKi", Ki);
  this->get_parameter("WKd", Kd);
  this->get_parameter("WTd", Td);
  w_controller = init_controller("W", Kp, Ki, Kd, Td, 100, 2.0, -2.0);
  // Publisher:
	// Referencias para los controladores PID Attitude y Rate
  pub_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);

  // Subscriber:
  GT_pose_ = this->create_subscription<nav_msgs::msg::Odometry>("ground_truth", 10, std::bind(&PositionController::gtposeCallback, this, _1));
	// Reference:
  ref_pose_ = this->create_subscription<geometry_msgs::msg::Pose>("goal_pose", 10, std::bind(&PositionController::positionreferenceCallback, this, _1));

  return true;
}

bool PositionController::iterate(){
  RCLCPP_INFO_ONCE(this->get_logger(),"PositionController::iterate() ok.");
  if (first_pose_received && first_ref_received) {
    RCLCPP_INFO_ONCE(this->get_logger(), "PositionController::iterate(). Running ...");
    // Ground Truth Yaw
    // yaw (z-axis rotation)
    double siny_cosp = 2 * (GT_pose.orientation.w * GT_pose.orientation.z + GT_pose.orientation.x * GT_pose.orientation.y);
    double cosy_cosp = 1 - 2 * (GT_pose.orientation.y * GT_pose.orientation.y + GT_pose.orientation.z * GT_pose.orientation.z);
    yaw_gt = std::atan2(siny_cosp, cosy_cosp); // * (180 / 3.14159265);

    auto msg_cmd = geometry_msgs::msg::Twist();
    // Errors
    angle = std::atan2(ref_pose.position.y-GT_pose.position.y, ref_pose.position.x-GT_pose.position.x);
    distance = std::sqrt(std::pow(ref_pose.position.x-GT_pose.position.x,2)+std::pow(ref_pose.position.y-GT_pose.position.y,2));

    // Controller "A Khepera IV library for robotic control education using V-REP"
    if(distance>0.05){
      w_controller.error[0] = std::sin(angle-yaw_gt);
      msg_cmd.angular.z = pid_controller(w_controller, dt);
      l_controller.error[0] = distance*std::cos(angle-yaw_gt);
      msg_cmd.linear.x = pid_controller(l_controller, dt);
      // if(msg_cmd.linear.x<0){
      //   msg_cmd.linear.x = 0.0;
      // }
    }
    else{
      msg_cmd.angular.z = 0.0;
      msg_cmd.linear.x = 0.0;
    }
    
    // Send command
    RCLCPP_DEBUG(this->get_logger(),"Distance: %f \tAngle: %f \tGT_Yaw: %f \tW: %f", distance, angle, yaw_gt, msg_cmd.angular.z);
    pub_cmd_->publish(msg_cmd);
  }

  return true;
}


int main(int argc, char ** argv){
  try{
    rclcpp::init(argc, argv);
    auto kheperaIV_position_controller = std::make_shared<PositionController>();
    rclcpp::Rate loop_rate(100);
    kheperaIV_position_controller->initialize();

    while (rclcpp::ok()){
      rclcpp::spin_some(kheperaIV_position_controller);
      kheperaIV_position_controller->iterate();
      loop_rate.sleep();
    }
    return 0;
  } catch (std::exception &e){
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception: %s",e.what());
	}
}

struct pid_s PositionController::init_controller(const char id[], double kp, double ki, double kd, double td, int nd, double upperlimit, double lowerlimit){
    struct pid_s controller;

    controller.kp = kp;
    controller.ki = ki;
    controller.kd = kd;
    controller.td = td;
    controller.nd = nd;
    controller.error[0] = 0.0;
    controller.error[1] = 0.0;
    controller.integral = 0.0;
    controller.derivative = 0.0;
    controller.upperlimit = upperlimit;
    controller.lowerlimit = lowerlimit;

    RCLCPP_INFO(this->get_logger(),"%s Controller: kp: %0.2f \tki: %0.2f \tkd: %0.2f", id, controller.kp, controller.ki, controller.kd);
    return controller;
}

double PositionController::pid_controller(struct pid_s &controller, double dt){
  double outP = controller.kp * controller.error[0];
	controller.integral = controller.integral + controller.ki * controller.error[1] * dt;
	controller.derivative = (controller.td/(controller.td+controller.nd+dt))*controller.derivative+(controller.kd*controller.nd/(controller.td+controller.nd*dt))*(controller.error[0]-controller.error[1]);
	double out = outP + controller.integral + controller.derivative;

	if(controller.upperlimit != 0.0){
		double out_i = out;

		if (out > controller.upperlimit)
			out = controller.upperlimit;
		if (out < controller.lowerlimit)
			out = controller.lowerlimit;

		// controller.integral = controller.integral - (out - out_i) * sqrt(controller.kp / controller.ki);
	}

	controller.error[1] = controller.error[0];

	return out;
}


void PositionController::gtposeCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
  auto msg_aux = geometry_msgs::msg::PoseWithCovariance();
  msg_aux = msg->pose;
  GT_pose = msg_aux.pose;

  if(!first_pose_received){
      RCLCPP_INFO(this->get_logger(),"Init Pose: x: %f \ty: %f \tz: %f", GT_pose.position.x, GT_pose.position.y, GT_pose.position.z);
      first_pose_received = true;
  }
}

void PositionController::positionreferenceCallback(const geometry_msgs::msg::Pose::SharedPtr msg){
  if(m_rel_pose){
    ref_pose.position.x = GT_pose.position.x + msg->position.x;
    ref_pose.position.y = GT_pose.position.y + msg->position.y;
    ref_pose.position.z = GT_pose.position.z + msg->position.z;
    ref_pose.orientation.x = GT_pose.orientation.x + msg->orientation.x;
    ref_pose.orientation.y = GT_pose.orientation.y + msg->orientation.y;
    ref_pose.orientation.z = GT_pose.orientation.z + msg->orientation.z;
    ref_pose.orientation.w = GT_pose.orientation.w + msg->orientation.w;
  }
  else{
    ref_pose.position = msg->position;
    ref_pose.orientation = msg->orientation;
  }
  if(!first_ref_received){
    first_ref_received = true;
  }
    
}