#include "uned_kheperaiv_controllers/KheperaPositionController.hpp"
using std::placeholders::_1;

bool PositionController::initialize(){
  RCLCPP_INFO_ONCE(this->get_logger(),"PositionController::inicialize() ok.");

  // Lectura de parámetros
	this->get_parameter("CONTROLLER_TYPE", m_controller_type);
	this->get_parameter("ROBOT_ID", m_robot_id);
	this->get_parameter("CONTROLLER_MODE", m_controller_mode);

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
  RCLCPP_WARN_ONCE(this->get_logger(),"PositionController::iterate() In-Progress.");
  if (first_pose_received && first_ref_received) {
    RCLCPP_INFO_ONCE(this->get_logger(),"PositionController::iterate() ok.");
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
      msg_cmd.angular.z = 1.0*std::sin(angle-yaw_gt);
      msg_cmd.linear.x = 1.0*distance*std::cos(angle-yaw_gt);
      if(msg_cmd.linear.x<0){
        msg_cmd.linear.x = 0.0;
      }
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
    ref_pose.position = msg->position;
    ref_pose.orientation = msg->orientation;
    if(!first_ref_received){
      first_ref_received = true;
    }
}