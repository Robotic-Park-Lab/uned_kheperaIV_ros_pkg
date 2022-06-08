#include "uned_kheperaiv_controllers/KheperaPositionController.hpp"
using std::placeholders::_1;

bool PositionController::initialize(){
  RCLCPP_INFO_ONCE(this->get_logger(),"PositionController::inicialize() ok.");

  // Lectura de parámetros
	this->get_parameter("CONTROLLER_TYPE", m_controller_type);
	this->get_parameter("ROBOT_ID", m_robot_id);
	this->get_parameter("CONTROLLER_MODE", m_controller_mode);
	this->get_parameter("X_POS", m_x_init);
	this->get_parameter("Y_POS", m_y_init);
	this->get_parameter("Z_POS", m_z_init);

  // Publisher:
	// Referencias para los controladores PID Attitude y Rate
  pub_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);

  // Subscriber:
  GT_pose_ = this->create_subscription<nav_msgs::msg::Odometry>("ground_truth", 10, std::bind(&PositionController::gtposeCallback, this, _1));
	// Reference:
  ref_pose_ = this->create_subscription<geometry_msgs::msg::Pose>("goal_pose", 10, std::bind(&PositionController::positionreferenceCallback, this, _1));

  // Init values
  ref_pose.position.x = m_x_init;
	ref_pose.position.y = m_y_init;
	ref_pose.position.z = m_z_init;
	ref_pose.orientation.x = 0;
	ref_pose.orientation.y = 0;
	ref_pose.orientation.z = 0;
	ref_pose.orientation.w = 1;
  RCLCPP_INFO(this->get_logger(),"New Pose: x: %f \ty: %f \tz: %f", ref_pose.position.x, ref_pose.position.y, ref_pose.position.z);

  return true;
}

bool PositionController::iterate(){
  RCLCPP_INFO_ONCE(this->get_logger(),"PositionController::iterate() ok.");
  RCLCPP_WARN_ONCE(this->get_logger(),"PositionController::iterate() In-Progress.");
  
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
    GT_pose.pose = msg->pose;
}

void PositionController::positionreferenceCallback(const geometry_msgs::msg::Pose::SharedPtr msg){
    ref_pose.position = msg->position;
    ref_pose.orientation = msg->orientation;
}