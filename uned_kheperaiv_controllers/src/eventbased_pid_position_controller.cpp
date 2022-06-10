#include "uned_kheperaiv_controllers/KheperaPositionController.hpp"
using std::placeholders::_1;

bool PositionController::initialize(){
  RCLCPP_INFO_ONCE(this->get_logger(),"PositionController::inicialize() ok.");

  // Lectura de parámetros
	this->get_parameter("ROBOT_ID", m_robot_id);

  // Linear Controller
  this->get_parameter("LKp", Kp);
  this->get_parameter("LKi", Ki);
  this->get_parameter("LKd", Kd);
  this->get_parameter("LTd", Td);
  l_controller = init_controller("L", Kp, Ki, Kd, Td, 100, 2.0, -1.0);
  this->get_parameter("Lco", Co);
	this->get_parameter("Lai", Ai);
l_threshold = init_triggering("L", Co, Ai);

  // Angular Controller
  this->get_parameter("WKp", Kp);
  this->get_parameter("WKi", Ki);
  this->get_parameter("WKd", Kd);
  this->get_parameter("WTd", Td);
  w_controller = init_controller("W", Kp, Ki, Kd, Td, 100, 2.0, -2.0);
  this->get_parameter("Wco", Co);
	this->get_parameter("Wai", Ai);
	w_threshold = init_triggering("W", Co, Ai);
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
  RCLCPP_WARN_ONCE(this->get_logger(),"TO-DO: Event-Based Position Controller.");
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
    if (eval_threshold(l_threshold, distance, 0)){
      RCLCPP_INFO(this->get_logger(), "Linear New Event. Dt: %.4f", l_threshold.dt);
      if(distance>0.05){
        l_controller.error[0] = distance*std::cos(angle-yaw_gt);
        msg_cmd.linear.x = pid_controller(l_controller, dt);
      }
      else{
        msg_cmd.linear.x = 0.0;
      }
    }
    if (eval_threshold(w_threshold, yaw_gt, angle)){
      RCLCPP_INFO(this->get_logger(), "Linear New Event. Dt: %.4f", l_threshold.dt);
      w_controller.error[0] = std::sin(angle-yaw_gt);
      msg_cmd.angular.z = pid_controller(w_controller, dt);
    }
    
    // Send command
    RCLCPP_DEBUG(this->get_logger(),"Distance: %f \tAngle: %f \tGT_Yaw: %f \tW: %f", distance, angle, yaw_gt, msg_cmd.angular.z);
    pub_cmd_->publish(msg_cmd);
  }

  return true;
  return true;
}

int main(int argc, char ** argv){
  try{
    rclcpp::init(argc, argv);
    auto kheperaIV_position_controller = std::make_shared<PositionController>();
    rclcpp::Rate loop_rate(10);
    kheperaIV_position_controller->initialize();

    while (rclcpp::ok()){
      kheperaIV_position_controller->iterate();
      rclcpp::spin_some(kheperaIV_position_controller);
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
}

void PositionController::positionreferenceCallback(const geometry_msgs::msg::Pose::SharedPtr msg){
    ref_pose.position = msg->position;
    ref_pose.orientation = msg->orientation;
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

struct threshold PositionController::init_triggering(const char id[], double co, double a){
    struct threshold trigger;

    trigger.co = co;
    trigger.ai = a;
		trigger.last_time = std::chrono::steady_clock::now();

    RCLCPP_INFO(this->get_logger(),"%s Threshold: C0: %0.2f \tai: %0.2f", id, trigger.co, trigger.ai);
    return trigger;
}

bool PositionController::eval_threshold(struct threshold &trigger, double signal, double ref){
	// Noise - Cn
	double mean = signal/20;
	for(int i = 0; i<19; i++){
		trigger.noise[i+1] = trigger.noise[i];
		mean += trigger.noise[i]/20;
	}
	trigger.noise[0] = signal;
	trigger.cn = 0.0;
	for(int i = 0; i<20; i++){
		if(abs(trigger.noise[i]-mean) > trigger.cn)
			trigger.cn = trigger.noise[i]-mean;
	}
	// a
	double a = trigger.ai * abs(signal - ref);
	if (a>trigger.ai)
		a = trigger.ai;
	// Threshold
	double th = trigger.co + a + trigger.cn;
	double inc = abs(abs(ref-signal) - trigger.last_signal);

	// Delta Error
	if (inc >= th){
		trigger.last_signal = abs(ref-signal);
		auto elapsed = std::chrono::steady_clock::now() - trigger.last_time;
		trigger.dt = std::chrono::duration<double>(elapsed).count();
		trigger.last_time = std::chrono::steady_clock::now();
		return true;
	}
	return false;
}