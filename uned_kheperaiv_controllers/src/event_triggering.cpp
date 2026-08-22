#include "uned_kheperaiv_controllers/KheperaEventTriggering.hpp"
using std::placeholders::_1;

bool EventTriggering::initialize(){
  RCLCPP_INFO(this->get_logger(),"EventTriggering::inicialize() ok.");

  // Triggering
  // events_ = this->create_publisher<uned_crazyflie_config::msg::Triggering>("events",10);

  // Crazyflie Pose
  GT_pose_ = this->create_subscription<geometry_msgs::msg::Pose>("ground_truth/pose", 10, std::bind(&EventTriggering::gtposeCallback, this, _1));
	// Reference:
  ref_pose_ = this->create_subscription<geometry_msgs::msg::Pose>("position_reference", 10, std::bind(&EventTriggering::positionreferenceCallback, this, _1));

  return true;
}

bool EventTriggering::iterate(){
  RCLCPP_INFO(this->get_logger(),"EventTriggering::iterate() ok.");
/*
  // Altitude Controller
  {
    if(events[0]){
      events[0] = false;
    }
    // Cn
    for(int i = 0; i<19; i++){
      z_value[i] = z_value[i+1];
    }
    z_value[19] = GT_pose.position.z;
    cn[0] = noiseEstimation(z_value, 20);
    // a
    error[0] = ref_pose.position.z - GT_pose.position.z;
    // Threshold
    threshold[0] = c0[0] + a[0]*abs(error[0]) + cn[0];
    // Signal
    signal = abs(error[0]-lastZSignal);
    // signal = abs(GT_pose.position.z-lastZSignal);
    // Check
    if(signal >= threshold[0]){
      events[0] = true;
      lastZSignal = error[0];
      // lastZSignal = GT_pose.position.z;
    }
  }
  // X-Y Controller
  // Convert quaternion to yw
	double siny_cosp_ref = 2 * (ref_pose.orientation.w*ref_pose.orientation.z+ref_pose.orientation.x*ref_pose.orientation.y);
	double cosy_cosp_ref = 1 - 2 * (ref_pose.orientation.y*ref_pose.orientation.y + ref_pose.orientation.z*ref_pose.orientation.z);
	double yaw_ref = std::atan2(siny_cosp_ref,cosy_cosp_ref);
	double siny_cosp = 2 * (GT_pose.orientation.w*GT_pose.orientation.z+GT_pose.orientation.x*GT_pose.orientation.y);
	double cosy_cosp = 1 - 2 * (GT_pose.orientation.y*GT_pose.orientation.y + GT_pose.orientation.z*GT_pose.orientation.z);
	double yaw = std::atan2(siny_cosp,cosy_cosp);
  {
    if(events[1]){
      events[1] = false;
    }
    x_error = (ref_pose.position.x - GT_pose.position.x)*cos(yaw)+(ref_pose.position.y - GT_pose.position.y)*sin(yaw);
    y_error = -(ref_pose.position.x - GT_pose.position.x)*sin(yaw)+(ref_pose.position.y - GT_pose.position.y)*cos(yaw);
    cn[1] = noiseEstimation(xy_signal, 20);
    error[1] = sqrt(pow(x_error,2)+pow(y_error,2));
    threshold[1] = c0[1] + a[1]*abs(error[1]) + cn[1];
    events[1] = false;
  }
  // Yaw Controller
  {
    if(events[2]){
      events[2] = false;
    }
    error[2] = (yaw_ref-yaw)*180/3.14159265;
    cn[2] = noiseEstimation(yaw_signal, 20);
    threshold[2] = c0[2] + a[2]*abs(error[2]) + cn[2];
    events[2] = false;
  }
  // Attitude Controller
  {
    if(events[3]){
      events[3] = false;
    }
    threshold[3] = c0[3] + a[3]*abs(error[3]) + cn[3];
    events[3] = true;
  }
  // Rate Controller
  {
    if(events[4]){
      events[4] = false;
    }
    threshold[4] = c0[4] + a[4]*abs(error[4]) + cn[4];
    events[4] = true;
  }

  auto msg_trig = uned_crazyflie_config::msg::Triggering();
  msg_trig.trig.clear();
  for(int i=0; i<5; i++){
    msg_trig.trig.push_back(events[i]);
  }
  events_->publish(msg_trig);
  for(int i=0; i<5; i++){
    events[i] = false;
  }
*/
  return true;
}

double EventTriggering::noiseEstimation(double signal[], int len){
  double sum = 0;
  for(int i =0; i<len; i++){
    sum += signal[i];
  }
  double mean = sum/len;
  double noise = 0.0;
  for(int i =0; i<len; i++){
    if(abs(signal[i]-mean)>noise){
      noise = abs(signal[i]-mean);
    }
  }
  return noise;
}

int main(int argc, char ** argv){
  try{
    rclcpp::init(argc, argv);
    auto event_triggering_node = std::make_shared<EventTriggering>();
    rclcpp::Rate loop_rate(10);
    event_triggering_node->initialize();

    while (rclcpp::ok()){
      event_triggering_node->iterate();
      rclcpp::spin_some(event_triggering_node);
      loop_rate.sleep();
    }

    return 0;
  } catch (std::exception &e){
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception: %s",e.what());
	}
}
