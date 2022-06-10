#include <cstdio>
#include <chrono>
#include <array>
#include <cstring>
#include <iostream>
#include <fstream>
#include <time.h>
#include <chrono>
#include <functional>
#include <vector>
#include <typeinfo>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <nav_msgs/msg/odometry.hpp>

struct pid_s{
    double kp, ki, kd, td;
    int nd;
    double error[2], integral, derivative, upperlimit, lowerlimit;
};
struct euler_angles {
    double roll, pitch, yaw;
};
struct threshold {
  double co, ai, cn;
  double dt;
  double noise[20] = {};
  double last_signal = 0.0;
  std::chrono::steady_clock::time_point last_time;
};

using namespace std::chrono_literals;

class PositionController : public rclcpp::Node
{
public:
  PositionController() : Node("position_controller") {
    this->declare_parameter("ROBOT_ID", "Khepera01");
    this->declare_parameter("LKp", 0.);
    this->declare_parameter("LKi", 0.);
    this->declare_parameter("LKd", 0.);
    this->declare_parameter("LTd", 0.);
    this->declare_parameter("WKp", 0.);
    this->declare_parameter("WKi", 0.);
    this->declare_parameter("WKd", 0.);
    this->declare_parameter("WTd", 0.);
    this->declare_parameter("Lco", 0.);
    this->declare_parameter("Lai", 0.);
    this->declare_parameter("Wco", 0.);
    this->declare_parameter("Wai", 0.);
  }

  bool initialize();
  bool iterate();

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr GT_pose_;
  void gtposeCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr ref_pose_;
  void positionreferenceCallback(const geometry_msgs::msg::Pose::SharedPtr msg);

  // Params
  double Kp, Ki, Kd, Td, Co, Ai, Cn;
  double dt = 0.01;
  // Controllers
  struct pid_s l_controller, w_controller;
  double m_x_init, m_y_init, m_z_init;
  double distance, angle, yaw_gt;
  bool first_pose_received, first_ref_received;
  std::string  m_controller_type, m_robot_id, m_controller_mode;
  geometry_msgs::msg::Pose GT_pose, ref_pose;
  // nav_msgs::msg::Odometry GT_pose;

  // Event Based Control
  bool events = false;
  bool z_event, w_event;
  struct threshold l_threshold, w_threshold;

  double pid_controller(struct pid_s &controller, double dt);
  struct pid_s init_controller(const char id[], double kp, double ki, double kd, double td, int nd, double upperlimit, double lowerlimit);
  struct threshold init_triggering(const char id[], double co, double a);
  bool eval_threshold(struct threshold &trigger, double signal, double ref);
};
