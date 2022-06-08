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
#include <nav_msgs/msg/odometry.hpp>

using namespace std::chrono_literals;

class PositionController : public rclcpp::Node
{
public:
  PositionController() : Node("position_controller") {}

  bool initialize();
  bool iterate();

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr GT_pose_;
  void gtposeCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr ref_pose_;
  void positionreferenceCallback(const geometry_msgs::msg::Pose::SharedPtr msg);

  double m_x_init, m_y_init, m_z_init;
  std::string  m_controller_type, m_robot_id, m_controller_mode;
  geometry_msgs::msg::Pose ref_pose;
  nav_msgs::msg::Odometry GT_pose;

};
