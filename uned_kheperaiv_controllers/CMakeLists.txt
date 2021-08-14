cmake_minimum_required(VERSION 3.8)
project(uned_kheperaiv_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

ament_export_dependencies(rclcpp std_msgs rclpy sensor_msgs geometry_msgs Eigen3)
include_directories(
  include/${PROJECT_NAME}
  ${${PROJECT_NAME}_SOURCE_DIR}/include
)

add_executable(eventbased_pid_position_controller src/eventbased_pid_position_controller.cpp)
ament_target_dependencies(eventbased_pid_position_controller
  rclcpp std_msgs rclpy sensor_msgs geometry_msgs)

add_executable(periodic_pid_position_controller src/periodic_pid_position_controller.cpp)
ament_target_dependencies(periodic_pid_position_controller
  rclcpp std_msgs rclpy sensor_msgs geometry_msgs)

add_executable(event_triggering src/event_triggering.cpp)
ament_target_dependencies(event_triggering
  rclcpp std_msgs rclpy sensor_msgs geometry_msgs)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include)

ament_package()
