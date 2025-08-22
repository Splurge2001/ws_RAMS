cmake_minimum_required(VERSION 3.8)
project(rams_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

add_executable(perception_node src/perception_node.cpp)
ament_target_dependencies(perception_node
  rclcpp
  sensor_msgs
  tf2_ros
  tf2_sensor_msgs
  moveit_msgs
)

install(TARGETS perception_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()