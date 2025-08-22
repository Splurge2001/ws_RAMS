cmake_minimum_required(VERSION 3.8)
project(rams_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rviz_common QUIET)
find_package(rviz_rendering QUIET)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveToPose.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

add_executable(move_to_pose_server src/move_to_pose_server.cpp)
target_include_directories(move_to_pose_server PRIVATE include)
ament_target_dependencies(move_to_pose_server
  rclcpp
  geometry_msgs
  std_msgs
  std_srvs
  moveit_ros_planning_interface
)
rosidl_target_interfaces(move_to_pose_server ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(gui_node
  src/main.cpp
  src/main_window.cpp
)
target_include_directories(gui_node PRIVATE include)
target_link_libraries(gui_node Qt5::Widgets)
ament_target_dependencies(gui_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
)
if(rviz_common_FOUND AND rviz_rendering_FOUND)
  ament_target_dependencies(gui_node rviz_common rviz_rendering)
  target_compile_definitions(gui_node PRIVATE HAS_RVIZ)
endif()
rosidl_target_interfaces(gui_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  move_to_pose_server
  gui_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
