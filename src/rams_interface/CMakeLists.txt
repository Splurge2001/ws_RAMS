cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0071 NEW)
project(rams_interface)

# C++ 标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Qt 自动 MOC
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)

# ====== 接口（srv）生成 ======
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveToPose.srv"
  DEPENDENCIES geometry_msgs std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

# 获取 C++ 类型支持目标（替代 rosidl_target_interfaces）
rosidl_get_typesupport_target(${PROJECT_NAME}__ts_cpp ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ====== move_to_pose_server ======
add_executable(move_to_pose_server
  src/move_to_pose_server.cpp
)
target_include_directories(move_to_pose_server PRIVATE include)
ament_target_dependencies(move_to_pose_server
  rclcpp
  geometry_msgs
  std_msgs
  std_srvs
  moveit_ros_planning_interface
)
target_link_libraries(move_to_pose_server ${${PROJECT_NAME}__ts_cpp})

# ====== gui_node（Qt + RViz2）======
add_executable(gui_node
  src/main.cpp
  src/main_window.cpp
  include/rams_interface/main_window.h   # 含 Q_OBJECT，确保 AUTOMOC 处理
)
target_include_directories(gui_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(gui_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  rviz_common
  rviz_rendering
)
target_link_libraries(gui_node
  Qt5::Widgets
  rviz_common::rviz_common
  rviz_rendering::rviz_rendering
  ${${PROJECT_NAME}__ts_cpp}
)

# ====== 安装 ======
install(TARGETS
  move_to_pose_server
  gui_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
