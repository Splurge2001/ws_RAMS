#include "rams_interface/main_window.h"

#include <QMessageBox>
#include <QMouseEvent>

#include <chrono>
#include <functional>
#include <future>
#include <memory>

#include <rclcpp/clock.hpp>
#include <rviz_common/interaction/selection_manager.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/window_manager_interface.hpp>
#include <rviz_rendering/render_window.hpp>

#include <OgreVector3.h>
#include <OgreViewport.h>

namespace rams_interface {

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
  node_ = rclcpp::Node::make_shared("rams_gui");

  render_panel_ = new rviz_common::RenderPanel(this);
  setCentralWidget(render_panel_);

  ros_node_abstraction_ =
      std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>(node_);
  viz_manager_ = std::make_unique<rviz_common::VisualizationManager>(
      render_panel_, ros_node_abstraction_, nullptr, node_->get_clock());
  render_panel_->initialize(viz_manager_.get());
  viz_manager_->initialize();
  viz_manager_->startUpdate();

  rviz_common::Display *robot = viz_manager_->createDisplay(
      "rviz_default_plugins/RobotModel", "Robot", true);
  Q_UNUSED(robot);

  rviz_common::Display *cloud = viz_manager_->createDisplay(
      "rviz_default_plugins/PointCloud2", "Cloud", true);
  if (cloud) {
    cloud->subProp("Topic")->setValue("/rams/perception/cloud");
  }

  cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/rams/perception/cloud", 1,
      std::bind(&MainWindow::cloudCallback, this, std::placeholders::_1));
  move_client_ = node_->create_client<rams_interface::srv::MoveToPose>(
      "/rams_interface/move_to_pose");

  render_panel_->installEventFilter(this);
}

void MainWindow::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr) {
  // no-op, subscription maintains connection
