#include "rams_interface/main_window.h"

#include <QMessageBox>
#include <QMouseEvent>

#include <chrono>
#include <functional>
#include <future>
#include <memory>

#include <rviz_common/interaction/selection_manager.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_rendering/render_window.hpp>

#include <OgreVector3.h>
#include <OgreViewport.h>

namespace rams_interface {

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
  node_ = rclcpp::Node::make_shared("rams_gui");

  render_panel_ = new rviz_common::RenderPanel(this);
  setCentralWidget(render_panel_);

  viz_manager_ =
      std::make_unique<rviz_common::VisualizationManager>(render_panel_);
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
}

void MainWindow::handlePose(const geometry_msgs::msg::PoseStamped &pose) {
  auto request = std::make_shared<rams_interface::srv::MoveToPose::Request>();
  request->target = pose;
  auto future = move_client_->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
    auto response = future.get();
    QString msg = QString::fromStdString(response->message);
    if (response->success)
      QMessageBox::information(this, "MoveToPose", msg);
    else
      QMessageBox::warning(this, "MoveToPose", msg);
  } else {
    QMessageBox::warning(this, "MoveToPose", "Service call failed");
  }
}

bool MainWindow::eventFilter(QObject *obj, QEvent *event) {
  if (obj == render_panel_ && event->type() == QEvent::MouseButtonPress) {
    QMouseEvent *mouse = static_cast<QMouseEvent *>(event);
    Ogre::Vector3 pos;
    auto *sel = viz_manager_->getSelectionManager();
    Ogre::Viewport *viewport = render_panel_->getRenderWindow()->getViewport(0);
    if (sel->get3DPoint(viewport, mouse->x(), mouse->y(), pos)) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = viz_manager_->getFixedFrame().toStdString();
      ps.header.stamp = node_->now();
      ps.pose.position.x = pos.x;
      ps.pose.position.y = pos.y;
      ps.pose.position.z = pos.z;
      ps.pose.orientation.w = 1.0;
      handlePose(ps);
    }
    return true;
  }
  return QMainWindow::eventFilter(obj, event);
}

} // namespace rams_interface