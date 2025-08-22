#include "rams_interface/main_window.h"

#include <QMessageBox>
#include <QMouseEvent>

#include <chrono>
#include <future>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/window_manager_interface.hpp>

#include <rviz_rendering/render_window.hpp>
#include <rviz_rendering/viewport_projection_finder.hpp>

#include <OgreVector3.h>

namespace rams_interface {

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
  // 1) 你自己的 ROS 2 节点（用于订阅/服务调用）
  node_ = rclcpp::Node::make_shared("rams_gui");

  // 2) RViz 渲染面板
  render_panel_ = new rviz_common::RenderPanel(this);
  setCentralWidget(render_panel_);
  render_panel_->installEventFilter(this);

  // 3) RViz 的“ROS 节点抽象”（注意：Humble 构造函数只接收节点名字符串）
  ros_node_abstraction_ =
      std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>(
          "rams_interface_gui");

  // 4) VisualizationManager（Humble 需要 4 个参数）
  rviz_common::WindowManagerInterface *window_manager = nullptr;
  viz_manager_ = std::make_unique<rviz_common::VisualizationManager>(
      render_panel_,
      rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr(ros_node_abstraction_),
      window_manager,
      node_->get_clock());

  // 先初始化管理器，再固定坐标系，然后初始化渲染面板（传 DisplayContext）
  viz_manager_->initialize();
  viz_manager_->setFixedFrame("base_link");
  render_panel_->initialize(viz_manager_.get(), true);
  viz_manager_->startUpdate();

  // 5) 添加 RobotModel 与 PointCloud2 显示，并设置点云话题
  {
    rviz_common::Display *robot =
        viz_manager_->createDisplay("rviz_default_plugins/RobotModel", "Robot", true);
    (void)robot;
  }
  {
    rviz_common::Display *cloud =
        viz_manager_->createDisplay("rviz_default_plugins/PointCloud2", "Cloud", true);
    if (cloud) {
      cloud->subProp("Topic")->setValue("/rams/perception/cloud");
    }
  }

  // 6) 订阅点云（保持连接；如果不需要可删）
  cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/rams/perception/cloud", rclcpp::SensorDataQoS(),
      std::bind(&MainWindow::cloudCallback, this, std::placeholders::_1));

  // 7) MoveToPose 服务客户端（服务名按你的服务端实际名字改）
  move_client_ =
      node_->create_client<rams_interface::srv::MoveToPose>("move_to_pose");
}

void MainWindow::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr) {
  // no-op: 仅维持订阅连接，需要时在这里处理点云
}

void MainWindow::handlePose(const geometry_msgs::msg::PoseStamped &pose) {
  auto request = std::make_shared<rams_interface::srv::MoveToPose::Request>();
  request->target = pose;

  if (!move_client_->wait_for_service(std::chrono::seconds(1))) {
    QMessageBox::warning(this, "MoveToPose", "Service not available");
    return;
  }

  auto future = move_client_->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
    auto response = future.get();
    const QString msg = QString::fromStdString(response->message);
    if (response->success) {
      QMessageBox::information(this, "MoveToPose", msg);
    } else {
      QMessageBox::warning(this, "MoveToPose", msg);
    }
  } else {
    QMessageBox::warning(this, "MoveToPose", "Service call timeout");
  }
}

bool MainWindow::eventFilter(QObject *obj, QEvent *event) {
  if (obj == render_panel_ && event->type() == QEvent::MouseButtonPress) {
    auto *mouse = static_cast<QMouseEvent *>(event);

    // 使用 rviz_rendering 的投影工具，把鼠标点击投到 Z=0 的 XY 平面
    rviz_rendering::ViewportProjectionFinder vpf;
    auto rw = render_panel_->getRenderWindow();  // rviz_rendering::RenderWindow*
    auto proj = vpf.getViewportPointProjectionOnXYPlane(
        rw, mouse->x(), mouse->y());

    if (proj.first) {
      const Ogre::Vector3 &pos = proj.second;

      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = viz_manager_->getFixedFrame().toStdString();
      ps.header.stamp = node_->now();
      ps.pose.position.x = pos.x;
      ps.pose.position.y = pos.y;
      ps.pose.position.z = pos.z;
      ps.pose.orientation.w = 1.0;  // 朝向先给单位四元数

      handlePose(ps);
    }
    return true;  // 已处理该事件
  }
  return QMainWindow::eventFilter(obj, event);
}

} // namespace rams_interface
