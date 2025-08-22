#ifndef RAMS_INTERFACE_MAIN_WINDOW_H
#define RAMS_INTERFACE_MAIN_WINDOW_H

#include <QMainWindow>
#include <memory>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>

#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>

#include <rams_interface/srv/move_to_pose.hpp>

namespace rviz_common {
class RenderPanel;
}

namespace rams_interface {

class MainWindow : public QMainWindow {
  Q_OBJECT
public:
  explicit MainWindow(QWidget *parent = nullptr);
  ~MainWindow() override = default;

  // ★ 加回这个 getter，供 main.cpp 使用
  rclcpp::Node::SharedPtr get_node() const { return node_; }

protected:
  bool eventFilter(QObject *obj, QEvent *event) override;

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void handlePose(const geometry_msgs::msg::PoseStamped &pose);

  // 你的 ROS 节点与通信对象
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Client<rams_interface::srv::MoveToPose>::SharedPtr move_client_;

  // RViz2 嵌入相关
  rviz_common::RenderPanel *render_panel_{nullptr};
  std::unique_ptr<rviz_common::VisualizationManager> viz_manager_;
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> ros_node_abstraction_;
};

} // namespace rams_interface

#endif // RAMS_INTERFACE_MAIN_WINDOW_H
