#ifndef RAMS_INTERFACE_MAIN_WINDOW_H
#define RAMS_INTERFACE_MAIN_WINDOW_H

#include <QMainWindow>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rams_interface/srv/move_to_pose.hpp>

namespace rviz_common
{
class RenderPanel;
class VisualizationManager;
}

namespace rams_interface
{

class MainWindow : public QMainWindow
{
  Q_OBJECT
public:
  explicit MainWindow(QWidget* parent = nullptr);
  ~MainWindow() override;

protected:
  bool eventFilter(QObject* obj, QEvent* event) override;

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void handlePose(const geometry_msgs::msg::PoseStamped& pose);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Client<rams_interface::srv::MoveToPose>::SharedPtr move_client_;

  rviz_common::RenderPanel* render_panel_;
  rviz_common::VisualizationManager* viz_manager_;

public:
  rclcpp::Node::SharedPtr get_node() const { return node_; }
};

}  // namespace rams_interface

#endif  // RAMS_INTERFACE_MAIN_WINDOW_H