#include <memory>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>

class PerceptionNode
{
public:
  explicit PerceptionNode(const rclcpp::Node::SharedPtr & node)
  : node_(node),
    tf_buffer_(node_->get_clock()),
    tf_listener_(tf_buffer_)
  {
    node_->declare_parameter("target_frame", "world");
    node_->get_parameter("target_frame", target_frame_);

    cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/rams/perception/cloud", rclcpp::QoS(1));
    planning_scene_pub_ = node_->create_publisher<moveit_msgs::msg::PlanningScene>(
      "planning_scene", rclcpp::QoS(1));

    cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera/depth/points", rclcpp::QoS(1),
      std::bind(&PerceptionNode::cloudCallback, this, std::placeholders::_1));
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    sensor_msgs::msg::PointCloud2 cloud_out;
    try
    {
      tf_buffer_.transform(*msg, cloud_out, target_frame_, tf2::durationFromSec(1.0));
      cloud_pub_->publish(cloud_out);

      moveit_msgs::msg::PlanningScene ps;
      ps.is_diff = true;
      planning_scene_pub_->publish(ps);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(node_->get_logger(), "Transform failed: %s", ex.what());
    }
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_pub_;
  tf2_ros::Buffer tf_buffer_{node_->get_clock()};
  tf2_ros::TransformListener tf_listener_{tf_buffer_};
  std::string target_frame_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("rams_perception_node");
  auto perception_node = std::make_shared<PerceptionNode>(node);
  (void)perception_node; // suppress unused variable warning
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
