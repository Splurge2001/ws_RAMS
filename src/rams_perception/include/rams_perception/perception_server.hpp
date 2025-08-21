#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "rams_perception/srv/region_to_goal.hpp"

/// Simple perception server that converts a selected region to a grasp goal.
class PerServer : public rclcpp::Node {
public:
  explicit PerServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PerServer() override = default;

private:
  void handle(const std::shared_ptr<rams_perception::srv::RegionToGoal::Request> req,
              std::shared_ptr<rams_perception::srv::RegionToGoal::Response> res);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Service<rams_perception::srv::RegionToGoal>::SharedPtr srv_;

  sensor_msgs::msg::PointCloud2 last_cloud_;

  std::unique_ptr<tf2_ros::Buffer> tf_buf_;
  std::unique_ptr<tf2_ros::TransformListener> tf_;
};
