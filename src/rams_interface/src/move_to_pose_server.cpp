#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "rams_interface/srv/move_to_pose.hpp"
#include <memory>
#include <string>
#include <functional>

class MoveToPoseServer
{
public:
  MoveToPoseServer(const rclcpp::Node::SharedPtr& node, const std::string& planning_group)
    : node_(node), move_group_(node_, planning_group)
  {
    service_ = node_->create_service<rams_interface::srv::MoveToPose>(
      "/rams_interface/move_to_pose",
      std::bind(&MoveToPoseServer::callback, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void callback(const std::shared_ptr<rams_interface::srv::MoveToPose::Request> req,
                std::shared_ptr<rams_interface::srv::MoveToPose::Response> res)
  {
    move_group_.setPoseTarget(req->target);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success)
    {
      success = (move_group_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    }
    res->success = success;
    res->message = success ? "Motion executed" : "Planning or execution failed";
  }

  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  rclcpp::Service<rams_interface::srv::MoveToPose>::SharedPtr service_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("move_to_pose_server");
  node->declare_parameter<std::string>("planning_group", "manipulator");
  std::string group = node->get_parameter("planning_group").as_string();
  MoveToPoseServer server(node, group);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}