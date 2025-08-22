#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "rams_interface/MoveToPose.h"

class MoveToPoseServer
{
public:
  MoveToPoseServer(const std::string& planning_group)
    : move_group_(planning_group)
  {
    service_ = nh_.advertiseService("move_to_pose", &MoveToPoseServer::callback, this);
  }

  bool callback(rams_interface::MoveToPose::Request& req,
                rams_interface::MoveToPose::Response& res)
  {
    move_group_.setPoseTarget(req.target);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
    {
      success = (move_group_.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }
    res.success = success;
    return true;
  }

private:
  ros::NodeHandle nh_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  ros::ServiceServer service_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_to_pose_server");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle pnh("~");
  std::string group;
  pnh.param<std::string>("planning_group", group, "manipulator");

  MoveToPoseServer server(group);
  ros::waitForShutdown();
  return 0;
}