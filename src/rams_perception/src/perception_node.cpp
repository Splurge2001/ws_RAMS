#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <moveit_msgs/PlanningScene.h>

class PerceptionNode
{
public:
  PerceptionNode()
  : tf_listener_(tf_buffer_)
  {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    pnh.param<std::string>("target_frame", target_frame_, std::string("world"));

    cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/rams/perception/cloud", 1);
    planning_scene_pub_ = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

    cloud_sub_ = nh.subscribe("/camera/depth/points", 1, &PerceptionNode::cloudCallback, this);
  }

private:
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    sensor_msgs::PointCloud2 cloud_out;
    try
    {
      tf_buffer_.transform(*msg, cloud_out, target_frame_, ros::Duration(1.0));
      cloud_pub_.publish(cloud_out);

      moveit_msgs::PlanningScene ps;
      ps.is_diff = true;
      planning_scene_pub_.publish(ps);
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN_STREAM("Transform failed: " << ex.what());
    }
  }

  ros::Subscriber cloud_sub_;
  ros::Publisher cloud_pub_;
  ros::Publisher planning_scene_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string target_frame_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rams_perception_node");
  PerceptionNode node;
  ros::spin();
  return 0;
}
