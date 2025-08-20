#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "roi_nav/srv/region_to_goal.hpp"

using roi_nav::srv::RegionToGoal;

class RoiServer : public rclcpp::Node {
public:
  RoiServer(): Node("roi_server") {
    std::string cloud_topic = declare_parameter<std::string>("cloud_topic", "/d415_fixed/depth/color/points");
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg){ last_cloud_ = *msg; });

    srv_ = create_service<RegionToGoal>("roi_nav/region_to_goal",
      std::bind(&RoiServer::handle, this, std::placeholders::_1, std::placeholders::_2));

    tf_buf_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_ = std::make_unique<tf2_ros::TransformListener>(*tf_buf_);
  }

private:
  void handle(const std::shared_ptr<RegionToGoal::Request> req,
              std::shared_ptr<RegionToGoal::Response> res) {
    if (last_cloud_.data.empty()) {
      res->success = false; res->message = "no point cloud yet"; return;
    }

    // 1) 点云 -> base_link
    sensor_msgs::msg::PointCloud2 cloud_base = last_cloud_;
    try {
      auto tf = tf_buf_->lookupTransform("base_link", last_cloud_.header.frame_id, tf2::TimePointZero);
      tf2::doTransform(last_cloud_, cloud_base, tf);
    } catch (const std::exception& e) {
      res->success = false; res->message = std::string("tf error: ") + e.what(); return;
    }

    // 2) 转 PCL 并下采样
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(cloud_base, *pc);
    pcl::VoxelGrid<pcl::PointXYZ> vg; vg.setLeafSize(0.02f,0.02f,0.02f); vg.setInputCloud(pc);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ds(new pcl::PointCloud<pcl::PointXYZ>);
    vg.filter(*pc_ds);

    // 3) 裁剪到盒子 ROI（以 base_link）
    Eigen::Vector3f t(req->box_pose.pose.position.x,
                      req->box_pose.pose.position.y,
                      req->box_pose.pose.position.z);
    Eigen::Quaternionf q( req->box_pose.pose.orientation.w,
                          req->box_pose.pose.orientation.x,
                          req->box_pose.pose.orientation.y,
                          req->box_pose.pose.orientation.z );
    Eigen::Affine3f T = Eigen::Translation3f(t) * q;

    pcl::CropBox<pcl::PointXYZ> crop;
    crop.setTransform(T.inverse());  // 把点到盒子局部系
    Eigen::Vector4f min(-req->box_size.x/2.0, -req->box_size.y/2.0, -req->box_size.z/2.0, 1);
    Eigen::Vector4f max( req->box_size.x/2.0,  req->box_size.y/2.0,  req->box_size.z/2.0, 1);
    crop.setMin(min); crop.setMax(max);
    crop.setInputCloud(pc_ds);
    pcl::PointCloud<pcl::PointXYZ>::Ptr roi(new pcl::PointCloud<pcl::PointXYZ>);
    crop.filter(*roi);

    if (roi->empty()) { res->success=false; res->message="ROI empty"; return; }

    // 4) RANSAC 平面（最大内点）
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(200);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(roi);
    seg.segment(*inliers, *coeff);
    if (inliers->indices.empty()) { res->success=false; res->message="no plane"; return; }

    // 平面 ax+by+cz+d=0，法向 n=(a,b,c)
    Eigen::Vector3f n(coeff->values[0], coeff->values[1], coeff->values[2]);
    n.normalize();

    // 5) 质心
    Eigen::Vector3f c(0,0,0);
    for (int idx: inliers->indices) c += roi->points[idx].getVector3fMap();
    c /= static_cast<float>(inliers->indices.size());

    // 让法向朝向机器人（可选）
    if (c.dot(n) < 0) n = -n;

    // 6) 目标位姿：按法向外退 approach_offset
    Eigen::Vector3f p = c + static_cast<float>(req->approach_offset) * n;

    // 让 eef 某轴对齐法向，默认 z 轴
    Eigen::Vector3f axis(0,0,1);
    if (req->eef_axis=="x") axis=Eigen::Vector3f(1,0,0);
    if (req->eef_axis=="y") axis=Eigen::Vector3f(0,1,0);
    Eigen::Quaternionf q_align = Eigen::Quaternionf::FromTwoVectors(axis, n);

    res->target_pose.header.frame_id = "base_link";
    res->target_pose.pose.position.x = p.x();
    res->target_pose.pose.position.y = p.y();
    res->target_pose.pose.position.z = p.z();
    res->target_pose.pose.orientation.w = q_align.w();
    res->target_pose.pose.orientation.x = q_align.x();
    res->target_pose.pose.orientation.y = q_align.y();
    res->target_pose.pose.orientation.z = q_align.z();
    res->success = true; res->message = "ok";
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  sensor_msgs::msg::PointCloud2 last_cloud_;
  rclcpp::Service<RegionToGoal>::SharedPtr srv_;
  std::unique_ptr<tf2_ros::Buffer> tf_buf_;
  std::unique_ptr<tf2_ros::TransformListener> tf_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoiServer>());
  rclcpp::shutdown();
  return 0;
}
