#pragma once

// ---- ROS2 基础 ----
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// ---- TF2 ----
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// ---- 线性代数 / PCL 基础类型（实现里会用）----
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// ---- 你自定义的服务接口（见下方 .srv 内容）----
#include "roi_nav/srv/region_to_goal.hpp"

namespace roi_nav
{

/**
 * @brief ROI 感知服务节点：接收用户圈定的区域(Polygon)，
 *        从最近点云裁剪、拟合平面，计算落点/姿态并返回 Pose。
 *
 * 典型流程（在 .cpp 实现）：
 * 1) 订阅点云，缓存 last_cloud_
 * 2) Service 回调中将 ROI 多边形变换到 world_frame_
 * 3) 用 ROI 裁剪 last_cloud_ -> roi_cloud
 * 4) RANSAC 拟合平面，算中心点与法向
 * 5) 生成机器人目标位姿（姿态对齐平面法向 + 接近偏移）
 * 6) 发布可视化 Marker
 */
class RoiServer : public rclcpp::Node
{
public:
  explicit RoiServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~RoiServer() override = default;

private:
  // ========== 回调 ==========
  /// 缓存最近点云（QoS: SENSOR_DATA）
  void cloudCb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);

  /// 服务回调：输入 ROI 多边形，输出目标位姿
  void handleSelectRoi(
      const std::shared_ptr<roi_nav::srv::SelectRoi::Request> req,
      std::shared_ptr<roi_nav::srv::SelectRoi::Response> res);

  // ========== 算法辅助函数（在 .cpp 里实现） ==========
  using Cloud = pcl::PointCloud<pcl::PointXYZRGB>;
  using CloudConstPtr = Cloud::ConstPtr;
  using CloudPtr = Cloud::Ptr;

  /// 将 ROI 多边形(任意 frame)变换到 world_frame_（失败返回 false）
  bool toWorldPolygon(const geometry_msgs::msg::PolygonStamped& in_poly,
                      geometry_msgs::msg::Polygon& out_world_poly);

  /// 依据多边形裁剪点云（假设多边形位于 world_frame_ 平面上）
  bool clipCloudByPolygon(const CloudConstPtr& in,
                          const geometry_msgs::msg::Polygon& poly_world,
                          CloudPtr& out);

  /// RANSAC 拟合平面，返回法向和 d（平面式 n·x + d = 0）
  bool fitPlaneRansac(const CloudConstPtr& in,
                      Eigen::Vector3d& n_world,
                      double& d_world,
                      Eigen::Vector3d& centroid_world);

  /// 由平面法向 + 质心 计算目标位姿（附加 approach_offset_ 沿法向偏置）
  bool computeTargetPose(const Eigen::Vector3d& plane_n,
                         const Eigen::Vector3d& centroid,
                         geometry_msgs::msg::PoseStamped& out_pose);

  /// 可视化发布（ROI 点云、坐标轴、文本等）
  void publishDebug(const CloudConstPtr& roi,
                    const geometry_msgs::msg::PoseStamped& target);

  // ========== ROS 通讯对象 ==========
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Service<roi_nav::srv::SelectRoi>::SharedPtr srv_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_pub_;

  // ========== 缓存 ==========
  sensor_msgs::msg::PointCloud2::ConstSharedPtr last_cloud_;  // 最近点云

  // ========== TF ==========
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ========== 参数 ==========
  std::string cloud_topic_;        // 点云话题名（如 /camera/depth/color/points）
  std::string camera_frame_;       // 相机坐标系（点云原始 frame）
  std::string world_frame_;        // 规划/显示使用的世界坐标系
  double voxel_leaf_;              // 体素滤波叶子大小
  double plane_dist_thresh_;       // 平面 RANSAC 距离阈值
  int    plane_max_iter_;          // 平面 RANSAC 最大迭代
  double approach_offset_;         // 末端沿法向的接近偏置（m）
  bool   debug_viz_;               // 是否发布调试可视化
};

} // namespace roi_nav
