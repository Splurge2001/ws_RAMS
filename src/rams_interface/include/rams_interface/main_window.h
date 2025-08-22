#ifndef RAMS_INTERFACE_MAIN_WINDOW_H
#define RAMS_INTERFACE_MAIN_WINDOW_H

#include <QMainWindow>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <rams_interface/MoveToPose.h>

namespace rviz
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
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
  void handlePose(const geometry_msgs::PoseStamped& pose);

  ros::NodeHandle nh_;
  ros::Subscriber cloud_sub_;
  ros::ServiceClient move_client_;

  rviz::RenderPanel* render_panel_;
  rviz::VisualizationManager* viz_manager_;
};

}  // namespace rams_interface

#endif  // RAMS_INTERFACE_MAIN_WINDOW_H