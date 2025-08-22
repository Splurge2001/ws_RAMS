#include "rams_interface/main_window.h"

#include <QMouseEvent>
#include <QMessageBox>

#include <rviz/render_panel.h>
#include <rviz/visualization_manager.h>
#include <rviz/selection/selection_manager.h>
#include <rviz/ogre_helpers/qt_ogre_render_window.h>

#include <OgreVector3.h>

namespace rams_interface
{

MainWindow::MainWindow(QWidget* parent)
  : QMainWindow(parent)
{
  render_panel_ = new rviz::RenderPanel(this);
  setCentralWidget(render_panel_);

  viz_manager_ = new rviz::VisualizationManager(render_panel_);
  render_panel_->initialize(viz_manager_->getSceneManager(), viz_manager_);
  viz_manager_->initialize();
  viz_manager_->startUpdate();

  rviz::Display* robot = viz_manager_->createDisplay("rviz/RobotModel", "Robot", true);
  Q_UNUSED(robot);

  rviz::Display* cloud = viz_manager_->createDisplay("rviz/PointCloud2", "Cloud", true);
  if (cloud)
  {
    cloud->subProp("Topic")->setValue("/rams/perception/cloud");
  }

  cloud_sub_ = nh_.subscribe("/rams/perception/cloud", 1, &MainWindow::cloudCallback, this);
  move_client_ = nh_.serviceClient<rams_interface::MoveToPose>("/rams_interface/move_to_pose");

  render_panel_->installEventFilter(this);
}

MainWindow::~MainWindow()
{
}

void MainWindow::cloudCallback(const sensor_msgs::PointCloud2ConstPtr&)
{
  // no-op, subscription maintains connection
}

void MainWindow::handlePose(const geometry_msgs::PoseStamped& pose)
{
  rams_interface::MoveToPose srv;
  srv.request.target = pose;
  if (move_client_.call(srv))
  {
    QString msg = srv.response.message.c_str();
    if (srv.response.success)
      QMessageBox::information(this, "MoveToPose", msg);
    else
      QMessageBox::warning(this, "MoveToPose", msg);
  }
  else
  {
    QMessageBox::warning(this, "MoveToPose", "Service call failed");
  }
}

bool MainWindow::eventFilter(QObject* obj, QEvent* event)
{
  if (obj == render_panel_ && event->type() == QEvent::MouseButtonPress)
  {
    QMouseEvent* mouse = static_cast<QMouseEvent*>(event);
    Ogre::Vector3 pos;
    rviz::SelectionManager* sel = viz_manager_->getSelectionManager();
    Ogre::Viewport* viewport = render_panel_->getRenderWindow()->getViewport(0);
    if (sel->get3DPoint(viewport, mouse->x(), mouse->y(), pos))
    {
      geometry_msgs::PoseStamped ps;
      ps.header.frame_id = viz_manager_->getFixedFrame().toStdString();
      ps.header.stamp = ros::Time::now();
      ps.pose.position.x = pos.x;
      ps.pose.position.y = pos.y;
      ps.pose.position.z = pos.z;
      ps.pose.orientation.w = 1.0;
      handlePose(ps);
    }
    return true;
  }
  return QMainWindow::eventFilter(obj, event);
}

}  // namespace rams_interface
