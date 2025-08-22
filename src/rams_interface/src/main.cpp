#include <QApplication>
#include <ros/ros.h>
#include "rams_interface/main_window.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rams_gui");
  QApplication app(argc, argv);
  rams_interface::MainWindow w;
  w.show();
  return app.exec();
}