#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "rams_interface/main_window.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);
  rams_interface::MainWindow w;

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(w.get_node());
  std::thread spin_thread([&executor]() { executor.spin(); });

  w.show();
  int ret = app.exec();

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return ret;
}
