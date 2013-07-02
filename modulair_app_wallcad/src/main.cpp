
#include <iostream>

#include <ros/ros.h>
#include "wallcad.h"

int main(int argc, char** argv)
{
  std::cout << "hello" << std::endl;

  ros::init(argc, argv, "wallcad_app");
  ros::NodeHandle node_handle;
  QApplication application(argc, argv);
  // quit the application once any window is closed.
  application.connect(&application, SIGNAL(lastWindowClosed()),
                      &application, SLOT(quit()));

  // wallcad app
  modulair::WallCadApp wallcad_app("WallCadApp", node_handle, 20);
  wallcad_app.build();
  wallcad_app.start();
  ROS_WARN_STREAM("WallCadApp: App Running");
  application.exec();
  wallcad_app.stop();
  ROS_WARN_STREAM("WallCadApp: App Finished");

  return 0;
}
