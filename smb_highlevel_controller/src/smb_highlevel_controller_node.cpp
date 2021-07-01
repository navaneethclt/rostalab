#include <ros/ros.h>
#include "smb_highlevel_controller/SmbHighlevelController.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "smb_highlevel_controller");
  ros::NodeHandle nodeHandle("~");

  smb_highlevel_controller::SmbHighlevelController smbHighlevelController(nodeHandle);
  ros::Rate loop_rate(100);
  while(ros::ok())
  {
  ros::spinOnce();
  loop_rate.sleep();
  }
  return 0;
}
