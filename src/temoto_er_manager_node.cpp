#include "ros/ros.h"
#include "temoto_er_manager/temoto_er_manager.h"

//#include <sstream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "temoto_er_manager");

  // Create instance of process manager
  temoto_er_manager::ERManager erm;

  ros::spin();
  
  return 0;
}
