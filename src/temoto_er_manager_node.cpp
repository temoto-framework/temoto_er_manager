#include "ros/ros.h"
#include "temoto_er_manager/temoto_er_manager.h"
#include "temoto_resource_registrar/temoto_logging.h"

//#include <sstream>

int main(int argc, char** argv)
{
  TEMOTO_LOG_ATTR.initialize("er_manager");
  ros::init(argc, argv, TEMOTO_LOG_ATTR.getSubsystemName());

  // Create instance of process manager
  temoto_er_manager::ERManager erm;

  ros::spin();
  
  return 0;
}
