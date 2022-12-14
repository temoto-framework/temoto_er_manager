#include "ros/ros.h"
#include "temoto_process_manager/process_manager.hpp"
#include "temoto_resource_registrar/temoto_logging.h"

//#include <sstream>

int main(int argc, char** argv)
{
  TEMOTO_LOG_ATTR.initialize("process_manager");
  ros::init(argc, argv, TEMOTO_LOG_ATTR.getSubsystemName());

  // Create instance of process manager
  temoto_process_manager::ProcessManager erm;

  ros::spin();
  
  return 0;
}
