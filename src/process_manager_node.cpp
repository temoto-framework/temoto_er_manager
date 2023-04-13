#include "ros/ros.h"
#include <boost/program_options.hpp>
#include "temoto_process_manager/process_manager.hpp"
#include "temoto_resource_registrar/temoto_logging.h"

//#include <sstream>

int main(int argc, char** argv)
{
  namespace po = boost::program_options;
  po::variables_map vm;
  po::options_description desc("Allowed options");
  desc.add_options()
    ("restore-from-catalog", po::value<bool>(), "Restore the state of the manager via RR catalog.");
  
  bool restore_from_catalog{false};
  if (vm.count("restore-from-catalog"))
  {
    restore_from_catalog = vm["restore-from-catalog"].as<bool>();
  }

  TEMOTO_LOG_ATTR.initialize("process_manager");
  ros::init(argc, argv, TEMOTO_LOG_ATTR.getSubsystemName());

  // Create instance of process manager
  temoto_process_manager::ProcessManager pm(restore_from_catalog);

  ros::spin();
  
  return 0;
}
