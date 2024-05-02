#include <rclcpp/rclcpp.hpp>
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
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;

  auto node = std::make_shared<rclcpp::Node>("process_manager");
  // auto node = std::make_shared<rclcpp::Node>("process_manager_node", TEMOTO_LOG_ATTR.getSubsystemName());


  // Create instance of process manager
  auto pm = std::make_shared<temoto_process_manager::ProcessManager>(restore_from_catalog, node);
  


  exec.add_node(node);
  exec.add_node(pm->resource_registrar_->getResourceRegistrarNode());
  
  exec.spin();
  // rclcpp::spin(node);
  
  return 0;
}