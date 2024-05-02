#include "rclcpp/rclcpp.hpp"
#include "temoto_process_manager/process_manager_interface.hpp"


void resourceFailureCallback(temoto_process_manager::LoadProcess_srv load_process_msg, temoto_resource_registrar::Status status_msg)
{
  RCLCPP_WARN_STREAM(rclcpp::get_logger("resource_failure_callback"), "The following resource stopped unexpectedly\n" << load_process_msg.request.action);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("test_er_client_node");

  std::cout << "Start test node" << std::endl;

  /*
   * Create  Manager Interface object that provides a simplified
   * API for communicating with the  Manager. The boolean "true", that's passed
   * to the constructor of ERM interface tells it whether it should be initialised immediately,
   * or that's done later by the user.
   */
  temoto_process_manager::ProcessManagerInterface pmi(true);

  /*
   * You can register a custom routine (not required) where resource failures are reported.
   */
  pmi.registerUpdateCallback(resourceFailureCallback);

  auto rr_node = pmi.resource_registrar_->getResourceRegistrarNode();
  std::thread spinner_thread([&rr_node]() { rclcpp::spin(rr_node); });

  /*
   * ER Manager allows to invoke ROS executables, ROS launch files and other programs.
   * The "loadRosResource" and "loadSysResource" methods return a "temoto_process_manager::LoadProcess"
   * object which contains the details regarding the query. This can be later used to unload the resource.
   */

  /*
   * Load the Gnome calculator as an example of a regular system program. Additional
   * arguments can also be passed as a second std::string variable.
   */
  RCLCPP_INFO(node->get_logger(), "Loading gnome-calculator");

  temoto_process_manager::LoadProcess_srv load_process_msg = pmi.loadSysResource("gnome-calculator");

  std::this_thread::sleep_for(std::chrono::seconds(5));

  RCLCPP_INFO(node->get_logger(), "Unloading gnome-calculator");
  pmi.unloadResource(load_process_msg);

  /*
   * Load a ROS program an example of a ROS executable (regularly invoked via 'rosrun'). The first
   * parameter indicates the ROS package name and the second indicates the executable. Additional
   * arguments can also be passed as a third std::string variable. The same method can be used to
   * load ROS launch files
   */
  RCLCPP_INFO(node->get_logger(), "Loading rqt_graph");
  pmi.loadRosResource("rqt_graph", "rqt_graph");

  
  RCLCPP_INFO(node->get_logger(), "Loading rviz");
  pmi.loadRosResource("rviz2", "rviz2");
  std::this_thread::sleep_for(std::chrono::seconds(5));

  /*
   * Note that this time the "unloadResource" was not invoked, as the destructor of "pmi" automatically
   * unloads all loaded resources.
   */ 


  while(rclcpp::ok()){}

  rclcpp::shutdown();

  return 0;
}