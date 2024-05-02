/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Copyright 2020 TeMoto Telerobotics
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifndef TEMOTO_PROCESS_MANAGER__PROCESS_MANAGER_H
#define TEMOTO_PROCESS_MANAGER__PROCESS_MANAGER_H

#include "rr/ros2_resource_registrar.h"
#include "temoto_process_manager/process_manager_services.hpp"
#include <unistd.h>
#include <mutex>
#include <thread>
#include <sys/stat.h>

namespace temoto_process_manager
{

struct LoadProcess_srv 
{
  temoto_process_manager::srv::LoadProcess::Request request;
  temoto_process_manager::srv::LoadProcess::Response response;
}; 

class ProcessManager
{
public:

  ProcessManager (bool restore_from_catalog, std::shared_ptr<rclcpp::Node> node);
  virtual ~ProcessManager ();

  std::string formatRequest(temoto_process_manager::srv::LoadProcess::Request& req);
  void formatResponse(temoto_process_manager::srv::LoadProcess::Response &res, int code, std::string message);
  void loadCb(temoto_process_manager::srv::LoadProcess::Request &req, temoto_process_manager::srv::LoadProcess::Response &res);
  void unloadCb(temoto_process_manager::srv::LoadProcess::Request &req, temoto_process_manager::srv::LoadProcess::Response &res);
  void statusCb(temoto_process_manager::srv::LoadProcess::Request &req, temoto_process_manager::srv::LoadProcess::Response &res, const temoto_resource_registrar::Status &status);
  void resourceLoadLoop();
  void resourceUnloadLoop();
  void resourceStatusLoop();

  std::shared_ptr<temoto_resource_registrar::ResourceRegistrarRos2> resource_registrar_;
  

private:

  // TODO: This section should be replaced by a single container which also holds
  // the state of each process.
  std::vector<LoadProcess_srv> loading_processes_;
  std::map<pid_t, LoadProcess_srv> running_processes_;
  std::map<pid_t, LoadProcess_srv> failed_processes_;
  std::vector<pid_t> unloading_processes_;
  std::string colcon_workspace_install_path_;

  std::thread resource_loading_thread_;  
  std::thread resource_unloading_thread_;
  std::thread resource_status_thread_;

  std::mutex loading_mutex_;
  std::mutex unloading_mutex_;
  std::mutex running_mutex_;

  temoto_resource_registrar::Configuration rr_catalog_config_;

  std::shared_ptr<rclcpp::Node> node_;

  void waitForLock(std::mutex& m);
  inline bool executableExists (const std::string& name)
  {
    struct stat buffer;
    return (stat(name.c_str(), &buffer) == 0);
  }
};
}

#endif
