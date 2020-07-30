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

#ifndef TEMOTO_ER_MANAGER__TEMOTO_ER_MANAGER_H
#define TEMOTO_ER_MANAGER__TEMOTO_ER_MANAGER_H

#include "temoto_core/trr/resource_registrar.h"
#include "temoto_core/common/base_subsystem.h"
#include "temoto_er_manager/temoto_er_manager_services.h"
#include <stdio.h> //pid_t TODO: check where pid_t actually is
#include <mutex>
#include <thread>
#include <sys/stat.h>

namespace temoto_er_manager
{
class ERManager : public temoto_core::BaseSubsystem
{
public:

  ERManager ();
  virtual ~ERManager ();

  std::string formatRequest(temoto_er_manager::LoadExtResource::Request& req);
  void formatResponse(temoto_er_manager::LoadExtResource::Response &res, int code, std::string message);
  void loadCb(temoto_er_manager::LoadExtResource::Request &req, temoto_er_manager::LoadExtResource::Response &res);
  void unloadCb(temoto_er_manager::LoadExtResource::Request &req, temoto_er_manager::LoadExtResource::Response &res);
  void resourceLoadLoop();
  void resourceUnloadLoop();
  void resourceStatusLoop();

private:

  // TODO: This section should be replaced by a single container which also holds
  // the state of each process.
  std::vector<temoto_er_manager::LoadExtResource> loading_processes_;
  std::map<pid_t, temoto_er_manager::LoadExtResource> running_processes_;
  std::map<pid_t, temoto_er_manager::LoadExtResource> failed_processes_;
  std::vector<pid_t> unloading_processes_;
  std::string catkin_workspace_devel_path_;

  std::thread  resource_loading_thread_;  
  std::thread  resource_unloading_thread_;
  std::thread  resource_status_thread_;

  std::mutex loading_mutex_;
  std::mutex unloading_mutex_;
  std::mutex running_mutex_;

  ros::NodeHandle nh_;

  // Resource management protocol
  temoto_core::trr::ResourceRegistrar<ERManager> resource_registrar_;

  // Listens for calls to start or kill processes
  //ros::ServiceServer spawn_kill_srv_;

  // Holds clients toi connect and send info to other (Sensor, Context, etc.) managers
  //ros::ServiceClient resource_status_client_;
  void waitForLock(std::mutex& m);
  inline bool executableExists (const std::string& name)
  {
      struct stat buffer;
      return (stat(name.c_str(), &buffer) == 0);
  }
};
}

#endif
