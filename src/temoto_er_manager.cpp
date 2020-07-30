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

#include "ros/package.h"
#include "temoto_er_manager/temoto_er_manager.h"
#include <stdio.h>
#include <csignal>
#include <sys/wait.h>
#include <algorithm>
#include <spawn.h>
#include <regex>

namespace temoto_er_manager
{
using namespace temoto_core;

ERManager::ERManager() 
: BaseSubsystem( "temoto_er_manager", error::Subsystem::PROCESS_MANAGER, __func__)
, resource_registrar_(srv_name::MANAGER, this)
{
  resource_registrar_.addServer<LoadExtResource>( srv_name::SERVER
                                                , &ERManager::loadCb
                                                , &ERManager::unloadCb);

  /*
   * Find the catkin workspace
   */
  const std::string current_node_path = ros::package::getPath(ROS_PACKAGE_NAME);
  std::vector<std::string> current_node_path_tokens;
  boost::split(current_node_path_tokens, current_node_path, boost::is_any_of("/"));

  // Remove all tokens up to "src" token. TODO: May potentially cause problems
  // if duplicate "src" tokens are present.
  bool src_token_found = false;
  while(!src_token_found)
  {
    if (current_node_path_tokens.size() == 0)
    {
      break;
    }

    if(current_node_path_tokens.back() != "src")
    {
      current_node_path_tokens.pop_back();
    }
    else
    {
      current_node_path_tokens.pop_back();
      src_token_found = true;
      break;
    }
  }

  // Assemble the devel path string
  for (const auto& token : current_node_path_tokens)
  {
    catkin_workspace_devel_path_ += token + "/";
  }
  catkin_workspace_devel_path_ += "devel/";

  // Start the load, unload and status monitoring threads
  resource_loading_thread_ = std::thread(&ERManager::resourceLoadLoop, this);;  
  resource_unloading_thread_ = std::thread(&ERManager::resourceUnloadLoop, this);;
  resource_status_thread_ = std::thread(&ERManager::resourceStatusLoop, this);;

  TEMOTO_INFO("Process manager is ready.");
}

ERManager::~ERManager()
{
  while (!resource_loading_thread_.joinable())
  {
    std::cout << "[" << __func__ << "] " << "waiting for the resource loading thread to finish ..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  resource_loading_thread_.join();

  while (!resource_unloading_thread_.joinable())
  {
    std::cout << "[" << __func__ << "] " << "waiting for the resource unloading thread to finish ..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  resource_unloading_thread_.join();

  while (!resource_status_thread_.joinable())
  {
    std::cout << "[" << __func__ << "] " << "waiting for the resource status thread to finish ..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  resource_status_thread_.join();
}

void ERManager::resourceLoadLoop()
{
while(ros::ok())
{
  // Make a copy of the loading_processes_ vector so that the original could be released for other threads
  std::vector<temoto_er_manager::LoadExtResource> loading_processes_cpy;

  // Closed scope for the lock
  {
    std::lock_guard<std::mutex> load_processes_lock(loading_mutex_);
    loading_processes_cpy = loading_processes_;
    loading_processes_.clear();
  }

  // execute each process in loading_processes vector
  for (auto& srv : loading_processes_cpy)
  {
    TEMOTO_DEBUG_STREAM("Loading resource: " << srv.request);
    const std::string& package_name = srv.request.package_name;
    const std::string& executable = srv.request.executable;
    const std::string& args = srv.request.args;

    std::string cmd = "";

    if (srv.request.action == action::ROS_EXECUTE)
    {
      if (srv.request.ros_namespace != "")
      {
        cmd += "ROS_NAMESPACE=" + common::getAbsolutePath(srv.request.ros_namespace) + " ";
      }
      std::regex rx(".*\\.launch$");
      cmd += (std::regex_match(executable, rx)) ? "roslaunch " : "rosrun ";
      cmd += package_name + " " + executable + " " + args;
    }
    else if (srv.request.action == action::SYS_EXECUTE)
    {
      cmd += executable + " " + args;
    }

    // Fork the parent process
    TEMOTO_DEBUG("Forking the process.");
    pid_t pid = fork();

    // Child process
    if (pid == 0)
    {
      // Refresh the environment variables
      TEMOTO_DEBUG_STREAM("Sourcing the .../devel/setup.sh ...");
      system(std::string(". " + catkin_workspace_devel_path_ + "setup.sh").c_str());

      // Execute the requested process
      execlp("/bin/bash", "/bin/bash", "-c", cmd.c_str() , (char*)NULL);
      return;
    }

    // Only parent gets here
    TEMOTO_DEBUG("Child %d forked.", pid);

    // Closed scope for the lock
    {
      std::lock_guard<std::mutex> running_processes_lock(running_mutex_);
      running_processes_.insert({ pid, srv });
    }
  }

  // Sleep for 1 second
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}
}

void ERManager::resourceUnloadLoop()
{
while(ros::ok())
{
  std::vector<pid_t> unloading_processes_cpy;

  // Closed scope for the lock
  {
    std::lock_guard<std::mutex> unload_processes_lock(unloading_mutex_);
    unloading_processes_cpy = unloading_processes_;
    unloading_processes_.clear();
  }

  // Closed scope for the lock
  {
    std::lock_guard<std::mutex> running_processes_lock(running_mutex_);
    for (auto pid : unloading_processes_cpy)
    {
      
      // consistency check, ensure that pid is still in running_processes
      auto proc_it = running_processes_.find(pid);
      if (proc_it != running_processes_.end())
      {
        // Kill the process
        TEMOTO_DEBUG("Sending kill(SIGTERM) to %d", pid);

        int ret = kill(pid, SIGTERM);
        TEMOTO_DEBUG("kill(SIGTERM) returned: %d", ret);
        // TODO: Check the returned value

        // Remove the process from the map
        running_processes_.erase(proc_it);
      }
      // Process failed and does not exist any more.
      // Remove it from the failed processes map
      else if(!failed_processes_.erase(pid))
      {
        TEMOTO_DEBUG("Unable to unload reource with pid: %d. Resource is not running nor marked as failed.", pid);
      }
    }
  }

  // Sleep for 1 second
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
} 
}

void ERManager::resourceStatusLoop()
{
while(ros::ok())
{
  // Check the status of all running processes
  // cache all to statuses before actual sending, so we can release the running_mutex.
  std::vector<temoto_core::ResourceStatus> statuses_to_send;

  // Closed scope for the lock guard
  {
    std::lock_guard<std::mutex> running_processes_lock(running_mutex_);

    auto proc_it = running_processes_.begin();
    while (proc_it != running_processes_.end())
    {
      int status;
      int kill_response = waitpid(proc_it->first, &status, WNOHANG);
      // If the child process has stopped running,
      if (kill_response != 0)
      {
        TEMOTO_ERROR("Process %d ('%s' '%s' '%s') has stopped.", proc_it->first,
                    proc_it->second.request.action.c_str(),
                    proc_it->second.request.package_name.c_str(),
                    proc_it->second.request.executable.c_str());

        // TODO: send error information to all related connections
        temoto_core::ResourceStatus srv;
        srv.request.resource_id = proc_it->second.response.trr.resource_id;
        srv.request.status_code = trr::status_codes::FAILED;
        std::stringstream ss;
        ss << "The process with pid '" << proc_it->first << "' has stopped.";
        srv.request.message = ss.str();
        srv.request.error_stack = CREATE_ERROR(error::Code::PROCESS_STOPPED, ss.str());

        // store statuses to send
        statuses_to_send.push_back(srv);
        
        // Remove the process from the map
        // Currently the status is propagated to who ever is using the resource,
        // each of which is responsible to unload the failed resource on its own.
        failed_processes_.insert(*proc_it);
        proc_it = running_processes_.erase(proc_it);
      }
      else
      {
        proc_it++;
      }
    }
  }

  for (auto& srv : statuses_to_send)
  {
    resource_registrar_.sendStatus(srv);

    // TODO: Normally unload command should come from upper chain, howerver, when sending status is unsucessful, we should unload the resource manually?
   // running_processes_.erase(proc_it++);
  }

  // Sleep for 1 second
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
} 
}

void ERManager::loadCb( temoto_er_manager::LoadExtResource::Request& req,
                        temoto_er_manager::LoadExtResource::Response& res)
{
  TEMOTO_DEBUG_STREAM("Received a request: " << req);

  // Validate the action command.
  if (req.action == action::ROS_EXECUTE) //|| action == action::SYS_EXECUTE)
  {
    std::string path = ros::package::getPath(req.package_name);
    if(path=="")
    {
      throw CREATE_ERROR(error::Code::PACKAGE_NOT_FOUND, "ROS Package: '%s' was not found.", req.package_name.c_str());
    }

    // Check if .launch file exists.
    std::regex rx(".*\\.launch$");
    if (std::regex_match(req.executable, rx))
    {
      if (!executableExists(path + "/launch/" + req.executable))
      {
        throw CREATE_ERROR(error::Code::EXECUTABLE_NOT_FOUND,
                           "ROS Package: '%s' does not contain the requsted launch file '%s'.",
                           req.package_name.c_str(), req.executable.c_str());
      }
    }
    // TODO: In whats below, catkin_find is unable to find .py files. That's why its commented out.
//    else
//    {
//      // Check if the requested binary exists
//      std::string catkin_find_cmd = "catkin_find " + req.package_name + " " + req.executable;
//      std::shared_ptr<FILE> pipe(popen(catkin_find_cmd.c_str(), "r"), pclose);
//      if(pipe)
//      {
//        std::array<char, 128> buffer;
//        std::string result;
//        while(!feof(pipe.get()))
//        {
//          if (fgets(buffer.data(), 128, pipe.get()) != nullptr)
//          {
//            result += buffer.data();
//          }
//        }
//
//        //catkin find does not write anything to stdout if the binary does not exist.
//        if (!result.size())
//        {
//          throw CREATE_ERROR(error::Code::EXECUTABLE_NOT_FOUND,
//                             "ROS Package: '%s' does not contain the executable '%s'.",
//                             req.package_name.c_str(), req.executable.c_str());
//        }
//      }
//    }

    // Yey, the executable and ros package exists. Add it to the loading queue.

    TEMOTO_DEBUG("Adding '%s' '%s' '%s' '%s' to the loading queue.", req.action.c_str(),
                 req.package_name.c_str(), req.executable.c_str(), req.args.c_str());

    temoto_er_manager::LoadExtResource srv;
    srv.request = req;
    srv.response = res;

    // Closed scope for the lock
    {
      std::lock_guard<std::mutex> load_processes_lock(loading_mutex_);
      loading_processes_.push_back(srv);
    }
  }
  else
  {
    throw CREATE_ERROR(error::Code::ACTION_UNKNOWN, "Action '%s' is not supported.",
                       req.action.c_str());
  }

  // Fill response
  res.trr.code = trr::status_codes::OK;
  res.trr.message = "Request added to the loading queue.";
}

void ERManager::unloadCb(temoto_er_manager::LoadExtResource::Request& req,
                              temoto_er_manager::LoadExtResource::Response& res)
{
  // Lookup the requested process by its resource id.
  std::lock_guard<std::mutex> running_processes_lock(running_mutex_);
  std::lock_guard<std::mutex> unload_processes_lock(unloading_mutex_);
  TEMOTO_DEBUG("Unloading resource with id '%ld' ...", res.trr.resource_id);

  auto proc_it =
      std::find_if(running_processes_.begin(), running_processes_.end(),
                   [&](const std::pair< pid_t, temoto_er_manager::LoadExtResource>& p) -> bool { return p.second.request == req; });
  auto failed_proc_it =
      std::find_if(failed_processes_.begin(), failed_processes_.end(),
                   [&](const std::pair< pid_t, temoto_er_manager::LoadExtResource>& p) -> bool { return p.second.request == req; });
  if (proc_it != running_processes_.end())
  {
    unloading_processes_.push_back(proc_it->first);
    res.trr.code = 0;
    res.trr.message = "Resource added to unload queue.";
    TEMOTO_DEBUG("Resource with id '%ld' added to unload queue.", res.trr.resource_id);
  }
  else if (failed_proc_it != failed_processes_.end())
  {
    failed_processes_.erase(failed_proc_it);
    res.trr.code = 0;
    res.trr.message = "Resource unloaded.";
    TEMOTO_DEBUG("Unloaded failed resource with id '%ld'.", res.trr.resource_id);
  }
  else
  {
    TEMOTO_ERROR("Unable to unload reource with resource_id: %ld. Resource is not running nor failed.", res.trr.resource_id);
    // Fill response
    res.trr.code = trr::status_codes::FAILED;
    res.trr.message = "Resource is not running nor failed. Unable to unload.";
  }
}

  void ERManager::waitForLock(std::mutex& m)
  {
    while (!m.try_lock())
    {
      TEMOTO_DEBUG("Waiting for lock()");
      ros::Duration(0.1).sleep();  // sleep for few ms
    }
  }
}  // namespace temoto_er_manager
