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

#include <csignal>
#include <sys/wait.h>
#include <algorithm>
#include <spawn.h>
#include <regex>
#include <functional>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include "temoto_process_manager/process_manager.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

// std::shared_ptr<temoto_resource_registrar::ResourceRegistrarRos2> rr;


namespace temoto_process_manager
{

ProcessManager::ProcessManager(bool restore_from_catalog, std::shared_ptr<rclcpp::Node> node)
:  resource_registrar_(std::make_shared<temoto_resource_registrar::ResourceRegistrarRos2>(srv_name::MANAGER)), node_(node)
{
  /*
   * Configure the RR catalog backup routine
   */
  std::string home_path = std::getenv("HOME");
  std::string rr_catalog_backup_path = home_path + "/.temoto/" + srv_name::MANAGER + ".rrcat";
  rr_catalog_config_.setName(srv_name::MANAGER);
  rr_catalog_config_.setLocation(rr_catalog_backup_path);
  rr_catalog_config_.setSaveOnModify(true);
  rr_catalog_config_.setEraseOnDestruct(true);
  resource_registrar_->updateConfiguration(rr_catalog_config_);

  /*
   * Add the LoadProcess server to the resource registrar
   */

  std::function<void(const std::shared_ptr<temoto_process_manager::srv::LoadProcess::Request>,
                   const std::shared_ptr<temoto_process_manager::srv::LoadProcess::Response>)> load_cb =
    [this](const std::shared_ptr<temoto_process_manager::srv::LoadProcess::Request> req,
           const std::shared_ptr<temoto_process_manager::srv::LoadProcess::Response> res) {
        this->loadCb(*req, *res); 
    };

  std::function<void(const std::shared_ptr<temoto_process_manager::srv::LoadProcess::Request>,
                   const std::shared_ptr<temoto_process_manager::srv::LoadProcess::Response>)> unload_cb =
    [this](const std::shared_ptr<temoto_process_manager::srv::LoadProcess::Request> req,
           const std::shared_ptr<temoto_process_manager::srv::LoadProcess::Response> res) {
        this->unloadCb(*req, *res); 
    };

  std::function<void(const std::shared_ptr<temoto_process_manager::srv::LoadProcess::Request>,
                   const std::shared_ptr<temoto_process_manager::srv::LoadProcess::Response>,
                   const temoto_resource_registrar::Status&)> status_cb =
    [this](const std::shared_ptr<temoto_process_manager::srv::LoadProcess::Request> req,
           const std::shared_ptr<temoto_process_manager::srv::LoadProcess::Response> res,
           const temoto_resource_registrar::Status& status) {
        this->statusCb(*req, *res, status); 
    };


  auto server = std::make_unique<Ros2Server<temoto_process_manager::srv::LoadProcess>>(
    srv_name::SERVER, node_, load_cb, unload_cb, status_cb);

  resource_registrar_->init();
  resource_registrar_->registerServer(std::move(server));
 
  /*
   * Check if this node should be recovered from a previous system failure
   */
  if (restore_from_catalog && boost::filesystem::exists(rr_catalog_backup_path))
  {
    resource_registrar_->loadCatalog();
    for (const auto& query : resource_registrar_->getServerQueries<temoto_process_manager::srv::LoadProcess>(srv_name::SERVER))
    {
      LoadProcess_srv query_srv;
      query_srv.request = *query.first;
      query_srv.response = *query.second;      
      
      running_processes_.insert({query.second->pid, query_srv});

      RCLCPP_INFO(rclcpp::get_logger("query.first.action "), query.first->action.c_str());
      // RCLCPP_INFO(rclcpp::get_logger("query.second.pid "), query.second->pid);

    }
  }

  /*
   * Find the colcon workspace
   */
  const std::string current_node_path = ament_index_cpp::get_package_share_directory("temoto_process_manager");
  
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

    if(current_node_path_tokens.back() != "install")
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

  // Assemble the install path string
  for (const auto& token : current_node_path_tokens)
  {
    colcon_workspace_install_path_ += token + "/";
  }
  colcon_workspace_install_path_ += "install/";

  // Start the load, unload and status monitoring threads
  resource_loading_thread_ = std::thread(&ProcessManager::resourceLoadLoop, this);;  
  resource_unloading_thread_ = std::thread(&ProcessManager::resourceUnloadLoop, this);;
  resource_status_thread_ = std::thread(&ProcessManager::resourceStatusLoop, this);;

  TEMOTO_INFO_(" Manager is ready.");
  
}

ProcessManager::~ProcessManager()
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

  /*
   * Stop all the processes
   */
  for (const auto& running_process : running_processes_)
  {
    int ret = kill(running_process.first, SIGINT);

    // If SIGINT did not do the trick, then try SIGTERM
    if (ret != 0)
    {
      TEMOTO_DEBUG_STREAM_("Process with PID=" << running_process.first << " did not stop successfully. " 
      "Stopping it via SIGTERM.");
      ret = kill(running_process.first, SIGTERM);
    }
  }
}

void ProcessManager::resourceLoadLoop()
{
  
  while(rclcpp::ok())
  {
    // Make a copy of the loading_processes_ vector so that the original could be released for other threads
    std::vector<LoadProcess_srv> loading_processes_cpy;

    // Closed scope for the lock
    {
      std::lock_guard<std::mutex> load_processes_lock(loading_mutex_);
      loading_processes_cpy = loading_processes_;
      loading_processes_.clear();
    }

    // execute each process in loading_processes vector
    for (auto& srv : loading_processes_cpy)
    {
      TEMOTO_INFO_STREAM_("Loading resource: \n" << srv.request);

      const std::string& package_name = srv.request.package_name;
      const std::string& executable = srv.request.executable;
      const std::string& args = srv.request.args;

      std::string cmd = "";

      if (srv.request.action == action::ROS_EXECUTE)
      {
        if (srv.request.ros_namespace.empty())
        {
          cmd += "ROS_NAMESPACE=" + TEMOTO_LOG_ATTR.getNs() + " ";
        }
        else
        {
          cmd += "ROS_NAMESPACE=" + srv.request.ros_namespace + " ";
        }
        
        std::regex rx(".*\\.launch$");
        cmd += (std::regex_match(executable, rx)) ? "ros2 launch " : "ros2 run ";
        cmd += package_name + " " + executable + " " + args;
      }
      else if (srv.request.action == action::SYS_EXECUTE)
      {
        cmd += executable + " " + args;
      }

      // Fork the parent process
      TEMOTO_DEBUG_("Forking the process.");
      pid_t pid = fork();

      // Child process
      if (pid == 0)
      {
        // Refresh the environment variables
        TEMOTO_DEBUG_STREAM_("Sourcing the .../install/setup.sh ...");
        system(std::string(". " + colcon_workspace_install_path_ + "setup.sh").c_str());

        // Execute the requested process
        execlp("/bin/bash", "/bin/bash", "-c", cmd.c_str() , (char*)NULL);
        return;
      }
      
      // Only parent gets here
      TEMOTO_DEBUG_("Child %d forked.", pid);

      // Closed scope for the lock
      {
        std::lock_guard<std::mutex> running_processes_lock(running_mutex_);
        srv.response.pid = pid;
        running_processes_.insert({ pid, srv });

        auto request_ptr = std::make_shared<temoto_process_manager::srv::LoadProcess_Request_<std::allocator<void>>>();
        auto response_ptr = std::make_shared<temoto_process_manager::srv::LoadProcess_Response_<std::allocator<void>>>();

        *request_ptr = srv.request;
        *response_ptr = srv.response;

        std::pair<
            std::shared_ptr<temoto_process_manager::srv::LoadProcess_Request_<std::allocator<void>>>,
            std::shared_ptr<temoto_process_manager::srv::LoadProcess_Response_<std::allocator<void>>>
        > srv_pair = std::make_pair(request_ptr, response_ptr);

        // Note: Had to manually create a std pair here, so I can use the updateQueryResponse template]
        resource_registrar_->updateQueryResponse<temoto_process_manager::srv::LoadProcess>(srv_name::SERVER, srv_pair);
        resource_registrar_->saveCatalog();
      }
    }

    // Sleep for 1 second
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}

void ProcessManager::resourceUnloadLoop()
{
while(rclcpp::ok())
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
        // Check parent pid
        std::string command = "pgrep -P ";
        command += std::to_string(pid);
        std::cout << "\033[1;35m [PM.cpp] command: \033[0m" << command << " = " << exec(command.c_str()) << std::endl;
        int parent_pid = std::atoi(exec(command.c_str()).c_str());

        pid_t target_pid = (parent_pid != 0) ? parent_pid : pid;

        // Kill the process
        std::cout << "\033[1;33m [PM.cpp] Sending kill(SIGINT) to :  \033[0m" << target_pid << std::endl;
        TEMOTO_DEBUG_("Sending kill(SIGINT) to %d ...", target_pid);
        int ret = kill(target_pid, SIGINT);

        // If SIGINT did not do the trick, then try SIGTERM
        if (ret != 0)
        {
          std::cout << "\033[1;33m [PM.cpp] Process did not stop successfully  \033[0m\n" << parent_pid << std::endl;
          TEMOTO_DEBUG_STREAM_("Process with PID=" << parent_pid << " did not stop successfully. "
          "Stopping it via SIGTERM.");
          ret = kill(target_pid, SIGTERM);
        }
        else
        {
          std::cout << "\033[1;33m [PM.cpp] Process with PID= :  \033[0m" << pid << " was stopped successfully" << std::endl;
          TEMOTO_DEBUG_STREAM_("Process with PID=" << pid << " was stopped successfully.");
        }
        
        // Remove the process from the map
        running_processes_.erase(proc_it);
      }
      // Process failed and does not exist any more.
      // Remove it from the failed processes map
      else if(!failed_processes_.erase(pid))
      {
        TEMOTO_WARN_("Unable to unload reource with pid: %d. Resource is not running nor marked as failed.", pid);
      }
    }
  }

  // Sleep for 1 second
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
} 
}

void ProcessManager::resourceStatusLoop()
{  
  while(rclcpp::ok())
  {
    // Check the status of all running processes
    // cache all to statuses before actual sending, so we can release the running_mutex.
    std::vector<temoto_resource_registrar::Status> statuses_to_send;

    // Closed scope for the lock guard
    {
      std::lock_guard<std::mutex> running_processes_lock(running_mutex_);

      auto proc_it = running_processes_.begin();
      while (proc_it != running_processes_.end())
      {
        int status;
        int waitpid_response =  waitpid(proc_it->first, &status, WNOHANG);
        int kill_response = kill(proc_it->first, 0);
        
        /*
        * Check both 'waitpid' and 'kill' syscall responses, because if the er_manager
        * was recovered after a failure, it has lost the ownership of all child processes
        * it has spawned. Thus 'waitpid' will always return a nonzero result, even if the
        * ex-child process is actually alive. Therefore 'kill' is additionally used to
        * determine whether the ex-child process actually exists, or not. Just checking
        * the 'kill' wont work either, as it cannot tell whether the child is a zombie process or
        * not. It all comes down to "who reaps the child". If er_manager hasn't crashed, then
        * OS expects er_manager to reap child processes (via wait or waitpid). If the er_manager
        * has recovered from a crash, all ex-children are adopted by OS (init) and thus the
        * OS will manage the reaping (hence just checking the 'kill' will suffice then).
        */
        if (waitpid_response != 0 && kill_response != 0)
        {
          TEMOTO_WARN_("Process %d ('%s' '%s' '%s') has stopped. waitpid = %d"
          , proc_it->first
          , proc_it->second.request.action.c_str()
          , proc_it->second.request.package_name.c_str()
          , proc_it->second.request.executable.c_str()
          , kill_response);

          // TODO: send error information to all related connections
          std::stringstream ss;
          ss << "The process with pid '" << proc_it->first << "' has stopped.";

          temoto_resource_registrar::Status status_msg;
          status_msg.id_ = proc_it->second.response.temoto_metadata.request_id;
          status_msg.state_ = temoto_resource_registrar::Status::State::FATAL;
          status_msg.message_ = ss.str();
          
          //srv.request.error_stack = TEMOTO_ERRSTACK(error::Code::PROCESS_STOPPED, ss.str());

          // store statuses to send
          statuses_to_send.push_back(status_msg);
          
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

    for (auto& status_msg : statuses_to_send)
    {
      try
      {
        resource_registrar_->sendStatus(status_msg.id_, status_msg);
      }
      catch (resource_registrar::TemotoErrorStack& error_stack)
      {
        TEMOTO_ERROR_STREAM_("Unable to send status message:" << status_msg.id_ << "; " << status_msg.message_);
      }
      catch (...)
      {
        TEMOTO_ERROR_STREAM_("Caught an unhandled exception");
      }
    }

    // Sleep for 1 second
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  } 
}

void ProcessManager::loadCb(temoto_process_manager::srv::LoadProcess::Request& req, temoto_process_manager::srv::LoadProcess::Response& res)
{ 
  START_SPAN
  TEMOTO_DEBUG_STREAM_("Received a request: " << req);
  
  // Validate the action command.
  if (req.action == action::ROS_EXECUTE)
  {
    std::string path = ament_index_cpp::get_package_share_directory(req.package_name);
    if(path.empty())
    {
      throw TEMOTO_ERRSTACK("ROS Package '" + req.package_name + "' was not found.");
    }

    // If the executable is a ROS launch file, then check if it exists
    // std::regex rx(".*\\.launch$");

    std::regex rx(".*\\.launch(?:\\.py)?$");

    if (std::regex_match(req.executable, rx))
    {
      if (!executableExists(path + "/launch/" + req.executable))
      {
        throw TEMOTO_ERRSTACK("ROS Package '" + req.package_name + "' does not contain the requsted launch file '" + req.executable + "'");
      }
    }
  }
  else if (req.action == action::SYS_EXECUTE)
  {
    // TODO check if the requested system program exists
  }
  else
  {
    throw TEMOTO_ERRSTACK("Action '" + req.action + "' is not supported");
  }

  TEMOTO_DEBUG_("Adding '%s' '%s' '%s' '%s' to the loading queue.", req.action.c_str(),
                 req.package_name.c_str(), req.executable.c_str(), req.args.c_str());

  LoadProcess_srv srv;
  srv.request = req;
  srv.response = res;  

  // Closed scope for the lock
  {
    std::lock_guard<std::mutex> load_processes_lock(loading_mutex_);
    loading_processes_.push_back(srv);
  }
}

void ProcessManager::unloadCb(temoto_process_manager::srv::LoadProcess::Request& req, temoto_process_manager::srv::LoadProcess::Response& res)
{
  // Lookup the requested process by its resource id.
  std::lock_guard<std::mutex> running_processes_lock(running_mutex_);
  std::lock_guard<std::mutex> unload_processes_lock(unloading_mutex_);

  std::cout << std::endl;
  TEMOTO_INFO_STREAM_("Unloading resource: \n" << req);

  for (const auto& pair : running_processes_) {
        std::cout << "Key: " << pair.first << ", Value: " << pair.second.response.pid << pair.second.response.temoto_metadata.request_id << std::endl;
    }

  auto proc_it =
      std::find_if(running_processes_.begin(), running_processes_.end(),
                   [&](const std::pair< pid_t, LoadProcess_srv>& p) -> bool { return p.second.response.temoto_metadata.request_id == res.temoto_metadata.request_id; });
                  //  [&](const std::pair< pid_t, LoadProcess_srv>& p) -> bool { return p.second.response.pid == res.pid; });
                  //  [&](const std::pair< pid_t, temoto_process_manager::srv::LoadProcess>& p) -> bool { return p.second.response.pid == res.pid; });
  auto failed_proc_it =
      std::find_if(failed_processes_.begin(), failed_processes_.end(),
                   [&](const std::pair< pid_t, LoadProcess_srv>& p) -> bool { return p.second.response.temoto_metadata.request_id == res.temoto_metadata.request_id; });
                  //  [&](const std::pair< pid_t, LoadProcess_srv>& p) -> bool { return p.second.response.pid == res.pid; });
                  //  [&](const std::pair< pid_t, temoto_process_manager::srv::LoadProcess>& p) -> bool { return p.second.response.pid == res.pid; });
  if (proc_it != running_processes_.end())
  {
    unloading_processes_.push_back(proc_it->first);
  }
  else if (failed_proc_it != failed_processes_.end())
  {
    failed_processes_.erase(failed_proc_it);
  }
  else
  {
    throw TEMOTO_ERRSTACK("Unable to unload reource with id: '" + res.temoto_metadata.request_id + "'. Resource is not running nor failed.");
  }
}

void ProcessManager::statusCb(temoto_process_manager::srv::LoadProcess::Request& req
  , temoto_process_manager::srv::LoadProcess::Response& res
  , const temoto_resource_registrar::Status &status)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "status pm _ rr");
}

void ProcessManager::waitForLock(std::mutex& m)
{
  while (!m.try_lock())
  {
    TEMOTO_DEBUG_("Waiting for lock()");
    rclcpp::sleep_for(std::chrono::milliseconds(100)); // sleep for few ms
  }
}

std::string ProcessManager::exec(const char* cmd) {
  std::array<char, 128> buffer;
  std::string result;
  std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
  if (!pipe) {
      throw std::runtime_error("popen() failed!");
  }
  while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe.get()) != nullptr) {
      result += buffer.data();
  }
  return result;
}

}  // namespace temoto_process_manager
