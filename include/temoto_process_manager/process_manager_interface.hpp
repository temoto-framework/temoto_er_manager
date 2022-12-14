/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Copyright 2019 TeMoto Telerobotics
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
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifndef TEMOTO_PROCESS_MANAGER__PROCESS_MANAGER_INTERFACE_H
#define TEMOTO_PROCESS_MANAGER__PROCESS_MANAGER_INTERFACE_H

#include "rr/ros1_resource_registrar.h"
#include "temoto_process_manager/process_manager_services.hpp"
#include <memory>
#include <ctime>
#include <functional>

namespace temoto_process_manager
{
class ProcessManagerInterface
{
public:

  ProcessManagerInterface(bool initialize_interface = false)
  : unique_suffix_(std::to_string(createID()))
  , initialized_(false)
  {
    if (initialize_interface)
    {
      initialize();
    }
  }

  void initialize()
  {
    if (!initialized_)
    {
      rr_name_ = TEMOTO_LOG_ATTR.getSubsystemNameWithSlash() + GET_CLASS_NAME + "_" + unique_suffix_;
      resource_registrar_ = std::make_unique<temoto_resource_registrar::ResourceRegistrarRos1>(rr_name_);
      resource_registrar_->init();
      initialized_ = true;
    }
    else
    {
      TEMOTO_WARN_STREAM_("The  Manager interface is already initialized");
    }
  }

  unsigned int createID()
  {
    std::srand(std::time(nullptr));
    return std::rand();
  }

  LoadProcess loadRosResource(const std::string& package_name
  , const std::string& ros_program_name
  , const std::string& args = "")
  {
    validateInterface();

    temoto_process_manager::LoadProcess load_process_msg;
    load_process_msg.request.action = temoto_process_manager::action::ROS_EXECUTE;
    load_process_msg.request.package_name = package_name;
    load_process_msg.request.executable = ros_program_name;
    load_process_msg.request.args = args;

    loadResource(load_process_msg);
    return load_process_msg;
  }

  LoadProcess loadSysResource(const std::string& sys_program_name
  , const std::string& arguments = "")
  {
    validateInterface();

    temoto_process_manager::LoadProcess load_process_msg;
    load_process_msg.request.action = temoto_process_manager::action::SYS_EXECUTE;
    load_process_msg.request.executable = sys_program_name;
    load_process_msg.request.args = arguments;

    loadResource(load_process_msg);
    return load_process_msg;
  }

  void loadResource(LoadProcess& load_process_msg)
  {
    validateInterface();

    // Call the server
    try
    {
      resource_registrar_->call<LoadProcess>(temoto_process_manager::srv_name::MANAGER
      , temoto_process_manager::srv_name::SERVER
      , load_process_msg
      , std::bind(&ProcessManagerInterface::statusInfoCb, this, std::placeholders::_1, std::placeholders::_2));

      allocated_processes_.emplace(load_process_msg.response.temoto_metadata.request_id, load_process_msg);
    }
    catch(resource_registrar::TemotoErrorStack e)
    {
      throw FWD_TEMOTO_ERRSTACK(e);
    }
    return;
  }

  void unloadResource(const LoadProcess& load_process_msg)
  {
    try
    {
      validateInterface();
    }
    catch (resource_registrar::TemotoErrorStack e)
    {
      throw FWD_TEMOTO_ERRSTACK(e);
    }

    try
    {
      auto resource_id = allocated_processes_.at(load_process_msg.response.temoto_metadata.request_id).response.temoto_metadata.request_id;

      resource_registrar_->unload(temoto_process_manager::srv_name::MANAGER, resource_id);
      allocated_processes_.erase(load_process_msg.response.temoto_metadata.request_id);
    }
    catch(resource_registrar::TemotoErrorStack e)
    {
      throw FWD_TEMOTO_ERRSTACK(e);
    }
  }

  void statusInfoCb(LoadProcess srv_msg, temoto_resource_registrar::Status status_msg)
  {
    try
    {
      validateInterface();
    }
    catch (resource_registrar::TemotoErrorStack e)
    {
      throw FWD_TEMOTO_ERRSTACK(e);
    }
    TEMOTO_ERROR_STREAM_("status info was received ...");
    
    /*
     * Check if the owner has a status routine defined
     */
    if (user_status_callback_)
    {
      TEMOTO_DEBUG_STREAM_("Invoking user-registered status callback");
      user_status_callback_(srv_msg, status_msg);
      return;
    }
    else
    {
      try
      {
        // Get the local copy of the query. TODO: replace with std::find
        std::string local_srv_msg_id;
        std::cout << __func__ << " incoming: " << srv_msg.response.temoto_metadata.request_id << std::endl;
        for (const auto& local_srv_msg : allocated_processes_)
        {
          std::cout << __func__ << ": " << local_srv_msg.second.response.temoto_metadata.request_id << std::endl;
          if (local_srv_msg.second.response.temoto_metadata.request_id ==
              srv_msg.response.temoto_metadata.request_id)
          {
            local_srv_msg_id = local_srv_msg.first;
            break;
          }
        }

        if (local_srv_msg_id.empty())
        {
          throw TEMOTO_ERRSTACK("Could not find a resource with id: " + srv_msg.response.temoto_metadata.request_id);
        }

        TEMOTO_DEBUG_STREAM_("Unloading the failed resource");
        resource_registrar_->unload(temoto_process_manager::srv_name::MANAGER
        , srv_msg.response.temoto_metadata.request_id);

        LoadProcess new_srv_msg;
        new_srv_msg.request = srv_msg.request;

        TEMOTO_DEBUG_STREAM_("Asking the same resource again");
        resource_registrar_->call<LoadProcess>(temoto_process_manager::srv_name::MANAGER
        , temoto_process_manager::srv_name::SERVER
        , new_srv_msg
        , std::bind(&ProcessManagerInterface::statusInfoCb, this, std::placeholders::_1, std::placeholders::_2));

        allocated_processes_[local_srv_msg_id] = new_srv_msg;
      }
      catch(resource_registrar::TemotoErrorStack e)
      {
        throw FWD_TEMOTO_ERRSTACK(e);
      }
    }
  }

  /**
   * @brief registerUpdateCallback
   */
  void registerUpdateCallback( std::function<void(LoadProcess, temoto_resource_registrar::Status)> user_status_callback)
  {
    user_status_callback_ = user_status_callback;
  }

  ~ProcessManagerInterface()
  {
  }

  const std::string& getName() const
  {
    return rr_name_;
  }

private:
  std::string rr_name_;
  std::string unique_suffix_;
  bool initialized_;
  std::map<std::string, LoadProcess> allocated_processes_;
  std::unique_ptr<temoto_resource_registrar::ResourceRegistrarRos1> resource_registrar_;
  std::function<void(LoadProcess, temoto_resource_registrar::Status)> user_status_callback_ = NULL;

  /**
   * @brief validateInterface
   */
  void validateInterface()
  {
    if (!initialized_)
    {
      TEMOTO_WARN_STREAM_("The  Manager interface is not initialized");
    }
  }
};

} // namespace

#endif
