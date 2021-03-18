#ifndef TEMOTO_ER_MANAGER__TEMOTO_ER_MANAGER_INTERFACE_H
#define TEMOTO_ER_MANAGER__TEMOTO_ER_MANAGER_INTERFACE_H

#include "temoto_core/common/base_subsystem.h"
#include "temoto_core/trr/resource_registrar.h"
#include "rr/ros1_resource_registrar.h"

#include "temoto_er_manager/temoto_er_manager_services.h"
#include <memory>
#include <ctime>
#include <functional>

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *                   EXTERNAL RESOURCE MANAGER INTERFACE
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

namespace temoto_er_manager
{
class ERManagerInterface : public temoto_core::BaseSubsystem
{
public:

  ERManagerInterface(bool initialize_interface = false)
  : unique_suffix_(std::to_string(createID()))
  , has_owner_(false)
  , initialized_(false)
  {
    class_name_ = __func__;
    if (initialize_interface)
    {
      initialize();
    }
  }

  void initialize(const BaseSubsystem& owner)
  {
    if (!initialized_)
    {
      initializeBase(owner);
      log_group_ = "interfaces." + owner.subsystem_name_;
      rr_name_ = owner.class_name_ + "/" + class_name_ + "_" + unique_suffix_;
      has_owner_ = true;
      initialize();
    }
    else
    {
      TEMOTO_WARN_STREAM("The External Resource Manager interface is already initialized");
    }
  }

  void initialize()
  {
    if (!initialized_)
    {
      if (!has_owner_)
      {
        rr_name_ = class_name_ + "_" + unique_suffix_;
      }
      resource_registrar_ = std::make_unique<temoto_resource_registrar::ResourceRegistrarRos1>(rr_name_);
      resource_registrar_->init();
      initialized_ = true;
    }
    else
    {
      TEMOTO_WARN_STREAM("The External Resource Manager interface is already initialized");
    }
  }

   unsigned int createID()
  {
    std::srand(std::time(nullptr));
    return std::rand();
  }

  LoadExtResource loadRosResource(const std::string& package_name
  , const std::string& ros_program_name
  , const std::string& args = "")
  {
    validateInterface();

    temoto_er_manager::LoadExtResource load_resource_msg;
    load_resource_msg.request.action = temoto_er_manager::action::ROS_EXECUTE;
    load_resource_msg.request.package_name = package_name;
    load_resource_msg.request.executable = ros_program_name;
    load_resource_msg.request.args = args;

    loadResource(load_resource_msg);
    return load_resource_msg;
  }

  LoadExtResource loadSysResource(const std::string& sys_program_name
  , const std::string& arguments = "")
  {
    validateInterface();

    temoto_er_manager::LoadExtResource load_resource_msg;
    load_resource_msg.request.action = temoto_er_manager::action::SYS_EXECUTE;
    load_resource_msg.request.executable = sys_program_name;
    load_resource_msg.request.args = arguments;

    loadResource(load_resource_msg);
    return load_resource_msg;
  }

  void loadResource(LoadExtResource& load_resource_msg)
  {
    validateInterface();

    // Call the server
    try
    {
      resource_registrar_->call<LoadExtResource>(temoto_er_manager::srv_name::MANAGER
      , temoto_er_manager::srv_name::SERVER
      , load_resource_msg
      , NULL
      , std::bind(&ERManagerInterface::statusInfoCb, this, std::placeholders::_1, std::placeholders::_2));
      
    }
    catch(temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }

    allocated_external_resources_.emplace(load_resource_msg.response.trr.resource_id, load_resource_msg);
    return;
  }

  void unloadResource(const LoadExtResource& load_resource_msg)
  {
    try
    {
      validateInterface();
    }
    catch (temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }

    try
    {
      resource_registrar_->unload(temoto_er_manager::srv_name::MANAGER
      , load_resource_msg.response.TemotoMetadata.requestId);
      allocated_external_resources_.erase(load_resource_msg.response.trr.resource_id);
    }
    catch(temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }
  }

  void statusInfoCb(LoadExtResource srv_msg, temoto_resource_registrar::Status status_msg)
  {
    try
    {
      validateInterface();
    }
    catch (temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }
    TEMOTO_ERROR_STREAM("status info was received ...");
    
    /*
     * Check if the owner has a status routine defined
     */
    if (user_status_callback_)
    {
      TEMOTO_DEBUG_STREAM("Invoking user-registered status callback");
      user_status_callback_(srv_msg, status_msg);
      return;
    }
    else
    {
      try
      {
        TEMOTO_DEBUG_STREAM("Unloading the failed resource");
        resource_registrar_->unload(temoto_er_manager::srv_name::MANAGER
        , srv_msg.response.TemotoMetadata.requestId);

        allocated_external_resources_.erase(srv_msg.response.trr.resource_id);
        LoadExtResource new_srv_msg;
        new_srv_msg.request = srv_msg.request;

        TEMOTO_DEBUG_STREAM("Asking the same resource again");
        resource_registrar_->call<LoadExtResource>(temoto_er_manager::srv_name::MANAGER
        , temoto_er_manager::srv_name::SERVER
        , new_srv_msg);

        allocated_external_resources_.emplace(new_srv_msg.response.trr.resource_id, new_srv_msg);
      }
      catch(temoto_core::error::ErrorStack& error_stack)
      {
        throw FORWARD_ERROR(error_stack);
      }
    }
  }

  /**
   * @brief registerUpdateCallback
   */
  void registerUpdateCallback( std::function<void(LoadExtResource, temoto_resource_registrar::Status)> user_status_callback)
  {
    user_status_callback_ = user_status_callback;
  }

  ~ERManagerInterface()
  {
  }

  const std::string& getName() const
  {
    return rr_name_;
  }

private:
  std::string rr_name_;
  std::string unique_suffix_;
  bool has_owner_;
  bool initialized_;
  std::map<unsigned int, temoto_er_manager::LoadExtResource> allocated_external_resources_;
  std::unique_ptr<temoto_resource_registrar::ResourceRegistrarRos1> resource_registrar_;
  std::function<void(LoadExtResource, temoto_resource_registrar::Status)> user_status_callback_ = NULL;

  /**
   * @brief validateInterface
   */
  void validateInterface()
  {
    // TODO: Deprecated, to be removed.
  }
};

} // namespace

#endif
