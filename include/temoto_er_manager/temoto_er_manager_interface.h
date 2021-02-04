#ifndef TEMOTO_ER_MANAGER__TEMOTO_ER_MANAGER_INTERFACE_H
#define TEMOTO_ER_MANAGER__TEMOTO_ER_MANAGER_INTERFACE_H

#include "temoto_core/common/base_subsystem.h"
#include "temoto_core/trr/resource_registrar.h"

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

  ERManagerInterface()
  : unique_suffix_(std::to_string(createID()))
  , has_owner_(false)
  {
    class_name_ = __func__;
  }

  void initialize(const BaseSubsystem& owner)
  {
    initializeBase(owner);
    log_group_ = "interfaces." + owner.subsystem_name_;
    rr_name_ = owner.class_name_ + "/" + class_name_ + "_" + unique_suffix_;
    has_owner_ = true;
    initialize();
  }

  void initialize()
  {
    if (!has_owner_)
    {
      rr_name_ = class_name_ + "_" + unique_suffix_;
    }
    resource_registrar_ = std::unique_ptr<temoto_core::trr::ResourceRegistrar<ERManagerInterface>>(
                        new temoto_core::trr::ResourceRegistrar<ERManagerInterface>(rr_name_, this));
    resource_registrar_->registerStatusCb(&ERManagerInterface::statusInfoCb);
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
      resource_registrar_->template call<LoadExtResource>(temoto_er_manager::srv_name::MANAGER
      , temoto_er_manager::srv_name::SERVER
      , load_resource_msg
      , temoto_core::trr::FailureBehavior::NONE);
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
      resource_registrar_->unloadClientResource(load_resource_msg.response.trr.resource_id);
      allocated_external_resources_.erase(load_resource_msg.response.trr.resource_id);
    }
    catch(temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }
  }

  void statusInfoCb(temoto_core::ResourceStatus& srv)
  {
    try
    {
      validateInterface();
    }
    catch (temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }

    TEMOTO_DEBUG_STREAM("status info was received");
    TEMOTO_DEBUG_STREAM(srv.request);

    /*
     * Find the associated query
     */ 
    LoadExtResource associated_query;
    try
    {
      associated_query = allocated_external_resources_.at(srv.request.resource_id);
    }
    catch(const std::exception& e)
    {
      throw CREATE_ERROR(temoto_core::error::Code::RESOURCE_NOT_FOUND, "Query not found");
    }
    
    /*
     * Check if the owner has a status routine defined
     */
    if (update_callback_)
    {
      TEMOTO_DEBUG_STREAM("Invoking user-registered status callback");
      update_callback_(associated_query);
      return;
    }
    else
    {
      try
      {
        TEMOTO_DEBUG_STREAM("Unloading the failed resource");
        resource_registrar_->unloadClientResource(associated_query.response.trr.resource_id);
        allocated_external_resources_.erase(associated_query.response.trr.resource_id);
        associated_query.response = LoadExtResource::Response();

        TEMOTO_DEBUG_STREAM("Asking the same resource again");
        resource_registrar_->template call<LoadExtResource>(temoto_er_manager::srv_name::MANAGER
        , temoto_er_manager::srv_name::SERVER
        , associated_query
        , temoto_core::trr::FailureBehavior::NONE);

        allocated_external_resources_.emplace(associated_query.response.trr.resource_id, associated_query);
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
  void registerUpdateCallback( std::function<void(LoadExtResource)> update_callback)
  {
    update_callback_ = update_callback;
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
  std::map<unsigned int, temoto_er_manager::LoadExtResource> allocated_external_resources_;
  std::unique_ptr<temoto_core::trr::ResourceRegistrar<ERManagerInterface>> resource_registrar_;
  std::function<void(LoadExtResource)> update_callback_ = NULL;

  /**
   * @brief validateInterface
   */
  void validateInterface()
  {
    if(!resource_registrar_)
    {
      throw CREATE_ERROR(temoto_core::error::Code::UNINITIALIZED, "Interface is not initalized.");
    }
  }
};

} // namespace

#endif
