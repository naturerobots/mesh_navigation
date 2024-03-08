/*
 *  Copyright 2020, Sebastian Pütz
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *
 */

#include "mbf_mesh_nav/mesh_navigation_server.h"

#include <algorithm>
#include <functional>

#include <geometry_msgs/msg/pose_array.hpp>
#include <mesh_map/mesh_map.h>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/logging.hpp>

namespace {
//! Helper function, intended for formatting available plugin types.
//! Returns a string like this: [el1, el2, el3, ..., eln]
std::string stringVectorToString(const std::vector<std::string>& vec) 
{
  std::stringstream s;
  s << "[";
  bool is_first = true;
  for (const auto& item : vec)
  {
    s << (is_first ? "" : ", ") << item;
    is_first = false;
  }
  s << "]";
  return s.str();
};

/*!
 * Helper function to reduce code duplication when loading plugins.
 * CorePluginTypes can be either planners, controllers or recovery behaviors.
 * @tparam AbstractCorePluginType AbstractCore type of the plugin to load. Needs to be the base class of SimpleCorePluginType and MeshCorePluginType. E.g. mbf_abstract_core::AbstractPlanner.
 * @tparam SimpleCorePluginType SimpleCore type of the plugin to load. E.g. mbf_simple_core::SimplePlanner
 * @tparam MeshCorePluginType MeshCore type of the plugin to load. E.g. mbf_simple_core::MeshPlanner
 * @param plugin_type typename of the plugin that shall be loaded, usually configured by user via ros params.
 * @param which_plugin_kind should be "planner", "controller", or "recovery behavior", used only for nicer logging messages.
 */
template<typename AbstractCorePluginType, typename MeshCorePluginType, typename SimpleCorePluginType>
typename AbstractCorePluginType::Ptr loadPlugin(const std::string& plugin_type, pluginlib::ClassLoader<MeshCorePluginType>& mesh_plugin_loader, pluginlib::ClassLoader<SimpleCorePluginType>& simple_plugin_loader, const std::string& which_plugin_kind, rclcpp::Logger logger)
{
  // support both simple core and mesh core plugins. We cannot see which type a plugin is from planner_type, so we need to rely on try catch.
  const auto available_mesh_plugins= mesh_plugin_loader.getDeclaredClasses();
  const auto available_simple_plugins = simple_plugin_loader.getDeclaredClasses();

  typename AbstractCorePluginType::Ptr plugin_ptr;
  std::string plugin_name;
  if (std::find(available_mesh_plugins.begin(), available_mesh_plugins.end(), plugin_type) != available_mesh_plugins.end())
  {
    // plugin_type is available as mesh plugin
    try
    {
      plugin_ptr = std::dynamic_pointer_cast<AbstractCorePluginType>(mesh_plugin_loader.createSharedInstance(plugin_type));
      plugin_name = mesh_plugin_loader.getName(plugin_type);
    }
    catch (const pluginlib::PluginlibException& ex_mbf_core)
    {
      RCLCPP_ERROR_STREAM(logger, "Error while loading " << plugin_type << " as mesh " << which_plugin_kind << ": " << ex_mbf_core.what());
    }
  }
  else if (std::find(available_simple_plugins.begin(), available_simple_plugins.end(), plugin_type) != available_simple_plugins.end())
  {
    // plugin_type is available as simple plugin
    try 
    {
      plugin_ptr = std::dynamic_pointer_cast<AbstractCorePluginType>(simple_plugin_loader.createSharedInstance(plugin_type));
      plugin_name = simple_plugin_loader.getName(plugin_type);
    }
    catch (const pluginlib::PluginlibException& ex_mbf_core)
    {
      RCLCPP_ERROR_STREAM(logger, "Error while loading " << plugin_type << " as simple " << which_plugin_kind << ": " << ex_mbf_core.what());
    }
  }
  else
  {
    // plugin was not found
    RCLCPP_ERROR_STREAM(logger, "Failed to find the " << plugin_type << " " << which_plugin_kind << ". "
                                << "Are you sure it's properly registered and that the containing library is built? "
                                << "Registered mesh " << which_plugin_kind << " are: " << stringVectorToString(available_mesh_plugins)
                                << ". Registered simple " << which_plugin_kind << " are: " << stringVectorToString(available_simple_plugins));
  }

  if (plugin_ptr) 
  {
    // success
    RCLCPP_DEBUG_STREAM(logger, "mbf_mesh_core-based " << which_plugin_kind << " plugin " << plugin_name << " loaded.");
  }
  return plugin_ptr; // return nullptr in case something went wrong
}

} // namespace

namespace mbf_mesh_nav
{
using namespace std::placeholders;

MeshNavigationServer::MeshNavigationServer(const TFPtr& tf_listener_ptr, const rclcpp::Node::SharedPtr& node)
  : AbstractNavigationServer(tf_listener_ptr, node)
  , recovery_plugin_loader_("mbf_mesh_core", "mbf_mesh_core::MeshRecovery")
  , controller_plugin_loader_("mbf_mesh_core", "mbf_mesh_core::MeshController")
  , planner_plugin_loader_("mbf_mesh_core", "mbf_mesh_core::MeshPlanner")
  , simple_recovery_plugin_loader_("mbf_simple_core", "mbf_simple_core::SimpleRecovery")
  , simple_controller_plugin_loader_("mbf_simple_core", "mbf_simple_core::SimpleController")
  , simple_planner_plugin_loader_("mbf_simple_core", "mbf_simple_core::SimplePlanner")
  , mesh_ptr_(new mesh_map::MeshMap(*tf_listener_ptr_, node))
{
  // advertise services and current goal topic
  check_pose_cost_srv_ =
      node_->create_service<mbf_msgs::srv::CheckPose>("check_pose_cost", std::bind(&MeshNavigationServer::callServiceCheckPoseCost, this, _1, _2, _3));
  check_path_cost_srv_ =
      node_->create_service<mbf_msgs::srv::CheckPath>("check_path_cost", std::bind(&MeshNavigationServer::callServiceCheckPathCost, this, _1, _2, _3));
  clear_mesh_srv_ = node_->create_service<std_srvs::srv::Empty>("clear_mesh", std::bind(&MeshNavigationServer::callServiceClearMesh, this, _1, _2, _3));

  RCLCPP_INFO_STREAM(node_->get_logger(), "Reading map file...");
  mesh_ptr_->readMap();

  // initialize all plugins
  initializeServerComponents();
}

mbf_abstract_nav::AbstractPlannerExecution::Ptr MeshNavigationServer::newPlannerExecution(
    const std::string &plugin_name, const mbf_abstract_core::AbstractPlanner::Ptr plugin_ptr)
{
  return std::make_shared<mbf_mesh_nav::MeshPlannerExecution>(
      plugin_name, std::static_pointer_cast<mbf_mesh_core::MeshPlanner>(plugin_ptr), robot_info_, mesh_ptr_, node_);
}

mbf_abstract_nav::AbstractControllerExecution::Ptr MeshNavigationServer::newControllerExecution(
    const std::string &plugin_name, const mbf_abstract_core::AbstractController::Ptr plugin_ptr)
{
  return std::make_shared<mbf_mesh_nav::MeshControllerExecution>(
      plugin_name, std::static_pointer_cast<mbf_mesh_core::MeshController>(plugin_ptr), robot_info_,
      vel_pub_, goal_pub_,
      mesh_ptr_, node_);
}

mbf_abstract_nav::AbstractRecoveryExecution::Ptr MeshNavigationServer::newRecoveryExecution(
    const std::string &plugin_name, const mbf_abstract_core::AbstractRecovery::Ptr plugin_ptr)
{
  return std::make_shared<mbf_mesh_nav::MeshRecoveryExecution>(
      plugin_name, std::static_pointer_cast<mbf_mesh_core::MeshRecovery>(plugin_ptr), robot_info_,
      mesh_ptr_, node_);
}

mbf_abstract_core::AbstractPlanner::Ptr MeshNavigationServer::loadPlannerPlugin(const std::string& planner_type)
{
  return loadPlugin<mbf_abstract_core::AbstractPlanner>(planner_type, planner_plugin_loader_, simple_planner_plugin_loader_, "planner", node_->get_logger());
}

bool MeshNavigationServer::initializePlannerPlugin(const std::string& name,
                                                   const mbf_abstract_core::AbstractPlanner::Ptr& planner_ptr)
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Initialize planner \"" << name << "\".");

  mbf_mesh_core::MeshPlanner::Ptr mesh_planner_ptr =
      std::dynamic_pointer_cast<mbf_mesh_core::MeshPlanner>(planner_ptr);
  if (mesh_planner_ptr)
  {
    if (!mesh_ptr_)
    {
      RCLCPP_FATAL_STREAM(node_->get_logger(), "The mesh pointer has not been initialized!");
      return false;
    }
    return mesh_planner_ptr->initialize(name, mesh_ptr_, node_);
  }

  mbf_simple_core::SimplePlanner::Ptr simple_planner_ptr =
    std::dynamic_pointer_cast<mbf_simple_core::SimplePlanner>(planner_ptr);
  if (simple_planner_ptr) 
  {
    simple_planner_ptr->initialize(name, node_);
    return true;
  }

  RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to initialize plugin " << name << ". Looks like it is neither a mesh planner nor a simple planner.");
  return false;
}

mbf_abstract_core::AbstractController::Ptr
MeshNavigationServer::loadControllerPlugin(const std::string& controller_type)
{
  return loadPlugin<mbf_abstract_core::AbstractController>(controller_type, controller_plugin_loader_, simple_controller_plugin_loader_, "controller", node_->get_logger());
}

bool MeshNavigationServer::initializeControllerPlugin(const std::string& name,
                                                      const mbf_abstract_core::AbstractController::Ptr& controller_ptr)
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Initialize controller \"" << name << "\".");

  if (!tf_listener_ptr_)
  {
    RCLCPP_FATAL_STREAM(node_->get_logger(), "The tf listener pointer has not been initialized!");
    return false;
  }

  mbf_mesh_core::MeshController::Ptr mesh_controller_ptr =
      std::dynamic_pointer_cast<mbf_mesh_core::MeshController>(controller_ptr);
  if (mesh_controller_ptr) 
  {
    if (!mesh_ptr_)
    {
      RCLCPP_FATAL_STREAM(node_->get_logger(), "The mesh pointer has not been initialized!");
      return false;
    }

    mesh_controller_ptr->initialize(name, tf_listener_ptr_, mesh_ptr_, node_);
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "Controller plugin \"" << name << "\" initialized.");
    return true;
  }

  mbf_simple_core::SimpleController::Ptr simple_controller_ptr =
    std::dynamic_pointer_cast<mbf_simple_core::SimpleController>(controller_ptr);
  if (simple_controller_ptr) 
  {
    simple_controller_ptr->initialize(name, tf_listener_ptr_, node_);
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "Controller plugin \"" << name << "\" initialized.");
    return true;
  }

  RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to initialize plugin " << name << ". Looks like it is neither a mesh controller nor a simple controller.");
  return false;
}

mbf_abstract_core::AbstractRecovery::Ptr MeshNavigationServer::loadRecoveryPlugin(const std::string& recovery_type)
{
  return loadPlugin<mbf_abstract_core::AbstractRecovery>(recovery_type, recovery_plugin_loader_, simple_recovery_plugin_loader_, "recovery behavior", node_->get_logger());
}

bool MeshNavigationServer::initializeRecoveryPlugin(const std::string& name,
                                                    const mbf_abstract_core::AbstractRecovery::Ptr& behavior_ptr)
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Initialize recovery behavior \"" << name << "\".");

  if (!tf_listener_ptr_)
  {
    RCLCPP_FATAL_STREAM(node_->get_logger(), "The tf listener pointer has not been initialized!");
    return false;
  }

  mbf_mesh_core::MeshRecovery::Ptr mesh_behavior_ptr =
      std::dynamic_pointer_cast<mbf_mesh_core::MeshRecovery>(behavior_ptr);
  if (mesh_behavior_ptr) 
  {
    if (!mesh_ptr_)
    {
      RCLCPP_FATAL_STREAM(node_->get_logger(), "The mesh pointer has not been initialized!");
      return false;
    }

    mesh_behavior_ptr->initialize(name, tf_listener_ptr_, mesh_ptr_, node_);
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "Recovery behavior plugin \"" << name << "\" initialized.");
    return true;
  }

  mbf_simple_core::SimpleRecovery::Ptr simple_behavior_ptr =
    std::dynamic_pointer_cast<mbf_simple_core::SimpleRecovery>(behavior_ptr);
  if (simple_behavior_ptr) 
  {
    simple_behavior_ptr->initialize(name, tf_listener_ptr_, node_);
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "Recovery behavior plugin \"" << name << "\" initialized.");
    return true;
  }

  RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to initialize plugin " << name << ". Looks like it is neither a mesh recovery behavior nor a simple recovery behavior.");
  return false;
}

void MeshNavigationServer::stop()
{
  AbstractNavigationServer::stop();
  // TODO
  // RCLCPP_INFO_STREAM_NAMED(node_->get_logger(), "mbf_mesh_nav", "Stopping mesh map for shutdown");
  // mesh_ptr_->stop();
}

MeshNavigationServer::~MeshNavigationServer()
{
}

void MeshNavigationServer::callServiceCheckPoseCost(std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<mbf_msgs::srv::CheckPose::Request> request, std::shared_ptr<mbf_msgs::srv::CheckPose::Response> response)
{
  // TODO implement
}

void MeshNavigationServer::callServiceCheckPathCost(std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<mbf_msgs::srv::CheckPath::Request> request, std::shared_ptr<mbf_msgs::srv::CheckPath::Response> response)
{
  // TODO implement
}

void MeshNavigationServer::callServiceClearMesh(std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  mesh_ptr_->resetLayers();
}

} /* namespace mbf_mesh_nav */