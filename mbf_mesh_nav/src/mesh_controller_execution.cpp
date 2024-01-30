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
#include "mbf_mesh_nav/mesh_controller_execution.h"

namespace mbf_mesh_nav
{

MeshControllerExecution::MeshControllerExecution(const std::string name,
                                                 const mbf_mesh_core::MeshController::Ptr& controller_ptr,
                                                 const ros::Publisher& vel_pub, const ros::Publisher& goal_pub,
                                                 const TFPtr& tf_listener_ptr, const MeshPtr& mesh_ptr,
                                                 const MoveBaseFlexConfig& config)
  : AbstractControllerExecution(name, controller_ptr, vel_pub, goal_pub, tf_listener_ptr, toAbstract(config))
  , mesh_ptr_(mesh_ptr)
{
  ros::NodeHandle private_nh("~");
  private_nh.param("controller_lock_mesh", lock_mesh_, true);
}

MeshControllerExecution::~MeshControllerExecution()
{
}

mbf_abstract_nav::MoveBaseFlexConfig MeshControllerExecution::toAbstract(const MoveBaseFlexConfig& config)
{
  // copy the controller-related abstract configuration common to all MBF-based
  // navigation
  mbf_abstract_nav::MoveBaseFlexConfig abstract_config;
  abstract_config.controller_frequency = config.controller_frequency;
  abstract_config.controller_patience = config.controller_patience;
  abstract_config.controller_max_retries = config.controller_max_retries;
  abstract_config.oscillation_timeout = config.oscillation_timeout;
  abstract_config.oscillation_distance = config.oscillation_distance;
  return abstract_config;
}

uint32_t MeshControllerExecution::computeVelocityCmd(const geometry_msgs::PoseStamped& robot_pose,
                                                     const geometry_msgs::TwistStamped& robot_velocity,
                                                     geometry_msgs::TwistStamped& vel_cmd, std::string& message)
{
  // Lock the mesh while planning, but following issue #4, we allow to move the
  // responsibility to the planner itself
  if (lock_mesh_)
  {
    // TODO
    // boost::lock_guard<mesh::Mesh::mutex_t> lock(*(mesh_ptr_->getMutex()));
    // return controller_->computeVelocityCommands(robot_pose, robot_velocity,
    // vel_cmd, message);
  }
  return controller_->computeVelocityCommands(robot_pose, robot_velocity, vel_cmd, message);
}

} /* namespace mbf_mesh_nav */
