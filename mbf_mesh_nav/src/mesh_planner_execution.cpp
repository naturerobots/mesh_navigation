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
#include "mbf_mesh_nav/mesh_planner_execution.h"

namespace mbf_mesh_nav
{
MeshPlannerExecution::MeshPlannerExecution(const std::string name,
                                           const mbf_mesh_core::MeshPlanner::Ptr& planner_ptr,
                                           const mbf_utility::RobotInformation::ConstPtr& robot_info,
                                           const MeshPtr& mesh_ptr,
                                           const rclcpp::Node::SharedPtr& node)
  : AbstractPlannerExecution(name, planner_ptr, robot_info, node), mesh_ptr_(mesh_ptr)
{
  lock_mesh_ = node_->declare_parameter("planner_lock_mesh", true);
}

MeshPlannerExecution::~MeshPlannerExecution()
{
}

uint32_t MeshPlannerExecution::makePlan(const geometry_msgs::msg::PoseStamped &start,
                                           const geometry_msgs::msg::PoseStamped &goal,
                                           double tolerance,
                                           std::vector<geometry_msgs::msg::PoseStamped> &plan,
                                           double &cost,
                                           std::string &message)
{
  rclcpp::Time start_time = node_->now();
  uint32_t outcome = planner_->makePlan(start, goal, tolerance, plan, cost, message);
  RCLCPP_INFO_STREAM(node_->get_logger(), "Runtime of " << plugin_name_ << ":" << (node_->now() - start_time).nanoseconds() * 1e-6 << "ms");
  return outcome;
}

} /* namespace mbf_mesh_nav */
