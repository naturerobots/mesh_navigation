/*
 *  Copyright 2019, Sebastian Pütz
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

#include <pluginlib/class_list_macros.h>
#include <mesh_planner/mesh_planner.h>
#include <mbf_msgs/GetPathResult.h>

PLUGINLIB_EXPORT_CLASS(mesh_planner::MeshPlanner, mbf_mesh_core::MeshPlanner);

namespace mesh_planner{

  MeshPlanner::MeshPlanner()
    : private_nh_("~")
  {

  }

  MeshPlanner::~MeshPlanner(){

  }

  uint32_t MeshPlanner::makePlan(
      const geometry_msgs::PoseStamped &start,
      const geometry_msgs::PoseStamped &goal,
      double tolerance,
      std::vector<geometry_msgs::PoseStamped> &plan,
      double &cost,
      std::string &message)
  {
    bool use_fmm = private_nh_.param<bool>("use_fmm", true);

    ROS_INFO("make plan...");
    if(mesh_ptr_->pathPlanning(start, goal, plan, use_fmm))
    {
      ROS_INFO("Found a plan!");
      return mbf_msgs::GetPathResult::SUCCESS;
    }

    ROS_WARN("Could not find a plan!");
    return mbf_msgs::GetPathResult::NO_PATH_FOUND;
  }

  bool MeshPlanner::cancel(){
    return false;
  }

  bool MeshPlanner::initialize(
      const std::string& name,
      const boost::shared_ptr<mesh_map::MeshMap>& mesh_map_ptr)
  {
    mesh_ptr_ = mesh_map_ptr;
    name_ = name;

    return false;
  }

} /* namespace mesh_planner */