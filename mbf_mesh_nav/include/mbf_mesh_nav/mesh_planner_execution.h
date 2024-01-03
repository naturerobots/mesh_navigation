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

#ifndef MBF_MESH_NAV__MESH_PLANNER_EXECUTION_H
#define MBF_MESH_NAV__MESH_PLANNER_EXECUTION_H

#include <mbf_abstract_nav/abstract_planner_execution.h>
#include <mbf_mesh_core/mesh_planner.h>
#include <mesh_map/mesh_map.h>

namespace mbf_mesh_nav
{
/**
 * @brief The MeshPlannerExecution binds a global mesh to the
 * AbstractPlannerExecution and uses the nav_core/BaseMeshPlanner class as base
 * plugin interface. This class makes move_base_flex compatible to the old
 * move_base.
 *
 * @ingroup planner_execution move_base_server
 */
class MeshPlannerExecution : public mbf_abstract_nav::AbstractPlannerExecution
{
public:
  typedef std::shared_ptr<mesh_map::MeshMap> MeshPtr;

  /**
   * @brief Constructor
   * @param condition Thread sleep condition variable, to wake up connected
   * threads
   * @param mesh Shared pointer to the mesh.
   */
  MeshPlannerExecution(const std::string name, const mbf_mesh_core::MeshPlanner::Ptr& planner_ptr, const mbf_utility::RobotInformation::ConstPtr& robot_info, const MeshPtr& mesh, const rclcpp::Node::SharedPtr& node);

  /**
   * @brief Destructor
   */
  virtual ~MeshPlannerExecution();

private:
  /**
   * @brief Calls the planner plugin to make a plan from the start pose to the
   * goal pose with the given tolerance, if a goal tolerance is enabled in the
   * planner plugin.
   * @param start The start pose for planning
   * @param goal The goal pose for planning
   * @param tolerance The goal tolerance
   * @param plan The computed plan by the plugin
   * @param cost The computed costs for the corresponding plan
   * @param message An optional message which should correspond with the
   * returned outcome
   * @return An outcome number, see also the action definition in the
   * GetPath.action file
   */
  virtual uint32_t makePlan(
      const geometry_msgs::msg::PoseStamped &start,
      const geometry_msgs::msg::PoseStamped &goal,
      double tolerance,
      std::vector<geometry_msgs::msg::PoseStamped> &plan,
      double &cost,
      std::string &message);

  //! Shared pointer to the mesh for 3d navigation planning
  const MeshPtr mesh_ptr_;

  //! Whether to lock mesh before calling the planner (see issue #4 for details)
  bool lock_mesh_;

  //! Name of the planner assigned by the class loader
  std::string planner_name_;
};

} /* namespace mbf_mesh_nav */

#endif /* MBF_MESH_NAV__MESH_PLANNER_EXECUTION_H */
