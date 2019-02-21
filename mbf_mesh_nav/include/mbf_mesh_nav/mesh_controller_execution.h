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

#ifndef MBF_MESH_NAV__MESH_CONTROLLER_EXECUTION_H
#define MBF_MESH_NAV__MESH_CONTROLLER_EXECUTION_H

#include <mesh_map/mesh_map.h>
#include <mbf_mesh_nav/MoveBaseFlexConfig.h>
#include <mbf_mesh_core/mesh_controller.h>
#include <mbf_abstract_nav/abstract_controller_execution.h>

namespace mbf_mesh_nav
{
/**
 * @brief The MeshControllerExecution binds a mesh to the AbstractControllerExecution and uses the
 *        mbf_mesh_core/MeshController class as base plugin interface.
 *
 * @ingroup controller_execution move_base_server
 */
class MeshControllerExecution : public mbf_abstract_nav::AbstractControllerExecution
{
public:

  typedef boost::shared_ptr<mesh_map::MeshMap> MeshPtr;

  /**
   * @brief Constructor
   * @param name                The user defined name of the corresponding plugin
   * @param controller_ptr      The shared pointer to the plugin object
   * @param vel_pub             The velocity publisher for the controller execution
   * @param goal_pub            The current goal publisher fir the controller execution
   * @param tf_listener_ptr     A shared pointer to the transform listener
   * @param mesh_ptr            A pointer to the mesh map object
   * @param config              The current config object
   * @param setup_fn            A setup function called before execution
   * @param cleanup_fn          A cleanup function called after execution
   */
  MeshControllerExecution(
      const std::string name,
      const mbf_mesh_core::MeshController::Ptr &controller_ptr,
      const ros::Publisher& vel_pub,
      const ros::Publisher& goal_pub,
      const TFPtr &tf_listener_ptr,
      MeshPtr &mesh_ptr,
      const MoveBaseFlexConfig &config,
      boost::function<void()> setup_fn,
      boost::function<void()> cleanup_fn);

  /**
   * @brief Destructor
   */
  virtual ~MeshControllerExecution();

protected:

  /**
   * @brief Request plugin for a new velocity command. We override this method so we can lock the local mesh
   *        before calling the planner.
   * @param robot_pose         The current pose of the robot.
   * @param robot_velocity     The current velocity of the robot.
   * @param cmd_vel            Will be filled with the velocity command to be passed to the robot base.
   * @param message            Optional more detailed outcome as a string.
   * @return                   Result code as described in the ExePath action result and plugin's header.
   */
  virtual uint32_t computeVelocityCmd(
      const geometry_msgs::PoseStamped& robot_pose,
      const geometry_msgs::TwistStamped& robot_velocity,
      geometry_msgs::TwistStamped& vel_cmd,
      std::string& message);

private:

  mbf_abstract_nav::MoveBaseFlexConfig toAbstract(const MoveBaseFlexConfig &config);

  //! mesh for 3d navigation planning
  MeshPtr &mesh_ptr_;

  //! Whether to lock mesh before calling the controller
  bool lock_mesh_;

  //! name of the controller plugin assigned by the class loader
  std::string controller_name_;
};

} /* namespace mbf_mesh_nav */

#endif /* MBF_MESH_NAV__MESH_CONTROLLER_EXECUTION_H */
