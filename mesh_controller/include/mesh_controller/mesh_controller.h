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
 */

#ifndef MESH_NAVIGATION__MESH_CONTROLLER_H
#define MESH_NAVIGATION__MESH_CONTROLLER_H

#include <mbf_mesh_core/mesh_controller.h>
#include <mbf_msgs/GetPathResult.h>
#include <mesh_controller/MeshControllerConfig.h>
#include <mesh_map/mesh_map.h>
#include <visualization_msgs/MarkerArray.h>

namespace mesh_controller
{
class MeshController : public mbf_mesh_core::MeshController
{
public:
  typedef boost::shared_ptr<mesh_controller::MeshController> Ptr;

  MeshController();

  /**
   * @brief Destructor
   */
  virtual ~MeshController();

  /**
   * @brief Given the current position, orientation, and velocity of the robot,
   * compute velocity commands to send to the base.
   * @param pose The current pose of the robot.
   * @param velocity The current velocity of the robot.
   * @param cmd_vel Will be filled with the velocity command to be passed to the
   * robot base.
   * @param message Optional more detailed outcome as a string
   * @return Result code as described on ExePath action result:
   *         SUCCESS         = 0
   *         1..9 are reserved as plugin specific non-error results
   *         FAILURE         = 100   Unspecified failure, only used for old,
   * non-mfb_core based plugins CANCELED        = 101 NO_VALID_CMD    = 102
   *         PAT_EXCEEDED    = 103
   *         COLLISION       = 104
   *         OSCILLATION     = 105
   *         ROBOT_STUCK     = 106
   *         MISSED_GOAL     = 107
   *         MISSED_PATH     = 108
   *         BLOCKED_PATH    = 109
   *         INVALID_PATH    = 110
   *         TF_ERROR        = 111
   *         NOT_INITIALIZED = 112
   *         INVALID_PLUGIN  = 113
   *         INTERNAL_ERROR  = 114
   *         121..149 are reserved as plugin specific errors
   */
  virtual uint32_t computeVelocityCommands(const geometry_msgs::PoseStamped& pose,
                                           const geometry_msgs::TwistStamped& velocity,
                                           geometry_msgs::TwistStamped& cmd_vel, std::string& message);

  /**
   * @brief Check if the goal pose has been achieved by the local planner
   * @param pose The current pose of the robot.
   * @param angle_tolerance The angle tolerance in which the current pose will
   * be partly accepted as reached goal
   * @param dist_tolerance The distance tolerance in which the current pose will
   * be partly accepted as reached goal
   * @return True if achieved, false otherwise
   */
  virtual bool isGoalReached(double dist_tolerance, double angle_tolerance);

  /**
   * @brief Set the plan that the local planner is following
   * @param plan The plan to pass to the local planner
   * @return True if the plan was updated successfully, false otherwise
   */
  virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

  /**
   * @brief Requests the planner to cancel, e.g. if it takes too much time.
   * @return True if a cancel has been successfully requested, false if not
   * implemented.
   */
  virtual bool cancel();

  /**
   * Transforms a PoseStamped into a direction mesh_map Vector
   * @param pose      any geometry_msgs PoseStamped
   * @return          direction mesh_map Normal
   */
  mesh_map::Normal poseToDirectionVector(
      const geometry_msgs::PoseStamped& pose,
      const tf2::Vector3& axis=tf2::Vector3(1,0,0));


  /**
   * Transforms a PoseStamped into a position vector
   * @param pose      any geometry_msgs PoseStamped
   * @return          position mesh_map Vector
   */
  mesh_map::Vector poseToPositionVector(
      const geometry_msgs::PoseStamped& pose);

  /**
   * A normal distribution / gaussian function
   * @param sigma_squared The squared sigma / variance
   * @param value         The value to be evaluated
   * @return              the gauss faction value
   */
  float gaussValue(const float& sigma_squared, const float& value);

  /**
   * Computes the angular and linear velocity
   * @param robot_pos     robot position
   * @param robot_dir     robot orientation
   * @param mesh_dir      supposed orientation
   * @param mesh_cost     cost value at robot position
   * @return              angular and linear velocity
   */
  std::array<float, 2> naiveControl(
      const mesh_map::Vector& robot_pos,
      const mesh_map::Normal& robot_dir,
      const mesh_map::Vector& mesh_dir,
      const mesh_map::Normal& mesh_normal,
      const float& mesh_cost);

  void reconfigureCallback(mesh_controller::MeshControllerConfig& cfg, uint32_t level);

  virtual bool initialize(const std::string& plugin_name, const boost::shared_ptr<tf2_ros::Buffer>& tf_ptr,
                          const boost::shared_ptr<mesh_map::MeshMap>& mesh_map_ptr);

private:
  boost::shared_ptr<mesh_map::MeshMap> map_ptr;
  vector<geometry_msgs::PoseStamped> current_plan;

  mesh_map::Vector goal_pos, robot_pos;
  mesh_map::Normal goal_dir, robot_dir;

  lvr2::OptionalFaceHandle current_face;
  // angle between pose vector and planned / supposed vector
  geometry_msgs::Pose current_pose;

  lvr2::DenseVertexMap<mesh_map::Vector> vector_map;

  boost::shared_ptr<dynamic_reconfigure::Server<mesh_controller::MeshControllerConfig>> reconfigure_server_ptr;
  dynamic_reconfigure::Server<mesh_controller::MeshControllerConfig>::CallbackType config_callback;
  MeshControllerConfig config;

  ros::Publisher angle_pub;

  std::atomic_bool cancel_requested;
};

} /* namespace mesh_controller */
#endif /* MESH_NAVIGATION__MESH_CONTROLLER_H */
