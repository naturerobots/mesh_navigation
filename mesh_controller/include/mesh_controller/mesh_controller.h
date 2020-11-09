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

  //! shared pointer typedef to simplify pointer access of the mesh controller
  typedef boost::shared_ptr<mesh_controller::MeshController> Ptr;

  /**
   * @brief Constructor
   */
  MeshController();

  /**
   * @brief Destructor
   */
  virtual ~MeshController();

  /**
   * @brief Given the current position, orientation, and velocity of the robot,
   * compute the next velocity commands to move the robot towards the goal.
   * @param pose The current pose of the robot
   * @param velocity The current velocity of the robot
   * @param cmd_vel Computed velocity command
   * @param message Detailed outcome as string message
   * @return An mbf_msgs/ExePathResult outcome code
   */
  virtual uint32_t computeVelocityCommands(const geometry_msgs::PoseStamped& pose,
                                           const geometry_msgs::TwistStamped& velocity,
                                           geometry_msgs::TwistStamped& cmd_vel, std::string& message);

  /**
   * @brief Checks if the robot reached to goal pose
   * @param pose The current pose of the robot
   * @param angle_tolerance The angle tolerance in which the current pose will
   * be partly accepted as reached goal
   * @param dist_tolerance The distance tolerance in which the current pose will
   * be partly accepted as reached goal
   * @return true if the goal is reached
   */
  virtual bool isGoalReached(double dist_tolerance, double angle_tolerance);

  /**
   * @brief Sets the current plan to follow, it also sets the vector field
   * @param plan The plan to follow
   * @return true if the plan was set successfully, false otherwise
   */
  virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

  /**
   * @brief Requests the planner to cancel, e.g. if it takes too much time
   * @return True if cancel has been successfully requested, false otherwise
   */
  virtual bool cancel();

  /**
   * Converts the orientation of a geometry_msgs/PoseStamped message to a direction vector
   * @param pose      the pose to convert
   * @return          direction normal vector
   */
  mesh_map::Normal poseToDirectionVector(
      const geometry_msgs::PoseStamped& pose,
      const tf2::Vector3& axis=tf2::Vector3(1,0,0));


  /**
   * Converts the position of a geometry_msgs/PoseStamped message to a position vector
   * @param pose      the pose to convert
   * @return          position vector
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

  /**
   * @brief reconfigure callback function which is called if a dynamic reconfiguration were triggered.
   */
  void reconfigureCallback(mesh_controller::MeshControllerConfig& cfg, uint32_t level);

  /**
   * @brief Initializes the controller plugin with a name, a tf pointer and a mesh map pointer
   * @param plugin_name The controller plugin name, defined by the user. It defines the controller namespace
   * @param tf_ptr A shared pointer to a transformation buffer
   * @param mesh_map_ptr A shared pointer to the mesh map
   * @return true if the plugin has been initialized successfully
   */
  virtual bool initialize(const std::string& plugin_name, const boost::shared_ptr<tf2_ros::Buffer>& tf_ptr,
                          const boost::shared_ptr<mesh_map::MeshMap>& mesh_map_ptr);

private:

  //! shared pointer to the used mesh map
  boost::shared_ptr<mesh_map::MeshMap> map_ptr;

  //! the current set plan
  vector<geometry_msgs::PoseStamped> current_plan;

  //! the goal and robot pose
  mesh_map::Vector goal_pos, robot_pos;

  //! the goal's and robot's orientation
  mesh_map::Normal goal_dir, robot_dir;

  //! The triangle on which the robot is located
  lvr2::OptionalFaceHandle current_face;

  //! The vector field to the goal.
  lvr2::DenseVertexMap<mesh_map::Vector> vector_map;

  //! shared pointer to dynamic reconfigure server
  boost::shared_ptr<dynamic_reconfigure::Server<mesh_controller::MeshControllerConfig>> reconfigure_server_ptr;

  //! dynamic reconfigure callback function binding
  dynamic_reconfigure::Server<mesh_controller::MeshControllerConfig>::CallbackType config_callback;

  //! current mesh controller configuration
  MeshControllerConfig config;

  //! publishes the angle between the robots orientation and the goal vector field for debug purposes
  ros::Publisher angle_pub;

  //! flag to handle cancel requests
  std::atomic_bool cancel_requested;
};

} /* namespace mesh_controller */
#endif /* MESH_NAVIGATION__MESH_CONTROLLER_H */
