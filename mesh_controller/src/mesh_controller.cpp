/*
 *  Copyright 2020, Sebastian Pütz, Sabrina Frohn
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
 *    Sebastian Pütz <spuetz@uos.de>
 *
 */

#include <lvr2/geometry/HalfEdgeMesh.hpp>
#include <lvr2/util/Meap.hpp>
#include <mbf_msgs/ExePathResult.h>
#include <mbf_msgs/GetPathResult.h>
#include <mesh_controller/mesh_controller.h>
#include <mesh_map/util.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float32.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <mbf_utility/exe_path_exception.h>

PLUGINLIB_EXPORT_CLASS(mesh_controller::MeshController, mbf_mesh_core::MeshController);

#define DEBUG

#ifdef DEBUG
#define DEBUG_CALL(method) method
#else
#define DEBUG_CALL(method)
#endif

namespace mesh_controller
{
MeshController::MeshController()
{
}

MeshController::~MeshController()
{
}

uint32_t MeshController::computeVelocityCommands(const geometry_msgs::PoseStamped& pose,
                                                 const geometry_msgs::TwistStamped& velocity,
                                                 geometry_msgs::TwistStamped& cmd_vel, std::string& message)
{
  const auto& mesh = map_ptr->mesh();

  robot_pos = poseToPositionVector(pose);
  robot_dir = poseToDirectionVector(pose);
  std::array<float, 3> bary_coords;
  std::array<mesh_map::Vector, 3> vertices;

  if (!current_face)
  {
    // initially search current face on complete map
    if (auto search_res_opt = map_ptr->searchContainingFace(
            robot_pos, config.max_search_distance))
    {
      auto search_res = *search_res_opt;
      current_face = std::get<0>(search_res);
      vertices = std::get<1>(search_res);
      bary_coords = std::get<2>(search_res);

      // project position onto surface
      robot_pos = mesh_map::linearCombineBarycentricCoords(vertices, bary_coords);
    }
    else
    {
      // no corresponding face has been found
      return mbf_msgs::ExePathResult::OUT_OF_MAP;
    }
  }
  else // current face is set
  {
    lvr2::FaceHandle face = current_face.unwrap();
    vertices = mesh.getVertexPositionsOfFace(face);
    DEBUG_CALL(map_ptr->publishDebugFace(face, mesh_map::color(1, 1, 1), "current_face");)
    DEBUG_CALL(map_ptr->publishDebugPoint(robot_pos, mesh_map::color(1, 1, 1), "robot_position");)

    float dist_to_surface;
    // check whether or not the position matches the current face
    // if not search for new current face
    if (mesh_map::projectedBarycentricCoords(
        robot_pos, vertices, bary_coords, dist_to_surface)
        && dist_to_surface < config.max_search_distance)
    {
      // current position is located inside and close enough to the face
      DEBUG_CALL(map_ptr->publishDebugPoint(robot_pos, mesh_map::color(0, 0, 1), "current_position");)
    }
    else if (auto search_res_opt = map_ptr->searchNeighbourFaces(
                 robot_pos, face, config.max_search_radius, config.max_search_distance))
    {
      // new face has been found out of the neighbour faces of the current face
      // update variables to new face
      auto search_res = *search_res_opt;
      current_face = face = std::get<0>(search_res);
      vertices = std::get<1>(search_res);
      bary_coords = std::get<2>(search_res);
      robot_pos = mesh_map::linearCombineBarycentricCoords(vertices, bary_coords);
      DEBUG_CALL(map_ptr->publishDebugFace(face, mesh_map::color(1, 0.5, 0), "search_neighbour_face");)
      DEBUG_CALL(map_ptr->publishDebugPoint(robot_pos, mesh_map::color(0, 0, 1), "search_neighbour_pos");)
    }
    else if(auto search_res_opt = map_ptr->searchContainingFace(
        robot_pos, config.max_search_distance))
    {
      // update variables to new face
      auto search_res = *search_res_opt;
      current_face = face = std::get<0>(search_res);
      vertices = std::get<1>(search_res);
      bary_coords = std::get<2>(search_res);
      robot_pos = mesh_map::linearCombineBarycentricCoords(vertices, bary_coords);
    }
    else
    {
      // no corresponding face has been found
      return mbf_msgs::ExePathResult::OUT_OF_MAP;
    }
  }

  const lvr2::FaceHandle& face = current_face.unwrap();
  std::array<lvr2::VertexHandle, 3> handles = map_ptr->mesh_ptr->getVerticesOfFace(face);

  // update to which position of the plan the robot is closest

  const auto& opt_dir = map_ptr->directionAtPosition(vector_map, handles, bary_coords);
  if (!opt_dir)
  {
    DEBUG_CALL(map_ptr->publishDebugFace(face, mesh_map::color(0.3, 0.4, 0), "no_directions");)
    ROS_ERROR_STREAM("Could not access vector field for the given face!");
    return mbf_msgs::ExePathResult::FAILURE;
  }
  mesh_map::Normal mesh_dir = opt_dir.get().normalized();
  float cost = map_ptr->costAtPosition(handles, bary_coords);
  const mesh_map::Normal& mesh_normal = poseToDirectionVector(pose, tf2::Vector3(0,0,1));
  std::array<float, 2> velocities = naiveControl(robot_pos, robot_dir, mesh_dir, mesh_normal, cost);
  cmd_vel.twist.linear.x = std::min(config.max_lin_velocity, velocities[0] * config.lin_vel_factor);
  cmd_vel.twist.angular.z = std::min(config.max_ang_velocity, velocities[1] * config.ang_vel_factor);
  cmd_vel.header.stamp = ros::Time::now();

  if (cancel_requested)
  {
    return mbf_msgs::ExePathResult::CANCELED;
  }
  return mbf_msgs::ExePathResult::SUCCESS;
}

bool MeshController::isGoalReached(double dist_tolerance, double angle_tolerance)
{
  float goal_distance = (goal_pos - robot_pos).length();
  float angle = acos(goal_dir.dot(robot_dir));
  return goal_distance <= static_cast<float>(dist_tolerance) && angle <= static_cast<float>(angle_tolerance);
}

bool MeshController::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
  // copy vector field // TODO just use vector field without copying
  vector_map = map_ptr->getVectorMap();
  DEBUG_CALL(map_ptr->publishDebugPoint(poseToPositionVector(plan.front()), mesh_map::color(0, 1, 0), "plan_start");)
  DEBUG_CALL(map_ptr->publishDebugPoint(poseToPositionVector(plan.back()), mesh_map::color(1, 0, 0), "plan_goal");)
  current_plan = plan;
  goal_pos = poseToPositionVector(current_plan.back());
  goal_dir = poseToDirectionVector(current_plan.back());

  // reset current and ahead face
  cancel_requested = false;
  current_face = lvr2::OptionalFaceHandle();
  return true;
}

bool MeshController::cancel()
{
  ROS_INFO_STREAM("The MeshController has been requested to cancel!");
  cancel_requested = true;
  return true;
}

mesh_map::Normal MeshController::poseToDirectionVector(const geometry_msgs::PoseStamped& pose, const tf2::Vector3& axis)
{
  tf2::Stamped<tf2::Transform> transform;
  fromMsg(pose, transform);
  tf2::Vector3 v = transform.getBasis() * axis;
  return mesh_map::Normal(v.x(), v.y(), v.z());
}

mesh_map::Vector MeshController::poseToPositionVector(const geometry_msgs::PoseStamped& pose)
{
  return mesh_map::Vector(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
}

float MeshController::gaussValue(const float& sigma_squared, const float& value)
{
  return exp(-value * value / 2 * sigma_squared) / sqrt(2 * M_PI * sigma_squared);
}

std::array<float, 2> MeshController::naiveControl(
    const mesh_map::Vector& robot_pos,
    const mesh_map::Normal& robot_dir,
    const mesh_map::Vector& mesh_dir,
    const mesh_map::Normal& mesh_normal,
    const float& mesh_cost)
{
  float phi = acos(mesh_dir.dot(robot_dir));
  float sign_phi = mesh_dir.cross(robot_dir).dot(mesh_normal);
  // debug output angle between supposed and current angle
  DEBUG_CALL(std_msgs::Float32 angle32; angle32.data = phi * 180 / M_PI; angle_pub.publish(angle32);)

  float angular_velocity = copysignf(phi * config.max_ang_velocity / M_PI, -sign_phi);
  const float max_angle = config.max_angle * M_PI / 180.0;
  const float max_linear = config.max_lin_velocity;
  float linear_velocity = phi <= max_angle ? max_linear - (phi * max_linear / max_angle) : 0.0;
  return { linear_velocity, angular_velocity };
}

void MeshController::reconfigureCallback(mesh_controller::MeshControllerConfig& cfg, uint32_t level)
{
  config = cfg;
}

bool MeshController::initialize(const std::string& plugin_name, const boost::shared_ptr<tf2_ros::Buffer>& tf_ptr,
                                const boost::shared_ptr<mesh_map::MeshMap>& mesh_map_ptr)
{
  ros::NodeHandle private_nh("~/" + plugin_name);
  map_ptr = mesh_map_ptr;
  reconfigure_server_ptr = boost::shared_ptr<dynamic_reconfigure::Server<mesh_controller::MeshControllerConfig>>(
      new dynamic_reconfigure::Server<mesh_controller::MeshControllerConfig>(private_nh));

  config_callback = boost::bind(&MeshController::reconfigureCallback, this, _1, _2);
  reconfigure_server_ptr->setCallback(config_callback);
  angle_pub = private_nh.advertise<std_msgs::Float32>("current_angle", 1);
  return true;
}
} /* namespace mesh_controller */
