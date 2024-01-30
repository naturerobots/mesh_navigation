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
#include <mbf_msgs/action/exe_path.hpp>
#include <mesh_controller/mesh_controller.h>
#include <mesh_map/util.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
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

uint32_t MeshController::computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose,
                                                 const geometry_msgs::msg::TwistStamped& velocity,
                                                 geometry_msgs::msg::TwistStamped& cmd_vel,
                                                 std::string& message) 
{
  const auto& mesh = map_ptr_->mesh();

  robot_pos_ = poseToPositionVector(pose);
  robot_dir_ = poseToDirectionVector(pose);
  std::array<float, 3> bary_coords;
  std::array<mesh_map::Vector, 3> vertices;

  if (!current_face_)
  {
    // initially search current face on complete map
    if (auto search_res_opt = map_ptr_->searchContainingFace(
            robot_pos_, config_.max_search_distance))
    {
      auto search_res = *search_res_opt;
      current_face_ = std::get<0>(search_res);
      vertices = std::get<1>(search_res);
      bary_coords = std::get<2>(search_res);

      // project position onto surface
      robot_pos_ = mesh_map::linearCombineBarycentricCoords(vertices, bary_coords);
    }
    else
    {
      // no corresponding face has been found
      return mbf_msgs::action::ExePath::Result::OUT_OF_MAP;
    }
  }
  else // current face is set
  {
    lvr2::FaceHandle face = current_face_.unwrap();
    vertices = mesh.getVertexPositionsOfFace(face);
    DEBUG_CALL(map_ptr_->publishDebugFace(face, mesh_map::color(1, 1, 1), "current_face");)
    DEBUG_CALL(map_ptr_->publishDebugPoint(robot_pos_, mesh_map::color(1, 1, 1), "robot_position");)

    float dist_to_surface;
    // check whether or not the position matches the current face
    // if not search for new current face
    if (mesh_map::projectedBarycentricCoords(
        robot_pos_, vertices, bary_coords, dist_to_surface)
        && dist_to_surface < config_.max_search_distance)
    {
      // current position is located inside and close enough to the face
      DEBUG_CALL(map_ptr_->publishDebugPoint(robot_pos_, mesh_map::color(0, 0, 1), "current_position");)
    }
    else if (auto search_res_opt = map_ptr_->searchNeighbourFaces(
                 robot_pos_, face, config_.max_search_radius, config_.max_search_distance))
    {
      // new face has been found out of the neighbour faces of the current face
      // update variables to new face
      auto search_res = *search_res_opt;
      current_face_ = face = std::get<0>(search_res);
      vertices = std::get<1>(search_res);
      bary_coords = std::get<2>(search_res);
      robot_pos_ = mesh_map::linearCombineBarycentricCoords(vertices, bary_coords);
      DEBUG_CALL(map_ptr_->publishDebugFace(face, mesh_map::color(1, 0.5, 0), "search_neighbour_face");)
      DEBUG_CALL(map_ptr_->publishDebugPoint(robot_pos_, mesh_map::color(0, 0, 1), "search_neighbour_pos");)
    }
    else if(auto search_res_opt = map_ptr_->searchContainingFace(
        robot_pos_, config_.max_search_distance))
    {
      // update variables to new face
      auto search_res = *search_res_opt;
      current_face_ = face = std::get<0>(search_res);
      vertices = std::get<1>(search_res);
      bary_coords = std::get<2>(search_res);
      robot_pos_ = mesh_map::linearCombineBarycentricCoords(vertices, bary_coords);
    }
    else
    {
      // no corresponding face has been found
      return mbf_msgs::action::ExePath::Result::OUT_OF_MAP;
    }
  }

  const lvr2::FaceHandle& face = current_face_.unwrap();
  std::array<lvr2::VertexHandle, 3> handles = map_ptr_->mesh_ptr->getVerticesOfFace(face);

  // update to which position of the plan the robot is closest

  const auto& opt_dir = map_ptr_->directionAtPosition(vector_map_, handles, bary_coords);
  if (!opt_dir)
  {
    DEBUG_CALL(map_ptr_->publishDebugFace(face, mesh_map::color(0.3, 0.4, 0), "no_directions");)
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Could not access vector field for the given face!");
    return mbf_msgs::action::ExePath::Result::FAILURE;
  }
  mesh_map::Normal mesh_dir = opt_dir.get().normalized();
  float cost = map_ptr_->costAtPosition(handles, bary_coords);
  const mesh_map::Normal& mesh_normal = poseToDirectionVector(pose, tf2::Vector3(0,0,1));
  std::array<float, 2> velocities = naiveControl(robot_pos_, robot_dir_, mesh_dir, mesh_normal, cost);
  cmd_vel.twist.linear.x = std::min(config_.max_lin_velocity, velocities[0] * config_.lin_vel_factor);
  cmd_vel.twist.angular.z = std::min(config_.max_ang_velocity, velocities[1] * config_.ang_vel_factor);
  cmd_vel.header.stamp = node_->now();

  if (cancel_requested_)
  {
    return mbf_msgs::action::ExePath::Result::CANCELED;
  }
  return mbf_msgs::action::ExePath::Result::SUCCESS;
}

bool MeshController::isGoalReached(double dist_tolerance, double angle_tolerance)
{
  float goal_distance = (goal_pos_ - robot_pos_).length();
  float angle = acos(goal_dir_.dot(robot_dir_));
  return goal_distance <= static_cast<float>(dist_tolerance) && angle <= static_cast<float>(angle_tolerance);
}

bool MeshController::setPlan(const std::vector<geometry_msgs::msg::PoseStamped>& plan)
{
  // copy vector field // TODO just use vector field without copying
  vector_map_ = map_ptr_->getVectorMap();
  DEBUG_CALL(map_ptr_->publishDebugPoint(poseToPositionVector(plan.front()), mesh_map::color(0, 1, 0), "plan_start");)
  DEBUG_CALL(map_ptr_->publishDebugPoint(poseToPositionVector(plan.back()), mesh_map::color(1, 0, 0), "plan_goal");)
  current_plan_ = plan;
  goal_pos_ = poseToPositionVector(current_plan_.back());
  goal_dir_ = poseToDirectionVector(current_plan_.back());

  // reset current and ahead face
  cancel_requested_ = false;
  current_face_ = lvr2::OptionalFaceHandle();
  return true;
}

bool MeshController::cancel()
{
  RCLCPP_INFO_STREAM(node_->get_logger(), "The MeshController has been requested to cancel!");
  cancel_requested_ = true;
  return true;
}

mesh_map::Normal MeshController::poseToDirectionVector(const geometry_msgs::msg::PoseStamped& pose, const tf2::Vector3& axis)
{
  tf2::Stamped<tf2::Transform> transform;
  fromMsg(pose, transform);
  tf2::Vector3 v = transform.getBasis() * axis;
  return mesh_map::Normal(v.x(), v.y(), v.z());
}

mesh_map::Vector MeshController::poseToPositionVector(const geometry_msgs::msg::PoseStamped& pose)
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
  DEBUG_CALL(example_interfaces::msg::Float32 angle32; angle32.data = phi * 180 / M_PI; angle_pub_->publish(angle32);)

  float angular_velocity = copysignf(phi * config_.max_ang_velocity / M_PI, -sign_phi);
  const float max_angle = config_.max_angle * M_PI / 180.0;
  const float max_linear = config_.max_lin_velocity;
  float linear_velocity = phi <= max_angle ? max_linear - (phi * max_linear / max_angle) : 0.0;
  return { linear_velocity, angular_velocity };
}

rcl_interfaces::msg::SetParametersResult MeshController::reconfigureCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    if (parameter.get_name() == name_ + ".max_lin_velocity") {
      config_.max_lin_velocity = parameter.as_double();
    } else if (parameter.get_name() == name_ + ".max_ang_velocity") {
      config_.max_ang_velocity= parameter.as_double();
    } else if (parameter.get_name() == name_ + ".arrival_fading") {
      config_.arrival_fading = parameter.as_double();
    } else if (parameter.get_name() == name_ + ".ang_vel_factor") {
      config_.ang_vel_factor = parameter.as_double();
    } else if (parameter.get_name() == name_ + ".lin_vel_factor") {
      config_.lin_vel_factor = parameter.as_double();
    } else if (parameter.get_name() == name_ + ".max_angle") {
      config_.max_angle = parameter.as_double();
    } else if (parameter.get_name() == name_ + ".max_search_radius") {
      config_.max_search_radius = parameter.as_double();
    } else if (parameter.get_name() == name_ + ".max_search_distance") {
      config_.max_search_distance = parameter.as_double();
    }
  }

  result.successful = true;
  return result;
}

bool MeshController::initialize(const std::string& plugin_name,
                                const std::shared_ptr<tf2_ros::Buffer>& tf_ptr,
                                const std::shared_ptr<mesh_map::MeshMap>& mesh_map_ptr,
                                const rclcpp::Node::SharedPtr& node)
{
  node_ = node;
  map_ptr_ = mesh_map_ptr;
  name_ = plugin_name;

  angle_pub_ = node_->create_publisher<example_interfaces::msg::Float32>("~/current_angle", rclcpp::QoS(1).transient_local());

  { // cost max_lin_velocity
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Defines the maximum linear velocity";
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.0;
    range.to_value = 5.0;
    descriptor.floating_point_range.push_back(range);
    config_.max_lin_velocity = node->declare_parameter(name_ + ".max_lin_velocity", config_.max_lin_velocity);
  }
  { // cost max_ang_velocity
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Defines the maximum angular velocity";
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.0;
    range.to_value = 2.0;
    descriptor.floating_point_range.push_back(range);
    config_.max_ang_velocity = node->declare_parameter(name_ + ".max_ang_velocity", config_.max_ang_velocity);
  }
  { // cost arrival_fading
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Distance to goal position where the robot starts to fade down the linear velocity";
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.0;
    range.to_value = 5.0;
    descriptor.floating_point_range.push_back(range);
    config_.arrival_fading = node->declare_parameter(name_ + ".arrival_fading", config_.arrival_fading);
  }
  { // cost ang_vel_factor
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Factor for angular velocity";
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.1;
    range.to_value = 10.0;
    descriptor.floating_point_range.push_back(range);
    config_.ang_vel_factor = node->declare_parameter(name_ + ".ang_vel_factor", config_.ang_vel_factor);
  }
  { // cost lin_vel_factor
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Factor for linear velocity";
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.1;
    range.to_value = 10.0;
    descriptor.floating_point_range.push_back(range);
    config_.lin_vel_factor = node->declare_parameter(name_ + ".lin_vel_factor", config_.lin_vel_factor);
  }
  { // cost max_angle
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "The maximum angle for the linear velocity function";
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 1.0;
    range.to_value = 180.0;
    descriptor.floating_point_range.push_back(range);
    config_.max_angle = node->declare_parameter(name_ + ".max_angle", config_.max_angle);
  }
  { // cost max_search_radius
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "The maximum radius in which to search for a consecutive neighbour face";
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.01;
    range.to_value = 2.0;
    descriptor.floating_point_range.push_back(range);
    config_.max_search_radius = node->declare_parameter(name_ + ".max_search_radius", config_.max_search_radius);
  }
  { // cost max_search_distance
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "The maximum distance from the surface which is accepted for projection";
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.01;
    range.to_value = 2.0;
    descriptor.floating_point_range.push_back(range);
    config_.max_search_distance = node->declare_parameter(name_ + ".max_search_distance", config_.max_search_distance);
  }

  reconfiguration_callback_handle_ = node_->add_on_set_parameters_callback(std::bind(
      &MeshController::reconfigureCallback, this, std::placeholders::_1));

  return true;
}
} /* namespace mesh_controller */
