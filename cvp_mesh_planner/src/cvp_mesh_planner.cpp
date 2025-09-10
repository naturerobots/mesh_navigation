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

#include <lvr2/geometry/Handles.hpp>
#include <lvr2/util/Meap.hpp>

#include <chrono>
#include <mesh_map/util.h>
#include <pluginlib/class_list_macros.hpp>

#include "cvp_mesh_planner/cvp_mesh_planner.h"
//#define DEBUG
//#define USE_UPDATE_WITH_S
//#define USE_UPDATE_FMM

PLUGINLIB_EXPORT_CLASS(cvp_mesh_planner::CVPMeshPlanner, mbf_mesh_core::MeshPlanner);

namespace cvp_mesh_planner
{
CVPMeshPlanner::CVPMeshPlanner()
{
}

CVPMeshPlanner::~CVPMeshPlanner()
{
}

uint32_t CVPMeshPlanner::makePlan(const geometry_msgs::msg::PoseStamped& start,
                            const geometry_msgs::msg::PoseStamped& goal,
                            double tolerance, 
                            std::vector<geometry_msgs::msg::PoseStamped>& plan, double& cost,
                            std::string& message)
{
  const auto mesh = mesh_map_->mesh();
  std::list<std::pair<mesh_map::Vector, lvr2::FaceHandle>> path;

  // mesh_map->combineVertexCosts(); // TODO should be outside the planner

  RCLCPP_DEBUG_STREAM(node_->get_logger(), "start wave front propagation.");

  mesh_map::Vector goal_vec = mesh_map::toVector(goal.pose.position);
  mesh_map::Vector start_vec = mesh_map::toVector(start.pose.position);

  const uint32_t outcome = waveFrontPropagation(goal_vec, start_vec, path, message);

  RCLCPP_DEBUG_STREAM(node_->get_logger(), "finished wave front propagation.");

  path.reverse();

  std_msgs::msg::Header header;
  header.stamp = node_->now();
  header.frame_id = mesh_map_->mapFrame();

  cost = 0;
  float dir_length;
  if (!cancel_planning_ && !path.empty())
  {
    mesh_map::Vector vec = path.front().first;
    lvr2::FaceHandle fH = path.front().second;
    path.pop_front();

    const auto& face_normals = mesh_map_->faceNormals();
    for (auto& next : path)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = header;
      pose.pose = mesh_map::calculatePoseFromPosition(vec, next.first, face_normals[fH], dir_length);
      cost += dir_length;
      vec = next.first;
      fH = next.second;
      plan.push_back(pose);
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    pose.pose = goal.pose;
    plan.push_back(pose);
  }

  nav_msgs::msg::Path path_msg;
  path_msg.poses = plan;
  path_msg.header = header;

  path_pub_->publish(path_msg);
  mesh_map_->publishVertexCosts(potential_, "Potential", header.stamp);
  RCLCPP_INFO_STREAM(node_->get_logger(), "Path length: " << cost << "m");

  if (config_.publish_vector_field)
  {
    mesh_map_->publishVectorField("vector_field", vector_map_, config_.publish_face_vectors);
  }

  return outcome;
}

bool CVPMeshPlanner::cancel()
{
  cancel_planning_ = true;
  return true;
}

bool CVPMeshPlanner::initialize(const std::string& plugin_name, const std::shared_ptr<mesh_map::MeshMap>& mesh_map_ptr, const rclcpp::Node::SharedPtr& node) 
{
  mesh_map_ = mesh_map_ptr;
  name_ = plugin_name;
  map_frame_ = mesh_map_->mapFrame();
  node_ = node;

  config_.publish_vector_field = node_->declare_parameter(name_ + ".publish_vector_field", config_.publish_vector_field);
  config_.publish_face_vectors = node_->declare_parameter(name_ + ".publish_face_vectors", config_.publish_face_vectors);
  config_.goal_dist_offset = node_->declare_parameter(name_ + ".goal_dist_offset", config_.goal_dist_offset);
  { // cost limit param
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Defines the vertex cost limit with which it can be accessed.";
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.0;
    range.to_value = 10.0;
    descriptor.floating_point_range.push_back(range);
    config_.cost_limit = node->declare_parameter(name_ + ".cost_limit", config_.cost_limit);
  }
  { // step width param
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "The vector field back tracking step width.";
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.01;
    range.to_value = 1.0;
    descriptor.floating_point_range.push_back(range);
    config_.step_width = node->declare_parameter(name_ + ".step_width", config_.step_width);
  }

  path_pub_ = node->create_publisher<nav_msgs::msg::Path>("~/path", rclcpp::QoS(1).transient_local());
  const auto mesh = mesh_map_->mesh();
  direction_ = lvr2::DenseVertexMap<float>(mesh->nextVertexIndex(), 0);
  // TODO check all map dependencies! (loaded layers etc...)

  reconfiguration_callback_handle_ = node_->add_on_set_parameters_callback(std::bind(
      &CVPMeshPlanner::reconfigureCallback, this, std::placeholders::_1));

  return true;
}

rcl_interfaces::msg::SetParametersResult CVPMeshPlanner::reconfigureCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    if (parameter.get_name() == name_ + ".cost_limit") {
      config_.cost_limit = parameter.as_double();
    } else if (parameter.get_name() == name_ + ".step_width") {
      config_.step_width = parameter.as_double();
    }
  }

  result.successful = true;
  return result;
}

void CVPMeshPlanner::computeVectorMap()
{
  const auto mesh = mesh_map_->mesh();
  const auto& face_normals = mesh_map_->faceNormals();
  const auto& vertex_normals = mesh_map_->vertexNormals();

  for (auto v3 : mesh->vertices())
  {
    // if(vertex_costs[v3] > config.cost_limit || !predecessors.containsKey(v3))
    // continue;

    const lvr2::VertexHandle& v1 = predecessors_[v3];

    // if predecessor is pointing to it self, continue with the next vertex.
    if (v1 == v3)
      continue;

    // get the cut face
    const auto& optFh = cutting_faces_.get(v3);
    // if no cut face, continue with the next vertex
    if (!optFh)
      continue;

    const lvr2::FaceHandle& fH = optFh.get();

    const auto& vec3 = mesh->getVertexPosition(v3);
    const auto& vec1 = mesh->getVertexPosition(v1);

    // compute the direction vector and rotate it by theta, which is stored in
    // the direction vertex map
    const auto dirVec = (vec1 - vec3).rotated(vertex_normals[v3], direction_[v3]);
    // store the normalized rotated vector in the vector map
    vector_map_.insert(v3, dirVec.normalized());
  }
  mesh_map_->setVectorMap(vector_map_);
}

uint32_t CVPMeshPlanner::waveFrontPropagation(const mesh_map::Vector& start, const mesh_map::Vector& goal,
                                              std::list<std::pair<mesh_map::Vector, lvr2::FaceHandle>>& path,
                                              std::string& message)
{
  return waveFrontPropagation(start, goal, mesh_map_->edgeWeights(), mesh_map_->vertexCosts(), path, message,
                              potential_, predecessors_);
}

inline bool CVPMeshPlanner::waveFrontUpdateWithS(lvr2::DenseVertexMap<float>& distances,
                                                   const lvr2::DenseEdgeMap<float>& edge_weights,
                                                   const lvr2::VertexHandle& v1, const lvr2::VertexHandle& v2,
                                                   const lvr2::VertexHandle& v3)
{
  const auto mesh = mesh_map_->mesh();

  const double u1 = distances[v1];
  const double u2 = distances[v2];
  const double u3 = distances[v3];

  const lvr2::OptionalEdgeHandle e12h = mesh->getEdgeBetween(v1, v2);
  const double c = edge_weights[e12h.unwrap()];
  const double c_sq = c * c;

  const lvr2::OptionalEdgeHandle e13h = mesh->getEdgeBetween(v1, v3);
  const double b = edge_weights[e13h.unwrap()];
  const double b_sq = b * b;

  const lvr2::OptionalEdgeHandle e23h = mesh->getEdgeBetween(v2, v3);
  const double a = edge_weights[e23h.unwrap()];
  const double a_sq = a * a;

  const double u1_sq = u1 * u1;
  const double u2_sq = u2 * u2;

  const double A = sqrt(std::max<double>((-u1 + u2 + c) * (u1 - u2 + c) * (u1 + u2 - c) * (u1 + u2 + c), 0));
  const double B = sqrt(std::max<double>((-a + b + c) * (a - b + c) * (a + b - c) * (a + b + c), 0));
  const double sx = (c_sq + u1_sq - u2_sq) / (2 * c);
  const double sy = -A / (2 * c);
  const double p = (-a_sq + b_sq + c_sq) / (2 * c);
  const double hc = B / (2 * c);
  const double dy = hc - sy;
  // const double dy = (A + B) / (2 * c);
  // const double dx = (u2_sq - u1_sq + b_sq - a_sq) / (2*c);
  const double dx = p - sx;

  const double u3tmp_sq = dx * dx + dy * dy;
  double u3tmp = sqrt(u3tmp_sq);

  if (u3tmp < u3)
  {
    if (distances[v1] < distances[v2])
    {
      const double S = sy * p - sx * hc;
      const double t1cos = (u3tmp_sq + b_sq - u1_sq) / (2 * u3tmp * b);
      if (S <= 0 && std::fabs(t1cos) <= 1)
      {
        const double theta = acos(t1cos);
        predecessors_[v3] = v1;
        direction_[v3] = static_cast<float>(theta);
        distances[v3] = static_cast<float>(u3tmp);
        const lvr2::FaceHandle fh = mesh->getFaceBetween(v1, v2, v3).unwrap();
        cutting_faces_.insert(v3, fh);
#ifdef DEBUG
        mesh_map->publishDebugVector(v3, v1, fh, theta, mesh_map::color(0.9, 0.9, 0.2),
                                     "dir_vec" + std::to_string(v3.idx()));
#endif
        return true;
      }
      else
      {
        u3tmp = u1 + b;
        if (u3tmp < u3)
        {
          predecessors_[v3] = v1;
          direction_[v3] = 0;
          distances[v3] = u3tmp;
          const lvr2::FaceHandle fh = mesh->getFaceBetween(v1, v2, v3).unwrap();
          cutting_faces_.insert(v3, fh);
#ifdef DEBUG
          mesh_map->publishDebugVector(v3, v1, fh, 0, mesh_map::color(0.9, 0.9, 0.2),
                                       "dir_vec" + std::to_string(v3.idx()));
#endif
          return true;
        }
        return false;
      }
    }
    else
    {
      const double S = sx * hc - hc * c + sy * c - sy * p;
      const double t2cos = (a_sq + u3tmp_sq - u2_sq) / (2 * a * u3tmp);
      if (S <= 0 && std::fabs(t2cos) <= 1)
      {
        const lvr2::FaceHandle fh = mesh->getFaceBetween(v1, v2, v3).unwrap();
        const double theta = -acos(t2cos);
        direction_[v3] = static_cast<float>(theta);
        distances[v3] = static_cast<float>(u3tmp);
        predecessors_[v3] = v2;
        cutting_faces_.insert(v3, fh);
#ifdef DEBUG
        mesh_map->publishDebugVector(v3, v2, fh, theta, mesh_map::color(0.9, 0.9, 0.2),
                                     "dir_vec" + std::to_string(v3.idx()));
#endif
        return true;
      }
      else
      {
        u3tmp = u2 + a;
        if (u3tmp < u3)
        {
          direction_[v3] = 0;
          distances[v3] = u3tmp;
          predecessors_[v3] = v2;
          const lvr2::FaceHandle fh = mesh->getFaceBetween(v1, v2, v3).unwrap();
          cutting_faces_.insert(v3, fh);
#ifdef DEBUG
          mesh_map->publishDebugVector(v3, v2, fh, 0, mesh_map::color(0.9, 0.9, 0.2),
                                       "dir_vec" + std::to_string(v3.idx()));
#endif
          return true;
        }
        return false;
      }
    }
  }
  return false;
}

inline bool CVPMeshPlanner::waveFrontUpdate(lvr2::DenseVertexMap<float>& distances,
                                              const lvr2::DenseEdgeMap<float>& edge_weights,
                                              const lvr2::VertexHandle& v1, const lvr2::VertexHandle& v2,
                                              const lvr2::VertexHandle& v3)
{
  const auto mesh = mesh_map_->mesh();

  const double u1 = distances[v1];
  const double u2 = distances[v2];
  const double u3 = distances[v3];

  const lvr2::OptionalEdgeHandle e12h = mesh->getEdgeBetween(v1, v2);
  const double c = edge_weights[e12h.unwrap()];
  const double c_sq = c * c;

  const lvr2::OptionalEdgeHandle e13h = mesh->getEdgeBetween(v1, v3);
  const double b = edge_weights[e13h.unwrap()];
  const double b_sq = b * b;

  const lvr2::OptionalEdgeHandle e23h = mesh->getEdgeBetween(v2, v3);
  const double a = edge_weights[e23h.unwrap()];
  const double a_sq = a * a;

  const double u1_sq = u1 * u1;
  const double u2_sq = u2 * u2;

  const double sx = (c_sq + u1_sq - u2_sq) / (2 * c);
  const double sy = -sqrt(std::max(u1_sq - sx*sx, 0.0));

  const double p = (b_sq + c_sq -a_sq) / (2 * c);
  const double hc = sqrt(std::max(b_sq - p*p, 0.0));

  const double dy = hc - sy;
  const double dx = p - sx;

  const double u3tmp_sq = dx * dx + dy * dy;
  double u3tmp = sqrt(u3tmp_sq);

  if (!std::isfinite(u3tmp))
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "u3 tmp is not finite!");
  }
  if (u3tmp < u3)
  {
    const double t0a = (a_sq + b_sq - c_sq) / (2 * a * b);
    const double t1a = (u3tmp_sq + b_sq - u1_sq) / (2 * u3tmp * b);
    const double t2a = (a_sq + u3tmp_sq - u2_sq) / (2 * a * u3tmp);

    // corner case: side b + u1 ~= u3
    if (std::fabs(t1a) > 1)
    {
      u3tmp = u1 + b;
      if (u3tmp < u3)
      {
        auto fH = mesh->getFaceBetween(v1, v2, v3).unwrap();
        cutting_faces_.insert(v3, fH);
        predecessors_[v3] = v1;
#ifdef DEBUG
        mesh_map->publishDebugVector(v3, v1, fH, 0, mesh_map::color(0.9, 0.9, 0.2),
                                     "dir_vec" + std::to_string(v3.idx()));
#endif
        distances[v3] = static_cast<float>(u3tmp);
        direction_[v3] = 0;
        return true;
      }
      return false;
    }
    // corner case: side a + u2 ~= u3
    else if (std::fabs(t2a) > 1)
    {
      u3tmp = u2 + a;
      if (u3tmp < u3)
      {
        auto fH = mesh->getFaceBetween(v1, v2, v3).unwrap();
        cutting_faces_.insert(v3, fH);
        predecessors_[v3] = v2;
#ifdef DEBUG
        mesh_map->publishDebugVector(v3, v2, fH, 0, mesh_map::color(0.9, 0.9, 0.2),
                                     "dir_vec" + std::to_string(v3.idx()));
#endif
        distances[v3] = static_cast<float>(u3tmp);
        direction_[v3] = 0;
        return true;
      }
      return false;
    }

    const double theta0 = acos(t0a);
    const double theta1 = acos(t1a);
    const double theta2 = acos(t2a);

#ifdef DEBUG
    if (!std::isfinite(theta0 + theta1 + theta2))
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "------------------");
      if (std::isnan(theta0))
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Theta0 is NaN!");
      if (std::isnan(theta1))
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Theta1 is NaN!");
      if (std::isnan(theta2))
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Theta2 is NaN!");
      if (std::isinf(theta2))
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Theta2 is inf!");
      if (std::isinf(theta2))
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Theta2 is inf!");
      if (std::isinf(theta2))
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Theta2 is inf!");
      if (std::isnan(t1a))
        RCLCPP_ERROR_STREAM(node_->get_logger(), "t1a is NaN!");
      if (std::isnan(t2a))
        RCLCPP_ERROR_STREAM(node_->get_logger(), "t2a is NaN!");
      if (std::fabs(t2a) > 1)
      {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "|t2a| is > 1: " << t2a);
        RCLCPP_INFO_STREAM(node_->get_logger(), "a: " << a << ", u3: " << u3tmp << ", u2: " << u2 << ", a+u2: " << a + u2);
      }
      if (std::fabs(t1a) > 1)
      {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "|t1a| is > 1: " << t1a);
        RCLCPP_INFO_STREAM(node_->get_logger(), "b: " << b << ", u3: " << u3tmp << ", u1: " << u1 << ", b+u1: " << b + u1);
      }
    }
#endif

    if (theta1 < theta0 && theta2 < theta0)
    {
      auto fH = mesh->getFaceBetween(v1, v2, v3).unwrap();
      cutting_faces_.insert(v3, fH);
      distances[v3] = static_cast<float>(u3tmp);
      if (theta1 < theta2)
      {
        predecessors_[v3] = v1;
        direction_[v3] = theta1;
#ifdef DEBUG
        mesh_map->publishDebugVector(v3, v1, fH, theta1, mesh_map::color(0.9, 0.9, 0.2),
                                     "dir_vec" + std::to_string(v3.idx()));
#endif
      }
      else
      {
        predecessors_[v3] = v2;
        direction_[v3] = -theta2;
#ifdef DEBUG
        mesh_map->publishDebugVector(v3, v2, fH, -theta2, mesh_map::color(0.9, 0.9, 0.2),
                                     "dir_vec" + std::to_string(v3.idx()));
#endif
      }
      return true;
    }
    else if (theta1 < theta2)
    {
      u3tmp = u1 + b;
      if (u3tmp < u3)
      {
        auto fH = mesh->getFaceBetween(v1, v2, v3).unwrap();
        cutting_faces_.insert(v3, fH);
        predecessors_[v3] = v1;
        distances[v3] = static_cast<float>(u3tmp);
#ifdef DEBUG
        mesh_map->publishDebugVector(v3, v1, fH, 0, mesh_map::color(0.9, 0.9, 0.2),
                                     "dir_vec" + std::to_string(v3.idx()));
#endif
        direction_[v3] = 0;
        return true;
      }
      return false;
    }
    else
    {
      u3tmp = u2 + a;
      if (u3tmp < u3)
      {
        auto fH = mesh->getFaceBetween(v1, v2, v3).unwrap();
        cutting_faces_.insert(v3, fH);
        predecessors_[v3] = v2;
        distances[v3] = static_cast<float>(u3tmp);
#ifdef DEBUG
        mesh_map->publishDebugVector(v3, v2, fH, 0, mesh_map::color(0.9, 0.9, 0.2),
                                     "dir_vec" + std::to_string(v3.idx()));
#endif
        direction_[v3] = 0;
        return true;
      }
      return false;
    }
  }
  return false;
}


inline bool CVPMeshPlanner::waveFrontUpdateFMM(
    lvr2::DenseVertexMap<float> &distances,
    const lvr2::DenseEdgeMap<float> &edge_weights,
    const lvr2::VertexHandle &v1tmp,
    const lvr2::VertexHandle &v2tmp,
    const lvr2::VertexHandle &v3)
{
  const auto mesh = mesh_map_->mesh();

  bool v1_smaller = distances[v1tmp] < distances[v2tmp];
  const lvr2::VertexHandle v1 = v1_smaller ? v1tmp : v2tmp;
  const lvr2::VertexHandle v2 = v1_smaller ? v2tmp : v1tmp;

  const double u1 = distances[v1];
  const double u2 = distances[v2];
  const double u3 = distances[v3];

  const lvr2::OptionalEdgeHandle e12h = mesh->getEdgeBetween(v1, v2);
  const double c = edge_weights[e12h.unwrap()];
  const double c_sq = c * c;

  const lvr2::OptionalEdgeHandle e13h = mesh->getEdgeBetween(v1, v3);
  const double b = edge_weights[e13h.unwrap()];
  const double b_sq = b * b;

  const lvr2::OptionalEdgeHandle e23h = mesh->getEdgeBetween(v2, v3);
  const double a = edge_weights[e23h.unwrap()];
  const double a_sq = a * a;

  const double delta_u = u2 - u1;
  const double cos_theta = (a_sq + b_sq - c_sq) / (2 * a * b);

  const double k0 = a_sq + b_sq - 2 * a * b * cos_theta;
  const double k1 = 2 * b * delta_u * (a * cos_theta - b);
  const double k2 = b_sq * (delta_u * delta_u - a_sq * (1 - cos_theta * cos_theta));

  double t;
  const double r = k1 * k1 - 4 * k0 * k2;
  if(r < 0)
  {
    t = -k1 / (2 * k0);
  }
  else
  {
    t = (-k1 + sqrt(r)) / (2 * k0);
  }

  const double e = b * (t - delta_u) / t;

  if(delta_u < t && (e < a / cos_theta) && (e > a * cos_theta))
  {
    const double u3_tmp = u1 + t;
    if(u3_tmp < u3)
    {
      auto fH = mesh->getFaceBetween(v1, v2, v3).unwrap();
      cutting_faces_.insert(v3, fH);
      predecessors_[v3] = v1;
      distances[v3] = static_cast<float>(u3_tmp);
      const double theta = acos(cos_theta);
      const double phi = asin((e * sin(theta)) / sqrt(a_sq * e * e - 2 * a * cos_theta));
      direction_[v3] = theta + phi - M_PI_2;
      return true;
    }
  }
  else {
    const double u1t = u1 + b;
    const double u2t = u2 + a;

    if (u1t < u2t) {
      if (u1t < u3) {
        auto fH = mesh->getFaceBetween(v1, v2, v3).unwrap();
        cutting_faces_.insert(v3, fH);
        predecessors_[v3] = v1;
        distances[v3] = static_cast<float>(u1t);
        direction_[v3] = 0;
        return true;
      }
    } else {
      if (u2t < u3) {
        auto fH = mesh->getFaceBetween(v1, v2, v3).unwrap();
        cutting_faces_.insert(v3, fH);
        predecessors_[v3] = v2;
        distances[v3] = static_cast<float>(u2t);
        direction_[v3] = 0;
        return true;
      }
    }
  }

  return false;
}

uint32_t CVPMeshPlanner::waveFrontPropagation(const mesh_map::Vector& original_start,
                                                const mesh_map::Vector& original_goal,
                                                const lvr2::DenseEdgeMap<float>& edge_weights,
                                                const lvr2::DenseVertexMap<float>& costs,
                                                std::list<std::pair<mesh_map::Vector, lvr2::FaceHandle>>& path,
                                                std::string& message,
                                                lvr2::DenseVertexMap<float>& distances,
                                                lvr2::DenseVertexMap<lvr2::VertexHandle>& predecessors)
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Init wave front propagation.");

  const auto mesh = mesh_map_->mesh();
  const auto& vertex_costs = mesh_map_->vertexCosts();
  auto& invalid = mesh_map_->invalid;

  mesh_map_->publishDebugPoint(original_start, mesh_map::color(0, 1, 0), "start_point");
  mesh_map_->publishDebugPoint(original_goal, mesh_map::color(0, 0, 1), "goal_point");

  mesh_map::Vector start = original_start;
  mesh_map::Vector goal = original_goal;

  // Find the containing faces of start and goal
  const lvr2::OptionalFaceHandle start_opt = mesh_map_->getContainingFace(start, 0.4);
  const lvr2::OptionalFaceHandle goal_opt = mesh_map_->getContainingFace(goal, 0.4);

  const auto t_initialization_start = std::chrono::steady_clock::now();

  // reset cancel planning
  cancel_planning_ = false;

  if (!start_opt) {
    message = "Could not find a face close enough to the given start pose";
    RCLCPP_WARN_STREAM(node_->get_logger(), "waveFrontPropagation(): " << message);
    return mbf_msgs::action::GetPath::Result::INVALID_START;
  }
  if (!goal_opt) {
    message = "Could not find a face close enough to the given goal pose";
    RCLCPP_WARN_STREAM(node_->get_logger(), "waveFrontPropagation(): " << message);
    return mbf_msgs::action::GetPath::Result::INVALID_GOAL;
  }

  const auto& start_face = start_opt.unwrap();
  const auto& goal_face = goal_opt.unwrap();

  mesh_map_->publishDebugFace(start_face, mesh_map::color(0, 0, 1), "start_face");
  mesh_map_->publishDebugFace(goal_face, mesh_map::color(0, 1, 0), "goal_face");

  path.clear();
  distances.clear();
  predecessors.clear();

  lvr2::DenseVertexMap<bool> fixed(mesh->nextVertexIndex(), false);

  // clear vector field map
  vector_map_.clear();

  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Init distances.");
  // initialize distances with infinity
  // initialize predecessor of each vertex with itself
  for (auto const& vH : mesh->vertices())
  {
    distances.insert(vH, std::numeric_limits<float>::infinity());
    predecessors.insert(vH, vH);
  }

  lvr2::Meap<lvr2::VertexHandle, float> pq;
  // Set start distance to zero
  // add start vertex to priority queue
  for (auto vH : mesh->getVerticesOfFace(start_face))
  {
    const mesh_map::Vector diff = start - mesh->getVertexPosition(vH);
    const float dist = diff.length();
    distances[vH] = dist;
    vector_map_.insert(vH, diff);
    cutting_faces_.insert(vH, start_face);
    fixed[vH] = true;
    pq.insert(vH, dist);
  }

  std::array<lvr2::VertexHandle, 3> goal_vertices = mesh->getVerticesOfFace(goal_face);
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "The goal is at (" << goal.x << ", " << goal.y << ", " << goal.z << ") at the face ("
                                     << goal_vertices[0] << ", " << goal_vertices[1] << ", " << goal_vertices[2]
                                     << ")");
  mesh_map_->publishDebugPoint(mesh->getVertexPosition(goal_vertices[0]), mesh_map::color(0, 0, 1), "goal_face_v1");
  mesh_map_->publishDebugPoint(mesh->getVertexPosition(goal_vertices[1]), mesh_map::color(0, 0, 1), "goal_face_v2");
  mesh_map_->publishDebugPoint(mesh->getVertexPosition(goal_vertices[2]), mesh_map::color(0, 0, 1), "goal_face_v3");

  float goal_dist = std::numeric_limits<float>::infinity();

  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Start wavefront propagation...");

  size_t fixed_cnt = 0;
  size_t fixed_set_cnt = 0;
  const auto t_wavefront_start = std::chrono::steady_clock::now();
  const auto initialization_duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_wavefront_start - t_initialization_start);

  while (!pq.isEmpty() && !cancel_planning_)
  {
    lvr2::VertexHandle current_vh = pq.popMin().key();

    fixed[current_vh] = true;
    fixed_set_cnt++;

    if (distances[current_vh] > goal_dist)
      continue;

    if (vertex_costs[current_vh] >= config_.cost_limit)
      continue;

    if (invalid[current_vh])
      continue;

    if (current_vh == goal_vertices[0] || current_vh == goal_vertices[1] || current_vh == goal_vertices[2])
    {
      if (goal_dist == std::numeric_limits<float>::infinity() && fixed[goal_vertices[0]] && fixed[goal_vertices[1]] &&
          fixed[goal_vertices[2]])
      {
        RCLCPP_DEBUG_STREAM(node_->get_logger(), "Wave front reached the goal!");
        goal_dist = distances[current_vh] + config_.goal_dist_offset;
      }
    }

    try
    {
      std::vector<lvr2::FaceHandle> faces;
      mesh->getFacesOfVertex(current_vh, faces);

      for (auto fh : faces)
      {
        const auto vertices = mesh->getVerticesOfFace(fh);
        const lvr2::VertexHandle& a = vertices[0];
        const lvr2::VertexHandle& b = vertices[1];
        const lvr2::VertexHandle& c = vertices[2];

        if (invalid[a] || invalid[b] || invalid[c])
          continue;

        // We are looking for a face where exactly
        // one vertex is not in the fixed set
        if (fixed[a] && fixed[b] && fixed[c])
        {
          // All distance values of the face are already fixed
#ifdef DEBUG
          mesh_map->publishDebugFace(fh, mesh_map::color(1, 0, 0), "fmm_fixed_" + std::to_string(fixed_cnt++));
#endif
          continue;
        }
        else if (fixed[a] && fixed[b] && !fixed[c])
        {
          // c is free
          // Skip vertices at or above the cost limit
          if (costs[c] >= config_.cost_limit)
          {
            continue;
          }
#ifdef USE_UPDATE_WITH_S
          if (waveFrontUpdateWithS(distances, edge_weights, a, b, c))
#elif defined USE_UPDATE_FMM
          if (waveFrontUpdateFMM(distances, edge_weights, a, b, c))
#else
          if (waveFrontUpdate(distances, edge_weights, a, b, c))
#endif
          {
            pq.insert(c, distances[c]);
#ifdef DEBUG
            mesh_map->publishDebugFace(fh, mesh_map::color(0, 1, 1), "fmm_update");
            sleep(2);
#endif
          }
        }
        else if (fixed[a] && !fixed[b] && fixed[c])
        {
          // b is free
          // Skip vertices at or above the cost limit
          if (costs[b] >= config_.cost_limit)
          {
            continue;
          }
#ifdef USE_UPDATE_WITH_S
          if (waveFrontUpdateWithS(distances, edge_weights, c, a, b))
#elif defined USE_UPDATE_FMM
          if (waveFrontUpdateFMM(distances, edge_weights, c, a, b))
#else
          if (waveFrontUpdate(distances, edge_weights, c, a, b))
#endif
          {
            pq.insert(b, distances[b]);
#ifdef DEBUG
            mesh_map->publishDebugFace(fh, mesh_map::color(0, 1, 1), "fmm_update");
            sleep(2);
#endif
          }
        }
        else if (!fixed[a] && fixed[b] && fixed[c])
        {
          // a if free
          // Skip vertices at or above the cost limit
          if (costs[a] >= config_.cost_limit)
          {
            continue;
          }
#ifdef USE_UPDATE_WITH_S
          if (waveFrontUpdateWithS(distances, edge_weights, b, c, a))
#elif defined USE_UPDATE_FMM
          if (waveFrontUpdateFMM(distances, edge_weights, b, c, a))
#else
          if (waveFrontUpdate(distances, edge_weights, b, c, a))
#endif
          {
            pq.insert(a, distances[a]);
#ifdef DEBUG
            mesh_map->publishDebugFace(fh, mesh_map::color(0, 1, 1), "fmm_update");
            sleep(2);
#endif
          }
        }
        else
        {
          // two free vertices -> skip that face
          continue;
        }
      }
    }
    catch (lvr2::PanicException exception)
    {
      invalid.insert(current_vh, true);
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Found invalid vertex!");
      continue;
    }
    catch (lvr2::VertexLoopException exception)
    {
      invalid.insert(current_vh, true);
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Found invalid vertex!");
      continue;
    }
  }

  if (cancel_planning_)
  {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Wave front propagation has been canceled!");
    return mbf_msgs::action::GetPath::Result::CANCELED;
  }
  const auto t_wavefront_end = std::chrono::steady_clock::now();
  const auto wavefront_propagation_duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_wavefront_end - t_wavefront_start);
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Finished wave front propagation.");
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Computing the vector map...");
  computeVectorMap();

  const auto t_vector_field_end = std::chrono::steady_clock::now();
  const auto vector_field_duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_vector_field_end - t_wavefront_end);

  // check if wave front propagation reached the goal
  bool predecessor_for_at_least_one_goal_vertex_exists = false;
  for (auto goal_vertex : goal_vertices)
  {
    if (goal_vertex != predecessors[goal_vertex])
    {
      predecessor_for_at_least_one_goal_vertex_exists = true;
      break;
    }
  }
  const bool path_found = !predecessor_for_at_least_one_goal_vertex_exists && goal_face != start_face;
  if (path_found)
  {
    message = "Predecessor of the goal is not set! No path found!";
    RCLCPP_WARN_STREAM(node_->get_logger(), message);
    return mbf_msgs::action::GetPath::Result::NO_PATH_FOUND;
  }

  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Start vector field back tracking!");

  lvr2::FaceHandle current_face = goal_face;
  mesh_map::Vector current_pos = goal;
  path.push_front(std::pair<mesh_map::Vector, lvr2::FaceHandle>(current_pos, current_face));

  // move from the goal position towards the start position
  while (current_pos.distance2(start) > config_.step_width && !cancel_planning_)
  {
    // move current pos ahead on the surface following the vector field,
    // updates the current face if necessary
    try
    {
      if (mesh_map_->meshAhead(current_pos, current_face, config_.step_width))
      {
        path.push_front(std::pair<mesh_map::Vector, lvr2::FaceHandle>(current_pos, current_face));
      }
      else
      {
        message = "Could not find a valid path, while back-tracking from the goal";
        RCLCPP_WARN_STREAM(node_->get_logger(), message);
        return mbf_msgs::action::GetPath::Result::NO_PATH_FOUND;
      }
    }
    catch (lvr2::PanicException exception)
    {
      message = "Could not find a valid path, while back-tracking from the goal: HalfEdgeMesh panicked!";
      RCLCPP_ERROR_STREAM(node_->get_logger(), message);
      return mbf_msgs::action::GetPath::Result::NO_PATH_FOUND;
    }
  }
  path.push_front(std::pair<mesh_map::Vector, lvr2::FaceHandle>(start, start_face));

  const auto t_path_backtracking = std::chrono::steady_clock::now();
  const auto path_backtracking_duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_path_backtracking - t_vector_field_end);

  RCLCPP_INFO_STREAM(node_->get_logger(), "Processed " << fixed_set_cnt << " vertices in the fixed set.");
  RCLCPP_INFO_STREAM(node_->get_logger(), "Initialization duration (ms): " << initialization_duration_ms.count());
  RCLCPP_INFO_STREAM(node_->get_logger(), "Execution time wavefront propagation (ms): "<< wavefront_propagation_duration_ms.count());
  RCLCPP_INFO_STREAM(node_->get_logger(), "Vector field post computation (ms): " << vector_field_duration_ms.count());
  RCLCPP_INFO_STREAM(node_->get_logger(), "Path backtracking duration (ms): " << path_backtracking_duration_ms.count());

  if (cancel_planning_)
  {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Wave front propagation has been canceled!");
    return mbf_msgs::action::GetPath::Result::CANCELED;
  }

  RCLCPP_INFO_STREAM(node_->get_logger(), "Successfully finished vector field back tracking!");
  return mbf_msgs::action::GetPath::Result::SUCCESS;
}

} /* namespace cvp_mesh_planner */

