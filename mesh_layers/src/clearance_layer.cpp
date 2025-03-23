/*
*  Copyright 2024, Justus Braun
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
 *    Justus Braun <jubraun@uos.de>
 *
 */

#include "mesh_layers/clearance_layer.h"

#include <lvr2/algorithm/GeometryAlgorithms.hpp>
#include <lvr2/algorithm/NormalAlgorithms.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mesh_layers::ClearanceLayer, mesh_map::AbstractLayer)

namespace mesh_layers
{
bool ClearanceLayer::readLayer()
{
  RCLCPP_INFO(node_->get_logger(), "Try to read '%s' from map file...", layer_name_.c_str());
  // Since this is called by the map we do not need to check if this is successful
  const auto map = map_ptr_.lock();
  auto clearance_opt = map->meshIO()->getDenseAttributeMap<lvr2::DenseVertexMap<float>>(layer_name_);

  if (clearance_opt)
  {
    RCLCPP_INFO(node_->get_logger(), "'%s' has been read successfully.", layer_name_.c_str());
    clearance_ = clearance_opt.get();

    return computeLethalsAndCosts();
  }

  return false;
}

bool ClearanceLayer::computeLethalsAndCosts()
{
  RCLCPP_INFO_STREAM(node_->get_logger(), "Compute lethals for '" << layer_name_ << "' (Clearance Layer) with robot_height " << config_.robot_height << " and height_inflation " << config_.height_inflation);
  lethal_vertices_.clear();
  costs_.clear();

  // The clearance_ map contains the actual free space above the vertices.
  // If the robot fits through a gap with the height inflation the cost is 0.
  // If the robot does not fit the cost is 1.
  // Between robot_height and robot_height + height_inflation the cost decreases.
  const double inflated_height = config_.robot_height + config_.height_inflation;
  for (auto vH : clearance_)
  {
    if (clearance_[vH] < config_.robot_height)
    {
      costs_.insert(vH, 1.0);
      lethal_vertices_.insert(vH);
    }
    else if (clearance_[vH] < inflated_height)
    {
      // Map the cost to [1, 0] using (cos(x * pi) + 1 / 2)
      const double diff = (clearance_[vH] - config_.robot_height) / config_.height_inflation;
      const double cost = (cos(diff * M_PI) + 1.0) / 2.0;
      costs_.insert(vH, cost);
    }
    else
    {
      costs_.insert(vH, 0.0);
    }
  }
  RCLCPP_INFO_STREAM(node_->get_logger(), "Found " << lethal_vertices_.size() << " lethal vertices.");
  return true;
}

bool ClearanceLayer::writeLayer()
{
  RCLCPP_INFO(node_->get_logger(), "Saving '%s' to map file...", layer_name_.c_str());
  const auto map = map_ptr_.lock();
  if (map->meshIO()->addDenseAttributeMap(clearance_, layer_name_))
  {
    RCLCPP_INFO(node_->get_logger(), "Saved '%s' to map file.", layer_name_.c_str());
    return true;
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "Could not save '%s' to map file!", layer_name_.c_str());
    return false;
  }
}

float ClearanceLayer::threshold()
{
  return config_.robot_height;
}

bool ClearanceLayer::computeLayer()
{
  RCLCPP_INFO(node_->get_logger(), "Computing clearance along vertex normals...");
  const auto map = map_ptr_.lock();

  // Load vertex normals
  using VertexNormalMap = lvr2::DenseVertexMap<mesh_map::Normal>;
  VertexNormalMap vertex_normals;
  auto vertex_normals_opt = map->meshIO()->getDenseAttributeMap<VertexNormalMap>("vertex_normals");

  if (vertex_normals_opt)
  {
    RCLCPP_INFO(node_->get_logger(), "Using vertex normals from map file");
    vertex_normals = vertex_normals_opt.value();
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(), "No vertex normals in map file!");

    // Load or calculate face normals
    using FaceNormalMap = lvr2::DenseFaceMap<mesh_map::Normal>;
    FaceNormalMap face_normals;
    auto face_normals_opt = map->meshIO()->getDenseAttributeMap<FaceNormalMap>("face_normals");
    if (face_normals_opt)
    {
      RCLCPP_INFO(node_->get_logger(), "Using face normals from map file");
      face_normals = face_normals_opt.value();
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "Calculating face normals");
      face_normals = lvr2::calcFaceNormals(*map->mesh());
    }

    RCLCPP_INFO(node_->get_logger(), "Calculating vertex normals");
    VertexNormalMap vertex_normals = lvr2::calcVertexNormals(*map->mesh(), face_normals);
  }

  // Finally calculate the clearance layer
  clearance_ = lvr2::calcNormalClearance(*map->mesh(), vertex_normals);

  return computeLethalsAndCosts();
}

lvr2::VertexMap<float>& ClearanceLayer::costs()
{
  return costs_;
}

rcl_interfaces::msg::SetParametersResult ClearanceLayer::reconfigureCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  bool needs_cost_recompute = false;
  for (auto parameter : parameters) {
    if (parameter.get_name() == mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".robot_height") {
      config_.robot_height = parameter.as_double();
      needs_cost_recompute = true;
    } else if (parameter.get_name() == mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".height_inflation") {
      config_.height_inflation = parameter.as_double();
      needs_cost_recompute = true;
    }
  }

  if (needs_cost_recompute) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Recompute costs and notify change from " << layer_name_ << " due to cfg change.");
    computeLethalsAndCosts();
    notifyChange();
  }

  return result;
}


bool ClearanceLayer::initialize()
{
  { // threshold
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Threshold for the robot to fit through the space in meters (height of the robot plus margin).";
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.05;
    range.to_value = 100.0; // What is a sane limit for robot size?
    descriptor.floating_point_range.push_back(range);
    config_.robot_height = node_->declare_parameter<double>(
      mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".robot_height",
      descriptor
    );
  }
  { // Extra clearance for the robot to avoid tight spaces
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Extra headroom for the robot in which the cost decreases from 1 to 0.";
    config_.height_inflation = node_->declare_parameter<double>(
      mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".height_inflation",
      config_.height_inflation,
      descriptor
    );
  }
  dyn_params_handler_ = node_->add_on_set_parameters_callback(std::bind(
      &ClearanceLayer::reconfigureCallback, this, std::placeholders::_1));
  return true;
}

} /* namespace mesh_layers */
