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

#include "mesh_layers/freespace_layer.h"

#include <lvr2/algorithm/GeometryAlgorithms.hpp>
#include <lvr2/algorithm/NormalAlgorithms.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mesh_layers::FreespaceLayer, mesh_map::AbstractLayer)

namespace mesh_layers
{
bool FreespaceLayer::readLayer()
{
  RCLCPP_INFO(node_->get_logger(), "Try to read '%s' from map file...", layer_name_.c_str());
  auto freespace_opt = map_ptr_->meshIO()->getDenseAttributeMap<lvr2::DenseVertexMap<float>>(layer_name_);

  if (freespace_opt)
  {
    RCLCPP_INFO(node_->get_logger(), "'%s' has been read successfully.", layer_name_.c_str());
    freespace_ = freespace_opt.get();

    return computeLethalsAndCosts();
  }

  return false;
}

bool FreespaceLayer::computeLethalsAndCosts()
{
  RCLCPP_INFO_STREAM(node_->get_logger(), "Compute lethals for '" << layer_name_ << "' (Freespace Layer) with threshold " << config_.required_freespace);
  lethal_vertices_.clear();
  costs_.clear();

  // The freespace_ map contains the actual free space above the vertices.
  // If the robot fits through a gap the cost is 0 and if not its 1
  for (auto vH : freespace_)
  {
    if (freespace_[vH] < config_.required_freespace)
    {
      costs_.insert(vH, 1.0);
      lethal_vertices_.insert(vH);
    }
    else
    {
      costs_.insert(vH, 0.0);
    }
  }
  RCLCPP_INFO_STREAM(node_->get_logger(), "Found " << lethal_vertices_.size() << " lethal vertices.");
  return true;
}

bool FreespaceLayer::writeLayer()
{
  RCLCPP_INFO(node_->get_logger(), "Saving '%s' to map file...", layer_name_.c_str());
  if (map_ptr_->meshIO()->addDenseAttributeMap(freespace_, layer_name_))
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

float FreespaceLayer::threshold()
{
  return config_.required_freespace;
}

bool FreespaceLayer::computeLayer()
{
  RCLCPP_INFO(node_->get_logger(), "Computing freespace along vertex normals...");

  // Load vertex normals
  using VertexNormalMap = lvr2::DenseVertexMap<mesh_map::Normal>;
  VertexNormalMap vertex_normals;
  auto vertex_normals_opt = map_ptr_->meshIO()->getDenseAttributeMap<VertexNormalMap>("vertex_normals");

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
    auto face_normals_opt = map_ptr_->meshIO()->getDenseAttributeMap<FaceNormalMap>("face_normals");
    if (face_normals_opt)
    {
      RCLCPP_INFO(node_->get_logger(), "Using face normals from map file");
      face_normals = face_normals_opt.value();
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "Calculating face normals");
      face_normals = lvr2::calcFaceNormals(*map_ptr_->mesh());
    }

    RCLCPP_INFO(node_->get_logger(), "Calculating vertex normals");
    VertexNormalMap vertex_normals = lvr2::calcVertexNormals(*map_ptr_->mesh(), face_normals);
  }

  // Finally calculate the freespace layer
  freespace_ = lvr2::calcNormalClearance(*map_ptr_->mesh(), vertex_normals);

  return computeLethalsAndCosts();
}

lvr2::VertexMap<float>& FreespaceLayer::costs()
{
  return costs_;
}

rcl_interfaces::msg::SetParametersResult FreespaceLayer::reconfigureCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  bool has_threshold_changed = false;
  for (auto parameter : parameters) {
    if (parameter.get_name() == mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".threshold") {
      config_.required_freespace = parameter.as_double();
      has_threshold_changed = true;
    } else if (parameter.get_name() == mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".factor") {
      config_.factor = parameter.as_double();
    }
  }

  if (has_threshold_changed) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Recompute lethals and notify change from " << layer_name_ << " due to cfg change.");
    computeLethalsAndCosts();
    notifyChange();
  }

  return result;
}


bool FreespaceLayer::initialize()
{
  { // threshold
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Threshold for the robot to fit through the space in meters (height of the robot plus margin).";
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.05;
    range.to_value = 100.0; // What is a sane limit for robot size?
    descriptor.floating_point_range.push_back(range);
    config_.required_freespace = node_->declare_parameter(mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".minimum_required_freespace", config_.required_freespace);
  }
  { // factor
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Using this factor to weight this layer.";
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.0;
    range.to_value = 1.0;
    descriptor.floating_point_range.push_back(range);
    config_.factor = node_->declare_parameter(mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".factor", config_.factor);
  }
  dyn_params_handler_ = node_->add_on_set_parameters_callback(std::bind(
      &FreespaceLayer::reconfigureCallback, this, std::placeholders::_1));
  return true;
}

} /* namespace mesh_layers */
