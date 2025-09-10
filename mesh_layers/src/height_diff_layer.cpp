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

#include "mesh_layers/height_diff_layer.h"
#include <mesh_map/mesh_map.h>

#include <lvr2/algorithm/GeometryAlgorithms.hpp>
#include <lvr2/algorithm/NormalAlgorithms.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mesh_layers::HeightDiffLayer, mesh_map::AbstractLayer)

namespace mesh_layers
{
bool HeightDiffLayer::readLayer()
{
  RCLCPP_INFO_STREAM(get_logger(), "Try to read height differences from map file...");
  auto map = map_ptr_.lock();
  auto mesh_io = map->meshIO();
  auto height_diff_opt = mesh_io->getDenseAttributeMap<lvr2::DenseVertexMap<float>>(layer_name_);

  if (height_diff_opt)
  {
    RCLCPP_INFO_STREAM(get_logger(), "Height differences have been read successfully.");
    height_diff_ = height_diff_opt.get();

    return computeLethals();
  }

  return false;
}

bool HeightDiffLayer::computeLethals()
{
  RCLCPP_INFO_STREAM(get_logger(), "Compute lethals for \"" << layer_name_ << "\" (Height Differences Layer) with threshold "
                                           << config_.threshold);
  lethal_vertices_.clear();
  for (auto vH : height_diff_)
  {
    if (height_diff_[vH] > config_.threshold)
      lethal_vertices_.insert(vH);
  }
  RCLCPP_INFO_STREAM(get_logger(), "Found " << lethal_vertices_.size() << " lethal vertices.");
  return true;
}

bool HeightDiffLayer::writeLayer()
{
  RCLCPP_INFO_STREAM(get_logger(), "Saving height_differences to map file...");
  auto map = map_ptr_.lock();
  auto mesh_io = map->meshIO();
  if (mesh_io->addDenseAttributeMap(height_diff_, layer_name_))
  {
    RCLCPP_INFO_STREAM(get_logger(), "Saved height differences to map file.");
    return true;
  }
  else
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Could not save height differences to map file!");
    return false;
  }
}

float HeightDiffLayer::threshold()
{
  return config_.threshold;
}

bool HeightDiffLayer::computeLayer()
{
  auto map = map_ptr_.lock();
  auto mesh = map->mesh();
  height_diff_ = lvr2::calcVertexHeightDifferences(*mesh, config_.radius);
  return computeLethals();
}

const lvr2::VertexMap<float>& HeightDiffLayer::costs()
{
  return height_diff_;
}

rcl_interfaces::msg::SetParametersResult HeightDiffLayer::reconfigureCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  bool recompute_lethals = false;
  for (auto parameter : parameters) {
    if (parameter.get_name() == mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".threshold") {
      config_.threshold = parameter.as_double();
      recompute_lethals = true;
    } else if (parameter.get_name() == mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".radius") {
      config_.radius = parameter.as_double();
      recompute_lethals = true;
    }
  }

  if (recompute_lethals) {
    RCLCPP_INFO_STREAM(get_logger(), "Recompute lethals and notify change from " << layer_name_ << " due to cfg change.");
    computeLethals();
    notifyChange();
  }

  return result;
}


bool HeightDiffLayer::initialize()
{
  { // threshold
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Threshold for the local height difference to be counted as lethal.";
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.05;
    range.to_value = 1.0;
    descriptor.floating_point_range.push_back(range);
    config_.threshold = node_->declare_parameter(mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".threshold", config_.threshold, descriptor);
  }
  { // radius
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "The radius used for calculating the local height difference.";
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.02;
    range.to_value = 1.0;
    descriptor.floating_point_range.push_back(range);
    config_.radius = node_->declare_parameter(mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".radius", config_.radius, descriptor);
  }
  dyn_params_handler_ = node_->add_on_set_parameters_callback(std::bind(
      &HeightDiffLayer::reconfigureCallback, this, std::placeholders::_1));
  return true;
}

} /* namespace mesh_layers */
