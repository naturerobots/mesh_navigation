/*
 *  Copyright 2020, Alexander Mock
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
 *    Alexander Mock <amock@uos.de>
 *
 */

#include "mesh_layers/border_layer.h"

#include <lvr2/algorithm/GeometryAlgorithms.hpp>
#include <lvr2/algorithm/NormalAlgorithms.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mesh_layers::BorderLayer, mesh_map::AbstractLayer)

namespace mesh_layers
{
bool BorderLayer::readLayer()
{
  RCLCPP_INFO_STREAM(node_->get_logger(), "Try to read border costs from map file...");
  auto border_costs_opt = mesh_io_ptr_->getDenseAttributeMap<lvr2::DenseVertexMap<float> >("border");

  if (border_costs_opt)
  {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Border costs have been read successfully.");
    border_costs_ = border_costs_opt.get();
    return computeLethals();
  }

  return false;
}

bool BorderLayer::computeLethals()
{
  RCLCPP_INFO_STREAM(node_->get_logger(), "Compute lethals for \"" << layer_name_ << "\" (Border Layer) with threshold "
                                           << config_.threshold);
  lethal_vertices_.clear();
  for (auto vH : border_costs_)
  {
    if (border_costs_[vH] > config_.threshold)
    {
      lethal_vertices_.insert(vH);
    }
  }
  RCLCPP_INFO_STREAM(node_->get_logger(), "Found " << lethal_vertices_.size() << " lethal vertices.");
  return true;
}

bool BorderLayer::writeLayer()
{
  RCLCPP_INFO_STREAM(node_->get_logger(), "Saving border costs to map file...");
  if (mesh_io_ptr_->addDenseAttributeMap(border_costs_, "border"))
  {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Saved border costs to map file.");
    return true;
  }
  else
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Could not save height differences to map file!");
    return false;
  }
}

float BorderLayer::threshold()
{
  return config_.threshold;
}

bool BorderLayer::computeLayer()
{
  border_costs_ = lvr2::calcBorderCosts(*mesh_ptr_, config_.border_cost);
  return computeLethals();
}

lvr2::VertexMap<float>& BorderLayer::costs()
{
  return border_costs_;
}

rcl_interfaces::msg::SetParametersResult BorderLayer::reconfigureCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  // bool has_threshold_changed = false;
  bool recompute_costs = false;
  bool recompute_lethals = false;

  for (auto parameter : parameters)
  {
    if (parameter.get_name() == mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".threshold") {
      config_.threshold = parameter.as_double();
      recompute_lethals = true;
    } else if (parameter.get_name() == mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".border_cost") {
      config_.border_cost = parameter.as_double();
      recompute_costs = true;
      recompute_lethals = true;
    } else if (parameter.get_name() == mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".factor") {
      config_.factor = parameter.as_double();
    }
  }

  if(recompute_costs) 
  {
    RCLCPP_INFO_STREAM(node_->get_logger(), "'" << layer_name_ << "': Recompute layer costs due to cfg change.");
    computeLayer();
  }

  if (recompute_lethals) 
  {
    RCLCPP_INFO_STREAM(node_->get_logger(), "'" << layer_name_ << "': Recompute lethals due to cfg change.");
    computeLethals();
  }

  if(recompute_costs || recompute_lethals)
  {
    RCLCPP_INFO_STREAM(node_->get_logger(), "'" << layer_name_ << "': Notify changes.");
    notifyChange();
  }

  return result;
}


bool BorderLayer::initialize()
{
  { // threshold
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Threshold for the local border costs to be counted as lethal.";
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.05;
    range.to_value = 1.0;
    descriptor.floating_point_range.push_back(range);
    config_.threshold = node_->declare_parameter(mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".threshold", config_.threshold, descriptor);
  }
  { // border cost
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "The cost value used for the border vertices.";
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.02;
    range.to_value = 10.0;
    descriptor.floating_point_range.push_back(range);
    config_.border_cost = node_->declare_parameter(mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".border_cost", config_.border_cost, descriptor);
  }
  { // factor
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Using this factor to weight this layer.";
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.0;
    range.to_value = 1.0;
    descriptor.floating_point_range.push_back(range);
    config_.factor = node_->declare_parameter(mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".factor", config_.factor, descriptor);
  }
  dyn_params_handler_ = node_->add_on_set_parameters_callback(std::bind(
      &BorderLayer::reconfigureCallback, this, std::placeholders::_1));
  return true;
}

} /* namespace mesh_layers */
