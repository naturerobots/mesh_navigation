/*
 *  Copyright 2025, Alexander Mock
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

#include <mesh_map/abstract_layer.h>

namespace mesh_map
{

rcl_interfaces::msg::SetParametersResult AbstractLayer::reconfigureCallback(
  const std::vector<rclcpp::Parameter>& parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  bool layer_changed = false;
  for (auto parameter : parameters) {
    if (parameter.get_name() == mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".combination_weight") {
      combination_weight_ = parameter.as_double();
      layer_changed = true;
    }
  }

  if(layer_changed)
  {
    notifyChange();
  }

  return result;
}

void AbstractLayer::declare_parameters()
{
  { // combination factor
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Weight used to combine this layer with others.";
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.0;
    range.to_value = 1.0;
    descriptor.floating_point_range.push_back(range);
    combination_weight_ = node_->declare_parameter(mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".combination_weight", combination_weight_, descriptor);
  }
}

bool AbstractLayer::initialize(
  const std::string& name,
  const notify_func notify_update,
  std::shared_ptr<mesh_map::MeshMap> map,
  const rclcpp::Node::SharedPtr node)
{
  layer_name_ = name;
  node_ = node;
  layer_namespace_ = "mesh_map/" + name;
  notify_ = notify_update;
  map_ptr_ = map;
  logger_ = node->get_logger().get_child(layer_name_);

  declare_parameters();
  dyn_params_handler_ = node_->add_on_set_parameters_callback(std::bind(&AbstractLayer::reconfigureCallback, this, std::placeholders::_1));

  // this calls the initialize function of the derived class
  return initialize();
}

} /* namespace mesh_map */
