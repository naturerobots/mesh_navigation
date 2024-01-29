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

#include "mesh_layers/roughness_layer.h"

#include <lvr2/algorithm/GeometryAlgorithms.hpp>
#include <lvr2/algorithm/NormalAlgorithms.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mesh_layers::RoughnessLayer, mesh_map::AbstractLayer)

namespace mesh_layers {

bool RoughnessLayer::readLayer() {
  RCLCPP_INFO_STREAM(node_->get_logger(), "Try to read roughness from map file...");
  auto roughness_opt =
      mesh_io_ptr->getDenseAttributeMap<lvr2::DenseVertexMap<float>>(
          "roughness");
  if (roughness_opt) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Successfully read roughness from map file.");
    roughness_ = roughness_opt.get();
    return computeLethals();
  }

  return false;
}

bool RoughnessLayer::writeLayer() {
  if (mesh_io_ptr->addDenseAttributeMap(roughness_, "roughness")) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Saved roughness to map file.");
    return true;
  } else {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Could not save roughness to map file!");
    return false;
  }
}

bool RoughnessLayer::computeLethals()
{
  RCLCPP_INFO_STREAM(node_->get_logger(), "Compute lethals for \"" << layer_name << "\" (Roughness Layer) with threshold " << config_.threshold);
  lethal_vertices_.clear();
  for (auto vH : roughness_) {
    if (roughness_[vH] > config_.threshold)
      lethal_vertices_.insert(vH);
  }
  RCLCPP_INFO_STREAM(node_->get_logger(), "Found " << lethal_vertices_.size() << " lethal vertices.");
  return true;
}

float RoughnessLayer::threshold() { return config_.threshold; }

bool RoughnessLayer::computeLayer() {
  RCLCPP_INFO_STREAM(node_->get_logger(), "Computing roughness...");

  lvr2::DenseFaceMap<mesh_map::Normal> face_normals;

  auto face_normals_opt =
      mesh_io_ptr->getDenseAttributeMap<lvr2::DenseFaceMap<mesh_map::Normal>>(
          "face_normals");

  if (face_normals_opt) {
    face_normals = face_normals_opt.get();
    RCLCPP_INFO_STREAM(node_->get_logger(), "Found " << face_normals.numValues()
                             << " face normals in map file.");
  } else {
    RCLCPP_INFO_STREAM(node_->get_logger(), 
        "No face normals found in the given map file, computing them...");
    face_normals = lvr2::calcFaceNormals(*mesh_ptr);
    RCLCPP_INFO_STREAM(node_->get_logger(), "Computed " << face_normals.numValues()
                                << " face normals.");
    if (mesh_io_ptr->addDenseAttributeMap(face_normals, "face_normals")) {
      RCLCPP_INFO_STREAM(node_->get_logger(), "Saved face normals to map file.");
    } else {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Could not save face normals to map file!");
      return false;
    }
  }

  lvr2::DenseVertexMap<mesh_map::Normal> vertex_normals;
  auto vertex_normals_opt =
      mesh_io_ptr->getDenseAttributeMap<lvr2::DenseVertexMap<mesh_map::Normal>>(
          "vertex_normals");

  if (vertex_normals_opt) {
    vertex_normals = vertex_normals_opt.get();
    RCLCPP_INFO_STREAM(node_->get_logger(), "Found " << vertex_normals.numValues()
                             << " vertex normals in map file!");
  } else {
    RCLCPP_INFO_STREAM(node_->get_logger(), 
        "No vertex normals found in the given map file, computing them...");
    vertex_normals = lvr2::calcVertexNormals(*mesh_ptr, face_normals);
    if (mesh_io_ptr->addDenseAttributeMap(vertex_normals, "vertex_normals")) {
      RCLCPP_INFO_STREAM(node_->get_logger(), "Saved vertex normals to map file.");
    } else {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Could not save vertex normals to map file!");
      return false;
    }
  }

  roughness_ =
      lvr2::calcVertexRoughness(*mesh_ptr, config_.radius, vertex_normals);

  return computeLethals();
}

lvr2::VertexMap<float> &RoughnessLayer::costs() { return roughness_; }

rcl_interfaces::msg::SetParametersResult RoughnessLayer::reconfigureCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  bool has_threshold_changed = false;
  for (auto parameter : parameters) {
    if (parameter.get_name() == layer_name_ + ".threshold") {
      config_.threshold = parameter.as_double();
      has_threshold_changed = true;
    } else if (parameter.get_name() == layer_name_ + ".radius") {
      config_.radius = parameter.as_double();
    } else if (parameter.get_name() == layer_name_ + ".factor") {
      config_.factor = parameter.as_double();
    }
  }

  if (has_threshold_changed) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Recompute lethals and notify change from " << layer_name_ << " due to cfg change.");
    computeLethals();
    notifyChange();
  }

  return result;
}

bool RoughnessLayer::initialize(const std::string &name, const rclcpp::Node::SharedPtr& node) {
  node_ = node;
  layer_name_ = name;
  dyn_params_handler_ = node_->add_on_set_parameters_callback(std::bind(
      &RoughnessLayer::reconfigureCallback, this, std::placeholders::_1));
  return true;
}

} /* namespace mesh_layers */
