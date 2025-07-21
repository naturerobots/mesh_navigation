/*
 *  Copyright 2020, The authors
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
 *    Malte kl. Piening <malte@klpiening.com>
 *
 */

#include "mesh_layers/ridge_layer.h"

#include <lvr2/geometry/BaseVector.hpp>
#include <lvr2/algorithm/GeometryAlgorithms.hpp>
#include <lvr2/algorithm/NormalAlgorithms.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <math.h>

PLUGINLIB_EXPORT_CLASS(mesh_layers::RidgeLayer, mesh_map::AbstractLayer)

namespace mesh_layers
{
bool RidgeLayer::readLayer()
{
  RCLCPP_INFO_STREAM(node_->get_logger(), "Try to read ridge from map file...");
  auto mesh_io = map_ptr_->meshIO();
  auto ridge_opt = mesh_io->getDenseAttributeMap<lvr2::DenseVertexMap<float>>(layer_name_);
  if (ridge_opt)
  {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Successfully read ridge from map file.");
    ridge_ = ridge_opt.get();
    return computeLethals();
  }

  return false;
}

bool RidgeLayer::writeLayer()
{
  auto mesh_io = map_ptr_->meshIO();
  if (mesh_io->addDenseAttributeMap(ridge_, layer_name_))
  {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Saved ridge to map file.");
    return true;
  }
  else
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Could not save ridge to map file!");
    return false;
  }
}

bool RidgeLayer::computeLethals()
{
  RCLCPP_INFO_STREAM(node_->get_logger(), "Compute lethals for \"" << layer_name_ << "\" (Ridge Layer) with threshold " << config_.threshold);
  lethal_vertices_.clear();
  for (auto vH : ridge_)
  {
    if (ridge_[vH] > config_.threshold)
      lethal_vertices_.insert(vH);
  }
  RCLCPP_INFO_STREAM(node_->get_logger(), "Found " << lethal_vertices_.size() << " lethal vertices.");
  return true;
}

float RidgeLayer::threshold()
{
  return config_.threshold;
}

bool RidgeLayer::computeLayer()
{
  RCLCPP_INFO_STREAM(node_->get_logger(), "Computing ridge...");

  lvr2::DenseFaceMap<mesh_map::Normal> face_normals;

  const auto mesh = map_ptr_->mesh();
  auto mesh_io = map_ptr_->meshIO();
  auto face_normals_opt = mesh_io->getDenseAttributeMap<lvr2::DenseFaceMap<mesh_map::Normal>>("face_normals");

  if (face_normals_opt)
  {
    face_normals = face_normals_opt.get();
    RCLCPP_INFO_STREAM(node_->get_logger(), "Found " << face_normals.numValues() << " face normals in map file.");
  }
  else
  {
    RCLCPP_INFO_STREAM(node_->get_logger(), "No face normals found in the given map file, computing them...");
    face_normals = lvr2::calcFaceNormals(*mesh);
    RCLCPP_INFO_STREAM(node_->get_logger(), "Computed " << face_normals.numValues() << " face normals.");
    if (mesh_io->addDenseAttributeMap(face_normals, "face_normals"))
    {
      RCLCPP_INFO_STREAM(node_->get_logger(), "Saved face normals to map file.");
    }
    else
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Could not save face normals to map file!");
      return false;
    }
  }

  lvr2::DenseVertexMap<mesh_map::Normal> vertex_normals;
  auto vertex_normals_opt = mesh_io->getDenseAttributeMap<lvr2::DenseVertexMap<mesh_map::Normal>>("vertex_normals");

  if (vertex_normals_opt)
  {
    vertex_normals = vertex_normals_opt.get();
    RCLCPP_INFO_STREAM(node_->get_logger(), "Found " << vertex_normals.numValues() << " vertex normals in map file!");
  }
  else
  {
    RCLCPP_INFO_STREAM(node_->get_logger(), "No vertex normals found in the given map file, computing them...");
    vertex_normals = lvr2::calcVertexNormals(*mesh, face_normals);
    if (mesh_io->addDenseAttributeMap(vertex_normals, "vertex_normals"))
    {
      RCLCPP_INFO_STREAM(node_->get_logger(), "Saved vertex normals to map file.");
    }
    else
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Could not save vertex normals to map file!");
      return false;
    }
  }

  ridge_.reserve(mesh->nextVertexIndex());

  for (size_t i = 0; i < mesh->nextVertexIndex(); i++)
  {
    auto vH = lvr2::VertexHandle(i);
    if (!mesh->containsVertex(vH))
    {
      ridge_.insert(vH, config_.threshold + 0.1);
      continue;
    }

    std::set<lvr2::VertexHandle> invalid;

    float value = 0.0;
    int num_neighbours = 0;
    lvr2::BaseVector<float> reference = mesh->getVertexPosition(vH) + vertex_normals[vH];
    visitLocalVertexNeighborhood(*mesh, invalid, vH, config_.radius, [&](auto vertex) {
      lvr2::BaseVector<float> current_point = mesh->getVertexPosition(vertex) + vertex_normals[vertex];
      value += (current_point - reference).length();
      num_neighbours++;
    });

    if (num_neighbours == 0)
    {
      ridge_.insert(vH, config_.threshold + 0.1);
      continue;
    }

    ridge_.insert(vH, value / num_neighbours);
  }

  return computeLethals();
}

lvr2::VertexMap<float>& RidgeLayer::costs()
{
  return ridge_;
}

rcl_interfaces::msg::SetParametersResult RidgeLayer::reconfigureCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  bool has_radius_changed = false;
  bool has_threshold_changed = false;

  for (auto parameter : parameters) {
    if (parameter.get_name() == mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".threshold") {
      config_.threshold = parameter.as_double();
      has_threshold_changed = true;
    } else if (parameter.get_name() == mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".radius") {
      config_.radius = parameter.as_double();
      has_radius_changed = true;
    }
  }

  if (has_threshold_changed) { computeLethals(); }
  if (has_radius_changed) { computeLayer(); }
  if (has_radius_changed || has_threshold_changed) 
  {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Notify change from " << layer_name_ << " (RidgeLayer) due to cfg change.");
    notifyChange(); 
  }

  return result;
}

bool RidgeLayer::initialize()
{
  { // threshold
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Threshold for the local ridge to be counted as lethal.";
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.01;
    range.to_value = 3.1415;
    descriptor.floating_point_range.push_back(range);
    config_.threshold = node_->declare_parameter(mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".threshold", config_.threshold, descriptor);
  }
  { // radius
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Radius of the inscribed area.";
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.01;
    range.to_value = 1.0;
    descriptor.floating_point_range.push_back(range);
    config_.radius = node_->declare_parameter(mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".radius", config_.radius, descriptor);
  }
  dyn_params_handler_ = node_->add_on_set_parameters_callback(std::bind(
      &RidgeLayer::reconfigureCallback, this, std::placeholders::_1));
  return true;
}

} /* namespace mesh_layers */
