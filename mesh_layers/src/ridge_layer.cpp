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
#include <pluginlib/class_list_macros.h>
#include <math.h>

PLUGINLIB_EXPORT_CLASS(mesh_layers::RidgeLayer, mesh_map::AbstractLayer)

namespace mesh_layers
{
bool RidgeLayer::readLayer()
{
  ROS_INFO_STREAM("Try to read ridge from map file...");
  auto ridge_opt = mesh_io_ptr->getDenseAttributeMap<lvr2::DenseVertexMap<float>>("ridge");
  if (ridge_opt)
  {
    ROS_INFO_STREAM("Successfully read ridge from map file.");
    ridge = ridge_opt.get();
    return computeLethals();
  }

  return false;
}

bool RidgeLayer::writeLayer()
{
  if (mesh_io_ptr->addDenseAttributeMap(ridge, "ridge"))
  {
    ROS_INFO_STREAM("Saved ridge to map file.");
    return true;
  }
  else
  {
    ROS_ERROR_STREAM("Could not save ridge to map file!");
    return false;
  }
}

bool RidgeLayer::computeLethals()
{
  ROS_INFO_STREAM("Compute lethals for \"" << layer_name << "\" (Ridge Layer) with threshold " << config.threshold);
  lethal_vertices.clear();
  for (auto vH : ridge)
  {
    if (ridge[vH] > config.threshold)
      lethal_vertices.insert(vH);
  }
  ROS_INFO_STREAM("Found " << lethal_vertices.size() << " lethal vertices.");
  return true;
}

float RidgeLayer::threshold()
{
  return config.threshold;
}

bool RidgeLayer::computeLayer()
{
  ROS_INFO_STREAM("Computing ridge...");

  lvr2::DenseFaceMap<mesh_map::Normal> face_normals;

  auto face_normals_opt = mesh_io_ptr->getDenseAttributeMap<lvr2::DenseFaceMap<mesh_map::Normal>>("face_normals");

  if (face_normals_opt)
  {
    face_normals = face_normals_opt.get();
    ROS_INFO_STREAM("Found " << face_normals.numValues() << " face normals in map file.");
  }
  else
  {
    ROS_INFO_STREAM("No face normals found in the given map file, computing them...");
    face_normals = lvr2::calcFaceNormals(*mesh_ptr);
    ROS_INFO_STREAM("Computed " << face_normals.numValues() << " face normals.");
    if (mesh_io_ptr->addDenseAttributeMap(face_normals, "face_normals"))
    {
      ROS_INFO_STREAM("Saved face normals to map file.");
    }
    else
    {
      ROS_ERROR_STREAM("Could not save face normals to map file!");
      return false;
    }
  }

  lvr2::DenseVertexMap<mesh_map::Normal> vertex_normals;
  auto vertex_normals_opt = mesh_io_ptr->getDenseAttributeMap<lvr2::DenseVertexMap<mesh_map::Normal>>("vertex_normals");

  if (vertex_normals_opt)
  {
    vertex_normals = vertex_normals_opt.get();
    ROS_INFO_STREAM("Found " << vertex_normals.numValues() << " vertex normals in map file!");
  }
  else
  {
    ROS_INFO_STREAM("No vertex normals found in the given map file, computing them...");
    vertex_normals = lvr2::calcVertexNormals(*mesh_ptr, face_normals);
    if (mesh_io_ptr->addDenseAttributeMap(vertex_normals, "vertex_normals"))
    {
      ROS_INFO_STREAM("Saved vertex normals to map file.");
    }
    else
    {
      ROS_ERROR_STREAM("Could not save vertex normals to map file!");
      return false;
    }
  }

  ridge.reserve(mesh_ptr->nextVertexIndex());

  for (size_t i = 0; i < mesh_ptr->nextVertexIndex(); i++)
  {
    auto vH = lvr2::VertexHandle(i);
    if (!mesh_ptr->containsVertex(vH))
    {
      ridge.insert(vH, config.threshold + 0.1);
      continue;
    }

    std::set<lvr2::VertexHandle> invalid;

    float value = 0.0;
    int num_neighbours = 0;
    lvr2::BaseVector<float> reference = mesh_ptr->getVertexPosition(vH) + vertex_normals[vH];
    visitLocalVertexNeighborhood(*mesh_ptr.get(), invalid, vH, config.radius, [&](auto vertex) {
      lvr2::BaseVector<float> current_point = mesh_ptr->getVertexPosition(vertex) + vertex_normals[vertex];
      value += (current_point - reference).length();
      num_neighbours++;
    });

    if (num_neighbours == 0)
    {
      ridge.insert(vH, config.threshold + 0.1);
      continue;
    }

    ridge.insert(vH, value / num_neighbours);
  }

  return computeLethals();
}

lvr2::VertexMap<float>& RidgeLayer::costs()
{
  return ridge;
}

void RidgeLayer::reconfigureCallback(mesh_layers::RidgeLayerConfig& cfg, uint32_t level)
{
  bool notify = false;

  ROS_INFO_STREAM("New ridge layer config through dynamic reconfigure.");
  if (first_config)
  {
    config = cfg;
    first_config = false;
    return;
  }

  if (config.threshold != cfg.threshold)
  {
    computeLethals();
    notify = true;
  }

  if (config.radius != cfg.radius)
  {
    computeLayer();
    notify = true;
  }

  if (notify)
    notifyChange();

  config = cfg;
}

bool RidgeLayer::initialize(const std::string& name)
{
  first_config = true;
  reconfigure_server_ptr = boost::shared_ptr<dynamic_reconfigure::Server<mesh_layers::RidgeLayerConfig>>(
      new dynamic_reconfigure::Server<mesh_layers::RidgeLayerConfig>(private_nh));

  config_callback = boost::bind(&RidgeLayer::reconfigureCallback, this, _1, _2);
  reconfigure_server_ptr->setCallback(config_callback);
  return true;
}

} /* namespace mesh_layers */
