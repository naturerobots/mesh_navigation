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

#ifndef MESH_MAP__ROUGHNESS_LAYER_H
#define MESH_MAP__ROUGHNESS_LAYER_H

#include <dynamic_reconfigure/server.h>
#include <mesh_layers/RoughnessLayerConfig.h>
#include <mesh_map/abstract_layer.h>

namespace mesh_layers
{
class RoughnessLayer : public mesh_map::AbstractLayer
{
  virtual bool readLayer();

  virtual bool writeLayer();

  virtual float threshold();

  virtual float defaultValue()
  {
    return std::numeric_limits<float>::infinity();
  }

  virtual bool computeLayer();

  bool computeLethals();

  virtual lvr2::VertexMap<float>& costs();

  virtual std::set<lvr2::VertexHandle>& lethals()
  {
    return lethal_vertices;
  }

  virtual void updateLethal(std::set<lvr2::VertexHandle>& added_lethal, std::set<lvr2::VertexHandle>& removed_lethal){};

  lvr2::DenseVertexMap<float> roughness;

  virtual bool initialize(const std::string& name);

  std::set<lvr2::VertexHandle> lethal_vertices;

  // Server for Reconfiguration
  boost::shared_ptr<dynamic_reconfigure::Server<mesh_layers::RoughnessLayerConfig>> reconfigure_server_ptr;
  dynamic_reconfigure::Server<mesh_layers::RoughnessLayerConfig>::CallbackType config_callback;
  bool first_config;
  RoughnessLayerConfig config;

  void reconfigureCallback(mesh_layers::RoughnessLayerConfig& cfg, uint32_t level);
};

} /* namespace mesh_layers */

#endif  // MESH_MAP__ROUGHNESS_LAYER_H
