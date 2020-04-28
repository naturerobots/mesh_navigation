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

#ifndef MESH_MAP__INFLATION_LAYER_H
#define MESH_MAP__INFLATION_LAYER_H

#include <dynamic_reconfigure/server.h>
#include <mesh_layers/InflationLayerConfig.h>
#include <mesh_map/abstract_layer.h>


namespace mesh_layers {

const float EPSILON = 1e-9;

class InflationLayer : public mesh_map::AbstractLayer {
  virtual bool readLayer();

  virtual bool writeLayer();

  virtual float defaultValue() { return 0; }

  virtual float threshold();

  void lethalCostInflation(const std::set<lvr2::VertexHandle> &lethals,
                           const float inflation_radius,
                           const float inscribed_radius,
                           const float inscribed_value,
                           const float lethal_value);

  inline float computeUpdateSethianMethod(const float &d1, const float &d2,
                                   const float &a, const float &b,
                                   const float &dot, const float &F);
  inline bool waveFrontUpdate(
      lvr2::DenseVertexMap<float> &distances,
      lvr2::DenseVertexMap<lvr2::VertexHandle> &predecessors,
      const float& max_distance,
      const lvr2::DenseEdgeMap<float> &edge_weights,
      const lvr2::FaceHandle &fh, const lvr2::BaseVector<float>& normal,
      const lvr2::VertexHandle &v1, const lvr2::VertexHandle &v2, const lvr2::VertexHandle &v3);

  float fading(const float val);

  void waveCostInflation(const std::set<lvr2::VertexHandle> &lethals,
                         const float inflation_radius,
                         const float inscribed_radius,
                         const float inscribed_value,
                         const float lethal_value);

  lvr2::BaseVector<float> vectorAt(
      const std::array<lvr2::VertexHandle, 3>& vertices,
      const std::array<float, 3>& barycentric_coords);

  lvr2::BaseVector<float> vectorAt(const lvr2::VertexHandle& vertex);

  const boost::optional<lvr2::VertexMap<lvr2::BaseVector<float>>&> vectorMap()
  {
    return vector_map;
  }

  void backToSource(
      const lvr2::VertexHandle& current_vertex,
      const lvr2::DenseVertexMap<lvr2::VertexHandle>& predecessors,
      lvr2::DenseVertexMap<lvr2::BaseVector<float>>& vector_map);

  virtual bool computeLayer();

  virtual lvr2::VertexMap<float> &costs();

  virtual std::set<lvr2::VertexHandle> &lethals() {
    return lethal_vertices;
  } // TODO remove... layer types

  virtual void updateLethal(std::set<lvr2::VertexHandle> &added_lethal,
                            std::set<lvr2::VertexHandle> &removed_lethal);

  virtual bool initialize(const std::string &name);

  lvr2::DenseVertexMap<float> riskiness;

  lvr2::DenseVertexMap<float> direction;

  lvr2::DenseVertexMap<lvr2::FaceHandle> cutting_faces;

  lvr2::DenseVertexMap<lvr2::BaseVector<float>> vector_map;

  lvr2::DenseVertexMap<float> distances;

  std::set<lvr2::VertexHandle> lethal_vertices;

  // Server for Reconfiguration
  boost::shared_ptr<
      dynamic_reconfigure::Server<mesh_layers::InflationLayerConfig>>
      reconfigure_server_ptr;
  dynamic_reconfigure::Server<mesh_layers::InflationLayerConfig>::CallbackType
      config_callback;
  bool first_config;
  InflationLayerConfig config;

  void reconfigureCallback(mesh_layers::InflationLayerConfig &cfg,
                           uint32_t level);
};

} /* namespace mesh_layers */

#endif // MESH_MAP__INFLATION_LAYER_H
