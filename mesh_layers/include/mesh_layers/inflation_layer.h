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

  void waveCostInflation(const std::set<lvr2::VertexHandle> &lethals,
                         const float inflation_radius,
                         const float inscribed_radius,
                         const float inscribed_value,
                         const float lethal_value);

  lvr2::BaseVector<float> vectorAt(
      const std::array<lvr2::VertexHandle, 3>& vertices,
      const std::array<float, 3>& barycentric_coords);

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
