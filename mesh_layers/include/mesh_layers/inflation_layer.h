#ifndef MESH_MAP__INFLATION_LAYER_H
#define MESH_MAP__INFLATION_LAYER_H

#include <mesh_map/abstract_layer.h>
#include <mesh_layers/InflationLayerConfig.h>
#include <dynamic_reconfigure/server.h>

namespace mesh_layers{

 class InflationLayer : public mesh_map::AbstractLayer
{
  virtual bool readLayer();

  virtual bool writeLayer();

  virtual float defaultValue(){ return 0; }

  virtual float threshold();

  void lethalCostInflation(
      const std::set<lvr2::VertexHandle>& lethals,
      const float inflation_radius,
      const float inscribed_radius,
      const float inscribed_value,
      const float lethal_value);

  virtual bool computeLayer();

  virtual lvr2::VertexMap<float>& costs();

  virtual std::set<lvr2::VertexHandle>& lethals(){return lethal_vertices;} // TODO remove... layer types

  virtual void updateLethal(
       std::set<lvr2::VertexHandle>& added_lethal,
       std::set<lvr2::VertexHandle>& removed_lethal);

  virtual bool initialize(const std::string &name);

  lvr2::DenseVertexMap<float> riskiness;

  std::set<lvr2::VertexHandle> lethal_vertices;

  // Server for Reconfiguration
  boost::shared_ptr<dynamic_reconfigure::Server<mesh_layers::InflationLayerConfig> > reconfigure_server_ptr;
  dynamic_reconfigure::Server<mesh_layers::InflationLayerConfig>::CallbackType config_callback;
  bool first_config;
  InflationLayerConfig config;

  void reconfigureCallback(mesh_layers::InflationLayerConfig& cfg, uint32_t level);

};

} /* namespace mesh_layers */

#endif //MESH_MAP__INFLATION_LAYER_H
