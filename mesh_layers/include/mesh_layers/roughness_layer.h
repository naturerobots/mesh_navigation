#ifndef MESH_MAP__ROUGHNESS_LAYER_H
#define MESH_MAP__ROUGHNESS_LAYER_H

#include <mesh_map/abstract_layer.h>
#include <mesh_layers/RoughnessLayerConfig.h>
#include <dynamic_reconfigure/server.h>

namespace mesh_layers{

class RoughnessLayer : public mesh_map::AbstractLayer
{
  virtual bool readLayer();

  virtual bool writeLayer();

  virtual float threshold();

  virtual float defaultValue(){ return std::numeric_limits<float>::infinity(); }

  virtual bool computeLayer();

  virtual lvr2::VertexMap<float>& costs();

  virtual std::set<lvr2::VertexHandle>& lethals(){return lethal_vertices;}

  virtual void updateLethal(
      std::set<lvr2::VertexHandle>& added_lethal,
      std::set<lvr2::VertexHandle>& removed_lethal){};

  lvr2::DenseVertexMap<float> roughness;

  virtual bool initialize(const std::string &name);

  std::set<lvr2::VertexHandle> lethal_vertices;

  // Server for Reconfiguration
  boost::shared_ptr<dynamic_reconfigure::Server<mesh_layers::RoughnessLayerConfig> > reconfigure_server_ptr;
  dynamic_reconfigure::Server<mesh_layers::RoughnessLayerConfig>::CallbackType config_callback;
  bool first_config;
  RoughnessLayerConfig config;

  void reconfigureCallback(mesh_layers::RoughnessLayerConfig& cfg, uint32_t level);

};

} /* namespace mesh_layers */

#endif //MESH_MAP__ROUGHNESS_LAYER_H
