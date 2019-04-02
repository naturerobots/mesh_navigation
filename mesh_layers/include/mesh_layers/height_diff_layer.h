#ifndef MESH_MAP__HEIGHTDIFF_LAYER_H
#define MESH_MAP__HEIGHTDIFF_LAYER_H

#include <mesh_map/abstract_layer.h>
#include <mesh_layers/HeightDiffLayerConfig.h>
#include <dynamic_reconfigure/server.h>

namespace mesh_layers{

 class HeightDiffLayer : public mesh_map::AbstractLayer
{
  virtual bool readLayer();

  virtual bool writeLayer();

  virtual float defaultValue(){ return std::numeric_limits<float>::infinity(); }

  virtual float threshold();

  virtual bool computeLayer();

  virtual lvr2::VertexMap<float>& costs();

  virtual std::set<lvr2::VertexHandle>& lethals(){return lethal_vertices;}

  virtual void updateLethal(
       std::set<lvr2::VertexHandle>& added_lethal,
       std::set<lvr2::VertexHandle>& removed_lethal){}

  virtual bool initialize(const std::string &name);

 private:
  // Server for Reconfiguration
   boost::shared_ptr<dynamic_reconfigure::Server<mesh_layers::HeightDiffLayerConfig> > reconfigure_server_ptr;
   dynamic_reconfigure::Server<mesh_layers::HeightDiffLayerConfig>::CallbackType config_callback;
   bool first_config;
   HeightDiffLayerConfig config;

   lvr2::DenseVertexMap<float> height_diff;

   std::set<lvr2::VertexHandle> lethal_vertices;

   void reconfigureCallback(mesh_layers::HeightDiffLayerConfig& cfg, uint32_t level);

};

} /* namespace mesh_layers */

#endif //MESH_MAP__HEIGHTDIFF_LAYER_H
