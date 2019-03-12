#ifndef MESH_MAP__ROUGHNESS_LAYER_H
#define MESH_MAP__ROUGHNESS_LAYER_H

#include <mesh_map/abstract_layer.h>

namespace mesh_map{

class RoughnessLayer : public AbstractLayer
{
  virtual bool readLayer();

  virtual bool writeLayer();

  virtual float getThreshold();

  virtual bool computeLayer(const MeshMapConfig& config);

  virtual lvr2::VertexMap<float>& getCosts();

  virtual const std::string getName();

  virtual void setLethals(std::set<lvr2::VertexHandle>& lethal){};

  lvr2::DenseVertexMap<float> roughness;


};

} /* namespace mesh_map */

#endif //MESH_MAP__ROUGHNESS_LAYER_H
