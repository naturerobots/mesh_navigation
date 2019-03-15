#ifndef MESH_MAP__HEIGHTDIFF_LAYER_H
#define MESH_MAP__HEIGHTDIFF_LAYER_H

#include <mesh_map/abstract_layer.h>

namespace mesh_layers{

 class HeightDiffLayer : public mesh_map::AbstractLayer
{
  virtual bool readLayer();

  virtual bool writeLayer();

  virtual float defaultValue(){ return std::numeric_limits<float>::infinity(); }

  virtual float threshold();

  virtual bool computeLayer(const mesh_map::MeshMapConfig& config);

  virtual lvr2::VertexMap<float>& costs();

  virtual const std::string getName();

  virtual void setLethals(std::set<lvr2::VertexHandle>& lethal){};

  lvr2::DenseVertexMap<float> height_diff;


};

} /* namespace mesh_layers */

#endif //MESH_MAP__HEIGHTDIFF_LAYER_H
