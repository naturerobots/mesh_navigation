#ifndef MESH_MAP__INFLATION_LAYER_H
#define MESH_MAP__INFLATION_LAYER_H

#include <mesh_map/abstract_layer.h>

namespace mesh_map{

class InflationLayer : public AbstractLayer
{
  virtual bool readLayer();

  virtual bool writeLayer();

  virtual float getThreshold();

  void lethalCostInflation(
      const std::set<lvr2::VertexHandle>& lethals,
      const float inflation_radius,
      const float inscribed_radius,
      const float inscribed_value,
      const float lethal_value);

  virtual bool computeLayer(const MeshMapConfig& config);

  virtual lvr2::VertexMap<float>& getCosts();

  virtual const std::string getName();

  virtual void setLethals(std::set<lvr2::VertexHandle>& lethal){lethal_vertices = lethal;};

  lvr2::DenseVertexMap<float> riskiness;

  std::set<lvr2::VertexHandle> lethal_vertices;


};

} /* namespace mesh_map */

#endif //MESH_MAP__INFLATION_LAYER_H
