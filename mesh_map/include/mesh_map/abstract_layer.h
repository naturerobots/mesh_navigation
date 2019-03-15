#include <lvr2/io/AttributeMeshIOBase.hpp>
#include <mesh_map/MeshMapConfig.h>


#ifndef MESH_MAP__ABSTRACT_LAYER_H
#define MESH_MAP__ABSTRACT_LAYER_H
namespace mesh_map{

typedef lvr2::BaseVector<float> VectorType;
typedef lvr2::Normal<float> NormalType;

class AbstractLayer
{

 public:
  typedef boost::shared_ptr<mesh_map::AbstractLayer> Ptr;

  virtual bool readLayer() = 0;

  virtual bool writeLayer() = 0;

  virtual float defaultValue() = 0;

  virtual float threshold() = 0;

  virtual bool computeLayer(const MeshMapConfig& config) = 0;

  virtual lvr2::VertexMap<float>& costs() = 0;

  virtual const std::string getName() = 0;

  virtual void setLethals(std::set<lvr2::VertexHandle>& lethal) = 0;

  virtual bool initialize(
      std::shared_ptr<lvr2::HalfEdgeMesh<VectorType>>& mesh,
      std::shared_ptr<lvr2::AttributeMeshIOBase>& io)
  {
    mesh_ptr = mesh;
    mesh_io_ptr = io;
    return true;
  }

 protected:
  std::shared_ptr<lvr2::AttributeMeshIOBase> mesh_io_ptr;
  std::shared_ptr<lvr2::HalfEdgeMesh<VectorType>> mesh_ptr;
};

} /* namespace mesh_map */

#endif //MESH_MAP__ABSTRACT_LAYER_H
