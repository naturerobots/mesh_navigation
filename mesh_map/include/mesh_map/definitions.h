#ifndef MESH_MAP__DEFINITIONS_H
#define MESH_MAP__DEFINITIONS_H

#include <lvr2/geometry/BaseVector.hpp>
#include <lvr2/geometry/Normal.hpp>
#include <lvr2/geometry/PMPMesh.hpp>

namespace mesh_map
{

using Normal = lvr2::Normal<float>;
using Vector = lvr2::BaseVector<float>;

class MeshMap;
class AbstractLayer;
class LayerManager;
class NanoFlannMeshAdaptor;
class LayerTimer;

} // namespace mesh_map

#endif // MESH_MAP__DEFINITIONS_H