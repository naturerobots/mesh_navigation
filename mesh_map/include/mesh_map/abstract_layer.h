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

#include <functional>
#include <lvr2/io/AttributeMeshIOBase.hpp>
#include <mesh_map/MeshMapConfig.h>
#include <mesh_map/mesh_map.h>
#include <boost/optional.hpp>

#ifndef MESH_MAP__ABSTRACT_LAYER_H
#define MESH_MAP__ABSTRACT_LAYER_H

namespace mesh_map
{
class MeshMap;

typedef lvr2::BaseVector<float> Vector;
typedef lvr2::Normal<float> Normal;

typedef std::function<void(const std::string&)> notify_func;

class AbstractLayer
{
public:
  typedef boost::shared_ptr<mesh_map::AbstractLayer> Ptr;

  virtual bool readLayer() = 0;

  virtual bool writeLayer() = 0;

  virtual float defaultValue() = 0;

  virtual float threshold() = 0;

  virtual bool computeLayer() = 0;

  virtual lvr2::VertexMap<float>& costs() = 0;

  virtual std::set<lvr2::VertexHandle>& lethals() = 0;

  virtual void updateLethal(std::set<lvr2::VertexHandle>& added_lethal,
                            std::set<lvr2::VertexHandle>& removed_lethal) = 0;

  virtual lvr2::BaseVector<float> vectorAt(const std::array<lvr2::VertexHandle, 3>& vertices,
                                           const std::array<float, 3>& barycentric_coords)
  {
    return lvr2::BaseVector<float>();
  }

  virtual const boost::optional<lvr2::VertexMap<lvr2::BaseVector<float>>&> vectorMap()
  {
    return boost::none;
  }

  virtual lvr2::BaseVector<float> vectorAt(const lvr2::VertexHandle& vertex)
  {
    return lvr2::BaseVector<float>();
  }

  virtual bool initialize(const std::string& name) = 0;

  virtual bool initialize(const std::string& name, const notify_func notify_update,
                          std::shared_ptr<mesh_map::MeshMap>& map, std::shared_ptr<lvr2::HalfEdgeMesh<Vector>>& mesh,
                          std::shared_ptr<lvr2::AttributeMeshIOBase>& io)
  {
    layer_name = name;
    private_nh = ros::NodeHandle("~/mesh_map/" + name);
    notify = notify_update;
    mesh_ptr = mesh;
    map_ptr = map;
    mesh_io_ptr = io;
    return initialize(name);
  }

  void notifyChange()
  {
    this->notify(layer_name);
  }

protected:
  std::string layer_name;
  std::shared_ptr<lvr2::AttributeMeshIOBase> mesh_io_ptr;
  std::shared_ptr<lvr2::HalfEdgeMesh<Vector>> mesh_ptr;
  std::shared_ptr<mesh_map::MeshMap> map_ptr;

  ros::NodeHandle private_nh;

private:
  notify_func notify;
};

} /* namespace mesh_map */

#endif  // MESH_MAP__ABSTRACT_LAYER_H
