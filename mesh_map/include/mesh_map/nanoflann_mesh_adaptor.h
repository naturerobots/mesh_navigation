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

#ifndef MESH_MAP__NANOFLANN_MESH_ADAPTOR_H
#define MESH_MAP__NANOFLANN_MESH_ADAPTOR_H

#include <lvr2/geometry/HalfEdgeMesh.hpp>
#include "nanoflann.hpp"

namespace mesh_map{
struct NanoFlannMeshAdaptor
{
  const lvr2::HalfEdgeMesh<lvr2::BaseVector<float>>& mesh;

  /// The constructor that sets the data set source
  NanoFlannMeshAdaptor(const lvr2::HalfEdgeMesh<lvr2::BaseVector<float>> &mesh) : mesh(mesh) { }

  inline lvr2::Index kdtree_get_point_count() const { return mesh.nextVertexIndex(); }

  inline float kdtree_get_pt(const lvr2::Index idx, const size_t dim) const
  {
    const lvr2::VertexHandle vH(idx);
    if(mesh.containsVertex(vH))
    {
      const lvr2::BaseVector<float>& vertex = mesh.getVertexPosition(vH);
      if (dim == 0) return vertex.x;
      else if (dim == 1) return vertex.y;
      else return vertex.z;
    }
    return std::nanf("");
  }

  template <class BBOX>
  bool kdtree_get_bbox(BBOX& /*bb*/) const { return false; }

}; // end of PointCloudAdaptor
}

#endif /* MESH_MAP__NANOFLANN_MESH_ADAPTOR_H */
