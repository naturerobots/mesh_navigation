/*
 *  Copyright 2019, Sebastian Pütz
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

#ifndef MESH_NAVIGATION__FAST_MARCHING_H
#define MESH_NAVIGATION__FAST_MARCHING_H

#include <lvr2/attrmaps/AttrMaps.hpp>
#include <lvr2/geometry/HalfEdgeMesh.hpp>
#include <lvr2/util/Meap.hpp>

namespace mesh_map {

const float EPSILON = 1e-9;

float computeUpdateSethianMethod(const float &d1, const float &d2,
                                 const float &a, const float &b,
                                 const float &dot, const float &F);

class FastMarching {


public:
  enum State {
    FAR,
    ALIVE,
    DEAD,
    INVALID,
  };

  /**
   * @brief Performs the Fast Marching Method
   * @param hem
   * @param start_vertices
   * @param distances
   * @return
   */
  bool
  performFastMarching(const lvr2::HalfEdgeMesh<lvr2::BaseVector<float>> &hem,
                      const lvr2::DenseEdgeMap<float> &edge_lengths,
                      const lvr2::DenseVertexMap<float> &weights,
                      const std::vector<lvr2::VertexHandle> &start_vertices,
                      lvr2::DenseVertexMap<float> &distances,
                      float max_dist = std::numeric_limits<float>::infinity(),
                      bool unfolding = true);

  float computeVertexDistance(
      const lvr2::HalfEdgeMesh<lvr2::BaseVector<float>> &hem,
      const lvr2::DenseEdgeMap<float> &edge_lengths,
      const lvr2::DenseVertexMap<float> &distances,
      const lvr2::FaceHandle &face, const lvr2::VertexHandle &v0h,
      const lvr2::VertexHandle &v1h, const lvr2::VertexHandle &v2h,
      const lvr2::VertexHandle &front, const float &F, const bool &unfolding);

  float computeMatrixMathod(const float &d1, const float &d2, const float &a,
                            const float &b, const float &dot, const float &F);

  lvr2::OptionalVertexHandle
  unfoldTriangle(const lvr2::HalfEdgeMesh<lvr2::BaseVector<float>> &hem,
                 const lvr2::FaceHandle &fh, const lvr2::VertexHandle &v0h,
                 const lvr2::VertexHandle &v1h, const lvr2::VertexHandle &v2h,
                 float &dist, float &dot1, float &dot2);

  //! current state of the vertex in the propagation process
  lvr2::DenseVertexMap<State> vertex_states;

  //! corresponding front vertices
  lvr2::DenseVertexMap<lvr2::VertexHandle> vertex_fronts;
};

}

#endif // MESH_NAVIGATION__FAST_MARCHING_H
