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

#include <mesh_planner/fast_marching.h>

namespace mesh_map {


bool FastMarching::performFastMarching(
    const lvr2::HalfEdgeMesh<lvr2::BaseVector<float>> &hem,
    const lvr2::DenseEdgeMap<float> &edge_lengths,
    const lvr2::DenseVertexMap<float> &weights,
    const std::vector<lvr2::VertexHandle> &start_vertices,
    lvr2::DenseVertexMap<float> &distances, float max_dist, bool unfolding) {
  // reset vertex_states with default state FAR
  vertex_states = lvr2::DenseVertexMap<State>(hem.nextVertexIndex(), FAR);

  // reset distance map with default value infinity
  distances = lvr2::DenseVertexMap<float>(
      hem.nextVertexIndex(), std::numeric_limits<float>::infinity());

  // active vertices stored in a min heap
  lvr2::Meap<lvr2::VertexHandle, float> active_vertices;

  // initialize start vertices
  for (auto vH : start_vertices) {
    vertex_states[vH] = ALIVE;
    vertex_fronts.insert(vH, vH);
    distances[vH] = 0;
    active_vertices.insert(vH, 0);
  }

  // propagate wave
  while (!active_vertices.isEmpty()) {
    // retrieve vertex with cheapest costs
    lvr2::VertexHandle current_vertex = active_vertices.popMin().key;
    const float &current_dist = distances[current_vertex];
    vertex_states[current_vertex] = DEAD;

    if (current_dist > max_dist)
      return true;

    // TODO implement new DEAD vretex callback

    hem.getNeighboursOfVertex(current_vertex);

    for (auto nH : hem.getNeighboursOfVertex(current_vertex)) {
      float new_dist = std::numeric_limits<float>::infinity();
      for (auto fH : hem.getFacesOfVertex(nH)) {
        auto face = hem.getVerticesOfFace(fH);

        size_t i = 0;
        while (nH != face[i])
          i++;
        lvr2::VertexHandle j(face[i + 1 % 3]);
        lvr2::VertexHandle k(face[i + 2 % 3]);

        // smaller distance value should correspond with j
        if (distances[j] > distances[k])
          std::swap(j, k);

        const float weight = 1; // weights[nH];
        // if(distances[j] < new_dist){
        float dist = computeVertexDistance(hem, edge_lengths, distances, fH, nH,
                                           j, k, vertex_fronts[current_vertex],
                                           weight, unfolding);
        new_dist = std::min(new_dist, dist);
        //}
      }

      switch (vertex_states[nH]) {
      case FAR:
        distances[nH] = new_dist;
        active_vertices.insert(nH, new_dist);
        vertex_states[nH] = ALIVE;
        vertex_fronts.insert(nH, vertex_fronts[current_vertex]);
        break;
      case ALIVE:
        // TODO frontOverlap(nH, vertex_fronts[current_vertex], new_dist)
        if (new_dist <= distances[nH]) {
          distances[nH] = new_dist;
          active_vertices.insert(nH, new_dist);
          vertex_fronts.insert(nH, vertex_fronts[current_vertex]);
        }
        break;
      case DEAD:
        // TODO frontOverlap(nH, vertex_fronts[current_vertex], new_dist)
        break;
      }
    }
  }

  return true;
}

float FastMarching::computeVertexDistance(
    const lvr2::HalfEdgeMesh<lvr2::BaseVector<float>> &hem,
    const lvr2::DenseEdgeMap<float> &edge_lengths,
    const lvr2::DenseVertexMap<float> &distances, const lvr2::FaceHandle &face,
    const lvr2::VertexHandle &v0h, const lvr2::VertexHandle &v1h,
    const lvr2::VertexHandle &v2h, const lvr2::VertexHandle &front,
    const float &F, const bool &unfolding) {

  lvr2::BaseVector<float> v0 = hem.getVertexPosition(v0h);
  lvr2::BaseVector<float> v1 = hem.getVertexPosition(v1h);
  lvr2::BaseVector<float> v2 = hem.getVertexPosition(v2h);
  lvr2::BaseVector<float> e1 = v1 - v0; // edge 1
  lvr2::BaseVector<float> e2 = v2 - v0; // edge 2

  const float b = e1.length();
  const float a = e2.length();

  e1 /= b;
  e2 /= a;

  const double d1 = distances[v1h];
  const double d2 = distances[v2h];

  bool uv1 = vertex_states[v1h] != FAR && vertex_fronts[v1h] == front;
  bool uv2 = vertex_states[v2h] != FAR && vertex_fronts[v2h] == front;

  if (!uv1 && uv2) {
    return d2 + a * F;
  } else if (uv1 && !uv2) {
    return d1 + b * F;
  } else if (uv1 && uv2) {
    double dot = e1.dot(e2);
    if (dot < 0 && unfolding) {
      float c, dot1, dot2;
      lvr2::OptionalVertexHandle opvh =
          unfoldTriangle(hem, face, v0h, v1h, v2h, c, dot1, dot2);
      if (opvh && vertex_states[opvh.unwrap()] != FAR) {
        const float d3 = distances[opvh.unwrap()];
        return std::min(mesh_map::computeUpdateSethianMethod(d1, d3, c, b, dot1, F),
                        mesh_map::computeUpdateSethianMethod(d3, d2, a, c, dot2, F));
      }
    }
    return mesh_map::computeUpdateSethianMethod(d1, d2, a, b, dot, F);
  }
}

float computeUpdateSethianMethod(const float &d1, const float &d2,
                                 const float &a, const float &b,
                                 const float &dot, const float &F) {
  float t = std::numeric_limits<float>::infinity();
  float r_cos_angle = dot;
  float r_sin_angle = sqrt(1 - dot * dot);

  float u = d2 - d1; // T(B) - T(A)

  float f2 = a * a + b * b - 2 * a * b * r_cos_angle;
  float f1 = b * u * (a * r_cos_angle - b);
  float f0 = b * b * (u * u - F * F * a * a * r_sin_angle);

  float delta = f1 * f1 - f0 * f2;

  if (delta >= 0) {
    if (std::fabs(f2) > mesh_map::EPSILON) {
      t = (-f1 - sqrt(delta)) / f2;
      if (t < u || b * (t - u) / t < a * r_cos_angle ||
          a / r_cos_angle < b * (t - u) / 2) {
        t = (-f1 + sqrt(delta)) / f2;
      } else {
        if (f1 != 0) {
          t = -f0 / f1;
        } else {
          t = -std::numeric_limits<float>::infinity();
        }
      }
    }
  } else {
    t = -std::numeric_limits<float>::infinity();
  }

  if (u < t && a * r_cos_angle < b * (t - u) / t &&
      b * (t - u) / t < a / r_cos_angle) {
    return t + d1;
  } else {
    return std::min(b * F + d1, a * F + d2);
  }
}

float FastMarching::computeMatrixMathod(const float &d1, const float &d2,
                                        const float &a, const float &b,
                                        const float &dot, const float &F) {

  float t;

  /* the directional derivative is D-t*L */
  Eigen::Vector2d D = Eigen::Vector2d(d1 / b, d2 / a);
  Eigen::Vector2d L = Eigen::Vector2d(1 / b, 1 / a);

  Eigen::Vector2d QL; // Q*L
  Eigen::Vector2d QD; // Q*L

  float det = 1 - dot * dot; // 1/det(Q) where Q=(P*P^T)^-1

  QD[0] = 1 / det * (D[0] - dot * D[1]);
  QD[1] = 1 / det * (-dot * D[0] + D[1]);
  QL[0] = 1 / det * (L[0] - dot * L[1]);
  QL[1] = 1 / det * (-dot * L[0] + L[1]);

  /* compute the equation 'e2*t?+ 2*e1*t + e0 = 0' */
  float e2 = QL[0] * L[0] + QL[1] * L[1];         // <L,Q*L>
  float e1 = -(QD[0] * L[0] + QD[1] * L[1]);      // -<L,Q*D>
  float e0 = QD[0] * D[0] + QD[1] * D[1] - F * F; // <D,Q*D> - F?

  float delta = e1 * e1 - e0 * e2;

  if (delta >= 0) {
    if (std::fabs(e2) > EPSILON) {
      /* there is a solution */
      t = (-e1 - sqrt(delta)) / e2;
      /* upwind criterion : Q*(D-t*l)<=0, i.e. QD<=t*QL */
      if (t < std::min(d1, d2) || QD[0] > t * QL[0] || QD[1] > t * QL[1])
        t = (-e1 + sqrt(delta)) /
            e2; // criterion not respected: choose bigger root.
    } else {
      if (e1 != 0)
        t = -e0 / e1;
      else
        t = -std::numeric_limits<float>::infinity();
    }
  } else
    t = -std::numeric_limits<float>::infinity();
  /* choose the update from the 2 vertex only if upwind criterion is met */
  if (t >= std::max(d1, d2) && QD[0] <= t * QL[0] && QD[1] <= t * QL[1])
    return t;
  else
    return std::min(b * F + d1, a * F + d2);
}

lvr2::OptionalVertexHandle FastMarching::unfoldTriangle(
    const lvr2::HalfEdgeMesh<lvr2::BaseVector<float>> &hem,
    const lvr2::FaceHandle &fh, const lvr2::VertexHandle &v0h,
    const lvr2::VertexHandle &v1h, const lvr2::VertexHandle &v2h, float &dist,
    float &dot1, float &dot2) {
  const lvr2::BaseVector<float> &v0 = hem.getVertexPosition(v0h);
  const lvr2::BaseVector<float> &v1 = hem.getVertexPosition(v1h);
  const lvr2::BaseVector<float> &v2 = hem.getVertexPosition(v2h);

  lvr2::BaseVector<float> e1 = v1 - v0;
  float le1 = e1.length();
  e1 /= le1;
  lvr2::BaseVector<float> e2 = v2 - v0;
  float le2 = e2.length();
  e2 /= le2;

  float dot = e1 * e2;

  /* the equation of the lines defining the unfolding region [e.g. line 1 : {x ; <x,eq1>=0} ]*/
  Eigen::Vector2f eq1(dot, sqrt(1 - dot * dot));
  Eigen::Vector2f eq2(1, 0);

  /* position of the 2 points on the unfolding plane */
  Eigen::Vector2f x1(le1, 0);
  Eigen::Vector2f x2 = eq1 * le2;

  /* keep track of the starting point */
  Eigen::Vector2f xs1 = x1;
  Eigen::Vector2f xs2 = x2;

  lvr2::VertexHandle pv1h = v1h;
  lvr2::VertexHandle pv2h = v2h;

  lvr2::OptionalFaceHandle opfh = hem.getOppositeFace(fh, v0h);
  for (int i = 0; i < 50 && opfh; i++) {
    lvr2::FaceHandle pfh = opfh.unwrap();
    lvr2::EdgeHandle peh = hem.getEdgeBetween(pv1h, pv2h).unwrap();
    lvr2::VertexHandle pvh = hem.getOppositeVertex(pfh, peh).unwrap();
    lvr2::BaseVector<float> pv = hem.getVertexPosition(pvh);
    lvr2::BaseVector<float> pv1 = hem.getVertexPosition(pv1h);
    lvr2::BaseVector<float> pv2 = hem.getVertexPosition(pv2h);

    e1 = pv2 - pv1;
    le1 = e1.length();
    e1 /= le1;

    e2 = pv - pv1;
    le2 = e2.length();
    e2 /= le2;

    /* compute the position of the new point x on the unfolding plane (via a
       rotation of -alpha on (x2-x1)/le1 ) | cos(alpha) sin(alpha)| x =
       |-sin(alpha) cos(alpha)| * [x2-x1]*le2/le1 + x1   where cos(alpha)=dot
    */
    Eigen::Vector2f vv = (x2 - x1) * le2 / le1;
    dot = e1.dot(e2);
    Eigen::Vector2f x = Eigen::Rotation2Df(-acos(dot)) * vv +
                        x1; // rotate vv by -acos(dot) and add x1

    /* compute the intersection points.
       We look for x=x1+lambda*(x-x1) or x=x2+lambda*(x-x2) with <x,eqi>=0, so
     */
    float lambda11 = -x1.dot(eq1) / (x - x1).dot(eq1); // left most
    float lambda12 = -x1.dot(eq2) / (x - x1).dot(eq2); // right most
    float lambda21 = -x2.dot(eq1) / (x - x2).dot(eq1); // left most
    float lambda22 = -x2.dot(eq2) / (x - x2).dot(eq2); // right most
    bool bIntersect11 = (lambda11 >= 0) && (lambda11 <= 1);
    bool bIntersect12 = (lambda12 >= 0) && (lambda12 <= 1);
    bool bIntersect21 = (lambda21 >= 0) && (lambda21 <= 1);
    bool bIntersect22 = (lambda22 >= 0) && (lambda22 <= 1);

    if (bIntersect11 && bIntersect12) {
      /* we should unfold on edge [x x1] */
      opfh = hem.getOppositeFace(pfh, pv2h);
      pv2h = pvh;
      x2 = x;
    } else if (bIntersect21 && bIntersect22) {
      /* we should unfold on edge [x x2] */
      opfh = hem.getOppositeFace(pfh, pv1h);
      pv1h = pvh;
      x1 = x;
    } else {
      /* that's it, we have found the point */
      dist = x.norm(); // length of y
      dot1 = x.dot(xs1) / (dist * xs1.norm());
      dot2 = x.dot(xs2) / (dist * xs2.norm());
      return pvh;
    }
  }

  return lvr2::OptionalVertexHandle();
}

}