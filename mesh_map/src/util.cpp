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

#include <mesh_map/util.h>
#include <std_msgs/ColorRGBA.h>
#include <tf/transform_datatypes.h>

namespace mesh_map {

void getMinMax(const lvr2::VertexMap<float> &costs, float &min, float &max) {
  max = std::numeric_limits<float>::min();
  min = std::numeric_limits<float>::max();

  // Calculate minimum and maximum values
  for (auto vH : costs) {
    if (max < costs[vH] && std::isfinite(costs[vH]))
      max = costs[vH];
    if (min > costs[vH] && std::isfinite(costs[vH]))
      min = costs[vH];
  }
}

Vector toVector(const geometry_msgs::Point &p) { return Vector(p.x, p.y, p.z); }

geometry_msgs::Pose calculatePoseFromDirection(const Vector &position,
                                               const Vector &direction,
                                               const Normal &normal) {
  Normal ez = normal.normalized();
  Normal ey = normal.cross(direction).normalized();
  Normal ex = ey.cross(normal).normalized();

  tf::Matrix3x3 tf_basis(ex.x, ey.x, ez.x, ex.y, ey.y, ez.y, ex.z, ey.z, ez.z);

  tf::Vector3 tf_origin(position.x, position.y, position.z);

  tf::Pose tf_pose;
  tf_pose.setBasis(tf_basis);
  tf_pose.setRotation(tf_pose.getRotation().normalize());
  tf_pose.setOrigin(tf_origin);
  geometry_msgs::Pose pose;
  tf::poseTFToMsg(tf_pose, pose);
  return pose;
}

geometry_msgs::Pose calculatePoseFromPosition(const Vector &current,
                                              const Vector &next,
                                              const Normal &normal) {
  float cost = 0;
  return calculatePoseFromPosition(current, next, normal, cost);
}

geometry_msgs::Pose calculatePoseFromPosition(const Vector &current,
                                              const Vector &next,
                                              const Normal &normal,
                                              float &cost) {
  const Vector direction = next - current;
  cost = direction.length();
  return calculatePoseFromDirection(current, direction, normal);
}

bool inTriangle(const Vector &p, const Vector &v0, const Vector &v1,
                const Vector &v2, const float &max_dist, const float &epsilon) {
  float dist;
  std::array<float, 3> barycentric_coords;
  return projectedBarycentricCoords(p, {v0, v1, v2}, barycentric_coords,
                                    dist) &&
         dist < max_dist;
}

Vector projectVectorOntoPlane(const Vector &vec, const Vector &ref,
                              const Normal &normal) {
  return vec - (normal * (vec.dot(normal) - (ref.dot(normal))));
}

bool projectedBarycentricCoords(const Vector &p,
                                const std::array<Vector, 3> &vertices,
                                std::array<float, 3> &barycentric_coords) {
  float dist;
  return projectedBarycentricCoords(p, vertices, barycentric_coords, dist);
}

bool projectedBarycentricCoords(const Vector &p,
                                const std::array<Vector, 3> &vertices,
                                std::array<float, 3> &barycentric_coords,
                                float &dist) {
  const Vector &a = vertices[0];
  const Vector &b = vertices[1];
  const Vector &c = vertices[2];

  Vector u = b - a;
  Vector v = c - a;
  Vector w = p - a;
  Vector n = u.cross(v);
  // Barycentric coordinates of the projection P′of P onto T:
  // γ=[(u×w)⋅n]/n²
  float oneOver4ASquared = 1.0 / n.dot(n);

  const float gamma = u.cross(w).dot(n) * oneOver4ASquared;
  // β=[(w×v)⋅n]/n²
  const float beta = w.cross(v).dot(n) * oneOver4ASquared;
  const float alpha = 1 - gamma - beta;

  barycentric_coords = {alpha, beta, gamma};
  dist = n.dot(w) / n.length();

  const float EPSILON = 0.01;
  // The point P′ lies inside T if:
  return ((0 - EPSILON <= alpha) && (alpha <= 1 + EPSILON) &&
          (0 - EPSILON <= beta) && (beta <= 1 + EPSILON) &&
          (0 - EPSILON <= gamma) && (gamma <= 1 + EPSILON));
}

std_msgs::ColorRGBA color(const float &r, const float &g, const float &b,
                          const float &a) {
  std_msgs::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

bool barycentricCoords(const Vector &p, const Vector &v0, const Vector &v1,
                       const Vector &v2, float &u, float &v, float &w) {
  // compute plane's normal
  Vector v0v1 = v1 - v0;
  Vector v0v2 = v2 - v0;

  // no need to normalize
  Vector N = v0v1.cross(v0v2); // N
  float denom = N.dot(N);

  // Step 2: inside-outside test
  Vector C; // vector perpendicular to triangle's plane

  // edge 0
  Vector edge0 = v1 - v0;
  Vector vp0 = p - v0;
  C = edge0.cross(vp0);
  if (N.dot(C) < 0)
    return false; // P is on the right side

  // edge 1
  Vector edge1 = v2 - v1;
  Vector vp1 = p - v1;
  C = edge1.cross(vp1);
  if ((u = N.dot(C)) < 0)
    return false; // P is on the right side

  // edge 2
  Vector edge2 = v0 - v2;
  Vector vp2 = p - v2;
  C = edge2.cross(vp2);
  if ((v = N.dot(C)) < 0)
    return false; // P is on the right side;

  u /= denom;
  v /= denom;
  w = 1 - u - v;

  return true;
}

} /* namespace mesh_map */