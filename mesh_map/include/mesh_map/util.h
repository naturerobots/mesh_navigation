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

#ifndef MESH_MAP__UTIL_H
#define MESH_MAP__UTIL_H

#include <lvr2/geometry/BaseVector.hpp>
#include <lvr2/geometry/Normal.hpp>
#include <lvr2/geometry/Handles.hpp>
#include <lvr2/attrmaps/AttrMaps.hpp>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

namespace mesh_map{

typedef lvr2::Normal<float> Normal;
typedef lvr2::BaseVector<float> Vector;

void getMinMax(const lvr2::VertexMap<float>& map, float& min, float& max);

Vector toVector(const geometry_msgs::Point& point);

Vector projectVectorOntoPlane(const Vector &vec, const Vector &ref, const Normal &normal);

geometry_msgs::Pose calculatePoseFromDirection(const Vector& position, const Vector& direction, const Normal& normal);

geometry_msgs::Pose calculatePoseFromPosition(const Vector& current, const Vector& next, const Normal& normal);

bool barycentricCoords(const Vector &p, const Vector &v0, const Vector &v1, const Vector &v2, float &u, float &v);

} /* namespace mesh_map */

#endif //MESH_MAP__UTIL_H
