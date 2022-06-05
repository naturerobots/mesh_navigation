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

#ifndef MESH_MAP__UTIL_H
#define MESH_MAP__UTIL_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <lvr2/geometry/Handles.hpp>
#include <lvr2/attrmaps/AttrMaps.hpp>
#include <lvr2/geometry/BaseVector.hpp>
#include <lvr2/geometry/Normal.hpp>
#include <std_msgs/ColorRGBA.h>
#include <tf2/LinearMath/Vector3.h>

namespace mesh_map
{

//! use normals with datatype float
typedef lvr2::Normal<float> Normal;

//! use vectors with datatype folat
typedef lvr2::BaseVector<float> Vector;


/**
 * @brief Function to build std_msgs color instances
 * @param r red, value between 0 and 1
 * @param g green, value between 0 and 1
 * @param b blue, value between 0 and 1
 * @param a alpha (optional, default is 1.0)
 * @return std_msgs/ColorRGBA message
 */
std_msgs::ColorRGBA color(const float& r, const float& g, const float& b, const float& a = 1.0);

/**
 * @brief Function to compute the minimum and maximum of a cost layer
 * @param map The cost layer or vertex float map to evaluate
 * @param min The computed minimum
 * @param max The computed maximum
 */
void getMinMax(const lvr2::VertexMap<float>& map, float& min, float& max);

/**
 * @brief Converts a ROS geometry_msgs/Point message to a lvr2 vector
 * @param point The point message to convert
 * @return The point converted to a lvr2 vector
 */
Vector toVector(const geometry_msgs::Point& point);

/**
 * Computes a geometry_msgs/Pose message from a position, direction and normal vector
 * @param position The position vector
 * @param direction The direction vector
 * @param normal The normal vector / z-axis
 * @return The converted pose message, in the right-hand / ROS coordinate system
 */
geometry_msgs::Pose calculatePoseFromDirection(const Vector& position, const Vector& direction, const Normal& normal);

/**
 * @brief Calculates a geometry_msgs/Pose message from two positions and a normal vector
 * @param current The position vector
 * @param next The vector in which the x-axis points to
 * @param normal The normal vector / z-axis
 * @return The converted pose message, in the right-hand / ROS coordinate system
 */
geometry_msgs::Pose calculatePoseFromPosition(const Vector& current, const Vector& next, const Normal& normal);

/**
 * @brief Calculates a geometry_msgs/Pose message from two positions and a normal vector
 * @param current The position vector
 * @param next The vector in which the x-axis points to
 * @param normal The normal vector / z-axis
 * @param cost The distance between "current" and "next"
 * @return The converted pose message, in the right-hand / ROS coordinate system
 */
geometry_msgs::Pose calculatePoseFromPosition(const Vector& current, const Vector& next, const Normal& normal,
                                              float& cost);
/**
 * @brief Projects a vector / point onto a plane, which is defined by the reference vector and the normal vector
 * @param vec The point which should be projected onto the plane
 * @param ref The plane's reference vector
 * @param normal The plane's normal vector
 * @return The projected vector
 */
Vector projectVectorOntoPlane(const Vector& vec, const Vector& ref, const Normal& normal);

/**
 * @brief Computes whether a points lies inside or outside a triangle with respect to a maximum distance while using an epsilon
 * @param p The query point
 * @param v0 First vertex of the triangle
 * @param v1 Second vertex of the triangle
 * @param v2 Third vertex of the triangle
 * @param max_dist The point's maximum distance to the triangle plane
 * @param epsilon The epsilon used for the inside / outside check
 * @return true if the point lies inside the triangle
 */
bool inTriangle(const Vector& p, const Vector& v0, const Vector& v1, const Vector& v2, const float& max_dist,
                const float& epsilon);

/**
 * @brief Computes projected barycentric coordinates and whether the query point lies inside or outside the given triangle
 * @param p The query point
 * @param vertices The three triangle vertices
 * @param barycentric_coords The computed barycentric coordinates
 * @param dist The distance from the plane to the point
 * @return true if the point lies inside or outside the triangle
 */
bool projectedBarycentricCoords(const Vector& p, const std::array<Vector, 3>& vertices,
                                std::array<float, 3>& barycentric_coords, float& dist);

/**
 * @brief Computes the barycentric coordinates u, v,q of a query point p onto the triangle v0, v1, v2
 * @param p The query point
 * @param v0 First vertex of the triangle
 * @param v1 Second vertex of the triangle
 * @param v2 Third vertex of the triangle
 * @param u First barycentric coordinate
 * @param v Second barycentric coordinate
 * @param w Third barycentric coordinate
 * @return true if the query point lies inside the triangle
 */
bool barycentricCoords(const Vector& p, const Vector& v0, const Vector& v1, const Vector& v2, float& u, float& v,
                       float& w);

/**
 * @brief Computes a linear combination of vertex properties and the barycentric coordinates
 * @tparam T The value to combine with barycentric coordinates, it must support the star/*-operator
 * @param vertex_properties The vertex properties of a triangle
 * @param barycentric_coords The barycentric coordinates
 * @return The linear combined value, e.g. a vector, or cost value
 */
template <typename T>
T linearCombineBarycentricCoords(const std::array<T, 3>& vertex_properties,
                                 const std::array<float, 3>& barycentric_coords)
{
  return vertex_properties[0] * barycentric_coords[0] + vertex_properties[1] * barycentric_coords[1] +
         vertex_properties[2] * barycentric_coords[2];
}

/**
 * @brief Computes a linear combination of vertex properties and the barycentric coordinates
 * @tparam T The value to combine with barycentric coordinates, it must support the star/*-operator
 * @param vertices The three vertex handles of a triangle
 * @param attribute_map An attribute map to access with the given vertex handles
 * @param barycentric_coords The barycentric coordinates
 * @return The linear combined value, e.g. a vector, or cost value
 */
template <typename T>
T linearCombineBarycentricCoords(const std::array<lvr2::VertexHandle, 3>& vertices,
                                 const lvr2::VertexMap<T>& attribute_map,
                                 const std::array<float, 3>& barycentric_coords)
{
  const std::array<T, 3> values = { attribute_map[vertices[0]], attribute_map[vertices[1]],
                                    attribute_map[vertices[2]] };

  return linearCombineBarycentricCoords<T>(values, barycentric_coords);
}

/**
 * @brief map value to color on color rainbow
 * @param value value in range from 0 to 1
 * @return color message
 */
std_msgs::ColorRGBA getRainbowColor(const float value);

/**
 * @brief map value to color on color rainbow
 * @param value value in range from 0 to 1
 * @param[out] r resulting red value
 * @param[out] g resulting green value
 * @param[out] b resultning blue value
 */
void getRainbowColor(float value, float& r, float& g, float& b);

/**
 * Converts the orientation of a geometry_msgs/PoseStamped message to a direction vector
 * @param pose      the pose to convert
 * @return          direction normal vector
 */
mesh_map::Normal poseToDirectionVector(
      const geometry_msgs::PoseStamped& pose,
      const tf2::Vector3& axis=tf2::Vector3(1,0,0));


/**
 * Converts the position of a geometry_msgs/PoseStamped message to a position vector
 * @param pose      the pose to convert
 * @return          position vector
 */
  mesh_map::Vector poseToPositionVector(
      const geometry_msgs::PoseStamped& pose);

} /* namespace mesh_map */

#endif  // MESH_MAP__UTIL_H
