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

#include "mesh_layers/inflation_layer.h"

#include <queue>
#include <lvr2/util/Meap.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <mesh_map/util.h>

PLUGINLIB_EXPORT_CLASS(mesh_layers::InflationLayer, mesh_map::AbstractLayer);

namespace mesh_layers
{
bool InflationLayer::readLayer()
{
  // riskiness
  RCLCPP_INFO_STREAM(node_->get_logger(), "Try to read riskiness from map file...");
  auto riskiness_opt = mesh_io_ptr_->getDenseAttributeMap<lvr2::DenseVertexMap<float>>("riskiness");

  if (riskiness_opt)
  {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Riskiness has been read successfully.");
    riskiness_ = riskiness_opt.get();
    return true;
  }
  else
  {
    return false;
  }
}

bool InflationLayer::writeLayer()
{
  RCLCPP_INFO_STREAM(node_->get_logger(), "Saving " << riskiness_.numValues() << " riskiness values to map file...");

  if (mesh_io_ptr_->addDenseAttributeMap(riskiness_, "riskiness"))
  {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Saved riskiness to map file.");
    return true;
  }
  else
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Could not save riskiness to map file!");
    return false;
  }
}

float InflationLayer::threshold()
{
  return std::numeric_limits<float>::quiet_NaN();
}

void InflationLayer::updateLethal(std::set<lvr2::VertexHandle>& added_lethal,
                                  std::set<lvr2::VertexHandle>& removed_lethal)
{
  lethal_vertices_ = added_lethal;

  RCLCPP_INFO_STREAM(node_->get_logger(), "Update lethal for inflation layer.");
  waveCostInflation(lethal_vertices_, config_.inflation_radius, config_.inscribed_radius, config_.inscribed_value,
                    std::numeric_limits<float>::infinity());

  /*lethalCostInflation(lethal_vertices_, config_.inflation_radius,
                      config_.inscribed_radius, config_.inscribed_value,
                      config_.lethal_value == -1
                          ? std::numeric_limits<float>::infinity()
                          : config_.lethal_value);
*/
}

inline float InflationLayer::computeUpdateSethianMethod(const float& d1, const float& d2, const float& a,
                                                        const float& b, const float& dot, const float& F)
{
  float t = std::numeric_limits<float>::infinity();
  float r_cos_angle = dot;
  float r_sin_angle = sqrt(1 - dot * dot);

  float u = d2 - d1;  // T(B) - T(A)

  float f2 = a * a + b * b - 2 * a * b * r_cos_angle;
  float f1 = b * u * (a * r_cos_angle - b);
  float f0 = b * b * (u * u - F * F * a * a * r_sin_angle);

  float delta = f1 * f1 - f0 * f2;

  if (delta >= 0)
  {
    if (std::fabs(f2) > mesh_layers::EPSILON)
    {
      t = (-f1 - sqrt(delta)) / f2;
      if (t < u || b * (t - u) / t < a * r_cos_angle || a / r_cos_angle < b * (t - u) / 2)
      {
        t = (-f1 + sqrt(delta)) / f2;
      }
      else
      {
        if (f1 != 0)
        {
          t = -f0 / f1;
        }
        else
        {
          t = -std::numeric_limits<float>::infinity();
        }
      }
    }
  }
  else
  {
    t = -std::numeric_limits<float>::infinity();
  }

  if (u < t && a * r_cos_angle < b * (t - u) / t && b * (t - u) / t < a / r_cos_angle)
  {
    return t + d1;
  }
  else
  {
    return std::min(b * F + d1, a * F + d2);
  }
}

inline bool InflationLayer::waveFrontUpdate(lvr2::DenseVertexMap<float>& distances_,
                                            lvr2::DenseVertexMap<lvr2::VertexHandle>& predecessors,
                                            const float& max_distance, const lvr2::DenseEdgeMap<float>& edge_weights,
                                            const lvr2::FaceHandle& fh, const lvr2::BaseVector<float>& normal,
                                            const lvr2::VertexHandle& v1h, const lvr2::VertexHandle& v2h,
                                            const lvr2::VertexHandle& v3h)
{
  const auto& mesh = map_ptr_->mesh();

  const double u1 = distances_[v1h];
  const double u2 = distances_[v2h];
  const double u3 = distances_[v3h];

  if (u3 == 0)
    return false;

  const lvr2::OptionalEdgeHandle e12h = mesh.getEdgeBetween(v1h, v2h);
  const float c = edge_weights[e12h.unwrap()];
  const float c_sq = c * c;

  const lvr2::OptionalEdgeHandle e13h = mesh.getEdgeBetween(v1h, v3h);
  const float b = edge_weights[e13h.unwrap()];
  const float b_sq = b * b;

  const lvr2::OptionalEdgeHandle e23h = mesh.getEdgeBetween(v2h, v3h);
  const float a = edge_weights[e23h.unwrap()];
  const float a_sq = a * a;

  float dot = (a_sq + b_sq - c_sq) / (2 * a * b);
  float u3tmp = computeUpdateSethianMethod(u1, u2, a, b, dot, 1.0);

  if (!std::isfinite(u3tmp))
    return false;

  const float d31 = u3tmp - u1;
  const float d32 = u3tmp - u2;

  if (u1 == 0 && u2 == 0)
  {
    const auto& v1 = mesh_ptr_->getVertexPosition(v1h);
    const auto& v2 = mesh_ptr_->getVertexPosition(v2h);
    const auto& v3 = mesh_ptr_->getVertexPosition(v3h);
    const auto& dir = ((v3 - v2) + (v3 - v1)).normalized();
    const auto& face_normals = map_ptr_->faceNormals();
    cutting_faces_.insert(v1h, fh);
    cutting_faces_.insert(v2h, fh);
    cutting_faces_.insert(v3h, fh);
    vector_map_[v1h] = (vector_map_[v1h] + dir).normalized();
    vector_map_[v2h] = (vector_map_[v2h] + dir).normalized();
    //vector_map_[v3h] = (vector_map_[v1h] * d31 + vector_map[v2h] * d32).normalized();
    vector_map_[v3h] = (vector_map_[v3h] + dir).normalized();
  }

  if (u3tmp < u3)
  {
    distances_[v3h] = static_cast<float>(u3tmp);

    if (u1 != 0 || u2 != 0)
    {
      cutting_faces_.insert(v3h, fh);
      vector_map_[v3h] = (vector_map_[v1h] * d31 + vector_map_[v2h] * d32).normalized();
    }

    if (d31 < d32)
    {
      predecessors.insert(v3h, v1h);
    }
    else  // right face check
    {
      predecessors.insert(v3h, v2h);
    }

    // backToSource(v3h, predecessors, vector_map);
    return u1 <= max_distance && u2 <= max_distance;
  }
  return false;
}

float InflationLayer::fading(const float val)
{
  if (val > config_.inflation_radius)
    return 0;

  // Inflation radius
  if (val > config_.inscribed_radius)
  {
    float alpha = (sqrt(val) - config_.inscribed_radius) / (config_.inflation_radius - config_.inscribed_radius) * M_PI;
    return config_.inscribed_value * (cos(alpha) + 1) / 2.0;
  }

  // Inscribed radius
  if (val > 0)
    return config_.inscribed_value;

  // Lethality
  return config_.lethal_value;
}

void InflationLayer::waveCostInflation(const std::set<lvr2::VertexHandle>& lethals, const float inflation_radius,
                                       const float inscribed_radius, const float inscribed_value,
                                       const float lethal_value)
{
  if (mesh_ptr_)
  {
    auto const& mesh = *mesh_ptr_;

    RCLCPP_INFO_STREAM(node_->get_logger(), "inflation radius:" << inflation_radius);
    RCLCPP_INFO_STREAM(node_->get_logger(), "Init wave inflation.");

    lvr2::DenseVertexMap<bool> seen(mesh_ptr_->nextVertexIndex(), false);
    distances_ = lvr2::DenseVertexMap<float>(mesh_ptr_->nextVertexIndex(), std::numeric_limits<float>::infinity());
    lvr2::DenseVertexMap<lvr2::VertexHandle> predecessors;
    predecessors.reserve(mesh_ptr_->nextVertexIndex());

    vector_map_ = lvr2::DenseVertexMap<lvr2::BaseVector<float>>(mesh.nextVertexIndex(), lvr2::BaseVector<float>());

    direction_ = lvr2::DenseVertexMap<float>();

    const auto& edge_distances = map_ptr_->edgeDistances();
    const auto& face_normals = map_ptr_->faceNormals();

    lvr2::DenseVertexMap<bool> fixed(mesh.nextVertexIndex(), false);

    // initialize distances with infinity
    // initialize predecessor of each vertex with itself
    for (auto const& vH : mesh.vertices())
    {
      predecessors.insert(vH, vH);
    }

    lvr2::Meap<lvr2::VertexHandle, float> pq;
    // Set start distance to zero
    // add start vertex to priority queue
    for (auto vH : lethals)
    {
      distances_[vH] = 0;
      fixed[vH] = true;
      pq.insert(vH, 0);
    }

    RCLCPP_INFO_STREAM(node_->get_logger(), "Start inflation wave front propagation");

    while (!pq.isEmpty())
    {
      lvr2::VertexHandle current_vh = pq.popMin().key();

      if (current_vh.idx() >= mesh.nextVertexIndex())
      {
        continue;
      }

      if (map_ptr_->invalid[current_vh])
        continue;

      // check if already fixed
      // if(fixed[current_vh]) continue;
      fixed[current_vh] = true;

      std::vector<lvr2::VertexHandle> neighbours;
      try
      {
        mesh.getNeighboursOfVertex(current_vh, neighbours);
      }
      catch (lvr2::PanicException exception)
      {
        map_ptr_->invalid.insert(current_vh, true);
        continue;
      }
      catch (lvr2::VertexLoopException exception)
      {
        map_ptr_->invalid.insert(current_vh, true);
        continue;
      }

      for (auto nh : neighbours)
      {
        std::vector<lvr2::FaceHandle> faces;
        try
        {
          mesh.getFacesOfVertex(nh, faces);
        }
        catch (lvr2::PanicException exception)
        {
          map_ptr_->invalid.insert(nh, true);
          continue;
        }

        for (auto fh : faces)
        {
          const auto vertices = mesh.getVerticesOfFace(fh);
          const lvr2::VertexHandle& a = vertices[0];
          const lvr2::VertexHandle& b = vertices[1];
          const lvr2::VertexHandle& c = vertices[2];

          try
          {
            if (fixed[a] && fixed[b] && fixed[c])
            {
              // RCLCPP_INFO_STREAM(node_->get_logger(), "All fixed!");
              continue;
            }
            else if (fixed[a] && fixed[b] && !fixed[c])
            {
              // c is free
              if (waveFrontUpdate(distances_, predecessors, inflation_radius, edge_distances, fh, face_normals[fh], a, b,
                                  c))
              {
                pq.insert(c, distances_[c]);
              }
              // if(pq.containsKey(c)) pq.updateValue(c, distances[c]);
            }
            else if (fixed[a] && !fixed[b] && fixed[c])
            {
              // b is free
              if (waveFrontUpdate(distances_, predecessors, inflation_radius, edge_distances, fh, face_normals[fh], c, a,
                                  b))
              {
                pq.insert(b, distances_[b]);
              }
              // if(pq.containsKey(b)) pq.updateValue(b, distances[b]);
            }
            else if (!fixed[a] && fixed[b] && fixed[c])
            {
              // a if free
              if (waveFrontUpdate(distances_, predecessors, inflation_radius, edge_distances, fh, face_normals[fh], b, c,
                                  a))
              {
                pq.insert(a, distances_[a]);
              }
              // if(pq.containsKey(a)) pq.updateValue(a, distances[a]);
            }
            else
            {
              // two free vertices -> skip that face
              // RCLCPP_INFO_STREAM(node_->get_logger(), "two vertices are free.");
              continue;
            }
          }
          catch (lvr2::PanicException exception)
          {
            map_ptr_->invalid.insert(nh, true);
          }
          catch (lvr2::VertexLoopException exception)
          {
            map_ptr_->invalid.insert(nh, true);
          }
        }
      }
    }

    RCLCPP_INFO_STREAM(node_->get_logger(), "Finished inflation wave front propagation.");

    for (auto vH : mesh_ptr_->vertices())
    {
      riskiness_.insert(vH, fading(distances_[vH]));
    }

    map_ptr_->publishVectorField("inflation", vector_map_, distances_,
                                std::bind(&InflationLayer::fading, this, std::placeholders::_1));
  }
  else
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Cannot init wave inflation: mesh_ptr points to null");
  }
}

lvr2::BaseVector<float> InflationLayer::vectorAt(const std::array<lvr2::VertexHandle, 3>& vertices,
                                                 const std::array<float, 3>& barycentric_coords)
{
  if (!config_.repulsive_field)
    return lvr2::BaseVector<float>();

  const float distance = mesh_map::linearCombineBarycentricCoords(vertices, distances_, barycentric_coords);

  if (distance > config_.inflation_radius)
    return lvr2::BaseVector<float>();

  // Inflation radius
  if (distance > config_.inscribed_radius)
  {
    float alpha =
        (sqrt(distance) - config_.inscribed_radius) / (config_.inflation_radius - config_.inscribed_radius) * M_PI;
    return mesh_map::linearCombineBarycentricCoords(vertices, vector_map_, barycentric_coords) * config_.inscribed_value *
           (cos(alpha) + 1) / 2.0;
  }

  // Inscribed radius
  if (distance > 0)
  {
    return mesh_map::linearCombineBarycentricCoords(vertices, vector_map_, barycentric_coords) * config_.inscribed_value;
  }

  // Lethality
  return mesh_map::linearCombineBarycentricCoords(vertices, vector_map_, barycentric_coords) * config_.lethal_value;
}

lvr2::BaseVector<float> InflationLayer::vectorAt(const lvr2::VertexHandle& vH)
{
  if (!config_.repulsive_field)
    return lvr2::BaseVector<float>();

  float distance = 0;
  lvr2::BaseVector<float> vec;

  auto dist_opt = distances_.get(vH);
  auto vector_opt = vector_map_.get(vH);
  if (dist_opt && vector_opt)
  {
    distance = dist_opt.get();
    vec = vector_opt.get();
  }
  else
  {
    return vec;
  }

  if (distance > config_.inflation_radius)

    // Inflation radius
    if (distance > config_.inscribed_radius)
    {
      float alpha =
          (sqrt(distance) - config_.inscribed_radius) / (config_.inflation_radius - config_.inscribed_radius) * M_PI;
      return vec * config_.inscribed_value * (cos(alpha) + 1) / 2.0;
    }

  // Inscribed radius
  if (distance > 0)
  {
    return vec * config_.inscribed_value;
  }

  // Lethality
  return vec * config_.lethal_value;
}

void InflationLayer::backToSource(const lvr2::VertexHandle& current_vertex,
                                  const lvr2::DenseVertexMap<lvr2::VertexHandle>& predecessors,
                                  lvr2::DenseVertexMap<lvr2::BaseVector<float>>& vector_map)
{
  if (vector_map.containsKey(current_vertex))
    return;

  const auto& pre = predecessors[current_vertex];

  if (pre != current_vertex)
  {
    backToSource(pre, predecessors, vector_map);
    const auto& v0 = mesh_ptr_->getVertexPosition(current_vertex);
    const auto& v1 = mesh_ptr_->getVertexPosition(pre);
    vector_map.insert(current_vertex, v1 - v0 + vector_map[pre]);
  }
  else
  {
    vector_map.insert(current_vertex, lvr2::BaseVector<float>());
  }
}

void InflationLayer::lethalCostInflation(const std::set<lvr2::VertexHandle>& lethals, const float inflation_radius,
                                         const float inscribed_radius, const float inscribed_value,
                                         const float lethal_value)
{
  RCLCPP_INFO_STREAM(node_->get_logger(), "lethal cost inflation.");

  struct Cmp
  {
    const lvr2::DenseVertexMap<float>& distances;
    Cmp(const lvr2::DenseVertexMap<float>& distances) : distances(distances)
    {
    }

    bool operator()(lvr2::VertexHandle& left, lvr2::VertexHandle& right)
    {
      return distances[left] > distances[right];
    }
  };

  lvr2::DenseVertexMap<bool> seen(mesh_ptr_->nextVertexIndex(), false);
  lvr2::DenseVertexMap<float> distances(mesh_ptr_->nextVertexIndex(), std::numeric_limits<float>::max());
  lvr2::DenseVertexMap<lvr2::VertexHandle> prev;
  prev.reserve(mesh_ptr_->nextVertexIndex());

  for (auto vH : mesh_ptr_->vertices())
  {
    prev.insert(vH, vH);
  }

  Cmp cmp(distances);
  std::priority_queue<lvr2::VertexHandle, std::vector<lvr2::VertexHandle>, Cmp> pq(cmp);

  for (auto vH : lethals)
  {
    distances[vH] = 0;
    pq.push(vH);
  }

  float inflation_radius_squared = inflation_radius * inflation_radius;
  float inscribed_radius_squared = inscribed_radius * inscribed_radius;

  while (!pq.empty())
  {
    lvr2::VertexHandle vH = pq.top();
    pq.pop();

    if (seen[vH])
      continue;
    seen[vH] = true;

    std::vector<lvr2::VertexHandle> neighbours;
    mesh_ptr_->getNeighboursOfVertex(vH, neighbours);
    for (auto n : neighbours)
    {
      if (distances[n] == 0 || seen[n])
        continue;
      float n_dist = mesh_ptr_->getVertexPosition(n).squaredDistanceFrom(mesh_ptr_->getVertexPosition(prev[vH]));
      if (n_dist < distances[n] && n_dist < inflation_radius_squared)
      {
        prev[n] = prev[vH];
        distances[n] = n_dist;
        pq.push(n);
      }
    }
  }

  for (auto vH : mesh_ptr_->vertices())
  {
    if (distances[vH] > inflation_radius_squared)
    {
      riskiness_.insert(vH, 0);
    }

    // Inflation radius
    else if (distances[vH] > inscribed_radius_squared)
    {
      float alpha = (sqrt(distances[vH]) - inscribed_radius) / (inflation_radius - inscribed_radius) * M_PI;
      riskiness_.insert(vH, inscribed_value * (cos(alpha) + 1) / 2.0);
    }

    // Inscribed radius
    else if (distances[vH] > 0)
    {
      riskiness_.insert(vH, inscribed_value);
    }

    // Lethality
    else
    {
      riskiness_.insert(vH, lethal_value);
    }
  }

  RCLCPP_INFO_STREAM(node_->get_logger(), "lethal cost inflation finished.");
}

bool InflationLayer::computeLayer()
{
  /*waveCostInflation(lethal_vertices_, config_.inflation_radius,
                      config_.inscribed_radius, config_.inscribed_value,
                      std::numeric_limits<float>::infinity());
  */
  // lethalCostInflation(lethal_vertices_, config_.inflation_radius,
  //                    config_.inscribed_radius, config_.inscribed_value,
  //                    std::numeric_limits<float>::infinity());
  // config_.lethal_value);

  return true;
}
lvr2::VertexMap<float>& InflationLayer::costs()
{
  return riskiness_;
}

rcl_interfaces::msg::SetParametersResult InflationLayer::reconfigureCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  bool has_inflation_radius_changed = false;
  bool has_vector_field_parameter_changed = false;
  for (auto parameter : parameters) {
    if (parameter.get_name() == layer_name_ + ".inflation_radius") {
      config_.inflation_radius = parameter.as_double();
      has_inflation_radius_changed = true;
      has_vector_field_parameter_changed = true;
    } else if (parameter.get_name() == layer_name_ + ".inscribed_radius") {
      config_.inscribed_radius = parameter.as_double();
      has_vector_field_parameter_changed = true;
    } else if (parameter.get_name() == layer_name_ + ".lethal_value") {
      config_.lethal_value = parameter.as_double();
      has_vector_field_parameter_changed = true;
    } else if (parameter.get_name() == layer_name_ + ".inscribed_value") {
      config_.inscribed_value = parameter.as_double();
      has_vector_field_parameter_changed = true;
    } else if (parameter.get_name() == layer_name_ + ".min_contour_size") {
      config_.min_contour_size = parameter.as_int();
    } else if (parameter.get_name() == layer_name_ + ".repulsive_field") {
      config_.repulsive_field = parameter.as_bool();
    }
  }

  if (has_inflation_radius_changed){
    waveCostInflation(lethal_vertices_, config_.inflation_radius, config_.inscribed_radius, config_.inscribed_value, std::numeric_limits<float>::infinity());
  }

  if (has_vector_field_parameter_changed)
  {
    map_ptr_->publishVectorField("inflation", vector_map_, distances_,
                                std::bind(&mesh_layers::InflationLayer::fading, this, std::placeholders::_1));
  }

  if (has_vector_field_parameter_changed || has_inflation_radius_changed) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Inflation layer (name: " << layer_name_ << ") changed due to cfg update.");
    notifyChange();
  }

  return result;
}

bool InflationLayer::initialize()
{
  { // inscribed_radius
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Defines the inscribed radius.";
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.01;
    range.to_value = 1.0;
    descriptor.floating_point_range.push_back(range);
    config_.inscribed_radius = node_->declare_parameter(layer_name_ + ".inscribed_radius", config_.inscribed_radius);
  }
  { // inflation_radius
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Defines the maximum inflation radius.";
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.01;
    range.to_value = 3.0;
    descriptor.floating_point_range.push_back(range);
    config_.inflation_radius = node_->declare_parameter(layer_name_ + ".inflation_radius", config_.inflation_radius);
  }
  { // factor
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "The factor to weight this layer";
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.0;
    range.to_value = 1.-0;
    descriptor.floating_point_range.push_back(range);
    config_.factor = node_->declare_parameter(layer_name_ + ".factor", config_.factor);
  }
  { // lethal_value
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Defines the 'lethal' value for obstacles. -1 results in infinity";
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = -1.0;
    range.to_value = 100000.0;
    descriptor.floating_point_range.push_back(range);
    config_.lethal_value = node_->declare_parameter(layer_name_ + ".lethal_value", config_.lethal_value);
  }
  { // inscribed_value
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Defines the 'inscribed' value for obstacles.";
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.0;
    range.to_value = 100000.0;
    descriptor.floating_point_range.push_back(range);
    config_.inscribed_value = node_->declare_parameter(layer_name_ + ".inscribed_value", config_.inscribed_value);
  }
  { // min_contour_size
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Defines the minimum size for a contour to be classified as 'lethal'.";
    rcl_interfaces::msg::IntegerRange range;
    range.from_value = 0;
    range.to_value = 100000;
    descriptor.integer_range.push_back(range);
    config_.min_contour_size = node_->declare_parameter(layer_name_ + ".min_contour_size", config_.min_contour_size);
  }
  { // repulsive_field
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Enable the repulsive vector field.";
    config_.repulsive_field = node_->declare_parameter(layer_name_ + ".repulsive_field", config_.repulsive_field);
  }
  dyn_params_handler_ = node_->add_on_set_parameters_callback(std::bind(&InflationLayer::reconfigureCallback, this, std::placeholders::_1));
  return true;
}

} /* namespace mesh_layers */
