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
#include <mesh_map/mesh_map.h>

#include <lvr2/util/Meap.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <mesh_map/util.h>
#include <mesh_map/timer.h>

PLUGINLIB_EXPORT_CLASS(mesh_layers::InflationLayer, mesh_map::AbstractLayer);

namespace mesh_layers
{
bool InflationLayer::readLayer()
{
  return false;
  // Since this is called by the map we do not need to check if this is successful
  const auto map = map_ptr_.lock();
  // riskiness
  RCLCPP_INFO_STREAM(get_logger(), "Try to read riskiness from map file...");
  auto mesh_io = map->meshIO();
  auto riskiness_opt = mesh_io->getDenseAttributeMap<lvr2::DenseVertexMap<float>>(layer_name_);
  if (riskiness_opt)
  {
    RCLCPP_INFO_STREAM(get_logger(), "Riskiness has been read successfully.");
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
  RCLCPP_INFO_STREAM(get_logger(), "Saving " << riskiness_.numValues() << " riskiness values to map file...");
  // Since this is called by the map we do not need to check if this is successful
  const auto map = map_ptr_.lock();

  auto mesh_io = map->meshIO();
  if (mesh_io->addDenseAttributeMap(riskiness_, layer_name_))
  {
    RCLCPP_INFO_STREAM(get_logger(), "Saved riskiness to map file.");
    return true;
  }
  else
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Could not save riskiness to map file!");
    return false;
  }
}

float InflationLayer::threshold()
{
  return std::numeric_limits<float>::quiet_NaN();
}


void InflationLayer::onInputChanged(
  const rclcpp::Time& timestamp,
  const std::set<lvr2::VertexHandle>& changed
)
{
  const mesh_map::LayerTimer::TimePoint t0 = mesh_map::LayerTimer::Clock::now();
  const std::vector<std::string> inputs = node_->get_parameter(
    mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".inputs"
  ).as_string_array();

  if (inputs.size() != 1)
  {
    RCLCPP_ERROR(get_logger(), "Exactly one input layer is required!");
    return;
  }

  auto map = map_ptr_.lock();
  if (nullptr == map)
  {
    RCLCPP_ERROR(get_logger(), "Failed to weak_ptr::lock() map in onInputChanged()!");
    return;
  }

  const auto input = map->layer(inputs[0]);
  if (nullptr == input)
  {
    RCLCPP_ERROR(get_logger(), "Could not get layer '%s' from map!", inputs[0].c_str());
    return;
  }

  // Copy lethal vertices of base layer
  mesh_map::LayerTimer::TimePoint t1 = mesh_map::LayerTimer::Clock::now();
  std::set<lvr2::VertexHandle> lethals;
  {
    const auto input_lock = input->readLock();
    t1 = mesh_map::LayerTimer::Clock::now();
    lethals = input->lethals();
  }

  // TODO: waveCostInflation uses the instance buffer to store distances and is
  // therefore not thread safe! We protect the public state (riskiness_, vector_map_
  // and lethals) against concurrent access but not waveCostInflation. We could
  // use another mutex to allow only one thread to enter this updateChanged method
  // and allocate all used buffers in the class (new_costs, new_vectors)
  lvr2::DenseVertexMap<float> new_costs;
  lvr2::DenseVertexMap<mesh_map::Vector> new_vectors;
  waveCostInflation(
    lethals,
    new_costs,
    new_vectors,
    config_.inflation_radius,
    config_.inscribed_radius,
    config_.inscribed_value,
    config_.lethal_value
  );

  // Figure out which vertices changed their costs
  std::set<lvr2::VertexHandle> update;
  {
    const auto rlock = this->readLock();
    for (const lvr2::VertexHandle& v: new_costs)
    {
      update.insert(v);
    }
    for (const lvr2::VertexHandle& v: riskiness_)
    {
      update.insert(v);
    }
  }

  // Update the public state
  {
    const auto wlock = this->writeLock();
    lethal_vertices_ = std::move(lethals);
    riskiness_ = std::move(new_costs);
    vector_map_ = std::move(new_vectors);
  }

  const mesh_map::LayerTimer::TimePoint t2 = mesh_map::LayerTimer::Clock::now();
  this->notifyChange(timestamp, update);
  const mesh_map::LayerTimer::TimePoint t3 = mesh_map::LayerTimer::Clock::now();
  mesh_map::LayerTimer::recordUpdateDuration(layer_name_, timestamp, t1 - t0, t2 - t1, t3 - t2);
}

inline float InflationLayer::computeUpdateSethianMethod(
  const float& d1, const float& d2,
  const float& a, const float& b,
  const float& dot, const float& F
) const
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

bool InflationLayer::waveFrontUpdate(
  const lvr2::PMPMesh<mesh_map::Vector>& mesh,
  lvr2::DenseVertexMap<float>& distances,
  lvr2::DenseVertexMap<mesh_map::Vector>& vector_map,
  const float& max_distance,
  const lvr2::DenseEdgeMap<float>& edge_weights,
  const lvr2::VertexHandle& v1h,
  const lvr2::VertexHandle& v2h,
  const lvr2::VertexHandle& v3h
) const
{
  const float inf = INFINITY;
  const float u1 = std::as_const(distances).get(v1h).value_or(inf);
  const float u2 = std::as_const(distances).get(v2h).value_or(inf);
  const float u3 = std::as_const(distances).get(v3h).value_or(inf);

  if (u3 == 0)
    return false;

  const lvr2::OptionalEdgeHandle e12h = mesh.getEdgeBetween(v1h, v2h);
  const lvr2::OptionalEdgeHandle e13h = mesh.getEdgeBetween(v1h, v3h);
  const lvr2::OptionalEdgeHandle e23h = mesh.getEdgeBetween(v2h, v3h);

  const float c = edge_weights.get(e12h.unwrap()).value_or(inf);
  const float c_sq = c * c;

  const float b = edge_weights.get(e13h.unwrap()).value_or(inf);
  const float b_sq = b * b;

  const float a = edge_weights.get(e23h.unwrap()).value_or(inf);
  const float a_sq = a * a;

  float dot = (a_sq + b_sq - c_sq) / (2 * a * b);
  float u3tmp = computeUpdateSethianMethod(u1, u2, a, b, dot, 1.0);

  if (!std::isfinite(u3tmp))
    return false;

  const float d31 = u3tmp - u1;
  const float d32 = u3tmp - u2;

  if (u1 == 0 && u2 == 0)
  {
    const auto& v1 = mesh.getVertexPosition(v1h);
    const auto& v2 = mesh.getVertexPosition(v2h);
    const auto& v3 = mesh.getVertexPosition(v3h);
    const auto& dir = ((v3 - v2) + (v3 - v1)).normalized();

    // Declaring the zero vector is necessary because boost cannot
    // bind rvalue references to lvalue references. The std::as_const is
    // necessary to call the const overload of lvr2::AttributeMap::get()
    // otherwise the zero vector would have to be mutable.
    // This is all due to the boost's support for optional types containing references!
    // Also the lvr2 maps do not provide a const get overload with provided default value
    // e.g.: vector_map.get(v1h, Vector()) <-- this would be much nicer to write and read;
    const mesh_map::Vector zero;
    vector_map.insert(v1h, (std::as_const(vector_map).get(v1h).value_or(zero) + dir).normalized());
    vector_map.insert(v2h, (std::as_const(vector_map).get(v2h).value_or(zero) + dir).normalized());
    vector_map.insert(v3h, (std::as_const(vector_map).get(v3h).value_or(zero) + dir).normalized());
  }

  if (u3tmp < u3)
  {
    distances.insert(v3h, static_cast<float>(u3tmp));

    if (u1 != 0 || u2 != 0)
    {
      // The zero vector is needed for the same reason as above
      const mesh_map::Vector zero;
      const mesh_map::Vector& vec_a = std::as_const(vector_map).get(v1h).value_or(zero);
      const mesh_map::Vector& vec_b = std::as_const(vector_map).get(v2h).value_or(zero);
      vector_map.insert(v3h, (vec_a * d31 + vec_b * d32).normalized());
    }

    return u1 <= max_distance && u2 <= max_distance;
  }
  return false;
}

float InflationLayer::fading(const float squared_distance)
{
  const float distance = sqrt(squared_distance);

  if (distance > config_.inflation_radius)
  {
    return 0;
  } 
  
  // <= Inflation radius
  if (distance > config_.inscribed_radius) 
  {
    // http://wiki.ros.org/costmap_2d - cost decay function
    const float factor = exp(-1.0f * config_.cost_scaling_factor * (distance - config_.inscribed_radius));
    const float cost = config_.inscribed_value * factor;
    return cost;
  }

  // <= Inscribed radius
  if (distance > 0)
  {
    return config_.inscribed_value;
  }

  // Lethality
  return config_.lethal_value;
}

void InflationLayer::waveCostInflation(
  const std::set<lvr2::VertexHandle>& lethals,
  lvr2::DenseVertexMap<float>& cost_out,
  lvr2::DenseVertexMap<mesh_map::Vector>& vector_out,
  const float inflation_radius,
  const float inscribed_radius,
  const float inscribed_value,
  const float lethal_value
)
{
  const auto map = map_ptr_.lock();
  if (nullptr == map)
  {
    RCLCPP_ERROR(get_logger(), "Failed to weak_ptr::lock() the map in waveCostInflation()!");
    return;
  }

  if (nullptr == map->mesh())
  {
    RCLCPP_ERROR(get_logger(), "Cannot init wave inflation: mesh_ptr points to null");
    return;
  }

  // The LVR2 PMP wrapper
  const auto mesh = std::dynamic_pointer_cast<lvr2::PMPMesh<mesh_map::Vector>>(map->mesh());
  if (nullptr == mesh)
  {
    RCLCPP_ERROR(get_logger(), "Failed to dynamic_cast mesh to lvr2::PMPMesh! Currently only the lvr2::PMPMesh backend is supported!");
    return;
  }

  // The PMP half-edge-mesh datastructure
  const pmp::SurfaceMesh& pmp_mesh = mesh->getSurfaceMesh();

  RCLCPP_DEBUG(get_logger(), "Inflation radius: %f", config_.inflation_radius);
  RCLCPP_DEBUG(get_logger(), "Lethals: %lu", lethals.size());

  // Buffer is allocated in the class to prevent unnecessary reallocations
  distances_.reserve(mesh->nextVertexIndex());
  cost_out.reserve(mesh->nextVertexIndex());
  vector_out.reserve(mesh->nextVertexIndex());

  const auto& edge_distances = map->edgeDistances();
  const auto& face_normals = map->faceNormals();

  lvr2::DenseVertexMap<bool> fixed(mesh->nextVertexIndex(), false);

  // Clear the distances map
  distances_.clear();

  // Clear the vectors map
  vector_out.clear();

  lvr2::Meap<lvr2::VertexHandle, float> pq;
  // Set start distance to zero
  // add start vertex to priority queue
  for (auto vH : lethals)
  {
    distances_.insert(vH, 0.0);
    fixed[vH] = true;
    pq.insert(vH, 0);
  }

  RCLCPP_DEBUG(get_logger(), "Initialized wave inflation");
  RCLCPP_DEBUG(get_logger(), "Starting inflation wave front propagation");

  while (!pq.isEmpty())
  {
    lvr2::VertexHandle current_vh = pq.popMin().key();
    auto _t1 = std::chrono::steady_clock::now();
    
    if (current_vh.idx() >= mesh->nextVertexIndex())
    {
      continue;
    }

    if (map->invalid[current_vh])
      continue;

    // check if already fixed
    // if(fixed[current_vh]) continue;
    fixed[current_vh] = true;
    for (const auto nh : pmp_mesh.vertices(current_vh))
    {
      // Current VH is now fixed, check if we can update the neighbours via one of the common faces
      const pmp::Halfedge halfedge = pmp_mesh.find_halfedge(current_vh, nh);
      for (const auto fh : {pmp_mesh.face(halfedge), pmp_mesh.face(halfedge.opposite())})
      {
        if (!fh.is_valid())
        {
          continue;
        }
        // Using the pmp::VertexAroundFaceCirculator is slower here because it builds
        // an internal unordered set to detect loops. We know that the mesh must be a
        // valid triangle mesh, therefore we can to this ourselves
        pmp::Halfedge half_edge_handle = pmp_mesh.halfedge(fh);
        const lvr2::VertexHandle& a = pmp_mesh.to_vertex(half_edge_handle);
        half_edge_handle = pmp_mesh.next_halfedge(half_edge_handle);
        const lvr2::VertexHandle& b = pmp_mesh.to_vertex(half_edge_handle);
        half_edge_handle = pmp_mesh.next_halfedge(half_edge_handle);
        const lvr2::VertexHandle& c = pmp_mesh.to_vertex(half_edge_handle);

        if (fixed[a] && fixed[b] && fixed[c])
        {
          continue;
        }
        else if (fixed[a] && fixed[b] && !fixed[c])
        {
          // c is free
          if (waveFrontUpdate(*mesh, distances_, vector_out, config_.inflation_radius, edge_distances, a, b, c))
          {
            pq.insert(c, distances_[c]);
          }
        }
        else if (fixed[a] && !fixed[b] && fixed[c])
        {
          // b is free
          if (waveFrontUpdate(*mesh, distances_, vector_out, config_.inflation_radius, edge_distances, c, a, b))
          {
            pq.insert(b, distances_[b]);
          }
        }
        else if (!fixed[a] && fixed[b] && fixed[c])
        {
          // a if free
          if (waveFrontUpdate(*mesh, distances_, vector_out, config_.inflation_radius, edge_distances, b, c, a))
          {
            pq.insert(a, distances_[a]);
          }
        }
        else
        {
          // two free vertices -> skip that face
          continue;
        }
      }
    }
  }

  RCLCPP_DEBUG(get_logger(), "Finished inflation wave front propagation.");

  cost_out.reserve(mesh->nextVertexIndex());
  cost_out.clear();
  for (auto vH : distances_)
  {
    if (!std::isinf(distances_[vH]))
    {
      cost_out.insert(vH, fading(distances_[vH]));
    }
  }
}

mesh_map::Vector InflationLayer::vectorAt(const std::array<lvr2::VertexHandle, 3>& vertices,
                                                 const std::array<float, 3>& barycentric_coords)
{
  if (!config_.repulsive_field)
    return mesh_map::Vector();

  const float distance = mesh_map::linearCombineBarycentricCoords(vertices, distances_, barycentric_coords);

  if (distance > config_.inflation_radius)
    return mesh_map::Vector();

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

mesh_map::Vector InflationLayer::vectorAt(const lvr2::VertexHandle& vH)
{
  if (!config_.repulsive_field)
    return mesh_map::Vector();

  float distance = 0;
  mesh_map::Vector vec;

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

bool InflationLayer::computeLayer()
{
  const std::vector<std::string> inputs = node_->get_parameter(
    mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".inputs"
  ).as_string_array();

  if (inputs.size() != 1)
  {
    RCLCPP_ERROR(get_logger(), "[InflationLayer] Exactly one input layer is required!");
    return false;
  }
  
  const auto map = map_ptr_.lock();
  const auto input = map->layer(inputs[0]);
  if (nullptr == input)
  {
    RCLCPP_ERROR(get_logger(), "[InflationLayer] Could not get layer '%s' from map!", inputs[0].c_str());
    return false;
  }
  
  // Copy lethal vertices of base layer
  lethal_vertices_ = input->lethals();
  waveCostInflation(
    lethal_vertices_,
    riskiness_,
    vector_map_,
    config_.inflation_radius,
    config_.inscribed_radius,
    config_.inscribed_value,
    config_.lethal_value
  );

  return true;
}

const lvr2::VertexMap<float>& InflationLayer::costs()
{
  return riskiness_;
}

rcl_interfaces::msg::SetParametersResult InflationLayer::reconfigureCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  bool recompute_inflation_costs = false;

  for (auto parameter : parameters) {
    if (parameter.get_name() == mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".inflation_radius") {
      config_.inflation_radius = parameter.as_double();
      recompute_inflation_costs = true;
    } else if (parameter.get_name() == mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".inscribed_radius") {
      config_.inscribed_radius = parameter.as_double();
      recompute_inflation_costs = true;
    } else if (parameter.get_name() == mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".lethal_value") {
      config_.lethal_value = parameter.as_double();
      recompute_inflation_costs = true;
    } else if (parameter.get_name() == mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".inscribed_value") {
      config_.inscribed_value = parameter.as_double();
      recompute_inflation_costs = true;
    } else if (parameter.get_name() == mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".cost_scaling_factor") {
      config_.cost_scaling_factor = parameter.as_double();
      recompute_inflation_costs = true;
    } else if (parameter.get_name() == mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".min_contour_size") {
      config_.min_contour_size = parameter.as_int();
      recompute_inflation_costs = true;
    } else if (parameter.get_name() == mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".repulsive_field") {
      config_.repulsive_field = parameter.as_bool();
      recompute_inflation_costs = true;
    }
  }

  if (recompute_inflation_costs)
  {
    {
      const auto wlock = this->writeLock();
      waveCostInflation(
        lethal_vertices_,
        riskiness_,
        vector_map_,
        config_.inflation_radius,
        config_.inscribed_radius,
        config_.inscribed_value,
        config_.lethal_value
      );
    }
    auto map = map_ptr_.lock();
    if (nullptr == map)
    {
      RCLCPP_ERROR(get_logger(), "Failed to weak_ptr::lock() map in reconfigureCallback()");
      result.successful = false;
      result.reason = "Failed to weak_ptr::lock() map!";
      return result;
    }
    {
      const auto rlock = this->readLock();
      map->publishVectorField("inflation", vector_map_, distances_,
                              std::bind(&mesh_layers::InflationLayer::fading, this, std::placeholders::_1));
    }
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
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.01;
    range.to_value = 5.0;
    descriptor.floating_point_range.push_back(range);
    config_.inscribed_radius = node_->declare_parameter(mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".inscribed_radius", config_.inscribed_radius, descriptor);
  }
  { // inflation_radius
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Defines the maximum inflation radius.";
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.01;
    range.to_value = 10.0;
    descriptor.floating_point_range.push_back(range);
    config_.inflation_radius = node_->declare_parameter(mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".inflation_radius", config_.inflation_radius, descriptor);
  }
  { // lethal_value
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Defines the 'lethal' value for obstacles. -1 results in infinity";
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = -1.0;
    range.to_value = 100000.0;
    descriptor.floating_point_range.push_back(range);
    config_.lethal_value = node_->declare_parameter(mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".lethal_value", config_.lethal_value, descriptor);
  }
  { // inscribed_value
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Defines the 'inscribed' value for obstacles.";
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.0;
    range.to_value = 100000.0;
    descriptor.floating_point_range.push_back(range);
    config_.inscribed_value = node_->declare_parameter(mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".inscribed_value", config_.inscribed_value, descriptor);
  }
  { // cost_scaling_factor
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Defines the cost scaling factor for the exponential fading outside the inscribed radius.";
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.0;
    range.to_value = 5.0;
    descriptor.floating_point_range.push_back(range);
    config_.cost_scaling_factor = node_->declare_parameter(mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".cost_scaling_factor", config_.cost_scaling_factor, descriptor);
  }
  { // min_contour_size
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Defines the minimum size for a contour to be classified as 'lethal'.";
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    rcl_interfaces::msg::IntegerRange range;
    range.from_value = 0;
    range.to_value = 100000;
    descriptor.integer_range.push_back(range);
    config_.min_contour_size = node_->declare_parameter(mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".min_contour_size", config_.min_contour_size, descriptor);
  }
  { // repulsive_field
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Enable the repulsive vector field.";
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    config_.repulsive_field = node_->declare_parameter(mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".repulsive_field", config_.repulsive_field, descriptor);
  }
  dyn_params_handler_ = node_->add_on_set_parameters_callback(std::bind(&InflationLayer::reconfigureCallback, this, std::placeholders::_1));
  return true;
}

} /* namespace mesh_layers */
