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

#include "mesh_layers/dynamic_inflation_layer.h"

#include <lvr2/util/Meap.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <mesh_map/util.h>
#include <mesh_map/timer.h>
#include <lvr2/geometry/PMPMesh.hpp>

PLUGINLIB_EXPORT_CLASS(mesh_layers::DynamicInflationLayer, mesh_map::AbstractLayer);

namespace mesh_layers
{
bool DynamicInflationLayer::readLayer()
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

bool DynamicInflationLayer::writeLayer()
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

float DynamicInflationLayer::threshold()
{
  return std::numeric_limits<float>::quiet_NaN();
}

void DynamicInflationLayer::updateLethal(std::set<lvr2::VertexHandle>& added_lethal,
                                  std::set<lvr2::VertexHandle>& removed_lethal)
{
  lethal_vertices_ = added_lethal;

  RCLCPP_INFO_STREAM(get_logger(), "Update lethal for inflation layer.");
  const auto lock = this->writeLock();
  waveCostInflation(
    lethal_vertices_,
    riskiness_,
    vector_map_
  );
}

void DynamicInflationLayer::updateInput(const rclcpp::Time& timestamp, const std::set<lvr2::VertexHandle>& changed)
{
  const mesh_map::LayerTimer::TimePoint t0 = mesh_map::LayerTimer::Clock::now();
  const std::vector<std::string> inputs = node_->get_parameter(
    mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".inputs"
  ).as_string_array();

  if (inputs.size() != 1)
  {
    RCLCPP_ERROR(get_logger(), "[InflationLayer] Exactly one input layer is required!");
    return;
  }

  auto map = map_ptr_.lock();
  if (nullptr == map)
  {
    RCLCPP_ERROR(get_logger(), "Failed to weak_ptr::lock() map in updateInput()!");
    return;
  }

  const auto input = map->layer(inputs[0]);
  if (nullptr == input)
  {
    RCLCPP_ERROR(get_logger(), "[InflationLayer] Could not get layer '%s' from map!", inputs[0].c_str());
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
  lvr2::DenseVertexMap<lvr2::BaseVector<float>> new_vectors;
  waveCostInflation(
    lethals,
    new_costs,
    new_vectors
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

inline float DynamicInflationLayer::computeUpdateSethianMethod(
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

inline bool DynamicInflationLayer::waveFrontUpdate(
  const pmp::SurfaceMesh& mesh,
  lvr2::DenseVertexMap<float>& distances,
  lvr2::DenseVertexMap<lvr2::BaseVector<float>>& vector_map,
  const float& max_distance,
  const lvr2::DenseEdgeMap<float>& edge_weights,
  const lvr2::FaceHandle& fh,
  const lvr2::BaseVector<float>& normal,
  const lvr2::VertexHandle& v1h,
  const lvr2::VertexHandle& v2h,
  const lvr2::VertexHandle& v3h
) const
{
  const double u1 = distances.containsKey(v1h) ? distances[v1h] : std::numeric_limits<float>::infinity();
  const double u2 = distances.containsKey(v2h) ? distances[v2h] : std::numeric_limits<float>::infinity();
  const double u3 = distances.containsKey(v3h) ? distances[v3h] : std::numeric_limits<float>::infinity();

  if (u3 == 0)
    return false;

  watch_.begin();
  // We should always find the edge because the 3 input vertices have to be part of the same triangle
  const lvr2::EdgeHandle e12h = mesh.find_edge(v1h, v2h);
  const lvr2::EdgeHandle e13h = mesh.find_edge(v1h, v3h);
  const lvr2::EdgeHandle e23h = mesh.find_edge(v2h, v3h);
  watch_.end_mesh();

  watch_.begin();
  // Somehow sometimes some edge does not exists??? Maybe the Mesh Topology is broken in this case
  const float c = edge_weights.containsKey(e12h) ? edge_weights[e12h] : INFINITY;
  const float c_sq = c * c;

  const float b = edge_weights.containsKey(e13h) ? edge_weights[e13h] : INFINITY;
  const float b_sq = b * b;

  const float a = edge_weights.containsKey(e23h) ? edge_weights[e23h] : INFINITY;
  const float a_sq = a * a;

  float dot = (a_sq + b_sq - c_sq) / (2 * a * b);
  float u3tmp = computeUpdateSethianMethod(u1, u2, a, b, dot, 1.0);
  watch_.end_seth();

  if (!std::isfinite(u3tmp))
    return false;

  const float d31 = u3tmp - u1;
  const float d32 = u3tmp - u2;

  if (u1 == 0 && u2 == 0)
  {
    watch_.begin();
    const auto& v1 = mesh.position(v1h);
    const auto& v2 = mesh.position(v2h);
    const auto& v3 = mesh.position(v3h);
    watch_.end_mesh();
  
    const auto& tmp = ((v3 - v2) + (v3 - v1)).normalized();
    const auto& dir = mesh_map::Vector(tmp.x(), tmp.y(), tmp.z());
    vector_map.insert(v1h, (vector_map.containsKey(v1h) ? vector_map[v1h] : mesh_map::Vector() + dir).normalized());
    vector_map.insert(v2h, (vector_map.containsKey(v2h) ? vector_map[v2h] : mesh_map::Vector() + dir).normalized());
    vector_map.insert(v3h, (vector_map.containsKey(v3h) ? vector_map[v3h] : mesh_map::Vector() + dir).normalized());
    watch_.end_vec();
  }

  if (u3tmp < u3)
  {
    distances.insert(v3h, static_cast<float>(u3tmp));
  
    watch_.begin();
    if (u1 != 0 || u2 != 0)
    {
      vector_map.insert(v3h, ((vector_map.containsKey(v1h) ? vector_map[v1h] : mesh_map::Vector()) * d31
                        + (vector_map.containsKey(v2h) ? vector_map[v2h] : mesh_map::Vector()) * d32).normalized());
    }
    watch_.end_vec();

    return u1 <= max_distance && u2 <= max_distance;
  }
  return false;
}

inline float DynamicInflationLayer::fading(const float val)
{
  if (val > config_.inflation_radius)
    return 0;

  // Inflation radius
  if (val > config_.inscribed_radius)
  {
    // Map values from [config_.inscribed_radius, config_.inflation_radius] to [0, pi]
    float alpha = ((val - config_.inscribed_radius) / (config_.inflation_radius - config_.inscribed_radius)) * M_PI;
    return config_.inscribed_value * (cos(alpha) + 1) / 2.0;
  }

  // Inscribed radius
  if (val > 0)
    return config_.inscribed_value;

  // Lethality
  return config_.lethal_value;
}

void DynamicInflationLayer::waveCostInflation(
  const std::set<lvr2::VertexHandle>& lethals,
  lvr2::DenseVertexMap<float>& cost_out,
  lvr2::DenseVertexMap<lvr2::BaseVector<float>>& vector_out
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
  const auto pmp_mesh = std::dynamic_pointer_cast<lvr2::PMPMesh<mesh_map::Vector>>(map->mesh());
  if (nullptr == pmp_mesh)
  {
    RCLCPP_ERROR(get_logger(), "Failed to dynamic_cast mesh to lvr2::PMPMesh! Currently only the lvr2::PMPMesh backend is supported!");
    return;
  }
  const auto& mesh = pmp_mesh->getSurfaceMesh();

  RCLCPP_DEBUG(get_logger(), "Inflation radius: %f", config_.inflation_radius);
  RCLCPP_DEBUG(get_logger(), "Lethals: %lu", lethals.size());

  watch_.begin();
  // Buffer is allocated in the class to prevent unnecessary reallocations
  distances_.reserve(mesh.vertices_size());
  cost_out.reserve(mesh.vertices_size());
  vector_out.reserve(mesh.vertices_size());

  const auto& edge_distances = map->edgeDistances();
  const auto& face_normals = map->faceNormals();

  lvr2::DenseVertexMap<bool> fixed(mesh.vertices_size(), false);
  watch_.end_alloc();

  // Clear the distances map
  distances_.clear();

  // Initialize vectors to 0
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
  watch_.end_init();
  RCLCPP_DEBUG(get_logger(), "Initialized wave inflation");
  RCLCPP_DEBUG(get_logger(), "Starting inflation wave front propagation");

  while (!pq.isEmpty())
  {
    watch_.begin();
    lvr2::VertexHandle current_vh = pq.popMin().key();
    auto _t1 = std::chrono::steady_clock::now();
    watch_.end_meap();
    
    if (current_vh.idx() >= mesh.vertices_size())
    {
      continue;
    }
    watch_.end_mesh();

    if (map->invalid[current_vh])
      continue;

    // check if already fixed
    // if(fixed[current_vh]) continue;
    fixed[current_vh] = true;
    for (const auto nh : mesh.vertices(current_vh))
    {
      // Current VH is now fixed, check if we can update the neighbours via one of the common faces
      const pmp::Halfedge halfedge = mesh.find_halfedge(current_vh, nh);
      for (const auto fh : {mesh.face(halfedge), mesh.face(halfedge.opposite())})
      {
        watch_.begin();
        if (!fh.is_valid())
        {
          continue;
        }
        // Using the pmp::VertexAroundFaceCirculator is slower here because it builds
        // an internal unordered set to detect loops. We know that the mesh must be a
        // valid triangle mesh, therefore we can to this ourselves
        pmp::Halfedge half_edge_handle = mesh.halfedge(fh);
        const lvr2::VertexHandle& a = mesh.to_vertex(half_edge_handle);
        half_edge_handle = mesh.next_halfedge(half_edge_handle);
        const lvr2::VertexHandle& b = mesh.to_vertex(half_edge_handle);
        half_edge_handle = mesh.next_halfedge(half_edge_handle);
        const lvr2::VertexHandle& c = mesh.to_vertex(half_edge_handle);
        watch_.end_mesh();

        if (fixed[a] && fixed[b] && fixed[c])
        {
          // RCLCPP_INFO_STREAM(get_logger(), "All fixed!");
          continue;
        }
        else if (fixed[a] && fixed[b] && !fixed[c])
        {
          // c is free
          if (waveFrontUpdate(mesh, distances_, vector_out, config_.inflation_radius, edge_distances, fh, face_normals[fh], a, b,
                              c))
          {
            watch_.begin();
            pq.insert(c, distances_[c]);
            watch_.end_meap();
          }
          // if(pq.containsKey(c)) pq.updateValue(c, distances[c]);
        }
        else if (fixed[a] && !fixed[b] && fixed[c])
        {
          // b is free
          if (waveFrontUpdate(mesh, distances_, vector_out, config_.inflation_radius, edge_distances, fh, face_normals[fh], c, a,
                              b))
          {
            watch_.begin();
            pq.insert(b, distances_[b]);
            watch_.end_meap();
          }
          // if(pq.containsKey(b)) pq.updateValue(b, distances[b]);
        }
        else if (!fixed[a] && fixed[b] && fixed[c])
        {
          // a if free
          if (waveFrontUpdate(mesh, distances_, vector_out, config_.inflation_radius, edge_distances, fh, face_normals[fh], b, c,
                              a))
          {
            watch_.begin();
            pq.insert(a, distances_[a]);
            watch_.end_meap();
          }
          // if(pq.containsKey(a)) pq.updateValue(a, distances[a]);
        }
        else
        {
          // two free vertices -> skip that face
          // RCLCPP_INFO_STREAM(get_logger(), "two vertices are free.");
          continue;
        }
      }
    }
  }

  RCLCPP_DEBUG(get_logger(), "Finished inflation wave front propagation");

  watch_.begin();
  cost_out.reserve(mesh.vertices_size());
  cost_out.clear();
  for (auto vH : distances_)
  {
    if (!std::isinf(distances_[vH]))
    {
      cost_out.insert(vH, fading(distances_[vH]));
    }
  }

  watch_.end_fading();
  watch_.commit();
}

lvr2::BaseVector<float> DynamicInflationLayer::vectorAt(const std::array<lvr2::VertexHandle, 3>& vertices,
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

lvr2::BaseVector<float> DynamicInflationLayer::vectorAt(const lvr2::VertexHandle& vH)
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

bool DynamicInflationLayer::computeLayer()
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
  
  // Prevent the input from changing while reading
  {
    const auto input_lock = input->readLock();
    lethal_vertices_ = input->lethals();
  }

  // Directly write to global state
  const auto write_lock = this->writeLock();
  waveCostInflation(
    lethal_vertices_,
    riskiness_,
    vector_map_
  );

  return true;
}
lvr2::VertexMap<float>& DynamicInflationLayer::costs()
{
  return riskiness_;
}

rcl_interfaces::msg::SetParametersResult DynamicInflationLayer::reconfigureCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  bool has_inflation_radius_changed = false;
  bool has_vector_field_parameter_changed = false;
  for (auto parameter : parameters) {
    if (parameter.get_name() == mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".inflation_radius") {
      config_.inflation_radius = parameter.as_double();
      has_inflation_radius_changed = true;
      has_vector_field_parameter_changed = true;
    } else if (parameter.get_name() == mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".inscribed_radius") {
      config_.inscribed_radius = parameter.as_double();
      has_vector_field_parameter_changed = true;
    } else if (parameter.get_name() == mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".lethal_value") {
      config_.lethal_value = parameter.as_double();
      has_vector_field_parameter_changed = true;
    } else if (parameter.get_name() == mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".inscribed_value") {
      config_.inscribed_value = parameter.as_double();
      has_vector_field_parameter_changed = true;
    } else if (parameter.get_name() == mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".min_contour_size") {
      config_.min_contour_size = parameter.as_int();
    } else if (parameter.get_name() == mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".repulsive_field") {
      config_.repulsive_field = parameter.as_bool();
    }
  }

  if (has_inflation_radius_changed){
    const auto lock = this->writeLock();
    waveCostInflation(
      lethal_vertices_,
      riskiness_,
      vector_map_
    );
  }

  if (has_vector_field_parameter_changed)
  {
    auto map = map_ptr_.lock();
    if (nullptr == map)
    {
      RCLCPP_ERROR(get_logger(), "Failed to weak_ptr::lock() map in reconfigureCallback()");
      result.successful = false;
      result.reason = "Failed to weak_ptr::lock() map!";
      return result;
    }
    map->publishVectorField("inflation", vector_map_, distances_,
                                std::bind(&mesh_layers::DynamicInflationLayer::fading, this, std::placeholders::_1));
  }

  if (has_vector_field_parameter_changed || has_inflation_radius_changed) {
    RCLCPP_INFO_STREAM(get_logger(), "Inflation layer (name: " << layer_name_ << ") changed due to cfg update.");
    notifyChange();
  }

  return result;
}

bool DynamicInflationLayer::initialize()
{
  { // inscribed_radius
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Defines the inscribed radius.";
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.01;
    range.to_value = 1.0;
    descriptor.floating_point_range.push_back(range);
    config_.inscribed_radius = node_->declare_parameter(mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".inscribed_radius", config_.inscribed_radius, descriptor);
  }
  { // inflation_radius
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Defines the maximum inflation radius.";
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.01;
    range.to_value = 3.0;
    descriptor.floating_point_range.push_back(range);
    config_.inflation_radius = node_->declare_parameter(mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".inflation_radius", config_.inflation_radius, descriptor);
  }
  { // factor
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "The factor to weight this layer";
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.0;
    range.to_value = 1.-0;
    descriptor.floating_point_range.push_back(range);
    config_.factor = node_->declare_parameter(mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".factor", config_.factor, descriptor);
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
  dyn_params_handler_ = node_->add_on_set_parameters_callback(std::bind(&DynamicInflationLayer::reconfigureCallback, this, std::placeholders::_1));
  
  watch_.open(layer_name_);

  return true;
}

} /* namespace mesh_layers */
