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
  waveCostInflation(lethal_vertices_, config_.inflation_radius, config_.inscribed_radius, config_.inscribed_value,
                    std::numeric_limits<float>::infinity());
}

void DynamicInflationLayer::updateInput(const std::set<lvr2::VertexHandle>& changed)
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


  // TODO: The inflation algorithm overwrites all costs
  // Copy all changed costs from the input layer
  // for (const lvr2::VertexHandle& v: changed)
  // {
  //   riskiness_.insert(v, input->costs()[v]);
  // }

  auto wlock = this->writeLock();
  // TODO: This layer should probably be sparse?
  std::set<lvr2::VertexHandle> old;
  for (const lvr2::VertexHandle& v: riskiness_)
  {
    if (riskiness_[v] > 0.0)
    {
      old.insert(v);
    }
  }
  
  // Copy lethal vertices of base layer
  // TODO: Copy all cost values
  // TODO: Implement update on change
  mesh_map::LayerTimer::TimePoint t1 = mesh_map::LayerTimer::Clock::now();
  {
    auto input_lock = input->readLock();
    t1 = mesh_map::LayerTimer::Clock::now();
    lethal_vertices_ = input->lethals();
  }
  waveCostInflation(
    lethal_vertices_,
    config_.inflation_radius,
    config_.inscribed_radius,
    config_.inscribed_value,
    1.0
  );

  std::set<lvr2::VertexHandle> new_;
  for (const lvr2::VertexHandle& v: riskiness_)
  {
    if (riskiness_[v] > 0.0)
    {
      new_.insert(v);
    }
  }
  // TODO: Is this performant?
  std::set<lvr2::VertexHandle> update;
  // All vertices which are now part of the layer and all old vertices probably changed their costs
  std::set_union(
    old.begin(), old.end(),
    new_.begin(), new_.end(),
    std::inserter(update, update.end())
  );
  // Unlock before notifying others
  wlock.unlock();
  const mesh_map::LayerTimer::TimePoint t2 = mesh_map::LayerTimer::Clock::now();
  this->notifyChange(update);
  const mesh_map::LayerTimer::TimePoint t3 = mesh_map::LayerTimer::Clock::now();
  mesh_map::LayerTimer::recordUpdateDuration(layer_name_, t0, t1 - t0, t2 - t1, t3 - t2);
}

inline float DynamicInflationLayer::computeUpdateSethianMethod(const float& d1, const float& d2, const float& a,
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

inline bool DynamicInflationLayer::waveFrontUpdate(
  const lvr2::BaseMesh<mesh_map::Vector>& mesh,
  lvr2::DenseVertexMap<float>& distances_,
  lvr2::DenseVertexMap<lvr2::VertexHandle>& predecessors,
  const float& max_distance,
  const lvr2::DenseEdgeMap<float>& edge_weights,
  const lvr2::FaceHandle& fh,
  const lvr2::BaseVector<float>& normal,
  const lvr2::VertexHandle& v1h,
  const lvr2::VertexHandle& v2h,
  const lvr2::VertexHandle& v3h
)
{
  const double u1 = distances_[v1h];
  const double u2 = distances_[v2h];
  const double u3 = distances_[v3h];

  if (u3 == 0)
    return false;

  watch_.begin();
  const lvr2::OptionalEdgeHandle e12h = mesh.getEdgeBetween(v1h, v2h);
  const lvr2::OptionalEdgeHandle e13h = mesh.getEdgeBetween(v1h, v3h);
  const lvr2::OptionalEdgeHandle e23h = mesh.getEdgeBetween(v2h, v3h);
  watch_.end_mesh();

  watch_.begin();
  const float c = edge_weights[e12h.unwrap()];
  const float c_sq = c * c;

  const float b = edge_weights[e13h.unwrap()];
  const float b_sq = b * b;

  const float a = edge_weights[e23h.unwrap()];
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
    const auto& v1 = mesh.getVertexPosition(v1h);
    const auto& v2 = mesh.getVertexPosition(v2h);
    const auto& v3 = mesh.getVertexPosition(v3h);
    watch_.end_mesh();
  
    const auto& dir = ((v3 - v2) + (v3 - v1)).normalized();
    cutting_faces_.insert(v1h, fh);
    cutting_faces_.insert(v2h, fh);
    cutting_faces_.insert(v3h, fh);
    vector_map_[v1h] = (vector_map_[v1h] + dir).normalized();
    vector_map_[v2h] = (vector_map_[v2h] + dir).normalized();
    //vector_map_[v3h] = (vector_map_[v1h] * d31 + vector_map[v2h] * d32).normalized();
    vector_map_[v3h] = (vector_map_[v3h] + dir).normalized();
    watch_.end_vec();
  }

  if (u3tmp < u3)
  {
    distances_[v3h] = static_cast<float>(u3tmp);
  
    watch_.begin();
    if (u1 != 0 || u2 != 0)
    {
      cutting_faces_.insert(v3h, fh);
      vector_map_[v3h] = (vector_map_[v1h] * d31 + vector_map_[v2h] * d32).normalized();
    }
    watch_.end_vec();

    if (d31 < d32)
    {
      predecessors.insert(v3h, v1h);
    }
    else  // right face check
    {
      predecessors.insert(v3h, v2h);
    }

    return u1 <= max_distance && u2 <= max_distance;
  }
  return false;
}

float DynamicInflationLayer::fading(const float val)
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
  // const auto pmp_mesh = std::dynamic_pointer_cast<lvr2::PMPMesh<mesh_map::Vector>>(map->mesh());
  const auto mesh = map->mesh();

  RCLCPP_INFO_STREAM(get_logger(), "inflation radius:" << inflation_radius);
  RCLCPP_INFO_STREAM(get_logger(), "Init wave inflation.");

  watch_.begin();

  lvr2::DenseVertexMap<bool> seen(mesh->nextVertexIndex(), false);
  distances_ = lvr2::DenseVertexMap<float>(mesh->nextVertexIndex(), std::numeric_limits<float>::infinity());
  lvr2::DenseVertexMap<lvr2::VertexHandle> predecessors;
  predecessors.reserve(mesh->nextVertexIndex());
  vector_map_ = lvr2::DenseVertexMap<lvr2::BaseVector<float>>(mesh->nextVertexIndex(), lvr2::BaseVector<float>());
  direction_ = lvr2::DenseVertexMap<float>();

  const auto& edge_distances = map->edgeDistances();
  const auto& face_normals = map->faceNormals();

  lvr2::DenseVertexMap<bool> fixed(mesh->nextVertexIndex(), false);
  watch_.end_alloc();

  // initialize distances with infinity
  // initialize predecessor of each vertex with itself
  for (auto const& vH : mesh->vertices())
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
  watch_.end_init();
  RCLCPP_INFO_STREAM(get_logger(), "Start inflation wave front propagation");

  // Preallocate the buffer with reasonable size (avg. vertex valence is 6)
  std::vector<lvr2::VertexHandle> neighbours(10);
  std::vector<lvr2::FaceHandle> faces(10);
  while (!pq.isEmpty())
  {
    watch_.begin();
    lvr2::VertexHandle current_vh = pq.popMin().key();
    auto _t1 = std::chrono::steady_clock::now();
    watch_.end_meap();
    
    if (current_vh.idx() >= mesh->nextVertexIndex())
    {
      continue;
    }
    watch_.end_mesh();

    if (map->invalid[current_vh])
      continue;

    // check if already fixed
    // if(fixed[current_vh]) continue;
    fixed[current_vh] = true;
    neighbours.clear();
    try
    {
      watch_.begin();
      mesh->getNeighboursOfVertex(current_vh, neighbours);
      watch_.end_mesh();
    }
    catch (lvr2::PanicException exception)
    {
      map->invalid.insert(current_vh, true);
      continue;
    }
    catch (lvr2::VertexLoopException exception)
    {
      map->invalid.insert(current_vh, true);
      continue;
    }

    for (auto nh : neighbours)
    {
      faces.clear();
      try
      {
        watch_.begin();
        mesh->getFacesOfVertex(nh, faces);
        watch_.end_mesh();
      }
      catch (lvr2::PanicException exception)
      {
        map->invalid.insert(nh, true);
        continue;
      }

      for (auto fh : faces)
      {
        watch_.begin();
        const auto vertices = mesh->getVerticesOfFace(fh);
        watch_.end_mesh();
        const lvr2::VertexHandle& a = vertices[0];
        const lvr2::VertexHandle& b = vertices[1];
        const lvr2::VertexHandle& c = vertices[2];

        try
        {
          if (fixed[a] && fixed[b] && fixed[c])
          {
            // RCLCPP_INFO_STREAM(get_logger(), "All fixed!");
            continue;
          }
          else if (fixed[a] && fixed[b] && !fixed[c])
          {
            // c is free
            if (waveFrontUpdate(*mesh, distances_, predecessors, inflation_radius, edge_distances, fh, face_normals[fh], a, b,
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
            if (waveFrontUpdate(*mesh, distances_, predecessors, inflation_radius, edge_distances, fh, face_normals[fh], c, a,
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
            if (waveFrontUpdate(*mesh, distances_, predecessors, inflation_radius, edge_distances, fh, face_normals[fh], b, c,
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
        catch (lvr2::PanicException exception)
        {
          map->invalid.insert(nh, true);
        }
        catch (lvr2::VertexLoopException exception)
        {
          map->invalid.insert(nh, true);
        }
      }
    }
  }

  RCLCPP_INFO_STREAM(get_logger(), "Finished inflation wave front propagation.");

  watch_.begin();
  for (auto vH : mesh->vertices())
  {
    riskiness_.insert(vH, fading(distances_[vH]));
  }
  watch_.end_fading();

  // TODO: Why is this published?? Seems to be expensive
  map->publishVectorField("inflation", vector_map_, distances_,
                              std::bind(&DynamicInflationLayer::fading, this, std::placeholders::_1));
  watch_.end_pub();
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
  
  // Copy lethal vertices of base layer
  // TODO: Copy all cost values
  // TODO: Implement update on change
  lethal_vertices_ = input->lethals();
  waveCostInflation(
    lethal_vertices_,
    config_.inflation_radius,
    config_.inscribed_radius,
    config_.inscribed_value,
    1.0
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
    waveCostInflation(lethal_vertices_, config_.inflation_radius, config_.inscribed_radius, config_.inscribed_value, std::numeric_limits<float>::infinity());
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
