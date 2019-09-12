#include <mesh_layers/inflation_layer.h>
#include <queue>
#include <lvr2/util/Meap.hpp>
#include <pluginlib/class_list_macros.h>
#include <mesh_map/util.h>

PLUGINLIB_EXPORT_CLASS(mesh_layers::InflationLayer, mesh_map::AbstractLayer);

namespace mesh_layers {

bool InflationLayer::readLayer() {
  // riskiness
  ROS_INFO_STREAM("Try to read riskiness from map file...");
  auto riskiness_opt =
      mesh_io_ptr->getDenseAttributeMap<lvr2::DenseVertexMap<float>>(
          "riskiness");

  if (riskiness_opt) {
    ROS_INFO_STREAM("Riskiness has been read successfully.");
    riskiness = riskiness_opt.get();
    return true;
  } else {
    return false;
  }
}

bool InflationLayer::writeLayer() {
  ROS_INFO_STREAM("Saving " << riskiness.numValues()
                            << " riskiness values to map file...");

  if (mesh_io_ptr->addDenseAttributeMap(riskiness, "riskiness")) {
    ROS_INFO_STREAM("Saved riskiness to map file.");
    return true;
  } else {
    ROS_ERROR_STREAM("Could not save riskiness to map file!");
    return false;
  }
}

float InflationLayer::threshold() {return std::numeric_limits<float>::quiet_NaN();}

void InflationLayer::updateLethal(
    std::set<lvr2::VertexHandle> &added_lethal,
    std::set<lvr2::VertexHandle> &removed_lethal) {
  lethal_vertices = added_lethal;

  waveCostInflation(lethal_vertices, config.inflation_radius,
                    config.inscribed_radius, config.inscribed_value,
                    std::numeric_limits<float>::infinity());


  /*lethalCostInflation(lethal_vertices, config.inflation_radius,
                      config.inscribed_radius, config.inscribed_value,
                      config.lethal_value == -1
                          ? std::numeric_limits<float>::infinity()
                          : config.lethal_value);
*/
}

inline float InflationLayer::computeUpdateSethianMethod(const float &d1, const float &d2,
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
    if (std::fabs(f2) > mesh_layers::EPSILON) {
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


inline bool InflationLayer::waveFrontUpdate(
    lvr2::DenseVertexMap<float> &distances,
    lvr2::DenseVertexMap<lvr2::VertexHandle> &predecessors,
    const float& max_distance,
    const lvr2::DenseEdgeMap<float> &edge_weights,
    const lvr2::FaceHandle& fh, const lvr2::BaseVector<float>& normal,
    const lvr2::VertexHandle &v1h, const lvr2::VertexHandle &v2h, const lvr2::VertexHandle &v3h) {
  const auto &mesh = map_ptr->mesh();

  const double u1 = distances[v1h];
  const double u2 = distances[v2h];
  const double u3 = distances[v3h];

  if(u3 == 0) return false;

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

  if(!std::isfinite(u3tmp)) return false;

  const float d31 = u3tmp - u1;
  const float d32 = u3tmp - u2;

  if(u1 == 0 && u2 == 0)
  {
    const auto& v1 = mesh_ptr->getVertexPosition(v1h);
    const auto& v2 = mesh_ptr->getVertexPosition(v2h);
    const auto &face_normals = map_ptr->faceNormals();
    const auto &dir = (v2-v1).normalized().rotated(normal, M_PI_2);
    cutting_faces.insert(v1h, fh);
    cutting_faces.insert(v2h, fh);
    cutting_faces.insert(v3h, fh);
    vector_map[v1h] = (vector_map[v1h] + dir).normalized();
    vector_map[v2h] = (vector_map[v2h] + dir).normalized();
    vector_map[v3h] = (vector_map[v1h] * d31 + vector_map[v2h] * d32) / (d31 + d32);
    //vector_map[v3h] = (vector_map[v3h] + dir).normalized();
  }

  if (u3tmp < u3) {
    distances[v3h] = static_cast<float>(u3tmp);

    if(u1 != 0 || u2 != 0)
    {
      cutting_faces.insert(v3h, fh);
      vector_map[v3h] = (vector_map[v1h] * d31 + vector_map[v2h] * d32) / (d31 + d32);
    }

    if (d31 < d32)
    {
      predecessors.insert(v3h, v1h);
    }
    else // right face check
    {
      predecessors.insert(v3h, v2h);
    }

    //backToSource(v3h, predecessors, vector_map);
    return u1 <= max_distance && u2 <= max_distance;
  }
  return false;
}


void InflationLayer::waveCostInflation(
    const std::set<lvr2::VertexHandle> &lethals, const float inflation_radius,
    const float inscribed_radius, const float inscribed_value,
    const float lethal_value) {

  auto const &mesh = *mesh_ptr;

  ROS_INFO_STREAM("inflation radius:" << inflation_radius);
  ROS_INFO_STREAM("Init wave inflation.");

  lvr2::DenseVertexMap<bool> seen(mesh_ptr->nextVertexIndex(), false);
  distances = lvr2::DenseVertexMap<float>(mesh_ptr->nextVertexIndex(),
                                        std::numeric_limits<float>::infinity());
  lvr2::DenseVertexMap<lvr2::VertexHandle> predecessors;
  predecessors.reserve(mesh_ptr->nextVertexIndex());

  vector_map = lvr2::DenseVertexMap<lvr2::BaseVector<float>>(mesh.nextVertexIndex(), lvr2::BaseVector<float>());

  direction = lvr2::DenseVertexMap<float>();

  const auto &edge_distances = map_ptr->edgeDistances();
  const auto &face_normals = map_ptr->faceNormals();

  lvr2::DenseVertexMap<bool> fixed(mesh.nextVertexIndex(), false);

  // initialize distances with infinity
  // initialize predecessor of each vertex with itself
  for (auto const &vH : mesh.vertices()) {
    predecessors.insert(vH, vH);
  }

  lvr2::Meap<lvr2::VertexHandle, float> pq;
  // Set start distance to zero
  // add start vertex to priority queue
  for (auto vH : lethals) {
    distances[vH] = 0;
    fixed[vH] = true;
    pq.insert(vH, 0);
  }

  ROS_INFO_STREAM("Start inflation wave front propagation");

  while (!pq.isEmpty()) {
    lvr2::VertexHandle current_vh = pq.popMin().key;

    // check if already fixed
    // if(fixed[current_vh]) continue;
    fixed[current_vh] = true;

    std::vector<lvr2::VertexHandle> neighbours;
    mesh.getNeighboursOfVertex(current_vh, neighbours);
    for(auto nh : neighbours){
      std::vector<lvr2::FaceHandle> faces;
      mesh.getFacesOfVertex(nh, faces);

      for (auto fh : faces) {
        const auto vertices = mesh.getVerticesOfFace(fh);
        const lvr2::VertexHandle &a = vertices[0];
        const lvr2::VertexHandle &b = vertices[1];
        const lvr2::VertexHandle &c = vertices[2];

        if (fixed[a] && fixed[b] && fixed[c]) {
          //ROS_INFO_STREAM("All fixed!");
          continue;
        }
        else if (fixed[a] && fixed[b] && !fixed[c]) {
          // c is free
          if (waveFrontUpdate(distances, predecessors, inflation_radius, edge_distances, fh, face_normals[fh], a, b, c))
          {
            pq.insert(c, distances[c]);
          }
          //if(pq.containsKey(c)) pq.updateValue(c, distances[c]);
        } else if (fixed[a] && !fixed[b] && fixed[c]) {
          // b is free
          if (waveFrontUpdate(distances, predecessors, inflation_radius, edge_distances, fh, face_normals[fh], c, a, b))
          {
            pq.insert(b, distances[b]);
          }
          //if(pq.containsKey(b)) pq.updateValue(b, distances[b]);
        } else if (!fixed[a] && fixed[b] && fixed[c]) {
          // a if free
          if (waveFrontUpdate(distances, predecessors, inflation_radius, edge_distances, fh, face_normals[fh], b, c, a))
          {
            pq.insert(a, distances[a]);
          }
          //if(pq.containsKey(a)) pq.updateValue(a, distances[a]);
        } else {
          // two free vertices -> skip that face
          //ROS_INFO_STREAM("two vertices are free.");
          continue;
        }
      }

    }
  }

  ROS_INFO_STREAM("Finished inflation wave front propagation.");

  for (auto vH : mesh_ptr->vertices()) {
    if (distances[vH] > inflation_radius) {
      riskiness.insert(vH, 0);
    }

      // Inflation radius
    else if (distances[vH] > inscribed_radius) {
      float alpha = (sqrt(distances[vH]) - inscribed_radius) /
          (inflation_radius - inscribed_radius) * M_PI;
      riskiness.insert(vH, inscribed_value * (cos(alpha) + 1) / 2.0);
    }

      // Inscribed radius
    else if (distances[vH] > 0) {
      riskiness.insert(vH, inscribed_value);
    }

      // Lethality
    else {
      riskiness.insert(vH, lethal_value);
    }
  }
  map_ptr->publishVectorField("inflation", vector_map, cutting_faces);
}

lvr2::BaseVector<float> InflationLayer::vectorAt(
    const std::array<lvr2::VertexHandle, 3>& vertices,
    const std::array<float, 3>& barycentric_coords)
{
  const float distance = mesh_map::linearCombineBarycentricCoords(vertices, distances, barycentric_coords);

  if (distance > config.inflation_radius)
    return lvr2::BaseVector<float>();

  // Inflation radius
  if (distance > config.inscribed_radius) {
    float alpha = (sqrt(distance) - config.inscribed_radius) /
        (config.inflation_radius - config.inscribed_radius) * M_PI;
    return mesh_map::linearCombineBarycentricCoords(vertices, vector_map, barycentric_coords) * config.inscribed_value * (cos(alpha) + 1) / 2.0;
  }

  // Inscribed radius
  if (distance > 0) {
    return mesh_map::linearCombineBarycentricCoords(vertices, vector_map, barycentric_coords) * config.inscribed_value;
  }

  // Lethality
  return mesh_map::linearCombineBarycentricCoords(vertices, vector_map, barycentric_coords) * config.lethal_value;
}

void InflationLayer::backToSource(
    const lvr2::VertexHandle& current_vertex,
    const lvr2::DenseVertexMap<lvr2::VertexHandle>& predecessors,
    lvr2::DenseVertexMap<lvr2::BaseVector<float>>& vector_map)
{
  if(vector_map.containsKey(current_vertex))
    return;

  const auto& pre = predecessors[current_vertex];

  if(pre != current_vertex)
  {
    backToSource(pre, predecessors, vector_map);
    const auto& v0 = mesh_ptr->getVertexPosition(current_vertex);
    const auto& v1 = mesh_ptr->getVertexPosition(pre);
    vector_map.insert(current_vertex, v1 - v0 + vector_map[pre]);
  }
  else
  {
    vector_map.insert(current_vertex, lvr2::BaseVector<float>());
  }
}

void InflationLayer::lethalCostInflation(
    const std::set<lvr2::VertexHandle> &lethals, const float inflation_radius,
    const float inscribed_radius, const float inscribed_value,
    const float lethal_value) {

  ROS_INFO_STREAM("lethal cost inflation.");

  struct Cmp {
    const lvr2::DenseVertexMap<float> &distances;
    Cmp(const lvr2::DenseVertexMap<float> &distances) : distances(distances) {}

    bool operator()(lvr2::VertexHandle &left, lvr2::VertexHandle &right) {
      return distances[left] > distances[right];
    }
  };

  lvr2::DenseVertexMap<bool> seen(mesh_ptr->nextVertexIndex(), false);
  lvr2::DenseVertexMap<float> distances(mesh_ptr->nextVertexIndex(),
                                        std::numeric_limits<float>::max());
  lvr2::DenseVertexMap<lvr2::VertexHandle> prev;
  prev.reserve(mesh_ptr->nextVertexIndex());

  for (auto vH : mesh_ptr->vertices()) {
    prev.insert(vH, vH);
  }

  Cmp cmp(distances);
  std::priority_queue<lvr2::VertexHandle, std::vector<lvr2::VertexHandle>, Cmp>
      pq(cmp);

  for (auto vH : lethals) {
    distances[vH] = 0;
    pq.push(vH);
  }

  float inflation_radius_squared = inflation_radius * inflation_radius;
  float inscribed_radius_squared = inscribed_radius * inscribed_radius;

  while (!pq.empty()) {
    lvr2::VertexHandle vH = pq.top();
    pq.pop();

    if (seen[vH])
      continue;
    seen[vH] = true;

    std::vector<lvr2::VertexHandle> neighbours;
    mesh_ptr->getNeighboursOfVertex(vH, neighbours);
    for (auto n : neighbours) {
      if (distances[n] == 0 || seen[n])
        continue;
      float n_dist = mesh_ptr->getVertexPosition(n).squaredDistanceFrom(
          mesh_ptr->getVertexPosition(prev[vH]));
      if (n_dist < distances[n] && n_dist < inflation_radius_squared) {
        prev[n] = prev[vH];
        distances[n] = n_dist;
        pq.push(n);
      }
    }
  }

  for (auto vH : mesh_ptr->vertices()) {
    if (distances[vH] > inflation_radius_squared) {
      riskiness.insert(vH, 0);
    }

    // Inflation radius
    else if (distances[vH] > inscribed_radius_squared) {
      float alpha = (sqrt(distances[vH]) - inscribed_radius) /
                    (inflation_radius - inscribed_radius) * M_PI;
      riskiness.insert(vH, inscribed_value * (cos(alpha) + 1) / 2.0);
    }

    // Inscribed radius
    else if (distances[vH] > 0) {
      riskiness.insert(vH, inscribed_value);
    }

    // Lethality
    else {
      riskiness.insert(vH, lethal_value);
    }
  }

  ROS_INFO_STREAM("lethal cost inflation finished.");
}

bool InflationLayer::computeLayer() {
  waveCostInflation(lethal_vertices, config.inflation_radius,
                      config.inscribed_radius, config.inscribed_value,
                      std::numeric_limits<float>::infinity());

  //lethalCostInflation(lethal_vertices, config.inflation_radius,
  //                    config.inscribed_radius, config.inscribed_value,
  //                    std::numeric_limits<float>::infinity());
  // config.lethal_value);

  return true;
}

lvr2::VertexMap<float> &InflationLayer::costs() { return riskiness; }

void InflationLayer::reconfigureCallback(
    mesh_layers::InflationLayerConfig &cfg, uint32_t level)
{
  ROS_INFO_STREAM("New inflation layer config through dynamic reconfigure.");
  if (first_config) {
    config = cfg;
    first_config = false;
  }

  if(config.inflation_radius != cfg.inflation_radius)
  {
    // TODO handle other config params
    waveCostInflation(lethal_vertices, config.inflation_radius,
                      config.inscribed_radius, config.inscribed_value,
                      std::numeric_limits<float>::infinity());
  }

  /*lethalCostInflation(lethal_vertices, cfg.inflation_radius,
                      cfg.inscribed_radius, cfg.inscribed_value,
                      cfg.lethal_value == -1
                          ? std::numeric_limits<float>::infinity()
                          : cfg.lethal_value);
  */
  config = cfg;

  notifyChange();
}

bool InflationLayer::initialize(const std::string &name) {
  reconfigure_server_ptr = boost::shared_ptr<
      dynamic_reconfigure::Server<mesh_layers::InflationLayerConfig>>(
      new dynamic_reconfigure::Server<mesh_layers::InflationLayerConfig>(
          private_nh));

  config_callback =
      boost::bind(&InflationLayer::reconfigureCallback, this, _1, _2);
  reconfigure_server_ptr->setCallback(config_callback);
  return true;
}

} /* namespace mesh_layers */
