#include <mesh_layers/inflation_layer.h>
#include <queue>

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mesh_layers::InflationLayer, mesh_map::AbstractLayer)

namespace mesh_layers{

  bool InflationLayer::readLayer()
  {
    // riskiness
    ROS_INFO_STREAM("Try to read riskiness from map file...");
    auto riskiness_opt = mesh_io_ptr->getDenseAttributeMap<lvr2::DenseVertexMap<float>>("riskiness");

    if(riskiness_opt)
    {
      ROS_INFO_STREAM("Riskiness has been read successfully.");
      riskiness = riskiness_opt.get();
      return true;
    }
    else
    {
      return false;
    }
  }

  bool InflationLayer::writeLayer()
  {
    ROS_INFO_STREAM("Saving " << riskiness.numValues() << " riskiness values to map file...");

    if(mesh_io_ptr->addDenseAttributeMap(riskiness, "riskiness"))
    {
      ROS_INFO_STREAM("Saved riskiness to map file.");
      return true;
    }
    else
    {
      ROS_ERROR_STREAM("Could not save riskiness to map file!");
      return false;
    }
  }

  float InflationLayer::threshold()
  {

  }

  void InflationLayer::lethalCostInflation(
    const std::set<lvr2::VertexHandle>& lethals,
    const float inflation_radius,
    const float inscribed_radius,
    const float inscribed_value,
    const float lethal_value)
  {

    ROS_INFO_STREAM("lethal cost inflation.");

    struct Cmp{
      const lvr2::DenseVertexMap<float>& distances;
      Cmp(const lvr2::DenseVertexMap<float>& distances) : distances(distances) {}

      bool operator() (lvr2::VertexHandle& left, lvr2::VertexHandle& right)
      {
        return distances[left] > distances[right];
      }
    };

    lvr2::DenseVertexMap<bool> seen(mesh_ptr->nextVertexIndex(), false);
    lvr2::DenseVertexMap<float> distances(mesh_ptr->nextVertexIndex(), std::numeric_limits<float>::max());
    lvr2::DenseVertexMap<lvr2::VertexHandle> prev;
    prev.reserve(mesh_ptr->nextVertexIndex());

    for(auto vH: mesh_ptr->vertices())
    {
      prev.insert(vH, vH);
    }

    Cmp cmp(distances);
    std::priority_queue<lvr2::VertexHandle, std::vector<lvr2::VertexHandle>, Cmp> pq(cmp);

    for(auto vH: lethals)
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

      if(seen[vH]) continue;
      seen[vH] = true;

      std::vector<lvr2::VertexHandle> neighbours;
      mesh_ptr->getNeighboursOfVertex(vH, neighbours);
      for(auto n : neighbours)
      {
        if(distances[n] == 0 || seen[n]) continue;
        float n_dist = mesh_ptr->getVertexPosition(n).squaredDistanceFrom(mesh_ptr->getVertexPosition(prev[vH]));
        if(n_dist < distances[n] && n_dist < inflation_radius_squared)
        {
          prev[n] = prev[vH];
          distances[n] = n_dist;
          pq.push(n);
        }
      }
    }

    for(auto vH: mesh_ptr->vertices())
    {
      if(distances[vH] > inflation_radius_squared)
      {
        riskiness.insert(vH, 0);
      }

        // Inflation radius
      else if(distances[vH] > inscribed_radius_squared)
      {
        float alpha = (sqrt(distances[vH]) - inscribed_radius) /
            (inflation_radius - inscribed_radius) * M_PI;
        riskiness.insert(vH, inscribed_value * (cos(alpha) + 1) / 2.0);
      }

        // Inscribed radius
      else if(distances[vH] > 0)
      {
        riskiness.insert(vH, inscribed_value);
      }

        // Lethality
      else
      {
        riskiness.insert(vH, lethal_value);
      }
    }

    ROS_INFO_STREAM("lethal cost inflation finished.");
  }

  bool InflationLayer::computeLayer(const mesh_map::MeshMapConfig& config)
  {
    lethalCostInflation(
        lethal_vertices,
        config.inflation_radius,
        config.inscribed_radius,
        config.inscribed_value,
        config.lethal_value);

    return true;
  }

  lvr2::VertexMap<float>& InflationLayer::costs()
  {
    return riskiness;
  }

  const std::string InflationLayer::getName()
  {
    return "riskiness";
  }

} /* namespace mesh_layers */
