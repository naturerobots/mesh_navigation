#include <mesh_layers/combination_layer.h>
#include <mesh_map/timer.h>
#include <mesh_map/mesh_map.h>

namespace mesh_layers
{

bool MaxCombinationLayer::initialize()
{
  // Read the layers dependents
  // TODO: Maybe these could be provided in the abstract layer
  std::vector<std::string> inputs = node_->get_parameter(
    mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".inputs"
  ).as_string_array();

  if (inputs.empty())
  {
    RCLCPP_ERROR(get_logger(), "[MaxCombinationLayer] No inputs provided in configuration!");
    return false;
  }

  const auto map = map_ptr_.lock();

  for (const std::string& dep: inputs)
  {
    const auto& layer = map->layer(dep);
    if (nullptr == layer)
    {
      RCLCPP_ERROR(get_logger(), "[MaxCombinationLayer] Could not get input layer '%s' from map!", dep.c_str());
      return false;
    }
    inputs_.push_back(layer);
  }

  costs_.reserve(map->mesh()->numVertices());
  for (const auto& v: map->mesh()->vertices())
  {
    costs_.insert(v, defaultValue());
  }

  return true;
}

bool MaxCombinationLayer::computeLayer()
{
  // Lock all layers so they cannot be destroyed while updating
  std::vector<std::shared_ptr<mesh_map::AbstractLayer>> layers(inputs_.size());
  layers.clear();
  for (const auto& ref: inputs_)
  {
    const auto& ptr = ref.lock();
    layers.push_back(ptr);
  }

  // Compute the maximum cost for each vertex
  auto map = map_ptr_.lock();
  for (const auto& vertex: map->mesh()->vertices())
  {
    float cost = defaultValue();
    for (const auto& ptr: layers)
    {
      const auto& cm = ptr->costs();
      const float def = ptr->defaultValue();
      const float tmp = cm.get(vertex).value_or(def);
      cost = std::max(cost, tmp);
    }
    costs_.insert(vertex, cost);
  }

  // NOTE: We decided to not normalize the combined costs to [0 - 1] to elimitate
  // the need to recompute/renormalize the costs when an input layer changes.

  // Build the combined lethal vertex set
  lethals_.clear();
  for (const auto& ptr: layers)
  {
    const auto& set = ptr->lethals();
    lethals_.insert(set.begin(), set.end());
  }

  // No special handling for lethal vertices.
  // This layer only takes the max from all inputs and does not modify the costs any other way.

  return true;
}

void MaxCombinationLayer::onInputChanged(
  const rclcpp::Time& timestamp,
  const std::set<lvr2::VertexHandle>& changed
)
{
  const auto t0 = mesh_map::LayerTimer::Clock::now();
  // Lock this layer for writing and all inputs for reading
  auto wlock = this->writeLock();
  std::vector<std::shared_ptr<mesh_map::AbstractLayer>> layers(inputs_.size());
  std::vector<std::shared_lock<std::shared_mutex>> locks(inputs_.size());
  layers.clear();
  locks.clear();
  for (const auto& ref: inputs_)
  {
    const auto& ptr = ref.lock();
    auto lock = ptr->readLock();
    layers.push_back(ptr);
    locks.push_back(std::move(lock));
  }
  const auto t1 = mesh_map::LayerTimer::Clock::now();

  // Do the update
  for (const lvr2::VertexHandle& v: changed)
  {
    float cost = 0;
    for (const auto& layer: layers)
    {
      const auto& cm = layer->costs();
      const float def = layer->defaultValue();
      const float tmp = cm.get(v).value_or(def);
      cost = std::max(tmp, cost);
    }
    costs_.insert(v, cost);
  }

  // Update lethal vertex set!
  for (const lvr2::VertexHandle v: changed)
  {
    bool lethal = false;
    for (auto layer: layers)
    {
      lethal = lethal || layer->lethals().count(v) > 0;
    }

    if (lethal)
    {
      lethals_.insert(v);
    }
    else
    {
      lethals_.erase(v);
    }
  }

  locks.clear();
  wlock.unlock();
  const auto t2 = mesh_map::LayerTimer::Clock::now();
  this->notifyChange(timestamp, changed);
  const auto t3 = mesh_map::LayerTimer::Clock::now();
  mesh_map::LayerTimer::recordUpdateDuration(layer_name_, timestamp, t1 - t0, t2 - t1, t3 - t2);
}


bool AvgCombinationLayer::initialize()
{
  // Read the layers dependents
  // TODO: Maybe these could be provided in the abstract layer
  std::vector<std::string> inputs = node_->get_parameter(
    mesh_map::MeshMap::MESH_MAP_NAMESPACE + "." + layer_name_ + ".inputs"
  ).as_string_array();

  if (inputs.empty())
  {
    RCLCPP_ERROR(get_logger(), "[CombinationLayer] No inputs provided in configuration!");
    return false;
  }

  auto map = map_ptr_.lock();
  for (const std::string& dep: inputs)
  {
    const auto& layer = map->layer(dep);
    if (nullptr == layer)
    {
      RCLCPP_ERROR(get_logger(), "[CombinationLayer] Could not get input layer '%s' from map!", dep.c_str());
      return false;
    }
    inputs_.push_back(layer);
  }

  costs_.reserve(map->mesh()->numVertices());
  for (const auto& v: map->mesh()->vertices())
  {
    costs_.insert(v, defaultValue());
  }

  return true;
}

bool AvgCombinationLayer::computeLayer()
{
  // Lock all layer points so they cannot be destroyed while updating. Is this necessary? Layers should have no dependency loops anyway?
  std::vector<std::shared_ptr<mesh_map::AbstractLayer>> layers;
  layers.reserve(inputs_.size());
  for (const auto& ref: inputs_)
  {
    const auto& ptr = ref.lock();
    layers.push_back(ptr);
  }

  // Calculate a weighted average over all input layers
  for (const auto& ptr: layers)
  {
    const auto lock = ptr->readLock();
    const auto& cm = ptr->costs();
    const float def = ptr->defaultValue();
    const float weight = ptr->combinationWeight();
    bool has_nan = false;

    for (const lvr2::VertexHandle v: costs_)
    {
      const float tmp = cm.get(v).value_or(def);
      costs_[v] += weight * tmp;

      has_nan = has_nan || std::isnan(tmp);
    }

    if (has_nan)
    {
      RCLCPP_ERROR(
        get_logger(),
        "Input layer '%s' contains NaN values! This breaks downstream costs and algorithms!",
        ptr->name().c_str()
      );
      // TODO: We could return false here to ensure users notice?
    }
  }

  float max = std::numeric_limits<float>::lowest();
  float min = std::numeric_limits<float>::max();
  for (auto vh: costs_)
  {
    max = std::max(costs_[vh], max);
    min = std::min(costs_[vh], min);
  }

  RCLCPP_DEBUG(get_logger(), "Cost values are in range [%f - %f]", min, max);

  // NOTE: We decided to not normalize the combined costs to [0 - 1] to elimitate
  // the need to recompute/renormalize the costs when an input layer changes.

  // Merge all lethals to a combined set
  lethals_.clear();
  for (const auto& ptr: layers)
  {
    const auto& set = ptr->lethals();
    lethals_.insert(set.begin(), set.end());
  }

  // TODO: Overwrite the cost of lethal vertices??

  return true;
}

void AvgCombinationLayer::onInputChanged(
  const rclcpp::Time& timestamp,
  const std::set<lvr2::VertexHandle>& changed
)
{
  std::vector<std::shared_ptr<mesh_map::AbstractLayer>> layers;
  std::vector<std::shared_lock<std::shared_mutex>> locks;
  layers.reserve(inputs_.size());
  locks.reserve(inputs_.size());
  for (const auto& ref: inputs_)
  {
    const auto& ptr = ref.lock();
    layers.push_back(ptr);
    locks.push_back(ptr->readLock());
  }

  auto lock = this->writeLock();
  for (const lvr2::VertexHandle v: changed)
  {
    float cost = 0;
    for (const auto& layer: layers)
    {
      const auto& cm = layer->costs();
      const float def = layer->defaultValue();
      cost += layer->combinationWeight() * cm.get(v).value_or(def);
    }
    costs_.insert(v, cost);
  }

  // Update lethal vertex set!
  for (const lvr2::VertexHandle v: changed)
  {
    bool lethal = false;
    for (auto layer: layers)
    {
      lethal = lethal || layer->lethals().count(v) > 0;
    }

    if (lethal)
    {
      lethals_.insert(v);
    }
    else
    {
      lethals_.erase(v);
    }
  }

  locks.clear();
  lock.unlock();

  this->notifyChange(timestamp, changed);
}

} // namespace mesh_layers

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mesh_layers::MaxCombinationLayer, mesh_map::AbstractLayer)
PLUGINLIB_EXPORT_CLASS(mesh_layers::AvgCombinationLayer, mesh_map::AbstractLayer)
