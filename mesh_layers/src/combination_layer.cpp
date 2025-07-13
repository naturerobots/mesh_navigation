#include <mesh_layers/combination_layer.h>
#include <mesh_map/timer.h>

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
  // TODO: Can this be more efficient?
  // TODO: Normalize each layers cost?

  // Lock all layers so they cannot be destroyed while updating
  std::vector<std::shared_ptr<mesh_map::AbstractLayer>> layers(inputs_.size());
  layers.clear();
  for (const auto& ref: inputs_)
  {
    const auto& ptr = ref.lock();
    layers.push_back(ptr);
  }

  auto map = map_ptr_.lock();
  for (const auto& vertex: map->mesh()->vertices())
  {
    float cost = defaultValue();
    for (const auto& ptr: layers)
    {
      const auto& cm = ptr->costs();
      const float tmp = cm.containsKey(vertex) ? cm[vertex] : ptr->defaultValue();
      cost = std::max(cost, tmp);
    }
    costs_.insert(vertex, cost);
  }

  lethals_.clear();
  for (const auto& ptr: layers)
  {
    const auto& set = ptr->lethals();
    lethals_.insert(set.begin(), set.end());
  }

  for (const auto& v: lethals_)
  {
    costs_[v] = 1.0;
  }

  return true;
}


void MaxCombinationLayer::updateLethal(
  std::set<lvr2::VertexHandle>& added_lethal,
  std::set<lvr2::VertexHandle>& removed_lethal
)
{}


void MaxCombinationLayer::updateInput(const std::set<lvr2::VertexHandle>& changed)
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
      const float tmp = layer->costs().containsKey(v) ? layer->costs()[v] : layer->defaultValue();
      cost = std::max(tmp, cost);
    }
    costs_.insert(v, cost);
  }

  locks.clear();
  wlock.unlock();
  const auto t2 = mesh_map::LayerTimer::Clock::now();
  this->notifyChange(changed);
  const auto t3 = mesh_map::LayerTimer::Clock::now();
  mesh_map::LayerTimer::recordUpdateDuration(layer_name_, t0, t1 - t0, t2 - t1, t3 - t2);
}


bool CombinationLayer::initialize()
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

template <typename MapT>
void integrate_vertex_map(lvr2::DenseVertexMap<float>& out, const std::remove_pointer_t<MapT>& costs, const float factor, const float default_value)
{

  float min = std::numeric_limits<float>::infinity();
  float max = -std::numeric_limits<float>::infinity();

  for (const auto& v: costs)
  {
    const float c = costs[v];
    if (std::isfinite(c))
    {
      min = std::min(min, c);
      max = std::max(max, c);
    }
  }

  min = std::min(min, default_value);
  max = std::max(max, default_value);

  // TODO: This is the old behavior, but is this correct?
  float norm = factor / (max - min);

  // If the range in the layer is too small we cannot normalize, but we also dont have to right?
  if (norm <= 0.00001)
  {
    norm = factor;
  }

  for (const auto& v: out)
  {
    if (costs.containsKey(v))
    {
      out[v] += norm * costs[v];
    }
    else
    {
      out[v] += norm * default_value;
    }
  }
}

bool CombinationLayer::computeLayer()
{
  // TODO: Is this faster than just using the interface?
  // TODO: Normalize each layers cost

  // Lock all layers so they cannot be destroyed while updating. Is this necessary? Layers should have no dependency loops anyway?
  std::vector<std::shared_ptr<mesh_map::AbstractLayer>> layers(inputs_.size());
  layers.clear();
  for (const auto& ref: inputs_)
  {
    const auto& ptr = ref.lock();
    layers.push_back(ptr);
  }

  for (const auto& ptr: layers)
  {
    auto lock = ptr->readLock();
    const auto* cm = &ptr->costs();
    // TODO: Check where factor is read
    if (const auto* p = dynamic_cast<const lvr2::DenseVertexMap<float>*>(cm))
    {
      integrate_vertex_map<lvr2::DenseVertexMap<float>>(costs_, *p, 1.0, ptr->defaultValue());
    }
    else if (const auto* p = dynamic_cast<const lvr2::SparseVertexMap<float>*>(cm))
    {
      integrate_vertex_map<lvr2::SparseVertexMap<float>>(costs_, *p, 1.0, ptr->defaultValue());
    }
    else if (const auto* p = dynamic_cast<const lvr2::TinyVertexMap<float>*>(cm))
    {
      integrate_vertex_map<lvr2::TinyVertexMap<float>>(costs_, *p, 1.0, ptr->defaultValue());
    }
    else
    {
      integrate_vertex_map<lvr2::VertexMap<float>>(costs_, *cm, 1.0, ptr->defaultValue());
    }
  }

  // Normalize the resulting layer to cost range 1
  const float norm = 1.0 / inputs_.size();
  for (const auto& v: costs_)
  {
    costs_[v] *= norm;
  }

  lethals_.clear();
  for (const auto& ptr: layers)
  {
    const auto& set = ptr->lethals();
    lethals_.insert(set.begin(), set.end());
  }

  for (const auto& v: lethals_)
  {
    costs_[v] = 1.0;
  }

  return true;
}


void CombinationLayer::updateLethal(
  std::set<lvr2::VertexHandle>& added_lethal,
  std::set<lvr2::VertexHandle>& removed_lethal
)
{}
  
void CombinationLayer::updateInput(const std::set<lvr2::VertexHandle>& changed)
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

  const float norm = 1.0 / layers.size();
  auto lock = this->writeLock();
  for (const lvr2::VertexHandle v: changed)
  {
    float cost = 0;
    for (const auto& layer: layers)
    {
      cost += layer->costs().containsKey(v) ? layer->costs()[v] : layer->defaultValue();
    }
    costs_.insert(v, cost * norm);
  }

  // Lethals!
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
      costs_.insert(v, 1.0);
    }
    else
    {
      lethals_.erase(v);
    }
  }

  locks.clear();
  lock.unlock();

  this->notifyChange(changed);
}

} // namespace mesh_layers

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mesh_layers::MaxCombinationLayer, mesh_map::AbstractLayer)
PLUGINLIB_EXPORT_CLASS(mesh_layers::CombinationLayer, mesh_map::AbstractLayer)
