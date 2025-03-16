#include <mesh_layers/combination_layer.h>

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
    RCLCPP_ERROR(node_->get_logger(), "[MaxCombinationLayer] No inputs provided in configuration!");
    return false;
  }

  for (const std::string& dep: inputs)
  {
    const auto& layer = map_ptr_->layer(dep);
    if (nullptr == layer)
    {
      RCLCPP_ERROR(node_->get_logger(), "[MaxCombinationLayer] Could not get input layer '%s' from map!", dep.c_str());
      return false;
    }
    inputs_.push_back(layer);
  }

  costs_.reserve(map_ptr_->mesh()->numVertices());
  for (const auto& v: map_ptr_->mesh()->vertices())
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

  for (const auto& vertex: map_ptr_->mesh()->vertices())
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
    lethals_.merge(ptr->lethals());
  }

  // TODO: This is the old behavior, is this good?
  for (const auto& v: lethals_)
  {
    costs_[v] = std::numeric_limits<float>::infinity();
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
  std::vector<std::shared_ptr<mesh_map::AbstractLayer>> layers(inputs_.size());
  layers.clear();
  for (const auto& ref: inputs_)
  {
    const auto& ptr = ref.lock();
    layers.push_back(ptr);
  }

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
    RCLCPP_ERROR(node_->get_logger(), "[CombinationLayer] No inputs provided in configuration!");
    return false;
  }

  for (const std::string& dep: inputs)
  {
    const auto& layer = map_ptr_->layer(dep);
    if (nullptr == layer)
    {
      RCLCPP_ERROR(node_->get_logger(), "[CombinationLayer] Could not get input layer '%s' from map!", dep.c_str());
      return false;
    }
    inputs_.push_back(layer);
  }

  costs_.reserve(map_ptr_->mesh()->numVertices());
  for (const auto& v: map_ptr_->mesh()->vertices())
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
    lethals_.merge(ptr->lethals());
  }

  // TODO: This is the old behavior, is this good?
  for (const auto& v: lethals_)
  {
    costs_[v] = std::numeric_limits<float>::infinity();
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
  std::vector<std::shared_ptr<mesh_map::AbstractLayer>> layers(inputs_.size());
  layers.clear();
  for (const auto& ref: inputs_)
  {
    const auto& ptr = ref.lock();
    layers.push_back(ptr);
  }

  const float norm = 1.0 / layers.size();
  for (const lvr2::VertexHandle& v: changed)
  {
    float cost = 0;
    for (const auto& layer: layers)
    {
      cost += layer->costs().containsKey(v) ? layer->costs()[v] : layer->defaultValue();
    }
    costs_.insert(v, cost * norm);
  }
}

} // namespace mesh_layers

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mesh_layers::MaxCombinationLayer, mesh_map::AbstractLayer)
PLUGINLIB_EXPORT_CLASS(mesh_layers::CombinationLayer, mesh_map::AbstractLayer)
