#include <mesh_map/layer_manager.h>
#include <mesh_map/abstract_layer.h>
#include <mesh_map/mesh_map.h>

#include <boost/graph/topological_sort.hpp>

namespace mesh_map
{

LayerManager::LayerManager()
: loader_("mesh_map", "mesh_map::AbstractLayer")
{}

void LayerManager::read_configured_layers(const rclcpp::Node::SharedPtr& node)
{
  const auto layer_names = node->declare_parameter(
    MeshMap::MESH_MAP_NAMESPACE + ".layers", std::vector<std::string>()
  );
  const rclcpp::ParameterType ros_param_type = rclcpp::ParameterType::PARAMETER_STRING;
  std::unordered_set<std::string> layer_names_in_use;
  for(const std::string& layer_name : layer_names)
  {
    if (layer_names_in_use.find(layer_name) != layer_names_in_use.end())
    {
      throw rclcpp::exceptions::InvalidParametersException("The layer name " + layer_name + " is used more than once. Layer names must be unique!");
    }
    // This will throws rclcpp::ParameterValue exception if mesh_map.layer_name.type is not set
    const std::string layer_type = node->declare_parameter<std::string>(
      MeshMap::MESH_MAP_NAMESPACE + "." + layer_name + ".type"
    );

    // populate map from layer name to layer type, which will be used in loadLayerPlugins()
    types_.insert_or_assign(layer_name, layer_type);
    names_.push_back(layer_name);

    layer_names_in_use.emplace(layer_name);
  }
  // output warning if no layer plugins were configured
  if (names_.size() == 0)
  {
    RCLCPP_WARN_STREAM(node->get_logger(), "No MeshMap layer plugins configured!"
      << " - Use the param \"" << MeshMap::MESH_MAP_NAMESPACE << ".layers\", which must be a list of strings with arbitrary layer names. "
      << "For each layer_name, also define layer_name.type with the respective type that shall be loaded via pluginlib.");
  }

  // Create vertices. The vertex id should be equal to the index in names_
  for (const auto& layer_name: names_)
  {
    const Vertex v = boost::add_vertex(graph_);
    vertices_.insert_or_assign(layer_name, v);
  }
  
  // Read the dependencies
  for (const auto& layer_name: names_)
  {
    const Vertex& v = vertices_[layer_name];
    // If mesh_map.layer_name.inputs is not set we assume the layer needs no inputs
    const auto dependencies = node->declare_parameter<std::vector<std::string>>(
      MeshMap::MESH_MAP_NAMESPACE + "." + layer_name + ".inputs", std::vector<std::string>()
    );
    
    // Insert the edges
    for (const auto& dep: dependencies)
    {
      const Vertex& v2 = vertices_[dep];
      boost::add_edge(v, v2, graph_);
    }
  }
  
  // Print the dependencies
  RCLCPP_INFO(node->get_logger(), "Layer dependencies:");
  for (const auto& [name, vertex]: vertices_)
  {
    auto [it, end] = boost::adjacent_vertices(vertex, graph_);

    if (it == end)
    {
      continue;
    }

    std::stringstream builder;
    builder << '[' << names_[(it++).dereference()];
    for (; it != end; it++)
    {
      builder << ", " << names_[it.dereference()];
    }
    builder << ']';

    RCLCPP_INFO(node->get_logger(), "  %s: %s", name.c_str(), builder.str().c_str());
  }
}

bool LayerManager::load_layer_plugins(const rclcpp::Logger& logger)
{
  for (const auto &[layer_name, layer_type] : types_)
  {
    try 
    {
      typename AbstractLayer::Ptr layer_ptr = loader_.createSharedInstance(layer_type);
      instances_.insert_or_assign(layer_name, layer_ptr);
      loaded_.push_back(layer_name);
      RCLCPP_INFO(
        logger,
        "The layer with the type \"%s\" has been loaded successfully under the name \"%s\".",
        layer_type.c_str(),
        layer_name.c_str());
    }
    catch (pluginlib::LibraryLoadException& e)
    {
      RCLCPP_ERROR(
        logger,
        "Could not load the layer with the name \"%s\" and the type \"%s\"! Error: %s",
        layer_name.c_str(),
        layer_type.c_str(),
        e.what()
      );
    }
  }
  
  // did we load any layer?
  return instances_.empty() ? false : true;
}

bool LayerManager::initialize_layer_plugins(const rclcpp::Node::SharedPtr& node, const MeshMap::Ptr& map)
{
  // Topological sort ensures that all dependencies are initialized before their dependents
  std::vector<Vertex> init_order(boost::num_vertices(graph_));
  boost::topological_sort(graph_, init_order.begin());

  for (const Vertex& v: init_order)
  {
    const auto& name = names_[v];
    const auto& instance = get_layer(name);

    // Skip initialization of layers which failed to load
    if (nullptr == instance)
    {
      RCLCPP_WARN(
        node->get_logger(),
        "Skipping initialization of layer '%s' because it failed to load",
        name.c_str()
      );
      continue;
    }

    auto callback = [map](const std::string& layer_name) {map->layerChanged(layer_name);};

    if (!instance->initialize(name, callback, map, node))
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Could not initialize the layer plugin with the name \"%s\"!",
        name.c_str()
      );
      return false;
    }

    if (!instance->readLayer())
    {
      RCLCPP_INFO(node->get_logger(), "Computing layer '%s' ...", name.c_str());
      instance->computeLayer();
    }
  }

  return true;
}

} // namespace mesh_map;
