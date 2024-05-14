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
#include <algorithm>
#include <unordered_set>
#include <boost/uuid/random_generator.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <functional>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <mesh_client/mesh_client.h>

#include <lvr2/geometry/Normal.hpp>
#include <lvr2/algorithm/GeometryAlgorithms.hpp>
#include <lvr2/algorithm/NormalAlgorithms.hpp>
#include <lvr2/io/hdf5/MeshIO.hpp>
#include <mesh_map/mesh_map.h>
#include <mesh_map/util.h>
#include <mesh_msgs/msg/mesh_geometry_stamped.hpp>
#include <mesh_msgs_conversions/conversions.h>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace mesh_map
{
using HDF5MeshIO = lvr2::Hdf5IO<lvr2::hdf5features::ArrayIO, lvr2::hdf5features::ChannelIO,
                                lvr2::hdf5features::VariantChannelIO, lvr2::hdf5features::MeshIO>;

MeshMap::MeshMap(tf2_ros::Buffer& tf, const rclcpp::Node::SharedPtr& node)
  : tf_buffer(tf_buffer)
  , node(node)
  , first_config(true)
  , map_loaded(false)
  , layer_loader("mesh_map", "mesh_map::AbstractLayer")
  , mesh_ptr(new lvr2::HalfEdgeMesh<Vector>())
{
  srv_url = node->declare_parameter(MESH_MAP_NAMESPACE + ".server_url", "");
  srv_username = node->declare_parameter(MESH_MAP_NAMESPACE + ".server_username", "");
  srv_password = node->declare_parameter(MESH_MAP_NAMESPACE + ".server_password", "");
  mesh_layer = node->declare_parameter(MESH_MAP_NAMESPACE + ".mesh_layer", "mesh0");
  min_roughness = node->declare_parameter(MESH_MAP_NAMESPACE + ".min_roughness", 0.0);
  max_roughness = node->declare_parameter(MESH_MAP_NAMESPACE + ".max_roughness", 0.0);
  min_height_diff = node->declare_parameter(MESH_MAP_NAMESPACE + ".min_height_diff", 0.0);
  max_height_diff = node->declare_parameter(MESH_MAP_NAMESPACE + ".max_height_diff", 0.0);
  bb_min_x = node->declare_parameter(MESH_MAP_NAMESPACE + ".bb_min_x", 0.0);
  bb_min_y = node->declare_parameter(MESH_MAP_NAMESPACE + ".bb_min_y", 0.0);
  bb_min_z = node->declare_parameter(MESH_MAP_NAMESPACE + ".bb_min_z", 0.0);
  bb_max_x = node->declare_parameter(MESH_MAP_NAMESPACE + ".bb_max_x", 0.0);
  bb_max_y = node->declare_parameter(MESH_MAP_NAMESPACE + ".bb_max_y", 0.0);
  bb_max_z = node->declare_parameter(MESH_MAP_NAMESPACE + ".bb_max_z", 0.0);

  auto min_contour_size_desc = rcl_interfaces::msg::ParameterDescriptor{}; 
  min_contour_size_desc.name = MESH_MAP_NAMESPACE + ".min_contour_size";
  min_contour_size_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;  
  min_contour_size_desc.description = "Defines the minimum size for a contour to be classified as 'lethal'.";
  auto min_contour_size_range = rcl_interfaces::msg::IntegerRange{};
  min_contour_size_range.from_value = 0;
  min_contour_size_range.to_value = 100000;
  min_contour_size_desc.integer_range.push_back(min_contour_size_range);
  min_contour_size = node->declare_parameter(MESH_MAP_NAMESPACE + ".min_contour_size", 3);

  auto layer_factor_desc = rcl_interfaces::msg::ParameterDescriptor{}; 
  layer_factor_desc.name = MESH_MAP_NAMESPACE + ".layer_factor";
  layer_factor_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;  
  layer_factor_desc.description = "Defines the factor for combining edge distances and vertex costs.";
  auto layer_factor_range = rcl_interfaces::msg::FloatingPointRange{};
  layer_factor_range.from_value = 0.0;
  layer_factor_range.to_value = 10.0;
  layer_factor_desc.floating_point_range.push_back(layer_factor_range);
  layer_factor = node->declare_parameter(MESH_MAP_NAMESPACE + ".layer_factor", 1.0, layer_factor_desc);

  auto cost_limit_desc = rcl_interfaces::msg::ParameterDescriptor{}; 
  cost_limit_desc.name = MESH_MAP_NAMESPACE + ".cost_limit";
  cost_limit_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;  
  cost_limit_desc.description = "Defines the vertex cost limit with which it can be accessed.";
  auto cost_limit_range = rcl_interfaces::msg::FloatingPointRange{};
  cost_limit_range.from_value = 0.0;
  cost_limit_range.to_value = 10.0;
  cost_limit_desc.floating_point_range.push_back(cost_limit_range);
  cost_limit = node->declare_parameter(MESH_MAP_NAMESPACE + ".cost_limit", 1.0);

  mesh_file = node->declare_parameter(MESH_MAP_NAMESPACE + ".mesh_file", "");
  mesh_part = node->declare_parameter(MESH_MAP_NAMESPACE + ".mesh_part", "");
  global_frame = node->declare_parameter(MESH_MAP_NAMESPACE + ".global_frame", "map");
  RCLCPP_INFO_STREAM(node->get_logger(), "mesh file is set to: " << mesh_file);

  // params for map layer names to types:
  const auto layer_names = node->declare_parameter(MESH_MAP_NAMESPACE + ".layers", std::vector<std::string>());
  const rclcpp::ParameterType ros_param_type = rclcpp::ParameterType::PARAMETER_STRING;
  std::unordered_set<std::string> layer_names_in_use;
  for(const std::string& layer_name : layer_names)
  {
    if (layer_names_in_use.find(layer_name) != layer_names_in_use.end())
    {
      throw rclcpp::exceptions::InvalidParametersException("The layer name " + layer_name + " is used more than once. Layer names must be unique!");
    }
    // This will throws rclcpp::ParameterValue exception if mesh_map.layer_name.type is not set
    const std::string layer_type = node->declare_parameter(MESH_MAP_NAMESPACE + "." + layer_name + ".type", ros_param_type).get<std::string>();

    // populate map from layer name to layer type, which will be used in loadLayerPlugins()
    configured_layers.push_back(std::make_pair(layer_name, layer_type));
    layer_names_in_use.emplace(layer_name);
  }
  // output warning if no layer plugins were configured
  if (configured_layers.size() == 0)
  {
    RCLCPP_WARN_STREAM(node->get_logger(), "No MeshMap layer plugins configured!"
      << " - Use the param \"" << MESH_MAP_NAMESPACE << ".layers\", which must be a list of strings with arbitrary layer names. "
      << "For each layer_name, also define layer_name.type with the respective type that shall be loaded via pluginlib.");
  }

  marker_pub = node->create_publisher<visualization_msgs::msg::Marker>("~/marker", 100);
  mesh_geometry_pub = node->create_publisher<mesh_msgs::msg::MeshGeometryStamped>("~/mesh", rclcpp::QoS(1).transient_local());
  vertex_costs_pub = node->create_publisher<mesh_msgs::msg::MeshVertexCostsStamped>("~/vertex_costs", rclcpp::QoS(1).transient_local());
  vertex_colors_pub = node->create_publisher<mesh_msgs::msg::MeshVertexColorsStamped>("~/vertex_colors", rclcpp::QoS(1).transient_local());
  vector_field_pub = node->create_publisher<visualization_msgs::msg::Marker>("~/vector_field", rclcpp::QoS(1).transient_local());
  config_callback = node->add_on_set_parameters_callback(std::bind(&MeshMap::reconfigureCallback, this, std::placeholders::_1));
}

bool MeshMap::readMap()
{
  RCLCPP_INFO_STREAM(node->get_logger(), "server url: " << srv_url);
  bool server = false;

  if (!srv_url.empty())
  {
    server = true;

    mesh_io_ptr = std::shared_ptr<lvr2::AttributeMeshIOBase>(
        new mesh_client::MeshClient(srv_url, srv_username, srv_password, mesh_layer));
    auto mesh_client_ptr = std::static_pointer_cast<mesh_client::MeshClient>(mesh_io_ptr);

    mesh_client_ptr->setBoundingBox(bb_min_x, bb_min_y, bb_min_z, bb_max_x, bb_max_y, bb_max_z);
    mesh_client_ptr->addFilter("roughness", min_roughness, max_roughness);
    mesh_client_ptr->addFilter("height_diff", min_height_diff, max_height_diff);
  }
  else if (!mesh_file.empty() && !mesh_part.empty())
  {
    RCLCPP_INFO_STREAM(node->get_logger(), "Load \"" << mesh_part << "\" from file \"" << mesh_file << "\"...");
    HDF5MeshIO* hdf_5_mesh_io = new HDF5MeshIO();
    hdf_5_mesh_io->open(mesh_file);
    hdf_5_mesh_io->setMeshName(mesh_part);
    mesh_io_ptr = std::shared_ptr<lvr2::AttributeMeshIOBase>(hdf_5_mesh_io);
  }
  else
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Could not open file or server connection!");
    return false;
  }

  if (server)
  {
    RCLCPP_INFO_STREAM(node->get_logger(), "Start reading the mesh from the server '" << srv_url);
  }
  else
  {
    RCLCPP_INFO_STREAM(node->get_logger(), "Start reading the mesh part '" << mesh_part << "' from the map file '" << mesh_file << "'...");
  }

  auto mesh_opt = mesh_io_ptr->getMesh();

  if (mesh_opt)
  {
    *mesh_ptr = mesh_opt.get();
    RCLCPP_INFO_STREAM(node->get_logger(), "The mesh has been loaded successfully with " 
      << mesh_ptr->numVertices() << " vertices and " << mesh_ptr->numFaces() << " faces and "
      << mesh_ptr->numEdges() << " edges.");

    adaptor_ptr = std::make_unique<NanoFlannMeshAdaptor>(*mesh_ptr);
    kd_tree_ptr = std::make_unique<KDTree>(3,*adaptor_ptr, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    kd_tree_ptr->buildIndex();
    RCLCPP_INFO_STREAM(node->get_logger(), "The k-d tree has been build successfully!");
  }
  else
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Could not load the mesh '" << mesh_part << "' from the map file '" << mesh_file << "' ");
    return false;
  }

  vertex_costs = lvr2::DenseVertexMap<float>(mesh_ptr->nextVertexIndex(), 0);
  edge_weights = lvr2::DenseEdgeMap<float>(mesh_ptr->nextEdgeIndex(), 0);
  invalid = lvr2::DenseVertexMap<bool>(mesh_ptr->nextVertexIndex(), false);

  // TODO read and write uuid
  boost::uuids::random_generator gen;
  boost::uuids::uuid uuid = gen();
  uuid_str = boost::uuids::to_string(uuid);

  auto face_normals_opt = mesh_io_ptr->getDenseAttributeMap<lvr2::DenseFaceMap<Normal>>("face_normals");

  if (face_normals_opt)
  {
    face_normals = face_normals_opt.get();
    RCLCPP_INFO_STREAM(node->get_logger(), "Found " << face_normals.numValues() << " face normals in map file.");
  }
  else
  {
    RCLCPP_INFO_STREAM(node->get_logger(), "No face normals found in the given map file, computing them...");
    face_normals = lvr2::calcFaceNormals(*mesh_ptr);
    RCLCPP_INFO_STREAM(node->get_logger(), "Computed " << face_normals.numValues() << " face normals.");
    if (mesh_io_ptr->addDenseAttributeMap(face_normals, "face_normals"))
    {
      RCLCPP_INFO_STREAM(node->get_logger(), "Saved face normals to map file.");
    }
    else
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Could not save face normals to map file!");
    }
  }

  auto vertex_normals_opt = mesh_io_ptr->getDenseAttributeMap<lvr2::DenseVertexMap<Normal>>("vertex_normals");

  if (vertex_normals_opt)
  {
    vertex_normals = vertex_normals_opt.get();
    RCLCPP_INFO_STREAM(node->get_logger(), "Found " << vertex_normals.numValues() << " vertex normals in map file!");
  }
  else
  {
    RCLCPP_INFO_STREAM(node->get_logger(), "No vertex normals found in the given map file, computing them...");
    vertex_normals = lvr2::calcVertexNormals(*mesh_ptr, face_normals);
    if (mesh_io_ptr->addDenseAttributeMap(vertex_normals, "vertex_normals"))
    {
      RCLCPP_INFO_STREAM(node->get_logger(), "Saved vertex normals to map file.");
    }
    else
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Could not save vertex normals to map file!");
    }
  }

  const rclcpp::Time map_stamp = node->now();
  mesh_geometry_pub->publish(mesh_msgs_conversions::toMeshGeometryStamped<float>(*mesh_ptr, global_frame, uuid_str, vertex_normals, map_stamp));
  publishVertexColors(map_stamp);

  RCLCPP_INFO_STREAM(node->get_logger(), "Try to read edge distances from map file...");
  auto edge_distances_opt = mesh_io_ptr->getAttributeMap<lvr2::DenseEdgeMap<float>>("edge_distances");

  if (edge_distances_opt)
  {
    RCLCPP_INFO_STREAM(node->get_logger(), "Vertex distances have been read successfully.");
    edge_distances = edge_distances_opt.get();
  }
  else
  {
    RCLCPP_INFO_STREAM(node->get_logger(), "Computing edge distances...");
    edge_distances = lvr2::calcVertexDistances(*mesh_ptr);
    RCLCPP_INFO_STREAM(node->get_logger(), "Saving " << edge_distances.numValues() << " edge distances to map file...");

    if (mesh_io_ptr->addAttributeMap(edge_distances, "edge_distances"))
    {
      RCLCPP_INFO_STREAM(node->get_logger(), "Saved edge distances to map file.");
    }
    else
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Could not save edge distances to map file!");
    }
  }

  RCLCPP_INFO_STREAM(node->get_logger(), "Load layer plugins...");
  if (!loadLayerPlugins())
  {
    RCLCPP_FATAL_STREAM(node->get_logger(), "Could not load any layer plugin!");
    return false;
  }

  RCLCPP_INFO_STREAM(node->get_logger(), "Initialize layer plugins...");
  if (!initLayerPlugins())
  {
    RCLCPP_FATAL_STREAM(node->get_logger(), "Could not initialize plugins!");
    return false;
  }

  sleep(1);

  combineVertexCosts(map_stamp);
  publishCostLayers(map_stamp);

  map_loaded = true;
  return true;
}

bool MeshMap::loadLayerPlugins()
{
  for (const auto &[layer_name, layer_type] : configured_layers)
  {
    try 
    {
      typename AbstractLayer::Ptr layer_ptr = layer_loader.createSharedInstance(layer_type);
      loaded_layers.push_back(std::make_pair(layer_name, layer_ptr));
      RCLCPP_INFO(node->get_logger(),
                  "The layer with the type \"%s\" has been loaded successfully under the name \"%s\".", layer_type.c_str(),
                  layer_name.c_str());
    }
    catch (pluginlib::LibraryLoadException& e)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Could not load the layer with the name \"" << layer_name << "\" and the type \"" << layer_type << "\"! Error: " << e.what());
    }
  }
  
  // did we load any layer?
  return loaded_layers.empty() ? false : true;
}

void MeshMap::layerChanged(const std::string& layer_name)
{
  std::lock_guard<std::mutex> lock(layer_mtx);

  RCLCPP_INFO_STREAM(node->get_logger(), "Layer \"" << layer_name << "\" changed.");

  lethals.clear();

  RCLCPP_INFO_STREAM(node->get_logger(), "Combine underlining lethal sets...");

  // TODO pre-compute combined lethals upto a layer level
  auto layer_iter = loaded_layers.begin();
  for (; layer_iter != loaded_layers.end(); layer_iter++)
  {
    // TODO add lethal and removae lethal sets
    lethals.insert(layer_iter->second->lethals().begin(), layer_iter->second->lethals().end());
    // TODO merge with std::set_merge
    if (layer_iter->first == layer_name)
      break;
  }

  vertex_costs_pub->publish(mesh_msgs_conversions::toVertexCostsStamped(layer_iter->second->costs(), mesh_ptr->numVertices(),
                                                         layer_iter->second->defaultValue(), layer_iter->first,
                                                         global_frame, uuid_str));

  if (layer_iter != loaded_layers.end())
    layer_iter++;

  RCLCPP_INFO_STREAM(node->get_logger(), "Combine  lethal sets...");

  for (; layer_iter != loaded_layers.end(); layer_iter++)
  {
    // TODO add lethal and remove lethal sets as param
    layer_iter->second->updateLethal(lethals, lethals);

    lethals.insert(layer_iter->second->lethals().begin(), layer_iter->second->lethals().end());

    vertex_costs_pub->publish(mesh_msgs_conversions::toVertexCostsStamped(layer_iter->second->costs(), mesh_ptr->numVertices(),
                                                           layer_iter->second->defaultValue(), layer_iter->first,
                                                           global_frame, uuid_str));
  }

  RCLCPP_INFO_STREAM(node->get_logger(), "Found " << lethals.size() << " lethal vertices");
  RCLCPP_INFO_STREAM(node->get_logger(), "Combine layer costs...");

  combineVertexCosts(node->now());
  // TODO new lethals old lethals -> renew potential field! around this areas
}

bool MeshMap::initLayerPlugins()
{
  lethals.clear();
  lethal_indices.clear();

  std::shared_ptr<mesh_map::MeshMap> map(this);

  for (auto& layer : loaded_layers)
  {
    auto& layer_plugin = layer.second;
    const auto& layer_name = layer.first;

    auto callback = [this](const std::string& layer_name) { layerChanged(layer_name); };

    if (!layer_plugin->initialize(layer_name, callback, map, mesh_ptr, mesh_io_ptr, node))
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Could not initialize the layer plugin with the name \"" << layer_name << "\"!");
      return false;
    }

    std::set<lvr2::VertexHandle> empty;
    layer_plugin->updateLethal(lethals, empty);
    if (!layer_plugin->readLayer())
    {
      layer_plugin->computeLayer();
    }

    lethal_indices[layer_name].insert(layer_plugin->lethals().begin(), layer_plugin->lethals().end());
    lethals.insert(layer_plugin->lethals().begin(), layer_plugin->lethals().end());
  }
  return true;
}

void MeshMap::combineVertexCosts(const rclcpp::Time& map_stamp)
{
  RCLCPP_INFO_STREAM(node->get_logger(), "Combining costs...");

  float combined_min = std::numeric_limits<float>::max();
  float combined_max = std::numeric_limits<float>::min();

  vertex_costs = lvr2::DenseVertexMap<float>(mesh_ptr->nextVertexIndex(), 0);

  bool hasNaN = false;
  for (auto layer : loaded_layers)
  {
    const auto& costs = layer.second->costs();
    float min, max;
    mesh_map::getMinMax(costs, min, max);
    const float norm = max - min;
    const float factor = 1.0; // TODO how to declare param for each plugin? Needs to be done after plugins are loaded, which happens when the map gets loaded. Who calls readMap() and when?
    // const float factor = private_nh.param<float>(MESH_MAP_NAMESPACE + "/" + layer.first + "/factor", 1.0);
    const float norm_factor = factor / norm;
    RCLCPP_INFO_STREAM(node->get_logger(), "Layer \"" << layer.first << "\" max value: " << max << " min value: " << min << " norm: " << norm
                               << " factor: " << factor << " norm factor: " << norm_factor);

    const float default_value = layer.second->defaultValue();
    hasNaN = false;
    for (auto vH : mesh_ptr->vertices())
    {
      const float cost = costs.containsKey(vH) ? costs[vH] : default_value;
      if (std::isnan(cost))
        hasNaN = true;
      vertex_costs[vH] += factor * cost;
      if (std::isfinite(cost))
      {
        combined_max = std::max(combined_max, vertex_costs[vH]);
        combined_min = std::min(combined_min, vertex_costs[vH]);
      }
    }
    if (hasNaN)
      RCLCPP_ERROR_STREAM(node->get_logger(), "Layer \"" << layer.first << "\" contains NaN values!");
  }

  const float combined_norm = combined_max - combined_min;

  for (auto vH : lethals)
  {
    vertex_costs[vH] = std::numeric_limits<float>::infinity();
  }

  vertex_costs_pub->publish(mesh_msgs_conversions::toVertexCostsStamped(vertex_costs, "Combined Costs", global_frame, uuid_str, map_stamp));

  hasNaN = false;

  RCLCPP_INFO_STREAM(node->get_logger(), "Layer weighting factor is: " << layer_factor);
  for (auto eH : mesh_ptr->edges())
  {
    // Get both Vertices of the current Edge
    std::array<lvr2::VertexHandle, 2> eH_vHs = mesh_ptr->getVerticesOfEdge(eH);
    const lvr2::VertexHandle& vH1 = eH_vHs[0];
    const lvr2::VertexHandle& vH2 = eH_vHs[1];
    // Get the Riskiness for the current Edge (the maximum value from both
    // Vertices)
    if (layer_factor != 0)
    {
      if (std::isinf(vertex_costs[vH1]) || std::isinf(vertex_costs[vH2]))
      {
        edge_weights[eH] = edge_distances[eH];
        // edge_weights[eH] = std::numeric_limits<float>::infinity();
      }
      else
      {
        float cost_diff = std::fabs(vertex_costs[vH1] - vertex_costs[vH2]);

        float vertex_factor = layer_factor * cost_diff;
        if (std::isnan(vertex_factor))
          RCLCPP_INFO_STREAM(node->get_logger(), "NaN: v1:" << vertex_costs[vH1] << " v2:" << vertex_costs[vH2]
                                     << " vertex_factor:" << vertex_factor << " cost_diff:" << cost_diff);
        edge_weights[eH] = edge_distances[eH] * (1 + vertex_factor);
      }
    }
    else
    {
      edge_weights[eH] = edge_distances[eH];
    }
  }

  RCLCPP_INFO(node->get_logger(), "Successfully combined costs!");
}

void MeshMap::findLethalByContours(const int& min_contour_size, std::set<lvr2::VertexHandle>& lethals)
{
  int size = lethals.size();
  std::vector<std::vector<lvr2::VertexHandle>> contours;
  findContours(contours, min_contour_size);
  for (auto contour : contours)
  {
    lethals.insert(contour.begin(), contour.end());
  }
  RCLCPP_INFO_STREAM(node->get_logger(), "Found " << lethals.size() - size << " lethal vertices as contour vertices");
}

void MeshMap::findContours(std::vector<std::vector<lvr2::VertexHandle>>& contours, int min_contour_size)
{
  RCLCPP_INFO_STREAM(node->get_logger(), "Find contours...");

  std::vector<std::vector<lvr2::VertexHandle>> tmp_contours;

  array<lvr2::OptionalFaceHandle, 2> facepair;
  lvr2::SparseEdgeMap<bool> usedEdges(false);
  for (auto eHStart : mesh_ptr->edges())
  {
    lvr2::SparseVertexMap<bool> usedVertices(false);
    lvr2::SparseEdgeMap<bool> local_usedEdges(false);
    int count = 0;

    // Look for border Edges
    facepair = mesh_ptr->getFacesOfEdge(eHStart);

    // If border Edge found
    if ((!facepair[0] || !facepair[1]) && !usedEdges[eHStart])
    {
    std:
      vector<lvr2::VertexHandle> contour;
      // Set vector which links to the following Edge
      array<lvr2::VertexHandle, 2> vertexPair = mesh_ptr->getVerticesOfEdge(eHStart);
      lvr2::VertexHandle vH = vertexPair[1];
      vector<lvr2::EdgeHandle> curEdges;
      lvr2::EdgeHandle eHTemp = eHStart;
      bool moving = true;
      bool vertex_flag = false;

      // While the conotur did not come full circle
      while (moving)
      {
        moving = false;
        usedEdges.insert(eHTemp, true);
        local_usedEdges.insert(eHTemp, true);
        // Set vector which links to the following Edge
        vertexPair = mesh_ptr->getVerticesOfEdge(eHTemp);
        // Eliminate the possibility to choose the previous Vertex
        if (vH != vertexPair[0])
        {
          vH = vertexPair[0];
        }
        else if (vH != vertexPair[1])
        {
          vH = vertexPair[1];
        }

        // Add the current Vertex to the contour
        usedVertices.insert(vH, true);
        count++;
        contour.push_back(vH);
        mesh_ptr->getEdgesOfVertex(vH, curEdges);

        // Look for other edge of vertex that is a border Edge
        for (auto eHT : curEdges)
        {
          if (!usedEdges[eHT] && !local_usedEdges[eHT])
          {
            facepair = mesh_ptr->getFacesOfEdge(eHT);
            if (!facepair[0] || !facepair[1])
            {
              eHTemp = eHT;
              moving = true;
              continue;
            }
          }
        }
      }
      // Add contour to list of contours
      if (contour.size() > min_contour_size)
      {
        contours.push_back(contour);
      }
    }
  }

  RCLCPP_INFO_STREAM(node->get_logger(), "Found " << contours.size() << " contours.");
}

void MeshMap::setVectorMap(lvr2::DenseVertexMap<mesh_map::Vector>& vector_map)
{
  this->vector_map = vector_map;
}

boost::optional<Vector> MeshMap::directionAtPosition(
    const lvr2::VertexMap<lvr2::BaseVector<float>>& vector_map,
    const std::array<lvr2::VertexHandle, 3>& vertices,
    const std::array<float, 3>& barycentric_coords)
{
  const auto& a = vector_map.get(vertices[0]);
  const auto& b = vector_map.get(vertices[1]);
  const auto& c = vector_map.get(vertices[2]);

  if (a || b || c)
  {
    lvr2::BaseVector<float> vec;
    if (a) vec += a.get() * barycentric_coords[0];
    if (b) vec += b.get() * barycentric_coords[1];
    if (c) vec += c.get() * barycentric_coords[2];
    if (std::isfinite(vec.x) && std::isfinite(vec.y) && std::isfinite(vec.z))
      return vec;
    else
      RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 300, "vector map contains invalid vectors!");
  }
  else
  {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 300, "vector map does not contain any of the corresponding vectors");
  }
  return boost::none;
}

float MeshMap::costAtPosition(const std::array<lvr2::VertexHandle, 3>& vertices,
                              const std::array<float, 3>& barycentric_coords)
{
  return costAtPosition(vertex_costs, vertices, barycentric_coords);
}

float MeshMap::costAtPosition(const lvr2::DenseVertexMap<float>& costs,
                              const std::array<lvr2::VertexHandle, 3>& vertices,
                              const std::array<float, 3>& barycentric_coords)
{
  const auto& a = costs.get(vertices[0]);
  const auto& b = costs.get(vertices[1]);
  const auto& c = costs.get(vertices[2]);

  if (a && b && c)
  {
    std::array<float, 3> costs = { a.get(), b.get(), c.get() };
    return mesh_map::linearCombineBarycentricCoords(costs, barycentric_coords);
  }
  return std::numeric_limits<float>::quiet_NaN();
}

void MeshMap::publishDebugPoint(const Vector pos, const std_msgs::msg::ColorRGBA& color, const std::string& name)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = mapFrame();
  marker.header.stamp = node->now();
  marker.ns = name;
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  geometry_msgs::msg::Vector3 scale;
  scale.x = 0.05;
  scale.y = 0.05;
  scale.z = 0.05;
  marker.scale = scale;

  geometry_msgs::msg::Pose p;
  p.position.x = pos.x;
  p.position.y = pos.y;
  p.position.z = pos.z;
  marker.pose = p;
  marker.color = color;
  marker_pub->publish(marker);
}

void MeshMap::publishDebugFace(const lvr2::FaceHandle& face_handle, const std_msgs::msg::ColorRGBA& color,
                               const std::string& name)
{
  const auto& vertices = mesh_ptr->getVerticesOfFace(face_handle);
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = mapFrame();
  marker.header.stamp = node->now();
  marker.ns = name;
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  geometry_msgs::msg::Vector3 scale;
  scale.x = 1.0;
  scale.y = 1.0;
  scale.z = 1.0;
  marker.scale = scale;

  for (auto vertex : vertices)
  {
    auto& pos = mesh_ptr->getVertexPosition(vertex);
    geometry_msgs::msg::Point p;
    p.x = pos.x;
    p.y = pos.y;
    p.z = pos.z;
    marker.points.push_back(p);
    marker.colors.push_back(color);
  }
  marker_pub->publish(marker);
}

void MeshMap::publishVectorField(const std::string& name,
                                 const lvr2::DenseVertexMap<lvr2::BaseVector<float>>& vector_map,
                                 const bool publish_face_vectors)
{
  publishVectorField(name, vector_map, vertex_costs, {}, publish_face_vectors);
}

void MeshMap::publishCombinedVectorField()
{
  lvr2::DenseVertexMap<Vector> vertex_vectors;
  lvr2::DenseFaceMap<Vector> face_vectors;

  vertex_vectors.reserve(mesh_ptr->nextVertexIndex());
  face_vectors.reserve(mesh_ptr->nextFaceIndex());

  for (const auto& [_, layer] : loaded_layers)
  {
    lvr2::DenseFaceMap<uint8_t> vector_field_faces(mesh_ptr->nextFaceIndex(), 0);
    auto opt_vec_map = layer->vectorMap();
    if (!opt_vec_map)
      continue;

    const auto& vecs = opt_vec_map.get();
    for (auto vH : vecs)
    {
      auto opt_val = vertex_vectors.get(vH);
      vertex_vectors.insert(vH, opt_val ? opt_val.get() + vecs[vH] : vecs[vH]);
      for (auto fH : mesh_ptr->getFacesOfVertex(vH))
        vector_field_faces[fH]++;
    }

    for (auto fH : vector_field_faces)
    {
      if (vector_field_faces[fH] != 3)
        continue;

      const auto& vertices = mesh_ptr->getVertexPositionsOfFace(fH);
      const auto& vertex_handles = mesh_ptr->getVerticesOfFace(fH);
      mesh_map::Vector center = (vertices[0] + vertices[1] + vertices[2]) / 3;
      std::array<float, 3> barycentric_coords;
      float dist;
      if (mesh_map::projectedBarycentricCoords(center, vertices, barycentric_coords, dist))
      {
        auto opt_val = face_vectors.get(fH);
        auto vec_at = layer->vectorAt(vertex_handles, barycentric_coords);
        if (vec_at != Vector())
        {
          face_vectors.insert(fH, opt_val ? opt_val.get() + vec_at : vec_at);
        }
      }
    }
  }
}

void MeshMap::publishVectorField(const std::string& name,
                                 const lvr2::DenseVertexMap<lvr2::BaseVector<float>>& vector_map,
                                 const lvr2::DenseVertexMap<float>& values,
                                 const std::function<float(float)>& cost_function, const bool publish_face_vectors)
{
  const auto& mesh = this->mesh();
  const auto& vertex_costs = vertexCosts();
  const auto& face_normals = faceNormals();

  visualization_msgs::msg::Marker vector_field;

  geometry_msgs::msg::Pose pose;
  pose.position.x = pose.position.y = pose.position.z = 0;
  pose.orientation.x = pose.orientation.y = pose.orientation.z = 0;
  pose.orientation.w = 1;
  vector_field.pose = pose;

  vector_field.type = visualization_msgs::msg::Marker::LINE_LIST;
  vector_field.header.frame_id = mapFrame();
  vector_field.header.stamp = node->now();
  vector_field.ns = name;
  vector_field.scale.x = 0.01;
  vector_field.color.a = 1;
  vector_field.id = 0;

  vector_field.colors.reserve(2 * vector_map.numValues());
  vector_field.points.reserve(2 * vector_map.numValues());

  unsigned int cnt = 0;
  unsigned int faces = 0;

  lvr2::DenseFaceMap<uint8_t> vector_field_faces(mesh.numFaces(), 0);
  std::set<lvr2::FaceHandle> complete_faces;

  for (auto vH : vector_map)
  {
    const auto& dir_vec = vector_map[vH];
    const float len2 = dir_vec.length2();
    if (len2 == 0 || !std::isfinite(len2))
    {
      RCLCPP_DEBUG_STREAM_THROTTLE(node->get_logger(), *node->get_clock(), 300, "Found invalid direction vector in vector field \"" << name << "\". Ignoring it!");
      continue;
    }

    auto u = mesh.getVertexPosition(vH);
    auto v = u + dir_vec * 0.1;

    u.z = u.z + 0.01;
    v.z = v.z + 0.01;

    if (!std::isfinite(u.x) || !std::isfinite(u.y) || !std::isfinite(u.z) || !std::isfinite(v.x) ||
        !std::isfinite(v.y) || !std::isfinite(v.z))
    {
      continue;
    }
    vector_field.points.push_back(toPoint(u));
    vector_field.points.push_back(toPoint(v));

    const float value = cost_function ? cost_function(values[vH]) : values[vH];
    std_msgs::msg::ColorRGBA color = getRainbowColor(value);
    vector_field.colors.push_back(color);
    vector_field.colors.push_back(color);

    cnt++;
    // vector.header.seq = cnt;
    // vector.id = cnt++;
    // vector_field.markers.push_back(vector);
    try
    {
      for (auto fH : mesh.getFacesOfVertex(vH))
      {
        if (++vector_field_faces[fH] == 3)
        {
          faces++;
          complete_faces.insert(fH);
        }
      }
    }
    catch (lvr2::PanicException exception)
    {
      invalid.insert(vH, true);
    }
  }

  size_t invalid_cnt = 0;
  for (auto vH : invalid)
  {
    if (invalid[vH])
      invalid_cnt++;
  }

  if (invalid_cnt > 0)
  {
    RCLCPP_WARN_STREAM(node->get_logger(), "Found " << invalid_cnt << " non manifold vertices!");
  }
  RCLCPP_INFO_STREAM(node->get_logger(), "Found " << faces << " complete vector faces!");

  if (publish_face_vectors)
  {
    vector_field.points.reserve(faces + cnt);

    for (auto fH : complete_faces)
    {
      const auto& vertices = mesh.getVertexPositionsOfFace(fH);
      const auto& vertex_handles = mesh.getVerticesOfFace(fH);
      mesh_map::Vector center = (vertices[0] + vertices[1] + vertices[2]) / 3;
      std::array<float, 3> barycentric_coords;
      float dist;
      if (mesh_map::projectedBarycentricCoords(center, vertices, barycentric_coords, dist))
      {
        boost::optional<mesh_map::Vector> dir_opt = directionAtPosition(vector_map, vertex_handles, barycentric_coords);
        if (dir_opt)
        {
          const float& cost = costAtPosition(values, vertex_handles, barycentric_coords);
          const float& value = cost_function ? cost_function(cost) : cost;

          // vector.color = getRainbowColor(value);
          // vector.pose = mesh_map::calculatePoseFromDirection(
          //    center, dir_opt.get(), face_normals[fH]);

          auto u = center;
          auto v = u + dir_opt.get() * 0.1;

          if (!std::isfinite(u.x) || !std::isfinite(u.y) || !std::isfinite(u.z) || !std::isfinite(v.x) ||
              !std::isfinite(v.y) || !std::isfinite(v.z))
          {
            continue;
          }

          // vector_field.header.seq = cnt;
          // vector_field.id = cnt++;
          vector_field.points.push_back(toPoint(u));
          vector_field.points.push_back(toPoint(v));

          std_msgs::msg::ColorRGBA color = getRainbowColor(value);
          vector_field.colors.push_back(color);
          vector_field.colors.push_back(color);
        }
        else
        {
          RCLCPP_ERROR_STREAM_THROTTLE(node->get_logger(), *node->get_clock(), 300, "Could not compute the direction!");
        }
      }
      else
      {
        RCLCPP_ERROR_STREAM_THROTTLE(node->get_logger(), *node->get_clock(), 300, "Could not compute the barycentric coords!");
      }
    }
  }
  vector_field_pub->publish(vector_field);
  RCLCPP_INFO_STREAM(node->get_logger(), "Published vector field \"" << name << "\" with " << cnt << " elements.");
}

bool MeshMap::inTriangle(const Vector& pos, const lvr2::FaceHandle& face, const float& dist)
{
  const auto& vertices = mesh_ptr->getVerticesOfFace(face);
  return mesh_map::inTriangle(pos, mesh_ptr->getVertexPosition(vertices[0]), mesh_ptr->getVertexPosition(vertices[1]),
                              mesh_ptr->getVertexPosition(vertices[2]), dist, 0.0001);
}

boost::optional<std::tuple<lvr2::FaceHandle, std::array<Vector, 3>, std::array<float, 3>>>
  MeshMap::searchNeighbourFaces(
    const Vector& pos, const lvr2::FaceHandle& face,
    const float& max_radius, const float& max_dist)
{
  std::list<lvr2::FaceHandle> possible_faces;
  possible_faces.push_back(face);
  std::list<lvr2::FaceHandle>::iterator face_iter = possible_faces.begin();

  Vector center(0, 0, 0);
  const auto& start_vertices = mesh_ptr->getVertexPositionsOfFace(face);
  for (auto vertex : start_vertices)
  {
    center += vertex;
  }
  center /= 3;

  float vertex_center_max = 0;
  for (auto vertex : start_vertices)
  {
    vertex_center_max = std::max(vertex_center_max, vertex.distance(center));
  }

  float ext_radius = max_radius + vertex_center_max;
  float max_radius_sq = ext_radius * ext_radius;

  lvr2::SparseFaceMap<bool> in_list_map;
  in_list_map.insert(face, true);

  std::array<float, 3> bary_coords;

  while (possible_faces.end() != face_iter)
  {
    const auto& vertices = mesh_ptr->getVertexPositionsOfFace(*face_iter);
    float dist;
    if (mesh_map::projectedBarycentricCoords(pos, vertices, bary_coords, dist) && std::fabs(dist) < max_dist)
    {
      return std::make_tuple(*face_iter, vertices, bary_coords);
    }
    else
    {
      const auto& vertices = mesh_ptr->getVerticesOfFace(*face_iter);
      for (auto vertex : vertices)
      {
        if (center.distance2(mesh_ptr->getVertexPosition(vertex)) < max_radius_sq)
        {
          try
          {
            const auto& nn_faces = mesh_ptr->getFacesOfVertex(vertex);
            for (auto nn_face : nn_faces)
            {
              if (!in_list_map.containsKey(nn_face))
              {
                possible_faces.push_back(nn_face);
                in_list_map.insert(nn_face, true);
              }
            }
          }
          catch (lvr2::PanicException exception)
          {
            // TODO handle case properly
          }
        }
      }
      ++face_iter;
    }
  }

  return boost::none;
}

bool MeshMap::meshAhead(mesh_map::Vector& pos, lvr2::FaceHandle& face, const float& step_size)
{
  std::array<float, 3> bary_coords;
  float dist;
  // get barycentric coordinates of the current face or the next neighbour face
  if (mesh_map::projectedBarycentricCoords(pos, mesh_ptr->getVertexPositionsOfFace(face), bary_coords, dist))
  {

  }
  else if (auto search_res_opt = searchNeighbourFaces(pos, face, step_size, 0.4))
  {
    auto search_res = *search_res_opt;
    face = std::get<0>(search_res);
    std::array<Vector, 3> vertices = std::get<1>(search_res);
    bary_coords = std::get<2>(search_res);

    // project position onto surface
    pos = mesh_map::linearCombineBarycentricCoords(vertices, bary_coords);
  }
  else
  {
    return false;
  }
  const auto& opt_dir = directionAtPosition(vector_map, mesh_ptr->getVerticesOfFace(face), bary_coords);
  if (opt_dir)
  {
    Vector dir = opt_dir.get().normalized();
    std::array<lvr2::VertexHandle, 3> handels = mesh_ptr->getVerticesOfFace(face);
    // iter over all layer vector fields
    for (auto layer : loaded_layers)
    {
      dir += layer.second->vectorAt(handels, bary_coords);
    }
    dir.normalize();
    pos += dir * step_size;
    return true;
  }
  return false;
}

lvr2::OptionalFaceHandle MeshMap::getContainingFace(Vector& position, const float& max_dist)
{
  auto search_result = searchContainingFace(position, max_dist);
  if(search_result)
    return std::get<0>(*search_result);
  return lvr2::OptionalFaceHandle();
}

boost::optional<std::tuple<lvr2::FaceHandle, std::array<mesh_map::Vector , 3>,
    std::array<float, 3>>> MeshMap::searchContainingFace(
    Vector& query_point, const float& max_dist)
{
  if(auto vH_opt = getNearestVertexHandle(query_point))
  {
    auto vH = vH_opt.unwrap();
    float lowest_distance_found = std::numeric_limits<float>::max();
    std::array<Vector, 3> closest_face_vertices;
    std::array<float, 3> bary_coords_on_closest_face;
    lvr2::OptionalFaceHandle opt_closest_face_handle;
    for(auto current_face_handle : mesh_ptr->getFacesOfVertex(vH))
    {
      const auto& current_vertices = mesh_ptr->getVertexPositionsOfFace(current_face_handle);
      float distance_to_current_face = 0;
      std::array<float, 3> current_bary_coords;
      const bool is_query_point_in_current_face = mesh_map::projectedBarycentricCoords(query_point, current_vertices, current_bary_coords, distance_to_current_face);

      if(is_query_point_in_current_face && distance_to_current_face < lowest_distance_found)
      {
        lowest_distance_found = distance_to_current_face;
        opt_closest_face_handle = current_face_handle;
        closest_face_vertices = current_vertices;
        bary_coords_on_closest_face = current_bary_coords;
      }
    }
    if(opt_closest_face_handle)
    {
      return std::make_tuple(opt_closest_face_handle.unwrap(), closest_face_vertices, bary_coords_on_closest_face);
    }
    RCLCPP_ERROR_STREAM(node->get_logger(), "No containing face found!");
    return boost::none;
  }
  RCLCPP_FATAL_STREAM(node->get_logger(), "Could not find the nearest vertex");
  return boost::none;
}

lvr2::OptionalVertexHandle MeshMap::getNearestVertexHandle(const Vector& pos)
{
  float querry_point[3] = {pos.x, pos.y, pos.z};
  size_t ret_index;
  float out_dist_sqr;
  size_t num_results = kd_tree_ptr->knnSearch(&querry_point[0], 1, &ret_index, &out_dist_sqr);
  return num_results == 0 ? lvr2::OptionalVertexHandle() : lvr2::VertexHandle(ret_index);
}

inline const geometry_msgs::msg::Point MeshMap::toPoint(const Vector& vec)
{
  geometry_msgs::msg::Point p;
  p.x = vec.x;
  p.y = vec.y;
  p.z = vec.z;
  return p;
}

constexpr float kEpsilon = 1e-8;

bool MeshMap::projectedBarycentricCoords(const Vector& p, const lvr2::FaceHandle& triangle,
                                         std::array<float, 3>& barycentric_coords, float& dist)
{
  const auto& face = mesh_ptr->getVertexPositionsOfFace(triangle);
  return mesh_map::projectedBarycentricCoords(p, face, barycentric_coords, dist);
}

mesh_map::AbstractLayer::Ptr MeshMap::layer(const std::string& layer_name)
{
  return std::find_if(loaded_layers.begin(), loaded_layers.end(), 
    [&layer_name](const std::pair<std::string, mesh_map::AbstractLayer::Ptr>& item) {
      return item.first == layer_name;
    })->second;
}

bool MeshMap::barycentricCoords(const Vector& p, const lvr2::FaceHandle& triangle, float& u, float& v, float& w)
{
  const auto& face = mesh_ptr->getVertexPositionsOfFace(triangle);
  return mesh_map::barycentricCoords(p, face[0], face[1], face[2], u, v, w);
}

bool MeshMap::rayTriangleIntersect(const Vector& orig, const Vector& dir, const Vector& v0, const Vector& v1,
                                   const Vector& v2, float& t, float& u, float& v, Vector& p)
{
  // compute plane's normal
  Vector v0v1 = v1 - v0;
  Vector v0v2 = v2 - v0;

  // no need to normalize
  Vector N = v0v1.cross(v0v2);  // N
  float denom = N.dot(N);

  // Step 1: finding P

  // check if ray and plane are parallel ?
  float NdotRayDirection = N.dot(dir);
  if (fabs(NdotRayDirection) < kEpsilon)  // almost 0
    return false;                         // they are parallel so they don't intersect !

  // compute d parameter using equation 2
  float d = N.dot(v0);

  // compute t (equation 3)
  t = (N.dot(orig) + d) / NdotRayDirection;

  // check if the triangle is in behind the ray
  // if (t < 0) return false; // the triangle is behind

  // compute the intersection point using equation 1
  p = orig + dir * t;

  // Step 2: inside-outside test
  Vector C;  // vector perpendicular to triangle's plane

  // edge 0
  Vector edge0 = v1 - v0;
  Vector vp0 = p - v0;
  C = edge0.cross(vp0);
  if (N.dot(C) < 0)
    return false;  // P is on the right side

  // edge 1
  Vector edge1 = v2 - v1;
  Vector vp1 = p - v1;
  C = edge1.cross(vp1);
  if ((u = N.dot(C)) < 0)
    return false;  // P is on the right side

  // edge 2
  Vector edge2 = v0 - v2;
  Vector vp2 = p - v2;
  C = edge2.cross(vp2);
  if ((v = N.dot(C)) < 0)
    return false;  // P is on the right side;

  u /= denom;
  v /= denom;

  return true;  // this ray hits the triangle
}

bool MeshMap::resetLayers()
{
  return true;  // TODO implement
}

void MeshMap::publishCostLayers(const rclcpp::Time& map_stamp)
{
  for (const auto& [layer_name, layer_ptr] : loaded_layers)
  {
    vertex_costs_pub->publish(mesh_msgs_conversions::toVertexCostsStamped(layer_ptr->costs(), mesh_ptr->numVertices(),
                                                           layer_ptr->defaultValue(), layer_name, global_frame,
                                                           uuid_str, map_stamp));
  }
  vertex_costs_pub->publish(mesh_msgs_conversions::toVertexCostsStamped(vertex_costs, "Combined Costs", global_frame, uuid_str, map_stamp));
}

void MeshMap::publishVertexCosts(const lvr2::VertexMap<float>& costs, const std::string& name, const rclcpp::Time& map_stamp)
{
  vertex_costs_pub->publish(
      mesh_msgs_conversions::toVertexCostsStamped(costs, mesh_ptr->numVertices(), 0, name, global_frame, uuid_str, map_stamp));
}

void MeshMap::publishVertexColors(const rclcpp::Time& map_stamp)
{
  using VertexColorMapOpt = lvr2::DenseVertexMapOptional<std::array<uint8_t, 3>>;
  using VertexColorMap = lvr2::DenseVertexMap<std::array<uint8_t, 3>>;
  VertexColorMapOpt vertex_colors_opt = this->mesh_io_ptr->getDenseAttributeMap<VertexColorMap>("vertex_colors");
  if (vertex_colors_opt)
  {
    const VertexColorMap colors = vertex_colors_opt.get();
    mesh_msgs::msg::MeshVertexColorsStamped msg;
    msg.header.frame_id = mapFrame();
    msg.header.stamp = map_stamp;
    msg.uuid = uuid_str;
    msg.mesh_vertex_colors.vertex_colors.reserve(colors.numValues());
    for (auto vH : colors)
    {
      std_msgs::msg::ColorRGBA color_rgba;
      const auto& color_array = colors[vH];
      color_rgba.a = 1;
      color_rgba.r = color_array[0] / 255.0;
      color_rgba.g = color_array[1] / 255.0;
      color_rgba.b = color_array[2] / 255.0;
      msg.mesh_vertex_colors.vertex_colors.push_back(color_rgba);
    }
    this->vertex_colors_pub->publish(msg);
  }
}

rcl_interfaces::msg::SetParametersResult MeshMap::reconfigureCallback(std::vector<rclcpp::Parameter> parameters)
{
  RCLCPP_DEBUG_STREAM(node->get_logger(), "Set parameters callback...");
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  if (first_config)
  {
    first_config = false;
  }
  else if (map_loaded)
  {
    for (const rclcpp::Parameter& param : parameters)
    {
      const auto& param_name = param.get_name();
      if (param_name == MESH_MAP_NAMESPACE + ".min_contour_size")
      {
        min_contour_size = param.as_int();
      }
      else if (param_name == MESH_MAP_NAMESPACE + ".layer_factor")
      {
        layer_factor = param.as_double();
      }
      else if (param_name == MESH_MAP_NAMESPACE + ".cost_limit")
      {
        cost_limit = param.as_double();
        combineVertexCosts(node->now());
        // TODO current implementation should mirror the old behavior; However, it seems like cost_limit and min_contour_size are never used in this class. Only layer_factor is used (in combineVertexCosts). We should probably remove the unused parameters and call combineVertexCosts whenever layer_factor changes.
      }
    }
  }
  return result;
}

const std::string MeshMap::getGlobalFrameID()
{
  return global_frame;
}

} /* namespace mesh_map */
