/*
 *  Copyright 2025, Justus Braun
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
 *    Justus Braun <jubraun@uni-osnabrueck.de>
 *
 */

#pragma once

#include <boost/graph/adjacency_list.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/node.hpp>

#include <lvr2/geometry/Handles.hpp>

namespace mesh_map
{

class AbstractLayer;
class MeshMap;

class LayerManager
{
public:

  /**
   *  @brief Read the configured layers and dependencies from the node
   */
  void read_configured_layers(const rclcpp::Node::SharedPtr& node);
  
  /**
   *  @brief Create instances of the configured layers
   *  @return true if at least one layer plugin was loaded successfully.
   */
  bool load_layer_plugins(const rclcpp::Logger& logger);
  
  /**
   *  @brief Initialize all layer plugins and load or compute their values
   *  @return true If all loaded layers were initialized sucessfully
   */
  bool initialize_layer_plugins(const rclcpp::Node::SharedPtr& node, const std::shared_ptr<MeshMap>& map);
  
  /**
   *  @brief Get the plugin type of a layer
   */
  const std::string& get_type(const std::string& name) const
  {
    return types_.at(name);
  }
  
  /**
   *  @brief Get the layer instance by name
   *  @return A pointer to the layer instance, nullptr if no instance was found
   */
  std::shared_ptr<AbstractLayer> get_layer(const std::string& name) const
  {
    const auto it = instances_.find(name);
    if (it == instances_.end())
    {
      return nullptr;
    }
    return it->second;
  }
  
  /**
   *  @brief Access all loaded layers for iteration in order of configuration
   */
  const std::vector<std::string>& loaded_layers()
  {
    return loaded_;
  }
  
  /**
   *  @brief Access all loaded layers for iteration with no guranteed order!
   */
  const std::map<std::string, std::shared_ptr<AbstractLayer>>& layer_instances()
  {
    return instances_;
  }

  /**
   *  @brief Init the layer manager. Stores a reference to the map, which is not used
   *  in the constructor!
   */
  LayerManager(MeshMap& map, const rclcpp::Node::SharedPtr& node);

private:
  
  /**
   *  @brief Handle a layer change by updating the depending layers
   *
   *  @param name The name of the changed layer
   *  @param changed All vertices with changed costs
   *  @param added_lethal All vertices which are now lethal
   *  @param remove_lethal All vertices which are no longer lethal
   */
  void layer_changed(
    const std::string& name,
    const std::set<lvr2::VertexHandle>& changed
  );

  //! Reference to the map to publish cost maps
  MeshMap& map_;

  //! ROS Node
  rclcpp::Node::SharedPtr node_;

  //! Mutex to prevent parallel updates
  std::recursive_mutex layer_mtx_;

  //! Graph containing an directed edge between a and b if a depends on b
  using DependencyGraph = boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS>;
  using Vertex = boost::graph_traits<DependencyGraph>::vertex_descriptor;
  DependencyGraph graph_;

  //! Pluginlib class loader for layers
  pluginlib::ClassLoader<AbstractLayer> loader_;

  //! Configured layer names
  std::vector<std::string> names_;

  //! Loaded layer names
  std::vector<std::string> loaded_;

  //! Maps layer name to graph vertex
  std::map<std::string, Vertex> vertices_;

  //! Maps layer name to type
  std::map<std::string, std::string> types_;

  //! Maps layer name to instance
  std::map<std::string, std::shared_ptr<AbstractLayer>> instances_;

};

} /* namespace mesh_map */
