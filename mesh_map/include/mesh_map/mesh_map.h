/*
 *  Copyright 2019, Sebastian Pütz
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

#ifndef MESH_NAVIGATION__MESH_MAP_H
#define MESH_NAVIGATION__MESH_MAP_H

#include <lvr2/io/HDF5IO.hpp>
#include <lvr2/geometry/BaseVector.hpp>
#include <tf/transform_listener.h>
#include <mesh_msgs/MeshVertexCosts.h>
#include <nav_msgs/Path.h>
#include <mesh_map/MeshMapConfig.h>
#include <dynamic_reconfigure/server.h>
#include <mesh_map/abstract_layer.h>
#include <pluginlib/class_loader.h>


namespace mesh_map{

class MeshMap
{
 public:



  typedef lvr2::Normal<float> NormalType;
  typedef lvr2::BaseVector<float> VectorType;

  typedef boost::shared_ptr<MeshMap> Ptr;

  MeshMap(tf::TransformListener& tf);

  bool readMap();

  bool readMap(const std::string& mesh_map, const std::string& mesh_part);

  bool loadLayerPlugins();

  bool initLayerPlugins();

  inline VectorType toVectorType(const geometry_msgs::Point& point);

  bool pathPlanning(
      const geometry_msgs::PoseStamped& start,
      const geometry_msgs::PoseStamped& goal,
      std::vector<geometry_msgs::PoseStamped>& plan,
      bool fmm = true);

  bool dijkstra(
      const VectorType& start,
      const VectorType& goal,
      std::list<lvr2::VertexHandle>& path);

  bool waveFrontPropagation(
      const VectorType& start,
      const VectorType& goal,
      std::list<lvr2::VertexHandle>& path);

  inline bool waveFrontPropagation(
      const VectorType& start,
      const VectorType& goal,
      const lvr2::DenseEdgeMap<float>& edge_weights,
      const lvr2::DenseVertexMap<float>& costs,
      std::list<lvr2::VertexHandle>& path,
      lvr2::DenseVertexMap<float>& distances,
      lvr2::DenseVertexMap<lvr2::VertexHandle>& predecessors);

  inline bool waveFrontUpdate(
      lvr2::DenseVertexMap<float>& distances,
      const lvr2::DenseEdgeMap<float>& edge_weights,
      const lvr2::VertexHandle& v1,
      const lvr2::VertexHandle& v2,
      const lvr2::VertexHandle& v3);

  lvr2::OptionalVertexHandle getNearestVertexHandle(const VectorType pos);

  void reconfigureCallback(mesh_map::MeshMapConfig& config, uint32_t level);

  void combineVertexCosts();

  void getMinMax(const lvr2::VertexMap<float>& map, float& min, float& max);

  void findLethalAreas(
      const int min_contour_size,
      const float height_diff_threshold,
      const float roughness_threshold);

  bool calculatePose(const VectorType& current,
                     const VectorType& next,
                     const NormalType& normal,
                     geometry_msgs::Pose& pose);
  void findContours(
      std::vector<std::vector<lvr2::VertexHandle> >& contours,
      int min_contour_size);

  void publishCostLayers();

  void findLethalInLayer(
      const lvr2::DenseVertexMap<float>& layer,
      const float& threshold,
      std::set<lvr2::VertexHandle>& lethals);

  void findLethalByContours(
      const int& min_contour_size,
      std::set<lvr2::VertexHandle>& lethals);

  const std::string getGlobalFrameID();

  inline bool isLethal(const lvr2::VertexHandle& vH);

  bool resetLayers();

 private:
  std::shared_ptr<lvr2::AttributeMeshIOBase> mesh_io_ptr;
  std::shared_ptr<lvr2::HalfEdgeMesh<VectorType>> mesh_ptr;

  pluginlib::ClassLoader<mesh_map::AbstractLayer> layer_loader;

  std::vector<std::string> layer_names;

  std::map<std::string, mesh_map::AbstractLayer::Ptr> layers;

  std::map<std::string, std::set<lvr2::VertexHandle>> lethal_indices;

  std::set<lvr2::VertexHandle> lethals;

  std::string global_frame;

  std::string mesh_file;
  std::string mesh_part;

  // combined layers
  lvr2::DenseVertexMap<float> vertex_costs;
  lvr2::DenseVertexMap<float> vertex_distances;

  // path surface potential
  lvr2::DenseVertexMap<float> potential;
  // predecessors while wave propagation
  lvr2::DenseVertexMap<lvr2::VertexHandle> predecessors;

  // edge vertex distances
  lvr2::DenseEdgeMap<float> edge_distances;
  lvr2::DenseEdgeMap<float> edge_weights;


  lvr2::DenseFaceMap<NormalType> face_normals;
  lvr2::DenseVertexMap<NormalType> vertex_normals;

  ros::Publisher vertex_costs_pub;
  ros::Publisher mesh_geometry_pub;
  ros::Publisher path_pub;


  // Server for Reconfiguration
  boost::shared_ptr<dynamic_reconfigure::Server<mesh_map::MeshMapConfig> > reconfigure_server_ptr;
  dynamic_reconfigure::Server<mesh_map::MeshMapConfig>::CallbackType config_callback;
  bool first_config;
  bool map_loaded;
  MeshMapConfig config;

  ros::NodeHandle private_nh;

  tf::TransformListener& tf_listener;

  std::string uuid_str;

};

} /* namespace mesh_map */


#endif //MESH_NAVIGATION__MESH_MAP_H
