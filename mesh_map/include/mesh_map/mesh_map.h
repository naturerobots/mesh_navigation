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
#include <mutex>
#include <atomic>

namespace mesh_map{

class MeshMap
{
 public:

  typedef lvr2::Normal<float> NormalType;
  typedef lvr2::BaseVector<float> Vector;

  typedef boost::shared_ptr<MeshMap> Ptr;

  MeshMap(tf::TransformListener& tf);

  bool readMap();

  bool readMap(const std::string& mesh_map, const std::string& mesh_part);

  bool loadLayerPlugins();

  bool initLayerPlugins();

  inline Vector toVector(const geometry_msgs::Point& point);

  void cancelPlanning(){cancel_planning = false;}

  bool pathPlanning(
      const geometry_msgs::PoseStamped& start,
      const geometry_msgs::PoseStamped& goal,
      std::vector<geometry_msgs::PoseStamped>& plan,
      bool fmm = true);

  bool dijkstra(
      const Vector& start,
      const Vector& goal,
      std::list<lvr2::VertexHandle>& path);

  bool waveFrontPropagation(
      const Vector& start,
      const Vector& goal,
      std::list<std::pair<Vector, lvr2::FaceHandle>>& path);

  inline bool waveFrontPropagation(
      const Vector& start,
      const Vector& goal,
      const lvr2::DenseEdgeMap<float>& edge_weights,
      const lvr2::DenseVertexMap<float>& costs,
      std::list<std::pair<Vector, lvr2::FaceHandle>>& path,
      lvr2::DenseVertexMap<float>& distances,
      lvr2::DenseVertexMap<lvr2::VertexHandle>& predecessors);

  inline bool waveFrontUpdate(
      lvr2::DenseVertexMap<float>& distances,
      const lvr2::DenseEdgeMap<float>& edge_weights,
      const lvr2::VertexHandle& v1,
      const lvr2::VertexHandle& v2,
      const lvr2::VertexHandle& v3);

  lvr2::OptionalVertexHandle getNearestVertexHandle(const Vector &pos);

  lvr2::OptionalFaceHandle getContainingFaceHandle(const Vector &pos);

  void reconfigureCallback(mesh_map::MeshMapConfig& config, uint32_t level);

  void combineVertexCosts();

  inline Vector projectVectorOntoPlane(const Vector &vec, const Vector &ref, const NormalType &normal);

  void getMinMax(const lvr2::VertexMap<float>& map, float& min, float& max);

  void findLethalAreas(
      const int min_contour_size,
      const float height_diff_threshold,
      const float roughness_threshold);

  geometry_msgs::Pose calculatePoseFromDirection(
      const Vector& position,
      const Vector& direction,
      const NormalType& normal);

  geometry_msgs::Pose calculatePoseFromPosition(
      const Vector& current,
      const Vector& next,
      const NormalType& normal);

  void findContours(
      std::vector<std::vector<lvr2::VertexHandle> >& contours,
      int min_contour_size);

  void publishCostLayers();

  void publishVectorField();

  bool barycentricCoords(const Vector &p, const lvr2::FaceHandle &triangle, float &u, float &v);

  bool barycentricCoords(const Vector &p, const Vector &v0, const Vector &v1, const Vector &v2, float &u, float &v);

  void findLethalInLayer(
      const lvr2::DenseVertexMap<float>& layer,
      const float& threshold,
      std::set<lvr2::VertexHandle>& lethals);

  void layerChanged(const std::string &layer_name);

  void computeVectorMap();

  void findLethalByContours(
      const int& min_contour_size,
      std::set<lvr2::VertexHandle>& lethals);

  const std::string getGlobalFrameID();

  inline bool isLethal(const lvr2::VertexHandle& vH);

  inline const geometry_msgs::Point toPoint(const Vector& vec);

  bool rayTriangleIntersect(
      const Vector &orig, const Vector &dir,
      const Vector &v0, const Vector &v1, const Vector &v2,
      float &t, float &u, float &v, Vector &p);

  bool resetLayers();

 private:
  std::shared_ptr<lvr2::AttributeMeshIOBase> mesh_io_ptr;
  std::shared_ptr<lvr2::HalfEdgeMesh<Vector>> mesh_ptr;

  pluginlib::ClassLoader<mesh_map::AbstractLayer> layer_loader;

  std::map<std::string, mesh_map::AbstractLayer::Ptr> layer_names;

  std::vector<std::pair<std::string, mesh_map::AbstractLayer::Ptr>> layers;

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
  lvr2::DenseVertexMap<float> direction;
  // predecessors while wave propagation
  lvr2::DenseVertexMap<lvr2::VertexHandle> predecessors;
  lvr2::DenseVertexMap<lvr2::FaceHandle> cutting_faces;

  lvr2::DenseVertexMap<Vector> vector_map;

  // edge vertex distances
  lvr2::DenseEdgeMap<float> edge_distances;
  lvr2::DenseEdgeMap<float> edge_weights;


  lvr2::DenseFaceMap<NormalType> face_normals;
  lvr2::DenseVertexMap<NormalType> vertex_normals;

  ros::Publisher vertex_costs_pub;
  ros::Publisher mesh_geometry_pub;
  ros::Publisher path_pub;
  ros::Publisher vector_pub;

  // Server for Reconfiguration
  boost::shared_ptr<dynamic_reconfigure::Server<mesh_map::MeshMapConfig> > reconfigure_server_ptr;
  dynamic_reconfigure::Server<mesh_map::MeshMapConfig>::CallbackType config_callback;
  bool first_config;
  bool map_loaded;
  MeshMapConfig config;

  ros::NodeHandle private_nh;

  tf::TransformListener& tf_listener;

  std::string uuid_str;

  std::mutex layer_mtx;

  std::atomic_bool cancel_planning;
};

} /* namespace mesh_map */


#endif //MESH_NAVIGATION__MESH_MAP_H
