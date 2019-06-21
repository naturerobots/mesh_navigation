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

#ifndef MESH_MAP__MESH_MAP_H
#define MESH_MAP__MESH_MAP_H

#include <lvr2/io/HDF5IO.hpp>
#include <lvr2/geometry/BaseVector.hpp>
#include <tf2_ros/buffer.h>
#include <mesh_msgs/MeshVertexCosts.h>
#include <mesh_map/MeshMapConfig.h>
#include <geometry_msgs/Point.h>
#include <dynamic_reconfigure/server.h>
#include <mesh_map/abstract_layer.h>
#include <pluginlib/class_loader.h>
#include <mutex>
#include <atomic>

namespace mesh_map{


class MeshMap
{
 public:


  typedef boost::shared_ptr<MeshMap> Ptr;

  MeshMap(tf2_ros::Buffer& tf);

  bool readMap();

  bool readMap(const std::string& mesh_map, const std::string& mesh_part);

  bool loadLayerPlugins();

  bool initLayerPlugins();

  lvr2::OptionalVertexHandle getNearestVertexHandle(const Vector &pos);

  lvr2::OptionalFaceHandle getContainingFaceHandle(const Vector &pos);

  void reconfigureCallback(mesh_map::MeshMapConfig& config, uint32_t level);

  void combineVertexCosts();


  void findLethalAreas(
      const int min_contour_size,
      const float height_diff_threshold,
      const float roughness_threshold);

  void findContours(
      std::vector<std::vector<lvr2::VertexHandle> >& contours,
      int min_contour_size);

  void publishVertexCosts(const lvr2::VertexMap<float>& costs, const std::string& name);

  void publishCostLayers();

  bool barycentricCoords(const Vector &p, const lvr2::FaceHandle &triangle, float &u, float &v);

  void layerChanged(const std::string &layer_name);

  void findLethalByContours(
      const int& min_contour_size,
      std::set<lvr2::VertexHandle>& lethals);

  const std::string getGlobalFrameID();

  inline bool isLethal(const lvr2::VertexHandle& vH);

  inline const geometry_msgs::Point toPoint(const Vector& vec);

  Vector directionAtPosition(lvr2::FaceHandle current_face, Vector pos);

  float costAtPosition(lvr2::FaceHandle current_face, Vector pos);

  bool rayTriangleIntersect(
      const Vector &orig, const Vector &dir,
      const Vector &v0, const Vector &v1, const Vector &v2,
      float &t, float &u, float &v, Vector &p);

  bool resetLayers();

  const lvr2::DenseVertexMap<mesh_map::Vector>& getVectorMap(){return vector_map;};

  const lvr2::HalfEdgeMesh<Vector>& mesh(){return *mesh_ptr;}

  const lvr2::DenseVertexMap<float>& vertexCosts(){return vertex_costs;}

  const lvr2::DenseVertexMap<float>& getPotential(){return potential;}

  const std::string& mapFrame(){return global_frame;}

  const lvr2::DenseFaceMap<Normal>& faceNormals(){return face_normals;}

  const lvr2::DenseVertexMap<Normal>& vertexNormals(){return vertex_normals;}

  const lvr2::DenseEdgeMap<float>& edgeWeights(){return edge_weights;}

  void setVectorMap(lvr2::DenseVertexMap<mesh_map::Vector>& vector_map);

  std::shared_ptr<lvr2::AttributeMeshIOBase> mesh_io_ptr;
  std::shared_ptr<lvr2::HalfEdgeMesh<Vector>> mesh_ptr;

 private:

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

  lvr2::DenseVertexMap<mesh_map::Vector> vector_map;

    // edge vertex distances
  lvr2::DenseEdgeMap<float> edge_distances;
  lvr2::DenseEdgeMap<float> edge_weights;


  lvr2::DenseFaceMap<Normal> face_normals;
  lvr2::DenseVertexMap<Normal> vertex_normals;

  ros::Publisher vertex_costs_pub;
  ros::Publisher mesh_geometry_pub;

  // Server for Reconfiguration
  boost::shared_ptr<dynamic_reconfigure::Server<mesh_map::MeshMapConfig> > reconfigure_server_ptr;
  dynamic_reconfigure::Server<mesh_map::MeshMapConfig>::CallbackType config_callback;
  bool first_config;
  bool map_loaded;
  MeshMapConfig config;

  ros::NodeHandle private_nh;

  tf2_ros::Buffer& tf_buffer;

  std::string uuid_str;

  std::mutex layer_mtx;

};

} /* namespace mesh_map */


#endif //MESH_NAVIGATION__MESH_MAP_H
