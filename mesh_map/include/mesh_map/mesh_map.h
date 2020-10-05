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

#ifndef MESH_MAP__MESH_MAP_H
#define MESH_MAP__MESH_MAP_H

#include <atomic>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Point.h>
#include <lvr2/geometry/BaseVector.hpp>
#include <lvr2/io/HDF5IO.hpp>
#include <mesh_map/MeshMapConfig.h>
#include <mesh_map/abstract_layer.h>
#include <mesh_msgs/MeshVertexCosts.h>
#include <mesh_msgs/MeshVertexColors.h>
#include <mutex>
#include <pluginlib/class_loader.h>
#include <std_msgs/ColorRGBA.h>
#include <tf2_ros/buffer.h>

namespace mesh_map
{
class MeshMap
{
public:
  typedef boost::shared_ptr<MeshMap> Ptr;

  MeshMap(tf2_ros::Buffer& tf);

  bool readMap();

  bool readMap(const std::string& uri);

  bool readMap(const std::string& mesh_map, const std::string& mesh_part);

  bool loadLayerPlugins();

  bool initLayerPlugins();

  lvr2::OptionalVertexHandle getNearestVertexHandle(const mesh_map::Vector& pos);

  bool inTriangle(const Vector& pos, const lvr2::FaceHandle& face, const float& dist);

  lvr2::OptionalFaceHandle getContainingFace(Vector& position, const float& max_dist);

  bool searchContainingFace(Vector& pos, lvr2::OptionalFaceHandle& face_handle,
                            std::array<float, 3>& barycentric_coords, const float& max_dist);

  void reconfigureCallback(mesh_map::MeshMapConfig& config, uint32_t level);

  void combineVertexCosts();

  void findLethalAreas(const int min_contour_size, const float height_diff_threshold, const float roughness_threshold);

  void findContours(std::vector<std::vector<lvr2::VertexHandle>>& contours, int min_contour_size);

  void publishVertexCosts(const lvr2::VertexMap<float>& costs, const std::string& name);

  void publishVertexColors();

  void publishCostLayers();

  bool projectedBarycentricCoords(const Vector& p, const lvr2::FaceHandle& triangle,
                                  std::array<float, 3>& barycentric_coords, float& dist);

  bool barycentricCoords(const Vector& p, const lvr2::FaceHandle& triangle, float& u, float& v, float& w);

  void layerChanged(const std::string& layer_name);

  void findLethalByContours(const int& min_contour_size, std::set<lvr2::VertexHandle>& lethals);

  const std::string getGlobalFrameID();

  inline bool isLethal(const lvr2::VertexHandle& vH);

  inline const geometry_msgs::Point toPoint(const Vector& vec);

  boost::optional<Vector> directionAtPosition(const lvr2::VertexMap<lvr2::BaseVector<float>>& vector_map,
                                              const std::array<lvr2::VertexHandle, 3>& vertices,
                                              const std::array<float, 3>& barycentric_coords);

  float costAtPosition(const lvr2::DenseVertexMap<float>& costs, const std::array<lvr2::VertexHandle, 3>& vertices,
                       const std::array<float, 3>& barycentric_coords);

  float costAtPosition(const std::array<lvr2::VertexHandle, 3>& vertices,
                       const std::array<float, 3>& barycentric_coords);

  bool rayTriangleIntersect(const Vector& orig, const Vector& dir, const Vector& v0, const Vector& v1, const Vector& v2,
                            float& t, float& u, float& v, Vector& p);

  bool resetLayers();

  const lvr2::DenseVertexMap<mesh_map::Vector>& getVectorMap()
  {
    return vector_map;
  };

  const lvr2::HalfEdgeMesh<Vector>& mesh()
  {
    return *mesh_ptr;
  }

  const lvr2::DenseVertexMap<float>& vertexCosts()
  {
    return vertex_costs;
  }

  const lvr2::DenseVertexMap<float>& getPotential()
  {
    return potential;
  }

  const std::string& mapFrame()
  {
    return global_frame;
  }

  const lvr2::DenseFaceMap<Normal>& faceNormals()
  {
    return face_normals;
  }

  const lvr2::DenseVertexMap<Normal>& vertexNormals()
  {
    return vertex_normals;
  }

  const lvr2::DenseEdgeMap<float>& edgeWeights()
  {
    return edge_weights;
  }

  const lvr2::DenseEdgeMap<float>& edgeDistances()
  {
    return edge_distances;
  }

  /**
   * Searches in the surrounding triangles for the triangle in which the given
   * position lies.
   * @param pose_vec    Vector of the position which searched for
   * @param face        Face handle from which search begins
   * @param max_radius  The radius in which the controller searches for a consecutive face
   * @param max_dist    The maximum distance to the given face vertices
   * @return            Face handle of the position - empty optional face handle
   * if position could not be found
   */
  bool searchNeighbourFaces(Vector& pos, lvr2::FaceHandle& face, std::array<float, 3>& barycentric_coords,
                            const float& max_radius, const float& max_dist);

  /**
   * Finds the next position given a position vector and its corresponding face
   * handle by following the direction For: look ahead when using mesh gradient
   * @param vec   direction vector from which the next step vector is calculated
   * @param face  face of the direction vector
   * @param step_width The step length to go ahead on the mesh surface
   * @return      new vector (also updates the ahead_face handle to correspond
   * to the new vector)
   */
  bool meshAhead(Vector& vec, lvr2::FaceHandle& face, const float& step_width);

  void setVectorMap(lvr2::DenseVertexMap<mesh_map::Vector>& vector_map);

  void publishDebugPoint(const Vector pos, const std_msgs::ColorRGBA& color, const std::string& name);

  void publishDebugVector(const lvr2::VertexHandle& a, const lvr2::VertexHandle& b, const lvr2::FaceHandle& fh,
                          const double angle, const std_msgs::ColorRGBA& color, const std::string& name);

  void publishDebugFace(const lvr2::FaceHandle& face_handle, const std_msgs::ColorRGBA& color, const std::string& name);

  void publishVectorField(const std::string& name, const lvr2::DenseVertexMap<lvr2::BaseVector<float>>& vector_map,
                          const lvr2::DenseVertexMap<lvr2::FaceHandle>& cutting_faces,
                          const bool publish_face_vectors = false);

  void publishVectorField(const std::string& name, const lvr2::DenseVertexMap<lvr2::BaseVector<float>>& vector_map,
                          const lvr2::DenseVertexMap<lvr2::FaceHandle>& cutting_faces,
                          const lvr2::DenseVertexMap<float>& values,
                          const std::function<float(float)>& cost_function = {},
                          const bool publish_face_vectors = false);

  void publishCombinedVectorField();

  mesh_map::AbstractLayer::Ptr layer(const std::string& layer_name);

  std::shared_ptr<lvr2::AttributeMeshIOBase> mesh_io_ptr;
  std::shared_ptr<lvr2::HalfEdgeMesh<Vector>> mesh_ptr;

  lvr2::DenseVertexMap<bool> invalid;

private:
  pluginlib::ClassLoader<mesh_map::AbstractLayer> layer_loader;

  std::map<std::string, mesh_map::AbstractLayer::Ptr> layer_names;

  std::vector<std::pair<std::string, mesh_map::AbstractLayer::Ptr>> layers;

  std::map<std::string, std::set<lvr2::VertexHandle>> lethal_indices;

  std::set<lvr2::VertexHandle> lethals;

  std::string global_frame;

  std::string srv_url;
  std::string srv_username;
  std::string srv_password;
  std::string mesh_layer;
  float min_roughness;
  float max_roughness;
  float min_height_diff;
  float max_height_diff;
  float bb_min_x;
  float bb_min_y;
  float bb_min_z;
  float bb_max_x;
  float bb_max_y;
  float bb_max_z;

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
  ros::Publisher vertex_colors_pub;
  ros::Publisher mesh_geometry_pub;
  ros::Publisher marker_pub;
  ros::Publisher vector_field_pub;

  // Server for Reconfiguration
  boost::shared_ptr<dynamic_reconfigure::Server<mesh_map::MeshMapConfig>> reconfigure_server_ptr;
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

#endif  // MESH_NAVIGATION__MESH_MAP_H
