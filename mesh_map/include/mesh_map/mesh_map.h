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

  void combineVertexCosts(
      const float& riskiness_factor,
      const float& roughness_factor,
      const float& height_diff_factor);

  void getMinAndMaxValues(
      float& risk_min, float& risk_max,
      float& rough_min, float& rough_max,
      float& height_min, float& height_max);

  void lethalCostInflation(
      const int min_contour_size,
      const float height_diff_threshold,
      const float roughness_threshold,
      const float inflation_radius,
      const float inscribed_radius,
      const float inscribed_value,
      const float lethal_value);

  void lethalCostInflation(
      const std::set<lvr2::VertexHandle>& lethals,
      float inflation_radius,
      float inscribed_radius,
      float inscribed_value,
      float lethal_value);

  void combineVertexCosts(
      float riskiness_factor,
      float riskiness_norm,
      float roughness_factor,
      float roughness_norm,
      float height_diff_factor,
      float height_diff_norm);

  void findLethalAreas(
      const int min_contour_size,
      const float height_diff_threshold,
      const float roughness_threshold);

  void calculateEdgeWeights(
      float roughness_factor,
      float height_diff_factor);

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

  bool isLethal(const lvr2::VertexHandle& vH);

  bool resetLayers();

 private:
  std::shared_ptr<lvr2::AttributeMeshIOBase> mesh_io_ptr;
  lvr2::HalfEdgeMesh<VectorType> mesh;

  std::string global_frame_;

  std::string mesh_file_;
  std::string mesh_part_;

  // layers
  lvr2::DenseVertexMap<float> roughness_;
  lvr2::DenseVertexMap<float> height_diff_;
  lvr2::DenseVertexMap<float> riskiness_;

  // combined layers
  lvr2::DenseVertexMap<float> vertex_costs_;
  lvr2::DenseVertexMap<float> vertex_distances_;

  // path surface potential
  lvr2::DenseVertexMap<float> potential_;
  // predecessors while wave propagation
  lvr2::DenseVertexMap<lvr2::VertexHandle> predecessors_;

  // edge vertex distances
  lvr2::DenseEdgeMap<float> edge_distances_;
  lvr2::DenseEdgeMap<float> edge_weights_;

  std::set<lvr2::VertexHandle> lethals_;

  lvr2::DenseFaceMap<NormalType> face_normals_;
  lvr2::DenseVertexMap<NormalType> vertex_normals_;

  ros::Publisher vertex_costs_pub_;
  ros::Publisher mesh_geometry_pub_;
  ros::Publisher path_pub_;


  // Server for Reconfiguration
  boost::shared_ptr<dynamic_reconfigure::Server<mesh_map::MeshMapConfig> > reconfigure_server_ptr;
  dynamic_reconfigure::Server<mesh_map::MeshMapConfig>::CallbackType config_callback;
  bool first_config_;
  bool map_loaded_;
  MeshMapConfig config_;

  ros::NodeHandle private_nh_;

  tf::TransformListener& tf_listener_;

  std::string uuid_str_;

};

} /* namespace mesh_map */


#endif //MESH_NAVIGATION__MESH_MAP_H
