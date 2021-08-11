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

#ifndef MESH_NAVIGATION__MESH_PLANNER_H
#define MESH_NAVIGATION__MESH_PLANNER_H

#include <mbf_mesh_core/mesh_planner.h>
#include <mbf_msgs/GetPathResult.h>
#include <mesh_map/mesh_map.h>
#include <cvp_mesh_planner/CVPMeshPlannerConfig.h>
#include <nav_msgs/Path.h>

namespace cvp_mesh_planner
{
class CVPMeshPlanner : public mbf_mesh_core::MeshPlanner
{
public:
  typedef boost::shared_ptr<cvp_mesh_planner::CVPMeshPlanner> Ptr;

  /**
   * @brief Constructor
   */
  CVPMeshPlanner();

  /**
   * @brief Destructor
   */
  virtual ~CVPMeshPlanner();

  /**
   * @brief Compute a continuous vector field and geodesic path on the mesh's surface
   * @param start The start pose
   * @param goal The goal pose
   * @param tolerance The goal tolerance, TODO is currently not used
   * @param plan The computed plan
   * @param cost The computed cost for the plan
   * @param message a detailed outcome message
   * @return result outcome code, see the GetPath action definition
   */
  virtual uint32_t makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                            double tolerance, std::vector<geometry_msgs::PoseStamped>& plan, double& cost,
                            std::string& message);

  /**
   * @brief Requests the planner to cancel, e.g. if it takes too much time.
   * @return true if cancel has been successfully requested, false otherwise
   */
  virtual bool cancel();

  /**
   * @brief Initializes the planner plugin with a user configured name and a shared pointer to the mesh map
   * @param name The user configured name, which is used as namespace for parameters, etc.
   * @param mesh_map_ptr A shared pointer to the mesh map instance to access attributes and helper functions, etc.
   * @return true if the plugin has been initialized successfully
   */
  virtual bool initialize(const std::string& name, const boost::shared_ptr<mesh_map::MeshMap>& mesh_map_ptr);

protected:

  /**
   * @brief Computes a wavefront propagation from the start until it reached the goal
   * @param start The seed of the wave, i.e. the robot's goal pose
   * @param goal The goal of the wavefront, where it will stop propagating
   * @param path The backtracked path
   * @return a ExePath action related outcome code
   */
  uint32_t waveFrontPropagation(const mesh_map::Vector& start, const mesh_map::Vector& goal,
                                std::list<std::pair<mesh_map::Vector, lvr2::FaceHandle>>& path);

  /**
   *
   * @brief Computes a wavefront propagation from the start until it reached the goal
   * @param start The seed of the wave, i.e. the robot's goal pose
   * @param goal The goal of the wavefront, where it will stop propagating
   * @param edge_weights The edge weights map to use for vertex distances in a triangle
   * @param costs The combined vertex costs to use during the propagation
   * @param path The backtracked path
   * @param distances The computed distances
   * @param predecessors The backtracked predecessors
   * @return a ExePath action related outcome code
   */
  uint32_t waveFrontPropagation(const mesh_map::Vector& start, const mesh_map::Vector& goal,
                                const lvr2::DenseEdgeMap<float>& edge_weights, const lvr2::DenseVertexMap<float>& costs,
                                std::list<std::pair<mesh_map::Vector, lvr2::FaceHandle>>& path,
                                lvr2::DenseVertexMap<float>& distances,
                                lvr2::DenseVertexMap<lvr2::VertexHandle>& predecessors);

  /**
   * Single source update step using the Hesse normal form to determine if the direction vector is cutting the current triangle
   * @param distances Distance map to the goal which stores the current state of all distances to the goal
   * @param edge_weights Distances assigned to each edge
   * @param v1 The first vertex of the triangle
   * @param v2 The second vertex of the triangle
   * @param v3 The thrid vertex of the triangle
   * @return true if the newly computed distance is shorter than before and if the current triangle is cut
   */
  inline bool waveFrontUpdateWithS(lvr2::DenseVertexMap<float>& distances,
                                   const lvr2::DenseEdgeMap<float>& edge_weights, const lvr2::VertexHandle& v1,
                                   const lvr2::VertexHandle& v2, const lvr2::VertexHandle& v3);

  /**
   * Fast Marching Method update step using the Law of Cosines to determine if the direction vector is cutting the current triangle
   * @param distances Distance map to the goal which stores the current state of all distances to the goal
   * @param edge_weights Distances assigned to each edge
   * @param v1 The first vertex of the triangle
   * @param v2 The second vertex of the triangle
   * @param v3 The thrid vertex of the triangle
   * @return true if the newly computed distance is shorter than before and if the current triangle is cut
   */
  inline bool waveFrontUpdateFMM(lvr2::DenseVertexMap<float>& distances, const lvr2::DenseEdgeMap<float>& edge_weights,
                              const lvr2::VertexHandle& v1, const lvr2::VertexHandle& v2, const lvr2::VertexHandle& v3);

  /**
   * Single source update step using the Law of Cosines to determine if the direction vector is cutting the current triangle
   * @param distances Distance map to the goal which stores the current state of all distances to the goal
   * @param edge_weights Distances assigned to each edge
   * @param v1 The first vertex of the triangle
   * @param v2 The second vertex of the triangle
   * @param v3 The thrid vertex of the triangle
   * @return true if the newly computed distance is shorter than before and if the current triangle is cut
   */
  inline bool waveFrontUpdate(lvr2::DenseVertexMap<float>& distances, const lvr2::DenseEdgeMap<float>& edge_weights,
                              const lvr2::VertexHandle& v1, const lvr2::VertexHandle& v2, const lvr2::VertexHandle& v3);

  /**
   * @brief Computes the vector field in a post processing. It rotates the predecessor edges by the stored angles
   */
  void computeVectorMap();

  /**
   * @brief Dynamic reconfigure callback
   */
  void reconfigureCallback(cvp_mesh_planner::CVPMeshPlannerConfig& cfg, uint32_t level);

private:

  //! shared pointer to the mesh map
  mesh_map::MeshMap::Ptr mesh_map;

  //! the user defined plugin name
  std::string name;

  //! the private node handle with the user defined namespace (name)
  ros::NodeHandle private_nh;

  //! flag if cancel has been requested
  std::atomic_bool cancel_planning;

  //! publisher for the backtracked path
  ros::Publisher path_pub;

  //! whether to publish the vector field or not
  bool publish_vector_field;

  //! whether to also publish direction vectors at the triangle centers
  bool publish_face_vectors;

  //! the map coordinate frame / system id
  std::string map_frame;

  //! an offset that determines how far beyond the goal (robot's position) is propagated.
  float goal_dist_offset;

  //! shared pointer to dynamic reconfigure server
  boost::shared_ptr<dynamic_reconfigure::Server<cvp_mesh_planner::CVPMeshPlannerConfig>> reconfigure_server_ptr;

  //! dynamic reconfigure callback function binding
  dynamic_reconfigure::Server<cvp_mesh_planner::CVPMeshPlannerConfig>::CallbackType config_callback;

  //! indicates if dynamic reconfigure has been called the first time
  bool first_config;

  //! the current dynamic reconfigure planner configuration
  CVPMeshPlannerConfig config;

  //! theta angles to the source of the wave front propagation
  lvr2::DenseVertexMap<float> direction;

  //! predecessors while wave propagation
  lvr2::DenseVertexMap<lvr2::VertexHandle> predecessors;

  //! the face which is cut by the computed line to the source
  lvr2::DenseVertexMap<lvr2::FaceHandle> cutting_faces;

  //! stores the current vector map containing vectors pointing to the seed
  lvr2::DenseVertexMap<mesh_map::Vector> vector_map;

  //! potential field / scalar distance field to the seed
  lvr2::DenseVertexMap<float> potential;
};

}  // namespace cvp_mesh_planner

#endif  // MESH_NAVIGATION__CVP_MESH_PLANNER_H
