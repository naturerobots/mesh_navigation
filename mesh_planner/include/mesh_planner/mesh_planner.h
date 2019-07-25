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

#ifndef MESH_NAVIGATION__MESH_PLANNER_H
#define MESH_NAVIGATION__MESH_PLANNER_H

#include <mbf_mesh_core/mesh_planner.h>
#include <mbf_msgs/GetPathResult.h>
#include <mesh_map/mesh_map.h>
#include <mesh_planner/MeshPlannerConfig.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>

namespace mesh_planner {

class MeshPlanner : public mbf_mesh_core::MeshPlanner {

public:
  typedef boost::shared_ptr<mesh_planner::MeshPlanner> Ptr;

  MeshPlanner();

  /**
   * @brief Destructor
   */
  virtual ~MeshPlanner();

  /**
   * @brief Given a goal pose in the world, compute a plan
   * @param start The start pose
   * @param goal The goal pose
   * @param tolerance If the goal is obstructed, how many meters the planner can
   * relax the constraint in x and y before failing
   * @param plan The plan... filled by the planner
   * @param cost The cost for the the plan
   * @param message Optional more detailed outcome as a string
   * @return Result code as described on GetPath action result:
   *         SUCCESS         = 0
   *         1..9 are reserved as plugin specific non-error results
   *         FAILURE         = 50  # Unspecified failure, only used for old,
   * non-mfb_core based plugins CANCELED        = 51 INVALID_START   = 52
   *         INVALID_GOAL    = 53
   *         NO_PATH_FOUND   = 54
   *         PAT_EXCEEDED    = 55
   *         EMPTY_PATH      = 56
   *         TF_ERROR        = 57
   *         NOT_INITIALIZED = 58
   *         INVALID_PLUGIN  = 59
   *         INTERNAL_ERROR  = 60
   *         71..99 are reserved as plugin specific errors
   */
  virtual uint32_t makePlan(const geometry_msgs::PoseStamped &start,
                            const geometry_msgs::PoseStamped &goal,
                            double tolerance,
                            std::vector<geometry_msgs::PoseStamped> &plan,
                            double &cost, std::string &message);

  /**
   * @brief Requests the planner to cancel, e.g. if it takes too much time.
   * @return True if a cancel has been successfully requested, false if not
   * implemented.
   */
  virtual bool cancel();

  virtual bool
  initialize(const std::string &name,
             const boost::shared_ptr<mesh_map::MeshMap> &mesh_map_ptr);

  lvr2::DenseVertexMap<mesh_map::Vector> getVectorMap();

protected:
  uint32_t waveFrontPropagation(
      const mesh_map::Vector &start, const mesh_map::Vector &goal,
      std::list<std::pair<mesh_map::Vector, lvr2::FaceHandle>> &path);

  uint32_t waveFrontPropagation(
      const mesh_map::Vector &start, const mesh_map::Vector &goal,
      const lvr2::DenseEdgeMap<float> &edge_weights,
      const lvr2::DenseVertexMap<float> &costs,
      std::list<std::pair<mesh_map::Vector, lvr2::FaceHandle>> &path,
      lvr2::DenseVertexMap<float> &distances,
      lvr2::DenseVertexMap<lvr2::VertexHandle> &predecessors);

  inline bool waveFrontUpdate(lvr2::DenseVertexMap<float> &distances,
                              const lvr2::DenseEdgeMap<float> &edge_weights,
                              const lvr2::VertexHandle &v1,
                              const lvr2::VertexHandle &v2,
                              const lvr2::VertexHandle &v3);

  void publishVectorField();

  void computeVectorMap();

  void reconfigureCallback(mesh_planner::MeshPlannerConfig &cfg,
                           uint32_t level);

private:
  mesh_map::MeshMap::Ptr mesh_map;
  std::string name;
  ros::NodeHandle private_nh;
  std::atomic_bool cancel_planning;
  ros::Publisher path_pub;
  ros::Publisher vector_pub;

  std::string map_frame;

  // Server for Reconfiguration
  boost::shared_ptr<
      dynamic_reconfigure::Server<mesh_planner::MeshPlannerConfig>>
      reconfigure_server_ptr;
  dynamic_reconfigure::Server<mesh_planner::MeshPlannerConfig>::CallbackType
      config_callback;
  bool first_config;
  MeshPlannerConfig config;

  // theta angles to the source of the wave front propagation vertices
  lvr2::DenseVertexMap<float> direction;
  // predecessors while wave propagation
  lvr2::DenseVertexMap<lvr2::VertexHandle> predecessors;
  // the face which is cut by line to the source
  lvr2::DenseVertexMap<lvr2::FaceHandle> cutting_faces;
  // stores the current vector map containing vectors pointing to the source
  // (path goal)
  lvr2::DenseVertexMap<mesh_map::Vector> vector_map;
  // potential field or distance values to the source (path goal)
  lvr2::DenseVertexMap<float> potential;
};

} // namespace mesh_planner

#endif // MESH_NAVIGATION__MESH_PLANNER_H
