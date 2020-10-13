/*
 *  Copyright 2020, Malte kl. Piening
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
 *    Malte kl. Piening <mklpiening@uni-osnabrueck.de>
 *
 */

#ifndef MESH_NAVIGATION_MMP_PLANNER_H
#define MESH_NAVIGATION_MMP_PLANNER_H

// #define EXPORT_MESH

#include <mbf_mesh_core/mesh_planner.h>
#include <mbf_msgs/GetPathResult.h>
#include <mesh_map/mesh_map.h>
#include <mmp_planner/MMPPlannerConfig.h>
#include <nav_msgs/Path.h>
#include <geodesic_algorithm_exact.h>

namespace mmp_planner
{
class MMPPlanner : public mbf_mesh_core::MeshPlanner
{
public:
  typedef boost::shared_ptr<mmp_planner::MMPPlanner> Ptr;

  MMPPlanner();
  virtual ~MMPPlanner();

  virtual uint32_t makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                            double tolerance, std::vector<geometry_msgs::PoseStamped>& plan, double& cost,
                            std::string& message);

  virtual bool cancel();
  virtual bool initialize(const std::string& name, const boost::shared_ptr<mesh_map::MeshMap>& mesh_map_ptr);

protected:
  void reconfigureCallback(mmp_planner::MMPPlannerConfig& cfg, uint32_t level);

private:
  mesh_map::MeshMap::Ptr mesh_map;
  geodesic::Mesh geodesic_mesh;

  ros::NodeHandle private_nh;

  ros::Publisher path_pub;

  boost::shared_ptr<dynamic_reconfigure::Server<mmp_planner::MMPPlannerConfig>> reconfigure_server_ptr;
  dynamic_reconfigure::Server<mmp_planner::MMPPlannerConfig>::CallbackType config_callback;
  MMPPlannerConfig config;
  bool geodesic_map_ready = false;
};

}  // namespace mmp_planner

#endif  // MESH_NAVIGATION_MMP_PLANNER_H
