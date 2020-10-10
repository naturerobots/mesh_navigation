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

// TODO fix lvr2 missing imports
#include <lvr2/geometry/Handles.hpp>

using namespace std;

#include <unordered_set>
#include <lvr2/util/Meap.hpp>
#include <lvr_ros/colors.h>
#include <mbf_msgs/GetPathResult.h>
#include <mesh_map/util.h>
#include <pluginlib/class_list_macros.h>

#include <mmp_planner/mmp_planner.h>

PLUGINLIB_EXPORT_CLASS(mmp_planner::MMPPlanner, mbf_mesh_core::MeshPlanner);

namespace mmp_planner
{
MMPPlanner::MMPPlanner()
{
}

MMPPlanner::~MMPPlanner()
{
}

uint32_t MMPPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                              double tolerance, std::vector<geometry_msgs::PoseStamped>& plan, double& cost,
                              std::string& message)
{
  // search points
  std::size_t source_idx = 0;
  std::size_t target_idx = 42;

  // setup points
  geodesic::SurfacePoint source(&geodesic_mesh.vertices()[source_idx]);
  geodesic::SurfacePoint target(&geodesic_mesh.vertices()[target_idx]);

  std::vector<geodesic::SurfacePoint> sources(1, source);

  // run propagation
  geodesic::GeodesicAlgorithmExact algorithm(&geodesic_mesh);
  double const distance_limit = geodesic::GEODESIC_INF;        // no limit for propagation
  std::vector<geodesic::SurfacePoint> stop_points(1, target);  // stop propagation when the target is covered
  algorithm.propagate(sources, distance_limit, &stop_points);

  // trace back path
  std::vector<geodesic::SurfacePoint> path;
  algorithm.trace_back(target, path);

  ROS_INFO_STREAM("size: " << path.size());

  return 0;
}

bool MMPPlanner::cancel()
{
  return false;
}

bool MMPPlanner::initialize(const std::string& plugin_name, const boost::shared_ptr<mesh_map::MeshMap>& mesh_map_ptr)
{
  mesh_map = mesh_map_ptr;

  const auto& mesh = mesh_map->mesh();
  const auto& vertex_costs = mesh_map->vertexCosts();
  const auto& invalid = mesh_map->invalid;

  std::vector<double> points;
  std::vector<unsigned> faces;

  size_t filtered_faces = 0;

  std::unordered_map<std::size_t, std::size_t> vertex_remapping;

  std::unordered_map<std::size_t, std::vector<std::size_t>> known_edges;
  for (auto const& fH : mesh.faces())
  {
    array<lvr2::VertexHandle, 3> vertices = mesh.getVerticesOfFace(fH);
    if (!invalid[vertices[0]] && !invalid[vertices[1]] && !invalid[vertices[2]])
    {
      // check whether or not the edge has already been created to filter non 2 manifold edges
      if (known_edges.find(vertices[0].idx()) != known_edges.end() &&
          std::find(known_edges[vertices[0].idx()].begin(), known_edges[vertices[0].idx()].end(), vertices[1].idx()) !=
              known_edges[vertices[0].idx()].end())
      {
        continue;
      }
      if (known_edges.find(vertices[1].idx()) != known_edges.end() &&
          std::find(known_edges[vertices[1].idx()].begin(), known_edges[vertices[1].idx()].end(), vertices[2].idx()) !=
              known_edges[vertices[1].idx()].end())
      {
        continue;
      }
      if (known_edges.find(vertices[2].idx()) != known_edges.end() &&
          std::find(known_edges[vertices[2].idx()].begin(), known_edges[vertices[2].idx()].end(), vertices[0].idx()) !=
              known_edges[vertices[2].idx()].end())
      {
        continue;
      }

      // check vertex angles to remove degenerated faces
      mesh_map::Vector vertex0_position = mesh.getVertexPosition(vertices[0]);
      mesh_map::Vector vertex1_position = mesh.getVertexPosition(vertices[1]);
      mesh_map::Vector vertex2_position = mesh.getVertexPosition(vertices[2]);

      float angle0 =
          acos(vertex0_position.dot(vertex1_position) / (vertex0_position.length() * vertex1_position.length()));
      float angle1 =
          acos(vertex1_position.dot(vertex2_position) / (vertex1_position.length() * vertex2_position.length()));
      float angle2 =
          acos(vertex2_position.dot(vertex0_position) / (vertex2_position.length() * vertex0_position.length()));

      if (isnan(angle0) || angle0 <= 0.001)
      {
        filtered_faces++;
        continue;
      }
      if (isnan(angle1) || angle1 <= 0.001)
      {
        filtered_faces++;
        continue;
      }
      if (isnan(angle2) || angle2 <= 0.001)
      {
        filtered_faces++;
        continue;
      }

      known_edges[vertices[0].idx()].push_back(vertices[1].idx());
      known_edges[vertices[1].idx()].push_back(vertices[2].idx());
      known_edges[vertices[2].idx()].push_back(vertices[0].idx());

      // remap to new vertex indices
      std::size_t vertex0 = 0;
      std::size_t vertex1 = 0;
      std::size_t vertex2 = 0;

      auto vertex0_mapping = vertex_remapping.find(vertices[0].idx());
      if (vertex0_mapping != vertex_remapping.end())
      {
        vertex0 = vertex_remapping[vertices[0].idx()];
      }
      else
      {
        vertex0 = vertex_remapping.size();
        vertex_remapping[vertices[0].idx()] = vertex0;
      }

      auto vertex1_mapping = vertex_remapping.find(vertices[1].idx());
      if (vertex1_mapping != vertex_remapping.end())
      {
        vertex1 = vertex_remapping[vertices[1].idx()];
      }
      else
      {
        vertex1 = vertex_remapping.size();
        vertex_remapping[vertices[1].idx()] = vertex0;
      }

      auto vertex2_mapping = vertex_remapping.find(vertices[2].idx());
      if (vertex2_mapping != vertex_remapping.end())
      {
        vertex2 = vertex_remapping[vertices[2].idx()];
      }
      else
      {
        vertex2 = vertex_remapping.size();
        vertex_remapping[vertices[2].idx()] = vertex0;
      }

      // add face
      faces.push_back(vertices[0].idx());
      faces.push_back(vertices[1].idx());
      faces.push_back(vertices[2].idx());
    }
  }

  // points.resize(vertex_remapping.size() * 3);
  for (auto const& vH : mesh.vertices())
  {
    auto vertex_index = vertex_remapping.find(vH.idx());
    if (vertex_index == vertex_remapping.end())
    {
        // continue;
    }

    mesh_map::Vector vertex = mesh.getVertexPosition(vH);

    // points[vertex_index->second] = vertex.x;
    // points[vertex_index->second] = vertex.y;
    // points[vertex_index->second] = vertex.z;

    points.push_back(vertex.x);
    points.push_back(vertex.y);
    points.push_back(vertex.z);
  }

  ROS_INFO_STREAM(*std::max_element(faces.begin(), faces.end()) << " " << points.size());

  ROS_INFO_STREAM("removed " << filtered_faces << " faces");
  ROS_INFO_STREAM("vertices before: " << mesh.numVertices() << "; vertices after: " << vertex_remapping.size());

  geodesic_mesh.initialize_mesh_data(points, faces);

  private_nh = ros::NodeHandle("~/" + plugin_name);
  path_pub = private_nh.advertise<nav_msgs::Path>("path", 1, true);

  reconfigure_server_ptr = boost::shared_ptr<dynamic_reconfigure::Server<mmp_planner::MMPPlannerConfig>>(
      new dynamic_reconfigure::Server<mmp_planner::MMPPlannerConfig>(private_nh));
  config_callback = boost::bind(&MMPPlanner::reconfigureCallback, this, _1, _2);
  reconfigure_server_ptr->setCallback(config_callback);

  return true;
}

void MMPPlanner::reconfigureCallback(mmp_planner::MMPPlannerConfig& cfg, uint32_t level)
{
  config = cfg;
}

} /* namespace mmp_planner */

