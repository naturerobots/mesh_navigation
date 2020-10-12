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
#include <limits>

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
  geodesic::Vertex* source_vertex = nullptr;
  geodesic::Vertex* target_vertex = nullptr;
  double source_distance = std::numeric_limits<double>::infinity();
  double target_distance = std::numeric_limits<double>::infinity();

  ROS_INFO_STREAM("Searching matching vertices on mesh");
  for (geodesic::Vertex& vertex : geodesic_mesh.vertices())
  {
    double current_source_distance = sqrt((start.pose.position.x - vertex.x()) * (start.pose.position.x - vertex.x()) +
                                          (start.pose.position.y - vertex.y()) * (start.pose.position.y - vertex.y()) +
                                          (start.pose.position.z - vertex.z()) * (start.pose.position.z - vertex.z()));
    double current_target_distance = sqrt((goal.pose.position.x - vertex.x()) * (goal.pose.position.x - vertex.x()) +
                                          (goal.pose.position.y - vertex.y()) * (goal.pose.position.y - vertex.y()) +
                                          (goal.pose.position.z - vertex.z()) * (goal.pose.position.z - vertex.z()));

    if (current_source_distance < source_distance)
    {
      source_distance = current_source_distance;
      source_vertex = &vertex;
    }

    if (current_target_distance < target_distance)
    {
      target_distance = current_target_distance;
      target_vertex = &vertex;
    }
  }

  // setup points
  geodesic::SurfacePoint source(source_vertex);
  geodesic::SurfacePoint target(target_vertex);

  std::vector<geodesic::SurfacePoint> sources(1, source);

  // run propagation
  geodesic::GeodesicAlgorithmExact algorithm(&geodesic_mesh);
  double const distance_limit = geodesic::GEODESIC_INF;        // no limit for propagation
  std::vector<geodesic::SurfacePoint> stop_points(1, target);  // stop propagation when the target is covered

  ROS_INFO_STREAM("Starting mmp propagation");
  algorithm.propagate(sources, distance_limit, &stop_points);
  ROS_INFO_STREAM("Finished mmp propagation");

  // trace back path
  std::vector<geodesic::SurfacePoint> path;
  algorithm.trace_back(target, path);

  ROS_INFO_STREAM("Path size: " << path.size());

  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = mesh_map->mapFrame();

  cost = 0;

  geometry_msgs::PoseStamped pose;
  pose.header = header;
  for (geodesic::SurfacePoint& point : path)
  {
    pose.pose.position.x = point.x();
    pose.pose.position.y = point.y();
    pose.pose.position.z = point.z();

    plan.push_back(pose);
  }

  ROS_INFO_STREAM("Path length: " << cost << "m");
  nav_msgs::Path path_msg;
  path_msg.poses = plan;
  path_msg.header = header;

  path_pub.publish(path_msg);

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

  size_t filtered_faces = 0;

  // create filter for non manifold vertices
  std::unordered_map<std::size_t, bool> filter_vertices;
  std::unordered_map<std::size_t, bool> filter_faces;
  for (auto const& vH : mesh.vertices())
  {
    filter_vertices[vH.idx()] = true;
  }

  for (auto const& fH : mesh.faces())
  {
    filter_faces[fH.idx()] = true;
  }

  std::unordered_map<std::size_t, std::vector<std::size_t>> known_edges;
  for (auto const& fH : mesh.faces())
  {
    array<lvr2::VertexHandle, 3> vertices = mesh.getVerticesOfFace(fH);
    if (!invalid[vertices[0]] && !invalid[vertices[1]] && !invalid[vertices[2]] && !isinf(vertex_costs[vertices[0]]) &&
        !isinf(vertex_costs[vertices[1]]) && !isinf(vertex_costs[vertices[2]]))

    {
      // check whether or not the edge has already been created to filter non 2 manifold edges
      if (known_edges.find(vertices[0].idx()) != known_edges.end() &&
          std::find(known_edges[vertices[0].idx()].begin(), known_edges[vertices[0].idx()].end(), vertices[1].idx()) !=
              known_edges[vertices[0].idx()].end())
      {
        filtered_faces++;
        continue;
      }
      if (known_edges.find(vertices[1].idx()) != known_edges.end() &&
          std::find(known_edges[vertices[1].idx()].begin(), known_edges[vertices[1].idx()].end(), vertices[2].idx()) !=
              known_edges[vertices[1].idx()].end())
      {
        filtered_faces++;
        continue;
      }
      if (known_edges.find(vertices[2].idx()) != known_edges.end() &&
          std::find(known_edges[vertices[2].idx()].begin(), known_edges[vertices[2].idx()].end(), vertices[0].idx()) !=
              known_edges[vertices[2].idx()].end())
      {
        filtered_faces++;
        continue;
      }

      // check vertex angles to remove degenerated faces
      mesh_map::Vector vertex0_position = mesh.getVertexPosition(vertices[0]);
      mesh_map::Vector vertex1_position = mesh.getVertexPosition(vertices[1]);
      mesh_map::Vector vertex2_position = mesh.getVertexPosition(vertices[2]);

      float angle0 = acos((vertex1_position - vertex0_position).dot(vertex2_position - vertex0_position) /
                          (vertex1_position.length() * vertex2_position.length()));
      float angle1 = acos((vertex0_position - vertex1_position).dot(vertex2_position - vertex1_position) /
                          (vertex0_position.length() * vertex2_position.length()));
      float angle2 = acos((vertex0_position - vertex2_position).dot(vertex1_position - vertex2_position) /
                          (vertex0_position.length() * vertex1_position.length()));

      if (isnan(angle0) || angle0 <= 0.01)
      {
        filtered_faces++;
        continue;
      }
      if (isnan(angle1) || angle1 <= 0.01)
      {
        filtered_faces++;
        continue;
      }
      if (isnan(angle2) || angle2 <= 0.01)
      {
        filtered_faces++;
        continue;
      }

      known_edges[vertices[0].idx()].push_back(vertices[1].idx());
      known_edges[vertices[1].idx()].push_back(vertices[2].idx());
      known_edges[vertices[2].idx()].push_back(vertices[0].idx());

      filter_vertices[vertices[0].idx()] = false;
      filter_vertices[vertices[1].idx()] = false;
      filter_vertices[vertices[2].idx()] = false;

      filter_faces[fH.idx()] = false;
    }
  }

  std::vector<double> points;
  std::vector<unsigned> faces;

  // remove non manifold vertices from list and build remapping map
  std::unordered_map<std::size_t, std::size_t> vertex_remapping;
  for (auto const& vH : mesh.vertices())
  {
    if (filter_vertices[vH.idx()])
    {
      continue;
    }

    vertex_remapping[vH.idx()] = points.size() / 3;

    mesh_map::Vector vertex = mesh.getVertexPosition(vH);
    points.push_back(vertex.x);
    points.push_back(vertex.y);
    points.push_back(vertex.z);
  }

  for (auto const& fH : mesh.faces())
  {
    array<lvr2::VertexHandle, 3> vertices = mesh.getVerticesOfFace(fH);

    if (filter_faces[fH.idx()])
    {
      continue;
    }

    faces.push_back(vertex_remapping[vertices[0].idx()]);
    faces.push_back(vertex_remapping[vertices[1].idx()]);
    faces.push_back(vertex_remapping[vertices[2].idx()]);
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

