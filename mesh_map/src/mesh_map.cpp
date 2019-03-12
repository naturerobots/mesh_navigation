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
#include <mesh_map/mesh_map.h>
#include <lvr2/io/HDF5IO.hpp>
#include <ros/ros.h>
#include <lvr2/algorithm/GeometryAlgorithms.hpp>
#include <lvr2/algorithm/NormalAlgorithms.hpp>
#include <mesh_msgs/MeshGeometryStamped.h>
#include <lvr_ros/conversions.h>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/random_generator.hpp>
#include <algorithm>

namespace mesh_map{

MeshMap::MeshMap(tf::TransformListener& tf_listener)
    : tf_listener_(tf_listener),
      private_nh_("~/mesh_map/"),
      first_config_(true),
      map_loaded_(false)
{
  private_nh_.param<std::string>("mesh_file", mesh_file_, "mesh.h5");
  private_nh_.param<std::string>("mesh_part", mesh_part_, "mesh");
  private_nh_.param<std::string>("global_frame", global_frame_, "map");
  ROS_INFO_STREAM("mesh file is set to: " << mesh_file_);

  mesh_geometry_pub_ = private_nh_.advertise<mesh_msgs::MeshGeometryStamped>("mesh", 1, true);
  vertex_costs_pub_ = private_nh_.advertise<mesh_msgs::MeshVertexCostsStamped>("vertex_costs", 1, false);
  path_pub_ = private_nh_.advertise<nav_msgs::Path>("path", true, false);

  reconfigure_server_ptr = boost::shared_ptr<dynamic_reconfigure::Server<mesh_map::MeshMapConfig> > (
      new dynamic_reconfigure::Server<mesh_map::MeshMapConfig>(private_nh_));

  config_callback = boost::bind(&MeshMap::reconfigureCallback,this, _1, _2);
  reconfigure_server_ptr->setCallback(config_callback);

}

bool MeshMap::readMap()
{
  return readMap(mesh_file_, mesh_part_);
}

bool MeshMap::readMap(const std::string& mesh_file, const std::string& mesh_part)
{
  ROS_INFO_STREAM("Start reading the mesh part '" << mesh_part << "' from the map file '"<< mesh_file << "'...");
  mesh_io_ptr = std::shared_ptr<lvr2::AttributeMeshIOBase>(new lvr2::HDF5IO(mesh_file, mesh_part, HighFive::File::ReadWrite));
  auto mesh_opt = mesh_io_ptr->getMesh();

  if(mesh_opt)
  {
    mesh = mesh_opt.get();
    ROS_INFO_STREAM("The mesh has been loaded successfully with " << mesh.numVertices() << " vertices and "
                                                                  << mesh.numFaces() << " faces and " << mesh.numEdges() << " edges." );
  }
  else
  {
    ROS_ERROR_STREAM("Could not load the mesh '" << mesh_part_ << "' from the map file '" << mesh_file_ << "' ");
    return false;
  }

  // TODO read and write uuid
  boost::uuids::random_generator gen;
  boost::uuids::uuid uuid = gen();
  uuid_str_ = boost::uuids::to_string(uuid);


  auto face_normals_opt =
      mesh_io_ptr->getDenseAttributeMap<lvr2::DenseFaceMap<NormalType>>("face_normals");

  if(face_normals_opt)
  {
    face_normals_ = face_normals_opt.get();
    ROS_INFO_STREAM("Found " << face_normals_.numValues() << " face normals in map file.");
  }
  else
  {
    ROS_INFO_STREAM("No face normals found in the given map file, computing them...");
    face_normals_ = lvr2::calcFaceNormals(mesh);
    ROS_INFO_STREAM("Computed "<< face_normals_.numValues() << " face normals.");
    if(mesh_io_ptr->addDenseAttributeMap(face_normals_, "face_normals"))
    {
      ROS_INFO_STREAM("Saved face normals to map file.");
    }
    else
    {
      ROS_ERROR_STREAM("Could not save face normals to map file!");
    }
  }

  auto vertex_normals_opt =
      mesh_io_ptr->getDenseAttributeMap<lvr2::DenseVertexMap<NormalType>>("vertex_normals");

  if(vertex_normals_opt)
  {
    vertex_normals_ = vertex_normals_opt.get();
    ROS_INFO_STREAM("Found "<< vertex_normals_.numValues() << " vertex normals in map file!");
  }
  else
  {
    ROS_INFO_STREAM("No vertex normals found in the given map file, computing them...");
    vertex_normals_ = lvr2::calcVertexNormals(mesh, face_normals_);
    if(mesh_io_ptr->addDenseAttributeMap(vertex_normals_, "vertex_normals"))
    {
      ROS_INFO_STREAM("Saved vertex normals to map file.");
    }
    else
    {
      ROS_ERROR_STREAM("Could not save vertex normals to map file!");
    }
  }

  mesh_geometry_pub_.publish(lvr_ros::toMeshGeometryStamped<float>(mesh, global_frame_, uuid_str_, vertex_normals_));

  ROS_INFO_STREAM("Try to read roughness from map file...");
  auto roughness_opt = mesh_io_ptr->getDenseAttributeMap<lvr2::DenseVertexMap<float>>("roughness");

  if(roughness_opt)
  {
    ROS_INFO_STREAM("Roughness has been read successfully.");
    roughness_ = roughness_opt.get();
  }
  else
  {
    ROS_INFO_STREAM("Computing roughness...");
    roughness_ = lvr2::calcVertexRoughness(mesh, config_.roughness_radius, vertex_normals_);
    ROS_INFO_STREAM("Saving roughness to map file...");
    if(mesh_io_ptr->addDenseAttributeMap(roughness_, "roughness"))
    {
      ROS_INFO_STREAM("Saved roughness to map file.");
    }
    else
    {
      ROS_ERROR_STREAM("Could not save roughness to map file!");
    }
  }

  ROS_INFO_STREAM("Try to read height differences from map file...");
  auto height_diff_opt = mesh_io_ptr->getDenseAttributeMap<lvr2::DenseVertexMap<float>>("height_diff");

  if(height_diff_opt)
  {
    ROS_INFO_STREAM("Height differences have been read successfully.");
    height_diff_ = height_diff_opt.get();
  }
  else
  {
    ROS_INFO_STREAM("Computing height differences...");
    height_diff_ = lvr2::calcVertexHeightDifferences(mesh, config_.height_diff_radius);
    ROS_INFO_STREAM("Saving height_differences to map file...");
    if(mesh_io_ptr->addDenseAttributeMap(height_diff_, "height_diff"))
    {
      ROS_INFO_STREAM("Saved height differences to map file.");
    }
    else
    {
      ROS_ERROR_STREAM("Could not save height differences to map file!");
    }
  }

  ROS_INFO_STREAM("Try to read edge distances from map file...");
  auto edge_distances_opt = mesh_io_ptr->getAttributeMap<lvr2::DenseEdgeMap<float>>("edge_distances");

  if(edge_distances_opt)
  {
    ROS_INFO_STREAM("Vertex distances have been read successfully.");
    edge_distances_ = edge_distances_opt.get();
  }
  else
  {
    ROS_INFO_STREAM("Computing edge distances...");
    edge_distances_ = lvr2::calcVertexDistances(mesh);
    ROS_INFO_STREAM("Saving " << edge_distances_.numValues() << " edge distances to map file...");

    if(mesh_io_ptr->addAttributeMap(edge_distances_, "edge_distances"))
    {
      ROS_INFO_STREAM("Saved edge distances to map file.");
    }
    else
    {
      ROS_ERROR_STREAM("Could not save edge distances to map file!");
    }
  }

  // riskiness
  ROS_INFO_STREAM("Try to read riskiness from map file...");
  auto riskiness_opt = mesh_io_ptr->getDenseAttributeMap<lvr2::DenseVertexMap<float>>("riskiness");

  if(riskiness_opt)
  {
    ROS_INFO_STREAM("Riskiness has been read successfully.");
    riskiness_ = riskiness_opt.get();
  }
  else
  {
    ROS_INFO_STREAM("Computing riskiness by finding lethal areas...");

    lethalCostInflation(
        config_.min_contour_size,
        config_.height_diff_threshold,
        config_.roughness_threshold,
        config_.inflation_radius,
        config_.inscribed_radius,
        config_.inscribed_value,
        config_.lethal_value);

    ROS_INFO_STREAM("Saving " << riskiness_.numValues() << " riskiness values to map file...");

    if(mesh_io_ptr->addDenseAttributeMap(riskiness_, "riskiness"))
    {
      ROS_INFO_STREAM("Saved riskiness to map file.");
    }
    else
    {
      ROS_ERROR_STREAM("Could not save riskiness to map file!");
    }
  }

  sleep(1);
  combineVertexCosts(config_.riskiness_factor, config_.roughness_factor, config_.height_diff_factor);
  publishCostLayers();

  map_loaded_ = true;
  return true;
}

inline MeshMap::VectorType MeshMap::toVectorType(const geometry_msgs::Point& p)
{
  return VectorType(p.x, p.y, p.z);
}

bool MeshMap::isLethal(const lvr2::VertexHandle& vH){
  return vertex_costs_[vH] >= 1;
}

bool MeshMap::dijkstra(
    const VectorType& start,
    const VectorType& goal,
    std::list<lvr2::VertexHandle>& path)
{
    lvr2::DenseVertexMap<bool> seen(mesh.nextVertexIndex(), false);

    lvr2::VertexHandle startH = getNearestVertexHandle(start).unwrap();
    lvr2::VertexHandle goalH = getNearestVertexHandle(goal).unwrap();

    return lvr2::Dijkstra<lvr2::BaseVector<float>>(
        mesh, startH, goalH, edge_weights_, path, potential_, predecessors_, seen, vertex_costs_);
}

bool MeshMap::waveFrontPropagation(
    const VectorType& start,
    const VectorType& goal,
    std::list<lvr2::VertexHandle>& path)
{
  return waveFrontPropagation(start, goal, edge_weights_, vertex_costs_, path, potential_, predecessors_);
}

void MeshMap::combineVertexCosts(
    const float& riskiness_factor,
    const float& roughness_factor,
    const float& height_diff_factor)
{

  ROS_INFO_STREAM("Combining costs...");

  float riskiness_min, riskiness_max, roughness_min, roughness_max, height_diff_min, height_diff_max;
  // Get normalized values
  getMinAndMaxValues(riskiness_min, riskiness_max, roughness_min, roughness_max, height_diff_min, height_diff_max);
  float riskiness_norm = riskiness_max - riskiness_min;
  float roughness_norm = roughness_max - roughness_min;
  float height_diff_norm = height_diff_max - height_diff_min;

  ROS_INFO_STREAM("Riskiness norm: " << riskiness_norm);
  ROS_INFO_STREAM("Roughness norm: " << roughness_norm);
  ROS_INFO_STREAM("Height differences norm: " << height_diff_norm);

  combineVertexCosts(
      riskiness_factor,
      riskiness_norm,
      roughness_factor,
      roughness_norm,
      height_diff_factor,
      height_diff_norm
  );
  ROS_INFO("Combined costs successfully!");
}


void MeshMap::getMinAndMaxValues(
    float& risk_min, float& risk_max,
    float& rough_min, float& rough_max,
    float& height_min, float& height_max)
{
  risk_max = std::numeric_limits<float>::min();
  risk_min = std::numeric_limits<float>::max();

  rough_max = std::numeric_limits<float>::min();
  rough_min = std::numeric_limits<float>::max();

  height_max = std::numeric_limits<float>::min();
  height_min = std::numeric_limits<float>::max();

  // Calculate minimum and maximum values
  for(auto vH: mesh.vertices())
  {
    // max values
    if(risk_max < riskiness_[vH] && std::isfinite(riskiness_[vH])) risk_max = riskiness_[vH];
    if(rough_max < roughness_[vH] && std::isfinite(roughness_[vH])) rough_max = roughness_[vH];
    if(height_max < height_diff_[vH] && std::isfinite(height_diff_[vH])) height_max = height_diff_[vH];
    // min values
    if(risk_min > riskiness_[vH] && std::isfinite(riskiness_[vH])) risk_min = riskiness_[vH];
    if(rough_min > roughness_[vH] && std::isfinite(roughness_[vH])) rough_min = roughness_[vH];
    if(height_min > height_diff_[vH] && std::isfinite(height_diff_[vH])) height_min = height_diff_[vH];
  }
}


void MeshMap::findLethalByContours(
    const int& min_contour_size,
    std::set<lvr2::VertexHandle>& lethals)
{
  int size = lethals.size();
  std::vector<std::vector<lvr2::VertexHandle> > contours;
  findContours(contours, min_contour_size);
  for(auto contour : contours)
  {
    lethals.insert(contour.begin(), contour.end());
  }
  ROS_INFO_STREAM("Found " << lethals.size() - size << " lethal vertices as contour vertices");
}

void MeshMap::findLethalAreas(
    const int min_contour_size,
    const float height_diff_threshold,
    const float roughness_threshold)
{
  ROS_INFO_STREAM("Find lethal vertices...");
  lethals_.clear();
  findLethalByContours(min_contour_size, lethals_);
  findLethalInLayer(height_diff_, height_diff_threshold, lethals_);
  findLethalInLayer(roughness_, roughness_threshold, lethals_);
}

void MeshMap::findLethalInLayer(
    const lvr2::DenseVertexMap<float>& layer,
    const float& threshold,
    std::set<lvr2::VertexHandle>& lethals)
{
  int size = lethals.size();
  for(auto vH: mesh.vertices())
  {
    // Store lethal areas by Roughness
    if(layer[vH] >= threshold)
    {
      lethals.insert(vH);
    }
  }
  ROS_INFO_STREAM("Found " << lethals.size() - size << " lethal vertices exceeding the threshold " << threshold << "!");
}

void MeshMap::findContours(
    std::vector<std::vector<lvr2::VertexHandle> >& contours,
    int min_contour_size) {

  ROS_INFO_STREAM("Find contours...");

  std::vector<std::vector<lvr2::VertexHandle> > tmp_contours;

  array<lvr2::OptionalFaceHandle, 2> facepair;
  lvr2::SparseEdgeMap<bool> usedEdges(false);
  for (auto eHStart: mesh.edges())
  {
    lvr2::SparseVertexMap<bool> usedVertices(false);
    lvr2::SparseEdgeMap<bool> local_usedEdges(false);
    int count = 0;

    // Look for border Edges
    facepair = mesh.getFacesOfEdge(eHStart);

    // If border Edge found
    if((!facepair[0]||!facepair[1])&&!usedEdges[eHStart])
    {
      std:vector<lvr2::VertexHandle> contour;
      // Set vector which links to the following Edge
      array<lvr2::VertexHandle, 2> vertexPair = mesh.getVerticesOfEdge(eHStart);
      lvr2::VertexHandle vH = vertexPair[1];
      vector<lvr2::EdgeHandle> curEdges;
      lvr2::EdgeHandle eHTemp = eHStart;
      bool moving = true;
      bool vertex_flag = false;

      // While the conotur did not come full circle
      while(moving)
      {
        moving = false;
        usedEdges.insert(eHTemp, true);
        local_usedEdges.insert(eHTemp, true);
        // Set vector which links to the following Edge
        vertexPair = mesh.getVerticesOfEdge(eHTemp);
        // Eliminate the possibility to choose the previous Vertex
        if(vH != vertexPair[0])
        {
          vH = vertexPair[0];
        }
        else if(vH != vertexPair[1])
        {
          vH = vertexPair[1];
        }

        // Add the current Vertex to the contour
        usedVertices.insert(vH, true);
        count++;
        contour.push_back(vH);
        mesh.getEdgesOfVertex(vH, curEdges);

        // Look for other edge of vertex that is a border Edge
        for(auto eHT: curEdges)
        {
          if(!usedEdges[eHT] && !local_usedEdges[eHT])
          {
            facepair = mesh.getFacesOfEdge(eHT);
            if(!facepair[0]||!facepair[1])
            {
              eHTemp = eHT;
              moving = true;
              continue;
            }
          }
        }
      }
      // Add contour to list of contours
      if(contour.size() > min_contour_size)
      {
        contours.push_back(contour);
      }
    }
  }

  ROS_INFO_STREAM("Found " << contours.size() << " contours.");
}

void MeshMap::combineVertexCosts(
    float riskiness_factor,
    float riskiness_norm,
    float roughness_factor,
    float roughness_norm,
    float height_diff_factor,
    float height_diff_norm)
{
  if(riskiness_norm != 0) riskiness_factor /= riskiness_norm;
  if(roughness_norm != 0) roughness_factor /= roughness_norm;
  if(height_diff_norm != 0) height_diff_factor /= height_diff_norm;

  float risk_max = std::numeric_limits<float>::min();
  float risk_min = std::numeric_limits<float>::max();

  float rough_max = std::numeric_limits<float>::min();
  float rough_min = std::numeric_limits<float>::max();

  float height_max = std::numeric_limits<float>::min();
  float height_min = std::numeric_limits<float>::max();

  // Calculate minimum and maximum values
  for(auto vH: mesh.vertices())
  {
    if(risk_max < riskiness_[vH] && std::isfinite(riskiness_[vH])) risk_max = riskiness_[vH];
    if(rough_max < roughness_[vH] && std::isfinite(roughness_[vH])) rough_max = roughness_[vH];
    if(height_max < height_diff_[vH] && std::isfinite(height_diff_[vH])) height_max = height_diff_[vH];
    if(risk_min > riskiness_[vH] && std::isfinite(riskiness_[vH])) risk_min = riskiness_[vH];
    if(rough_min > roughness_[vH] && std::isfinite(roughness_[vH])) rough_min = roughness_[vH];
    if(height_min > height_diff_[vH] && std::isfinite(height_diff_[vH])) height_min = height_diff_[vH];
  }

  ROS_INFO_STREAM("Roughness min: " << rough_min << " roughness max: " << rough_max);
  ROS_INFO_STREAM("Height Differences min: " << height_min << " height differences max: " << height_max);
  ROS_INFO_STREAM("Riskiness min: " << risk_min << " riskiness max: " << risk_max);

  for(auto vH: mesh.vertices())
  {
    vertex_costs_.insert(vH, riskiness_factor * riskiness_[vH] +
        roughness_factor * roughness_[vH] +
        height_diff_factor * height_diff_[vH]);
  }

  calculateEdgeWeights(roughness_factor, height_diff_factor);
}
void MeshMap::lethalCostInflation(
    const int min_contour_size,
    const float height_diff_threshold,
    const float roughness_threshold,
    const float inflation_radius,
    const float inscribed_radius,
    const float inscribed_value,
    const float lethal_value)
{
  findLethalAreas(min_contour_size, height_diff_threshold, roughness_threshold);
  lethalCostInflation(lethals_, inflation_radius, inscribed_radius, inscribed_value, lethal_value);

}

void MeshMap::lethalCostInflation(
    const std::set<lvr2::VertexHandle>& lethals,
    float inflation_radius,
    float inscribed_radius,
    float inscribed_value,
    float lethal_value)
{

  ROS_INFO_STREAM("lethal cost inflation.");

  struct Cmp{
    const lvr2::DenseVertexMap<float>& distances;
    Cmp(const lvr2::DenseVertexMap<float>& distances) : distances(distances) {}

    bool operator() (lvr2::VertexHandle& left, lvr2::VertexHandle& right)
    {
      return distances[left] > distances[right];
    }
  };

  lvr2::DenseVertexMap<bool> seen(mesh.numVertices(), false);
  lvr2::DenseVertexMap<lvr2::VertexHandle> prev;
  prev.reserve(mesh.numVertices());

  for(auto vH: mesh.vertices())
  {
    riskiness_.insert(vH, 0);
    vertex_distances_.insert(vH, std::numeric_limits<float>::max());
    prev.insert(vH, vH);
  }

  Cmp cmp(vertex_distances_);
  std::priority_queue<lvr2::VertexHandle, std::vector<lvr2::VertexHandle>, Cmp> pq(cmp);

  for(auto vH: lethals)
  {
    vertex_distances_[vH] = 0;
    pq.push(vH);
  }

  float inflation_radius_squared = inflation_radius * inflation_radius;
  float inscribed_radius_squared = inscribed_radius * inscribed_radius;

  while (!pq.empty())
  {
    lvr2::VertexHandle vH = pq.top();
    pq.pop();

    if(seen[vH]) continue;
    seen[vH] = true;

    std::vector<lvr2::VertexHandle> neighbours;
    mesh.getNeighboursOfVertex(vH, neighbours);
    for(auto n : neighbours)
    {
      if(vertex_distances_[n] == 0 || seen[n]) continue;
      float n_dist = mesh.getVertexPosition(n).squaredDistanceFrom(mesh.getVertexPosition(prev[vH]));
      if(n_dist < vertex_distances_[n] && n_dist < inflation_radius_squared)
      {
        prev[n] = prev[vH];
        vertex_distances_[n] = n_dist;
        pq.push(n);
      }
    }
  }

  for(auto vH: mesh.vertices())
  {
    if(vertex_distances_[vH] > inflation_radius_squared)
    {
      riskiness_.insert(vH, 0);
    }

      // Inflation radius
    else if(vertex_distances_[vH] > inscribed_radius_squared)
    {
      float alpha = (sqrt(vertex_distances_[vH]) - inscribed_radius) /
          (inflation_radius - inscribed_radius) * M_PI;
      riskiness_.insert(vH, inscribed_value * (cos(alpha) + 1) / 2.0);
    }

      // Inscribed radius
    else if(vertex_distances_[vH] > 0)
    {
      riskiness_.insert(vH, inscribed_value);
    }

      // Lethality
    else
    {
      riskiness_.insert(vH, lethal_value);
    }
  }

  ROS_INFO_STREAM("lethal cost inflation finished.");
}

void MeshMap::calculateEdgeWeights(
    float roughness_factor,
    float height_diff_factor)
{
  ROS_INFO_STREAM("Compute edge weights...");

  // For all found Edges
  for(auto eH: mesh.edges())
  {
    // Get both Vertices of the current Edge
    array<lvr2::VertexHandle, 2> eH_vHs = mesh.getVerticesOfEdge(eH);
    lvr2::VertexHandle vH1 = eH_vHs[0];
    lvr2::VertexHandle vH2 = eH_vHs[1];

    // Get the Riskiness for the current Edge (the maximum value from both Vertices)
    double edge_risk = std::max(riskiness_[vH1], riskiness_[vH2]);

    // Get the Roughness for the current Edge, using the given Roughness-Factor
    double edge_roughness_factor = std::max(roughness_[vH1], roughness_[vH2]) * roughness_factor + 1;

    // Get the Height Difference for Edge, using the given Height Difference-Factor
    double edge_height_diff_factor = std::max(
        height_diff_[vH1], height_diff_[vH2])
        * height_diff_factor + 1;

    // Save calculated edge weight
    edge_weights_.insert(eH, edge_distances_[eH] * edge_roughness_factor
        * edge_height_diff_factor + edge_risk);
  }
  ROS_INFO_STREAM("Compute edge weights done.");
}



bool MeshMap::calculatePose(
    const VectorType& current,
    const VectorType& next,
    const NormalType& normal,
    geometry_msgs::Pose& pose)
{
  // Check the pose integrity
  VectorType direction(next - current);
  if(!isfinite(direction.x)) return false;
  if(!isfinite(direction.y)) return false;
  if(!isfinite(direction.z)) return false;

  if(direction.x == 0 && direction.y == 0 && direction.z == 0)
  {
    return false;
  }

  NormalType ez = normal;
  NormalType ey = normal.cross(direction).normalized();
  NormalType ex = ey.cross(normal).normalized();

  tf::Matrix3x3 tf_basis(
      ex.x, ey.x, ez.x,
      ex.y, ey.y, ez.y,
      ex.z, ey.z, ez.z);

  tf::Vector3 tf_origin(
      current.x,
      current.y,
      current.z);

  tf::Pose tf_pose;
  tf_pose.setBasis(tf_basis);
  tf_pose.setOrigin(tf_origin);
  tf::poseTFToMsg(tf_pose, pose);

  return true;
}

inline bool MeshMap::waveFrontUpdate(
    lvr2::DenseVertexMap<float>& distances,
    const lvr2::DenseEdgeMap<float>& edge_weights,
    const lvr2::VertexHandle& v1,
    const lvr2::VertexHandle& v2,
    const lvr2::VertexHandle& v3)
{

  const double T1 = distances[v1];
  const double T2 = distances[v2];
  const double T3 = distances[v3];

  const lvr2::OptionalEdgeHandle e12h = mesh.getEdgeBetween(v1, v2);
  const float c = edge_weights[e12h.unwrap()];

  const lvr2::OptionalEdgeHandle e13h = mesh.getEdgeBetween(v1, v3);
  const float b = edge_weights[e13h.unwrap()];
  const float b_sq = b*b;

  const lvr2::OptionalEdgeHandle e23h = mesh.getEdgeBetween(v2, v3);
  const float a = edge_weights[e23h.unwrap()];
  const float a_sq = a * a;

  if(a < 0.01 || b < 0.01 || c < 0.01){
    double T3tmp = T3;
    if (a < 0.005) T3tmp = T2 + a;
    if (c < 0.005 || b < 0.005) T3tmp = T1 + b;

    if (T3tmp < T3)
    {
        distances[v3] = static_cast<float>(T3tmp);
        return true;
    }
    return false;
  }

  const double T1sq = T1*T1;
  const double T2sq = T2*T2;

  double A = std::max<double>((-T1+T2+c)*(T1-T2+c)*(T1+T2-c)*(T1+T2+c), 0);
  double B = std::max<double>((-a+b+c)*(a-b+c)*(a+b-c)*(a+b+c), 0);

  const double dy = (sqrt(A) + sqrt(B)) / (2*c);
  const double dx = (T2sq - T1sq + b_sq - a_sq) / (2*c);

  const double T3tmp = sqrt(dx*dx + dy*dy);

  if(T3tmp < T3){
    distances[v3] = static_cast<float>(T3tmp);
    return true;
  }

  return false;
}

inline bool MeshMap::waveFrontPropagation(
    const VectorType& start,
    const VectorType& goal,
    const lvr2::DenseEdgeMap<float>& edge_weights,
    const lvr2::DenseVertexMap<float>& costs,
    std::list<lvr2::VertexHandle>& path,
    lvr2::DenseVertexMap<float>& distances,
    lvr2::DenseVertexMap<lvr2::VertexHandle>& predecessors)
{
  // Find nearest Vertex to start and end
  const auto& start_opt = getNearestVertexHandle(start);
  const auto& goal_opt = getNearestVertexHandle(goal);

  if(!start_opt || !goal_opt)
  {
    return false;
  }

  const auto& start_vertex = start_opt.unwrap();
  const auto& goal_vertex = goal_opt.unwrap();

  path.clear();
  distances.clear();
  predecessors.clear();

  if(goal_vertex == start_vertex)
  {
    return true;
  }

  VectorType start_point = mesh.getVertexPosition(start_vertex);

  lvr2::DenseVertexMap<bool> fixed(mesh.numVertices(), false);

  // initialize distances with infinity
  // initialize predecessor of each vertex with itself
  for(auto const &vH : mesh.vertices())
  {
    distances.insert(vH, std::numeric_limits<float>::infinity());
    predecessors.insert(vH, vH);
  }

  struct Cmp{
    const lvr2::DenseVertexMap<float>& distances;
    Cmp(const lvr2::DenseVertexMap<float>& distances) : distances(distances) {}
    bool operator() (lvr2::VertexHandle& left, lvr2::VertexHandle& right)
    {
      return distances[left] > distances[right];
    }
  };

  Cmp cmp(distances);

  std::priority_queue<lvr2::VertexHandle, std::vector<lvr2::VertexHandle>, Cmp> pq(cmp);

  // Set start distance to zero
  // add start vertex to priority queue
  distances[start_vertex] = 0;
  fixed[start_vertex] = true;

  ROS_INFO_STREAM("Init wave front propagation.");

  // initialize around seed / start vertex
  std::vector<lvr2::VertexHandle> neighbours;
  mesh.getNeighboursOfVertex(start_vertex, neighbours);

  ROS_INFO_STREAM("Number of neighbours " << neighbours.size());

  for(auto n_vh : neighbours)
  {
    lvr2::OptionalEdgeHandle edge = mesh.getEdgeBetween(start_vertex, n_vh);
    distances[n_vh] = edge_weights[edge.unwrap()];
    predecessors[n_vh] = start_vertex;
    pq.push(n_vh);
  }
  ROS_INFO_STREAM("Start wave front propagation");

  while(!pq.empty())
  {
    lvr2::VertexHandle current_vh = pq.top();
    pq.pop();

    // check if already fixed
    if(fixed[current_vh]) continue;
    fixed[current_vh] = true;

    std::vector<lvr2::FaceHandle> faces;
    mesh.getFacesOfVertex(current_vh, faces);

    for(auto fh : faces)
    {
      const auto vertices = mesh.getVerticesOfFace(fh);
      const lvr2::VertexHandle& a = vertices[0];
      const lvr2::VertexHandle& b = vertices[1];
      const lvr2::VertexHandle& c = vertices[2];

      if(fixed[a] && fixed[b] && fixed[c]){
        continue;
      }else if(fixed[a] && fixed[b] && !fixed[c]){
        // c is free
        if(costs[c] <= 1 && waveFrontUpdate(distances, edge_weights, a, b, c)){
          predecessors[c] = (distances[a] < distances[b]) ? a : b;
          pq.push(c);
        }
      }else if(fixed[a] && !fixed[b] && fixed[c]){
        // b is free
        if(costs[b] <= 1 && waveFrontUpdate(distances, edge_weights, c, a, b)){
          predecessors[b] = (distances[c] < distances[a]) ? c : a;
          pq.push(b);
        }
      }else if(!fixed[a] && fixed[b] && fixed[c]){
        // a if free
        if(costs[a] <= 1 && waveFrontUpdate(distances, edge_weights, b, c, a)){
          predecessors[a] = (distances[b] < distances[c]) ? b : c;
          pq.push(a);
        }
      }else{
        // two free vertices -> skip that face
        ROS_DEBUG_STREAM("two vertices are free.");
        continue;
      }
    }
  }

  ROS_INFO_STREAM("Finished wave front propagation.");

  lvr2::VertexHandle prev = predecessors[goal_vertex];

  if(prev == goal_vertex)
  {
    return false;
  }

  while(prev != start_vertex)
  {
    path.push_front(prev);
    if(predecessors[prev] == prev){
      ROS_INFO_STREAM("No path found!");
      return false;
    }
    prev = predecessors[prev];
  }
  path.push_front(start_vertex);

  return true;

}

lvr2::OptionalVertexHandle MeshMap::getNearestVertexHandle(const VectorType pos)
{
  double smallest_dist = std::numeric_limits<double>::max();
  lvr2::OptionalVertexHandle nearest_handle;

  // For all Vertices of the BaseMesh
  for(auto vH: mesh.vertices())
  {
    float dist = pos.distanceFrom(mesh.getVertexPosition(vH));
    if(dist < smallest_dist)
    {
      smallest_dist = dist;
      nearest_handle = vH;
    }
  }
  return nearest_handle;
}

bool MeshMap::pathPlanning(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped> &plan,
    bool fmm)
{
  std::list<lvr2::VertexHandle> path;

  ros::WallTime t_start, t_end;
  t_start = ros::WallTime::now();
  if(fmm){
    ROS_INFO("start wave front propagation.");
    if(!waveFrontPropagation(toVectorType(goal.pose.position), toVectorType(start.pose.position), path)){
      return false;
    }
  }
  else
  {
    ROS_INFO("start dijkstra.");
    if(!dijkstra(toVectorType(goal.pose.position), toVectorType(start.pose.position), path)){
      return false;
    }
  }
  t_end = ros::WallTime::now();
  double execution_time = (t_end - t_start).toNSec() * 1e-6;
  ROS_INFO_STREAM("Exectution time (ms): " << execution_time << " for " << mesh.numVertices() << " num vertices in the mesh.");

  path.reverse();

  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = global_frame_;

  lvr2::VertexHandle prev = path.front();

  for(auto &vH : path)
  {
    geometry_msgs::PoseStamped pose;
    pose.header = header;
    if(calculatePose(
        mesh.getVertexPosition(prev),
        mesh.getVertexPosition(vH),
        vertex_normals_[prev],
        pose.pose
    )){
      plan.push_back(pose);
      prev = vH;
    }
  }

  nav_msgs::Path path_msg;
  path_msg.poses = plan;
  path_msg.header = header;

  path_pub_.publish(path_msg);
  vertex_costs_pub_.publish(lvr_ros::toVertexCostsStamped(potential_, "Potential", global_frame_, uuid_str_));

  return true;
}


bool MeshMap::resetLayers()
{
  return true; //TODO implement
}

void MeshMap::publishCostLayers()
{
  vertex_costs_pub_.publish(lvr_ros::toVertexCostsStamped(riskiness_, "Riskiness", global_frame_, uuid_str_));
  vertex_costs_pub_.publish(lvr_ros::toVertexCostsStamped(roughness_, "Roughness", global_frame_, uuid_str_));
  vertex_costs_pub_.publish(lvr_ros::toVertexCostsStamped(height_diff_, "Height Diff", global_frame_, uuid_str_));
  vertex_costs_pub_.publish(lvr_ros::toVertexCostsStamped(vertex_costs_, "Combined Costs", global_frame_, uuid_str_));
}

void MeshMap::reconfigureCallback(mesh_map::MeshMapConfig& config, uint32_t level)
{
  ROS_INFO_STREAM("Dynamic reconfigure callback...");
  if(first_config_)
  {
    config_ = config;
    first_config_ = false;
  }

  ROS_INFO_STREAM("Reconfigure request: " );
  ROS_INFO_STREAM("inscribed radius: " << config.inscribed_radius);
  ROS_INFO_STREAM("inflation radius: " << config.inflation_radius);
  ROS_INFO_STREAM("roughness threshold: " << config.roughness_threshold);
  ROS_INFO_STREAM("height diff threshold: " << config.height_diff_threshold);
  ROS_INFO_STREAM("roughness_radius: " << config.roughness_radius);
  ROS_INFO_STREAM("height_diff_radius: " << config.height_diff_radius);
  ROS_INFO_STREAM("riskiness factor: " << config.riskiness_factor);
  ROS_INFO_STREAM("roughness factor: " << config.roughness_factor);
  ROS_INFO_STREAM("height diff factor: " << config.height_diff_factor);
  ROS_INFO_STREAM("min contour size: " << config.min_contour_size);
  ROS_INFO_STREAM("lethal value: " << config.lethal_value);
  ROS_INFO_STREAM("inscribed value: " << config.inscribed_value);

  if(!first_config_ && map_loaded_ )
  {
    // Check the dynamic local radius parameter
    if( config_.roughness_radius != config.roughness_radius ||
        config_.height_diff_radius != config.height_diff_radius ||
        config_.roughness_threshold != config.roughness_threshold ||
        config_.height_diff_threshold != config.height_diff_threshold ||
        config_.min_contour_size != config.min_contour_size ||
        config_.inscribed_radius != config.inscribed_radius ||
        config_.inflation_radius != config.inflation_radius ||
        config_.inscribed_value != config.inscribed_value ||
        config_.lethal_value != config.lethal_value)
    {
      if(config_.roughness_radius != config.roughness_radius)
      {
        roughness_ = lvr2::calcVertexRoughness(mesh, config.roughness_radius, vertex_normals_);
      }
      else if(config_.height_diff_radius != config.height_diff_radius)
      {
        height_diff_ = lvr2::calcVertexRoughness(mesh, config.height_diff_radius, vertex_normals_);
      }

      lethalCostInflation(
          config.min_contour_size,
          config.height_diff_threshold,
          config.roughness_threshold,
          config.inflation_radius,
          config.inscribed_radius,
          config.inscribed_value,
          config.lethal_value);

      combineVertexCosts(config.riskiness_factor, config.roughness_factor, config.height_diff_factor);
      publishCostLayers();
    }

    config_ = config;
  }
  // Apply the current configuration
}



const std::string MeshMap::getGlobalFrameID()
{
  return global_frame_;
}

} /* namespace mesh_map */





