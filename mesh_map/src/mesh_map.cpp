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
#include <XmlRpcException.h>
#include <lvr2/util/Meap.hpp>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Vector3.h>
#include <lvr_ros/colors.h>

namespace mesh_map{

MeshMap::MeshMap(tf::TransformListener& tf_listener)
    : tf_listener(tf_listener),
      private_nh("~/mesh_map/"),
      first_config(true),
      map_loaded(false),
      layer_loader("mesh_map", "mesh_map::AbstractLayer"),
      mesh_ptr(new lvr2::HalfEdgeMesh<VectorType>())
{
  private_nh.param<std::string>("mesh_file", mesh_file, "mesh.h5");
  private_nh.param<std::string>("mesh_part", mesh_part, "mesh");
  private_nh.param<std::string>("global_frame", global_frame, "map");
  ROS_INFO_STREAM("mesh file is set to: " << mesh_file);

  mesh_geometry_pub = private_nh.advertise<mesh_msgs::MeshGeometryStamped>("mesh", 1, true);
  vertex_costs_pub = private_nh.advertise<mesh_msgs::MeshVertexCostsStamped>("vertex_costs", 1, false);
  path_pub = private_nh.advertise<nav_msgs::Path>("path", 1, false);
  vector_pub = private_nh.advertise<visualization_msgs::MarkerArray>("vector_field", 1, true);
  reconfigure_server_ptr = boost::shared_ptr<dynamic_reconfigure::Server<mesh_map::MeshMapConfig> > (
      new dynamic_reconfigure::Server<mesh_map::MeshMapConfig>(private_nh));

  config_callback = boost::bind(&MeshMap::reconfigureCallback,this, _1, _2);
  reconfigure_server_ptr->setCallback(config_callback);

}

bool MeshMap::readMap()
{
  return readMap(mesh_file, mesh_part);
}

bool MeshMap::readMap(const std::string& mesh_file, const std::string& mesh_part)
{
  ROS_INFO_STREAM("Start reading the mesh part '" << mesh_part << "' from the map file '"<< mesh_file << "'...");
  mesh_io_ptr = std::shared_ptr<lvr2::AttributeMeshIOBase>(new lvr2::HDF5IO(mesh_file, mesh_part, HighFive::File::ReadWrite));
  auto mesh_opt = mesh_io_ptr->getMesh();

  if(mesh_opt)
  {
    *mesh_ptr = mesh_opt.get();
    ROS_INFO_STREAM("The mesh has been loaded successfully with " << mesh_ptr->numVertices() << " vertices and "
                                                                  << mesh_ptr->numFaces() << " faces and " << mesh_ptr->numEdges() << " edges." );
  }
  else
  {
    ROS_ERROR_STREAM("Could not load the mesh '" << mesh_part << "' from the map file '" << mesh_file << "' ");
    return false;
  }

  vertex_costs = lvr2::DenseVertexMap<float>(mesh_ptr->nextVertexIndex(), 0);
  edge_weights = lvr2::DenseEdgeMap<float>(mesh_ptr->nextEdgeIndex(), 0);
  direction = lvr2::DenseVertexMap<float>(mesh_ptr->nextVertexIndex(), 0);

  // TODO read and write uuid
  boost::uuids::random_generator gen;
  boost::uuids::uuid uuid = gen();
  uuid_str = boost::uuids::to_string(uuid);

  auto face_normals_opt =
      mesh_io_ptr->getDenseAttributeMap<lvr2::DenseFaceMap<NormalType>>("face_normals");

  if(face_normals_opt)
  {
    face_normals = face_normals_opt.get();
    ROS_INFO_STREAM("Found " << face_normals.numValues() << " face normals in map file.");
  }
  else
  {
    ROS_INFO_STREAM("No face normals found in the given map file, computing them...");
    face_normals = lvr2::calcFaceNormals(*mesh_ptr);
    ROS_INFO_STREAM("Computed "<< face_normals.numValues() << " face normals.");
    if(mesh_io_ptr->addDenseAttributeMap(face_normals, "face_normals"))
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
    vertex_normals = vertex_normals_opt.get();
    ROS_INFO_STREAM("Found "<< vertex_normals.numValues() << " vertex normals in map file!");
  }
  else
  {
    ROS_INFO_STREAM("No vertex normals found in the given map file, computing them...");
    vertex_normals = lvr2::calcVertexNormals(*mesh_ptr, face_normals);
    if(mesh_io_ptr->addDenseAttributeMap(vertex_normals, "vertex_normals"))
    {
      ROS_INFO_STREAM("Saved vertex normals to map file.");
    }
    else
    {
      ROS_ERROR_STREAM("Could not save vertex normals to map file!");
    }
  }

  mesh_geometry_pub.publish(lvr_ros::toMeshGeometryStamped<float>(*mesh_ptr, global_frame, uuid_str, vertex_normals));

  ROS_INFO_STREAM("Try to read edge distances from map file...");
  auto edge_distances_opt = mesh_io_ptr->getAttributeMap<lvr2::DenseEdgeMap<float>>("edge_distances");

  if(edge_distances_opt)
  {
    ROS_INFO_STREAM("Vertex distances have been read successfully.");
    edge_distances = edge_distances_opt.get();
  }
  else
  {
    ROS_INFO_STREAM("Computing edge distances...");
    edge_distances = lvr2::calcVertexDistances(*mesh_ptr);
    ROS_INFO_STREAM("Saving " << edge_distances.numValues() << " edge distances to map file...");

    if(mesh_io_ptr->addAttributeMap(edge_distances, "edge_distances"))
    {
      ROS_INFO_STREAM("Saved edge distances to map file.");
    }
    else
    {
      ROS_ERROR_STREAM("Could not save edge distances to map file!");
    }
  }

  if(!loadLayerPlugins())
  {
    ROS_FATAL_STREAM("Could not load any layer plugin!");
    return false;
  }
  if(!initLayerPlugins())
  {
    ROS_FATAL_STREAM("Could not initialize plugins!");
    return false;
  }

  sleep(1);

  combineVertexCosts();
  publishCostLayers();

  map_loaded = true;
  return true;
}

bool MeshMap::loadLayerPlugins()
{
  XmlRpc::XmlRpcValue plugin_param_list;
  if(!private_nh.getParam("layers", plugin_param_list))
  {
    ROS_WARN_STREAM("No layer plugins configured! - Use the param \"layers\" in the namespace \""
                        << private_nh.getNamespace() << "\". \"layers\" must be must be a list of "
                                                        "tuples with a name and a type.");
    return false;
  }

  try
  {
    for (int i = 0; i < plugin_param_list.size(); i++)
    {
      XmlRpc::XmlRpcValue elem = plugin_param_list[i];

      std::string name = elem["name"];
      std::string type = elem["type"];

      typename AbstractLayer::Ptr plugin_ptr;

      if (layers.find(name) != layers.end())
      {
        ROS_ERROR_STREAM("The plugin \"" << name << "\" has already been loaded! Names must be unique!");
        return false;
      }

      try
      {
        plugin_ptr = layer_loader.createInstance(type);
      }
      catch(pluginlib::LibraryLoadException &e)
      {
        ROS_ERROR_STREAM(e.what());
      }

      if(plugin_ptr)
      {
        if(plugin_ptr->initialize(mesh_ptr, mesh_io_ptr)){
          layers.insert(std::pair<std::string, typename mesh_map::AbstractLayer::Ptr>(name, plugin_ptr));
          layer_names.push_back(name);
          ROS_INFO_STREAM("The layer plugin with the type \"" << type
            << "\" has been loaded and initialized successfully under the name \"" << name << "\".");
        }
        else
        {
          ROS_ERROR_STREAM("Could not initialize the layer plugin with the name \""
            << name << "\" and the type \"" << type << "\"!");
        }
      }
      else
      {
        ROS_ERROR_STREAM("Could not load the layer plugin with the name \""
          << name << "\" and the type \"" << type << "\"!");
      }
    }
  }
  catch (XmlRpc::XmlRpcException &e)
  {
    ROS_ERROR_STREAM("Invalid parameter structure. The \"layers\" parameter has to be a list of structs "
                         << "with fields \"name\" and \"type\"!");
    ROS_ERROR_STREAM(e.getMessage());
    return false;
  }
  // is there any layer plugin loaded for the map?
  return !layers.empty();
}

bool MeshMap::initLayerPlugins()
{
  lethals.clear();
  for(auto& layer_name : layer_names)
  {
    auto& layer_plugin = layers[layer_name];
    layer_plugin->setLethals(lethals);
    if(!layer_plugin->readLayer())
    {
      layer_plugin->computeLayer(config);
    }

    // compute lethal vertices
    const auto& threshold = layer_plugin->threshold();
    auto& mesh = *mesh_ptr;
    auto& costs = layer_plugin->costs();
    auto& layer_lethals = lethal_indices[layer_name];
    layer_lethals.clear();
    for(auto vH : mesh.vertices())
    {
      if(costs[vH] > threshold)
      {
        layer_lethals.insert(vH);
        lethals.insert(vH);
      }
    }
  }
}

inline MeshMap::VectorType MeshMap::toVectorType(const geometry_msgs::Point& p)
{
  return VectorType(p.x, p.y, p.z);
}

inline bool MeshMap::isLethal(const lvr2::VertexHandle& vH){
  return vertex_costs[vH] >= 1;
}

bool MeshMap::dijkstra(
    const VectorType& start,
    const VectorType& goal,
    std::list<lvr2::VertexHandle>& path)
{
  lvr2::DenseVertexMap<bool> seen(mesh_ptr->nextVertexIndex(), false);

  lvr2::VertexHandle startH = getNearestVertexHandle(start).unwrap();
  lvr2::VertexHandle goalH = getNearestVertexHandle(goal).unwrap();

  return lvr2::Dijkstra<lvr2::BaseVector<float>>(
      *mesh_ptr, startH, goalH, edge_weights, path, potential, predecessors, seen, vertex_costs);
}

bool MeshMap::waveFrontPropagation(
    const VectorType& start,
    const VectorType& goal,
    std::list<lvr2::VertexHandle>& path)
{
  return waveFrontPropagation(start, goal, edge_weights, vertex_costs, path, potential, predecessors);
}

void MeshMap::combineVertexCosts()
{
  ROS_INFO_STREAM("Combining costs...");

  float combined_min = std::numeric_limits<float>::max();
  float combined_max = std::numeric_limits<float>::min();

  vertex_costs = lvr2::DenseVertexMap<float>(mesh_ptr->nextVertexIndex(), 0);

  for(auto layer : layers)
  {
    const auto& costs = layer.second->costs();
    float min, max;
    getMinMax(costs, min, max);
    const float norm = max - min;
    const float factor = private_nh.param<float>(layer.first + "/factor", 1.0);
    const float norm_factor = factor / norm;
    ROS_INFO_STREAM("Layer \"" << layer.first
                               << "\" max value: " << max << " min value: " << min
                               << " norm: " << norm << " factor: " << factor << " norm factor: " << norm_factor);


    const float default_value = layer.second->defaultValue();
    bool hasNaN = false;
    for(auto vH: mesh_ptr->vertices())
    {
      const float cost = costs.containsKey(vH) ? costs[vH] : default_value;
      if(std::isnan(cost)) hasNaN = true;
      vertex_costs[vH] += factor * cost;
      if(std::isfinite(cost))
      {
        combined_max = std::max(combined_max, vertex_costs[vH]);
        combined_min = std::max(combined_min, vertex_costs[vH]);
      }
    }
    if(hasNaN) ROS_WARN_STREAM("Layer \"" << layer.first << "\" contains NaN values!");
  }

  const float combined_norm = combined_max - combined_min;

  for(auto eH: mesh_ptr->edges())
  {
    // Get both Vertices of the current Edge
    std::array<lvr2::VertexHandle, 2> eH_vHs = mesh_ptr->getVerticesOfEdge(eH);
    lvr2::VertexHandle vH1 = eH_vHs[0];
    lvr2::VertexHandle vH2 = eH_vHs[1];
    // Get the Riskiness for the current Edge (the maximum value from both Vertices)
    float cost_diff = std::abs(vertex_costs[vH1] - vertex_costs[vH2]);
    edge_weights[eH] = edge_distances[eH]; //+ (edge_distances[eH] * config.path_layer_factor * cost_diff);
  }

  ROS_INFO("Successfully combined costs!");
}


void MeshMap::getMinMax(const lvr2::VertexMap<float>& costs, float& min, float& max)
{
  max = std::numeric_limits<float>::min();
  min = std::numeric_limits<float>::max();

  // Calculate minimum and maximum values
  for(auto vH: mesh_ptr->vertices())
  {
    if(max < costs[vH] && std::isfinite(costs[vH])) max = costs[vH];
    if(min > costs[vH] && std::isfinite(costs[vH])) min = costs[vH];
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
  lethals.clear();
  findLethalByContours(min_contour_size, lethals);
}

void MeshMap::findContours(
    std::vector<std::vector<lvr2::VertexHandle> >& contours,
    int min_contour_size) {

  ROS_INFO_STREAM("Find contours...");

  std::vector<std::vector<lvr2::VertexHandle> > tmp_contours;

  array<lvr2::OptionalFaceHandle, 2> facepair;
  lvr2::SparseEdgeMap<bool> usedEdges(false);
  for (auto eHStart: mesh_ptr->edges())
  {
    lvr2::SparseVertexMap<bool> usedVertices(false);
    lvr2::SparseEdgeMap<bool> local_usedEdges(false);
    int count = 0;

    // Look for border Edges
    facepair = mesh_ptr->getFacesOfEdge(eHStart);

    // If border Edge found
    if((!facepair[0]||!facepair[1])&&!usedEdges[eHStart])
    {
      std:vector<lvr2::VertexHandle> contour;
      // Set vector which links to the following Edge
      array<lvr2::VertexHandle, 2> vertexPair = mesh_ptr->getVerticesOfEdge(eHStart);
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
        vertexPair = mesh_ptr->getVerticesOfEdge(eHTemp);
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
        mesh_ptr->getEdgesOfVertex(vH, curEdges);

        // Look for other edge of vertex that is a border Edge
        for(auto eHT: curEdges)
        {
          if(!usedEdges[eHT] && !local_usedEdges[eHT])
          {
            facepair = mesh_ptr->getFacesOfEdge(eHT);
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

geometry_msgs::Pose MeshMap::calculatePose(
    const VectorType& current,
    const VectorType& next,
    const NormalType& normal)
{
  VectorType direction(next - current);

  NormalType ez = normal.normalized();
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
  tf_pose.setRotation(tf_pose.getRotation().normalize());
  tf_pose.setOrigin(tf_origin);
  geometry_msgs::Pose pose;
  tf::poseTFToMsg(tf_pose, pose);
  return pose;
}

inline bool MeshMap::waveFrontUpdate(
    lvr2::DenseVertexMap<float>& distances,
    const lvr2::DenseEdgeMap<float>& edge_weights,
    const lvr2::VertexHandle& v1,
    const lvr2::VertexHandle& v2,
    const lvr2::VertexHandle& v3)
{

  const double u1 = distances[v1];
  const double u2 = distances[v2];
  const double u3 = distances[v3];

  const lvr2::OptionalEdgeHandle e12h = mesh_ptr->getEdgeBetween(v1, v2);
  const float c = edge_weights[e12h.unwrap()];
  const float c_sq = c * c;

  const lvr2::OptionalEdgeHandle e13h = mesh_ptr->getEdgeBetween(v1, v3);
  const float b = edge_weights[e13h.unwrap()];
  const float b_sq = b * b;

  const lvr2::OptionalEdgeHandle e23h = mesh_ptr->getEdgeBetween(v2, v3);
  const float a = edge_weights[e23h.unwrap()];
  const float a_sq = a * a;

  /*
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
  */

  const double u1sq = u1*u1;
  const double u2sq = u2*u2;

  const double A = sqrt(std::max<double>((-u1+u2+c)*(u1-u2+c)*(u1+u2-c)*(u1+u2+c), 0));
  const double B = sqrt(std::max<double>((-a+b+c)*(a-b+c)*(a+b-c)*(a+b+c), 0));

  const double sx = (c_sq+u1sq-u2sq) / (2*c);
  const double sx_sq = sx * sx;

  const double sy = -A/(2*c);
  const double sy_sq = sy * sy;

  const double p  = (-a_sq+b_sq+c_sq) / (2*c);
  const double hc = B / (2*c);

  const double dy = (A + B) / (2*c);
  //const double dx = (u2sq - u1sq + b_sq - a_sq) / (2*c);
  const double dx = p-sx;

  //const double x = dx != sx ? (A*sx - B*p)/((A + B)*c) : dx/c;
  //const double dy = hc-sy;

  const double u3tmp_sq = dx * dx + dy * dy;
  const double u3tmp = sqrt(u3tmp_sq);


  if(u3tmp < u3){
    distances[v3] = static_cast<float>(u3tmp);

    /**
     * compute cutting face
     */

    // left face check

    double S = 0;
    double gamma = 0;
    const lvr2::FaceHandle f0 = mesh_ptr->getFaceBetween(v1, v2, v3).unwrap();
    if(distances[v1] < distances[v2])
    {
      predecessors[v3] = v1;
      S = sy*p - sx*hc;
      gamma = acos((u3tmp_sq + b_sq - sx_sq - sy_sq) / (2*u3tmp*b));
    }
    else  // right face check
    {
      predecessors[v3] = v2;
      S = sx*hc - hc*c + sy*c - sy*p;
      gamma = -acos((a_sq + u3tmp_sq + 2*sx*c - sx_sq - c_sq - sy_sq)/(2*a*u3tmp));
    }

    auto faces = mesh_ptr->getFacesOfEdge(mesh_ptr->getEdgeBetween(predecessors[v3], v3).unwrap());
    lvr2::OptionalFaceHandle f1;

    direction[v3] = static_cast<float>(gamma);

    if(!faces[0] || !faces[1]){  // if contour face, cutting face is the current one
      f1 = f0;
      direction[v3] = 0; // direction lies on g1 or a of the triangle
    }
    else if(faces[0].unwrap() != f0){
      f1 = faces[0]; // since faces[0] must be f1, set it as cutting face
    }
    else if(faces[1].unwrap() != f0){
      f1 = faces[1];
    }

    if (S > 0){
      cutting_faces.insert(v3, f1.unwrap());
      cutting_faces2.insert(v3, f0);
    }
    else if( S < 0)
    {
      cutting_faces.insert(v3, f0);
      cutting_faces2.insert(v3, f1.unwrap());
      direction[v3] *= -1;
    }
    else
    {
      cutting_faces.insert(v3, f0);
      cutting_faces2.insert(v3, f0);
      direction[v3] = 0; // direction lies on g1 or g2
    }

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

  VectorType start_point = mesh_ptr->getVertexPosition(start_vertex);

  lvr2::DenseVertexMap<bool> fixed(mesh_ptr->nextVertexIndex(), false);

  // initialize distances with infinity
  // initialize predecessor of each vertex with itself
  for(auto const &vH : mesh_ptr->vertices())
  {
    distances.insert(vH, std::numeric_limits<float>::infinity());
    predecessors.insert(vH, vH);
  }

  lvr2::Meap<lvr2::VertexHandle, float> pq;

  // Set start distance to zero
  // add start vertex to priority queue
  distances[start_vertex] = 0;
  fixed[start_vertex] = true;

  ROS_INFO_STREAM("Init wave front propagation.");

  // initialize around seed / start vertex
  std::vector<lvr2::VertexHandle> neighbours;
  mesh_ptr->getNeighboursOfVertex(start_vertex, neighbours);

  ROS_INFO_STREAM("Number of neighbours " << neighbours.size());

  for(auto n_vh : neighbours)
  {
    lvr2::OptionalEdgeHandle edge = mesh_ptr->getEdgeBetween(start_vertex, n_vh);
    distances[n_vh] = edge_weights[edge.unwrap()];
    predecessors[n_vh] = start_vertex;
    fixed[n_vh] = true;
    pq.insert(n_vh, distances[n_vh]);
  }
  ROS_INFO_STREAM("Start wave front propagation");

  while(!pq.isEmpty())
  {
    lvr2::VertexHandle current_vh = pq.popMin().key;

    // check if already fixed
    //if(fixed[current_vh]) continue;
    fixed[current_vh] = true;

    std::vector<lvr2::FaceHandle> faces;
    mesh_ptr->getFacesOfVertex(current_vh, faces);

    for(auto fh : faces)
    {
      const auto vertices = mesh_ptr->getVerticesOfFace(fh);
      const lvr2::VertexHandle& a = vertices[0];
      const lvr2::VertexHandle& b = vertices[1];
      const lvr2::VertexHandle& c = vertices[2];

      if(fixed[a] && fixed[b] && fixed[c])
      {
        continue;
      }
      else if(fixed[a] && fixed[b] && !fixed[c]){
        // c is free
        if(costs[c] <= config.cost_limit && waveFrontUpdate(distances, edge_weights, a, b, c))
        {
            //predecessors[c] = (distances[a] < distances[b]) ? a : b;
            pq.insert(c, distances[c]);
        }
      }
      else if(fixed[a] && !fixed[b] && fixed[c]){
        // b is free
        if(costs[b] <= config.cost_limit && waveFrontUpdate(distances, edge_weights, c, a, b))
        {
            //predecessors[b] = (distances[c] < distances[a]) ? c : a;
            pq.insert(b, distances[b]);
        }
      }
      else if(!fixed[a] && fixed[b] && fixed[c]){
        // a if free
        if(costs[a] <= config.cost_limit && waveFrontUpdate(distances, edge_weights, b, c, a))
        {
            //predecessors[a] = (distances[b] < distances[c]) ? b : c;
            pq.insert(a, distances[a]);
        }
      }
      else
      {
        // two free vertices -> skip that face
        ROS_DEBUG_STREAM("two vertices are free.");
        continue;
      }

      //TODO animation mode
      //vertex_costs_pub.publish(lvr_ros::toVertexCostsStamped(distances, "Potential Debug", global_frame, uuid_str));
    }
  }

  ROS_INFO_STREAM("Finished wave front propagation.");

  lvr2::VertexHandle prev = predecessors[goal_vertex];

  if(prev == goal_vertex)
  {
    ROS_WARN("Predecessor of goal not set!");
    return false;
  }

  while(prev != start_vertex)
  {
    path.push_front(prev);
    if(predecessors[prev] == prev){
      ROS_WARN_STREAM("No path found!");
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
  for(auto vH: mesh_ptr->vertices())
  {
    float dist = pos.distanceFrom(mesh_ptr->getVertexPosition(vH));
    if(dist < smallest_dist)
    {
      smallest_dist = dist;
      nearest_handle = vH;
    }
  }
  return nearest_handle;
}

inline const geometry_msgs::Point MeshMap::toPoint(const VectorType& vec)
{
  geometry_msgs::Point p;
  p.x = vec.x;
  p.y = vec.y;
  p.z = vec.z;
  return p;
}


bool MeshMap::pathPlanning(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped> &plan,
    bool fmm)
{
  std::list<lvr2::VertexHandle> path;

  combineVertexCosts();
  ros::WallTime t_start, t_end;
  t_start = ros::WallTime::now();
  if(fmm){
    ROS_INFO("start wave front propagation.");
    if(!waveFrontPropagation(toVectorType(goal.pose.position), toVectorType(start.pose.position), path)){
      vertex_costs_pub.publish(lvr_ros::toVertexCostsStamped(potential, "Potential Debug", global_frame, uuid_str));
      return false;
    }
  }
  else
  {
    ROS_INFO("start dijkstra.");
    if(!dijkstra(toVectorType(goal.pose.position), toVectorType(start.pose.position), path)){
      vertex_costs_pub.publish(lvr_ros::toVertexCostsStamped(potential, "Potential Debug", global_frame, uuid_str));
      return false;
    }
  }
  t_end = ros::WallTime::now();
  double execution_time = (t_end - t_start).toNSec() * 1e-6;
  ROS_INFO_STREAM("Exectution time (ms): " << execution_time << " for " << mesh_ptr->numVertices() << " num vertices in the mesh_ptr->");


  visualization_msgs::MarkerArray vector_field;
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = global_frame;
  geometry_msgs::Vector3 scale;
  scale.x = 0.07;
  scale.y = 0.02;
  scale.z = 0.02;

  unsigned int cnt = 0;
  vector_field.markers.reserve(2*predecessors.numValues());

  visualization_msgs::Marker vector;
  vector.type = visualization_msgs::Marker::ARROW;
  vector.header = header;
  vector.ns = "vector_field";


  for(auto v3 : mesh_ptr->vertices())
  {
    if(vertex_costs[v3] > config.cost_limit || !predecessors.containsKey(v3)) continue;
    const lvr2::VertexHandle& v1 = predecessors[v3];
    if(v1 != v3)
    {
      vector.color = lvr_ros::getRainbowColor(vertex_costs[v3]);

      // find b of the corresponding face
      const auto& edge_b = mesh_ptr->getEdgeBetween(v3, v1).unwrap();
      const auto& faces = mesh_ptr->getFacesOfEdge(edge_b);

      lvr2::OptionalVertexHandle v2;
      lvr2::OptionalVertexHandle v22;

      /*
      lvr2::OptionalFaceHandle face;
      for(auto fH : faces)
      {
        if(!fH) continue;
        const auto& vertices = mesh_ptr->getVerticesOfFace(fH.unwrap());
        for(auto vH : vertices)
        {
          if(vH != v3 && vH != v1 && (!v2 || potential[vH] < potential[v2.unwrap()]))
          {
            face = fH;
            v2 = vH;
            break;
          }
        }
      }
      */

      const auto& optFh = cutting_faces.get(v3);
      const auto& optFh2 = cutting_faces2.get(v3);
      if(!optFh || !optFh2) continue;
      const lvr2::FaceHandle& fH = optFh.get();
      const lvr2::FaceHandle& fH2 = optFh2.get();

      auto face = mesh_ptr->getVerticesOfFace(fH);
      auto face2 = mesh_ptr->getVerticesOfFace(fH2);

      vector.scale = scale;
      for(auto vH : face)
      {
        if (vH!=v1 && vH!=v3)
        {
          v2 = vH;
          break;
        }
      }

      for(auto vH : face2)
      {
        if (vH!=v1 && vH!=v3)
        {
          v22 = vH;
          break;
        }
      }

      const auto& vec3 = mesh_ptr->getVertexPosition(v3);
      const auto& vec1 = mesh_ptr->getVertexPosition(v1);

      if(v2 && v22)
      {
        const auto& vec2 = mesh_ptr->getVertexPosition(v2.unwrap());
        const auto& vec22 = mesh_ptr->getVertexPosition(v22.unwrap());
        //const auto dirVec = vec2 - vec1;
        const auto dir2Vec = vec22 - vec1;
        //VectorType sVec = vec1+(dirVec*0.5);//direction[v3]);
        VectorType sVec2 = vec1+(dir2Vec*0.5);//direction[v3]);

        const auto dirVec = vec1 - vec3;
        VectorType sVec = vec3 + dirVec.rotated(face_normals[fH], -direction[v3]);
        vector.pose = calculatePose(vec3, sVec, face_normals[fH]);
        //vector.scale.x = (vec3-sVec).length();
        vector.header.seq = cnt;
        vector.id = cnt++;
        vector_field.markers.push_back(vector);

        vector.color = lvr_ros::getRainbowColor(1);
        //const auto dirVec = vec3 - vec1;
        //VectorType sVec = vec1 + dirVec.rotated(face_normals[face.unwrap()], direction[v3]);
        vector.pose = calculatePose(vec3, sVec2, face_normals[fH2]);
        //vector.scale.x = (vec3-sVec).length();
        vector.header.seq = cnt;
        vector.id = cnt++;
        vector_field.markers.push_back(vector);
      }
      else{
        ROS_ERROR_STREAM("v2 does not exists!");
        vector.pose = calculatePose(vec3, vec1, vertex_normals[v3]);
        vector.header.seq = cnt;
        vector.id = cnt++;
        vector_field.markers.push_back(vector);
      }

    }
  }
  vector_pub.publish(vector_field);

  path.reverse();

  lvr2::VertexHandle prev = path.front();

  for(auto &vH : path)
  {
    geometry_msgs::PoseStamped pose;
    pose.header = header;
    pose.pose = calculatePose(
        mesh_ptr->getVertexPosition(prev),
        mesh_ptr->getVertexPosition(vH),
        vertex_normals[prev]);
    prev = vH;
    plan.push_back(pose);
  }

  nav_msgs::Path path_msg;
  path_msg.poses = plan;
  path_msg.header = header;

  path_pub.publish(path_msg);
  vertex_costs_pub.publish(lvr_ros::toVertexCostsStamped(potential, "Potential", global_frame, uuid_str));

  return true;
}


bool MeshMap::resetLayers()
{
  return true; //TODO implement
}

void MeshMap::publishCostLayers()
{

  for(auto& layer : layers)
  {
    vertex_costs_pub.publish(
        lvr_ros::toVertexCostsStamped(
            layer.second->costs(),
            mesh_ptr->numVertices(),
            layer.second->defaultValue(),
            layer.first,
            global_frame,
            uuid_str));
  }
  vertex_costs_pub.publish(lvr_ros::toVertexCostsStamped(vertex_costs, "Combined Costs", global_frame, uuid_str));
}

void MeshMap::reconfigureCallback(mesh_map::MeshMapConfig& cfg, uint32_t level)
{
  ROS_INFO_STREAM("Dynamic reconfigure callback...");
  if(first_config)
  {
    config = cfg;
    first_config = false;
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

  if(!first_config && map_loaded)
  {
    // Check the dynamic local radius parameter
    if( config.roughness_radius != cfg.roughness_radius ||
        config.height_diff_radius != cfg.height_diff_radius ||
        config.roughness_threshold != cfg.roughness_threshold ||
        config.height_diff_threshold != cfg.height_diff_threshold ||
        config.min_contour_size != cfg.min_contour_size ||
        config.inscribed_radius != cfg.inscribed_radius ||
        config.inflation_radius != cfg.inflation_radius ||
        config.inscribed_value != cfg.inscribed_value ||
        config.lethal_value != cfg.lethal_value)
    {
      /*
      if(config_.roughness_radius != config.roughness_radius)
      {
        roughness_ = lvr2::calcVertexRoughness(*mesh_ptr, config.roughness_radius, vertex_normals_);
      }
      else if(config_.height_diff_radius != config.height_diff_radius)
      {
        height_diff_ = lvr2::calcVertexRoughness(*mesh_ptr, config.height_diff_radius, vertex_normals_);
      }

      lethalCostInflation(
          config.min_contour_size,
          config.height_diff_threshold,
          config.roughness_threshold,
          config.inflation_radius,
          config.inscribed_radius,
          config.inscribed_value,
          config.lethal_value);

      */

      combineVertexCosts();
      publishCostLayers();
    }

    config = cfg;
  }
  // Apply the current configuration
}



const std::string MeshMap::getGlobalFrameID()
{
  return global_frame;
}

} /* namespace mesh_map */





