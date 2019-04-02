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
#include <functional>
#include <mutex>

namespace mesh_map{

MeshMap::MeshMap(tf::TransformListener& tf_listener)
    : tf_listener(tf_listener),
      private_nh("~/mesh_map/"),
      first_config(true),
      map_loaded(false),
      layer_loader("mesh_map", "mesh_map::AbstractLayer"),
      mesh_ptr(new lvr2::HalfEdgeMesh<Vector>())
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

      if (layer_names.find(name) != layer_names.end())
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
        std::pair<std::string, typename mesh_map::AbstractLayer::Ptr> elem(name, plugin_ptr);

        layers.push_back(elem);
        layer_names.insert(elem);

        ROS_INFO_STREAM("The layer plugin with the type \"" << type
          << "\" has been loaded successfully under the name \"" << name << "\".");

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

void MeshMap::layerChanged(const std::string &layer_name){

  std::lock_guard<std::mutex> lock(layer_mtx);

  ROS_INFO_STREAM("Layer \"" << layer_name << "\" changed.");

  lethals.clear();

  ROS_INFO_STREAM("Combine underlining lethal sets...");

  // TODO pre-compute combined lethals upto a layer level
  auto layer_iter = layers.begin();
  for(;layer_iter != layers.end(); layer_iter++)
  {
    lethals.insert(layer_iter->second->lethals().begin(), layer_iter->second->lethals().end());
    // TODO merge with std::set_merge
    if(layer_iter->first == layer_name) break;
  }

  vertex_costs_pub.publish(
      lvr_ros::toVertexCostsStamped(
          layer_iter->second->costs(),
          mesh_ptr->numVertices(),
          layer_iter->second->defaultValue(),
          layer_iter->first,
          global_frame,
          uuid_str));

  if(layer_iter != layers.end()) layer_iter++;

  ROS_INFO_STREAM("Combine  lethal sets...");

  for(;layer_iter != layers.end(); layer_iter++)
  {

    layer_iter->second->updateLethal(lethals, lethals);

    lethals.insert(
        layer_iter->second->lethals().begin(),
        layer_iter->second->lethals().end());

    vertex_costs_pub.publish(
        lvr_ros::toVertexCostsStamped(
            layer_iter->second->costs(),
            mesh_ptr->numVertices(),
            layer_iter->second->defaultValue(),
            layer_iter->first,
            global_frame,
            uuid_str));
  }

  ROS_INFO_STREAM("Found " << lethals.size() << " lethal vertices");
  ROS_INFO_STREAM("Combine layer costs...");

  combineVertexCosts();
  // TODO new lethals old lethals -> renew potential field! around this areas

}

bool MeshMap::initLayerPlugins()
{
  lethals.clear();
  lethal_indices.clear();
  for(auto& layer : layers)
  {
    auto& layer_plugin = layer.second;
    const auto& layer_name = layer.first;

    auto callback = [this](const std::string &layer_name){layerChanged(layer_name);};
    if(!layer_plugin->initialize(layer_name, callback, mesh_ptr, mesh_io_ptr)){
      ROS_ERROR_STREAM("Could not initialize the layer plugin with the name \"" << layer_name << "\"!");
      return false;
    }

    std::set<lvr2::VertexHandle> empty;
    layer_plugin->updateLethal(lethals, empty);
    if(!layer_plugin->readLayer())
    {
      layer_plugin->computeLayer();
    }

    lethal_indices[layer_name].insert(layer_plugin->lethals().begin(), layer_plugin->lethals().end());
    lethals.insert(layer_plugin->lethals().begin(), layer_plugin->lethals().end());
  }
  return true;
}

inline MeshMap::Vector MeshMap::toVector(const geometry_msgs::Point& p)
{
  return Vector(p.x, p.y, p.z);
}

bool MeshMap::dijkstra(
    const Vector& start,
    const Vector& goal,
    std::list<lvr2::VertexHandle>& path)
{
  lvr2::DenseVertexMap<bool> seen(mesh_ptr->nextVertexIndex(), false);

  lvr2::VertexHandle startH = getNearestVertexHandle(start).unwrap();
  lvr2::VertexHandle goalH = getNearestVertexHandle(goal).unwrap();

  return lvr2::Dijkstra<lvr2::BaseVector<float>>(
      *mesh_ptr, startH, goalH, edge_weights, path, potential, predecessors, seen, vertex_costs);
}

bool MeshMap::waveFrontPropagation(
    const Vector& start,
    const Vector& goal,
    std::list<std::pair<Vector, lvr2::FaceHandle>>& path)
{
  return waveFrontPropagation(start, goal, edge_weights, vertex_costs, path, potential, predecessors);
}

void MeshMap::combineVertexCosts()
{
  ROS_INFO_STREAM("Combining costs...");

  float combined_min = std::numeric_limits<float>::max();
  float combined_max = std::numeric_limits<float>::min();

  vertex_costs = lvr2::DenseVertexMap<float>(mesh_ptr->nextVertexIndex(), 0);

  bool hasNaN = false;
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
    hasNaN = false;
    for(auto vH: mesh_ptr->vertices())
    {
      const float cost = costs.containsKey(vH) ? costs[vH] : default_value;
      if(std::isnan(cost)) hasNaN = true;
      vertex_costs[vH] += factor * cost;
      if(std::isfinite(cost))
      {
        combined_max = std::max(combined_max, vertex_costs[vH]);
        combined_min = std::min(combined_min, vertex_costs[vH]);
      }
    }
    if(hasNaN) ROS_ERROR_STREAM("Layer \"" << layer.first << "\" contains NaN values!");
  }

  const float combined_norm = combined_max - combined_min;

  for(auto vH : lethals)
  {
    vertex_costs[vH] = std::numeric_limits<float>::infinity();
  }

  vertex_costs_pub.publish(lvr_ros::toVertexCostsStamped(vertex_costs, "Combined Costs", global_frame, uuid_str));

  hasNaN = false;

  ROS_INFO_STREAM("Layer weighting factor is: " << config.layer_factor);
  for(auto eH: mesh_ptr->edges())
  {
    // Get both Vertices of the current Edge
    std::array<lvr2::VertexHandle, 2> eH_vHs = mesh_ptr->getVerticesOfEdge(eH);
    const lvr2::VertexHandle &vH1 = eH_vHs[0];
    const lvr2::VertexHandle &vH2 = eH_vHs[1];
    // Get the Riskiness for the current Edge (the maximum value from both Vertices)
    if(config.layer_factor != 0)
    {
      if(std::isinf(vertex_costs[vH1]) || std::isinf(vertex_costs[vH2]))
      {
        edge_weights[eH] = edge_distances[eH];
        //edge_weights[eH] = std::numeric_limits<float>::infinity();
      }
      else
      {
        float cost_diff = std::fabs(vertex_costs[vH1] - vertex_costs[vH2]);

        float vertex_factor = config.layer_factor * cost_diff;
        if(std::isnan(vertex_factor))
          ROS_INFO_STREAM("NaN: v1:" << vertex_costs[vH1] << " v2:" << vertex_costs[vH2]
            << " vertex_factor:" << vertex_factor << " cost_diff:" << cost_diff);
        edge_weights[eH] = edge_distances[eH] * (1 + vertex_factor);
      }
    }
    else
    {
      edge_weights[eH] = edge_distances[eH];
    }
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

geometry_msgs::Pose MeshMap::calculatePoseFromDirection(
    const Vector& position, const Vector& direction, const NormalType& normal)
{
  NormalType ez = normal.normalized();
  NormalType ey = normal.cross(direction).normalized();
  NormalType ex = ey.cross(normal).normalized();

  tf::Matrix3x3 tf_basis(
      ex.x, ey.x, ez.x,
      ex.y, ey.y, ez.y,
      ex.z, ey.z, ez.z);

  tf::Vector3 tf_origin(
      position.x,
      position.y,
      position.z);

  tf::Pose tf_pose;
  tf_pose.setBasis(tf_basis);
  tf_pose.setRotation(tf_pose.getRotation().normalize());
  tf_pose.setOrigin(tf_origin);
  geometry_msgs::Pose pose;
  tf::poseTFToMsg(tf_pose, pose);
  return pose;

}

geometry_msgs::Pose MeshMap::calculatePoseFromPosition(
    const Vector& current,
    const Vector& next,
    const NormalType& normal)
{
  return calculatePoseFromDirection(current, next - current, normal);
}

inline bool MeshMap::waveFrontUpdate(
    lvr2::DenseVertexMap<float>& distances,
    const lvr2::DenseEdgeMap<float>& edge_weights,
    const lvr2::VertexHandle& v1,
    const lvr2::VertexHandle& v2,
    const lvr2::VertexHandle& v3)
{

  //TODO all maps as parameters?

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
      gamma = -acos((u3tmp_sq + b_sq - sx_sq - sy_sq) / (2*u3tmp*b));
    }
    else  // right face check
    {
      predecessors[v3] = v2;
      S = sx*hc - hc*c + sy*c - sy*p;
      gamma = acos((a_sq + u3tmp_sq + 2*sx*c - sx_sq - c_sq - sy_sq)/(2*a*u3tmp));
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
    }
    else if( S < 0)
    {
      cutting_faces.insert(v3, f0);
      direction[v3] *= -1;
    }
    else
    {
      cutting_faces.insert(v3, f0);
      direction[v3] = 0; // direction lies on g1 or g2
    }

    return vertex_costs[v3] <= config.cost_limit;
  }
  return false;
}

inline Vector MeshMap::projectVectorOntoPlane(const Vector &vec, const Vector &ref, const NormalType &normal)
{
  return vec - (normal * (vec.dot(normal) - (ref.dot(normal))));
}

inline bool MeshMap::waveFrontPropagation(
    const Vector& start,
    const Vector& goal,
    const lvr2::DenseEdgeMap<float>& edge_weights,
    const lvr2::DenseVertexMap<float>& costs,
    std::list<std::pair<Vector, lvr2::FaceHandle>>& path,
    lvr2::DenseVertexMap<float>& distances,
    lvr2::DenseVertexMap<lvr2::VertexHandle>& predecessors)
{
  ROS_INFO_STREAM("Init wave front propagation.");

  // Find the containing faces of start and goal
  const auto& start_opt = getContainingFaceHandle(start);
  const auto& goal_opt = getContainingFaceHandle(goal);

  // reset cancel planning
  cancel_planning = false;

  if(!start_opt || !goal_opt)
  {
    return false;
  }

  const auto& start_face = start_opt.unwrap();
  const auto& goal_face = goal_opt.unwrap();

  path.clear();
  distances.clear();
  predecessors.clear();

  // TODO in face planning for a single face
  if(goal_face == start_face)
  {
    return true;
  }

  lvr2::DenseVertexMap<bool> fixed(mesh_ptr->nextVertexIndex(), false);

  // clear vector field map
  vector_map.clear();

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
  for(auto vH : mesh_ptr->getVerticesOfFace(start_face))
  {
    const Vector diff = start - mesh_ptr->getVertexPosition(vH);
    const float dist = diff.length();
    distances[vH] = dist;
    vector_map.insert(vH, diff);
    cutting_faces.insert(vH, start_face);
    fixed[vH] = true;
    pq.insert(vH, dist);
  }

  /*
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

  */

  ROS_INFO_STREAM("Start wave front propagation");

  while(!pq.isEmpty() && !cancel_planning)
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
        continue;
      else if(fixed[a] && fixed[b] && !fixed[c]){
        // c is free
        if(waveFrontUpdate(distances, edge_weights, a, b, c))
            pq.insert(c, distances[c]);
      }
      else if(fixed[a] && !fixed[b] && fixed[c]){
        // b is free
        if(waveFrontUpdate(distances, edge_weights, c, a, b))
            pq.insert(b, distances[b]);
      }
      else if(!fixed[a] && fixed[b] && fixed[c]){
        // a if free
        if(waveFrontUpdate(distances, edge_weights, b, c, a))
            pq.insert(a, distances[a]);
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

  if(cancel_planning){
    ROS_WARN_STREAM("Wave front propagation has been canceled!");
    return false;
  }

  ROS_INFO_STREAM("Finished wave front propagation.");

  /*
   * Sampling the path by backtracking the vector field
   */

  computeVectorMap();

  bool path_exists = false;
  for(auto goal_vertex : mesh_ptr->getVerticesOfFace(goal_face))
  {
    if(goal_vertex != predecessors[goal_vertex])
    {
      path_exists = true;
      break;
    }
  }

  if(!path_exists)
  {
    ROS_WARN("Predecessor of the goal is not set! No path found!");
    return false;
  }

  ROS_INFO_STREAM("Start vector field back tracking!");

  constexpr float step_width = 0.03;

  lvr2::FaceHandle current_face = goal_face;
  auto face = mesh_ptr->getVerticesOfFace(current_face);
  auto vertices = mesh_ptr->getVertexPositionsOfFace(current_face);
  Vector vec = projectVectorOntoPlane(goal, vertices[0], face_normals[current_face]);
  path.push_front(std::pair<Vector, lvr2::FaceHandle>(vec, current_face));

  Vector dir;
  while(vec.distance2(start) > step_width && !cancel_planning)
  {
    float u, v, t;
    if(barycentricCoords(vec, vertices[0], vertices[1], vertices[2], u, v))
    {
      float w = 1 - u - v;
      dir = ( vector_map[face[0]]*u + vector_map[face[1]]*v + vector_map[face[2]]*w ).normalized() * step_width ;
    }
    else
    {
      bool foundConnectedFace = false;
      std::list<lvr2::FaceHandle> possible_faces;
      std::vector<lvr2::FaceHandle> neighbour_faces;
      mesh_ptr->getNeighboursOfFace(current_face, neighbour_faces);
      possible_faces.insert(possible_faces.end(), neighbour_faces.begin(), neighbour_faces.end());
      std::list<lvr2::FaceHandle>::iterator current = possible_faces.begin();

      int cnt = 0;
      int max = 40; // TODO to config

      while(possible_faces.end() != current && max != cnt++)
      {
        lvr2::FaceHandle fH = *current;
        vertices = mesh_ptr->getVertexPositionsOfFace(fH);
        face = mesh_ptr->getVerticesOfFace(fH);

        // Projection onto the triangle plane
        Vector tmp_vec = projectVectorOntoPlane(vec, vertices[0], face_normals[fH]);

        // Check if the projected point lies in the current testing face
        if(vector_map.containsKey(face[0]) && vector_map.containsKey(face[1]) && vector_map.containsKey(face[2])
         && barycentricCoords(tmp_vec, vertices[0], vertices[1], vertices[2], u, v))
        {
          foundConnectedFace = true;
          current_face = fH;
          vec = tmp_vec;
          float w = 1 - u - v;
          dir = ( vector_map[face[0]]*u + vector_map[face[1]]*v + vector_map[face[2]]*w ).normalized() * step_width ;
          break;
        }
        else
        {
          // add neighbour of neighbour, if we overstep a small face or the peak of it
          std::vector<lvr2::FaceHandle> nn_faces;
          mesh_ptr->getNeighboursOfFace(fH, nn_faces);
          possible_faces.insert(possible_faces.end(), nn_faces.begin(), nn_faces.end());
        }
        current++;
      }
      if(!foundConnectedFace){
        ROS_ERROR_STREAM("Sample path failed! Could not find a connected face in vector direction!");
        return true; // TODO
      }
    }
    path.push_front(std::pair<Vector, lvr2::FaceHandle>(vec, current_face));
    vec += dir;
  }
  path.push_front(std::pair<Vector, lvr2::FaceHandle>(vec, current_face));
  path.push_front(std::pair<Vector, lvr2::FaceHandle>(start, current_face));

  if(cancel_planning){
    ROS_WARN_STREAM("Wave front propagation has been canceled!");
    return false;
  }

  /*
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
*/

  return true;

}

lvr2::OptionalFaceHandle MeshMap::getContainingFaceHandle(const Vector &pos)
{
  lvr2::OptionalFaceHandle fH;
  lvr2::OptionalVertexHandle vH_opt = getNearestVertexHandle(pos);
  if(vH_opt)
  {
    lvr2::VertexHandle vH = vH_opt.unwrap();
    std::vector<lvr2::FaceHandle> faces;
    mesh_ptr->getFacesOfVertex(vH, faces);

    for(auto face : faces)
    {
      float u, v;
      if(barycentricCoords(pos, face, u, v))
      {
        fH = face;
        break;
      }
    }
  }
  return fH;
}

lvr2::OptionalVertexHandle MeshMap::getNearestVertexHandle(const Vector &pos)
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

inline const geometry_msgs::Point MeshMap::toPoint(const Vector& vec)
{
  geometry_msgs::Point p;
  p.x = vec.x;
  p.y = vec.y;
  p.z = vec.z;
  return p;
}

void MeshMap::publishVectorField()
{
  visualization_msgs::MarkerArray vector_field;
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = global_frame;
  geometry_msgs::Vector3 scale;
  scale.x = 0.07;
  scale.y = 0.02;
  scale.z = 0.02;

  vector_field.markers.reserve(vector_map.numValues());

  unsigned int cnt = 0;
  visualization_msgs::Marker vector;
  vector.type = visualization_msgs::Marker::ARROW;
  vector.header = header;
  vector.ns = "vector_field";
  vector.scale = scale;

  for(auto vH : vector_map)
  {
    vector.color = lvr_ros::getRainbowColor(vertex_costs[vH]);
    vector.pose = calculatePoseFromDirection(mesh_ptr->getVertexPosition(vH), vector_map[vH], face_normals[cutting_faces[vH]]);
    vector.header.seq = cnt;
    vector.id = cnt++;
    vector_field.markers.push_back(vector);
  }

  for(auto fH : mesh_ptr->faces())
  {
    const auto& face = mesh_ptr->getVerticesOfFace(fH);
    if(vector_map.containsKey(face[0]) && vector_map.containsKey(face[1]) && vector_map.containsKey(face[2]))
    {
      const auto& vertices = mesh_ptr->getVertexPositionsOfFace(fH);
      Vector center = (vertices[0] + vertices[1] + vertices[2]) /3;
      float u, v;
      if(barycentricCoords(center, vertices[0], vertices[1], vertices[2], u, v))
      {
        float w = 1 - u - v;
        Vector direction = vector_map[face[0]] * u + vector_map[face[1]] * v + vector_map[face[2]] * w;
        vector.color = lvr_ros::getRainbowColor(u * vertex_costs[face[0]] + v * vertex_costs[face[1]] + w * vertex_costs[face[2]]);
        vector.pose = calculatePoseFromDirection(center, direction, face_normals[fH]);
        vector.header.seq = cnt;
        vector.id = cnt++;
        vector_field.markers.push_back(vector);
      }
      else
      {
        ROS_ERROR_STREAM("Could not compute the barycentric coords!");
      }
    }
  }

  vector_pub.publish(vector_field);
}

void MeshMap::computeVectorMap()
{
  for(auto v3 : mesh_ptr->vertices())
  {
    //if(vertex_costs[v3] > config.cost_limit || !predecessors.containsKey(v3)) continue;
    if(predecessors[v3] == v3) continue;
    const lvr2::VertexHandle& v1 = predecessors[v3];
    // if not predecessor it is pointing to it self
    if(v1 == v3) continue;

    //get the cut face
    const auto& optFh = cutting_faces.get(v3);
    if(!optFh) continue;
    const lvr2::FaceHandle& fH = optFh.get();

    const auto& vec3 = mesh_ptr->getVertexPosition(v3);
    const auto& vec1 = mesh_ptr->getVertexPosition(v1);

    // compute the direction vector and rotate it by theta, which is stored in the direction vertex map
    const auto dirVec = (vec1 - vec3).rotated(face_normals[fH], direction[v3]);
    vector_map.insert(v3, dirVec);
  }
}

bool MeshMap::pathPlanning(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped> &plan,
    bool fmm)
{
  std::list<std::pair<Vector, lvr2::FaceHandle>> path;

  combineVertexCosts();
  ros::WallTime t_start, t_end;
  t_start = ros::WallTime::now();
  if(fmm){
    ROS_INFO("start wave front propagation.");
    if(!waveFrontPropagation(toVector(goal.pose.position), toVector(start.pose.position), path)){
      vertex_costs_pub.publish(lvr_ros::toVertexCostsStamped(potential, "Potential Debug", global_frame, uuid_str));
      return false;
    }
  }

  /*
  else
  {
    ROS_INFO("start dijkstra.");
    if(!dijkstra(toVector(goal.pose.position), toVector(start.pose.position), path)){
      vertex_costs_pub.publish(lvr_ros::toVertexCostsStamped(potential, "Potential Debug", global_frame, uuid_str));
      return false;
    }
  }
   */
  t_end = ros::WallTime::now();
  double execution_time = (t_end - t_start).toNSec() * 1e-6;
  ROS_INFO_STREAM("Exectution time (ms): " << execution_time << " for " << mesh_ptr->numVertices() << " num vertices in the mesh_ptr->");

  path.reverse();

  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = global_frame;


  if(!path.empty()){
    Vector vec = path.front().first;
    lvr2::FaceHandle fH = path.front().second;
    path.pop_front();

    for(auto &next : path)
    {
      geometry_msgs::PoseStamped pose;
      pose.header = header;
      pose.pose = calculatePoseFromPosition(
          vec,
          next.first,
          face_normals[fH]);
      vec = next.first;
      fH = next.second;
      plan.push_back(pose);
    }

  }

  nav_msgs::Path path_msg;
  path_msg.poses = plan;
  path_msg.header = header;

  path_pub.publish(path_msg);
  vertex_costs_pub.publish(lvr_ros::toVertexCostsStamped(potential, "Potential", global_frame, uuid_str));

  //computeVectorMap();
  publishVectorField();

  return true;
}

constexpr float kEpsilon = 1e-8;

bool MeshMap::barycentricCoords(const Vector &p, const lvr2::FaceHandle &triangle, float &u, float &v)
{
  const auto& face = mesh_ptr->getVertexPositionsOfFace(triangle);
  return barycentricCoords(p, face[0], face[1], face[2], u, v);
}

bool MeshMap::barycentricCoords(
    const Vector &p,
    const Vector &v0, const Vector &v1, const Vector &v2,
    float &u, float &v)
{
  // compute plane's normal
  Vector v0v1 = v1 - v0;
  Vector v0v2 = v2 - v0;

  // no need to normalize
  Vector N = v0v1.cross(v0v2); // N
  float denom = N.dot(N);

  // Step 2: inside-outside test
  Vector C; // vector perpendicular to triangle's plane

  // edge 0
  Vector edge0 = v1 - v0;
  Vector vp0 = p - v0;
  C = edge0.cross(vp0);
  if (N.dot(C) < 0) return false; // P is on the right side

  // edge 1
  Vector edge1 = v2 - v1;
  Vector vp1 = p - v1;
  C = edge1.cross(vp1);
  if ((u = N.dot(C)) < 0) return false; // P is on the right side

  // edge 2
  Vector edge2 = v0 - v2;
  Vector vp2 = p - v2;
  C = edge2.cross(vp2);
  if ((v = N.dot(C)) < 0) return false; // P is on the right side;

  u /= denom;
  v /= denom;

  return true; // this ray hits the triangle

}

bool MeshMap::rayTriangleIntersect(const Vector &orig, const Vector &dir,
    const Vector &v0, const Vector &v1, const Vector &v2,
    float &t, float &u, float &v, Vector &p)
{
  // compute plane's normal
  Vector v0v1 = v1 - v0;
  Vector v0v2 = v2 - v0;

  // no need to normalize
  Vector N = v0v1.cross(v0v2); // N
  float denom = N.dot(N);

  // Step 1: finding P

  // check if ray and plane are parallel ?
  float NdotRayDirection = N.dot(dir);
  if (fabs(NdotRayDirection) < kEpsilon) // almost 0
    return false; // they are parallel so they don't intersect !

  // compute d parameter using equation 2
  float d = N.dot(v0);

  // compute t (equation 3)
  t = (N.dot(orig) + d) / NdotRayDirection;

  // check if the triangle is in behind the ray
  //if (t < 0) return false; // the triangle is behind

  // compute the intersection point using equation 1
  p = orig + dir * t;

  // Step 2: inside-outside test
  Vector C; // vector perpendicular to triangle's plane

  // edge 0
  Vector edge0 = v1 - v0;
  Vector vp0 = p - v0;
  C = edge0.cross(vp0);
  if (N.dot(C) < 0) return false; // P is on the right side

  // edge 1
  Vector edge1 = v2 - v1;
  Vector vp1 = p - v1;
  C = edge1.cross(vp1);
  if ((u = N.dot(C)) < 0) return false; // P is on the right side

  // edge 2
  Vector edge2 = v0 - v2;
  Vector vp2 = p - v2;
  C = edge2.cross(vp2);
  if ((v = N.dot(C)) < 0) return false; // P is on the right side;

  u /= denom;
  v /= denom;

  return true; // this ray hits the triangle
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

  if(!first_config && map_loaded)
  {
    config = cfg;
  }
}



const std::string MeshMap::getGlobalFrameID()
{
  return global_frame;
}

} /* namespace mesh_map */





