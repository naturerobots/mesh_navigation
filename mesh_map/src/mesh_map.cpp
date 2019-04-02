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
#include <mesh_map/util.h>
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

  // TODO read and write uuid
  boost::uuids::random_generator gen;
  boost::uuids::uuid uuid = gen();
  uuid_str = boost::uuids::to_string(uuid);

  auto face_normals_opt =
      mesh_io_ptr->getDenseAttributeMap<lvr2::DenseFaceMap<Normal>>("face_normals");

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
      mesh_io_ptr->getDenseAttributeMap<lvr2::DenseVertexMap<Normal>>("vertex_normals");

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
    mesh_map::getMinMax(costs, min, max);
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



constexpr float kEpsilon = 1e-8;

bool MeshMap::barycentricCoords(const Vector &p, const lvr2::FaceHandle &triangle, float &u, float &v)
{
  const auto& face = mesh_ptr->getVertexPositionsOfFace(triangle);
  return mesh_map::barycentricCoords(p, face[0], face[1], face[2], u, v);
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


void MeshMap::publishVertexCosts(const lvr2::VertexMap<float>& costs, const std::string& name)
{
  vertex_costs_pub.publish(
      lvr_ros::toVertexCostsStamped(
          costs,
          mesh_ptr->numVertices(),
          0,
          name,
          global_frame,
          uuid_str));
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





