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

namespace mesh_map{

  MeshMap::MeshMap(tf::TransformListener& tf_listener)
    : tf_listener_(tf_listener),
      private_nh_("~")
  {
    private_nh_.param<std::string>("mesh_file", mesh_file_, "mesh.h5");
    private_nh_.param<std::string>("mesh_part", mesh_part_, "mesh");
    private_nh_.param<std::string>("global_frame", global_frame_, "map");
    ROS_INFO_STREAM("mesh file is set to: " << mesh_file_);

    mesh_geometry_pub_ = private_nh_.advertise<mesh_msgs::MeshGeometryStamped>("mesh", 1, true);
    vertex_costs_pub_ = private_nh_.advertise<mesh_msgs::MeshVertexCostsStamped>("vertex_costs", 1, false);
  }

  bool MeshMap::readMap()
  {
    return readMap(mesh_file_, mesh_part_);
  }

  bool MeshMap::readMap(const std::string& mesh_file, const std::string& mesh_part)
  {
    ROS_INFO_STREAM("Start reading the mesh part '" << mesh_part << "' from the map file '"<< mesh_file << "'...");
    mesh_io_ptr = std::shared_ptr<lvr2::AttributeMeshIOBase>(new lvr2::HDF5IO(mesh_file, mesh_part));
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

    private_nh_.param<float>("local_neighborhood", local_neighborhood_, 0.3f);


    // TODO read and write uuid
    boost::uuids::random_generator gen;
    boost::uuids::uuid uuid = gen();
    std::string uuid_str = boost::uuids::to_string(uuid);


    auto face_normals_opt =
        mesh_io_ptr->getDenseAttributeMap<lvr2::DenseFaceMap<lvr2::Normal<BaseVec>>>("face_normals");

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
        mesh_io_ptr->getDenseAttributeMap<lvr2::DenseVertexMap<lvr2::Normal<BaseVec>>>("vertex_normals");

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

    mesh_geometry_pub_.publish(lvr_ros::toMeshGeometryStamped<lvr2::BaseVec>(mesh, global_frame_, uuid_str, vertex_normals_));

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
      roughness_ = lvr2::calcVertexRoughness(mesh, local_neighborhood_, vertex_normals_);
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

    sleep(1);

    vertex_costs_pub_.publish(lvr_ros::toVertexCostsStamped(roughness_, "Roughness", global_frame_, uuid_str));

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
      height_diff_ = lvr2::calcVertexHeightDifferences(mesh, local_neighborhood_);
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

    vertex_costs_pub_.publish(lvr_ros::toVertexCostsStamped(height_diff_, "Height Differences", global_frame_, uuid_str));

  }

  bool MeshMap::resetLayers()
  {
    return true; //TODO implement
  }

  const std::string MeshMap::getGlobalFrameID()
  {
    return global_frame_;
  }

} /* namespace mesh_map */





