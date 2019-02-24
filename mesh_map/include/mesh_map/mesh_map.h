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

#ifndef MESH_NAVIGATION__MESH_MAP_H
#define MESH_NAVIGATION__MESH_MAP_H

#include <lvr2/io/HDF5IO.hpp>
#include <lvr2/geometry/BaseVector.hpp>
#include <tf/transform_listener.h>
#include <mesh_msgs/MeshVertexCosts.h>

namespace mesh_map{

using BaseVec = lvr2::BaseVector<float>;

class MeshMap
{
 public:

  typedef boost::shared_ptr<MeshMap> Ptr;

  MeshMap(tf::TransformListener& tf);

  bool readMap();

  bool readMap(const std::string& mesh_map, const std::string& mesh_part);

  const std::string getGlobalFrameID();

  bool resetLayers();

 private:
  std::shared_ptr<lvr2::AttributeMeshIOBase> mesh_io_ptr;
  lvr2::HalfEdgeMesh<BaseVec> mesh;

  std::string global_frame_;

  std::string mesh_file_;
  std::string mesh_part_;

  lvr2::DenseVertexMap<float> roughness_;
  lvr2::DenseVertexMap<float> height_diff_;
  lvr2::DenseVertexMap<float> riskiness_;
  lvr2::DenseVertexMap<float> potential_;


  lvr2::DenseFaceMap<lvr2::Normal<BaseVec>> face_normals_;
  lvr2::DenseVertexMap<lvr2::Normal<BaseVec>> vertex_normals_;

  ros::Publisher vertex_costs_pub_;

  ros::NodeHandle private_nh_;

  float local_neighborhood_;

};

} /* namespace mesh_map */


#endif //MESH_NAVIGATION__MESH_MAP_H
