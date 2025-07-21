/*
 *  Copyright 2020, Sebastian Pütz
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

#ifndef MESH_MAP__INFLATION_LAYER_H
#define MESH_MAP__INFLATION_LAYER_H

#include <mesh_map/abstract_layer.h>
#include <rclcpp/rclcpp.hpp>

namespace mesh_layers
{
const float EPSILON = 1e-9;

/**
 * @brief Costmap layer which inflates around existing lethal vertices
 */
class InflationLayer : public mesh_map::AbstractLayer
{
  /**
   * @brief try read layer from map file
   *
   * @return true if successul; else false
   */
  virtual bool readLayer() override;

  /**
   * @brief try to write layer to map file
   *
   * @return true if successfull; else false
   */
  virtual bool writeLayer() override;

  /**
   * @brief delivers the default layer value
   *
   * @return default value used for this layer
   */
  virtual float defaultValue() override
  {
    return 0;
  }

  /**
   * @brief delivers the threshold above which vertices are marked lethal
   *
   * @return lethal threshold
   */
  virtual float threshold() override;

  inline float computeUpdateSethianMethod(const float& d1, const float& d2, const float& a, const float& b,
                                          const float& dot, const float& F);

  /**
   * @brief updates the wavefront
   *
   * @param distances current distances from the start vertices
   * @param predecessors current predecessors of vertices visited during the wave front propagation
   * @param max_distance max distance of propagation
   * @param edge_weights weights of the edges
   * @param fh current face
   * @param normal normal of the current face
   * @param v1 first vertex of the current face
   * @param v2 second vertex of the current face
   * @param v3 third vertex of the current face
   *
   * @return true if successful; else false
   */
  inline bool waveFrontUpdate(lvr2::DenseVertexMap<float>& distances,
                              lvr2::DenseVertexMap<lvr2::VertexHandle>& predecessors, const float& max_distance,
                              const lvr2::DenseEdgeMap<float>& edge_weights, const lvr2::FaceHandle& fh,
                              const lvr2::BaseVector<float>& normal, const lvr2::VertexHandle& v1,
                              const lvr2::VertexHandle& v2, const lvr2::VertexHandle& v3);

  /**
   * @brief fade cost value based on lethal and inscribed area
   *
   * @param val input cost value
   *
   * @return resulting cost value
   */
  float fading(const float squared_distance);

  /**
   * @brief inflate around lethal vertices by using an wave front propagation and assign riskiness values to vertices
   *
   * @param lethals set of current lethal vertices
   * @param inflation_radius radius of inflation
   * @param inscribed_radius rarius of inscribed area
   * @param inscribed_value value assigned to inscribed vertices
   * @param lethal_value value of lethal vertices
   */
  void waveCostInflation(const std::set<lvr2::VertexHandle>& lethals, const float inflation_radius,
                         const float inscribed_radius, const float inscribed_value, const float lethal_value);

  /**
   * @brief returns repulsive vector at a given position inside a face
   *
   * @param vertices vertices of the face
   * @param barycentric_coords barycentric coordinates of the requested point inside the face
   *
   * @return vector of the current vectorfield
   */
  lvr2::BaseVector<float> vectorAt(const std::array<lvr2::VertexHandle, 3>& vertices,
                                   const std::array<float, 3>& barycentric_coords);

  /**
   * @brief returns the repulsive vector at a given vertex
   *
   * @param vertex requested vertex
   *
   * @return vector of vectorfield at requested vertex
   */
  lvr2::BaseVector<float> vectorAt(const lvr2::VertexHandle& vertex);

  /**
   * @brief delivers the current repulsive vectorfield
   *
   * @return repulsive vectorfield
   */
  const boost::optional<lvr2::VertexMap<lvr2::BaseVector<float>>&> vectorMap()
  {
    return vector_map_;
  }

  /**
   * @brief adds vector pointing back to source of inflation source
   *
   * @param current_vertex vertex to calculate vector for
   * @param predecessors current predecessor map
   * @param[out] vector_map resulting vectorfield
   */
  void backToSource(const lvr2::VertexHandle& current_vertex,
                    const lvr2::DenseVertexMap<lvr2::VertexHandle>& predecessors,
                    lvr2::DenseVertexMap<lvr2::BaseVector<float>>& vector_map);

  /**
   * @brief calculate the values of this layer
   *
   * @return true if successfull; else false
   */
  virtual bool computeLayer() override;

  /**
   * @brief deliver the current costmap
   *
   * @return calculated costmap
   */
  virtual lvr2::VertexMap<float>& costs() override;

  /**
   * @brief deliver set containing all vertices marked as lethal
   *
   * @return lethal vertices
   */
  virtual std::set<lvr2::VertexHandle>& lethals() override
  {
    return lethal_vertices_;
  }  // TODO remove... layer types

  /**
   * @brief update set of lethal vertices by adding and removing vertices
   *
   * @param added_lethal vertices to be marked as lethal
   * @param removed_lethal vertices to be removed from the set of lethal vertices
   */
  virtual void updateLethal(std::set<lvr2::VertexHandle>& added_lethal, std::set<lvr2::VertexHandle>& removed_lethal);

  /**
   * @brief initializes this layer plugin
   *
   * @return true if initialization was successfull; else false
   */
  virtual bool initialize() override;

  /**
   * @brief callback for incoming param changes
   */
  rcl_interfaces::msg::SetParametersResult reconfigureCallback(std::vector<rclcpp::Parameter> parameters);

  lvr2::DenseVertexMap<float> riskiness_;

  lvr2::DenseVertexMap<float> direction_;

  lvr2::DenseVertexMap<lvr2::FaceHandle> cutting_faces_;

  lvr2::DenseVertexMap<lvr2::BaseVector<float>> vector_map_;

  lvr2::DenseVertexMap<float> distances_;

  std::set<lvr2::VertexHandle> lethal_vertices_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  struct {
    double inscribed_radius = 0.25;
    double inflation_radius = 0.4;
    double lethal_value = 1.0;
    double inscribed_value = 0.99;
    double cost_scaling_factor = 1.0;
    int min_contour_size = 3;
    bool repulsive_field = true;
  } config_;
};

} /* namespace mesh_layers */

#endif  // MESH_MAP__INFLATION_LAYER_H
