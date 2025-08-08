/*
 *  Copyright 2024, Justus Braun
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
 *    Justus Braun <jubraun@uos.de>
 *
 */

#ifndef MESH_MAP__FREESPACE_LAYER_H
#define MESH_MAP__FREESPACE_LAYER_H

#include <mesh_map/abstract_layer.h>
#include <rclcpp/rclcpp.hpp>

namespace mesh_layers
{
/**
 * @brief Costmap layer which calculates the free space in the direction of the vertex normals
 */
class ClearanceLayer : public mesh_map::AbstractLayer
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
    return std::numeric_limits<float>::infinity();
  }

  /**
   * @brief delivers the threshold above which vertices are marked lethal
   *
   * @return lethal threshold
   */
  virtual float threshold() override;

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
  }

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

private:
  /**
   * @brief mark vertices without enough clearance as lethal and compute the costs
   *
   * @return true if successfull; else false
   */
  bool computeLethalsAndCosts();

  // distance along vertex normal until the next face
  lvr2::DenseVertexMap<float> clearance_;
  // Actual costs values
  lvr2::DenseVertexMap<float> costs_;
  // set of all current lethal vertices
  std::set<lvr2::VertexHandle> lethal_vertices_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  struct {
    double robot_height = 0.5;
    double height_inflation = 0.3;
  } config_;
};

} /* namespace mesh_layers */

#endif  // MESH_MAP__FREESPACE_LAYER_H
