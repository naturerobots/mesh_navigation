/*
 *  Copyright 2020, The authors
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
 *    Malte kl. Piening <malte@klpiening.de>
 *
 */

#ifndef MESH_MAP__RIDGE_LAYER_H
#define MESH_MAP__RIDGE_LAYER_H

#include <dynamic_reconfigure/server.h>
#include <mesh_layers/RidgeLayerConfig.h>
#include <mesh_map/abstract_layer.h>

namespace mesh_layers
{
/**
 * @brief Costmap layer which assigns high costs to ridges. This is useful for dam avoidance in agricultural
 * applications
 */
class RidgeLayer : public mesh_map::AbstractLayer
{
  /**
   * @brief try read layer from map file
   *
   * @return true if successul; else false
   */
  virtual bool readLayer();

  /**
   * @brief try to write layer to map file
   *
   * @return true if successfull; else false
   */
  virtual bool writeLayer();

  /**
   * @brief delivers the threshold above which vertices are marked lethal
   *
   * @return lethal threshold
   */
  virtual float threshold();

  /**
   * @brief delivers the default layer value
   *
   * @return default value used for this layer
   */
  virtual float defaultValue()
  {
    return std::numeric_limits<float>::infinity();
  }

  /**
   * @brief calculate the values of this layer
   *
   * @return true if successfull; else false
   */
  virtual bool computeLayer(bool hasIO = true);

  /**
   * @brief mark vertices with values above the threshold as lethal
   *
   * @return true if successfull; else false
   */
  bool computeLethals();

  /**
   * @brief deliver the current costmap
   *
   * @return calculated costmap
   */
  virtual lvr2::VertexMap<float>& costs();

  /**
   * @brief deliver set containing all vertices marked as lethal
   *
   * @return lethal vertices
   */
  virtual std::set<lvr2::VertexHandle>& lethals()
  {
    return lethal_vertices;
  }

  /**
   * @brief update set of lethal vertices by adding and removing vertices
   *
   * @param added_lethal vertices to be marked as lethal
   * @param removed_lethal vertices to be removed from the set of lethal vertices
   */
  virtual void updateLethal(std::set<lvr2::VertexHandle>& added_lethal, std::set<lvr2::VertexHandle>& removed_lethal, bool hasIO=true){};

  /**
   * @brief initializes this layer plugin
   *
   * @param name name of this plugin
   *
   * @return true if initialization was successfull; else false
   */
  virtual bool initialize(const std::string& name);

  // costmap
  lvr2::DenseVertexMap<float> ridge;
  // set of lethal vertices
  std::set<lvr2::VertexHandle> lethal_vertices;

  // Server for Reconfiguration
  boost::shared_ptr<dynamic_reconfigure::Server<mesh_layers::RidgeLayerConfig>> reconfigure_server_ptr;
  dynamic_reconfigure::Server<mesh_layers::RidgeLayerConfig>::CallbackType config_callback;
  // true if the first reconfigure config has been received; else false
  bool first_config;
  // current reconfigure config
  RidgeLayerConfig config;

  /**
   * @brief callback for incoming reconfigure configs
   *
   * @param cfg new config
   * @param level level
   */
  void reconfigureCallback(mesh_layers::RidgeLayerConfig& cfg, uint32_t level);
};

} /* namespace mesh_layers */

#endif  // MESH_MAP__RIDGE_LAYER_H
