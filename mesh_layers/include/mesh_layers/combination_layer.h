/*
 *  Copyright 2025, Justus Braun
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
 *    Justus Braun <jubraun@uni-osnabrueck.de>
 *
 */
#pragma once

#include <mesh_map/abstract_layer.h>
#include <rclcpp/rclcpp.hpp>

namespace mesh_layers
{

class MaxCombinationLayer
: public mesh_map::AbstractLayer
{
public:

  bool readLayer() override {return false;}
  bool writeLayer() override {return true;}
  float defaultValue() override {return 0.0;}
  float threshold() override {return 1.0;}
  const lvr2::VertexMap<float>& costs() override {return costs_;}
  const std::set<lvr2::VertexHandle>& lethals() override {return lethals_;}

  /**
  * @brief Combines the configured layers
  */
  bool computeLayer() override;
  
  /**
  * @brief Process a change in a lower layer
  */
  virtual void onInputChanged(
    const rclcpp::Time& timestamp,
    const std::set<lvr2::VertexHandle>& changed
  ) override;

  /**
  * @brief Initialize the layer
  */
  bool initialize() override;

private:
  // Combined costs
  lvr2::DenseVertexMap<float> costs_;

  // Combined lethal vertices
  std::set<lvr2::VertexHandle> lethals_;

  // Configured layers; use weak_ptr to prevent shared_ptr cycles
  std::vector<std::weak_ptr<mesh_map::AbstractLayer>> inputs_;
};


class CombinationLayer
: public mesh_map::AbstractLayer
{
public:

  bool readLayer() override {return false;}
  bool writeLayer() override {return true;}
  float defaultValue() override {return 0.0;}
  float threshold() override {return 1.0;}
  lvr2::VertexMap<float>& costs() override {return costs_;}
  std::set<lvr2::VertexHandle>& lethals() override {return lethals_;}

  /**
  * @brief Combines the configured layers
  */
  bool computeLayer() override;
  
  /**
  * @brief Process a change in a lower layer
  */
  virtual void onInputChanged(
    const rclcpp::Time& timestamp,
    const std::set<lvr2::VertexHandle>& changed
  ) override;

  /**
  * @brief Initialize the layer
  */
  bool initialize() override;

private:
  // Combined costs
  lvr2::DenseVertexMap<float> costs_;

  // Combined lethal vertices
  std::set<lvr2::VertexHandle> lethals_;

  // Configured layers; use weak_ptr to prevent shared_ptr cycles
  std::vector<std::weak_ptr<mesh_map::AbstractLayer>> inputs_;
};

} // namespace mesh_map
