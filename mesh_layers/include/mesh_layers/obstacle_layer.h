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
 *    Justus Braun <jubraun@uos.de>
 *
 */

#ifndef MESH_MAP__OBSTACLE_LAYER_H
#define MESH_MAP__OBSTACLE_LAYER_H

#include <mesh_map/abstract_layer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace mesh_layers
{
/**
 * @brief Costmap layer that maps obstacle point clouds to the MeshMap's surface.
 *
 * This layer processes PointCloud2 messages that represent obstacles in the environment.
 * Each point in the incomming message is treated as an obstacle. The layer casts a
 * ray downwards along the vertical axis from the origin of each obstacle point.
 * The intersected triangles of the MeshMap are determined to be blocked and their vertices
 * are stored in the `lethal` set of this layer. The vertex costs of these vertices are
 * set to `infinity`, while the default cost value for all other vertices is zero.
 */
class ObstacleLayer
: public mesh_map::AbstractLayer
{
public:
  /**
   * @brief We don't read the ObstacleLayer from disk -> always successful.
   *
   * @return true
   */
  virtual bool readLayer() override { return true; }

  /**
   * @brief We don't write the ObstacleLayer to disk -> always successful.
   *
   * @return true
   */
  virtual bool writeLayer() override { return true; }

  /**
   * @brief Get the default layer value
   *
   * @return default value used for this layer
   */
  virtual float defaultValue() override { return 0.0; }

  /**
   * @brief Returns the threshold above which vertices are marked lethal
   *
   * Not applicable to this layer. This layer directly sets the lethal vertices.
   *
   * @return lethal threshold
   */
  virtual float threshold() override { return std::numeric_limits<float>::infinity(); };

  /**
   * @brief NO-OP: This layer is computed when sensor data is received!
   *
   * @return true
   */
  virtual bool computeLayer() override { return true; };

  /**
   * @brief Get the current costmap
   *
   * @return costmap
   */
  virtual const lvr2::VertexMap<float>& costs() override { return costs_; }

  /**
   * @brief Get set containing all vertices currently marked as lethal
   *
   * @return lethal vertices
   */
  virtual const std::set<lvr2::VertexHandle>& lethals() override { return lethals_; }

private:
  /**
   * @brief initializes this layer plugin
   *
   * @return true if initialization was successfull; else false
   */
  virtual bool initialize() override;

  /**
   * @brief Compute the lethal vertices and cost map from a point cloud message.
   */
  void processPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);

  /**
   * @brief callback for incoming param changes
   */
  rcl_interfaces::msg::SetParametersResult reconfigureCallback(
    std::vector<rclcpp::Parameter> parameters
  );

  // The costmap
  lvr2::SparseVertexMap<float> costs_;
  // The current set of lethal vertices
  std::set<lvr2::VertexHandle> lethals_;
  // Callback group for parallel execution with other layers
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  // The obstacle point cloud subscriber
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  struct {
    double robot_height = std::numeric_limits<float>::infinity();
    double max_obstacle_dist = std::numeric_limits<float>::infinity();
    lvr2::Vector3f down_axis;
    std::string axis_frame_id;
    std::string topic;
    rclcpp::QoS qos = rclcpp::QoS(1).reliable();
    rclcpp::Duration tf_tolerance = rclcpp::Duration::from_seconds(0.1);
  } config_;
};

} /* namespace mesh_layers */

#endif  // MESH_MAP__OBSTACLE_LAYER_H
