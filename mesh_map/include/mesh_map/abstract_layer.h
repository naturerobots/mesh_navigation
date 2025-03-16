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

#include <functional>
#include <memory>

#include <boost/optional.hpp>
#include <lvr2/io/AttributeMeshIOBase.hpp>
#include <rclcpp/node.hpp>

#include <mesh_map/mesh_map.h>

#ifndef MESH_MAP__ABSTRACT_LAYER_H
#define MESH_MAP__ABSTRACT_LAYER_H
namespace mesh_map
{
class MeshMap;

typedef lvr2::BaseVector<float> Vector;
typedef lvr2::Normal<float> Normal;

typedef std::function<void(const std::string&, const std::set<lvr2::VertexHandle>&)> notify_func;

class AbstractLayer
{
public:
  typedef std::shared_ptr<mesh_map::AbstractLayer> Ptr;

  /**
   * @brief reads the layer data, e.g. from a file, or a database
   * @return true, if the layer data has been read successfully
   */
  virtual bool readLayer() = 0;

  /**
   * @brief Writes the layer data, e.g. to a file, or a database
   * @return true, if the layer data has been written successfully
   */
  virtual bool writeLayer() = 0;

  /**
   * @brief Defines the default value for all vertex costs which are not set in the layer
   * @return The default layer cost value, e.g. nan, 0, or -1, etc.
   */
  virtual float defaultValue() = 0;

  /**
   * @brief Defines the threshold value to consider vertices as a "lethal" obstacle.
   * All vertices with cost values in the layer which a greater than the threshold value
   * are marked as "lethal obstacle".
   * @return
   */
  virtual float threshold() = 0;

  /**
   * @brief Function which is called to compute the layer costs. It is called if the
   * layer information could not be loaded or if another layer triggered an update
   * @return true, if the layer costs have been computed successfully.
   */
  virtual bool computeLayer() = 0;

  /**
   * @brief Returns a vertex map, which associates a cost to each vertex handle.
   * If a vertex handle is not associated with a cost value, i.e. there is no value
   * in the map, the mesh_map will use the default value from threshold().
   * @return a vertex map containing floats.
   */
  virtual lvr2::VertexMap<float>& costs() = 0;

  /**
   * @brief Returns a set of vertex handles which are associated with "lethal" obstacles.
   * @return set of vertex handles which are associated with lethal obstalces.
   */
  virtual std::set<lvr2::VertexHandle>& lethals() = 0;

  /**
   * @brief Called by the mesh map if another previously processed layer triggers an update.
   * @param added_lethal    The "lethal" obstacle vertex handles which are new with respect to the previous call.
   * @param removed_lethal  Old "lethal" obstacle vertex handles, i.e. vertices which are no "lethal" obstacles anymore.
   */
  virtual void updateLethal(std::set<lvr2::VertexHandle>& added_lethal,
                            std::set<lvr2::VertexHandle>& removed_lethal) = 0;
  
  // TODO: Should this replace updateLethal
  virtual void updateInput(const std::set<lvr2::VertexHandle>& /* changed */)
  {}

  /**
   * @brief Optional method if the layer computes vectors. Computes a vector within a triangle using barycentric coordinates.
   * @param vertices The three triangle vertices.
   * @param barycentric_coords The thee barycentric coordinates.
   * @return The vector for the given barycentric coordinates with respect to the corresponding triangle. Default is an vertex with length 0.
   */
  virtual lvr2::BaseVector<float> vectorAt(const std::array<lvr2::VertexHandle, 3>& vertices,
                                           const std::array<float, 3>& barycentric_coords)
  {
    return lvr2::BaseVector<float>();
  }

  /**
   * @brief Optional vector map. Can be implemented if the layer should also compute vectors.
   * If the implmented layer computes a vector field, this method is used to inject
   * the vector field into the mesh map.
   * @return an optional vector map.
   */
  virtual const boost::optional<lvr2::VertexMap<lvr2::BaseVector<float>>&> vectorMap()
  {
    return boost::none;
  }

  /**
   * @brief Optional method if the layer computes vectors. Computes a vector for a given vertex handle
   * @return a vector for the given vertex. Default is an vertex with length 0.
   */
  virtual lvr2::BaseVector<float> vectorAt(const lvr2::VertexHandle& vertex)
  {
    return lvr2::BaseVector<float>();
  }

  /**
   * @brief Initializes the layer plugin under the mesh_map namespace ans sets some basic attributes.
   */
  bool initialize(
    const std::string& name,
    const notify_func notify_update,
    std::shared_ptr<mesh_map::MeshMap> map,
    const rclcpp::Node::SharedPtr node);

  void notifyChange()
  {
    std::set<lvr2::VertexHandle> changed;
    for (const auto& v: costs())
    {
      changed.insert(v);
    }
    this->notifyChange(changed);
  }

  /**
   *  @param changed The vertices whose costs have changed
   */
  void notifyChange(const std::set<lvr2::VertexHandle>& changed)
  {
    this->notify_(layer_name_, changed);
  }

  inline float combinationWeight() const
  {
    return combination_weight_;
  }

protected:

  /**
   * @brief Initializes a custom layer plugin and is invoked at the end of the abstract layer's initialization.
   */
  virtual bool initialize() = 0;

  std::string layer_name_;
  std::shared_ptr<mesh_map::MeshMap> map_ptr_;

  rclcpp::Node::SharedPtr node_;
  std::string layer_namespace_;

  //! Factor used to linearly combine layers to a combined layer in MeshMap
  float combination_weight_ = 1.0;

private:
  notify_func notify_;

  /**
   * @brief declares all ROS parameters
   */
  void declare_parameters();

  /**
   * @brief callback for incoming param changes
   */
  rcl_interfaces::msg::SetParametersResult reconfigureCallback(
    const std::vector<rclcpp::Parameter>& parameters);

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

} /* namespace mesh_map */

#endif  // MESH_MAP__ABSTRACT_LAYER_H
