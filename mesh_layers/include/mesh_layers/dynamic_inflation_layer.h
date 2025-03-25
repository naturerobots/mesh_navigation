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

#ifndef MESH_MAP__DYNAMIC_INFLATION_LAYER
#define MESH_MAP__DYNAMIC_INFLATION_LAYER

#include <mesh_map/abstract_layer.h>
#include <rclcpp/rclcpp.hpp>

namespace mesh_layers
{
const float EPSILON = 1e-9;

struct StopWatch
{
  using Duration = std::chrono::nanoseconds;
  using TS = std::chrono::steady_clock::time_point;

  std::optional<TS> ts_;
  TS begin_;
  std::ofstream logfile_;

#define MAKE_NAMED_END_FN(name) Duration name##_ = Duration(0); \
  inline void end_##name() { \
    TS now = std::chrono::steady_clock::now(); \
    name##_ += now - begin_; \
    begin_ = now; \
  }

  MAKE_NAMED_END_FN(alloc);
  MAKE_NAMED_END_FN(init);
  MAKE_NAMED_END_FN(fading);
  MAKE_NAMED_END_FN(meap);
  MAKE_NAMED_END_FN(seth);
  MAKE_NAMED_END_FN(weak);
  MAKE_NAMED_END_FN(mesh);
  MAKE_NAMED_END_FN(pub);
  MAKE_NAMED_END_FN(vec);

  inline void begin()
  {
    begin_ = std::chrono::steady_clock::now();
    if (!ts_)
    {
      ts_ = begin_;
    }
  }

  inline void open(const std::string& name)
  {
    const std::string filename = name + "_waveCostInflationTimes.csv";
    logfile_.open(filename, std::ios::trunc);
    logfile_ << "Timestamp, Unknown, Allocation, Initialization, Fading, Meap, Sethian, WeakPtr, MeshCalls, Publish, VecField" << std::endl;
  }

  inline void commit()
  {
    const auto end = std::chrono::steady_clock::now();
    // Total minus everything else we measured
    const Duration unknown = (end - ts_.value()) - alloc_ - init_ - fading_ - meap_ - seth_ - weak_ - mesh_ - pub_ - vec_;
    logfile_ << ts_.value().time_since_epoch().count() << ',';
    logfile_ << unknown.count() << ',';
    logfile_ << alloc_.count() << ',';
    logfile_ << init_.count() << ',';
    logfile_ << fading_.count() << ',';
    logfile_ << meap_.count() << ',';
    logfile_ << seth_.count() << ',';
    logfile_ << weak_.count() << ',';
    logfile_ << mesh_.count() << ',';
    logfile_ << pub_.count() << ',';
    logfile_ << vec_.count() << std::endl;

    alloc_ = Duration(0);
    init_ = Duration(0);
    fading_ = Duration(0);
    meap_ = Duration(0);
    seth_ = Duration(0);
    weak_ = Duration(0);
    mesh_ = Duration(0);
    pub_ = Duration(0);
    vec_ = Duration(0);
    
    ts_.reset();
  }
};

/**
 * @brief Costmap layer which inflates around existing lethal vertices
 */
class DynamicInflationLayer : public mesh_map::AbstractLayer
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
  inline bool waveFrontUpdate(
    const lvr2::BaseMesh<mesh_map::Vector>& mesh,
    lvr2::DenseVertexMap<float>& distances,
    lvr2::DenseVertexMap<lvr2::VertexHandle>& predecessors,
    const float& max_distance,
    const lvr2::DenseEdgeMap<float>& edge_weights,
    const lvr2::FaceHandle& fh,
    const lvr2::BaseVector<float>& normal,
    const lvr2::VertexHandle& v1,
    const lvr2::VertexHandle& v2,
    const lvr2::VertexHandle& v3
  );

  /**
   * @brief fade cost value based on lethal and inscribed area
   *
   * @param val input cost value
   *
   * @return resulting cost value
   */
  float fading(const float val);

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
  virtual void updateLethal(std::set<lvr2::VertexHandle>& added_lethal, std::set<lvr2::VertexHandle>& removed_lethal) override;

  virtual void updateInput(const std::set<lvr2::VertexHandle>& changed) override;

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
    double factor = 1.0;
    double lethal_value = 2.0;
    double inscribed_value = 1.0;
    int min_contour_size = 3;
    bool repulsive_field = true;
  } config_;
  
  StopWatch watch_;
};

} /* namespace mesh_layers */

#endif  // MESH_MAP__DYNAMIC_INFLATION_LAYER
