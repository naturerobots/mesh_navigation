#include <mesh_map/abstract_layer.h>
#include <pluginlib/class_list_macros.hpp>

namespace mesh_map {

class TestLayer : public mesh_map::AbstractLayer {
  virtual bool readLayer() override { return true; };

  virtual bool writeLayer() override { return true; };

  virtual float threshold() override { return 1.0; };

  virtual float defaultValue() override {
    return std::numeric_limits<float>::infinity();
  }

  virtual bool computeLayer() override{ return true; };

  virtual lvr2::VertexMap<float> &costs() override { return test_costs_; };

  virtual std::set<lvr2::VertexHandle> &lethals() override {
    return lethal_vertices_;
  };

  virtual void
  updateLethal(std::set<lvr2::VertexHandle> &added_lethal,
               std::set<lvr2::VertexHandle> &removed_lethal) override{};

  virtual bool initialize() override { return true; };

  // set of all current lethal vertices
  std::set<lvr2::VertexHandle> lethal_vertices_;
  lvr2::DenseVertexMap<float> test_costs_;
};

PLUGINLIB_EXPORT_CLASS(mesh_map::TestLayer, mesh_map::AbstractLayer)

} // namespace mesh_map