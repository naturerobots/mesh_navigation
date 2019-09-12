// TODO fix lvr2
using namespace std;
#include <lvr2/geometry/Normal.hpp>
#include <lvr2/io/Timestamp.hpp>

#include <lvr2/algorithm/GeometryAlgorithms.hpp>
#include <lvr2/algorithm/NormalAlgorithms.hpp>
#include <mesh_layers/height_diff_layer.h>

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mesh_layers::HeightDiffLayer, mesh_map::AbstractLayer)

namespace mesh_layers {

bool HeightDiffLayer::readLayer() {
  ROS_INFO_STREAM("Try to read height differences from map file...");
  auto height_diff_opt =
      mesh_io_ptr->getDenseAttributeMap<lvr2::DenseVertexMap<float>>(
          "height_diff");

  if (height_diff_opt) {
    ROS_INFO_STREAM("Height differences have been read successfully.");
    height_diff = height_diff_opt.get();

    return computeLethals();
  }

  return false;
}

bool HeightDiffLayer::computeLethals()
{
  lethal_vertices.clear();
  for (auto vH : height_diff) {
    if (height_diff[vH] > config.threshold)
      lethal_vertices.insert(vH);
  }
  return true;
}

bool HeightDiffLayer::writeLayer() {
  ROS_INFO_STREAM("Saving height_differences to map file...");
  if (mesh_io_ptr->addDenseAttributeMap(height_diff, "height_diff")) {
    ROS_INFO_STREAM("Saved height differences to map file.");
    return true;
  } else {
    ROS_ERROR_STREAM("Could not save height differences to map file!");
    return false;
  }
}

float HeightDiffLayer::threshold() { return config.threshold; }

bool HeightDiffLayer::computeLayer() {
  height_diff = lvr2::calcVertexHeightDifferences(*mesh_ptr, config.radius);
  return computeLethals();
}

lvr2::VertexMap<float> &HeightDiffLayer::costs() { return height_diff; }

void HeightDiffLayer::reconfigureCallback(
    mesh_layers::HeightDiffLayerConfig &cfg, uint32_t level) {
  ROS_INFO_STREAM("New height diff layer config through dynamic reconfigure.");

  if (first_config) {
    config = cfg;
    first_config = false;
    return;
  }

  if(config.threshold != cfg.threshold)
  {
    computeLethals();
    notifyChange();
  }

  config = cfg;
}

bool HeightDiffLayer::initialize(const std::string &name) {
  reconfigure_server_ptr = boost::shared_ptr<
      dynamic_reconfigure::Server<mesh_layers::HeightDiffLayerConfig>>(
      new dynamic_reconfigure::Server<mesh_layers::HeightDiffLayerConfig>(
          private_nh));

  config_callback =
      boost::bind(&HeightDiffLayer::reconfigureCallback, this, _1, _2);
  reconfigure_server_ptr->setCallback(config_callback);
  return true;
}

} /* namespace mesh_layers */
