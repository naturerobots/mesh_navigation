// TODO fixing LVR

using namespace std;
#include <lvr2/geometry/Normal.hpp>
#include <lvr2/io/Timestamp.hpp>

#include <lvr2/algorithm/GeometryAlgorithms.hpp>
#include <lvr2/algorithm/NormalAlgorithms.hpp>
#include <mesh_layers/roughness_layer.h>

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mesh_layers::RoughnessLayer, mesh_map::AbstractLayer)

namespace mesh_layers {

bool RoughnessLayer::readLayer() {
  ROS_INFO_STREAM("Try to read roughness from map file...");
  auto roughness_opt =
      mesh_io_ptr->getDenseAttributeMap<lvr2::DenseVertexMap<float>>(
          "roughness");
  if (roughness_opt) {
    ROS_INFO_STREAM("Successfully read roughness from map file.");
    roughness = roughness_opt.get();
    return computeLethals();
  }

  return false;
}

bool RoughnessLayer::writeLayer() {
  if (mesh_io_ptr->addDenseAttributeMap(roughness, "roughness")) {
    ROS_INFO_STREAM("Saved roughness to map file.");
    return true;
  } else {
    ROS_ERROR_STREAM("Could not save roughness to map file!");
    return false;
  }
}

bool RoughnessLayer::computeLethals()
{
  ROS_INFO_STREAM("Compute lethals for \"" << layer_name << "\" (Roughness Layer) with threshold " << config.threshold );
  lethal_vertices.clear();
  for (auto vH : roughness) {
    if (roughness[vH] > config.threshold)
      lethal_vertices.insert(vH);
  }
  ROS_INFO_STREAM("Found " << lethal_vertices.size() << " lethal vertices.");
  return true;
}

float RoughnessLayer::threshold() { return config.threshold; }

bool RoughnessLayer::computeLayer() {
  ROS_INFO_STREAM("Computing roughness...");

  lvr2::DenseFaceMap<mesh_map::Normal> face_normals;

  auto face_normals_opt =
      mesh_io_ptr->getDenseAttributeMap<lvr2::DenseFaceMap<mesh_map::Normal>>(
          "face_normals");

  if (face_normals_opt) {
    face_normals = face_normals_opt.get();
    ROS_INFO_STREAM("Found " << face_normals.numValues()
                             << " face normals in map file.");
  } else {
    ROS_INFO_STREAM(
        "No face normals found in the given map file, computing them...");
    face_normals = lvr2::calcFaceNormals(*mesh_ptr);
    ROS_INFO_STREAM("Computed " << face_normals.numValues()
                                << " face normals.");
    if (mesh_io_ptr->addDenseAttributeMap(face_normals, "face_normals")) {
      ROS_INFO_STREAM("Saved face normals to map file.");
    } else {
      ROS_ERROR_STREAM("Could not save face normals to map file!");
      return false;
    }
  }

  lvr2::DenseVertexMap<mesh_map::Normal> vertex_normals;
  auto vertex_normals_opt =
      mesh_io_ptr->getDenseAttributeMap<lvr2::DenseVertexMap<mesh_map::Normal>>(
          "vertex_normals");

  if (vertex_normals_opt) {
    vertex_normals = vertex_normals_opt.get();
    ROS_INFO_STREAM("Found " << vertex_normals.numValues()
                             << " vertex normals in map file!");
  } else {
    ROS_INFO_STREAM(
        "No vertex normals found in the given map file, computing them...");
    vertex_normals = lvr2::calcVertexNormals(*mesh_ptr, face_normals);
    if (mesh_io_ptr->addDenseAttributeMap(vertex_normals, "vertex_normals")) {
      ROS_INFO_STREAM("Saved vertex normals to map file.");
    } else {
      ROS_ERROR_STREAM("Could not save vertex normals to map file!");
      return false;
    }
  }

  roughness =
      lvr2::calcVertexRoughness(*mesh_ptr, config.radius, vertex_normals);

  return computeLethals();
}

lvr2::VertexMap<float> &RoughnessLayer::costs() { return roughness; }

void RoughnessLayer::reconfigureCallback(mesh_layers::RoughnessLayerConfig &cfg, uint32_t level) {
  bool notify = false;

  ROS_INFO_STREAM("New roughness layer config through dynamic reconfigure.");
  if (first_config) {
    config = cfg;
    first_config = false;
    return;
  }

  if(config.threshold != cfg.threshold)
  {
    computeLethals();
    notify = true;
  }

  if(notify) notifyChange();

  config = cfg;
}

bool RoughnessLayer::initialize(const std::string &name) {
  first_config = true;
  reconfigure_server_ptr = boost::shared_ptr<
      dynamic_reconfigure::Server<mesh_layers::RoughnessLayerConfig>>(
      new dynamic_reconfigure::Server<mesh_layers::RoughnessLayerConfig>(
          private_nh));

  config_callback =
      boost::bind(&RoughnessLayer::reconfigureCallback, this, _1, _2);
  reconfigure_server_ptr->setCallback(config_callback);
  return true;
}

} /* namespace mesh_layers */
