#include <mesh_layers/height_diff_layer.h>
#include <lvr2/algorithm/GeometryAlgorithms.hpp>
#include <lvr2/algorithm/NormalAlgorithms.hpp>

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mesh_layers::HeightDiffLayer, mesh_map::AbstractLayer)

namespace mesh_layers{

  bool HeightDiffLayer::readLayer()
  {
    ROS_INFO_STREAM("Try to read height differences from map file...");
    auto height_diff_opt = mesh_io_ptr->getDenseAttributeMap<lvr2::DenseVertexMap<float>>("height_diff");

    if(height_diff_opt)
    {
      ROS_INFO_STREAM("Height differences have been read successfully.");
      height_diff = height_diff_opt.get();
      return true;
    }

    return false;
  }

  bool HeightDiffLayer::writeLayer()
  {
      ROS_INFO_STREAM("Saving height_differences to map file...");
      if(mesh_io_ptr->addDenseAttributeMap(height_diff, "height_diff"))
      {
        ROS_INFO_STREAM("Saved height differences to map file.");
        return true;
      }
      else
      {
        ROS_ERROR_STREAM("Could not save height differences to map file!");
        return false;
      }
  }


  float HeightDiffLayer::getThreshold()
  {

  }

  bool HeightDiffLayer::computeLayer(const mesh_map::MeshMapConfig& config)
  {
    height_diff = lvr2::calcVertexHeightDifferences(*mesh_ptr, config.height_diff_radius);
    return true;
  }

  lvr2::VertexMap<float>& HeightDiffLayer::getCosts()
  {
    return height_diff;
  }

  const std::string HeightDiffLayer::getName()
  {
    return "height_diff";
  }

} /* namespace mesh_layers */
