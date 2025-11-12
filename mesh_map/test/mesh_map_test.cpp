#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <mesh_map/mesh_map.h>

using namespace ::testing;

struct MeshMapTest : public Test
{
protected:

  void SetUp() override {
    rclcpp::init(0, nullptr);
  }

  // Call this manually at the beginning of each test.
  // Allows setting parameter overrides via NodeOptions (mirrors behavior of how parameters are loaded from yaml via launch file for example)
  void initNodeAndMeshMap(const rclcpp::NodeOptions nodeOptions = rclcpp::NodeOptions()) {
    node_ptr_ = std::make_shared<rclcpp::Node>("mesh_map", "test", nodeOptions);
    tf_buffer_ptr_ = std::make_shared<tf2_ros::Buffer>(node_ptr_->get_clock());
    mesh_map_ptr_ = std::make_shared<mesh_map::MeshMap>(
      *tf_buffer_ptr_, node_ptr_);
  }

  void TearDown() override {
    rclcpp::shutdown();
    mesh_map_ptr_.reset();
    tf_buffer_ptr_.reset();
    node_ptr_.reset();
  }

  std::shared_ptr<mesh_map::MeshMap> mesh_map_ptr_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr_;
  rclcpp::Node::SharedPtr node_ptr_;
};

TEST_F(MeshMapTest, loadsSinglePlugin)
{
  const std::vector<std::string> layer_names{"test_layer"};
  initNodeAndMeshMap(rclcpp::NodeOptions()
    .append_parameter_override("mesh_map.layers", layer_names)
    .append_parameter_override("mesh_map.default_layer", "test_layer")
    .append_parameter_override("mesh_map.test_layer.type", "mesh_map/TestLayer")
  );
  // The MeshMap requires that a map file is loaded to initialize, so we test the layer manager directly
  mesh_map::LayerManager manager(*mesh_map_ptr_, node_ptr_);

  EXPECT_NO_THROW(manager.read_configured_layers());
  EXPECT_TRUE(manager.load_layer_plugins(node_ptr_->get_logger()));
  // This calls back to the mesh map and segfaults because the MeshMap has its own internal LayerManager :(
  // EXPECT_TRUE(manager.initialize_layer_plugins(node_ptr_, mesh_map_ptr_));

  EXPECT_NE(manager.get_layer("test_layer"), nullptr);
}

TEST_F(MeshMapTest, loadsMultiplePlugins)
{
  const std::vector<std::string> layer_names{"t3", "t1", "t2"};
  initNodeAndMeshMap(rclcpp::NodeOptions()
    .append_parameter_override("mesh_map.layers", layer_names)
    .append_parameter_override("mesh_map.default_layer", "t3")
    .append_parameter_override("mesh_map.t1.type", "mesh_map/TestLayer")
    .append_parameter_override("mesh_map.t2.type", "mesh_map/TestLayer")
    .append_parameter_override("mesh_map.t3.type", "mesh_map/TestLayer")
  );
  // The MeshMap requires that a map file is loaded to initialize, so we test the layer manager directly
  mesh_map::LayerManager manager(*mesh_map_ptr_, node_ptr_);

  EXPECT_NO_THROW(manager.read_configured_layers());
  EXPECT_TRUE(manager.load_layer_plugins(node_ptr_->get_logger()));

  EXPECT_NE(manager.get_layer("t1"), nullptr);
  EXPECT_NE(manager.get_layer("t2"), nullptr);
  EXPECT_NE(manager.get_layer("t3"), nullptr);
}
