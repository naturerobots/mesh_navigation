#include <gtest/gtest.h>
#include <gmock/gmock.h>
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
  void initNodeAndPluginManager(const rclcpp::NodeOptions nodeOptions = rclcpp::NodeOptions()) {
    node_ptr_ = std::make_shared<rclcpp::Node>("plugin_manager_test_node", "namespace", nodeOptions);
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
  initNodeAndPluginManager(rclcpp::NodeOptions()
    .append_parameter_override("mesh_map.layers", layer_names)
    .append_parameter_override("mesh_map.test_layer.type", "mesh_map/TestLayer")
  );
  ASSERT_TRUE(mesh_map_ptr_->loadLayerPlugins());
  EXPECT_THAT(mesh_map_ptr_->layer("test_layer"), NotNull());
}

TEST_F(MeshMapTest, loadsMultiplePlugins)
{
  const std::vector<std::string> layer_names{"t3", "t1", "t2"};
  initNodeAndPluginManager(rclcpp::NodeOptions()
    .append_parameter_override("mesh_map.layers", layer_names)
    .append_parameter_override("mesh_map.t1.type", "mesh_map/TestLayer")
    .append_parameter_override("mesh_map.t2.type", "mesh_map/TestLayer")
    .append_parameter_override("mesh_map.t3.type", "mesh_map/TestLayer")
  );
  ASSERT_TRUE(mesh_map_ptr_->loadLayerPlugins());
  EXPECT_THAT(mesh_map_ptr_->layer("t1"), NotNull());
  EXPECT_THAT(mesh_map_ptr_->layer("t2"), NotNull());
  EXPECT_THAT(mesh_map_ptr_->layer("t3"), NotNull());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}