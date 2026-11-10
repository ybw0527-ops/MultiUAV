#include <memory>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "voxel_mapping/map_server.h"

namespace voxel_mapping {
namespace {

void ensureRosInitialized() {
  if (!rclcpp::ok()) {
    int argc = 0;
    char **argv = nullptr;
    rclcpp::init(argc, argv);
  }
}

TEST(MapServerTest, InitializesFromRosParametersAndTransformsPointcloud) {
  ensureRosInitialized();

  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("transformer.pose_topic_type", "transform"),
      rclcpp::Parameter("transformer.sensor_pose_topic", "/test_sensor_pose"),
      rclcpp::Parameter("voxel_mapping.resolution_fine", 1.0),
      rclcpp::Parameter("voxel_mapping.resolution_coarse", 1.0),
      rclcpp::Parameter("map_config.map_size.map_min_x", -1.0),
      rclcpp::Parameter("map_config.map_size.map_min_y", -1.0),
      rclcpp::Parameter("map_config.map_size.map_min_z", 0.0),
      rclcpp::Parameter("map_config.map_size.map_max_x", 1.0),
      rclcpp::Parameter("map_config.map_size.map_max_y", 1.0),
      rclcpp::Parameter("map_config.map_size.map_max_z", 2.0),
  });

  auto node = std::make_shared<rclcpp::Node>("map_server_test", options);
  MapServer server(*node);

  EXPECT_DOUBLE_EQ(server.getResolution(), 1.0);
  EXPECT_EQ(server.getVoxelNum(), 8);
  EXPECT_TRUE(server.isInBox(Position(0.0, 0.0, 1.0)));
  EXPECT_FALSE(server.isInBox(Position(2.0, 0.0, 1.0)));

  PointCloudType input;
  input.push_back(PointType(1.0F, 2.0F, 3.0F));

  Transformation transform;
  transform.setIdentity();
  transform.getPosition() = Position(4.0, 5.0, 6.0);

  PointCloudType output;
  server.pointcloudTransform(input, transform, output);

  ASSERT_EQ(output.size(), 1U);
  EXPECT_FLOAT_EQ(output.front().x, 5.0F);
  EXPECT_FLOAT_EQ(output.front().y, 7.0F);
  EXPECT_FLOAT_EQ(output.front().z, 9.0F);
}

} // namespace
} // namespace voxel_mapping
