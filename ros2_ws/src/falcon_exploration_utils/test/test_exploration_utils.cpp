#include <gtest/gtest.h>

#include <memory>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "perception_utils/perception_utils.h"
#include "transformer/transformer.h"

namespace {

void ensureRosInitialized() {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
}

geometry_msgs::msg::TransformStamped makeTransform(int seconds, double x) {
  geometry_msgs::msg::TransformStamped message;
  message.header.stamp.sec = seconds;
  message.header.frame_id = "world";
  message.child_frame_id = "camera";
  message.transform.translation.x = x;
  message.transform.rotation.w = 1.0;
  return message;
}

TEST(PerceptionUtilsTest, ClassifiesPinholePoints) {
  fast_planner::PerceptionUtils::Config config;
  config.fov_type = "pinhole";
  config.horizontal_fov = M_PI_2;
  config.vertical_fov = M_PI_2;
  config.max_dist = 5.0;
  config.vis_dist = 1.0;

  fast_planner::PerceptionUtils perception(config);
  perception.setPose(Eigen::Vector3d::Zero(), 0.0);

  EXPECT_TRUE(perception.insideFOV(Eigen::Vector3d(2.0, 0.0, 0.0)));
  EXPECT_FALSE(perception.insideFOV(Eigen::Vector3d(-2.0, 0.0, 0.0)));
  EXPECT_FALSE(perception.insideFOV(Eigen::Vector3d(6.0, 0.0, 0.0)));
}

TEST(PerceptionUtilsTest, ClassifiesOmnidirectionalLidarElevation) {
  fast_planner::PerceptionUtils::Config config;
  config.fov_type = "omni_lidar";
  config.horizontal_fov = 2.0 * M_PI - 1e-3;
  config.vertical_fov = M_PI / 3.0;

  fast_planner::PerceptionUtils perception(config);
  perception.setPose(Eigen::Vector3d::Zero(), 0.0);

  EXPECT_TRUE(perception.insideFOV(Eigen::Vector3d(2.0, 0.0, 0.0)));
  EXPECT_FALSE(perception.insideFOV(Eigen::Vector3d(0.0, 0.0, 2.0)));
}

TEST(TransformerTest, InterpolatesBetweenTimestampedTransforms) {
  ensureRosInitialized();
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("transformer.pose_topic_type", "transform"),
      rclcpp::Parameter("transformer.sensor_pose_topic", "/test_pose")});
  auto node = std::make_shared<rclcpp::Node>("transformer_test", options);
  Transformer transformer(*node);

  transformer.transformCallback(makeTransform(1, 1.0));
  transformer.transformCallback(makeTransform(3, 3.0));

  Transformation interpolated;
  ASSERT_TRUE(transformer.lookupTransform(rclcpp::Time(2, 0, RCL_ROS_TIME), interpolated));
  EXPECT_NEAR(interpolated.getPosition().x(), 2.0, 1e-9);
  EXPECT_NEAR(interpolated.getPosition().y(), 0.0, 1e-9);
  EXPECT_FALSE(transformer.lookupTransform(rclcpp::Time(4, 0, RCL_ROS_TIME), interpolated));
}

}  // namespace
