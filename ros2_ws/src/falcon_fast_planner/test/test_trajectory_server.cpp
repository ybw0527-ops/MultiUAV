#include <cmath>
#include <memory>
#include <string>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "bspline/non_uniform_bspline.h"
#include "fast_planner/trajectory_server.h"

namespace fast_planner {
namespace {

void ensureRosInitialized() {
  if (!rclcpp::ok()) {
    int argc = 0;
    char **argv = nullptr;
    rclcpp::init(argc, argv);
  }
}

rclcpp::NodeOptions serverOptions() {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("traj_server.perform_initial_motion", false),
      rclcpp::Parameter("traj_server.init_dx", 0.0),
      rclcpp::Parameter("traj_server.init_dy", 0.0),
      rclcpp::Parameter("traj_server.init_dz", 0.0),
      rclcpp::Parameter("perception_utils.max_dist", 2.0),
      rclcpp::Parameter("perception_utils.vis_dist", 1.0),
  });
  return options;
}

falcon_interfaces::msg::Bspline makeTrajectoryMessage(const rclcpp::Time &start_time) {
  Eigen::MatrixXd position_points(6, 3);
  position_points << 0.0, 0.0, 1.0,
      0.0, 0.0, 1.0,
      0.5, 0.0, 1.0,
      1.0, 0.2, 1.1,
      1.5, 0.2, 1.1,
      1.5, 0.2, 1.1;
  NonUniformBspline spline(position_points, 3, 1.0);

  falcon_interfaces::msg::Bspline message;
  message.order = 3;
  message.traj_id = 7;
  message.start_time = static_cast<builtin_interfaces::msg::Time>(start_time);
  const Eigen::VectorXd knots = spline.getKnot();
  message.knots.resize(static_cast<size_t>(knots.size()));
  for (Eigen::Index index = 0; index < knots.size(); ++index)
    message.knots[static_cast<size_t>(index)] = knots(index);
  message.pos_pts.resize(static_cast<size_t>(position_points.rows()));
  for (Eigen::Index index = 0; index < position_points.rows(); ++index) {
    auto &point = message.pos_pts[static_cast<size_t>(index)];
    point.x = position_points(index, 0);
    point.y = position_points(index, 1);
    point.z = position_points(index, 2);
  }
  message.yaw_pts = {0.0, 0.0, 0.2, 0.4, 0.5, 0.5};
  message.yaw_dt = 1.0;
  return message;
}

TEST(TrajectoryServerTest, ParsesAndSamplesPositionCommands) {
  ensureRosInitialized();
  auto server = std::make_shared<TrajectoryServer>(serverOptions());
  const rclcpp::Time start = server->now();
  const auto message = makeTrajectoryMessage(start);

  std::string error;
  ASSERT_TRUE(server->setTrajectory(message, &error)) << error;
  EXPECT_TRUE(server->hasTrajectory());
  EXPECT_EQ(server->trajectoryId(), 7);
  EXPECT_NEAR(server->trajectoryDuration(), 3.0, 1e-9);

  falcon_interfaces::msg::PositionCommand command;
  EXPECT_FALSE(server->sampleCommand(start - rclcpp::Duration::from_seconds(0.1), command));
  ASSERT_TRUE(server->sampleCommand(start + rclcpp::Duration::from_seconds(1.0), command));
  EXPECT_EQ(command.trajectory_id, 7U);
  EXPECT_EQ(command.header.frame_id, "world");
  EXPECT_TRUE(std::isfinite(command.position.x));
  EXPECT_TRUE(std::isfinite(command.velocity.x));
  EXPECT_TRUE(std::isfinite(command.acceleration.x));
  EXPECT_TRUE(std::isfinite(command.jerk.x));
  EXPECT_GE(command.yaw, -M_PI);
  EXPECT_LE(command.yaw, M_PI);

  ASSERT_TRUE(server->sampleCommand(start + rclcpp::Duration::from_seconds(4.0), command));
  EXPECT_DOUBLE_EQ(command.velocity.x, 0.0);
  EXPECT_DOUBLE_EQ(command.velocity.y, 0.0);
  EXPECT_DOUBLE_EQ(command.velocity.z, 0.0);
  EXPECT_DOUBLE_EQ(command.yaw_dot, 0.0);
}

TEST(TrajectoryServerTest, RejectsMalformedAndMisorderedTrajectories) {
  ensureRosInitialized();
  auto server = std::make_shared<TrajectoryServer>(serverOptions());
  auto message = makeTrajectoryMessage(server->now());

  std::string error;
  ASSERT_TRUE(server->setTrajectory(message, &error)) << error;
  EXPECT_FALSE(server->setTrajectory(message, &error));
  EXPECT_NE(error.find("strictly increasing"), std::string::npos);

  message.traj_id = 8;
  message.knots.pop_back();
  EXPECT_FALSE(server->setTrajectory(message, &error));
  EXPECT_NE(error.find("knot count"), std::string::npos);
}

} // namespace
} // namespace fast_planner
