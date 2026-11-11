#include <memory>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "bspline/bspline_optimizer.h"
#include "bspline/non_uniform_bspline.h"
#include "polynomial/polynomial_traj.h"
#include "traj_utils/planning_visualization.h"

namespace fast_planner {
namespace {

void ensureRosInitialized() {
  if (!rclcpp::ok()) {
    int argc = 0;
    char **argv = nullptr;
    rclcpp::init(argc, argv);
  }
}

TEST(PolynomialTrajectoryTest, SatisfiesEndpointPositionVelocityAndAcceleration) {
  PolynomialTraj trajectory;
  const Eigen::Vector3d start(0.0, 1.0, 2.0);
  const Eigen::Vector3d goal(2.0, 3.0, 4.0);
  const Eigen::Vector3d zero = Eigen::Vector3d::Zero();

  PolynomialTraj::oneSegmentTraj(start, goal, zero, zero, zero, zero, 2.0, trajectory);

  EXPECT_TRUE(trajectory.evaluate(0.0, 0).isApprox(start, 1e-9));
  EXPECT_TRUE(trajectory.evaluate(2.0, 0).isApprox(goal, 1e-9));
  EXPECT_TRUE(trajectory.evaluate(3.0, 0).isApprox(goal, 1e-9));
  EXPECT_NEAR(trajectory.evaluate(0.0, 1).norm(), 0.0, 1e-9);
  EXPECT_NEAR(trajectory.evaluate(2.0, 1).norm(), 0.0, 1e-9);
  EXPECT_NEAR(trajectory.evaluate(0.0, 2).norm(), 0.0, 1e-9);
  EXPECT_NEAR(trajectory.evaluate(2.0, 2).norm(), 0.0, 1e-9);
  EXPECT_DOUBLE_EQ(trajectory.getTotalTime(), 2.0);

  PolynomialTraj empty;
  EXPECT_THROW(empty.evaluate(0.0, 0), std::logic_error);
  EXPECT_THROW(PolynomialTraj::oneSegmentTraj(start, goal, zero, zero, zero, zero, 0.0, empty),
               std::invalid_argument);
}

TEST(NonUniformBsplineTest, InterpolatesWaypointsAndBoundaryDerivatives) {
  const std::vector<Eigen::Vector3d> waypoints = {
      {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {3.0, 0.0, 0.0}};
  const std::vector<Eigen::Vector3d> derivatives = {
      {1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};

  Eigen::MatrixXd control_points;
  NonUniformBspline::parameterizeToBspline(1.0, waypoints, derivatives, 3, control_points);
  NonUniformBspline spline(control_points, 3, 1.0);

  EXPECT_NEAR(spline.getTimeSum(), 3.0, 1e-9);
  EXPECT_NEAR(spline.evaluateDeBoorT(0.0).norm(), 0.0, 1e-9);
  EXPECT_TRUE(spline.evaluateDeBoorT(3.0).isApprox(waypoints.back(), 1e-9));

  NonUniformBspline velocity = spline.getDerivative();
  EXPECT_TRUE(velocity.evaluateDeBoorT(0.0).isApprox(derivatives[0], 1e-9));
  EXPECT_TRUE(velocity.evaluateDeBoorT(3.0).isApprox(derivatives[1], 1e-9));
  NonUniformBspline acceleration = velocity.getDerivative();
  NonUniformBspline jerk = acceleration.getDerivative();
  EXPECT_TRUE(jerk.evaluateDeBoorT(0.0).allFinite());
  EXPECT_TRUE(jerk.evaluateDeBoorT(3.0).allFinite());

  spline.setPhysicalLimits(2.0, 2.0);
  EXPECT_TRUE(spline.checkFeasibility());
  EXPECT_NEAR(spline.getLength(0.01), 3.0, 1e-3);
}

TEST(BsplineOptimizerTest, LoadsRos2Parameters) {
  ensureRosInitialized();
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("bspline_opt.bspline_degree", 4),
                               rclcpp::Parameter("bspline_opt.safe_distance", 0.5),
                               rclcpp::Parameter("bspline_opt.max_iteration_num", 25),
                               rclcpp::Parameter("bspline_opt.max_iteration_time", 0.02)});
  auto node = std::make_shared<rclcpp::Node>("bspline_optimizer_test", options);

  BsplineOptimizer optimizer;
  optimizer.setParam(*node);
  EXPECT_EQ(optimizer.getBsplineDegree(), 4);
  EXPECT_THROW(optimizer.setPhysicalLimits(0.0, 1.0), std::invalid_argument);
}

TEST(NonUniformBsplineTest, DerivativeBoundsCoverWholeCurveAndRejectDegenerateKnots) {
  Eigen::MatrixXd points(7,3);
  points << 0,0,0, 0,0,0, 0.2,0.2,0, 1,-0.3,0.5, 2,1,0.2, 2,1,0.2, 2,1,0.2;
  NonUniformBspline spline(points,3,0.2);
  const auto bounds=spline.derivativeNormBounds();
  auto velocity=spline.getDerivative(); auto acceleration=velocity.getDerivative();
  ASSERT_GT(bounds[0],0.0); ASSERT_GT(bounds[1],0.0);
  for(double t=0;t<=spline.getTimeSum();t+=0.001) {
    EXPECT_LE(velocity.evaluateDeBoorT(t).norm(),bounds[0]+1e-9);
    EXPECT_LE(acceleration.evaluateDeBoorT(t).norm(),bounds[1]+1e-9);
  }
  auto knots=spline.getKnot(); knots.setZero(); spline.setKnot(knots);
  EXPECT_FALSE(spline.derivativeNormBounds().allFinite());
}

TEST(BsplineOptimizerTest, RunsNloptOnYawControlPoints) {
  ensureRosInitialized();
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("transformer.pose_topic_type", "transform"),
      rclcpp::Parameter("transformer.sensor_pose_topic", "/trajectory_test_pose"),
      rclcpp::Parameter("voxel_mapping.resolution_fine", 1.0),
      rclcpp::Parameter("voxel_mapping.resolution_coarse", 1.0),
      rclcpp::Parameter("map_config.map_size.map_min_x", 0.0),
      rclcpp::Parameter("map_config.map_size.map_min_y", 0.0),
      rclcpp::Parameter("map_config.map_size.map_min_z", 0.0),
      rclcpp::Parameter("map_config.map_size.map_max_x", 4.0),
      rclcpp::Parameter("map_config.map_size.map_max_y", 4.0),
      rclcpp::Parameter("map_config.map_size.map_max_z", 4.0),
      rclcpp::Parameter("bspline_opt.max_iteration_num", 100),
      rclcpp::Parameter("bspline_opt.max_iteration_time", 0.1),
  });
  auto node = std::make_shared<rclcpp::Node>("bspline_nlopt_test", options);
  auto map_server = std::make_shared<voxel_mapping::MapServer>(*node);

  BsplineOptimizer optimizer;
  optimizer.setParam(*node);
  optimizer.setMap(map_server);
  optimizer.setPhysicalLimits(2.0, 2.0);
  optimizer.setBoundaryStates(
      {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
      {{1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}});
  optimizer.setWaypoints({{0.5, 0.0, 0.0}}, {1});

  Eigen::MatrixXd control_points(6, 1);
  control_points << 0.0, 0.2, 0.1, 0.9, 0.8, 1.0;
  const Eigen::MatrixXd initial_points = control_points;
  double knot_span = 1.0;

  optimizer.optimize(control_points, knot_span, BsplineOptimizer::EXPLORATION_YAW_PHASE);

  EXPECT_TRUE(control_points.allFinite());
  EXPECT_GT(knot_span, 0.0);
  EXPECT_FALSE(control_points.isApprox(initial_points, 1e-6));
}

TEST(PlanningVisualizationTest, CreatesRos2MarkerMetadata) {
  ensureRosInitialized();
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("planning_vis.world_frame", "inspection_map")});
  auto node = std::make_shared<rclcpp::Node>("planning_visualization_test", options);
  PlanningVisualization visualization(*node);

  visualization_msgs::msg::Marker marker;
  visualization.fillBasicInfo(marker, Eigen::Vector3d(0.2, 0.3, 0.4),
                              Eigen::Vector4d(1.0, 0.5, 0.25, 0.75), "test_marker", 7,
                              visualization_msgs::msg::Marker::CUBE);

  EXPECT_EQ(marker.header.frame_id, "inspection_map");
  EXPECT_EQ(marker.ns, "test_marker");
  EXPECT_EQ(marker.id, 7);
  EXPECT_EQ(marker.type, visualization_msgs::msg::Marker::CUBE);
  EXPECT_DOUBLE_EQ(marker.scale.x, 0.2);
  EXPECT_DOUBLE_EQ(marker.scale.y, 0.3);
  EXPECT_DOUBLE_EQ(marker.scale.z, 0.4);
  EXPECT_FLOAT_EQ(marker.color.a, 0.75F);
}

} // namespace
} // namespace fast_planner
