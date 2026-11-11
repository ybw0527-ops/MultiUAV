#include <cmath>
#include <memory>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "fast_planner/planner_manager.h"

namespace fast_planner {
namespace {

void ensureRosInitialized() {
  if (!rclcpp::ok()) {
    int argc = 0;
    char **argv = nullptr;
    rclcpp::init(argc, argv);
  }
}

rclcpp::NodeOptions plannerNodeOptions() {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("uav_model.dynamics_parameters.max_linear_velocity", 2.0),
      rclcpp::Parameter("uav_model.dynamics_parameters.max_linear_acceleration", 2.0),
      rclcpp::Parameter("fast_planner.control_points_distance", 0.25),
      rclcpp::Parameter("transformer.pose_topic_type", "transform"),
      rclcpp::Parameter("transformer.sensor_pose_topic", "/test_sensor_pose"),
      rclcpp::Parameter("voxel_mapping.resolution_fine", 1.0),
      rclcpp::Parameter("voxel_mapping.resolution_coarse", 1.0),
      rclcpp::Parameter("map_config.map_size.map_min_x", -5.0),
      rclcpp::Parameter("map_config.map_size.map_min_y", -5.0),
      rclcpp::Parameter("map_config.map_size.map_min_z", 0.0),
      rclcpp::Parameter("map_config.map_size.map_max_x", 5.0),
      rclcpp::Parameter("map_config.map_size.map_max_y", 5.0),
      rclcpp::Parameter("map_config.map_size.map_max_z", 5.0),
      rclcpp::Parameter("astar.allocate_num", 512),
      rclcpp::Parameter("bspline_opt.max_iteration_num", 100),
      rclcpp::Parameter("bspline_opt.max_iteration_time", 0.02),
  });
  return options;
}

TEST(FastPlannerManagerTest, PlansPositionYawAndChecksCollision) {
  ensureRosInitialized();
  auto node = std::make_shared<rclcpp::Node>("fast_planner_manager_test", plannerNodeOptions());

  FastPlannerManager manager;
  manager.init(*node);

  EXPECT_DOUBLE_EQ(manager.pp_.max_vel_, 2.0);
  EXPECT_DOUBLE_EQ(manager.pp_.max_acc_, 2.0);
  EXPECT_DOUBLE_EQ(manager.pp_.ctrl_pt_dist_, 0.25);
  ASSERT_TRUE(manager.map_server_);
  ASSERT_TRUE(manager.path_finder_);

  const std::vector<Eigen::Vector3d> tour{
      Eigen::Vector3d(0.0, 0.0, 1.0),
      Eigen::Vector3d(0.2, 0.0, 1.0),
      Eigen::Vector3d(0.4, 0.0, 1.0),
  };
  manager.planExplorationPositionTraj(tour, Eigen::Vector3d::Zero(),
                                      Eigen::Vector3d::Zero(), 0.0);

  EXPECT_EQ(manager.local_data_.traj_id_, 1);
  EXPECT_GT(manager.local_data_.duration_, 0.0);
  EXPECT_TRUE(manager.local_data_.start_pos_.isApprox(tour.front(), 1e-3))
      << "actual start: " << manager.local_data_.start_pos_.transpose();
  EXPECT_TRUE(manager.local_data_.end_pos_.isApprox(tour.back(), 1e-3))
      << "actual end: " << manager.local_data_.end_pos_.transpose();

  const std::vector<Eigen::Vector3d> yaw_waypoints{Eigen::Vector3d(0.25, 0.0, 0.0)};
  const std::vector<int> yaw_indices{6};
  manager.planExplorationYawWaypointsTraj(0.0, 0.0, 0.5, yaw_waypoints, yaw_indices);
  EXPECT_TRUE(std::isfinite(manager.local_data_.start_yaw_));
  EXPECT_TRUE(std::isfinite(manager.local_data_.end_yaw_));

  manager.local_data_.start_time_ = node->now();
  EXPECT_TRUE(manager.checkTrajCollision());
  // The UAV center may be inside the task box while its body crosses the wall.
  Eigen::MatrixXd near_boundary = Eigen::MatrixXd::Zero(6,3);
  near_boundary.col(0).setConstant(4.85);
  near_boundary.col(2).setConstant(1.0);
  manager.local_data_.position_traj_ = NonUniformBspline(near_boundary,3,0.1);
  manager.local_data_.duration_ = manager.local_data_.position_traj_.getTimeSum();
  EXPECT_FALSE(manager.checkTrajCollision());
  // Even a trajectory shorter than the sample interval must check its endpoints.
  near_boundary.col(0).setConstant(0.0);
  manager.local_data_.position_traj_ = NonUniformBspline(near_boundary,3,0.001);
  manager.local_data_.duration_ = manager.local_data_.position_traj_.getTimeSum();
  manager.local_data_.start_time_ = node->now();
  auto grid = manager.map_server_->getOccupancyGrid();
  grid->setOccupancy(grid->positionToAddress(Eigen::Vector3d(0,0,1)),voxel_mapping::OccupancyType::OCCUPIED);
  EXPECT_FALSE(manager.checkTrajCollision());
}

TEST(FastPlannerManagerTest, RejectsInvalidTours) {
  ensureRosInitialized();
  auto node = std::make_shared<rclcpp::Node>("fast_planner_invalid_tour_test",
                                             plannerNodeOptions());
  FastPlannerManager manager;
  manager.init(*node);

  EXPECT_THROW(manager.planExplorationPositionTraj({}, Eigen::Vector3d::Zero(),
                                                   Eigen::Vector3d::Zero(), 0.0),
               std::invalid_argument);

  const std::vector<Eigen::Vector3d> repeated{
      Eigen::Vector3d(0.0, 0.0, 1.0),
      Eigen::Vector3d(0.0, 0.0, 1.0),
  };
  EXPECT_THROW(manager.planExplorationPositionTraj(repeated, Eigen::Vector3d::Zero(),
                                                   Eigen::Vector3d::Zero(), 0.0),
               std::invalid_argument);
}

TEST(FastPlannerManagerTest, PlansMultiSegmentTourWithFiniteTiming) {
  ensureRosInitialized();
  auto node = std::make_shared<rclcpp::Node>("fast_planner_multisegment_test",
                                             plannerNodeOptions());
  FastPlannerManager manager;
  manager.init(*node);

  const std::vector<Eigen::Vector3d> tour{
      Eigen::Vector3d(-0.6, -0.3, 3.0),
      Eigen::Vector3d(-0.2, 0.1, 3.1),
      Eigen::Vector3d(0.2, -0.1, 3.0),
      Eigen::Vector3d(0.6, 0.3, 3.1),
  };
  manager.planExplorationPositionTraj(tour, Eigen::Vector3d(0.1, 0.0, 0.0),
                                      Eigen::Vector3d::Zero(), 1.0);

  EXPECT_TRUE(std::isfinite(manager.local_data_.duration_));
  EXPECT_GE(manager.local_data_.duration_, 1.0);
  EXPECT_TRUE(manager.local_data_.position_traj_.evaluateDeBoorT(0.0).allFinite());
  EXPECT_TRUE(manager.local_data_.start_pos_.isApprox(tour.front(),1e-9));
  EXPECT_TRUE(manager.local_data_.velocity_traj_.evaluateDeBoorT(0.0).isApprox(Eigen::Vector3d(0.1,0,0),1e-9));
  EXPECT_LT(manager.local_data_.acceleration_traj_.evaluateDeBoorT(0.0).norm(),1e-9);
  EXPECT_TRUE(manager.local_data_.position_traj_
                  .evaluateDeBoorT(manager.local_data_.duration_)
                  .allFinite());
}

} // namespace
} // namespace fast_planner
