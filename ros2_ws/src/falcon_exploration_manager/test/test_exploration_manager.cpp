#include <filesystem>
#include <gtest/gtest.h>
#include "exploration_manager/exploration_manager.h"
#include "exploration_manager/exploration_data.h"
#include "exploration_manager/exploration_fsm.h"
#include "fast_planner/planner_manager.h"

rclcpp::NodeOptions managerOptions() {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("uav_model.dynamics_parameters.max_linear_velocity", 2.0),
    rclcpp::Parameter("uav_model.dynamics_parameters.max_linear_acceleration", 2.0),
    rclcpp::Parameter("fast_planner.control_points_distance", 0.5),
    rclcpp::Parameter("transformer.pose_topic_type", "transform"),
    rclcpp::Parameter("voxel_mapping.resolution_fine", 0.5),
    rclcpp::Parameter("voxel_mapping.resolution_coarse", 0.5),
    rclcpp::Parameter("map_config.map_size.map_min_x", -3.0),
    rclcpp::Parameter("map_config.map_size.map_min_y", -3.0),
    rclcpp::Parameter("map_config.map_size.map_min_z", 0.0),
    rclcpp::Parameter("map_config.map_size.map_max_x", 3.0),
    rclcpp::Parameter("map_config.map_size.map_max_y", 3.0),
    rclcpp::Parameter("map_config.map_size.map_max_z", 4.0),
    rclcpp::Parameter("hgrid.cell_size_max", 3.0),
    rclcpp::Parameter("hgrid.ccl_step", 1)});
  return options;
}

TEST(ExplorationManagerTest, InitializesAndBuildsUnknownMapTour) {
  if (!rclcpp::ok()) { int argc = 0; rclcpp::init(argc, nullptr); }
  auto options = managerOptions();
  auto node = std::make_shared<rclcpp::Node>("manager_test", options);
  fast_planner::ExplorationManager manager;
  manager.initialize(*node);
  ASSERT_TRUE(manager.hierarchical_grid_);
  ASSERT_TRUE(manager.visualization_);
  EXPECT_EQ(manager.hierarchical_grid_->getLayerNumCells(0), 4);
  EXPECT_EQ(manager.updateFrontierStruct(Eigen::Vector3d(0,0,1)), 0);
  manager.ed_->update_bbox_min_ = Eigen::Vector3d(-3,-3,0);
  manager.ed_->update_bbox_max_ = Eigen::Vector3d(3,3,4);
  manager.initializeHierarchicalGrid(Eigen::Vector3d(0,0,1), Eigen::Vector3d::Zero());
  EXPECT_GT(manager.ed_->grid_tour2_.size(), 1U);
  EXPECT_TRUE(std::filesystem::exists(manager.ep_->tsp_dir_ + "/coverage_path.txt"));
  auto map = manager.planner_manager_->map_server_;
  for (int address = 0; address < map->getVoxelNum(); ++address) {
    ASSERT_TRUE(map->getOccupancyGrid()->setOccupancy(address, voxel_mapping::OccupancyType::FREE));
  }
  auto obstacles = std::make_shared<PointCloudType>();
  obstacles->push_back(PointType(2.75F, 2.75F, 0.25F));
  map->getESDF()->loadMapFromPCL(obstacles);
  EXPECT_EQ(manager.planTrajToView(Eigen::Vector3d(-1,0,2), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d(1,0,2), 0.3),
      fast_planner::SUCCEED);
  EXPECT_GT(manager.planner_manager_->local_data_.duration_, 0.0);
  EXPECT_TRUE(manager.planner_manager_->local_data_.position_traj_.evaluateDeBoorT(0.0).allFinite());
  // A live replanning yaw is unwrapped, whereas frontier yaws lie in [-pi, pi].
  EXPECT_EQ(manager.planTrajToView(Eigen::Vector3d(-1,0,2), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(), Eigen::Vector3d(8 * M_PI + 0.2,0,0),
      Eigen::Vector3d(1,0,2), -0.3), fast_planner::SUCCEED);
  EXPECT_EQ(manager.planTrajToView(Eigen::Vector3d(-1,0,2), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(), Eigen::Vector3d(8 * M_PI,0,0),
      Eigen::Vector3d(-1,0,2), 0.0), fast_planner::FAIL);
  EXPECT_EQ(manager.planTrajToView(Eigen::Vector3d(-1,0,2), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      Eigen::Vector3d(1,0,0.1), 0.0), fast_planner::FAIL);
}


TEST(ExplorationManagerTest, FsmRequiresValidOdometryAndFrontiers) {
  if (!rclcpp::ok()) { int argc = 0; rclcpp::init(argc, nullptr); }
  auto node = std::make_shared<rclcpp::Node>("fsm_test", managerOptions());
  fast_planner::ExplorationFSM fsm;
  fsm.init(*node);
  auto odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("/odom_world", 10);
  auto trigger_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 10);
  nav_msgs::msg::Odometry odom;
  odom.pose.pose.position.z = 1.0;
  odom.pose.pose.orientation.w = 0.0;
  // The zero quaternion must not release INIT.
  auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(2100);
  while (std::chrono::steady_clock::now() < until) {
    odom_pub->publish(odom);
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(fsm.state(), fast_planner::INIT);
  odom.pose.pose.orientation.w = 1.0;
  until = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (std::chrono::steady_clock::now() < until && fsm.state() == fast_planner::INIT) {
    odom_pub->publish(odom);
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_EQ(fsm.state(), fast_planner::WAIT_TRIGGER);
  trigger_pub->publish(geometry_msgs::msg::PoseStamped());
  for (int i=0; i<10; ++i) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(fsm.state(), fast_planner::WAIT_TRIGGER);
}

TEST(ExplorationManagerTest, RejectsInvalidFsmTiming) {
  if (!rclcpp::ok()) { int argc = 0; rclcpp::init(argc, nullptr); }
  auto node = std::make_shared<rclcpp::Node>("invalid_fsm_test", managerOptions());
  node->declare_parameter("exploration_manager.fsm.replan_duration_fast", -1.0);
  fast_planner::ExplorationFSM fsm;
  EXPECT_THROW(fsm.init(*node), std::invalid_argument);
}
