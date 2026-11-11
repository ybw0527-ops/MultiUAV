#include <cmath>
#include <memory>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "pathfinding/astar.h"
#include "pathfinding/path_cost_evaluator.h"

namespace fast_planner {
namespace {

void ensureRosInitialized() {
  if (!rclcpp::ok()) {
    int argc = 0;
    char **argv = nullptr;
    rclcpp::init(argc, argv);
  }
}

std::shared_ptr<rclcpp::Node> makeTestNode() {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("transformer.pose_topic_type", "transform"),
      rclcpp::Parameter("transformer.sensor_pose_topic", "/pathfinding_test_pose"),
      rclcpp::Parameter("voxel_mapping.resolution_fine", 1.0),
      rclcpp::Parameter("voxel_mapping.resolution_coarse", 1.0),
      rclcpp::Parameter("map_config.map_size.map_min_x", 0.0),
      rclcpp::Parameter("map_config.map_size.map_min_y", 0.0),
      rclcpp::Parameter("map_config.map_size.map_min_z", 0.0),
      rclcpp::Parameter("map_config.map_size.map_max_x", 6.0),
      rclcpp::Parameter("map_config.map_size.map_max_y", 6.0),
      rclcpp::Parameter("map_config.map_size.map_max_z", 3.0),
      rclcpp::Parameter("astar.lambda_heuristic", 1.0),
      rclcpp::Parameter("astar.allocate_num", 5000),
      rclcpp::Parameter("astar.epsilon", 1e-4),
      rclcpp::Parameter("astar.profile.default.resolution", 1.0),
      rclcpp::Parameter("astar.profile.default.max_search_time", 1.0),
  });
  return std::make_shared<rclcpp::Node>("pathfinding_test", options);
}

TEST(AstarTest, FindsStraightAndObstacleAvoidingPaths) {
  ensureRosInitialized();
  auto node = makeTestNode();
  auto map_server = std::make_shared<voxel_mapping::MapServer>(*node);
  auto occupancy = map_server->getOccupancyGrid();

  for (VoxelAddress address = 0; address < map_server->getVoxelNum(); ++address)
    ASSERT_TRUE(occupancy->setOccupancy(address, voxel_mapping::OccupancyType::FREE));

  Astar astar;
  astar.init(*node, map_server);

  const Eigen::Vector3d start(0.5, 2.5, 1.5);
  const Eigen::Vector3d goal(5.5, 2.5, 1.5);

  EXPECT_EQ(astar.search(start, goal), Astar::REACH_END);
  const auto straight_path = astar.getPath();
  ASSERT_EQ(straight_path.size(), 2U);
  EXPECT_NEAR(Astar::pathLength(straight_path), 5.0, 1e-9);

  astar.reset();
  const VoxelAddress obstacle = occupancy->positionToAddress(Position(3.5, 2.5, 1.5));
  ASSERT_TRUE(occupancy->setOccupancy(obstacle, voxel_mapping::OccupancyType::OCCUPIED));

  EXPECT_EQ(astar.search(start, goal), Astar::REACH_END);
  const auto detour_path = astar.getPath();
  ASSERT_GE(detour_path.size(), 3U);
  EXPECT_GT(Astar::pathLength(detour_path), 5.0);
  for (const auto &point : detour_path)
    EXPECT_NE(map_server->getOccupancy(point), voxel_mapping::OccupancyType::OCCUPIED);

  rclcpp::NodeOptions limited_options;
  limited_options.parameter_overrides({
      rclcpp::Parameter("astar.allocate_num", 1),
      rclcpp::Parameter("astar.profile.default.resolution", 1.0),
      rclcpp::Parameter("astar.profile.default.max_search_time", 1.0),
  });
  auto limited_node = std::make_shared<rclcpp::Node>("limited_astar_test", limited_options);
  Astar limited_astar;
  limited_astar.init(*limited_node, map_server);
  EXPECT_EQ(limited_astar.search(start, goal), Astar::NO_PATH);
}

TEST(PathCostEvaluatorTest, ComputesTranslationAndYawLowerBound) {
  PathCostEvaluator::vm_ = 2.0;
  PathCostEvaluator::am_ = 1.0;
  PathCostEvaluator::yd_ = 1.0;
  PathCostEvaluator::ydd_ = 1.0;

  const Eigen::Vector3d start = Eigen::Vector3d::Zero();
  const Eigen::Vector3d goal(2.0, 0.0, 0.0);
  const std::vector<Eigen::Vector3d> empty_path;

  EXPECT_NEAR(PathCostEvaluator::computePathCost(start, goal, 0.0, 0.0,
                                                 Eigen::Vector3d::Zero(), 0.0, empty_path, 2.0),
              1.0, 1e-9);
  EXPECT_NEAR(PathCostEvaluator::computePathCost(start, goal, 0.0, M_PI,
                                                 Eigen::Vector3d::Zero(), 0.0, empty_path, 2.0),
              M_PI + 0.5, 1e-9);
  EXPECT_NEAR(PathCostEvaluator::computePathCost(start, goal, 8 * M_PI, M_PI,
                                                 Eigen::Vector3d::Zero(), 0.0, empty_path, 2.0),
              M_PI + 0.5, 1e-9);
}

} // namespace
} // namespace fast_planner
