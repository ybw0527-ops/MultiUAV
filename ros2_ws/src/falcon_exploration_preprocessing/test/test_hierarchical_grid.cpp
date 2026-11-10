#include <gtest/gtest.h>
#include "exploration_preprocessing/hierarchical_grid.h"

namespace fast_planner {
namespace {
class GridTest : public ::testing::Test {
protected:
  void SetUp() override {
    if (!rclcpp::ok()) { int argc = 0; rclcpp::init(argc, nullptr); }
    rclcpp::NodeOptions options;
    options.parameter_overrides({
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
    node = std::make_shared<rclcpp::Node>("hgrid_test", options);
    map = std::make_shared<voxel_mapping::MapServer>(*node);
    PathCostEvaluator::map_server_ = map;
    PathCostEvaluator::astar_ = std::make_shared<Astar>();
    PathCostEvaluator::astar_->init(*node, map);
    PathCostEvaluator::caster_ = std::make_shared<RayCaster>();
    PathCostEvaluator::caster_->setParams(map->getResolution(), map->getOrigin());
  }
  void TearDown() override {
    PathCostEvaluator::astar_.reset();
    PathCostEvaluator::caster_.reset();
    PathCostEvaluator::map_server_.reset();
  }
  rclcpp::Node::SharedPtr node;
  voxel_mapping::MapServer::Ptr map;
};

TEST_F(GridTest, BoundsAndFrontierAssignment) {
  HierarchicalGrid grid(*node, map);
  EXPECT_EQ(grid.getLayerNumCells(0), 4);
  EXPECT_EQ(grid.getLayerCellId(0, Position(3, 3, 4)), 3);
  EXPECT_EQ(grid.getLayerCellId(0, Position(4, -2, 1)), -1);
  EXPECT_EQ(grid.getLayerCellId(0, Position(-4, 2, 1)), -1);
  int cell = 0, center = 0;
  grid.getLayerPositionCellCenterId(0, Position(4, -2, 1), cell, center);
  EXPECT_EQ(cell, -1);
  EXPECT_EQ(center, -1);
  grid.inputFrontiers({Position(-1, -1, 1)}, {0.0});
  std::vector<int> ids;
  std::vector<Position> viewpoints;
  grid.getLayerCellFrontiers(0, 0, ids, viewpoints);
  ASSERT_EQ(ids.size(), 1U);
  EXPECT_EQ(ids.front(), 0);
  EXPECT_THROW(grid.inputFrontiers({Position(4, 0, 1)}, {0.0}), std::invalid_argument);
  EXPECT_THROW(grid.inputFrontiers({Position(0, 0, 1)}, {}), std::invalid_argument);
}

TEST_F(GridTest, UnknownMapUpdateAndCostMatrix) {
  HierarchicalGrid grid(*node, map);
  grid.updateHierarchicalGridFromVoxelMap(Position(-3,-3,0), Position(3,3,4));
  grid.inputFrontiers({}, {});
  grid.updateHierarchicalGridFrontierInfo(Position(-3,-3,0), Position(3,3,4));
  Eigen::MatrixXd costs;
  std::map<int, std::pair<int,int>> mapping;
  grid.calculateCostMatrix2(Position(-1,-1,1), Position::Zero(), 0.0, {}, costs, mapping);
  EXPECT_GT(costs.rows(), 1);
  EXPECT_EQ(costs.rows(), costs.cols());
  EXPECT_TRUE(costs.allFinite());
  EXPECT_TRUE((costs.diagonal().array() == 0.0).all());
  double elapsed;
  grid.getLayerSpaceDecompTime(0, elapsed);
  EXPECT_GE(elapsed, 0.0);
}

TEST_F(GridTest, RejectsInvalidParameters) {
  node->declare_parameter("hgrid.num_levels", 2);
  EXPECT_THROW(HierarchicalGrid(*node, map), std::invalid_argument);
  EXPECT_THROW(HierarchicalGrid(*node, nullptr), std::invalid_argument);
}
} // namespace
} // namespace fast_planner
