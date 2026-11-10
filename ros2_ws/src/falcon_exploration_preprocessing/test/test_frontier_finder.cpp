#include <memory>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "exploration_preprocessing/frontier_finder.h"

namespace fast_planner {
namespace {

void ensureRosInitialized() {
  if (!rclcpp::ok()) {
    int argc = 0;
    char **argv = nullptr;
    rclcpp::init(argc, argv);
  }
}

rclcpp::NodeOptions frontierOptions(const bool invalid_radius = false) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("transformer.pose_topic_type", "transform"),
      rclcpp::Parameter("transformer.sensor_pose_topic", "/frontier_test_sensor_pose"),
      rclcpp::Parameter("voxel_mapping.mode", "airsim"),
      rclcpp::Parameter("voxel_mapping.resolution_fine", 0.5),
      rclcpp::Parameter("voxel_mapping.resolution_coarse", 0.5),
      rclcpp::Parameter("map_config.map_size.map_min_x", -3.0),
      rclcpp::Parameter("map_config.map_size.map_min_y", -3.0),
      rclcpp::Parameter("map_config.map_size.map_min_z", 0.0),
      rclcpp::Parameter("map_config.map_size.map_max_x", 3.0),
      rclcpp::Parameter("map_config.map_size.map_max_y", 3.0),
      rclcpp::Parameter("map_config.map_size.map_max_z", 4.0),
      rclcpp::Parameter("frontier_finder.cluster_min", 2),
      rclcpp::Parameter("frontier_finder.cluster_size_xy", 2.0),
      rclcpp::Parameter("frontier_finder.cluster_size_z", 2.0),
      rclcpp::Parameter("frontier_finder.min_candidate_dist", 0.0),
      rclcpp::Parameter("frontier_finder.min_candidate_clearance", 0.0),
      rclcpp::Parameter("frontier_finder.min_candidate_occupied_clearance", 0.0),
      rclcpp::Parameter("frontier_finder.candidate_dphi", 0.7853981633974483),
      rclcpp::Parameter("frontier_finder.candidate_rmin", 0.5),
      rclcpp::Parameter("frontier_finder.candidate_rmax", invalid_radius ? 0.5 : 1.0),
      rclcpp::Parameter("frontier_finder.candidate_rnum", 2),
      rclcpp::Parameter("frontier_finder.down_sample", 1),
      rclcpp::Parameter("frontier_finder.min_visib_num", 0),
      rclcpp::Parameter("frontier_finder.min_view_finish_fraction", 0.1),
      rclcpp::Parameter("frontier_finder.cell_height_clearance", 0.05),
      rclcpp::Parameter("perception_utils.max_dist", 2.0),
      rclcpp::Parameter("perception_utils.vis_dist", 1.0),
  });
  return options;
}

TEST(FrontierFinderTest, DetectsFrontierCandidatesFromTsdfUpdate) {
  ensureRosInitialized();
  auto node = std::make_shared<rclcpp::Node>("frontier_finder_test", frontierOptions());
  auto map_server = std::make_shared<voxel_mapping::MapServer>(*node);
  FrontierFinder finder(*node, map_server);
  finder.setCellHeights({});

  PointCloudType cloud;
  cloud.sensor_origin_ = Eigen::Vector4f(0.0F, 0.0F, 1.0F, 1.0F);
  cloud.sensor_orientation_ = Eigen::Quaternionf::Identity();
  for (float x = -0.5F; x <= 0.5F; x += 0.5F) {
    for (float z = 0.5F; z <= 1.5F; z += 0.5F)
      cloud.push_back(PointType(x, 2.0F, z));
  }
  map_server->getTSDF()->inputPointCloud(cloud);

  finder.searchFrontiers();
  Eigen::Vector3d update_min;
  Eigen::Vector3d update_max;
  finder.getUpdateBBox(update_min, update_max);
  EXPECT_TRUE(update_min.allFinite());
  EXPECT_TRUE(update_max.allFinite());
  EXPECT_TRUE((update_min.array() <= update_max.array()).all());

  finder.computeFrontiersToVisit();
  std::vector<std::vector<Eigen::Vector3d>> active;
  std::vector<std::vector<Eigen::Vector3d>> dormant;
  std::vector<std::vector<Eigen::Vector3d>> tiny;
  finder.getFrontiers(active);
  finder.getDormantFrontiers(dormant);
  finder.getTinyFrontiers(tiny);
  EXPECT_GT(active.size() + dormant.size() + tiny.size(), 0U);
}

TEST(FrontierFinderTest, RejectsDegenerateCandidateRadiusRange) {
  ensureRosInitialized();
  auto node =
      std::make_shared<rclcpp::Node>("frontier_finder_invalid_test", frontierOptions(true));
  auto map_server = std::make_shared<voxel_mapping::MapServer>(*node);
  EXPECT_THROW(FrontierFinder(*node, map_server), std::invalid_argument);
}

} // namespace
} // namespace fast_planner
