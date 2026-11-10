#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include <Eigen/Eigen>

#include "voxel_mapping/esdf.h"
#include "voxel_mapping/occupancy_grid.h"
#include "voxel_mapping/tsdf.h"

namespace {

voxel_mapping::MapConfig makeConfig(const Eigen::Vector3i &size) {
  voxel_mapping::MapConfig config;
  config.resolution_ = 1.0;
  config.resolution_inv_ = 1.0;
  config.resolution_fine_ = 1.0;
  config.resolution_coarse_ = 1.0;
  config.map_size_idx_ = size;
  config.map_size_ = size.cast<double>();
  config.map_min_ = Position::Zero();
  config.map_max_ = config.map_size_;
  config.box_min_ = config.map_min_;
  config.box_max_ = config.map_max_;
  config.box_min_idx_ = VoxelIndex::Zero();
  config.box_max_idx_ = size - VoxelIndex::Ones();
  config.vbox_min_ = config.box_min_;
  config.vbox_max_ = config.box_max_;
  config.vbox_min_idx_ = config.box_min_idx_;
  config.vbox_max_idx_ = config.box_max_idx_;
  return config;
}

TEST(MapBaseTest, ConvertsBetweenPositionIndexAndAddress) {
  voxel_mapping::OccupancyGrid grid;
  grid.map_config_ = makeConfig(Eigen::Vector3i(4, 3, 2));
  grid.initMapData();

  const Position position(1.2, 2.2, 0.2);
  const VoxelIndex expected_index(1, 2, 0);
  EXPECT_EQ(grid.positionToIndex(position), expected_index);

  const VoxelAddress address = grid.indexToAddress(expected_index);
  EXPECT_EQ(address, 10);
  EXPECT_EQ(grid.addressToIndex(address), expected_index);
  EXPECT_TRUE(grid.isInMap(expected_index));
  EXPECT_FALSE(grid.isInMap(VoxelIndex(-1, 0, 0)));
}

TEST(OccupancyGridTest, InflationDoesNotWrapAcrossMapBoundaries) {
  voxel_mapping::OccupancyGrid grid;
  grid.map_config_ = makeConfig(Eigen::Vector3i(4, 3, 2));
  grid.initMapData();

  PointCloudType::Ptr cloud(new PointCloudType);
  cloud->push_back(PointType(1.5F, 0.5F, 0.5F));
  grid.loadMapFromPCL(cloud, voxel_mapping::OccupancyType::OCCUPIED,
                      Eigen::Vector3i(0, 1, 1));

  EXPECT_EQ(grid.getVoxel(VoxelIndex(1, 0, 0)).value,
            voxel_mapping::OccupancyType::OCCUPIED);
  EXPECT_EQ(grid.getVoxel(VoxelIndex(1, 1, 1)).value,
            voxel_mapping::OccupancyType::OCCUPIED);
  EXPECT_EQ(grid.getVoxel(VoxelIndex(0, 2, 1)).value,
            voxel_mapping::OccupancyType::UNKNOWN);
}

TEST(TSDFTest, FusesWeightedSignedDistances) {
  voxel_mapping::TSDF tsdf;
  tsdf.map_config_ = makeConfig(Eigen::Vector3i(3, 3, 3));
  tsdf.config_.truncated_dist_ = 1.0;
  tsdf.config_.epsilon_ = 1e-4;
  tsdf.initMapData();

  const VoxelAddress address = tsdf.indexToAddress(VoxelIndex(1, 1, 1));
  tsdf.updateTSDFVoxel(address, 0.5, 1.0);
  tsdf.updateTSDFVoxel(address, -0.5, 1.0);

  const auto voxel = tsdf.getVoxel(address);
  EXPECT_NEAR(voxel.value, 0.0, 1e-12);
  EXPECT_NEAR(voxel.weight, 2.0, 1e-12);
}

TEST(ESDFTest, ComputesDistanceFromOccupiedVoxel) {
  const auto config = makeConfig(Eigen::Vector3i(5, 5, 5));
  auto occupancy = std::make_shared<voxel_mapping::OccupancyGrid>();
  occupancy->map_config_ = config;
  occupancy->initMapData();
  ASSERT_TRUE(occupancy->setOccupancy(
      occupancy->indexToAddress(VoxelIndex(2, 2, 2)),
      voxel_mapping::OccupancyType::OCCUPIED));

  voxel_mapping::ESDF esdf;
  esdf.map_config_ = config;
  esdf.setOccupancyGrid(occupancy);
  esdf.initMapData();
  esdf.updateLocalESDF(VoxelIndex::Zero(), VoxelIndex(4, 4, 4));

  EXPECT_NEAR(esdf.getDistance(VoxelIndex(2, 2, 2)), 0.0, 1e-12);
  EXPECT_NEAR(esdf.getDistance(VoxelIndex(3, 2, 2)), 1.0, 1e-12);
  EXPECT_NEAR(esdf.getDistance(VoxelIndex(3, 3, 2)), std::sqrt(2.0), 1e-12);
}

}  // namespace
