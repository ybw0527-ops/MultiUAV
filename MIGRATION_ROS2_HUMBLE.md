# FALCON migration to Ubuntu 22.04 and ROS 2 Humble

This branch ports the upstream `ros1-noetic` implementation to a native ROS 2
Humble workspace. The ROS 1 source tree is kept unchanged as a behavioral
reference until each package has a tested ROS 2 replacement.

## Target platform

- Ubuntu 22.04 LTS
- ROS 2 Humble
- GCC/G++ 11
- CMake 3.22 or newer
- NVIDIA RTX 5060 (compute capability 12.0)
- CUDA toolkit with `sm_120` support for the rendering stage

## Migration principles

1. No ROS 1 runtime and no permanent `ros1_bridge` dependency.
2. Preserve message semantics and topic contracts before changing algorithms.
3. Build and test packages in dependency order.
4. Keep the original EPEE simulator first; Gazebo integration is a later layer.
5. Keep planner behavior reproducible before adding multi-UAV communication work.

## Package migration order

### Phase 1: interfaces and build foundation

- `falcon_interfaces`: consolidated ROS 2 messages used by the planner and UAV simulator.
- `falcon_poscmd_to_odom`: first native `rclcpp` execution-chain node.
- Colcon workspace and repeatable build scripts.
- Topic, frame, parameter, and QoS contract documentation.

### Phase 2: algorithm and mapping libraries

- `exploration_utils`
- `voxel_mapping`
- `pathfinding`
- `trajectory`
- `fast_planner`

Pure algorithm code should remain independent of `rclcpp` where practical.
ROS publishers, subscribers, clocks, parameters, and visualization are placed in
thin node wrappers.

### Phase 3: exploration pipeline

- `exploration_preprocessing`
- `exploration_manager`
- ROS 2 launch and parameter files
- Regression checks on the six upstream maps

### Phase 4: EPEE simulator

- `map_render`
- `pointcloud_render`
- `mesh_render`
- odometry and trajectory execution nodes
- SO3 simulator and controller

CUDA is intentionally deferred until the CPU-side planner and interfaces build.

### Phase 5: research extensions

- Damaged-building STL/PLY/PCD scenarios
- Multi-UAV map and frontier sharing
- Communication-quality and occlusion maps
- Connectivity-aware task allocation and relay behavior
- Surface inspection and reconstruction metrics

## Current status

Completed milestones:

- `falcon_interfaces` builds all 18 migrated message types.
- `falcon_poscmd_to_odom` builds and passes a live ROS 2 topic smoke test.
- `falcon_exploration_utils` builds as a native ament library with the original
  ray casting, LKH-TSP, SOP, system-information, and Kindr math code.
- `falcon_voxel_mapping` builds the ROS-independent TSDF, ESDF, occupancy-grid,
  coordinate-conversion, map I/O, and point-cloud integration core.
- Perception parameters now use ROS 2 parameter names and support both pinhole
  cameras and 360-degree lidar models.
- The pose transformer uses `rclcpp::Time`, bounded message storage, and
  timestamp interpolation.
- Unit tests cover pinhole FOV, omnidirectional lidar elevation, and transform
  interpolation.
- Voxel-map tests cover index/address conversion, boundary-safe inflation,
  weighted TSDF fusion, and Euclidean ESDF distance propagation.

The next target is the ROS 2 `MapServer` wrapper for depth images, point clouds,
map visualization, and map persistence. Pathfinding and trajectory generation
follow after that wrapper is operational.

Validation commands:

```bash
./scripts/build_ros2_humble.bash --packages-select falcon_exploration_utils
cd ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select falcon_exploration_utils
colcon test-result --verbose
```

## Ubuntu 22.04 compatibility notes

- GCC 10 and newer default to `-fno-common`; the vendored C implementation of
  LKH requires `-fcommon` for its historical global-variable layout.
- The vendored Kindr snapshot only uses glog-style assertion macros. A small
  local compatibility header provides those checks, avoiding an extra system
  package and improving offline reproducibility.
- Legacy ROS 1 `minkindr_conversions` headers are intentionally not exported by
  the ROS 2 package. The transformer performs native ROS 2 message conversion.
- The upstream ESDF implementation used variable-length stack arrays, a GNU
  extension rather than standard C++17. The ROS 2 package uses bounded vectors.
- Occupancy inflation now rejects out-of-map neighbor indices before computing
  linear addresses, preventing boundary voxels from wrapping across the map.

## Known upstream issues to preserve or address explicitly

- The upstream implementation is ROS 1 Noetic and uses catkin/nodelets.
- Velocity and acceleration limits are applied per axis in the upstream B-spline optimizer.
- The lightweight simulator is not Gazebo and uses rendered depth plus synthetic odometry.
- The repository does not currently provide a root license file; redistribution terms
  must be confirmed before publishing a derivative repository.
