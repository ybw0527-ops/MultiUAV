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

## Current milestone

The first milestone is complete when `falcon_interfaces` and
`falcon_poscmd_to_odom` build on a clean ROS 2 Humble environment, generated
message types can be imported, and a position command produces an odometry
message through the native ROS 2 node.

## Known upstream issues to preserve or address explicitly

- The upstream implementation is ROS 1 Noetic and uses catkin/nodelets.
- Velocity and acceleration limits are applied per axis in the upstream B-spline optimizer.
- The lightweight simulator is not Gazebo and uses rendered depth plus synthetic odometry.
- The repository does not currently provide a root license file; redistribution terms
  must be confirmed before publishing a derivative repository.
