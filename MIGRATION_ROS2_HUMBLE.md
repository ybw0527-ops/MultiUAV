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
- `falcon_voxel_mapping` builds both the TSDF/ESDF/occupancy-grid core and the
  native ROS 2 `map_server_node` runtime.
- `MapServer` accepts depth images, local point clouds, and one-shot global maps
  with ROS 2 sensor-data QoS; it publishes map layers, slices, poses, coverage,
  and debug markers and exposes the map-save service.
- Map bounds, camera intrinsics, visualization controls, and map output path are
  provided by a ROS 2 YAML file. The hard-coded upstream user path was removed.
- Perception parameters now use ROS 2 parameter names and support both pinhole
  cameras and 360-degree lidar models.
- The pose transformer uses `rclcpp::Time`, bounded message storage, and
  timestamp interpolation.
- Unit tests cover pinhole FOV, omnidirectional lidar elevation, and transform
  interpolation.
- Voxel-map tests cover index/address conversion, boundary-safe inflation,
  weighted TSDF fusion, and Euclidean ESDF distance propagation.
- A `MapServer` test covers ROS 2 parameter construction, map geometry, box
  queries, and point-cloud coordinate transformation.
- The runtime node was smoke-tested with the migrated YAML at 0.1 m resolution
  on a 20 x 20 x 5 m map.
- `falcon_pathfinding` builds the original 3-D A* search profiles and path-cost
  evaluator as an ament library with ROS 2 parameters.
- Pathfinding tests cover unobstructed paths, obstacle avoidance, safe node-pool
  exhaustion, and translation/yaw travel-time lower bounds.
- `falcon_trajectory` builds the polynomial trajectory, non-uniform B-spline,
  NLopt B-spline optimizer, and planning-visualization code as a native ament
  library.
- Trajectory tests cover polynomial endpoint constraints, B-spline waypoint and
  derivative interpolation, ROS 2 optimizer parameters, a real NLopt yaw
  optimization, and ROS 2 visualization-marker metadata.
- Optimizer inputs, physical limits, knot intervals, sampling resolution, and
  visualization time steps are validated. Paired visualization lists are
  bounded by their common length and point-cloud display scale cannot collapse
  to zero.
- The trajectory milestone brought the workspace to 20 passing tests.
- `falcon_fast_planner` now integrates the migrated map server, A* search,
  polynomial/B-spline trajectory generation, yaw optimization, and trajectory
  collision checking behind a native ROS 2 manager library.
- Planner-manager tests cover short position and yaw trajectories, invalid tour
  rejection, collision checks, and a fully optimized four-point tour.
- Multi-segment timing is now initialized deterministically. The upstream code
  left intermediate segment times uninitialized for tours containing more than
  three points.
- The planner-manager milestone brought the workspace to 24 passing tests.
- `trajectory_server_node` accepts migrated `falcon_interfaces/Bspline`
  messages, samples position/velocity/acceleration/jerk/yaw commands at 100 Hz,
  and publishes native ROS 2 `PositionCommand` messages and debug markers.
- Initial positioning is a non-blocking timer state machine. B-spline messages
  are rejected when IDs, dimensions, knots, timestamps, or finite-value checks
  are invalid, and command sampling is mutex-protected for multithreaded
  executors.
- Trajectory-server tests cover normal command sampling, final-state holding,
  pre-start behavior, malformed messages, and misordered trajectory IDs.
- The trajectory-server milestone brought the workspace to 27 passing tests.
- `falcon_exploration_preprocessing` now provides a ROS-independent,
  C++17 connectivity-graph library as the first exploration-manager dependency.
- Connectivity tests cover hop-based BFS, minimum-cost graph search, node and
  edge removal, unknown connected-component classification, and lossless graph
  persistence.
- Graph loading no longer doubles every saved edge cost, malformed files do not
  destroy the current graph, and references to missing nodes are rejected.
- The connectivity-graph milestone brought the workspace to 31 passing tests.
- `FrontierFinder` now consumes the migrated TSDF/occupancy map, detects and
  clusters free/unknown boundaries, samples viewpoints with the ROS 2 sensor
  model, and classifies active, dormant, and tiny frontiers.
- A synthetic point-cloud regression test exercises the real TSDF update and
  frontier-search path. Degenerate sampling radii and other invalid parameters
  are rejected before the search begins.
- Empty hierarchical cell-height input is handled safely, and low-resolution
  cluster thresholds are clamped instead of collapsing to zero.
- `hierarchical_grid` now builds in the ROS 2 preprocessing library with native
  parameters, typed marker/point-cloud publishers, and monotonic profiling.
- Grid coordinate queries reject out-of-bounds positions before flattening IDs;
  bounding-box updates clamp each axis independently to prevent row wrapping.
- Frontier inputs and grid parameters are validated before changing grid state.
  Low-resolution component thresholds are bounded by the available sample count,
  and CCL occupancy queries reject out-of-map indices.
- Regression tests cover 2D grid bounds, frontier assignment, unknown-map CCL
  updates, finite cost matrices, and invalid configuration. The 3D path is ported
  but has not yet received a dedicated regression test.
- The hierarchical-grid milestone reported 38 passing checks with no failures
  (colcon's aggregate includes both GoogleTest cases and CTest entries).

- `falcon_exploration_manager` now integrates hierarchical coverage planning,
  frontier refinement, LKH/SOP ordering, and position/yaw trajectory generation.
- The exploration state machine uses ROS 2 timers, subscriptions, typed publishers,
  and ROS clock timestamps. Visualization is synchronous within the default
  mutually exclusive callback group; detached threads were removed.
- LKH files use a unique writable temporary directory rather than the package
  installation. Invalid odometry and replan timing are rejected, and planning
  computation profiling uses a steady clock.
- The exploration launch graph starts the exploration node (including its map
  server), trajectory server, and optional simulated odometry node. External
  odometry, sensor pose, depth, and point-cloud topics are configurable.
- Manager regressions exercise real LKH ordering, position/yaw motion generation,
  and FSM odometry/trigger prerequisites. A live isolated-domain smoke check
  verifies node startup, command/odometry traffic, and topic subscription wiring.
- Trajectory optimization now reserves at least half a voxel for ESDF interpolation
  instead of a fixed 0.1 m. The manager's 0.5 m-map trajectory integration test
  exposed the insufficient upstream margin.
- The manager milestone reports 42 passing colcon checks across nine packages,
  with no errors, failures, or skips. All migrated map/robot YAML files load via
  native ROS 2 parameter parsing, and the three-node launch smoke passes.

- `falcon_map_render` provides a native CPU depth-rendering baseline: point-cloud
  depth projection for PCD/PLY and BVH-accelerated ray/triangle intersection for
  STL/OBJ. It reuses the original map assets without duplicating them in Git.
- The renderer publishes calibrated float depth, camera intrinsics, and an optical
  camera transform with matching odometry timestamps. Map scale and both map/world
  and body/camera transforms are validated and applied explicitly.
- The simulation launch now connects exploration, trajectory commands, simulated
  odometry, depth rendering, map fusion, and frontier detection. Camera parameter
  files are shared by the renderer and mapping node.
- Sensor regressions exercise all six original maps, checking actual depth hits,
  paired camera poses, and nonzero fused map volume. A bounded full-graph check
  additionally verifies nonempty frontier output on `complex_office`.
- `classical_office`'s integer-valued box bound was corrected to a double to match
  native ROS 2 typed parameter declarations. Large-cloud rendering uses a Release
  default build; tests isolate each map in its own process and ROS domain.
- CUDA and Open3D development tools are absent in the current environment. The
  original GPU renderers and RGB output are deferred; CPU rendering is not claimed
  to be pixel-identical or performance-equivalent to the original GPU backends.
- The rendering milestone reports 47 passing colcon checks across ten packages.
  Six-map depth/fusion checks and the full simulation graph are additional live
  integration checks, not extra cases counted in that aggregate.

The next targets are bounded autonomous exploration runs, full-mission coverage
and collision checks across the six environments, and later GPU acceleration.
Upstream currently implements one hierarchical-grid level only; the ROS 2 port
rejects other level counts. The new simulation graph uses command-to-odometry
execution; the SO3 controller/dynamics simulator is still unmigrated.

Validation commands:

```bash
./scripts/build_ros2_humble.bash --packages-select falcon_voxel_mapping
cd ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select falcon_voxel_mapping
colcon test --packages-select falcon_pathfinding
colcon test --packages-select falcon_trajectory
colcon test --packages-select falcon_fast_planner
colcon test --packages-select falcon_exploration_preprocessing
colcon test-result --verbose

ros2 run falcon_voxel_mapping map_server_node --ros-args \
  --params-file src/falcon_voxel_mapping/config/voxel_mapping.yaml

ros2 launch falcon_voxel_mapping map_server.launch.py

ros2 run falcon_fast_planner trajectory_server_node --ros-args \
  --params-file src/falcon_fast_planner/config/fast_planner.yaml
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
- The upstream map callbacks launched detached worker threads that captured the
  server object. The initial ROS 2 baseline performs deterministic, mutex-guarded
  fusion inside the callback so shutdown cannot race a destroyed `MapServer`.
  Controlled callback-group parallelism can be added after end-to-end profiling.
- A* search budgets use `std::chrono::steady_clock` instead of ROS time, so a
  paused simulator clock cannot stall the search and a clock reset cannot end it
  prematurely.
- A* now validates positive search profiles and checks node-pool capacity before
  indexing the pool. Resource exhaustion returns `NO_PATH` with a warning.
- Ubuntu 22.04 provides NLopt 2.7.1. Its CMake package may export
  `NLopt::nlopt_cxx` rather than `NLopt::nlopt`; the trajectory build accepts
  either target name.
- The B-spline optimizer now initializes every cost weight and result field and
  preserves a valid initial solution before invoking NLopt. This removes
  undefined behavior present when optional upstream costs were disabled.
- The planner validates finite, distinct tour points and nonnegative time lower
  bounds before trajectory construction. Timing and optimization profiling use
  a monotonic clock so simulation-time changes do not corrupt measurements.
- B-spline evaluation is const-correct and permits the mathematically valid
  zero-degree spline produced by taking the third derivative of a cubic
  trajectory. Negative-degree splines remain invalid.

## Known upstream issues to preserve or address explicitly

- The upstream implementation is ROS 1 Noetic and uses catkin/nodelets.
- Velocity and acceleration limits are applied per axis in the upstream B-spline optimizer.
- The lightweight simulator is not Gazebo and uses rendered depth plus synthetic odometry.
- The repository does not currently provide a root license file; redistribution terms
  must be confirmed before publishing a derivative repository.
- Direct ESDF interpolation queries still require an interpolation cube within
  the map. Optimization bounds now reserve a resolution-dependent half-voxel
  margin, but other callers must still respect the map boundary.
