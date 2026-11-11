# ROS 2 map and depth rendering

`falcon_map_render` provides the CPU simulation baseline for FALCON's six original
maps. PCD/PLY inputs use a projected point-depth buffer and configurable point
inflation. STL/OBJ inputs use two-sided ray/triangle intersection accelerated by
a binary bounding-volume hierarchy. Mesh depth is exact triangle depth; point
depth approximates a surface using square pixel splats.

The original CUDA point renderer and Open3D/Filament mesh renderer are not built
by this package. The development environment has an RTX 5060 but no CUDA compiler
or Open3D development package. GPU acceleration, RGB rendering, and comparisons
against the original GPU images remain future work. This CPU baseline requires
PCL and Assimp and defaults to a Release build unless a build type is selected.

From the repository root:

```bash
./scripts/build_ros2_humble.bash --packages-up-to falcon_map_render
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch falcon_map_render simulation.launch.py map_name:=complex_office
```

The simulation launch includes the exploration manager/map server, trajectory
server, command-to-odometry simulator, and renderer. `map_name` selects one of
`complex_office`, `classical_office`, `duplex_office`, `octa_maze`, `darpa_tunnel`,
or `power_plant`. Assets are installed from the original repository resource
directory; they are not duplicated in the new package's source tree.

`map_params_file` can override the selected map configuration and `sensor_params_file`
can override camera and simulation settings. The default camera is the upstream
640x480 model from the simulator robot configuration; CPU throughput depends on
scene, image size, and point inflation. Lower image resolution can be selected
through a sensor configuration file. The renderer by itself defaults to 160x120.
Use `exploration_manager.auto_start: false` in a supplied parameter file to wait
for the upstream start trigger instead of automatically exploring.

The renderer subscribes to `/uav_simulator/odometry` and publishes:

- `/uav_simulator/depth_image`: `sensor_msgs/msg/Image`, `32FC1`, optical z depth
  in meters, in the camera frame.
- `/uav_simulator/sensor_pose`: `geometry_msgs/msg/TransformStamped`, world to
  optical camera, with the same odometry timestamp as the depth image.
- `/uav_simulator/camera_info`: calibrated pinhole intrinsics and dimensions.

Missing depth is `max_depth + 1` meters. This allows the mapping raycaster to mark
free space out to its range limit without creating an obstacle on the range
boundary. The camera depth range is limited to 60 m so the existing map server's
millimeter `uint16` conversion can represent both hits and missing samples.
Configure `transformer.pose_topic_type: transform` when connecting this renderer
to a standalone MapServer. The simulation launch already sets it.

Coordinates use `T_w_c = T_w_b * T_b_c`. Input model coordinates are divided by
`map_config.scale` and transformed by the inverse of `map_config.T_m_w` into
world coordinates. Transform parameters contain 16 row-major numbers and must
be rigid transforms. The default optical frame has z forward, x right, y down;
the body frame has x forward, y left, z up. Invalid camera settings, empty maps,
nonfinite points, and invalid odometry are handled explicitly.

Validation from the sourced workspace:

```bash
cd ros2_ws
colcon test --packages-select falcon_map_render
colcon test-result --verbose
python3 src/falcon_map_render/test/sensor_regression.py
python3 src/falcon_map_render/test/pipeline_smoke.py
```

Unit tests cover occlusion, axial depth, camera motion, near/far clipping, point
projection, input rejection, BVH traversal, and loading the six original maps.
The sensor regression checks actual depth hits, paired pose timestamps, and
nonzero fused coverage volume for each map. The full graph test additionally
requires a nonempty frontier output. These bounded checks do not establish
complete autonomous exploration, collision-free mission completion, or six-map
coverage targets.
