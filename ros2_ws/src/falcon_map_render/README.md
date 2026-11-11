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

The initial baseline is `test/autonomous_run_summary.json`: all six 60-second
runs passed, with geometry clearance lower bounds of 0.284-0.707 m. The complete
workspace reported 49 passing colcon checks. These are observed runs, not exact
numerical expectations for later runs.

The newer `test/linear_limits_run_summary.json` records 53 passing workspace
checks, six 60-second runs with hard linear-motion limits, and a 120-second power
plant run. The isolated native planner/server suites also passed 20 repetitions
(180 case executions). One earlier shared-domain parallel run exited with SIGSEGV;
its cause was not captured and did not reproduce during debugger or isolated
native repetitions. That lifecycle observation remains recorded for follow-up.
ROS planner/server gtests now use Humble's native isolated-domain runner.

```bash
cd ros2_ws
colcon test --packages-select falcon_map_render
colcon test-result --verbose
python3 src/falcon_map_render/test/sensor_regression.py
python3 src/falcon_map_render/test/pipeline_smoke.py
python3 src/falcon_map_render/test/autonomous_regression.py --map all --duration 60
```

Unit tests cover occlusion, axial depth, camera motion, near/far clipping, point
projection, input rejection, BVH traversal, nearest geometry distance, and loading
the six original maps.
The sensor regression checks actual depth hits, paired pose timestamps, and
nonzero fused coverage volume for each map. The full graph test additionally
requires a nonempty frontier output.

The autonomous regression starts the complete graph with automatic exploration.
It publishes a shared `/clock` from monotonic elapsed time and gives each map a
fresh process and ROS domain. It requires at least two trajectories, more than
one meter of motion after the first plan, increased known volume, finite flight
commands, no position jump larger than `0.5 + 3*dt` meters, and no node failure or
rejected trajectory. It also requires both the received spline's continuous
derivative bounds and the executed commands to stay within 2 m/s and 2 m/s²,
using Euclidean vector norms. A run must keep moving during its final 15 seconds unless
the FSM has announced completion. It saves logs, sampled positions, and JSON metrics under
`/tmp/falcon_autonomous` (change with `--output`). `fsm_finished` records whether
the FSM announced completion; a bounded run can pass while exploration continues.
The announcement means no active frontier was detected at that point. Dormant
frontiers and later rediscovery can still exist, so this boolean is not a
complete-coverage certificate or a claim about the final frontier list.
The test uses a 160x120 camera, 0.25 m mapping, and 0.5 m coarse mapping. Power plant
uses the original 0.1/0.2 m resolutions and all pixels because the coarse setting
could not generate a usable initial viewpoint in that environment. Its sensor
runs at 3 Hz to match fine-map CPU fusion throughput; the other maps use 10 Hz.

The standalone `check_clearance` executable checks the recorded path against
original world-transformed mesh triangles or point samples, independently of the
fused occupancy map. It checks a 0.2 m vehicle radius, samples each polyline segment
at intervals no larger than 0.025 m, and subtracts half the interval to give a
conservative continuous-polyline clearance bound. Mesh queries include triangle
interiors, edges, and vertices. Point-cloud distances apply to the supplied point
samples; missing geometry in a sparse cloud cannot be certified. Checks begin at
the first exploration trajectory and exclude the initial positioning maneuver.

Known volume is measured within the task box in cubic meters. The reported
`known_task_box_fraction` divides it by box volume, including inaccessible solid
space; it is not a percentage of reachable free space. These tests use ideal
command-to-odometry execution and do not establish SO3/dynamics tracking, strict
dynamic feasibility, or complete six-map mission coverage targets.
The optimizer still uses soft costs. The planner now retimes cubic curves while
preserving the input position, velocity, and acceleration, fixes the goal at rest,
and checks derivative control-point norms. By the B-spline convex-hull property,
these bound speed and acceleration over the entire curve. The backup
spline is checked too; failed feasibility returns to the FSM for stopping and
replanning. When both the fitted path and the optimized curve are feasible, the
shorter-duration candidate is tried first and the other is retained for the FSM's
collision-checked fallback. The trajectory server independently rejects any incoming position
spline whose bounds exceed the configured limits, keeping its previous valid
trajectory. This does not establish roll/pitch, jerk/thrust limits, physical
translation braking during emergency stops, or SO3/dynamics tracking.

Yaw planning reserves a constructive turning duration before position planning.
It preserves the initial yaw/rate/acceleration and ends at rest. After restoring
these exact states to the optimizer output, the planner checks the full yaw
derivative control hulls and falls back to a quintic-shaped cubic curve when
needed. An infeasible yaw plan is rejected, including in the FSM collision
fallback; yaw is never independently stretched away from the position clock.
The trajectory server independently enforces `max_yaw_velocity` and
`max_yaw_acceleration` (both default pi/2, in rad/s and rad/s²) and terminal rest.
Explicit STOP/FINISH messages decelerate yaw at the angular acceleration limit;
repeated messages do not restart the deceleration. Translation still holds
immediately on these messages and is not a physical emergency-braking model.
The FSM waits for fresh translational and angular rest before a static retry.

The command-to-odometry adapter now differentiates its flatness attitude to
publish body angular velocity. Upstream placed linear acceleration in that field;
no ROS 2 consumer should depend on that legacy encoding. A finite-difference
attitude test independently checks the angular velocity mapping. The live harness
checks whole-curve yaw bounds, yaw command continuity modulo 2 pi, and finite-
difference command yaw acceleration, including replans and stops. These remain
kinematic checks and do not certify roll/pitch actuator limits or tracked flight.

The observed yaw-limit baseline is recorded in `test/yaw_limits_run_summary.json`:
60 passing colcon checks and six passing 60-second live runs. The runs contain no
position/yaw command jumps or unsafe source-geometry samples, with a minimum
geometry clearance lower bound of 0.371 m. These are bounded-duration observations,
not complete exploration missions or exact numerical golden expectations.

Online collision checks require the center to be in known free space and test a
vehicle sphere against occupied voxel boxes. `fast_planner.collision_clearance`
sets the sphere radius (default 0.2 m); half a sampling interval times the speed
limit is added to cover motion between 0.02 s checks. Both candidate curves are
checked over their full duration before selection and checked again by the FSM.
Unknown neighboring volume and unobserved obstacles remain uncertified; the
independent original-geometry regression is still required. The trusted handoff
body itself is treated as known free when its sensor's near-range blind spot
leaves the center unobserved. This exception is limited to the configured vehicle
radius around that handoff and never overrides occupied center or neighbor cells.
Failed plans refresh frontier observations before retrying stale viewpoints.
Shutdown is also checked. The harness signals the launch process once and lets
it stop its nodes; a node crash during teardown fails the run.
