# ROS 2 exploration manager

The package connects the upstream frontier search, hierarchical coverage planning,
LKH/SOP ordering, local viewpoint refinement, position/yaw trajectory planning,
and exploration state machine to the native ROS 2 libraries.

Build from the repository root:

```bash
./scripts/build_ros2_humble.bash --packages-up-to falcon_exploration_manager
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch falcon_exploration_manager exploration.launch.py
```

The map server is owned by the exploration node; do not launch a second map
server for this graph. The default launch starts the exploration node and
trajectory server. Supply sensor data and odometry from an external simulator
or recorded data. `simulate_odometry:=true` adds the migrated command-to-odometry
node; it does not render a map or generate sensor measurements.

Inputs default to `/uav_simulator/odometry`, `/uav_simulator/sensor_pose`
(`nav_msgs/msg/Odometry`), `/uav_simulator/depth_image`, and
`/uav_simulator/pointcloud`. Use `odometry_topic`, `sensor_pose_topic`,
`sensor_pose_type` (`odometry`, `pose`, or `transform`), `depth_image_topic`, and
`pointcloud_topic` to adapt these. A one-shot global cloud can also be supplied
to `/voxel_mapping/global_map`.

Use `map_params_file:=/absolute/path/to/params.yaml` for a ROS 2 map/configuration
file. It is applied after the packaged defaults, so it can also override planner
settings, `exploration_manager.auto_start`, and `traj_server.perform_initial_motion`.
The six upstream map configuration files are converted to ROS 2 parameter format;
their 4x4 matrices use flat row-major arrays. Map assets and rendering are not
provided by this package. These six environments have not yet received end-to-end
exploration regression tests.

The FSM waits for valid in-map odometry, then waits two seconds before accepting
frontiers. With `auto_start: false`, publish a `geometry_msgs/msg/PoseStamped` on
`/move_base_simple/goal` after frontiers exist to begin exploration. Its position
is not a navigation destination; this topic is the upstream start trigger.
`/planning/bspline` and `/planning/replan` connect the manager to the trajectory
server, which publishes `/planning/pos_cmd`.

Callbacks share the default mutually exclusive callback group, and the executable
uses a single-threaded executor. Visualization runs synchronously. The legacy
PathCostEvaluator and LKH/SOP solvers retain process-global state, so run separate
processes for independent agents. LKH files use a unique temporary directory per
manager, and generated files are removed on normal destruction.

Validation:

```bash
cd ros2_ws
colcon test --packages-select falcon_exploration_manager
colcon test-result --verbose
python3 src/falcon_exploration_manager/test/smoke_launch.py
```

The smoke test runs in ROS domain 177, launches all three nodes, checks command
and odometry traffic plus planner/sensor subscription wiring, and shuts down its
own process group. This validates graph startup and transport, not autonomous
exploration of a rendered environment.
