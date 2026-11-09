# FALCON ROS 2 Humble workspace

This workspace contains the native ROS 2 port. The original ROS 1 packages at
the repository root remain available only as migration references.

Build from the repository root:

```bash
./scripts/build_ros2_humble.bash
```

After a successful build:

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 interface show falcon_interfaces/msg/Bspline
```
