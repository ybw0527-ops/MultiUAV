#!/usr/bin/env bash
set -eo pipefail

repository_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
workspace="${repository_root}/ros2_ws"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble was not found at /opt/ros/humble" >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
set -u
cd "${workspace}"

colcon build \
  --symlink-install \
  --event-handlers console_direct+ \
  "$@"
