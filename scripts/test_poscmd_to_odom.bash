#!/usr/bin/env bash
set -eo pipefail

repository_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)

source /opt/ros/humble/setup.bash
source "${repository_root}/ros2_ws/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=1
set -u

node_log=$(mktemp)
node_pid=""

cleanup() {
  if [[ -n "${node_pid}" ]] && kill -0 "${node_pid}" 2>/dev/null; then
    kill "${node_pid}"
    wait "${node_pid}" 2>/dev/null || true
  fi
  rm -f -- "${node_log}"
}
trap cleanup EXIT

ros2 run falcon_poscmd_to_odom poscmd_to_odom_node >"${node_log}" 2>&1 &
node_pid=$!
sleep 1
if ! timeout 15 python3 "${repository_root}/scripts/poscmd_to_odom_smoke.py"; then
  echo "--- poscmd_to_odom node log ---" >&2
  cat "${node_log}" >&2
  exit 1
fi
