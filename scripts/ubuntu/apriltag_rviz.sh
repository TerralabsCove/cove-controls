#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

set +u
source /opt/ros/jazzy/setup.bash
source "$REPO_ROOT/install/setup.bash"
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export CYCLONEDDS_URI="file://$REPO_ROOT/config/ubuntu/simple_assembly_cyclonedds_ubuntu.xml"

cleanup() {
  kill "${rviz_pid:-}" "${image_pid:-}" 2>/dev/null || true
  wait "${rviz_pid:-}" "${image_pid:-}" 2>/dev/null || true
}

trap cleanup EXIT INT TERM

ros2 launch simple_assembly_moveit_config moveit_rviz.launch.py \
  rviz_config:="$REPO_ROOT/src/cove_vision/config/apriltag_robot.rviz" \
  "$@" &
rviz_pid=$!

ros2 run rqt_image_view rqt_image_view /camera/image_raw &
image_pid=$!

wait "$rviz_pid" "$image_pid"
