#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
URDF_PATH="$REPO_ROOT/install/full_assembly/share/full_assembly/urdf/simple_assembly.urdf"

set +u
source /opt/ros/jazzy/setup.bash
source "$REPO_ROOT/install/setup.bash"
set -u

if [[ ! -f "$URDF_PATH" ]]; then
  echo "URDF not found at $URDF_PATH — run 'colcon build --packages-select full_assembly' first." >&2
  exit 1
fi

URDF="$(cat "$URDF_PATH")"

cleanup() {
  kill "${rsp_pid:-}" "${jsp_pid:-}" "${stf_pid:-}" 2>/dev/null || true
  wait "${rsp_pid:-}" "${jsp_pid:-}" "${stf_pid:-}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p "robot_description:=$URDF" &
rsp_pid=$!

ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id root &
stf_pid=$!

ros2 run joint_state_publisher_gui joint_state_publisher_gui \
  --ros-args -p "robot_description:=$URDF" &
jsp_pid=$!

sleep 1
rviz2
