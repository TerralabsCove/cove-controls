#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
URDF_PATH="$REPO_ROOT/install/simple_turret/share/simple_turret/urdf/simple_turret.urdf"

set +u
source /opt/ros/jazzy/setup.bash
source "$REPO_ROOT/install/setup.bash"
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$REPO_ROOT/config/pi/cyclonedds.xml"

killall robot_state_publisher static_transform_publisher rviz2 2>/dev/null || true
sleep 1

URDF="$(cat "$URDF_PATH")"

ros2 run robot_state_publisher robot_state_publisher --ros-args -p "robot_description:=$URDF" &
RSP_PID=$!
sleep 2

ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id root &
STF_PID=$!
sleep 1

ros2 topic pub /joint_states sensor_msgs/msg/JointState \
  '{name: [pan, tilt], position: [0.0, 0.0], velocity: [], effort: []}' \
  --rate 50 &
JSP_PID=$!
sleep 2

rviz2

kill "$RSP_PID" "$STF_PID" "$JSP_PID" 2>/dev/null || true
