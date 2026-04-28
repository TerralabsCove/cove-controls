#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"

set +u
source /opt/ros/jazzy/setup.bash
source "$REPO_ROOT/install/setup.bash"
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$REPO_ROOT/config/pi/simple_assembly_cyclonedds_pi.xml}"

CAN_INTERFACE="${CAN_INTERFACE:-can1}"
MOTOR_SPEC="${MOTOR_SPEC:-revolute_7_0:DM4340:0x01:0x11}"
LOOP_RATE="${LOOP_RATE:-50.0}"
CONTROL_MODE="${CONTROL_MODE:-velocity}"
AUTO_ENABLE="${AUTO_ENABLE:-false}"
DISABLE_ON_SHUTDOWN="${DISABLE_ON_SHUTDOWN:-true}"

ros2 launch damiao_socketcan_driver damiao_socketcan_launch.py \
  can_interface:="$CAN_INTERFACE" \
  motors:="$MOTOR_SPEC" \
  loop_rate:="$LOOP_RATE" \
  control_mode:="$CONTROL_MODE" \
  auto_enable:="$AUTO_ENABLE" \
  disable_on_shutdown:="$DISABLE_ON_SHUTDOWN" \
  "$@"
