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

VEL="${VEL:-0.4}"
DURATION="${DURATION:-2.0}"
DISABLE_AFTER="${DISABLE_AFTER:-true}"

ros2 service call /damiao_socketcan_driver/enable_motors std_srvs/srv/Trigger
ros2 topic pub --once /cmd_vel_motors std_msgs/msg/Float64MultiArray "{data: [$VEL]}"
sleep "$DURATION"
ros2 topic pub --once /cmd_vel_motors std_msgs/msg/Float64MultiArray "{data: [0.0]}"

if [[ "$DISABLE_AFTER" == "true" || "$DISABLE_AFTER" == "1" ]]; then
  ros2 service call /damiao_socketcan_driver/disable_motors std_srvs/srv/Trigger
fi
