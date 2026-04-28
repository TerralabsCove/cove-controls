#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"

set +u
source /opt/ros/jazzy/setup.bash
source "$REPO_ROOT/install/setup.bash"
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export CYCLONEDDS_URI="file://$REPO_ROOT/config/ubuntu/simple_assembly_cyclonedds_ubuntu.xml"

source "$REPO_ROOT/scripts/ubuntu/canhat/warehouse_db_common.sh"

cleanup() {
  stop_moveit_warehouse
}

trap cleanup EXIT INT TERM

start_moveit_warehouse simple_assembly_tracking_moveit_config

ros2 launch simple_assembly_tracking_moveit_config moveit_rviz_canhat.launch.py "$@"
