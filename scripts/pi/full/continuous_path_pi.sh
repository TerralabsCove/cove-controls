#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"

set +u
source /opt/ros/jazzy/setup.bash
source "$REPO_ROOT/install/setup.bash"
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export CYCLONEDDS_URI="file://$REPO_ROOT/config/pi/simple_assembly_cyclonedds_pi.xml"

WAYPOINT_FILE="${WAYPOINT_FILE:-$REPO_ROOT/scripts/ubuntu/util/waypoint.txt}"
if [[ ! -f "$WAYPOINT_FILE" && -f "$REPO_ROOT/scripts/ubuntu/util/waypoints.txt" ]]; then
  WAYPOINT_FILE="$REPO_ROOT/scripts/ubuntu/util/waypoints.txt"
fi

if [[ ! -f "$WAYPOINT_FILE" ]]; then
  echo "Waypoint file not found: $WAYPOINT_FILE" >&2
  echo "Expected a show_eef_pose.sh capture at scripts/ubuntu/util/waypoint.txt" >&2
  exit 1
fi

ros2 launch simple_assembly_tracking continuous_path.launch.py \
  waypoint_file:="$WAYPOINT_FILE" \
  "$@"
