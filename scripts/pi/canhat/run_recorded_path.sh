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

WAYPOINT_FILE="${WAYPOINT_FILE:-$REPO_ROOT/recorded_waypoints/canhat_waypoints.jsonl}"
DEFAULT_WAYPOINT_SEQUENCE="7,6,5,6,7"
WAYPOINT_SEQUENCE="${WAYPOINT_SEQUENCE:-$DEFAULT_WAYPOINT_SEQUENCE}"
MOVE_DURATION="${MOVE_DURATION:-5.0}"
GPIO_CHIP="${GPIO_CHIP:-gpiochip4}"
GPIO_LINE="${GPIO_LINE:-17}"

magnet_args=()
if [[ -n "${MAGNET_ACTIONS:-}" ]]; then
  magnet_args+=(--magnet-actions "$MAGNET_ACTIONS")
elif [[ "$WAYPOINT_SEQUENCE" == "$DEFAULT_WAYPOINT_SEQUENCE" ]]; then
  magnet_args+=(--magnet-actions "on,none,none,none,off")
fi

python3 "$REPO_ROOT/scripts/common/run_recorded_path.py" \
  --waypoint-file "$WAYPOINT_FILE" \
  --sequence "$WAYPOINT_SEQUENCE" \
  "${magnet_args[@]}" \
  --duration "$MOVE_DURATION" \
  --gpio-chip "$GPIO_CHIP" \
  --gpio-line "$GPIO_LINE" \
  "$@"
