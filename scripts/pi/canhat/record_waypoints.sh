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

STAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT="${OUTPUT:-$REPO_ROOT/recorded_waypoints/canhat_waypoints_${STAMP}.jsonl}"
LATEST_LINK="$REPO_ROOT/recorded_waypoints/canhat_waypoints_latest.jsonl"

mkdir -p "$(dirname "$OUTPUT")"
ln -sfn "$OUTPUT" "$LATEST_LINK"
echo "Recording to $OUTPUT"
echo "Latest link: $LATEST_LINK"

python3 "$REPO_ROOT/scripts/common/record_waypoints.py" \
  --output "$OUTPUT" \
  "$@"
