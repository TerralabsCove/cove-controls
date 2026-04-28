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

OUTPUT="${OUTPUT:-$REPO_ROOT/recorded_waypoints/canhat_waypoints.jsonl}"

python3 "$REPO_ROOT/scripts/common/record_waypoints.py" \
  --output "$OUTPUT" \
  "$@"
