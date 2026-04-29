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

# Edit this sequence. Entries can be RViz Stored State names or SQLite row IDs.
# Example:
# STORED_STATE_SEQUENCE=(home grabbing_bottle lift_drink to_cup pour1)
STORED_STATE_SEQUENCE=(
)

WAREHOUSE_DB="${WAREHOUSE_DB:-$HOME/.ros/simple_assembly_tracking_warehouse.sqlite}"
UBUNTU_WAREHOUSE_DB="${UBUNTU_WAREHOUSE_DB:-parallels@100.86.104.75:/home/parallels/.ros/simple_assembly_tracking_warehouse.sqlite}"
SYNC_WAREHOUSE_DB="${SYNC_WAREHOUSE_DB:-0}"
MOVE_DURATION="${MOVE_DURATION:-5.0}"
ROBOT_ID="${ROBOT_ID:-simple_assembly_tracking}"

if [[ "$SYNC_WAREHOUSE_DB" == "1" || "$SYNC_WAREHOUSE_DB" == "true" ]]; then
  mkdir -p "$(dirname "$WAREHOUSE_DB")"
  scp "$UBUNTU_WAREHOUSE_DB" "$WAREHOUSE_DB"
fi

sequence_csv="${STORED_STATE_SEQUENCE_CSV:-}"
if [[ -z "$sequence_csv" && "${#STORED_STATE_SEQUENCE[@]}" -gt 0 ]]; then
  sequence_csv="$(IFS=,; echo "${STORED_STATE_SEQUENCE[*]}")"
fi

if [[ -z "$sequence_csv" ]]; then
  echo "No stored state sequence configured in $0."
  echo "Available states from $WAREHOUSE_DB:"
  python3 "$REPO_ROOT/scripts/common/run_stored_states.py" \
    --database "$WAREHOUSE_DB" \
    --robot-id "$ROBOT_ID" \
    --list
  exit 1
fi

python3 "$REPO_ROOT/scripts/common/run_stored_states.py" \
  --database "$WAREHOUSE_DB" \
  --robot-id "$ROBOT_ID" \
  --sequence "$sequence_csv" \
  --duration "$MOVE_DURATION" \
  "$@"
