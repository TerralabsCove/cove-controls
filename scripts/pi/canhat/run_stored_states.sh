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

# Edit this sequence. Entries can be RViz Stored State names, SQLite row IDs,
# magnet_on, magnet_off, or quoted manual-duration states like "lift_drink(manual)".
# Example:
# STORED_STATE_SEQUENCE=(home magnet_on grabbing_bottle "lift_drink(manual)" to_cup magnet_off)
STORED_STATE_SEQUENCE=(home to_drink lift_drink to_cup pour1 to_cup lift_drink to_drink home
)

WAREHOUSE_DB="${WAREHOUSE_DB:-$HOME/.ros/simple_assembly_tracking_warehouse.sqlite}"
UBUNTU_WAREHOUSE_DB="${UBUNTU_WAREHOUSE_DB:-parallels@100.86.104.75:/home/parallels/.ros/simple_assembly_tracking_warehouse.sqlite}"
SYNC_WAREHOUSE_DB="${SYNC_WAREHOUSE_DB:-0}"
MOVE_DURATION="${MOVE_DURATION:-5.0}"
ROBOT_ID="${ROBOT_ID:-simple_assembly_tracking}"
SKIP_CONFIRMATION="${SKIP_CONFIRMATION:-0}"
GPIO_CHIP="${GPIO_CHIP:-gpiochip4}"
GPIO_LINE="${GPIO_LINE:-17}"

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

confirm_args=()
if [[ "$SKIP_CONFIRMATION" == "1" || "$SKIP_CONFIRMATION" == "true" ]]; then
  confirm_args+=(--skip-confirmation)
fi

python3 "$REPO_ROOT/scripts/common/run_stored_states.py" \
  --database "$WAREHOUSE_DB" \
  --robot-id "$ROBOT_ID" \
  --sequence "$sequence_csv" \
  --duration "$MOVE_DURATION" \
  --gpio-chip "$GPIO_CHIP" \
  --gpio-line "$GPIO_LINE" \
  "${confirm_args[@]}" \
  "$@"
