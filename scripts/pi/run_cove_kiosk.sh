#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PORT="${PORT:-8080}"
MOTION_SCRIPT_COMMAND="${MOTION_SCRIPT_COMMAND:-}"
MOTION_SCRIPT_CWD="${MOTION_SCRIPT_CWD:-$REPO_ROOT}"

usage() {
  cat <<'EOF'
Usage:
  scripts/pi/run_cove_kiosk.sh [--port PORT] [--script COMMAND] [--script-cwd DIR]

The iPad opens the printed http://<pi-ip>:PORT URL.

The script command is run once per submitted frontend order. Its stdout is
shown on the frontend. Plain text works; JSON lines with state/phase/step/message
and progress fields give cleaner status.

Examples:
  scripts/pi/run_cove_kiosk.sh
  scripts/pi/run_cove_kiosk.sh --script "python3 /home/pi/cove-controls/src/simple_assembly/motion_scripts/pick_and_place.py"
  MOTION_SCRIPT_COMMAND="python3 /home/pi/current_arm_script.py" scripts/pi/run_cove_kiosk.sh
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --port)
      [[ $# -ge 2 ]] || { echo "--port requires a value" >&2; exit 1; }
      PORT="$2"
      shift 2
      ;;
    --script)
      [[ $# -ge 2 ]] || { echo "--script requires a command string" >&2; exit 1; }
      MOTION_SCRIPT_COMMAND="$2"
      shift 2
      ;;
    --script-cwd)
      [[ $# -ge 2 ]] || { echo "--script-cwd requires a directory" >&2; exit 1; }
      MOTION_SCRIPT_CWD="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

set +u
source /opt/ros/jazzy/setup.bash
source "$REPO_ROOT/install/setup.bash"
set -u

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$REPO_ROOT/config/pi/simple_assembly_cyclonedds_pi.xml}"

pi_ip="$(hostname -I 2>/dev/null | awk '{print $1}')"
if [[ -n "$pi_ip" ]]; then
  printf '[cove-kiosk] iPad URL: http://%s:%s\n' "$pi_ip" "$PORT"
else
  printf '[cove-kiosk] iPad URL: http://<raspberry-pi-ip>:%s\n' "$PORT"
fi

args=(
  --ros-args
  -p "bind_host:=0.0.0.0"
  -p "bind_port:=${PORT}"
  -p "motion_script_cwd:=${MOTION_SCRIPT_CWD}"
)

if [[ -n "$MOTION_SCRIPT_COMMAND" ]]; then
  args+=(-p "motion_script_command:=${MOTION_SCRIPT_COMMAND}")
fi

exec ros2 run cove_kiosk_bridge kiosk_bridge "${args[@]}"
