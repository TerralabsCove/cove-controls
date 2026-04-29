#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

MODE="sim"
BUILD=0
SYNC_BUNDLE=1
PORT=8080
WAIT_SECONDS=12

usage() {
  cat <<'EOF'
Usage:
  scripts/ubuntu/run_cove_kiosk.sh [sim|rviz-sim|full|smoke-test] [--build] [--no-sync] [--port PORT] [--wait SECONDS]

Modes:
  sim         Start only the kiosk backend in simulated mode. Best for UI and API testing.
  rviz-sim    Start fake ros2_control + MoveIt + RViz + kiosk bridge for visual testing.
  full        Start the full robot stack through robot.launch.py.
  smoke-test  Start the simulated kiosk backend in the background, verify the API, then exit.

Options:
  --build       Build the required packages before running.
  --no-sync     Skip regenerating the kiosk HTML bundle.
  --port PORT   HTTP port for sim/smoke-test mode. Default: 8080.
  --wait SEC    Startup wait for smoke-test mode. Default: 12.
  -h, --help    Show this help text.

Examples:
  scripts/ubuntu/run_cove_kiosk.sh sim --build
  scripts/ubuntu/run_cove_kiosk.sh rviz-sim --build
  scripts/ubuntu/run_cove_kiosk.sh full --build
  scripts/ubuntu/run_cove_kiosk.sh smoke-test --build
EOF
}

log() {
  printf '[cove-kiosk] %s\n' "$*"
}

die() {
  printf '[cove-kiosk] %s\n' "$*" >&2
  exit 1
}

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || die "Missing required command: $1"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    sim|rviz-sim|full|smoke-test)
      MODE="$1"
      shift
      ;;
    --build)
      BUILD=1
      shift
      ;;
    --no-sync)
      SYNC_BUNDLE=0
      shift
      ;;
    --port)
      [[ $# -ge 2 ]] || die "--port requires a value"
      PORT="$2"
      shift 2
      ;;
    --wait)
      [[ $# -ge 2 ]] || die "--wait requires a value"
      WAIT_SECONDS="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "Unknown argument: $1"
      ;;
  esac
done

require_cmd python3

set +u
source /opt/ros/jazzy/setup.bash
set -u

require_cmd ros2

if [[ "$MODE" == "smoke-test" ]]; then
  require_cmd curl
fi

if [[ "$SYNC_BUNDLE" -eq 1 ]]; then
  log "Syncing kiosk HTML bundle"
  python3 "$REPO_ROOT/scripts/update_kiosk_bundle.py"
fi

if [[ "$BUILD" -eq 1 ]]; then
  log "Building required packages"
  if [[ "$MODE" == "full" || "$MODE" == "rviz-sim" ]]; then
    "$REPO_ROOT/scripts/build_workspace.sh" --packages-select simple_assembly cove_kiosk_bridge simple_assembly_moveit_config
  else
    "$REPO_ROOT/scripts/build_workspace.sh" --packages-select cove_kiosk_bridge
  fi
fi

[[ -f "$REPO_ROOT/install/setup.bash" ]] || die "Workspace is not built yet. Run with --build or build it first."

set +u
source "$REPO_ROOT/install/setup.bash"
set -u

run_sim() {
  log "Starting kiosk backend on http://localhost:${PORT}"
  exec ros2 run cove_kiosk_bridge kiosk_bridge --ros-args -p bind_port:="${PORT}"
}

run_full() {
  log "Starting full robot stack on http://localhost:8080"
  exec ros2 launch simple_assembly_moveit_config robot.launch.py
}

run_rviz_sim() {
  log "Starting RViz simulation stack on http://localhost:${PORT}"
  exec ros2 launch simple_assembly_moveit_config sim_kiosk.launch.py bind_port:="${PORT}"
}

run_smoke_test() {
  local server_pid
  local state_json
  local order_json

  log "Starting simulated kiosk backend for smoke test on http://localhost:${PORT}"
  ros2 run cove_kiosk_bridge kiosk_bridge --ros-args -p bind_port:="${PORT}" >/tmp/cove_kiosk_smoke.log 2>&1 &
  server_pid=$!

  cleanup() {
    if kill -0 "$server_pid" >/dev/null 2>&1; then
      kill "$server_pid" >/dev/null 2>&1 || true
      wait "$server_pid" >/dev/null 2>&1 || true
    fi
  }
  trap cleanup EXIT

  sleep "$WAIT_SECONDS"

  state_json="$(curl -fsS "http://127.0.0.1:${PORT}/api/state")" || die "Smoke test failed to fetch /api/state"
  python3 -c 'import json,sys; payload=json.loads(sys.argv[1]); assert "state" in payload and "queue" in payload' "$state_json" \
    || die "Smoke test received an invalid state payload"

  order_json="$(curl -fsS -X POST "http://127.0.0.1:${PORT}/api/orders" -H 'Content-Type: application/json' -d '{"name":"Smoke Test","isMe":true}')" || die "Smoke test failed to submit an order"
  python3 -c 'import json,sys; payload=json.loads(sys.argv[1]); assert payload["ok"] is True; assert payload["order"]["name"] == "Smoke Test"' "$order_json" \
    || die "Smoke test order response was invalid"

  sleep 2
  state_json="$(curl -fsS "http://127.0.0.1:${PORT}/api/state")" || die "Smoke test failed to fetch post-order state"
  python3 -c 'import json,sys; payload=json.loads(sys.argv[1]); assert any(order["name"] == "Smoke Test" for order in payload["queue"])' "$state_json" \
    || die "Smoke test post-order state was invalid"

  log "Smoke test passed"
  printf 'URL: http://localhost:%s\n' "$PORT"
  printf 'Order response: %s\n' "$order_json"
  printf 'State snapshot: %s\n' "$state_json"

  trap - EXIT
  cleanup
}

case "$MODE" in
  sim)
    run_sim
    ;;
  full)
    run_full
    ;;
  rviz-sim)
    run_rviz_sim
    ;;
  smoke-test)
    run_smoke_test
    ;;
esac
