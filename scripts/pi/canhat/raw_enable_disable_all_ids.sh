#!/usr/bin/env bash
set -euo pipefail

CAN_INTERFACE="${CAN_INTERFACE:-can1}"
START_ID="${START_ID:-1}"
END_ID="${END_ID:-7}"
ACTION="${ACTION:-cycle}"
PASSES="${PASSES:-1}"
INTERVAL="${INTERVAL:-0.08}"
CAPTURE_SECS="${CAPTURE_SECS:-8}"
LEAVE_ENABLED="${LEAVE_ENABLED:-0}"
LOG="${LOG:-/tmp/dm_raw_enable_disable_${CAN_INTERFACE}.log}"

require_tool() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Missing required tool: $1" >&2
    exit 1
  fi
}

print_can_state() {
  ip -details -statistics link show "$CAN_INTERFACE" | awk '
    /can state/ {
      print "CAN state: " $3
    }
    /RX:/ {
      getline
      print "RX packets: " $2 ", errors: " $3
    }
    /TX:/ {
      getline
      print "TX packets: " $2 ", errors: " $3
    }
  '
}

parse_int() {
  local value="$1"
  if [[ "$value" == 0x* || "$value" == 0X* ]]; then
    printf "%d" "$value"
  else
    printf "%d" "$value"
  fi
}

frame_id() {
  printf "%03X" "$1"
}

dm_id_le() {
  local id="$1"
  printf "%02X%02X" "$((id & 0xff))" "$(((id >> 8) & 0xff))"
}

send_to_range() {
  local data="$1"
  local pass id
  for ((pass = 1; pass <= PASSES; pass++)); do
    for ((id = START_ID_DEC; id <= END_ID_DEC; id++)); do
      cansend "$CAN_INTERFACE" "$(frame_id "$id")#$data"
      sleep "$INTERVAL"
    done
  done
}

send_status_to_range() {
  local pass id
  for ((pass = 1; pass <= PASSES; pass++)); do
    for ((id = START_ID_DEC; id <= END_ID_DEC; id++)); do
      cansend "$CAN_INTERFACE" "7FF#$(dm_id_le "$id")CC0000000000"
      sleep "$INTERVAL"
    done
  done
}

summarize_log() {
  echo
  echo "Observed non-command CAN frames during capture:"
  awk '
    $3 ~ /#/ {
      split($3, f, "#")
      id = toupper(f[1])
      data = toupper(f[2])
      if (id == "7FF") {
        next
      }
      if (data == "FFFFFFFFFFFFFFFC" || data == "FFFFFFFFFFFFFFFD") {
        next
      }
      count[id]++
      if (!(id in first)) {
        first[id] = data
      }
      last[id] = data
    }
    END {
      found = 0
      for (id in count) {
        found = 1
        printf "  %s count=%d first=%s last=%s\n", id, count[id], first[id], last[id]
      }
      if (!found) {
        print "  none"
      }
    }
  ' "$LOG" | sort

  echo
  echo "Expected master IDs if configured as slave+0x10:"
  local id
  for ((id = START_ID_DEC; id <= END_ID_DEC; id++)); do
    printf "  slave 0x%02X -> feedback 0x%02X\n" "$id" "$((id + 0x10))"
  done
}

case "$ACTION" in
  cycle|enable|disable)
    ;;
  *)
    echo "ACTION must be one of: cycle, enable, disable" >&2
    exit 1
    ;;
esac

require_tool cansend
require_tool candump

START_ID_DEC="$(parse_int "$START_ID")"
END_ID_DEC="$(parse_int "$END_ID")"

if ((START_ID_DEC < 0 || END_ID_DEC > 0x7ff || START_ID_DEC > END_ID_DEC)); then
  echo "Invalid START_ID/END_ID range: $START_ID..$END_ID" >&2
  exit 1
fi

rm -f "$LOG"

candump -L "$CAN_INTERFACE" > "$LOG" &
DUMP_PID=$!
cleanup() {
  kill "$DUMP_PID" >/dev/null 2>&1 || true
  wait "$DUMP_PID" >/dev/null 2>&1 || true
}
trap cleanup EXIT

sleep 0.2

echo "CAN interface: $CAN_INTERFACE"
echo "ID range: 0x$(printf "%02X" "$START_ID_DEC")..0x$(printf "%02X" "$END_ID_DEC")"
echo "Action: $ACTION"
echo "Capture log: $LOG"
print_can_state

case "$ACTION" in
  cycle)
    echo "Sending disable frames..."
    send_to_range "FFFFFFFFFFFFFFFD"
    sleep 0.25
    echo "Sending enable frames..."
    send_to_range "FFFFFFFFFFFFFFFC"
    sleep 0.25
    echo "Sending status requests..."
    send_status_to_range
    if [[ "$LEAVE_ENABLED" != "1" && "$LEAVE_ENABLED" != "true" ]]; then
      sleep 0.25
      echo "Sending final disable frames. Set LEAVE_ENABLED=1 to skip this."
      send_to_range "FFFFFFFFFFFFFFFD"
    fi
    ;;
  enable)
    echo "Sending enable frames..."
    send_to_range "FFFFFFFFFFFFFFFC"
    sleep 0.25
    echo "Sending status requests..."
    send_status_to_range
    ;;
  disable)
    echo "Sending disable frames..."
    send_to_range "FFFFFFFFFFFFFFFD"
    sleep 0.25
    echo "Sending status requests..."
    send_status_to_range
    ;;
esac

sleep "$CAPTURE_SECS"
cleanup
trap - EXIT

echo
RAW_LINES="$(wc -l < "$LOG")"
echo "Raw capture lines: $RAW_LINES"
print_can_state
if [[ "$RAW_LINES" == "0" ]]; then
  echo "Warning: no raw frames were captured. If cansend returned success, this usually means the CAN frame was not ACKed by any node or the bus is not physically connected."
fi
summarize_log
