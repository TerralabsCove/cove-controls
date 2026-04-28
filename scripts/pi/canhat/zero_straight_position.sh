#!/usr/bin/env bash
set -euo pipefail

CAN_INTERFACE="${CAN_INTERFACE:-can1}"
START_ID="${START_ID:-1}"
END_ID="${END_ID:-7}"
INTERVAL="${INTERVAL:-0.10}"
CAPTURE_SECS="${CAPTURE_SECS:-2}"
DISABLE_FIRST="${DISABLE_FIRST:-0}"
ENABLE_AFTER="${ENABLE_AFTER:-0}"
LOG="${LOG:-/tmp/dm_zero_straight_${CAN_INTERFACE}.log}"

require_tool() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Missing required tool: $1" >&2
    exit 1
  fi
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
  local id
  for ((id = START_ID_DEC; id <= END_ID_DEC; id++)); do
    cansend "$CAN_INTERFACE" "$(frame_id "$id")#$data"
    sleep "$INTERVAL"
  done
}

send_status_to_range() {
  local id
  for ((id = START_ID_DEC; id <= END_ID_DEC; id++)); do
    cansend "$CAN_INTERFACE" "7FF#$(dm_id_le "$id")CC0000000000"
    sleep "$INTERVAL"
  done
}

summarize_log() {
  echo
  echo "Observed feedback/non-command frames:"
  awk '
    $3 ~ /#/ {
      split($3, f, "#")
      id = toupper(f[1])
      data = toupper(f[2])
      if (id == "7FF") next
      if (data == "FFFFFFFFFFFFFFFC" || data == "FFFFFFFFFFFFFFFD" || data == "FFFFFFFFFFFFFFFE") next
      count[id]++
      if (!(id in first)) first[id] = data
      last[id] = data
    }
    END {
      found = 0
      for (id in count) {
        found = 1
        printf "  %s count=%d first=%s last=%s\n", id, count[id], first[id], last[id]
      }
      if (!found) print "  none"
    }
  ' "$LOG" | sort
}

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
echo "Zeroing DM motor IDs 0x$(printf "%02X" "$START_ID_DEC")..0x$(printf "%02X" "$END_ID_DEC")"
echo "Put the arm in the straight-line zero pose before running this script."
echo "Capture log: $LOG"

if [[ "$DISABLE_FIRST" == "1" || "$DISABLE_FIRST" == "true" ]]; then
  echo "Sending disable frames first..."
  send_to_range "FFFFFFFFFFFFFFFD"
fi

echo "Sending set-zero-position frames..."
send_to_range "FFFFFFFFFFFFFFFE"

if [[ "$ENABLE_AFTER" == "1" || "$ENABLE_AFTER" == "true" ]]; then
  echo "Sending enable frames after zeroing..."
  send_to_range "FFFFFFFFFFFFFFFC"
fi

echo "Requesting status after zeroing..."
send_status_to_range

sleep "$CAPTURE_SECS"
cleanup
trap - EXIT

echo
echo "Raw capture lines: $(wc -l < "$LOG")"
summarize_log

echo
echo "Expected feedback IDs for 0x01..0x07: 011 012 013 014 015 016 017"
