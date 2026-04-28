#!/usr/bin/env bash
set -euo pipefail

CAN_INTERFACE="${CAN_INTERFACE:-can1}"
START_ID="${START_ID:-1}"
END_ID="${END_ID:-7}"
PASSES="${PASSES:-3}"
INTERVAL="${INTERVAL:-0.04}"
CAPTURE_SECS="${CAPTURE_SECS:-2}"
LOG="${LOG:-/tmp/dm_raw_scan_${CAN_INTERFACE}.log}"

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

dm_id_le() {
  local id="$1"
  printf "%02X%02X" "$((id & 0xff))" "$(((id >> 8) & 0xff))"
}

summarize_log() {
  echo
  echo "Observed feedback/non-scan frames:"
  awk '
    $3 ~ /#/ {
      split($3, f, "#")
      id = toupper(f[1])
      data = toupper(f[2])
      if (id == "7FF") {
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

require_tool cansend
require_tool candump

START_ID_DEC="$(parse_int "$START_ID")"
END_ID_DEC="$(parse_int "$END_ID")"

if ((START_ID_DEC < 0 || END_ID_DEC > 0xffff || START_ID_DEC > END_ID_DEC)); then
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
echo "Scanning DM slave IDs 0x$(printf "%02X" "$START_ID_DEC")..0x$(printf "%02X" "$END_ID_DEC") with status requests only"
echo "Capture log: $LOG"
print_can_state

for ((pass = 1; pass <= PASSES; pass++)); do
  for ((id = START_ID_DEC; id <= END_ID_DEC; id++)); do
    cansend "$CAN_INTERFACE" "7FF#$(dm_id_le "$id")CC0000000000"
    sleep "$INTERVAL"
  done
done

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
