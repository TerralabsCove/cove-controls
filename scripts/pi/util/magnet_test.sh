#!/usr/bin/env bash
# Quick test for the magnet wired to GPIO17.
# Energises the pin until you press Enter, then releases.
set -euo pipefail

PIN="${PIN:-17}"
# RPi 5: header GPIOs live on gpiochip4. Fall back to gpiochip0 for older kernels.
CHIP="${CHIP:-}"

if ! command -v gpioset >/dev/null 2>&1; then
  echo "gpioset not found. Installing libgpiod tools (sudo)…"
  sudo apt install -y gpiod
fi

if [[ -z "$CHIP" ]]; then
  if gpioinfo gpiochip4 2>/dev/null | grep -qE "line\s+${PIN}\b"; then
    CHIP="gpiochip4"
  elif gpioinfo gpiochip0 2>/dev/null | grep -qE "line\s+${PIN}\b"; then
    CHIP="gpiochip0"
  else
    echo "Could not locate GPIO${PIN} on gpiochip4 or gpiochip0." >&2
    echo "Available chips:" >&2
    gpiodetect >&2 || true
    exit 1
  fi
fi

echo "Driving ${CHIP} line ${PIN} LOW (magnet OFF) for 1s…"
gpioset "${CHIP}" "${PIN}=0"
sleep 1
#gpioset "${CHIP}" "${PIN}=1"

#echo "Driving ${CHIP} line ${PIN} HIGH (magnet ON)…"
#echo "Press Ctrl+C to release."

# --mode=signal holds the line until the process is killed.
#exec gpioset --mode=signal "${CHIP}" "${PIN}=1"
