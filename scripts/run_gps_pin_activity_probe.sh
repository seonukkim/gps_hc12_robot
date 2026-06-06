#!/usr/bin/env bash
# Passive PIN-LEVEL activity probe (READ-ONLY, no rewire).
#
# Reads raw digital activity on the OpenRB UART pins (D13/D14/D26/D27/D28/D29) to
# prove whether a GPS TX electrical signal reaches any pin. INPUT only (no pullup),
# digitalRead only; it NEVER writes to the GPS UARTs. No motors / no motion.
#
# One pass samples each pin 10 s (~60 s total) then repeats. Let one
# PIN_SWEEP_DONE print, then Ctrl-C.
#
# Usage: scripts/run_gps_pin_activity_probe.sh [-p PORT]

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"

FQBN="OpenRB-150:samd:OpenRB-150"
SKETCH="firmware/gps_pin_activity_probe"
TS="$(date +%Y%m%d_%H%M%S)"
BUILD_PATH="/private/tmp/openrb-gps-pin-activity-${TS}"   # fresh build path

PORT="${1:-}"
if [ "${PORT}" = "-p" ]; then PORT="${2:-}"; fi
if [ -z "${PORT}" ]; then
  PORT="$(arduino-cli board list 2>/dev/null | awk '/OpenRB-150/ {print $1; exit}')"
fi
if [ -z "${PORT}" ]; then
  echo "ABORT: OpenRB-150 port not found. Plug in the board or pass -p /dev/cu.usbmodemXXXX." >&2
  exit 1
fi

mkdir -p outputs/logs
LOG="outputs/logs/gps_pin_activity_${TS}.log"

echo "port=${PORT}"
echo "READ-ONLY pin-level probe: INPUT (no pullup), digitalRead only; never writes; no motors/motion."
echo "pins=D13(S3_RX) D14(S3_TX) D26(S1_TX) D27(S1_RX) D28(S2_TX) D29(S2_RX center 5-pin GPS-TX target)"
echo "log=${LOG}"

arduino-cli compile --fqbn "$FQBN" --build-path "$BUILD_PATH" "$SKETCH"
arduino-cli upload -p "$PORT" --fqbn "$FQBN" --build-path "$BUILD_PATH" "$SKETCH"
sleep 2

echo "Monitoring ${PORT} @ 115200 -> ${LOG}  (let one PIN_SWEEP_DONE print, then Ctrl-C)"
arduino-cli monitor -p "$PORT" --config baudrate=115200 | tee "$LOG"

echo
echo "Done. Check with: scripts/check_gps_pin_activity_log.sh ${LOG}"
