#!/usr/bin/env bash
# GPS UART probe on the known-good wiring: Serial2 @ 9600 (center 5-pin connector).
#
# GPS is on the OpenRB-150 CENTER 5-pin connector = Serial2 at 9600, confirmed by
# the user and old known-good logs (selected_port=Serial2, chars_1s ~500, fix=true,
# sats ~5, hdop ~1.6). It is NOT on the side Serial3 D13/D14 pins. This script
# forces the probe to Serial2 so a zero-byte result means wiring/power/baud, not a
# wrong-UART assumption.
#
# No motors / no autonomy: gps_uart_probe only reads one UART and prints status.
#
# Usage: scripts/run_gps_probe_serial2_9600.sh [-p PORT]

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"

FQBN="OpenRB-150:samd:OpenRB-150"
SKETCH="firmware/gps_uart_probe"
TS="$(date +%Y%m%d_%H%M%S)"
BUILD_PATH="/private/tmp/openrb-gps-probe-serial2-9600-${TS}"   # fresh build path
FLAGS="-DGPS_PROBE_MODE=2 -DGPS_PROBE_BAUD=9600"

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
LOG="outputs/logs/gps_probe_serial2_9600_${TS}.log"

echo "port=${PORT}"
echo "flags=${FLAGS}"
echo "expected selected_port=Serial2 (center 5-pin connector), baud=9600"
echo "log=${LOG}"

arduino-cli compile --fqbn "$FQBN" --build-path "$BUILD_PATH" \
  --build-property "compiler.cpp.extra_flags=${FLAGS}" "$SKETCH"
arduino-cli upload -p "$PORT" --fqbn "$FQBN" --build-path "$BUILD_PATH" "$SKETCH"
sleep 2

echo "Monitoring ${PORT} @ 115200 -> ${LOG}  (Ctrl-C to stop)"
echo "Expect: selected_port=Serial2, chars_1s>0, then fix=true / sats / hdop."
arduino-cli monitor -p "$PORT" --config baudrate=115200 | tee "$LOG"

echo
echo "Done. Check with: scripts/check_gps_probe_log.sh ${LOG}"
