#!/usr/bin/env bash
# Passive GPS UART + baud sweep (READ-ONLY, no rewire).
#
# Listens on Serial1 / Serial2 / Serial3 at {4800,9600,38400,57600,115200} and
# reports where real NMEA appears. It NEVER writes to the GPS UARTs and does not
# require unplugging or rewiring anything. No motors / no motion.
#
# One full sweep is ~15 candidates x ~6 s = ~90 s; it then repeats. Let it run a
# full pass (watch for SWEEP_DONE), then Ctrl-C.
#
# Usage: scripts/run_gps_passive_uart_baud_sweep.sh [-p PORT]

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"

FQBN="OpenRB-150:samd:OpenRB-150"
SKETCH="firmware/gps_passive_uart_baud_sweep_probe"
TS="$(date +%Y%m%d_%H%M%S)"
BUILD_PATH="/private/tmp/openrb-gps-passive-sweep-${TS}"   # fresh build path

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
LOG="outputs/logs/gps_passive_uart_baud_sweep_${TS}.log"

echo "port=${PORT}"
echo "READ-ONLY no-rewire sweep: never writes to GPS UARTs; no motors / no motion."
echo "ports=Serial1,Serial2,Serial3  bauds=4800,9600,38400,57600,115200"
echo "log=${LOG}"

arduino-cli compile --fqbn "$FQBN" --build-path "$BUILD_PATH" "$SKETCH"
arduino-cli upload -p "$PORT" --fqbn "$FQBN" --build-path "$BUILD_PATH" "$SKETCH"
sleep 2

echo "Monitoring ${PORT} @ 115200 -> ${LOG}  (let one SWEEP_DONE print, then Ctrl-C)"
arduino-cli monitor -p "$PORT" --config baudrate=115200 | tee "$LOG"

echo
echo "Done. Check with: scripts/check_gps_passive_uart_baud_sweep_log.sh ${LOG}"
