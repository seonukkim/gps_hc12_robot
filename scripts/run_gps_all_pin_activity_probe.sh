#!/usr/bin/env bash
# Read-only all-pin activity probe for OpenRB-150.
#
# Locates where GPS TX electrical edges actually arrive on OpenRB without
# unplugging individual wires. The sketch sets D0-D29 to INPUT and reads them
# with digitalRead only. It never writes Serial1/2/3 and uses no motors/motion.
#
# One pass samples D0-D29 for 5 s each (~150 s total) then repeats. This script
# captures for 300 s by default so D29 and ALL_PIN_SWEEP_DONE are included.
#
# Usage:
#   scripts/run_gps_all_pin_activity_probe.sh [-p PORT] [--monitor-seconds 360]
# Optional compile-time override:
#   ALL_PIN_SAMPLE_MS=3000 scripts/run_gps_all_pin_activity_probe.sh

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"

FQBN="OpenRB-150:samd:OpenRB-150"
SKETCH="firmware/gps_all_pin_activity_probe"
TS="$(date +%Y%m%d_%H%M%S)"
BUILD_PATH="/private/tmp/openrb-gps-all-pin-activity-${TS}"
SAMPLE_MS="${ALL_PIN_SAMPLE_MS:-5000}"
MONITOR_SECONDS=300
PORT=""

usage() {
  cat <<EOF
Usage: scripts/run_gps_all_pin_activity_probe.sh [options]

Options:
  -p, --port PORT              OpenRB serial port, e.g. /dev/cu.usbmodem12101
  --monitor-seconds SECONDS    Capture duration after upload. Default: 300.
  --sample-ms MS               Per-pin sample duration. Default: ${SAMPLE_MS}.
  -h, --help                   Show this help.

This is read-only on OpenRB pins: D0-D29 INPUT + digitalRead only. No motors.
EOF
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    -p|--port)
      shift
      PORT="${1:-}"
      ;;
    --monitor-seconds)
      shift
      MONITOR_SECONDS="${1:-}"
      ;;
    --sample-ms)
      shift
      SAMPLE_MS="${1:-}"
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "ABORT: unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

case "$SAMPLE_MS" in
  ''|*[!0-9]*)
    echo "ABORT: --sample-ms / ALL_PIN_SAMPLE_MS must be a positive integer, got '${SAMPLE_MS}'." >&2
    exit 2
    ;;
esac
case "$MONITOR_SECONDS" in
  ''|*[!0-9]*)
    echo "ABORT: --monitor-seconds must be a positive integer, got '${MONITOR_SECONDS}'." >&2
    exit 2
    ;;
esac

REQUIRED_SECONDS=$(( (SAMPLE_MS * 30 + 999) / 1000 + 60 ))
if [ "$REQUIRED_SECONDS" -lt 240 ]; then
  REQUIRED_SECONDS=240
fi
if [ "$MONITOR_SECONDS" -lt "$REQUIRED_SECONDS" ]; then
  echo "WARNING: monitor_seconds=${MONITOR_SECONDS} is too short for D0-D29 at sample_ms_each=${SAMPLE_MS}; using ${REQUIRED_SECONDS}." >&2
  MONITOR_SECONDS="$REQUIRED_SECONDS"
fi

detect_openrb_port() {
  local detected=""
  detected="$(arduino-cli board list 2>/dev/null | awk '/OpenRB-150/ {print $1; exit}' || true)"
  if [ -n "$detected" ]; then
    printf '%s\n' "$detected"
    return 0
  fi

  detected="$(python3 -c 'from serial.tools import list_ports
ports=list(list_ports.comports())
for p in ports:
    text=" ".join(str(x or "") for x in (p.device,p.description,p.hwid))
    if "OpenRB-150" in text or "VID:PID=2F5D:2202" in text:
        print(p.device); raise SystemExit
for p in ports:
    if "usbmodem" in (p.device or ""):
        print(p.device); raise SystemExit
' 2>/dev/null || true)"
  if [ -n "$detected" ]; then
    printf '%s\n' "$detected"
    return 0
  fi
  return 1
}

if [ -z "${PORT}" ]; then
  PORT="$(detect_openrb_port || true)"
fi
if [ -z "${PORT}" ]; then
  echo "ABORT: OpenRB-150 port not found by arduino-cli or pyserial list_ports." >&2
  echo "       Plug in OpenRB, then rerun or pass -p /dev/cu.usbmodemXXXX." >&2
  exit 1
fi

mkdir -p outputs/logs
LOG="outputs/logs/gps_all_pin_activity_${TS}.log"
FLAGS="-DALL_PIN_SAMPLE_MS=${SAMPLE_MS}"

echo "port=${PORT}"
echo "READ-ONLY all-pin probe: D0-D29 INPUT, digitalRead only; never writes; no motors/motion."
echo "sample_ms_each=${SAMPLE_MS}"
echo "monitor_seconds=${MONITOR_SECONDS}"
echo "log=${LOG}"

arduino-cli compile \
  --fqbn "$FQBN" \
  --build-path "$BUILD_PATH" \
  --build-property "compiler.cpp.extra_flags=${FLAGS}" \
  "$SKETCH"
arduino-cli upload -p "$PORT" --fqbn "$FQBN" --build-path "$BUILD_PATH" "$SKETCH"
sleep 2

MONITOR_PORT="$(detect_openrb_port || true)"
if [ -z "$MONITOR_PORT" ]; then
  echo "ABORT: upload completed, but OpenRB port was not found again for monitor." >&2
  echo "       Replug OpenRB and rerun, or pass -p explicitly." >&2
  exit 1
fi

echo "monitor_port=${MONITOR_PORT}"
echo "Capturing ${MONITOR_PORT} @ 115200 for ${MONITOR_SECONDS}s -> ${LOG}"
python3 -c 'import sys, time, serial
port=sys.argv[1]
seconds=int(sys.argv[2])
log_path=sys.argv[3]
deadline=time.time()+seconds
with serial.Serial(port, 115200, timeout=0.2) as ser, open(log_path, "wb") as log:
    while time.time() < deadline:
        data=ser.read(4096)
        if not data:
            continue
        log.write(data)
        log.flush()
        sys.stdout.buffer.write(data)
        sys.stdout.buffer.flush()
' "$MONITOR_PORT" "$MONITOR_SECONDS" "$LOG"

echo
if grep -q '^ALL_PIN_SWEEP_DONE' "$LOG"; then
  echo "ALL_PIN_SWEEP_DONE found in ${LOG}."
else
  echo "WARNING: ALL_PIN_SWEEP_DONE missing from ${LOG}; the capture is incomplete." >&2
  echo "         Re-run with a longer capture, e.g. --monitor-seconds 360." >&2
fi
echo "Done. Check with: scripts/check_gps_all_pin_activity_log.sh ${LOG}"
