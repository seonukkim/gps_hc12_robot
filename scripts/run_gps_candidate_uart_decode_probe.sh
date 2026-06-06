#!/usr/bin/env bash
# Read-only bit-bang UART decode probe for GPS candidate pins.
#
# Samples D6/D13/D27/D29 as INPUT only and tries 8N1 decode at common GPS bauds.
# The firmware never writes Serial1/2/3 and uses no motors/motion.
#
# Usage:
#   scripts/run_gps_candidate_uart_decode_probe.sh [-p PORT] [--seconds 12]

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"

FQBN="OpenRB-150:samd:OpenRB-150"
SKETCH="firmware/gps_candidate_uart_decode_probe"
TS="$(date +%Y%m%d_%H%M%S)"
BUILD_PATH="/private/tmp/openrb-gps-candidate-uart-decode-${TS}"
SECONDS_EACH=12
PORT=""

usage() {
  cat <<EOF
Usage: scripts/run_gps_candidate_uart_decode_probe.sh [options]

Options:
  -p, --port PORT      OpenRB serial port, e.g. /dev/cu.usbmodem12101
  --seconds SECONDS    Seconds per pin/baud pair. Default: 12.
  -h, --help           Show this help.

This is read-only: D6/D13/D27/D29 INPUT + digitalRead bit-bang decode. No motors.
EOF
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    -p|--port)
      shift
      PORT="${1:-}"
      ;;
    --seconds)
      shift
      SECONDS_EACH="${1:-}"
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

case "$SECONDS_EACH" in
  ''|*[!0-9]*)
    echo "ABORT: --seconds must be a positive integer, got '${SECONDS_EACH}'." >&2
    exit 2
    ;;
esac

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

if [ -z "$PORT" ]; then
  PORT="$(detect_openrb_port || true)"
fi
if [ -z "$PORT" ]; then
  echo "ABORT: OpenRB-150 port not found by arduino-cli or pyserial list_ports." >&2
  echo "       Plug in OpenRB, then rerun or pass -p /dev/cu.usbmodemXXXX." >&2
  exit 1
fi

mkdir -p outputs/logs
LOG="outputs/logs/gps_candidate_uart_decode_${TS}.log"
FLAGS="-DUART_DECODE_SECONDS=${SECONDS_EACH}"
MONITOR_SECONDS=$((SECONDS_EACH * 4 * 5 + 60))
if [ "$MONITOR_SECONDS" -lt 300 ]; then
  MONITOR_SECONDS=300
fi

echo "port=${PORT}"
echo "READ-ONLY candidate UART decode: D6,D13,D27,D29 INPUT, digitalRead only; no motors/motion."
echo "seconds_each=${SECONDS_EACH}"
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
set +e
python3 -c 'import sys, time, serial
port=sys.argv[1]
seconds=int(sys.argv[2])
log_path=sys.argv[3]
deadline=time.time()+seconds
exit_code=0
try:
    with serial.Serial(port, 115200, timeout=0.2) as ser, open(log_path, "wb") as log:
        while time.time() < deadline:
            data=ser.read(4096)
            if not data:
                continue
            log.write(data)
            log.flush()
            sys.stdout.buffer.write(data)
            sys.stdout.buffer.flush()
except Exception as exc:
    exit_code=3
    msg=("\nSERIAL_CAPTURE_ERROR: %s: %s\n" % (exc.__class__.__name__, exc)).encode()
    sys.stderr.buffer.write(msg)
    try:
        with open(log_path, "ab") as log:
            log.write(msg)
    except Exception:
        pass
sys.exit(exit_code)
' "$MONITOR_PORT" "$MONITOR_SECONDS" "$LOG"
CAPTURE_STATUS=$?
set -e

echo
if [ "$CAPTURE_STATUS" -ne 0 ]; then
  echo "WARNING: serial capture ended with status ${CAPTURE_STATUS}; partial log kept at ${LOG}." >&2
fi
if grep -q '^UART_DECODE_SWEEP_DONE' "$LOG"; then
  echo "UART_DECODE_SWEEP_DONE found in ${LOG}."
else
  echo "WARNING: UART_DECODE_SWEEP_DONE missing from ${LOG}; the capture is incomplete." >&2
fi
echo "Done. Check with: scripts/check_gps_candidate_uart_decode_log.sh ${LOG}"
