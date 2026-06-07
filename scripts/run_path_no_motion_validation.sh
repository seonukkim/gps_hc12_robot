#!/usr/bin/env bash
set -euo pipefail

PORT="/dev/ttyACM0"
PATH_PACKAGE=""
SAMPLE_LOG=""
OUT_DIR="outputs/path_no_motion_validation/latest"
MODE="auto"
DURATION_S="0"
CONCISE="true"
NO_MOTION_GPS_MODE="position_only"
MIN_SATS="4"
MAX_HDOP="3.0"
VERBOSE="false"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --port)
      PORT="$2"
      shift 2
      ;;
    --path-package)
      PATH_PACKAGE="$2"
      shift 2
      ;;
    --sample-log)
      SAMPLE_LOG="$2"
      shift 2
      ;;
    --mode)
      MODE="$2"
      shift 2
      ;;
    --duration-s)
      DURATION_S="$2"
      shift 2
      ;;
    --out-dir)
      OUT_DIR="$2"
      shift 2
      ;;
    --concise)
      CONCISE="$2"
      shift 2
      ;;
    --no-motion-gps-mode)
      NO_MOTION_GPS_MODE="$2"
      shift 2
      ;;
    --min-sats)
      MIN_SATS="$2"
      shift 2
      ;;
    --max-hdop)
      MAX_HDOP="$2"
      shift 2
      ;;
    --verbose)
      VERBOSE="true"
      shift
      ;;
    -h|--help)
      cat <<'USAGE'
Usage:
  scripts/run_path_no_motion_validation.sh \
    --port /dev/ttyACM0 \
    --path-package latest \
    [--mode package_check|live_serial|auto] \
    [--duration-s 120] \
    [--sample-log outputs/logs/no_motion_status_sample.log] \
    [--concise true] \
    [--no-motion-gps-mode position_only] \
    [--min-sats 4] \
    [--max-hdop 3.0] \
    [--verbose] \
    [--out-dir outputs/path_no_motion_validation/latest]

No-motion only: this wrapper does not upload firmware, does not send HC-12
frames, and does not generate rover motor commands. The Python validator reports
the port for traceability but does not open serial by default. In position_only
mode, GPS course heading is diagnostic-only for stationary validation.

Use --mode package_check for path-package-only validation. Use --mode live_serial
--duration-s 120 only when you intentionally want to open the OpenRB serial port
for live no-motion target validation.
USAGE
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

if [[ -z "${PATH_PACKAGE}" ]]; then
  echo "ERROR: --path-package is required." >&2
  exit 2
fi

CMD=(
  uv run python tools/path_no_motion_validation.py
  --path-package "${PATH_PACKAGE}"
  --mode "${MODE}"
  --duration-s "${DURATION_S}"
  --out-dir "${OUT_DIR}"
  --concise "${CONCISE}"
  --no-motion-gps-mode "${NO_MOTION_GPS_MODE}"
  --min-sats "${MIN_SATS}"
  --max-hdop "${MAX_HDOP}"
)
if [[ -n "${PORT}" ]]; then
  CMD+=(--port "${PORT}")
fi
if [[ -n "${SAMPLE_LOG}" ]]; then
  CMD+=(--sample-log "${SAMPLE_LOG}")
fi
if [[ "${VERBOSE}" == "true" ]]; then
  CMD+=(--verbose)
fi

echo "NO-MOTION VALIDATION ONLY"
echo "physical_output_active=false"
echo "motor_command_generated=false"
"${CMD[@]}"
