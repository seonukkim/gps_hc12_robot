#!/usr/bin/env bash
# Deprecated. Use scripts/run_physical_path_planner.sh <mode>.
set -euo pipefail

PORT="/dev/ttyACM0"
MODE="evaluate_logs"
MANUAL_LOG=""
SENSOR_LOG=""
STATION_PULSE_LOG=""
STATION_PULSE_CSV=""
COMBINED_LOG=""
OUT_DIR="outputs/stage29_baseline_lock/latest"
MIN_SATS="5"
MAX_HDOP="2.5"
MANUAL_DURATION_S="60"
SENSOR_DURATION_S="30"
REQUIRE_ENTER="true"
UPLOAD_STAGE20="false"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --port) PORT="$2"; shift 2 ;;
    --mode) MODE="$2"; shift 2 ;;
    --manual-log) MANUAL_LOG="$2"; shift 2 ;;
    --sensor-log) SENSOR_LOG="$2"; shift 2 ;;
    --station-pulse-log) STATION_PULSE_LOG="$2"; shift 2 ;;
    --station-pulse-csv) STATION_PULSE_CSV="$2"; shift 2 ;;
    --combined-log) COMBINED_LOG="$2"; shift 2 ;;
    --out-dir) OUT_DIR="$2"; shift 2 ;;
    --min-sats) MIN_SATS="$2"; shift 2 ;;
    --max-hdop) MAX_HDOP="$2"; shift 2 ;;
    --manual-duration-s) MANUAL_DURATION_S="$2"; shift 2 ;;
    --sensor-duration-s) SENSOR_DURATION_S="$2"; shift 2 ;;
    --require-enter) REQUIRE_ENTER="$2"; shift 2 ;;
    --upload-stage20) UPLOAD_STAGE20="$2"; shift 2 ;;
    -h|--help)
      cat <<'USAGE'
Usage:
  legacy/stage_scripts/run_stage29_baseline_lock.sh \
    --mode evaluate_logs \
    --manual-log outputs/manual_rc_passthrough_validation/latest/raw_usbdbg.log \
    --sensor-log outputs/gps_link_and_fix_watch/latest/raw_usbdbg.log \
    --station-pulse-csv outputs/stage26_calibrated_primitive_sequence/latest/stage26_calibrated_primitive_sequence.csv \
    --out-dir outputs/stage29_baseline_lock/latest

Live mode reads USB telemetry and can run the fixed calibrated Stage20 pulse
sequence. It is a baseline validator, not path following.
USAGE
      exit 0
      ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"
mkdir -p "$OUT_DIR"

FQBN="OpenRB-150:samd:OpenRB-150"
BUILD_PATH="/private/tmp/openrb-stage29-baseline-lock"
SKETCH="firmware/openrb_robot_controller"
FLAGS="-DSTAGE20_PHYSICAL_AB_GUARDED_CRAWL=1 \
-DSTAGE20_MAX_ABS_A=0.35 \
-DSTAGE20_MAX_ABS_B=0.35 \
-DSTAGE20_MAX_MS=1000 \
-DIMU_ENABLE=1 \
-DIMU_YAW_DIAG=1 \
-DPHYSICAL_PATH_FOLLOWING_ENABLE=0 \
-DPATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0 \
-DPATH_FOLLOWING_DRYRUN=0 \
-DPATH_FOLLOWING_HC12_ENABLED=0 \
-DGROUND_CRAWL_TEST_MODE=0 \
-DAUTO_MOTION_ARMED=0 \
-DSTAGE15_GUARDED_CRAWL_TEST=0 \
-DSTAGE16_USB_GUARDED_CRAWL=0 \
-DSTAGE17_FIRST_PRIMITIVE_CRAWL=0 \
-DSTAGE18_MOTOR_MAPPING_PROBE=0"

if printf '%s' "$FLAGS" | grep -Eq 'PHYSICAL_PATH_FOLLOWING_ENABLE=1|PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=1|PATH_FOLLOWING_DRYRUN=1|PATH_FOLLOWING_HC12_ENABLED=1|GROUND_CRAWL_TEST_MODE=1|AUTO_MOTION_ARMED=1'; then
  echo "ABORT: path-following, HC-12, or autonomous motion flag present." >&2
  exit 2
fi

echo "STAGE29 BASELINE LOCK"
echo "NOT FULL PATH FOLLOWING"
echo "NO HC-12"
echo "mode=${MODE}"
echo "upload_stage20=${UPLOAD_STAGE20}"
echo "port=${PORT}"
echo "out_dir=${OUT_DIR}"
echo "calibration_forward=A0.30_B0.00_800ms"
echo "calibration_backward=A-0.08_B0.00_300ms"
echo "calibration_turn_left=A0.00_B0.26_700ms"
echo "calibration_turn_right=A0.00_B-0.08_250ms"
echo "flags=${FLAGS}"

if [[ "$UPLOAD_STAGE20" == "true" ]]; then
  arduino-cli compile --fqbn "$FQBN" --build-path "$BUILD_PATH" \
    --build-property "compiler.cpp.extra_flags=${FLAGS}" "$SKETCH"
  arduino-cli upload -p "$PORT" --fqbn "$FQBN" --build-path "$BUILD_PATH" "$SKETCH"
fi

ARGS=(
  legacy/stage_tools/stage29_baseline_lock.py
  --mode "$MODE"
  --port "$PORT"
  --out-dir "$OUT_DIR"
  --min-sats "$MIN_SATS"
  --max-hdop "$MAX_HDOP"
  --manual-duration-s "$MANUAL_DURATION_S"
  --sensor-duration-s "$SENSOR_DURATION_S"
  --require-enter "$REQUIRE_ENTER"
)
if [[ -n "$MANUAL_LOG" ]]; then ARGS+=(--manual-log "$MANUAL_LOG"); fi
if [[ -n "$SENSOR_LOG" ]]; then ARGS+=(--sensor-log "$SENSOR_LOG"); fi
if [[ -n "$STATION_PULSE_LOG" ]]; then ARGS+=(--station-pulse-log "$STATION_PULSE_LOG"); fi
if [[ -n "$STATION_PULSE_CSV" ]]; then ARGS+=(--station-pulse-csv "$STATION_PULSE_CSV"); fi
if [[ -n "$COMBINED_LOG" ]]; then ARGS+=(--combined-log "$COMBINED_LOG"); fi

UV_NO_SYNC=1 uv run python "${ARGS[@]}"
UV_NO_SYNC=1 uv run python legacy/stage_tools/check_stage29_baseline_lock.py "$OUT_DIR"
