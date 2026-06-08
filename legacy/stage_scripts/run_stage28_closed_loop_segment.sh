#!/usr/bin/env bash
# Deprecated. Use scripts/run_physical_path_planner.sh <mode>.
set -euo pipefail

PORT="/dev/ttyACM0"
PATH_PACKAGE="latest"
FINE_CALIBRATION_JSON="outputs/stage20_physical_ab_probe/calibration/physical_ab_fine_motion_calibration.json"
TURN_CALIBRATION_JSON="outputs/stage23_turn_calibration/calibration/physical_ab_turn_twitch_calibration.json"
MODE="dry_run"
SEGMENT_INDEX="0"
DIRECTION="left"
TARGET_ANGLE_DEG="90"
ANGLE_TOLERANCE_DEG="15"
MAX_PULSES="5"
PULSE_MS=""
MAX_CORRECTION_B="0.12"
REVERSE_TURN_CORRECTION_FOR_BACKWARD="true"
CLOSED_LOOP_MODE="GPS_IMU_CLOSED_LOOP"
ALLOW_IMU_ONLY="false"
IMU_CALIBRATION_TIMEOUT_S="30"
MANUAL_RC_SUMMARY=""
GPS_WATCH_SUMMARY=""
IMU_READINESS_SUMMARY=""
STAGE26_SUMMARY=""
OVERRIDE_RECOVERY_GATES="false"
REQUIRE_ENTER="true"
REQUIRE_ENTER_EACH_PULSE="true"
INTERACTIVE_VISIBLE_MOTION="true"
OUT_DIR="outputs/stage28_closed_loop/latest"
SKIP_UPLOAD="false"
PORT_WAS_PROVIDED="false"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --port)
      PORT="$2"
      PORT_WAS_PROVIDED="true"
      shift 2
      ;;
    --path-package)
      PATH_PACKAGE="$2"
      shift 2
      ;;
    --fine-calibration-json)
      FINE_CALIBRATION_JSON="$2"
      shift 2
      ;;
    --turn-calibration-json)
      TURN_CALIBRATION_JSON="$2"
      shift 2
      ;;
    --mode)
      MODE="$2"
      shift 2
      ;;
    --segment-index)
      SEGMENT_INDEX="$2"
      shift 2
      ;;
    --direction)
      DIRECTION="$2"
      shift 2
      ;;
    --target-angle-deg)
      TARGET_ANGLE_DEG="$2"
      shift 2
      ;;
    --angle-tolerance-deg)
      ANGLE_TOLERANCE_DEG="$2"
      shift 2
      ;;
    --max-pulses)
      MAX_PULSES="$2"
      shift 2
      ;;
    --pulse-ms)
      PULSE_MS="$2"
      shift 2
      ;;
    --max-correction-b)
      MAX_CORRECTION_B="$2"
      shift 2
      ;;
    --reverse-turn-correction-for-backward)
      REVERSE_TURN_CORRECTION_FOR_BACKWARD="$2"
      shift 2
      ;;
    --closed-loop-mode)
      CLOSED_LOOP_MODE="$2"
      shift 2
      ;;
    --allow-imu-only)
      ALLOW_IMU_ONLY="$2"
      shift 2
      ;;
    --imu-calibration-timeout-s)
      IMU_CALIBRATION_TIMEOUT_S="$2"
      shift 2
      ;;
    --manual-rc-summary)
      MANUAL_RC_SUMMARY="$2"
      shift 2
      ;;
    --gps-watch-summary)
      GPS_WATCH_SUMMARY="$2"
      shift 2
      ;;
    --imu-readiness-summary)
      IMU_READINESS_SUMMARY="$2"
      shift 2
      ;;
    --stage26-summary)
      STAGE26_SUMMARY="$2"
      shift 2
      ;;
    --override-recovery-gates)
      OVERRIDE_RECOVERY_GATES="$2"
      shift 2
      ;;
    --require-enter)
      REQUIRE_ENTER="$2"
      shift 2
      ;;
    --require-enter-each-pulse)
      REQUIRE_ENTER_EACH_PULSE="$2"
      shift 2
      ;;
    --interactive-visible-motion)
      INTERACTIVE_VISIBLE_MOTION="$2"
      shift 2
      ;;
    --skip-upload|--no-upload)
      if [[ $# -ge 2 && "$2" != --* ]]; then
        SKIP_UPLOAD="$2"
        shift 2
      else
        SKIP_UPLOAD="true"
        shift
      fi
      ;;
    --out-dir)
      OUT_DIR="$2"
      shift 2
      ;;
    -h|--help)
      cat <<'USAGE'
Usage:
  legacy/stage_scripts/run_stage28_closed_loop_segment.sh \
    --port /dev/ttyACM0 \
    --path-package outputs/field_ab_serpentine_georef/latest/path_package.json \
    --fine-calibration-json outputs/stage20_physical_ab_probe/calibration/physical_ab_fine_motion_calibration.json \
    --turn-calibration-json outputs/stage23_turn_calibration/calibration/physical_ab_turn_twitch_calibration.json \
    --mode correction_forward_segment \
    --segment-index 0 \
    --max-pulses 5 \
    --pulse-ms 250 \
    --max-correction-b 0.12 \
    --require-enter true \
    --require-enter-each-pulse true \
    --out-dir outputs/stage28_closed_loop/forward_segment_0

Stage28 is USB-supervised closed-loop segment testing. It uses GPS/IMU feedback
station-side and sends only bounded Stage20 physical A/B pulses. It is not full
path following and does not use HC-12.
USAGE
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

python3 - "$PULSE_MS" "$MAX_PULSES" "$MAX_CORRECTION_B" <<'PY'
import sys
pulse_text = sys.argv[1]
pulse_ms = None if pulse_text == "" else int(float(pulse_text))
max_pulses = int(float(sys.argv[2]))
max_correction_b = float(sys.argv[3])
if pulse_ms is not None and (pulse_ms <= 0 or pulse_ms > 1000):
    print("ABORT: --pulse-ms must be > 0 and <= 1000 for Stage28.", file=sys.stderr)
    raise SystemExit(2)
if max_pulses <= 0:
    print("ABORT: --max-pulses must be > 0.", file=sys.stderr)
    raise SystemExit(2)
if max_correction_b < 0 or max_correction_b > 0.35:
    print("ABORT: --max-correction-b must be between 0 and 0.35.", file=sys.stderr)
    raise SystemExit(2)
PY

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"
mkdir -p "$OUT_DIR"

UPLOAD_ENABLED="true"
if [[ "$SKIP_UPLOAD" == "true" || "$MODE" == "dry_run" ]]; then
  UPLOAD_ENABLED="false"
fi

OPENRB_PORT_DETECTED="$(arduino-cli board list 2>/dev/null | awk '/OpenRB-150/ {print $1; exit}')"
HC12_PORTS_IGNORED="$(arduino-cli board list 2>/dev/null | awk '/usbserial|CP210|CP2104|HC-12/ {print $1}' | paste -sd, -)"
if [[ -z "$HC12_PORTS_IGNORED" ]]; then
  HC12_PORTS_IGNORED="none"
fi

FQBN="OpenRB-150:samd:OpenRB-150"
BUILD_PATH="/private/tmp/openrb-stage28-closed-loop-segment"
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

if ! printf '%s' "$FLAGS" | grep -q 'STAGE20_PHYSICAL_AB_GUARDED_CRAWL=1'; then
  echo "ABORT: Stage20 guarded transport flag missing." >&2
  exit 2
fi
if printf '%s' "$FLAGS" | grep -Eq 'PHYSICAL_PATH_FOLLOWING_ENABLE=1|PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=1|PATH_FOLLOWING_DRYRUN=1|PATH_FOLLOWING_HC12_ENABLED=1|GROUND_CRAWL_TEST_MODE=1|AUTO_MOTION_ARMED=1'; then
  echo "ABORT: full path following, HC-12, or autonomous motion flag present." >&2
  exit 2
fi

echo "STAGE28 SUPERVISED CLOSED-LOOP SEGMENT ONLY"
echo "NOT FULL PATH FOLLOWING"
echo "NO HC-12"
echo "mode=${MODE}"
echo "path_package=${PATH_PACKAGE}"
echo "pulse_ms=${PULSE_MS:-calibrated_or_correction_default}"
echo "max_pulses=${MAX_PULSES}"
echo "max_correction_b=${MAX_CORRECTION_B}"
echo "closed_loop_mode=${CLOSED_LOOP_MODE}"
echo "allow_imu_only=${ALLOW_IMU_ONLY}"
echo "override_recovery_gates=${OVERRIDE_RECOVERY_GATES}"
echo "upload_enabled=${UPLOAD_ENABLED}"
echo "upload_port=${PORT}"
echo "openrb_port_detected=${OPENRB_PORT_DETECTED}"
echo "hc12_ports_ignored=${HC12_PORTS_IGNORED}"
echo "flags=${FLAGS}"

if [[ "$UPLOAD_ENABLED" == "true" ]]; then
  arduino-cli compile --fqbn "$FQBN" --build-path "$BUILD_PATH" \
    --build-property "compiler.cpp.extra_flags=${FLAGS}" "$SKETCH"
  arduino-cli upload -p "$PORT" --fqbn "$FQBN" --build-path "$BUILD_PATH" "$SKETCH"

  POST_UPLOAD_PORT="$(arduino-cli board list 2>/dev/null | awk '/OpenRB-150/ {print $1; exit}')"
  if [[ -z "$POST_UPLOAD_PORT" ]]; then
    POST_UPLOAD_PORT="$PORT"
  fi
else
  echo "skip_upload=true"
  if [[ "$PORT_WAS_PROVIDED" == "false" && -n "$OPENRB_PORT_DETECTED" ]]; then
    POST_UPLOAD_PORT="$OPENRB_PORT_DETECTED"
  else
    POST_UPLOAD_PORT="$PORT"
  fi
fi
echo "command_port=${POST_UPLOAD_PORT}"

PY_ARGS=(
  legacy/stage_tools/stage28_closed_loop_segment.py
  --port "$POST_UPLOAD_PORT" \
  --path-package "$PATH_PACKAGE" \
  --fine-calibration-json "$FINE_CALIBRATION_JSON" \
  --turn-calibration-json "$TURN_CALIBRATION_JSON" \
  --mode "$MODE" \
  --segment-index "$SEGMENT_INDEX" \
  --direction "$DIRECTION" \
  --target-angle-deg "$TARGET_ANGLE_DEG" \
  --angle-tolerance-deg "$ANGLE_TOLERANCE_DEG" \
  --max-pulses "$MAX_PULSES" \
  --max-correction-b "$MAX_CORRECTION_B" \
  --reverse-turn-correction-for-backward "$REVERSE_TURN_CORRECTION_FOR_BACKWARD" \
  --closed-loop-mode "$CLOSED_LOOP_MODE" \
  --allow-imu-only "$ALLOW_IMU_ONLY" \
  --imu-calibration-timeout-s "$IMU_CALIBRATION_TIMEOUT_S" \
  --override-recovery-gates "$OVERRIDE_RECOVERY_GATES" \
  --require-enter "$REQUIRE_ENTER" \
  --require-enter-each-pulse "$REQUIRE_ENTER_EACH_PULSE" \
  --interactive-visible-motion "$INTERACTIVE_VISIBLE_MOTION" \
  --out-dir "$OUT_DIR"
)
if [[ -n "$PULSE_MS" ]]; then
  PY_ARGS+=(--pulse-ms "$PULSE_MS")
fi
if [[ -n "$MANUAL_RC_SUMMARY" ]]; then
  PY_ARGS+=(--manual-rc-summary "$MANUAL_RC_SUMMARY")
fi
if [[ -n "$GPS_WATCH_SUMMARY" ]]; then
  PY_ARGS+=(--gps-watch-summary "$GPS_WATCH_SUMMARY")
fi
if [[ -n "$IMU_READINESS_SUMMARY" ]]; then
  PY_ARGS+=(--imu-readiness-summary "$IMU_READINESS_SUMMARY")
fi
if [[ -n "$STAGE26_SUMMARY" ]]; then
  PY_ARGS+=(--stage26-summary "$STAGE26_SUMMARY")
fi

UV_NO_SYNC=1 uv run python "${PY_ARGS[@]}"

if [[ "$MODE" != "dry_run" ]]; then
  UV_NO_SYNC=1 uv run python legacy/stage_tools/check_stage28_closed_loop_segment.py "$OUT_DIR"
fi
