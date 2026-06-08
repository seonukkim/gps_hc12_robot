#!/usr/bin/env bash
# Deprecated. Use scripts/run_physical_path_planner.sh <mode>.
set -euo pipefail

PORT="/dev/ttyACM0"
FINE_CALIBRATION_JSON="outputs/stage20_physical_ab_probe/calibration/physical_ab_fine_motion_calibration.json"
TURN_CALIBRATION_JSON="outputs/stage23_turn_calibration/calibration/physical_ab_turn_twitch_calibration.json"
SEQUENCE="forward,turn_left,backward,turn_right"
REQUIRE_ENTER="true"
INTERACTIVE_VISIBLE_MOTION="true"
OUT_DIR="outputs/stage26_calibrated_primitive_sequence/latest"
SKIP_UPLOAD="false"
DRY_RUN="false"
MAX_ABS_A="0.35"
MAX_ABS_B="0.35"
MAX_MS="1000"
PORT_WAS_PROVIDED="false"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --port)
      PORT="$2"
      PORT_WAS_PROVIDED="true"
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
    --sequence)
      SEQUENCE="$2"
      shift 2
      ;;
    --require-enter)
      REQUIRE_ENTER="$2"
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
    --dry-run)
      DRY_RUN="$2"
      shift 2
      ;;
    --max-abs-a)
      MAX_ABS_A="$2"
      shift 2
      ;;
    --max-abs-b)
      MAX_ABS_B="$2"
      shift 2
      ;;
    --max-ms)
      MAX_MS="$2"
      shift 2
      ;;
    --out-dir)
      OUT_DIR="$2"
      shift 2
      ;;
    -h|--help)
      cat <<'USAGE'
Usage:
  legacy/stage_scripts/run_stage26_calibrated_primitive_sequence.sh \
    --port /dev/ttyACM0 \
    --fine-calibration-json outputs/stage20_physical_ab_probe/calibration/physical_ab_fine_motion_calibration.json \
    --turn-calibration-json outputs/stage23_turn_calibration/calibration/physical_ab_turn_twitch_calibration.json \
    --sequence "forward,turn_left,backward,turn_right" \
    --require-enter true \
    --skip-upload true \
    --out-dir outputs/stage26_calibrated_primitive_sequence/latest

Stage 26 runs a bounded calibrated primitive sequence through Stage20 physical
A/B guarded transport. It is not full path following, not serpentine execution,
and not HC-12 control.
USAGE
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

python3 - "$MAX_ABS_A" "$MAX_ABS_B" "$MAX_MS" <<'PY'
import sys
max_a = float(sys.argv[1])
max_b = float(sys.argv[2])
max_ms = int(float(sys.argv[3]))
if max_a <= 0 or max_a > 0.35:
    print("ABORT: --max-abs-a must be > 0 and <= 0.35.", file=sys.stderr)
    raise SystemExit(2)
if max_b <= 0 or max_b > 0.35:
    print("ABORT: --max-abs-b must be > 0 and <= 0.35.", file=sys.stderr)
    raise SystemExit(2)
if max_ms <= 0 or max_ms > 1000:
    print("ABORT: --max-ms must be > 0 and <= 1000.", file=sys.stderr)
    raise SystemExit(2)
PY

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"
mkdir -p "$OUT_DIR"

UPLOAD_ENABLED="true"
if [[ "$SKIP_UPLOAD" == "true" || "$DRY_RUN" == "true" ]]; then
  UPLOAD_ENABLED="false"
fi

OPENRB_PORT_DETECTED="$(arduino-cli board list 2>/dev/null | awk '/OpenRB-150/ {print $1; exit}')"
HC12_PORTS_IGNORED="$(arduino-cli board list 2>/dev/null | awk '/usbserial|CP210|CP2104|HC-12/ {print $1}' | paste -sd, -)"
if [[ -z "$HC12_PORTS_IGNORED" ]]; then
  HC12_PORTS_IGNORED="none"
fi

FQBN="OpenRB-150:samd:OpenRB-150"
BUILD_PATH="/private/tmp/openrb-stage26-calibrated-primitive-sequence"
SKETCH="firmware/openrb_robot_controller"
FLAGS="-DSTAGE20_PHYSICAL_AB_GUARDED_CRAWL=1 \
-DSTAGE20_MAX_ABS_A=${MAX_ABS_A} \
-DSTAGE20_MAX_ABS_B=${MAX_ABS_B} \
-DSTAGE20_MAX_MS=${MAX_MS} \
-DPHYSICAL_PATH_FOLLOWING_ENABLE=0 \
-DPATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0 \
-DPATH_FOLLOWING_DRYRUN=0 \
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
if printf '%s' "$FLAGS" | grep -Eq 'PHYSICAL_PATH_FOLLOWING_ENABLE=1|PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=1|PATH_FOLLOWING_DRYRUN=1|GROUND_CRAWL_TEST_MODE=1|AUTO_MOTION_ARMED=1|STAGE15_GUARDED_CRAWL_TEST=1|STAGE16_USB_GUARDED_CRAWL=1|STAGE17_FIRST_PRIMITIVE_CRAWL=1|STAGE18_MOTOR_MAPPING_PROBE=1'; then
  echo "ABORT: normal path-following, autonomous motion, or another stage flag present." >&2
  exit 2
fi

echo "STAGE26 CALIBRATED PRIMITIVE SEQUENCE ONLY"
echo "NOT FULL PATH FOLLOWING"
echo "NO HC-12"
echo "NO SERPENTINE EXECUTION"
echo "sequence=${SEQUENCE}"
echo "fine_calibration_json=${FINE_CALIBRATION_JSON}"
echo "turn_calibration_json=${TURN_CALIBRATION_JSON}"
echo "dry_run=${DRY_RUN}"
echo "upload_enabled=${UPLOAD_ENABLED}"
echo "upload_port=${PORT}"
echo "openrb_port_detected=${OPENRB_PORT_DETECTED}"
echo "hc12_ports_ignored=${HC12_PORTS_IGNORED}"
echo "max_abs_a=${MAX_ABS_A}"
echo "max_abs_b=${MAX_ABS_B}"
echo "max_ms=${MAX_MS}"
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

uv run python legacy/stage_tools/stage26_calibrated_primitive_sequence.py \
  --port "$POST_UPLOAD_PORT" \
  --fine-calibration-json "$FINE_CALIBRATION_JSON" \
  --turn-calibration-json "$TURN_CALIBRATION_JSON" \
  --sequence "$SEQUENCE" \
  --require-enter "$REQUIRE_ENTER" \
  --interactive-visible-motion "$INTERACTIVE_VISIBLE_MOTION" \
  --dry-run "$DRY_RUN" \
  --max-abs-a "$MAX_ABS_A" \
  --max-abs-b "$MAX_ABS_B" \
  --max-ms "$MAX_MS" \
  --out-dir "$OUT_DIR"

if [[ "$DRY_RUN" != "true" ]]; then
  uv run python legacy/stage_tools/check_stage26_calibrated_primitive_sequence.py "$OUT_DIR"
fi
