#!/usr/bin/env bash
# Deprecated. Use scripts/run_physical_path_planner.sh <mode>.
set -euo pipefail

PORT="/dev/ttyACM0"
PATH_PACKAGE="latest"
POSE_MODE="gps_georef"
CURRENT_X=""
CURRENT_Y=""
CURRENT_HEADING_DEG=""
FORWARD_SIGN="1.0"
TURN_SIGN="1.0"
CALIBRATION_JSON=""
FINE_CALIBRATION_JSON=""
USE_RECOMMENDED_CRAWL_COMMAND="true"
STRAIGHT_ONLY="true"
TURN_DISABLED="true"
MIN_EFFECTIVE_A="0.08"
MIN_EFFECTIVE_B="0.08"
MAX_FORWARD_CMD=""
MAX_BACKWARD_CMD=""
MAX_TURN_LEFT_CMD=""
MAX_TURN_RIGHT_CMD=""
MAX_ABS_A="0.25"
MAX_ABS_B="0.25"
MAX_MS="800"
PULSE_MS=""
DURATION_S="30"
REQUIRE_ENTER="true"
DRY_RUN="true"
OUT_DIR="outputs/stage21_path_physical_ab_first_primitive/latest"
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
    --pose-mode)
      POSE_MODE="$2"
      shift 2
      ;;
    --current-x)
      CURRENT_X="$2"
      shift 2
      ;;
    --current-y)
      CURRENT_Y="$2"
      shift 2
      ;;
    --current-heading-deg)
      CURRENT_HEADING_DEG="$2"
      shift 2
      ;;
    --forward-sign)
      FORWARD_SIGN="$2"
      shift 2
      ;;
    --turn-sign)
      TURN_SIGN="$2"
      shift 2
      ;;
    --calibration-json)
      CALIBRATION_JSON="$2"
      shift 2
      ;;
    --fine-calibration-json)
      FINE_CALIBRATION_JSON="$2"
      shift 2
      ;;
    --use-recommended-crawl-command)
      USE_RECOMMENDED_CRAWL_COMMAND="$2"
      shift 2
      ;;
    --straight-only)
      STRAIGHT_ONLY="$2"
      shift 2
      ;;
    --turn-disabled)
      TURN_DISABLED="$2"
      shift 2
      ;;
    --min-effective-a)
      MIN_EFFECTIVE_A="$2"
      shift 2
      ;;
    --min-effective-b)
      MIN_EFFECTIVE_B="$2"
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
    --max-forward-cmd)
      MAX_FORWARD_CMD="$2"
      shift 2
      ;;
    --max-backward-cmd)
      MAX_BACKWARD_CMD="$2"
      shift 2
      ;;
    --max-turn-left-cmd)
      MAX_TURN_LEFT_CMD="$2"
      shift 2
      ;;
    --max-turn-right-cmd)
      MAX_TURN_RIGHT_CMD="$2"
      shift 2
      ;;
    --pulse-ms)
      PULSE_MS="$2"
      shift 2
      ;;
    --duration-s)
      DURATION_S="$2"
      shift 2
      ;;
    --require-enter)
      REQUIRE_ENTER="$2"
      shift 2
      ;;
    --dry-run)
      DRY_RUN="$2"
      shift 2
      ;;
    --out-dir)
      OUT_DIR="$2"
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
    -h|--help)
      cat <<'USAGE'
Usage:
  legacy/stage_scripts/run_stage21_path_physical_ab_first_primitive.sh \
    --port /dev/ttyACM0 \
    --path-package latest \
    --pose-mode manual_local \
    --current-x 0 --current-y 1.2 --current-heading-deg 0 \
    --dry-run true \
    --calibration-json outputs/stage20_physical_ab_probe/calibration/physical_ab_directional_calibration.json \
    --fine-calibration-json outputs/stage20_physical_ab_probe/calibration/physical_ab_fine_motion_calibration.json \
    --use-recommended-crawl-command true \
    --straight-only true \
    --turn-disabled true \
    --max-abs-a 0.25 \
    --max-abs-b 0.25 \
    --max-ms 800 \
    --skip-upload true \
    --out-dir outputs/stage21_path_physical_ab_first_primitive/latest

Stage 21 loads the path package, computes one first-primitive station target,
converts virtual forward/turn into physical A/B, and sends only one bounded
Stage20 pulse if --dry-run false. It is not full path following.
USAGE
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

python3 - "$MAX_ABS_A" "$MAX_ABS_B" "$MAX_MS" "$PULSE_MS" <<'PY'
import sys
max_a = float(sys.argv[1])
max_b = float(sys.argv[2])
max_ms = int(float(sys.argv[3]))
raw_pulse = sys.argv[4].strip()
if max_a <= 0 or max_b <= 0 or max_a > 0.35 or max_b > 0.35:
    print("ABORT: --max-abs-a/b must be > 0 and <= 0.35.", file=sys.stderr)
    raise SystemExit(2)
if max_ms <= 0 or max_ms > 1000:
    print("ABORT: --max-ms must be > 0 and <= 1000.", file=sys.stderr)
    raise SystemExit(2)
if raw_pulse:
    pulse_ms = int(float(raw_pulse))
    if pulse_ms <= 0 or pulse_ms > max_ms:
        print("ABORT: --pulse-ms must be > 0 and <= --max-ms.", file=sys.stderr)
        raise SystemExit(2)
PY

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"
mkdir -p "$OUT_DIR"
UPLOAD_ENABLED="true"
if [[ "$SKIP_UPLOAD" == "true" ]]; then
  UPLOAD_ENABLED="false"
fi
OPENRB_PORT_DETECTED="$(arduino-cli board list 2>/dev/null | awk '/OpenRB-150/ {print $1; exit}')"
HC12_PORTS_IGNORED="$(arduino-cli board list 2>/dev/null | awk '/usbserial|CP210|CP2104|HC-12/ {print $1}' | paste -sd, -)"
if [[ -z "$HC12_PORTS_IGNORED" ]]; then
  HC12_PORTS_IGNORED="none"
fi

FQBN="OpenRB-150:samd:OpenRB-150"
BUILD_PATH="/private/tmp/openrb-stage21-path-physical-ab-first-primitive"
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
  echo "ABORT: Stage 20 compile flag missing for Stage 21 transport." >&2
  exit 2
fi
if printf '%s' "$FLAGS" | grep -Eq 'PHYSICAL_PATH_FOLLOWING_ENABLE=1|PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=1|PATH_FOLLOWING_DRYRUN=1|GROUND_CRAWL_TEST_MODE=1|AUTO_MOTION_ARMED=1|STAGE15_GUARDED_CRAWL_TEST=1|STAGE16_USB_GUARDED_CRAWL=1|STAGE17_FIRST_PRIMITIVE_CRAWL=1|STAGE18_MOTOR_MAPPING_PROBE=1'; then
  echo "ABORT: normal path-following, autonomous motion, or another stage flag present." >&2
  exit 2
fi

echo "STAGE21 PATH PHYSICAL A/B FIRST PRIMITIVE ONLY"
echo "NOT FULL PATH FOLLOWING"
echo "ONE PRIMITIVE ONLY"
echo "NO HC-12"
echo "path_package=${PATH_PACKAGE}"
echo "pose_mode=${POSE_MODE}"
echo "dry_run=${DRY_RUN}"
echo "calibration_json=${CALIBRATION_JSON}"
echo "fine_calibration_json=${FINE_CALIBRATION_JSON}"
echo "use_recommended_crawl_command=${USE_RECOMMENDED_CRAWL_COMMAND}"
echo "straight_only=${STRAIGHT_ONLY}"
echo "turn_disabled=${TURN_DISABLED}"
echo "max_abs_a=${MAX_ABS_A}"
echo "max_abs_b=${MAX_ABS_B}"
echo "max_ms=${MAX_MS}"
echo "max_forward_cmd=${MAX_FORWARD_CMD}"
echo "max_backward_cmd=${MAX_BACKWARD_CMD}"
echo "max_turn_left_cmd=${MAX_TURN_LEFT_CMD}"
echo "max_turn_right_cmd=${MAX_TURN_RIGHT_CMD}"
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
echo "post_upload_port=${POST_UPLOAD_PORT}"

ARGS=(
  uv run python legacy/stage_tools/stage21_path_physical_ab_first_primitive.py
  --port "$POST_UPLOAD_PORT"
  --path-package "$PATH_PACKAGE"
  --pose-mode "$POSE_MODE"
  --forward-sign "$FORWARD_SIGN"
  --turn-sign "$TURN_SIGN"
  --straight-only "$STRAIGHT_ONLY"
  --turn-disabled "$TURN_DISABLED"
  --min-effective-a "$MIN_EFFECTIVE_A"
  --min-effective-b "$MIN_EFFECTIVE_B"
  --max-abs-a "$MAX_ABS_A"
  --max-abs-b "$MAX_ABS_B"
  --max-ms "$MAX_MS"
  --duration-s "$DURATION_S"
  --require-enter "$REQUIRE_ENTER"
  --dry-run "$DRY_RUN"
  --out-dir "$OUT_DIR"
)

if [[ -n "$CALIBRATION_JSON" ]]; then
  ARGS+=(--calibration-json "$CALIBRATION_JSON")
fi
if [[ -n "$FINE_CALIBRATION_JSON" ]]; then
  ARGS+=(--fine-calibration-json "$FINE_CALIBRATION_JSON")
fi
if [[ -n "$USE_RECOMMENDED_CRAWL_COMMAND" ]]; then
  ARGS+=(--use-recommended-crawl-command "$USE_RECOMMENDED_CRAWL_COMMAND")
fi
if [[ -n "$PULSE_MS" ]]; then
  ARGS+=(--pulse-ms "$PULSE_MS")
fi
if [[ -n "$CURRENT_X" ]]; then
  ARGS+=(--current-x "$CURRENT_X")
fi
if [[ -n "$CURRENT_Y" ]]; then
  ARGS+=(--current-y "$CURRENT_Y")
fi
if [[ -n "$CURRENT_HEADING_DEG" ]]; then
  ARGS+=(--current-heading-deg "$CURRENT_HEADING_DEG")
fi
if [[ -n "$MAX_FORWARD_CMD" ]]; then
  ARGS+=(--max-forward-cmd "$MAX_FORWARD_CMD")
fi
if [[ -n "$MAX_BACKWARD_CMD" ]]; then
  ARGS+=(--max-backward-cmd "$MAX_BACKWARD_CMD")
fi
if [[ -n "$MAX_TURN_LEFT_CMD" ]]; then
  ARGS+=(--max-turn-left-cmd "$MAX_TURN_LEFT_CMD")
fi
if [[ -n "$MAX_TURN_RIGHT_CMD" ]]; then
  ARGS+=(--max-turn-right-cmd "$MAX_TURN_RIGHT_CMD")
fi

"${ARGS[@]}"
