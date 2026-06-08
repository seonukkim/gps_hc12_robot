#!/usr/bin/env bash
# Deprecated. Use scripts/run_physical_path_planner.sh <mode>.
set -euo pipefail

PORT="/dev/ttyACM0"
MODE="forward"
CMD_LIST="0.08,0.12,0.16,0.20,0.25"
PULSE_MS_LIST="300,500,800"
FORWARD_SIGN="1.0"
TURN_SIGN="1.0"
REQUIRE_ENTER="true"
INTERACTIVE_VISIBLE_MOTION="true"
CONTINUE_AFTER_VISIBLE="false"
STOP_AFTER_BIG_TURN="true"
OUT_DIR="outputs/stage20_physical_ab_probe/latest"
MAX_ABS_A="0.25"
MAX_ABS_B="0.25"
MAX_MS="800"
DANGEROUSLY_ALLOW_HIGHER_CMD="false"
DANGEROUSLY_ALLOW_LONGER_PULSE="false"
TURN_PROFILE=""
IMU_ANGLE_COMPARE="false"
TARGET_ANGLE_DEG="90"
ANGLE_TOLERANCE_DEG="10"
SAVE_TURN_CALIBRATION="false"
TURN_CALIBRATION_OUT="outputs/stage23_turn_calibration/calibration/physical_ab_turn_angle_calibration.json"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --port)
      PORT="$2"
      shift 2
      ;;
    --mode)
      MODE="$2"
      shift 2
      ;;
    --turn-profile)
      TURN_PROFILE="$2"
      shift 2
      ;;
    --cmd-list)
      CMD_LIST="$2"
      shift 2
      ;;
    --pulse-ms-list)
      PULSE_MS_LIST="$2"
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
    --require-enter)
      REQUIRE_ENTER="$2"
      shift 2
      ;;
    --interactive-visible-motion)
      INTERACTIVE_VISIBLE_MOTION="$2"
      shift 2
      ;;
    --continue-after-visible)
      CONTINUE_AFTER_VISIBLE="$2"
      shift 2
      ;;
    --stop-after-big-turn)
      STOP_AFTER_BIG_TURN="$2"
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
    --dangerously-allow-higher-cmd)
      DANGEROUSLY_ALLOW_HIGHER_CMD="$2"
      shift 2
      ;;
    --dangerously-allow-longer-pulse)
      DANGEROUSLY_ALLOW_LONGER_PULSE="$2"
      shift 2
      ;;
    --out-dir)
      OUT_DIR="$2"
      shift 2
      ;;
    --imu-angle-compare)
      IMU_ANGLE_COMPARE="$2"
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
    --save-turn-calibration)
      SAVE_TURN_CALIBRATION="$2"
      shift 2
      ;;
    --turn-calibration-out)
      TURN_CALIBRATION_OUT="$2"
      shift 2
      ;;
    -h|--help)
      cat <<'USAGE'
Usage:
  legacy/stage_scripts/run_stage20_physical_ab_probe.sh \
    --port /dev/ttyACM0 \
    --mode forward \
    --turn-profile left_twitch \
    --cmd-list "0.08,0.12,0.16,0.20,0.25" \
    --pulse-ms-list "300,500,800" \
    --max-abs-a 0.25 \
    --max-abs-b 0.25 \
    --max-ms 800 \
    --forward-sign 1.0 \
    --turn-sign 1.0 \
    --stop-after-big-turn true \
    --require-enter true \
    --interactive-visible-motion true \
    --imu-angle-compare true \
    --target-angle-deg 90 \
    --angle-tolerance-deg 10 \
    --save-turn-calibration true \
    --turn-calibration-out outputs/stage23_turn_calibration/calibration/physical_ab_turn_angle_calibration.json \
    --out-dir outputs/stage20_physical_ab_probe/forward

Stage 20 sends direct physical A/B throttle/turn pulses through the
manual-equivalent guarded path. It is not full path following.

IMU angle compare wraps the same Stage20 command with before/after heartbeat yaw
measurement. It does not add an autonomous turn runner or candidate sweep.

Turn profiles:
  --turn-profile left_twitch   mode=turn_left, cmd-list 0.25..0.35, pulse-ms 700..1000
  --turn-profile right_twitch  mode=turn_right, cmd-list 0.12..0.32, pulse-ms 400..850
USAGE
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

case "$TURN_PROFILE" in
  "")
    ;;
  left_twitch)
    MODE="turn_left"
    CMD_LIST="0.25,0.28,0.30,0.32,0.35"
    PULSE_MS_LIST="700,850,1000"
    MAX_ABS_B="0.35"
    MAX_MS="1000"
    ;;
  right_twitch)
    MODE="turn_right"
    CMD_LIST="0.12,0.16,0.20,0.24,0.28,0.32"
    PULSE_MS_LIST="400,550,700,850"
    MAX_ABS_B="0.35"
    MAX_MS="1000"
    ;;
  *)
    echo "Unknown --turn-profile: ${TURN_PROFILE}" >&2
    exit 2
    ;;
esac

python3 - "$MODE" "$CMD_LIST" "$MAX_ABS_A" "$MAX_ABS_B" "$DANGEROUSLY_ALLOW_HIGHER_CMD" <<'PY'
import sys
mode = sys.argv[1]
max_abs_a = float(sys.argv[3])
max_abs_b = float(sys.argv[4])
danger = sys.argv[5].strip().lower() == "true"
cmd_limit = max(max_abs_a, max_abs_b)
if max_abs_a <= 0 or max_abs_b <= 0:
    print("ABORT: --max-abs-a/b must be > 0.", file=sys.stderr)
    raise SystemExit(2)
if cmd_limit > 0.35 and not danger:
    print("ABORT: --max-abs-a/b above 0.35 require --dangerously-allow-higher-cmd true.", file=sys.stderr)
    raise SystemExit(2)
if cmd_limit > 0.50:
    print("ABORT: --max-abs-a/b must be <= 0.50 even with dangerous override.", file=sys.stderr)
    raise SystemExit(2)
axis_modes = {
    "forward": ("A", max_abs_a),
    "backward": ("A", max_abs_a),
    "a_sweep": ("A", max_abs_a),
    "turn_left": ("B", max_abs_b),
    "turn_right": ("B", max_abs_b),
    "b_sweep": ("B", max_abs_b),
}
selected = ["forward", "backward", "turn_left", "turn_right", "a_sweep", "b_sweep"] if mode == "all" else [mode]
for raw in sys.argv[2].split(","):
    value = float(raw.strip())
    if value <= 0:
        print("ABORT: --cmd-list values must be > 0.", file=sys.stderr)
        raise SystemExit(2)
    for selected_mode in selected:
        axis, limit = axis_modes[selected_mode]
        if value > limit:
            print(
                f"ABORT: --cmd-list value {value} exceeds {axis}-axis max {limit} for mode {selected_mode}.",
                file=sys.stderr,
            )
            raise SystemExit(2)
PY

python3 - "$PULSE_MS_LIST" "$MAX_MS" "$DANGEROUSLY_ALLOW_LONGER_PULSE" <<'PY'
import sys
pulse_limit = int(float(sys.argv[2]))
danger = sys.argv[3].strip().lower() == "true"
if pulse_limit <= 0:
    print("ABORT: --max-ms must be > 0.", file=sys.stderr)
    raise SystemExit(2)
if pulse_limit > 1000 and not danger:
    print("ABORT: --max-ms above 1000 requires --dangerously-allow-longer-pulse true.", file=sys.stderr)
    raise SystemExit(2)
if pulse_limit > 1500:
    print("ABORT: --max-ms must be <= 1500 even with dangerous override.", file=sys.stderr)
    raise SystemExit(2)
for raw in sys.argv[1].split(","):
    value = int(float(raw.strip()))
    if value <= 0:
        print("ABORT: --pulse-ms-list values must be > 0.", file=sys.stderr)
        raise SystemExit(2)
    if value > pulse_limit:
        print(f"ABORT: --pulse-ms-list value {value} exceeds configured max {pulse_limit}.", file=sys.stderr)
        raise SystemExit(2)
PY

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"
mkdir -p "$OUT_DIR"

FQBN="OpenRB-150:samd:OpenRB-150"
BUILD_PATH="/private/tmp/openrb-stage20-physical-ab-probe"
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

# Turn-angle calibration needs live IMU yaw, so enable the BMI160 yaw diagnostics
# only when --imu-angle-compare is requested. These are motor-output-neutral and
# are not in the reject list below.
case "$(printf '%s' "$IMU_ANGLE_COMPARE" | tr '[:upper:]' '[:lower:]')" in
  true|1|yes)
    FLAGS="$FLAGS -DIMU_ENABLE=1 -DIMU_YAW_DIAG=1"
    echo "imu_angle_compare requested: appended -DIMU_ENABLE=1 -DIMU_YAW_DIAG=1"
    ;;
esac

if ! printf '%s' "$FLAGS" | grep -q 'STAGE20_PHYSICAL_AB_GUARDED_CRAWL=1'; then
  echo "ABORT: Stage 20 compile flag missing." >&2
  exit 2
fi
if printf '%s' "$FLAGS" | grep -Eq 'PHYSICAL_PATH_FOLLOWING_ENABLE=1|PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=1|PATH_FOLLOWING_DRYRUN=1|GROUND_CRAWL_TEST_MODE=1|AUTO_MOTION_ARMED=1|STAGE15_GUARDED_CRAWL_TEST=1|STAGE16_USB_GUARDED_CRAWL=1|STAGE17_FIRST_PRIMITIVE_CRAWL=1|STAGE18_MOTOR_MAPPING_PROBE=1'; then
  echo "ABORT: normal path-following, autonomous motion, or another stage flag present." >&2
  exit 2
fi

echo "STAGE20 PHYSICAL A/B GUARDED PROBE ONLY"
echo "NOT FULL PATH FOLLOWING"
echo "NO HC-12"
echo "NO PATH PACKAGE TARGET"
echo "A=physical throttle, B=physical turn"
echo "cmd_list=${CMD_LIST}"
echo "pulse_ms_list=${PULSE_MS_LIST}"
echo "turn_profile=${TURN_PROFILE}"
echo "max_abs_a=${MAX_ABS_A}"
echo "max_abs_b=${MAX_ABS_B}"
echo "max_ms=${MAX_MS}"
echo "forward_sign=${FORWARD_SIGN}"
echo "turn_sign=${TURN_SIGN}"
echo "stop_after_big_turn=${STOP_AFTER_BIG_TURN}"
echo "imu_angle_compare=${IMU_ANGLE_COMPARE}"
echo "target_angle_deg=${TARGET_ANGLE_DEG}"
echo "angle_tolerance_deg=${ANGLE_TOLERANCE_DEG}"
echo "save_turn_calibration=${SAVE_TURN_CALIBRATION}"
echo "turn_calibration_out=${TURN_CALIBRATION_OUT}"
echo "upload_port=${PORT}"
echo "flags=${FLAGS}"

arduino-cli compile --fqbn "$FQBN" --build-path "$BUILD_PATH" \
  --build-property "compiler.cpp.extra_flags=${FLAGS}" "$SKETCH"
arduino-cli upload -p "$PORT" --fqbn "$FQBN" --build-path "$BUILD_PATH" "$SKETCH"

POST_UPLOAD_PORT="$(arduino-cli board list 2>/dev/null | awk '/OpenRB-150/ {print $1; exit}')"
if [[ -z "$POST_UPLOAD_PORT" ]]; then
  POST_UPLOAD_PORT="$PORT"
fi
echo "post_upload_port=${POST_UPLOAD_PORT}"

uv run python tools/stage20_physical_ab_probe.py \
  --port "$POST_UPLOAD_PORT" \
  --mode "$MODE" \
  --cmd-list "$CMD_LIST" \
  --pulse-ms-list "$PULSE_MS_LIST" \
  --max-abs-a "$MAX_ABS_A" \
  --max-abs-b "$MAX_ABS_B" \
  --max-ms "$MAX_MS" \
  --forward-sign "$FORWARD_SIGN" \
  --turn-sign "$TURN_SIGN" \
  --stop-after-big-turn "$STOP_AFTER_BIG_TURN" \
  --imu-angle-compare "$IMU_ANGLE_COMPARE" \
  --target-angle-deg "$TARGET_ANGLE_DEG" \
  --angle-tolerance-deg "$ANGLE_TOLERANCE_DEG" \
  --save-turn-calibration "$SAVE_TURN_CALIBRATION" \
  --turn-calibration-out "$TURN_CALIBRATION_OUT" \
  --require-enter "$REQUIRE_ENTER" \
  --interactive-visible-motion "$INTERACTIVE_VISIBLE_MOTION" \
  --continue-after-visible "$CONTINUE_AFTER_VISIBLE" \
  --out-dir "$OUT_DIR"

uv run python tools/check_stage20_physical_ab_probe.py "$OUT_DIR" \
  --max-abs-a "$MAX_ABS_A" \
  --max-abs-b "$MAX_ABS_B" \
  --max-ms "$MAX_MS"
