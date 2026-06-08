#!/usr/bin/env bash
# Deprecated. Use scripts/run_physical_path_planner.sh <mode>.
set -euo pipefail

PORT="/dev/ttyACM0"
MODE="all"
CMD_LIST="0.04,0.06,0.08"
PULSE_MS_LIST="300,500,800"
REQUIRE_ENTER="true"
INTERACTIVE_VISIBLE_MOTION="true"
CONTINUE_AFTER_VISIBLE="false"
OUT_DIR="outputs/stage18_motor_mapping_probe/latest"

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
    --cmd-list)
      CMD_LIST="$2"
      shift 2
      ;;
    --pulse-ms-list)
      PULSE_MS_LIST="$2"
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
    --out-dir)
      OUT_DIR="$2"
      shift 2
      ;;
    -h|--help)
      cat <<'USAGE'
Usage:
  scripts/run_stage18_motor_mapping_probe.sh \
    --port /dev/ttyACM0 \
    --mode all \
    --cmd-list "0.04,0.06,0.08" \
    --pulse-ms-list "300,500,800" \
    --require-enter true \
    --interactive-visible-motion true \
    --out-dir outputs/stage18_motor_mapping_probe/latest

Stage 18 is bounded motor mapping and power diagnostics only. It is not full
path following, does not use path_package targets, and does not use HC-12.
USAGE
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

python3 - "$CMD_LIST" <<'PY'
import sys
for raw in sys.argv[1].split(","):
    value = float(raw.strip())
    if value <= 0 or value > 0.08:
        print("ABORT: --cmd-list values must be > 0 and <= 0.080.", file=sys.stderr)
        raise SystemExit(2)
PY

python3 - "$PULSE_MS_LIST" <<'PY'
import sys
for raw in sys.argv[1].split(","):
    value = int(float(raw.strip()))
    if value <= 0 or value > 800:
        print("ABORT: --pulse-ms-list values must be > 0 and <= 800.", file=sys.stderr)
        raise SystemExit(2)
PY

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"
mkdir -p "$OUT_DIR"

FQBN="OpenRB-150:samd:OpenRB-150"
BUILD_PATH="/private/tmp/openrb-stage18-motor-mapping-probe"
SKETCH="firmware/openrb_robot_controller"
FLAGS="-DSTAGE18_MOTOR_MAPPING_PROBE=1 \
-DSTAGE18_MAX_CMD=0.08 \
-DSTAGE18_MAX_MS=800 \
-DPHYSICAL_PATH_FOLLOWING_ENABLE=0 \
-DPATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0 \
-DPATH_FOLLOWING_DRYRUN=0 \
-DGROUND_CRAWL_TEST_MODE=0 \
-DAUTO_MOTION_ARMED=0 \
-DSTAGE15_GUARDED_CRAWL_TEST=0 \
-DSTAGE16_USB_GUARDED_CRAWL=0 \
-DSTAGE17_FIRST_PRIMITIVE_CRAWL=0"

if ! printf '%s' "$FLAGS" | grep -q 'STAGE18_MOTOR_MAPPING_PROBE=1'; then
  echo "ABORT: Stage 18 compile flag missing." >&2
  exit 2
fi
if printf '%s' "$FLAGS" | grep -Eq 'PHYSICAL_PATH_FOLLOWING_ENABLE=1|PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=1|PATH_FOLLOWING_DRYRUN=1|GROUND_CRAWL_TEST_MODE=1|AUTO_MOTION_ARMED=1|STAGE16_USB_GUARDED_CRAWL=1|STAGE17_FIRST_PRIMITIVE_CRAWL=1'; then
  echo "ABORT: normal path-following, autonomous motion, or other stage flag present." >&2
  exit 2
fi

echo "STAGE18 MOTOR MAPPING PROBE ONLY"
echo "NOT FULL PATH FOLLOWING"
echo "NO HC-12"
echo "NO PATH PACKAGE TARGET"
echo "cmd_list=${CMD_LIST}"
echo "pulse_ms_list=${PULSE_MS_LIST}"
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

uv run python tools/stage18_motor_mapping_probe.py \
  --port "$POST_UPLOAD_PORT" \
  --mode "$MODE" \
  --cmd-list "$CMD_LIST" \
  --pulse-ms-list "$PULSE_MS_LIST" \
  --require-enter "$REQUIRE_ENTER" \
  --interactive-visible-motion "$INTERACTIVE_VISIBLE_MOTION" \
  --continue-after-visible "$CONTINUE_AFTER_VISIBLE" \
  --out-dir "$OUT_DIR"

uv run python tools/check_stage18_motor_mapping_probe.py "$OUT_DIR"
