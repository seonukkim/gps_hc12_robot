#!/usr/bin/env bash
# Deprecated. Use scripts/run_physical_path_planner.sh <mode>.
set -euo pipefail

PORT="/dev/ttyACM0"
PATH_PACKAGE="latest"
POSE_MODE="gps_georef"
CURRENT_X=""
CURRENT_Y=""
CURRENT_HEADING_DEG=""
MAX_CMD="0.045"
PULSE_MS="300"
MAX_TOTAL_DISTANCE_M="0.20"
DURATION_S="30"
REQUIRE_ENTER="true"
DRY_RUN="true"
FORCE_STRAIGHT_TEST="false"
ALLOW_NEXT_PRIMITIVE="false"
OUT_DIR="outputs/stage17_first_primitive_crawl/latest"
DANGEROUSLY_ALLOW_HIGHER_CMD="false"
DANGEROUSLY_ALLOW_LONGER_PULSE="false"

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
    --max-cmd)
      MAX_CMD="$2"
      shift 2
      ;;
    --pulse-ms)
      PULSE_MS="$2"
      shift 2
      ;;
    --max-total-distance-m)
      MAX_TOTAL_DISTANCE_M="$2"
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
    --force-straight-test)
      FORCE_STRAIGHT_TEST="$2"
      shift 2
      ;;
    --allow-next-primitive)
      ALLOW_NEXT_PRIMITIVE="$2"
      shift 2
      ;;
    --out-dir)
      OUT_DIR="$2"
      shift 2
      ;;
    --dangerously-allow-higher-cmd)
      DANGEROUSLY_ALLOW_HIGHER_CMD="true"
      shift
      ;;
    --dangerously-allow-longer-pulse)
      DANGEROUSLY_ALLOW_LONGER_PULSE="true"
      shift
      ;;
    -h|--help)
      cat <<'USAGE'
Usage:
  scripts/run_stage17_first_primitive_crawl.sh \
    --port /dev/ttyACM0 \
    --path-package latest \
    --dry-run true \
    --pose-mode manual_local \
    --current-x 0 --current-y 1.2 --current-heading-deg 0 \
    --max-cmd 0.045 \
    --pulse-ms 300 \
    --max-total-distance-m 0.20 \
    --require-enter true \
    --out-dir outputs/stage17_first_primitive_crawl/latest

Stage 17 is a bounded first-primitive crawl only. It is not full path following,
does not run the serpentine, and does not use HC-12.
USAGE
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

python3 - "$MAX_CMD" "$DANGEROUSLY_ALLOW_HIGHER_CMD" <<'PY'
import sys
max_cmd = float(sys.argv[1])
allow = sys.argv[2].lower() == "true"
if max_cmd > 0.08:
    print("ABORT: --max-cmd > 0.080 is not allowed.", file=sys.stderr)
    raise SystemExit(2)
if max_cmd > 0.06 and not allow:
    print("ABORT: --max-cmd > 0.060 requires --dangerously-allow-higher-cmd.", file=sys.stderr)
    raise SystemExit(2)
PY

python3 - "$PULSE_MS" "$DANGEROUSLY_ALLOW_LONGER_PULSE" <<'PY'
import sys
pulse_ms = int(float(sys.argv[1]))
allow = sys.argv[2].lower() == "true"
if pulse_ms > 800:
    print("ABORT: --pulse-ms > 800 is not allowed.", file=sys.stderr)
    raise SystemExit(2)
if pulse_ms > 500 and not allow:
    print("ABORT: --pulse-ms > 500 requires --dangerously-allow-longer-pulse.", file=sys.stderr)
    raise SystemExit(2)
PY

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"
mkdir -p "$OUT_DIR"

FQBN="OpenRB-150:samd:OpenRB-150"
BUILD_PATH="/private/tmp/openrb-stage17-first-primitive-crawl"
SKETCH="firmware/openrb_robot_controller"
FLAGS="-DSTAGE17_FIRST_PRIMITIVE_CRAWL=1 \
-DSTAGE17_MAX_CMD=0.06 \
-DSTAGE17_MAX_MS=500 \
-DPHYSICAL_PATH_FOLLOWING_ENABLE=0 \
-DPATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0 \
-DPATH_FOLLOWING_DRYRUN=0 \
-DGROUND_CRAWL_TEST_MODE=0 \
-DAUTO_MOTION_ARMED=0 \
-DSTAGE15_GUARDED_CRAWL_TEST=0 \
-DSTAGE16_USB_GUARDED_CRAWL=0"

if ! printf '%s' "$FLAGS" | grep -q 'STAGE17_FIRST_PRIMITIVE_CRAWL=1'; then
  echo "ABORT: Stage 17 compile flag missing." >&2
  exit 2
fi
if printf '%s' "$FLAGS" | grep -Eq 'PHYSICAL_PATH_FOLLOWING_ENABLE=1|PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=1|PATH_FOLLOWING_DRYRUN=1|GROUND_CRAWL_TEST_MODE=1|AUTO_MOTION_ARMED=1|STAGE16_USB_GUARDED_CRAWL=1'; then
  echo "ABORT: normal path-following, autonomous motion, or Stage16 flag present." >&2
  exit 2
fi

echo "STAGE17 FIRST-PRIMITIVE CRAWL ONLY"
echo "NOT FULL PATH FOLLOWING"
echo "NO HC-12"
echo "MAX_CMD=${MAX_CMD}"
echo "PULSE_MS=${PULSE_MS}"
echo "path_package=${PATH_PACKAGE}"
echo "dry_run=${DRY_RUN}"
echo "force_straight_test=${FORCE_STRAIGHT_TEST}"
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

CMD=(
  uv run python tools/station_first_primitive_crawl.py
  --path-package "$PATH_PACKAGE"
  --port "$POST_UPLOAD_PORT"
  --pose-mode "$POSE_MODE"
  --max-cmd "$MAX_CMD"
  --pulse-ms "$PULSE_MS"
  --max-total-distance-m "$MAX_TOTAL_DISTANCE_M"
  --duration-s "$DURATION_S"
  --require-enter "$REQUIRE_ENTER"
  --dry-run "$DRY_RUN"
  --force-straight-test "$FORCE_STRAIGHT_TEST"
  --allow-next-primitive "$ALLOW_NEXT_PRIMITIVE"
  --out-dir "$OUT_DIR"
)
if [[ -n "$CURRENT_X" ]]; then
  CMD+=(--current-x "$CURRENT_X")
fi
if [[ -n "$CURRENT_Y" ]]; then
  CMD+=(--current-y "$CURRENT_Y")
fi
if [[ -n "$CURRENT_HEADING_DEG" ]]; then
  CMD+=(--current-heading-deg "$CURRENT_HEADING_DEG")
fi
"${CMD[@]}"

uv run python tools/check_stage17_first_primitive_crawl_log.py "$OUT_DIR" \
  --max-cmd "$MAX_CMD" \
  --max-ms "$PULSE_MS"
