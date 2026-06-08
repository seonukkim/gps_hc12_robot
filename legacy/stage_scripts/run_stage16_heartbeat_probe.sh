#!/usr/bin/env bash
# Deprecated. Use scripts/run_physical_path_planner.sh <mode>.
set -euo pipefail

PORT="/dev/ttyACM0"
DURATION_S="15"
OUT_DIR="outputs/stage16_heartbeat_probe/latest"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --port)
      PORT="$2"
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
    -h|--help)
      cat <<'USAGE'
Usage:
  legacy/stage_scripts/run_stage16_heartbeat_probe.sh \
    --port /dev/ttyACM0 \
    [--duration-s 15] \
    [--out-dir outputs/stage16_heartbeat_probe/latest]

Compiles/uploads Stage 16 USB guarded firmware, monitors heartbeat only, and
does not send STAGE16_CMD motor pulses.
USAGE
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"
mkdir -p "$OUT_DIR"

FQBN="OpenRB-150:samd:OpenRB-150"
BUILD_PATH="/private/tmp/openrb-stage16-heartbeat-probe"
SKETCH="firmware/openrb_robot_controller"
FLAGS="-DSTAGE16_USB_GUARDED_CRAWL=1 \
-DSTAGE16_MAX_CMD=0.04 \
-DSTAGE16_MAX_MS=150 \
-DPHYSICAL_PATH_FOLLOWING_ENABLE=0 \
-DPATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0 \
-DPATH_FOLLOWING_DRYRUN=0 \
-DGROUND_CRAWL_TEST_MODE=0 \
-DAUTO_MOTION_ARMED=0 \
-DSTAGE15_GUARDED_CRAWL_TEST=0"

if printf '%s' "$FLAGS" | grep -Eq 'PHYSICAL_PATH_FOLLOWING_ENABLE=1|PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=1|PATH_FOLLOWING_DRYRUN=1|GROUND_CRAWL_TEST_MODE=1|AUTO_MOTION_ARMED=1'; then
  echo "ABORT: normal path-following or autonomous motion flag present." >&2
  exit 2
fi

echo "STAGE16 HEARTBEAT PROBE ONLY"
echo "No STAGE16_CMD pulses will be sent."
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

uv run python legacy/stage_tools/stage16_heartbeat_probe.py \
  --port "$POST_UPLOAD_PORT" \
  --duration-s "$DURATION_S" \
  --out-dir "$OUT_DIR"
