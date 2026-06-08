#!/usr/bin/env bash
# Deprecated. Use scripts/run_physical_path_planner.sh <mode>.
set -euo pipefail

PORT="/dev/ttyACM0"
FINE_CALIBRATION_JSON="outputs/stage20_physical_ab_probe/calibration/physical_ab_fine_motion_calibration.json"
TURN_CALIBRATION_JSON="outputs/stage23_turn_calibration/calibration/physical_ab_turn_twitch_calibration.json"
SEQUENCE="forward,turn_left,backward,turn_right"
REQUIRE_ENTER="true"
INTERACTIVE_VISIBLE_MOTION="true"
DRY_RUN="false"
OUT_DIR="outputs/stage28_calibrated_motion_with_sensors/latest"
SKIP_UPLOAD="true"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --port) PORT="$2"; shift 2 ;;
    --fine-calibration-json) FINE_CALIBRATION_JSON="$2"; shift 2 ;;
    --turn-calibration-json) TURN_CALIBRATION_JSON="$2"; shift 2 ;;
    --sequence) SEQUENCE="$2"; shift 2 ;;
    --require-enter) REQUIRE_ENTER="$2"; shift 2 ;;
    --interactive-visible-motion) INTERACTIVE_VISIBLE_MOTION="$2"; shift 2 ;;
    --dry-run) DRY_RUN="$2"; shift 2 ;;
    --skip-upload|--no-upload)
      if [[ $# -ge 2 && "$2" != --* ]]; then
        SKIP_UPLOAD="$2"
        shift 2
      else
        SKIP_UPLOAD="true"
        shift
      fi
      ;;
    --out-dir) OUT_DIR="$2"; shift 2 ;;
    -h|--help)
      cat <<'USAGE'
Usage:
  legacy/stage_scripts/run_stage28_calibrated_motion_with_sensors.sh \
    --port "$PORT" \
    --fine-calibration-json outputs/stage20_physical_ab_probe/calibration/physical_ab_fine_motion_calibration.json \
    --turn-calibration-json outputs/stage23_turn_calibration/calibration/physical_ab_turn_twitch_calibration.json \
    --sequence "forward,turn_left,backward,turn_right" \
    --require-enter true \
    --out-dir outputs/stage28_calibrated_motion_with_sensors/latest
USAGE
      exit 0
      ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"
mkdir -p "$OUT_DIR"

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
-DAUTO_MOTION_ARMED=0"

echo "STAGE28C CALIBRATED MOTION WITH SENSOR LOGGING"
echo "NOT CLOSED LOOP"
echo "NOT FULL PATH FOLLOWING"
echo "NO HC-12"
echo "skip_upload=${SKIP_UPLOAD}"
echo "flags=${FLAGS}"

if [[ "$SKIP_UPLOAD" != "true" && "$DRY_RUN" != "true" ]]; then
  arduino-cli compile --fqbn OpenRB-150:samd:OpenRB-150 \
    --build-path /private/tmp/openrb-stage28-calibrated-sensors \
    --build-property "compiler.cpp.extra_flags=${FLAGS}" firmware/openrb_robot_controller
  arduino-cli upload -p "$PORT" --fqbn OpenRB-150:samd:OpenRB-150 \
    --build-path /private/tmp/openrb-stage28-calibrated-sensors firmware/openrb_robot_controller
fi

UV_NO_SYNC=1 uv run python legacy/stage_tools/stage28_calibrated_motion_with_sensors.py \
  --port "$PORT" \
  --fine-calibration-json "$FINE_CALIBRATION_JSON" \
  --turn-calibration-json "$TURN_CALIBRATION_JSON" \
  --sequence "$SEQUENCE" \
  --require-enter "$REQUIRE_ENTER" \
  --interactive-visible-motion "$INTERACTIVE_VISIBLE_MOTION" \
  --dry-run "$DRY_RUN" \
  --out-dir "$OUT_DIR"

if [[ "$DRY_RUN" != "true" ]]; then
  UV_NO_SYNC=1 uv run python legacy/stage_tools/check_stage28_calibrated_motion_with_sensors.py "$OUT_DIR" || true
fi
