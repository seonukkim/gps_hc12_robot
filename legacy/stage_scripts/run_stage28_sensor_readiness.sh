#!/usr/bin/env bash
# Deprecated. Use scripts/run_physical_path_planner.sh <mode>.
set -euo pipefail

PORT="/dev/ttyACM0"
PATH_PACKAGE="latest"
DURATION_S="30"
OUT_DIR="outputs/stage28_sensor_readiness/latest"
UPLOAD="false"
LOG=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --port) PORT="$2"; shift 2 ;;
    --path-package) PATH_PACKAGE="$2"; shift 2 ;;
    --duration-s) DURATION_S="$2"; shift 2 ;;
    --out-dir) OUT_DIR="$2"; shift 2 ;;
    --upload) UPLOAD="$2"; shift 2 ;;
    --log) LOG="$2"; shift 2 ;;
    -h|--help)
      cat <<'USAGE'
Usage:
  legacy/stage_scripts/run_stage28_sensor_readiness.sh \
    --port "$PORT" \
    --path-package outputs/field_ab_serpentine_georef/latest/path_package.json \
    --duration-s 30 \
    --out-dir outputs/stage28_sensor_readiness/latest

Stage28A monitors GPS, BMI160, RC, and motor safety only. It sends no motor
commands. Upload is disabled unless --upload true is explicitly provided.
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

echo "STAGE28A SENSOR READINESS ONLY"
echo "NO MOTOR COMMANDS"
echo "upload_enabled=${UPLOAD}"
echo "port=${PORT}"
echo "path_package=${PATH_PACKAGE}"
echo "duration_s=${DURATION_S}"
echo "flags=${FLAGS}"

if [[ "$UPLOAD" == "true" ]]; then
  arduino-cli compile --fqbn OpenRB-150:samd:OpenRB-150 \
    --build-path /private/tmp/openrb-stage28-sensor-readiness \
    --build-property "compiler.cpp.extra_flags=${FLAGS}" firmware/openrb_robot_controller
  arduino-cli upload -p "$PORT" --fqbn OpenRB-150:samd:OpenRB-150 \
    --build-path /private/tmp/openrb-stage28-sensor-readiness firmware/openrb_robot_controller
fi

ARGS=(legacy/stage_tools/stage28_sensor_readiness.py --port "$PORT" --path-package "$PATH_PACKAGE" --duration-s "$DURATION_S" --out-dir "$OUT_DIR")
if [[ -n "$LOG" ]]; then
  ARGS+=(--log "$LOG")
fi
UV_NO_SYNC=1 uv run python "${ARGS[@]}"
UV_NO_SYNC=1 uv run python legacy/stage_tools/check_stage28_sensor_readiness.py "$OUT_DIR" || true
