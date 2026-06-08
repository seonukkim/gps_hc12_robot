#!/usr/bin/env bash
set -euo pipefail

PORT="/dev/ttyACM0"
DURATION_S="30"
OUT_DIR="outputs/manual_rc_passthrough_validation/latest"
UPLOAD="false"
LOG=""
DIAGNOSE_ONLY="false"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --port) PORT="$2"; shift 2 ;;
    --duration-s) DURATION_S="$2"; shift 2 ;;
    --out-dir) OUT_DIR="$2"; shift 2 ;;
    --upload) UPLOAD="$2"; shift 2 ;;
    --log) LOG="$2"; shift 2 ;;
    --diagnose-only) DIAGNOSE_ONLY="$2"; shift 2 ;;
    -h|--help)
      cat <<'USAGE'
Usage:
  scripts/run_manual_rc_passthrough_validation.sh \
    --port "$PORT" \
    --duration-s 30 \
    --out-dir outputs/manual_rc_passthrough_validation/latest

Telemetry-only manual RC validation. Keep AUTO OFF, transmitter ON, and move
sticks manually. No guarded pulse command is sent.
USAGE
      exit 0
      ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"
mkdir -p "$OUT_DIR"

FLAGS="-DMANUAL_RC_RECOVERY=1 \
-DMANUAL_FORWARD_SIGN=-1 \
-DMANUAL_TURN_SIGN=1 \
-DMOTOR_OUTPUT_SWAP_LR=0 \
-DDRIVE_CALIBRATION_ENABLE=0 \
-DPHYSICAL_PATH_FOLLOWING_ENABLE=0 \
-DPATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0 \
-DPATH_FOLLOWING_DRYRUN=0 \
-DPATH_FOLLOWING_HC12_ENABLED=0 \
-DGROUND_CRAWL_TEST_MODE=0 \
-DAUTO_MOTION_ARMED=0 \
-DSTAGE15_GUARDED_CRAWL_TEST=0 \
-DSTAGE20_PHYSICAL_AB_GUARDED_CRAWL=0 \
-DSTAGE16_USB_GUARDED_CRAWL=0 \
-DSTAGE17_FIRST_PRIMITIVE_CRAWL=0 \
-DSTAGE18_MOTOR_MAPPING_PROBE=0"

echo "MANUAL RC PASSTHROUGH VALIDATION"
echo "NO STATION MOTOR COMMANDS"
echo "upload_enabled=${UPLOAD}"
echo "diagnose_only=${DIAGNOSE_ONLY}"
echo "port=${PORT}"
echo "flags=${FLAGS}"

if [[ "$UPLOAD" == "true" ]]; then
  arduino-cli compile --fqbn OpenRB-150:samd:OpenRB-150 \
    --build-path /private/tmp/openrb-manual-rc-passthrough \
    --build-property "compiler.cpp.extra_flags=${FLAGS}" firmware/openrb_robot_controller
  arduino-cli upload -p "$PORT" --fqbn OpenRB-150:samd:OpenRB-150 \
    --build-path /private/tmp/openrb-manual-rc-passthrough firmware/openrb_robot_controller
fi

ARGS=(legacy/stage_tools/manual_rc_passthrough_validation.py --port "$PORT" --duration-s "$DURATION_S" --out-dir "$OUT_DIR")
ARGS+=(--diagnose-only "$DIAGNOSE_ONLY")
if [[ -n "$LOG" ]]; then
  ARGS+=(--log "$LOG")
fi
UV_NO_SYNC=1 uv run python "${ARGS[@]}"
UV_NO_SYNC=1 uv run python legacy/stage_tools/check_manual_rc_passthrough_validation.py "$OUT_DIR" || true
