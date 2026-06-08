#!/usr/bin/env bash
set -euo pipefail

PORT="/dev/ttyACM0"
MODE_CHANNEL_INDEX="4"
STEER_CHANNEL_INDEX="0"
THROTTLE_CHANNEL_INDEX="1"
RC_INPUT_MODE="old_known_good"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --port) PORT="$2"; shift 2 ;;
    --rc-input-mode) RC_INPUT_MODE="$2"; shift 2 ;;
    --mode-channel-index) MODE_CHANNEL_INDEX="$2"; shift 2 ;;
    --steer-channel-index) STEER_CHANNEL_INDEX="$2"; shift 2 ;;
    --throttle-channel-index) THROTTLE_CHANNEL_INDEX="$2"; shift 2 ;;
    -h|--help)
      cat <<'USAGE'
Usage:
  scripts/upload_manual_rc_recovery_firmware.sh --port "$PORT"

Uploads the safe manual-RC recovery build. Full path following, HC-12 path
commands, and autonomous motion gates are disabled.
USAGE
      exit 0
      ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"

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
-DSTAGE18_MOTOR_MAPPING_PROBE=0 \
-DMODE_CHANNEL_INDEX=${MODE_CHANNEL_INDEX}"

echo "UPLOAD MANUAL RC RECOVERY FIRMWARE"
echo "FULL PATH FOLLOWING DISABLED"
echo "HC-12 PATH COMMANDS DISABLED"
echo "port=${PORT}"
echo "rc_input_mode=${RC_INPUT_MODE}"
echo "mode_channel_index=${MODE_CHANNEL_INDEX}"
echo "steer_channel_index=${STEER_CHANNEL_INDEX}"
echo "throttle_channel_index=${THROTTLE_CHANNEL_INDEX}"
echo "old_known_good_ppm_mapping=true"
echo "flags=${FLAGS}"

if [[ "${RC_INPUT_MODE}" != "old_known_good" && "${RC_INPUT_MODE}" != "auto" && "${RC_INPUT_MODE}" != "ppm" ]]; then
  echo "WARNING: RC_INPUT_MODE_FLAG_NOT_IMPLEMENTED requested=${RC_INPUT_MODE}; firmware still uses old known-good PPM input." >&2
fi
if [[ "${STEER_CHANNEL_INDEX}" != "0" || "${THROTTLE_CHANNEL_INDEX}" != "1" ]]; then
  echo "WARNING: RC_STEER_THROTTLE_CHANNEL_FLAGS_NOT_IMPLEMENTED; firmware still uses CH1 steering and CH2 throttle." >&2
fi

arduino-cli compile --fqbn OpenRB-150:samd:OpenRB-150 \
  --build-path /private/tmp/openrb-manual-rc-recovery \
  --build-property "compiler.cpp.extra_flags=${FLAGS}" firmware/openrb_robot_controller
arduino-cli upload -p "$PORT" --fqbn OpenRB-150:samd:OpenRB-150 \
  --build-path /private/tmp/openrb-manual-rc-recovery firmware/openrb_robot_controller
