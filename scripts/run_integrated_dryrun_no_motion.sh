#!/usr/bin/env bash
# Integrated GPS + IMU + (optional) HC-12 path-following DRY-RUN, motors disabled.
#
# This build computes path-following + IMU heading diagnostics over the USB
# monitor and CANNOT move motors: the physical-output compile gate is off
# (PHYSICAL_PATH_FOLLOWING_ENABLE=0 and PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0), so
# physical_block_reason stays COMPILE_GATE_OFF and final_left_cmd/final_right_cmd
# stay 0.000 in AUTO. RC MANUAL drive is unchanged. No motion is enabled.
#
# GPS UART: PATH_FOLLOWING_DRYRUN makes the controller read GPS on Serial2 (the
# center 5-pin connector, known-good at 9600) — NOT Serial3 D13/D14. HC-12 runs on
# Serial3 instead, so GPS (Serial2) and HC-12 (Serial3) do not collide.
#
# HC-12 is OPTIONAL here: the build sets hc12_enabled=true on Serial3, but the
# dry-run succeeds with no HC-12 RX (hc12_rx_count may stay 0). Mac-station HC-12
# RF validation is deferred to the Ubuntu Station (see docs/current_hardware_status.md).
#
# Usage: scripts/run_integrated_dryrun_no_motion.sh [-p PORT]

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"

FQBN="OpenRB-150:samd:OpenRB-150"
BUILD_PATH="/private/tmp/openrb-integrated-dryrun-hc12-deferred"
SKETCH="firmware/openrb_robot_controller"

# Motors are disabled by construction. Do NOT add the four motion gates here.
FLAGS="-DPATH_FOLLOWING_DRYRUN=1 \
-DIMU_ENABLE=1 \
-DIMU_HEADING_DRYRUN=1 \
-DIMU_YAW_DIAG=1 \
-DHEADING_SOURCE_MODE=2 \
-DPHYSICAL_PATH_FOLLOWING_ENABLE=0 \
-DPATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0 \
-DPATH_FOLLOWING_HC12_SERIAL_PORT=3"

# Safety guard: refuse to run if any motion-enabling flag leaked into FLAGS.
if printf '%s' "$FLAGS" | grep -Eq 'PHYSICAL_PATH_FOLLOWING_ENABLE=1|PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=1|AUTO_MOTION_ARMED=1|GROUND_CRAWL_TEST_MODE=1'; then
  echo "ABORT: a motion-enabling flag is present; this script is no-motion only." >&2
  exit 2
fi

PORT="${1:-}"
if [ -n "${PORT}" ] && [ "${PORT}" = "-p" ]; then PORT="${2:-}"; fi
if [ -z "${PORT}" ]; then
  PORT="$(arduino-cli board list 2>/dev/null | awk '/OpenRB-150/ {print $1; exit}')"
fi
if [ -z "${PORT}" ]; then
  echo "ABORT: OpenRB-150 port not found. Plug in the board or pass -p /dev/cu.usbmodemXXXX." >&2
  exit 1
fi

mkdir -p outputs/logs
TS="$(date +%Y%m%d_%H%M%S)"
LOG="outputs/logs/integrated_dryrun_hc12_deferred_${TS}.log"

echo "port=${PORT}"
echo "flags=${FLAGS}"
echo "log=${LOG}"
echo "GPS: Serial2 (center 5-pin, 9600). HC-12: optional (Serial3). motors: DISABLED (compile gate off)."

arduino-cli compile --fqbn "$FQBN" --build-path "$BUILD_PATH" \
  --build-property "compiler.cpp.extra_flags=${FLAGS}" "$SKETCH"
arduino-cli upload -p "$PORT" --fqbn "$FQBN" --build-path "$BUILD_PATH" "$SKETCH"
sleep 2

echo "Monitoring ${PORT} @ 115200 -> ${LOG}  (Ctrl-C to stop)"
arduino-cli monitor -p "$PORT" --config baudrate=115200 | tee "$LOG"

echo
echo "Done. Inspect with: scripts/check_integrated_dryrun_log.sh ${LOG}"
