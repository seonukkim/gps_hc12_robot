#!/usr/bin/env bash
set -euo pipefail

# Unified launcher for the integrated physical path planner.
#
#   preview         no firmware, no serial -- pure geometry + render.
#   calibrate-turn  forwards to the Python CLI, which shells out to
#                   scripts/run_stage20_physical_ab_probe.sh (that script owns the
#                   IMU-flag compile + upload). No firmware logic here.
#   run / execute-plan
#                   flashes the STAGE20 guarded-crawl firmware (the same 4-flag
#                   safety gate as the Stage20 probe, plus IMU yaw diagnostics for
#                   heading-hold) then drives the continuous-motion controller.
#   diagnose        read-only telemetry; --from-log needs no serial.
#
# The firmware compile gate below is the real motor-output safety -- identical in
# spirit to scripts/run_stage20_physical_ab_probe.sh. This launcher never weakens
# it. --print-plan (run) and --from-log (diagnose) stay fully no-hardware.

MODE="${1:-}"
if [[ -z "$MODE" || "$MODE" == "-h" || "$MODE" == "--help" ]]; then
  cat <<'USAGE'
Usage:
  scripts/run_physical_path_planner.sh <mode> [options]

Modes:
  preview         Build + render the A->B serpentine (or direct) plan. No motion.
                  e.g. preview --start-lat 35.1 --start-lon 129.1 \
                       --goal-mode bearing_distance --goal-bearing-deg 90 \
                       --goal-distance-m 6 --workspace-width-m 2

  calibrate-turn  Measure a 90-degree turn via the Stage20 probe (IMU yaw).
                  e.g. calibrate-turn --port /dev/ttyACM0 --mode turn_left \
                       --save-turn-calibration true

  run             Drive the continuous-motion controller along the planned path.
  execute-plan    (alias of run)
                  e.g. run --start-lat 35.1 --start-lon 129.1 --goal-lat 35.1006 \
                       --goal-lon 129.1006 --workspace-width-m 2
                  Add --print-plan to build the plan without opening serial.

  diagnose        Read-only telemetry summary.
                  e.g. diagnose --from-log outputs/.../run_serial.log

All other options are passed straight through to
`python -m tools.physical_path_planning.cli <mode>` (see its --help).
USAGE
  exit 0
fi
shift || true

PORT="/dev/ttyACM0"
MAX_ABS_A="0.35"
MAX_ABS_B="0.35"
MAX_MS="1000"
PRINT_PLAN="false"
PASSTHRU=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --port)
      PORT="$2"
      PASSTHRU+=("--port" "$2")
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
    --print-plan)
      PRINT_PLAN="true"
      PASSTHRU+=("--print-plan")
      shift
      ;;
    *)
      PASSTHRU+=("$1")
      shift
      ;;
  esac
done

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"

flash_guarded_crawl_firmware() {
  local upload_port="$1"
  local fqbn="OpenRB-150:samd:OpenRB-150"
  local build_path="/private/tmp/openrb-physical-path-planner"
  local sketch="firmware/openrb_robot_controller"
  # Same guarded-crawl gate as the Stage20 probe; limits widened to admit the
  # controller's calibrated forward (a=0.30) and turn (b=0.26) primitives. IMU
  # yaw diagnostics are enabled for heading-hold and are motor-output-neutral.
  local flags="-DSTAGE20_PHYSICAL_AB_GUARDED_CRAWL=1 \
-DSTAGE20_MAX_ABS_A=${MAX_ABS_A} \
-DSTAGE20_MAX_ABS_B=${MAX_ABS_B} \
-DSTAGE20_MAX_MS=${MAX_MS} \
-DIMU_ENABLE=1 \
-DIMU_YAW_DIAG=1 \
-DPHYSICAL_PATH_FOLLOWING_ENABLE=0 \
-DPATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0 \
-DPATH_FOLLOWING_DRYRUN=0 \
-DGROUND_CRAWL_TEST_MODE=0 \
-DAUTO_MOTION_ARMED=0 \
-DSTAGE15_GUARDED_CRAWL_TEST=0 \
-DSTAGE16_USB_GUARDED_CRAWL=0 \
-DSTAGE17_FIRST_PRIMITIVE_CRAWL=0 \
-DSTAGE18_MOTOR_MAPPING_PROBE=0"

  if ! printf '%s' "$flags" | grep -q 'STAGE20_PHYSICAL_AB_GUARDED_CRAWL=1'; then
    echo "ABORT: guarded-crawl compile flag missing." >&2
    exit 2
  fi
  if printf '%s' "$flags" | grep -Eq 'PHYSICAL_PATH_FOLLOWING_ENABLE=1|PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=1|PATH_FOLLOWING_DRYRUN=1|GROUND_CRAWL_TEST_MODE=1|AUTO_MOTION_ARMED=1|STAGE15_GUARDED_CRAWL_TEST=1|STAGE16_USB_GUARDED_CRAWL=1|STAGE17_FIRST_PRIMITIVE_CRAWL=1|STAGE18_MOTOR_MAPPING_PROBE=1'; then
    echo "ABORT: full path-following, autonomous motion, or another stage flag present." >&2
    exit 2
  fi

  echo "PHYSICAL PATH PLANNER -- GUARDED CONTINUOUS MOTION"
  echo "guarded-crawl firmware only; NOT full path following"
  echo "max_abs_a=${MAX_ABS_A} max_abs_b=${MAX_ABS_B} max_ms=${MAX_MS}"
  echo "upload_port=${upload_port}"
  echo "flags=${flags}"

  arduino-cli compile --fqbn "$fqbn" --build-path "$build_path" \
    --build-property "compiler.cpp.extra_flags=${flags}" "$sketch"
  arduino-cli upload -p "$upload_port" --fqbn "$fqbn" --build-path "$build_path" "$sketch"
}

resolve_post_upload_port() {
  local fallback="$1"
  local detected
  detected="$(arduino-cli board list 2>/dev/null | awk '/OpenRB-150/ {print $1; exit}')"
  if [[ -z "$detected" ]]; then
    detected="$fallback"
  fi
  printf '%s' "$detected"
}

case "$MODE" in
  preview|calibrate-turn|diagnose)
    exec uv run python -m tools.physical_path_planning.cli "$MODE" "${PASSTHRU[@]}"
    ;;
  run|execute-plan)
    if [[ "$PRINT_PLAN" == "true" ]]; then
      # No motion, no serial: build/write the plan only.
      exec uv run python -m tools.physical_path_planning.cli "$MODE" "${PASSTHRU[@]}"
    fi
    flash_guarded_crawl_firmware "$PORT"
    POST_UPLOAD_PORT="$(resolve_post_upload_port "$PORT")"
    echo "post_upload_port=${POST_UPLOAD_PORT}"
    # The trailing --port wins in argparse, pinning the controller to the
    # re-enumerated board.
    exec uv run python -m tools.physical_path_planning.cli "$MODE" \
      "${PASSTHRU[@]}" --port "$POST_UPLOAD_PORT"
    ;;
  *)
    echo "Unknown mode: ${MODE}" >&2
    echo "Run with --help for usage." >&2
    exit 2
    ;;
esac
