#!/usr/bin/env bash
set -euo pipefail

# Unified launcher for the field-facing physical path planner.
#
# Field workflows should use this script.

MODE="${1:-}"
if [[ -z "$MODE" || "$MODE" == "-h" || "$MODE" == "--help" ]]; then
  cat <<'USAGE'
Usage:
  scripts/run_physical_path_planner.sh <mode> [options]

Modes:
  diagnose              Read-only telemetry summary. No motor commands.
  rc-input-diagnose     Read-only receiver input/channel diagnostic. No motors.
  manual-rc             Upload and validate manual RC recovery.
  manual-control        Upload and monitor PPM physical manual control.
  station-hw-diagnose   Read-only physical station hardware link diagnostic.
  station-hw-manual     Deprecated serial-frame monitor; use manual-control for PPM.
  usb-pulse-test        Laptop USB bounded A/B pulse motor validation.
  tune-motion           Interactive visual/IMU-assisted motion calibration.
  guarded-pulse-ready   Upload/check IMU-enabled guarded pulse firmware.
  calibrate-turn        Run guarded pulse turn angle calibration.
  preview               Build + render a rectangle coverage plan. No motion.
  execute-plan          Execute an existing/specified plan with guarded pulses.
  run                   Plan and execute with guarded pulses.

Examples:
  bash scripts/run_physical_path_planner.sh diagnose \
    --out-dir outputs/physical_path_planning/diagnose

  bash scripts/run_physical_path_planner.sh rc-input-diagnose \
    --out-dir outputs/physical_path_planning/rc_input_diagnose

  bash scripts/run_physical_path_planner.sh manual-rc \
    --out-dir outputs/physical_path_planning/manual_rc

  bash scripts/run_physical_path_planner.sh manual-control \
    --out-dir outputs/physical_path_planning/manual_control

  bash scripts/run_physical_path_planner.sh station-hw-diagnose \
    --out-dir outputs/physical_path_planning/station_hw_diagnose

  bash scripts/run_physical_path_planner.sh station-hw-manual \
    --out-dir outputs/physical_path_planning/station_hw_manual

  bash scripts/run_physical_path_planner.sh usb-pulse-test \
    --print-command true \
    --out-dir outputs/physical_path_planning/usb_pulse_test_print

  bash scripts/run_physical_path_planner.sh tune-motion \
    --primitive forward \
    --out-dir outputs/physical_path_planning/tune_forward

  bash scripts/run_physical_path_planner.sh guarded-pulse-ready \
    --out-dir outputs/physical_path_planning/guarded_pulse_ready

  bash scripts/run_physical_path_planner.sh calibrate-turn \
    --direction left --b-cmd 0.22 --pulse-ms 1200 \
    --target-angle-deg 90 --angle-tolerance-deg 10 \
    --out-dir outputs/physical_path_planning/calibration/left_022_1200

  bash scripts/run_physical_path_planner.sh preview \
    --goal-mode relative_enu --goal-east-m 4.0 --goal-north-m -1.2 \
    --workspace-width-m 1.2 --step-spacing-m 0.25 \
    --out-dir outputs/physical_path_planning/preview_relative_enu

  bash scripts/run_physical_path_planner.sh execute-plan \
    --plan-dir outputs/physical_path_planning/preview_relative_enu \
    --out-dir outputs/physical_path_planning/execute_preview_relative_enu

  bash scripts/run_physical_path_planner.sh run \
    --goal-mode relative_enu --goal-east-m 4.0 --goal-north-m -1.2 \
    --workspace-width-m 1.2 --step-spacing-m 0.25 \
    --out-dir outputs/physical_path_planning/run_relative_enu

Every mode writes:
  <out-dir>/summary.md
  <out-dir>/summary.json

OpenRB-150 is auto-detected when --port is omitted. Use --port "$PORT" only
if auto-detection fails.
USAGE
  exit 0
fi
shift || true

ENV_PORT="${PORT:-}"
PORT=""
PORT_SOURCE="none"
EXPLICIT_PORT="false"
MAX_ABS_A="0.35"
MAX_ABS_B="0.35"
MAX_MS="1000"
PRINT_PLAN="false"
PRINT_CMD="false"
PRINT_CANDIDATE="false"
FROM_LOG="false"
PASSTHRU=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --port)
      PORT="$2"
      EXPLICIT_PORT="true"
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
    --print-cmd)
      PRINT_CMD="true"
      PASSTHRU+=("--print-cmd")
      shift
      ;;
    --print-command)
      PRINT_CMD="$2"
      PASSTHRU+=("--print-command" "$2")
      shift 2
      ;;
    --print-candidate)
      PRINT_CANDIDATE="$2"
      PASSTHRU+=("--print-candidate" "$2")
      shift 2
      ;;
    --from-log)
      FROM_LOG="true"
      PASSTHRU+=("--from-log" "$2")
      shift 2
      ;;
    *)
      PASSTHRU+=("$1")
      shift
      ;;
  esac
done

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"

PASSTHRU_HELP="false"
for arg in "${PASSTHRU[@]}"; do
  if [[ "$arg" == "--help" || "$arg" == "-h" ]]; then
    PASSTHRU_HELP="true"
  fi
done

if [[ "$PASSTHRU_HELP" == "true" ]]; then
  case "$MODE" in
    station-hw-diagnose)
      cat <<'USAGE'
Usage:
  scripts/run_physical_path_planner.sh station-hw-diagnose [options]

Read-only physical station hardware link monitor. No motor commands are sent.

Options:
  --port PORT              OpenRB USB debug port; auto-detected when omitted.
  --baud BAUD              OpenRB USB debug baudrate. Default: 115200.
  --duration-s SECONDS     Monitor duration. Default: 20.
  --upload true|false|auto Upload station hardware monitor firmware. Default: auto.
  --from-log PATH          Parse an existing raw USB debug log instead of serial.
  --verbose-raw true|false Print raw telemetry in addition to concise status.
  --print-command true     Print firmware commands and exit.
  --out-dir DIR            Output directory.
USAGE
      exit 0
      ;;
    station-hw-manual)
      cat <<'USAGE'
Usage:
  scripts/run_physical_path_planner.sh station-hw-manual [options]

Deprecated for the current PPM station/controller path. Use manual-control.

Options:
  --port PORT              OpenRB USB debug port; auto-detected when omitted.
  --baud BAUD              OpenRB USB debug baudrate. Default: 115200.
  --duration-s SECONDS     Monitor duration; <=0 means continuous until Ctrl-C.
                            Default: 0.
  --upload true|false|auto Upload station hardware manual firmware. Default: auto.
  --from-log PATH          Parse an existing raw USB debug log instead of serial.
  --verbose-raw true|false Print raw telemetry in addition to concise status.
  --print-command true     Print firmware commands and exit.
  --out-dir DIR            Output directory.
USAGE
      exit 0
      ;;
    manual-control)
      cat <<'USAGE'
Usage:
  scripts/run_physical_path_planner.sh manual-control [options]

PPM physical manual control. Uploads/monitors firmware using OpenRB D6:
CH1 steering, CH2 throttle, CH5 manual/auto mode.

Options:
  --port PORT              OpenRB USB debug port; auto-detected when omitted.
  --baud BAUD              OpenRB USB debug baudrate. Default: 115200.
  --duration-s SECONDS     Monitor duration. Default: 45.
  --upload true|false|auto Upload PPM manual firmware. Default: true.
  --from-log PATH          Parse an existing raw USB debug log instead of serial.
  --verbose-raw true|false Print raw telemetry in addition to concise status.
  --print-cmd              Print firmware commands and exit.
  --out-dir DIR            Output directory.
USAGE
      exit 0
      ;;
  esac
fi

detect_openrb_port() {
  arduino-cli board list 2>/dev/null | awk '/OpenRB-150/ {print $1; exit}'
}

resolve_port() {
  if [[ "$EXPLICIT_PORT" == "true" ]]; then
    PORT_SOURCE="explicit"
  elif [[ -n "$ENV_PORT" ]]; then
    PORT="$ENV_PORT"
    PORT_SOURCE="env"
  else
    local detected
    detected="$(detect_openrb_port)"
    if [[ -n "$detected" ]]; then
      PORT="$detected"
      PORT_SOURCE="arduino_cli"
    elif [[ "$(uname -s)" == "Linux" && -e /dev/ttyACM0 ]]; then
      PORT="/dev/ttyACM0"
      PORT_SOURCE="linux_default"
    else
      PORT=""
      PORT_SOURCE="none"
    fi
  fi
}

mode_needs_port() {
  case "$MODE" in
    diagnose)
      [[ "$FROM_LOG" != "true" ]]
      ;;
    calibrate-turn|rc-input-diagnose|manual-rc|manual-control|station-hw-diagnose|station-hw-manual|usb-pulse-test|station-drive|station-manual|guarded-pulse-ready)
      [[ "$PRINT_CMD" != "true" ]]
      ;;
    tune-motion)
      [[ "$PRINT_CANDIDATE" != "true" ]]
      ;;
    run|execute-plan)
      [[ "$PRINT_PLAN" != "true" ]]
      ;;
    *)
      return 1
      ;;
  esac
}

if mode_needs_port; then
  resolve_port
  if [[ -n "$PORT" ]]; then
    echo "resolved_port=${PORT}"
    echo "port_source=${PORT_SOURCE}"
    if [[ "$EXPLICIT_PORT" != "true" ]]; then
      PASSTHRU+=("--port" "$PORT")
    fi
  else
    echo "resolved_port="
    echo "port_source=none"
  fi
fi

flash_guarded_crawl_firmware() {
  local upload_port="$1"
  local fqbn="OpenRB-150:samd:OpenRB-150"
  local build_path="/private/tmp/openrb-physical-path-planner"
  local sketch="firmware/openrb_robot_controller"
  # USB pulse-test firmware. Limits are widened to admit the calibrated
  # forward (a=0.30) and turn (b=0.26) primitives.
  local flags="-DUSB_PULSE_TEST_GUARDED=1 \
-DUSB_PULSE_TEST_IGNORE_RC_INPUT=1 \
-DUSB_PULSE_TEST_MAX_ABS_A=${MAX_ABS_A} \
-DUSB_PULSE_TEST_MAX_ABS_B=${MAX_ABS_B} \
-DUSB_PULSE_TEST_MAX_MS=${MAX_MS} \
-DIMU_ENABLE=1 \
-DIMU_YAW_DIAG=1 \
-DPHYSICAL_PATH_FOLLOWING_ENABLE=0 \
-DPATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0 \
-DPATH_FOLLOWING_DRYRUN=0 \
-DGROUND_CRAWL_TEST_MODE=0 \
-DAUTO_MOTION_ARMED=0"

  if ! printf '%s' "$flags" | grep -q 'USB_PULSE_TEST_GUARDED=1'; then
    echo "ABORT: usb-pulse-test compile flag missing." >&2
    exit 2
  fi
  if printf '%s' "$flags" | grep -Eq 'PHYSICAL_PATH_FOLLOWING_ENABLE=1|PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=1|PATH_FOLLOWING_DRYRUN=1|GROUND_CRAWL_TEST_MODE=1|AUTO_MOTION_ARMED=1'; then
    echo "ABORT: full path-following, autonomous motion, or incompatible mode flag present." >&2
    exit 2
  fi

  echo "PHYSICAL PATH PLANNER -- GUARDED CONTINUOUS MOTION"
  echo "guarded pulse firmware only; NOT full path following"
  echo "max_abs_a=${MAX_ABS_A} max_abs_b=${MAX_ABS_B} max_ms=${MAX_MS}"
  echo "upload_port=${upload_port}"
  echo "firmware_flags=guarded_pulse_safe_defaults"

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
  preview|calibrate-turn|diagnose|rc-input-diagnose|manual-rc|manual-control|station-hw-diagnose|station-hw-manual|usb-pulse-test|tune-motion|station-drive|station-manual|guarded-pulse-ready)
    exec uv run python -m tools.physical_path_planning.cli "$MODE" "${PASSTHRU[@]}"
    ;;
  run|execute-plan)
    if [[ "$PRINT_PLAN" == "true" ]]; then
      # No motion, no serial: build/write the plan only.
      exec uv run python -m tools.physical_path_planning.cli "$MODE" "${PASSTHRU[@]}"
    fi
    if [[ -z "$PORT" ]]; then
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
