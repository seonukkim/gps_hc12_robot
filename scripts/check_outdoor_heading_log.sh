#!/usr/bin/env bash
# Read-only outdoor heading log checker. It never uploads, monitors, or touches
# firmware; it only summarizes an existing integrated USBDBG log.
#
# Usage:
#   scripts/check_outdoor_heading_log.sh [LOG]
#
# If LOG is omitted, the newest outdoor/integrated no-motion log is used.

set -uo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"

LOG="${1:-}"
if [ -z "$LOG" ]; then
  LOG="$(ls -t \
    outputs/logs/outdoor_heading_*.log \
    outputs/logs/outdoor_heading_redo_*.log \
    outputs/logs/outdoor_course_latch_diag_*.log \
    outputs/logs/outdoor_integrated_watch_*.log \
    outputs/logs/integrated_dryrun_hc12_deferred_*.log \
    2>/dev/null | head -1 || true)"
fi

if [ -z "$LOG" ] || [ ! -f "$LOG" ]; then
  echo "FAIL: no outdoor/integrated log found. Pass a log path explicitly." >&2
  exit 1
fi

echo "log=$LOG"
PYTHON_BIN="${PYTHON_BIN:-python3}"
ANALYSIS="$("$PYTHON_BIN" tools/analyze_heading_readiness_log.py "$LOG")"
printf '%s\n' "$ANALYSIS"

VERDICT="$(printf '%s\n' "$ANALYSIS" | awk -F= '/^verdict=/ {print $2; exit}')"
NEXT_ACTION="$(printf '%s\n' "$ANALYSIS" | awk -F= '/^next_action=/ {print $2; exit}')"

echo "--- checklist decision ---"
echo "verdict=${VERDICT:-UNKNOWN}"
echo "next_action=${NEXT_ACTION:-UNKNOWN}"

case "$VERDICT" in
  MOTOR_SAFETY_FAIL)
    echo "ABORT: motor safety failed. Do not continue outdoor validation."
    exit 3
    ;;
  SENSOR_BASELINE_FAIL)
    echo "NEXT: fix GPS/BMI160/RC baseline before heading validation."
    ;;
  HEADING_READY_PASS)
    echo "NEXT: proceed only to offline path-planning preview. Physical motor enable remains prohibited."
    ;;
  HEADING_COURSE_LATCH_AVAILABLE)
    echo "NEXT: review heading agreement diagnostics, then proceed only to offline path-planning preview."
    ;;
  HEADING_COURSE_ACCEPTED_BUT_NOT_OUTPUT)
    echo "NEXT: inspect gps_course_output_block_reason; movement and accepted course are already proven."
    ;;
  HEADING_NOT_READY_BUT_MOVEMENT_SUFFICIENT)
    echo "NEXT: use the new diagnostic firmware/log fields and repeat the straight hand-carry."
    ;;
  SENSOR_BASELINE_PASS)
    echo "NEXT: repeat hand-carry with a longer straight segment, then rerun this checker."
    ;;
  *)
    echo "NEXT: inspect the analyzer output manually before another outdoor run."
    ;;
esac
