#!/usr/bin/env bash
# Summarize firmware/gps_selected_pin_activity_probe logs.
#
# Usage:
#   scripts/check_gps_selected_pin_activity_log.sh [LOG]
#   LOG defaults to newest outputs/logs/gps_selected_pin_activity_*.log

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"

LOG="${1:-}"
if [ -z "$LOG" ]; then
  LOG="$(ls -t outputs/logs/gps_selected_pin_activity_*.log 2>/dev/null | head -1 || true)"
fi
if [ -z "$LOG" ] || [ ! -f "$LOG" ]; then
  echo "No selected-pin activity log found. Run scripts/run_gps_selected_pin_activity_probe.sh first, or pass a path." >&2
  exit 1
fi

PASSES="$(grep -c '^SELECTED_PIN_SWEEP_DONE' "$LOG" || true)"
echo "log=${LOG}"
echo "passes=${PASSES}"
if [ "$PASSES" -gt 0 ]; then
  echo "selected_pin_sweep_done_seen=true"
else
  echo "selected_pin_sweep_done_seen=false"
fi
echo

echo "Selected pins:"
missing=0
for pin in 6 13 14 26 27 28 29; do
  line="$(grep "pin_number=${pin} " "$LOG" | tail -1 || true)"
  if [ -n "$line" ]; then
    echo "  ${line}"
  else
    echo "  pin_number=${pin} MISSING"
    missing=1
  fi
done
echo

echo "Pins with possible_uart_activity=true:"
activity_lines="$(grep 'possible_uart_activity=true' "$LOG" || true)"
if [ -n "$activity_lines" ]; then
  echo "$activity_lines"
else
  echo "  none"
fi
echo

echo "Top transition_count pins from latest line per pin:"
awk '
  /^pin_number=/ {
    pin=""; trans=""; ratio=""; state=""; uart=""; known="";
    for (i=1; i<=NF; i++) {
      split($i, kv, "=");
      if (kv[1] == "pin_number") pin=kv[2];
      else if (kv[1] == "transition_count") trans=kv[2];
      else if (kv[1] == "high_ratio") ratio=kv[2];
      else if (kv[1] == "pin_state") state=kv[2];
      else if (kv[1] == "possible_uart_activity") uart=kv[2];
      else if (kv[1] == "known_function") known=kv[2];
    }
    if (pin != "") latest[pin]=sprintf("pin=%s trans=%s high_ratio=%s state=%s uart=%s function=%s", pin, trans, ratio, state, uart, known);
    if (pin != "") trans_by_pin[pin]=trans + 0;
  }
  END {
    for (pin in latest) print trans_by_pin[pin], latest[pin];
  }
' "$LOG" | sort -nr | head -10 | sed 's/^/  /'
echo

if [ "$missing" -ne 0 ]; then
  echo "FAIL: INCOMPLETE_SELECTED_PIN_SWEEP"
  echo "Meaning: at least one selected pin was not captured. Re-run:"
  echo "  scripts/run_gps_selected_pin_activity_probe.sh --monitor-seconds 180"
  exit 4
fi

if grep -q 'possible_uart_activity=true' "$LOG"; then
  echo "VERDICT=UART_ACTIVITY_FOUND"
  echo "Meaning: at least one selected OpenRB pin has electrical edges. This locates"
  echo "activity only; it does not prove NMEA parsing."
  exit 0
fi

echo "VERDICT=NO_SELECTED_PIN_UART_ACTIVITY"
echo "Meaning: none of D6/D13/D14/D26/D27/D28/D29 showed UART-like edges. If GPS was"
echo "powered and connected, GPS TX is not reaching these OpenRB pins or GPS is not"
echo "transmitting. This is not a GPS-fix issue yet."
exit 2
