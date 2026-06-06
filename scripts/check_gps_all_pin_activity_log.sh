#!/usr/bin/env bash
# Summarize firmware/gps_all_pin_activity_probe logs.
#
# Highlights:
# - pins with possible_uart_activity=true
# - highest transition_count pins
# - D29 / Serial2_RX status if present
#
# Usage:
#   scripts/check_gps_all_pin_activity_log.sh [LOG]
#   LOG defaults to newest outputs/logs/gps_all_pin_activity_*.log

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"

LOG="${1:-}"
if [ -z "${LOG}" ]; then
  LOG="$(ls -t outputs/logs/gps_all_pin_activity_*.log 2>/dev/null | head -1 || true)"
fi
if [ -z "${LOG}" ] || [ ! -f "${LOG}" ]; then
  echo "No all-pin activity log found. Run scripts/run_gps_all_pin_activity_probe.sh first, or pass a path." >&2
  exit 1
fi

echo "log=${LOG}"
PASSES="$(grep -c '^ALL_PIN_SWEEP_DONE' "$LOG" || true)"
echo "passes=${PASSES}"
if [ "${PASSES}" -gt 0 ]; then
  echo "all_pin_sweep_done_seen=true"
else
  echo "all_pin_sweep_done_seen=false"
fi
echo

echo "Pins with possible_uart_activity=true:"
activity_lines="$(grep 'possible_uart_activity=true' "$LOG" || true)"
if [ -n "$activity_lines" ]; then
  echo "$activity_lines"
else
  echo "  none"
fi
echo

echo "Required late-sweep pins D22-D29:"
missing_late_pins=0
for pin in 22 23 24 25 26 27 28 29; do
  line="$(grep "pin_number=${pin} " "$LOG" | tail -1 || true)"
  if [ -n "$line" ]; then
    echo "  ${line}"
  else
    echo "  pin_number=${pin} MISSING"
    missing_late_pins=1
  fi
done
echo

echo "Key pins if present:"
for pin in 6 13 14 26 27 28 29; do
  line="$(grep "pin_number=${pin} " "$LOG" | tail -1 || true)"
  if [ -n "$line" ]; then
    echo "  ${line}"
  else
    echo "  pin_number=${pin} MISSING"
  fi
done
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

echo "D29 / Serial2_RX expected GPS-TX target:"
grep 'pin_number=29 ' "$LOG" | tail -1 | sed 's/^/  /' || echo "  no D29 line found"
echo

if [ "$missing_late_pins" -ne 0 ]; then
  echo "FAIL: INCOMPLETE_SWEEP"
  echo "Meaning: pins 22-29 were not all captured. The log cannot answer whether GPS TX"
  echo "reaches D29. Re-run with the fixed runner or a longer duration:"
  echo "  scripts/run_gps_all_pin_activity_probe.sh --monitor-seconds 360"
  exit 4
fi

if grep -q 'possible_uart_activity=true' "$LOG"; then
  echo "VERDICT=UART_ACTIVITY_FOUND"
  echo "Meaning: at least one OpenRB pin has electrical edges. This locates activity only;"
  echo "it does not prove NMEA parsing. Next, map the active pin to its UART and run a"
  echo "UART/baud/NMEA probe on that path."
  exit 0
fi

echo "VERDICT=NO_ALL_PIN_UART_ACTIVITY"
echo "Meaning: no sampled OpenRB D0-D29 pin showed UART-like edges. If the GPS was powered"
echo "and connected during the test, GPS TX is not reaching OpenRB or the GPS is not"
echo "transmitting. This is not a GPS-fix problem and not a baud/parsing problem yet."
exit 2
