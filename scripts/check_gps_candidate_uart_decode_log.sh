#!/usr/bin/env bash
# Summarize firmware/gps_candidate_uart_decode_probe logs.
#
# Usage:
#   scripts/check_gps_candidate_uart_decode_log.sh [LOG]
#   LOG defaults to newest outputs/logs/gps_candidate_uart_decode_*.log

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"

LOG="${1:-}"
if [ -z "$LOG" ]; then
  LOG="$(ls -t outputs/logs/gps_candidate_uart_decode_*.log 2>/dev/null | head -1 || true)"
fi
if [ -z "$LOG" ] || [ ! -f "$LOG" ]; then
  echo "No candidate UART decode log found. Run scripts/run_gps_candidate_uart_decode_probe.sh first, or pass a path." >&2
  exit 1
fi

echo "log=${LOG}"
PASSES="$(grep -c '^UART_DECODE_SWEEP_DONE' "$LOG" || true)"
echo "uart_decode_sweep_done_count=${PASSES}"
echo

echo "NMEA-like candidates:"
nmea_lines="$(grep 'UART_DECODE_RESULT' "$LOG" | grep 'nmea_like=true' || true)"
if [ -n "$nmea_lines" ]; then
  echo "$nmea_lines"
else
  echo "  none"
fi
echo

echo "D6 results:"
grep 'UART_DECODE_RESULT pin_number=6 ' "$LOG" || echo "  none"
echo

echo "D29 / Serial2_RX expected GPS-TX target results:"
grep 'UART_DECODE_RESULT pin_number=29 ' "$LOG" || echo "  none"
echo

echo "All pin/baud summaries:"
grep 'UART_DECODE_RESULT' "$LOG" | sed 's/ ascii_preview=.*//' || true
echo

echo "Top byte_count pin/baud pairs:"
awk '
  /^UART_DECODE_RESULT/ {
    pin=""; label=""; baud=""; bytes=""; gp=""; gn=""; rmc=""; gga=""; nmea="";
    for (i=1; i<=NF; i++) {
      split($i, kv, "=");
      if (kv[1] == "pin_number") pin=kv[2];
      else if (kv[1] == "label") label=kv[2];
      else if (kv[1] == "baud") baud=kv[2];
      else if (kv[1] == "byte_count") bytes=kv[2];
      else if (kv[1] == "contains_gp") gp=kv[2];
      else if (kv[1] == "contains_gn") gn=kv[2];
      else if (kv[1] == "contains_rmc") rmc=kv[2];
      else if (kv[1] == "contains_gga") gga=kv[2];
      else if (kv[1] == "nmea_like") nmea=kv[2];
    }
    if (pin != "") {
      print (bytes + 0), sprintf("pin=%s label=%s baud=%s byte_count=%s gp=%s gn=%s rmc=%s gga=%s nmea_like=%s", pin, label, baud, bytes, gp, gn, rmc, gga, nmea);
    }
  }
' "$LOG" | sort -nr | head -12 | sed 's/^/  /'
echo

if [ -n "$nmea_lines" ]; then
  echo "VERDICT=NMEA_LIKE_UART_DECODE_FOUND"
  echo "Meaning: a candidate pin/baud decoded GPS-like text. This still needs a normal"
  echo "UART probe on that path, but it is stronger evidence than edge activity alone."
  exit 0
fi

if grep -q 'UART_DECODE_RESULT' "$LOG"; then
  echo "VERDICT=NO_NMEA_LIKE_UART_DECODE"
  echo "Meaning: candidate pins may have edge/noise bytes, but no $GP/$GN/RMC/GGA text"
  echo "was decoded. D6 edge activity is not proven to be GPS unless NMEA-like text appears."
  exit 2
fi

echo "VERDICT=INCOMPLETE_UART_DECODE_LOG"
echo "Meaning: no UART_DECODE_RESULT lines were captured."
exit 4
