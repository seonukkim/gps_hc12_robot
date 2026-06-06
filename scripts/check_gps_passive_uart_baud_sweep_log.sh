#!/usr/bin/env bash
# Classify a passive GPS UART/baud sweep log (firmware/gps_passive_uart_baud_sweep_probe).
#
# Verdicts:
#   NMEA_FOUND     some port/baud had sustained NMEA-like sentences (nmea_like_seen>=1)
#   BYTES_NO_NMEA  bytes appear (>= MIN) but no NMEA-like text anywhere
#   NO_UART_BYTES  no meaningful bytes on any port/baud
#
# Usage: scripts/check_gps_passive_uart_baud_sweep_log.sh [LOG]
#   LOG defaults to the newest outputs/logs/gps_passive_uart_baud_sweep_*.log

set -uo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"

MIN_BYTES=20

LOG="${1:-}"
if [ -z "${LOG}" ]; then
  LOG="$(ls -t outputs/logs/gps_passive_uart_baud_sweep_*.log 2>/dev/null | head -1 || true)"
fi
if [ -z "${LOG}" ] || [ ! -f "${LOG}" ]; then
  echo "No sweep log found. Run scripts/run_gps_passive_uart_baud_sweep.sh first, or pass a path." >&2
  exit 1
fi

field() { echo "$1" | grep -oE "$2=[^ ]+" | tail -1 | sed 's/.*=//'; }

# Prefer the last completed sweep summary; fall back to scanning candidate lines.
done_line="$(grep '^SWEEP_DONE' "${LOG}" | tail -1)"
best_port="$(field "${done_line}" best_port)"
best_baud="$(field "${done_line}" best_baud)"
best_chars="$(field "${done_line}" best_chars)"
best_nmea="$(field "${done_line}" best_nmea_like_seen)"
best_is_nmea="$(field "${done_line}" best_is_nmea)"

# Direct cross-checks from candidate lines.
nmea_line="$(grep -E 'nmea_like_seen=[1-9]' "${LOG}" | tail -1)"
max_chars="$(grep -oE 'chars_seen=[0-9]+' "${LOG}" | sed 's/.*=//' | sort -n | tail -1)"
max_chars="${max_chars:-0}"

# The candidate with the most chars (for the BYTES_NO_NMEA report).
top_bytes_line="$(grep '^candidate_port=' "${LOG}" \
  | sort -t= -k4 -n 2>/dev/null | tail -1)"

echo "log=${LOG}"
echo "  sweeps_completed     = $(grep -c '^SWEEP_DONE' "${LOG}")"
echo "  best_port            = ${best_port:-NA}"
echo "  best_baud            = ${best_baud:-NA}"
echo "  best_chars           = ${best_chars:-NA}"
echo "  best_nmea_like_seen  = ${best_nmea:-NA}"
echo "  best_is_nmea         = ${best_is_nmea:-NA}"
echo "  max chars_seen       = ${max_chars}   (min meaningful: ${MIN_BYTES})"
if [ -n "${nmea_line}" ]; then
  echo "  nmea candidate       = $(field "${nmea_line}" candidate_port)@$(field "${nmea_line}" candidate_baud) (nmea_like_seen=$(field "${nmea_line}" nmea_like_seen))"
fi

echo "--- SWEEP VERDICT ---"

# NMEA found anywhere?
if grep -qE 'nmea_like_seen=[1-9]' "${LOG}" || [ "${best_is_nmea}" = "true" ]; then
  rp="${best_port}"; rb="${best_baud}"
  if [ "${best_is_nmea}" != "true" ] && [ -n "${nmea_line}" ]; then
    rp="$(field "${nmea_line}" candidate_port)"; rb="$(field "${nmea_line}" candidate_baud)"
  fi
  echo "  NMEA_FOUND — real NMEA on ${rp}@${rb}."
  if [ "${rp}" = "Serial2" ] && [ "${rb}" = "9600" ]; then
    echo "  This matches the known-good baseline (Serial2 @ 9600). The earlier"
    echo "  total_chars=2 run was intermittent; re-run gps_uart_probe on Serial2/9600."
  else
    echo "  The old assumption is WRONG or the baud differs: GPS NMEA is on ${rp}@${rb},"
    echo "  NOT Serial2@9600. Use that port/baud (gps_uart_probe -DGPS_PROBE_MODE for the"
    echo "  port, -DGPS_PROBE_BAUD=${rb}); confirm the physical connector matches ${rp}."
  fi
  exit 0
fi

if [ "${max_chars}" -ge "${MIN_BYTES}" ]; then
  echo "  BYTES_NO_NMEA — bytes appear (max chars_seen=${max_chars}) but no NMEA-like text"
  echo "  on any port/baud. Likely wrong baud, a non-GPS device, or a logic-level/framing"
  echo "  problem. Top byte candidate:"
  echo "    ${top_bytes_line:-NA}"
  echo "  Try the GPS at its real baud, and verify it is a GPS (not another module)."
  exit 4
fi

echo "  NO_UART_BYTES — no meaningful bytes on any UART at any baud."
echo "  A physical power / connector / module / logic-level issue is most likely:"
echo "  GPS VCC + common GND, connector fully seated and correctly oriented on the"
echo "  center 5-pin (Serial2), GPS TX -> OpenRB RX, and a powered GPS (fix/power LED)."
echo "  This is NOT a code/UART-assumption problem; it is physical."
exit 3
