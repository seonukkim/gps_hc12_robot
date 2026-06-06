#!/usr/bin/env bash
# Pass/fail check for a firmware/gps_uart_probe log (expects Serial2 @ 9600).
#
# GPS UART is considered VALID only when sustained, real NMEA-like data is seen.
# A few stray bytes (e.g. total_chars=2 over ~70 s) are noise/floating, not NMEA,
# and must NOT pass. Any chars_1s>0 alone is NOT sufficient.
#
# Verdicts:
#   WRONG_UART                    selected_port != Serial2                  (FAIL, exit 2)
#   NO_BYTES                      total_chars == 0                          (FAIL, exit 3)
#   INSUFFICIENT_BYTES_OR_NO_NMEA total_chars < 50, or no RMC/GGA/$GP/$GN   (FAIL, exit 4)
#   BYTES_NO_FIX                  sustained real NMEA, but no fix yet        (pass, exit 0)
#   FIX_OK                        fix=true / lat-lon valid / sats available  (pass, exit 0)
#
# Pass threshold for a valid GPS UART path: selected_port=Serial2 AND
# total_chars >= 50 (preferably much higher) AND at least one of rmc_preview /
# gga_preview != "NA", or raw NMEA contains "$GP" / "$GN".
#
# Usage: scripts/check_gps_probe_log.sh [LOG]
#   LOG defaults to the newest outputs/logs/gps_probe_serial2_9600_*.log

set -uo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"

LOG="${1:-}"
if [ -z "${LOG}" ]; then
  LOG="$(ls -t outputs/logs/gps_probe_serial2_9600_*.log 2>/dev/null | head -1 || true)"
fi
if [ -z "${LOG}" ] || [ ! -f "${LOG}" ]; then
  echo "No GPS probe log found. Run scripts/run_gps_probe_serial2_9600.sh first, or pass a path." >&2
  exit 1
fi

last() { grep -oE " ?$1=[^ ]+" "${LOG}" 2>/dev/null | tail -1 | sed 's/^ //; s/^[^=]*=//'; }

port="$(grep -oE 'selected_port=[^ ]+' "${LOG}" | tail -1 | sed 's/.*=//')"
chars_last="$(last chars_1s)"
total="$(last total_chars)"
state="$(last gps_probe_state)"
block="$(last gps_block_reason)"
cvf="$(last current_valid_fix)"
sats="$(last sats)"
hdop="$(last hdop)"
latln="$(grep -oE ' lat=[^ ]+ lon=[^ ]+' "${LOG}" | tail -1 | sed 's/^ //')"

MIN_TOTAL_CHARS=50

# Real NMEA seen: a $GP/$GN/... sentence appears anywhere (the probe stores the
# last RMC/GGA sentence in rmc_preview/gga_preview), or a preview is not "NA".
if grep -qiE '\$G[A-Z]{2}' "${LOG}" \
   || grep -qE '(rmc|gga)_preview="\$' "${LOG}"; then nmea_seen=true; else nmea_seen=false; fi
if grep -q 'current_valid_fix=true' "${LOG}" || grep -qE ' lat=-?[0-9]+\.[0-9]+' "${LOG}"; then fix=true; else fix=false; fi

total_num="${total:-0}"; case "${total_num}" in ''|*[!0-9]*) total_num=0;; esac
if [ "${total_num}" -ge "${MIN_TOTAL_CHARS}" ] && [ "${nmea_seen}" = true ]; then
  sustained_nmea=true
else
  sustained_nmea=false
fi

echo "log=${LOG}"
echo "  selected_port        = ${port:-MISSING}   (expected Serial2)"
echo "  chars_1s(last)       = ${chars_last:-MISSING}"
echo "  total_chars(last)    = ${total:-MISSING}   (min for valid UART: ${MIN_TOTAL_CHARS})"
echo "  real_nmea_seen       = ${nmea_seen}   (\$GP/\$GN or non-NA RMC/GGA preview)"
echo "  sustained_nmea       = ${sustained_nmea}"
echo "  gps_probe_state      = ${state:-MISSING}"
echo "  gps_block_reason     = ${block:-MISSING}"
echo "  current_valid_fix    = ${cvf:-MISSING}"
echo "  sats / hdop          = ${sats:-NA} / ${hdop:-NA}"
echo "  ${latln:-lat=NA lon=NA}"

echo "--- GPS PROBE VERDICT ---"
if [ "${port:-}" != "Serial2" ]; then
  echo "  FAIL: WRONG_UART (selected_port=${port:-MISSING}, expected Serial2)."
  echo "  GPS is on the center 5-pin connector = Serial2. Recompile with -DGPS_PROBE_MODE=2"
  echo "  (use scripts/run_gps_probe_serial2_9600.sh)."
  exit 2
fi
if [ "${total_num}" -eq 0 ]; then
  echo "  FAIL: NO_BYTES (total_chars=0)."
  echo "  Wrong UART / wiring / power / baud — NOT a dead GPS module and NOT a code bug."
  echo "  Confirm GPS on the Serial2 center 5-pin connector at 9600 (GPS TX -> OpenRB Serial2"
  echo "  RX, VCC/GND). Old known-good baseline: Serial2 9600, chars_1s ~500, \$GP NMEA."
  exit 3
fi
if [ "${sustained_nmea}" = false ]; then
  echo "  FAIL: INSUFFICIENT_BYTES_OR_NO_NMEA (total_chars=${total_num} < ${MIN_TOTAL_CHARS}"
  echo "        and/or real_nmea_seen=${nmea_seen})."
  echo "  A handful of bytes with rmc/gga_preview=\"NA\" is noise/floating/intermittent, not"
  echo "  real NMEA. The GPS UART path is NOT validated. Check the center 5-pin connector:"
  echo "  seating/orientation, GPS TX -> OpenRB Serial2 RX, VCC and GND, and 9600 baud."
  exit 4
fi
if [ "${fix}" = true ]; then
  echo "  PASS: FIX_OK — Serial2 GPS UART works (sustained NMEA) and a fix is present."
  exit 0
fi
echo "  BYTES_NO_FIX — Serial2 GPS UART works (sustained real NMEA: total_chars=${total_num},"
echo "  real_nmea_seen=true) but no satellite fix yet. UART path VALIDATED; move to open sky"
echo "  and wait for fix=true / sats>=4 / low hdop."
exit 0
