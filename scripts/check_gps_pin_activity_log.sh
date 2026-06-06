#!/usr/bin/env bash
# Classify a pin-activity log (firmware/gps_pin_activity_probe).
#
# Question answered: does a GPS TX electrical signal reach any OpenRB pin?
#
# Verdicts (primary):
#   UART_ACTIVITY_FOUND  at least one candidate pin has meaningful transitions
#   STATIC_HIGH          likely-RX pins are static HIGH, no transitions
#   STATIC_LOW           likely-RX pins are static LOW, no transitions
#   NO_PIN_ACTIVITY      no meaningful activity anywhere (mixed/floating)
# Plus, when no likely-RX pin shows activity:
#   POSSIBLE_POWER_OFF_OR_TX_NOT_CONNECTED
#
# Usage: scripts/check_gps_pin_activity_log.sh [LOG]
#   LOG defaults to the newest outputs/logs/gps_pin_activity_*.log

set -uo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"

LOG="${1:-}"
if [ -z "${LOG}" ]; then
  LOG="$(ls -t outputs/logs/gps_pin_activity_*.log 2>/dev/null | head -1 || true)"
fi
if [ -z "${LOG}" ] || [ ! -f "${LOG}" ]; then
  echo "No pin-activity log found. Run scripts/run_gps_pin_activity_probe.sh first, or pass a path." >&2
  exit 1
fi

# Last-pass value of a field from the last line of a given pin.
pin_field() { grep "pin_label=$1 " "${LOG}" | tail -1 | grep -oE " $2=[^ ]+" | tail -1 | sed 's/^ //; s/^[^=]*=//'; }

RX_PINS="Serial1_RX Serial2_RX Serial3_RX"

echo "log=${LOG}"
echo "  passes               = $(grep -c '^PIN_SWEEP_DONE' "${LOG}")"
echo "  per-pin (last pass):"
for lbl in Serial1_RX Serial2_RX Serial3_RX Serial1_TX Serial2_TX Serial3_TX; do
  line="$(grep "pin_label=${lbl} " "${LOG}" | tail -1)"
  [ -z "${line}" ] && continue
  printf '    %-11s pin=%-3s trans=%-8s high_ratio=%-6s state=%-18s uart=%s\n' \
    "${lbl}" "$(pin_field "${lbl}" arduino_pin)" "$(pin_field "${lbl}" transition_count)" \
    "$(pin_field "${lbl}" high_ratio)" "$(pin_field "${lbl}" pin_state)" \
    "$(pin_field "${lbl}" possible_uart_activity)"
done

# Aggregate the likely-RX pins (GPS-TX targets) from the last pass.
rx_total=0; rx_activity=0; rx_static_high=0; rx_static_low=0; rx_other=0
for lbl in ${RX_PINS}; do
  line="$(grep "pin_label=${lbl} " "${LOG}" | tail -1)"
  [ -z "${line}" ] && continue
  rx_total=$((rx_total + 1))
  ua="$(pin_field "${lbl}" possible_uart_activity)"
  st="$(pin_field "${lbl}" pin_state)"
  if [ "${ua}" = "true" ]; then rx_activity=$((rx_activity + 1));
  elif [ "${st}" = "STATIC_HIGH" ]; then rx_static_high=$((rx_static_high + 1));
  elif [ "${st}" = "STATIC_LOW" ]; then rx_static_low=$((rx_static_low + 1));
  else rx_other=$((rx_other + 1)); fi
done

echo "  rx_pins=${rx_total} rx_with_activity=${rx_activity} rx_static_high=${rx_static_high} rx_static_low=${rx_static_low} rx_other=${rx_other}"
echo "--- PIN ACTIVITY VERDICT ---"

if grep -q 'possible_uart_activity=true' "${LOG}"; then
  act_line="$(grep 'possible_uart_activity=true' "${LOG}" | tail -1)"
  act_pin="$(echo "${act_line}" | grep -oE 'pin_label=[^ ]+' | sed 's/.*=//')"
  echo "  UART_ACTIVITY_FOUND — real transitions on ${act_pin} (and possibly others)."
  echo "  The GPS TX electrical signal IS reaching an OpenRB pin, so this is a UART"
  echo "  config / baud / framing problem, NOT a missing connection. Re-check the baud"
  echo "  on that pin's UART with the passive UART/baud sweep."
  exit 0
fi

# No activity on any pin: classify the likely-RX pins and flag the physical cause.
echo "  POSSIBLE_POWER_OFF_OR_TX_NOT_CONNECTED — no likely-RX pin shows UART activity."
echo "  The GPS TX signal is NOT reaching the OpenRB RX pins (no electrical edges),"
echo "  which matches chars_seen=0 across every UART/baud. Most likely: GPS not"
echo "  powered, GPS TX not actually connected to OpenRB RX (D29 center 5-pin), a"
echo "  break/cold joint in the TX wire, or a dead/levels-mismatched GPS module."

if [ "${rx_total}" -gt 0 ] && [ "${rx_static_high}" -eq "${rx_total}" ]; then
  echo "  STATIC_HIGH — all RX pins idle HIGH with no transitions: line is held high"
  echo "  (GPS powered-but-silent, or pulled high / TX not driving). A live GPS should"
  echo "  send NMEA ~1 Hz even without a fix — silence suggests it is not transmitting."
  exit 5
fi
if [ "${rx_total}" -gt 0 ] && [ "${rx_static_low}" -eq "${rx_total}" ]; then
  echo "  STATIC_LOW — all RX pins held LOW with no transitions: most consistent with"
  echo "  GPS unpowered / TX tied low / wiring fault. Check GPS VCC and common GND."
  exit 6
fi

echo "  NO_PIN_ACTIVITY — RX pins are mixed/floating (no clean driven level, no"
echo "  meaningful transitions). With no pullup a disconnected pin floats; this is"
echo "  consistent with GPS TX not connected to the OpenRB RX pin."
exit 3
