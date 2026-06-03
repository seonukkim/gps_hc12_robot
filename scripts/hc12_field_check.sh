#!/usr/bin/env bash
# HC-12 field check helper.
#
# Prints the exact ordered terminal commands for the in-place HC-12 / UART
# diagnosis (no unplugging, no loopback, no AT mode). It does NOT try to run the
# concurrent steps itself: the OpenRB monitor and the station tool must run in
# two separate terminals at the same time. Read-only / diagnostic text only; no
# motor commands and no physical motion.
#
# Usage: scripts/hc12_field_check.sh

set -u

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR" || exit 1

# Best-effort port auto-detection (commands below still work if these are blank).
OPENRB_PORT="$(arduino-cli board list 2>/dev/null | awk '/OpenRB-150/ {print $1; exit}')"
STATION_PORT="$(ls /dev/cu.usbserial* /dev/cu.wchusbserial* /dev/cu.SLAB_USBtoUART* 2>/dev/null | grep -v usbmodem | head -n1)"
TS='$(date +%Y%m%d_%H%M%S)'

OPENRB_PORT_SHOWN="${OPENRB_PORT:-/dev/cu.usbmodemXXXX}"
STATION_PORT_SHOWN="${STATION_PORT:-auto}"

cat <<EOF
=== HC-12 field check — run these in order, in two terminals ===
repo:          $REPO_DIR
OpenRB port:   ${OPENRB_PORT:-<not detected: plug in OpenRB>}
station port:  ${STATION_PORT:-<not detected: --port auto will search>}

Rule: HC-12 is expected on Serial3 (D14/D13); GPS stays on Serial2.
If the counterpart HC-12 side is OFF, total_bytes=0 / NO_RX is EXPECTED — add
--station-off to record the run as TEST_INVALID_STATION_OFF.

--- Step 1 — Terminal 1 (OpenRB): UART sweep upload + monitor (capture to log) ---
mkdir -p outputs/logs
arduino-cli compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-uart-sweep firmware/uart_port_sweep_probe
arduino-cli upload -p $OPENRB_PORT_SHOWN --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-uart-sweep firmware/uart_port_sweep_probe
arduino-cli monitor -p $OPENRB_PORT_SHOWN --config baudrate=115200 | tee outputs/logs/uart_sweep_openrb_$TS.log

--- Step 2 — Terminal 2 (station): USB stability, then read-only (run while Step 1 monitor is open) ---
uv run python tools/hc12_operational_diagnose.py --port $STATION_PORT_SHOWN --mode stability --duration-s 20 --reconnect
uv run python tools/hc12_operational_diagnose.py --port $STATION_PORT_SHOWN --mode read-only --duration-s 60 --reconnect --raw-log

--- Step 3 — Terminal 2 (station): write-only (watch SerialN_rx rise in Terminal 1) ---
uv run python tools/hc12_operational_diagnose.py --port $STATION_PORT_SHOWN --mode write-only --duration-s 60 --reconnect

--- Step 4 — Terminal 1 (OpenRB): HC-12 link probe on Serial3 (both sides must be powered) ---
arduino-cli compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-hc12-s3 --build-property 'compiler.cpp.extra_flags=-DHC12_PROBE_SERIAL_PORT=3' firmware/hc12_link_probe
arduino-cli upload -p $OPENRB_PORT_SHOWN --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-hc12-s3 firmware/hc12_link_probe
arduino-cli monitor -p $OPENRB_PORT_SHOWN --config baudrate=115200

--- Step 5 — Terminal 2 (station): ping-pong (with Step 4 running) ---
uv run python tools/hc12_operational_diagnose.py --port $STATION_PORT_SHOWN --mode ping-pong --duration-s 60 --reconnect

--- Step 6 — Generate the Markdown diagnosis report ---
uv run python tools/hc12_diagnose_report.py
#   add --station-off if the counterpart HC-12 side was off during the run:
# uv run python tools/hc12_diagnose_report.py --station-off
EOF
