# Field Test Log

Use this file as the project-level index for bench and field tests. Detailed raw
logs should remain in `data/`, `outputs/logs/`, or another run-specific output
directory.

All motor tests are wheel-off-ground unless a later entry explicitly documents a
safe ground-contact setup.

## Log Template

```text
Date:
Location:
Operator:
Firmware marker:
Station command:
Serial port:
GPS fix:
RC mode:
Purpose:
Procedure:
Observed result:
STOP behavior:
Artifacts/logs:
Next action:
```

## 2026-05-26: Integrated Firmware Upload And Manual Direction Work

Firmware marker:

```text
openrb_robot_controller station-manual rc-cardinal-remap 2026-05-26
```

Station command:

```text
not a station motion test; OpenRB USB firmware upload and USBDBG check
```

Serial port:

```text
/dev/cu.usbmodem12101
```

Purpose:

- Put the repository integrated rover firmware onto the OpenRB.
- Preserve manual control and station manual protocol behavior.
- Correct RC manual axis mapping so straight up/down are intended to be
  forward/reverse.

Observed result:

- Arduino compile succeeded.
- OpenRB upload succeeded.
- Flash verify succeeded.
- USBDBG reported neutral command state:
  - `manual_steer_cmd=0.000`
  - `manual_throttle_cmd=0.000`
  - `left_cmd=0.000`
  - `right_cmd=0.000`

STOP behavior:

- Neutral startup confirmed through USBDBG.
- Full wheel-off-ground direction validation still needs operator observation.

Artifacts/logs:

- firmware source: `firmware/openrb_robot_controller/openrb_robot_controller.ino`
- procedure docs: `docs/manual_control.md`

Next action:

- Wheel-off-ground cardinal direction validation:
  - straight up should produce forward
  - straight down should produce reverse
  - straight left/right should steer without large throttle bias

## 2026-05-26: GPS Serial3 Zero-Byte Result And UART Probe Setup

Sketch:

```text
GPS-only test on OpenRB-150, Serial3 RX D13 at 9600
```

Observed output:

```text
chars_1s=0 total_chars=0 tinygps_chars=0 status=NO_FIX sats=NA hdop=NA age_ms=NA
```

Interpretation:

- The sketch is running and USB serial monitor works.
- No GPS bytes are reaching the configured `Serial3` input.
- This is a wiring, selected UART, baudrate, module power, or GPS output
  configuration problem until proven otherwise.
- This is not only a `NO_FIX` / satellite visibility problem.

Repository action:

- Added standalone safe probe:
  `firmware/gps_uart_probe/gps_uart_probe.ino`.
- Added GPS bring-up workflow:
  `docs/gps_bringup.md`.
- Did not modify `firmware/openrb_robot_controller/openrb_robot_controller.ino`.
- Did not add any motor-driving behavior.

Compile checks:

| Candidate | Result |
|---|---|
| `Serial3` RX `D13` at `9600` | pass |
| `Serial3` RX `D13` at `38400` | pass |
| `Serial3` RX `D13` at `115200` | pass |
| `Serial2` at `9600` | pass |
| `SoftwareSerial` RX `D8` / TX `D9` at `9600` | not supported; `SoftwareSerial.h` unavailable in this OpenRB build |

Next action:

- Run the standalone probe in the order documented in
  `docs/gps_bringup.md`.
- First rerun `Serial3` RX `D13` at `9600` with the new probe so the raw
  preview field confirms whether bytes are truly absent.

## 2026-05-26: GPS Serial2 UART Success, Initial No-Fix Result

Sketch:

```text
firmware/gps_uart_probe/gps_uart_probe.ino
GPS_PROBE_MODE=2
GPS_PROBE_BAUD=9600
```

Observed output summary:

```text
selected_port=Serial2 baud=9600
chars_1s roughly 349-370
raw_preview contains $GPRMC, $GPVTG, $GPGGA
tinygps_chars increases
sats fluctuates between 0 and 3
hdop around 3.65 or 99.99
fix=false
lat=NA lon=NA
```

Interpretation:

- GPS UART communication is working on `Serial2` at `9600`.
- The GPS is connected to the central OpenRB connector, not the `Serial3`
  D13/D14 pins used by the earlier repo assumption.
- `Serial3` D13/D14 probe failures are explained by current wiring, not by
  GPS module failure.
- Remaining issue is satellite fix quality; likely indoor or poor sky-view
  testing.

Architecture note:

- Integrated firmware currently defines `HC12_SERIAL` as `Serial2` and
  `GPS_SERIAL` as `Serial3`.
- The current GPS wiring also uses `Serial2`, so GPS and HC-12 UART allocation
  must be decided before integrating live GPS with manual HC-12 control.
- Recommendation is to preserve the known-working HC-12 manual control path
  unless the instructor or hardware constraints require moving it.

Historical next decision, later superseded by final Option A:

- Option A: keep GPS on `Serial2` and move HC-12 to another verified UART.
- Option B: keep HC-12 on `Serial2` and move GPS to verified `Serial3` pins.

Superseded by:

- `2026-05-26: GPS Serial2 Fix Success`

Repeat command:

```bash
arduino-cli monitor -p /dev/cu.usbmodem12101 --config baudrate=115200
```

Logging variant:

```bash
mkdir -p outputs/logs
arduino-cli monitor -p /dev/cu.usbmodem12101 --config baudrate=115200 | tee outputs/logs/gps_outdoor_fix_serial2_$(date +%Y%m%d_%H%M%S).log
```

## 2026-05-26: GPS Serial2 Fix Success

Sketch:

```text
firmware/gps_uart_probe/gps_uart_probe.ino
GPS_PROBE_MODE=2
GPS_PROBE_BAUD=9600
```

Observed final result:

```text
selected_port=Serial2 baud=9600
chars_1s roughly 350-520
raw_preview includes $GPRMC, $GPVTG, $GPGGA, $GPGSV, $GPGLL
tinygps_chars increases
fix=true
lat around 35.57107
lon around 129.1860
sats around 5
hdop around 1.61-1.62
```

Conclusion:

- GPS UART bring-up succeeded.
- GPS is physically connected to the central OpenRB connector.
- The confirmed working GPS path is `Serial2` at `9600` baud.
- `Serial3` D13/D14 tests at `9600`, `38400`, and `115200` produced zero
  bytes because the current wiring is not on D13/D14.
- Those `Serial3` failures are invalid for the current wiring and are not GPS
  module failures.

Architecture note:

- Integrated firmware currently defines `HC12_SERIAL` as `Serial2`.
- GPS is physically confirmed on `Serial2`.
- Integrated firmware currently defines `GPS_SERIAL` as `Serial3`.
- This is a likely UART conflict that must be resolved before live integrated
  GPS telemetry can be trusted with HC-12 manual control.

Next milestone:

- Do not implement autonomy.
- Do not modify motor control.
- Do not weaken STOP, failsafe, heartbeat timeout, or manual control.
- Historical UART allocation decision was Option B, now superseded.
- Serial3 physical pin mapping is unresolved; locate actual `Serial3` RX/TX
  using loopback and pin-finder tests.
- Next software milestone should be a GPS diagnostic integrated firmware mode,
  not autonomous movement.

## 2026-05-26: Historical UART Plan Switched To Option A

Historical decision, now superseded by Fixed Wiring Plan:

- Keep GPS on the current central OpenRB connector.
- Treat the current central connector as the confirmed GPS `Serial2` path.
- Final target mapping:
  - `GPS_SERIAL=Serial2`
  - `HC12_SERIAL=Serial3`
- HC-12 appears to be mounted under or behind the OpenRB board and needs its
  UART wiring verified separately.
- The purple module appears to be an IMU on an I2C-style connection; do not
  treat it as UART.

Superseded:

- Previous Option B decision is superseded.
- Do not move GPS to Serial3.
- Do not treat the Serial3 D13/D14 zero-byte GPS tests as GPS failure; the GPS
  was not wired there.

Historical next hardware milestone, no longer current:

- Identify actual Serial3 RX/TX physical pins.
- Run Serial3 TX-to-RX loopback.
- Move HC-12 data lines to verified Serial3 RX/TX.
- Run HC-12 Serial3 echo test.
- Only after that, update `openrb_robot_controller` mapping.

Current correction:

- Do not execute this rewiring plan under the Fixed Wiring Plan.
- Do not move GPS.
- Do not move HC-12.

Safety boundary:

- Do not modify motor control.
- Do not implement autonomy.
- Do not weaken STOP, heartbeat timeout, failsafe, or manual control.

## 2026-05-27: Fixed Wiring Plan

Final decision:

- GPS cannot be moved.
- HC-12 cannot be moved.
- Proceed with current physical wiring.
- GPS stays on the current central OpenRB connector.
- The current central connector is confirmed as `Serial2` at `9600`.
- GPS NMEA receive and `fix=true` are confirmed.
- HC-12 appears mounted under or behind the OpenRB board; audit its current
  data-line wiring before assuming it shares or does not share GPS `Serial2`.
- The purple module appears to be an IMU on an I2C-style connection; do not
  treat it as UART.

Superseded:

- Previous Option A and Option B UART-rewiring plans are superseded.
- Do not move GPS.
- Do not move HC-12.

Decision table:

| Current HC-12 wiring audit result | Decision |
|---|---|
| HC-12 is independent from GPS `Serial2` | Proceed with integrated GPS on `Serial2` plus HC-12 telemetry after diagnostics confirm both paths can coexist. |
| HC-12 shares GPS `Serial2` | Do not use GPS and HC-12 simultaneously. Use USB/onboard mission flow for GPS-dependent work and mark HC-12 operation blocked by fixed hardware. |

Next milestone:

- Integrated GPS `Serial2` diagnostic firmware mode.
- Current HC-12 wiring audit.
- Receive-only station telemetry test if safe.
- Station-side path planning dry-run.

Safety boundary:

- Do not modify motor control.
- Do not implement autonomy.
- Do not weaken STOP, heartbeat timeout, failsafe, manual override, or RC
  safety.

## 2026-05-27: Default Firmware Restored And Mode Expectations

Observed default firmware USBDBG:

```text
fixed_wiring_gps_serial2_diag=false
hc12_enabled=true
gps_chars=0
mode=AUTO_READY
auto_sw=true
mode_us≈2000
station_deadman=false
control_source=STOP
```

Interpretation:

- Default `openrb_robot_controller` was restored successfully.
- `gps_chars=0` is expected in the default build because default firmware still
  reads GPS from `Serial3`, while the fixed GPS wiring is on `Serial2`.
- This should not be treated as GPS failure.
- Manual control appears stopped because the RC mode switch is currently
  AUTO_READY. To validate manual control, switch RC mode out of AUTO and verify
  `control_source=RC_MANUAL`.
- `FIXED_WIRING_GPS_SERIAL2_DIAG` is for GPS testing only:
  - `GPS_SERIAL=Serial2`
  - HC-12 disabled/ignored
  - motors forced neutral
  - manual driving does not work by design

Do not repeat:

- Do not expect GPS in the default build under current fixed wiring.
- Do not expect manual driving in the GPS diagnostic build.
- Do not connect both OpenRB USB and station USB-serial during OpenRB upload if
  `arduino-cli` selects the wrong upload port.
- If upload fails because it selected `/dev/cu.usbserial-02444963`, unplug the
  station USB-serial and upload with only OpenRB connected.

Next architecture:

- For current fixed wiring, implement future GPS `Serial2` plus RC switch mode.
- Auto OFF: RC manual drive.
- Auto ON: onboard GPS mission/autonomy after separate safety design.
- HC-12 is not used in this mode until hardware can be revised or proven
  independent from GPS `Serial2`.
- Station-side path planning remains dry-run.

## 2026-05-27: GPS Serial2 Diagnostic Sky-Fix Success

Firmware marker:

```text
openrb_robot_controller with FIXED_WIRING_GPS_SERIAL2_DIAG=1
```

Observed USBDBG:

```text
fixed_wiring_gps_serial2_diag=true
hc12_enabled=false
gps_chars increased continuously
gps_fix=true after moving the external GPS antenna farther outside/open sky
gps_lat/gps_lon appeared
gps_sats valid
gps_hdop valid
```

Observed safety state:

- HC-12 was disabled/ignored by the diagnostic build.
- Motors remained disarmed/neutral.
- No manual driving was expected or attempted in this diagnostic build.

Interpretation:

- Integrated firmware can read the fixed GPS wiring on `Serial2` at `9600`
  when `FIXED_WIRING_GPS_SERIAL2_DIAG=1` is enabled.
- The previous default-build `gps_chars=0` result is still expected and is not
  GPS failure; default firmware reads GPS from `Serial3`.
- The previous `gps_sats=0` and `gps_hdop=99.99` result was poor
  indoor/window-side satellite reception, not UART or firmware failure.
- Outdoor/open-sky placement is required for reliable first fix. Indoor or
  window-side tests may receive NMEA bytes while failing to acquire enough
  satellites for `gps_fix=true`.
- Rain did not prevent fix when the antenna had open-sky exposure, but
  electronics and antenna connectors must be protected from water.

Checklist:

- `gps_chars` increasing means UART is OK.
- `gps_sats=0` and `gps_hdop=99.99` means no satellite acquisition.
- Move the antenna outside/open sky before suspecting code.
- Protect electronics and antenna connectors from rain.

Next action:

- Keep default firmware for HC-12/manual-control baseline.
- Use the diagnostic build only for GPS `Serial2` debug with motors neutral.
- Do not implement autonomy from this result alone.

## Known Manual Direction Attempts

These are recorded to prevent repeating the same fixes:

| Attempt | Observed Result | Status |
|---|---|---|
| old unknown board firmware | manual control existed, but source was not in repo | replaced by integrated repo firmware |
| first 45-degree remap | left/right behaved like forward/reverse | rejected |
| direct CH1/CH2 map | straight up/down did not align with forward/reverse | rejected |
| direct CH2 inversion | upper-left became forward and lower-right became reverse | rejected |
| current cardinal remap | intended to rotate raw diagonal axes into straight up/down/left/right | uploaded; needs wheel-off-ground direction validation |

## 2026-05-03: Historical Verified Status From Existing Docs

Source:

- `docs/current_hardware_status.md`
- `docs/project_notes/repo_audit.md`

Confirmed at that time:

- OpenRB USB debug working.
- RC receiver PPM input working.
- RC manual mode working.
- Failsafe STOP behavior working.
- GPS module communication working.
- GPS FIX confirmed.

Pending at that time:

- station-side HC-12 USB confirmation
- end-to-end station HC-12 link test
- GPS telemetry schema reconciliation
