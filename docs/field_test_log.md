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

## 2026-05-27: Unified Fixed-Wiring RC + GPS Dry-Run Validation

Firmware marker:

```text
openrb_robot_controller with FIXED_WIRING_GPS_SERIAL2_RC_AUTONOMY_DRYRUN=1
```

Observed configuration:

- `GPS_SERIAL=Serial2`
- HC-12 disabled/ignored
- GPS antenna outside/open sky produced `gps_fix=true`

MANUAL mode observed:

- RC manual control works.
- `control_source=RC_MANUAL`.
- Stick input changes `left_cmd` and `right_cmd`.

AUTO mode observed:

- `autonomy_dryrun=true`.
- GPS fields are printed.
- Target distance/bearing fields are printed.
- `left_cmd=0` and `right_cmd=0`.
- No motor movement in AUTO dry-run.

Rule:

- This is the first firmware mode where MANUAL and GPS dry-run coexist in one
  firmware.
- AUTO is still computation-only.
- Real motion is not enabled yet.

## 2026-05-27: Station-Side Coverage Path Planning Dry-Run

Command tested:

```bash
uv run python scripts/station/plan_coverage_path.py \
  --point-a 35.571070,129.186000 \
  --point-b 35.571070,129.186300 \
  --sweep-width-m 20.0 \
  --lane-spacing-m 5.0 \
  --speed-mps 0.4 \
  --mission-name codex_station_path_smoke
```

Observed result:

- CLI generated `mission.json`.
- CLI generated `mission.csv`.
- CLI generated `preview.png`.
- Inputs included point A/B, sweep width, and lane spacing.
- Preview showed lawnmower/boustrophedon lanes.
- No rover firmware was modified.
- No commands were sent to the rover.
- This is PC/Mac-side dry-run only.

Artifacts:

```text
outputs/missions/codex_station_path_smoke/mission.json
outputs/missions/codex_station_path_smoke/mission.csv
outputs/missions/codex_station_path_smoke/preview.png
```

Rule:

- This mission output is not yet executed by the rover.
- Next step is onboard mission dry-run, not real motion.

## 2026-05-28: Station Planner Geometry Correction

Issue found:

- The first station-side coverage planner interpreted Point A and Point B as the
  baseline of one side of the work area.
- `sweep_width_m` expanded the area perpendicular to that baseline.
- That made Point B appear on the first lane instead of representing the final
  opposite corner of the coverage rectangle.

Correction:

- Default planner mode is now `corner-rectangle`.
- Point A is the start corner of the work rectangle.
- Point B is the opposite/end corner of the work rectangle.
- A/B define an axis-aligned rectangle in the local East/North frame.
- `lane_spacing_m` is the sweep interval.
- `sweep_width_m` is not used in the default mode.
- Generated lanes must remain inside or on the coverage boundary.
- A final connector waypoint is added when needed so the mission ends exactly at
  Point B.

Corrected dry-run command:

```bash
uv run python scripts/station/plan_coverage_path.py \
  --point-a 35.571070,129.186000 \
  --point-b 35.571250,129.186300 \
  --lane-spacing-m 5.0 \
  --speed-mps 0.4 \
  --mission-name codex_corner_rectangle_smoke
```

Observed corrected result:

- `dry_run=true`.
- `sends_rover_commands=false`.
- `lane_count=6`.
- `waypoint_count=13`.
- `mission.json`, `mission.csv`, and `preview.png` were generated under
  `outputs/missions/codex_corner_rectangle_smoke/`.

Safety:

- No rover firmware was modified.
- No serial port was opened.
- No HC-12 frames or rover commands were sent.
- This remains PC/Mac-side mission-file generation only.

## 2026-05-28: Path Planning Visualization Complete

Completed state:

- Station-side planner generated `mission.json`, `mission.csv`, and
  `preview.png`.
- Preview image is sufficient to review Point A, Point B, lane order, coverage
  boundary, and final endpoint before any rover-side execution work.
- This is station-side path visualization only.

Edge/remainder policy:

- If the rectangle extent is not exactly divisible by `lane_spacing_m`, a small
  remaining margin at the edge is acceptable.
- Do not add an extra lane outside the boundary just to remove the margin.
- Keep generated lanes boundary-safe.

Safety and next step:

- No rover firmware was modified.
- No serial port was opened.
- No HC-12 frames or rover commands were sent.
- The next step is onboard mission dry-run, not real motion.
- Real autonomous motion remains disabled until a separate safety-gated
  milestone.

## 2026-05-28: Unified Fixed-Wiring RC + GPS Dry-Run Complete

USBDBG build identification:

- `fixed_wiring_gps_serial2_diag=false`
- `hc12_enabled=false`
- `autonomy_dryrun=true`

Interpretation:

- The running firmware was the unified fixed-wiring RC + GPS dry-run build.
- It was not the GPS-only diagnostic build.
- It was not the default HC-12/manual legacy build.

AUTO / dry-run safety:

- `mode=AUTO_READY`
- `auto_sw=true`
- `control_source=STOP`
- `left_cmd=0.000`
- `right_cmd=0.000`

This confirms AUTO dry-run did not drive motors.

MANUAL:

- `mode=MANUAL`
- `auto_sw=false`
- `control_source=RC_MANUAL`
- RC stick input changed `manual_steer_cmd`, `manual_throttle_cmd`,
  `left_cmd`, and `right_cmd`.

This confirms manual driving remains available in the unified dry-run build.

GPS:

- `gps_chars` increased continuously.
- `gps_fix=true` appeared when the external antenna had open-sky exposure.
- `gps_lat` and `gps_lon` appeared.
- `target_distance_m` and `target_bearing_deg` were computed.
- Earlier `gps_sats=0` / `gps_hdop=99.99` was poor antenna placement, not UART
  failure.

Safety interpretation:

- AUTO is still computation-only.
- No real autonomous motion is enabled yet.
- MANUAL remains the recovery/manual override path.
- HC-12 is disabled in this fixed-wiring GPS mode because GPS and HC-12 cannot
  both use the current `Serial2` wiring safely.
- Next step is single-waypoint controlled motion preparation, not full
  coverage/lawnmower driving.

## 2026-05-28: Single-Waypoint Candidate Dry-Run And GPS Frame Issue

Observed candidate dry-run:

- Build flags:
  - `FIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT=1`
  - `AUTO_MOTION_ARMED=0`
- USB debug printed:
  - `single_waypoint_experiment=true`
  - `auto_motion_armed=false`
  - `auto_motor_inhibit=true`
  - `gps_fix=true` eventually
  - `target_distance_m`
  - `target_bearing_deg`
- AUTO kept `left_cmd=0` and `right_cmd=0`.
- MANUAL returned to `RC_MANUAL`, and stick input changed motor command fields.

Interpretation:

- Candidate dry-run safety is confirmed.
- Basic GPS distance/bearing computation is confirmed.
- Real motion is still not enabled.

Important sensor-frame issue:

- The GPS antenna was placed far outside while the rover body remained indoors.
- Therefore `gps_lat` / `gps_lon` represented the antenna location, not the
  rover body location.
- This is acceptable for GPS reception validation.
- This is not valid for floor navigation.
- IMU data cannot fully correct a detached GPS antenna into rover body
  position.
- IMU may help later with heading/rotation sensing, but it does not replace a
  rover-mounted GPS position source.

Decision:

- Do not proceed to floor waypoint driving yet.
- Do not approve `AUTO_MOTION_ARMED=1` floor testing yet.
- Real outdoor navigation requires the GPS antenna to be rigidly mounted on the
  rover or its offset from the rover body to be fixed and known.

Next required validation before motion:

- IMU I2C scan.
- IMU orientation and axis check.
- GPS mounted/open-sky candidate retest.
- Wheel-off-ground bench test only after safety gates and sensor assumptions
  are clear.

## 2026-05-28: Fixed D11/D12 IMU Bit-Bang Scanner Inconclusive

Observed setup:

- Physical IMU wiring should not be moved.
- Physical wiring is believed to be:
  - SCL: OpenRB D12 / PA09 / SCL(SC2)
  - SDA: OpenRB D11 / PA08 / SDA(SC2)

Observed bit-bang scanner results:

- The original bit-bang scanner produced impossible all-address detection.
- The hardened bit-bang scanner was then tested with:
  - SDA=D11, SCL=D12
  - SDA=D12, SCL=D11
- Both tests repeatedly showed:
  - `released_sda=LOW`
  - `released_scl=LOW`
  - `SDA stuck low`
  - `SCL stuck low`
  - `raw_found_count=0`
  - `valid_found_count=0`
  - `stable_valid_address=NA`

Initial interpretation:

- IMU is not verified.
- All-address detection was invalid and must not be treated as success.
- The fixed D11/D12 PA08/PA09 wiring may require a SERCOM2 hardware I2C setup
  instead of bit-bang.
- Another possible cause is IMU power, GND, pullup, or a bus-stuck issue.
- Do not rely on IMU data for autonomy yet.
- Do not move IMU wires casually.

Updated variant-file finding:

- OpenRB-150 variant files confirm:
  - Arduino D11 = SDA = PA08
  - Arduino D12 = SCL = PA09
  - `PIN_WIRE_SDA = 11`
  - `PIN_WIRE_SCL = 12`
  - `Wire` is constructed using those pins
- Therefore, do not create a custom SERCOM2 scanner yet.
- The correct next diagnostic is a robust default `Wire` scanner for D11/D12.
- D11/D12 stuck-low should be treated as an electrical or bus issue, such as
  power, GND, pullups, or a stuck device, not as a pin mapping issue.
- If the robust default `Wire` scanner also fails, continue GPS+RC workflow
  without IMU support.
- Do not enable motion based on IMU assumptions.

## 2026-05-28: Default Wire Scanner Confirms D11/D12 Bus Stuck Low

Observed setup:

- OpenRB-150 variant files confirm:
  - Arduino D11 = SDA = PA08
  - Arduino D12 = SCL = PA09
  - `PIN_WIRE_SDA = 11`
  - `PIN_WIRE_SCL = 12`
  - `Wire` is constructed using those pins
- Current IMU wiring matches the default `Wire` pins.
- Physical IMU wiring should not be moved casually.

Observed robust `Wire` scanner result:

- The scanner runs and prints repeated scan passes.
- Every pass shows:
  - `pre_scan_sda=LOW`
  - `pre_scan_scl=LOW`
  - `BUS_STUCK_LOW_BEFORE_SCAN`
  - `found_count=0`
  - `stable_valid_address=NA`

Interpretation:

- The scanner is not hanging.
- It is correctly refusing to scan because the I2C bus is stuck low before
  address probing.
- IMU is not verified and must not be used for autonomy yet.
- This is likely an electrical or bus issue rather than an Arduino pin mapping
  issue.
- Possible causes include IMU power, GND, missing pullups, bus held low,
  connector/solder issue, or sensor board issue.
- Previous all-address bit-bang scan results remain invalid and must not be
  treated as successful IMU detection.

Decision:

- Continue the GPS+RC safety-gated workflow without IMU for now.
- Do not enable motion based on IMU assumptions.

## 2026-05-28: Nearby Single-Waypoint Candidate Retest Blocked By Target Override

Build/upload:

- Build/upload succeeded with intended flags:
  - `FIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT=1`
  - `AUTO_MOTION_ARMED=0`
  - `SINGLE_WP_TARGET_LAT=35.5716800`
  - `SINGLE_WP_TARGET_LON=129.1866516`
- The compile command intended to override the target to a nearby point.

Observed USBDBG:

- Runtime target remained the old placeholder:
  - `target_lat=35.571120`
  - `target_lon=129.186050`
- GPS fix was achieved.
- At least one log line reached:
  - `gps_hdop=1.19`
  - `gps_ready=true`
- Because the target was still the old placeholder, `target_distance_m`
  remained around `40` to `60` m.
- `distance_allowed=false`.
- `safety_ready=false`.
- `candidate_left_cmd=0.000`.
- `candidate_right_cmd=0.000`.
- `AUTO_MOTION_ARMED=0` correctly kept:
  - `final_left_cmd=0.000`
  - `final_right_cmd=0.000`
- MANUAL mode still returned to `control_source=RC_MANUAL`.

Interpretation:

- This is a safe failed validation, not a successful nearby candidate-command
  test.
- Safety gates worked correctly.
- AUTO motor inhibit worked correctly.
- GPS can become ready.
- The next blocker is target override plumbing.
- The runtime `target_lat` / `target_lon` fields are the source of truth for
  interpreting `target_distance_m`, `distance_allowed`, and `safety_ready`.
- Bench test is not approved yet.
- Floor driving is not approved.

## 2026-05-28: Single-Waypoint Target Override Verified, Distance Gate Blocks

Build/upload:

- Build/upload succeeded with:
  - `FIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT=1`
  - `AUTO_MOTION_ARMED=0`
  - `SINGLE_WP_TARGET_LAT=35.5710210`
  - `SINGLE_WP_TARGET_LON=129.1864016`

Observed USBDBG target fields:

- `target_override_enabled=true`
- `target_source=compile_time`
- `target_lat_macro=35.5710210`
- `target_lon_macro=129.1864016`
- `target_lat=35.571021`
- `target_lon=129.186402`

Observed GPS and safety fields:

- GPS fix was true.
- Current GPS position was around:
  - `gps_lat≈35.56752` to `35.56756`
  - `gps_lon≈129.18688`
- `target_distance_m` remained around `380` to `392` m.
- `max_target_distance_m=30.0`.
- `distance_allowed=false`.
- `safety_ready=false`.
- `candidate_left_cmd=0.000`.
- `candidate_right_cmd=0.000`.
- `AUTO_MOTION_ARMED=0` correctly kept:
  - `final_left_cmd=0.000`
  - `final_right_cmd=0.000`

Interpretation:

- Target override plumbing is fixed.
- This was a safe blocked validation, not a nearby candidate-command success.
- `distance_allowed=false` is expected because `target_distance_m` exceeded
  `max_target_distance_m`.
- Target override success must be checked separately from
  `distance_allowed` / `safety_ready`.
- The next step is to recompute a nearby target from the current GPS position
  and rerun with `AUTO_MOTION_ARMED=0`.
- Bench test is still blocked.
- Floor driving is still blocked.

## 2026-05-29: Next-Day GPS Retest Blocked By Stale Target And GPS Quality

Setup:

- The GPS antenna was placed outside again on the next day.
- Runtime GPS position changed to approximately:
  - `gps_lat=35.571310`
  - `gps_lon=129.188630`
- Firmware still used the previous compile-time target:
  - `target_lat=35.567560`
  - `target_lon=129.186792`
- `target_override_enabled=true`.
- `target_source=compile_time`.

Observed GPS and safety fields:

- `target_distance_m` was around `448.9` m.
- `max_target_distance_m=30.0`.
- `distance_allowed=false`.
- `safety_ready=false`.
- `gps_fix=true` appeared, but `gps_ready=false` remained because GPS
  quality/freshness was not stable:
  - `gps_age_ms` was very large in many lines
  - `gps_sats` fluctuated
  - `gps_hdop` was often `99.99` and only occasionally around `4.7`
- `AUTO_MOTION_ARMED=0`.
- `auto_motor_inhibit=true`.
- Final outputs stayed neutral:
  - `final_left_cmd=0.000`
  - `final_right_cmd=0.000`

Interpretation:

- This is a safe blocked validation, not a candidate-command success.
- Safety gates worked correctly.
- Target override is working.
- The previous nearby target became stale because the GPS antenna was placed at
  a new location on the next day.
- `distance_allowed=false` is expected because `target_distance_m` exceeded
  `max_target_distance_m`.
- `gps_fix=true` alone is not enough; GPS age, HDOP, and satellite count must
  also satisfy readiness gates.
- The next step is to recompute a nearby target from the current GPS location
  and rerun with `AUTO_MOTION_ARMED=0`.
- Bench test remains blocked.
- Floor driving remains blocked.

## 2026-05-29: Nearby Single-Waypoint Attempt Blocked By Actual Fix Distance

Build/upload:

- Build/upload succeeded with:
  - `FIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT=1`
  - `AUTO_MOTION_ARMED=0`
  - `SINGLE_WP_TARGET_LAT=35.5713100`
  - `SINGLE_WP_TARGET_LON=129.1885416`

Observed USBDBG target fields:

- `target_override_enabled=true`
- `target_source=compile_time`
- `target_lat_macro=35.5713100`
- `target_lon_macro=129.1885416`
- `target_lat=35.571310`
- `target_lon=129.188542`

Observed GPS and safety fields:

- GPS UART was alive and `gps_chars` increased.
- GPS fix was eventually acquired in MANUAL:
  - `gps_fix=true`
  - `gps_lat=35.571384`
  - `gps_lon=129.187514`
  - `gps_sats=4`
  - `gps_hdop=3.39` to `4.12`
  - `gps_age_ms` was initially fresh but later grew stale
- The target was not nearby relative to the actual GPS fix:
  - `target_distance_m=93.3`
  - `max_target_distance_m=30.0`
- `distance_allowed=false`.
- `safety_ready=false`.
- `candidate_left_cmd=0.000`.
- `candidate_right_cmd=0.000`.
- `final_left_cmd=0.000`.
- `final_right_cmd=0.000`.
- `AUTO_MOTION_ARMED=0`.
- `auto_motor_inhibit=true`.

Interpretation:

- This is a safe blocked result, not a nearby candidate-command success.
- Target override is working.
- GPS UART and GPS fix acquisition are working.
- The main blocker is that the target was computed from a stale or assumed GPS
  position, not from the actual runtime GPS fix.
- `distance_allowed=false` is expected because `target_distance_m=93.3`
  exceeded `max_target_distance_m=30.0`.
- The next step is to recompute the target from the actual GPS fix position:
  `gps_lat=35.571384`, `gps_lon=129.187514`, then rerun with
  `AUTO_MOTION_ARMED=0`.
- Bench test remains blocked.
- Floor driving remains blocked.

## 2026-05-29: Window/Outside-Antenna GPS Attempt Blocked

Setup:

- The GPS antenna was placed or thrown outside from indoors.
- Build/upload used:
  - `FIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT=1`
  - `AUTO_MOTION_ARMED=0`
  - `SINGLE_WP_TARGET_LAT=35.5713840`
  - `SINGLE_WP_TARGET_LON=129.1874256`

Observed USBDBG target fields:

- `target_override_enabled=true`
- `target_source=compile_time`
- `target_lat_macro=35.5713840`
- `target_lon_macro=129.1874256`
- `target_lat=35.571384`
- `target_lon=129.187426`

Observed GPS and safety fields:

- Target override is working.
- GPS fix was eventually seen, but it was not stable enough for candidate
  validation:
  - `gps_fix=true` appeared
  - `gps_lat` around `35.571284`
  - `gps_lon` around `129.188456`
  - `gps_sats` often became `0`
  - `gps_hdop` often became `99.99`
  - `gps_age_ms` grew very large
- The target was not nearby relative to the actual GPS position:
  - `target_distance_m` around `93.9`
  - `max_target_distance_m=30.0`
- `distance_allowed=false`.
- `gps_ready=false`.
- `safety_ready=false`.
- `candidate_left_cmd=0.000`.
- `candidate_right_cmd=0.000`.
- `final_left_cmd=0.000`.
- `final_right_cmd=0.000`.
- `AUTO_MOTION_ARMED=0`.
- `auto_motor_inhibit=true`.

Interpretation:

- This is a safe blocked result, not a candidate-command success.
- GPS reception is possible, but indoor/window-side antenna placement is not
  stable enough for candidate validation.
- The GPS antenna position is not equivalent to the rover body position when
  the rover remains indoors.
- `gps_fix=true` alone is not enough; `gps_age_ms`, `gps_sats`, `gps_hdop`,
  `target_distance_m`, and `safety_ready` must also be checked.
- The next step is to go fully outdoors with the rover and GPS fixed together,
  acquire a fresh GPS fix in MANUAL, recompute the target from that actual fix,
  and rerun with `AUTO_MOTION_ARMED=0`.
- Bench test remains blocked.
- Floor driving remains blocked.

## 2026-05-29: Outdoor Manual/Auto Recovery And Stale-Target Safety Block

Setup:

- The rover/GPS was tested outdoors.
- The previous RC mode-switch issue was traced to the station/controller being
  off.
- After restoring the controller/link, Manual/Auto switching worked again.
- Build remained the single-waypoint experiment with `AUTO_MOTION_ARMED=0`.

Observed Manual/Auto recovery:

- AUTO was verified:
  - `mode=AUTO_READY`
  - `auto_sw=true`
  - `mode_us` around `2001` to `2002`
  - `control_source=STOP`
- MANUAL was verified:
  - `mode=MANUAL`
  - `auto_sw=false`
  - `mode_us` around `1000` to `1001`
  - `control_source=RC_MANUAL`
- Manual stick input was also verified:
  - `steer_us` / `throttle_us` changed
  - at least one MANUAL line produced nonzero `left_cmd` /
    `final_left_cmd`

Observed GPS and target state:

- GPS was usable outdoors at several points:
  - `gps_fix=true`
  - `gps_ready=true` appeared when HDOP was good
- The compile-time target was stale:
  - `target_lat=35.570675`
  - `target_lon=129.186769`
  - runtime GPS was around `35.5716,129.1875`
  - `target_distance_m` was around `100` to `131` m
  - `max_target_distance_m=30.0`

Observed safety state:

- `distance_allowed=false`.
- `safety_ready=false`.
- `candidate_left_cmd=0.000`.
- `candidate_right_cmd=0.000`.
- `AUTO_MOTION_ARMED=0`.
- `auto_motor_inhibit=true`.
- AUTO final commands stayed zero.

Interpretation:

- This is a partial success: RC Manual/Auto operation is recovered, and outdoor
  GPS can become usable.
- This is also a safety-blocked autonomy dry-run, not a nearby candidate-command
  success.
- The remaining blocker is stale target and GPS/timeout gate handling.
- The next step is to recompute a nearby target from the current runtime GPS
  fix and rerun with `AUTO_MOTION_ARMED=0`.
- Bench test remains blocked until `safety_ready=true` and nonzero candidate
  commands are observed while final outputs remain inhibited.
- Floor driving remains blocked.

## 2026-05-29: Outdoor Nearby Dry-Run Reached Distance Gate But Stayed Blocked

Build/upload:

- Build/upload used:
  - `FIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT=1`
  - `AUTO_MOTION_ARMED=0`
  - `SINGLE_WP_TARGET_LAT=35.5707680`
  - `SINGLE_WP_TARGET_LON=129.1867906`

Observed USBDBG target fields:

- `target_override_enabled=true`
- `target_source=compile_time`
- `target_lat_macro=35.5707680`
- `target_lon_macro=129.1867906`
- `target_lat=35.570768`
- `target_lon=129.186791`

Observed GPS and distance state:

- GPS was good outdoors in many lines:
  - `gps_fix=true`
  - `gps_ready=true` appeared repeatedly
  - `gps_sats` often `7` to `8`
  - `gps_hdop` reached around `0.95` to `1.98`
  - `gps_age_ms` was fresh in many lines
- The rover/GPS eventually became close enough to the target:
  - `target_distance_m` decreased from around `45` to `55` m to around
    `27.1`, `25.4`, `23.5`, `21.1`, `18.8`, and `18.5` m
  - `distance_allowed=true` was observed after `target_distance_m` became less
    than `max_target_distance_m=30.0`

Observed blocker:

- `mode` stayed mostly `MANUAL`.
- `auto_sw=false`.
- `timeout_ok=false`.
- `safety_ready=false`.
- `candidate_left_cmd=0.000`.
- `candidate_right_cmd=0.000`.
- A brief failsafe-like PPM glitch was also observed:
  - `mode=FAILSAFE`
  - `rc_ok=false`
  - `steer_us` around `495`
  - `throttle_us` around `2504`
  - `control_source=STOP`
- `AUTO_MOTION_ARMED=0` remained safe.

Interpretation:

- This is partial progress and a safe blocked validation, not a full success.
- Target override is working.
- Outdoor GPS is now good enough for candidate dry-run work.
- The distance gate can become true with a nearby target.
- `distance_allowed=true` in MANUAL is not enough.
- A successful candidate dry-run still requires the same condition in
  `AUTO_READY` with `gps_ready=true`, `distance_allowed=true`,
  `timeout_ok=true`, `safety_ready=true`, and candidate commands observed while
  final motor outputs remain inhibited.
- The repeated `timeout_ok=false` suggests the current timeout semantics are
  unsuitable for long MANUAL GPS-waiting workflows.
- Next step is either:
  - quick reset/reupload and immediate AUTO dry-run while `distance_allowed=true`
  - firmware improvement so timeout is based on AUTO entry rather than total
    boot/manual waiting time
- Bench test remains blocked.
- Floor driving remains blocked.

## 2026-05-29: Single-Waypoint AUTO-Entry Timeout Semantics Updated

Change:

- The single-waypoint experiment timeout now starts on AUTO entry instead of
  being consumed during boot or long MANUAL GPS-waiting time.
- Leaving AUTO resets the AUTO entry timestamp.
- MANUAL no longer consumes the AUTO candidate timeout.
- USB debug now reports:
  - `timeout_source=auto_entry`
  - `auto_entry_ms=NA` or a numeric timestamp
  - `auto_elapsed_ms=NA` or a numeric elapsed time
  - `timeout_limit_ms`
  - `timeout_ok`

Safety:

- Default rover behavior is preserved outside
  `FIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT`.
- `AUTO_MOTION_ARMED=0` remains the required validation build.
- AUTO final motor outputs remain inhibited when `AUTO_MOTION_ARMED=0`.
- RC channel mapping and motor mixing were not changed.
- Real motion is still not enabled.

Next validation:

- Rebuild and upload the single-waypoint experiment with `AUTO_MOTION_ARMED=0`.
- Wait for outdoor GPS readiness in MANUAL without consuming AUTO timeout.
- Switch to AUTO while nearby and verify:
  - `mode=AUTO_READY`
  - `gps_ready=true`
  - `distance_allowed=true`
  - `auto_entry_ms` is numeric
  - `auto_elapsed_ms` is below `timeout_limit_ms`
  - `timeout_ok=true`
  - `safety_ready=true`
  - candidate commands are nonzero
  - final outputs remain zero because `AUTO_MOTION_ARMED=0`
- Bench test remains blocked until that dry-run succeeds.
- Floor driving remains blocked.

## 2026-05-29: Outdoor Dry-Run Blocked By GPS No-Fix After Timeout Fix

Observed timeout and target diagnostics:

- Firmware printed the new timeout fields:
  - `timeout_source=auto_entry`
  - `auto_entry_ms=NA`
  - `auto_elapsed_ms=NA`
  - `timeout_limit_ms=15000`
  - `timeout_ok=true`
- This indicates the previous MANUAL-wait timeout issue is improved.
- Runtime target override was confirmed:
  - `target_override_enabled=true`
  - `target_source=compile_time`
  - `target_lat_macro=35.5708340`
  - `target_lon_macro=129.1869576`
  - `target_lat=35.570834`
  - `target_lon=129.186958`
  - `target_ready=true`

Observed RC state:

- RC was in MANUAL:
  - `mode=MANUAL`
  - `auto_sw=false`
  - `mode_us` around `1000` to `1001`
  - `control_source=RC_MANUAL`

Observed GPS no-fix state:

- GPS UART was alive:
  - `gps_chars` increased continuously
- GPS fix was not acquired:
  - `gps_fix=false`
  - `gps_lat=NA`
  - `gps_lon=NA`
  - `gps_sats=0`
  - `gps_hdop=99.99`
  - `gps_age_ms=NA`

Observed safety state:

- `target_distance_m=NA`.
- `distance_allowed=false`.
- `safety_ready=false`.
- `candidate_left_cmd=0.000`.
- `candidate_right_cmd=0.000`.
- `final_left_cmd=0.000`.
- `final_right_cmd=0.000`.
- `AUTO_MOTION_ARMED=0` remained safe.

Interpretation:

- This is a safe blocked validation, not an AUTO candidate dry-run success.
- Timeout semantics appear fixed or improved.
- Target override is working.
- RC MANUAL is working.
- The current blocker is GPS satellite fix, not timeout, target override, or RC
  mode mapping.
- `gps_chars` increasing only proves NMEA/serial input is alive; it does not
  mean the module has a valid position fix.
- `gps_sats=0` and `gps_hdop=99.99` mean no usable satellite acquisition for
  candidate validation.
- Next step is to reacquire a stable outdoor GPS fix in MANUAL, then attempt
  AUTO_READY validation with `AUTO_MOTION_ARMED=0`.
- Bench test remains blocked.
- Floor driving remains blocked.

## 2026-05-29: GPS Readiness And Stale-Coordinate Handling Improved

Problem:

- Recent USBDBG logs showed stale cached TinyGPS coordinates being printed as if
  they were an operational position:
  - `gps_fix=true`
  - `gps_lat=34.944214`
  - `gps_lon=128.985885`
  - `gps_sats=0`
  - `gps_hdop=99.99`
  - `gps_age_ms` increasing above `20` to `45` seconds
  - `gps_ready=false`
  - `target_distance_m=71943.9`
- This indicated that the old `gps_fix` field was based on TinyGPS cached
  `location.isValid()`, not a fresh usable GPS solution.

Firmware change:

- USBDBG now splits GPS validity into explicit fields:
  - `gps_location_valid`
  - `gps_location_fresh`
  - `gps_age_ok`
  - `gps_sats_ok`
  - `gps_hdop_ok`
  - `gps_ready`
  - `gps_block_reason`
- USBDBG prints readiness constants:
  - `gps_stale_ms`
  - `gps_min_sats`
  - `gps_max_hdop`
- `gps_ready` now requires all of:
  - valid TinyGPS location
  - location age no more than `GPS_STALE_MS`
  - satellites at least `GPS_MIN_SATS`
  - HDOP no more than `GPS_MAX_HDOP`
- Operational `gps_lat` / `gps_lon` now print `NA` unless `gps_ready=true`.
- Cached TinyGPS coordinates are separated into debug-only fields:
  - `gps_cached_lat`
  - `gps_cached_lon`
  - `gps_cached_age_ms`
- `target_distance_m` and `target_bearing_deg` are computed only when
  `gps_ready=true`.
- The single-waypoint experiment now prints `gps_coord_sane` and blocks safety
  if the ready GPS position is absurdly far from the compile-time target.
- USBDBG now includes lightweight NMEA status diagnostics:
  - `last_rmc_status` for `GPRMC` / `GNRMC`
  - `last_gga_fix_quality` for `GPGGA` / `GNGGA`

Safety:

- No motion was enabled.
- `AUTO_MOTION_ARMED=0` behavior remains inhibited.
- RC manual behavior, RC channel mapping, motor mixing, and default safety
  behavior were preserved.

Validation:

- Default `openrb_robot_controller` build compiled successfully.
- Single-waypoint experiment with `AUTO_MOTION_ARMED=0` compiled successfully.

## 2026-05-29: GPS Readiness Tiers Added For No-Motion Dry-Run

Problem:

- Recent outdoor logs showed a real but lower-quality GPS solution:
  - `last_rmc_status=A`
  - `last_gga_fix_quality=1`
  - `gps_location_valid=true`
  - `gps_location_fresh=true`
  - `gps_age_ok=true`
  - `gps_sats` around `3` to `5`
  - `gps_hdop` around `4.95` to `5.00`
- The previous `gps_ready` gate required strict motion-level quality
  (`gps_max_hdop=2.5`, `gps_min_sats=4`), so `target_distance_m` remained `NA`
  and no-motion candidate-command calculation could not be validated.

Firmware change:

- GPS readiness is now tiered:
  - `gps_solution_valid`: valid, fresh location with RMC status `A` or GGA fix
    quality at least `1` when those NMEA fields are available.
  - `gps_dryrun_ready`: solution valid, dry-run age limit, at least
    `GPS_DRYRUN_MIN_SATS=4`, and `GPS_DRYRUN_MAX_HDOP=6.0`.
  - `gps_motion_ready`: solution valid, motion age limit, at least
    `GPS_MOTION_MIN_SATS=5`, and `GPS_MOTION_MAX_HDOP=2.5`.
- `gps_ready` remains the stricter motion-level alias.
- USBDBG now prints:
  - `gps_solution_valid`
  - `gps_dryrun_ready`
  - `gps_motion_ready`
  - `gps_dryrun_block_reason`
  - `gps_motion_block_reason`
  - `gps_dryrun_stale_ms`
  - `gps_dryrun_min_sats`
  - `gps_dryrun_max_hdop`
  - `gps_motion_stale_ms`
  - `gps_motion_min_sats`
  - `gps_motion_max_hdop`
  - `dryrun_ready`
  - `motion_ready`
  - `safety_ready_source`
- In the single-waypoint experiment:
  - `AUTO_MOTION_ARMED=0` uses dry-run readiness for distance/bearing and
    candidate-command computation.
  - `AUTO_MOTION_ARMED=1` still requires motion readiness.
  - final outputs remain inhibited when `AUTO_MOTION_ARMED=0`.

Safety:

- No motion was enabled.
- Manual control, RC mapping, and motor mixing were preserved.
- HDOP around `5` is acceptable only for no-motion dry-run candidate
  calculation.
- HDOP around `5` is not approved for floor driving.
- Wheel-off-ground bench test remains blocked until dry-run candidate commands
  are observed with final commands inhibited.

Validation:

- Default `openrb_robot_controller` build compiled successfully.
- Single-waypoint experiment with `AUTO_MOTION_ARMED=0` compiled successfully.

## 2026-05-29: GPS-only Serial2 Probe Shows Intermittent Fix

Setup:

- Sketch: `firmware/gps_uart_probe`
- Build flags:
  - `GPS_PROBE_MODE=2`
  - `GPS_PROBE_BAUD=9600`
- Physical path: current GPS wiring on OpenRB `Serial2`

Observed:

- `Serial2` at `9600` continuously received NMEA characters.
- The probe output showed readable GPS data, so the GPS UART wiring and baudrate
  are likely correct.
- Most lines showed no usable satellite solution:
  - RMC status `V`
  - GGA fix quality `0`
  - `sats=0`
  - `hdop=99.99`
  - `fix=false`, or a stale cached TinyGPS++ fix
  - `lat` / `lon` as `NA`, or stale cached coordinates
- A few short bursts showed valid fixes:
  - RMC status `A`
  - valid latitude/longitude
  - `sats=4..5`
  - `hdop≈1.77..2.48`
- The valid fix did not remain stable. The output returned to RMC `V`, GGA
  quality `0`, `sats=0`, and `hdop=99.99`.

Interpretation:

- GPS UART receive is alive on `Serial2/9600`.
- GPS satellite acquisition is intermittent and currently not stable enough for
  autonomy validation.
- This is not a target override issue, not an RC issue, and not a timeout
  issue.
- TinyGPS++ may keep cached fix/lat/lon after RMC returns to `V`; cached
  coordinates must not be used for target distance or safety decisions.

Safety decision:

- Do not proceed to AUTO dry-run validation, wheel-off-ground bench testing, or
  floor driving until a stable GPS fix is observed.
- Stable GPS validation should require sustained RMC `A` or GGA fix quality
  `>=1`, nonzero satellites, acceptable HDOP, fresh age, and no immediate
  fallback to RMC `V` / GGA quality `0`.

## 2026-05-29: GPS Fix Recovered After Moving Rover Farther Outdoors

Setup:

- Sketch: `firmware/gps_uart_probe`
- Build flags:
  - `GPS_PROBE_MODE=2`
  - `GPS_PROBE_BAUD=9600`
- Physical path: current GPS wiring on OpenRB `Serial2`
- Change from previous failed/intermittent runs: rover/GPS was moved farther
  outdoors with clearer sky view.

Observed recovery sequence:

- Previous persistent no-fix was primarily caused by rover/GPS placement.
- After moving farther outside, the probe transitioned from `NO_FIX` to
  `INTERMITTENT_FIX`.
- Valid fix fields appeared with RMC status `A`, GGA fix quality `>=1`,
  valid lat/lon around `35.5708,129.1870`, satellites around `5`, and HDOP
  around `4.0`.
- The latest pasted lines then reached stable fix:
  - `last_rmc_status=A`
  - `last_gga_fix_quality=2`
  - `current_valid_fix=true`
  - `gps_probe_state=STABLE_FIX`
  - `lat≈35.570284..35.570296`
  - `lon≈129.187078`
  - `age_ms≈85..89`
  - `sats=9`
  - `hdop=3.56`
  - `valid_fix_seconds_consecutive=58..60`

Interpretation:

- GPS UART and the GPS module are working on `Serial2/9600`.
- GPS RF/sky-view placement is the dominant issue. Indoor, near-building, or
  partially covered positions can produce persistent `NO_FIX` even with
  continuous NMEA.
- `STABLE_FIX` requires sustained valid fix, not a momentary RMC `A` burst.
  The current probe rule is `valid_fix_seconds_consecutive >= 30`.

Safety decision:

- This clears the standalone GPS probe stability check for the current outdoor
  placement.
- Do not proceed to floor driving.
- The next software validation is main-controller
  `FIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT=1` with
  `AUTO_MOTION_ARMED=0`, after confirming `STABLE_FIX` outdoors and recomputing
  the target from the actual runtime GPS position.

## 2026-05-29: Main Controller GPS And AUTO Gate Recovery

Setup:

- Firmware: `firmware/openrb_robot_controller`
- Mode:
  - `FIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT=1`
  - `AUTO_MOTION_ARMED=0`
- Physical change: rover/GPS moved farther outdoors.

Observed GPS readiness:

- The earlier GPS no-fix issue was resolved by outdoor placement.
- Main-controller USBDBG showed good GPS quality:
  - `gps_location_valid=true`
  - `gps_location_fresh=true`
  - `gps_age_ok=true`
  - `gps_sats_ok=true`
  - `gps_hdop_ok=true`
  - `gps_solution_valid=true`
  - `gps_dryrun_ready=true`
  - `gps_motion_ready=true`
  - `gps_ready=true`
  - `gps_block_reason=OK`
  - `last_rmc_status=A`
  - `last_gga_fix_quality=2`
  - `gps_sats≈9..11`
  - `gps_hdop≈1.46`

Observed AUTO gate:

- AUTO entry was verified:
  - `mode=AUTO_READY`
  - `auto_sw=true`
  - `mode_us≈2001..2002`
  - `timeout_source=auto_entry`
  - `timeout_ok=true`
- Motor inhibition remained correct:
  - `AUTO_MOTION_ARMED=0`
  - `auto_motor_inhibit=true`
  - `final_left_cmd=0.000`
  - `final_right_cmd=0.000`

Blocked target result:

- The compile-time target was stale/far from the current GPS position:
  - `target_lat_macro=35.5702838`
  - `target_lon_macro=129.1869899`
  - current GPS was around `35.57050,129.18736`
  - `target_distance_m≈41`
  - `max_target_distance_m=30.0`
- Therefore:
  - `distance_allowed=false`
  - `safety_ready=false`
  - `candidate_left_cmd=0.000`
  - `candidate_right_cmd=0.000`

Interpretation:

- GPS placement issue is resolved when the rover is placed farther outdoors.
- Main-controller GPS readiness and AUTO-entry timeout behavior are working.
- This is a successful GPS/AUTO gate recovery and a safe blocked validation.
- It is not yet a successful nearby waypoint candidate dry-run because the
  target distance gate is still blocking.

Next step:

- Recompute a target from the current outdoor GPS position within roughly
  `5..15` m.
- Rerun the main-controller single-waypoint experiment with
  `AUTO_MOTION_ARMED=0`.
- At this point, bench testing remained blocked until `distance_allowed=true`,
  `safety_ready=true`, and nonzero candidate commands are observed while final
  outputs remain inhibited.
- Floor driving remains blocked.

## 2026-05-29: Successful No-Motion AUTO Waypoint Dry-Run

Setup:

- Firmware: `firmware/openrb_robot_controller`
- Mode:
  - `FIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT=1`
  - `AUTO_MOTION_ARMED=0`
- Rover/GPS placement: farther outdoors with stable enough sky view for
  main-controller testing.
- Compile-time target:
  - `target_lat_macro=35.5705010`
  - `target_lon_macro=129.1872696`

Observed MANUAL GPS readiness:

- GPS readiness was confirmed in MANUAL before AUTO validation:
  - `gps_location_valid=true`
  - `gps_location_fresh=true`
  - `gps_age_ok=true`
  - `gps_sats_ok=true`
  - `gps_hdop_ok=true` on good samples
  - `gps_solution_valid=true`
  - `gps_dryrun_ready=true`
  - `gps_motion_ready=true` on good samples
  - `gps_ready=true` on good samples
  - `last_rmc_status=A`
  - `last_gga_fix_quality=2`
  - `gps_sats≈7..9`
  - `gps_hdop≈1.28..1.56`

Observed target gate:

- In MANUAL, the compile-time target entered a usable nearby range:
  - `target_distance_m≈8.4..15.2`
  - `distance_allowed=true`

Observed AUTO dry-run:

- AUTO entry was verified:
  - `mode=AUTO_READY`
  - `auto_sw=true`
  - `mode_us≈2001`
  - `timeout_source=auto_entry`
  - `timeout_ok=true`
- The no-motion candidate command gate succeeded:
  - `safety_ready=true`
  - `candidate_left_cmd=0.100`
  - `candidate_right_cmd=0.100`
- Motor output inhibition remained correct:
  - `AUTO_MOTION_ARMED=0`
  - `auto_motor_inhibit=true`
  - `final_left_cmd=0.000`
  - `final_right_cmd=0.000`

Interpretation:

- Outdoor GPS placement is good enough for no-motion dry-run validation.
- MANUAL/AUTO switching works.
- The firmware computes candidate autonomous commands when the dry-run gates
  pass.
- The firmware correctly suppresses final motor commands when
  `AUTO_MOTION_ARMED=0`.
- Some AUTO lines still showed motion-level `gps_ready=false` /
  `gps_block_reason=BAD_HDOP` because motion-level GPS gating is stricter than
  dry-run gating. This is acceptable only for `AUTO_MOTION_ARMED=0` dry-run
  when `gps_dryrun_ready=true` / `active_gps_ready=true`; it is not sufficient
  for real motion.

Safety decision:

- No-motion AUTO waypoint dry-run is validated.
- Wheel-off-ground bench testing is the next step and still requires a strict
  safety procedure.
- Floor driving remains blocked.

## 2026-05-29: Armed AUTO Final Output Reached (0.100), No Visible Motion — Motor Deadband Suspected

Setup:

- Firmware: `firmware/openrb_robot_controller`
- Mode:
  - `FIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT=1`
  - `AUTO_MOTION_ARMED=1` (armed, no ground crawl harness in this build)
- Rover/GPS placement: outdoors with motion-grade sky view.

Observed armed AUTO state:

```text
mode=AUTO_RUNNING control_source=AUTO
auto_motion_armed=true auto_motor_inhibit=false
gps_ready=true gps_motion_ready=true gps_block_reason=OK
last_rmc_status=A last_gga_fix_quality=2 gps_sats=7..9 gps_hdop=1.0..1.4
distance_allowed=true safety_ready=true
candidate_left_cmd=0.100 candidate_right_cmd=0.100
final_left_cmd=0.100 final_right_cmd=0.100
```

Returning to MANUAL:

```text
mode=MANUAL control_source=RC_MANUAL
final_left_cmd=0.000 final_right_cmd=0.000
```

Interpretation:

- This is **NOT** a GPS problem and **NOT** an AUTO-gate problem. Firmware-side
  armed AUTO output was achieved for the first time: `final_left_cmd=0.100` /
  `final_right_cmd=0.100` while armed, motion-grade GPS, and all gates passing.
- **No visible rover movement was observed.** The likely cause is the
  **motor/ESC/friction deadband**: `0.100` maps to only ≈1530 µs (30 µs above the
  1500 µs neutral), below the threshold to overcome static friction.
- Returning to MANUAL correctly drove final commands to `0.000`.

Safety decision:

- This is **NOT** a successful physical ground crawl. Normal floor driving is
  **NOT** approved.
- We must **NOT** simply raise the AUTO command magnitude in this build — an
  ungated, time-unbounded armed command is a runaway risk.
- The next motion test must use the **guarded ground crawl** harness
  (`GROUND_CRAWL_TEST_MODE=1`): command clamped to ±`GROUND_CRAWL_MAX_CMD`
  (default `0.08`), hard latch stop after `GROUND_CRAWL_MAX_AUTO_MS` (default
  `1200` ms), latch clears only on MANUAL, plus neutral-RC + motion-GPS +
  near-field-target (5–20 m) gates. The cap (0.08) starts below the observed
  deadband by design — raise it only via `-DGROUND_CRAWL_MAX_CMD` in small steps,
  under latch protection, wheels-off-ground or open-area-with-killswitch only.
- Firmware change made the crawl harness the **only** path to armed motion: any
  armed build without `GROUND_CRAWL_TEST_MODE=1` now holds final commands at zero.

## 2026-05-30: Guarded Ground Crawl 0.08 Safety Validation

Setup:

- Firmware: `firmware/openrb_robot_controller`
- Mode:
  - `FIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT=1`
  - `AUTO_MOTION_ARMED=1`
  - `GROUND_CRAWL_TEST_MODE=1`
  - `GROUND_CRAWL_MAX_CMD=0.08`

Observed GPS and AUTO gates:

- GPS motion readiness was intermittently good:
  - `gps_motion_ready=true`
  - `gps_sats=5`
  - `gps_hdop≈1.34`
  - `gps_block_reason=OK`
- During a good GPS window, AUTO reached `AUTO_RUNNING`.
- Guarded crawl gates passed:
  - `ground_crawl_ready=true`
  - `ground_crawl_block_reason=OK`

Observed command clamp:

```text
candidate_left_cmd=0.100
candidate_right_cmd=0.100
final_left_cmd=0.080
final_right_cmd=0.080
```

This confirms the guarded crawl harness clamps the final command under
`GROUND_CRAWL_MAX_CMD=0.08`.

Observed stop and block behavior:

- After the duration limit, `ground_crawl_latched_stop=true` was observed and
  final commands were forced to zero.
- Later, the rover/GPS position drifted or moved close to the compile-time
  target:
  - `target_distance_m≈3.9..4.4`
- Because `GROUND_CRAWL_MIN_TARGET_DISTANCE_M=5.0`, the harness correctly
  blocked further motion with `ground_crawl_block_reason=DISTANCE_OUT_OF_RANGE`.
- GPS also intermittently dropped to `gps_sats=4` or stale/no-fix states, and
  the harness blocked motion as `GPS_NOT_MOTION_READY` or `LATCHED_STOP`.

Interpretation:

- The 0.08 guarded crawl safety behavior is validated:
  - crawl mode can reach `AUTO_RUNNING`;
  - final commands are clamped to 0.08;
  - the duration latch stops output;
  - too-close targets are blocked;
  - degraded GPS blocks output.
- This is still **not** full autonomous driving.
- The current compile-time target is now too close for another crawl test and
  must not be reused.

Next action:

- Reacquire current outdoor GPS.
- Compute a fresh target roughly `10..12` m away from the current rover/GPS
  position.
- If no visible physical movement was observed at `0.08`, the next guarded
  crawl test may use `SINGLE_WP_CRAWL_BASE_CMD=0.12` with
  `GROUND_CRAWL_MAX_CMD=0.12` and the same latch protection. The base command
  raises the candidate speed; the ground-crawl max remains the final clamp.
- Floor driving and coverage driving remain blocked.

## 2026-05-30: Guarded Ground Crawl 0.12 Cap-Only Result

Setup:

- Firmware: `firmware/openrb_robot_controller`
- Log marker: `ground_crawl_012_east9m_current`
- Mode:
  - `FIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT=1`
  - `AUTO_MOTION_ARMED=1`
  - `GROUND_CRAWL_TEST_MODE=1`
  - `GROUND_CRAWL_MAX_CMD=0.120`

Observed result:

- Guarded crawl system worked:
  - `AUTO_RUNNING` reached
  - `ground_crawl_ready=true`
  - `ground_crawl_block_reason=OK`
  - latch stop worked after the duration limit
- Candidate command stayed at the firmware default:
  - `candidate_left_cmd=0.100`
  - `candidate_right_cmd=0.100`
- Final command was therefore also `0.100`, because the `0.120` cap was above
  the candidate command:
  - `final_left_cmd=0.100`
  - `final_right_cmd=0.100`

Interpretation:

- `GROUND_CRAWL_MAX_CMD` is only the final safety clamp.
- Raising only `GROUND_CRAWL_MAX_CMD` cannot produce more than the candidate
  command.
- Firmware now has a separate compile-time candidate speed define:
  `SINGLE_WP_CRAWL_BASE_CMD`, default `0.100`.

Next action:

- If physical motion was still not observed, retry only after reacquiring fresh
  GPS and computing a fresh nearby target.
- Compile the next 0.12 attempt with both:
  - `SINGLE_WP_CRAWL_BASE_CMD=0.12`
  - `GROUND_CRAWL_MAX_CMD=0.12`
- Keep `GROUND_CRAWL_MAX_AUTO_MS` latch protection and all GPS/RC/target gates.

## 2026-05-30: GPS-Independent Motor Pulse Calibration Mode Added

Reason:

- Guarded crawl output has been observed at `0.120`, but the rover still did
  not physically move.
- GPS is intermittent, so GPS-gated tests are too slow and noisy for isolating
  motor deadband / drivetrain / output scaling.

Repository action:

- Added `MOTOR_PULSE_TEST_MODE=1` to
  `firmware/openrb_robot_controller/openrb_robot_controller.ino`.
- Added compile-time parameters:
  - `MOTOR_PULSE_CMD`, default `0.15`
  - `MOTOR_PULSE_LEFT_CMD`, default `MOTOR_PULSE_CMD`
  - `MOTOR_PULSE_RIGHT_CMD`, default `MOTOR_PULSE_CMD`
  - `MOTOR_PULSE_MS`, default `300`
- In this mode, HC-12 is disabled and GPS readiness / waypoint target distance
  are not used.
- RC MANUAL mode remains available.
- AUTO emits one neutral-stick pulse, then latches stop until returning to
  MANUAL.
- Added shared drive calibration layer behind `DRIVE_CALIBRATION_ENABLE=1`.
  Defaults are identity/off, and the layer is applied to MANUAL, station manual,
  single-waypoint AUTO, and motor pulse outputs through the common drive output
  path.

Validation status:

- Firmware compile is required before field use.
- Physical motor pulse calibration has not yet been run.

## 2026-05-30: Motor Pulse GPS-Looking Fields Are Expected To Be Empty

Observed flow:

- New GPS was verified with `firmware/gps_uart_probe` on `Serial2/9600`.
- Probe output showed healthy GPS UART/fix fields:
  - `chars_1s≈490..508`
  - `last_rmc_status=A`
  - `last_gga_fix_quality=2`
  - `sats≈6..8`
  - `hdop≈1.31..1.71`
- Then `openrb_robot_controller` was uploaded with:
  - `MOTOR_PULSE_TEST_MODE=1`
  - `MOTOR_PULSE_CMD=0.18`
  - `MOTOR_PULSE_MS=300`
- In the motor pulse build, USBDBG repeatedly showed:
  - `motor_pulse_test_mode=true`
  - `gps_chars=0`
  - `last_rmc_status=NA`
  - `last_gga_fix_quality=NA`
  - `gps_block_reason=NO_LOCATION`

Interpretation:

- This is expected. `MOTOR_PULSE_TEST_MODE` intentionally bypasses GPS
  initialization and GPS byte processing.
- The motor pulse build must not be used to validate GPS hardware or GPS UART.
- Keep test flow separated:
  1. GPS UART validation: `firmware/gps_uart_probe`.
  2. Main-controller GPS validation: single-waypoint experiment with
     `AUTO_MOTION_ARMED=0`, no motor pulse mode.
  3. Motor pulse deadband validation: `MOTOR_PULSE_TEST_MODE=1`, ignore GPS
     fields.

## 2026-05-30: Motor Pulse Deadband And Drivetrain Asymmetry

Setup:

- Firmware: `firmware/openrb_robot_controller`
- Mode: `MOTOR_PULSE_TEST_MODE=1`
- Pulse duration: `MOTOR_PULSE_MS=300`

Observed result:

- `MOTOR_PULSE_CMD=0.180` produced valid software output but no visible physical
  rover motion.
- `MOTOR_PULSE_CMD=0.220` produced visible physical rover motion.
- The 0.22 USBDBG log showed symmetric software output:
  - `left_cmd=0.220`
  - `right_cmd=0.220`
  - `motor_pulse_ready=true`
  - `motor_pulse_block_reason=OK`
- However, the observed physical motion looked more like rotation than straight
  forward motion.
- Manual RC driving also appears asymmetric:
  - forward driving tends to drift/curve left;
  - backward driving tends to drift/curve right.

Interpretation:

- The current blocker is drivetrain / motor output calibration, not GPS and not
  path planning.
- The pattern suggests the right-side drive may be stronger than the left side,
  or the left side may have higher friction/deadband.
- Possible causes include mechanical friction, wiring/electrical differences,
  motor/ESC mismatch, or software output scaling.

Next action:

- Do not proceed to GPS path planning yet.
- Run differential left/right motor pulse calibration so each side can be tested
  independently and then together.
- Use the shared drive calibration layer for measured trim or deadband
  compensation, not path planning code.

## 2026-05-30: Differential Motor Pulse Observations

Setup:

- Firmware: `firmware/openrb_robot_controller`
- Mode: `MOTOR_PULSE_TEST_MODE=1`
- Pulse duration: `MOTOR_PULSE_MS=300`
- Differential command support:
  - `MOTOR_PULSE_LEFT_CMD`
  - `MOTOR_PULSE_RIGHT_CMD`

Observed pulse threshold:

- `MOTOR_PULSE_CMD=0.180` produced valid software output but no visible physical
  motion.
- `MOTOR_PULSE_CMD=0.220` produced visible physical motion.
- In the 0.22 log, software output was symmetric:
  - `left_cmd=0.220`
  - `right_cmd=0.220`
  - `motor_pulse_ready=true`
  - `motor_pulse_block_reason=OK`

Differential pulse observations:

1. Left-only `+0.22`: left wheel rotates in the forward direction.
2. Right-only `+0.22`: right wheel rotates in the forward direction, and the
   rover curves left as expected for right-only drive.
3. Both `+0.22/+0.22`: both wheels appear to rotate forward, but the rover
   curves/rotates right instead of going straight.
4. Both `-0.22/-0.22`: both wheels appear to rotate backward, but the rover
   curves left while reversing.

Manual RC comparison:

- Forward manual driving tends to curve left.
- Reverse manual driving tends to curve right or less severely.

Code-path inspection:

- Motor pulse output bypasses RC stick angle remapping.
- In `MOTOR_PULSE_TEST_MODE`, RC steering/throttle are used only for the neutral
  precondition.
- Initial code-path inspection showed the AUTO pulse did not call
  `applyManualOverride(...)` or `mapRcManualAxes(...)`, but later physical
  results showed the output pins behaved like steer/throttle inputs rather than
  direct left/right wheel outputs.

Interpretation:

- This interpretation was later superseded by the physical output pin probe.
- Motor pulse output bypassed RC stick angle remapping, but the physical PWM
  pins were not direct left/right wheel outputs.

Next action:

- Do not proceed to GPS path planning yet.
- Resolve physical pin mapping before any side compensation.

## 2026-05-30: Direct Wheel Pulse Path Fix

Observed after the initial differential pulse notes:

1. `MOTOR_PULSE_LEFT_CMD=+0.25`, `MOTOR_PULSE_RIGHT_CMD=0.00`
   - physical left wheel rotated forward
   - physical right wheel rotated backward
2. `MOTOR_PULSE_LEFT_CMD=0.00`, `MOTOR_PULSE_RIGHT_CMD=+0.25`
   - physical left wheel rotated backward
   - physical right wheel rotated forward

Interpretation:

- This is not a simple left/right output swap and not a scale-only calibration
  result.
- Before drivetrain trim, the firmware must prove that motor pulse values are
  direct wheel commands and are not being interpreted as steering/throttle or
  remixed later.

Firmware update:

- `MOTOR_PULSE_LEFT_CMD` and `MOTOR_PULSE_RIGHT_CMD` are documented and routed
  as direct logical wheel commands.
- USBDBG now separates:
  - `logical_left_cmd` / `logical_right_cmd`
  - `calibrated_left_cmd` / `calibrated_right_cmd`
  - `output_left_cmd` / `output_right_cmd`
  - `output_left_pin_cmd` / `output_right_pin_cmd`
  - `motor_output_swap_lr`
- `mixer_bypassed_for_motor_pulse=true` means firmware RC/station/waypoint
  mixers were bypassed. The direct wheel command is then converted to the
  current motor controller's steer/throttle-style PWM inputs.
- `MOTOR_OUTPUT_SWAP_LR` exists for final output-stage swapping only and remains
  off by default.

Next action:

- Re-run left-only, right-only, both-forward, and both-reverse pulse tests.
- Only proceed to scale/deadband calibration after direct wheel commands behave
  as expected.

## 2026-05-30: Physical Output Pin Probe Added

Latest physical observations after direct logical wheel command tests:

1. `MOTOR_PULSE_LEFT_CMD=+0.25`, `MOTOR_PULSE_RIGHT_CMD=0.00`
   - `logical_left_cmd=0.250`
   - `logical_right_cmd=0.000`
   - `output_left_pin_cmd=0.125`
   - `output_right_pin_cmd=0.125`
   - rover stopped
2. `MOTOR_PULSE_LEFT_CMD=0.00`, `MOTOR_PULSE_RIGHT_CMD=+0.25`
   - `logical_left_cmd=0.000`
   - `logical_right_cmd=0.250`
   - `output_left_pin_cmd=-0.125`
   - `output_right_pin_cmd=0.125`
   - brief backward twitch or nearly stopped
3. `MOTOR_PULSE_LEFT_CMD=+0.22`, `MOTOR_PULSE_RIGHT_CMD=+0.22`
   - `output_left_pin_cmd=0.000`
   - `output_right_pin_cmd=0.220`
   - left wheel forward, right wheel backward, rover rotated left
4. `MOTOR_PULSE_LEFT_CMD=-0.22`, `MOTOR_PULSE_RIGHT_CMD=-0.22`
   - `output_left_pin_cmd=0.000`
   - `output_right_pin_cmd=-0.220`
   - left wheel backward, right wheel forward, rover rotated right

Interpretation:

- Logical wheel commands are now reaching the firmware correctly.
- The remaining bug is the logical-wheel-to-physical-pin conversion.
- `output_left_pin_cmd` and `output_right_pin_cmd` are physical controller pin
  inputs whose roles must be discovered. They are not confirmed left/right wheel
  outputs.
- Do not tune scale or minimum command compensation yet.

Firmware addition:

- Added `firmware/physical_output_pin_probe/physical_output_pin_probe.ino`.
- The probe writes directly to OpenRB D4 and D5, the same final Servo PWM pins
  used by `openrb_robot_controller`.
- It bypasses RC, GPS, HC-12, station commands, manual mix, waypoint logic,
  logical wheel conversion, drive calibration, and motor pulse logic.

Next action:

- Run the physical pin truth table:
  - A `+0.25`, B `0.00`
  - A `-0.25`, B `0.00`
  - A `0.00`, B `+0.25`
  - A `0.00`, B `-0.25`
- If physical pin A is throttle and physical pin B is steering/turn, update the
  integrated conversion to `throttle = (left + right) / 2` and
  `turn = (right - left) / 2`, assigned to the confirmed pins.

## 2026-05-30: Physical Pin Truth Table Applied

Standalone physical pin probe result:

1. A `+0.25`, B `0.00`: both wheels forward, rover forward.
2. A `-0.25`, B `0.00`: both wheels backward, rover backward.
3. A `0.00`, B `+0.25`: left wheel backward, right wheel forward, rover rotates
   left.
4. A `0.00`, B `-0.25`: left wheel forward, right wheel backward, rover rotates
   right.

Conclusion:

- Physical output channel A is throttle / forward-backward.
- Physical output channel B is turn / steering.
- B positive means right wheel forward and left wheel backward.
- Physical wheel model: `physical_left_wheel = A - B`,
  `physical_right_wheel = A + B`.

Firmware update:

- `MOTOR_PULSE_LEFT_CMD` / `MOTOR_PULSE_RIGHT_CMD` remain logical wheel commands.
- The integrated low-level output layer now converts:
  - `physical_a_cmd = (calibrated_left_cmd + calibrated_right_cmd) / 2`
  - `physical_b_cmd = (calibrated_right_cmd - calibrated_left_cmd) / 2`
- USBDBG prints `physical_a_cmd`, `physical_b_cmd`,
  `physical_a_role=throttle`, `physical_b_role=turn`, and
  `wheel_to_physical_mapping=diff_to_throttle_turn`.

Validation guidance:

- Start with both-wheel forward/reverse tests because single-wheel logical
  commands are halved at the physical pin level.
- Do not tune scale/min-cmd until the corrected mapping is physically verified.

## Known Manual Direction Attempts

These are recorded to prevent repeating the same fixes:

| Attempt | Observed Result | Status |
|---|---|---|
| old unknown board firmware | manual control existed, but source was not in repo | replaced by integrated repo firmware |
| first 45-degree remap | left/right behaved like forward/reverse | rejected |
| direct CH1/CH2 map | straight up/down did not align with forward/reverse | rejected |
| direct CH2 inversion | upper-left became forward and lower-right became reverse | rejected |
| old cardinal / angle remap | became harmful after physical A/B output mapping was fixed; upper-right acted like forward | rejected |
| current arcade mixer with `MANUAL_FORWARD_SIGN=-1`, `MANUAL_TURN_SIGN=1` | throttle -> forward, steering -> turn, then `left=forward+turn`, `right=forward-turn` | active for current rover/controller |

## 2026-05-30: MANUAL RC Arcade Mixer Fix

Problem:

- After the physical A/B output mapping was fixed, MOTOR PULSE and AUTO logical
  wheel commands behaved correctly.
- MANUAL RC still behaved diagonally:
  - upper-right -> forward
  - upper-left -> right turn
  - lower-left -> backward
  - lower-right -> left turn

Interpretation:

- The old manual diagonal / angle remap was still the wrong final drive path.
- The issue was manual mixing, not GPS, path planning, or the physical A/B
  mapping.

Firmware update:

- MANUAL now computes direct forward and turn commands from the RC axes:
  - `manual_forward_cmd = MANUAL_FORWARD_SIGN * throttle_norm`
  - `manual_turn_cmd = MANUAL_TURN_SIGN * steer_norm`
- It then uses an arcade-to-logical-wheel mixer:
  - `manual_logical_left_cmd = clamp(manual_forward_cmd + manual_turn_cmd)`
  - `manual_logical_right_cmd = clamp(manual_forward_cmd - manual_turn_cmd)`
- MANUAL then uses the same common output path as AUTO and motor pulse:
  logical wheel commands -> optional drive calibration -> physical A/B mapping
  `A=(L+R)/2`, `B=(R-L)/2` -> Servo PWM writes.
- The old angle remap is bypassed in the final MANUAL path. USBDBG prints
  `old_angle_remap_active=false`.

Validation required:

- Wheel-off-ground manual direction test:
  - stick up -> both wheels forward
  - stick down -> both wheels backward
  - stick right -> right turn
  - stick left -> left turn
  - upper-right -> forward-right arc
  - upper-left -> forward-left arc
- Later validation finalized the sign convention as
  `MANUAL_FORWARD_SIGN=-1`, `MANUAL_TURN_SIGN=1`; do not reintroduce the
  diagonal remap.

## 2026-05-30: MANUAL RC Sign Convention Finalized

Observed:

- After fixing the logical wheel to physical A/B mapping, motor pulse and AUTO
  logical wheel mapping passed.
- MANUAL RC left/right steering is correct.
- MANUAL forward/reverse was inverted with `MANUAL_FORWARD_SIGN=1`:
  - pulling the stick drove forward
  - pushing the stick drove backward
- Testing with `MANUAL_FORWARD_SIGN=-1` and `MANUAL_TURN_SIGN=1` produced the
  correct manual sign convention for this hardware/controller setup.

Final current baseline:

- `MANUAL_FORWARD_SIGN=-1`
- `MANUAL_TURN_SIGN=1`
- `MOTOR_OUTPUT_SWAP_LR=0`
- `DRIVE_CALIBRATION_ENABLE=0`

Interpretation:

- This is only an RC throttle-axis sign fix.
- It is not a wheel mapping issue and not a physical A/B output issue.
- Preserve the probe-confirmed physical output model:
  - A = throttle
  - B = turn
  - `A=(L+R)/2`
  - `B=(R-L)/2`

Manual validation checklist:

- stick up = forward
- stick down = backward
- stick right = right turn
- stick left = left turn

## 2026-05-30: First Successful Guarded AUTO Crawl

Observed:

- Manual RC drive now works with:
  - `MANUAL_FORWARD_SIGN=-1`
  - `MANUAL_TURN_SIGN=1`
  - `old_angle_remap_active=false`
- Physical output mapping is:
  - physical A = throttle
  - physical B = turn
  - `A=(logical_left+logical_right)/2`
  - `B=(logical_right-logical_left)/2`
- Guarded AUTO crawl succeeded physically: the rover moved briefly forward.

Successful `AUTO_RUNNING` log:

- `gps_motion_ready=true`
- `gps_block_reason=OK`
- `gps_sats≈9`
- `gps_hdop≈1.0..1.2`
- `target_distance_m≈9.6`
- `distance_allowed=true`
- `ground_crawl_test_mode=true`
- `ground_crawl_max_cmd=0.220`
- `single_wp_crawl_base_cmd=0.220`
- `ground_crawl_ready=true`
- `ground_crawl_block_reason=OK`
- `left_cmd=0.220`
- `right_cmd=0.220`
- `final_left_cmd=0.220`
- `final_right_cmd=0.220`
- `physical_a_cmd=0.220`
- `physical_b_cmd=0.000`

Safety latch:

- At about `ground_crawl_elapsed_ms=510`, the duration latch asserted:
  `ground_crawl_latched_stop=true`.
- Final outputs returned to zero.

Interpretation:

- This confirms short guarded forward motion.
- This is not full waypoint following.
- This is not station-side path execution or coverage/lawnmower driving.

Next stage:

1. Repeat or extend guarded crawl to `1000` ms.
2. Analyze GPS delta and target-distance change after the crawl.
3. Add single-waypoint steering dry-run.
4. Then proceed back to station-side path planning preview.

## 2026-05-30: Repeated 1000 ms Guarded AUTO Crawl

Setup:

- Manual RC drive and physical A/B mapping were already fixed.
- Build flags:
  - `GROUND_CRAWL_TEST_MODE=1`
  - `GROUND_CRAWL_MAX_CMD=0.220`
  - `GROUND_CRAWL_MAX_AUTO_MS=1000`
  - `SINGLE_WP_CRAWL_BASE_CMD=0.220`
  - `AUTO_MOTION_ARMED=1`
  - `MANUAL_FORWARD_SIGN=-1`
  - `MANUAL_TURN_SIGN=1`

Observed:

- The user toggled AUTO/MANUAL about `3..4` times.
- `AUTO_RUNNING` was observed multiple times.
- One AUTO attempt was shorter because the user returned to MANUAL early.

Valid `AUTO_RUNNING` fields:

- `gps_block_reason=OK`
- `gps_motion_ready=true`
- `distance_allowed=true`
- `ground_crawl_ready=true`
- `ground_crawl_block_reason=OK`
- `left_cmd=0.220`
- `right_cmd=0.220`
- `final_left_cmd=0.220`
- `final_right_cmd=0.220`
- `physical_a_cmd=0.220`
- `physical_b_cmd=0.000`

GPS quality:

- `gps_sats≈8..10`
- `gps_hdop≈1.0..1.65`
- `last_gga_fix_quality=2`

Safety latch:

- After roughly `1000` ms, `ground_crawl_latched_stop=true`.
- Final outputs returned to zero.

Target-distance behavior:

- `target_distance_m` did not monotonically decrease.
- It varied around `16.8..18.0` m.
- This is expected because the current guarded crawl only drives straight with
  `physical_b_cmd=0.000`.
- There is no steering/course correction in this mode yet.

Interpretation:

- This proves repeated short guarded autonomous forward actuation.
- It does not prove waypoint following.
- It does not prove station-side path execution or coverage driving.

Next stage:

1. Station-side path planning preview only, no motor execution.
2. Single-waypoint steering dry-run.
3. Heading/course estimation before physical waypoint following.

## 2026-05-30: Single-Waypoint Steering Dry-Run Diagnostics Added

Reason:

- Repeated 1000 ms guarded crawl proved straight guarded actuation.
- The crawl used `physical_a_cmd=0.220` and `physical_b_cmd=0.000`.
- `target_distance_m` did not monotonically decrease because the rover was not
  necessarily facing the target.
- GPS position can provide bearing to target, but not rover heading by itself.

Firmware addition:

- Added compile-time flag `SINGLE_WP_STEERING_DRYRUN`, default `0`.
- When enabled, the single-waypoint experiment prints:
  - `current_gps_lat`, `current_gps_lon`
  - `steering_target_lat`, `steering_target_lon`
  - `target_distance_m`, `target_bearing_deg`
  - `heading_ready`, `heading_source`
  - `estimated_course_deg`, `bearing_error_deg`
  - `desired_forward_cmd`, `desired_turn_cmd`
  - `desired_logical_left_cmd`, `desired_logical_right_cmd`
  - `desired_physical_a_cmd`, `desired_physical_b_cmd`
  - `steering_block_reason`

Heading/course policy:

- Course-over-ground is estimated only after at least `2.0` m of GPS
  displacement.
- If movement is too small, `heading_ready=false` and
  `steering_block_reason=NO_HEADING`.
- `target_bearing_deg` alone must not be used as a motor steering command.

Safety:

- This diagnostic does not drive motors by itself.
- It does not change `GROUND_CRAWL_TEST_MODE` or `AUTO_MOTION_ARMED` behavior.
- Physical waypoint following remains blocked until heading/course and steering
  behavior are validated.

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
