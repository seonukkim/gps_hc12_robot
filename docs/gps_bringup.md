# GPS Bring-Up

This page is the working procedure for recovering and validating the rover GPS
UART link. It is intentionally separate from the integrated rover controller.

## Safety Boundary

- Do not modify `firmware/openrb_robot_controller/openrb_robot_controller.ino`
  for this probe.
- Use `firmware/gps_uart_probe/gps_uart_probe.ino`; it does not include Servo,
  attach ESC pins, or drive motor outputs.
- Keep motor power disconnected when possible. If motor power must stay
  connected, keep the rover wheel-off-ground.
- Test one UART/baud candidate at a time and record the exact output before
  moving to the next candidate.

## Current Results

Date: 2026-05-26

Final status:

- GPS UART bring-up succeeded.
- Current physical GPS wiring is the central OpenRB connector.
- The confirmed GPS UART is `Serial2` at `9600` baud.
- GPS module baudrate is confirmed as `9600`.
- Valid NMEA is received.
- GPS fix eventually became valid.
- Final UART decision: choose Option A based on actual hardware wiring.
- Keep GPS on the current central OpenRB connector as `GPS_SERIAL=Serial2`.
- Move HC-12 data lines to verified physical `Serial3` RX/TX pins later, then
  update integrated firmware to `HC12_SERIAL=Serial3`.
- The purple module appears to be an IMU on an I2C-style connection; do not
  treat it as a UART device.

### Serial3 D13/D14 Failure

Uploaded GPS-only test result:

```text
chars_1s=0 total_chars=0 tinygps_chars=0 status=NO_FIX sats=NA hdop=NA age_ms=NA
```

Configuration assumed by that test:

- GPS port: `Serial3`
- OpenRB-150 RX pin assumption: `D13`
- GPS baud: `9600`
- USB monitor baud: `115200`

Interpretation:

- The sketch is running.
- USB serial monitor is working.
- `chars_1s=0`, `total_chars=0`, and `tinygps_chars=0` mean no bytes reached
  the configured GPS serial input.
- This is not merely a satellite-fix problem. It points to wiring, selected
  UART, baudrate, module power, or module output configuration.

Follow-up probe result:

- `Serial3` D13/D14 tests failed because the current GPS wiring is not on
  D13/D14.
- Do not interpret the D13/D14 failure as a failed GPS module while the module
  is plugged into the central OpenRB connector.
- Serial3 physical RX/TX pin mapping remains unresolved for the current rover
  hardware and must be located before moving HC-12 data lines.

### Serial2 Success On Current Wiring

Final successful probe result:

```text
selected_port=Serial2 baud=9600
chars_1s roughly 350-520
raw_preview contains $GPRMC, $GPVTG, $GPGGA, $GPGSV, $GPGLL
tinygps_chars increases
fix=true
lat around 35.57107
lon around 129.1860
sats around 5
hdop around 1.61-1.62
```

Interpretation:

- GPS UART communication is working on `Serial2` at `9600` with the current
  central OpenRB connector wiring.
- GPS satellite fix was acquired.
- The Serial3 D13/D14 zero-byte result is invalid for the current wiring; it
  does not indicate GPS module failure.
- GPS should stay on the current central connector.
- The next problem is HC-12 UART relocation and integrated firmware UART
  allocation.

## Architecture Decision: Option A

The integrated rover firmware currently defines:

```cpp
#define HC12_SERIAL Serial2
#define GPS_SERIAL Serial3
```

The current hardware probe confirms GPS data on `Serial2`, not on `Serial3`
D13/D14. That creates a UART allocation conflict if the HC-12 also needs
`Serial2`.

Decision:

- Choose Option A based on the actual hardware wiring.
- Keep GPS on the current central OpenRB connector.
- Final target: `GPS_SERIAL=Serial2`.
- Move HC-12 data lines to a verified physical `Serial3` RX/TX path.
- Final target: `HC12_SERIAL=Serial3`.
- The previous Option B decision is superseded.

Options considered:

| Option | Hardware change | Firmware impact | Tradeoff |
|---|---|---|---|
| A | Keep GPS on the current central connector / `Serial2`; move HC-12 to another verified UART | later change `HC12_SERIAL` after HC-12 wiring is verified | preserves the newly confirmed GPS UART path, but risks disrupting known manual HC-12 control |
| B | Keep HC-12 on `Serial2`; move GPS wiring to verified `Serial3` RX/TX pins | keep current integrated firmware UART roles | preserves the known-working HC-12 manual control path, but requires physically moving and revalidating GPS wiring |

Rationale:

- The GPS module, GPS baudrate, central connector, NMEA receive, and GPS fix are
  all proven working on `Serial2`.
- The purple module appears to be an IMU on an I2C-style connection and should
  not be treated as a UART relocation target.
- HC-12 appears to be mounted under or behind the OpenRB board and its UART
  wiring needs a separate verification path.
- The remaining uncertainty is the physical `Serial3` RX/TX mapping for HC-12.
- Do not change `firmware/openrb_robot_controller/openrb_robot_controller.ino`
  until HC-12 is physically moved to verified `Serial3` pins and echo-tested.

Next hardware action:

- Locate the actual `Serial3` RX/TX pins using loopback and pin-finder tests.
- Run Serial3 TX-to-RX loopback.
- Move HC-12 data lines to verified `Serial3` RX/TX.
- Run an HC-12 Serial3 echo test.
- Only after HC-12 echo works on `Serial3`, update integrated firmware mapping
  to `GPS_SERIAL=Serial2` and `HC12_SERIAL=Serial3`.

## Serial3 Physical Pin Verification

Purpose:

- Keep GPS on the current central OpenRB connector / `Serial2`.
- Find the actual physical `Serial3` RX/TX pins.
- Move HC-12 data lines to verified `Serial3` pins only after a loopback PASS.
- Do not modify `firmware/openrb_robot_controller/openrb_robot_controller.ino`.
- Do not attach motors or Servo.

Safe sketches:

| Sketch | Purpose | Motor behavior |
|---|---|---|
| `firmware/pin_finder_test/pin_finder_test.ino` | toggles candidate pins so the physical pads can be found with a meter, logic probe, or LED plus resistor | no Servo, no ESC attach, no motor output |
| `firmware/serial3_loopback_test/serial3_loopback_test.ino` | writes known text to `Serial3` and reads it back through a TX-to-RX jumper | no Servo, no ESC attach, no motor output |

Pin-finder candidates:

- Default: `D13`, `D14`.
- Extended, only when explicitly compiled with
  `PIN_FINDER_INCLUDE_EXTENDED=1`: `D0`, `D1`, `D26`, `D27`, `D28`, `D29`.
- Extended candidates are skipped only if the board core exposes `PINS_COUNT`
  and the candidate is outside that range. A compiled candidate still may not
  exist as an accessible physical pad; confirm electrically.

Required wiring for pin finder:

1. Disconnect GPS.
2. Disconnect HC-12 if necessary, especially if candidate pins may overlap or
   if the current under-board/behind-board UART wiring is uncertain.
3. Keep motor power disconnected when possible.
4. Connect only USB and the measurement tool.
5. Measure each active candidate pin against OpenRB `GND`.
6. Use a logic probe, oscilloscope, multimeter, or LED with a suitable resistor.

Required wiring for Serial3 loopback:

1. Disconnect GPS.
2. Disconnect HC-12 if necessary, especially if candidate pins may overlap or
   if the current under-board/behind-board UART wiring is uncertain.
3. Keep motor power disconnected when possible.
4. Connect candidate `Serial3 TX` directly to candidate `Serial3 RX`.
5. Keep the board powered by USB for the test.
6. Open the USB monitor at `115200`.

Loopback decision rule:

- `result=PASS` is valid only when `chars_1s > 0` and `bytes_seen=true`.
- A PASS should also show the known `$S3LOOP,...` text in `raw_preview`.
- `result=FAIL` with `chars_1s=0` means the selected physical TX/RX pair is not
  looped back to the actual `Serial3` peripheral.
- Do not move HC-12 data lines to Serial3 until loopback passes.
- Do not move GPS; GPS stays on the current central connector.

### Pin Finder Commands

Default D13/D14 scan:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-pin-finder-d13-d14 firmware/pin_finder_test
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-pin-finder-d13-d14 firmware/pin_finder_test
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

Extended candidate scan:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-pin-finder-extended --build-property 'compiler.cpp.extra_flags=-DPIN_FINDER_INCLUDE_EXTENDED=1' firmware/pin_finder_test
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-pin-finder-extended firmware/pin_finder_test
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

### Serial3 Loopback Commands

Use these after selecting a candidate physical TX/RX pair. The sketch writes
known `$S3LOOP,...` text to `Serial3`; it prints PASS only after bytes are
received back on `Serial3`.

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-serial3-loopback-9600 --build-property 'compiler.cpp.extra_flags=-DSERIAL3_LOOPBACK_BAUD=9600' firmware/serial3_loopback_test
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-serial3-loopback-9600 firmware/serial3_loopback_test
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

Expected PASS line shape:

```text
selected_port=Serial3 baud=9600 writes_1s=4 total_writes=... chars_1s=... total_chars=... raw_preview="$S3LOOP,SEQ=..." bytes_seen=true result=PASS
```

Expected FAIL line shape:

```text
selected_port=Serial3 baud=9600 writes_1s=4 total_writes=... chars_1s=0 total_chars=0 raw_preview="" bytes_seen=false result=FAIL
```

### HC-12 Serial3 Echo Milestone

Run this only after Serial3 loopback has passed:

1. Disconnect the Serial3 TX-to-RX loopback jumper.
2. Move HC-12 `TX` to verified `Serial3 RX`.
3. Move HC-12 `RX` to verified `Serial3 TX`.
4. Keep GPS connected to the current central connector / `Serial2`.
5. Confirm common `GND`.
6. Run an HC-12 echo test that explicitly uses `Serial3` at `9600`.
7. Only after HC-12 echo succeeds on `Serial3`, update
   `firmware/openrb_robot_controller/openrb_robot_controller.ino` mapping.

Current note:

- `firmware/hc12_echo_test/hc12_echo_test.ino` must be reviewed before this
  milestone because it currently uses a SoftwareSerial-style test path, not the
  final `Serial3` hardware UART path.

## Probe Sketch

Standalone sketch:

```text
firmware/gps_uart_probe/gps_uart_probe.ino
```

Compile-time options:

| Macro | Meaning |
|---|---|
| `GPS_PROBE_MODE=3` | `Serial3`, expected OpenRB RX `D13` |
| `GPS_PROBE_MODE=2` | `Serial2`, board UART candidate |
| `GPS_PROBE_MODE=89` | `SoftwareSerial` RX `D8` / TX `D9` candidate |
| `GPS_PROBE_BAUD=9600` | GPS candidate baudrate |

Every report line prints:

- selected port
- selected baud
- pin assumption
- `chars_1s`
- `total_chars`
- escaped `raw_preview`
- TinyGPS++ `charsProcessed`
- parsed `fix`, latitude/longitude age, satellites, and HDOP when available

## Probe Matrix

The software build check was run against OpenRB-150 core
`OpenRB-150:samd:OpenRB-150`.

| Order | Candidate | Compile Status | Bench Result | Action |
|---:|---|---|---|---|
| 1 | `Serial3` RX `D13` at `9600` | pass | zero bytes because GPS was not wired to D13/D14 | invalid for current wiring; use loopback/pin-finder tests to locate real Serial3 RX/TX |
| 2 | `Serial3` RX `D13` at `38400` | pass | zero bytes because GPS was not wired to D13/D14 | invalid for current wiring; GPS baud is already confirmed as 9600 |
| 3 | `Serial3` RX `D13` at `115200` | pass | zero bytes because GPS was not wired to D13/D14 | invalid for current wiring; GPS baud is already confirmed as 9600 |
| 4 | `Serial2` at `9600` | pass | success: `chars_1s` roughly 350-520, readable NMEA `$GPRMC`, `$GPVTG`, `$GPGGA`, `$GPGSV`, `$GPGLL`, TinyGPS++ chars increasing, `fix=true`, lat around `35.57107`, lon around `129.1860`, sats around `5`, HDOP around `1.61-1.62` | GPS UART bring-up complete; keep GPS on central connector / `Serial2`; verify HC-12 on `Serial3` next |
| 5 | `SoftwareSerial` RX `D8` / TX `D9` at `9600` | not supported in this OpenRB build | not uploaded | `SoftwareSerial.h` is unavailable; do not use this candidate unless the core/library support changes and it recompiles |

## Wiring Checklist

Check these before changing firmware assumptions:

- GPS `VCC` / `UCC` connected to the intended OpenRB supply rail.
- GPS `GND` connected to OpenRB `GND`; USB, OpenRB, GPS, and any external power
  domains must share ground for UART testing.
- GPS `TX` goes to the selected OpenRB `RX`; UART is crossed TX-to-RX.
- For `Serial3`, the earlier repo assumption was GPS `TX` -> OpenRB `D13` /
  RX, but the physical `Serial3` pin mapping is currently unresolved.
- For the current central OpenRB connector wiring, GPS data was confirmed on
  `Serial2` at `9600`.
- With Option A selected, GPS should remain on the current central connector /
  `Serial2`.
- HC-12 should move to verified physical `Serial3` pins after loopback and
  echo testing.
- The purple module appears to be an IMU on an I2C-style connection; do not
  treat it as a UART device or HC-12 substitute.
- GPS `RX` is not required for read-only NMEA receive tests. Leave it
  disconnected unless intentionally configuring the GPS module.
- GPS `PPS` is not required for this test.
- Confirm the GPS module is powered and its antenna has sky view. Sky view is
  needed for a fix, but not needed for `chars_1s > 0`.
- Confirm the GPS module still emits NMEA text and was not reconfigured to a
  binary-only protocol.
- Do not connect GPS and HC-12 to the same UART RX/TX pair at the same time
  during this probe.
- The integrated controller currently expects HC-12 on `Serial2`; do not assume
  GPS and HC-12 can share the current `Serial2` path in normal operation.
- With Option A selected, keep GPS on `Serial2` and move HC-12 only after the
  actual `Serial3` RX/TX pins are verified.
- Confirm logic-level compatibility before moving HC-12 UART wires to another
  OpenRB pin.

## Decision Rules

- `chars_1s=0`: wiring, selected port, selected baudrate, module power, or GPS
  output configuration problem. Do not wait indoors for a GPS fix; no data is
  arriving.
- `chars_1s>0` and `fix=false`: GPS bytes are arriving. Keep the same UART and
  baudrate, move outdoors or improve antenna placement, and wait for satellite
  acquisition.
- `chars_1s>0` with a readable NMEA `raw_preview`: parser issues are unlikely;
  focus on fix quality and sky view.
- `tinygps_chars=0` while `chars_1s>0`: raw bytes are arriving but may not be
  NMEA text, may be wrong baud gibberish, or may be a binary-only GPS output.

Current decision from the successful probe:

- `Serial2` at `9600` with readable `$GP...` NMEA means the GPS UART is working.
- `fix=true`, numeric latitude, numeric longitude, and stable HDOP confirm the
  GPS can produce a usable position on the current wiring.
- Do not keep cycling UART ports once `chars_1s>0` and readable NMEA are
  confirmed.
- Do not implement autonomous movement as the next step; resolve UART
  allocation and add diagnostics first.
- Because Option A is selected, the next proof point is not another GPS baud
  scan and not moving GPS. It is physical `Serial3` RX/TX verification for
  HC-12.

## Test Procedure

For each supported candidate:

1. Wire only the candidate UART path.
2. Compile the candidate into its own build directory.
3. Upload that exact build directory.
4. Open USB serial monitor at `115200`.
5. Record at least 10 report lines.
6. If `chars_1s=0`, stop that candidate and move to the next baud/port.
7. If `chars_1s>0`, keep that UART/baud and debug fix quality separately.

## Commands

The commands below use the Arduino CLI bundled with Arduino IDE on this
machine, FQBN `OpenRB-150:samd:OpenRB-150`, and OpenRB USB port
`/dev/cu.usbmodem12101`. Replace the port only if the board enumerates
differently.

### Serial3 RX D13 at 9600

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/gps-probe-s3-9600 --build-property 'compiler.cpp.extra_flags=-DGPS_PROBE_MODE=3 -DGPS_PROBE_BAUD=9600' firmware/gps_uart_probe
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/gps-probe-s3-9600 firmware/gps_uart_probe
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

### Serial3 RX D13 at 38400

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/gps-probe-s3-38400 --build-property 'compiler.cpp.extra_flags=-DGPS_PROBE_MODE=3 -DGPS_PROBE_BAUD=38400' firmware/gps_uart_probe
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/gps-probe-s3-38400 firmware/gps_uart_probe
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

### Serial3 RX D13 at 115200

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/gps-probe-s3-115200 --build-property 'compiler.cpp.extra_flags=-DGPS_PROBE_MODE=3 -DGPS_PROBE_BAUD=115200' firmware/gps_uart_probe
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/gps-probe-s3-115200 firmware/gps_uart_probe
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

### Serial2 at 9600

This is the confirmed working probe for the current central OpenRB connector
wiring. GPS should stay on this connector. Do not run integrated GPS and HC-12
on `Serial2` at the same time; move HC-12 to verified `Serial3` before changing
the integrated firmware mapping.

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/gps-probe-s2-9600 --build-property 'compiler.cpp.extra_flags=-DGPS_PROBE_MODE=2 -DGPS_PROBE_BAUD=9600' firmware/gps_uart_probe
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/gps-probe-s2-9600 firmware/gps_uart_probe
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

### Outdoor GPS Fix Test

This test has succeeded on `Serial2` at `9600`. Use this command to repeat it
after the `Serial2` probe sketch is already uploaded:

```bash
arduino-cli monitor -p /dev/cu.usbmodem12101 --config baudrate=115200
```

Logging variant:

```bash
mkdir -p outputs/logs
arduino-cli monitor -p /dev/cu.usbmodem12101 --config baudrate=115200 | tee outputs/logs/gps_outdoor_fix_serial2_$(date +%Y%m%d_%H%M%S).log
```

Outdoor pass criteria:

- `chars_1s > 0`
- `raw_preview` continues to show readable `$GP...` NMEA
- `fix=true`
- `lat` and `lon` are numeric
- Prefer `sats >= 4`
- Prefer stable HDOP below roughly `5.0`; lower is better

Observed passing values:

- `chars_1s`: roughly `350-520`
- NMEA: `$GPRMC`, `$GPVTG`, `$GPGGA`, `$GPGSV`, `$GPGLL`
- `fix`: `true`
- `lat`: around `35.57107`
- `lon`: around `129.1860`
- `sats`: around `5`
- `hdop`: around `1.61-1.62`

## Next Software Milestone

The next software milestone should be a GPS diagnostic integrated firmware mode,
not autonomous movement.

Goal:

- Verify the selected final UART allocation inside
  `firmware/openrb_robot_controller/openrb_robot_controller.ino`.
- Keep manual control, STOP, heartbeat timeout, RC priority, and failsafe
  behavior unchanged.
- Print or transmit explicit GPS diagnostics from integrated firmware:
  selected GPS UART, `chars_1s`, TinyGPS++ chars processed, fix state, sats,
  HDOP, lat/lon, and GPS age.
- Final target mapping after hardware verification:
  - `GPS_SERIAL=Serial2`
  - `HC12_SERIAL=Serial3`
- Do not send or enable live `AUTO` driving commands.

Recommended order:

1. Locate actual `Serial3` RX/TX pins using loopback and pin-finder tests.
2. Run Serial3 TX-to-RX loopback.
3. Move HC-12 data lines to verified `Serial3` RX/TX.
4. Run an HC-12 Serial3 echo test.
5. Only then update integrated firmware mapping to `GPS_SERIAL=Serial2` and
   `HC12_SERIAL=Serial3`.
6. Add a non-motion GPS diagnostic path to integrated firmware.
7. Run USB debug and HC-12 manual-control regression checks.
8. Only after telemetry and safety remain stable, continue station-side GPS
   display or logging work.

### SoftwareSerial D8/D9 at 9600

This candidate is documented but currently not supported by this OpenRB build.
Compilation fails because `SoftwareSerial.h` is unavailable. Do not upload it
until support is added and the compile succeeds.

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/gps-probe-soft-9600 --build-property 'compiler.cpp.extra_flags=-DGPS_PROBE_MODE=89 -DGPS_PROBE_BAUD=9600' firmware/gps_uart_probe
```
