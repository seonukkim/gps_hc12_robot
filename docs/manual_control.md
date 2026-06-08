# Manual Control Bring-Up

This document describes the current rover manual-control setup, which files are
used, how to flash the rover firmware, how to run the station-side manual tool,
and how the current code maps joystick input to motor commands.

`manual-control` is PPM manual drive with full telemetry display. It is not a
sensor-disabled mode: GPS and IMU status continue to print when available, but
they are diagnostic-only and do not gate manual motor output.

All motor and ESC checks are wheel-off-ground only.

## Active Files

Rover firmware:

```text
firmware/openrb_robot_controller/openrb_robot_controller.ino
```

Station keyboard manual tool:

```text
tools/station_keyboard_manual.py
```

Station serial/protocol helper:

```text
tools/station_controller.py
gps_coverage_core/protocol.py
```

Related documentation:

```text
docs/rc_channel_map.md
docs/protocol.md
firmware/README.md
```

## Current Rover Firmware Marker

After flashing the expected integrated firmware, the OpenRB USB serial output
should include:

```text
Firmware: openrb_robot_controller station-hw-manual rc-arcade-manual-fwdneg 2026-05-30
```

If USB serial prints older lines such as `STAT,...,MANUAL_CENTER_STOP,...`, the
OpenRB is not running the current repository firmware and must be flashed again.

## RC Manual Mapping

The receiver PPM input is read on OpenRB `D6`.

Current channel mapping:

| PPM Channel | Firmware Constant | Meaning |
|---:|---|---|
| CH1 | `STEERING_CHANNEL_INDEX = 0` | steering |
| CH2 | `THROTTLE_CHANNEL_INDEX = 1` | throttle |
| CH5 | `MODE_CHANNEL_INDEX = 4` | Manual/Auto mode |
| CH7 | unused | reserved |

## RC Channel Probe

Use the safe channel probe when the physical RC mode switch is unclear or when
`mode_us` stays near `1500` instead of reaching the AUTO range:

```text
firmware/rc_channel_probe/rc_channel_probe.ino
```

The probe uses the same PPM input pin and frame decoder style as
`openrb_robot_controller`:

- PPM input: OpenRB `D6`
- channels: `ch1_us` through `ch8_us`
- frame sync: pulse width greater than `4000 us`
- interrupt edge: `FALLING` (matches the verified `firmware/rc_mix_test`
  decoder; the wrong edge can show short invalid pulses such as about `200 us`)

It does not attach Servo or motor outputs. It prints current values, min/max
observed values, and `changed_channels` every `0.5` seconds.

Compile/upload/monitor on this macOS setup:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-rc-channel-probe firmware/rc_channel_probe
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-rc-channel-probe firmware/rc_channel_probe
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

Probe procedure:

1. Keep motors disconnected or wheels off ground.
2. Confirm the station/controller is powered on and linked. A controller-off
   state can make RC channels appear static or failsafe-like even when the
   rover PPM reader is working.
3. Move each stick and each switch one at a time.
4. Record which `chN_us` changes and the min/max range for that channel.
5. Treat the channel that reaches around `2000 us` as the AUTO switch
   candidate.
6. Do not change the controller mode-channel mapping until the raw PPM probe
   confirms the intended switch.

Latest outdoor recovery note:

- The previous stuck-looking RC mode issue was caused by the
  station/controller being off.
- After restoring the controller/link, AUTO was verified with
  `mode=AUTO_READY`, `auto_sw=true`, `mode_us≈2001..2002`, and
  `control_source=STOP`.
- MANUAL was verified with `mode=MANUAL`, `auto_sw=false`,
  `mode_us≈1000..1001`, and `control_source=RC_MANUAL`.
- Manual stick input changed `steer_us` / `throttle_us`, and at least one
  MANUAL line produced nonzero `left_cmd` / `final_left_cmd`.
- A later outdoor nearby dry-run briefly showed a failsafe-like PPM glitch
  (`rc_ok=false`, invalid-looking `steer_us` / `throttle_us`); the firmware kept
  `control_source=STOP`, which is the correct safe behavior.
- Current `manual-control` status should still show the full USBDBG context:
  `rc_input_detected`, `rc_ok`, `mode`, `auto_sw`, `steer_us`,
  `throttle_us`, `mode_us`, physical A/B commands, `control_source`,
  motor-output fields, `gps_block_reason`, `current_lat`, `current_lon`,
  `gps_sats`, `gps_hdop`, `imu_present`, `imu_relative_yaw_deg`, and
  `imu_heading_block_reason`.

Current calibration and direction constants:

```cpp
constexpr uint16_t STEERING_CENTER_US = 1504;
constexpr uint16_t THROTTLE_CENTER_US = 1500;
constexpr uint16_t RC_DEADBAND_US = 80;
constexpr uint16_t RC_AUTO_SWITCH_ON_US = 1600;
constexpr float MANUAL_FORWARD_SIGN_VALUE = MANUAL_FORWARD_SIGN;
constexpr float MANUAL_TURN_SIGN_VALUE = MANUAL_TURN_SIGN;
```

Bench observations showed that the old manual path was wrong:

- A first 45-degree remap had the wrong sign and made left/right behave like
  forward/reverse.
- Direct CH1/CH2 mapping did not make straight up/down the forward/reverse axis.
- Direct CH2 inversion still left forward/reverse on a diagonal: upper-left
  acted like forward and lower-right acted like reverse.
- After the physical A/B output mapping was fixed, the old 45-degree remap
  became harmful: upper-right behaved like forward, upper-left like right turn,
  lower-left like backward, and lower-right like left turn.

The current correction bypasses the old angle remap in the final MANUAL path.
RC MANUAL now uses an arcade-to-logical-wheel mixer:

```cpp
manual_forward_cmd = MANUAL_FORWARD_SIGN * throttle_norm;
manual_turn_cmd = MANUAL_TURN_SIGN * steer_norm;
logical_left_cmd = clamp(manual_forward_cmd + manual_turn_cmd);
logical_right_cmd = clamp(manual_forward_cmd - manual_turn_cmd);
```

Then MANUAL uses the same common output path as AUTO and motor pulse:

```text
logical_left_cmd / logical_right_cmd
-> optional drive calibration
-> physical A/B conversion: A=(L+R)/2, B=(R-L)/2
-> Servo PWM writes
```

The working sign convention for the current rover/controller is:

- `MANUAL_FORWARD_SIGN=-1`
- `MANUAL_TURN_SIGN=1`
- `MOTOR_OUTPUT_SWAP_LR=0`
- `DRIVE_CALIBRATION_ENABLE=0` for the current uncalibrated baseline

This is an RC axis sign fix only. It does not change the wheel mapping or the
probe-confirmed physical A/B output model.

Target behavior:

- Push straight up -> `manual_forward_cmd > 0`, `manual_turn_cmd ~= 0` -> forward.
- Pull straight down -> `manual_forward_cmd < 0`, `manual_turn_cmd ~= 0` -> reverse.
- Push straight right -> `manual_turn_cmd > 0`, `manual_forward_cmd ~= 0`.
- Push straight left -> `manual_turn_cmd < 0`, `manual_forward_cmd ~= 0`.

The firmware path is:

1. `readRcChannels()` copies CH1, CH2, CH5 from the PPM ISR buffer.
2. `normRcCentered()` converts pulse width to normalized `-1.0..1.0`.
3. `computeManualArcadeCommands()` applies `MANUAL_FORWARD_SIGN` /
   `MANUAL_TURN_SIGN` and mixes forward/turn to logical wheel commands.
4. `applyDriveCommand()` applies optional drive calibration once.
5. The common output layer converts logical wheels to physical A/B pins:
   `A=(L+R)/2`, `B=(R-L)/2`.
6. Servo PWM writes go to the physical controller input pins.

The current RC MANUAL arcade mix is:

```cpp
left = forward + turn;
right = forward - turn;
```

ESC output uses:

```cpp
ESC_NEUTRAL_US = 1500;
ESC_RANGE_US = 300;
```

So normalized `+1.0` maps to `1800 us`, `-1.0` maps to `1200 us`, and neutral
maps to `1500 us`.

## Rover Firmware Compile And Upload

Arduino CLI must know the OpenRB board:

```bash
arduino-cli core list
arduino-cli board list
```

Expected board FQBN:

```text
OpenRB-150:samd:OpenRB-150
```

Compile:

```bash
arduino-cli compile --fqbn OpenRB-150:samd:OpenRB-150 firmware/openrb_robot_controller
```

Upload on this macOS setup:

```bash
arduino-cli upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 firmware/openrb_robot_controller
```

On Linux/WSL, the OpenRB port may be:

```text
/dev/ttyACM0
```

Use the actual port shown by:

```bash
arduino-cli board list
```

## Verify Rover Firmware

Read USB debug at `115200` baud. On this macOS setup:

```bash
uv run python - <<'PY'
import time
import serial

port = "/dev/tty.usbmodem12101"
with serial.Serial(port, 115200, timeout=0.5) as ser:
    deadline = time.time() + 5
    while time.time() < deadline:
        line = ser.readline()
        if line:
            print(line.decode("utf-8", errors="replace").rstrip())
PY
```

Expected neutral debug shape:

```text
USBDBG mode=MANUAL rc_ok=true auto_sw=false ... steer_norm=0.000 throttle_norm=0.000 manual_forward_cmd=0.000 manual_turn_cmd=0.000 manual_logical_left_cmd=0.000 manual_logical_right_cmd=0.000 old_angle_remap_active=false ... left_cmd=0.000 right_cmd=0.000
```

For direction validation, keep wheels off ground and check:

- Straight up: `manual_turn_cmd` stays near `0.000`,
  `manual_forward_cmd` becomes positive, and both motor commands should move in
  the forward direction.
- Straight down: `manual_turn_cmd` stays near `0.000`,
  `manual_forward_cmd` becomes negative, and both motor commands should move in
  the reverse direction.
- Straight right: `manual_turn_cmd` becomes positive and
  `manual_forward_cmd` stays near `0.000`.
- Straight left: `manual_turn_cmd` becomes negative and
  `manual_forward_cmd` stays near `0.000`.
- Diagonal stick positions should produce mixed steering plus throttle, not be
  required for straight forward or straight reverse.

## Direction Debug Plan

Use this plan whenever the manual direction mapping is changed. Do not judge by
wheel motion alone before checking `USBDBG`, because motor wiring and ESC
direction can also invert the final physical wheel direction.

1. Keep wheels off ground.
2. Flash the firmware and confirm the firmware marker.
3. Leave the stick centered and confirm:
   - `manual_forward_cmd=0.000`
   - `manual_turn_cmd=0.000`
   - `manual_logical_left_cmd=0.000`
   - `manual_logical_right_cmd=0.000`
   - `old_angle_remap_active=false`
   - `left_cmd=0.000`
   - `right_cmd=0.000`
4. Move the stick straight up and confirm:
   - `manual_turn_cmd` remains near zero
   - `manual_forward_cmd` is positive
   - `left_cmd` and `right_cmd` are both positive or both forward-equivalent
5. Move the stick straight down and confirm:
   - `manual_turn_cmd` remains near zero
   - `manual_forward_cmd` is negative
   - `left_cmd` and `right_cmd` are both negative or both reverse-equivalent
6. Move the stick straight right and left and confirm steering changes without a
   large throttle bias.
7. Only after the command values are correct, check whether either physical motor
   direction is reversed. If the command values are correct but a wheel spins the
   wrong way, fix motor/ESC side direction separately instead of changing the RC
   axis map.

Recommended current manual RC test build:

```bash
arduino-cli compile \
  --fqbn OpenRB-150:samd:OpenRB-150 \
  --build-path /private/tmp/openrb-manual-final-sign \
  --build-property 'compiler.cpp.extra_flags=-DMANUAL_FORWARD_SIGN=-1 -DMANUAL_TURN_SIGN=1 -DMOTOR_OUTPUT_SWAP_LR=0 -DDRIVE_CALIBRATION_ENABLE=0' \
  firmware/openrb_robot_controller
```

The current correction was chosen from the observed failure sequence:

| Attempt | Observed Problem | Result |
|---|---|---|
| wrong 45-degree remap | left/right became forward/reverse | rejected |
| direct CH1/CH2 map | straight up/down did not become forward/reverse | rejected |
| direct CH2 inversion | upper-left became forward and lower-right became reverse | rejected |
| old cardinal / angle remap | became harmful after physical A/B output mapping was fixed; upper-right became forward | rejected |
| arcade mixer with `MANUAL_FORWARD_SIGN=1`, `MANUAL_TURN_SIGN=1` | steering correct, but forward/reverse inverted | rejected for current controller |
| current arcade mixer with `MANUAL_FORWARD_SIGN=-1`, `MANUAL_TURN_SIGN=1` | stick up forward, down backward, right right-turn, left left-turn | active |

## Station Keyboard Manual Tool

Run station-side keyboard manual control only when the HC-12 USB serial device is
connected to the station host:

```bash
uv run python tools/station_keyboard_manual.py --port /dev/ttyACM0 --baud 9600 --max-speed 0.25
```

Use the actual station HC-12 USB serial port if it is different.

Keyboard controls:

| Key | Action |
|---|---|
| `e` | arm/disarm station manual mode |
| space | toggle deadman while armed |
| `W` or up arrow | throttle forward |
| `S` or down arrow | throttle backward |
| `A` or left arrow | steer left |
| `D` or right arrow | steer right |
| `n` | neutral axes |
| `x` | local E-stop/disarm and send `STOP` |
| `q` | quit and send repeated `STOP` |

Startup behavior is safe: before pressing `e`, the tool sends heartbeat plus
periodic `STOP`, not live motor-driving manual frames.

## Station Manual Protocol

The station sends:

```text
@CMD,SEQ,MANUAL,steer,throttle,deadman,estop*CS
```

Payload fields:

```text
MANUAL,steer,throttle,deadman,estop
```

Rules:

- `steer` and `throttle` are normalized `-1.0..1.0`.
- `deadman=1` is required for station-hw-manual.
- `estop=1` forces stop.
- Rover station-hw-manual frames must be fresh within `STATION_TIMEOUT_MS = 500`.
- First station-hw-manual tests are capped by the station tool at `--max-speed 0.25`.
- Rover-side station-hw-manual output is also capped by
  `STATION_MANUAL_MAX_OUTPUT = 0.25f`.

Station-drive command mixing in firmware uses the same differential formula:

```cpp
left = stationManual.throttle - stationManual.steer;
right = stationManual.throttle + stationManual.steer;
```

## Safety Gates In Current Firmware

The rover loop chooses the active control source in this order:

1. Station E-stop -> `STOP`.
2. Fresh station manual frame with `deadman=1` -> `STATION_MANUAL`.
3. Valid RC with CH5 manual mode -> `RC_MANUAL`.
4. Valid RC with CH5 auto mode and explicit station `AUTO` command -> `AUTO`.
5. Anything stale/invalid -> `STOP`, `AUTO_READY`, `DISARMED`, or `FAILSAFE`.

Important defaults:

- `motorStop()` runs during setup.
- CH5 high alone does not run autonomous drive; it only enters `AUTO_READY`.
- In the current inhibited single-waypoint experiment, `AUTO_MOTION_ARMED=0`
  and `auto_motor_inhibit=true` must keep AUTO final commands at zero.
- `distance_allowed=true` observed in MANUAL is not enough to approve motion;
  the same run must be verified in `AUTO_READY` with all safety flags true
  before any later wheel-off-ground bench test.
- Station startup must not send live motor-driving `AUTO` commands.
- Link loss or stale manual frames return outputs to neutral.

## Quick Checklist

1. Keep rover wheels off ground.
2. Flash `firmware/openrb_robot_controller/openrb_robot_controller.ino`.
3. Confirm firmware marker is `rc-arcade-manual-fwdneg`.
4. Confirm the station/controller is powered on and linked.
5. Confirm MANUAL shows `mode_us≈1000` and `control_source=RC_MANUAL`.
6. Confirm AUTO shows `mode_us≈2000`, `mode=AUTO_READY`, and
   `control_source=STOP` before any autonomy dry-run interpretation.
7. Confirm neutral USBDBG has `left_cmd=0.000 right_cmd=0.000`.
8. Push stick forward and confirm forward wheel direction.
9. Pull stick backward and confirm reverse wheel direction.
10. Confirm `x`/`STOP` behavior before any ground-contact test.
