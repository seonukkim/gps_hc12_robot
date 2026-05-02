# RC Channel Map

Context:
- Station controller: disassembled RadioLink transmitter integrated into the Jetson/station box
- Receiver output: PPM into OpenRB D6
- Verification sources: `firmware/rc_mix_test/rc_mix_test.ino` output and rover `USBDBG` logs

## Station Integrated Controller Mapping

| PPM Channel | Station Control | Firmware Use | Notes |
|---:|---|---|---|
| CH1 | Steering joystick horizontal | Steering | `STEERING_CHANNEL_INDEX = 0` |
| CH2 | Throttle joystick vertical | Throttle | `THROTTLE_CHANNEL_INDEX = 1` |
| CH5 | Manual/Auto switch | Mode select | `MODE_CHANNEL_INDEX = 4` |
| CH7 | Reserved / unused | None | Do not use for mode |

The station panel can physically look like `CH7`, but the firmware uses the receiver's PPM `CH5` for Manual/Auto mode. `CH7` is reserved/unused for now.

## Firmware Constants

- `STEERING_CHANNEL_INDEX = 0`
- `THROTTLE_CHANNEL_INDEX = 1`
- `MODE_CHANNEL_INDEX = 4`
- `STEERING_CENTER_US = 1504`
- `THROTTLE_CENTER_US = 1500`
- `RC_DEADBAND_US = 80`
- `RC_AUTO_SWITCH_ON_US = 1600`

## Mode Interpretation

- `CH5 <= 1600 us` -> `MANUAL`
- `CH5 > 1600 us` -> `AUTO_READY`
- `CH5` high by itself must not drive motors
- In `AUTO_READY`, `control_source` remains `STOP` until a valid explicit station/autonomous command is accepted
- `MANUAL` allows RC manual drive

## Confirmed Observations

- `CH1` moves with steering input
- `CH2` moves with throttle input
- `CH5` low/mid stays `MANUAL`
- `CH5` high selects `AUTO_READY`
- `CH7` is not used by firmware for mode
- Transmitter/link loss drives RC invalid handling and motor stop behavior

## Bench Check

1. Move the steering joystick and confirm `CH1` changes.
2. Move the throttle joystick and confirm `CH2` changes.
3. Toggle the Manual/Auto switch and confirm `CH5` changes between low/high values.
4. Confirm `CH7` may move independently, but it does not affect rover mode.
5. Turn the transmitter off or break the RC link and confirm the rover returns to stop/failsafe behavior.
