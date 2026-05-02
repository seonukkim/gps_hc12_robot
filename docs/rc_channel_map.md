# RC Channel Map

Controller: RadioLink T8FB BT  
Receiver output: PPM  
OpenRB PPM input pin: D6  
Test firmware: `firmware/ppm_test/ppm_test.ino`  
Serial baudrate: 115200

## Confirmed T8FB channel map

| Printed Channel | T8FB Function | Rover Use |
|---:|---|---|
| CH1 | AIL / right stick horizontal | Steering |
| CH2 | ELE / right stick vertical | Unused initially |
| CH3 | THR / left stick vertical | Throttle |
| CH4 | RUD / left stick horizontal | Optional steering/yaw |
| CH5 | SWB / right top switch | Manual/Auto mode switch |
| CH6 | VR-B / right knob | Optional speed limit |
| CH7 | SWA / left top switch | Optional arm/disarm |
| CH8 | VR-A / left knob | Optional tuning |

## Index note for Arduino code

The PPM printout is 1-based, but the firmware array is 0-based.

- Printed CH1 -> code index 0
- Printed CH2 -> code index 1
- Printed CH3 -> code index 2
- Printed CH4 -> code index 3
- Printed CH5 -> code index 4
- Printed CH6 -> code index 5
- Printed CH7 -> code index 6
- Printed CH8 -> code index 7

## Mapping used by rover firmware

- Steering: CH1 -> index 0
- Throttle: CH3 -> index 2
- Manual/Auto: CH5 -> index 4

## Required final checks before motor test

1. Move right stick left/right and confirm CH1 changes.
2. Move left stick up/down and confirm CH3 changes.
3. Toggle SWB and confirm CH5 changes between low/high values.
4. Turn transmitter off and record receiver failsafe behavior.
5. Do not upload motor-control firmware until wheels are off the ground.

## Confirmed observations

- SWB up: CH5 ≈ 1001 us → MANUAL
- SWB down: CH5 ≈ 2001 us → AUTO
- Transmitter OFF: CH3 ≈ 876 us
  - Current firmware threshold: RC_MIN_VALID_US = 900
  - Therefore transmitter OFF should be treated as RC_BAD / FAILSAFE.

## Current safety interpretation

- MODE_CHANNEL_INDEX = 4
- RC_AUTO_SWITCH_ON_US = 1600
- CH5 > 1600 → AUTO
- CH5 <= 1600 → MANUAL
- CH3 < 900 when transmitter is OFF → invalid RC signal

## Confirmed rc_mix_test result after calibration

- Steering center: around 1490 us
- Throttle center: around 1546 us after transmitter reset/recenter
- CH5 low: around 1001 us -> MANUAL
- CH5 middle: around 1501 us -> MANUAL
- CH5 high: around 2001~2002 us -> AUTO
- Transmitter OFF: CH3 around 875~876 us -> RC_BAD
- RC_BAD output: virtual_left_us=1500, virtual_right_us=1500
- Hands-off output: throttle=0.00, virtual_left_us=1500, virtual_right_us=1500

Safety conclusion:
- RC mode switch mapping is valid.
- RC failsafe detection is valid.
- Neutral motor command is now safe in rc_mix_test.
