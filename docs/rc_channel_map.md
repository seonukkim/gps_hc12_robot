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
