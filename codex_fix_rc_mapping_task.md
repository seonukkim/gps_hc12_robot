Read firmware/openrb_robot_controller/openrb_robot_controller.ino and update the RC channel mapping to match the confirmed RadioLink T8FB PPM mapping.

Confirmed mapping:
- Printed CH1 = steering = code index 0
- Printed CH3 = throttle = code index 2
- Printed CH5 = manual/auto mode switch = code index 4

Important:
- Do not upload firmware.
- Do not add motor-driving behavior.
- Only modify source files and documentation.
- Preserve existing safety logic.
- If the current file does not have clearly named constants for steering/throttle/mode channels, add explicit constants near the top:
  STEERING_CHANNEL_INDEX = 0
  THROTTLE_CHANNEL_INDEX = 2
  MODE_CHANNEL_INDEX = 4
  or equivalent names consistent with the file style.
- Replace any hard-coded old channel indexes with these constants.
- Add comments explaining that printed CH numbers are 1-based while arrays are 0-based.
- Update docs/rc_channel_map.md if needed.
- Run:
  arduino-cli compile --fqbn OpenRB-150:samd:OpenRB-150 firmware/openrb_robot_controller
  arduino-cli compile --fqbn OpenRB-150:samd:OpenRB-150 firmware/ppm_test
- Report what changed and whether compile passed.
