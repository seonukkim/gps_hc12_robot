Update firmware/openrb_robot_controller/openrb_robot_controller.ino to add periodic USB Serial debug output for dry-run validation.

Context:
- The firmware compiles for OpenRB-150.
- It currently initializes Serial in setup and prints startup messages only once.
- Status frames are mostly sent through HC12_SERIAL / Serial2, so USB picocom appears silent after boot.
- rc_mix_test has already validated:
  - STEERING_CHANNEL_INDEX = 0
  - THROTTLE_CHANNEL_INDEX = 2
  - MODE_CHANNEL_INDEX = 4
  - STEERING_CENTER_US = 1490
  - THROTTLE_CENTER_US = 1546
  - RC_DEADBAND_US = 80
  - CH5 low/mid -> MANUAL
  - CH5 high -> AUTO
  - transmitter off -> RC_BAD/failsafe
  - neutral output -> 1500/1500

Requirements:
1. Do not upload firmware.
2. Preserve all motor safety behavior.
3. Add:
   constexpr bool ENABLE_USB_DEBUG = true;
   constexpr uint32_t USB_DEBUG_PERIOD_MS = 500;
   uint32_t lastUsbDebugMs = 0;
4. Add a debugPrintStatus() function that prints to USB Serial every 500 ms:
   - mode name using existing modeName() if available
   - rcChannelsValid()
   - rcAutoSwitchOn()
   - age since last PPM frame
   - selected steering/throttle/mode pulse values
   - normalized steering/throttle values using the same centered calibration function
   - station link age
   - last station sequence if available
   - explicit motor/command safety state such as STOP/NEUTRAL when RC invalid
5. The debug output must not send any motor-driving command by itself.
6. Call debugPrintStatus() from loop().
7. Compile:
   arduino-cli compile --fqbn OpenRB-150:samd:OpenRB-150 firmware/openrb_robot_controller
8. Report changed sections and compile result.
