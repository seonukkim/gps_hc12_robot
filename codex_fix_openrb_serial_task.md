Fix firmware/openrb_robot_controller/openrb_robot_controller.ino for OpenRB-150 SAMD compilation.

Current error:
- #include <SoftwareSerial.h> fails on OpenRB-150 SAMD core.

Important hardware context:
- OpenRB-150 is SAMD21 based.
- Do not use SoftwareSerial.
- Use hardware serial instead.
- For now, use Serial2 as the HC-12 UART because OpenRB documentation exposes a Serial2 serial port.
- GPS telemetry should be guarded behind a compile-time flag until the actual second UART mapping is confirmed.
- Do not upload firmware.
- Do not change motor behavior except what is necessary to compile.
- Preserve safety logic:
  - RC invalid -> failsafe stop
  - manual override priority
  - station timeout -> stop
  - STOP command -> stop
  - AUTO accepted only when RC valid and mode switch is on

Required changes:
1. Remove `#include <SoftwareSerial.h>`.
2. Remove any `SoftwareSerial` object declarations.
3. Add near the top:
   constexpr bool ENABLE_GPS_TELEMETRY = false;
   #define HC12_SERIAL Serial2
4. Ensure HC-12 code uses `HC12_SERIAL.begin(HC12_BAUD)`, `.available()`, `.read()`, `.print()`, `.println()`, `.write()` as appropriate.
5. Guard all GPS serial begin/read code with:
   #if ENABLE_GPS_TELEMETRY
   ...
   #endif
6. If GPS telemetry is disabled, status/telemetry should still compile and either skip GPS sending or send a clear GPS_DISABLED/NO_FIX frame.
7. Keep channel mapping:
   STEERING_CHANNEL_INDEX = 0
   THROTTLE_CHANNEL_INDEX = 2
   MODE_CHANNEL_INDEX = 4
8. Run:
   arduino-cli compile --fqbn OpenRB-150:samd:OpenRB-150 firmware/openrb_robot_controller
9. If compilation fails, inspect and fix the next compile error.
10. Report changed sections and compile result.
