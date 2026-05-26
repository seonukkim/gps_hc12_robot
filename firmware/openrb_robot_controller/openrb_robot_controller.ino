#include <Servo.h>
#include <TinyGPS++.h>

#define ENABLE_GPS_TELEMETRY 1
#define HC12_SERIAL Serial2
#define GPS_SERIAL Serial3

constexpr const char *FIRMWARE_ID = "openrb_robot_controller station-manual rc-cardinal-remap 2026-05-26";
constexpr uint8_t PPM_PIN = 6;
constexpr uint8_t ESC_LEFT_PIN = 4;
constexpr uint8_t ESC_RIGHT_PIN = 5;
constexpr long HC12_BAUD = 9600;
constexpr long GPS_BAUD = 9600;
constexpr long USB_BAUD = 115200;

constexpr uint8_t CHANNEL_COUNT = 8;
// Printed transmitter labels are 1-based (CH1-CH8), while ppmChannels[] uses 0-based indexes.
// The integrated station controller can physically look like CH7 at the panel,
// but the receiver PPM mapping used by firmware is CH1 steering, CH2 throttle,
// and CH5 Manual/Auto. CH7 is reserved/unused for now.
constexpr uint8_t STEERING_CHANNEL_INDEX = 0;  // Station joystick horizontal: PPM CH1
constexpr uint8_t THROTTLE_CHANNEL_INDEX = 1;  // Station joystick vertical: PPM CH2
constexpr uint8_t MODE_CHANNEL_INDEX = 4;      // Station Manual/Auto switch: PPM CH5
constexpr uint16_t STEERING_CENTER_US = 1504;
constexpr uint16_t THROTTLE_CENTER_US = 1500;
constexpr uint16_t RC_DEADBAND_US = 80;
constexpr uint16_t RC_MIN_VALID_US = 900;
constexpr uint16_t RC_MAX_VALID_US = 2100;
constexpr uint16_t RC_AUTO_SWITCH_ON_US = 1600;
constexpr float RC_MANUAL_AXIS_ROTATION_SCALE = 0.70710678f;
constexpr uint16_t ESC_NEUTRAL_US = 1500;
constexpr uint16_t ESC_RANGE_US = 300;
constexpr uint32_t STATION_TIMEOUT_MS = 500;
constexpr float STATION_MANUAL_MAX_OUTPUT = 0.25f;
constexpr uint32_t TELEMETRY_PERIOD_MS = 1000;
constexpr uint32_t STATUS_PERIOD_MS = 500;
constexpr bool ENABLE_USB_DEBUG = true;
constexpr uint32_t USB_DEBUG_PERIOD_MS = 500;

enum RobotMode {
  DISARMED,
  MANUAL,
  AUTO_READY,
  AUTO_RUNNING,
  LINK_LOST,
  FAILSAFE
};

enum ControlSource {
  CONTROL_SOURCE_STOP,
  CONTROL_SOURCE_RC_MANUAL,
  CONTROL_SOURCE_STATION_MANUAL,
  CONTROL_SOURCE_AUTO
};

struct StationManualCommand {
  float steer;
  float throttle;
  bool deadman;
  uint32_t lastFrameMs;
};

#if ENABLE_GPS_TELEMETRY
TinyGPSPlus gps;
#endif
Servo escLeft;
Servo escRight;

volatile uint16_t ppmChannels[CHANNEL_COUNT] = {0};
volatile uint8_t ppmIndex = 0;
volatile uint32_t lastPpmEdgeMicros = 0;
volatile uint32_t lastPpmFrameMs = 0;

RobotMode currentMode = DISARMED;
String hc12Line;
uint32_t lastStationFrameMs = 0;
long lastStationSeq = -1;
StationManualCommand stationManual = {0.0f, 0.0f, false, 0};
bool stationEstop = false;
bool autoCommandActive = false;
uint32_t lastTelemetryMs = 0;
uint32_t lastStatusMs = 0;
uint32_t lastUsbDebugMs = 0;
ControlSource currentControlSource = CONTROL_SOURCE_STOP;
float autoLeftCmd = 0.0f;
float autoRightCmd = 0.0f;
float lastLeftOutputCmd = 0.0f;
float lastRightOutputCmd = 0.0f;

void ppmISR() {
  uint32_t now = micros();
  uint32_t width = now - lastPpmEdgeMicros;
  lastPpmEdgeMicros = now;

  if (width > 3000) {
    ppmIndex = 0;
    lastPpmFrameMs = millis();
    return;
  }

  if (ppmIndex < CHANNEL_COUNT) {
    ppmChannels[ppmIndex] = width;
    ppmIndex++;
  }
}

uint8_t checksumXor(const String &body) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < body.length(); ++i) {
    checksum ^= static_cast<uint8_t>(body[i]);
  }
  return checksum;
}

void writeFrame(const char *type, uint32_t seq, const String &payload) {
  String body = String(type) + "," + String(seq);
  if (payload.length() > 0) {
    body += "," + payload;
  }

  uint8_t checksum = checksumXor(body);
  HC12_SERIAL.print("@");
  HC12_SERIAL.print(body);
  HC12_SERIAL.print("*");
  if (checksum < 0x10) {
    HC12_SERIAL.print("0");
  }
  HC12_SERIAL.println(checksum, HEX);
}

void motorStop() {
  escLeft.writeMicroseconds(ESC_NEUTRAL_US);
  escRight.writeMicroseconds(ESC_NEUTRAL_US);
  lastLeftOutputCmd = 0.0f;
  lastRightOutputCmd = 0.0f;
}

void clearAutoCommand() {
  autoLeftCmd = 0.0f;
  autoRightCmd = 0.0f;
  autoCommandActive = false;
}

uint16_t clampPulse(int pulse) {
  if (pulse < ESC_NEUTRAL_US - ESC_RANGE_US) {
    return ESC_NEUTRAL_US - ESC_RANGE_US;
  }
  if (pulse > ESC_NEUTRAL_US + ESC_RANGE_US) {
    return ESC_NEUTRAL_US + ESC_RANGE_US;
  }
  return pulse;
}

float clampUnit(float value) {
  if (value > 1.0f) {
    return 1.0f;
  }
  if (value < -1.0f) {
    return -1.0f;
  }
  return value;
}

float absFloat(float value) {
  return value < 0.0f ? -value : value;
}

void applyDriveCommand(float left, float right) {
  left = clampUnit(left);
  right = clampUnit(right);
  lastLeftOutputCmd = left;
  lastRightOutputCmd = right;

  int leftPulse = ESC_NEUTRAL_US + static_cast<int>(left * ESC_RANGE_US);
  int rightPulse = ESC_NEUTRAL_US + static_cast<int>(right * ESC_RANGE_US);
  escLeft.writeMicroseconds(clampPulse(leftPulse));
  escRight.writeMicroseconds(clampPulse(rightPulse));
}

void applyAutoCommand(float left, float right) {
  // Wheel-off-ground only for ESC validation.
  applyDriveCommand(left, right);
}

bool stationLinkValid() {
  return lastStationFrameMs != 0 && millis() - lastStationFrameMs <= STATION_TIMEOUT_MS;
}

bool stationManualValid() {
  return stationManual.lastFrameMs != 0 && millis() - stationManual.lastFrameMs <= STATION_TIMEOUT_MS;
}

void clearStationManualCommand() {
  stationManual.steer = 0.0f;
  stationManual.throttle = 0.0f;
  stationManual.deadman = false;
  stationManual.lastFrameMs = 0;
}

bool rcFrameRecent() {
  return millis() - lastPpmFrameMs < 200;
}

void readRcChannels(uint16_t &steering, uint16_t &throttle, uint16_t &mode) {
  noInterrupts();
  steering = ppmChannels[STEERING_CHANNEL_INDEX];
  throttle = ppmChannels[THROTTLE_CHANNEL_INDEX];
  mode = ppmChannels[MODE_CHANNEL_INDEX];
  interrupts();
}

bool rcPulseValid(uint16_t pulseUs) {
  return pulseUs >= RC_MIN_VALID_US && pulseUs <= RC_MAX_VALID_US;
}

float normRcCentered(uint16_t pulseUs, uint16_t centerUs) {
  int delta = static_cast<int>(pulseUs) - static_cast<int>(centerUs);

  if (abs(delta) <= RC_DEADBAND_US) {
    return 0.0f;
  }

  float denom = delta > 0 ? (2000.0f - centerUs) : (centerUs - 1000.0f);
  if (denom < 1.0f) {
    return 0.0f;
  }

  return clampUnit(static_cast<float>(delta) / denom);
}

void mapRcManualAxes(float rawSteering, float rawThrottle, float &steeringOut, float &throttleOut) {
  steeringOut = clampUnit((rawSteering + rawThrottle) * RC_MANUAL_AXIS_ROTATION_SCALE);
  throttleOut = clampUnit((rawSteering - rawThrottle) * RC_MANUAL_AXIS_ROTATION_SCALE);
}

bool rcChannelsValid(uint16_t steering, uint16_t throttle, uint16_t mode) {
  if (!rcFrameRecent()) {
    return false;
  }

  return rcPulseValid(steering) && rcPulseValid(throttle) && rcPulseValid(mode);
}

bool rcChannelsValid() {
  uint16_t steering = 0;
  uint16_t throttle = 0;
  uint16_t mode = 0;
  readRcChannels(steering, throttle, mode);
  return rcChannelsValid(steering, throttle, mode);
}

bool rcAutoSwitchOn(uint16_t mode) {
  return mode > RC_AUTO_SWITCH_ON_US;
}

bool rcAutoSwitchOn() {
  uint16_t steering = 0;
  uint16_t throttle = 0;
  uint16_t mode = 0;
  readRcChannels(steering, throttle, mode);
  return rcAutoSwitchOn(mode);
}

void applyManualOverride(uint16_t steeringUs, uint16_t throttleUs) {
  float rawSteering = normRcCentered(steeringUs, STEERING_CENTER_US);
  float rawThrottle = normRcCentered(throttleUs, THROTTLE_CENTER_US);
  float steering = 0.0f;
  float throttle = 0.0f;
  mapRcManualAxes(rawSteering, rawThrottle, steering, throttle);

  float left = throttle - steering;
  float right = throttle + steering;
  applyDriveCommand(left, right);
}

void applyStationManualCommand() {
  float left = stationManual.throttle - stationManual.steer;
  float right = stationManual.throttle + stationManual.steer;
  float maxMagnitude = absFloat(left);
  if (absFloat(right) > maxMagnitude) {
    maxMagnitude = absFloat(right);
  }
  if (maxMagnitude > STATION_MANUAL_MAX_OUTPUT && maxMagnitude > 0.0f) {
    float scale = STATION_MANUAL_MAX_OUTPUT / maxMagnitude;
    left *= scale;
    right *= scale;
  }
  applyDriveCommand(left, right);
}

const char *modeName(RobotMode mode) {
  switch (mode) {
    case DISARMED: return "DISARMED";
    case MANUAL: return "MANUAL";
    case AUTO_READY: return "AUTO_READY";
    case AUTO_RUNNING: return "AUTO_RUNNING";
    case LINK_LOST: return "LINK_LOST";
    case FAILSAFE: return "FAILSAFE";
    default: return "UNKNOWN";
  }
}

const char *controlSourceName(ControlSource source) {
  switch (source) {
    case CONTROL_SOURCE_RC_MANUAL: return "RC_MANUAL";
    case CONTROL_SOURCE_STATION_MANUAL: return "STATION_MANUAL";
    case CONTROL_SOURCE_AUTO: return "AUTO";
    case CONTROL_SOURCE_STOP:
    default: return "STOP";
  }
}

bool parseFloatToken(const String &token, float &valueOut) {
  char *endPtr = nullptr;
  valueOut = strtof(token.c_str(), &endPtr);
  return endPtr != token.c_str() && *endPtr == '\0';
}

bool parseBoolToken(const String &token, bool &valueOut) {
  if (token == "1" || token.equalsIgnoreCase("true")) {
    valueOut = true;
    return true;
  }
  if (token == "0" || token.equalsIgnoreCase("false")) {
    valueOut = false;
    return true;
  }
  return false;
}

bool splitCsvFields(const String &text, String *fields, size_t maxFields, size_t &fieldCount) {
  fieldCount = 0;
  int start = 0;
  while (start <= text.length()) {
    if (fieldCount >= maxFields) {
      return false;
    }
    int comma = text.indexOf(',', start);
    if (comma < 0) {
      fields[fieldCount++] = text.substring(start);
      return true;
    }
    fields[fieldCount++] = text.substring(start, comma);
    start = comma + 1;
  }
  return true;
}

void debugPrintStatus() {
  if (!ENABLE_USB_DEBUG) {
    return;
  }

  uint32_t now = millis();
  if (now - lastUsbDebugMs < USB_DEBUG_PERIOD_MS) {
    return;
  }
  lastUsbDebugMs = now;

  uint16_t steeringUs = 0;
  uint16_t throttleUs = 0;
  uint16_t modeUs = 0;
  uint32_t ppmFrameMs = 0;
  noInterrupts();
  steeringUs = ppmChannels[STEERING_CHANNEL_INDEX];
  throttleUs = ppmChannels[THROTTLE_CHANNEL_INDEX];
  modeUs = ppmChannels[MODE_CHANNEL_INDEX];
  ppmFrameMs = lastPpmFrameMs;
  interrupts();

  bool rcValid = rcChannelsValid(steeringUs, throttleUs, modeUs);
  bool autoSwitchOn = rcAutoSwitchOn(modeUs);
  float steeringNorm = normRcCentered(steeringUs, STEERING_CENTER_US);
  float throttleNorm = normRcCentered(throttleUs, THROTTLE_CENTER_US);
  float manualSteering = 0.0f;
  float manualThrottle = 0.0f;
  mapRcManualAxes(steeringNorm, throttleNorm, manualSteering, manualThrottle);

  Serial.print(F("USBDBG mode="));
  Serial.print(modeName(currentMode));
  Serial.print(F(" rc_ok="));
  Serial.print(rcValid ? F("true") : F("false"));
  Serial.print(F(" auto_sw="));
  Serial.print(autoSwitchOn ? F("true") : F("false"));
  Serial.print(F(" ppm_age_ms="));
  if (ppmFrameMs == 0) {
    Serial.print(F("NA"));
  } else {
    Serial.print(now - ppmFrameMs);
  }
  Serial.print(F(" steer_us="));
  Serial.print(steeringUs);
  Serial.print(F(" throttle_us="));
  Serial.print(throttleUs);
  Serial.print(F(" mode_us="));
  Serial.print(modeUs);
  Serial.print(F(" steer_norm="));
  Serial.print(steeringNorm, 3);
  Serial.print(F(" throttle_norm="));
  Serial.print(throttleNorm, 3);
  Serial.print(F(" manual_steer_cmd="));
  Serial.print(manualSteering, 3);
  Serial.print(F(" manual_throttle_cmd="));
  Serial.print(manualThrottle, 3);
  Serial.print(F(" station_age_ms="));
  if (lastStationFrameMs == 0) {
    Serial.print(F("NA"));
  } else {
    Serial.print(now - lastStationFrameMs);
  }
  Serial.print(F(" station_seq="));
  if (lastStationSeq < 0) {
    Serial.print(F("NA"));
  } else {
    Serial.print(lastStationSeq);
  }
  Serial.print(F(" station_manual_valid="));
  Serial.print(stationManualValid() ? F("true") : F("false"));
  Serial.print(F(" station_deadman="));
  Serial.print(stationManual.deadman ? F("true") : F("false"));
  Serial.print(F(" station_estop="));
  Serial.print(stationEstop ? F("true") : F("false"));
  Serial.print(F(" control_source="));
  Serial.print(controlSourceName(currentControlSource));
  Serial.print(F(" left_cmd="));
  Serial.print(lastLeftOutputCmd, 3);
  Serial.print(F(" right_cmd="));
  Serial.print(lastRightOutputCmd, 3);
  Serial.print(F(" gps_fix="));
  Serial.print(gps.location.isValid() ? F("true") : F("false"));
  Serial.print(F(" gps_lat="));
  if (gps.location.isValid()) {
    Serial.print(gps.location.lat(), 6);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" gps_lon="));
  if (gps.location.isValid()) {
    Serial.print(gps.location.lng(), 6);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" gps_sats="));
  if (gps.satellites.isValid()) {
    Serial.print(gps.satellites.value());
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" gps_hdop="));
  if (gps.hdop.isValid()) {
    Serial.print(gps.hdop.hdop(), 2);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" gps_age_ms="));
  if (gps.location.isValid()) {
    Serial.println(gps.location.age());
  } else {
    Serial.println(F("NA"));
  }
}

void sendStatus(uint32_t refSeq) {
  String payload = String(modeName(currentMode));
  payload += rcChannelsValid() ? ",RC_OK" : ",RC_BAD";
  payload += stationLinkValid() ? ",LINK_OK," : ",LINK_LOST,";
  payload += String(refSeq);
  writeFrame("STAT", refSeq, payload);
}

void sendAck(uint32_t seq, const char *result) {
  writeFrame("ACK", seq, String(result));
}

void sendErr(uint32_t seq, const char *reason) {
  writeFrame("ERR", seq, String(reason));
}

void sendGpsTelemetry() {
#if ENABLE_GPS_TELEMETRY
  String payload;
  payload.reserve(96);
  payload += "fix=";
  payload += gps.location.isValid() ? "1" : "0";
  payload += ",lat=";
  if (gps.location.isValid()) {
    payload += String(gps.location.lat(), 6);
  } else {
    payload += "NA";
  }
  payload += ",lon=";
  if (gps.location.isValid()) {
    payload += String(gps.location.lng(), 6);
  } else {
    payload += "NA";
  }
  payload += ",sats=";
  if (gps.satellites.isValid()) {
    payload += String(gps.satellites.value());
  } else {
    payload += "NA";
  }
  payload += ",hdop=";
  if (gps.hdop.isValid()) {
    payload += String(gps.hdop.hdop(), 2);
  } else {
    payload += "NA";
  }
  payload += ",age_ms=";
  if (gps.location.isValid()) {
    payload += String(gps.location.age());
  } else {
    payload += "NA";
  }
  writeFrame("GPS", millis(), payload);
#else
  writeFrame("GPS", millis(), "GPS_DISABLED");
#endif
}

bool decodeFrame(const String &line, String &typeOut, long &seqOut, String &payloadOut) {
  if (!line.startsWith("@")) {
    return false;
  }

  int star = line.lastIndexOf('*');
  if (star < 0) {
    return false;
  }

  String body = line.substring(1, star);
  String checksumText = line.substring(star + 1);
  checksumText.trim();
  if (checksumText.length() == 0) {
    return false;
  }

  char *endPtr = nullptr;
  long expected = strtol(checksumText.c_str(), &endPtr, 16);
  if (endPtr == checksumText.c_str() || *endPtr != '\0') {
    return false;
  }
  if (checksumXor(body) != static_cast<uint8_t>(expected)) {
    return false;
  }

  int firstComma = body.indexOf(',');
  int secondComma = body.indexOf(',', firstComma + 1);
  if (firstComma <= 0 || secondComma <= firstComma) {
    return false;
  }

  typeOut = body.substring(0, firstComma);
  String seqText = body.substring(firstComma + 1, secondComma);
  payloadOut = body.substring(secondComma + 1);
  seqOut = seqText.toInt();
  return seqText == String(seqOut);
}

void handleCommand(long seq, const String &payload) {
  String fields[5];
  size_t fieldCount = 0;
  if (!splitCsvFields(payload, fields, 5, fieldCount) || fieldCount == 0) {
    sendErr(seq, "CMD_PAYLOAD");
    return;
  }

  String command = fields[0];

  if (command == "STOP") {
    stationEstop = true;
    clearStationManualCommand();
    clearAutoCommand();
    motorStop();
    currentControlSource = CONTROL_SOURCE_STOP;
    sendAck(seq, "OK");
    return;
  }

  if (command == "MANUAL") {
    if (fieldCount != 5) {
      sendErr(seq, "MANUAL_PAYLOAD");
      return;
    }

    float steer = 0.0f;
    float throttle = 0.0f;
    bool deadman = false;
    bool estop = false;
    if (!parseFloatToken(fields[1], steer) || !parseFloatToken(fields[2], throttle) ||
        !parseBoolToken(fields[3], deadman) || !parseBoolToken(fields[4], estop)) {
      sendErr(seq, "MANUAL_PAYLOAD");
      return;
    }

    clearAutoCommand();
    stationManual.steer = clampUnit(steer);
    stationManual.throttle = clampUnit(throttle);
    stationManual.deadman = deadman;
    stationManual.lastFrameMs = millis();
    stationEstop = estop;
    if (stationEstop) {
      motorStop();
      currentControlSource = CONTROL_SOURCE_STOP;
    }
    sendAck(seq, "OK");
    return;
  }

  if (command == "AUTO" || command == "START") {
    if (fieldCount != 3) {
      sendErr(seq, "AUTO_PAYLOAD");
      return;
    }

    if (!rcChannelsValid() || !rcAutoSwitchOn()) {
      clearAutoCommand();
      motorStop();
      sendErr(seq, "AUTO_REJECTED");
      return;
    }

    float left = 0.0f;
    float right = 0.0f;
    if (!parseFloatToken(fields[1], left) || !parseFloatToken(fields[2], right)) {
      sendErr(seq, "AUTO_PAYLOAD");
      return;
    }

    autoLeftCmd = clampUnit(left);
    autoRightCmd = clampUnit(right);
    autoCommandActive = true;
    clearStationManualCommand();
    stationEstop = false;
    currentMode = AUTO_RUNNING;
    currentControlSource = CONTROL_SOURCE_AUTO;
    applyAutoCommand(autoLeftCmd, autoRightCmd);
    sendAck(seq, "OK");
    return;
  }

  sendErr(seq, "UNKNOWN_CMD");
}

void processHC12Line(const String &line) {
  String type;
  String payload;
  long seq = 0;
  if (!decodeFrame(line, type, seq, payload)) {
    sendErr(0, "BAD_FRAME");
    return;
  }

  lastStationFrameMs = millis();
  lastStationSeq = seq;

  if (type == "HB") {
    sendAck(seq, "OK");
  } else if (type == "CMD") {
    handleCommand(seq, payload);
  } else {
    sendErr(seq, "UNSUPPORTED");
  }
}

void setup() {
  Serial.begin(USB_BAUD);
  HC12_SERIAL.begin(HC12_BAUD);
#if ENABLE_GPS_TELEMETRY
  GPS_SERIAL.begin(GPS_BAUD);
#endif

  pinMode(PPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmISR, RISING);

  escLeft.attach(ESC_LEFT_PIN);
  escRight.attach(ESC_RIGHT_PIN);
  motorStop();

  Serial.print("Firmware: ");
  Serial.println(FIRMWARE_ID);
  Serial.println("OpenRB robot controller starting.");
  Serial.println("GPS telemetry uses OpenRB-150 Serial3 (D13/RX) at 9600 baud.");
  Serial.println("Motor tests are wheel-off-ground only.");
  Serial.println("RC mode input uses receiver PPM CH5; PPM CH7 is reserved/unused.");
  Serial.println("CH5 high enters AUTO_READY only; drive stays STOP until explicit AUTO.");
  Serial.println("Station manual accepts CMD,MANUAL only when fresh frames and deadman=1.");
}

void loop() {
#if ENABLE_GPS_TELEMETRY
  while (GPS_SERIAL.available() > 0) {
    gps.encode(GPS_SERIAL.read());
  }
#endif

  while (HC12_SERIAL.available() > 0) {
    char c = static_cast<char>(HC12_SERIAL.read());
    if (c == '\n') {
      processHC12Line(hc12Line);
      hc12Line = "";
    } else if (c != '\r') {
      hc12Line += c;
    }
  }

  uint16_t steeringUs = 0;
  uint16_t throttleUs = 0;
  uint16_t modeUs = 0;
  readRcChannels(steeringUs, throttleUs, modeUs);

  uint32_t now = millis();
  bool rcValid = rcChannelsValid(steeringUs, throttleUs, modeUs);
  bool autoSwitchOn = rcAutoSwitchOn(modeUs);
  bool stationManualFresh = stationManualValid();
  bool stationManualActive = stationManualFresh && stationManual.deadman && !stationEstop;
  bool rcManualActive = rcValid && !autoSwitchOn;

  if (autoCommandActive && !stationLinkValid()) {
    currentMode = LINK_LOST;
    clearAutoCommand();
    motorStop();
  }

  bool autoActive = rcValid && autoSwitchOn && autoCommandActive && stationLinkValid() && !stationEstop;

  if (stationEstop) {
    currentControlSource = CONTROL_SOURCE_STOP;
    currentMode = DISARMED;
    motorStop();
  } else if (stationManualActive) {
    currentMode = MANUAL;
    currentControlSource = CONTROL_SOURCE_STATION_MANUAL;
    applyStationManualCommand();
  } else if (rcManualActive) {
    currentMode = MANUAL;
    currentControlSource = CONTROL_SOURCE_RC_MANUAL;
    clearAutoCommand();
    applyManualOverride(steeringUs, throttleUs);
  } else if (autoActive) {
    currentMode = AUTO_RUNNING;
    currentControlSource = CONTROL_SOURCE_AUTO;
    applyAutoCommand(autoLeftCmd, autoRightCmd);
  } else {
    currentControlSource = CONTROL_SOURCE_STOP;
    motorStop();
    if (!rcValid) {
      currentMode = FAILSAFE;
    } else if (autoSwitchOn) {
      if (currentMode != LINK_LOST) {
        currentMode = AUTO_READY;
      }
    } else {
      currentMode = DISARMED;
    }
  }

  if (now - lastTelemetryMs > TELEMETRY_PERIOD_MS) {
    sendGpsTelemetry();
    lastTelemetryMs = now;
  }

  if (now - lastStatusMs > STATUS_PERIOD_MS) {
    sendStatus(lastStationSeq >= 0 ? static_cast<uint32_t>(lastStationSeq) : 0);
    lastStatusMs = now;
  }

  debugPrintStatus();
}
