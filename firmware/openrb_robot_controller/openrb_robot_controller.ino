#include <Servo.h>
#include <TinyGPS++.h>

constexpr bool ENABLE_GPS_TELEMETRY = false;
#define HC12_SERIAL Serial2

constexpr uint8_t PPM_PIN = 6;
constexpr uint8_t ESC_LEFT_PIN = 4;
constexpr uint8_t ESC_RIGHT_PIN = 5;
constexpr long HC12_BAUD = 9600;
constexpr long GPS_BAUD = 9600;

constexpr uint8_t CHANNEL_COUNT = 8;
// Printed transmitter labels are 1-based (CH1-CH8), while ppmChannels[] uses 0-based indexes.
constexpr uint8_t STEERING_CHANNEL_INDEX = 0;  // Printed CH1
constexpr uint8_t THROTTLE_CHANNEL_INDEX = 2;  // Printed CH3
constexpr uint8_t MODE_CHANNEL_INDEX = 4;      // Printed CH5
constexpr uint16_t RC_MIN_VALID_US = 900;
constexpr uint16_t RC_MAX_VALID_US = 2100;
constexpr uint16_t RC_AUTO_SWITCH_ON_US = 1600;
constexpr uint16_t ESC_NEUTRAL_US = 1500;
constexpr uint16_t ESC_RANGE_US = 300;
constexpr uint32_t STATION_TIMEOUT_MS = 1500;
constexpr uint32_t TELEMETRY_PERIOD_MS = 1000;
constexpr uint32_t STATUS_PERIOD_MS = 500;

enum RobotMode {
  DISARMED,
  MANUAL,
  AUTO_READY,
  AUTO_RUNNING,
  LINK_LOST,
  FAILSAFE
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
uint32_t lastTelemetryMs = 0;
uint32_t lastStatusMs = 0;
float autoLeftCmd = 0.0f;
float autoRightCmd = 0.0f;

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

void applyAutoCommand(float left, float right) {
  // Wheel-off-ground only for ESC validation.
  int leftPulse = ESC_NEUTRAL_US + static_cast<int>(left * ESC_RANGE_US);
  int rightPulse = ESC_NEUTRAL_US + static_cast<int>(right * ESC_RANGE_US);
  escLeft.writeMicroseconds(clampPulse(leftPulse));
  escRight.writeMicroseconds(clampPulse(rightPulse));
}

bool rcFrameRecent() {
  return millis() - lastPpmFrameMs < 200;
}

bool rcChannelsValid() {
  if (!rcFrameRecent()) {
    return false;
  }
  noInterrupts();
  uint16_t steering = ppmChannels[STEERING_CHANNEL_INDEX];
  uint16_t throttle = ppmChannels[THROTTLE_CHANNEL_INDEX];
  uint16_t mode = ppmChannels[MODE_CHANNEL_INDEX];
  interrupts();

  return steering >= RC_MIN_VALID_US && steering <= RC_MAX_VALID_US &&
         throttle >= RC_MIN_VALID_US && throttle <= RC_MAX_VALID_US &&
         mode >= RC_MIN_VALID_US && mode <= RC_MAX_VALID_US;
}

bool rcAutoSwitchOn() {
  noInterrupts();
  uint16_t mode = ppmChannels[MODE_CHANNEL_INDEX];
  interrupts();
  return mode > RC_AUTO_SWITCH_ON_US;
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

void sendStatus(uint32_t refSeq) {
  String payload = String(modeName(currentMode));
  payload += rcChannelsValid() ? ",RC_OK" : ",RC_BAD";
  payload += (millis() - lastStationFrameMs < STATION_TIMEOUT_MS) ? ",LINK_OK," : ",LINK_LOST,";
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
  if (!gps.location.isValid()) {
    writeFrame("GPS", millis(), "NO_FIX");
    return;
  }

  String payload;
  payload.reserve(64);
  payload += String(gps.location.lat(), 6);
  payload += ",";
  payload += String(gps.location.lng(), 6);
  payload += ",";
  payload += String(gps.altitude.meters(), 1);
  payload += ",";
  payload += String(gps.satellites.value());
  payload += ",";
  payload += String(gps.hdop.hdop(), 1);
  payload += ",";
  payload += gps.location.isValid() ? "1" : "0";
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
  int comma = payload.indexOf(',');
  String command = comma >= 0 ? payload.substring(0, comma) : payload;

  if (command == "STOP") {
    motorStop();
    currentMode = rcAutoSwitchOn() ? AUTO_READY : MANUAL;
    sendAck(seq, "OK");
    return;
  }

  if (command == "AUTO") {
    if (!rcChannelsValid() || !rcAutoSwitchOn()) {
      currentMode = MANUAL;
      motorStop();
      sendErr(seq, "AUTO_REJECTED");
      return;
    }

    int first = payload.indexOf(',');
    int second = payload.indexOf(',', first + 1);
    if (first < 0 || second < 0) {
      sendErr(seq, "AUTO_PAYLOAD");
      return;
    }

    autoLeftCmd = payload.substring(first + 1, second).toFloat();
    autoRightCmd = payload.substring(second + 1).toFloat();
    currentMode = AUTO_RUNNING;
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

  if (type == "HB") {
    if (currentMode == DISARMED) {
      currentMode = rcAutoSwitchOn() ? AUTO_READY : MANUAL;
    }
    sendAck(seq, "OK");
  } else if (type == "CMD") {
    handleCommand(seq, payload);
  } else {
    sendErr(seq, "UNSUPPORTED");
  }
}

void setup() {
  Serial.begin(115200);
  HC12_SERIAL.begin(HC12_BAUD);
#if ENABLE_GPS_TELEMETRY
  // Confirm the OpenRB-150 secondary GPS UART mapping before enabling this path.
  Serial1.begin(GPS_BAUD);
#endif

  pinMode(PPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmISR, RISING);

  escLeft.attach(ESC_LEFT_PIN);
  escRight.attach(ESC_RIGHT_PIN);
  motorStop();

  Serial.println("OpenRB robot controller skeleton starting.");
  Serial.println("Confirm OpenRB-150 UART mapping before deployment.");
  Serial.println("Motor tests are wheel-off-ground only.");
}

void loop() {
#if ENABLE_GPS_TELEMETRY
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
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

  if (!rcChannelsValid()) {
    currentMode = FAILSAFE;
    motorStop();
  } else if (!rcAutoSwitchOn()) {
    if (currentMode == AUTO_RUNNING || currentMode == AUTO_READY || currentMode == LINK_LOST) {
      currentMode = MANUAL;
      motorStop();
    } else if (currentMode == DISARMED) {
      currentMode = MANUAL;
    }
  } else if (currentMode == MANUAL || currentMode == DISARMED) {
    currentMode = AUTO_READY;
    motorStop();
  }

  if (currentMode == AUTO_RUNNING && millis() - lastStationFrameMs > STATION_TIMEOUT_MS) {
    currentMode = LINK_LOST;
    motorStop();
  }

  if (millis() - lastTelemetryMs > TELEMETRY_PERIOD_MS) {
    sendGpsTelemetry();
    lastTelemetryMs = millis();
  }

  if (millis() - lastStatusMs > STATUS_PERIOD_MS) {
    sendStatus(lastStationFrameMs);
    lastStatusMs = millis();
  }
}
