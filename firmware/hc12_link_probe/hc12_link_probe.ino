// firmware/hc12_link_probe/hc12_link_probe.ino
//
// Safe standalone HC-12 wireless-link probe for the OpenRB-150 rover/breadboard.
//
// Purpose: validate the HC-12 link only. It sends PING frames (auto and/or on
// USB command), prints any received frames, replies PONG to received PINGs, and
// reports TX/RX counters, parse status, and a link-status summary over USB.
//
// SAFETY: this sketch initializes ONLY USB Serial and one HC-12 UART. It does
// NOT initialize or drive motors/ESC/Servo, GPS, I2C/IMU, RC/PPM, or any rover
// autonomy. It only ever emits PING/PONG frames and never emits motor-driving
// frames (no CMD,MANUAL / CMD,AUTO). USB input is restricted to PING payloads.
//
// Frame format matches gps_coverage_core/protocol.py and the main controller:
//   @TYPE,SEQ,PAYLOAD*CS\n   where CS = XOR of the ASCII bytes of "TYPE,SEQ,PAYLOAD".
//
// UART NOTE (OpenRB-150 exposes three hardware UARTs):
//   Serial1 = D26 TX / D27 RX (SERCOM2)
//   Serial2 = D28 TX / D29 RX (SERCOM4)  <-- GPS central connector lives here
//   Serial3 = D14 TX / D13 RX (SERCOM5)
// GPS is confirmed on Serial2. To validate HC-12 and GPS at the SAME time, the
// HC-12 must be on a DIFFERENT UART than GPS: wire HC-12 to Serial1 or Serial3
// and select it below. On Serial2 this probe also receives GPS NMEA bytes, which
// show up as parse errors and mean GPS+HC-12 cannot share that port.

#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef HC12_PROBE_SERIAL_PORT
#define HC12_PROBE_SERIAL_PORT 2  // 1=Serial1, 2=Serial2, 3=Serial3
#endif

#ifndef HC12_PROBE_BAUD
#define HC12_PROBE_BAUD 9600
#endif

#ifndef HC12_PROBE_AUTO_PING
#define HC12_PROBE_AUTO_PING 1
#endif

#ifndef HC12_PROBE_PING_INTERVAL_MS
#define HC12_PROBE_PING_INTERVAL_MS 1000
#endif

#if HC12_PROBE_SERIAL_PORT == 1
#define HC12_PORT Serial1
constexpr const char *HC12_PORT_NAME = "Serial1";
constexpr const char *HC12_PORT_PINS = "D26 TX / D27 RX";
constexpr bool HC12_PORT_SHARES_GPS = false;
#elif HC12_PROBE_SERIAL_PORT == 2
#define HC12_PORT Serial2
constexpr const char *HC12_PORT_NAME = "Serial2";
constexpr const char *HC12_PORT_PINS = "D28 TX / D29 RX (shared with GPS central connector)";
constexpr bool HC12_PORT_SHARES_GPS = true;
#elif HC12_PROBE_SERIAL_PORT == 3
#define HC12_PORT Serial3
constexpr const char *HC12_PORT_NAME = "Serial3";
constexpr const char *HC12_PORT_PINS = "D14 TX / D13 RX";
constexpr bool HC12_PORT_SHARES_GPS = false;
#else
#error "Unsupported HC12_PROBE_SERIAL_PORT (use 1, 2, or 3)"
#endif

constexpr long USB_BAUD = 115200;
constexpr long HC12_BAUD = HC12_PROBE_BAUD;
constexpr uint32_t REPORT_PERIOD_MS = 1000;
constexpr uint32_t LINK_OK_MAX_AGE_MS = 3000;
constexpr size_t LINE_MAX = 96;

uint32_t txCount = 0;
uint32_t rxCount = 0;
uint32_t pingTxCount = 0;
uint32_t pongRxCount = 0;
uint32_t parseOkCount = 0;
uint32_t parseErrCount = 0;
uint32_t txSeq = 0;
uint32_t lastRxMs = 0;
bool anyRxReceived = false;
uint32_t lastReportMs = 0;
uint32_t lastPingMs = 0;

char rxLine[LINE_MAX + 1] = {0};
size_t rxLineLen = 0;
char usbLine[LINE_MAX + 1] = {0};
size_t usbLineLen = 0;

char lastRxType[24] = "NA";
char lastRxPayload[48] = "NA";
long lastRxSeq = -1;

uint8_t checksumXor(const char *body) {
  uint8_t checksum = 0;
  for (const char *p = body; *p != '\0'; ++p) {
    checksum ^= static_cast<uint8_t>(*p);
  }
  return checksum;
}

// Replace frame-reserved characters so USB-supplied text cannot corrupt or
// inject extra frame fields. Keeps the probe to a single safe PING payload.
void sanitizePayload(char *text) {
  for (char *p = text; *p != '\0'; ++p) {
    if (*p == ',' || *p == '@' || *p == '*' || *p == '\r' || *p == '\n') {
      *p = '_';
    }
  }
}

void sendFrame(const char *type, uint32_t seq, const char *payload) {
  char body[80];
  snprintf(body, sizeof(body), "%s,%lu,%s", type, static_cast<unsigned long>(seq), payload);
  uint8_t cs = checksumXor(body);
  HC12_PORT.print('@');
  HC12_PORT.print(body);
  HC12_PORT.print('*');
  if (cs < 0x10) {
    HC12_PORT.print('0');
  }
  HC12_PORT.println(cs, HEX);
  txCount++;
}

void sendPing(const char *payload) {
  txSeq++;
  sendFrame("PING", txSeq, payload);
  pingTxCount++;
  Serial.print("hc12_tx type=PING seq=");
  Serial.print(txSeq);
  Serial.print(" payload=");
  Serial.println(payload);
}

bool parseFrame(const char *line, char *typeOut, size_t typeSize, long &seqOut,
                char *payloadOut, size_t payloadSize) {
  if (line[0] != '@') {
    return false;
  }
  const char *star = strrchr(line, '*');
  if (star == nullptr || star == line) {
    return false;
  }
  size_t bodyLen = static_cast<size_t>(star - (line + 1));
  char body[LINE_MAX + 1];
  if (bodyLen >= sizeof(body)) {
    return false;
  }
  memcpy(body, line + 1, bodyLen);
  body[bodyLen] = '\0';

  const char *csText = star + 1;
  char *endPtr = nullptr;
  long expected = strtol(csText, &endPtr, 16);
  if (endPtr == csText) {
    return false;
  }
  if (checksumXor(body) != static_cast<uint8_t>(expected)) {
    return false;
  }

  const char *firstComma = strchr(body, ',');
  if (firstComma == nullptr) {
    return false;
  }
  const char *secondComma = strchr(firstComma + 1, ',');
  if (secondComma == nullptr) {
    return false;
  }

  size_t typeLen = static_cast<size_t>(firstComma - body);
  if (typeLen >= typeSize) {
    typeLen = typeSize - 1;
  }
  memcpy(typeOut, body, typeLen);
  typeOut[typeLen] = '\0';

  char seqText[16];
  size_t seqLen = static_cast<size_t>(secondComma - (firstComma + 1));
  if (seqLen >= sizeof(seqText)) {
    seqLen = sizeof(seqText) - 1;
  }
  memcpy(seqText, firstComma + 1, seqLen);
  seqText[seqLen] = '\0';
  seqOut = atol(seqText);

  const char *payloadStart = secondComma + 1;
  size_t payloadLen = strlen(payloadStart);
  if (payloadLen >= payloadSize) {
    payloadLen = payloadSize - 1;
  }
  memcpy(payloadOut, payloadStart, payloadLen);
  payloadOut[payloadLen] = '\0';
  return true;
}

void handleRxLine(const char *line) {
  if (line[0] == '\0') {
    return;
  }
  rxCount++;
  lastRxMs = millis();
  anyRxReceived = true;

  char type[24] = {0};
  char payload[48] = {0};
  long seq = 0;
  if (parseFrame(line, type, sizeof(type), seq, payload, sizeof(payload))) {
    parseOkCount++;
    strncpy(lastRxType, type, sizeof(lastRxType) - 1);
    lastRxType[sizeof(lastRxType) - 1] = '\0';
    strncpy(lastRxPayload, payload, sizeof(lastRxPayload) - 1);
    lastRxPayload[sizeof(lastRxPayload) - 1] = '\0';
    lastRxSeq = seq;

    Serial.print("hc12_rx frame_ok=true type=");
    Serial.print(type);
    Serial.print(" seq=");
    Serial.print(seq);
    Serial.print(" payload=");
    Serial.println(payload);

    if (strcmp(type, "PING") == 0) {
      sendFrame("PONG", static_cast<uint32_t>(seq), payload);  // answer the link test
    } else if (strcmp(type, "PONG") == 0) {
      pongRxCount++;
    }
  } else {
    parseErrCount++;
    strncpy(lastRxType, "PARSE_ERR", sizeof(lastRxType) - 1);
    lastRxType[sizeof(lastRxType) - 1] = '\0';
    Serial.print("hc12_rx frame_ok=false raw=\"");
    Serial.print(line);
    Serial.println("\"");
  }
}

void handleUsbLine(char *line) {
  // USB input only ever produces a PING. Empty line or "ping" -> default probe
  // payload; otherwise the typed text becomes the (sanitized) PING payload.
  if (line[0] == '\0' || strcasecmp(line, "ping") == 0) {
    sendPing("PROBE");
    return;
  }
  if (line[0] == '@') {
    Serial.println("hc12_usb_refused reason=raw_frame_injection_not_allowed");
    return;
  }
  sanitizePayload(line);
  sendPing(line);
}

const char *linkStatus() {
  if (!anyRxReceived) {
    return "NO_RX_YET";
  }
  return (millis() - lastRxMs) <= LINK_OK_MAX_AGE_MS ? "LINK_OK" : "LINK_STALE";
}

void printReport() {
  Serial.print("hc12_link_probe_alive=true port=");
  Serial.print(HC12_PORT_NAME);
  Serial.print(" baud=");
  Serial.print(HC12_BAUD);
  Serial.print(" hc12_tx_count=");
  Serial.print(txCount);
  Serial.print(" hc12_rx_count=");
  Serial.print(rxCount);
  Serial.print(" ping_tx_count=");
  Serial.print(pingTxCount);
  Serial.print(" pong_rx_count=");
  Serial.print(pongRxCount);
  Serial.print(" hc12_parse_ok=");
  Serial.print(parseOkCount);
  Serial.print(" hc12_parse_error=");
  Serial.print(parseErrCount);
  Serial.print(" hc12_last_rx_age_ms=");
  if (anyRxReceived) {
    Serial.print(millis() - lastRxMs);
  } else {
    Serial.print("NA");
  }
  Serial.print(" hc12_last_cmd=");
  Serial.print(lastRxType);
  Serial.print(" last_rx_seq=");
  if (lastRxSeq >= 0) {
    Serial.print(lastRxSeq);
  } else {
    Serial.print("NA");
  }
  Serial.print(" last_rx_payload=\"");
  Serial.print(lastRxPayload);
  Serial.print("\" link_status=");
  Serial.println(linkStatus());
}

void readPort() {
  while (HC12_PORT.available() > 0) {
    char c = static_cast<char>(HC12_PORT.read());
    if (c == '\n') {
      rxLine[rxLineLen] = '\0';
      handleRxLine(rxLine);
      rxLineLen = 0;
    } else if (c != '\r') {
      if (rxLineLen < LINE_MAX) {
        rxLine[rxLineLen++] = c;
      } else {
        rxLineLen = 0;  // overflow: drop the malformed line
      }
    }
  }
}

void readUsb() {
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());
    if (c == '\n') {
      usbLine[usbLineLen] = '\0';
      handleUsbLine(usbLine);
      usbLineLen = 0;
    } else if (c != '\r') {
      if (usbLineLen < LINE_MAX) {
        usbLine[usbLineLen++] = c;
      } else {
        usbLineLen = 0;
      }
    }
  }
}

void setup() {
  Serial.begin(USB_BAUD);
  uint32_t start = millis();
  while (!Serial && millis() - start < 1500) {
    delay(10);
  }

  HC12_PORT.begin(HC12_BAUD);

  Serial.println();
  Serial.println("Firmware: hc12_link_probe link-validation 2026-05-30");
  Serial.println("Safe HC-12 probe. No motors, GPS, IMU, RC, or autonomy are initialized.");
  Serial.print("hc12_port=");
  Serial.print(HC12_PORT_NAME);
  Serial.print(" pins=\"");
  Serial.print(HC12_PORT_PINS);
  Serial.print("\" baud=");
  Serial.println(HC12_BAUD);
  Serial.print("auto_ping=");
  Serial.print(HC12_PROBE_AUTO_PING ? "true" : "false");
  Serial.print(" ping_interval_ms=");
  Serial.println(HC12_PROBE_PING_INTERVAL_MS);
  Serial.println("USB input sends PING only (type text -> PING payload; 'ping' -> PROBE).");
  if (HC12_PORT_SHARES_GPS) {
    Serial.println(
        "WARNING: Serial2 is the GPS port. GPS NMEA bytes will appear here as "
        "parse errors. For GPS+HC-12 together, recompile with "
        "-DHC12_PROBE_SERIAL_PORT=3 (or 1) and wire HC-12 to that UART.");
  }
}

void loop() {
  readPort();
  readUsb();

  uint32_t now = millis();
#if HC12_PROBE_AUTO_PING
  if (now - lastPingMs >= HC12_PROBE_PING_INTERVAL_MS) {
    sendPing("PROBE");
    lastPingMs = now;
  }
#endif

  if (now - lastReportMs >= REPORT_PERIOD_MS) {
    printReport();
    lastReportMs = now;
  }
}
