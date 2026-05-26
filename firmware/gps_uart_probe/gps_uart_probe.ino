#include <Arduino.h>
#include <TinyGPS++.h>

// Safe GPS UART probe for OpenRB-150.
//
// This sketch does not include Servo, does not attach ESC pins, and does not
// initialize motor outputs. It only opens one GPS candidate input and reports
// raw byte activity plus TinyGPS++ parser status over USB Serial.
//
// Compile-time options:
//   GPS_PROBE_MODE=3  -> Serial3, expected OpenRB RX D13
//   GPS_PROBE_MODE=2  -> Serial2, board UART candidate
//   GPS_PROBE_MODE=89 -> SoftwareSerial RX D8 / TX D9 candidate
//   GPS_PROBE_BAUD    -> GPS candidate baudrate

#define GPS_PROBE_MODE_SERIAL2 2
#define GPS_PROBE_MODE_SERIAL3 3
#define GPS_PROBE_MODE_SOFTWARE_SERIAL_8_9 89

#ifndef GPS_PROBE_MODE
#define GPS_PROBE_MODE GPS_PROBE_MODE_SERIAL3
#endif

#ifndef GPS_PROBE_BAUD
#define GPS_PROBE_BAUD 9600
#endif

#if GPS_PROBE_MODE == GPS_PROBE_MODE_SERIAL3
#define GPS_PORT Serial3
constexpr const char *PROBE_PORT_NAME = "Serial3";
constexpr const char *PROBE_PIN_ASSUMPTION = "OpenRB Serial3 RX D13, TX D14";
#elif GPS_PROBE_MODE == GPS_PROBE_MODE_SERIAL2
#define GPS_PORT Serial2
constexpr const char *PROBE_PORT_NAME = "Serial2";
constexpr const char *PROBE_PIN_ASSUMPTION = "OpenRB Serial2 board UART pins";
#elif GPS_PROBE_MODE == GPS_PROBE_MODE_SOFTWARE_SERIAL_8_9
#if __has_include(<SoftwareSerial.h>)
#include <SoftwareSerial.h>
SoftwareSerial gpsSoftwareSerial(8, 9);  // RX, TX
#define GPS_PORT gpsSoftwareSerial
constexpr const char *PROBE_PORT_NAME = "SoftwareSerial";
constexpr const char *PROBE_PIN_ASSUMPTION = "SoftwareSerial RX D8, TX D9";
#else
#error "SoftwareSerial.h is not available for this OpenRB build; do not use the D8/D9 SoftwareSerial probe on this environment."
#endif
#else
#error "Unsupported GPS_PROBE_MODE"
#endif

constexpr long USB_BAUD = 115200;
constexpr long GPS_BAUD = GPS_PROBE_BAUD;
constexpr uint32_t REPORT_PERIOD_MS = 1000;
constexpr size_t RAW_PREVIEW_MAX = 96;

TinyGPSPlus gps;

uint32_t lastReportMs = 0;
uint32_t charsThisSecond = 0;
uint32_t totalChars = 0;
char rawPreview[RAW_PREVIEW_MAX + 1] = {0};
size_t rawPreviewLen = 0;

void appendPreviewChar(char c) {
  if (rawPreviewLen >= RAW_PREVIEW_MAX) {
    return;
  }

  const uint8_t byteValue = static_cast<uint8_t>(c);
  const char *escaped = nullptr;
  char hexText[5] = {0};
  switch (c) {
    case '\r':
      escaped = "\\r";
      break;
    case '\n':
      escaped = "\\n";
      break;
    case '\t':
      escaped = "\\t";
      break;
    default:
      if (byteValue >= 32 && byteValue <= 126) {
        rawPreview[rawPreviewLen++] = c;
        rawPreview[rawPreviewLen] = '\0';
        return;
      }
      snprintf(hexText, sizeof(hexText), "\\x%02X", byteValue);
      escaped = hexText;
      break;
  }

  for (size_t i = 0; escaped[i] != '\0' && rawPreviewLen < RAW_PREVIEW_MAX; ++i) {
    rawPreview[rawPreviewLen++] = escaped[i];
  }
  rawPreview[rawPreviewLen] = '\0';
}

void resetPreview() {
  rawPreviewLen = 0;
  rawPreview[0] = '\0';
}

void printProbeHeader() {
  Serial.println();
  Serial.println("GPS UART probe starting.");
  Serial.println("No motor pins are attached or driven by this sketch.");
  Serial.print("selected_port=");
  Serial.print(PROBE_PORT_NAME);
  Serial.print(" baud=");
  Serial.print(GPS_BAUD);
  Serial.print(" pin_assumption=\"");
  Serial.print(PROBE_PIN_ASSUMPTION);
  Serial.println("\"");
  Serial.println("Decision rule: chars_1s=0 => wiring/port/baud issue; chars_1s>0 and fix=false => GPS bytes arrive but no fix yet.");
}

void printReport() {
  Serial.print("selected_port=");
  Serial.print(PROBE_PORT_NAME);
  Serial.print(" baud=");
  Serial.print(GPS_BAUD);
  Serial.print(" pin_assumption=\"");
  Serial.print(PROBE_PIN_ASSUMPTION);
  Serial.print("\" chars_1s=");
  Serial.print(charsThisSecond);
  Serial.print(" total_chars=");
  Serial.print(totalChars);
  Serial.print(" raw_preview=\"");
  Serial.print(rawPreview);
  Serial.print("\" tinygps_chars=");
  Serial.print(gps.charsProcessed());
  Serial.print(" fix=");
  Serial.print(gps.location.isValid() ? "true" : "false");

  if (gps.location.isValid()) {
    Serial.print(" lat=");
    Serial.print(gps.location.lat(), 7);
    Serial.print(" lon=");
    Serial.print(gps.location.lng(), 7);
    Serial.print(" age_ms=");
    Serial.print(gps.location.age());
  } else {
    Serial.print(" lat=NA lon=NA age_ms=NA");
  }

  Serial.print(" sats=");
  if (gps.satellites.isValid()) {
    Serial.print(gps.satellites.value());
  } else {
    Serial.print("NA");
  }

  Serial.print(" hdop=");
  if (gps.hdop.isValid()) {
    Serial.print(gps.hdop.hdop(), 2);
  } else {
    Serial.print("NA");
  }

  Serial.println();
}

void setup() {
  Serial.begin(USB_BAUD);

  uint32_t start = millis();
  while (!Serial && millis() - start < 1200) {
    delay(10);
  }

  GPS_PORT.begin(GPS_BAUD);
  printProbeHeader();
}

void loop() {
  while (GPS_PORT.available() > 0) {
    char c = static_cast<char>(GPS_PORT.read());
    gps.encode(c);
    appendPreviewChar(c);
    charsThisSecond++;
    totalChars++;
  }

  uint32_t now = millis();
  if (now - lastReportMs >= REPORT_PERIOD_MS) {
    lastReportMs = now;
    printReport();
    charsThisSecond = 0;
    resetPreview();
  }
}
