#include <Arduino.h>
#include <TinyGPS++.h>
#include <stdlib.h>
#include <string.h>

// Safe GPS UART probe for OpenRB-150.
//
// This sketch does not include Servo, does not attach ESC pins, and does not
// initialize motor outputs. It only opens one GPS candidate input and reports
// raw byte activity, NMEA fix status, and GPS stability over USB Serial.
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
constexpr uint32_t FIX_STABLE_SECONDS = 30;
constexpr uint32_t FIX_MAX_AGE_MS = 2000;
constexpr uint8_t FIX_MIN_SATS = 4;
constexpr double FIX_MAX_HDOP = 5.0;
constexpr size_t NMEA_LINE_MAX = 128;
constexpr size_t SENTENCE_PREVIEW_MAX = 88;

TinyGPSPlus gps;

uint32_t lastReportMs = 0;
uint32_t charsThisSecond = 0;
uint32_t totalChars = 0;
uint32_t validFixSecondsConsecutive = 0;
uint32_t noFixSecondsConsecutive = 0;
uint32_t validFixSecondsTotal = 0;
uint32_t noFixSecondsTotal = 0;

char nmeaLine[NMEA_LINE_MAX + 1] = {0};
size_t nmeaLineLen = 0;
char lastRmcStatus = '\0';
int lastGgaFixQuality = -1;
char lastRmcPreview[SENTENCE_PREVIEW_MAX + 1] = "NA";
char lastGgaPreview[SENTENCE_PREVIEW_MAX + 1] = "NA";

bool nmeaSentenceType(const char *sentence, const char *type) {
  return sentence[0] == '$' &&
         ((sentence[1] == 'G' && sentence[2] == 'P') ||
          (sentence[1] == 'G' && sentence[2] == 'N')) &&
         sentence[3] == type[0] && sentence[4] == type[1] &&
         sentence[5] == type[2];
}

bool nmeaField(const char *sentence, uint8_t index, char *out, size_t outSize) {
  if (outSize == 0) {
    return false;
  }

  uint8_t current = 0;
  const char *start = sentence;
  while (*start != '\0') {
    const char *end = start;
    while (*end != '\0' && *end != ',' && *end != '*') {
      ++end;
    }

    if (current == index) {
      size_t len = static_cast<size_t>(end - start);
      if (len >= outSize) {
        len = outSize - 1;
      }
      memcpy(out, start, len);
      out[len] = '\0';
      return true;
    }

    if (*end == '\0' || *end == '*') {
      break;
    }
    start = end + 1;
    ++current;
  }

  out[0] = '\0';
  return false;
}

void copySentencePreview(const char *sentence, char *preview, size_t previewSize) {
  if (previewSize == 0) {
    return;
  }

  size_t len = strlen(sentence);
  bool truncated = len >= previewSize;
  if (truncated) {
    len = previewSize - 1;
  }
  memcpy(preview, sentence, len);
  preview[len] = '\0';

  if (truncated && previewSize > 4) {
    preview[previewSize - 4] = '.';
    preview[previewSize - 3] = '.';
    preview[previewSize - 2] = '.';
    preview[previewSize - 1] = '\0';
  }
}

void processNmeaLine(const char *sentence) {
  char field[16] = {0};
  if (nmeaSentenceType(sentence, "RMC")) {
    copySentencePreview(sentence, lastRmcPreview, sizeof(lastRmcPreview));
    if (nmeaField(sentence, 2, field, sizeof(field)) && field[0] != '\0') {
      lastRmcStatus = field[0];
    }
  } else if (nmeaSentenceType(sentence, "GGA")) {
    copySentencePreview(sentence, lastGgaPreview, sizeof(lastGgaPreview));
    if (nmeaField(sentence, 6, field, sizeof(field)) && field[0] != '\0') {
      lastGgaFixQuality = atoi(field);
    }
  }
}

void processGpsChar(char c) {
  gps.encode(c);

  if (c == '\n') {
    if (nmeaLineLen > 0) {
      nmeaLine[nmeaLineLen] = '\0';
      processNmeaLine(nmeaLine);
      nmeaLineLen = 0;
      nmeaLine[0] = '\0';
    }
    return;
  }

  if (c == '\r') {
    return;
  }

  if (nmeaLineLen < NMEA_LINE_MAX) {
    nmeaLine[nmeaLineLen++] = c;
    nmeaLine[nmeaLineLen] = '\0';
  } else {
    nmeaLineLen = 0;
    nmeaLine[0] = '\0';
  }
}

bool rmcOrGgaFixStatusOk() {
  return lastRmcStatus == 'A' || lastGgaFixQuality >= 1;
}

bool currentValidFix() {
  return rmcOrGgaFixStatusOk() &&
         gps.location.isValid() &&
         gps.location.age() <= FIX_MAX_AGE_MS &&
         gps.satellites.isValid() &&
         gps.satellites.value() >= FIX_MIN_SATS &&
         gps.hdop.isValid() &&
         gps.hdop.hdop() <= FIX_MAX_HDOP;
}

void updateStabilityCounters(bool validCurrentFix) {
  if (validCurrentFix) {
    ++validFixSecondsConsecutive;
    ++validFixSecondsTotal;
    noFixSecondsConsecutive = 0;
  } else {
    ++noFixSecondsConsecutive;
    ++noFixSecondsTotal;
    validFixSecondsConsecutive = 0;
  }
}

const char *gpsProbeState(bool validCurrentFix) {
  if (!validCurrentFix) {
    return "NO_FIX";
  }
  if (validFixSecondsConsecutive >= FIX_STABLE_SECONDS) {
    return "STABLE_FIX";
  }
  return "INTERMITTENT_FIX";
}

// Single-word reason the current fix is not usable, using the same vocabulary as
// the main controller's gps_block_reason. Indoor/no-fix states are normal here;
// the probe keeps printing raw status regardless.
const char *gpsBlockReason(bool validCurrentFix) {
  if (totalChars == 0) {
    return "NO_BYTES";
  }
  if (!rmcOrGgaFixStatusOk()) {
    return "NO_NMEA_FIX";
  }
  if (!gps.location.isValid()) {
    return "NO_LOCATION";
  }
  if (gps.location.age() > FIX_MAX_AGE_MS) {
    return "STALE_LOCATION";
  }
  if (!gps.satellites.isValid() || gps.satellites.value() < FIX_MIN_SATS) {
    return "LOW_SATS";
  }
  if (!gps.hdop.isValid() || gps.hdop.hdop() > FIX_MAX_HDOP) {
    return "HIGH_HDOP";
  }
  return validCurrentFix ? "OK" : "NO_FIX";
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
  Serial.println("Decision rule: chars_1s=0 => wiring/port/baud issue; chars_1s>0 but gps_probe_state=NO_FIX => GPS bytes arrive but no current fix.");
  Serial.println("Stable fix rule: RMC A or GGA quality >=1, fresh lat/lon age <=2000 ms, sats >=4, hdop <=5.0, for 30 consecutive seconds.");
}

void printReport() {
  bool validCurrentFix = currentValidFix();
  updateStabilityCounters(validCurrentFix);

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
  Serial.print(" tinygps_chars=");
  Serial.print(gps.charsProcessed());
  Serial.print(" last_rmc_status=");
  if (lastRmcStatus != '\0') {
    Serial.print(lastRmcStatus);
  } else {
    Serial.print("NA");
  }
  Serial.print(" last_gga_fix_quality=");
  if (lastGgaFixQuality >= 0) {
    Serial.print(lastGgaFixQuality);
  } else {
    Serial.print("NA");
  }
  Serial.print(" current_valid_fix=");
  Serial.print(validCurrentFix ? "true" : "false");
  Serial.print(" gps_probe_state=");
  Serial.print(gpsProbeState(validCurrentFix));
  Serial.print(" gps_block_reason=");
  Serial.print(gpsBlockReason(validCurrentFix));
  Serial.print(" lat=");
  if (validCurrentFix) {
    Serial.print(gps.location.lat(), 7);
  } else {
    Serial.print("NA");
  }
  Serial.print(" lon=");
  if (validCurrentFix) {
    Serial.print(gps.location.lng(), 7);
  } else {
    Serial.print("NA");
  }
  Serial.print(" age_ms=");
  if (gps.location.isValid()) {
    Serial.print(gps.location.age());
  } else {
    Serial.print("NA");
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
  Serial.print(" valid_fix_seconds_consecutive=");
  Serial.print(validFixSecondsConsecutive);
  Serial.print(" no_fix_seconds_consecutive=");
  Serial.print(noFixSecondsConsecutive);
  Serial.print(" valid_fix_seconds_total=");
  Serial.print(validFixSecondsTotal);
  Serial.print(" no_fix_seconds_total=");
  Serial.print(noFixSecondsTotal);
  Serial.print(" rmc_preview=\"");
  Serial.print(lastRmcPreview);
  Serial.print("\" gga_preview=\"");
  Serial.print(lastGgaPreview);
  Serial.print("\"");

  if (lastRmcStatus == 'V' && gps.location.isValid()) {
    Serial.print(" warning=\"TinyGPS cached fix is not stable current fix\"");
    Serial.print(" cached_lat=");
    Serial.print(gps.location.lat(), 7);
    Serial.print(" cached_lon=");
    Serial.print(gps.location.lng(), 7);
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
    processGpsChar(c);
    charsThisSecond++;
    totalChars++;
  }

  uint32_t now = millis();
  if (now - lastReportMs >= REPORT_PERIOD_MS) {
    lastReportMs = now;
    printReport();
    charsThisSecond = 0;
  }
}
