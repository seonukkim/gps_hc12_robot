// firmware/gps_passive_uart_baud_sweep_probe/gps_passive_uart_baud_sweep_probe.ino
//
// Passive (READ-ONLY) GPS UART + baud sweep for OpenRB-150. A no-rewire diagnostic.
//
// Why: the GPS is on the center 5-pin connector (Serial2), and the probe now
// correctly selects Serial2, but a Serial2 @ 9600 run showed only a couple of
// stray bytes and no real NMEA. Rewiring is costly right now, so this probe
// passively listens on every hardware UART at every common GPS baud and reports
// where (if anywhere) real NMEA-like data appears.
//
// SAFETY / READ-ONLY: USB Serial is used for logs only. This sketch NEVER writes
// to Serial1/Serial2/Serial3 (only begin / read / end). It does not initialize or
// drive motors / ESC / Servo, RC / PPM, HC-12, or I2C, and runs no autonomy. It
// changes no motor / path-following behavior and enables no motion.
//
// Candidates: Serial1 (D26/D27), Serial2 (D28/D29 = center 5-pin / GPS),
//             Serial3 (D14/D13)  x  bauds {4800, 9600, 38400, 57600, 115200}.
// Real NMEA = '$' + a 2-letter GNSS talker (GP/GN/GL/GA/GB/GQ) + a known sentence
// type (RMC/GGA/GSV/GSA/VTG/GLL/TXT/ZDA). A few isolated bytes do NOT count.

#include <Arduino.h>
#include <string.h>

constexpr long USB_BAUD = 115200;
#ifndef GPS_SWEEP_LISTEN_MS
#define GPS_SWEEP_LISTEN_MS 6000
#endif
constexpr uint32_t LISTEN_MS = GPS_SWEEP_LISTEN_MS;
constexpr size_t PREVIEW_MAX = 48;
constexpr size_t LINE_MAX = 120;
constexpr uint32_t MIN_BYTES_FOR_UART = 20;  // below this is noise, not a stream

struct PortRef {
  HardwareSerial *uart;
  const char *name;
  const char *pins;
};

PortRef ports[] = {
    {&Serial1, "Serial1", "D26 TX / D27 RX (PA12/PA13, SD-SPI)"},
    {&Serial2, "Serial2", "D28 TX / D29 RX (PA14/PA15, center 5-pin / GPS)"},
    {&Serial3, "Serial3", "D14 TX / D13 RX (PB22/PB23, side exp)"},
};
constexpr uint8_t PORT_COUNT = sizeof(ports) / sizeof(ports[0]);

const long bauds[] = {4800, 9600, 38400, 57600, 115200};
constexpr uint8_t BAUD_COUNT = sizeof(bauds) / sizeof(bauds[0]);

const char *bestPort = "none";
long bestBaud = 0;
uint32_t bestChars = 0;
uint32_t bestNmea = 0;
bool bestIsNmea = false;
uint32_t sweepPass = 0;

bool isUpperAZ(char c) { return c >= 'A' && c <= 'Z'; }

// s points at the two characters after '$'. Accept standard GNSS talkers only.
bool nmeaTalker(const char *s) {
  if (!isUpperAZ(s[0]) || !isUpperAZ(s[1])) {
    return false;
  }
  return s[0] == 'G' &&
         (s[1] == 'P' || s[1] == 'N' || s[1] == 'L' || s[1] == 'A' ||
          s[1] == 'B' || s[1] == 'Q');
}

bool isNmeaLine(const char *line) {
  if (line[0] != '$') {
    return false;
  }
  if (!nmeaTalker(line + 1)) {
    return false;
  }
  return strstr(line, "RMC") || strstr(line, "GGA") || strstr(line, "GSV") ||
         strstr(line, "GSA") || strstr(line, "VTG") || strstr(line, "GLL") ||
         strstr(line, "TXT") || strstr(line, "ZDA");
}

void listenCandidate(PortRef &p, long baud) {
  uint32_t chars = 0;
  uint32_t nmeaCount = 0;
  bool rmc = false;
  bool gga = false;
  char preview[PREVIEW_MAX + 1];
  size_t prevLen = 0;
  preview[0] = '\0';
  char line[LINE_MAX + 1];
  size_t lineLen = 0;

  p.uart->begin(baud);
  delay(60);
  while (p.uart->available() > 0) {  // drop stale / baud-transition bytes
    p.uart->read();
  }

  uint32_t start = millis();
  while (millis() - start < LISTEN_MS) {
    while (p.uart->available() > 0) {
      int b = p.uart->read();
      if (b < 0) {
        break;
      }
      chars++;
      char c = static_cast<char>(b);
      if (prevLen < PREVIEW_MAX) {
        preview[prevLen++] = (c >= 32 && c < 127) ? c : '.';
        preview[prevLen] = '\0';
      }
      if (c == '\n' || c == '\r') {
        if (lineLen > 0) {
          line[lineLen] = '\0';
          if (isNmeaLine(line)) {
            nmeaCount++;
            if (strstr(line, "RMC")) rmc = true;
            if (strstr(line, "GGA")) gga = true;
          }
          lineLen = 0;
        }
      } else if (lineLen < LINE_MAX) {
        line[lineLen++] = c;
      } else {
        lineLen = 0;  // overflow: drop the malformed line
      }
    }
  }
  p.uart->end();

  const char *verdict;
  if (nmeaCount >= 1) {
    verdict = "NMEA";
  } else if (chars >= MIN_BYTES_FOR_UART) {
    verdict = "BYTES_NO_NMEA";
  } else if (chars > 0) {
    verdict = "FEW_BYTES";
  } else {
    verdict = "NO_BYTES";
  }

  Serial.print("candidate_port=");
  Serial.print(p.name);
  Serial.print(" candidate_baud=");
  Serial.print(baud);
  Serial.print(" chars_seen=");
  Serial.print(chars);
  Serial.print(" nmea_like_seen=");
  Serial.print(nmeaCount);
  Serial.print(" rmc_seen=");
  Serial.print(rmc ? "true" : "false");
  Serial.print(" gga_seen=");
  Serial.print(gga ? "true" : "false");
  Serial.print(" verdict=");
  Serial.print(verdict);
  Serial.print(" raw_preview=\"");
  Serial.print(preview);
  Serial.println("\"");

  // Best tracking: NMEA candidates always beat non-NMEA; then most chars wins.
  bool better = false;
  if (nmeaCount > 0) {
    if (!bestIsNmea || nmeaCount > bestNmea ||
        (nmeaCount == bestNmea && chars > bestChars)) {
      better = true;
    }
  } else if (!bestIsNmea && chars > bestChars) {
    better = true;
  }
  if (better) {
    bestIsNmea = nmeaCount > 0;
    bestPort = p.name;
    bestBaud = baud;
    bestChars = chars;
    bestNmea = nmeaCount;
  }
}

void runSweep() {
  sweepPass++;
  bestPort = "none";
  bestBaud = 0;
  bestChars = 0;
  bestNmea = 0;
  bestIsNmea = false;

  Serial.println();
  Serial.print("SWEEP_START sweep_pass=");
  Serial.print(sweepPass);
  Serial.print(" listen_ms=");
  Serial.println(LISTEN_MS);

  for (uint8_t i = 0; i < PORT_COUNT; ++i) {
    for (uint8_t j = 0; j < BAUD_COUNT; ++j) {
      listenCandidate(ports[i], bauds[j]);
    }
  }

  Serial.print("SWEEP_DONE sweep_pass=");
  Serial.print(sweepPass);
  Serial.print(" best_port=");
  Serial.print(bestPort);
  Serial.print(" best_baud=");
  Serial.print(bestBaud);
  Serial.print(" best_chars=");
  Serial.print(bestChars);
  Serial.print(" best_nmea_like_seen=");
  Serial.print(bestNmea);
  Serial.print(" best_is_nmea=");
  Serial.println(bestIsNmea ? "true" : "false");
}

void setup() {
  Serial.begin(USB_BAUD);
  uint32_t start = millis();
  while (!Serial && millis() - start < 8000) {
    delay(10);
  }

  Serial.println();
  Serial.println("Firmware: gps_passive_uart_baud_sweep_probe read-only-no-rewire 2026-06-03");
  Serial.println(
      "READ-ONLY: USB logs only. Never writes Serial1/2/3. No motors, RC, HC-12, "
      "I2C, or autonomy. No motion.");
  Serial.print("ports=");
  for (uint8_t i = 0; i < PORT_COUNT; ++i) {
    Serial.print(ports[i].name);
    if (i + 1 < PORT_COUNT) Serial.print(",");
  }
  Serial.print(" bauds=");
  for (uint8_t j = 0; j < BAUD_COUNT; ++j) {
    Serial.print(bauds[j]);
    if (j + 1 < BAUD_COUNT) Serial.print(",");
  }
  Serial.print(" listen_ms_each=");
  Serial.println(LISTEN_MS);
  Serial.println(
      "Real NMEA = '$' + talker (GP/GN/GL/GA/...) + RMC/GGA/GSV/GSA/VTG/GLL. A few "
      "isolated bytes do NOT count.");
}

void loop() {
  runSweep();
  delay(3000);
}
