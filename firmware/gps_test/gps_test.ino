#include <Arduino.h>
#include <TinyGPS++.h>

// GPS-only test for OpenRB-150 using D13/D14.
//
// OpenRB:
//   D14 = Serial3 TX
//   D13 = Serial3 RX
//
// GPS module:
//   PPS | RX | TX | GND | UCC
//
// Wiring for this test:
//   GPS UCC/VCC -> OpenRB +5V
//   GPS GND     -> OpenRB GND
//   GPS TX      -> OpenRB D13/RX
//   GPS RX      -> not connected for this read-only test
//   GPS PPS     -> not connected
//
// This is GPS-only. HC-12 cannot use D13/D14 during this test.

#define GPS_SERIAL Serial3

constexpr long GPS_BAUD = 9600;
constexpr long USB_BAUD = 115200;
constexpr uint32_t REPORT_PERIOD_MS = 1000;

TinyGPSPlus gps;

uint32_t lastReportMs = 0;
uint32_t charsThisSecond = 0;
uint32_t totalChars = 0;

void setup() {
  Serial.begin(USB_BAUD);

  uint32_t start = millis();
  while (!Serial && millis() - start < 1200) {
    delay(10);
  }

  GPS_SERIAL.begin(GPS_BAUD);

  Serial.println();
  Serial.println("GPS Serial3 D13 test ready.");
  Serial.println("Wiring: GPS TX -> OpenRB D13/RX, GPS GND -> GND, GPS UCC/VCC -> +5V.");
  Serial.println("GPS RX and PPS are not connected.");
}

void loop() {
  while (GPS_SERIAL.available() > 0) {
    char c = static_cast<char>(GPS_SERIAL.read());
    gps.encode(c);
    charsThisSecond++;
    totalChars++;
  }

  uint32_t now = millis();
  if (now - lastReportMs >= REPORT_PERIOD_MS) {
    lastReportMs = now;

    Serial.print("chars_1s=");
    Serial.print(charsThisSecond);

    Serial.print(" total_chars=");
    Serial.print(totalChars);

    Serial.print(" tinygps_chars=");
    Serial.print(gps.charsProcessed());

    Serial.print(" status=");
    if (gps.location.isValid()) {
      Serial.print("FIX lat=");
      Serial.print(gps.location.lat(), 7);
      Serial.print(" lon=");
      Serial.print(gps.location.lng(), 7);
    } else {
      Serial.print("NO_FIX");
    }

    Serial.print(" sats=");
    if (gps.satellites.isValid()) Serial.print(gps.satellites.value());
    else Serial.print("NA");

    Serial.print(" hdop=");
    if (gps.hdop.isValid()) Serial.print(gps.hdop.hdop(), 2);
    else Serial.print("NA");

    Serial.print(" age_ms=");
    if (gps.location.isValid()) Serial.print(gps.location.age());
    else Serial.print("NA");

    Serial.println();

    charsThisSecond = 0;
  }
}
