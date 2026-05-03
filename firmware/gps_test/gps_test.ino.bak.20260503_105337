#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// Confirm the actual OpenRB-150 UART pin mapping before use.
constexpr uint8_t GPS_RX_PIN = 8;
constexpr uint8_t GPS_TX_PIN = 9;
constexpr long GPS_BAUD = 9600;

SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPSPlus gps;

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(GPS_BAUD);
  Serial.println("GPS test ready.");
  Serial.println("Confirm actual OpenRB-150 GPS UART pins before deployment.");
}

void loop() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint < 1000) {
    return;
  }
  lastPrint = millis();

  if (!gps.location.isValid()) {
    Serial.println("Waiting for GPS fix...");
    return;
  }

  Serial.print("lat=");
  Serial.print(gps.location.lat(), 6);
  Serial.print(" lon=");
  Serial.print(gps.location.lng(), 6);
  Serial.print(" sats=");
  Serial.print(gps.satellites.value());
  Serial.print(" hdop=");
  Serial.println(gps.hdop.hdop());
}
