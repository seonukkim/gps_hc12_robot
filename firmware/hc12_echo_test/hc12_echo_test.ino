#include <SoftwareSerial.h>

// Confirm the actual OpenRB-150 UART-capable pins before use.
constexpr uint8_t HC12_RX_PIN = 10;
constexpr uint8_t HC12_TX_PIN = 11;
constexpr long HC12_BAUD = 9600;

SoftwareSerial hc12Serial(HC12_RX_PIN, HC12_TX_PIN);

void setup() {
  Serial.begin(115200);
  hc12Serial.begin(HC12_BAUD);
  Serial.println("HC-12 echo test ready.");
  Serial.println("Confirm actual OpenRB-150 UART pin mapping before field use.");
}

void loop() {
  while (Serial.available() > 0) {
    hc12Serial.write(Serial.read());
  }

  while (hc12Serial.available() > 0) {
    Serial.write(hc12Serial.read());
  }
}
