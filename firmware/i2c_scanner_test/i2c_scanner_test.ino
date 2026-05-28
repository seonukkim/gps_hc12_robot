#include <Wire.h>

constexpr long USB_BAUD = 115200;
constexpr uint8_t I2C_ADDR_MIN = 0x03;
constexpr uint8_t I2C_ADDR_MAX = 0x77;
constexpr uint32_t SCAN_PERIOD_MS = 3000;

uint32_t lastScanMs = 0;

void printHexAddress(uint8_t address) {
  Serial.print("0x");
  if (address < 0x10) {
    Serial.print("0");
  }
  Serial.print(address, HEX);
}

void scanI2cBus() {
  uint8_t foundCount = 0;

  Serial.println();
  Serial.println("I2C scanner test");
  Serial.println("No motors or Servo outputs are used by this sketch.");
  Serial.println("Scanning addresses 0x03 through 0x77...");
  Serial.println(
      "Note: common IMU/sensor addresses may include 0x68, 0x69, 0x76, etc.; "
      "do not assume device type from address alone.");

  for (uint8_t address = I2C_ADDR_MIN; address <= I2C_ADDR_MAX; ++address) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("found address=");
      printHexAddress(address);
      Serial.println();
      foundCount++;
    } else if (error == 4) {
      Serial.print("unknown error at address=");
      printHexAddress(address);
      Serial.println();
    }
  }

  if (foundCount == 0) {
    Serial.println("No I2C devices found.");
  } else {
    Serial.print("I2C devices found=");
    Serial.println(foundCount);
  }
}

void setup() {
  Serial.begin(USB_BAUD);
  while (!Serial && millis() < 3000) {
    delay(10);
  }

  Wire.begin();
  Serial.println("OpenRB I2C scanner starting.");
  scanI2cBus();
  lastScanMs = millis();
}

void loop() {
  uint32_t now = millis();
  if (now - lastScanMs >= SCAN_PERIOD_MS) {
    scanI2cBus();
    lastScanMs = now;
  }
}
