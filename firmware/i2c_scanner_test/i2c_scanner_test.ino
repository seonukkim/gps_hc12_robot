#include <Wire.h>

constexpr long USB_BAUD = 115200;
constexpr uint8_t DEFAULT_WIRE_SDA_PIN = 11;
constexpr uint8_t DEFAULT_WIRE_SCL_PIN = 12;
constexpr uint8_t I2C_ADDR_MIN = 0x03;
constexpr uint8_t I2C_ADDR_MAX = 0x77;
constexpr uint32_t SERIAL_WAIT_TIMEOUT_MS = 3000;
constexpr uint32_t SCAN_PERIOD_MS = 2000;
constexpr uint8_t MAX_VALID_FOUND_COUNT = 8;
constexpr uint8_t STABLE_REQUIRED_PASSES = 3;

uint32_t lastScanMs = 0;
uint32_t scanPass = 0;
uint8_t lastSingleAddress = 0;
uint8_t stablePassCount = 0;
uint8_t stableAddress = 0;
bool hasStableAddress = false;

void printHexAddress(uint8_t address) {
  Serial.print("0x");
  if (address < 0x10) {
    Serial.print("0");
  }
  Serial.print(address, HEX);
}

const char *levelName(uint8_t level) {
  return level == HIGH ? "HIGH" : "LOW";
}

void configurePinsAsInputs() {
  pinMode(DEFAULT_WIRE_SDA_PIN, INPUT_PULLUP);
  pinMode(DEFAULT_WIRE_SCL_PIN, INPUT_PULLUP);
  delay(2);
}

void printPinStates(const char *prefix) {
  uint8_t sdaState = digitalRead(DEFAULT_WIRE_SDA_PIN);
  uint8_t sclState = digitalRead(DEFAULT_WIRE_SCL_PIN);
  Serial.print(prefix);
  Serial.print("_sda=");
  Serial.print(levelName(sdaState));
  Serial.print(" ");
  Serial.print(prefix);
  Serial.print("_scl=");
  Serial.println(levelName(sclState));
}

bool readBusReleasedHigh() {
  uint8_t sdaState = digitalRead(DEFAULT_WIRE_SDA_PIN);
  uint8_t sclState = digitalRead(DEFAULT_WIRE_SCL_PIN);
  return sdaState == HIGH && sclState == HIGH;
}

void printStartup() {
  Serial.println();
  Serial.println("OpenRB default Wire I2C scanner");
  Serial.println("Expected SDA=D11/PA08");
  Serial.println("Expected SCL=D12/PA09");
  Serial.print("USB baud=");
  Serial.println(USB_BAUD);
  Serial.println("No motors or Servo outputs are used by this sketch.");
  Serial.println(
      "A valid IMU result requires one stable address, not all addresses.");
  Serial.println(
      "Common sensor addresses may include 0x68, 0x69, or 0x76, but address "
      "alone does not identify the device.");
}

void updateStableAddress(uint8_t foundCount, const uint8_t *foundAddresses,
                         bool validScan) {
  if (!validScan || foundCount != 1) {
    stablePassCount = 0;
    hasStableAddress = false;
    return;
  }

  uint8_t address = foundAddresses[0];
  if (stablePassCount > 0 && address == lastSingleAddress) {
    if (stablePassCount < STABLE_REQUIRED_PASSES) {
      stablePassCount++;
    }
  } else {
    stablePassCount = 1;
    lastSingleAddress = address;
  }

  if (stablePassCount >= STABLE_REQUIRED_PASSES) {
    stableAddress = address;
    hasStableAddress = true;
  } else {
    hasStableAddress = false;
  }
}

void printStableAddress() {
  Serial.print("stable_valid_address=");
  if (hasStableAddress) {
    printHexAddress(stableAddress);
    Serial.println();
  } else {
    Serial.println("NA");
  }
}

void scanI2cBus() {
  uint8_t foundCount = 0;
  uint8_t foundAddresses[MAX_VALID_FOUND_COUNT + 1] = {0};
  scanPass++;

  Serial.println();
  Serial.print("scan_pass=");
  Serial.println(scanPass);
  printPinStates("pre_scan");

  if (!readBusReleasedHigh()) {
    Serial.println("BUS_STUCK_LOW_BEFORE_SCAN");
    Serial.println("Skipping Wire address scan to avoid treating a stuck bus as a valid device.");
    Serial.println("found_count=0");
    Serial.println("No I2C devices found");
    updateStableAddress(0, foundAddresses, false);
    printStableAddress();
    return;
  }

  for (uint8_t address = I2C_ADDR_MIN; address <= I2C_ADDR_MAX; ++address) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();

    if (error == 0) {
      if (foundCount <= MAX_VALID_FOUND_COUNT) {
        foundAddresses[foundCount] = address;
      }
      foundCount++;
    } else if (error == 4) {
      Serial.print("unknown_error_address=");
      printHexAddress(address);
      Serial.println();
    }
  }

  bool validScan = foundCount <= MAX_VALID_FOUND_COUNT;
  Serial.print("found_count=");
  Serial.println(foundCount);

  if (!validScan) {
    Serial.println("INVALID_SCAN_TOO_MANY_ADDRESSES");
    Serial.println("valid_found_count=0");
    updateStableAddress(0, foundAddresses, false);
  } else if (foundCount == 0) {
    Serial.println("No I2C devices found");
    updateStableAddress(0, foundAddresses, true);
  } else {
    for (uint8_t i = 0; i < foundCount; ++i) {
      Serial.print("found address=");
      printHexAddress(foundAddresses[i]);
      Serial.println();
    }
    Serial.print("valid_found_count=");
    Serial.println(foundCount);
    updateStableAddress(foundCount, foundAddresses, true);
  }

  printStableAddress();
}

void setup() {
  Serial.begin(USB_BAUD);
  uint32_t serialWaitStartMs = millis();
  while (!Serial && millis() - serialWaitStartMs < SERIAL_WAIT_TIMEOUT_MS) {
    delay(10);
  }

  printStartup();
  configurePinsAsInputs();
  printPinStates("pre_wire");
  Serial.println("About to call Wire.begin()");
  Wire.begin();
  Serial.println("Wire.begin() returned / completed");
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(25000, true);
  Serial.println("Wire timeout enabled");
#else
  Serial.println("Wire timeout unsupported by this core");
#endif
  printPinStates("post_wire");

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
