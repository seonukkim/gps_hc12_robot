#ifndef I2C_BITBANG_SDA_PIN
#define I2C_BITBANG_SDA_PIN 11
#endif

#ifndef I2C_BITBANG_SCL_PIN
#define I2C_BITBANG_SCL_PIN 12
#endif

constexpr long USB_BAUD = 115200;
constexpr uint8_t I2C_ADDR_MIN = 0x03;
constexpr uint8_t I2C_ADDR_MAX = 0x77;
constexpr uint32_t SERIAL_WAIT_TIMEOUT_MS = 3000;
constexpr uint32_t SCAN_PERIOD_MS = 2000;
constexpr uint16_t I2C_HALF_PERIOD_US = 5;
constexpr uint8_t MAX_VALID_FOUND_COUNT = 8;
constexpr uint8_t STABLE_REQUIRED_PASSES = 3;

uint32_t scanPass = 0;
uint32_t lastScanMs = 0;
uint8_t lastValidAddresses[8] = {0};
uint8_t lastValidCount = 0;
uint8_t stableAddress = 0;
uint8_t stablePassCount = 0;
bool hasStableAddress = false;

void releaseLine(uint8_t pin) {
  pinMode(pin, INPUT_PULLUP);
}

void driveLow(uint8_t pin) {
  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT);
}

void releaseSda() {
  releaseLine(I2C_BITBANG_SDA_PIN);
}

void releaseScl() {
  releaseLine(I2C_BITBANG_SCL_PIN);
}

void sdaLow() {
  driveLow(I2C_BITBANG_SDA_PIN);
}

void sclLow() {
  driveLow(I2C_BITBANG_SCL_PIN);
}

bool readSda() {
  return digitalRead(I2C_BITBANG_SDA_PIN) == HIGH;
}

bool readScl() {
  return digitalRead(I2C_BITBANG_SCL_PIN) == HIGH;
}

void i2cDelay() {
  delayMicroseconds(I2C_HALF_PERIOD_US);
}

void i2cStart() {
  releaseSda();
  releaseScl();
  i2cDelay();
  sdaLow();
  i2cDelay();
  sclLow();
  i2cDelay();
}

void i2cStop() {
  sdaLow();
  i2cDelay();
  releaseScl();
  i2cDelay();
  releaseSda();
  i2cDelay();
}

void releaseBus() {
  releaseSda();
  releaseScl();
  delayMicroseconds(50);
}

bool i2cWriteByte(uint8_t value) {
  for (int bit = 7; bit >= 0; --bit) {
    if ((value & (1 << bit)) != 0) {
      releaseSda();
    } else {
      sdaLow();
    }
    i2cDelay();
    releaseScl();
    i2cDelay();
    sclLow();
    i2cDelay();
  }

  releaseSda();
  i2cDelay();
  releaseScl();
  i2cDelay();
  bool sclHigh = readScl();
  bool ackReceived = sclHigh && !readSda();
  sclLow();
  i2cDelay();
  return ackReceived;
}

void printHexAddress(uint8_t address) {
  Serial.print("0x");
  if (address < 0x10) {
    Serial.print("0");
  }
  Serial.print(address, HEX);
}

void printBanner() {
  Serial.println();
  Serial.println("D11/D12 bitbang I2C scanner starting");
  Serial.print("SDA pin=");
  Serial.println(I2C_BITBANG_SDA_PIN);
  Serial.print("SCL pin=");
  Serial.println(I2C_BITBANG_SCL_PIN);
  Serial.print("USB baud=");
  Serial.println(USB_BAUD);
  Serial.println("Open-drain style: release=INPUT_PULLUP, drive-low=OUTPUT LOW, never drive HIGH.");
  Serial.println("No motors or Servo outputs are used by this sketch.");
}

void readReleasedBusState(bool &sdaHigh, bool &sclHigh) {
  releaseBus();
  sdaHigh = readSda();
  sclHigh = readScl();
}

void printReleasedBusState(bool sdaHigh, bool sclHigh) {
  Serial.print("released_sda=");
  Serial.print(sdaHigh ? "HIGH" : "LOW");
  Serial.print(" released_scl=");
  Serial.println(sclHigh ? "HIGH" : "LOW");
  if (!sdaHigh) {
    Serial.println("SDA stuck low");
  }
  if (!sclHigh) {
    Serial.println("SCL stuck low");
  }
}

void recoverBus() {
  Serial.println("bus_recovery=start");
  releaseSda();
  for (uint8_t pulse = 0; pulse < 9; ++pulse) {
    sclLow();
    i2cDelay();
    releaseScl();
    i2cDelay();
  }
  i2cStop();
  releaseBus();
  bool sdaHigh = readSda();
  bool sclHigh = readScl();
  Serial.print("after_recovery_sda=");
  Serial.print(sdaHigh ? "HIGH" : "LOW");
  Serial.print(" after_recovery_scl=");
  Serial.println(sclHigh ? "HIGH" : "LOW");
  Serial.println("bus_recovery=end");
}

void updateStableDetection(const uint8_t *addresses, uint8_t count, bool validScan) {
  if (!validScan || count == 0) {
    stablePassCount = 0;
    hasStableAddress = false;
    lastValidCount = 0;
    return;
  }

  if (count == 1 && lastValidCount == 1 && addresses[0] == lastValidAddresses[0]) {
    if (stablePassCount < STABLE_REQUIRED_PASSES) {
      stablePassCount++;
    }
  } else {
    stablePassCount = 1;
  }

  lastValidCount = count;
  for (uint8_t i = 0; i < count && i < MAX_VALID_FOUND_COUNT; ++i) {
    lastValidAddresses[i] = addresses[i];
  }

  if (count == 1 && stablePassCount >= STABLE_REQUIRED_PASSES) {
    stableAddress = addresses[0];
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
  uint8_t rawFoundCount = 0;
  uint8_t foundAddresses[MAX_VALID_FOUND_COUNT + 1] = {0};
  scanPass++;

  Serial.println();
  Serial.print("scan_pass=");
  Serial.println(scanPass);
  bool sdaHigh = false;
  bool sclHigh = false;
  readReleasedBusState(sdaHigh, sclHigh);
  printReleasedBusState(sdaHigh, sclHigh);

  bool busStuckLow = !sdaHigh || !sclHigh;
  if (busStuckLow) {
    Serial.println("bus_stuck_low=true");
    recoverBus();
    updateStableDetection(foundAddresses, 0, false);
    Serial.print("raw_found_count=0 valid_found_count=0 ");
    printStableAddress();
    Serial.print("after scan pass=");
    Serial.println(scanPass);
    return;
  }

  Serial.println("bus_stuck_low=false");

  for (uint8_t address = I2C_ADDR_MIN; address <= I2C_ADDR_MAX; ++address) {
    i2cStart();
    bool ackReceived = i2cWriteByte(static_cast<uint8_t>(address << 1));
    i2cStop();
    releaseBus();

    if (ackReceived) {
      if (rawFoundCount <= MAX_VALID_FOUND_COUNT) {
        foundAddresses[rawFoundCount] = address;
      }
      rawFoundCount++;
    }
  }

  bool validScan = rawFoundCount <= MAX_VALID_FOUND_COUNT;
  uint8_t validFoundCount = validScan ? rawFoundCount : 0;

  if (!validScan) {
    Serial.println("INVALID_SCAN_ACK_STUCK_LOW");
  } else if (validFoundCount == 0) {
    Serial.println("No I2C devices found");
  } else {
    for (uint8_t i = 0; i < validFoundCount; ++i) {
      Serial.print("found address=");
      printHexAddress(foundAddresses[i]);
      Serial.println();
    }
  }

  updateStableDetection(foundAddresses, validFoundCount, validScan);
  Serial.print("raw_found_count=");
  Serial.print(rawFoundCount);
  Serial.print(" valid_found_count=");
  Serial.print(validFoundCount);
  Serial.print(" ");
  printStableAddress();
  Serial.print("after scan pass=");
  Serial.println(scanPass);
}

void setup() {
  Serial.begin(USB_BAUD);
  uint32_t serialWaitStartMs = millis();
  while (!Serial && millis() - serialWaitStartMs < SERIAL_WAIT_TIMEOUT_MS) {
    delay(10);
  }

  printBanner();
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
