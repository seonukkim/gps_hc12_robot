// firmware/imu_probe/imu_probe.ino
//
// Standalone IMU / I2C SIGNAL VALIDATION probe for the OpenRB-150 rover.
//
// Goal: confirm whether an IMU is electrically present and *readable* on the
// board's default Wire bus. This is signal validation ONLY. It does not provide
// trusted heading/yaw and must not be used for autonomy yet.
//
// SAFETY: this sketch initializes ONLY USB Serial and Wire (I2C). It does NOT
// initialize or drive:
//   - motors / ESC / Servo outputs
//   - GPS UART
//   - HC-12 UART
//   - RC / PPM input
//   - any rover autonomy / waypoint logic
//
// Fixed IMU wiring (see docs/current_hardware_status.md):
//   - SDA = OpenRB D11 = PA08  (PIN_WIRE_SDA = 11)
//   - SCL = OpenRB D12 = PA09  (PIN_WIRE_SCL = 12)
// These are the OpenRB-150 default `Wire` pins, so this probe uses the hardware
// Wire peripheral (the same path as firmware/i2c_scanner_test).
//
// History: earlier scans repeatedly reported the bus stuck LOW before probing
// (BUS_STUCK_LOW_BEFORE_SCAN), so this probe keeps the proven bus-released-high
// guard before scanning and refuses to treat a stuck/floating bus as a device.

#include <Wire.h>

// ---- Compile-time options ---------------------------------------------------
// Generic-scanner-only mode: scan + label I2C addresses, no register access.
#ifndef IMU_PROBE_SCAN_ONLY
#define IMU_PROBE_SCAN_ONLY 0
#endif
// Raw-register reads for 0x68/0x69 MPU/ICM-class devices. When 1, the probe
// wakes the device (writes PWR_MGMT_1) and burst-reads accel/temp/gyro. When 0,
// the probe stays read-only and reports WHO_AM_I only (requirement 8: keep it
// simple if raw support is uncertain).
#ifndef IMU_PROBE_RAW_ENABLE
#define IMU_PROBE_RAW_ENABLE 1
#endif

constexpr long USB_BAUD = 115200;
constexpr uint8_t DEFAULT_WIRE_SDA_PIN = 11;  // PA08
constexpr uint8_t DEFAULT_WIRE_SCL_PIN = 12;  // PA09
constexpr uint8_t I2C_ADDR_MIN = 0x03;
constexpr uint8_t I2C_ADDR_MAX = 0x77;
constexpr uint32_t SERIAL_WAIT_TIMEOUT_MS = 3000;
constexpr uint32_t SCAN_PERIOD_MS = 1000;
constexpr uint8_t MAX_VALID_FOUND_COUNT = 8;
constexpr uint32_t I2C_CLOCK_HZ = 100000;

// MPU6050 / MPU9250 / ICM-2068x style register map (0x68 / 0x69 family).
constexpr uint8_t MPU_REG_WHO_AM_I = 0x75;
constexpr uint8_t MPU_REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t MPU_REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t MPU_BURST_LEN = 14;  // accel(6) + temp(2) + gyro(6)

uint32_t scanPass = 0;
uint32_t lastScanMs = 0;

void printHexByte(uint8_t value) {
  Serial.print("0x");
  if (value < 0x10) {
    Serial.print("0");
  }
  Serial.print(value, HEX);
}

const char *levelName(uint8_t level) { return level == HIGH ? "HIGH" : "LOW"; }

bool isMpuClassAddress(uint8_t address) {
  return address == 0x68 || address == 0x69;
}

// Address-only candidate hint per requirement 6. Address alone does NOT identify
// a device; it only narrows the candidate family.
const char *candidateLabel(uint8_t address) {
  switch (address) {
    case 0x68:
    case 0x69:
      return "MPU6050_MPU9250_ICM_CANDIDATE";
    case 0x0C:
    case 0x1C:
    case 0x1E:
      return "MAGNETOMETER_CANDIDATE";
    case 0x28:
    case 0x29:
      return "BNO055_CANDIDATE";
    default:
      return "UNKNOWN_I2C_DEVICE";
  }
}

// Decode the MPU-style WHO_AM_I (register 0x75). Note: ICM-209xx and ICM-426xx
// parts use a different WHO_AM_I register, so an unknown value here is expected
// for those and is not a failure.
//
// 0x6F: observed on the known-good breadboard module at I2C 0x69. It is not a
// standard InvenSense/ST device ID, but the part ACKs at 0x69 and answers the
// 0x75 WHO_AM_I register, so it is MPU register-map compatible (a likely
// MPU-6050/6500-class clone or variant). Treat it as signal-validation only; do
// not assume a specific datasheet, scale factor, or trusted yaw/heading from it.
const char *whoamiLabel(uint8_t whoami) {
  switch (whoami) {
    case 0x68:
      return "MPU6050_or_MPU9150";
    case 0x6F:
      return "MPU_CLASS_CLONE_OR_VARIANT_0x6F_signal_only";
    case 0x70:
      return "MPU6500";
    case 0x71:
      return "MPU9250";
    case 0x73:
      return "MPU9255";
    case 0x74:
      return "MPU6515";
    case 0x98:
      return "ICM20689";
    case 0x11:
      return "ICM20602";
    case 0x12:
      return "ICM20601";
    default:
      return "MPU_ICM_FAMILY_UNKNOWN_WHOAMI";
  }
}

int16_t be16(uint8_t hi, uint8_t lo) {
  return static_cast<int16_t>(static_cast<uint16_t>(hi) << 8 | lo);
}

bool readRegisters(uint8_t address, uint8_t reg, uint8_t *buffer, uint8_t length) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {  // repeated start, keep bus
    return false;
  }
  uint8_t received = Wire.requestFrom(address, length);
  if (received != length) {
    while (Wire.available()) {
      Wire.read();
    }
    return false;
  }
  for (uint8_t i = 0; i < length; ++i) {
    buffer[i] = Wire.read();
  }
  return true;
}

bool writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

void configurePinsAsInputs() {
  pinMode(DEFAULT_WIRE_SDA_PIN, INPUT_PULLUP);
  pinMode(DEFAULT_WIRE_SCL_PIN, INPUT_PULLUP);
  delay(2);
}

void printPinStates(const char *prefix) {
  Serial.print(prefix);
  Serial.print("_sda=");
  Serial.print(levelName(digitalRead(DEFAULT_WIRE_SDA_PIN)));
  Serial.print(" ");
  Serial.print(prefix);
  Serial.print("_scl=");
  Serial.println(levelName(digitalRead(DEFAULT_WIRE_SCL_PIN)));
}

bool busReleasedHigh() {
  return digitalRead(DEFAULT_WIRE_SDA_PIN) == HIGH &&
         digitalRead(DEFAULT_WIRE_SCL_PIN) == HIGH;
}

void reportAddress(uint8_t address) {
  Serial.print("i2c_addr=");
  printHexByte(address);
  Serial.print(" imu_candidate=");
  Serial.print(candidateLabel(address));

#if IMU_PROBE_SCAN_ONLY
  Serial.println(" mode=scan_only");
#else
  if (!isMpuClassAddress(address)) {
    // No portable register map for non-0x68 candidates here. Report address +
    // candidate hint only; a magnetometer/BNO055 needs its own driver.
    Serial.println(" whoami=NA note=no_raw_register_support_for_this_address");
    return;
  }

  uint8_t whoami = 0;
  bool whoamiOk = readRegisters(address, MPU_REG_WHO_AM_I, &whoami, 1);
  Serial.print(" whoami=");
  if (whoamiOk) {
    printHexByte(whoami);
    Serial.print(" whoami_label=");
    Serial.print(whoamiLabel(whoami));
  } else {
    Serial.print("NA whoami_label=READ_FAILED");
  }

#if IMU_PROBE_RAW_ENABLE
  // Wake the device (clear SLEEP bit) so accel/gyro produce live data. Writing
  // 0x00 to PWR_MGMT_1 is the standard, idempotent wake for the MPU/ICM-2068x
  // family and is gated to 0x68/0x69 addresses only.
  bool wakeOk = writeRegister(address, MPU_REG_PWR_MGMT_1, 0x00);
  Serial.print(" wake_attempted=true wake_ok=");
  Serial.print(wakeOk ? "true" : "false");

  uint8_t buf[MPU_BURST_LEN];
  if (readRegisters(address, MPU_REG_ACCEL_XOUT_H, buf, MPU_BURST_LEN)) {
    int16_t accelX = be16(buf[0], buf[1]);
    int16_t accelY = be16(buf[2], buf[3]);
    int16_t accelZ = be16(buf[4], buf[5]);
    int16_t tempRaw = be16(buf[6], buf[7]);
    int16_t gyroX = be16(buf[8], buf[9]);
    int16_t gyroY = be16(buf[10], buf[11]);
    int16_t gyroZ = be16(buf[12], buf[13]);
    // MPU6050 temperature formula (approximate; MPU9250/ICM use different
    // constants), so temp_raw is the trustworthy field.
    float tempC = static_cast<float>(tempRaw) / 340.0f + 36.53f;

    Serial.print(" accel_raw_x=");
    Serial.print(accelX);
    Serial.print(" accel_raw_y=");
    Serial.print(accelY);
    Serial.print(" accel_raw_z=");
    Serial.print(accelZ);
    Serial.print(" gyro_raw_x=");
    Serial.print(gyroX);
    Serial.print(" gyro_raw_y=");
    Serial.print(gyroY);
    Serial.print(" gyro_raw_z=");
    Serial.print(gyroZ);
    Serial.print(" temp_raw=");
    Serial.print(tempRaw);
    Serial.print(" temp_c_mpu6050_approx=");
    Serial.print(tempC, 2);
    Serial.println();
  } else {
    Serial.println(
        " accel_raw_x=NA accel_raw_y=NA accel_raw_z=NA gyro_raw_x=NA "
        "gyro_raw_y=NA gyro_raw_z=NA temp_raw=NA raw_read=FAILED");
  }
#else
  Serial.println(" raw_read=disabled");
#endif  // IMU_PROBE_RAW_ENABLE
#endif  // IMU_PROBE_SCAN_ONLY
}

void scanAndReport() {
  scanPass++;
  uint32_t sampleMs = millis();

  Serial.println();
  Serial.print("imu_probe_alive=true scan_pass=");
  Serial.print(scanPass);
  Serial.print(" sample_ms=");
  Serial.println(sampleMs);

  printPinStates("pre_scan");

  if (!busReleasedHigh()) {
    Serial.println("bus_state=BUS_STUCK_LOW_BEFORE_SCAN");
    Serial.println(
        "note=SDA/SCL did not release HIGH; skipping scan. Check IMU power, "
        "GND, pull-ups, and wiring before trusting any result.");
    Serial.println("i2c_scan_count=0");
    Serial.println("imu_present=false");
    return;
  }
  Serial.println("bus_state=RELEASED_HIGH");

  uint8_t found[MAX_VALID_FOUND_COUNT + 1] = {0};
  uint16_t foundCount = 0;
  for (uint8_t address = I2C_ADDR_MIN; address <= I2C_ADDR_MAX; ++address) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      if (foundCount < MAX_VALID_FOUND_COUNT) {
        found[foundCount] = address;
      }
      foundCount++;
    }
  }

  if (foundCount > MAX_VALID_FOUND_COUNT) {
    Serial.print("i2c_scan_count=");
    Serial.println(foundCount);
    Serial.println("scan_result=INVALID_SCAN_TOO_MANY_ADDRESSES");
    Serial.println(
        "note=Many ACKs usually means a stuck/floating bus, not many devices.");
    Serial.println("imu_present=false");
    return;
  }

  Serial.print("i2c_scan_count=");
  Serial.println(foundCount);
  if (foundCount == 0) {
    Serial.println("scan_result=NO_I2C_DEVICES");
    Serial.println("imu_present=false");
    return;
  }

  bool mpuClassSeen = false;
  for (uint16_t i = 0; i < foundCount; ++i) {
    reportAddress(found[i]);
    if (isMpuClassAddress(found[i])) {
      mpuClassSeen = true;
    }
  }
  Serial.print("imu_present=true imu_mpu_class_present=");
  Serial.println(mpuClassSeen ? "true" : "false");
}

void setup() {
  Serial.begin(USB_BAUD);
  uint32_t serialWaitStartMs = millis();
  while (!Serial && millis() - serialWaitStartMs < SERIAL_WAIT_TIMEOUT_MS) {
    delay(10);
  }

  Serial.println();
  Serial.println("Firmware: imu_probe i2c-signal-validation 2026-05-30");
  Serial.println(
      "Safe IMU/I2C probe. No motors, GPS, HC-12, RC, or autonomy are "
      "initialized.");
  Serial.print("usb_baud=");
  Serial.println(USB_BAUD);
  Serial.print("scan_only=");
  Serial.println(IMU_PROBE_SCAN_ONLY ? "true" : "false");
  Serial.print("raw_read_enable=");
  Serial.println(IMU_PROBE_RAW_ENABLE ? "true" : "false");
  Serial.println(
      "Expected fixed IMU wiring: SDA=D11/PA08, SCL=D12/PA09 (default Wire).");
  Serial.println(
      "WARNING: IMU heading/yaw is NOT trusted yet (needs calibration and "
      "drift checks). This probe validates signal only.");

  configurePinsAsInputs();
  printPinStates("pre_wire");
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(25000, true);
  Serial.println("wire_timeout=enabled");
#else
  Serial.println("wire_timeout=unsupported_by_core");
#endif
  printPinStates("post_wire");

  scanAndReport();
  lastScanMs = millis();
}

void loop() {
  uint32_t now = millis();
  if (now - lastScanMs >= SCAN_PERIOD_MS) {
    scanAndReport();
    lastScanMs = now;
  }
}
