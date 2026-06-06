// firmware/imu_bmi160_probe/imu_bmi160_probe.ino
//
// Safe read-only BMI160 diagnostic probe for the OpenRB-150 rover.
//
// SAFETY: this sketch initializes ONLY USB Serial and Wire (I2C). It does NOT
// initialize or drive motors/ESC/Servo, GPS UART, HC-12 UART, RC/PPM, or any
// rover autonomy.
//
// READ-ONLY POLICY: this probe reads BMI160 registers only. It does not write
// BMI160 command/configuration registers and does not switch the sensor into
// normal mode. If CHIP_ID is correct but PMU_STATUS indicates suspend and raw
// values are not plausible, record that result first before deciding whether a
// separate normal-mode probe is warranted.
//
// OpenRB-150 IMU wiring: SDA = D11/PA08, SCL = D12/PA09 (default Wire bus).

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

constexpr long USB_BAUD = 115200;
constexpr uint8_t DEFAULT_WIRE_SDA_PIN = 11;
constexpr uint8_t DEFAULT_WIRE_SCL_PIN = 12;
constexpr uint32_t SERIAL_WAIT_TIMEOUT_MS = 3000;
constexpr uint32_t REPORT_PERIOD_MS = 500;
constexpr uint32_t I2C_CLOCK_HZ = 100000;

constexpr uint8_t BMI160_ADDR_A = 0x68;
constexpr uint8_t BMI160_ADDR_B = 0x69;
constexpr uint8_t BMI160_CHIP_ID_EXPECTED = 0xD1;

constexpr uint8_t BMI160_REG_CHIP_ID = 0x00;
constexpr uint8_t BMI160_REG_ERR_REG = 0x02;
constexpr uint8_t BMI160_REG_PMU_STATUS = 0x03;
constexpr uint8_t BMI160_REG_GYRO_X_LSB = 0x0C;
constexpr uint8_t BMI160_REG_ACCEL_X_LSB = 0x12;
constexpr uint8_t BMI160_REG_ACC_RANGE = 0x41;
constexpr uint8_t BMI160_REG_GYR_RANGE = 0x43;

uint32_t reportCount = 0;
uint32_t lastReportMs = 0;

const char *levelName(uint8_t level) {
  return level == HIGH ? "HIGH" : "LOW";
}

void printHexByte(uint8_t value) {
  Serial.print("0x");
  if (value < 0x10) {
    Serial.print("0");
  }
  Serial.print(value, HEX);
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
  Serial.print(levelName(digitalRead(DEFAULT_WIRE_SCL_PIN)));
}

bool readRegisters(uint8_t address, uint8_t reg, uint8_t *buffer, uint8_t length) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
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

bool readByte(uint8_t address, uint8_t reg, uint8_t &valueOut) {
  return readRegisters(address, reg, &valueOut, 1);
}

int16_t le16(uint8_t lo, uint8_t hi) {
  return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo);
}

double accelLsbPerG(uint8_t accRangeReg) {
  switch (accRangeReg & 0x0F) {
    case 0x03: return 16384.0;  // +/-2g
    case 0x05: return 8192.0;   // +/-4g
    case 0x08: return 4096.0;   // +/-8g
    case 0x0C: return 2048.0;   // +/-16g
    default: return 0.0;
  }
}

const char *accelRangeLabel(uint8_t accRangeReg) {
  switch (accRangeReg & 0x0F) {
    case 0x03: return "plus_minus_2g";
    case 0x05: return "plus_minus_4g";
    case 0x08: return "plus_minus_8g";
    case 0x0C: return "plus_minus_16g";
    default: return "UNKNOWN";
  }
}

double gyroLsbPerDps(uint8_t gyroRangeReg) {
  switch (gyroRangeReg & 0x07) {
    case 0x00: return 16.4;   // +/-2000 dps
    case 0x01: return 32.8;   // +/-1000 dps
    case 0x02: return 65.6;   // +/-500 dps
    case 0x03: return 131.2;  // +/-250 dps
    case 0x04: return 262.4;  // +/-125 dps
    default: return 0.0;
  }
}

const char *gyroRangeLabel(uint8_t gyroRangeReg) {
  switch (gyroRangeReg & 0x07) {
    case 0x00: return "plus_minus_2000dps";
    case 0x01: return "plus_minus_1000dps";
    case 0x02: return "plus_minus_500dps";
    case 0x03: return "plus_minus_250dps";
    case 0x04: return "plus_minus_125dps";
    default: return "UNKNOWN";
  }
}

const char *pmuModeLabel(uint8_t mode) {
  switch (mode & 0x03) {
    case 0x00: return "SUSPEND";
    case 0x01: return "NORMAL";
    case 0x02: return "LOW_POWER_OR_FAST_START";
    default: return "RESERVED";
  }
}

void reportAddress(uint8_t address) {
  uint8_t chipId = 0;
  uint8_t errReg = 0;
  uint8_t pmuStatus = 0;
  uint8_t accRange = 0;
  uint8_t gyrRange = 0;
  uint8_t gyroBytes[6] = {0};
  uint8_t accelBytes[6] = {0};

  bool chipOkRead = readByte(address, BMI160_REG_CHIP_ID, chipId);
  bool errOk = readByte(address, BMI160_REG_ERR_REG, errReg);
  bool pmuOk = readByte(address, BMI160_REG_PMU_STATUS, pmuStatus);
  bool accRangeOk = readByte(address, BMI160_REG_ACC_RANGE, accRange);
  bool gyrRangeOk = readByte(address, BMI160_REG_GYR_RANGE, gyrRange);
  bool gyroOk = readRegisters(address, BMI160_REG_GYRO_X_LSB, gyroBytes, sizeof(gyroBytes));
  bool accelOk = readRegisters(address, BMI160_REG_ACCEL_X_LSB, accelBytes, sizeof(accelBytes));

  bool chipIdOk = chipOkRead && chipId == BMI160_CHIP_ID_EXPECTED;
  uint8_t gyroPmu = (pmuStatus >> 2) & 0x03;
  uint8_t accelPmu = (pmuStatus >> 4) & 0x03;
  bool gyroNormal = pmuOk && gyroPmu == 0x01;
  bool accelNormal = pmuOk && accelPmu == 0x01;

  int16_t gyroRawX = le16(gyroBytes[0], gyroBytes[1]);
  int16_t gyroRawY = le16(gyroBytes[2], gyroBytes[3]);
  int16_t gyroRawZ = le16(gyroBytes[4], gyroBytes[5]);
  int16_t accelRawX = le16(accelBytes[0], accelBytes[1]);
  int16_t accelRawY = le16(accelBytes[2], accelBytes[3]);
  int16_t accelRawZ = le16(accelBytes[4], accelBytes[5]);

  double accelScale = accRangeOk ? accelLsbPerG(accRange) : 0.0;
  double gyroScale = gyrRangeOk ? gyroLsbPerDps(gyrRange) : 0.0;
  double axG = accelScale > 0.0 ? accelRawX / accelScale : 0.0;
  double ayG = accelScale > 0.0 ? accelRawY / accelScale : 0.0;
  double azG = accelScale > 0.0 ? accelRawZ / accelScale : 0.0;
  double accelMagG = sqrt(axG * axG + ayG * ayG + azG * azG);
  double gxDps = gyroScale > 0.0 ? gyroRawX / gyroScale : 0.0;
  double gyDps = gyroScale > 0.0 ? gyroRawY / gyroScale : 0.0;
  double gzDps = gyroScale > 0.0 ? gyroRawZ / gyroScale : 0.0;
  double gyroMagDps = sqrt(gxDps * gxDps + gyDps * gyDps + gzDps * gzDps);
  bool accelPlausible = accelOk && accelScale > 0.0 && accelMagG >= 0.5 && accelMagG <= 1.5;
  bool gyroPlausible = gyroOk && gyroScale > 0.0 && gyroMagDps < 500.0;
  bool dataPlausible = accelPlausible && gyroPlausible;
  bool probePass = chipIdOk && dataPlausible;

  Serial.print("bmi160_probe_alive=true report=");
  Serial.print(reportCount);
  Serial.print(" i2c_addr=");
  printHexByte(address);
  Serial.print(" chip_id=");
  if (chipOkRead) {
    printHexByte(chipId);
  } else {
    Serial.print("NA");
  }
  Serial.print(" expected_chip_id=0xD1 chip_id_ok=");
  Serial.print(chipIdOk ? "true" : "false");
  Serial.print(" err_reg=");
  if (errOk) {
    printHexByte(errReg);
  } else {
    Serial.print("NA");
  }
  Serial.print(" pmu_status=");
  if (pmuOk) {
    printHexByte(pmuStatus);
  } else {
    Serial.print("NA");
  }
  Serial.print(" accel_pmu=");
  Serial.print(pmuOk ? pmuModeLabel(accelPmu) : "NA");
  Serial.print(" gyro_pmu=");
  Serial.print(pmuOk ? pmuModeLabel(gyroPmu) : "NA");
  Serial.print(" acc_range=");
  if (accRangeOk) {
    printHexByte(accRange);
  } else {
    Serial.print("NA");
  }
  Serial.print(" acc_range_label=");
  Serial.print(accRangeOk ? accelRangeLabel(accRange) : "NA");
  Serial.print(" gyr_range=");
  if (gyrRangeOk) {
    printHexByte(gyrRange);
  } else {
    Serial.print("NA");
  }
  Serial.print(" gyr_range_label=");
  Serial.print(gyrRangeOk ? gyroRangeLabel(gyrRange) : "NA");
  Serial.print(" accel_raw_x=");
  Serial.print(accelRawX);
  Serial.print(" accel_raw_y=");
  Serial.print(accelRawY);
  Serial.print(" accel_raw_z=");
  Serial.print(accelRawZ);
  Serial.print(" accel_mag_g=");
  if (accelScale > 0.0) {
    Serial.print(accelMagG, 3);
  } else {
    Serial.print("NA");
  }
  Serial.print(" gyro_raw_x=");
  Serial.print(gyroRawX);
  Serial.print(" gyro_raw_y=");
  Serial.print(gyroRawY);
  Serial.print(" gyro_raw_z=");
  Serial.print(gyroRawZ);
  Serial.print(" gyro_mag_dps=");
  if (gyroScale > 0.0) {
    Serial.print(gyroMagDps, 2);
  } else {
    Serial.print("NA");
  }
  Serial.print(" accel_plausible=");
  Serial.print(accelPlausible ? "true" : "false");
  Serial.print(" gyro_plausible=");
  Serial.print(gyroPlausible ? "true" : "false");
  Serial.print(" bmi160_probe_pass=");
  Serial.print(probePass ? "true" : "false");
  Serial.print(" bmi160_probe_block_reason=");
  if (!chipOkRead) {
    Serial.print("CHIP_ID_READ_FAILED");
  } else if (!chipIdOk) {
    Serial.print("CHIP_ID_NOT_BMI160");
  } else if (!accelNormal || !gyroNormal) {
    Serial.print("PMU_NOT_NORMAL_READ_ONLY");
  } else if (!dataPlausible) {
    Serial.print("RAW_DATA_NOT_PLAUSIBLE");
  } else {
    Serial.print("OK");
  }
  Serial.println();
}

void printHeader() {
  Serial.println();
  Serial.println("Firmware: imu_bmi160_probe read-only 2026-06-06");
  Serial.println("Safe BMI160 probe. No motors, GPS, HC-12, RC, or autonomy are initialized.");
  Serial.println("Read-only: reads BMI160 registers; does not write normal-mode commands.");
  Serial.print("usb_baud=");
  Serial.println(USB_BAUD);
  Serial.println("expected_chip_id=0xD1 chip_id_register=0x00");
  Serial.println("candidate_i2c_addrs=0x68,0x69");
}

void setup() {
  Serial.begin(USB_BAUD);
  uint32_t serialWaitStartMs = millis();
  while (!Serial && millis() - serialWaitStartMs < SERIAL_WAIT_TIMEOUT_MS) {
    delay(10);
  }

  printHeader();
  configurePinsAsInputs();
  printPinStates("pre_wire");
  Serial.println();
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(25000, true);
  Serial.println("wire_timeout=enabled");
#else
  Serial.println("wire_timeout=unsupported_by_core");
#endif
  printPinStates("post_wire");
  Serial.println();
}

void loop() {
  uint32_t now = millis();
  if (now - lastReportMs < REPORT_PERIOD_MS) {
    return;
  }
  lastReportMs = now;
  reportCount++;

  printPinStates("pre_report");
  Serial.println();
  reportAddress(BMI160_ADDR_A);
  reportAddress(BMI160_ADDR_B);
}
