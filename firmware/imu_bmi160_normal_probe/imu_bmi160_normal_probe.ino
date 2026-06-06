// firmware/imu_bmi160_normal_probe/imu_bmi160_normal_probe.ino
//
// Safe BMI160 normal-mode diagnostic probe for the OpenRB-150 rover.
//
// SAFETY: this sketch initializes ONLY USB Serial and Wire (I2C). It does NOT
// initialize or drive motors/ESC/Servo, GPS UART, HC-12 UART, RC/PPM, or any
// rover autonomy.
//
// WRITE POLICY: this diagnostic writes ONLY the BMI160 CMD register (0x7E) with
// the minimal PMU commands needed to read live data:
//   accel normal mode: 0x11, then a conservative 10 ms delay
//   gyro normal mode:  0x15, then a conservative 100 ms delay
// It intentionally skips soft reset (CMD 0xB6) so the probe does not disturb
// the chip more than necessary for this bring-up test.
//
// OpenRB-150 IMU wiring: SDA = D11/PA08, SCL = D12/PA09 (default Wire bus).

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

constexpr long USB_BAUD = 115200;
constexpr uint8_t DEFAULT_WIRE_SDA_PIN = 11;
constexpr uint8_t DEFAULT_WIRE_SCL_PIN = 12;
constexpr uint32_t SERIAL_WAIT_TIMEOUT_MS = 3000;
constexpr uint32_t REPORT_PERIOD_MS = 1000;
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
constexpr uint8_t BMI160_REG_CMD = 0x7E;

constexpr uint8_t BMI160_CMD_ACC_NORMAL = 0x11;
constexpr uint8_t BMI160_CMD_GYR_NORMAL = 0x15;

struct Bmi160InitState {
  uint8_t address;
  bool chipIdReadOk;
  uint8_t chipId;
  bool chipIdOk;
  bool cmdAccelOk;
  bool cmdGyroOk;
  bool pmuBeforeOk;
  uint8_t pmuBefore;
  bool pmuAfterOk;
  uint8_t pmuAfter;
};

Bmi160InitState bmiStates[] = {
    {BMI160_ADDR_A, false, 0, false, false, false, false, 0, false, 0},
    {BMI160_ADDR_B, false, 0, false, false, false, false, 0, false, 0},
};

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

bool writeByte(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
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
    default: return 16384.0;    // diagnostic fallback: assume +/-2g
  }
}

const char *accelRangeLabel(uint8_t accRangeReg) {
  switch (accRangeReg & 0x0F) {
    case 0x03: return "plus_minus_2g";
    case 0x05: return "plus_minus_4g";
    case 0x08: return "plus_minus_8g";
    case 0x0C: return "plus_minus_16g";
    default: return "UNKNOWN_DEFAULT_PLUS_MINUS_2G";
  }
}

double gyroLsbPerDps(uint8_t gyroRangeReg) {
  switch (gyroRangeReg & 0x07) {
    case 0x00: return 16.4;   // +/-2000 dps
    case 0x01: return 32.8;   // +/-1000 dps
    case 0x02: return 65.6;   // +/-500 dps
    case 0x03: return 131.2;  // +/-250 dps
    case 0x04: return 262.4;  // +/-125 dps
    default: return 16.4;     // diagnostic fallback: assume +/-2000 dps
  }
}

const char *gyroRangeLabel(uint8_t gyroRangeReg) {
  switch (gyroRangeReg & 0x07) {
    case 0x00: return "plus_minus_2000dps";
    case 0x01: return "plus_minus_1000dps";
    case 0x02: return "plus_minus_500dps";
    case 0x03: return "plus_minus_250dps";
    case 0x04: return "plus_minus_125dps";
    default: return "UNKNOWN_DEFAULT_PLUS_MINUS_2000DPS";
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

uint8_t accelPmuBits(uint8_t pmuStatus) {
  return (pmuStatus >> 4) & 0x03;
}

uint8_t gyroPmuBits(uint8_t pmuStatus) {
  return (pmuStatus >> 2) & 0x03;
}

void initializeAddress(Bmi160InitState &state) {
  state.chipIdReadOk = readByte(state.address, BMI160_REG_CHIP_ID, state.chipId);
  state.chipIdOk = state.chipIdReadOk && state.chipId == BMI160_CHIP_ID_EXPECTED;
  state.pmuBeforeOk = readByte(state.address, BMI160_REG_PMU_STATUS, state.pmuBefore);
  if (!state.chipIdOk) {
    return;
  }

  state.cmdAccelOk = writeByte(state.address, BMI160_REG_CMD, BMI160_CMD_ACC_NORMAL);
  delay(10);
  state.cmdGyroOk = writeByte(state.address, BMI160_REG_CMD, BMI160_CMD_GYR_NORMAL);
  delay(100);
  state.pmuAfterOk = readByte(state.address, BMI160_REG_PMU_STATUS, state.pmuAfter);
}

void printByteOrNa(bool ok, uint8_t value) {
  if (ok) {
    printHexByte(value);
  } else {
    Serial.print("NA");
  }
}

void reportAddress(const Bmi160InitState &state) {
  uint8_t errReg = 0;
  uint8_t pmuStatus = 0;
  uint8_t accRange = 0;
  uint8_t gyrRange = 0;
  uint8_t gyroBytes[6] = {0};
  uint8_t accelBytes[6] = {0};

  bool errOk = readByte(state.address, BMI160_REG_ERR_REG, errReg);
  bool pmuOk = readByte(state.address, BMI160_REG_PMU_STATUS, pmuStatus);
  bool accRangeOk = readByte(state.address, BMI160_REG_ACC_RANGE, accRange);
  bool gyrRangeOk = readByte(state.address, BMI160_REG_GYR_RANGE, gyrRange);
  bool gyroOk = readRegisters(state.address, BMI160_REG_GYRO_X_LSB, gyroBytes, sizeof(gyroBytes));
  bool accelOk = readRegisters(state.address, BMI160_REG_ACCEL_X_LSB, accelBytes, sizeof(accelBytes));

  uint8_t accelPmu = pmuOk ? accelPmuBits(pmuStatus) : 0;
  uint8_t gyroPmu = pmuOk ? gyroPmuBits(pmuStatus) : 0;
  bool accelNormal = pmuOk && accelPmu == 0x01;
  bool gyroNormal = pmuOk && gyroPmu == 0x01;
  bool errRegOk = errOk && errReg == 0x00;
  bool pmuCommandOk = state.cmdAccelOk && state.cmdGyroOk && state.pmuAfterOk &&
                      accelPmuBits(state.pmuAfter) == 0x01 &&
                      gyroPmuBits(state.pmuAfter) == 0x01 && accelNormal && gyroNormal;

  int16_t gyroRawX = le16(gyroBytes[0], gyroBytes[1]);
  int16_t gyroRawY = le16(gyroBytes[2], gyroBytes[3]);
  int16_t gyroRawZ = le16(gyroBytes[4], gyroBytes[5]);
  int16_t accelRawX = le16(accelBytes[0], accelBytes[1]);
  int16_t accelRawY = le16(accelBytes[2], accelBytes[3]);
  int16_t accelRawZ = le16(accelBytes[4], accelBytes[5]);
  bool rawAllZero = gyroOk && accelOk &&
                    gyroRawX == 0 && gyroRawY == 0 && gyroRawZ == 0 &&
                    accelRawX == 0 && accelRawY == 0 && accelRawZ == 0;

  double accelScale = accelLsbPerG(accRangeOk ? accRange : 0x03);
  double gyroScale = gyroLsbPerDps(gyrRangeOk ? gyrRange : 0x00);
  double axG = accelRawX / accelScale;
  double ayG = accelRawY / accelScale;
  double azG = accelRawZ / accelScale;
  double accelMagG = sqrt(axG * axG + ayG * ayG + azG * azG);
  double gxDps = gyroRawX / gyroScale;
  double gyDps = gyroRawY / gyroScale;
  double gzDps = gyroRawZ / gyroScale;
  double gyroMagDps = sqrt(gxDps * gxDps + gyDps * gyDps + gzDps * gzDps);
  bool accelPlausible = accelOk && accelMagG >= 0.5 && accelMagG <= 1.5;
  bool gyroPlausible = gyroOk && gyroMagDps < 20.0;
  bool dataPlausible = !rawAllZero && accelPlausible && gyroPlausible;
  bool probePass = state.chipIdOk && errRegOk && pmuCommandOk && dataPlausible;

  Serial.print("bmi160_normal_probe_alive=true report=");
  Serial.print(reportCount);
  Serial.print(" i2c_addr=");
  printHexByte(state.address);
  Serial.print(" chip_id=");
  printByteOrNa(state.chipIdReadOk, state.chipId);
  Serial.print(" expected_chip_id=0xD1 chip_id_ok=");
  Serial.print(state.chipIdOk ? "true" : "false");
  Serial.print(" err_reg=");
  printByteOrNa(errOk, errReg);
  Serial.print(" pmu_status_before=");
  printByteOrNa(state.pmuBeforeOk, state.pmuBefore);
  Serial.print(" pmu_status_after=");
  printByteOrNa(pmuOk, pmuStatus);
  Serial.print(" accel_pmu=");
  Serial.print(pmuOk ? pmuModeLabel(accelPmu) : "NA");
  Serial.print(" gyro_pmu=");
  Serial.print(pmuOk ? pmuModeLabel(gyroPmu) : "NA");
  Serial.print(" acc_range=");
  printByteOrNa(accRangeOk, accRange);
  Serial.print(" acc_range_label=");
  Serial.print(accRangeOk ? accelRangeLabel(accRange) : "NA_DEFAULT_PLUS_MINUS_2G");
  Serial.print(" gyr_range=");
  printByteOrNa(gyrRangeOk, gyrRange);
  Serial.print(" gyr_range_label=");
  Serial.print(gyrRangeOk ? gyroRangeLabel(gyrRange) : "NA_DEFAULT_PLUS_MINUS_2000DPS");
  Serial.print(" accel_raw_x=");
  Serial.print(accelRawX);
  Serial.print(" accel_raw_y=");
  Serial.print(accelRawY);
  Serial.print(" accel_raw_z=");
  Serial.print(accelRawZ);
  Serial.print(" accel_mag_g=");
  Serial.print(accelMagG, 3);
  Serial.print(" gyro_raw_x=");
  Serial.print(gyroRawX);
  Serial.print(" gyro_raw_y=");
  Serial.print(gyroRawY);
  Serial.print(" gyro_raw_z=");
  Serial.print(gyroRawZ);
  Serial.print(" gyro_mag_dps=");
  Serial.print(gyroMagDps, 2);
  Serial.print(" accel_plausible=");
  Serial.print(accelPlausible ? "true" : "false");
  Serial.print(" gyro_plausible=");
  Serial.print(gyroPlausible ? "true" : "false");
  Serial.print(" raw_all_zero=");
  Serial.print(rawAllZero ? "true" : "false");
  Serial.print(" bmi160_normal_probe_pass=");
  Serial.print(probePass ? "true" : "false");
  Serial.print(" bmi160_normal_probe_block_reason=");
  if (!state.chipIdReadOk) {
    Serial.print("CHIP_ID_READ_FAILED");
  } else if (!state.chipIdOk) {
    Serial.print("CHIP_ID_NOT_BMI160");
  } else if (!pmuCommandOk) {
    Serial.print("PMU_NORMAL_COMMAND_FAILED");
  } else if (rawAllZero) {
    Serial.print("RAW_DATA_ALL_ZERO");
  } else if (!errRegOk) {
    Serial.print("ERR_REG_NONZERO");
  } else if (!dataPlausible) {
    Serial.print("RAW_DATA_NOT_PLAUSIBLE");
  } else {
    Serial.print("OK");
  }
  Serial.println();
}

void printHeader() {
  Serial.println();
  Serial.println("Firmware: imu_bmi160_normal_probe diagnostic 2026-06-06");
  Serial.println("Safe BMI160 normal-mode probe. No motors, GPS, HC-12, RC, or autonomy are initialized.");
  Serial.println("Writes only BMI160 CMD register 0x7E: accel normal 0x11, gyro normal 0x15.");
  Serial.println("No BMI160 soft reset is performed.");
  Serial.print("usb_baud=");
  Serial.println(USB_BAUD);
  Serial.println("expected_chip_id=0xD1 chip_id_register=0x00 candidate_i2c_addrs=0x68,0x69");
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

  for (uint8_t i = 0; i < sizeof(bmiStates) / sizeof(bmiStates[0]); ++i) {
    initializeAddress(bmiStates[i]);
    Serial.print("bmi160_init i2c_addr=");
    printHexByte(bmiStates[i].address);
    Serial.print(" chip_id=");
    printByteOrNa(bmiStates[i].chipIdReadOk, bmiStates[i].chipId);
    Serial.print(" chip_id_ok=");
    Serial.print(bmiStates[i].chipIdOk ? "true" : "false");
    Serial.print(" pmu_status_before=");
    printByteOrNa(bmiStates[i].pmuBeforeOk, bmiStates[i].pmuBefore);
    Serial.print(" cmd_accel_normal_ok=");
    Serial.print(bmiStates[i].cmdAccelOk ? "true" : "false");
    Serial.print(" cmd_gyro_normal_ok=");
    Serial.print(bmiStates[i].cmdGyroOk ? "true" : "false");
    Serial.print(" pmu_status_after_init=");
    printByteOrNa(bmiStates[i].pmuAfterOk, bmiStates[i].pmuAfter);
    Serial.println();
  }
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
  for (uint8_t i = 0; i < sizeof(bmiStates) / sizeof(bmiStates[0]); ++i) {
    reportAddress(bmiStates[i]);
  }
}
