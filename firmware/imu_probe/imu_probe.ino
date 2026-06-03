// firmware/imu_probe/imu_probe.ino
//
// Standalone IMU diagnostic probe for the OpenRB-150 rover (breadboard prep).
//
// Goal: validate the IMU signal and provide the motion / gyro-bias / relative-yaw
// diagnostics needed BEFORE outdoor GPS-course-vs-IMU comparison. This is a
// diagnostic tool only. It does NOT provide a trusted/absolute heading.
//
// SAFETY: this sketch initializes ONLY USB Serial and Wire (I2C). It does NOT
// initialize or drive motors/ESC/Servo, GPS UART, HC-12 UART, RC/PPM, or any
// rover autonomy.
//
// HEADING REALITY CHECK:
//   - Gyro-integrated yaw here is RELATIVE and DRIFTS. It is reset to 0 at start.
//   - It is NOT an absolute compass heading.
//   - An absolute heading requires a GPS course-over-ground seed, a magnetometer,
//     or another reference, plus calibration and a drift/agreement check.
//   - Use this probe only to: (1) confirm raw signal, (2) estimate gyro bias,
//     (3) find the yaw axis/sign, (4) watch relative-yaw drift while stationary,
//     so it can later be compared against GPS course-over-ground outdoors.
//
// IMU wiring: SDA = D11/PA08, SCL = D12/PA09 (OpenRB default Wire bus).

#include <Wire.h>

// ---- Compile-time options ---------------------------------------------------
#ifndef IMU_PROBE_SCAN_ONLY
#define IMU_PROBE_SCAN_ONLY 0          // 1 = generic I2C scanner + labels only
#endif
#ifndef IMU_PROBE_RAW_ENABLE
#define IMU_PROBE_RAW_ENABLE 1         // 0 = WHO_AM_I only, no wake/raw reads
#endif
#ifndef IMU_CALIBRATION_SECONDS
#define IMU_CALIBRATION_SECONDS 5      // stationary gyro-bias estimation window
#endif
#ifndef IMU_YAW_DIAG
#define IMU_YAW_DIAG 0                 // 1 = integrate gyro axis into relative yaw
#endif
#ifndef IMU_YAW_AXIS
#define IMU_YAW_AXIS 2                 // 0=X, 1=Y, 2=Z (yaw is usually Z)
#endif
#ifndef IMU_YAW_SIGN
#define IMU_YAW_SIGN 1                 // +1 or -1
#endif
#ifndef IMU_GYRO_LSB_PER_DPS
#define IMU_GYRO_LSB_PER_DPS 131.0     // MPU default FS_SEL=0 (+/-250 dps)
#endif
#ifndef IMU_ACCEL_LSB_PER_G
#define IMU_ACCEL_LSB_PER_G 16384.0    // MPU default AFS_SEL=0 (+/-2 g)
#endif

constexpr long USB_BAUD = 115200;
constexpr uint8_t DEFAULT_WIRE_SDA_PIN = 11;  // PA08
constexpr uint8_t DEFAULT_WIRE_SCL_PIN = 12;  // PA09
constexpr uint8_t I2C_ADDR_MIN = 0x03;
constexpr uint8_t I2C_ADDR_MAX = 0x77;
constexpr uint32_t SERIAL_WAIT_TIMEOUT_MS = 3000;
constexpr uint32_t SCAN_PERIOD_MS = 1000;
constexpr uint32_t SAMPLE_PERIOD_MS = 20;     // ~50 Hz continuous sampling
constexpr uint32_t REPORT_PERIOD_MS = 250;    // 4 Hz USB report
constexpr uint8_t MAX_VALID_FOUND_COUNT = 8;
constexpr uint32_t I2C_CLOCK_HZ = 100000;

// Motion / stationary thresholds (calibrated gyro and accel-vs-1g).
constexpr double STATIONARY_GYRO_DPS = 2.0;
constexpr double STATIONARY_ACCEL_G_TOL = 0.12;

// MPU6050 / MPU9250 / ICM-2068x style register map (0x68 / 0x69 family).
constexpr uint8_t MPU_REG_WHO_AM_I = 0x75;
constexpr uint8_t MPU_REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t MPU_REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t MPU_BURST_LEN = 14;  // accel(6) + temp(2) + gyro(6)

// Continuous diagnostics (calibration/yaw/motion) need register reads.
#define IMU_PROBE_CONTINUOUS (!IMU_PROBE_SCAN_ONLY && IMU_PROBE_RAW_ENABLE)

uint32_t scanPass = 0;
uint32_t lastScanMs = 0;

// Locked IMU device + live sample state.
uint8_t imuAddress = 0;
bool imuWhoamiOk = false;
uint8_t imuWhoami = 0;
int16_t accelRawX = 0, accelRawY = 0, accelRawZ = 0;
int16_t gyroRawX = 0, gyroRawY = 0, gyroRawZ = 0;
int16_t tempRaw = 0;
double accelMagG = 0.0;
double gyroMagDps = 0.0;
bool stationaryDetected = false;
bool motionDetected = false;

// Gyro-bias calibration.
bool imuCalibrating = false;
bool imuCalibrated = false;
uint32_t calStartMs = 0;
double gyroSumX = 0.0, gyroSumY = 0.0, gyroSumZ = 0.0;
uint32_t calSampleCount = 0;
double gyroBiasX = 0.0, gyroBiasY = 0.0, gyroBiasZ = 0.0;

// Relative-yaw integration diagnostic.
double imuRelativeYawDeg = 0.0;
bool haveLastSample = false;
uint32_t lastSampleMicros = 0;
uint32_t lastReportMs = 0;
uint32_t lastSampleMs = 0;

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

// Decode the MPU-style WHO_AM_I (register 0x75). 0x6F is the known-good
// breadboard module at 0x69: not a standard InvenSense/ST ID but MPU register-map
// compatible (a likely MPU-6050/6500-class clone/variant), signal-only.
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

double wrap180(double deg) {
  while (deg > 180.0) {
    deg -= 360.0;
  }
  while (deg < -180.0) {
    deg += 360.0;
  }
  return deg;
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

int16_t gyroAxisRaw(uint8_t axis) {
  if (axis == 0) {
    return gyroRawX;
  }
  if (axis == 1) {
    return gyroRawY;
  }
  return gyroRawZ;
}

double gyroAxisBias(uint8_t axis) {
  if (axis == 0) {
    return gyroBiasX;
  }
  if (axis == 1) {
    return gyroBiasY;
  }
  return gyroBiasZ;
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
  Serial.println();
#endif  // IMU_PROBE_SCAN_ONLY
}

// Scan the bus; print discovery; return the first MPU-class address (or 0).
uint8_t scanAndReport() {
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
    Serial.println(
        "i2c_scan_count=0 imu_present=false imu_probe_pass=false "
        "imu_probe_block_reason=BUS_STUCK_LOW");
    return 0;
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
    Serial.println(
        "scan_result=INVALID_SCAN_TOO_MANY_ADDRESSES imu_present=false "
        "imu_probe_pass=false imu_probe_block_reason=INVALID_SCAN");
    return 0;
  }

  Serial.print("i2c_scan_count=");
  Serial.println(foundCount);
  if (foundCount == 0) {
    Serial.println(
        "scan_result=NO_I2C_DEVICES imu_present=false imu_probe_pass=false "
        "imu_probe_block_reason=NO_I2C_DEVICES");
    return 0;
  }

  uint8_t firstMpu = 0;
  for (uint16_t i = 0; i < foundCount; ++i) {
    reportAddress(found[i]);
    if (firstMpu == 0 && isMpuClassAddress(found[i])) {
      firstMpu = found[i];
    }
  }
  bool mpuClassPresent = firstMpu != 0;
  Serial.print("imu_present=true imu_mpu_class_present=");
  Serial.print(mpuClassPresent ? "true" : "false");
  // imu_present=true alone is NOT a pass: a single non-MPU address such as 0x03
  // (UNKNOWN_I2C_DEVICE) is IMU_NOT_MPU_CLASS_DETECTED. A pass requires an
  // MPU/ICM-style address (0x68/0x69); whoami is then read in continuous mode.
  Serial.print(" imu_probe_pass=");
  Serial.print(mpuClassPresent ? "true" : "false");
  Serial.print(" imu_probe_block_reason=");
  Serial.println(mpuClassPresent ? "OK" : "IMU_NOT_MPU_CLASS_DETECTED");
  return firstMpu;
}

#if IMU_PROBE_CONTINUOUS
bool imuReadRaw() {
  uint8_t buf[MPU_BURST_LEN];
  if (!readRegisters(imuAddress, MPU_REG_ACCEL_XOUT_H, buf, MPU_BURST_LEN)) {
    return false;
  }
  accelRawX = be16(buf[0], buf[1]);
  accelRawY = be16(buf[2], buf[3]);
  accelRawZ = be16(buf[4], buf[5]);
  tempRaw = be16(buf[6], buf[7]);
  gyroRawX = be16(buf[8], buf[9]);
  gyroRawY = be16(buf[10], buf[11]);
  gyroRawZ = be16(buf[12], buf[13]);
  return true;
}

// Lock onto an MPU-class device: read WHO_AM_I, wake it, start the calibration.
void imuLockOn(uint8_t address) {
  imuAddress = address;
  imuWhoamiOk = readRegisters(address, MPU_REG_WHO_AM_I, &imuWhoami, 1);
  writeRegister(address, MPU_REG_PWR_MGMT_1, 0x00);  // wake (clear SLEEP)
  delay(10);
  imuCalibrating = true;
  imuCalibrated = false;
  calStartMs = millis();
  gyroSumX = gyroSumY = gyroSumZ = 0.0;
  calSampleCount = 0;
  imuRelativeYawDeg = 0.0;
  haveLastSample = false;
  Serial.print("imu_locked_addr=");
  printHexByte(address);
  Serial.print(" whoami=");
  if (imuWhoamiOk) {
    printHexByte(imuWhoami);
    Serial.print(" whoami_label=");
    Serial.print(whoamiLabel(imuWhoami));
  } else {
    Serial.print("NA");
  }
  Serial.print(" calibrating_for_s=");
  Serial.println(IMU_CALIBRATION_SECONDS);
  Serial.println("Hold the IMU STILL during calibration.");
}

void imuSample() {
  if (!imuReadRaw()) {
    return;
  }
  uint32_t nowMicros = micros();

  // Magnitudes for motion/stationary diagnostics.
  double ax = accelRawX / IMU_ACCEL_LSB_PER_G;
  double ay = accelRawY / IMU_ACCEL_LSB_PER_G;
  double az = accelRawZ / IMU_ACCEL_LSB_PER_G;
  accelMagG = sqrt(ax * ax + ay * ay + az * az);

  double gx = (gyroRawX - gyroBiasX) / IMU_GYRO_LSB_PER_DPS;
  double gy = (gyroRawY - gyroBiasY) / IMU_GYRO_LSB_PER_DPS;
  double gz = (gyroRawZ - gyroBiasZ) / IMU_GYRO_LSB_PER_DPS;
  gyroMagDps = sqrt(gx * gx + gy * gy + gz * gz);

  stationaryDetected =
      (gyroMagDps < STATIONARY_GYRO_DPS) &&
      (fabs(accelMagG - 1.0) < STATIONARY_ACCEL_G_TOL);
  motionDetected = !stationaryDetected;

  if (imuCalibrating) {
    gyroSumX += gyroRawX;
    gyroSumY += gyroRawY;
    gyroSumZ += gyroRawZ;
    calSampleCount++;
    if (millis() - calStartMs >= (uint32_t)IMU_CALIBRATION_SECONDS * 1000UL &&
        calSampleCount > 0) {
      gyroBiasX = gyroSumX / calSampleCount;
      gyroBiasY = gyroSumY / calSampleCount;
      gyroBiasZ = gyroSumZ / calSampleCount;
      imuCalibrating = false;
      imuCalibrated = true;
      Serial.print("imu_calibrated=true gyro_bias_x=");
      Serial.print(gyroBiasX, 2);
      Serial.print(" gyro_bias_y=");
      Serial.print(gyroBiasY, 2);
      Serial.print(" gyro_bias_z=");
      Serial.println(gyroBiasZ, 2);
    }
  }

#if IMU_YAW_DIAG
  // Relative yaw integration (diagnostic only; drifts; not absolute heading).
  if (imuCalibrated) {
    if (haveLastSample) {
      double dtS = (nowMicros - lastSampleMicros) / 1000000.0;
      double rateDps = (gyroAxisRaw(IMU_YAW_AXIS) - gyroAxisBias(IMU_YAW_AXIS)) /
                       IMU_GYRO_LSB_PER_DPS;
      imuRelativeYawDeg = wrap180(imuRelativeYawDeg + (IMU_YAW_SIGN) * rateDps * dtS);
    }
    haveLastSample = true;
  }
#endif
  lastSampleMicros = nowMicros;
}

void imuReport() {
  Serial.print("imu_probe_alive=true sample_ms=");
  Serial.print(millis());
  Serial.print(" bus_state=RELEASED_HIGH i2c_addr=");
  printHexByte(imuAddress);
  Serial.print(" whoami=");
  if (imuWhoamiOk) {
    printHexByte(imuWhoami);
  } else {
    Serial.print("NA");
  }
  Serial.print(" whoami_label=");
  Serial.print(imuWhoamiOk ? whoamiLabel(imuWhoami) : "NA");
  Serial.print(" accel_raw_x=");
  Serial.print(accelRawX);
  Serial.print(" accel_raw_y=");
  Serial.print(accelRawY);
  Serial.print(" accel_raw_z=");
  Serial.print(accelRawZ);
  Serial.print(" gyro_raw_x=");
  Serial.print(gyroRawX);
  Serial.print(" gyro_raw_y=");
  Serial.print(gyroRawY);
  Serial.print(" gyro_raw_z=");
  Serial.print(gyroRawZ);
  Serial.print(" temp_raw=");
  Serial.print(tempRaw);
  Serial.print(" accel_mag_g=");
  Serial.print(accelMagG, 3);
  Serial.print(" gyro_mag_dps=");
  Serial.print(gyroMagDps, 2);
  Serial.print(" stationary_detected=");
  Serial.print(stationaryDetected ? "true" : "false");
  Serial.print(" motion_detected=");
  Serial.print(motionDetected ? "true" : "false");
  Serial.print(" imu_calibrated=");
  Serial.print(imuCalibrated ? "true" : "false");
  Serial.print(" gyro_bias_x=");
  Serial.print(gyroBiasX, 2);
  Serial.print(" gyro_bias_y=");
  Serial.print(gyroBiasY, 2);
  Serial.print(" gyro_bias_z=");
  Serial.print(gyroBiasZ, 2);
  Serial.print(" gyro_cal_x=");
  Serial.print(gyroRawX - gyroBiasX, 1);
  Serial.print(" gyro_cal_y=");
  Serial.print(gyroRawY - gyroBiasY, 1);
  Serial.print(" gyro_cal_z=");
  Serial.print(gyroRawZ - gyroBiasZ, 1);
#if IMU_YAW_DIAG
  Serial.print(" imu_yaw_diag=true imu_relative_yaw_deg=");
  Serial.print(imuRelativeYawDeg, 2);
  Serial.print(" imu_yaw_axis=");
  Serial.print(IMU_YAW_AXIS);
  Serial.print(" imu_yaw_sign=");
  Serial.print(IMU_YAW_SIGN);
  Serial.print(" yaw_note=");
  Serial.print(imuCalibrated ? "RELATIVE_DRIFTS_NOT_ABSOLUTE" : "WAITING_CALIBRATION");
#else
  Serial.print(" imu_yaw_diag=false");
#endif
  Serial.println();
}
#endif  // IMU_PROBE_CONTINUOUS

void setup() {
  Serial.begin(USB_BAUD);
  uint32_t serialWaitStartMs = millis();
  while (!Serial && millis() - serialWaitStartMs < SERIAL_WAIT_TIMEOUT_MS) {
    delay(10);
  }

  Serial.println();
  Serial.println("Firmware: imu_probe imu-motion-yaw-diagnostic 2026-06-03");
  Serial.println(
      "Safe IMU probe. No motors, GPS, HC-12, RC, or autonomy are initialized.");
  Serial.print("usb_baud=");
  Serial.println(USB_BAUD);
  Serial.print("scan_only=");
  Serial.print(IMU_PROBE_SCAN_ONLY ? "true" : "false");
  Serial.print(" raw_read_enable=");
  Serial.print(IMU_PROBE_RAW_ENABLE ? "true" : "false");
  Serial.print(" continuous_diag=");
  Serial.println(IMU_PROBE_CONTINUOUS ? "true" : "false");
  Serial.print("calibration_seconds=");
  Serial.print(IMU_CALIBRATION_SECONDS);
  Serial.print(" yaw_diag=");
  Serial.print(IMU_YAW_DIAG ? "true" : "false");
  Serial.print(" yaw_axis=");
  Serial.print(IMU_YAW_AXIS);
  Serial.print(" yaw_sign=");
  Serial.println(IMU_YAW_SIGN);
  Serial.println(
      "WARNING: gyro yaw is RELATIVE and DRIFTS; it is not an absolute compass "
      "heading. Absolute heading needs a GPS-course seed or magnetometer plus a "
      "drift/agreement check. This probe is diagnostic only.");

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

  uint8_t firstMpu = scanAndReport();
  lastScanMs = millis();
#if IMU_PROBE_CONTINUOUS
  if (firstMpu != 0) {
    imuLockOn(firstMpu);
  }
#else
  (void)firstMpu;
#endif
}

void loop() {
  uint32_t now = millis();

#if IMU_PROBE_CONTINUOUS
  if (imuAddress == 0) {
    // Not locked yet: keep scanning until an MPU-class device appears.
    if (now - lastScanMs >= SCAN_PERIOD_MS) {
      uint8_t firstMpu = scanAndReport();
      lastScanMs = now;
      if (firstMpu != 0) {
        imuLockOn(firstMpu);
      }
    }
    return;
  }
  if (now - lastSampleMs >= SAMPLE_PERIOD_MS) {
    imuSample();
    lastSampleMs = now;
  }
  if (now - lastReportMs >= REPORT_PERIOD_MS) {
    imuReport();
    lastReportMs = now;
  }
#else
  // Scanner-only / raw-disabled fallback: periodic scan + labels.
  if (now - lastScanMs >= SCAN_PERIOD_MS) {
    scanAndReport();
    lastScanMs = now;
  }
#endif
}
