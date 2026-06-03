#!/usr/bin/env bash
# Strict pass/fail check for a firmware/imu_probe log.
#
# imu_present=true ALONE is NOT a pass: any ACK (e.g. a single 0x03 address that
# reads UNKNOWN_I2C_DEVICE) sets imu_present=true. A real IMU pass requires an
# MPU/ICM-style device: imu_mpu_class_present=true with a 0x68/0x69 address and a
# readable WHO_AM_I.
#
# FAIL conditions: i2c_addr=0x03, UNKNOWN_I2C_DEVICE, imu_mpu_class_present=false,
# imu_probe_block_reason!=OK, or no MPU-class address seen.
#
# Usage: scripts/check_imu_probe_log.sh LOG     (e.g. an arduino-cli monitor | tee log)

set -uo pipefail

LOG="${1:-}"
if [ -z "${LOG}" ] || [ ! -f "${LOG}" ]; then
  echo "Usage: scripts/check_imu_probe_log.sh <imu_probe_log>" >&2
  echo "  Capture one with: arduino-cli monitor -p <PORT> --config baudrate=115200 | tee log" >&2
  exit 1
fi

last() { grep -oE " ?$1=[^ ]+" "${LOG}" 2>/dev/null | tail -1 | sed 's/^ //; s/^[^=]*=//'; }

mpu_class="$(last imu_mpu_class_present)"
block="$(last imu_probe_block_reason)"
addr="$(grep -oE 'i2c_addr=0x[0-9A-Fa-f]+' "${LOG}" | tail -1 | sed 's/.*=//')"
whoami="$(last whoami)"
probe_pass="$(last imu_probe_pass)"

echo "log=${LOG}"
echo "  last i2c_addr            = ${addr:-MISSING}"
echo "  last whoami              = ${whoami:-MISSING}"
echo "  imu_mpu_class_present    = ${mpu_class:-MISSING}"
echo "  imu_probe_pass           = ${probe_pass:-MISSING}"
echo "  imu_probe_block_reason   = ${block:-MISSING}"

fail=0
fail_reasons=""
add_fail() { fail=1; fail_reasons="${fail_reasons}\n  - $1"; }

grep -q "imu_mpu_class_present=true" "${LOG}" || add_fail "imu_mpu_class_present never true"
grep -qE 'i2c_addr=0x(68|69)\b' "${LOG}" || add_fail "no MPU/ICM-style address (0x68/0x69) found"
if grep -q "UNKNOWN_I2C_DEVICE" "${LOG}"; then add_fail "UNKNOWN_I2C_DEVICE present (non-MPU device, e.g. 0x03)"; fi
if grep -qE 'i2c_addr=0x03\b' "${LOG}"; then add_fail "i2c_addr=0x03 seen (bus artifact / non-IMU)"; fi
if [ -n "${block}" ] && [ "${block}" != "OK" ]; then add_fail "imu_probe_block_reason=${block}"; fi

echo "--- IMU STRICT VERDICT ---"
if [ "${fail}" -eq 0 ]; then
  echo "  PASS: MPU/ICM-class IMU detected with a valid address."
  echo "  (Also confirm whoami is a real value and gyro calibration completes in continuous mode.)"
  exit 0
else
  echo "  FAIL: IMU_BLOCKED / IMU_NOT_MPU_CLASS_DETECTED"
  printf '%b\n' "${fail_reasons}"
  echo "  Known-good reference: i2c_addr=0x69, whoami=0x6F, imu_mpu_class_present=true."
  echo "  Fix the IMU wiring/power/module before trusting heading; GPS course heading is the"
  echo "  fallback. This does NOT block the no-motion dry-run (which still runs for logging)."
  exit 3
fi
