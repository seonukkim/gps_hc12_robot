#!/usr/bin/env bash
# Summarize firmware/imu_bmi160_normal_probe logs.
#
# Usage:
#   scripts/check_bmi160_normal_probe_log.sh [LOG]
#   LOG defaults to newest outputs/logs/imu_bmi160_normal_probe_*.log

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"

LOG="${1:-}"
if [ -z "$LOG" ]; then
  LOG="$(ls -t outputs/logs/imu_bmi160_normal_probe_*.log 2>/dev/null | head -1 || true)"
fi
if [ -z "$LOG" ] || [ ! -f "$LOG" ]; then
  echo "No BMI160 normal probe log found. Run scripts/run_bmi160_normal_probe.sh first, or pass a path." >&2
  exit 1
fi

last_field_from_line() {
  local line="$1"
  local field="$2"
  printf '%s\n' "$line" | grep -oE "(^| )${field}=[^ ]+" | tail -1 | sed 's/^ //; s/^[^=]*=//' || true
}

line_count="$(wc -l < "$LOG" | tr -d ' ')"
alive_count="$(grep -c 'bmi160_normal_probe_alive=true' "$LOG" || true)"

echo "log=${LOG}"
echo "lines=${line_count}"
echo "alive_lines=${alive_count}"
echo

echo "--- Latest address summaries ---"
missing=0
for addr in 0x68 0x69; do
  line="$(grep "i2c_addr=${addr} " "$LOG" | grep 'bmi160_normal_probe_alive=true' | tail -1 || true)"
  if [ -z "$line" ]; then
    echo "  i2c_addr=${addr} MISSING"
    missing=1
    continue
  fi
  printf '  i2c_addr=%s chip_id=%s chip_id_ok=%s pmu_after=%s accel_pmu=%s gyro_pmu=%s accel_mag_g=%s gyro_mag_dps=%s pass=%s block=%s\n' \
    "$addr" \
    "$(last_field_from_line "$line" chip_id)" \
    "$(last_field_from_line "$line" chip_id_ok)" \
    "$(last_field_from_line "$line" pmu_status_after)" \
    "$(last_field_from_line "$line" accel_pmu)" \
    "$(last_field_from_line "$line" gyro_pmu)" \
    "$(last_field_from_line "$line" accel_mag_g)" \
    "$(last_field_from_line "$line" gyro_mag_dps)" \
    "$(last_field_from_line "$line" bmi160_normal_probe_pass)" \
    "$(last_field_from_line "$line" bmi160_normal_probe_block_reason)"
done
echo

echo "--- PASS observations ---"
pass_lines="$(grep 'bmi160_normal_probe_pass=true' "$LOG" || true)"
if [ -n "$pass_lines" ]; then
  echo "$pass_lines" | tail -5
else
  echo "  none"
fi
echo

echo "--- Block reasons observed ---"
grep -oE 'bmi160_normal_probe_block_reason=[^ ]+' "$LOG" | sort | uniq -c | sed 's/^/  /' || true
echo

echo "--- Verdict ---"
if [ "$alive_count" -eq 0 ]; then
  echo "FAIL: NO_PROBE_OUTPUT"
  echo "Meaning: no bmi160_normal_probe_alive lines were captured."
  exit 2
fi
if [ "$missing" -ne 0 ]; then
  echo "WARN: one or more candidate addresses were not present in the captured log."
fi
if grep -q 'bmi160_normal_probe_pass=true' "$LOG"; then
  echo "PASS: BMI160_NORMAL_DATA_OK"
  echo "Meaning: chip_id=0xD1, PMU normal, err_reg clear, and plausible accel/gyro were observed."
  exit 0
fi
if grep -q 'bmi160_normal_probe_block_reason=PMU_NORMAL_COMMAND_FAILED' "$LOG"; then
  echo "FAIL: PMU_NORMAL_COMMAND_FAILED"
  echo "Meaning: BMI160 chip may be present, but accel/gyro did not enter NORMAL after CMD writes."
  exit 3
fi
if grep -q 'bmi160_normal_probe_block_reason=RAW_DATA_ALL_ZERO' "$LOG"; then
  echo "FAIL: RAW_DATA_ALL_ZERO"
  echo "Meaning: chip identity/PMU may be OK, but live accel/gyro data remained zero."
  exit 4
fi
if grep -q 'bmi160_normal_probe_block_reason=RAW_DATA_NOT_PLAUSIBLE' "$LOG"; then
  echo "FAIL: RAW_DATA_NOT_PLAUSIBLE"
  echo "Meaning: raw values were read, but stationary accel/gyro magnitudes are not plausible."
  exit 5
fi
if grep -q 'bmi160_normal_probe_block_reason=ERR_REG_NONZERO' "$LOG"; then
  echo "FAIL: ERR_REG_NONZERO"
  echo "Meaning: BMI160 ERR_REG reported a nonzero status."
  exit 6
fi
if grep -q 'bmi160_normal_probe_block_reason=CHIP_ID_NOT_BMI160' "$LOG"; then
  echo "FAIL: CHIP_ID_NOT_BMI160"
  exit 7
fi
if grep -q 'bmi160_normal_probe_block_reason=CHIP_ID_READ_FAILED' "$LOG"; then
  echo "FAIL: CHIP_ID_READ_FAILED"
  exit 8
fi

echo "FAIL: BMI160_NORMAL_PROBE_NOT_PASSED"
echo "Meaning: no pass line was observed; inspect latest address summaries above."
exit 9
