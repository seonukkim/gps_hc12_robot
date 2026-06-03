#!/usr/bin/env bash
# Inspect an integrated dry-run log: extract the key IMU / GPS / heading /
# path-following / HC-12 fields and assert that motor output stayed disabled.
#
# Usage: scripts/check_integrated_dryrun_log.sh [LOG]
#   LOG defaults to the newest outputs/logs/integrated_dryrun_hc12_deferred_*.log

set -uo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"

LOG="${1:-}"
if [ -z "${LOG}" ]; then
  LOG="$(ls -t outputs/logs/integrated_dryrun_hc12_deferred_*.log 2>/dev/null | head -1 || true)"
fi
if [ -z "${LOG}" ] || [ ! -f "${LOG}" ]; then
  echo "No log found. Pass a path, or run scripts/run_integrated_dryrun_no_motion.sh first." >&2
  exit 1
fi

# Last value of a space-preceded "field=" token (space anchor avoids matching
# e.g. imu_heading_ready when asking for heading_ready, or unclamped_final_*).
last() { grep -oE " $1=[^ ]+" "${LOG}" 2>/dev/null | tail -1 | sed 's/^ //; s/^[^=]*=//'; }
show() { printf '  %-28s %s\n' "$1" "$(v="$(last "$1")"; echo "${v:-MISSING}")"; }

echo "log=${LOG}  lines=$(wc -l < "${LOG}" | tr -d ' ')"

echo "--- IMU ---"
for f in imu_enabled imu_present imu_calibrated imu_relative_yaw_deg imu_yaw_axis \
         imu_yaw_sign imu_heading_ready imu_heading_block_reason; do show "$f"; done

echo "--- GPS / heading ---"
for f in gps_location_valid gps_motion_ready gps_block_reason heading_ready \
         heading_source heading_agreement_diag target_ready; do show "$f"; done
echo "  note: gps_location_valid/target_ready appear in the single-waypoint no-motion"
echo "        build; the path-following dry-run uses gps_dryrun_ready/gps_motion_ready."

echo "--- path-following ---"
for f in path_following_dryrun path_following_block_reason physical_compile_gate \
         physical_block_reason physical_output_active final_left_cmd final_right_cmd; do show "$f"; done

echo "--- HC-12 (optional; deferred to Ubuntu Station) ---"
for f in hc12_enabled hc12_port hc12_rx_count hc12_tx_count; do show "$f"; done

echo "--- readiness (ever observed true in this log) ---"
for f in imu_present imu_calibrated gps_motion_ready heading_ready; do
  if grep -q " ${f}=true" "${LOG}"; then echo "  ${f}: seen true"; else echo "  ${f}: never true"; fi
done

echo "--- STRICT IMU CHECK (imu_present alone is NOT a pass) ---"
imu_present_last="$(last imu_present)"
imu_whoami_last="$(last imu_whoami)"
imu_ok=false
if grep -q " imu_present=true" "${LOG}" \
   && [ -n "${imu_whoami_last}" ] && [ "${imu_whoami_last}" != "NA" ]; then
  imu_ok=true
  echo "  IMU present at the configured address with whoami=${imu_whoami_last}."
  echo "  (Confirm whoami is an MPU/ICM value and imu_calibrated=true before trusting IMU heading.)"
else
  echo "  IMU NOT validated: imu_present=${imu_present_last:-MISSING} whoami=${imu_whoami_last:-MISSING}"
  echo "  -> IMU_BLOCKED for heading. A single 0x03/UNKNOWN device is not an IMU."
fi

echo "--- HEADING VALIDATION (need MPU-class IMU OR GPS course) ---"
gps_head_ok=false
if grep -q " gps_motion_ready=true" "${LOG}" && grep -q " heading_ready=true" "${LOG}"; then
  gps_head_ok=true
fi
if [ "${imu_ok}" = true ] || [ "${gps_head_ok}" = true ]; then
  echo "  PASS: heading source available (imu_ok=${imu_ok} gps_course_ok=${gps_head_ok})."
else
  echo "  NOT VALIDATED: no MPU-class IMU and no GPS course heading in this log."
  echo "  The no-motion dry-run is still valid for safety/logging, but DO NOT proceed to"
  echo "  outdoor heading validation or any guarded crawl until a heading source is valid."
fi

echo "--- MOTOR-SAFETY CHECK ---"
if grep -q "physical_output_active=true" "${LOG}"; then
  echo "  FAIL: physical_output_active=true found — motor output engaged. STOP."
  exit 3
elif grep -qE "physical_block_reason=(COMPILE_GATE_OFF|MOTOR_OUTPUT_DISABLED)" "${LOG}"; then
  echo "  PASS: physical output blocked (compile gate off). Motors cannot move."
  echo "        (final_left_cmd/final_right_cmd may be nonzero only during RC MANUAL.)"
else
  echo "  WARN: COMPILE_GATE_OFF not confirmed; inspect physical_block_reason above."
fi
