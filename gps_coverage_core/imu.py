"""Small IMU helper functions used by host-side tests and log tooling."""

from __future__ import annotations

import math
from dataclasses import dataclass


BMI160_CHIP_ID = 0xD1


def le_i16(lo: int, hi: int) -> int:
    """Decode a little-endian signed 16-bit integer."""
    value = ((hi & 0xFF) << 8) | (lo & 0xFF)
    if value & 0x8000:
        value -= 0x10000
    return value


def bmi160_accel_lsb_per_g(range_reg: int | None) -> tuple[float, bool]:
    """Return (lsb_per_g, defaulted) for BMI160 ACC_RANGE."""
    table = {
        0x03: 16384.0,
        0x05: 8192.0,
        0x08: 4096.0,
        0x0C: 2048.0,
    }
    if range_reg is None:
        return 16384.0, True
    value = table.get(range_reg & 0x0F)
    if value is None:
        return 16384.0, True
    return value, False


def bmi160_gyro_lsb_per_dps(range_reg: int | None) -> tuple[float, bool]:
    """Return (lsb_per_dps, defaulted) for BMI160 GYR_RANGE."""
    table = {
        0x00: 16.4,
        0x01: 32.8,
        0x02: 65.6,
        0x03: 131.2,
        0x04: 262.4,
    }
    if range_reg is None:
        return 16.4, True
    value = table.get(range_reg & 0x07)
    if value is None:
        return 16.4, True
    return value, False


def bmi160_pmu_modes(pmu_status: int) -> tuple[str, str]:
    """Decode BMI160 accel and gyro PMU mode names from PMU_STATUS."""
    names = {
        0x00: "SUSPEND",
        0x01: "NORMAL",
        0x02: "LOW_POWER_OR_FAST_START",
        0x03: "RESERVED",
    }
    gyro = names[(pmu_status >> 2) & 0x03]
    accel = names[(pmu_status >> 4) & 0x03]
    return accel, gyro


@dataclass(frozen=True)
class Bmi160Sample:
    chip_id: int | None
    err_reg: int | None
    pmu_status: int | None
    accel_raw: tuple[int, int, int]
    gyro_raw: tuple[int, int, int]
    acc_range: int | None = 0x03
    gyr_range: int | None = 0x00

    @property
    def accel_mag_g(self) -> float:
        scale, _ = bmi160_accel_lsb_per_g(self.acc_range)
        ax, ay, az = (axis / scale for axis in self.accel_raw)
        return math.sqrt(ax * ax + ay * ay + az * az)

    @property
    def gyro_mag_dps(self) -> float:
        scale, _ = bmi160_gyro_lsb_per_dps(self.gyr_range)
        gx, gy, gz = (axis / scale for axis in self.gyro_raw)
        return math.sqrt(gx * gx + gy * gy + gz * gz)

    @property
    def raw_all_zero(self) -> bool:
        return all(axis == 0 for axis in (*self.accel_raw, *self.gyro_raw))


def bmi160_block_reason(sample: Bmi160Sample) -> str:
    """Mirror the firmware's BMI160 normal-mode pass/block vocabulary."""
    if sample.chip_id is None:
        return "CHIP_ID_READ_FAILED"
    if sample.chip_id != BMI160_CHIP_ID:
        return "CHIP_ID_NOT_BMI160"
    if sample.pmu_status is None:
        return "PMU_NORMAL_COMMAND_FAILED"
    accel_pmu, gyro_pmu = bmi160_pmu_modes(sample.pmu_status)
    if accel_pmu != "NORMAL" or gyro_pmu != "NORMAL":
        return "PMU_NORMAL_COMMAND_FAILED"
    if sample.raw_all_zero:
        return "RAW_DATA_ALL_ZERO"
    if sample.err_reg is not None and sample.err_reg != 0:
        return "ERR_REG_NONZERO"
    accel_ok = 0.5 <= sample.accel_mag_g <= 1.5
    gyro_ok = sample.gyro_mag_dps < 20.0
    if not (accel_ok and gyro_ok):
        return "RAW_DATA_NOT_PLAUSIBLE"
    return "OK"
