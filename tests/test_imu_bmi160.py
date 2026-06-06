from gps_coverage_core.imu import (
    Bmi160Sample,
    bmi160_accel_lsb_per_g,
    bmi160_block_reason,
    bmi160_gyro_lsb_per_dps,
    bmi160_pmu_modes,
    le_i16,
)


def test_le_i16_decodes_bmi160_little_endian_raw_values() -> None:
    assert le_i16(0x34, 0x12) == 0x1234
    assert le_i16(0x00, 0x80) == -32768
    assert le_i16(0xFF, 0xFF) == -1


def test_bmi160_range_scale_defaults_are_explicit() -> None:
    assert bmi160_accel_lsb_per_g(0x03) == (16384.0, False)
    assert bmi160_accel_lsb_per_g(0x99) == (16384.0, True)
    assert bmi160_gyro_lsb_per_dps(0x00) == (16.4, False)
    assert bmi160_gyro_lsb_per_dps(0x07) == (16.4, True)


def test_bmi160_pmu_status_decoding() -> None:
    assert bmi160_pmu_modes(0x14) == ("NORMAL", "NORMAL")
    assert bmi160_pmu_modes(0x00) == ("SUSPEND", "SUSPEND")


def test_bmi160_block_reason_ok_for_plausible_stationary_sample() -> None:
    sample = Bmi160Sample(
        chip_id=0xD1,
        err_reg=0x00,
        pmu_status=0x14,
        accel_raw=(0, 0, 16500),
        gyro_raw=(2, -3, 4),
    )
    assert bmi160_block_reason(sample) == "OK"


def test_bmi160_block_reason_distinguishes_failures() -> None:
    assert (
        bmi160_block_reason(
            Bmi160Sample(
                chip_id=None,
                err_reg=None,
                pmu_status=None,
                accel_raw=(0, 0, 0),
                gyro_raw=(0, 0, 0),
            )
        )
        == "CHIP_ID_READ_FAILED"
    )
    assert (
        bmi160_block_reason(
            Bmi160Sample(
                chip_id=0xD1,
                err_reg=0x00,
                pmu_status=0x00,
                accel_raw=(0, 0, 0),
                gyro_raw=(0, 0, 0),
            )
        )
        == "PMU_NORMAL_COMMAND_FAILED"
    )
    assert (
        bmi160_block_reason(
            Bmi160Sample(
                chip_id=0xD1,
                err_reg=0x00,
                pmu_status=0x14,
                accel_raw=(0, 0, 0),
                gyro_raw=(0, 0, 0),
            )
        )
        == "RAW_DATA_ALL_ZERO"
    )
