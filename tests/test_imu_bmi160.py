"""BMI160 IMU 호스트 헬퍼 계약 테스트 / Contract test for host-side BMI160 IMU helpers.

목적/역할:
    `gps_coverage_core.imu`의 순수 헬퍼들이 펌웨어와 **동일한** 해석을 내는지 잠근다.
    리틀엔디안 16비트 디코드, 레인지 레지스터→스케일(LSB/g, LSB/dps) 표, PMU 전원모드
    디코드, 그리고 정상모드 통과/차단 판정(bmi160_block_reason)의 계약을 검증한다.

시스템 내 위치:
    imu.py는 로버 펌웨어의 IMU 판정 로직을 호스트에서 미러링한 것이고, 이 테스트가 그
    계약을 고정한다. 센서/시리얼 의존이 없어 오프라인으로 돌아간다.

핵심 개념·불변식:
    - 판정 문자열("OK", "CHIP_ID_READ_FAILED" 등)과 판정 순서·임계값(가속 0.5~1.5 g,
      자이로 <20 dps)은 펌웨어와의 계약 — 여기가 그 계약의 잠금 지점이다.
    - 알 수 없는/None 레인지 레지스터는 가장 민감한 기본값 + defaulted=True 로 폴백한다.

리팩토링 노트:
    imu.py의 문자열·임계값·표를 바꾸면 이 테스트가 먼저 깨진다. 펌웨어와 동시에 갱신할 것.

Contract test: the pure BMI160 helpers in imu.py must interpret raw registers exactly as the
firmware does — little-endian i16 decode, range->scale tables, PMU mode decode, and the
normal-mode pass/block verdict. imu.py mirrors the firmware; this file locks that contract
(reason strings, decision order, and thresholds accel 0.5-1.5 g / gyro <20 dps).
"""

from gps_coverage_core.imu import (
    Bmi160Sample,
    bmi160_accel_lsb_per_g,
    bmi160_block_reason,
    bmi160_gyro_lsb_per_dps,
    bmi160_pmu_modes,
    le_i16,
)


def test_le_i16_decodes_bmi160_little_endian_raw_values() -> None:
    """리틀엔디안 부호있는 16비트 디코드 검증(양수·최소·-1). / le_i16 decodes LE signed 16-bit (positive, min, -1)."""
    assert le_i16(0x34, 0x12) == 0x1234
    assert le_i16(0x00, 0x80) == -32768
    assert le_i16(0xFF, 0xFF) == -1


def test_bmi160_range_scale_defaults_are_explicit() -> None:
    """레인지 표 적중 시 defaulted=False, 미적중/미지값은 민감 기본값+True. / Known range -> (scale, False); unknown -> (default, True)."""
    assert bmi160_accel_lsb_per_g(0x03) == (16384.0, False)
    assert bmi160_accel_lsb_per_g(0x99) == (16384.0, True)
    assert bmi160_gyro_lsb_per_dps(0x00) == (16.4, False)
    assert bmi160_gyro_lsb_per_dps(0x07) == (16.4, True)


def test_bmi160_pmu_status_decoding() -> None:
    """PMU_STATUS 비트필드에서 (가속, 자이로) 모드 이름 추출 검증. / Decode (accel, gyro) PMU mode names from the bitfield."""
    assert bmi160_pmu_modes(0x14) == ("NORMAL", "NORMAL")
    assert bmi160_pmu_modes(0x00) == ("SUSPEND", "SUSPEND")


def test_bmi160_block_reason_ok_for_plausible_stationary_sample() -> None:
    """정상 칩·NORMAL 모드·중력만 받는 정지 샘플은 "OK". / A healthy, stationary (≈1 g, ≈0 dps) sample yields "OK".

    chip_id·pmu·err_reg 모두 정상이고 가속≈1 g, 자이로≈0 일 때 통과함을 확인한다.
    / Verifies the all-good path passes when every gate is satisfied.
    """
    sample = Bmi160Sample(
        chip_id=0xD1,
        err_reg=0x00,
        pmu_status=0x14,
        accel_raw=(0, 0, 16500),
        gyro_raw=(2, -3, 4),
    )
    assert bmi160_block_reason(sample) == "OK"


def test_bmi160_block_reason_distinguishes_failures() -> None:
    """각 고장 모드가 고유 사유 문자열로 구분됨을 검증. / Each failure mode maps to its distinct reason string.

    칩ID 읽기 실패→CHIP_ID_READ_FAILED, PMU 비정상→PMU_NORMAL_COMMAND_FAILED,
    원시값 전부 0→RAW_DATA_ALL_ZERO. 판정 순서(칩→PMU→raw)까지 함께 고정한다.
    / Locks the reason vocabulary and the gate order (chip -> PMU -> raw-all-zero).
    """
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
