"""Small IMU helper functions used by host-side tests and log tooling.

목적/역할:
    로버의 BMI160 IMU에서 나온 원시 레지스터 값을 호스트(PC) 쪽에서 해석하기 위한 순수 헬퍼 모음이다.
    리틀엔디안 16비트 디코드, ACC/GYR 레인지 레지스터 → 스케일(LSB/g, LSB/dps) 변환, PMU 전원 모드
    해석, 그리고 펌웨어의 "정상 모드 통과/차단" 판정 로직을 그대로 복제한 `bmi160_block_reason`을 제공한다.

시스템 내 위치:
    센서·시리얼에 의존하지 않으므로 `tests/test_imu_bmi160.py`와 IMU 로그 분석 도구가 import 한다.
    로버 펌웨어와 **동일한 판정 어휘/임계값**을 호스트에서 재현하는 것이 목적이라, 펌웨어가 진짜 소스이고
    이 파일은 그 미러다(파이프라인상 오프라인 검증/분석 쪽).

핵심 개념·불변식:
    - 스케일 표는 BMI160 데이터시트의 레인지 코드에 대응한다. 레인지 레지스터가 None이거나 표에 없으면
      가장 민감한 기본값을 쓰고 `defaulted=True`를 함께 반환한다(호출측이 추정값임을 알 수 있게).
    - `bmi160_block_reason`의 반환 문자열(예 "CHIP_ID_NOT_BMI160", "RAW_DATA_ALL_ZERO", "OK")과
      판정 순서·임계값(가속도 0.5~1.5 g, 자이로 <20 dps)은 **펌웨어와 정확히 일치**해야 한다. 바꾸면
      펌웨어도 함께 바꿔야 하며, 테스트가 이 계약을 잠근다.

리팩토링 노트:
    새 BMI160 레인지/모드를 추가할 때 스케일 표만 확장하면 되지만, 판정 문자열은 펌웨어와의 계약이므로
    임의로 이름을 바꾸지 말 것. `Bmi160Sample`은 frozen(불변)이며 크기·자이로 크기를 프로퍼티로 계산한다.

Purpose: host-side pure helpers to interpret raw BMI160 IMU registers — little-endian 16-bit
decode, ACC/GYR range-register -> scale (LSB/g, LSB/dps), PMU power-mode decode, and
``bmi160_block_reason`` which mirrors the firmware's normal-mode pass/block decision. No sensor or
serial dependency; imported by ``tests/test_imu_bmi160.py`` and IMU log tooling. The firmware is the
source of truth and this file mirrors it: the returned reason strings and the decision order +
thresholds (accel 0.5-1.5 g, gyro <20 dps) must match the firmware exactly — changing them means
changing the firmware too (tests lock this contract). Unknown/None range registers fall back to the
most-sensitive default and report ``defaulted=True``. ``Bmi160Sample`` is a frozen dataclass.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


# BMI160 CHIP_ID 레지스터의 고정 기대값 — 이 값이 아니면 센서 오식별/통신 오류.
# / Fixed expected value of the BMI160 CHIP_ID register; a mismatch means wrong/miswired sensor.
BMI160_CHIP_ID = 0xD1


# ── 저수준 디코드 / Low-level register decode ──
def le_i16(lo: int, hi: int) -> int:
    """리틀엔디안 부호있는 16비트 정수 디코드. / Decode a little-endian signed 16-bit integer.

    인자/Args: lo=하위 바이트, hi=상위 바이트(각 0..255 가정). 반환/Returns: -32768..32767.
    """
    value = ((hi & 0xFF) << 8) | (lo & 0xFF)
    # 최상위 비트가 서 있으면 2의 보수 음수 → 2^16을 빼서 부호 복원.
    # / High bit set => two's-complement negative; subtract 2^16 to restore sign.
    if value & 0x8000:
        value -= 0x10000
    return value


# ── 레인지 레지스터 → 스케일 변환 / Range register -> scale factor ──
def bmi160_accel_lsb_per_g(range_reg: int | None) -> tuple[float, bool]:
    """ACC_RANGE 값에 대한 (LSB/g, 기본값사용여부) 반환. / Return (lsb_per_g, defaulted) for BMI160 ACC_RANGE.

    표에 없거나 None이면 가장 민감한 16384 LSB/g로 폴백하고 defaulted=True.
    Unknown/None falls back to the most-sensitive 16384 LSB/g and defaulted=True.
    """
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
    """GYR_RANGE 값에 대한 (LSB/dps, 기본값사용여부) 반환. / Return (lsb_per_dps, defaulted) for BMI160 GYR_RANGE.

    표에 없거나 None이면 가장 민감한 16.4 LSB/dps로 폴백하고 defaulted=True.
    Unknown/None falls back to the most-sensitive 16.4 LSB/dps and defaulted=True.
    """
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
    """PMU_STATUS에서 (가속, 자이로) 전원 모드 이름 디코드. / Decode (accel, gyro) PMU mode names from PMU_STATUS.

    자이로 모드는 비트 2~3, 가속 모드는 비트 4~5에 들어있다(2비트씩). 반환/Returns: (accel, gyro).
    Gyro mode lives in bits 2-3, accel in bits 4-5 (two bits each).
    """
    names = {
        0x00: "SUSPEND",
        0x01: "NORMAL",
        0x02: "LOW_POWER_OR_FAST_START",
        0x03: "RESERVED",
    }
    gyro = names[(pmu_status >> 2) & 0x03]
    accel = names[(pmu_status >> 4) & 0x03]
    return accel, gyro


# ── 센서 스냅샷 모델 / Sensor snapshot model ──
@dataclass(frozen=True)
class Bmi160Sample:
    """BMI160 레지스터 한 스냅샷(불변). 크기값은 프로퍼티로 파생. / One immutable BMI160 register snapshot; magnitudes derived via properties.

    필드 중 chip_id/err_reg/pmu_status는 읽기 실패 시 None일 수 있다. acc_range/gyr_range는 스케일
    계산에 쓰이며 기본값은 각각 0x03(±2g), 0x00(±2000dps).
    """

    chip_id: int | None
    err_reg: int | None
    pmu_status: int | None
    accel_raw: tuple[int, int, int]
    gyro_raw: tuple[int, int, int]
    acc_range: int | None = 0x03
    gyr_range: int | None = 0x00

    @property
    def accel_mag_g(self) -> float:
        """가속도 벡터 크기(g). / Accelerometer vector magnitude in g. 정지 시 ≈1.0(중력). """
        scale, _ = bmi160_accel_lsb_per_g(self.acc_range)
        ax, ay, az = (axis / scale for axis in self.accel_raw)
        return math.sqrt(ax * ax + ay * ay + az * az)

    @property
    def gyro_mag_dps(self) -> float:
        """자이로 벡터 크기(dps). / Gyro vector magnitude in deg/s. 정지 시 ≈0. """
        scale, _ = bmi160_gyro_lsb_per_dps(self.gyr_range)
        gx, gy, gz = (axis / scale for axis in self.gyro_raw)
        return math.sqrt(gx * gx + gy * gy + gz * gz)

    @property
    def raw_all_zero(self) -> bool:
        """가속+자이로 원시값이 모두 0인가(센서 미기동 징후). / True if all raw axes are 0 (sensor not producing data)."""
        return all(axis == 0 for axis in (*self.accel_raw, *self.gyro_raw))


# ── 펌웨어 미러 판정 / Firmware-mirrored pass/block decision ──
def bmi160_block_reason(sample: Bmi160Sample) -> str:
    """펌웨어의 BMI160 정상모드 통과/차단 판정을 그대로 복제. / Mirror the firmware's BMI160 normal-mode pass/block vocabulary.

    반환/Returns: 실패 원인 문자열(예 "CHIP_ID_NOT_BMI160") 또는 모두 통과 시 "OK". 판정 순서와
    임계값은 펌웨어와의 계약이므로 변경 금지. / Reason string or "OK"; order & thresholds are a
    firmware contract — do not alter.
    """
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
    # 정지 상태 타당성 검사: 중력만 받으면 |a|≈1g, 회전이 없으면 |ω|≈0. 창(0.5~1.5 g, <20 dps)은
    # 펌웨어와 동일한 임계값 — 노이즈는 통과시키되 명백한 이상은 걸러낸다.
    # / Stationary plausibility: |a|≈1g under gravity, |ω|≈0 when still. The (0.5-1.5 g, <20 dps)
    #   window matches the firmware — tolerant of noise but rejects clearly bad data.
    accel_ok = 0.5 <= sample.accel_mag_g <= 1.5
    gyro_ok = sample.gyro_mag_dps < 20.0
    if not (accel_ok and gyro_ok):
        return "RAW_DATA_NOT_PLAUSIBLE"
    return "OK"
