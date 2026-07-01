"""무동작 헤딩 준비도 로그 오프라인 분석기 / Offline no-motion heading-readiness log analyzer.

목적/역할:
    "손으로 들고 다니며(hand-carry)" 수집한 통합 무동작(no-motion) 로그를 읽어, 실제 모터를
    돌리기 전에 **자율주행 헤딩(진행방향) 추정이 준비되었는지**를 하드웨어 없이 판정한다.
    모터 안전 게이트가 닫혀 있는지, 센서 기준선(GPS 품질·IMU·수동 RC)이 정상인지, GPS 코스
    래치가 확보됐는지를 종합해 하나의 ``verdict`` 와 다음 행동 ``next_action`` 을 산출한다.

시스템 내 위치:
    - CLI 진입점: ``python tools/analyze_heading_readiness_log.py <log>`` → ``main()``.
      쉘 래퍼 ``scripts/check_outdoor_heading_log.sh`` 가 이 CLI 를 호출한다.
    - 테스트 결합: ``tests/test_analyze_heading_readiness_log.py`` 가 ``parse_usbdbg_rows`` 와
      ``summarize`` 를 import 한다 → 이 두 함수의 시그니처/출력 형식은 계약이다.
    - 의존: 로컬 평면 좌표 변환을 위해 ``gps_coverage_core.geo`` 의 ``GeoPoint``,
      ``latlon_to_local`` 을 사용. ``_bootstrap`` 은 sys.path 세팅용(직접 실행/패키지 실행 겸용).

핵심 개념·불변식:
    - 입력은 ``USBDBG `` 로 시작하는 줄들의 ``key=value`` 토큰. ``parse_usbdbg_rows`` 가 각 줄을
      dict 로, 로그 전체를 dict 리스트(rows)로 만든다.
    - 판정 우선순위(가장 심각한 것 먼저): 모터 안전 실패 → 센서 기준선 실패 → 헤딩 준비/코스
      래치/코스 수락 → 이동량 충분 → 기준선 통과. ``_verdict`` 의 if 순서가 곧 우선순위다.
    - 이 도구는 **읽기 전용**이다. 어떤 결론이 나와도 다음 단계는 항상 "오프라인 경로
      미리보기(preview-only)" 이며 모터를 돌리라고 지시하지 않는다.

사용법/진입점:
    ``summarize(rows)`` 가 사람이 읽는 ``key=value`` 요약 줄들을 리스트로 돌려주고, ``main()``
    은 그것을 한 줄씩 출력한 뒤 항상 0을 반환한다.

리팩토링 노트:
    임계값 상수(BBOX/ PATH 최소 이동, 위성/HDOP 컷)는 모두 모듈 상단에 모아 둠. 판정 로직을
    바꿀 때는 ``_verdict`` 의 우선순위와 ``_next_action`` 의 대응 문자열을 짝으로 유지할 것.
    ``summarize`` 출력 형식을 바꾸면 위 테스트가 깨진다.

English:
    Offline analyzer that decides whether autonomous-heading estimation is ready from a
    hand-carried, no-motion integrated log, without touching hardware. Combines motor-safety
    gates, sensor baseline (GPS quality / IMU / manual RC), and GPS course latch into a single
    ``verdict`` plus a recommended ``next_action``. CLI entry ``main()`` (wrapped by
    scripts/check_outdoor_heading_log.sh); ``parse_usbdbg_rows`` and ``summarize`` are imported by
    tests, so their signatures/output format are a contract. Read-only: every verdict leads only to
    offline preview, never to running motors. ``_verdict``'s if-order encodes severity priority.
"""
from __future__ import annotations

import argparse
import math
from collections import Counter
from pathlib import Path
from typing import Sequence

# _bootstrap 은 리포지토리 루트를 sys.path 에 넣어 gps_coverage_core import 를 가능케 한다.
# 패키지(python -m)로도, 스크립트(직접 실행)로도 돌 수 있게 두 경로를 모두 시도한다.
# / _bootstrap puts the repo root on sys.path; try both package and script import paths.
try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401

from gps_coverage_core.geo import GeoPoint, latlon_to_local

# ── 판정 임계값·허용 집합 / Verdict thresholds and allow-sets ──
# 무동작 판정에서 허용되는 "모터가 막힌 이유". 이 집합 밖 이유가 있으면 안전 실패로 본다.
# / Motor-block reasons that are acceptable in a no-motion run; anything else => unsafe.
ALLOWED_MOTOR_BLOCK_REASONS = {"COMPILE_GATE_OFF", "MOTOR_OUTPUT_DISABLED"}
# 이동 "충분" 판정 임계값. / thresholds for the "moved enough" check.
# 최소 바운딩박스 대각(m). / min bbox diagonal (m).
HEADING_MOVEMENT_BBOX_MIN_M = 10.0
# 최소 누적 경로 길이(m). / min cumulative path length (m).
HEADING_MOVEMENT_PATH_MIN_M = 20.0


# ── 로그 파싱 / Log parsing ──
def parse_usbdbg_rows(text: str) -> list[dict[str, str]]:
    """로그 텍스트에서 ``USBDBG `` 줄들을 dict 리스트로 파싱. / Parse ``USBDBG `` lines into row dicts.

    각 줄의 ``key=value`` 토큰을 dict 로 만든다. 이 함수는 테스트가 직접 import 하는 공개 API.
    / Turns each line's ``key=value`` tokens into a dict. Imported by tests, so it is public API.
    """
    rows: list[dict[str, str]] = []
    for line in text.splitlines():
        # 접두어 'USBDBG '(뒤 공백 포함)로 시작하는 줄만 계량 로그로 인정. / require the exact prefix
        if not line.startswith("USBDBG "):
            continue
        row: dict[str, str] = {}
        for token in line.split():
            if "=" in token:
                key, value = token.split("=", 1)
                row[key] = value
        rows.append(row)
    return rows


# ── 행 집계 헬퍼 / Row-aggregation helpers ──
def _float_values(rows: Sequence[dict[str, str]], field: str) -> list[float]:
    """해당 필드의 유효 실수만 모아 반환('' 와 'NA' 는 제외). / Collect finite floats for a field.

    빈 값·``NA``·파싱 불가 토큰은 조용히 건너뛴다. / Empty, ``NA``, or unparsable values are skipped.
    """
    values: list[float] = []
    for row in rows:
        value = row.get(field)
        if not value or value == "NA":
            continue
        try:
            values.append(float(value))
        except ValueError:
            continue
    return values


def _counter(rows: Sequence[dict[str, str]], field: str) -> Counter[str]:
    """필드 값의 도수분포(없으면 'MISSING'). / Value histogram for a field ('MISSING' when absent)."""
    return Counter(row.get(field, "MISSING") for row in rows)


def _true_count(rows: Sequence[dict[str, str]], field: str) -> int:
    """필드 값이 'true' 인 행 수. / Number of rows where the field equals 'true'."""
    return sum(1 for row in rows if row.get(field) == "true")


def _any_true(rows: Sequence[dict[str, str]], field: str) -> bool:
    """필드가 'true' 인 행이 하나라도 있는가. / Whether any row has the field set to 'true'."""
    return _true_count(rows, field) > 0


def _range_text(values: Sequence[float]) -> str:
    """값들의 'min..max' 문자열(빈 값은 'NA'). / 'min..max' text for values ('NA' when empty)."""
    if not values:
        return "NA"
    return f"{min(values):.3f}..{max(values):.3f}"


# ── 이동량·리셋 이벤트 분석 / Movement and reset-event analysis ──
def _movement_summary(rows: Sequence[dict[str, str]]) -> dict[str, float | int | str]:
    """hand-carry 이동량을 로컬 평면에서 요약. / Summarize hand-carry movement in a local plane.

    첫 좌표를 원점으로 로컬 (x,y) 변환 후 바운딩박스 span·대각, 누적 경로 길이, 최대 스텝을
    계산한다. 대각이나 누적 경로가 임계값 이상이면 ``moved_enough='true'`` — 즉 코스 추정에
    쓸 만큼 실제로 움직였는지의 대략적 판단이다.
    / Projects to a local plane about the first point, then reports bbox spans/diagonal, cumulative
    path length, and max step. ``moved_enough`` is 'true' when the diagonal or the cumulative path
    exceeds the thresholds — a coarse check that the carry actually moved enough for course estimation.
    """
    points: list[GeoPoint] = []
    for row in rows:
        try:
            points.append(GeoPoint(lat=float(row["current_lat"]), lon=float(row["current_lon"])))
        except (KeyError, ValueError):
            continue
    if not points:
        return {
            "point_count": 0,
            "bbox_x_span_m": 0.0,
            "bbox_y_span_m": 0.0,
            "bbox_diag_m": 0.0,
            "cumulative_path_m": 0.0,
            "max_step_m": 0.0,
            "moved_enough": "false",
        }

    # 첫 fix 를 원점으로 로컬 좌표 변환(작은 스케일이라 평면 근사 OK). / origin = first point, local plane
    origin = points[0]
    local_points = [latlon_to_local(origin, point) for point in points]
    xs = [point.x_m for point in local_points]
    ys = [point.y_m for point in local_points]
    bbox_x_span_m = max(xs) - min(xs)
    bbox_y_span_m = max(ys) - min(ys)
    bbox_diag_m = math.hypot(bbox_x_span_m, bbox_y_span_m)
    steps = [
        math.hypot(curr.x_m - prev.x_m, curr.y_m - prev.y_m)
        for prev, curr in zip(local_points, local_points[1:])
    ]
    cumulative_path_m = sum(steps)
    max_step_m = max(steps) if steps else 0.0
    return {
        "point_count": len(points),
        "bbox_x_span_m": bbox_x_span_m,
        "bbox_y_span_m": bbox_y_span_m,
        "bbox_diag_m": bbox_diag_m,
        "cumulative_path_m": cumulative_path_m,
        "max_step_m": max_step_m,
        "moved_enough": (
            "true"
            if bbox_diag_m >= HEADING_MOVEMENT_BBOX_MIN_M
            or cumulative_path_m >= HEADING_MOVEMENT_PATH_MIN_M
            else "false"
        ),
    }


def _reset_like_events(course_displacements: Sequence[float]) -> list[tuple[int, float, float]]:
    """코스 변위가 급락한 지점(앵커 리셋 추정)을 찾는다. / Detect sharp drops in course displacement.

    직전 값이 ≥1.5 인데 현재 값이 ≤0.25 로 떨어지면 앵커 리시드(reseed)로 코스가 초기화된
    "리셋 유사" 이벤트로 본다. 반환: (인덱스, 직전값, 현재값) 목록.
    / A prev≥1.5 → curr≤0.25 drop looks like an anchor reseed that reset the accumulated course.
    Returns a list of (index, previous, current).
    """
    events: list[tuple[int, float, float]] = []
    for index, (previous, current) in enumerate(
        zip(course_displacements, course_displacements[1:]), start=1
    ):
        if previous >= 1.5 and current <= 0.25:
            events.append((index, previous, current))
    return events


def _numeric_nonzero(values: Sequence[float], epsilon: float = 1e-6) -> bool:
    """부동소수 오차를 감안해 0이 아닌 값이 하나라도 있는지. / Any value nonzero within epsilon."""
    return any(abs(value) > epsilon for value in values)


# ── 안전·센서 게이트 판정 / Safety and sensor-baseline gates ──
def _motor_safety_ok(rows: Sequence[dict[str, str]]) -> bool:
    """모터가 실제로 막혀 있었는지 검증. / Verify motors were genuinely blocked throughout.

    조건(모두 만족해야 True): 물리 출력이 한 번도 활성 아님, 최종 좌/우 명령이 항상 0,
    관측된 차단 이유가 존재하고 모두 ``ALLOWED_MOTOR_BLOCK_REASONS`` 안에 있음. 하나라도
    어기면 무동작 안전 실패. / All must hold: physical output never active, final L/R commands
    all zero, and every observed block reason is within the allowed set. Otherwise: unsafe.
    """
    if not rows:
        return False
    if any(row.get("physical_output_active") == "true" for row in rows):
        return False

    left_values = _float_values(rows, "final_left_cmd")
    right_values = _float_values(rows, "final_right_cmd")
    if _numeric_nonzero(left_values) or _numeric_nonzero(right_values):
        return False

    block_reasons = {
        row["physical_block_reason"] for row in rows if "physical_block_reason" in row
    }
    return bool(block_reasons) and block_reasons.issubset(ALLOWED_MOTOR_BLOCK_REASONS)


def _gps_quality_ok(rows: Sequence[dict[str, str]]) -> bool:
    """GPS 품질 기준선 충족 여부. / Whether GPS quality meets the baseline.

    차단 이유가 모두 'OK' 이어야 하고, (관측됐다면) 최대 위성 수 ≥4, 최소 HDOP ≤6.0 이어야
    한다. sats/hdop 값이 아예 없으면 해당 컷은 통과로 간주(있을 때만 검사).
    / Requires all block reasons 'OK', and (when present) max sats ≥4 and min HDOP ≤6.0. Absent
    sats/hdop values pass that cut (checked only when available).
    """
    if not rows:
        return False
    gps_reasons = {row.get("gps_block_reason") for row in rows if "gps_block_reason" in row}
    if not gps_reasons or any(reason != "OK" for reason in gps_reasons):
        return False

    sats_values = _float_values(rows, "gps_sats")
    hdop_values = _float_values(rows, "gps_hdop")
    if sats_values and max(sats_values) < 4:
        return False
    if hdop_values and min(hdop_values) > 6.0:
        return False
    return True


def _bmi160_valid(row: dict[str, str]) -> bool:
    """한 행이 정상 BMI160 IMU 상태를 나타내는지. / Whether a single row shows a healthy BMI160 IMU.

    IMU 타입이 BMI160 이고, 존재/전원(PMU)/데이터 타당성 플래그가 'false' 가 아니며, 칩 ID 가
    없거나 예상값(0xD1)일 때 유효. / Valid when type is BMI160, the present/PMU/plausible flags are
    not 'false', and the chip id is absent or the expected 0xD1.
    """
    if row.get("imu_type") != "BMI160":
        return False
    if row.get("imu_present") == "false":
        return False
    chip_id = row.get("imu_chip_id")
    if chip_id not in {None, "0xD1"}:
        return False
    if row.get("imu_pmu_normal") == "false":
        return False
    if row.get("imu_data_plausible") == "false":
        return False
    return True


def _imu_ok(rows: Sequence[dict[str, str]]) -> bool:
    """로그 중 한 행이라도 정상 BMI160 이면 IMU OK. / IMU OK if any row shows a healthy BMI160."""
    if not rows:
        return False
    return any(_bmi160_valid(row) for row in rows)


def _rc_manual_ok(rows: Sequence[dict[str, str]]) -> bool:
    """수동 RC 기준선(정상 조종 수신) 관측 여부. / Whether a valid manual-RC baseline was observed.

    rc_ok=true 이면서 mode=MANUAL 이고 control_source=RC_MANUAL 인 행이 하나라도 있으면 True.
    / True if any row has rc_ok=true, mode=MANUAL, and control_source=RC_MANUAL together.
    """
    return any(
        row.get("rc_ok") == "true"
        and row.get("mode") == "MANUAL"
        and row.get("control_source") == "RC_MANUAL"
        for row in rows
    )


# ── 코스/헤딩 카운트 헬퍼 / Course/heading count helpers ──
def _heading_ready_count(rows: Sequence[dict[str, str]]) -> int:
    """heading_ready=true 인 샘플 수. / Count of samples with heading_ready=true."""
    return _true_count(rows, "heading_ready")


def _accepted_course_count(rows: Sequence[dict[str, str]]) -> int:
    """GPS 코스가 "수락"된 정도(두 신호의 최댓값). / How often a GPS course was accepted (max of two signals).

    앵커 리셋 이유가 재시드-수락이거나, 마지막 수락 코스가 유효(latch)한 두 카운트 중 큰 값을
    쓴다. / Max of: anchor-reset reason == accepted-reseed, and last-accepted-course valid (latch).
    """
    accepted_by_reset = sum(
        1 for row in rows if row.get("gps_course_anchor_reset_reason") == "COURSE_ACCEPTED_RESEED"
    )
    accepted_by_latch = _true_count(rows, "gps_course_last_accepted_valid")
    return max(accepted_by_reset, accepted_by_latch)


def _heading_diag_partial_count(rows: Sequence[dict[str, str]]) -> int:
    """수락 코스가 있고 헤딩 일치오차도 관측된 샘플 수(부분 진단). / Rows with accepted course AND agreement error.

    현재 요약 출력에는 직접 쓰이지 않는 보조 진단용 카운트다.
    / An auxiliary diagnostic count, not directly emitted by the current summary.
    """
    return sum(
        1
        for row in rows
        if row.get("gps_course_last_accepted_valid") == "true"
        and row.get("heading_agreement_error_deg") not in {None, "NA"}
    )


def _course_latch_available_count(rows: Sequence[dict[str, str]]) -> int:
    """출력 가능한 코스 래치가 확보된 정도. / Extent to which an output-ready course latch exists.

    출력 유효 플래그 true 수와 출력 코스(deg) 실수 개수 중 큰 값. / Max of output-valid true-count
    and the number of numeric output-course(deg) values.
    """
    return max(
        _true_count(rows, "gps_course_output_valid"),
        len(_float_values(rows, "gps_course_output_deg")),
    )


# ── 종합 판정 / Overall verdict ──
def _sensor_baseline_fail_reason(rows: Sequence[dict[str, str]]) -> str:
    """센서 기준선 실패 사유를 우선순위대로 반환('OK' 면 통과). / First sensor-baseline failure, else 'OK'.

    순서: GPS 품질 → IMU → 수동 RC. 먼저 걸리는 것을 반환. / Order: GPS quality → IMU → manual RC.
    """
    if not _gps_quality_ok(rows):
        return "GPS_QUALITY_FAIL"
    if not _imu_ok(rows):
        return "IMU_FAIL"
    if not _rc_manual_ok(rows):
        return "RC_MANUAL_FAIL"
    return "OK"


def _verdict(rows: Sequence[dict[str, str]], movement: dict[str, float | int | str]) -> str:
    """전체 로그에 대한 단일 판정 문자열 산출. / Compute the single verdict string for the whole log.

    if 순서가 곧 심각도 우선순위다(위가 더 심각/우선): 모터 안전 실패 → 센서 기준선 실패 →
    헤딩 준비 통과 → 코스 래치 확보 → 코스 수락됐으나 미출력 → 이동량은 충분(헤딩 미준비) →
    센서 기준선만 통과. 이 순서를 바꾸면 판정 의미가 달라지므로 주의.
    / The if-order encodes severity priority (top = most severe/preferred). Reordering changes
    the meaning of the verdict, so change with care.
    """
    if not _motor_safety_ok(rows):
        return "MOTOR_SAFETY_FAIL"
    if _sensor_baseline_fail_reason(rows) != "OK":
        return "SENSOR_BASELINE_FAIL"
    if _heading_ready_count(rows) > 0:
        return "HEADING_READY_PASS"
    if _course_latch_available_count(rows) > 0:
        return "HEADING_COURSE_LATCH_AVAILABLE"
    if _accepted_course_count(rows) > 0:
        return "HEADING_COURSE_ACCEPTED_BUT_NOT_OUTPUT"
    if movement["moved_enough"] == "true":
        return "HEADING_NOT_READY_BUT_MOVEMENT_SUFFICIENT"
    return "SENSOR_BASELINE_PASS"


def _next_action(verdict: str) -> str:
    """판정에 대응하는 권장 다음 행동 문자열. / Map a verdict to its recommended next action.

    ``_verdict`` 의 각 결과와 1:1로 짝지어야 한다(둘을 함께 유지). 어떤 경우에도 다음 단계는
    오프라인 미리보기까지이며 모터 구동을 지시하지 않는다.
    / Must stay 1:1 with the outcomes of ``_verdict``. No branch ever tells the operator to run
    motors; the furthest step is offline preview.
    """
    if verdict == "MOTOR_SAFETY_FAIL":
        return "ABORT_SAFETY_INSPECT_NO_MOTION_GATES"
    if verdict == "SENSOR_BASELINE_FAIL":
        return "FIX_SENSOR_OR_RC_BASELINE_BEFORE_HEADING"
    if verdict == "HEADING_READY_PASS":
        return "PROCEED_TO_OFFLINE_PATH_PLANNING_PREVIEW_ONLY"
    if verdict == "HEADING_COURSE_LATCH_AVAILABLE":
        return "REVIEW_HEADING_AGREEMENT_AND_PROCEED_TO_OFFLINE_PREVIEW_ONLY"
    if verdict == "HEADING_COURSE_ACCEPTED_BUT_NOT_OUTPUT":
        return "REVIEW_COURSE_OUTPUT_BLOCK_REASON"
    if verdict == "HEADING_NOT_READY_BUT_MOVEMENT_SUFFICIENT":
        return "USE_NEW_DIAGNOSTIC_FIRMWARE_AND_REPEAT_HAND_CARRY"
    return "REPEAT_HAND_CARRY_WITH_LONGER_STRAIGHT_SEGMENT"


def summarize(rows: Sequence[dict[str, str]], total_line_count: int | None = None) -> list[str]:
    """행들을 사람이 읽는 ``key=value`` 요약 줄 리스트로 만든다. / Build human-readable summary lines.

    도수분포·수치 범위·이동량·리셋 이벤트·코스 카운트를 모은 뒤 최종적으로 ``verdict`` 와
    ``next_action`` 을 덧붙인다. 부수효과 없음(순수 함수). 이 함수의 **출력 형식은 테스트
    계약**이므로 키 이름/순서를 바꿀 때는 tests/test_analyze_heading_readiness_log.py 를 함께
    갱신할 것. / Aggregates histograms, numeric ranges, movement, reset events, and course counts,
    then appends ``verdict`` and ``next_action``. Pure (no side effects). The output format is a
    test contract; update the test if you change key names/order.
    """
    lines: list[str] = []
    if total_line_count is not None:
        lines.append(f"total_line_count={total_line_count}")
    lines.append(f"usbdbg_rows={len(rows)}")
    # ── 범주형 필드 도수분포 / Categorical-field histograms ──
    for field in (
        "gps_block_reason",
        "position_source",
        "heading_agreement_diag",
        "path_following_block_reason",
        "gps_course_anchor_reset_reason",
        "gps_course_block_reason",
        "gps_course_output_block_reason",
        "imu_type",
        "imu_present",
        "imu_i2c_addr",
        "imu_chip_id",
        "imu_pmu_normal",
        "imu_data_plausible",
        "physical_block_reason",
        "physical_output_active",
        "mode",
        "control_source",
        "rc_ok",
        "neutral_ok",
    ):
        lines.append(f"{field}_counts={dict(_counter(rows, field))}")

    # ── 수치형 필드 범위 / Numeric-field ranges ──
    for field in (
        "gps_sats",
        "gps_hdop",
        "course_displacement_m",
        "estimated_course_deg",
        "gps_course_candidate_deg",
        "gps_course_last_accepted_deg",
        "gps_course_last_accepted_age_ms",
        "gps_course_last_accepted_displacement_m",
        "gps_course_deg",
        "gps_course_output_deg",
        "gps_course_output_age_ms",
        "heading_agreement_error_deg",
        "imu_accel_mag_g",
        "imu_gyro_mag_dps",
        "final_left_cmd",
        "final_right_cmd",
    ):
        values = _float_values(rows, field)
        lines.append(f"{field}_range={_range_text(values)} count={len(values)}")

    # ── 코스 변위·추정 코스 다양성 / Course displacement and estimated-course diversity ──
    course_values = _float_values(rows, "course_displacement_m")
    estimated_values = _float_values(rows, "estimated_course_deg")
    # 0.1도로 반올림해 중복 제거: 추정 코스가 몇 가지나 관측됐는지(다양성)를 본다. / de-dup at 0.1deg
    unique_estimated = sorted({round(value, 1) for value in estimated_values})
    lines.append(f"max_course_displacement_m={max(course_values):.3f}" if course_values else "max_course_displacement_m=NA")
    lines.append(f"estimated_course_deg_unique_sample_count={len(unique_estimated)}")
    if unique_estimated:
        preview = ",".join(f"{value:.1f}" for value in unique_estimated[:16])
        lines.append(f"estimated_course_deg_unique_preview={preview}")

    # ── 이동량 요약 병합 / Merge movement summary ──
    movement = _movement_summary(rows)
    for key, value in movement.items():
        if isinstance(value, float):
            lines.append(f"{key}={value:.3f}")
        else:
            lines.append(f"{key}={value}")

    # ── 리셋 유사 이벤트 / Reset-like events ──
    course_displacements = _float_values(rows, "course_displacement_m")
    reset_events = _reset_like_events(course_displacements)
    lines.append(f"reset_like_event_count={len(reset_events)}")
    if reset_events:
        preview = ", ".join(f"idx{idx}:{prev:.2f}->{cur:.2f}" for idx, prev, cur in reset_events[:12])
        lines.append(f"reset_like_events_preview={preview}")

    # ── 코스 래치·판정 요약 / Course-latch counts and final verdict ──
    gps_course_count = len(_float_values(rows, "gps_course_deg"))
    gps_course_output_count = len(_float_values(rows, "gps_course_output_deg"))
    course_latch_available_count = _course_latch_available_count(rows)
    accepted_course_count = _accepted_course_count(rows)
    heading_ready_true_count = _true_count(rows, "heading_ready")
    lines.append(f"gps_course_deg_non_na_count={gps_course_count}")
    lines.append(f"gps_course_output_deg_non_na_count={gps_course_output_count}")
    lines.append(f"gps_course_output_valid_true_count={_true_count(rows, 'gps_course_output_valid')}")
    lines.append(f"course_latch_available_count={course_latch_available_count}")
    lines.append(f"accepted_course_count={accepted_course_count}")
    lines.append(
        "accepted_course_without_output="
        f"{str(accepted_course_count > 0 and course_latch_available_count == 0).lower()}"
    )
    lines.append(f"heading_ready_true_count={heading_ready_true_count}")
    lines.append(f"baseline_fail_reason={_sensor_baseline_fail_reason(rows)}")
    lines.append(f"bmi160_valid={str(_imu_ok(rows)).lower()}")
    lines.append(f"rc_manual_ok={str(_rc_manual_ok(rows)).lower()}")
    lines.append(f"motor_safety_ok={str(_motor_safety_ok(rows)).lower()}")
    lines.append(f"gps_quality_ok={str(_gps_quality_ok(rows)).lower()}")
    verdict = _verdict(rows, movement)
    lines.append(f"verdict={verdict}")
    lines.append(f"next_action={_next_action(verdict)}")
    # 변위는 있었는데 코스가 하나도 안 잡힌 흔한 원인을 설명으로 남긴다(자주 헷갈리는 지점).
    # / Explain the common "displacement but no course" case (a frequent point of confusion).
    if gps_course_count == 0 and course_displacements:
        lines.append(
            "likely_block_reason=course heading is computed only briefly when the segment crosses "
            "the displacement threshold; the anchor then reseeds and the debug sample sees NO_HEADING"
        )
    return lines


# ── CLI 진입점 / CLI entry point ──
def build_parser() -> argparse.ArgumentParser:
    """CLI 인자 파서 생성(로그 파일 1개). / Build the CLI parser (a single log file)."""
    parser = argparse.ArgumentParser(
        description="Analyze integrated no-motion heading readiness logs without touching hardware."
    )
    parser.add_argument("log", type=Path)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """로그 파일을 읽어 요약 줄을 출력. 항상 0 반환. / Read the log, print summary lines; always returns 0.

    판정 결과는 종료 코드가 아니라 출력 텍스트(``verdict=...``)로 전달된다. / The verdict is
    conveyed as output text (``verdict=...``), not via the exit code.
    """
    args = build_parser().parse_args(argv)
    text = args.log.read_text(encoding="utf-8", errors="ignore")
    rows = parse_usbdbg_rows(text)
    for line in summarize(rows, total_line_count=len(text.splitlines())):
        print(line)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
