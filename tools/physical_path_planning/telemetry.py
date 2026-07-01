"""Shared USBDBG telemetry parsing and canonical row-field accessors.

This is the single home for the scalar coercion helpers (``_parse_bool``,
``_optional_float``, ``_latest``, ``_fmt``) and the named accessors for the
USBDBG telemetry fields that the executor / safety / controller layers read.
The raw line parser ``parse_usbdbg_rows`` is re-exported (not moved) from
``station_path_package_tracker``, which keeps its own large test surface.

Accessors take a single already-parsed row (``dict[str, str]``) and return a
normalized value, so call sites stop re-deriving field names and coercions.

목적/역할 (KO):
    USBDBG 텔레메트리 라인 파싱과 "정규(canonical)" 필드 접근자의 단일 집이다.
    로버가 직렬로 흘려보내는 텔레메트리는 문자열 필드로 도착하므로, 스칼라 강제
    변환 헬퍼(``_parse_bool``/``_optional_float``/``_latest``/``_fmt``)와,
    executor/safety/controller 가 읽는 개별 필드를 이름 붙은 함수로 감싼 접근자를
    여기 모아 둔다. 이렇게 하면 호출부마다 필드 이름과 형 변환을 재구현하지 않아도
    되고, 필드명이 바뀌어도 이 파일 한 곳만 고치면 된다.

시스템 내 위치 (KO):
    잎(leaf) 모듈. 원시 라인 파서 ``parse_usbdbg_rows`` 는 여기서 재구현하지 않고
    ``station_path_package_tracker`` 에서 그대로 재노출(re-export)한다 -- 그쪽이
    이미 큰 테스트 표면을 갖고 있어 파서 로직의 단일 출처를 유지한다. safety 는
    이 모듈의 헬퍼/접근자 위에 순수 술어(predicate)를 쌓고, executor 는 접근자로
    이벤트/명령 필드를 읽는다.

핵심 개념·함정 (KO):
    - 접근자는 "이미 파싱된 한 행"(``dict[str, str]``)을 받아 정규화된 값을
      돌려준다. 부수효과 없음.
    - ``_optional_float`` 은 빈 문자열·"NA"/"NAN"/"NONE"/"NULL"·비유한(non-finite)
      값을 모두 ``None`` 으로 접는다. 즉 "값 없음"과 "0" 은 반드시 구분된다.
    - ``_parse_bool`` 은 화이트리스트("1/true/yes/y/ok/active")만 참으로 본다.

Purpose (EN):
    Single home for USBDBG telemetry parsing and the canonical per-field
    accessors. Telemetry arrives as string fields over serial, so the scalar
    coercion helpers plus name-bound accessors live here and every call site
    (executor/safety/controller) reads through them instead of re-deriving field
    names. The raw line parser ``parse_usbdbg_rows`` is re-exported (not copied)
    from ``station_path_package_tracker`` to keep one source of truth. Accessors
    are pure: given one parsed row they return a normalized value.
"""
from __future__ import annotations

import math
from typing import Sequence

from tools import station_path_package_tracker

# Re-export the canonical raw parser; do not reimplement it here.
# KO: 파서 단일 출처 유지 -- 여기서 재구현하면 두 파서가 어긋날 위험이 있다.
parse_usbdbg_rows = station_path_package_tracker.parse_usbdbg_rows


def _parse_bool(value: object, default: bool = False) -> bool:
    """Coerce a telemetry string to bool via a truthy whitelist.

    KO: 화이트리스트("1/true/yes/y/ok/active")에 속할 때만 True. ``None`` 이면
    ``default``. 알 수 없는 문자열은 False 로 본다(안전한 기본값).
    EN: True only if the (whitespace-stripped, lower-cased) text is in the
    whitelist; ``None`` -> ``default``; anything else -> False.
    """
    if value is None:
        return default
    return str(value).strip().lower() in {"1", "true", "yes", "y", "ok", "active"}


def _optional_float(value: object) -> float | None:
    """Parse a float, mapping 'no value' sentinels and non-finite to ``None``.

    KO: 빈 문자열/"NA"/"NAN"/"NONE"/"NULL", 파싱 실패, 비유한(inf/nan) 값은 모두
    ``None`` 으로 접는다. 그래서 "값 없음"과 "0.0" 이 절대 뒤섞이지 않는다.
    EN: Empty/NA/NAN/NONE/NULL, parse failure, or non-finite all become ``None``
    so 'missing' is never confused with 0.0.
    """
    if value is None:
        return None
    text = str(value).strip()
    if text.upper() in {"", "NA", "NAN", "NONE", "NULL"}:
        return None
    try:
        parsed = float(text)
    except ValueError:
        return None
    return parsed if math.isfinite(parsed) else None


def _latest(rows: Sequence[dict[str, str]], key: str, default: str = "NA") -> str:
    """Return the most recent row's value for ``key`` (scans newest-first).

    KO: rows 를 뒤에서부터 훑어 ``key`` 를 가진 첫 행의 값을 반환한다. 텔레메트리는
    시간순 append 이므로 "가장 최근" 값을 얻는 방법. 없으면 ``default``.
    EN: Iterates rows in reverse (rows are appended in time order) to get the
    latest value for ``key``; ``default`` if no row has it.
    """
    for row in reversed(rows):
        if key in row:
            return row[key]
    return default


def _fmt(value: float | None, digits: int = 6) -> str:
    """Format an optional float, rendering ``None`` as the literal ``"NA"``.

    KO: 리포트/로그용 문자열화. ``None`` 은 "NA", 그 외는 고정 소수 자리로.
    """
    return "NA" if value is None else f"{value:.{digits}f}"


# ── 정규 단일-행 필드 접근자 / Canonical single-row field accessors ──
# KO: 각 함수는 한 텔레메트리 행에서 한 필드를 이름으로 읽어 정규화한다.
#     여기가 필드명↔의미의 단일 계약이므로, 펌웨어 필드명이 바뀌면 이 블록만 고친다.


def event(row: dict[str, str]) -> str:
    """Upper-cased ``event`` field (ARM / PULSE / ACK / STOP / ...), ``""`` if absent."""
    return str(row.get("event", "")).upper()


def final_left_cmd(row: dict[str, str]) -> float | None:
    """Latest resolved left-motor command from this row / 이 행의 최종 좌측 모터 명령."""
    return _optional_float(row.get("final_left_cmd"))


def final_right_cmd(row: dict[str, str]) -> float | None:
    """Latest resolved right-motor command from this row / 이 행의 최종 우측 모터 명령."""
    return _optional_float(row.get("final_right_cmd"))


def physical_output_active(row: dict[str, str]) -> bool:
    """True if firmware reports motor output physically active / 물리 모터 출력 활성 여부."""
    return _parse_bool(row.get("physical_output_active"))


def imu_relative_yaw_deg(row: dict[str, str]) -> float | None:
    """IMU yaw relative to the pulse/segment start, degrees / 시작 기준 상대 IMU yaw(도)."""
    return _optional_float(row.get("imu_relative_yaw_deg"))


def gps_block_reason(row: dict[str, str], default: str = "NA") -> str:
    """Reason string for a GPS motion block / GPS 이동 차단 사유 문자열 (없으면 ``default``)."""
    value = row.get("gps_block_reason")
    return default if value is None else str(value)


def gps_sats(row: dict[str, str]) -> float | None:
    """Satellite count from this row / 이 행의 위성 수 (없으면 ``None``)."""
    return _optional_float(row.get("gps_sats"))


def gps_hdop(row: dict[str, str]) -> float | None:
    """Horizontal dilution of precision / 수평 정밀도 저하(HDOP) 값 (없으면 ``None``)."""
    return _optional_float(row.get("gps_hdop"))
