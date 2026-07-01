"""공유 텔레메트리 헬퍼와 행(row) 접근자의 파싱 규칙을 고정하는 테스트.

목적/역할 (KO):
    ``telemetry`` 모듈이 USB 디버그 로그 행(dict[str, str]) 에서 값을 뽑을 때
    쓰는 저수준 파서와 필드 접근자의 계약을 못 박는다. 이 헬퍼들은 안전
    판정(safety) 과 리포팅의 입력을 정규화하므로, 파싱 규칙이 흔들리면 상위
    로직 전체가 조용히 오작동한다.

핵심 계약·불변식 (KO):
    - ``parse_usbdbg_rows`` 는 ``station_path_package_tracker`` 의 것을 *재수출*
      한 것이지 복사본이 아니다(``is`` 동일성). 파서 이원화로 인한 drift 방지.
    - ``_parse_bool``: "1/true/yes/y/ok/active" 류를 참으로, "0/false/no/빈칸/NA"
      를 거짓으로 본다. ``None`` 은 ``default`` 값을 따른다.
    - ``_optional_float``: 빈칸·NA·NaN·none·null·``None`` 및 무한대(inf)·비수치는
      모두 ``None`` 으로 안전 처리한다(예외를 던지지 않음).
    - ``_latest``: 행 목록에서 해당 키가 *존재하는 마지막* 값을 반환하며, 없으면
      ``default``.
    - ``_fmt``: 기본 6 자리 포맷, 자릿수 지정 가능, ``None`` 은 ``"NA"``.
    - 필드 접근자(event/final_*_cmd/gps_* 등): 위 파서 위에서 동작하며, 값이 없는
      빈 행에서는 각자의 안전 기본값(예: ``""``, ``None``, ``False``, ``"NA"``)을 낸다.

Purpose (EN):
    Locks the parsing contract of the shared telemetry helpers and row
    accessors: the truthy/NA-token rules, ``inf``/non-numeric -> ``None``,
    last-present-value lookup, the six-digit ``_fmt``, and the per-accessor
    safe defaults on an empty row. Also asserts ``parse_usbdbg_rows`` is a
    re-export (identity), not a drifting copy.
"""
from __future__ import annotations

from tools.physical_path_planning import telemetry
from tools import station_path_package_tracker


def test_parse_usbdbg_rows_is_reexport_not_copy() -> None:
    """행 파서는 tracker 것을 재수출한 동일 객체이다(복사본 아님) / the row parser
    is re-exported from the tracker (identity), not a divergent copy."""
    assert telemetry.parse_usbdbg_rows is station_path_package_tracker.parse_usbdbg_rows


def test_parse_bool_truthy_tokens() -> None:
    """불리언 토큰 파싱 규칙과 None 기본값 처리 / boolean token parsing rules and the
    ``None`` -> ``default`` behaviour."""
    for token in ("1", "true", "YES", "y", "ok", "active"):
        assert telemetry._parse_bool(token) is True
    for token in ("0", "false", "no", "", "NA"):
        assert telemetry._parse_bool(token) is False
    assert telemetry._parse_bool(None, default=True) is True


def test_optional_float_handles_na_tokens_and_inf() -> None:
    """NA류 토큰·무한대·비수치는 모두 None 으로 안전 처리 / NA-like tokens, ``inf``
    and non-numeric strings all safely coerce to ``None``."""
    assert telemetry._optional_float("1.5") == 1.5
    for na in ("", "NA", "NaN", "none", "null", None):
        assert telemetry._optional_float(na) is None
    assert telemetry._optional_float("inf") is None
    assert telemetry._optional_float("not-a-number") is None


def test_latest_returns_last_present_value() -> None:
    """행 목록에서 키가 존재하는 마지막 값을, 없으면 기본값을 반환 / returns the
    last row where the key is present, else the supplied default."""
    rows = [{"k": "a"}, {"k": "b"}, {"other": "c"}]
    assert telemetry._latest(rows, "k") == "b"
    assert telemetry._latest(rows, "missing", default="X") == "X"


def test_fmt_default_six_digits_and_na() -> None:
    """숫자 포맷: 기본 6 자리, 자릿수 지정 가능, None 은 "NA" / number formatting:
    six digits by default, configurable precision, ``None`` renders ``"NA"``."""
    assert telemetry._fmt(0.123456789) == "0.123457"
    assert telemetry._fmt(0.123456789, 3) == "0.123"
    assert telemetry._fmt(None) == "NA"


def test_row_accessors() -> None:
    """채워진 행에서 각 필드 접근자가 올바른 타입·값으로 값을 뽑는다 / on a populated
    row each field accessor extracts the right typed value."""
    row = {
        "event": "stop",
        "final_left_cmd": "0.12",
        "final_right_cmd": "NA",
        "physical_output_active": "yes",
        "imu_relative_yaw_deg": "12.5",
        "gps_block_reason": "BAD_HDOP",
        "gps_sats": "7",
        "gps_hdop": "1.8",
    }
    assert telemetry.event(row) == "STOP"
    assert telemetry.final_left_cmd(row) == 0.12
    assert telemetry.final_right_cmd(row) is None
    assert telemetry.physical_output_active(row) is True
    assert telemetry.imu_relative_yaw_deg(row) == 12.5
    assert telemetry.gps_block_reason(row) == "BAD_HDOP"
    assert telemetry.gps_sats(row) == 7.0
    assert telemetry.gps_hdop(row) == 1.8


def test_accessors_default_on_empty_row() -> None:
    """빈 행에서는 각 접근자가 안전 기본값을 낸다 / on an empty row each accessor
    yields its safe default (``""`` / ``None`` / ``False`` / ``"NA"``)."""
    assert telemetry.event({}) == ""
    assert telemetry.final_left_cmd({}) is None
    assert telemetry.physical_output_active({}) is False
    assert telemetry.gps_block_reason({}) == "NA"
