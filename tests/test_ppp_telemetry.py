"""Tests for the shared telemetry helpers and row accessors."""
from __future__ import annotations

from tools.physical_path_planning import telemetry
from tools import station_path_package_tracker


def test_parse_usbdbg_rows_is_reexport_not_copy() -> None:
    assert telemetry.parse_usbdbg_rows is station_path_package_tracker.parse_usbdbg_rows


def test_parse_bool_truthy_tokens() -> None:
    for token in ("1", "true", "YES", "y", "ok", "active"):
        assert telemetry._parse_bool(token) is True
    for token in ("0", "false", "no", "", "NA"):
        assert telemetry._parse_bool(token) is False
    assert telemetry._parse_bool(None, default=True) is True


def test_optional_float_handles_na_tokens_and_inf() -> None:
    assert telemetry._optional_float("1.5") == 1.5
    for na in ("", "NA", "NaN", "none", "null", None):
        assert telemetry._optional_float(na) is None
    assert telemetry._optional_float("inf") is None
    assert telemetry._optional_float("not-a-number") is None


def test_latest_returns_last_present_value() -> None:
    rows = [{"k": "a"}, {"k": "b"}, {"other": "c"}]
    assert telemetry._latest(rows, "k") == "b"
    assert telemetry._latest(rows, "missing", default="X") == "X"


def test_fmt_default_six_digits_and_na() -> None:
    assert telemetry._fmt(0.123456789) == "0.123457"
    assert telemetry._fmt(0.123456789, 3) == "0.123"
    assert telemetry._fmt(None) == "NA"


def test_row_accessors() -> None:
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
    assert telemetry.event({}) == ""
    assert telemetry.final_left_cmd({}) is None
    assert telemetry.physical_output_active({}) is False
    assert telemetry.gps_block_reason({}) == "NA"
