"""Tests for AUTO-switch-triggered relative path planning.

``auto-relative-preview`` resolves the relative A->B field with no motion;
``auto-relative-run`` waits for GPS, watches the physical mode switch, and only
starts closed-loop execution on AUTO (or a keyboard fallback). The serial-facing
core is driven through a fake handle that replays scripted telemetry, mirroring
``test_ppp_controller``.
"""
from __future__ import annotations

import json
from pathlib import Path

import pytest

from tools.physical_path_planning import cli, geometry


class FakeSerial:
    def __init__(self, responses: list[bytes]) -> None:
        self._responses = list(responses)
        self.writes: list[str] = []

    def write(self, data: bytes) -> int:
        self.writes.append(data.decode("ascii").strip())
        return len(data)

    def flush(self) -> None:
        pass

    def readline(self) -> bytes:
        return self._responses.pop(0) if self._responses else b""

    def close(self) -> None:
        pass


def _hb(
    *,
    lat: float = 35.0,
    lon: float = 129.0,
    sats: int = 6,
    hdop: float = 1.0,
    gps_ready: bool = True,
    gps_block: str = "OK",
    mode_switch: str = "AUTO",
    rc_ok: bool = False,
    imu_yaw: float = 0.0,
    include_mode: bool = True,
) -> bytes:
    parts = [
        "USB_PULSE_TEST event=HEARTBEAT usb_pulse_test_mode=true usb_drive_live_mode=true",
        f"rc_ok={str(rc_ok).lower()} neutral_ok=true physical_output_active=false",
        f"gps_block_reason={gps_block} gps_ready={str(gps_ready).lower()} "
        f"gps_solution_valid={str(gps_ready).lower()}",
        f"gps_sats={sats} gps_hdop={hdop}",
        f"current_lat={lat:.7f} current_lon={lon:.7f}",
        f"imu_present=true imu_relative_yaw_deg={imu_yaw:.1f}",
    ]
    if include_mode:
        auto = "true" if mode_switch == "AUTO" else "false"
        mode_us = 1700 if mode_switch == "AUTO" else 1100
        parts.append(
            f"mode_switch={mode_switch} mode_us={mode_us} auto_sw={auto} mode_channel_present=true"
        )
    return (" ".join(parts) + "\n").encode("ascii")


def _run_args(out_dir: Path, **overrides: object):
    parser = cli.build_parser()
    args = parser.parse_args(
        [
            "auto-relative-run",
            "--goal-east-m", "0.3",
            "--goal-north-m", "0.0",
            "--path-shape", "direct_line",
            "--nominal-forward-pulse-m", "1.0",  # 0.3m -> a single pulse chunk
            "--straight-motion-mode", "pulse",
            "--gps-timeout-s", "1.0",
            "--auto-switch-timeout-s", "1.0",
            "--heartbeat-timeout-s", "0.3",
            "--event-timeout-s", "0.3",
            "--no-png",
            "--out-dir", str(out_dir),
        ]
    )
    for key, value in overrides.items():
        setattr(args, key, value)
    return args


# --- preview ------------------------------------------------------------------


def test_auto_relative_preview_resolves_a_b_distance(tmp_path: Path) -> None:
    rc = cli.main(
        [
            "auto-relative-preview",
            "--start-lat", "35.5709",
            "--start-lon", "129.1871",
            "--goal-east-m", "3.0",
            "--goal-north-m", "4.0",
            "--workspace-width-m", "1.5",
            "--step-spacing-m", "0.30",
            "--no-png",
            "--out-dir", str(tmp_path),
        ]
    )
    assert rc == 0
    field = json.loads((tmp_path / "field_config_resolved.json").read_text())
    assert field["start_x_m"] == 0.0
    assert field["start_y_m"] == 0.0
    assert field["goal_mode"] == "relative_enu"
    assert field["resolved_goal_x_m"] == pytest.approx(3.0)
    assert field["resolved_goal_y_m"] == pytest.approx(4.0)
    assert field["expected_goal_distance_m"] == pytest.approx(5.0, abs=1e-3)
    assert field["workspace_width_m"] == 1.5
    summary = json.loads((tmp_path / "summary.json").read_text())
    assert summary["mode"] == "auto-relative-preview"
    assert summary["ready_for_full_path_following"] is False


# --- run: GPS gate ------------------------------------------------------------


def test_auto_relative_run_waits_for_gps_before_resolving_start(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(cli, "DEFAULT_GPS_CACHE", tmp_path / "cache" / "latest_start.json")
    called: list[object] = []
    monkeypatch.setattr(cli.controller, "run_controller", lambda *a, **k: called.append(k) or ([], [], "NONE"))
    handle = FakeSerial([_hb(gps_ready=False, gps_block="BAD_HDOP", sats=2)])
    args = _run_args(tmp_path, gps_timeout_s=0.3)
    rc = cli._auto_relative_run_on_handle(
        handle, args, geometry.FALLBACK_RESOLVED_CALIBRATION, tmp_path,
        plan=None, field_config=None, plan_dir_used=False, input_fn=lambda: "",
    )
    assert rc == 2
    assert not called  # never started execution without a fix
    summary = json.loads((tmp_path / "summary.json").read_text())
    assert summary["execution_started"] is False
    assert summary["stop_reason"] == "NO_USABLE_START_GPS"


# --- run: AUTO switch end-to-end (real controller, pulse path) ----------------


def test_auto_relative_run_auto_switch_starts_execution(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(cli, "DEFAULT_GPS_CACHE", tmp_path / "cache" / "latest_start.json")
    cal = dict(geometry.FALLBACK_RESOLVED_CALIBRATION)
    cal["forward"] = {"a": 0.30, "b": 0.0, "ms": 5, "source": "approved_test"}
    handle = FakeSerial(
        [
            _hb(),  # GPS wait resolves start A
            _hb(),  # AUTO switch detected
            _hb(),  # run_controller pre-pulse heartbeat
            b"USB_PULSE_TEST event=ARM\n",
            b"USB_PULSE_TEST event=ACK\n",
            b"USB_PULSE_TEST event=PULSE_COMPLETE\n",
            b"USB_PULSE_TEST event=STOP final_left_cmd=0.000 final_right_cmd=0.000 physical_output_active=false\n",
            _hb(),  # post-pulse heartbeat
        ]
    )
    # This test covers the AUTO-switch execution path; initial heading alignment
    # is a separate concern (own coverage), so opt out of the gps_probe default.
    args = _run_args(tmp_path, initial_heading_align="none")
    rc = cli._auto_relative_run_on_handle(
        handle, args, cal, tmp_path,
        plan=None, field_config=None, plan_dir_used=False, input_fn=lambda: "",
    )
    assert rc == 0
    summary = json.loads((tmp_path / "summary.json").read_text())
    assert summary["auto_switch_detected"] is True
    assert summary["execution_started"] is True
    assert summary["start_source"] == "live_gps"
    assert summary["mode"] == "auto-relative-run"
    assert summary["goal_mode"] == "relative_enu"
    assert (tmp_path / "field_config_resolved.json").exists()
    assert (tmp_path / "closed_loop_trace.csv").exists()
    assert (tmp_path / "planned_vs_actual.csv").exists()
    assert (tmp_path / "raw_usbdbg.log").exists()
    trace = (tmp_path / "closed_loop_trace.csv").read_text()
    for column in (
        "heading_error_deg", "cross_track_error_m", "b_heading_correction",
        "b_cross_track_correction", "final_b_cmd", "correction_source",
        "segment_start_x_m", "segment_end_y_m",
    ):
        assert column in trace


def test_auto_relative_run_stop_correct_go_aborts_before_motion_when_incomplete(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(cli, "DEFAULT_GPS_CACHE", tmp_path / "cache" / "latest_start.json")

    def _no_align(*_a: object, **_k: object) -> dict[str, object]:
        raise AssertionError("alignment (motion) must not run when calibration is incomplete")

    monkeypatch.setattr(cli, "_run_initial_alignment", _no_align)
    # Default fallback calibration leaves forward as repeated-pulses fallback.
    cal = dict(geometry.FALLBACK_RESOLVED_CALIBRATION)
    handle = FakeSerial([_hb(), _hb()])  # GPS wait, then AUTO; gate fires before alignment
    args = _run_args(tmp_path, path_control_mode="stop_correct_go", initial_heading_align="none")
    rc = cli._auto_relative_run_on_handle(
        handle, args, cal, tmp_path,
        plan=None, field_config=None, plan_dir_used=False, input_fn=lambda: "",
    )
    assert rc == 2
    summary = json.loads((tmp_path / "summary.json").read_text())
    assert summary["reason"] == "CALIBRATION_INCOMPLETE"
    assert summary["execution_started"] is False
    assert summary["missing_required_calibration"] == ["forward"]
    assert summary["ready_for_full_path_following"] is False


# --- run: MANUAL prevents execution -------------------------------------------


def test_auto_relative_run_manual_switch_prevents_execution(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(cli, "DEFAULT_GPS_CACHE", tmp_path / "cache" / "latest_start.json")
    called: list[object] = []
    monkeypatch.setattr(cli.controller, "run_controller", lambda *a, **k: called.append(k) or ([], [], "NONE"))
    handle = FakeSerial([_hb(mode_switch="MANUAL") for _ in range(6)])
    args = _run_args(tmp_path, auto_switch_timeout_s=0.3)
    rc = cli._auto_relative_run_on_handle(
        handle, args, geometry.FALLBACK_RESOLVED_CALIBRATION, tmp_path,
        plan=None, field_config=None, plan_dir_used=False, input_fn=lambda: "",
    )
    assert rc == 2
    assert not called  # MANUAL never starts the controller
    summary = json.loads((tmp_path / "summary.json").read_text())
    assert summary["execution_started"] is False
    assert summary["auto_switch_detected"] is False
    assert summary["stop_reason"] == "AUTO_SWITCH_NOT_DETECTED"
    # The field config was still resolved from the GPS start.
    assert (tmp_path / "field_config_resolved.json").exists()


def test_auto_relative_run_manual_during_execution_stops_safely(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(cli, "DEFAULT_GPS_CACHE", tmp_path / "cache" / "latest_start.json")
    seen: dict[str, object] = {}

    def fake_run_controller(handle, **kwargs):
        seen.update(kwargs)
        return [], ["USB_PULSE_TEST event=STOP final_left_cmd=0.000 final_right_cmd=0.000"], "USER_SWITCHED_TO_MANUAL"

    monkeypatch.setattr(cli.controller, "run_controller", fake_run_controller)
    handle = FakeSerial([_hb(), _hb()])
    # Initial heading alignment has its own coverage; this test isolates the
    # mid-execution MANUAL stop, so opt out of the gps_probe default.
    args = _run_args(tmp_path, initial_heading_align="none")
    rc = cli._auto_relative_run_on_handle(
        handle, args, geometry.FALLBACK_RESOLVED_CALIBRATION, tmp_path,
        plan=None, field_config=None, plan_dir_used=False, input_fn=lambda: "",
    )
    assert seen.get("require_auto_switch") is True  # controller told to watch the switch
    summary = json.loads((tmp_path / "summary.json").read_text())
    assert summary["execution_started"] is True
    assert summary["stop_reason"] == "USER_SWITCHED_TO_MANUAL"
    assert rc == 0  # a deliberate operator stop is a clean stop


# --- run: keyboard fallback ---------------------------------------------------


def test_auto_relative_run_keyboard_fallback_when_mode_channel_absent(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(cli, "DEFAULT_GPS_CACHE", tmp_path / "cache" / "latest_start.json")
    started: list[object] = []
    monkeypatch.setattr(cli.controller, "run_controller", lambda *a, **k: started.append(k) or ([], [], "NONE"))
    handle = FakeSerial([_hb(include_mode=False) for _ in range(4)])
    # Keyboard-start path under test; alignment is covered separately.
    args = _run_args(tmp_path, allow_keyboard_start="true", initial_heading_align="none")
    pressed: list[int] = []
    rc = cli._auto_relative_run_on_handle(
        handle, args, geometry.FALLBACK_RESOLVED_CALIBRATION, tmp_path,
        plan=None, field_config=None, plan_dir_used=False,
        input_fn=lambda: pressed.append(1) or "",
    )
    assert rc == 0
    assert pressed  # the operator was prompted
    assert started  # execution began after the keyboard start
    summary = json.loads((tmp_path / "summary.json").read_text())
    assert summary["execution_started"] is True
    assert summary["auto_switch_detected"] is False
    assert summary["auto_start_reason"] == "KEYBOARD_START"


def test_auto_relative_run_no_keyboard_without_opt_in(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(cli, "DEFAULT_GPS_CACHE", tmp_path / "cache" / "latest_start.json")
    called: list[object] = []
    monkeypatch.setattr(cli.controller, "run_controller", lambda *a, **k: called.append(k) or ([], [], "NONE"))
    handle = FakeSerial([_hb(include_mode=False) for _ in range(4)])
    args = _run_args(tmp_path, allow_keyboard_start="false", auto_switch_timeout_s=0.3)
    rc = cli._auto_relative_run_on_handle(
        handle, args, geometry.FALLBACK_RESOLVED_CALIBRATION, tmp_path,
        plan=None, field_config=None, plan_dir_used=False, input_fn=lambda: "",
    )
    assert rc == 2
    assert not called


# --- run: initial heading alignment gate --------------------------------------


def test_auto_relative_run_aborts_when_gps_probe_alignment_fails(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """The auto-relative default (gps_probe) must abort before execution when the
    initial heading alignment fails, and it must never reach the controller."""
    monkeypatch.setattr(cli, "DEFAULT_GPS_CACHE", tmp_path / "cache" / "latest_start.json")
    called: list[object] = []
    monkeypatch.setattr(
        cli.controller, "run_controller",
        lambda *a, **k: called.append(k) or ([], [], "NONE"),
    )
    # GPS resolves the start and AUTO is detected, but every heartbeat sits at the
    # same lat/lon -> the gps_probe sees no usable displacement -> alignment fails.
    handle = FakeSerial([_hb() for _ in range(4)])
    args = _run_args(tmp_path)  # keep the gps_probe default
    rc = cli._auto_relative_run_on_handle(
        handle, args, geometry.FALLBACK_RESOLVED_CALIBRATION, tmp_path,
        plan=None, field_config=None, plan_dir_used=False, input_fn=lambda: "",
    )
    assert rc == 2
    assert not called  # alignment failure must short-circuit the controller
    summary = json.loads((tmp_path / "summary.json").read_text())
    assert summary["execution_started"] is False
    assert str(summary["stop_reason"]).startswith("INITIAL_ALIGNMENT_FAILED")
    assert summary["ready_for_full_path_following"] is False
    assert (tmp_path / "alignment").is_dir()  # alignment artifacts were written
