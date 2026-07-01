"""AUTO 스위치로 트리거되는 상대 경로계획(auto-relative) 계약 테스트.

목적/역할: 두 CLI 모드를 잠근다.
  - ``auto-relative-preview`` : 모션 없이 상대 A->B 필드를 해석하고 산출물을 쓴다.
  - ``auto-relative-run`` : GPS 를 기다려 시작점 A 를 확정하고, 물리 모드 스위치를 감시하며,
    AUTO(또는 옵트인된 키보드 폴백)일 때만 폐루프 실행을 시작한다.

시스템 내 위치: run 코어(``cli._auto_relative_run_on_handle``)는 시리얼을 다루므로,
대본 텔레메트리를 재생하는 ``FakeSerial`` 로 구동한다(``test_ppp_controller`` 와 동일한 방식).
실제 폐루프는 ``controller.run_controller`` — 테스트마다 진짜로 돌리거나 monkeypatch 로 대체.

핵심 개념·불변식(상태 게이트):
  - GPS 미확보 -> NO_USABLE_START_GPS 로 실행 미시작(rc=2).
  - MANUAL 유지 -> AUTO_SWITCH_NOT_DETECTED 로 미시작; 실행 중 MANUAL 전환은 안전 정지(rc=0).
  - 모드 채널 부재 + 옵트인 -> KEYBOARD_START; 옵트인 없으면 미시작.
  - stop_correct_go + 불완전 보정 -> 정렬(모션) 전에 CALIBRATION_INCOMPLETE 로 abort.
  - gps_probe 초기 정렬 실패 -> 컨트롤러 도달 전 INITIAL_ALIGNMENT_FAILED 로 abort.
  - ``ready_for_full_path_following`` 은 모든 경로에서 False.

Contract tests for AUTO-switch-triggered relative path planning.
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


# ── 픽스처·헬퍼 / Fixtures & helpers ──────────────────────────────────────────


class FakeSerial:
    """write 를 기록하고 readline() 마다 대본 응답을 반환하는 시리얼 대역.

    Serial fake: records writes and replays scripted response lines per readline()."""

    def __init__(self, responses: list[bytes]) -> None:
        self._responses = list(responses)
        self.writes: list[str] = []

    def write(self, data: bytes) -> int:
        """보낸 명령을 디코드해 로그에 남긴다. / Record the decoded outgoing command."""
        self.writes.append(data.decode("ascii").strip())
        return len(data)

    def flush(self) -> None:
        """no-op. / No-op."""
        pass

    def readline(self) -> bytes:
        """다음 대본 라인, 소진되면 b"". / Next scripted line, or b"" when drained."""
        return self._responses.pop(0) if self._responses else b""

    def close(self) -> None:
        """no-op. / No-op."""
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
    """USBDBG HEARTBEAT 한 줄을 합성(GPS/IMU/모드스위치 필드 포함, 개행+ascii 인코딩).

    include_mode=False 는 모드 채널 부재를 흉내내 키보드 폴백 경로를 시험할 때 쓴다.
    Build one USBDBG HEARTBEAT line (GPS/IMU/mode-switch fields); include_mode=False
    simulates an absent mode channel to exercise the keyboard-fallback path."""
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
    """auto-relative-run 인자를 짧은 타임아웃으로 파싱하고 overrides 로 필드를 덮어써 반환.

    Parse auto-relative-run args with short timeouts, then apply keyword overrides."""
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


# ── preview / preview ─────────────────────────────────────────────────────────


def test_auto_relative_preview_resolves_a_b_distance(tmp_path: Path) -> None:
    """preview 는 시작을 (0,0)으로 두고 상대 목표(3,4)와 거리 5m 를 해석해 파일로 쓴다(모션 없음).

    Preview places start at (0,0), resolves the relative (3,4) goal and 5m distance,
    and writes the field/summary artifacts with no motion."""
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


# ── run: GPS 게이트 / run: GPS gate ───────────────────────────────────────────


def test_auto_relative_run_waits_for_gps_before_resolving_start(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """GPS 미확보(BAD_HDOP)면 rc=2, run_controller 미호출, stop_reason=NO_USABLE_START_GPS.

    Without a fix (BAD_HDOP): rc=2, controller never called, NO_USABLE_START_GPS."""
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


# ── run: AUTO 스위치 종단간(실제 컨트롤러, 펄스 경로) / run: AUTO switch end-to-end ──


def test_auto_relative_run_auto_switch_starts_execution(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """종단간(실제 컨트롤러): GPS 로 A 확정 -> AUTO 감지 -> 펄스 실행, rc=0 + 트레이스 산출물.

    closed_loop_trace.csv 에 헤딩/크로스트랙/보정 컬럼이 모두 존재하는지 확인.
    End-to-end (real controller): resolve A via GPS, detect AUTO, run a pulse; rc=0
    with trace artifacts and all heading/cross-track/correction columns present."""
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
    """stop_correct_go + 폴백(불완전) 보정: AUTO 이후에도 정렬(모션) 전에 CALIBRATION_INCOMPLETE 로 abort.

    정렬 함수를 폭발시켜, 게이트가 모션 이전에 막지 못하는 회귀가 실패하도록 한다.
    stop_correct_go with fallback (incomplete) calibration aborts as
    CALIBRATION_INCOMPLETE before alignment (motion) even after AUTO; alignment is
    booby-trapped so a regression that reaches motion fails."""
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


# ── run: MANUAL 은 실행을 막음 / run: MANUAL prevents execution ────────────────


def test_auto_relative_run_manual_switch_prevents_execution(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """스위치가 계속 MANUAL 이면 rc=2, 컨트롤러 미시작, AUTO_SWITCH_NOT_DETECTED(필드는 해석됨).

    A persistently MANUAL switch => rc=2, controller never starts,
    AUTO_SWITCH_NOT_DETECTED (field config still resolved)."""
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
    """실행 중 MANUAL 전환은 깨끗한 정지: 컨트롤러에 require_auto_switch=True 를 전달하고 rc=0.

    stop_reason=USER_SWITCHED_TO_MANUAL — 의도된 조작자 정지이므로 성공으로 간주.
    A mid-execution MANUAL switch is a clean stop: the controller is told to watch
    the switch (require_auto_switch=True), stop_reason=USER_SWITCHED_TO_MANUAL, rc=0."""
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


# ── run: 키보드 폴백 / run: keyboard fallback ─────────────────────────────────


def test_auto_relative_run_keyboard_fallback_when_mode_channel_absent(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """모드 채널 부재 + allow_keyboard_start=true 면 조작자 프롬프트 후 실행 시작(KEYBOARD_START).

    With no mode channel and keyboard start opted in, the operator is prompted and
    execution begins with auto_start_reason=KEYBOARD_START."""
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
    """옵트인이 없으면(allow_keyboard_start=false) 모드 채널이 없어도 키보드로 시작하지 않음(rc=2).

    Without opt-in, an absent mode channel does not enable keyboard start: rc=2, no run."""
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


# ── run: 초기 헤딩 정렬 게이트 / run: initial heading alignment gate ───────────


def test_auto_relative_run_aborts_when_gps_probe_alignment_fails(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """gps_probe 기본값에서 초기 정렬이 실패하면(모든 하트비트가 같은 좌표->변위 없음) 실행 전 abort.

    컨트롤러에 절대 도달하지 않고, INITIAL_ALIGNMENT_FAILED + alignment 아티팩트 디렉터리 생성.
    The auto-relative default (gps_probe) must abort before execution when the
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
