"""보정 완결성 + ``calibration-check`` 모드 + stop_correct_go 프리플라이트 게이트.

목적/역할: stop_correct_go 는 계획이 요구하는 모션 보정이 아직 안전한 반복-펄스
폴백일 때 *어떤 모션도 시작하기 전에* 실패해야 한다. 이 테스트는 그 안전 계약을 잠근다.

세 층위를 다룬다:
  1. 순수 완결성 로직(디스크 없음) — ``calibration.calibration_completeness``.
  2. 읽기 전용 ``calibration-check`` 리포트(시리얼 없음) — 요약 JSON 을 검증.
  3. ``run`` 프리플라이트 게이트 — 불완전하면 시리얼을 열지도 않고 abort(rc=2).

핵심 개념·불변식:
  - "승인된" 프리미티브 = ``source`` 가 ``fallback_known_`` 로 시작하지 *않음*.
  - turn_*_90 은 커넥터 폴백이므로 required_for_current_plan 에 절대 들어가지 않는다.
  - 게이트는 stop_correct_go 전용: 기본 폐루프 모드는 보정 게이트에 걸리지 않는다.
  - ``ready_for_full_path_following`` 은 모든 경로에서 False 로 유지된다.

Calibration completeness + ``calibration-check`` mode + the stop_correct_go
``CALIBRATION_INCOMPLETE`` preflight gate. stop_correct_go must fail *before any
motion* when the motion calibration its plan requires is still the safe
repeated-pulses fallback. Covers the pure completeness logic (no disk), the
read-only ``calibration-check`` report, and the ``run`` preflight gate (which
must abort without ever opening serial).
"""
from __future__ import annotations

import json
from pathlib import Path

import pytest

from tools.physical_path_planning import calibration, cli


# --- helpers / 픽스처·헬퍼 ----------------------------------------------------


def _cal(*, forward_ok: bool, backward_ok: bool) -> dict[str, object]:
    """최소 resolved-calibration 딕셔너리. forward/backward 승인 여부를 스위치로 만든다.

    A motion primitive is "approved" when its ``source`` does not start with
    ``fallback_known_`` (the resolver's safe-default marker)."""
    return {
        "forward": {"a": 0.30, "ms": 800, "source": "real.json" if forward_ok else "fallback_known_forward"},
        "backward": {"a": -0.08, "ms": 300, "source": "real.json" if backward_ok else "fallback_known_backward"},
        "turn_left_90": {"available": False, "source": "missing"},
        "turn_right_90": {"available": False, "source": "missing"},
        "fallback_to_repeated_pulses": True,
        "calibration_files": {},
    }


def _forward_only_segments() -> list[dict[str, object]]:
    """전진만 요구하는 단일-세그먼트 계획. / Single-segment plan needing forward only."""
    return [{"segment_index": 1, "expected_motion_direction": "forward"}]


def _multi_lane_segments() -> list[dict[str, object]]:
    """forward/backward 를 모두 요구하는 다중-레인 계획. / Multi-lane plan needing both dirs."""
    return [
        {"segment_index": 1, "expected_motion_direction": "forward"},
        {"segment_index": 2, "expected_motion_direction": "backward"},
        {"segment_index": 3, "expected_motion_direction": "forward"},
    ]


# ── 순수 완결성 로직 / Pure completeness logic ────────────────────────────────


def test_forward_only_plan_requires_only_forward() -> None:
    """전진만 하는 계획은 forward 만 요구; forward 승인되면 stop_correct_go 가능.

    A forward-only plan requires only forward; approved forward => can run."""
    result = calibration.calibration_completeness(
        _cal(forward_ok=True, backward_ok=False), segments=_forward_only_segments()
    )
    assert result["plan_requires_backward"] is False
    assert result["required_for_current_plan"] == ["forward"]
    assert result["missing_required"] == []
    assert result["can_run_stop_correct_go"] is True
    assert result["ready_for_full_path_following"] is False


def test_multi_lane_plan_requires_backward() -> None:
    """다중 레인은 backward 도 요구; backward 미승인이면 missing 에 잡혀 실행 불가.

    Multi-lane also requires backward; missing backward blocks the run."""
    result = calibration.calibration_completeness(
        _cal(forward_ok=True, backward_ok=False), segments=_multi_lane_segments()
    )
    assert result["plan_requires_backward"] is True
    assert result["required_for_current_plan"] == ["forward", "backward"]
    assert result["missing_required"] == ["backward"]
    assert result["can_run_stop_correct_go"] is False


def test_fully_calibrated_multi_lane_can_run() -> None:
    """forward+backward 모두 승인되면 다중 레인도 missing 없이 실행 가능.

    With both forward and backward approved, a multi-lane plan can run."""
    result = calibration.calibration_completeness(
        _cal(forward_ok=True, backward_ok=True), segments=_multi_lane_segments()
    )
    assert result["missing_required"] == []
    assert result["can_run_stop_correct_go"] is True


def test_missing_forward_blocks_even_single_lane() -> None:
    """전진 미승인이면 단일 레인조차 막힌다(전진은 언제나 필수).

    Missing forward blocks even a single-lane plan (forward is always required)."""
    result = calibration.calibration_completeness(
        _cal(forward_ok=False, backward_ok=True), segments=_forward_only_segments()
    )
    assert result["missing_required"] == ["forward"]
    assert result["can_run_stop_correct_go"] is False


def test_turn_90_entries_are_never_required() -> None:
    """turn_*_90 은 커넥터 폴백이므로 required_for_current_plan 에 절대 포함되지 않는다.

    turn_*_90 are connector fallbacks and never gate stop_correct_go."""
    # turn_left_90 / turn_right_90 are connector fallbacks (repeated_pulses when
    # missing); they must never appear in required_for_current_plan.
    result = calibration.calibration_completeness(
        _cal(forward_ok=True, backward_ok=True), segments=_multi_lane_segments()
    )
    assert "turn_left_90" not in result["required_for_current_plan"]
    assert "turn_right_90" not in result["required_for_current_plan"]
    assert result["can_run_stop_correct_go"] is True


def test_no_segments_defaults_to_forward_only() -> None:
    """세그먼트가 없으면(None) 보수적으로 forward-only 계획으로 간주한다.

    Absent segments default conservatively to a forward-only requirement."""
    result = calibration.calibration_completeness(
        _cal(forward_ok=True, backward_ok=False), segments=None
    )
    assert result["plan_requires_backward"] is False
    assert result["required_for_current_plan"] == ["forward"]


# ── calibration-check CLI 모드(시리얼 없음) / calibration-check CLI mode (no serial) ──


def test_calibration_check_incomplete_multi_lane(tmp_path: Path) -> None:
    """다중 레인 목표 + 보정 파일 부재 -> rc=1, 요약에 CALIBRATION_INCOMPLETE.

    누락 목록에 forward/backward, plan_requires_backward=True, plan_source=goal_flags 확인.
    Multi-lane goal + absent calibration => rc=1 and an INCOMPLETE summary listing
    both forward and backward as missing, sourced from goal flags."""
    rc = cli.main(
        [
            "calibration-check",
            "--goal-mode", "relative_enu",
            "--goal-east-m", "0",
            "--goal-north-m", "4.0",
            "--workspace-width-m", "1.5",
            "--motion-calibration-json", str(tmp_path / "absent_motion.json"),
            "--fine-calibration-json", str(tmp_path / "absent_fine.json"),
            "--out-dir", str(tmp_path / "out"),
        ]
    )
    assert rc == 1
    data = json.loads((tmp_path / "out" / "calibration_check_summary.json").read_text())
    assert data["reason"] == "CALIBRATION_INCOMPLETE"
    assert data["success"] is False
    assert data["can_run_stop_correct_go"] is False
    assert "forward" in data["missing_required_calibration"]
    assert "backward" in data["missing_required_calibration"]
    assert data["plan_requires_backward"] is True
    assert data["plan_source"] == "goal_flags"
    assert data["ready_for_full_path_following"] is False


def test_calibration_check_complete_via_monkeypatch(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """완전 승인 보정 + forward-only 계획을 주입하면 rc=0, 요약에 CALIBRATION_COMPLETE.

    디스크 보정에 의존하지 않고 핸들러의 리포트/종료 로직만 격리해 검증한다.
    Injecting a fully approved calibration + forward-only plan yields rc=0 and a
    COMPLETE summary, exercising the handler's report/exit path without disk I/O."""
    # Inject a fully approved calibration + a forward-only plan so the handler's
    # report/exit logic is exercised without depending on on-disk calibration.
    monkeypatch.setattr(cli, "resolve_calibration", lambda args: _cal(forward_ok=True, backward_ok=True))
    monkeypatch.setattr(cli, "_calibration_check_segments", lambda args, cal: (_forward_only_segments(), "goal_flags"))
    rc = cli.main(
        [
            "calibration-check",
            "--goal-mode", "relative_enu",
            "--goal-east-m", "0",
            "--goal-north-m", "1.0",
            "--path-shape", "direct_line",
            "--out-dir", str(tmp_path / "out"),
        ]
    )
    assert rc == 0
    data = json.loads((tmp_path / "out" / "calibration_check_summary.json").read_text())
    assert data["reason"] == "CALIBRATION_COMPLETE"
    assert data["success"] is True
    assert data["can_run_stop_correct_go"] is True
    assert data["required_for_current_plan"] == ["forward"]
    assert data["ready_for_full_path_following"] is False


def test_calibration_check_detail_reports_sources(tmp_path: Path) -> None:
    """calibration_detail 이 프리미티브별 approved 플래그와 실제 source 를 노출하는지.

    The calibration_detail block surfaces per-primitive approved flags and source."""
    rc = cli.main(
        [
            "calibration-check",
            "--goal-mode", "relative_enu",
            "--goal-east-m", "0",
            "--goal-north-m", "1.0",
            "--path-shape", "direct_line",
            "--motion-calibration-json", str(tmp_path / "absent_motion.json"),
            "--fine-calibration-json", str(tmp_path / "absent_fine.json"),
            "--out-dir", str(tmp_path / "out"),
        ]
    )
    assert rc == 1
    data = json.loads((tmp_path / "out" / "calibration_check_summary.json").read_text())
    detail = data["calibration_detail"]
    assert detail["forward"]["approved"] is False
    assert detail["forward"]["source"] == "fallback_known_forward"
    assert detail["turn_left_90"]["approved"] is False


# ── run 프리플라이트 게이트(시리얼 이전에 abort) / run preflight gate (abort before serial) ──


def test_run_stop_correct_go_aborts_before_serial_when_incomplete(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """stop_correct_go + 불완전 보정 -> ensure_port 도달 전 rc=2 로 abort.

    ensure_port 를 폭발시켜, 게이트를 지나쳐 실행되는 회귀가 시끄럽게 실패하도록 한다.
    stop_correct_go with incomplete calibration aborts (rc=2) before ensure_port;
    ensure_port is booby-trapped so a regression past the gate fails loudly."""
    # If the gate works, ensure_port is never reached; make it explode so a
    # regression that lets execution past the gate fails loudly.
    def _boom(_args: object) -> bool:
        raise AssertionError("ensure_port must not be called when calibration is incomplete")

    monkeypatch.setattr(cli, "ensure_port", _boom)
    rc = cli.main(
        [
            "run",
            "--start-lat", "37.5",
            "--start-lon", "127.0",
            "--goal-mode", "relative_enu",
            "--goal-east-m", "0",
            "--goal-north-m", "4.0",
            "--workspace-width-m", "1.5",
            "--path-control-mode", "stop_correct_go",
            "--motion-calibration-json", str(tmp_path / "absent_motion.json"),
            "--fine-calibration-json", str(tmp_path / "absent_fine.json"),
            "--out-dir", str(tmp_path / "out"),
        ]
    )
    assert rc == 2
    data = json.loads((tmp_path / "out" / "run_summary.json").read_text())
    assert data["reason"] == "CALIBRATION_INCOMPLETE"
    assert data["aborted"] is True
    assert data["missing_required_calibration"] == ["forward", "backward"]
    assert data["ready_for_full_path_following"] is False


def test_run_default_mode_does_not_trip_calibration_gate(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """기본 폐루프 모드는 보정 게이트에 걸리지 않고 포트 체크까지 도달(여기서 rc=2).

    게이트는 stop_correct_go 전용이므로, 기본 모드 실패 사유는 CALIBRATION_INCOMPLETE 가 아니다.
    The default closed-loop mode bypasses the calibration gate and fails at the
    (stubbed) port check instead, so no CALIBRATION_INCOMPLETE summary is written."""
    # The gate is stop_correct_go-only: the default closed-loop mode must reach
    # the port check (which we stub to fail) rather than abort on calibration.
    monkeypatch.setattr(cli, "ensure_port", lambda args: False)
    rc = cli.main(
        [
            "run",
            "--start-lat", "37.5",
            "--start-lon", "127.0",
            "--goal-mode", "relative_enu",
            "--goal-east-m", "0",
            "--goal-north-m", "4.0",
            "--workspace-width-m", "1.5",
            "--motion-calibration-json", str(tmp_path / "absent_motion.json"),
            "--fine-calibration-json", str(tmp_path / "absent_fine.json"),
            "--out-dir", str(tmp_path / "out"),
        ]
    )
    # ensure_port stubbed False -> cmd_run returns 2 from the port gate, NOT the
    # calibration gate; no CALIBRATION_INCOMPLETE summary is written.
    assert rc == 2
    run_summary = tmp_path / "out" / "run_summary.json"
    if run_summary.exists():
        assert json.loads(run_summary.read_text()).get("reason") != "CALIBRATION_INCOMPLETE"
