"""Calibration completeness + ``calibration-check`` mode + the stop_correct_go
``CALIBRATION_INCOMPLETE`` preflight gate.

stop_correct_go must fail *before any motion* when the motion calibration its
plan requires is still the safe repeated-pulses fallback. These tests cover the
pure completeness logic (no disk), the read-only ``calibration-check`` report,
and the ``run`` preflight gate (which must abort without ever opening serial).
"""
from __future__ import annotations

import json
from pathlib import Path

import pytest

from tools.physical_path_planning import calibration, cli


# --- helpers -----------------------------------------------------------------


def _cal(*, forward_ok: bool, backward_ok: bool) -> dict[str, object]:
    """A minimal resolved-calibration dict.

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
    return [{"segment_index": 1, "expected_motion_direction": "forward"}]


def _multi_lane_segments() -> list[dict[str, object]]:
    return [
        {"segment_index": 1, "expected_motion_direction": "forward"},
        {"segment_index": 2, "expected_motion_direction": "backward"},
        {"segment_index": 3, "expected_motion_direction": "forward"},
    ]


# --- pure completeness logic -------------------------------------------------


def test_forward_only_plan_requires_only_forward() -> None:
    result = calibration.calibration_completeness(
        _cal(forward_ok=True, backward_ok=False), segments=_forward_only_segments()
    )
    assert result["plan_requires_backward"] is False
    assert result["required_for_current_plan"] == ["forward"]
    assert result["missing_required"] == []
    assert result["can_run_stop_correct_go"] is True
    assert result["ready_for_full_path_following"] is False


def test_multi_lane_plan_requires_backward() -> None:
    result = calibration.calibration_completeness(
        _cal(forward_ok=True, backward_ok=False), segments=_multi_lane_segments()
    )
    assert result["plan_requires_backward"] is True
    assert result["required_for_current_plan"] == ["forward", "backward"]
    assert result["missing_required"] == ["backward"]
    assert result["can_run_stop_correct_go"] is False


def test_fully_calibrated_multi_lane_can_run() -> None:
    result = calibration.calibration_completeness(
        _cal(forward_ok=True, backward_ok=True), segments=_multi_lane_segments()
    )
    assert result["missing_required"] == []
    assert result["can_run_stop_correct_go"] is True


def test_missing_forward_blocks_even_single_lane() -> None:
    result = calibration.calibration_completeness(
        _cal(forward_ok=False, backward_ok=True), segments=_forward_only_segments()
    )
    assert result["missing_required"] == ["forward"]
    assert result["can_run_stop_correct_go"] is False


def test_turn_90_entries_are_never_required() -> None:
    # turn_left_90 / turn_right_90 are connector fallbacks (repeated_pulses when
    # missing); they must never appear in required_for_current_plan.
    result = calibration.calibration_completeness(
        _cal(forward_ok=True, backward_ok=True), segments=_multi_lane_segments()
    )
    assert "turn_left_90" not in result["required_for_current_plan"]
    assert "turn_right_90" not in result["required_for_current_plan"]
    assert result["can_run_stop_correct_go"] is True


def test_no_segments_defaults_to_forward_only() -> None:
    result = calibration.calibration_completeness(
        _cal(forward_ok=True, backward_ok=False), segments=None
    )
    assert result["plan_requires_backward"] is False
    assert result["required_for_current_plan"] == ["forward"]


# --- calibration-check CLI mode (no serial) ----------------------------------


def test_calibration_check_incomplete_multi_lane(tmp_path: Path) -> None:
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


# --- run preflight gate (must abort before serial) ---------------------------


def test_run_stop_correct_go_aborts_before_serial_when_incomplete(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
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
