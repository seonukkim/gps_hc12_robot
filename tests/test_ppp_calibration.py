"""Calibration resolver tests for the consolidated path-planning package.

Cover fine/twitch/angle/smooth source loading, the auto-mode connector selection
(angle-calibrated -> smooth -> repeated-pulses fallback), the explicit
angle-calibrated guard, and the connector-primitive accessor -- all against
``tools.physical_path_planning.calibration``. ``ready_for_full_path_following``
stays false on every path.
"""
from __future__ import annotations

import json
from pathlib import Path

import pytest

from tools.physical_path_planning import calibration as resolver


def _write_json(path: Path, payload: dict[str, object]) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return path


def test_resolver_loads_fine_calibration(tmp_path: Path) -> None:
    fine = _write_json(
        tmp_path / "physical_ab_fine_motion_calibration.json",
        {
            "stage22_forward_a_cmd": 0.31,
            "stage22_forward_pulse_ms": 820,
            "stage21_backward_a_cmd": -0.09,
            "stage21_backward_pulse_ms": 320,
        },
    )
    resolved = resolver.resolve_physical_calibration(
        fine_calibration_json=fine,
        turn_calibration_json=tmp_path / "missing_turn.json",
        turn_angle_calibration_json=tmp_path / "missing_angle.json",
        smooth_turn_calibration_json=tmp_path / "missing_smooth.json",
    )
    assert resolved["forward"] == {"a": 0.31, "b": 0.0, "ms": 820, "source": "physical_ab_fine_motion_calibration.json"}
    assert resolved["backward"] == {"a": -0.09, "b": 0.0, "ms": 320, "source": "physical_ab_fine_motion_calibration.json"}
    assert resolved["ready_for_full_path_following"] is False


def test_resolver_loads_turn_twitch_calibration(tmp_path: Path) -> None:
    turn = _write_json(
        tmp_path / "physical_ab_turn_twitch_calibration.json",
        {
            "turn_left_min_b_cmd": 0.27,
            "turn_left_min_pulse_ms": 710,
            "turn_right_min_b_cmd": -0.10,
            "turn_right_min_pulse_ms": 260,
        },
    )
    resolved = resolver.resolve_physical_calibration(
        fine_calibration_json=tmp_path / "missing_fine.json",
        turn_calibration_json=turn,
        turn_angle_calibration_json=tmp_path / "missing_angle.json",
        smooth_turn_calibration_json=tmp_path / "missing_smooth.json",
    )
    assert resolved["turn_left"] == {"a": 0.0, "b": 0.27, "ms": 710, "source": "physical_ab_turn_twitch_calibration.json"}
    assert resolved["turn_right"] == {"a": 0.0, "b": -0.10, "ms": 260, "source": "physical_ab_turn_twitch_calibration.json"}


def test_resolver_loads_angle_calibration(tmp_path: Path) -> None:
    angle = _write_json(
        tmp_path / "physical_ab_turn_angle_calibration.json",
        {
            "turn_left_90": {
                "a_cmd": 0.0,
                "b_cmd": 0.26,
                "pulse_ms": 700,
                "imu_yaw_delta_deg": 87.5,
                "visual_confirmation": "yes",
                "ready": True,
            },
            "turn_right_90": {
                "a_cmd": 0.0,
                "b_cmd": -0.08,
                "pulse_ms": 250,
                "imu_yaw_delta_deg": -91.2,
                "visual_confirmation": "yes",
                "ready": True,
            },
        },
    )
    resolved = resolver.resolve_physical_calibration(
        fine_calibration_json=tmp_path / "missing_fine.json",
        turn_calibration_json=tmp_path / "missing_turn.json",
        turn_angle_calibration_json=angle,
        smooth_turn_calibration_json=tmp_path / "missing_smooth.json",
    )
    assert resolved["turn_left_90"]["available"] is True
    assert resolved["turn_left_90"]["imu_yaw_delta_deg"] == 87.5
    assert resolved["turn_right_90"]["available"] is True
    assert resolved["ready_for_angle_based_connectors"] is True


def test_auto_mode_chooses_angle_calibrated_when_both_90_turns_ready(tmp_path: Path) -> None:
    angle = _write_json(
        tmp_path / "physical_ab_turn_angle_calibration.json",
        {
            "turn_left_90": {"a_cmd": 0, "b_cmd": 0.26, "pulse_ms": 700, "visual_confirmation": "yes", "ready": True},
            "turn_right_90": {"a_cmd": 0, "b_cmd": -0.08, "pulse_ms": 250, "visual_confirmation": "yes", "ready": True},
        },
    )
    resolved = resolver.resolve_physical_calibration(
        fine_calibration_json=tmp_path / "missing_fine.json",
        turn_calibration_json=tmp_path / "missing_turn.json",
        turn_angle_calibration_json=angle,
        smooth_turn_calibration_json=tmp_path / "missing_smooth.json",
        calibration_mode="auto",
    )
    assert resolved["connector_mode_recommended"] == "angle_calibrated"
    assert resolved["connector_mode_effective"] == "angle_calibrated"
    left = resolver.connector_primitive(resolved, "left")
    assert left["b_cmd"] == 0.26
    assert left["calibration_source"] == "physical_ab_turn_angle_calibration.json"


def test_auto_mode_falls_back_to_repeated_pulses_without_connector_calibration(tmp_path: Path) -> None:
    resolved = resolver.resolve_physical_calibration(
        fine_calibration_json=tmp_path / "missing_fine.json",
        turn_calibration_json=tmp_path / "missing_turn.json",
        turn_angle_calibration_json=tmp_path / "missing_angle.json",
        smooth_turn_calibration_json=tmp_path / "missing_smooth.json",
        calibration_mode="auto",
    )
    assert resolved["connector_mode_recommended"] == "repeated_pulses"
    assert resolved["connector_mode_effective"] == "repeated_pulses"
    assert resolved["fallback_to_repeated_pulses"] is True
    assert resolver.connector_primitive(resolved, "right")["b_cmd"] == -0.08


def test_angle_calibrated_mode_fails_if_angle_calibration_missing(tmp_path: Path) -> None:
    with pytest.raises(RuntimeError, match="angle-calibrated connector requested"):
        resolver.resolve_physical_calibration(
            fine_calibration_json=tmp_path / "missing_fine.json",
            turn_calibration_json=tmp_path / "missing_turn.json",
            turn_angle_calibration_json=tmp_path / "missing_angle.json",
            smooth_turn_calibration_json=tmp_path / "missing_smooth.json",
            calibration_mode="angle_calibrated",
        )


def test_smooth_calibration_is_used_when_angle_missing(tmp_path: Path) -> None:
    smooth = _write_json(
        tmp_path / "smooth_turn_connector_calibration.json",
        {
            "smooth_left_b_cmd": 0.20,
            "smooth_left_max_ms": 3000,
            "smooth_right_b_cmd": 0.09,
            "smooth_right_max_ms": 2800,
            "ready_for_smooth_connectors": True,
            "ready_for_full_path_following": False,
        },
    )
    resolved = resolver.resolve_physical_calibration(
        fine_calibration_json=tmp_path / "missing_fine.json",
        turn_calibration_json=tmp_path / "missing_turn.json",
        turn_angle_calibration_json=tmp_path / "missing_angle.json",
        smooth_turn_calibration_json=smooth,
        calibration_mode="auto",
    )
    assert resolved["connector_mode_effective"] == "smooth_imu"
    assert resolved["ready_for_smooth_connectors"] is True
    assert resolver.connector_primitive(resolved, "left")["pulse_ms"] == 3000
