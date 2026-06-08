from __future__ import annotations

import json
from pathlib import Path

import pytest

from tools.physical_path_planning import calibration, cli, controller, geometry, tuning


def _write_json(path: Path, payload: dict[str, object]) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return path


def test_tune_motion_adjusts_candidate_from_user_feedback() -> None:
    candidate = tuning.initial_candidate("forward")
    adjusted = tuning.adjust_candidate(candidate, "weak")
    assert adjusted["a"] == 0.30
    assert adjusted["ms"] == 900

    adjusted = tuning.adjust_candidate(candidate, "strong")
    assert adjusted["ms"] == 700

    turn = tuning.initial_candidate("left")
    adjusted_turn = tuning.adjust_candidate(turn, "weak")
    assert adjusted_turn["b"] == 0.26
    assert adjusted_turn["ms"] == 800


def test_approve_saves_motion_calibration_json(tmp_path: Path) -> None:
    candidate = tuning.initial_candidate("turn-right-90")
    candidate["b"] = -0.12
    candidate["ms"] = 1600
    path = tmp_path / "motion_calibration.json"

    saved = tuning.save_approved_calibration(path, candidate, yaw_delta_deg=-91.0)

    assert saved["turn_right_90"]["a"] == 0.0
    assert saved["turn_right_90"]["b"] == -0.12
    assert saved["turn_right_90"]["ms"] == 1600
    assert saved["turn_right_90"]["target_angle_deg"] == 90.0
    assert saved["turn_right_90"]["last_imu_yaw_delta_deg"] == -91.0
    assert saved["turn_right_90"]["approved_by_user"] is True
    assert saved["ready_for_full_path_following"] is False


def test_turn_tuning_uses_imu_yaw_delta_when_available() -> None:
    rows = [
        {"imu_relative_yaw_deg": "175.0"},
        {"imu_relative_yaw_deg": "-100.0"},
    ]
    assert tuning.yaw_delta_from_rows(rows) == 85.0

    candidate = tuning.initial_candidate("turn-left-90")
    adjusted = tuning.adjust_candidate(candidate, "good", yaw_delta_deg=45.0)
    assert adjusted["ms"] > candidate["ms"]

    near_target = tuning.adjust_candidate(candidate, "good", yaw_delta_deg=88.0)
    assert near_target["ms"] == candidate["ms"]


def test_tune_motion_has_no_observed_distance_option() -> None:
    parser = cli.build_parser()
    help_text = parser.format_help()
    assert "tune-motion" in help_text
    assert "set-motion-calibration" in help_text
    assert "observed-distance" not in help_text


def test_opposite_sign_transient_detects_reverse_kick() -> None:
    rows = [
        {
            "motor_write_called": "true",
            "physical_a_cmd": "-0.020",
            "final_left_cmd": "-0.020",
            "final_right_cmd": "-0.020",
        }
    ]
    assert tuning.opposite_sign_transient("forward", rows) is True
    assert tuning.opposite_sign_transient("backward", rows) is False


def test_tune_motion_print_candidate_writes_summary(tmp_path: Path) -> None:
    rc = cli.main(
        [
            "tune-motion",
            "--primitive",
            "forward",
            "--print-candidate",
            "true",
            "--out-dir",
            str(tmp_path),
        ]
    )
    assert rc == 0
    summary = json.loads((tmp_path / "summary.json").read_text())
    assert summary["mode"] == "tune-motion"
    assert summary["reason"] == "CANDIDATE_PRINTED"
    assert summary["ready_for_full_path_following"] is False
    assert (tmp_path / "tune_motion_candidate.json").exists()


def test_motion_calibration_resolver_prefers_approved_values(tmp_path: Path) -> None:
    motion = _write_json(
        tmp_path / "motion_calibration.json",
        {
            "forward": {"a": 0.32, "b": 0.01, "ms": 900, "approved_by_user": True},
            "backward": {"a": -0.10, "b": 0.0, "ms": 350, "approved_by_user": True},
            "turn_left_90": {
                "a": 0.0,
                "b": 0.24,
                "ms": 1800,
                "target_angle_deg": 90,
                "last_imu_yaw_delta_deg": 88.5,
                "approved_by_user": True,
            },
            "turn_right_90": {
                "a": 0.0,
                "b": -0.12,
                "ms": 1600,
                "target_angle_deg": 90,
                "last_imu_yaw_delta_deg": -91.0,
                "approved_by_user": True,
            },
            "ready_for_full_path_following": False,
        },
    )

    resolved = calibration.resolve_physical_calibration(
        motion_calibration_json=motion,
        fine_calibration_json=tmp_path / "missing_fine.json",
        turn_calibration_json=tmp_path / "missing_turn.json",
        turn_angle_calibration_json=tmp_path / "missing_angle.json",
        smooth_turn_calibration_json=tmp_path / "missing_smooth.json",
        calibration_mode="auto",
    )

    assert resolved["forward"] == {"a": 0.32, "b": 0.01, "ms": 900, "source": "motion_calibration.json"}
    assert resolved["backward"] == {"a": -0.10, "b": 0.0, "ms": 350, "source": "motion_calibration.json"}
    assert resolved["connector_mode_effective"] == "angle_calibrated"
    assert resolved["fallback_to_repeated_pulses"] is False
    assert calibration.connector_primitive(resolved, "left")["pulse_ms"] == 1800
    assert resolved["ready_for_full_path_following"] is False


def test_set_motion_calibration_preset_writes_manual_high_soft_right(tmp_path: Path) -> None:
    cal_path = tmp_path / "cal" / "motion_calibration.json"
    _write_json(
        cal_path,
        {
            "forward": {"a": 0.25, "b": 0.0, "ms": 700, "approved_by_user": True},
            "operator_note": "preserve me",
            "ready_for_full_path_following": False,
        },
    )
    out_dir = tmp_path / "manual_override"

    rc = cli.main(
        [
            "set-motion-calibration",
            "--preset",
            "field_manual_high_except_soft_right",
            "--calibration-out",
            str(cal_path),
            "--out-dir",
            str(out_dir),
        ]
    )

    assert rc == 0
    updated = json.loads(cal_path.read_text())
    assert updated["forward"] == {
        "a": 0.30,
        "b": 0.0,
        "ms": 1000,
        "approved_by_user": True,
        "source": "manual_high_preset",
    }
    assert updated["backward"]["a"] == -0.08
    assert updated["backward"]["ms"] == 350
    assert updated["turn_left_90"]["b"] == 0.26
    assert updated["turn_left_90"]["ms"] == 2400
    assert updated["turn_right_90"]["b"] == -0.08
    assert updated["turn_right_90"]["ms"] == 1000
    assert updated["turn_right_90"]["source"] == "manual_soft_right_preset"
    assert updated["operator_note"] == "preserve me"
    assert updated["ready_for_full_path_following"] is False

    summary = json.loads((out_dir / "summary.json").read_text())
    assert summary["mode"] == "set-motion-calibration"
    assert summary["preset"] == "field_manual_high_except_soft_right"
    assert summary["backup_path"] != "NONE"
    assert Path(summary["backup_path"]).exists()
    assert (out_dir / "manual_motion_calibration_change.json").exists()
    assert (out_dir / "motion_calibration_updated.json").exists()
    assert summary["ready_for_full_path_following"] is False


def test_set_motion_calibration_explicit_override_preserves_other_primitives(tmp_path: Path) -> None:
    cal_path = _write_json(
        tmp_path / "cal" / "motion_calibration.json",
        {
            "forward": {
                "a": 0.30,
                "b": 0.0,
                "ms": 1000,
                "approved_by_user": True,
                "source": "manual_high_preset",
            },
            "turn_right_90": {
                "a": 0.0,
                "b": -0.08,
                "ms": 1000,
                "target_angle_deg": 90,
                "approved_by_user": True,
                "source": "manual_soft_right_preset",
            },
            "ready_for_full_path_following": False,
        },
    )
    out_dir = tmp_path / "right_soft"

    rc = cli.main(
        [
            "set-motion-calibration",
            "--primitive",
            "turn-right-90",
            "--a",
            "0.0",
            "--b",
            "-0.06",
            "--ms",
            "800",
            "--target-angle-deg",
            "90",
            "--source",
            "manual_soft_right_test",
            "--calibration-out",
            str(cal_path),
            "--out-dir",
            str(out_dir),
        ]
    )

    assert rc == 0
    updated = json.loads(cal_path.read_text())
    assert updated["forward"]["ms"] == 1000
    assert updated["turn_right_90"] == {
        "a": 0.0,
        "b": -0.06,
        "ms": 800,
        "target_angle_deg": 90.0,
        "approved_by_user": True,
        "source": "manual_soft_right_test",
    }
    change = json.loads((out_dir / "manual_motion_calibration_change.json").read_text())
    assert change["updated_primitives"] == ["turn_right_90"]


def test_set_motion_calibration_rejects_wrong_turn_sign(tmp_path: Path) -> None:
    out_dir = tmp_path / "bad"
    rc = cli.main(
        [
            "set-motion-calibration",
            "--primitive",
            "turn-right-90",
            "--a",
            "0.0",
            "--b",
            "0.06",
            "--ms",
            "800",
            "--target-angle-deg",
            "90",
            "--calibration-out",
            str(tmp_path / "cal" / "motion_calibration.json"),
            "--out-dir",
            str(out_dir),
        ]
    )

    assert rc == 2
    summary = json.loads((out_dir / "summary.json").read_text())
    assert summary["reason"] == "MOTION_CALIBRATION_OVERRIDE_INVALID"
    assert "B < 0" in summary["message"]
    assert summary["ready_for_full_path_following"] is False


def test_manual_calibration_sign_validation() -> None:
    with pytest.raises(ValueError, match="forward calibration requires A > 0"):
        tuning.manual_calibration_entry("forward", a=-0.30, b=0.0, ms=1000, source="bad")
    with pytest.raises(ValueError, match="right/turn-right calibration requires B < 0"):
        tuning.manual_calibration_entry("turn-right-90", a=0.0, b=0.08, ms=1000, source="bad")


def test_mac_execute_plan_loads_approved_motion_calibration(tmp_path: Path) -> None:
    motion = _write_json(
        tmp_path / "motion_calibration.json",
        {"forward": {"a": 0.33, "b": 0.02, "ms": 950, "approved_by_user": True}},
    )
    parser = cli.build_parser()
    args = parser.parse_args(
        [
            "execute-plan",
            "--motion-calibration-json",
            str(motion),
            "--print-plan",
            "--start-lat",
            "35.1",
            "--start-lon",
            "129.1",
            "--goal-mode",
            "bearing_distance",
            "--goal-bearing-deg",
            "90",
            "--goal-distance-m",
            "4",
            "--path-shape",
            "direct_line",
        ]
    )
    resolved = cli.resolve_calibration(args)
    assert resolved["forward"]["a"] == 0.33
    assert resolved["forward"]["b"] == 0.02
    assert resolved["forward"]["ms"] == 950


def test_gps_degraded_continue_policy_does_not_abort_when_imu_available() -> None:
    cache: dict[str, object] = {"lat": 35.1, "lon": 129.1, "degraded": False}
    gps = controller.dead_reckon_gps(
        {"gps_block_reason": "BAD_HDOP", "current_lat": "NA", "current_lon": "NA"},
        cache,
    )
    assert gps["gps_degraded"] is True
    assert gps["gps_cached_used"] is True
    assert geometry.gps_policy_action(bool(gps["gps_degraded"]), "continue") == "continue"
