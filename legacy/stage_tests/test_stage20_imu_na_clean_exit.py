"""Stage20 must STOP CLEANLY (no traceback) when IMU yaw is unavailable.

The guarantee under test: when an operator runs ``--imu-angle-compare`` but the
BMI160 never produces a usable yaw, the probe used to raise ``RuntimeError`` and
crash. It must instead exit cleanly -- surfacing ``imu_heading_block_reason=
IMU_YAW_NOT_AVAILABLE`` in the summary and writing NO turn-angle calibration JSON.
These are pure-logic tests of the decision helper, the summary surfacing, and the
save gate; none of them open serial or touch firmware.
"""
from __future__ import annotations

from pathlib import Path

from tools import stage20_physical_ab_probe as stage20


def test_block_reason_when_compare_enabled_and_yaw_missing() -> None:
    assert (
        stage20.imu_yaw_block_reason(imu_angle_compare=True, yaw_before_deg=None)
        == "IMU_YAW_NOT_AVAILABLE"
    )


def test_no_block_reason_when_yaw_present() -> None:
    assert stage20.imu_yaw_block_reason(imu_angle_compare=True, yaw_before_deg=12.5) is None


def test_no_block_reason_when_compare_disabled() -> None:
    # Without --imu-angle-compare a missing yaw is irrelevant: never block.
    assert stage20.imu_yaw_block_reason(imu_angle_compare=False, yaw_before_deg=None) is None
    assert stage20.imu_yaw_block_reason(imu_angle_compare=False, yaw_before_deg=0.0) is None


def test_build_summary_defaults_block_reason_to_none() -> None:
    summary = stage20.build_summary([])
    assert summary["imu_heading_block_reason"] == "NONE"
    assert summary["ready_for_full_path_following"] is False


def test_build_summary_surfaces_block_reason() -> None:
    summary = stage20.build_summary([], imu_heading_block_reason="IMU_YAW_NOT_AVAILABLE")
    assert summary["imu_heading_block_reason"] == "IMU_YAW_NOT_AVAILABLE"
    # The readiness flag is never flipped on a blocked run.
    assert summary["ready_for_full_path_following"] is False


def test_save_gate_writes_nothing_when_yaw_delta_missing(tmp_path: Path) -> None:
    # Even when the operator confirms "yes", a missing yaw delta (the field state
    # that triggers the block) must NOT write the turn-angle calibration JSON.
    path = tmp_path / "physical_ab_turn_angle_calibration.json"
    planned = stage20.planned_probe_commands(
        seq=1,
        mode="turn_left",
        cmd=0.26,
        pulse_ms=700,
        max_abs_a=0.35,
        max_abs_b=0.35,
    )
    angle = stage20.angle_comparison_from_yaw(
        yaw_before_deg=0.0,
        yaw_after_deg=None,
        target_angle_deg=90.0,
        angle_tolerance_deg=10.0,
    )
    assert angle["yaw_delta_deg"] is None
    row = {
        "ack_seen": True,
        "stop_seen": True,
        "reject_seen": False,
        "physical_output_active_after_stop": False,
        "valid_trial": True,
        "final_left_cmd": "0.000",
        "final_right_cmd": "0.000",
    }
    assert (
        stage20.save_turn_calibration_result(
            path=path,
            planned=planned,
            row=row,
            angle=angle,
            visual_confirmation="yes",
        )
        is False
    )
    assert not path.exists()
