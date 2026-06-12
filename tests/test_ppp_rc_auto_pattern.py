"""Untethered rc-auto-pattern mode: firmware flag builder + print-cmd path."""

from __future__ import annotations

import json
from pathlib import Path

from tools.physical_path_planning import cli


def test_rc_auto_pattern_flags_combine_manual_profile_imu_and_pattern() -> None:
    flags = cli.rc_auto_pattern_firmware_flags(
        lanes=4,
        lane_ms=4200,
        step_ms=1400,
        forward_a=0.30,
        reverse_a=-0.30,
        turn_b_left=0.24,
        turn_b_right=-0.12,
        turn_target_deg=90.0,
        turn_tol_deg=8.0,
        turn_timeout_ms=15000,
        pause_ms=500,
    )
    # Proven manual profile basis (RC manual keeps working in MANUAL). The
    # default base is full-telemetry-ppm: firmware-default PPM decode, the
    # only configuration field-proven to decode this receiver's channels.
    assert "-DMANUAL_CONTROL_PPM=1" in flags
    assert "-DMANUAL_FORWARD_SIGN=-1" in flags
    assert "-DPPM_INTERRUPT_EDGE_FALLING=1" not in flags
    # IMU yaw must be compiled in for the pivot feedback.
    assert "-DIMU_ENABLE=1" in flags
    assert "-DIMU_ENABLE=0" not in flags
    # The onboard pattern and its parameters.
    assert "-DRC_AUTO_PATTERN=1" in flags
    assert "-DRC_AUTO_PATTERN_LANES=4" in flags
    assert "-DRC_AUTO_PATTERN_LANE_MS=4200" in flags
    assert "-DRC_AUTO_PATTERN_STEP_MS=1400" in flags
    assert "-DRC_AUTO_PATTERN_TURN_B_RIGHT=-0.12f" in flags
    # Heading-hold straights + RC-loss grace defaults are always baked in so
    # lanes stay straight untethered and a radio glitch cannot kill a run.
    assert "-DRC_AUTO_PATTERN_HEADING_KP=0.015f" in flags
    assert "-DRC_AUTO_PATTERN_HEADING_MAX_B=0.25f" in flags
    assert "-DRC_AUTO_PATTERN_DRIVE_B_TRIM=0.0f" in flags
    assert "-DRC_AUTO_PATTERN_RC_LOSS_GRACE_MS=1500" in flags
    # This drivetrain's yaw response to B while DRIVING is inverted vs a
    # stationary pivot (2026-06-12 field log), so the lane feedback sign
    # defaults to -1. Square corners come from coast-compensated stops plus
    # settle-verify retries, with an anti-donut lane abort as the safety net.
    assert "-DRC_AUTO_PATTERN_DRIVE_STEER_SIGN=-1.0f" in flags
    assert "-DRC_AUTO_PATTERN_DRIVE_ABORT_ERR_DEG=60.0" in flags
    assert "-DRC_AUTO_PATTERN_TURN_COAST_S=0.15f" in flags
    assert "-DRC_AUTO_PATTERN_TURN_SETTLE_RETRIES=2" in flags
    # Autonomy gates stay off: this is the RC-switch pattern, not path following.
    assert "-DPHYSICAL_PATH_FOLLOWING_ENABLE=0" in flags
    assert "-DAUTO_MOTION_ARMED=0" in flags


def test_rc_auto_pattern_flags_accept_heading_hold_overrides() -> None:
    flags = cli.rc_auto_pattern_firmware_flags(
        lanes=4,
        lane_ms=4200,
        step_ms=1400,
        forward_a=0.30,
        reverse_a=-0.30,
        turn_b_left=0.24,
        turn_b_right=-0.12,
        turn_target_deg=90.0,
        turn_tol_deg=8.0,
        turn_timeout_ms=15000,
        pause_ms=500,
        heading_kp=0.02,
        heading_hold_max_b=0.3,
        drive_b_trim=-0.05,
        drive_steer_sign=1.0,
        drive_abort_err_deg=45.0,
        turn_coast_s=0.2,
        turn_settle_retries=3,
        rc_loss_grace_ms=2000,
    )
    assert "-DRC_AUTO_PATTERN_HEADING_KP=0.02f" in flags
    assert "-DRC_AUTO_PATTERN_HEADING_MAX_B=0.3f" in flags
    assert "-DRC_AUTO_PATTERN_DRIVE_B_TRIM=-0.05f" in flags
    assert "-DRC_AUTO_PATTERN_DRIVE_STEER_SIGN=1.0f" in flags
    assert "-DRC_AUTO_PATTERN_DRIVE_ABORT_ERR_DEG=45.0" in flags
    assert "-DRC_AUTO_PATTERN_TURN_COAST_S=0.2f" in flags
    assert "-DRC_AUTO_PATTERN_TURN_SETTLE_RETRIES=3" in flags
    assert "-DRC_AUTO_PATTERN_RC_LOSS_GRACE_MS=2000" in flags


def test_rc_auto_pattern_print_cmd_writes_config_without_serial(tmp_path: Path) -> None:
    rc = cli.main(
        [
            "rc-auto-pattern",
            "--print-cmd",
            "--lane-ms", "4200",
            "--step-ms", "1400",
            "--out-dir", str(tmp_path),
        ]
    )
    assert rc == 0
    config = json.loads((tmp_path / "rc_auto_pattern_config.json").read_text())
    assert config["mode"] == "rc-auto-pattern"
    assert config["untethered"] is True
    assert config["lane_ms"] == 4200
    assert config["heading_kp"] == 0.015
    assert config["rc_loss_grace_ms"] == 1500
    assert config["drive_steer_sign"] == -1.0
    assert config["turn_settle_retries"] == 2
    assert config["ready_for_full_path_following"] is False
    # The operator can SEE the expected path before going untethered.
    assert config["estimated_lane_m"] > 0
    assert (tmp_path / "rc_auto_pattern_preview.png").exists()
    summary = json.loads((tmp_path / "summary.json").read_text())
    assert summary["reason"] == "COMMAND_PRINTED"
