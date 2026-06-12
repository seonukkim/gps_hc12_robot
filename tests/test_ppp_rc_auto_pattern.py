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
    # Autonomy gates stay off: this is the RC-switch pattern, not path following.
    assert "-DPHYSICAL_PATH_FOLLOWING_ENABLE=0" in flags
    assert "-DAUTO_MOTION_ARMED=0" in flags


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
    assert config["ready_for_full_path_following"] is False
    summary = json.loads((tmp_path / "summary.json").read_text())
    assert summary["reason"] == "COMMAND_PRINTED"
