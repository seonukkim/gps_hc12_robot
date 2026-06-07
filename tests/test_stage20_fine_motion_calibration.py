from __future__ import annotations

import csv
import json
from pathlib import Path

from tools import build_stage20_fine_motion_calibration as fine
from tools import stage20_physical_ab_probe


def _row(**overrides: object) -> dict[str, object]:
    row: dict[str, object] = {
        "trial_index": 1,
        "mode": "forward",
        "cmd": 0.16,
        "pulse_ms": 500,
        "seq": 1,
        "forward_sign": 1.0,
        "turn_sign": 1.0,
        "requested_a_cmd": 0.16,
        "requested_b_cmd": 0.0,
        "left_wheel_user_report": "yes",
        "right_wheel_user_report": "yes",
        "body_motion_user_report": "twitch",
        "turn_angle_report": "none",
        "motion_class": "twitch",
        "visible_motion_confirmed": True,
        "valid_trial": True,
        "invalid_reason": "NONE",
        "ready_for_full_path_following": False,
    }
    row.update(overrides)
    return row


def _write_probe_dir(path: Path, rows: list[dict[str, object]]) -> None:
    path.mkdir(parents=True, exist_ok=True)
    with (path / "stage20_physical_ab_probe.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=stage20_physical_ab_probe.PROBE_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in stage20_physical_ab_probe.PROBE_FIELDS})


def _base() -> dict[str, object]:
    return {
        "forward_sign": 1.0,
        "turn_sign": 1.0,
        "ready_for_stage21_turning": False,
        "ready_for_full_path_following": False,
    }


def test_stage20c_twitch_does_not_become_recommended_forward() -> None:
    calibration = fine.build_fine_calibration(
        forward_rows=[
            _row(requested_a_cmd=0.16, pulse_ms=500, body_motion_user_report="twitch", motion_class="twitch"),
        ],
        backward_rows=[
            _row(mode="backward", requested_a_cmd=-0.08, pulse_ms=300, body_motion_user_report="backward", motion_class="visible"),
        ],
        base_calibration=_base(),
    )
    assert calibration["forward_twitch_a_cmd"] == 0.16
    assert calibration["forward_twitch_pulse_ms"] == 500
    assert calibration["forward_visible_a_cmd"] is None
    assert calibration["forward_recommended_crawl_a_cmd"] is None
    assert calibration["forward_recommended_crawl_pulse_ms"] is None
    assert calibration["stage21_forward_a_cmd"] is None
    assert calibration["stage21_forward_pulse_ms"] is None
    assert calibration["forward_motion_class"] == "twitch"
    assert calibration["motion_class_source"] == "forward_twitch_unresolved"
    assert calibration["forward_threshold_unresolved"] is True
    assert calibration["ready_for_stage21_forward"] is False
    assert calibration["ready_for_stage21_straight_only"] is False
    assert calibration["forward_specific_deadband_or_sign_suspect"] is True
    assert calibration["motor_power_or_mapping_suspect"] is False
    assert calibration["ready_for_full_path_following"] is False


def test_stage20c_visible_forward_becomes_recommended() -> None:
    calibration = fine.build_fine_calibration(
        forward_rows=[
            _row(requested_a_cmd=0.16, pulse_ms=500, body_motion_user_report="twitch", motion_class="twitch"),
            _row(requested_a_cmd=0.20, pulse_ms=650, body_motion_user_report="forward", motion_class="visible"),
        ],
        backward_rows=[
            _row(mode="backward", requested_a_cmd=-0.08, pulse_ms=300, body_motion_user_report="backward", motion_class="visible"),
        ],
        base_calibration=_base(),
    )
    assert calibration["forward_visible_a_cmd"] == 0.20
    assert calibration["forward_recommended_crawl_a_cmd"] == 0.20
    assert calibration["forward_recommended_crawl_pulse_ms"] == 650
    assert calibration["stage21_forward_a_cmd"] == 0.20
    assert calibration["stage21_forward_pulse_ms"] == 650
    assert calibration["motion_class_source"] == "forward_visible"
    assert calibration["ready_for_stage21_forward"] is True
    assert calibration["ready_for_full_path_following"] is False


def test_stage20c_backward_recommended_is_capped() -> None:
    calibration = fine.build_fine_calibration(
        forward_rows=[
            _row(requested_a_cmd=0.20, pulse_ms=650, body_motion_user_report="forward", motion_class="visible"),
        ],
        backward_rows=[
            _row(mode="backward", requested_a_cmd=-0.18, pulse_ms=800, body_motion_user_report="backward", motion_class="visible"),
        ],
        base_calibration=_base(),
    )
    assert calibration["backward_visible_a_cmd"] == -0.18
    assert calibration["backward_recommended_crawl_a_cmd"] == -0.12
    assert calibration["backward_recommended_crawl_pulse_ms"] == 500
    assert calibration["backward_is_sensitive"] is False


def test_stage20c_invalid_latched_stop_trial_is_ignored() -> None:
    calibration = fine.build_fine_calibration(
        forward_rows=[
            _row(
                requested_a_cmd=0.25,
                pulse_ms=800,
                body_motion_user_report="none",
                motion_class="none",
                valid_trial=False,
                invalid_reason="LATCHED_STOP",
            ),
        ],
        backward_rows=[
            _row(mode="backward", requested_a_cmd=-0.08, pulse_ms=300, body_motion_user_report="backward", motion_class="visible"),
        ],
        base_calibration=_base(),
    )
    assert calibration["forward_visible_a_cmd"] is None
    assert calibration["stage21_forward_a_cmd"] is None
    assert calibration["ready_for_stage21_forward"] is False
    assert calibration["motor_power_or_mapping_suspect"] is False


def test_stage20c_invalid_rc_trial_is_ignored() -> None:
    calibration = fine.build_fine_calibration(
        forward_rows=[
            _row(
                requested_a_cmd=0.25,
                pulse_ms=800,
                body_motion_user_report="none",
                motion_class="none",
                valid_trial=False,
                invalid_reason="RC_INVALID",
            ),
        ],
        backward_rows=[],
        base_calibration=_base(),
    )
    assert calibration["forward_visible_a_cmd"] is None
    assert calibration["stage21_forward_a_cmd"] is None
    assert calibration["ready_for_stage21_forward"] is False
    assert calibration["motor_power_or_mapping_suspect"] is True


def test_stage20c_builder_writes_json(tmp_path: Path) -> None:
    forward_dir = tmp_path / "forward"
    backward_dir = tmp_path / "backward"
    base = tmp_path / "base.json"
    out = tmp_path / "fine" / "physical_ab_fine_motion_calibration.json"
    base.write_text(json.dumps(_base()), encoding="utf-8")
    _write_probe_dir(forward_dir, [
        _row(requested_a_cmd=0.16, pulse_ms=500, body_motion_user_report="twitch", motion_class="twitch"),
        _row(requested_a_cmd=0.22, pulse_ms=650, body_motion_user_report="forward", motion_class="visible"),
    ])
    _write_probe_dir(backward_dir, [
        _row(mode="backward", requested_a_cmd=-0.08, pulse_ms=300, body_motion_user_report="backward", motion_class="visible"),
    ])

    rc = fine.main([
        "--forward-fine-dir", str(forward_dir),
        "--backward-fine-dir", str(backward_dir),
        "--base-calibration-json", str(base),
        "--out", str(out),
    ])

    assert rc == 0
    data = json.loads(out.read_text(encoding="utf-8"))
    assert data["stage21_forward_a_cmd"] == 0.22
    assert data["stage21_backward_a_cmd"] == -0.08
    assert data["turn_disabled"] is True
