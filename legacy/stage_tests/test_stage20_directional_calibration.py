from __future__ import annotations

import csv
import json
from pathlib import Path

from tools import build_stage20_directional_calibration as calib
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
        "body_motion_user_report": "forward",
        "turn_angle_report": "none",
        "visible_motion_confirmed": True,
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


def test_stage20b_directional_calibration_detects_asymmetry() -> None:
    calibration = calib.build_calibration(
        forward_rows=[_row(requested_a_cmd=0.16, pulse_ms=500, body_motion_user_report="twitch")],
        backward_rows=[_row(mode="backward", requested_a_cmd=-0.08, pulse_ms=300, body_motion_user_report="backward")],
        turn_left_rows=[_row(mode="turn_left", requested_a_cmd=0.0, requested_b_cmd=0.22, pulse_ms=600, body_motion_user_report="twitch", turn_angle_report="twitch")],
        turn_right_rows=[_row(mode="turn_right", requested_a_cmd=0.0, requested_b_cmd=-0.08, pulse_ms=250, body_motion_user_report="right", turn_angle_report="big_over_45deg")],
    )
    assert calibration["forward_sign"] == 1.0
    assert calibration["forward_min_a_cmd"] == 0.16
    assert calibration["forward_min_pulse_ms"] == 500
    assert calibration["forward_motion_class"] == "twitch"
    assert calibration["backward_min_a_cmd"] == -0.08
    assert calibration["backward_min_pulse_ms"] == 300
    assert calibration["turn_left_gain_class"] == "weak"
    assert calibration["turn_right_gain_class"] == "overstrong"
    assert calibration["turn_asymmetry_detected"] is True
    assert calibration["ready_for_stage21_straight_only"] is True
    assert calibration["ready_for_stage21_turning"] is False
    assert calibration["ready_for_full_path_following"] is False


def test_stage20b_builder_writes_json(tmp_path: Path) -> None:
    forward_dir = tmp_path / "forward"
    backward_dir = tmp_path / "backward"
    left_dir = tmp_path / "left"
    right_dir = tmp_path / "right"
    _write_probe_dir(forward_dir, [_row(requested_a_cmd=0.16, pulse_ms=500)])
    _write_probe_dir(backward_dir, [_row(mode="backward", requested_a_cmd=-0.08, pulse_ms=300, body_motion_user_report="backward")])
    _write_probe_dir(left_dir, [_row(mode="turn_left", requested_a_cmd=0.0, requested_b_cmd=0.22, pulse_ms=600, turn_angle_report="twitch", body_motion_user_report="twitch")])
    _write_probe_dir(right_dir, [_row(mode="turn_right", requested_a_cmd=0.0, requested_b_cmd=-0.08, pulse_ms=250, turn_angle_report="big_over_45deg", body_motion_user_report="right")])
    out = tmp_path / "calibration" / "physical_ab_directional_calibration.json"

    rc = calib.main([
        "--forward-dir", str(forward_dir),
        "--backward-dir", str(backward_dir),
        "--turn-left-dir", str(left_dir),
        "--turn-right-dir", str(right_dir),
        "--out", str(out),
    ])

    assert rc == 0
    data = json.loads(out.read_text(encoding="utf-8"))
    assert data["forward_min_a_cmd"] == 0.16
    assert data["turn_right_gain_class"] == "overstrong"
    assert data["ready_for_stage21_turning"] is False
