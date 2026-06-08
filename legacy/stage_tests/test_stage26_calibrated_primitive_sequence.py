from __future__ import annotations

import csv
import json
from argparse import Namespace
from pathlib import Path

from tools import check_stage26_calibrated_primitive_sequence
from tools import stage26_calibrated_primitive_sequence


REPO = Path(__file__).resolve().parents[1]
SCRIPT = REPO / "legacy" / "stage_scripts" / "run_stage26_calibrated_primitive_sequence.sh"


def _write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=stage26_calibrated_primitive_sequence.STAGE26_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in stage26_calibrated_primitive_sequence.STAGE26_FIELDS})


def _row(**overrides: object) -> dict[str, object]:
    row: dict[str, object] = {
        "primitive_index": 1,
        "primitive_name": "forward",
        "repeat_index": 1,
        "a_cmd": 0.30,
        "b_cmd": 0.0,
        "pulse_ms": 800,
        "arm_seen": True,
        "ack_seen": True,
        "stop_seen": True,
        "reject_seen": False,
        "reject_reason": "NONE",
        "rc_ok_before_cmd": "true",
        "neutral_ok_before_cmd": "true",
        "physical_output_active_seen": True,
        "physical_output_active_after_stop": False,
        "final_left_cmd": "0.000",
        "final_right_cmd": "0.000",
        "final_left_cmd_zero": True,
        "final_right_cmd_zero": True,
        "user_motion_report": "forward",
        "user_turn_angle_report": "none",
        "valid_primitive": True,
        "invalid_reason": "NONE",
        "ready_for_full_path_following": False,
    }
    row.update(overrides)
    return row


def test_stage26_loads_confirmed_calibration_values() -> None:
    primitive_map = stage26_calibrated_primitive_sequence.build_primitive_map(
        {
            "stage21_forward_a_cmd": 0.30,
            "stage21_forward_pulse_ms": 800,
            "stage21_backward_a_cmd": -0.08,
            "stage21_backward_pulse_ms": 300,
        },
        {
            "turn_left_min_b_cmd": 0.26,
            "turn_left_min_pulse_ms": 700,
            "turn_right_min_b_cmd": -0.08,
            "turn_right_min_pulse_ms": 250,
        },
    )
    assert primitive_map["forward"]["a_cmd"] == 0.30
    assert primitive_map["forward"]["b_cmd"] == 0.0
    assert primitive_map["forward"]["pulse_ms"] == 800
    assert primitive_map["backward"]["a_cmd"] == -0.08
    assert primitive_map["backward"]["b_cmd"] == 0.0
    assert primitive_map["backward"]["pulse_ms"] == 300
    assert primitive_map["turn_left"]["a_cmd"] == 0.0
    assert primitive_map["turn_left"]["b_cmd"] == 0.26
    assert primitive_map["turn_left"]["pulse_ms"] == 700
    assert primitive_map["turn_right"]["a_cmd"] == 0.0
    assert primitive_map["turn_right"]["b_cmd"] == -0.08
    assert primitive_map["turn_right"]["pulse_ms"] == 250


def test_stage26_parses_repeat_syntax() -> None:
    sequence = stage26_calibrated_primitive_sequence.parse_sequence("forward,turn_leftx2,backward,turn_right")
    assert [item["primitive_name"] for item in sequence] == [
        "forward",
        "turn_left",
        "turn_left",
        "backward",
        "turn_right",
    ]
    assert sequence[1]["repeat_index"] == 1
    assert sequence[2]["repeat_index"] == 2


def test_stage26_build_plan_uses_expected_commands() -> None:
    primitive_map = stage26_calibrated_primitive_sequence.build_primitive_map({}, {})
    plan = stage26_calibrated_primitive_sequence.build_plan(
        "forward,turn_left,backward,turn_right",
        primitive_map,
    )
    assert plan[0]["stage20_command_text"] == "STAGE20_CMD seq=1 a=0.300 b=0.000 ms=800"
    assert plan[1]["stage20_command_text"] == "STAGE20_CMD seq=2 a=0.000 b=0.260 ms=700"
    assert plan[2]["stage20_command_text"] == "STAGE20_CMD seq=3 a=-0.080 b=0.000 ms=300"
    assert plan[3]["stage20_command_text"] == "STAGE20_CMD seq=4 a=0.000 b=-0.080 ms=250"
    assert all(row["ready_for_full_path_following"] is False for row in plan)


def test_stage26_row_requires_stop_after_primitive() -> None:
    planned = stage26_calibrated_primitive_sequence.build_plan(
        "forward",
        stage26_calibrated_primitive_sequence.build_primitive_map({}, {}),
    )[0]
    rows = stage26_calibrated_primitive_sequence.station_path_package_tracker.parse_usbdbg_rows(
        "STAGE20 event=ARM rc_ok=true neutral_ok=true physical_output_active=false\n"
        "STAGE20 event=ACK physical_output_active=true\n"
    )
    row = stage26_calibrated_primitive_sequence.row_from_primitive(
        planned=planned,
        rows=rows,
        user_motion_report="unknown",
        user_turn_angle_report="none",
    )
    assert row["valid_primitive"] is False
    assert row["invalid_reason"] == "STOP_MISSING"
    assert row["ready_for_full_path_following"] is False


def test_stage26_row_aborts_on_reject_and_rc_invalid() -> None:
    planned = stage26_calibrated_primitive_sequence.build_plan(
        "forward",
        stage26_calibrated_primitive_sequence.build_primitive_map({}, {}),
    )[0]
    rows = stage26_calibrated_primitive_sequence.station_path_package_tracker.parse_usbdbg_rows(
        "STAGE20 event=ARM rc_ok=true neutral_ok=true physical_output_active=false\n"
        "STAGE20 event=REJECT stage20_reject_reason=RC_INVALID rc_ok=false neutral_ok=false\n"
        "STAGE20 event=STOP final_left_cmd=0.000 final_right_cmd=0.000 physical_output_active=false\n"
    )
    row = stage26_calibrated_primitive_sequence.row_from_primitive(
        planned=planned,
        rows=rows,
        user_motion_report="unknown",
        user_turn_angle_report="none",
    )
    assert row["valid_primitive"] is False
    assert row["invalid_reason"] == "RC_INVALID"


def test_stage26_checker_passes_safe_sequence(tmp_path: Path) -> None:
    csv_path = tmp_path / "stage26_calibrated_primitive_sequence.csv"
    _write_csv(csv_path, [
        _row(primitive_index=1, primitive_name="forward", a_cmd=0.30, b_cmd=0.0, pulse_ms=800),
        _row(primitive_index=2, primitive_name="turn_left", a_cmd=0.0, b_cmd=0.26, pulse_ms=700, user_motion_report="left", user_turn_angle_report="medium_15_45deg"),
    ])
    result = check_stage26_calibrated_primitive_sequence.evaluate_path(csv_path)
    assert result["verdict"] == "PASS"
    assert result["ready_for_full_path_following"] is False


def test_stage26_checker_fails_final_nonzero(tmp_path: Path) -> None:
    csv_path = tmp_path / "stage26_calibrated_primitive_sequence.csv"
    _write_csv(csv_path, [_row(final_left_cmd="0.020", final_left_cmd_zero=False)])
    result = check_stage26_calibrated_primitive_sequence.evaluate_path(csv_path)
    assert result["verdict"] == "FAIL"
    assert "FINAL_COMMANDS_NONZERO" in result["reasons"]


def test_stage26_dry_run_writes_plan_only(tmp_path: Path) -> None:
    fine = tmp_path / "fine.json"
    turn = tmp_path / "turn.json"
    fine.write_text(json.dumps({
        "stage21_forward_a_cmd": 0.30,
        "stage21_forward_pulse_ms": 800,
        "stage21_backward_a_cmd": -0.08,
        "stage21_backward_pulse_ms": 300,
    }), encoding="utf-8")
    turn.write_text(json.dumps({
        "turn_left_min_b_cmd": 0.26,
        "turn_left_min_pulse_ms": 700,
        "turn_right_min_b_cmd": -0.08,
        "turn_right_min_pulse_ms": 250,
    }), encoding="utf-8")
    out_dir = tmp_path / "out"
    result = stage26_calibrated_primitive_sequence.run_stage26(Namespace(
        port="/dev/ttyACM0",
        fine_calibration_json=str(fine),
        turn_calibration_json=str(turn),
        sequence="forward,turn_leftx2",
        require_enter="true",
        interactive_visible_motion="true",
        dry_run="true",
        max_abs_a=0.35,
        max_abs_b=0.35,
        max_ms=1000,
        heartbeat_timeout_s=0.01,
        event_timeout_s=0.01,
        out_dir=str(out_dir),
    ))
    assert result == 0
    assert (out_dir / "primitive_sequence_used.json").exists()
    assert (out_dir / "summary.md").exists()
    assert not (out_dir / "stage26_calibrated_primitive_sequence.csv").exists()


def test_stage26_script_safety_flags() -> None:
    text = SCRIPT.read_text(encoding="utf-8")
    assert "STAGE20_PHYSICAL_AB_GUARDED_CRAWL=1" in text
    assert "PHYSICAL_PATH_FOLLOWING_ENABLE=0" in text
    assert "PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0" in text
    assert "NO HC-12" in text
    assert "--skip-upload|--no-upload" in text
    assert "--dry-run" in text
    assert "ready_for_full_path_following=false" not in text or "check_stage26_calibrated_primitive_sequence.py" in text
