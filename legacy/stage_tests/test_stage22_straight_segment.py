from __future__ import annotations

import csv
from pathlib import Path

from tools import check_stage22_straight_segment
from tools import stage22_straight_segment


REPO = Path(__file__).resolve().parents[1]
SCRIPT = REPO / "legacy" / "stage_scripts" / "run_stage22_straight_segment.sh"


def _row(**overrides: object) -> dict[str, object]:
    row: dict[str, object] = {
        "pulse_index": 1,
        "seq": 1,
        "path_package_loaded": True,
        "fine_calibration_json": "fine.json",
        "straight_only": True,
        "turn_disabled": True,
        "selected_a_cmd": 0.30,
        "selected_b_cmd": 0.0,
        "selected_pulse_ms": 800,
        "max_abs_a": 0.35,
        "max_abs_b": 0.25,
        "max_ms": 1000,
        "command_within_firmware_limits": True,
        "arm_command_text": "STAGE20_ARM seq=1",
        "stage20_command_text": "STAGE20_CMD seq=1 a=0.300 b=0.000 ms=800",
        "stop_command_text": "STAGE20_STOP seq=1",
        "arm_ack_seen": True,
        "ack_seen": True,
        "stop_seen": True,
        "reject_seen": False,
        "reject_reason": "NONE",
        "physical_output_active_seen": True,
        "physical_output_active_after_stop": False,
        "final_left_cmd": "0.000",
        "final_right_cmd": "0.000",
        "final_left_cmd_zero": True,
        "final_right_cmd_zero": True,
        "body_motion_user_report": "forward",
        "visible_forward_or_twitch": True,
        "ready_for_full_path_following": False,
    }
    row.update(overrides)
    return row


def _write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=stage22_straight_segment.STAGE22_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in stage22_straight_segment.STAGE22_FIELDS})


def test_stage22_script_uses_stage20_guarded_transport() -> None:
    text = SCRIPT.read_text(encoding="utf-8")
    assert "STAGE20_PHYSICAL_AB_GUARDED_CRAWL=1" in text
    assert "STAGE20_MAX_ABS_A=${MAX_ABS_A}" in text
    assert "STAGE20_MAX_MS=${MAX_MS}" in text
    assert "PHYSICAL_PATH_FOLLOWING_ENABLE=0" in text
    assert "PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0" in text
    assert "NOT FULL PATH FOLLOWING" in text
    assert "B forced zero" in text
    assert "--skip-upload|--no-upload" in text
    assert 'if [[ "$UPLOAD_ENABLED" == "true" ]]; then' in text
    assert "arduino-cli compile" in text
    assert "arduino-cli upload -p \"$PORT\"" in text
    assert "awk '/OpenRB-150/ {print $1; exit}'" in text
    assert "hc12_ports_ignored=${HC12_PORTS_IGNORED}" in text
    assert "/dev/tty.usbmodem" not in text


def test_stage22_selects_fine_calibrated_straight_command() -> None:
    a_cmd, b_cmd, pulse_ms = stage22_straight_segment.select_straight_command(
        {
            "stage21_forward_a_cmd": 0.30,
            "stage21_forward_pulse_ms": 800,
        }
    )
    assert a_cmd == 0.30
    assert b_cmd == 0.0
    assert pulse_ms == 800


def test_stage22_uses_one_serial_session_for_repeated_pulses() -> None:
    text = (REPO / "tools" / "stage22_straight_segment.py").read_text(encoding="utf-8")
    assert text.count("with serial.Serial") == 1
    assert "for pulse_index in range(1, args.repeat_count + 1):" in text


def test_stage22_planned_pulse_forces_b_zero() -> None:
    planned = stage22_straight_segment.planned_pulse(
        pulse_index=2,
        seq=2,
        a_cmd=0.30,
        pulse_ms=800,
        max_abs_a=0.35,
        max_abs_b=0.25,
        max_ms=1000,
    )
    assert planned["stage20_command_text"] == "STAGE20_CMD seq=2 a=0.300 b=0.000 ms=800"
    assert planned["selected_b_cmd"] == 0.0
    assert planned["straight_only"] is True
    assert planned["turn_disabled"] is True
    assert planned["command_within_firmware_limits"] is True
    assert planned["ready_for_full_path_following"] is False


def test_stage22_row_from_pulse_marks_reject() -> None:
    planned = stage22_straight_segment.planned_pulse(pulse_index=1, seq=1, a_cmd=0.30, pulse_ms=800)
    rows = stage22_straight_segment.station_path_package_tracker.parse_usbdbg_rows(
        "STAGE20 event=ARM stage20_cmd_state=ARMED\n"
        "STAGE20 event=REJECT stage16_reject_reason=RC_INVALID\n"
        "STAGE20 event=STOP final_left_cmd=0.000 final_right_cmd=0.000 physical_output_active=false\n"
    )
    row = stage22_straight_segment.row_from_pulse(
        planned=planned,
        rows=rows,
        body_motion_user_report="unknown",
    )
    assert row["reject_seen"] is True
    assert row["reject_reason"] == "RC_INVALID"
    assert row["ready_for_full_path_following"] is False


def test_stage22_checker_passes_when_two_of_three_visible(tmp_path: Path) -> None:
    csv_path = tmp_path / "stage22_straight_segment.csv"
    _write_csv(csv_path, [
        _row(pulse_index=1, seq=1, body_motion_user_report="forward", visible_forward_or_twitch=True),
        _row(pulse_index=2, seq=2, body_motion_user_report="twitch", visible_forward_or_twitch=True),
        _row(pulse_index=3, seq=3, body_motion_user_report="none", visible_forward_or_twitch=False),
    ])
    result = check_stage22_straight_segment.evaluate_path(csv_path)
    assert result["verdict"] == "PASS"
    assert result["visible_forward_or_twitch_count"] == 2
    assert result["ready_for_full_path_following"] is False


def test_stage22_checker_passes_single_visible_pulse(tmp_path: Path) -> None:
    csv_path = tmp_path / "stage22_straight_segment.csv"
    _write_csv(csv_path, [
        _row(pulse_index=1, seq=1, body_motion_user_report="twitch", visible_forward_or_twitch=True),
    ])
    result = check_stage22_straight_segment.evaluate_path(
        csv_path,
        expected_repeat_count=1,
        required_visible_count=2,
    )
    assert result["verdict"] == "PASS"
    assert result["visible_forward_or_twitch_count"] == 1
    assert result["ready_for_full_path_following"] is False


def test_stage22_summary_passes_single_visible_pulse_for_next_stage() -> None:
    summary = stage22_straight_segment.build_summary(
        [_row(body_motion_user_report="twitch", visible_forward_or_twitch=True)],
        repeat_count=1,
        selected_a=0.30,
        selected_b=0.0,
        pulse_ms=800,
    )
    assert summary["ready_for_stage23_turn_calibration"] is True
    assert summary["ready_for_full_path_following"] is False


def test_stage22_checker_fails_final_nonzero(tmp_path: Path) -> None:
    csv_path = tmp_path / "stage22_straight_segment.csv"
    _write_csv(csv_path, [
        _row(pulse_index=1, seq=1),
        _row(pulse_index=2, seq=2),
        _row(pulse_index=3, seq=3, final_left_cmd="0.020", final_left_cmd_zero=False),
    ])
    result = check_stage22_straight_segment.evaluate_path(csv_path)
    assert result["verdict"] == "FAIL"
    assert "FINAL_COMMANDS_NONZERO" in result["reasons"]


def test_stage22_checker_fails_on_reject(tmp_path: Path) -> None:
    csv_path = tmp_path / "stage22_straight_segment.csv"
    _write_csv(csv_path, [
        _row(pulse_index=1, seq=1),
        _row(pulse_index=2, seq=2, reject_seen=True, reject_reason="RC_INVALID"),
        _row(pulse_index=3, seq=3),
    ])
    result = check_stage22_straight_segment.evaluate_path(csv_path)
    assert result["verdict"] == "FAIL"
    assert "REJECT_SEEN" in result["reasons"]


def test_stage22_checker_fails_when_b_not_zero(tmp_path: Path) -> None:
    csv_path = tmp_path / "stage22_straight_segment.csv"
    _write_csv(csv_path, [
        _row(pulse_index=1, seq=1),
        _row(pulse_index=2, seq=2, selected_b_cmd=0.01),
        _row(pulse_index=3, seq=3),
    ])
    result = check_stage22_straight_segment.evaluate_path(csv_path)
    assert result["verdict"] == "FAIL"
    assert "B_NOT_ZERO" in result["reasons"]


def test_stage22_checker_fails_when_visible_below_threshold(tmp_path: Path) -> None:
    csv_path = tmp_path / "stage22_straight_segment.csv"
    _write_csv(csv_path, [
        _row(pulse_index=1, seq=1, body_motion_user_report="forward", visible_forward_or_twitch=True),
        _row(pulse_index=2, seq=2, body_motion_user_report="none", visible_forward_or_twitch=False),
        _row(pulse_index=3, seq=3, body_motion_user_report="none", visible_forward_or_twitch=False),
    ])
    result = check_stage22_straight_segment.evaluate_path(csv_path)
    assert result["verdict"] == "FAIL"
    assert "VISIBLE_FORWARD_OR_TWITCH_BELOW_THRESHOLD" in result["reasons"]
