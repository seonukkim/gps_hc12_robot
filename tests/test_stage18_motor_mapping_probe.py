from __future__ import annotations

import csv
from pathlib import Path

from tools import check_stage18_motor_mapping_probe
from tools import stage18_motor_mapping_probe


REPO = Path(__file__).resolve().parents[1]
SCRIPT = REPO / "legacy" / "stage_scripts" / "run_stage18_motor_mapping_probe.sh"


def _write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=stage18_motor_mapping_probe.PROBE_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in stage18_motor_mapping_probe.PROBE_FIELDS})


def _safe_row(**overrides: object) -> dict[str, object]:
    row: dict[str, object] = {
        "trial_index": 1,
        "mode": "forward",
        "cmd": 0.06,
        "pulse_ms": 500,
        "seq": 1,
        "arm_command_text": "STAGE18_ARM seq=1",
        "stage18_command_text": "STAGE18_CMD seq=1 left=0.060 right=0.060 ms=500",
        "stop_command_text": "STAGE18_STOP seq=1",
        "requested_left_cmd": 0.06,
        "requested_right_cmd": 0.06,
        "logical_left_cmd": 0.06,
        "logical_right_cmd": 0.06,
        "physical_a_cmd": 0.06,
        "physical_b_cmd": 0.0,
        "final_left_cmd": 0.0,
        "final_right_cmd": 0.0,
        "motor_write_called": True,
        "motor_backend": "servo_pwm",
        "motor_enable_state": "attached",
        "pwm_or_dynamixel_write_status": "OK",
        "firmware_active_target_source": "station_motor_mapping_probe",
        "physical_path_following_enable": False,
        "allow_motor_output": False,
        "physical_output_active_seen": True,
        "physical_output_active_after_stop": False,
        "arm_ack_seen": True,
        "ack_seen": True,
        "stop_seen": True,
        "reject_seen": False,
        "reject_reason": "NONE",
        "left_wheel_user_report": "yes",
        "right_wheel_user_report": "yes",
        "body_motion_user_report": "forward",
        "visible_motion_confirmed": True,
        "motor_power_or_mapping_suspect": False,
        "next_action": "OK",
        "ready_for_full_path_following": False,
    }
    row.update(overrides)
    return row


def test_stage18_script_uses_guarded_flags() -> None:
    text = SCRIPT.read_text(encoding="utf-8")
    assert "STAGE18_MOTOR_MAPPING_PROBE=1" in text
    assert "STAGE18_MAX_CMD=0.08" in text
    assert "STAGE18_MAX_MS=800" in text
    assert "PHYSICAL_PATH_FOLLOWING_ENABLE=0" in text
    assert "PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0" in text
    assert "STAGE16_USB_GUARDED_CRAWL=0" in text
    assert "STAGE17_FIRST_PRIMITIVE_CRAWL=0" in text


def test_stage18_probe_mode_signs() -> None:
    assert stage18_motor_mapping_probe.command_for_probe_mode("forward", 0.08) == (0.08, 0.08)
    assert stage18_motor_mapping_probe.command_for_probe_mode("backward", 0.08) == (-0.08, -0.08)
    assert stage18_motor_mapping_probe.command_for_probe_mode("rotate_left", 0.08) == (-0.08, 0.08)
    assert stage18_motor_mapping_probe.command_for_probe_mode("rotate_right", 0.08) == (0.08, -0.08)
    assert stage18_motor_mapping_probe.command_for_probe_mode("left_wheel_only", 0.08) == (0.08, 0.0)
    assert stage18_motor_mapping_probe.command_for_probe_mode("right_wheel_only", 0.08) == (0.0, 0.08)


def test_stage18_planned_probe_command() -> None:
    planned = stage18_motor_mapping_probe.planned_probe_commands(
        seq=3,
        mode="rotate_right",
        cmd=0.08,
        pulse_ms=800,
    )
    assert planned["arm_command_text"] == "STAGE18_ARM seq=3"
    assert planned["stage18_command_text"] == "STAGE18_CMD seq=3 left=0.080 right=-0.080 ms=800"
    assert planned["stop_command_text"] == "STAGE18_STOP seq=3"


def test_stage18_row_from_trial_extracts_motor_mapping_fields() -> None:
    planned = stage18_motor_mapping_probe.planned_probe_commands(seq=1, mode="forward", cmd=0.08, pulse_ms=800)
    rows = stage18_motor_mapping_probe.station_path_package_tracker.parse_usbdbg_rows(
        "STAGE18 stage18_motor_mapping_probe=true event=ARM stage16_cmd_state=ARMED latched_stop=false\n"
        "STAGE18 stage18_motor_mapping_probe=true event=ACK stage16_cmd_state=ACTIVE "
        "requested_left_cmd=0.080 requested_right_cmd=0.080 logical_left_cmd=0.080 logical_right_cmd=0.080 "
        "physical_a_cmd=0.080 physical_b_cmd=0.000 motor_write_called=true motor_backend=servo_pwm "
        "motor_enable_state=attached pwm_or_dynamixel_write_status=OK physical_output_active=true\n"
        "STAGE18 stage18_motor_mapping_probe=true event=STOP stage16_cmd_state=STOPPED "
        "final_left_cmd=0.000 final_right_cmd=0.000 physical_output_active=false\n"
    )
    row = stage18_motor_mapping_probe.row_from_trial(
        trial_index=1,
        planned=planned,
        rows=rows,
        left_report="yes",
        right_report="yes",
        body_report="forward",
        upper_limit_trial=True,
    )
    assert row["logical_left_cmd"] == "0.080"
    assert row["physical_a_cmd"] == "0.080"
    assert row["motor_write_called"] is True
    assert row["visible_motion_confirmed"] is True
    assert row["ready_for_full_path_following"] is False


def test_stage18_summary_reports_suspect_at_upper_limit() -> None:
    summary = stage18_motor_mapping_probe.build_summary(
        [_safe_row(visible_motion_confirmed=False, motor_power_or_mapping_suspect=True, cmd=0.08, pulse_ms=800)]
    )
    assert summary["motor_power_or_mapping_suspect"] is True
    assert "check battery" in str(summary["next_action"])
    assert summary["ready_for_full_path_following"] is False


def test_stage18_checker_passes_safe_probe(tmp_path: Path) -> None:
    csv_path = tmp_path / "stage18_motor_mapping_probe.csv"
    _write_csv(csv_path, [_safe_row()])
    result = check_stage18_motor_mapping_probe.evaluate_path(csv_path)
    assert result["verdict"] == "PASS"
    assert result["ready_for_full_path_following"] is False


def test_stage18_checker_fails_missing_motor_write_called(tmp_path: Path) -> None:
    csv_path = tmp_path / "stage18_motor_mapping_probe.csv"
    _write_csv(csv_path, [_safe_row(motor_write_called=False)])
    result = check_stage18_motor_mapping_probe.evaluate_path(csv_path)
    assert result["verdict"] == "FAIL"
    assert "MOTOR_WRITE_CALLED_FIELD_FALSE_OR_MISSING" in result["reasons"]


def test_stage18_checker_reports_mapping_suspect(tmp_path: Path) -> None:
    csv_path = tmp_path / "stage18_motor_mapping_probe.csv"
    _write_csv(csv_path, [_safe_row(visible_motion_confirmed=False, motor_power_or_mapping_suspect=True)])
    result = check_stage18_motor_mapping_probe.evaluate_path(csv_path)
    assert result["verdict"] == "PASS"
    assert result["motor_power_or_mapping_suspect"] is True


def test_stage18_checker_fails_continuous_output(tmp_path: Path) -> None:
    csv_path = tmp_path / "stage18_motor_mapping_probe.csv"
    _write_csv(csv_path, [_safe_row(physical_output_active_after_stop=True)])
    result = check_stage18_motor_mapping_probe.evaluate_path(csv_path)
    assert result["verdict"] == "FAIL"
    assert "OUTPUT_ACTIVE_AFTER_STOP" in result["reasons"]


def test_stage18_checker_fails_compile_time_target(tmp_path: Path) -> None:
    csv_path = tmp_path / "stage18_motor_mapping_probe.csv"
    _write_csv(csv_path, [_safe_row(firmware_active_target_source="compile_time")])
    result = check_stage18_motor_mapping_probe.evaluate_path(csv_path)
    assert result["verdict"] == "FAIL"
    assert "COMPILE_TIME_TARGET_ACTIVE_DURING_STAGE18" in result["reasons"]
