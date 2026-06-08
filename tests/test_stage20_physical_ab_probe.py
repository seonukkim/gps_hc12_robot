from __future__ import annotations

import csv
from pathlib import Path

from tools import check_stage20_physical_ab_probe
from tools import stage20_physical_ab_probe


REPO = Path(__file__).resolve().parents[1]
SCRIPT = REPO / "legacy" / "stage_scripts" / "run_stage20_physical_ab_probe.sh"
FIRMWARE = REPO / "firmware" / "openrb_robot_controller" / "openrb_robot_controller.ino"


def _write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=stage20_physical_ab_probe.PROBE_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in stage20_physical_ab_probe.PROBE_FIELDS})


def _safe_row(**overrides: object) -> dict[str, object]:
    row: dict[str, object] = {
        "trial_index": 1,
        "mode": "forward",
        "cmd": 0.12,
        "pulse_ms": 500,
        "command_axis": "A",
        "axis_limit_used": 0.25,
        "max_abs_a": 0.25,
        "max_abs_b": 0.25,
        "command_within_axis_limit": True,
        "seq": 1,
        "forward_sign": 1.0,
        "turn_sign": 1.0,
        "arm_command_text": "STAGE20_ARM seq=1",
        "stage20_command_text": "STAGE20_CMD seq=1 a=0.120 b=0.000 ms=500",
        "stop_command_text": "STAGE20_STOP seq=1",
        "requested_a_cmd": 0.12,
        "requested_b_cmd": 0.0,
        "physical_a_cmd": 0.12,
        "physical_b_cmd": 0.0,
        "manual_equivalent_forward_cmd": 0.12,
        "manual_equivalent_turn_cmd": 0.0,
        "final_left_cmd": 0.0,
        "final_right_cmd": 0.0,
        "motor_write_called": True,
        "motor_backend": "servo_pwm",
        "motor_enable_state": "attached",
        "pwm_or_dynamixel_write_status": "OK",
        "wheel_to_physical_mapping": "physical_ab_manual_equivalent",
        "firmware_active_target_source": "station_physical_ab_guarded",
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


def test_stage20_script_uses_stage20_only_flags() -> None:
    text = SCRIPT.read_text(encoding="utf-8")
    assert "STAGE20_PHYSICAL_AB_GUARDED_CRAWL=1" in text
    assert "STAGE20_MAX_ABS_A=${MAX_ABS_A}" in text
    assert "STAGE20_MAX_ABS_B=${MAX_ABS_B}" in text
    assert "STAGE20_MAX_MS=${MAX_MS}" in text
    assert "--max-abs-a" in text
    assert "--max-abs-b" in text
    assert "--max-ms" in text
    assert "--turn-profile" in text
    assert "left_twitch" in text
    assert "right_twitch" in text
    assert "PHYSICAL_PATH_FOLLOWING_ENABLE=0" in text
    assert "PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0" in text
    assert "STAGE17_FIRST_PRIMITIVE_CRAWL=0" in text
    assert "STAGE18_MOTOR_MAPPING_PROBE=0" in text


def test_stage20_default_signs_match_manual_rc_firmware() -> None:
    text = FIRMWARE.read_text(encoding="utf-8")
    assert "#define MANUAL_FORWARD_SIGN -1" in text
    assert "#define MANUAL_TURN_SIGN 1" in text
    assert stage20_physical_ab_probe.DEFAULT_FORWARD_SIGN == 1.0
    assert stage20_physical_ab_probe.DEFAULT_TURN_SIGN == 1.0


def test_stage20_physical_ab_command_formatting() -> None:
    assert stage20_physical_ab_probe.command_for_probe_mode("forward", 0.12) == (0.12, 0.0)
    assert stage20_physical_ab_probe.command_for_probe_mode("backward", 0.12) == (-0.12, 0.0)
    assert stage20_physical_ab_probe.command_for_probe_mode("turn_left", 0.12) == (0.0, 0.12)
    assert stage20_physical_ab_probe.command_for_probe_mode("turn_right", 0.12) == (0.0, -0.12)


def test_stage20_turn_left_uses_b_axis_limit() -> None:
    planned = stage20_physical_ab_probe.planned_probe_commands(
        seq=1,
        mode="turn_left",
        cmd=0.32,
        pulse_ms=850,
        max_abs_a=0.25,
        max_abs_b=0.35,
    )
    assert planned["command_axis"] == "B"
    assert planned["axis_limit_used"] == 0.35
    assert planned["command_within_axis_limit"] is True
    assert planned["stage20_command_text"] == "STAGE20_CMD seq=1 a=0.000 b=0.320 ms=850"


def test_stage20_turn_right_uses_b_axis_limit() -> None:
    planned = stage20_physical_ab_probe.planned_probe_commands(
        seq=1,
        mode="turn_right",
        cmd=0.32,
        pulse_ms=850,
        max_abs_a=0.25,
        max_abs_b=0.35,
    )
    assert planned["command_axis"] == "B"
    assert planned["axis_limit_used"] == 0.35
    assert planned["command_within_axis_limit"] is True
    assert planned["stage20_command_text"] == "STAGE20_CMD seq=1 a=0.000 b=-0.320 ms=850"


def test_stage20_forward_uses_a_axis_limit() -> None:
    try:
        stage20_physical_ab_probe.planned_probe_commands(
            seq=1,
            mode="forward",
            cmd=0.32,
            pulse_ms=850,
            max_abs_a=0.25,
            max_abs_b=0.35,
        )
    except ValueError as exc:
        assert "A-axis limit 0.25" in str(exc)
    else:
        raise AssertionError("forward command above max_abs_a should fail")


def test_stage20_planned_probe_command() -> None:
    planned = stage20_physical_ab_probe.planned_probe_commands(
        seq=4,
        mode="turn_left",
        cmd=0.16,
        pulse_ms=300,
    )
    assert planned["arm_command_text"] == "STAGE20_ARM seq=4"
    assert planned["stage20_command_text"] == "STAGE20_CMD seq=4 a=0.000 b=0.160 ms=300"
    assert planned["stop_command_text"] == "STAGE20_STOP seq=4"
    assert planned["command_axis"] == "B"


def test_stage20_row_from_trial_extracts_physical_ab_fields() -> None:
    planned = stage20_physical_ab_probe.planned_probe_commands(seq=1, mode="forward", cmd=0.12, pulse_ms=500)
    rows = stage20_physical_ab_probe.station_path_package_tracker.parse_usbdbg_rows(
        "STAGE20 stage20_physical_ab_guarded_crawl=true event=ARM stage20_cmd_state=ARMED latched_stop=false\n"
        "STAGE20 stage20_physical_ab_guarded_crawl=true event=ACK stage20_cmd_state=ACTIVE "
        "requested_a_cmd=0.120 requested_b_cmd=0.000 physical_a_cmd=0.120 physical_b_cmd=0.000 "
        "manual_equivalent_forward_cmd=0.120 manual_equivalent_turn_cmd=0.000 "
        "motor_write_called=true motor_backend=servo_pwm motor_enable_state=attached "
        "pwm_or_dynamixel_write_status=OK wheel_to_physical_mapping=physical_ab_manual_equivalent "
        "active_target_source=station_physical_ab_guarded physical_output_active=true\n"
        "STAGE20 stage20_physical_ab_guarded_crawl=true event=STOP stage20_cmd_state=STOPPED "
        "final_left_cmd=0.000 final_right_cmd=0.000 physical_output_active=false\n"
    )
    row = stage20_physical_ab_probe.row_from_trial(
        trial_index=1,
        planned=planned,
        rows=rows,
        left_report="yes",
        right_report="yes",
        body_report="forward",
        upper_limit_trial=False,
    )
    assert row["physical_a_cmd"] == "0.120"
    assert row["manual_equivalent_forward_cmd"] == "0.120"
    assert row["wheel_to_physical_mapping"] == "physical_ab_manual_equivalent"
    assert row["firmware_active_target_source"] == "station_physical_ab_guarded"
    assert row["visible_motion_confirmed"] is True
    assert row["motion_class"] == "visible"
    assert row["ready_for_full_path_following"] is False


def test_stage20_row_from_turn_trial_extracts_physical_b_field() -> None:
    planned = stage20_physical_ab_probe.planned_probe_commands(
        seq=1,
        mode="turn_left",
        cmd=0.32,
        pulse_ms=850,
        max_abs_a=0.25,
        max_abs_b=0.35,
    )
    rows = stage20_physical_ab_probe.station_path_package_tracker.parse_usbdbg_rows(
        "STAGE20 stage20_physical_ab_guarded_crawl=true event=ARM stage20_cmd_state=ARMED latched_stop=false\n"
        "STAGE20 stage20_physical_ab_guarded_crawl=true event=ACK stage20_cmd_state=ACTIVE "
        "requested_a_cmd=0.000 requested_b_cmd=0.320 physical_a_cmd=0.000 physical_b_cmd=0.320 "
        "manual_equivalent_forward_cmd=0.000 manual_equivalent_turn_cmd=0.320 "
        "motor_write_called=true wheel_to_physical_mapping=physical_ab_manual_equivalent "
        "active_target_source=station_physical_ab_guarded physical_output_active=true\n"
        "STAGE20 stage20_physical_ab_guarded_crawl=true event=STOP stage20_cmd_state=STOPPED "
        "final_left_cmd=0.000 final_right_cmd=0.000 physical_output_active=false\n"
    )
    row = stage20_physical_ab_probe.row_from_trial(
        trial_index=1,
        planned=planned,
        rows=rows,
        left_report="unknown",
        right_report="unknown",
        body_report="left",
        turn_angle_report="small_under_15deg",
        upper_limit_trial=False,
    )
    assert row["command_axis"] == "B"
    assert row["physical_b_cmd"] == "0.320"
    assert row["manual_equivalent_turn_cmd"] == "0.320"
    assert row["visible_motion_confirmed"] is True
    assert row["ready_for_full_path_following"] is False


def test_stage20_row_marks_latched_stop_trial_invalid() -> None:
    planned = stage20_physical_ab_probe.planned_probe_commands(seq=1, mode="forward", cmd=0.25, pulse_ms=800)
    rows = stage20_physical_ab_probe.station_path_package_tracker.parse_usbdbg_rows(
        "STAGE20 stage20_physical_ab_guarded_crawl=true event=REJECT "
        "stage16_reject_reason=LATCHED_STOP rc_ok=true neutral_ok=true\n"
    )
    row = stage20_physical_ab_probe.row_from_trial(
        trial_index=1,
        planned=planned,
        rows=rows,
        left_report="no",
        right_report="no",
        body_report="none",
        upper_limit_trial=True,
    )
    assert row["valid_trial"] is False
    assert row["invalid_reason"] == "LATCHED_STOP"
    assert row["motor_power_or_mapping_suspect"] is False


def test_stage20_row_marks_rc_invalid_trial_invalid() -> None:
    planned = stage20_physical_ab_probe.planned_probe_commands(seq=1, mode="forward", cmd=0.25, pulse_ms=800)
    rows = stage20_physical_ab_probe.station_path_package_tracker.parse_usbdbg_rows(
        "STAGE20 stage20_physical_ab_guarded_crawl=true event=REJECT "
        "stage16_reject_reason=RC_INVALID rc_ok=false neutral_ok=false\n"
    )
    row = stage20_physical_ab_probe.row_from_trial(
        trial_index=1,
        planned=planned,
        rows=rows,
        left_report="no",
        right_report="no",
        body_report="none",
        upper_limit_trial=True,
    )
    assert row["valid_trial"] is False
    assert row["invalid_reason"] == "RC_INVALID"
    assert row["motor_power_or_mapping_suspect"] is False
    assert row["motion_class"] == "none"


def test_stage20_stable_precondition_requires_three_good_heartbeats() -> None:
    rows = stage20_physical_ab_probe.station_path_package_tracker.parse_usbdbg_rows(
        "STAGE20 event=HEARTBEAT rc_ok=true neutral_ok=true physical_output_active=false\n"
        "STAGE20 event=HEARTBEAT rc_ok=true neutral_ok=true physical_output_active=false\n"
    )
    assert stage20_physical_ab_probe.stable_precondition_met(rows, consecutive=3) is False
    rows = stage20_physical_ab_probe.station_path_package_tracker.parse_usbdbg_rows(
        "STAGE20 event=HEARTBEAT rc_ok=true neutral_ok=true physical_output_active=false\n"
        "STAGE20 event=HEARTBEAT rc_ok=true neutral_ok=true physical_output_active=false\n"
        "STAGE20 event=HEARTBEAT rc_ok=true neutral_ok=true physical_output_active=false\n"
    )
    assert stage20_physical_ab_probe.stable_precondition_met(rows, consecutive=3) is True


def test_stage20_stop_search_on_turn_twitch_reports() -> None:
    assert stage20_physical_ab_probe.stop_search_after_user_report({
        "body_motion_user_report": "none",
        "turn_angle_report": "small_under_15deg",
        "visible_motion_confirmed": False,
    }) is True


def test_stage20_checker_passes_safe_probe(tmp_path: Path) -> None:
    csv_path = tmp_path / "stage20_physical_ab_probe.csv"
    _write_csv(csv_path, [_safe_row()])
    result = check_stage20_physical_ab_probe.evaluate_path(csv_path)
    assert result["verdict"] == "PASS"
    assert result["ready_for_full_path_following"] is False


def test_stage20_checker_fails_final_nonzero(tmp_path: Path) -> None:
    csv_path = tmp_path / "stage20_physical_ab_probe.csv"
    _write_csv(csv_path, [_safe_row(final_left_cmd=0.01)])
    result = check_stage20_physical_ab_probe.evaluate_path(csv_path)
    assert result["verdict"] == "FAIL"
    assert "FINAL_COMMANDS_NONZERO" in result["reasons"]


def test_stage20_checker_fails_compile_time_target(tmp_path: Path) -> None:
    csv_path = tmp_path / "stage20_physical_ab_probe.csv"
    _write_csv(csv_path, [_safe_row(firmware_active_target_source="compile_time")])
    result = check_stage20_physical_ab_probe.evaluate_path(csv_path)
    assert result["verdict"] == "FAIL"
    assert "COMPILE_TIME_TARGET_ACTIVE_DURING_STAGE20" in result["reasons"]


def test_stage20_checker_fails_full_path_following_enabled(tmp_path: Path) -> None:
    csv_path = tmp_path / "stage20_physical_ab_probe.csv"
    _write_csv(csv_path, [_safe_row(physical_path_following_enable=True)])
    result = check_stage20_physical_ab_probe.evaluate_path(csv_path)
    assert result["verdict"] == "FAIL"
    assert "NORMAL_PATH_FOLLOWING_ENABLED" in result["reasons"]
