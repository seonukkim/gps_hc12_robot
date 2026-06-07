from __future__ import annotations

import csv
from pathlib import Path

from tools import check_stage16_deadband_calibration
from tools import stage16_motor_deadband_calibration


REPO = Path(__file__).resolve().parents[1]
SCRIPT = REPO / "scripts" / "run_stage16_motor_deadband_calibration.sh"


def _write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=stage16_motor_deadband_calibration.CALIBRATION_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in stage16_motor_deadband_calibration.CALIBRATION_FIELDS})


def _safe_row(**overrides: object) -> dict[str, object]:
    row: dict[str, object] = {
        "trial_index": 1,
        "mode": "forward",
        "cmd": 0.025,
        "pulse_ms": 100,
        "seq": 1,
        "arm_command_text": "STAGE16_ARM seq=1",
        "stage16_command_text": "STAGE16_CMD seq=1 left=0.025 right=0.025 ms=100",
        "stop_command_text": "STAGE16_STOP seq=1",
        "arm_sent": True,
        "arm_ack_seen": True,
        "ack_seen": True,
        "stop_seen": True,
        "reject_seen": False,
        "reject_reason": "NONE",
        "physical_output_active_seen": True,
        "physical_output_active_after_stop": False,
        "final_left_cmd": 0.0,
        "final_right_cmd": 0.0,
        "firmware_active_target_source": "station_path_package_guarded",
        "physical_path_following_enable": False,
        "allow_motor_output": False,
        "visible_motion_response": "forward",
        "visible_motion_confirmed": True,
        "no_visible_motion_at_limit": False,
        "ready_for_full_path_following": False,
    }
    row.update(overrides)
    return row


def test_stage16_deadband_script_uses_guarded_flags() -> None:
    text = SCRIPT.read_text(encoding="utf-8")
    assert "STAGE16_USB_GUARDED_CRAWL=1" in text
    assert "STAGE16_MAX_CMD=0.04" in text
    assert "STAGE16_MAX_MS=150" in text
    assert "PHYSICAL_PATH_FOLLOWING_ENABLE=0" in text
    assert "PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0" in text
    assert "GROUND_CRAWL_TEST_MODE=0" in text
    assert "AUTO_MOTION_ARMED=0" in text


def test_forward_command_formatting() -> None:
    assert stage16_motor_deadband_calibration.command_for_mode("forward", 0.02) == (0.02, 0.02)
    trial = stage16_motor_deadband_calibration.planned_trial_commands(seq=1, mode="forward", cmd=0.02, pulse_ms=100)
    assert trial["stage16_command_text"] == "STAGE16_CMD seq=1 left=0.020 right=0.020 ms=100"


def test_backward_command_formatting() -> None:
    assert stage16_motor_deadband_calibration.command_for_mode("backward", 0.02) == (-0.02, -0.02)
    trial = stage16_motor_deadband_calibration.planned_trial_commands(seq=2, mode="backward", cmd=0.02, pulse_ms=100)
    assert trial["stage16_command_text"] == "STAGE16_CMD seq=2 left=-0.020 right=-0.020 ms=100"


def test_rotate_left_command_formatting() -> None:
    assert stage16_motor_deadband_calibration.command_for_mode("rotate_left", 0.02) == (-0.02, 0.02)
    trial = stage16_motor_deadband_calibration.planned_trial_commands(seq=3, mode="rotate_left", cmd=0.02, pulse_ms=100)
    assert trial["stage16_command_text"] == "STAGE16_CMD seq=3 left=-0.020 right=0.020 ms=100"


def test_rotate_right_command_formatting() -> None:
    assert stage16_motor_deadband_calibration.command_for_mode("rotate_right", 0.02) == (0.02, -0.02)
    trial = stage16_motor_deadband_calibration.planned_trial_commands(seq=4, mode="rotate_right", cmd=0.02, pulse_ms=100)
    assert trial["stage16_command_text"] == "STAGE16_CMD seq=4 left=0.020 right=-0.020 ms=100"


def test_arm_before_command_and_stop_after_command() -> None:
    trial = stage16_motor_deadband_calibration.planned_trial_commands(seq=7, mode="forward", cmd=0.03, pulse_ms=120)
    assert trial["arm_command_text"] == "STAGE16_ARM seq=7"
    assert str(trial["stage16_command_text"]).startswith("STAGE16_CMD seq=7")
    assert trial["stop_command_text"] == "STAGE16_STOP seq=7"


def test_summary_never_reports_full_path_ready() -> None:
    summary = stage16_motor_deadband_calibration.build_summary([_safe_row()])
    assert summary["ready_for_full_path_following"] is False
    assert summary["first_visible_motion_cmd"] == 0.025


def test_deadband_checker_passes_safe_calibration_log(tmp_path: Path) -> None:
    csv_path = tmp_path / "deadband_calibration.csv"
    _write_csv(csv_path, [_safe_row()])
    result = check_stage16_deadband_calibration.evaluate_path(csv_path)
    assert result["verdict"] == "PASS"
    assert result["visible_motion_count"] == 1
    assert result["ready_for_full_path_following"] is False


def test_deadband_checker_reports_no_visible_motion_at_limit(tmp_path: Path) -> None:
    csv_path = tmp_path / "deadband_calibration.csv"
    _write_csv(
        csv_path,
        [
            _safe_row(
                cmd=0.04,
                pulse_ms=150,
                visible_motion_response="none",
                visible_motion_confirmed=False,
                no_visible_motion_at_limit=True,
            )
        ],
    )
    result = check_stage16_deadband_calibration.evaluate_path(csv_path)
    assert result["verdict"] == "PASS"
    assert result["visible_motion_count"] == 0
    assert result["no_visible_motion_at_limit"] is True


def test_deadband_checker_fails_continuous_output(tmp_path: Path) -> None:
    csv_path = tmp_path / "deadband_calibration.csv"
    _write_csv(csv_path, [_safe_row(physical_output_active_after_stop=True)])
    result = check_stage16_deadband_calibration.evaluate_path(csv_path)
    assert result["verdict"] == "FAIL"
    assert "OUTPUT_ACTIVE_AFTER_STOP" in result["reasons"]


def test_deadband_checker_fails_compile_time_target(tmp_path: Path) -> None:
    csv_path = tmp_path / "deadband_calibration.csv"
    _write_csv(csv_path, [_safe_row(firmware_active_target_source="compile_time")])
    result = check_stage16_deadband_calibration.evaluate_path(csv_path)
    assert result["verdict"] == "FAIL"
    assert "COMPILE_TIME_TARGET_ACTIVE_DURING_STAGE16" in result["reasons"]
