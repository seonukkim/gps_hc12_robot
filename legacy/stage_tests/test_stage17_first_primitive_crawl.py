from __future__ import annotations

import csv
from pathlib import Path

from tools import check_stage17_first_primitive_crawl_log
from tools import field_ab_to_serpentine
from tools import station_first_primitive_crawl


REPO = Path(__file__).resolve().parents[1]
SCRIPT = REPO / "legacy" / "stage_scripts" / "run_stage17_first_primitive_crawl.sh"


def _georef_package() -> dict[str, object]:
    georef = {
        "georeference_available": True,
        "raw_A_lat": 35.0,
        "raw_A_lon": 129.0,
        "raw_B_lat": 35.00001,
        "raw_B_lon": 129.00010,
        "origin_lat": 35.0,
        "origin_lon": 129.0,
        "local_frame_type": "equirectangular_enu",
        "x_axis_source": "normalized_rectangle",
        "x_axis_bearing_deg": 90.0,
        "meters_per_deg_lat": field_ab_to_serpentine.METERS_PER_DEG_LAT,
        "meters_per_deg_lon": field_ab_to_serpentine._meters_per_deg_lon(35.000005),
        "meters_per_lat": field_ab_to_serpentine.METERS_PER_DEG_LAT,
        "meters_per_lon": field_ab_to_serpentine._meters_per_deg_lon(35.000005),
        "motor_command_generated": False,
    }
    return field_ab_to_serpentine.build_path_package(
        raw_a=(0.0, 0.0),
        raw_b=(9.1, 1.1),
        current_pose=(0.0, 0.0, 0.0),
        step_spacing_m=0.25,
        tool_side="left",
        tool_lateral_offset_m=0.24,
        tool_width_m=0.30,
        tool_length_m=0.18,
        robot_width_m=0.18,
        robot_length_m=0.18,
        georeference=georef,
    )


def _write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=station_first_primitive_crawl.CRAWL_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in station_first_primitive_crawl.CRAWL_FIELDS})


def _safe_row(**overrides: object) -> dict[str, object]:
    row: dict[str, object] = {
        "row_index": 0,
        "dry_run": False,
        "path_package_loaded": True,
        "station_package_target_source": "path_package",
        "pose_mode": "manual_local",
        "local_pose_available": True,
        "firmware_mode": "stage17",
        "firmware_ready": True,
        "firmware_active_target_source": "station_first_primitive_guarded",
        "physical_path_following_enable": False,
        "allow_motor_output": False,
        "active_primitive_index": 0,
        "active_tool_segment_id": "T000",
        "target_distance_m": 1.0,
        "target_bearing_deg": 0.0,
        "cross_track_error_m": 0.0,
        "virtual_left_cmd": 0.045,
        "virtual_right_cmd": 0.045,
        "stage17_left_cmd": 0.045,
        "stage17_right_cmd": 0.045,
        "stage17_ms": 300,
        "stage17_command_text": "STAGE17_CMD seq=1 left=0.045 right=0.045 ms=300",
        "force_straight_test": False,
        "diagnostic_only": False,
        "arm_sent": True,
        "arm_ack_seen": True,
        "ack_seen": True,
        "stop_seen": True,
        "reject_seen": False,
        "reject_reason": "NONE",
        "pulse_count": 1,
        "estimated_distance_m": 0.0135,
        "visible_motion_user_report": "twitch",
        "final_left_cmd": 0.0,
        "final_right_cmd": 0.0,
        "final_left_cmd_zero": True,
        "final_right_cmd_zero": True,
        "physical_output_active_seen": True,
        "physical_output_active_after_stop": False,
        "ready_for_next_stage": True,
        "ready_for_full_path_following": False,
    }
    row.update(overrides)
    return row


def test_stage17_script_uses_guarded_compile_flags() -> None:
    text = SCRIPT.read_text(encoding="utf-8")
    assert "STAGE17_FIRST_PRIMITIVE_CRAWL=1" in text
    assert "STAGE17_MAX_CMD=0.06" in text
    assert "STAGE17_MAX_MS=500" in text
    assert "PHYSICAL_PATH_FOLLOWING_ENABLE=0" in text
    assert "PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0" in text
    assert "STAGE16_USB_GUARDED_CRAWL=0" in text


def test_stage17_detects_stage17_firmware_ready() -> None:
    rows = station_first_primitive_crawl.station_path_package_tracker.parse_usbdbg_rows(
        "STAGE17 stage17_first_primitive_crawl=true event=HEARTBEAT "
        "stage17_firmware_ready=true physical_output_active=false\n"
    )
    detected = station_first_primitive_crawl.detect_firmware_mode(rows)
    assert detected["firmware_mode"] == "stage17"
    assert detected["command_prefix"] == "STAGE17"
    assert detected["firmware_ready"] is True


def test_stage17_detects_stage16_fallback_ready() -> None:
    rows = station_first_primitive_crawl.station_path_package_tracker.parse_usbdbg_rows(
        "STAGE16 stage16_guarded_crawl=true event=HEARTBEAT "
        "stage16_firmware_ready=true physical_output_active=false\n"
    )
    detected = station_first_primitive_crawl.detect_firmware_mode(rows)
    assert detected["firmware_mode"] == "stage16"
    assert detected["command_prefix"] == "STAGE16"


def test_stage17_builds_bounded_path_command() -> None:
    virtual = {"virtual_left_cmd": 0.2, "virtual_right_cmd": -0.2}
    command = station_first_primitive_crawl.command_from_virtual(
        virtual,
        max_cmd=0.045,
        pulse_ms=300,
        seq=2,
        command_prefix="STAGE17",
    )
    assert command["stage17_left_cmd"] == 0.045
    assert command["stage17_right_cmd"] == -0.045
    assert command["stage17_command_text"] == "STAGE17_CMD seq=2 left=0.045 right=-0.045 ms=300"


def test_stage17_force_straight_test_is_diagnostic() -> None:
    package = _georef_package()
    log = (
        "STAGE17 stage17_first_primitive_crawl=true event=HEARTBEAT "
        "stage17_firmware_ready=true physical_output_active=false active_target_source=NONE\n"
    )
    row = station_first_primitive_crawl.build_crawl_row(
        package,
        log,
        dry_run=True,
        pose_mode="manual_local",
        current_x=0.0,
        current_y=0.0,
        current_heading_deg=0.0,
        max_cmd=0.045,
        pulse_ms=300,
        max_total_distance_m=0.20,
        force_straight_test=True,
    )
    assert row["stage17_left_cmd"] == 0.045
    assert row["stage17_right_cmd"] == 0.045
    assert row["diagnostic_only"] is True
    assert row["ready_for_full_path_following"] is False


def test_stage17_dry_run_builds_no_pulse() -> None:
    package = _georef_package()
    log = (
        "STAGE17 stage17_first_primitive_crawl=true event=HEARTBEAT "
        "stage17_firmware_ready=true physical_output_active=false active_target_source=NONE\n"
    )
    row = station_first_primitive_crawl.build_crawl_row(
        package,
        log,
        dry_run=True,
        pose_mode="manual_local",
        current_x=0.0,
        current_y=0.0,
        current_heading_deg=0.0,
        max_cmd=0.045,
        pulse_ms=300,
        max_total_distance_m=0.20,
    )
    assert row["arm_sent"] is False
    assert row["ack_seen"] is False
    assert row["ready_for_full_path_following"] is False


def test_stage17_checker_passes_safe_log(tmp_path: Path) -> None:
    csv_path = tmp_path / "stage17_first_primitive_crawl.csv"
    _write_csv(csv_path, [_safe_row()])
    result = check_stage17_first_primitive_crawl_log.evaluate_path(csv_path)
    assert result["verdict"] == "PASS"
    assert result["ready_for_full_path_following"] is False


def test_stage17_checker_fails_no_stop(tmp_path: Path) -> None:
    csv_path = tmp_path / "stage17_first_primitive_crawl.csv"
    _write_csv(csv_path, [_safe_row(stop_seen=False)])
    result = check_stage17_first_primitive_crawl_log.evaluate_path(csv_path)
    assert result["verdict"] == "FAIL"
    assert "STOP_MISSING_AFTER_PULSE" in result["reasons"]


def test_stage17_checker_fails_final_nonzero(tmp_path: Path) -> None:
    csv_path = tmp_path / "stage17_first_primitive_crawl.csv"
    _write_csv(csv_path, [_safe_row(final_left_cmd=0.01, final_left_cmd_zero=False)])
    result = check_stage17_first_primitive_crawl_log.evaluate_path(csv_path)
    assert result["verdict"] == "FAIL"
    assert "FINAL_COMMANDS_NONZERO" in result["reasons"]


def test_stage17_checker_fails_continuous_output(tmp_path: Path) -> None:
    csv_path = tmp_path / "stage17_first_primitive_crawl.csv"
    _write_csv(csv_path, [_safe_row(physical_output_active_after_stop=True)])
    result = check_stage17_first_primitive_crawl_log.evaluate_path(csv_path)
    assert result["verdict"] == "FAIL"
    assert "OUTPUT_ACTIVE_AFTER_STOP" in result["reasons"]


def test_stage17_checker_fails_compile_time_target(tmp_path: Path) -> None:
    csv_path = tmp_path / "stage17_first_primitive_crawl.csv"
    _write_csv(csv_path, [_safe_row(firmware_active_target_source="compile_time")])
    result = check_stage17_first_primitive_crawl_log.evaluate_path(csv_path)
    assert result["verdict"] == "FAIL"
    assert "COMPILE_TIME_TARGET_ACTIVE_DURING_STAGE17" in result["reasons"]


def test_stage17_summary_never_full_path_ready() -> None:
    summary = station_first_primitive_crawl.build_summary([_safe_row()])
    assert summary["ready_for_next_stage"] is True
    assert summary["ready_for_full_path_following"] is False
