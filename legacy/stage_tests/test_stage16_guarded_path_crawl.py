from __future__ import annotations

from pathlib import Path

from tools import check_stage16_guarded_crawl_log
from tools import field_ab_to_serpentine
from tools import station_guarded_path_crawl


REPO = Path(__file__).resolve().parents[1]
SCRIPT = REPO / "legacy" / "stage_scripts" / "run_stage16_guarded_path_crawl.sh"


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


def test_stage16_script_uses_guarded_compile_flags() -> None:
    text = SCRIPT.read_text(encoding="utf-8")
    assert "STAGE16_USB_GUARDED_CRAWL=1" in text
    assert "STAGE16_MAX_CMD=0.04" in text
    assert "STAGE16_MAX_MS=150" in text
    assert "PHYSICAL_PATH_FOLLOWING_ENABLE=0" in text
    assert "PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0" in text
    assert "PATH_FOLLOWING_DRYRUN=0" in text


def test_stage16_command_parser_accepts_valid_command() -> None:
    parsed = station_guarded_path_crawl.parse_stage16_command_text(
        "STAGE16_CMD seq=1 left=0.025 right=0.030 ms=120",
        max_cmd=0.04,
        max_ms=150,
    )
    assert parsed["valid"] is True
    assert parsed["seq"] == 1


def test_stage16_command_parser_rejects_over_limit_command() -> None:
    parsed = station_guarded_path_crawl.parse_stage16_command_text(
        "STAGE16_CMD seq=1 left=0.050 right=0.030 ms=120",
        max_cmd=0.04,
        max_ms=150,
    )
    assert parsed["valid"] is False
    assert parsed["reason"] == "COMMAND_EXCEEDS_MAX_CMD"


def test_stage16_command_parser_rejects_over_duration_command() -> None:
    parsed = station_guarded_path_crawl.parse_stage16_command_text(
        "STAGE16_CMD seq=1 left=0.020 right=0.030 ms=200",
        max_cmd=0.04,
        max_ms=150,
    )
    assert parsed["valid"] is False
    assert parsed["reason"] == "COMMAND_EXCEEDS_MAX_MS"


def test_station_tool_dry_run_sends_no_command() -> None:
    package = _georef_package()
    log_text = (
        "STAGE16 stage16_guarded_crawl=true event=HEARTBEAT stage16_firmware_ready=true "
        "stage16_cmd_state=IDLE physical_output_active=false active_target_source=NONE "
        "rc_ok=true current_lat=35.0 current_lon=129.0\n"
    )
    rows = station_guarded_path_crawl.build_crawl_rows(
        package,
        log_text,
        dry_run=True,
        max_cmd=0.03,
        pulse_ms=100,
        max_total_distance_m=0.20,
    )
    assert rows[0]["stage16_firmware_ready"] is True
    assert rows[0]["arm_sent"] is False
    assert rows[0]["command_sent"] is False
    assert rows[0]["motor_command_generated"] is False
    assert rows[0]["ready_for_full_path_following"] is False


def test_station_tool_non_dry_run_waits_for_arm_before_command() -> None:
    package = _georef_package()
    log_text = (
        "STAGE16 stage16_guarded_crawl=true event=HEARTBEAT stage16_firmware_ready=true "
        "stage16_cmd_state=IDLE physical_output_active=false active_target_source=NONE "
        "rc_ok=true current_lat=35.0 current_lon=129.0\n"
    )
    rows = station_guarded_path_crawl.build_crawl_rows(
        package,
        log_text,
        dry_run=False,
        max_cmd=0.03,
        pulse_ms=100,
        max_total_distance_m=0.20,
    )
    assert rows[0]["arm_sent"] is False
    assert rows[0]["command_sent"] is False
    assert rows[0]["motor_command_generated"] is False


def test_station_tool_non_dry_run_formats_bounded_command_after_arm() -> None:
    package = _georef_package()
    log_text = "\n".join(
        [
            "STAGE16 stage16_guarded_crawl=true event=HEARTBEAT stage16_firmware_ready=true "
            "stage16_cmd_state=IDLE physical_output_active=false active_target_source=NONE "
            "rc_ok=true current_lat=35.0 current_lon=129.0 latched_stop=true",
            "STAGE16 stage16_guarded_crawl=true event=ARM stage16_firmware_ready=true "
            "stage16_cmd_state=ARMED physical_output_active=false active_target_source=NONE "
            "rc_ok=true neutral_ok=true current_lat=35.0 current_lon=129.0 latched_stop=false",
        ]
    )
    rows = station_guarded_path_crawl.build_crawl_rows(
        package,
        log_text,
        dry_run=False,
        max_cmd=0.03,
        pulse_ms=100,
        max_total_distance_m=0.20,
    )
    assert rows[0]["arm_sent"] is True
    assert rows[0]["arm_ack_seen"] is True
    assert rows[0]["command_sent"] is True
    assert rows[0]["motor_command_generated"] is True
    assert str(rows[0]["stage16_command_text"]).startswith("STAGE16_CMD seq=1")
    assert abs(float(rows[0]["stage16_left_cmd"])) <= 0.03
    assert abs(float(rows[0]["stage16_right_cmd"])) <= 0.03


def test_stage16_checker_passes_safe_sample_log() -> None:
    log = "\n".join(
        [
            "STAGE16 stage16_guarded_crawl=true event=ARM stage16_cmd_state=ARMED "
            "physical_output_active=false final_left_cmd=0.000 final_right_cmd=0.000 "
            "active_target_source=NONE physical_path_following_enable=false allow_motor_output=false",
            "STAGE16 stage16_guarded_crawl=true event=ACK stage16_cmd_state=ACTIVE "
            "stage16_left_cmd=0.025 stage16_right_cmd=0.030 stage16_ms=120 "
            "physical_output_active=true final_left_cmd=0.025 final_right_cmd=0.030 "
            "active_target_source=station_path_package_guarded estimated_distance_m=0.01 "
            "physical_path_following_enable=false allow_motor_output=false",
            "STAGE16 stage16_guarded_crawl=true event=STOP stage16_cmd_state=STOPPED "
            "stage16_left_cmd=0.000 stage16_right_cmd=0.000 stage16_ms=0 "
            "physical_output_active=false final_left_cmd=0.000 final_right_cmd=0.000 "
            "active_target_source=NONE estimated_distance_m=0.01",
        ]
    )
    result = check_stage16_guarded_crawl_log.evaluate_log_text(log)
    assert result["verdict"] == "PASS"
    assert result["arm_count"] == 1
    assert result["ready_for_full_path_following"] is False


def test_stage16_checker_fails_cmd_without_arm() -> None:
    log = "\n".join(
        [
            "STAGE16 stage16_guarded_crawl=true event=ACK stage16_cmd_state=ACTIVE "
            "stage16_left_cmd=0.025 stage16_right_cmd=0.030 stage16_ms=120 "
            "physical_output_active=true final_left_cmd=0.025 final_right_cmd=0.030 "
            "active_target_source=station_path_package_guarded",
            "STAGE16 stage16_guarded_crawl=true event=STOP stage16_cmd_state=STOPPED "
            "physical_output_active=false final_left_cmd=0.000 final_right_cmd=0.000",
        ]
    )
    result = check_stage16_guarded_crawl_log.evaluate_log_text(log)
    assert result["verdict"] == "FAIL"
    assert "ACTIVE_WITHOUT_ARM" in result["reasons"]


def test_stage16_checker_fails_continuous_output() -> None:
    log = (
        "STAGE16 stage16_guarded_crawl=true event=ACK stage16_cmd_state=ACTIVE "
        "stage16_left_cmd=0.025 stage16_right_cmd=0.030 stage16_ms=120 "
        "physical_output_active=true final_left_cmd=0.025 final_right_cmd=0.030 "
        "active_target_source=station_path_package_guarded\n"
    )
    result = check_stage16_guarded_crawl_log.evaluate_log_text(log)
    assert result["verdict"] == "FAIL"
    assert "SERIAL_DISCONNECT_OR_LOG_END_WHILE_ACTIVE" in result["reasons"]


def test_stage16_checker_fails_compile_time_target_during_active_pulse() -> None:
    log = "\n".join(
        [
            "STAGE16 stage16_guarded_crawl=true event=ACK stage16_cmd_state=ACTIVE "
            "stage16_left_cmd=0.025 stage16_right_cmd=0.030 stage16_ms=120 "
            "physical_output_active=true final_left_cmd=0.025 final_right_cmd=0.030 "
            "active_target_source=compile_time",
            "STAGE16 stage16_guarded_crawl=true event=STOP stage16_cmd_state=STOPPED "
            "physical_output_active=false final_left_cmd=0.000 final_right_cmd=0.000",
        ]
    )
    result = check_stage16_guarded_crawl_log.evaluate_log_text(log)
    assert result["verdict"] == "FAIL"
    assert "COMPILE_TIME_TARGET_ACTIVE_DURING_STAGE16" in result["reasons"]


def test_stage16_heartbeat_line_parser_sets_firmware_ready() -> None:
    rows = station_guarded_path_crawl.station_path_package_tracker.parse_usbdbg_rows(
        "STAGE16 stage16_guarded_crawl=true event=HEARTBEAT "
        "stage16_firmware_ready=true stage16_cmd_state=IDLE physical_output_active=false\n"
    )
    ready, heartbeat, *_ = station_guarded_path_crawl.parse_stage16_ready(rows)
    assert ready is True
    assert heartbeat is True


def test_stage16_stop_line_sets_ready_when_stopped_safely() -> None:
    rows = station_guarded_path_crawl.station_path_package_tracker.parse_usbdbg_rows(
        "STAGE16 stage16_guarded_crawl=true event=STOP stage16_cmd_state=STOPPED "
        "physical_output_active=false final_left_cmd=0.000 final_right_cmd=0.000\n"
    )
    ready, heartbeat, *_ = station_guarded_path_crawl.parse_stage16_ready(rows)
    assert ready is True
    assert heartbeat is False


def test_stage16_arm_event_parser_accepts_safe_arm() -> None:
    rows = station_guarded_path_crawl.station_path_package_tracker.parse_usbdbg_rows(
        "STAGE16 stage16_guarded_crawl=true event=ARM stage16_firmware_ready=true "
        "stage16_cmd_state=ARMED stage16_reject_reason=NONE latched_stop=false "
        "rc_ok=true neutral_ok=true physical_output_active=false "
        "final_left_cmd=0.000 final_right_cmd=0.000\n"
    )
    flags = station_guarded_path_crawl.parse_stage16_event_flags(rows)
    assert flags["arm_ack_seen"] is True
    assert flags["arm_reject_seen"] is False
    assert flags["latched_stop_after"] is False


def test_stage16_arm_rejected_when_rc_not_ok() -> None:
    rows = station_guarded_path_crawl.station_path_package_tracker.parse_usbdbg_rows(
        "STAGE16 stage16_guarded_crawl=true event=REJECT stage16_firmware_ready=true "
        "stage16_cmd_state=REJECTED stage16_reject_reason=ARM_RC_NOT_OK "
        "latched_stop=true rc_ok=false neutral_ok=true\n"
    )
    flags = station_guarded_path_crawl.parse_stage16_event_flags(rows)
    assert flags["arm_reject_seen"] is True
    assert flags["command_rejected_reason"] == "ARM_RC_NOT_OK"


def test_stage16_arm_rejected_when_neutral_not_ok() -> None:
    rows = station_guarded_path_crawl.station_path_package_tracker.parse_usbdbg_rows(
        "STAGE16 stage16_guarded_crawl=true event=REJECT stage16_firmware_ready=true "
        "stage16_cmd_state=REJECTED stage16_reject_reason=ARM_NEUTRAL_NOT_OK "
        "latched_stop=true rc_ok=true neutral_ok=false\n"
    )
    flags = station_guarded_path_crawl.parse_stage16_event_flags(rows)
    assert flags["arm_reject_seen"] is True
    assert flags["command_rejected_reason"] == "ARM_NEUTRAL_NOT_OK"


def test_stage16_stop_relatches() -> None:
    rows = station_guarded_path_crawl.station_path_package_tracker.parse_usbdbg_rows(
        "STAGE16 stage16_guarded_crawl=true event=ARM stage16_cmd_state=ARMED latched_stop=false\n"
        "STAGE16 stage16_guarded_crawl=true event=STOP stage16_cmd_state=STOPPED "
        "latched_stop=true final_left_cmd=0.000 final_right_cmd=0.000\n"
    )
    flags = station_guarded_path_crawl.parse_stage16_event_flags(rows)
    assert flags["latched_stop_before"] is False
    assert flags["latched_stop_after"] is True


def test_stage16_command_rejected_when_not_armed() -> None:
    rows = station_guarded_path_crawl.station_path_package_tracker.parse_usbdbg_rows(
        "STAGE16 stage16_guarded_crawl=true event=REJECT stage16_firmware_ready=true "
        "stage16_cmd_state=REJECTED stage16_reject_reason=NOT_ARMED "
        "latched_stop=false rc_ok=true neutral_ok=true\n"
    )
    flags = station_guarded_path_crawl.parse_stage16_event_flags(rows)
    assert flags["arm_ack_seen"] is False
    assert flags["command_rejected_reason"] == "NOT_ARMED"


def test_stage16_stop_without_gps_reports_missing_gps_fields() -> None:
    package = _georef_package()
    rows = station_guarded_path_crawl.build_crawl_rows(
        package,
        "STAGE16 stage16_guarded_crawl=true event=STOP stage16_cmd_state=STOPPED "
        "physical_output_active=false rc_ok=true\n",
        dry_run=True,
        max_cmd=0.03,
        pulse_ms=100,
        max_total_distance_m=0.20,
    )
    assert rows[0]["stage16_firmware_ready"] is True
    assert rows[0]["reason"] == "NO_GPS_POSITION_FIELDS_IN_STAGE16_LOG"


def test_stage16_heartbeat_with_gps_yields_local_pose_available() -> None:
    package = _georef_package()
    rows = station_guarded_path_crawl.build_crawl_rows(
        package,
        "STAGE16 stage16_guarded_crawl=true event=HEARTBEAT stage16_firmware_ready=true "
        "stage16_cmd_state=IDLE physical_output_active=false rc_ok=true "
        "current_lat=35.0 current_lon=129.0 gps_block_reason=OK\n",
        dry_run=True,
        max_cmd=0.03,
        pulse_ms=100,
        max_total_distance_m=0.20,
    )
    summary = station_guarded_path_crawl._summary(
        rows,
        package=package,
        dry_run=True,
        max_cmd=0.03,
        pulse_ms=100,
        max_total_distance_m=0.20,
    )
    assert rows[0]["local_pose_available"] is True
    assert rows[0]["reason"] == "OK"
    assert summary["ready_for_stage16_guarded_crawl"] is True


def test_stage16_manual_local_pose_mode_computes_without_gps() -> None:
    package = _georef_package()
    rows = station_guarded_path_crawl.build_crawl_rows(
        package,
        "STAGE16 stage16_guarded_crawl=true event=HEARTBEAT stage16_firmware_ready=true "
        "stage16_cmd_state=IDLE physical_output_active=false rc_ok=true\n",
        dry_run=True,
        max_cmd=0.03,
        pulse_ms=100,
        max_total_distance_m=0.20,
        pose_mode="manual_local",
        current_x=0.0,
        current_y=0.0,
        current_heading_deg=0.0,
    )
    assert rows[0]["local_pose_available"] is True
    assert rows[0]["local_pose_source"] == "manual_local"
    assert rows[0]["gps_used_for_local_pose"] is False
    assert rows[0]["reason"] == "OK"


def test_stage16_non_dry_run_requires_readiness_and_local_pose() -> None:
    package = _georef_package()
    rows = station_guarded_path_crawl.build_crawl_rows(
        package,
        "STAGE16 stage16_guarded_crawl=true event=HEARTBEAT stage16_firmware_ready=false "
        "stage16_cmd_state=IDLE physical_output_active=false rc_ok=true current_lat=35.0 current_lon=129.0\n",
        dry_run=False,
        max_cmd=0.03,
        pulse_ms=100,
        max_total_distance_m=0.20,
    )
    assert rows[0]["stage16_firmware_ready"] is False
    assert rows[0]["command_sent"] is False
    assert rows[0]["motor_command_generated"] is False


def test_stage16_raw_usbdbg_log_is_generated(tmp_path: Path, monkeypatch) -> None:
    package = _georef_package()
    package_path = tmp_path / "path_package.json"
    import json

    package_path.write_text(json.dumps(package), encoding="utf-8")
    monkeypatch.setattr(
        station_guarded_path_crawl,
        "_read_live_lines",
        lambda *args, **kwargs: (
            "STAGE16 stage16_guarded_crawl=true event=HEARTBEAT stage16_firmware_ready=true "
            "stage16_cmd_state=IDLE physical_output_active=false rc_ok=true current_lat=35.0 current_lon=129.0\n"
        ),
    )
    assert station_guarded_path_crawl.main(
        [
            "--path-package",
            str(package_path),
            "--port",
            "/dev/null",
            "--duration-s",
            "0.1",
            "--dry-run",
            "true",
            "--out-dir",
            str(tmp_path / "out"),
        ]
    ) == 0
    raw = tmp_path / "out" / "raw_usbdbg.log"
    assert raw.exists()
    assert "event=HEARTBEAT" in raw.read_text(encoding="utf-8")


def test_stage16_dry_run_rc_not_ok_computes_but_not_ready() -> None:
    package = _georef_package()
    rows = station_guarded_path_crawl.build_crawl_rows(
        package,
        "STAGE16 stage16_guarded_crawl=true event=HEARTBEAT stage16_firmware_ready=true "
        "stage16_cmd_state=IDLE physical_output_active=false rc_ok=false current_lat=35.0 current_lon=129.0\n",
        dry_run=True,
        max_cmd=0.03,
        pulse_ms=100,
        max_total_distance_m=0.20,
    )
    summary = station_guarded_path_crawl._summary(
        rows,
        package=package,
        dry_run=True,
        max_cmd=0.03,
        pulse_ms=100,
        max_total_distance_m=0.20,
    )
    assert rows[0]["local_pose_available"] is True
    assert rows[0]["reason"] == "RC_NOT_OK"
    assert rows[0]["command_sent"] is False
    assert summary["ready_for_stage16_guarded_crawl"] is False
    assert summary["ready_for_full_path_following"] is False
