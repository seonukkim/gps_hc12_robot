from __future__ import annotations

import csv
import json
from pathlib import Path

from tools import check_stage28_closed_loop_segment
from tools import field_ab_to_serpentine
from tools import manual_rc_passthrough_validation
from tools import stage26_calibrated_primitive_sequence
from tools import stage28_closed_loop_segment
from tools import stage28_sensor_readiness
from tools import station_path_package_tracker


REPO = Path(__file__).resolve().parents[1]
SCRIPT = REPO / "legacy" / "stage_scripts" / "run_stage28_closed_loop_segment.sh"


def _georef_package() -> dict[str, object]:
    georef = {
        "georeference_available": True,
        "raw_A_lat": 35.0,
        "raw_A_lon": 129.0,
        "raw_B_lat": 35.0 - (1.2 / field_ab_to_serpentine.METERS_PER_DEG_LAT),
        "raw_B_lon": 129.0 + (8.0 / field_ab_to_serpentine._meters_per_deg_lon(35.0)),
        "origin_lat": 35.0,
        "origin_lon": 129.0,
        "local_frame_type": "equirectangular_enu",
        "x_axis_source": "normalized_rectangle",
        "x_axis_bearing_deg": 90.0,
        "meters_per_deg_lat": field_ab_to_serpentine.METERS_PER_DEG_LAT,
        "meters_per_deg_lon": field_ab_to_serpentine._meters_per_deg_lon(35.0),
        "meters_per_lat": field_ab_to_serpentine.METERS_PER_DEG_LAT,
        "meters_per_lon": field_ab_to_serpentine._meters_per_deg_lon(35.0),
    }
    return field_ab_to_serpentine.build_path_package(
        raw_a=(0.0, 1.2),
        raw_b=(8.0, 0.0),
        current_pose=(0.0, 1.2, 0.0),
        step_spacing_m=0.25,
        tool_side="left",
        tool_lateral_offset_m=0.24,
        tool_width_m=0.30,
        tool_length_m=0.18,
        robot_width_m=0.18,
        robot_length_m=0.18,
        georeference=georef,
    )


def _row(**overrides: object) -> dict[str, object]:
    row: dict[str, object] = {
        "pulse_index": 1,
        "mode": "straight_forward_segment",
        "segment_index": 0,
        "current_lat": 35.0,
        "current_lon": 129.0,
        "current_x_m": 0.0,
        "current_y_m": 1.2,
        "imu_relative_yaw_deg": 0.0,
        "current_heading_deg": 0.0,
        "target_heading_deg": 0.0,
        "heading_error_deg": 0.0,
        "cross_track_error_m": 0.0,
        "along_track_progress_m": 0.0,
        "a_cmd": 0.30,
        "b_cmd": 0.0,
        "pulse_ms": 250,
        "b_heading_component": 0.0,
        "b_cte_component": 0.0,
        "b_cmd_clamped": False,
        "arm_seen": True,
        "ack_seen": True,
        "stop_seen": True,
        "reject_seen": False,
        "rc_invalid_seen": False,
        "final_left_cmd": "0.000",
        "final_right_cmd": "0.000",
        "final_left_cmd_zero": True,
        "final_right_cmd_zero": True,
        "physical_output_active_after_stop": False,
        "user_motion_report": "forward",
        "valid_pulse": True,
        "invalid_reason": "NONE",
        "ready_for_full_path_following": False,
    }
    row.update(overrides)
    return row


def _write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=stage28_closed_loop_segment.STAGE28_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in stage28_closed_loop_segment.STAGE28_FIELDS})


def _write_stage26_csv(path: Path) -> None:
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
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=stage26_calibrated_primitive_sequence.STAGE26_FIELDS)
        writer.writeheader()
        writer.writerow({field: row.get(field, "") for field in stage26_calibrated_primitive_sequence.STAGE26_FIELDS})


def test_stage28_gps_lat_lon_to_local_conversion() -> None:
    package = _georef_package()
    georef = package["georeference"]  # type: ignore[index]
    lon = float(georef["origin_lon"]) + 1.0 / float(georef["meters_per_lon"])  # type: ignore[index]
    local = station_path_package_tracker.lat_lon_to_local(package, float(georef["origin_lat"]), lon)  # type: ignore[arg-type]
    assert local is not None
    assert abs(local[0] - 1.0) < 1e-6
    assert abs(local[1]) < 1e-6


def test_stage28_cross_track_error_sign() -> None:
    cte = stage28_closed_loop_segment.signed_cross_track_error(
        start_x=0.0,
        start_y=0.0,
        end_x=8.0,
        end_y=0.0,
        point_x=1.0,
        point_y=0.2,
    )
    assert cte == 0.2


def test_stage28_heading_unwrap() -> None:
    assert stage28_closed_loop_segment.unwrap_delta_deg(-179.0, 179.0) == 2.0
    assert stage28_closed_loop_segment.unwrap_delta_deg(179.0, -179.0) == -2.0


def test_stage28_heading_correction_sign_and_clamp() -> None:
    b_cmd, heading_component, cte_component = stage28_closed_loop_segment.closed_loop_b_command(
        heading_error_deg=20.0,
        cross_track_error_m=0.0,
        max_correction_b=0.12,
    )
    assert b_cmd == 0.12
    assert heading_component == 0.12
    assert cte_component == 0.0


def test_stage28_backward_reverses_turn_correction() -> None:
    forward_b, *_ = stage28_closed_loop_segment.closed_loop_b_command(
        heading_error_deg=10.0,
        cross_track_error_m=0.0,
        max_correction_b=0.12,
        backward=False,
    )
    backward_b, *_ = stage28_closed_loop_segment.closed_loop_b_command(
        heading_error_deg=10.0,
        cross_track_error_m=0.0,
        max_correction_b=0.12,
        backward=True,
        reverse_turn_correction_for_backward=True,
    )
    assert forward_b == 0.06
    assert backward_b == -0.06


def test_stage28_compute_control_snapshot() -> None:
    package = _georef_package()
    segment = stage28_closed_loop_segment.select_segment(package, 0)
    georef = package["georeference"]  # type: ignore[index]
    row = {
        "current_lat": str(georef["origin_lat"]),
        "current_lon": str(georef["origin_lon"]),
        "imu_relative_yaw_deg": "0.0",
    }
    snapshot = stage28_closed_loop_segment.compute_control_snapshot(
        package=package,
        row=row,
        segment=segment,
        mode="straight_forward_segment",
        segment_index=0,
        pulse_index=1,
        imu_yaw_zero_deg=0.0,
        initial_path_heading_deg=0.0,
        commands={
            "forward_base_a": 0.30,
            "forward_ms": 800,
            "backward_base_a": -0.08,
            "backward_ms": 300,
            "left_b": 0.26,
            "left_ms": 700,
            "right_b": -0.08,
            "right_ms": 250,
        },
        pulse_ms=250,
    )
    assert snapshot["a_cmd"] == 0.30
    assert abs(float(snapshot["b_cmd"])) <= 0.12
    assert snapshot["ready_for_full_path_following"] is False


def test_stage28_row_from_pulse_aborts_on_reject_and_rc_invalid() -> None:
    rows = stage28_closed_loop_segment.station_path_package_tracker.parse_usbdbg_rows(
        "STAGE20 event=ARM rc_ok=true neutral_ok=true physical_output_active=false\n"
        "STAGE20 event=REJECT stage20_reject_reason=RC_INVALID rc_ok=false\n"
        "STAGE20 event=STOP final_left_cmd=0.000 final_right_cmd=0.000 physical_output_active=false\n"
    )
    result = stage28_closed_loop_segment.row_from_pulse(_row(), rows, "unknown")
    assert result["valid_pulse"] is False
    assert result["invalid_reason"] == "RC_INVALID"
    assert result["rc_invalid_seen"] is True


def test_stage28_checker_requires_final_zero(tmp_path: Path) -> None:
    csv_path = tmp_path / "stage28_closed_loop_segment.csv"
    _write_csv(csv_path, [_row(final_left_cmd="0.020", final_left_cmd_zero=False, valid_pulse=False, invalid_reason="FINAL_COMMANDS_NONZERO")])
    result = check_stage28_closed_loop_segment.evaluate_path(csv_path)
    assert result["verdict"] == "FAIL"
    assert "FINAL_COMMANDS_NONZERO" in result["reasons"]
    assert result["ready_for_full_path_following"] is False


def test_stage28_checker_passes_safe_row(tmp_path: Path) -> None:
    csv_path = tmp_path / "stage28_closed_loop_segment.csv"
    _write_csv(csv_path, [_row()])
    result = check_stage28_closed_loop_segment.evaluate_path(csv_path)
    assert result["verdict"] == "PASS"
    assert result["ready_for_full_path_following"] is False


def test_stage28_script_safety_flags() -> None:
    text = SCRIPT.read_text(encoding="utf-8")
    assert "STAGE20_PHYSICAL_AB_GUARDED_CRAWL=1" in text
    assert "IMU_ENABLE=1" in text
    assert "IMU_YAW_DIAG=1" in text
    assert "PHYSICAL_PATH_FOLLOWING_ENABLE=0" in text
    assert "PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0" in text
    assert "PATH_FOLLOWING_HC12_ENABLED=0" in text
    assert "NOT FULL PATH FOLLOWING" in text
    assert "STAGE20_MAX_MS=1000" in text


def test_stage28_loads_calibrated_thresholds_and_durations() -> None:
    commands = stage28_closed_loop_segment.load_calibrated_commands(
        {
            "stage26_forward_a_cmd": 0.30,
            "stage26_forward_pulse_ms": 800,
            "stage26_backward_a_cmd": -0.08,
            "stage26_backward_pulse_ms": 300,
        },
        {
            "stage26_turn_left_b_cmd": 0.26,
            "stage26_turn_left_pulse_ms": 700,
            "stage26_turn_right_b_cmd": -0.08,
            "stage26_turn_right_pulse_ms": 250,
        },
    )
    assert commands["forward_base_a"] == 0.30
    assert commands["forward_ms"] == 800
    assert commands["backward_base_a"] == -0.08
    assert commands["backward_ms"] == 300
    assert commands["left_b"] == 0.26
    assert commands["left_ms"] == 700
    assert commands["right_b"] == -0.08
    assert commands["right_ms"] == 250
    assert stage28_closed_loop_segment.calibrated_pulse_for_mode("calibrated_forward_once", commands) == (0.30, 0.0, 800)


def test_stage28_imu_only_heading_hold_does_not_require_gps() -> None:
    package = _georef_package()
    segment = stage28_closed_loop_segment.select_segment(package, 0)
    snapshot = stage28_closed_loop_segment.compute_control_snapshot(
        package=package,
        row={"gps_block_reason": "NO_LOCATION", "imu_relative_yaw_deg": "3.0"},
        segment=segment,
        mode="correction_forward_segment",
        segment_index=0,
        pulse_index=1,
        imu_yaw_zero_deg=0.0,
        initial_path_heading_deg=0.0,
        commands={
            "forward_base_a": 0.30,
            "forward_ms": 800,
            "backward_base_a": -0.08,
            "backward_ms": 300,
            "left_b": 0.26,
            "left_ms": 700,
            "right_b": -0.08,
            "right_ms": 250,
        },
        pulse_ms=250,
        require_local_pose=False,
    )
    assert snapshot["current_x_m"] == ""
    assert snapshot["cross_track_error_m"] == 0.0
    assert snapshot["a_cmd"] == 0.30


def test_stage28_gps_closed_loop_requires_local_pose() -> None:
    class Args:
        mode = "correction_forward_segment"
        closed_loop_mode = "GPS_IMU_CLOSED_LOOP"
        allow_imu_only = "false"

    assert stage28_closed_loop_segment.gps_local_pose_required_for_args(Args()) is True
    Args.closed_loop_mode = "IMU_ONLY_HEADING_HOLD"
    assert stage28_closed_loop_segment.gps_local_pose_required_for_args(Args()) is False


def test_stage28_sensor_readiness_blocked_gps_and_imu_only() -> None:
    blocked = stage28_sensor_readiness.summarize_rows(stage28_sensor_readiness.parse_rows(
        "STAGE20 event=HEARTBEAT gps_block_reason=NO_LOCATION position_source=NA "
        "imu_type=BMI160 imu_present=true imu_data_plausible=true imu_calibrated=true "
        "rc_ok=true neutral_ok=true physical_output_active=false final_left_cmd=0.000 final_right_cmd=0.000\n"
    ))
    assert blocked["sensor_readiness"] == "READY_IMU_ONLY_HEADING"
    imu_only = stage28_sensor_readiness.summarize_rows(stage28_sensor_readiness.parse_rows(
        "STAGE20 event=HEARTBEAT position_source=NA "
        "imu_type=BMI160 imu_present=true imu_data_plausible=true imu_calibrated=true "
        "rc_ok=true neutral_ok=true physical_output_active=false final_left_cmd=0.000 final_right_cmd=0.000\n"
    ))
    assert imu_only["sensor_readiness"] == "READY_IMU_ONLY_HEADING"


def test_stage28_sensor_readiness_blocks_imu_calibrating() -> None:
    summary = stage28_sensor_readiness.summarize_rows(stage28_sensor_readiness.parse_rows(
        "STAGE20 event=HEARTBEAT gps_block_reason=OK position_source=gps current_lat=35 current_lon=129 "
        "imu_type=BMI160 imu_present=true imu_data_plausible=true imu_calibrated=false "
        "imu_heading_block_reason=CALIBRATING rc_ok=true neutral_ok=true physical_output_active=false "
        "final_left_cmd=0.000 final_right_cmd=0.000\n"
    ))
    assert summary["sensor_readiness"] == "BLOCKED_IMU_CALIBRATING"


def test_manual_rc_passthrough_detects_manual_final_commands() -> None:
    rows = manual_rc_passthrough_validation.parse_rows(
        "USBDBG mode=MANUAL control_source=RC_MANUAL rc_ok=true neutral_ok=false "
        "manual_forward_cmd=0.220 manual_turn_cmd=0.000 final_left_cmd=0.220 final_right_cmd=0.220 "
        "physical_output_active=true active_target_source=manual_rc\n"
        "USBDBG mode=MANUAL control_source=RC_MANUAL rc_ok=true neutral_ok=true "
        "manual_forward_cmd=0.000 manual_turn_cmd=0.000 final_left_cmd=0.000 final_right_cmd=0.000 "
        "physical_output_active=false active_target_source=manual_rc\n"
    )
    summary = manual_rc_passthrough_validation.evaluate_rows(rows)
    assert summary["manual_rc_passthrough_pass"] is True
    assert summary["station_command_used"] is False
    assert summary["ready_for_full_path_following"] is False


def test_stage28_recovery_gates_block_manual_rc_failure(tmp_path: Path) -> None:
    manual = tmp_path / "manual_rc_passthrough_summary.json"
    manual.write_text(json.dumps({
        "manual_rc_passthrough_pass": False,
        "ready_for_full_path_following": False,
    }), encoding="utf-8")
    stage26_csv = tmp_path / "stage26_calibrated_primitive_sequence.csv"
    _write_stage26_csv(stage26_csv)

    class Args:
        mode = "calibrated_forward_once"
        override_recovery_gates = "false"
        manual_rc_summary = str(manual)
        stage26_summary = str(stage26_csv)
        imu_readiness_summary = None
        gps_watch_summary = None
        closed_loop_mode = "GPS_IMU_CLOSED_LOOP"
        allow_imu_only = "false"

    reasons = stage28_closed_loop_segment.recovery_gate_reasons(Args())
    assert "BLOCKED_MANUAL_RC" in reasons


def test_stage28_recovery_gates_block_gps_no_serial_data(tmp_path: Path) -> None:
    manual = tmp_path / "manual_rc_passthrough_summary.json"
    manual.write_text(json.dumps({
        "manual_rc_passthrough_pass": True,
        "ready_for_full_path_following": False,
    }), encoding="utf-8")
    gps = tmp_path / "gps_link_and_fix_watch_summary.json"
    gps.write_text(json.dumps({
        "gps_status": "GPS_NO_SERIAL_DATA",
        "ready_for_full_path_following": False,
    }), encoding="utf-8")
    imu = tmp_path / "imu_readiness_summary.json"
    imu.write_text(json.dumps({
        "imu_status": "IMU_READY",
        "ready_for_full_path_following": False,
    }), encoding="utf-8")
    stage26_csv = tmp_path / "stage26_calibrated_primitive_sequence.csv"
    _write_stage26_csv(stage26_csv)

    class Args:
        mode = "correction_forward_segment"
        override_recovery_gates = "false"
        manual_rc_summary = str(manual)
        stage26_summary = str(stage26_csv)
        imu_readiness_summary = str(imu)
        gps_watch_summary = str(gps)
        closed_loop_mode = "GPS_IMU_CLOSED_LOOP"
        allow_imu_only = "false"

    reasons = stage28_closed_loop_segment.recovery_gate_reasons(Args())
    assert "BLOCKED_GPS_NO_SERIAL_DATA" in reasons


def test_stage28_recovery_gates_allow_imu_only_without_gps(tmp_path: Path) -> None:
    manual = tmp_path / "manual_rc_passthrough_summary.json"
    manual.write_text(json.dumps({
        "manual_rc_passthrough_pass": True,
        "ready_for_full_path_following": False,
    }), encoding="utf-8")
    imu = tmp_path / "imu_readiness_summary.json"
    imu.write_text(json.dumps({
        "imu_status": "IMU_READY",
        "ready_for_full_path_following": False,
    }), encoding="utf-8")
    stage26_csv = tmp_path / "stage26_calibrated_primitive_sequence.csv"
    _write_stage26_csv(stage26_csv)

    class Args:
        mode = "correction_forward_segment"
        override_recovery_gates = "false"
        manual_rc_summary = str(manual)
        stage26_summary = str(stage26_csv)
        imu_readiness_summary = str(imu)
        gps_watch_summary = None
        closed_loop_mode = "IMU_ONLY_HEADING_HOLD"
        allow_imu_only = "true"

    reasons = stage28_closed_loop_segment.recovery_gate_reasons(Args())
    assert reasons == []
