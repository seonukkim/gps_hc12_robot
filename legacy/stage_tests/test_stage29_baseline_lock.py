from __future__ import annotations

import csv
from pathlib import Path

from tools import check_stage29_baseline_lock
from tools import stage26_calibrated_primitive_sequence
from tools import stage29_baseline_lock


REPO = Path(__file__).resolve().parents[1]
SCRIPT = REPO / "legacy" / "stage_scripts" / "run_stage29_baseline_lock.sh"


def _stage26_row(name: str, a: float, b: float, ms: int) -> dict[str, object]:
    return {
        "primitive_index": 1,
        "primitive_name": name,
        "repeat_index": 1,
        "a_cmd": a,
        "b_cmd": b,
        "pulse_ms": ms,
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
        "user_motion_report": name,
        "user_turn_angle_report": "none",
        "valid_primitive": True,
        "invalid_reason": "NONE",
        "ready_for_full_path_following": False,
    }


def _station_pulse_rows() -> list[dict[str, str]]:
    rows = [
        _stage26_row("forward", 0.30, 0.0, 800),
        _stage26_row("backward", -0.08, 0.0, 300),
        _stage26_row("turn_left", 0.0, 0.26, 700),
        _stage26_row("turn_right", 0.0, -0.08, 250),
    ]
    return [{key: str(row.get(key, "")) for key in stage26_calibrated_primitive_sequence.STAGE26_FIELDS} for row in rows]


def test_stage29_parser_detects_manual_rc_pass() -> None:
    rows = stage29_baseline_lock.parse_usb_rows(
        "USBDBG manual_rc_recovery=true mode=MANUAL control_source=RC_MANUAL rc_ok=true "
        "active_target_source=manual_rc neutral_ok=false final_left_cmd=0.210 final_right_cmd=0.210 "
        "motor_write_called=true physical_output_active=true\n"
        "USBDBG manual_rc_recovery=true mode=MANUAL control_source=RC_MANUAL rc_ok=true "
        "active_target_source=manual_rc neutral_ok=true final_left_cmd=0.000 final_right_cmd=0.000 "
        "motor_write_called=false physical_output_active=false\n"
    )
    result = stage29_baseline_lock.evaluate_manual_rc_baseline(rows)
    assert result["status"] == "PASS"
    assert result["ready_for_full_path_following"] is False


def test_stage29_parser_detects_gps_pass() -> None:
    rows = stage29_baseline_lock.parse_usb_rows(
        "USBDBG gps_block_reason=OK gps_location_valid=true gps_location_fresh=true "
        "gps_solution_valid=true gps_ready=true gps_lat=35.0 gps_lon=129.0 "
        "gps_sats=6 gps_hdop=1.85 last_rmc_status=A last_gga_fix_quality=1\n"
    )
    result = stage29_baseline_lock.evaluate_gps_baseline(rows)
    assert result["status"] == "PASS"


def test_stage29_parser_detects_imu_pass() -> None:
    rows = stage29_baseline_lock.parse_usb_rows(
        "USBDBG imu_type=BMI160 imu_present=true imu_data_plausible=true "
        "imu_calibrated=true imu_heading_ready=true imu_heading_block_reason=RELATIVE_YAW_DIAG_ONLY\n"
    )
    result = stage29_baseline_lock.evaluate_imu_baseline(rows)
    assert result["status"] == "PASS"


def test_stage29_parser_detects_station_pulse_pass() -> None:
    result = stage29_baseline_lock.evaluate_station_pulse_baseline(csv_rows=_station_pulse_rows())
    assert result["status"] == "PASS"


def test_stage29_ready_for_stage30_only_when_all_baselines_pass() -> None:
    manual_rows = stage29_baseline_lock.parse_usb_rows(
        "USBDBG mode=MANUAL control_source=RC_MANUAL rc_ok=true active_target_source=manual_rc "
        "neutral_ok=false final_left_cmd=0.200 final_right_cmd=0.200 physical_output_active=true\n"
        "USBDBG mode=MANUAL control_source=RC_MANUAL rc_ok=true active_target_source=manual_rc "
        "neutral_ok=true final_left_cmd=0.000 final_right_cmd=0.000 physical_output_active=false\n"
    )
    sensor_rows = stage29_baseline_lock.parse_usb_rows(
        "USBDBG gps_block_reason=OK gps_location_valid=true gps_lat=35 gps_lon=129 gps_sats=6 gps_hdop=1.8 "
        "imu_type=BMI160 imu_present=true imu_data_plausible=true imu_calibrated=true imu_heading_ready=true\n"
    )
    payload, _rows = stage29_baseline_lock.build_baseline_lock(
        manual_rows=manual_rows,
        sensor_rows=sensor_rows,
        station_pulse_csv_rows=_station_pulse_rows(),
        station_pulse_usb_rows=[],
        min_sats=5,
        max_hdop=2.5,
    )
    assert payload["ready_for_stage30_physical_path_planning"] is True
    assert payload["ready_for_full_path_following"] is False
    check = check_stage29_baseline_lock.evaluate_payload(payload)
    assert check["verdict"] == "PASS"


def test_stage29_stage30_blocked_when_gps_fails() -> None:
    payload, _rows = stage29_baseline_lock.build_baseline_lock(
        manual_rows=stage29_baseline_lock.parse_usb_rows(
            "USBDBG mode=MANUAL control_source=RC_MANUAL rc_ok=true active_target_source=manual_rc "
            "neutral_ok=false final_left_cmd=0.200 final_right_cmd=0.200 physical_output_active=true\n"
            "USBDBG mode=MANUAL control_source=RC_MANUAL rc_ok=true active_target_source=manual_rc "
            "neutral_ok=true final_left_cmd=0.000 final_right_cmd=0.000 physical_output_active=false\n"
        ),
        sensor_rows=stage29_baseline_lock.parse_usb_rows(
            "USBDBG gps_block_reason=NO_LOCATION gps_lat=NA gps_lon=NA "
            "imu_type=BMI160 imu_present=true imu_data_plausible=true imu_calibrated=true imu_heading_ready=true\n"
        ),
        station_pulse_csv_rows=_station_pulse_rows(),
        station_pulse_usb_rows=[],
        min_sats=5,
        max_hdop=2.5,
    )
    assert payload["gps_baseline"] == "FAIL"
    assert payload["ready_for_stage30_physical_path_planning"] is False
    assert payload["ready_for_full_path_following"] is False


def test_stage29_writes_expected_outputs(tmp_path: Path) -> None:
    manual_log = tmp_path / "manual.log"
    sensor_log = tmp_path / "sensor.log"
    pulse_csv = tmp_path / "stage26_calibrated_primitive_sequence.csv"
    manual_log.write_text(
        "USBDBG mode=MANUAL control_source=RC_MANUAL rc_ok=true active_target_source=manual_rc "
        "neutral_ok=false final_left_cmd=0.200 final_right_cmd=0.200 physical_output_active=true\n"
        "USBDBG mode=MANUAL control_source=RC_MANUAL rc_ok=true active_target_source=manual_rc "
        "neutral_ok=true final_left_cmd=0.000 final_right_cmd=0.000 physical_output_active=false\n",
        encoding="utf-8",
    )
    sensor_log.write_text(
        "USBDBG gps_block_reason=OK gps_location_valid=true gps_lat=35 gps_lon=129 gps_sats=6 gps_hdop=1.8 "
        "imu_type=BMI160 imu_present=true imu_data_plausible=true imu_calibrated=true imu_heading_ready=true\n",
        encoding="utf-8",
    )
    with pulse_csv.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=stage26_calibrated_primitive_sequence.STAGE26_FIELDS)
        writer.writeheader()
        for row in _station_pulse_rows():
            writer.writerow(row)
    out_dir = tmp_path / "out"
    assert stage29_baseline_lock.main([
        "--mode", "evaluate_logs",
        "--manual-log", str(manual_log),
        "--sensor-log", str(sensor_log),
        "--station-pulse-csv", str(pulse_csv),
        "--out-dir", str(out_dir),
    ]) == 0
    assert (out_dir / "summary.md").exists()
    assert (out_dir / "baseline_lock.json").exists()
    assert (out_dir / "stage29_baseline_lock.csv").exists()


def test_stage29_script_syntax_and_safety_flags() -> None:
    text = SCRIPT.read_text(encoding="utf-8")
    assert "PHYSICAL_PATH_FOLLOWING_ENABLE=0" in text
    assert "PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0" in text
    assert "PATH_FOLLOWING_HC12_ENABLED=0" in text
    assert "STAGE20_MAX_MS=1000" in text
    assert "calibration_forward=A0.30_B0.00_800ms" in text
