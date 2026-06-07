from __future__ import annotations

import csv
import json
from pathlib import Path

from tools import capture_field_ab_points
from tools import capture_georef_ab_points
from tools import check_georef_path_package
from tools import check_path_no_motion_summary
from tools import field_ab_to_serpentine
from tools import inspect_path_package
from tools import path_no_motion_validation
from tools import physical_path_preview_from_package
from tools import analyze_integrated_dryrun_for_path_package
from tools import station_path_package_tracker
from tools import station_virtual_path_controller


def test_capture_field_ab_points_writes_manual_outputs(tmp_path: Path) -> None:
    assert capture_field_ab_points.main(
        [
            "--a-x",
            "2",
            "--a-y",
            "0",
            "--b-x",
            "10",
            "--b-y",
            "1.2",
            "--out-dir",
            str(tmp_path),
        ]
    ) == 0

    data = json.loads((tmp_path / "field_points.json").read_text(encoding="utf-8"))
    assert data["points"]["A"]["x_m"] == 2.0
    assert data["points"]["B"]["y_m"] == 1.2
    assert data["motor_command_generated"] is False
    with (tmp_path / "field_points.csv").open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert [row["point_label"] for row in rows] == ["A", "B"]


def test_ab_normalization_produces_a_prime_top_left_and_b_prime_bottom_right() -> None:
    workspace = field_ab_to_serpentine.normalize_field_ab(a_x=8.0, a_y=0.0, b_x=0.0, b_y=1.2)

    assert workspace["x_min_m"] == 0.0
    assert workspace["x_max_m"] == 8.0
    assert workspace["y_min_m"] == 0.0
    assert workspace["y_max_m"] == 1.2
    assert workspace["A_prime_top_left"] == {"x_m": 0.0, "y_m": 1.2}
    assert workspace["B_prime_bottom_right"] == {"x_m": 8.0, "y_m": 0.0}


def test_field_ab_to_serpentine_package_has_safe_tool_and_primitives(tmp_path: Path) -> None:
    assert field_ab_to_serpentine.main(
        [
            "--a-x",
            "8",
            "--a-y",
            "0",
            "--b-x",
            "0",
            "--b-y",
            "1.2",
            "--current-x",
            "8",
            "--current-y",
            "0",
            "--current-heading-deg",
            "0",
            "--step-spacing-m",
            "0.25",
            "--tool-side",
            "left",
            "--tool-lateral-offset-m",
            "0.24",
            "--tool-width-m",
            "0.30",
            "--tool-length-m",
            "0.18",
            "--robot-width-m",
            "0.18",
            "--robot-length-m",
            "0.18",
            "--out-dir",
            str(tmp_path),
        ]
    ) == 0

    package = json.loads((tmp_path / "path_package.json").read_text(encoding="utf-8"))
    summary = package["summary"]
    assert summary["A_prime_top_left"] == {"x_m": 0.0, "y_m": 1.2}
    assert summary["B_prime_bottom_right"] == {"x_m": 8.0, "y_m": 0.0}
    assert summary["approach_path_generated"] is True
    assert summary["serpentine_path_generated"] is True
    assert summary["tool_side"] == "left"
    assert summary["primitive_sequence_valid"] is True
    assert summary["motor_command_generated"] is False
    assert summary["physical_output_active"] is False
    assert summary["ready_for_outdoor_no_motion_validation"] is True

    with (tmp_path / "tool_path.csv").open(encoding="utf-8", newline="") as handle:
        tool_rows = list(csv.DictReader(handle))
    assert tool_rows[0]["tool_start_x_m"] == "0.0"
    assert tool_rows[0]["tool_start_y_m"] == "1.2"
    assert tool_rows[-1]["tool_end_x_m"] == "8.0"
    assert tool_rows[-1]["tool_end_y_m"] == "0.0"
    assert {row["tool_active"] for row in tool_rows if row["tool_segment_type"] == "tool_sweep_track"} == {"True"}
    assert {row["tool_active"] for row in tool_rows if row["tool_segment_type"] == "tool_spacing_connector"} == {"False"}

    with (tmp_path / "primitive_sequence.csv").open(encoding="utf-8", newline="") as handle:
        primitive_rows = list(csv.DictReader(handle))
    assert {row["primitive_type"] for row in primitive_rows} <= {
        "move_forward",
        "move_backward",
        "rotate_left",
        "rotate_right",
    }
    assert {row["tool_active"] for row in primitive_rows if row["segment_role"].startswith("approach")} == {"False"}
    assert {row["motor_command_generated"] for row in primitive_rows} == {"False"}
    assert (tmp_path / "normalized_workspace.json").exists()
    assert (tmp_path / "preview_workspace_ab_aprime_bprime.png").exists()
    assert (tmp_path / "preview_tool_path_primary.png").exists()
    assert (tmp_path / "preview_primitive_sequence.png").exists()
    assert (tmp_path / "preview_approach_then_serpentine.png").exists()


def test_no_motion_validation_parser_accepts_sample_log(tmp_path: Path) -> None:
    package = field_ab_to_serpentine.build_path_package(
        raw_a=(8.0, 0.0),
        raw_b=(0.0, 1.2),
        current_pose=(8.0, 0.0, 0.0),
        step_spacing_m=0.25,
        tool_side="left",
        tool_lateral_offset_m=0.24,
        tool_width_m=0.30,
        tool_length_m=0.18,
        robot_width_m=0.18,
        robot_length_m=0.18,
    )
    package_path = tmp_path / "path_package.json"
    package_path.write_text(json.dumps(package), encoding="utf-8")
    sample_log = tmp_path / "status.log"
    sample_log.write_text(
        "position_source=gps gps_sats=6 gps_hdop=1.2 "
        "bmi160_ok=true rc_manual_ok=true x_m=8.0 y_m=0.0 "
        "gps_course_deg=NA physical_output_active=false\n",
        encoding="utf-8",
    )

    assert path_no_motion_validation.main(
        [
            "--path-package",
            str(package_path),
            "--sample-log",
            str(sample_log),
            "--port",
            "/dev/ttyACM0",
            "--concise",
            "true",
            "--no-motion-gps-mode",
            "position_only",
            "--min-sats",
            "4",
            "--max-hdop",
            "3.0",
            "--mode",
            "package_check",
            "--out-dir",
            str(tmp_path / "validation"),
        ]
    ) == 0

    summary = (tmp_path / "validation" / "summary.md").read_text(encoding="utf-8")
    assert "validation_mode: `package_check`" in summary
    assert "serial_opened: `False`" in summary
    assert f"selected_path_package: `{package_path}`" in summary
    assert "gps_status: `OK`" in summary
    assert "gps_sats: `6`" in summary
    assert "gps_hdop: `1.2`" in summary
    assert "position_source: `gps`" in summary
    assert "heading_status: `WAITING_FOR_MOTION_OR_DIAG_ONLY`" in summary
    assert "ready_for_outdoor_no_motion_validation: `True`" in summary
    with (tmp_path / "validation" / "no_motion_validation.csv").open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert rows
    assert "along_track_progress_m" not in rows[0]
    assert rows[0]["heading_error_deg"] == "NA_DIAG_ONLY"
    assert {row["motor_command_generated"] for row in rows} == {"False"}
    assert {row["physical_output_active"] for row in rows} == {"False"}


def test_path_package_latest_resolves_field_ab_latest(tmp_path: Path, monkeypatch) -> None:
    latest = tmp_path / "outputs" / "field_ab_serpentine" / "latest" / "path_package.json"
    latest.parent.mkdir(parents=True)
    latest.write_text("{}", encoding="utf-8")
    monkeypatch.chdir(tmp_path)

    assert path_no_motion_validation.resolve_path_package("latest") == latest


def test_nonexistent_path_package_fails_without_traceback(tmp_path: Path, capsys) -> None:
    assert path_no_motion_validation.main(
        [
            "--path-package",
            str(tmp_path / "missing" / "path_package.json"),
            "--out-dir",
            str(tmp_path / "validation"),
        ]
    ) == 2
    output = capsys.readouterr().out
    assert "provided_path_package=" in output
    assert "file_exists=false" in output
    assert "nearest_candidates:" in output
    assert "Traceback" not in output


def test_package_check_mode_labels_serial_not_opened(tmp_path: Path) -> None:
    package = field_ab_to_serpentine.build_path_package(
        raw_a=(8.0, 0.0),
        raw_b=(0.0, 1.2),
        current_pose=(8.0, 0.0, 0.0),
        step_spacing_m=0.25,
        tool_side="left",
        tool_lateral_offset_m=0.24,
        tool_width_m=0.30,
        tool_length_m=0.18,
        robot_width_m=0.18,
        robot_length_m=0.18,
    )
    package_path = tmp_path / "path_package.json"
    package_path.write_text(json.dumps(package), encoding="utf-8")

    assert path_no_motion_validation.main(
        [
            "--path-package",
            str(package_path),
            "--port",
            "/dev/ttyACM0",
            "--mode",
            "package_check",
            "--out-dir",
            str(tmp_path / "validation"),
        ]
    ) == 0
    summary = (tmp_path / "validation" / "summary.md").read_text(encoding="utf-8")
    assert "validation_mode: `package_check`" in summary
    assert "serial_opened: `False`" in summary
    assert "gps_status: `NOT_CHECKED_PACKAGE_ONLY`" in summary
    assert "Package-only check complete" in summary


def test_live_serial_mode_fails_clearly_if_port_cannot_open(tmp_path: Path, capsys) -> None:
    package = field_ab_to_serpentine.build_path_package(
        raw_a=(8.0, 0.0),
        raw_b=(0.0, 1.2),
        current_pose=(8.0, 0.0, 0.0),
        step_spacing_m=0.25,
        tool_side="left",
        tool_lateral_offset_m=0.24,
        tool_width_m=0.30,
        tool_length_m=0.18,
        robot_width_m=0.18,
        robot_length_m=0.18,
    )
    package_path = tmp_path / "path_package.json"
    package_path.write_text(json.dumps(package), encoding="utf-8")

    assert path_no_motion_validation.main(
        [
            "--path-package",
            str(package_path),
            "--port",
            str(tmp_path / "missing_port"),
            "--mode",
            "live_serial",
            "--duration-s",
            "0.1",
            "--out-dir",
            str(tmp_path / "validation"),
        ]
    ) == 2
    output = capsys.readouterr().out
    assert "validation_mode=live_serial" in output
    assert "serial_opened=false" in output
    assert "error=" in output
    assert "Traceback" not in output


def test_no_motion_summary_checker_reports_pass_wait_and_fail() -> None:
    base = {
        "motor_command_generated": "False",
        "physical_output_active": "False",
        "path_package_loaded": "True",
        "serial_opened": "True",
        "validation_mode": "live_serial",
        "position_source": "gps",
        "gps_status": "OK",
        "gps_sats": "6",
        "gps_hdop": "1.2",
        "target_distance_finite": "True",
        "target_bearing_finite": "True",
        "heading_status": "OK",
    }
    assert check_path_no_motion_summary.evaluate_summary(base) == (
        "PASS",
        "target preview ready",
    )

    marginal = base | {"gps_sats": "3", "gps_status": "WAIT"}
    verdict, action = check_path_no_motion_summary.evaluate_summary(marginal)
    assert verdict == "WAIT"
    assert "satellites" in action

    stationary = base | {"heading_status": "WAITING_FOR_MOTION_OR_DIAG_ONLY"}
    verdict, action = check_path_no_motion_summary.evaluate_summary(stationary)
    assert verdict == "WAIT"
    assert "stationary" in action

    unsafe = base | {"physical_output_active": "True"}
    verdict, action = check_path_no_motion_summary.evaluate_summary(unsafe)
    assert verdict == "FAIL"
    assert "physical output" in action


def test_inspect_path_package_validates_known_good_package(tmp_path: Path) -> None:
    package = field_ab_to_serpentine.build_path_package(
        raw_a=(8.0, 0.0),
        raw_b=(0.0, 1.2),
        current_pose=(8.0, 0.0, 0.0),
        step_spacing_m=0.25,
        tool_side="left",
        tool_lateral_offset_m=0.24,
        tool_width_m=0.30,
        tool_length_m=0.18,
        robot_width_m=0.18,
        robot_length_m=0.18,
    )
    package_path = tmp_path / "path_package.json"
    package_path.write_text(json.dumps(package), encoding="utf-8")

    assert inspect_path_package.main(
        ["--path-package", str(package_path), "--out-dir", str(tmp_path / "inspection")]
    ) == 0
    inspection = json.loads((tmp_path / "inspection" / "path_package_inspection.json").read_text(encoding="utf-8"))
    validation = inspection["validation"]
    assert validation["tool_side_left"] is True
    assert validation["tool_path_starts_at_A_prime"] is True
    assert validation["tool_path_ends_at_B_prime"] is True
    assert validation["tool_path_continuous"] is True
    assert validation["connectors_inactive"] is True
    assert validation["sweep_tracks_active"] is True
    assert validation["primitive_sequence_allowed"] is True
    assert validation["motor_command_generated_false"] is True
    assert (tmp_path / "inspection" / "path_package_inspection.md").exists()


def test_physical_path_preview_from_package_generates_offline_outputs(tmp_path: Path) -> None:
    package = field_ab_to_serpentine.build_path_package(
        raw_a=(8.0, 0.0),
        raw_b=(0.0, 1.2),
        current_pose=(8.0, 0.0, 0.0),
        step_spacing_m=0.25,
        tool_side="left",
        tool_lateral_offset_m=0.24,
        tool_width_m=0.30,
        tool_length_m=0.18,
        robot_width_m=0.18,
        robot_length_m=0.18,
    )
    package_path = tmp_path / "path_package.json"
    package_path.write_text(json.dumps(package), encoding="utf-8")

    assert physical_path_preview_from_package.main(
        ["--path-package", str(package_path), "--out-dir", str(tmp_path / "preview")]
    ) == 0
    assert (tmp_path / "preview" / "summary.md").exists()
    assert (tmp_path / "preview" / "primitive_sequence_checked.csv").exists()
    assert (tmp_path / "preview" / "preview_tool_path.png").exists()
    with (tmp_path / "preview" / "primitive_sequence_checked.csv").open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert rows
    assert {row["primitive_allowed"] for row in rows} == {"True"}
    assert {row["motor_command_generated"] for row in rows} == {"False"}


def test_integrated_dryrun_analyzer_detects_compile_time_target_and_course_skip(tmp_path: Path) -> None:
    log = tmp_path / "integrated.log"
    log.write_text(
        "gps_block_reason=OK gps_sats=5 gps_hdop=1.88 position_source=gps "
        "imu_type=BMI160 imu_data_plausible=true imu_calibrated=true "
        "physical_compile_gate=false physical_path_following_enable=false "
        "allow_motor_output=false physical_output_active=false "
        "gps_course_deg=NA gps_course_output_block_reason=NO_ACCEPTED_COURSE_YET "
        "path_following_block_reason=NO_HEADING active_target_source=compile_time wp_count=3 "
        "target_distance_m=68.0\n",
        encoding="utf-8",
    )
    result = analyze_integrated_dryrun_for_path_package.analyze_files([log])
    assert result["gps_position_ok"] is True
    assert result["imu_ok"] is True
    assert result["motor_safety_ok"] is True
    assert result["gps_course_status"] == "SKIPPED_NO_MOTION_OR_TETHERED"
    assert result["active_target_source"] == "compile_time"
    assert result["live_path_package_connected"] is False
    assert result["current_target_source_is_compile_time"] is True
    assert result["reason"] == "ACTIVE_TARGET_SOURCE_COMPILE_TIME"
    assert "Do not proceed to motor tests" in str(result["recommendation"])
    assert result["motor_command_generated"] is False


def test_station_tracker_offline_pose_computes_target_status(tmp_path: Path) -> None:
    package = field_ab_to_serpentine.build_path_package(
        raw_a=(8.0, 0.0),
        raw_b=(0.0, 1.2),
        current_pose=(8.0, 0.0, 0.0),
        step_spacing_m=0.25,
        tool_side="left",
        tool_lateral_offset_m=0.24,
        tool_width_m=0.30,
        tool_length_m=0.18,
        robot_width_m=0.18,
        robot_length_m=0.18,
    )
    package_path = tmp_path / "path_package.json"
    package_path.write_text(json.dumps(package), encoding="utf-8")

    assert station_path_package_tracker.main(
        [
            "--path-package",
            str(package_path),
            "--mode",
            "offline_pose",
            "--current-x",
            "0",
            "--current-y",
            "1.2",
            "--current-heading-deg",
            "0",
            "--out-dir",
            str(tmp_path / "tracker"),
        ]
    ) == 0
    with (tmp_path / "tracker" / "station_target_status.csv").open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert rows
    assert rows[0]["station_package_target_source"] == "path_package"
    assert rows[0]["target_distance_m"] != "NA"
    assert rows[0]["target_bearing_deg"] != "NA"
    assert rows[0]["motor_command_generated"] == "False"
    summary = (tmp_path / "tracker" / "summary.md").read_text(encoding="utf-8")
    assert "ready_for_station_side_target_preview: `True`" in summary
    assert "ready_for_motor_test: `False`" in summary


def test_station_tracker_replay_log_reports_no_georeference_and_compile_time_source(tmp_path: Path) -> None:
    package = field_ab_to_serpentine.build_path_package(
        raw_a=(8.0, 0.0),
        raw_b=(0.0, 1.2),
        current_pose=(8.0, 0.0, 0.0),
        step_spacing_m=0.25,
        tool_side="left",
        tool_lateral_offset_m=0.24,
        tool_width_m=0.30,
        tool_length_m=0.18,
        robot_width_m=0.18,
        robot_length_m=0.18,
    )
    package_path = tmp_path / "path_package.json"
    package_path.write_text(json.dumps(package), encoding="utf-8")
    log = tmp_path / "usbdbg.log"
    log.write_text(
        "current_lat=37.1 current_lon=127.2 active_target_source=compile_time "
        "gps_block_reason=OK gps_sats=5 gps_hdop=1.8 physical_output_active=false\n",
        encoding="utf-8",
    )

    assert station_path_package_tracker.main(
        [
            "--path-package",
            str(package_path),
            "--mode",
            "replay_log",
            "--log",
            str(log),
            "--out-dir",
            str(tmp_path / "tracker"),
        ]
    ) == 0
    with (tmp_path / "tracker" / "station_target_status.csv").open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert rows[0]["local_pose_available"] == "False"
    assert rows[0]["reason"] == "NO_GEOREFERENCE_FOR_LAT_LON_TO_LOCAL"
    assert rows[0]["firmware_active_target_source"] == "compile_time"
    assert rows[0]["station_package_target_source"] == "path_package"
    assert rows[0]["target_distance_m"] == "NA"
    summary = (tmp_path / "tracker" / "summary.md").read_text(encoding="utf-8")
    assert "firmware_still_compile_time: `True`" in summary
    assert "ready_for_station_side_target_preview: `False`" in summary
    assert "ready_for_motor_test: `False`" in summary


def test_station_tracker_usbdbg_parser_handles_local_rows_without_serial() -> None:
    package = field_ab_to_serpentine.build_path_package(
        raw_a=(8.0, 0.0),
        raw_b=(0.0, 1.2),
        current_pose=(8.0, 0.0, 0.0),
        step_spacing_m=0.25,
        tool_side="left",
        tool_lateral_offset_m=0.24,
        tool_width_m=0.30,
        tool_length_m=0.18,
        robot_width_m=0.18,
        robot_length_m=0.18,
    )
    rows = station_path_package_tracker.build_rows_from_replay(
        package,
        "current_x_m=0 current_y_m=1.2 current_heading_deg=0 active_target_source=compile_time\n",
        "live_usbdbg",
    )
    assert rows[0]["local_pose_available"] is True
    assert rows[0]["station_package_target_source"] == "path_package"
    assert rows[0]["firmware_active_target_source"] == "compile_time"
    assert rows[0]["motor_command_generated"] is False


def test_manual_latlon_georef_capture_writes_json(tmp_path: Path) -> None:
    assert capture_georef_ab_points.main(
        [
            "--mode",
            "manual_latlon",
            "--a-lat",
            "35.0",
            "--a-lon",
            "129.0",
            "--b-lat",
            "35.00001",
            "--b-lon",
            "129.00010",
            "--out-dir",
            str(tmp_path / "capture"),
        ]
    ) == 0
    data = json.loads((tmp_path / "capture" / "field_points_georef.json").read_text(encoding="utf-8"))
    assert data["georeference_available"] is True
    assert data["points"]["A"]["lat"] == 35.0
    assert data["points"]["B"]["lon"] == 129.00010
    assert data["motor_command_generated"] is False
    assert (tmp_path / "capture" / "field_points_georef.csv").exists()


def test_field_ab_to_serpentine_accepts_georef_points(tmp_path: Path) -> None:
    capture_dir = tmp_path / "capture"
    assert capture_georef_ab_points.main(
        [
            "--mode",
            "manual_latlon",
            "--a-lat",
            "35.0",
            "--a-lon",
            "129.0",
            "--b-lat",
            "35.00001",
            "--b-lon",
            "129.00010",
            "--out-dir",
            str(capture_dir),
        ]
    ) == 0
    assert field_ab_to_serpentine.main(
        [
            "--field-points-georef-json",
            str(capture_dir / "field_points_georef.json"),
            "--step-spacing-m",
            "0.25",
            "--tool-side",
            "left",
            "--tool-lateral-offset-m",
            "0.24",
            "--tool-width-m",
            "0.30",
            "--tool-length-m",
            "0.18",
            "--robot-width-m",
            "0.18",
            "--robot-length-m",
            "0.18",
            "--out-dir",
            str(tmp_path / "package"),
        ]
    ) == 0
    package = json.loads((tmp_path / "package" / "path_package.json").read_text(encoding="utf-8"))
    assert package["georeference"]["georeference_available"] is True
    assert package["georeference"]["local_frame_type"] == "equirectangular_enu"
    assert package["georeference"]["origin_lat"] == 35.0
    assert package["summary"]["georeference_available"] is True
    assert package["summary"]["motor_command_generated"] is False


def test_station_tracker_replay_log_converts_lat_lon_with_georef(tmp_path: Path) -> None:
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
    package = field_ab_to_serpentine.build_path_package(
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
    package_path = tmp_path / "path_package.json"
    package_path.write_text(json.dumps(package), encoding="utf-8")
    log = tmp_path / "usbdbg.log"
    log.write_text(
        "current_lat=35.0 current_lon=129.0 active_target_source=compile_time physical_output_active=false\n",
        encoding="utf-8",
    )
    assert station_path_package_tracker.main(
        [
            "--path-package",
            str(package_path),
            "--mode",
            "replay_log",
            "--log",
            str(log),
            "--out-dir",
            str(tmp_path / "tracker"),
        ]
    ) == 0
    with (tmp_path / "tracker" / "station_target_status.csv").open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert rows[0]["local_pose_available"] == "True"
    assert rows[0]["local_pose_source"] == "gps_georeference"
    assert rows[0]["reason"] == "OK"
    assert rows[0]["firmware_active_target_source"] == "compile_time"
    assert rows[0]["station_package_target_source"] == "path_package"
    assert rows[0]["target_distance_m"] != "NA"
    summary = (tmp_path / "tracker" / "summary.md").read_text(encoding="utf-8")
    assert "firmware_still_compile_time: `True`" in summary
    assert "ready_for_station_side_target_preview: `True`" in summary
    assert "ready_for_motor_test: `False`" in summary


def test_check_georef_path_package_reports_metadata(tmp_path: Path) -> None:
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
    package = field_ab_to_serpentine.build_path_package(
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
    package_path = tmp_path / "path_package.json"
    package_path.write_text(json.dumps(package), encoding="utf-8")
    result = check_georef_path_package.inspect_georef(package, package_path)
    assert result["georeference_available"] is True
    assert result["local_frame_type"] == "equirectangular_enu"
    assert result["conversion_sanity_check"] is True
    assert result["motor_command_generated"] is False


def test_virtual_controller_offline_pose_produces_virtual_control(tmp_path: Path) -> None:
    package = field_ab_to_serpentine.build_path_package(
        raw_a=(8.0, 0.0),
        raw_b=(0.0, 1.2),
        current_pose=(8.0, 0.0, 0.0),
        step_spacing_m=0.25,
        tool_side="left",
        tool_lateral_offset_m=0.24,
        tool_width_m=0.30,
        tool_length_m=0.18,
        robot_width_m=0.18,
        robot_length_m=0.18,
    )
    package_path = tmp_path / "path_package.json"
    package_path.write_text(json.dumps(package), encoding="utf-8")
    assert station_virtual_path_controller.main(
        [
            "--path-package",
            str(package_path),
            "--mode",
            "offline_pose",
            "--current-x",
            "0",
            "--current-y",
            "1.2",
            "--current-heading-deg",
            "0",
            "--out-dir",
            str(tmp_path / "virtual"),
        ]
    ) == 0
    with (tmp_path / "virtual" / "virtual_control.csv").open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert rows
    assert rows[0]["virtual_control_generated"] == "True"
    assert rows[0]["virtual_heading_status"] == "OK"
    assert float(rows[0]["virtual_forward_cmd"]) <= 0.10
    assert abs(float(rows[0]["virtual_turn_cmd"])) <= 0.05
    assert rows[0]["motor_command_generated"] == "False"
    assert rows[0]["ready_for_motor_test"] == "False"
    assert (tmp_path / "virtual" / "preview_virtual_control.png").exists()


def test_virtual_controller_replay_log_with_georef_produces_rows(tmp_path: Path) -> None:
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
    package = field_ab_to_serpentine.build_path_package(
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
    package_path = tmp_path / "path_package.json"
    package_path.write_text(json.dumps(package), encoding="utf-8")
    log = tmp_path / "usbdbg.log"
    log.write_text("current_lat=35.0 current_lon=129.0 active_target_source=compile_time\n", encoding="utf-8")
    assert station_virtual_path_controller.main(
        [
            "--path-package",
            str(package_path),
            "--mode",
            "replay_log",
            "--log",
            str(log),
            "--out-dir",
            str(tmp_path / "virtual"),
        ]
    ) == 0
    with (tmp_path / "virtual" / "virtual_control.csv").open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert rows[0]["firmware_active_target_source"] == "compile_time"
    assert rows[0]["virtual_control_generated"] == "True"
    assert rows[0]["motor_command_generated"] == "False"
    assert rows[0]["physical_output_active"] == "False"
    summary = (tmp_path / "virtual" / "summary.md").read_text(encoding="utf-8")
    assert "ready_for_station_virtual_control_preview: `True`" in summary
    assert "ready_for_motor_test: `False`" in summary


def test_virtual_controller_heading_missing_is_diag_only_not_motor_ready() -> None:
    row = {
        "local_pose_available": True,
        "target_distance_m": 2.0,
        "target_bearing_deg": 10.0,
        "cross_track_error_m": 0.2,
        "current_heading_deg": "",
        "heading_error_deg": "NA_DIAG_ONLY",
        "motor_command_generated": False,
        "physical_output_active": False,
    }
    control = station_virtual_path_controller.compute_virtual_control(row)
    assert control["virtual_control_generated"] is True
    assert control["virtual_heading_status"] == "DIAG_ONLY"
    assert control["ready_for_station_virtual_control_preview"] is True
    assert control["ready_for_motor_test"] is False
    assert control["motor_command_generated"] is False


def test_virtual_controller_live_parser_rows_are_diagnostic_without_serial() -> None:
    package = field_ab_to_serpentine.build_path_package(
        raw_a=(8.0, 0.0),
        raw_b=(0.0, 1.2),
        current_pose=(8.0, 0.0, 0.0),
        step_spacing_m=0.25,
        tool_side="left",
        tool_lateral_offset_m=0.24,
        tool_width_m=0.30,
        tool_length_m=0.18,
        robot_width_m=0.18,
        robot_length_m=0.18,
    )
    target_rows = station_path_package_tracker.build_rows_from_replay(
        package,
        "current_x_m=0 current_y_m=1.2 active_target_source=compile_time\n",
        "live_usbdbg",
    )
    virtual_rows = station_virtual_path_controller.build_virtual_rows(target_rows)
    assert virtual_rows[0]["virtual_control_generated"] is True
    assert virtual_rows[0]["virtual_heading_status"] == "DIAG_ONLY"
    assert virtual_rows[0]["ready_for_motor_test"] is False
    assert virtual_rows[0]["motor_command_generated"] is False
