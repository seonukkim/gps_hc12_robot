from __future__ import annotations

from pathlib import Path

from tools import field_ab_to_serpentine
from tools import stage21_path_physical_ab_first_primitive as stage21


REPO = Path(__file__).resolve().parents[1]
SCRIPT = REPO / "legacy" / "stage_scripts" / "run_stage21_path_physical_ab_first_primitive.sh"


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


def test_stage21_script_uses_stage20_guarded_transport() -> None:
    text = SCRIPT.read_text(encoding="utf-8")
    assert "STAGE20_PHYSICAL_AB_GUARDED_CRAWL=1" in text
    assert "STAGE20_MAX_ABS_A=${MAX_ABS_A}" in text
    assert "STAGE20_MAX_ABS_B=${MAX_ABS_B}" in text
    assert "STAGE20_MAX_MS=${MAX_MS}" in text
    assert "max_abs_a=${MAX_ABS_A}" in text
    assert "max_ms=${MAX_MS}" in text
    assert "PHYSICAL_PATH_FOLLOWING_ENABLE=0" in text
    assert "PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0" in text
    assert "STAGE17_FIRST_PRIMITIVE_CRAWL=0" in text
    assert "STAGE18_MOTOR_MAPPING_PROBE=0" in text
    assert "NOT FULL PATH FOLLOWING" in text
    assert "--skip-upload|--no-upload" in text
    assert 'if [[ "$UPLOAD_ENABLED" == "true" ]]; then' in text
    assert "arduino-cli compile" in text
    assert "arduino-cli upload -p \"$PORT\"" in text
    assert "awk '/OpenRB-150/ {print $1; exit}'" in text
    assert "hc12_ports_ignored=${HC12_PORTS_IGNORED}" in text
    assert "/dev/tty.usbmodem" not in text


def test_stage21_applies_minimum_effective_a_b() -> None:
    virtual = {
        "virtual_forward_cmd": 0.02,
        "virtual_turn_cmd": 0.03,
    }
    a_cmd, b_cmd = stage21.physical_ab_from_virtual(
        virtual,
        forward_sign=1.0,
        turn_sign=1.0,
        min_effective_a=0.08,
        min_effective_b=0.08,
        max_abs_a=0.25,
        max_abs_b=0.25,
    )
    assert a_cmd == 0.08
    assert b_cmd == 0.08


def test_stage21_planned_command_uses_physical_ab() -> None:
    planned = stage21.planned_stage21_command(
        seq=7,
        virtual_row={"virtual_forward_cmd": 0.02, "virtual_turn_cmd": -0.01},
        pulse_ms=300,
        forward_sign=1.0,
        turn_sign=1.0,
        min_effective_a=0.08,
        min_effective_b=0.08,
        max_abs_a=0.25,
        max_abs_b=0.25,
    )
    assert planned["arm_command_text"] == "STAGE20_ARM seq=7"
    assert planned["stage20_command_text"] == "STAGE20_CMD seq=7 a=0.080 b=-0.080 ms=300"
    assert planned["stop_command_text"] == "STAGE20_STOP seq=7"


def test_stage21_stage20_ready_parser_requires_guarded_heartbeat() -> None:
    rows = stage21.station_path_package_tracker.parse_usbdbg_rows(
        "STAGE20 stage20_physical_ab_guarded_crawl=true event=HEARTBEAT "
        "stage20_firmware_ready=true physical_path_following_enable=false allow_motor_output=false\n"
    )
    ready, heartbeat = stage21.parse_stage20_ready(rows)
    assert ready is True
    assert heartbeat is True


def test_stage21_stage20_ready_parser_rejects_path_following_enabled() -> None:
    rows = stage21.station_path_package_tracker.parse_usbdbg_rows(
        "STAGE20 stage20_physical_ab_guarded_crawl=true event=HEARTBEAT "
        "stage20_firmware_ready=true physical_path_following_enable=true allow_motor_output=false\n"
    )
    ready, heartbeat = stage21.parse_stage20_ready(rows)
    assert ready is False
    assert heartbeat is True


def test_stage21_build_row_never_claims_full_path_readiness() -> None:
    package = _georef_package()
    log = (
        "STAGE20 stage20_physical_ab_guarded_crawl=true event=HEARTBEAT "
        "stage20_firmware_ready=true physical_output_active=false active_target_source=NONE\n"
    )
    row = stage21.build_stage21_row(
        package,
        log,
        dry_run=True,
        pose_mode="manual_local",
        current_x=0.0,
        current_y=0.0,
        current_heading_deg=0.0,
        pulse_ms=300,
        forward_sign=1.0,
        turn_sign=1.0,
        min_effective_a=0.08,
        min_effective_b=0.08,
        max_abs_a=0.25,
        max_abs_b=0.25,
    )
    assert row["path_package_loaded"] is True
    assert row["station_package_target_source"] == "path_package"
    assert row["local_pose_available"] is True
    assert row["stage20_command_text"].startswith("STAGE20_CMD seq=1 a=")
    assert row["ready_for_full_path_following"] is False


def test_stage21_calibrated_straight_only_emits_b_zero() -> None:
    calibration = {
        "forward_sign": 1.0,
        "turn_sign": 1.0,
        "forward_min_a_cmd": 0.16,
        "backward_min_a_cmd": -0.08,
        "max_forward_cmd": 0.20,
        "max_backward_cmd": 0.12,
        "ready_for_stage21_turning": False,
    }
    a_cmd, b_cmd = stage21.physical_ab_from_virtual(
        {"virtual_forward_cmd": 0.03, "virtual_turn_cmd": 0.20},
        calibration=calibration,
        straight_only=True,
        turn_disabled=True,
    )
    assert a_cmd == 0.16
    assert b_cmd == 0.0


def test_stage21_calibrated_backward_applies_negative_min() -> None:
    calibration = {
        "forward_sign": 1.0,
        "turn_sign": 1.0,
        "forward_min_a_cmd": 0.16,
        "backward_min_a_cmd": -0.08,
        "max_forward_cmd": 0.20,
        "max_backward_cmd": 0.12,
        "ready_for_stage21_turning": False,
    }
    a_cmd, b_cmd = stage21.physical_ab_from_virtual(
        {"virtual_forward_cmd": -0.02, "virtual_turn_cmd": 0.20},
        calibration=calibration,
        straight_only=True,
    )
    assert a_cmd == -0.08
    assert b_cmd == 0.0


def test_stage21_calibrated_forward_caps_command() -> None:
    calibration = {
        "forward_sign": 1.0,
        "turn_sign": 1.0,
        "forward_min_a_cmd": 0.16,
        "backward_min_a_cmd": -0.08,
        "max_forward_cmd": 0.20,
        "max_backward_cmd": 0.12,
        "ready_for_stage21_turning": False,
    }
    a_cmd, b_cmd = stage21.physical_ab_from_virtual(
        {"virtual_forward_cmd": 0.50, "virtual_turn_cmd": 0.20},
        calibration=calibration,
        straight_only=True,
    )
    assert a_cmd == 0.20
    assert b_cmd == 0.0


def test_stage21_planned_calibrated_straight_command() -> None:
    planned = stage21.planned_stage21_command(
        seq=9,
        virtual_row={"virtual_forward_cmd": 0.03, "virtual_turn_cmd": 0.20},
        pulse_ms=500,
        calibration={
            "forward_sign": 1.0,
            "turn_sign": 1.0,
            "forward_min_a_cmd": 0.16,
            "backward_min_a_cmd": -0.08,
            "max_forward_cmd": 0.20,
            "max_backward_cmd": 0.12,
            "ready_for_stage21_turning": False,
        },
        calibration_json="cal.json",
        straight_only=True,
        turn_disabled=True,
    )
    assert planned["stage20_command_text"] == "STAGE20_CMD seq=9 a=0.160 b=0.000 ms=500"
    assert planned["diagnostic_only_straight_primitive"] is True


def test_stage21_planned_command_a030_passes_with_firmware_cap_035() -> None:
    planned = stage21.planned_stage21_command(
        seq=13,
        virtual_row={"virtual_forward_cmd": 0.50, "virtual_turn_cmd": 0.20},
        pulse_ms=800,
        calibration={
            "forward_sign": 1.0,
            "turn_sign": 1.0,
            "forward_min_a_cmd": 0.25,
            "backward_min_a_cmd": -0.08,
            "ready_for_stage21_turning": False,
        },
        max_abs_a=0.35,
        max_abs_b=0.25,
        max_forward_cmd=0.30,
        max_ms=1000,
        straight_only=True,
        turn_disabled=True,
    )
    assert planned["stage20_command_text"] == "STAGE20_CMD seq=13 a=0.300 b=0.000 ms=800"
    assert planned["command_within_firmware_limits"] is True
    assert planned["stage21_command_blocked_reason"] == "NONE"


def test_stage21_planned_command_a030_blocks_with_firmware_cap_025() -> None:
    planned = stage21.planned_stage21_command(
        seq=14,
        virtual_row={"virtual_forward_cmd": 0.03, "virtual_turn_cmd": 0.20},
        pulse_ms=800,
        fine_calibration={
            "stage21_forward_a_cmd": 0.30,
            "stage21_forward_pulse_ms": 800,
            "turn_disabled": True,
            "ready_for_stage21_forward": True,
            "ready_for_stage21_backward": True,
        },
        max_abs_a=0.25,
        max_abs_b=0.25,
        max_ms=1000,
        straight_only=True,
        turn_disabled=True,
    )
    assert planned["stage20_command_text"] == "STAGE20_CMD seq=14 a=0.300 b=0.000 ms=800"
    assert planned["command_within_firmware_limits"] is False
    assert planned["stage21_command_blocked_reason"] == "COMMAND_EXCEEDS_FIRMWARE_LIMIT"


def test_stage21_planned_pulse_blocks_when_above_firmware_max_ms() -> None:
    planned = stage21.planned_stage21_command(
        seq=15,
        virtual_row={"virtual_forward_cmd": 0.03, "virtual_turn_cmd": 0.20},
        pulse_ms=1000,
        fine_calibration={
            "stage21_forward_a_cmd": 0.25,
            "stage21_forward_pulse_ms": 1000,
            "turn_disabled": True,
            "ready_for_stage21_forward": True,
            "ready_for_stage21_backward": True,
        },
        max_abs_a=0.35,
        max_abs_b=0.25,
        max_ms=800,
        straight_only=True,
        turn_disabled=True,
    )
    assert planned["command_within_firmware_limits"] is False
    assert planned["stage21_command_blocked_reason"] == "PULSE_EXCEEDS_FIRMWARE_LIMIT"


def test_stage21_uses_fine_calibration_forward_command() -> None:
    planned = stage21.planned_stage21_command(
        seq=10,
        virtual_row={"virtual_forward_cmd": 0.03, "virtual_turn_cmd": 0.20},
        pulse_ms=None,
        fine_calibration={
            "stage21_forward_a_cmd": 0.22,
            "stage21_forward_pulse_ms": 650,
            "stage21_backward_a_cmd": -0.08,
            "stage21_backward_pulse_ms": 300,
            "turn_disabled": True,
            "ready_for_stage21_forward": True,
            "ready_for_stage21_backward": True,
        },
        fine_calibration_json="fine.json",
        straight_only=True,
        turn_disabled=True,
    )
    assert planned["stage20_command_text"] == "STAGE20_CMD seq=10 a=0.220 b=0.000 ms=650"
    assert planned["b_forced_zero"] is True
    assert planned["stage21_command_blocked_reason"] == "NONE"


def test_stage21_blocks_forward_when_fine_calibration_unresolved() -> None:
    planned = stage21.planned_stage21_command(
        seq=11,
        virtual_row={"virtual_forward_cmd": 0.03, "virtual_turn_cmd": 0.20},
        pulse_ms=None,
        fine_calibration={
            "stage21_forward_a_cmd": None,
            "stage21_forward_pulse_ms": None,
            "stage21_backward_a_cmd": -0.08,
            "stage21_backward_pulse_ms": 300,
            "forward_threshold_unresolved": True,
            "turn_disabled": True,
            "ready_for_stage21_forward": False,
            "ready_for_stage21_backward": True,
        },
        fine_calibration_json="fine.json",
        straight_only=True,
        turn_disabled=True,
    )
    assert planned["stage20_command_text"] == "STAGE20_CMD seq=11 a=0.000 b=0.000 ms=300"
    assert planned["stage21_command_blocked_reason"] == "FORWARD_THRESHOLD_UNRESOLVED"
    assert planned["ready_for_stage21_forward"] is False
    assert planned["forward_threshold_unresolved"] is True


def test_stage21_explicit_pulse_ms_overrides_fine_calibration() -> None:
    planned = stage21.planned_stage21_command(
        seq=12,
        virtual_row={"virtual_forward_cmd": -0.03, "virtual_turn_cmd": 0.20},
        pulse_ms=450,
        fine_calibration={
            "stage21_backward_a_cmd": -0.08,
            "stage21_backward_pulse_ms": 300,
            "turn_disabled": True,
            "ready_for_stage21_backward": True,
        },
        straight_only=True,
        turn_disabled=True,
    )
    assert planned["stage20_command_text"] == "STAGE20_CMD seq=12 a=-0.080 b=0.000 ms=450"
