"""Dispatch + no-hardware-path tests for the unified CLI.

These exercise every mode that can run WITHOUT firmware or serial: ``preview``
(geometry only), ``run --print-plan`` / ``execute-plan --print-plan`` (build the
plan, never open serial), ``calibrate-turn --print-cmd`` (print the shell-out
argv instead of invoking it), and ``diagnose --from-log`` (parse a saved serial
log). The hardware paths (real serial / firmware upload) are intentionally NOT
exercised here -- the controller's own serial loop is covered by
``test_ppp_controller`` with a fake handle.
"""
from __future__ import annotations

import json
import subprocess
import sys
import types
from pathlib import Path

import pytest

from tools.physical_path_planning import cli

_HEARTBEAT = (
    "STAGE20 event=HEARTBEAT stage20_physical_ab_guarded_crawl=true rc_ok=true "
    "neutral_ok=true physical_output_active=false gps_block_reason=OK gps_sats=9 "
    "gps_hdop=1.20 imu_relative_yaw_deg=3.5"
)
_LOG = f"{_HEARTBEAT}\nSTAGE20 event=ACK\nSTAGE20 event=STOP final_left_cmd=0.000 final_right_cmd=0.000 physical_output_active=false\n"
_RC_ABSENT_LOG = (
    "USBDBG manual_rc_recovery=true mode=FAILSAFE rc_ok=false auto_sw=false "
    "steer_us=0 throttle_us=0 mode_us=0 raw_ch1_us=0 raw_ch2_us=0 raw_ch3_us=0 raw_ch4_us=0 "
    "raw_ch5_us=0 raw_ch6_us=0 raw_ch7_us=0 raw_ch8_us=0 control_source=STOP "
    "final_left_cmd=0 final_right_cmd=0 physical_output_active=false\n"
)

_PREVIEW_GOAL = [
    "--start-lat", "35.1",
    "--start-lon", "129.1",
    "--goal-mode", "bearing_distance",
    "--goal-bearing-deg", "90",
    "--goal-distance-m", "6",
]


def _assert_standard_summary(out_dir: Path) -> dict[str, object]:
    assert (out_dir / "summary.md").exists()
    assert (out_dir / "summary.json").exists()
    data = json.loads((out_dir / "summary.json").read_text())
    assert data["ready_for_full_path_following"] is False
    return data


# --- calibrate-turn argv (always --imu-angle-compare true) --------------------


def test_calibrate_turn_argv_always_enables_imu_angle_compare() -> None:
    argv = cli.build_calibrate_turn_argv(
        script="legacy/stage_scripts/run_stage20_physical_ab_probe.sh",
        port="/dev/ttyACM0",
        mode="turn_left",
        target_angle_deg=90.0,
        angle_tolerance_deg=10.0,
        save_turn_calibration="true",
        turn_calibration_out="out.json",
        out_dir="od",
    )
    assert argv[0] == "bash"
    assert argv[1].endswith("run_stage20_physical_ab_probe.sh")
    # The IMU flag is what makes the launcher compile the BMI160 yaw defines.
    assert argv[argv.index("--imu-angle-compare") + 1] == "true"
    assert argv[argv.index("--mode") + 1] == "turn_left"
    assert argv[argv.index("--save-turn-calibration") + 1] == "true"
    assert not any("stage36" in part.lower() for part in argv)


def test_calibrate_turn_argv_accepts_direction_command_values() -> None:
    argv = cli.build_calibrate_turn_argv(
        script="legacy/stage_scripts/run_stage20_physical_ab_probe.sh",
        port="/dev/cu.usbmodem212101",
        mode="turn_left",
        b_cmd=0.22,
        pulse_ms=1200,
        target_angle_deg=90.0,
        angle_tolerance_deg=10.0,
        save_turn_calibration="true",
        turn_calibration_out="out.json",
        out_dir="od",
    )
    assert argv[argv.index("--cmd-list") + 1] == "0.22"
    assert argv[argv.index("--pulse-ms-list") + 1] == "1200"
    assert argv[argv.index("--imu-angle-compare") + 1] == "true"


def test_calibrate_turn_print_cmd_does_not_invoke_subprocess(capsys: pytest.CaptureFixture[str], tmp_path: Path) -> None:
    rc = cli.main(
        [
            "calibrate-turn",
            "--print-cmd",
            "--direction",
            "right",
            "--b-cmd",
            "0.08",
            "--pulse-ms",
            "250",
            "--out-dir",
            str(tmp_path),
        ]
    )
    assert rc == 0
    out = capsys.readouterr().out
    assert "--imu-angle-compare true" in out
    assert "run_stage20_physical_ab_probe.sh" in out
    assert "turn_right" in out
    assert "--cmd-list 0.08" in out
    assert "--pulse-ms-list 250" in out
    data = _assert_standard_summary(tmp_path)
    assert data["mode"] == "calibrate-turn"
    assert data["reason"] == "COMMAND_PRINTED"


# --- port resolution and unified mode vocabulary -----------------------------


def test_resolve_port_explicit_wins(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(cli, "arduino_cli_openrb_port", lambda: "/dev/cu.usbmodem212101")
    resolved = cli.resolve_port("/dev/custom", env={"PORT": "/dev/env"}, system_name="Darwin")
    assert resolved == {"port": "/dev/custom", "source": "explicit"}


def test_resolve_port_env_used(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(cli, "arduino_cli_openrb_port", lambda: "/dev/cu.usbmodem212101")
    resolved = cli.resolve_port(None, env={"PORT": "/dev/env"}, system_name="Darwin")
    assert resolved == {"port": "/dev/env", "source": "env"}


def test_resolve_port_uses_arduino_openrb_on_macos(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(cli, "arduino_cli_openrb_port", lambda: "/dev/cu.usbmodem212101")
    resolved = cli.resolve_port(None, env={}, system_name="Darwin")
    assert resolved == {"port": "/dev/cu.usbmodem212101", "source": "arduino_cli"}


def test_resolve_port_macos_does_not_default_to_ttyacm0(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(cli, "arduino_cli_openrb_port", lambda: None)
    resolved = cli.resolve_port(None, env={}, system_name="Darwin")
    assert resolved == {"port": None, "source": "none"}


def test_missing_port_writes_summary_without_traceback(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    monkeypatch.setattr(cli, "arduino_cli_openrb_port", lambda: None)
    monkeypatch.setattr(cli.platform, "system", lambda: "Darwin")
    monkeypatch.setattr(cli, "detected_serial_ports", lambda: ["/dev/cu.usbserial-02442CA5"])
    rc = cli.main(["diagnose", "--out-dir", str(tmp_path)])
    assert rc == 2
    out = capsys.readouterr().out
    assert "SERIAL_PORT_NOT_FOUND" in out
    summary = json.loads((tmp_path / "summary.json").read_text())
    assert summary["reason"] == "SERIAL_PORT_NOT_FOUND"
    assert summary["detected_ports"] == ["/dev/cu.usbserial-02442CA5"]
    assert summary["ready_for_full_path_following"] is False


def test_help_uses_functional_mode_names() -> None:
    help_text = cli.build_parser().format_help()
    assert "rc-input-diagnose" in help_text
    assert "manual-rc" in help_text
    assert "station-hw-diagnose" in help_text
    assert "station-hw-manual" in help_text
    assert "usb-pulse-test" in help_text
    assert "station-drive" not in help_text
    assert "station-manual" not in help_text
    assert "guarded-pulse-ready" in help_text
    assert "calibrate-turn" in help_text
    assert "Stage20" not in help_text
    assert "Stage16" not in help_text
    assert "Stage35" not in help_text
    assert "Stage36" not in help_text
    assert "stage20-imu" not in help_text


def test_shell_help_uses_functional_mode_names() -> None:
    completed = subprocess.run(
        ["bash", "scripts/run_physical_path_planner.sh", "--help"],
        check=False,
        capture_output=True,
        text=True,
    )
    assert completed.returncode == 0
    help_text = completed.stdout
    assert "rc-input-diagnose" in help_text
    assert "manual-rc" in help_text
    assert "station-hw-diagnose" in help_text
    assert "station-hw-manual" in help_text
    assert "usb-pulse-test" in help_text
    assert "station-drive" not in help_text
    assert "station-manual" not in help_text
    assert "guarded-pulse-ready" in help_text
    assert "Stage20" not in help_text
    assert "Stage16" not in help_text
    assert "Stage35" not in help_text
    assert "Stage36" not in help_text


def test_shell_station_hw_subcommand_help_is_no_hardware() -> None:
    for mode in ("station-hw-diagnose", "station-hw-manual"):
        completed = subprocess.run(
            ["bash", "scripts/run_physical_path_planner.sh", mode, "--help"],
            check=False,
            capture_output=True,
            text=True,
        )
        assert completed.returncode == 0
        assert f"scripts/run_physical_path_planner.sh {mode}" in completed.stdout
        assert "resolved_port=" not in completed.stdout
        assert "Stage20" not in completed.stdout
        assert "Stage16" not in completed.stdout


def test_guarded_pulse_ready_print_cmd_contains_imu_flags(
    capsys: pytest.CaptureFixture[str], tmp_path: Path
) -> None:
    rc = cli.main(["guarded-pulse-ready", "--print-cmd", "--out-dir", str(tmp_path)])
    assert rc == 0
    out = capsys.readouterr().out
    assert "IMU_ENABLE=1" in out
    assert "IMU_YAW_DIAG=1" in out
    assert "ready_for_full_path_following=false" in out
    data = _assert_standard_summary(tmp_path)
    assert data["mode"] == "guarded-pulse-ready"


def test_usb_pulse_test_print_command_uses_exact_bounded_ab_mapping(
    capsys: pytest.CaptureFixture[str], tmp_path: Path
) -> None:
    rc = cli.main(["usb-pulse-test", "--print-command", "true", "--out-dir", str(tmp_path)])
    assert rc == 0
    out = capsys.readouterr().out
    assert out == (
        "FORWARD:\n"
        "A=+0.300 B=+0.000 ms=800\n\n"
        "BACKWARD:\n"
        "A=-0.080 B=+0.000 ms=300\n\n"
        "LEFT:\n"
        "A=+0.000 B=+0.260 ms=700\n\n"
        "RIGHT:\n"
        "A=+0.000 B=-0.080 ms=250\n"
    )
    data = _assert_standard_summary(tmp_path)
    assert data["mode"] == "usb-pulse-test"
    assert data["rc_input_required"] is False
    assert data["rc_input_ignored"] is True
    assert "stage20" not in json.dumps(data).lower()


def test_usb_pulse_test_raw_print_cmd_uses_usb_pulse_protocol(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    rc = cli.main(["usb-pulse-test", "--print-cmd", "--out-dir", str(tmp_path)])
    assert rc == 0
    out = capsys.readouterr().out
    assert "USB_PULSE_TEST_ARM seq=1" in out
    assert "USB_PULSE_TEST_CMD seq=1 a=0.300 b=0.000 ms=800" in out
    assert "USB_PULSE_TEST_STOP seq=1" in out
    assert "STAGE20" not in out


def test_usb_pulse_test_plan_preserves_physical_ab_mapping() -> None:
    plan = cli.usb_pulse_test_plan()
    by_name = {str(item["primitive"]): item for item in plan}
    assert by_name["forward"]["a"] == pytest.approx(0.30)
    assert by_name["forward"]["b"] == pytest.approx(0.0)
    assert by_name["forward"]["ms"] == 800
    assert by_name["backward"]["a"] == pytest.approx(-0.08)
    assert by_name["backward"]["b"] == pytest.approx(0.0)
    assert by_name["backward"]["ms"] == 300
    assert by_name["left"]["a"] == pytest.approx(0.0)
    assert by_name["left"]["b"] == pytest.approx(0.26)
    assert by_name["left"]["ms"] == 700
    assert by_name["right"]["a"] == pytest.approx(0.0)
    assert by_name["right"]["b"] == pytest.approx(-0.08)
    assert by_name["right"]["ms"] == 250


def test_usb_pulse_test_single_and_sequence_aliases() -> None:
    assert [item["primitive"] for item in cli.usb_pulse_test_plan(single="turn_left")] == ["left"]
    assert [item["primitive"] for item in cli.usb_pulse_test_plan(sequence="forward,left,right")] == [
        "forward",
        "left",
        "right",
    ]


def test_usb_pulse_test_default_does_not_require_rc_input() -> None:
    parser = cli.build_parser()
    args = parser.parse_args(["usb-pulse-test"])
    assert args.require_rc_input == "false"
    assert args.max_ms == 1000


def test_usb_pulse_test_firmware_flags_include_rc_bypass() -> None:
    flags = cli.usb_pulse_test_firmware_flags(max_ms=1000)
    assert "USB_PULSE_TEST_GUARDED=1" in flags
    assert "USB_PULSE_TEST_IGNORE_RC_INPUT=1" in flags
    assert "PHYSICAL_PATH_FOLLOWING_ENABLE=0" in flags
    assert "PATH_FOLLOWING_HC12_ENABLED=0" in flags


def test_manual_rc_recovery_flags_pin_old_known_good_mapping() -> None:
    flags = cli.manual_rc_recovery_flags(mode_channel_index=4)
    assert "MANUAL_RC_RECOVERY=1" in flags
    assert "MANUAL_FORWARD_SIGN=-1" in flags
    assert "MANUAL_TURN_SIGN=1" in flags
    assert "MOTOR_OUTPUT_SWAP_LR=0" in flags
    assert "DRIVE_CALIBRATION_ENABLE=0" in flags
    assert "MODE_CHANNEL_INDEX=4" in flags
    assert "PHYSICAL_PATH_FOLLOWING_ENABLE=0" in flags
    assert "PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0" in flags
    assert "STAGE20_PHYSICAL_AB_GUARDED_CRAWL=0" in flags
    assert "IMU_ENABLE=1" not in flags


def test_usb_pulse_test_classifies_no_command_sent_waiting_for_user() -> None:
    result, reason, action = cli.station_drive_classification([], user_aborted=True)
    assert result == "WAITING_FOR_USER_ENTER"
    assert reason == "WAITING_FOR_USER_ENTER"
    assert "Press Enter" in action


def test_usb_pulse_test_classifies_telemetry_motion_but_user_none() -> None:
    rows = [
        {
            "command_sent": True,
            "valid_pulse": True,
            "skipped": False,
            "ack_seen": True,
            "active_seen": True,
            "stop_seen": True,
            "reject_seen": False,
            "motor_write_called_seen": True,
            "physical_output_active_seen": True,
            "final_zero": True,
            "user_motion_report": "none",
        }
    ]
    result, reason, _ = cli.station_drive_classification(rows)
    assert result == "TELEMETRY_OUTPUT_ACTIVE_BUT_USER_SAW_NONE"
    assert reason == "TELEMETRY_OUTPUT_ACTIVE_BUT_USER_SAW_NONE"


def test_usb_pulse_test_summary_names_are_clean() -> None:
    rows = [
        {
            "command_sent": True,
            "valid_pulse": True,
            "skipped": False,
            "ack_seen": True,
            "active_seen": True,
            "stop_seen": True,
            "reject_seen": False,
            "motor_write_called_seen": True,
            "physical_output_active_seen": True,
            "final_zero": True,
            "user_motion_report": "forward",
        }
    ]
    result, reason, _ = cli.station_drive_classification(rows)
    assert result == "USB_PULSE_TEST_PASS"
    assert "Stage" not in reason


def test_station_drive_alias_warns_and_maps_to_usb_pulse_test(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    rc = cli.main(["station-drive", "--print-command", "true", "--out-dir", str(tmp_path)])
    assert rc == 0
    out = capsys.readouterr().out
    assert "Deprecated alias: use usb-pulse-test." in out
    data = _assert_standard_summary(tmp_path)
    assert data["mode"] == "usb-pulse-test"


def test_station_manual_alias_warns_and_maps_to_usb_pulse_test(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    rc = cli.main(["station-manual", "--print-command", "true", "--out-dir", str(tmp_path)])
    assert rc == 0
    out = capsys.readouterr().out
    assert "Deprecated alias: use usb-pulse-test." in out
    data = _assert_standard_summary(tmp_path)
    assert data["mode"] == "usb-pulse-test"


def test_station_hw_diagnose_classifies_absent_link() -> None:
    rows = [
        {
            "station_age_ms": "NA",
            "station_seq": "NA",
            "station_manual_valid": "false",
            "station_deadman": "false",
            "station_estop": "false",
        }
    ]
    summary = cli.evaluate_station_hw_rows(rows, mode="station-hw-diagnose")
    assert summary["reason"] == "STATION_HW_LINK_ABSENT"
    assert summary["station_link_seen"] is False
    assert summary["ready_for_full_path_following"] is False


def test_station_hw_diagnose_classifies_rx_count_parse_fail() -> None:
    rows = [
        {
            "hc12_rx_count": "5",
            "station_frame_count": "0",
            "station_parse_ok_count": "0",
            "station_parse_error_count": "5",
            "station_manual_valid": "false",
            "station_deadman": "false",
            "station_estop": "false",
        }
    ]
    summary = cli.evaluate_station_hw_rows(rows, mode="station-hw-diagnose")
    assert summary["reason"] == "STATION_HW_FRAMES_PRESENT_PARSE_FAIL"
    assert summary["station_link_seen"] is True
    assert summary["station_parse_error_count"] == 5


def test_station_hw_diagnose_classifies_deadman_false() -> None:
    rows = [
        {
            "station_seq": "7",
            "station_age_ms": "25",
            "station_manual_valid": "true",
            "station_deadman": "false",
            "station_estop": "false",
        }
    ]
    summary = cli.evaluate_station_hw_rows(rows, mode="station-hw-diagnose")
    assert summary["reason"] == "STATION_HW_DEADMAN_NOT_ACTIVE"


def test_station_hw_diagnose_classifies_estop_true() -> None:
    rows = [
        {
            "station_seq": "7",
            "station_age_ms": "25",
            "station_manual_valid": "true",
            "station_deadman": "true",
            "station_estop": "true",
        }
    ]
    summary = cli.evaluate_station_hw_rows(rows, mode="station-hw-diagnose")
    assert summary["reason"] == "STATION_HW_ESTOP_ACTIVE"


def test_station_hw_manual_maps_forward_and_turn_to_physical_ab() -> None:
    rows = [
        {
            "station_hw_manual_mode": "true",
            "station_link_seen": "true",
            "station_seq": "9",
            "station_age_ms": "30",
            "station_manual_valid": "true",
            "station_deadman": "true",
            "station_estop": "false",
            "station_forward_cmd": "0.300",
            "station_turn_cmd": "0.260",
            "station_physical_a_cmd": "0.300",
            "station_physical_b_cmd": "0.260",
            "control_source": "STATION_HW_MANUAL",
            "motor_write_called": "true",
            "physical_output_active": "true",
            "final_left_cmd": "0.100",
            "final_right_cmd": "0.200",
        }
    ]
    summary = cli.evaluate_station_hw_rows(rows, mode="station-hw-manual")
    assert summary["reason"] == "STATION_HW_MANUAL_PASS"
    assert summary["station_physical_a_nonzero_seen"] is True
    assert summary["station_physical_b_nonzero_seen"] is True
    assert summary["rc_input_required"] is False


def test_station_hw_manual_pass_does_not_require_gps_imu_or_rc() -> None:
    rows = [
        {
            "station_hw_manual_mode": "true",
            "station_link_seen": "true",
            "station_seq": "11",
            "station_age_ms": "20",
            "station_manual_valid": "true",
            "station_deadman": "true",
            "station_estop": "false",
            "station_forward_cmd": "0.300",
            "station_turn_cmd": "0.260",
            "station_physical_a_cmd": "0.300",
            "station_physical_b_cmd": "0.260",
            "control_source": "STATION_HW_MANUAL",
            "motor_write_called": "true",
            "physical_output_active": "true",
            "final_left_cmd": "0.100",
            "final_right_cmd": "0.200",
            "rc_ok": "false",
            "gps_block_reason": "NO_LOCATION",
            "imu_present": "false",
        }
    ]
    summary = cli.evaluate_station_hw_rows(rows, mode="station-hw-manual")
    assert summary["reason"] == "STATION_HW_MANUAL_PASS"
    assert summary["success"] is True
    assert summary["rc_input_required"] is False
    assert summary["gps_required"] is False
    assert summary["imu_required"] is False


def test_station_hw_manual_classifies_valid_ab_without_motor_as_output_blocked() -> None:
    rows = [
        {
            "station_hw_manual_mode": "true",
            "station_link_seen": "true",
            "station_seq": "9",
            "station_age_ms": "30",
            "station_manual_valid": "true",
            "station_deadman": "true",
            "station_estop": "false",
            "station_forward_cmd": "0.300",
            "station_turn_cmd": "0.260",
            "station_physical_a_cmd": "0.300",
            "station_physical_b_cmd": "0.260",
            "control_source": "STATION_HW_MANUAL",
            "motor_write_called": "false",
            "physical_output_active": "false",
            "final_left_cmd": "0.000",
            "final_right_cmd": "0.000",
        }
    ]
    manual_summary = cli.evaluate_station_hw_rows(rows, mode="station-hw-manual")
    diagnose_summary = cli.evaluate_station_hw_rows(rows, mode="station-hw-diagnose")
    assert manual_summary["reason"] == "STATION_HW_MANUAL_OUTPUT_BLOCKED"
    assert manual_summary["success"] is False
    assert diagnose_summary["reason"] == "STATION_HW_MANUAL_READY"
    assert diagnose_summary["success"] is True


def test_station_hw_status_line_contains_pipeline_fields() -> None:
    rows = [
        {
            "hc12_rx_count": "3",
            "station_seq": "9",
            "station_age_ms": "25",
            "station_manual_valid": "true",
            "station_deadman": "true",
            "station_estop": "false",
            "station_physical_a_cmd": "0.300",
            "station_physical_b_cmd": "0.260",
        }
    ]
    line = cli.station_hw_status_line(rows, mode="station-hw-diagnose", elapsed_s=4.2)
    assert "elapsed_s=4" in line
    assert "station_link_seen=true" in line
    assert "station_frame_count=3" in line
    assert "station_deadman=true" in line
    assert "station_physical_a_cmd=0.300" in line
    assert "station_physical_b_cmd=0.260" in line
    assert "reason_so_far=STATION_HW_MANUAL_READY" in line


def test_station_hw_firmware_flags_enable_ab_mapping_and_hc12() -> None:
    flags = cli.station_hw_manual_firmware_flags()
    assert "STATION_HW_MANUAL_ENABLE=1" in flags
    assert "STATION_HW_MANUAL_A_B_MAPPING=1" in flags
    assert "STATION_HW_MANUAL_IGNORE_RC_INPUT=1" in flags
    assert "PATH_FOLLOWING_HC12_ENABLED=1" in flags


def test_station_hw_diagnose_print_command_writes_summary(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    rc = cli.main(["station-hw-diagnose", "--print-command", "true", "--out-dir", str(tmp_path)])
    assert rc == 0
    out = capsys.readouterr().out
    assert "STATION HARDWARE DIAGNOSE" in out
    summary = _assert_standard_summary(tmp_path)
    assert summary["mode"] == "station-hw-diagnose"


def test_station_hw_manual_default_duration_is_continuous() -> None:
    parser = cli.build_parser()
    manual_args = parser.parse_args(["station-hw-manual"])
    diagnose_args = parser.parse_args(["station-hw-diagnose"])
    assert manual_args.duration_s == 0.0
    assert diagnose_args.duration_s > 0.0


def test_station_hw_firmware_loop_stops_on_stale_deadman_or_estop() -> None:
    source = Path("firmware/openrb_robot_controller/openrb_robot_controller.ino").read_text()
    assert "bool stationManualActive = stationManualFresh && stationManual.deadman && !stationEstop;" in source
    assert "if (stationEstop) {" in source
    assert "} else if (stationManualActive) {" in source
    assert "currentControlSource = CONTROL_SOURCE_STATION_MANUAL;" in source
    assert "applyStationManualCommand();" in source


def test_station_hw_monitor_ctrl_c_writes_summary_without_traceback(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    class FakeSerialHandle:
        def __enter__(self) -> "FakeSerialHandle":
            return self

        def __exit__(self, *exc: object) -> None:
            return None

        def readline(self) -> bytes:
            raise KeyboardInterrupt

    fake_serial = types.SimpleNamespace(
        Serial=lambda *args, **kwargs: FakeSerialHandle(),
        serialutil=types.SimpleNamespace(SerialException=Exception),
    )
    monkeypatch.setitem(sys.modules, "serial", fake_serial)
    monkeypatch.setattr(cli, "ensure_port", lambda args: True)
    args = types.SimpleNamespace(
        out_dir=str(tmp_path),
        from_log=None,
        port="/dev/fake",
        baud=115200,
        duration_s=0.0,
        upload="false",
        verbose_raw="false",
    )
    rc = cli._read_station_hw_rows(
        args,
        title="Station Hardware Diagnose",
        csv_name="station_hw_diagnose.csv",
        summary_name="station_hw_diagnose_summary.json",
        mode="station-hw-diagnose",
    )
    out = capsys.readouterr().out
    assert rc == 130
    assert "User aborted station hardware monitor" in out
    assert "Traceback" not in out
    summary = _assert_standard_summary(tmp_path)
    assert summary["reason"] == "USER_ABORTED"
    assert summary["station_hw_result"] == "USER_ABORTED"
    assert (tmp_path / "station_hw_diagnose.csv").exists()
    assert (tmp_path / "raw_usbdbg.log").exists()


def test_docs_quickstart_uses_unified_launcher_only() -> None:
    readme = Path("README.md").read_text()
    ppp_readme = Path("docs/README_physical_path_planning.md").read_text()
    field_manual = Path("docs/field_test_manual.md").read_text()
    for text in (readme, ppp_readme, field_manual):
        assert "scripts/run_physical_path_planner.sh" in text
        assert "legacy/stage_scripts/run_stage" not in text
    assert "RC_INPUT_ABSENT" in ppp_readme
    assert "RC_INPUT_ABSENT" in field_manual


# --- preview (no serial, no motion) ------------------------------------------


def test_preview_writes_guarded_summary(tmp_path: Path) -> None:
    rc = cli.main(
        ["preview", *_PREVIEW_GOAL, "--workspace-width-m", "2", "--no-png", "--out-dir", str(tmp_path)]
    )
    assert rc == 0
    data = json.loads((tmp_path / "preview_summary.json").read_text())
    assert data["ready_for_full_path_following"] is False
    assert data["segment_count"] >= 1
    assert data["lane_count"] >= 1
    assert data["path_shape"] == "diagonal_rectangle_serpentine"
    summary = _assert_standard_summary(tmp_path)
    assert summary["mode"] == "preview"


def test_preview_direct_line_needs_no_width(tmp_path: Path) -> None:
    rc = cli.main(
        ["preview", *_PREVIEW_GOAL, "--path-shape", "direct_line", "--no-png", "--out-dir", str(tmp_path)]
    )
    assert rc == 0
    data = json.loads((tmp_path / "preview_summary.json").read_text())
    assert data["path_shape"] == "direct_line"


def test_preview_diagonal_without_width_fails_cleanly(tmp_path: Path) -> None:
    # A->B is the diagonal, so a serpentine plan requires a workspace width.
    rc = cli.main(["preview", *_PREVIEW_GOAL, "--no-png", "--out-dir", str(tmp_path)])
    assert rc == 2
    assert not (tmp_path / "preview_summary.json").exists()
    summary = _assert_standard_summary(tmp_path)
    assert summary["reason"] == "PLAN_INPUT_INVALID"


# --- run / execute-plan: --print-plan stays no-serial -------------------------


def test_run_print_plan_builds_without_opening_serial(tmp_path: Path) -> None:
    rc = cli.main(
        ["run", *_PREVIEW_GOAL, "--workspace-width-m", "2", "--print-plan", "--out-dir", str(tmp_path)]
    )
    assert rc == 0
    plan = json.loads((tmp_path / "plan.json").read_text())
    assert plan["ready_for_full_path_following"] is False
    assert plan["segment_count"] >= 1
    summary = _assert_standard_summary(tmp_path)
    assert summary["reason"] == "PLAN_PRINTED"


def test_execute_plan_is_an_alias_of_run(tmp_path: Path) -> None:
    rc = cli.main(
        ["execute-plan", *_PREVIEW_GOAL, "--workspace-width-m", "2", "--print-plan", "--out-dir", str(tmp_path)]
    )
    assert rc == 0
    assert (tmp_path / "plan.json").exists()


def test_execute_plan_can_load_existing_plan_dir_without_start_coords(tmp_path: Path) -> None:
    plan_dir = tmp_path / "preview"
    rc = cli.main(
        ["preview", *_PREVIEW_GOAL, "--workspace-width-m", "2", "--no-png", "--out-dir", str(plan_dir)]
    )
    assert rc == 0
    out_dir = tmp_path / "execute"
    rc = cli.main(
        ["execute-plan", "--plan-dir", str(plan_dir), "--print-plan", "--out-dir", str(out_dir)]
    )
    assert rc == 0
    assert (out_dir / "plan.json").exists()


# --- diagnose --from-log (no serial) -----------------------------------------


def test_diagnose_from_log_summarizes_telemetry(tmp_path: Path) -> None:
    log = tmp_path / "serial.log"
    log.write_text(_LOG)
    rc = cli.main(["diagnose", "--from-log", str(log), "--out-dir", str(tmp_path)])
    assert rc == 0
    data = json.loads((tmp_path / "diagnose_summary.json").read_text())
    assert data["row_count"] == 3
    assert data["heartbeat_count"] == 1
    assert data["last_gps_block_reason"] == "OK"
    assert data["ready_for_full_path_following"] is False
    summary = _assert_standard_summary(tmp_path)
    assert summary["mode"] == "diagnose"


def test_manual_rc_print_cmd_writes_standard_summary(tmp_path: Path) -> None:
    rc = cli.main(["manual-rc", "--print-cmd", "--out-dir", str(tmp_path)])
    assert rc == 0
    data = _assert_standard_summary(tmp_path)
    assert data["mode"] == "manual-rc"
    assert data["rc_input_mode_requested"] == "old_known_good"
    assert data["rc_input_mode_effective"] == "ppm_old_known_good"
    assert data["old_known_good_rc_path"] is True


def test_rc_input_diagnose_absent_from_log(tmp_path: Path) -> None:
    log = tmp_path / "ppm.log"
    log.write_text(
        "PPMHDR ppm_channel_map_probe - motors/GPS/HC12 disabled, read-only\n"
        "PPMSUM timestamp_ms=1000 frame_counter=0 frame_age_ms=999999 frames=0 invalid_frames=0 "
        "CH1_min=0 CH1_max=0 CH1_low=0 CH1_mid=0 CH1_high=0\n",
        encoding="utf-8",
    )
    rc = cli.main(["rc-input-diagnose", "--from-log", str(log), "--out-dir", str(tmp_path)])
    assert rc == 2
    data = _assert_standard_summary(tmp_path)
    assert data["reason"] == "RC_INPUT_ABSENT"
    assert data["rc_input_detected"] is False


def test_rc_input_diagnose_valid_ppm_from_log(tmp_path: Path) -> None:
    log = tmp_path / "ppm.log"
    log.write_text(
        "PPMSUM timestamp_ms=1000 frame_counter=40 frame_age_ms=2 frames=40 invalid_frames=0 "
        "CH1_min=1490 CH1_max=1510 CH1_low=0 CH1_mid=40 CH1_high=0 "
        "CH2_min=1490 CH2_max=1510 CH2_low=0 CH2_mid=40 CH2_high=0 "
        "CH3_min=1490 CH3_max=1510 CH3_low=0 CH3_mid=40 CH3_high=0 "
        "CH4_min=1490 CH4_max=1510 CH4_low=0 CH4_mid=40 CH4_high=0 "
        "CH5_min=1000 CH5_max=2000 CH5_low=20 CH5_mid=0 CH5_high=20\n",
        encoding="utf-8",
    )
    rc = cli.main(["rc-input-diagnose", "--from-log", str(log), "--out-dir", str(tmp_path)])
    assert rc == 0
    data = _assert_standard_summary(tmp_path)
    assert data["reason"] == "RC_CHANNELS_PRESENT_AND_VALID"
    assert data["rc_input_signal_class"] == "RC_INPUT_PRESENT_PPM"


def test_rc_input_diagnose_print_cmd_writes_summary(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    rc = cli.main(["rc-input-diagnose", "--print-cmd", "--out-dir", str(tmp_path)])
    assert rc == 0
    out = capsys.readouterr().out
    assert "ppm_channel_map_probe" in out
    data = _assert_standard_summary(tmp_path)
    assert data["mode"] == "rc-input-diagnose"


def test_manual_rc_upload_success_and_rc_absent_summary(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    calls: list[list[str]] = []

    def fake_run(argv: list[str], check: bool = False) -> subprocess.CompletedProcess[str]:
        calls.append(argv)
        if "run_manual_rc_passthrough_validation.sh" in argv[1]:
            summary = {
                "manual_rc_passthrough_ok": False,
                "validation_success": False,
                "reason": "RC_INPUT_ABSENT",
                "rc_input_detected": False,
                "ready_for_full_path_following": False,
            }
            (tmp_path / "summary.json").write_text(json.dumps(summary))
            return subprocess.CompletedProcess(argv, 2)
        return subprocess.CompletedProcess(argv, 0)

    monkeypatch.setattr(cli.subprocess, "run", fake_run)
    monkeypatch.setattr(cli.Path, "exists", lambda self: True)
    rc = cli.main(["manual-rc", "--port", "/dev/cu.usbmodem212101", "--out-dir", str(tmp_path)])
    assert rc == 2
    data = _assert_standard_summary(tmp_path)
    assert data["upload_success"] is True
    assert data["validation_success"] is False
    assert data["reason"] == "RC_INPUT_ABSENT"
    assert data["rc_input_detected"] is False
    assert any("upload_manual_rc_recovery_firmware.sh" in call[1] for call in calls)


def test_diagnose_summary_is_pure_and_guarded(tmp_path: Path) -> None:
    log = tmp_path / "serial.log"
    log.write_text(_LOG)
    rows = cli.load_rows_from_log(log)
    summary = cli.diagnose_summary(rows)
    assert summary["event_counts"]["HEARTBEAT"] == 1
    assert summary["event_counts"]["ACK"] == 1
    assert summary["last_imu_relative_yaw_deg"] == "3.500000"
    assert summary["ready_for_full_path_following"] is False
