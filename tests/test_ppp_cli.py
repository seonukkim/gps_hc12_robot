"""통합 CLI 디스패치 + 무(無)하드웨어 경로 계약 테스트.

목적/역할:
    펌웨어/시리얼 없이 실행 가능한 모든 모드를 잠근다: ``preview``(기하만),
    ``run --print-plan`` / ``execute-plan --print-plan``(계획만 만들고 시리얼을 열지
    않음), ``calibrate-turn --print-cmd``(셸 argv 를 실행 대신 출력),
    ``diagnose --from-log``(저장된 시리얼 로그 파싱). 목적은 인자 파싱, 모드 어휘,
    firmware-flag 조립, 요약(summary) 스키마, 표준 가드를 하드웨어 없이 고정하는 것.

시스템 내 위치:
    ``tools.physical_path_planning.cli`` 를 import 한다. 실제 시리얼/펌웨어 업로드
    경로는 의도적으로 다루지 않는다 -- 컨트롤러 자체의 시리얼 루프는 자매 파일
    ``test_ppp_controller`` 가 가짜 핸들로 담당한다. 셸 런처(run_physical_path_planner.sh)
    도움말 동등성은 subprocess 로 교차 검증한다.

핵심 개념·불변식:
  - 모드 어휘는 기능 이름(gps-wait, manual-control, usb-pulse-test 등)만 노출하고,
    옛 "Stage" 용어(Stage20/16/35/36...)는 도움말/요약 어디에도 남지 않아야 한다.
  - 물리 A/B 매핑은 정확한 경계값(bounded)이며 프리미티브별 a/b/ms 가 고정된다.
  - 모든 표준 요약은 ``ready_for_full_path_following=False`` 를 유지한다
    (헬퍼 ``_assert_standard_summary`` 로 검사).
  - 오류/중단(포트 없음, Ctrl-C, 시리얼 끊김)은 트레이스백 없이 요약 파일로 귀결된다.

------------------------------------------------------------------------------
Dispatch + no-hardware-path tests for the unified CLI.

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

# ── 공유 픽스처: 텔레메트리 로그 샘플 / Shared fixtures: telemetry log samples ──

_HEARTBEAT = (
    "USB_PULSE_TEST event=HEARTBEAT usb_pulse_test_mode=true rc_ok=true "
    "neutral_ok=true physical_output_active=false gps_block_reason=OK gps_sats=9 "
    "gps_hdop=1.20 imu_relative_yaw_deg=3.5"
)
_LOG = f"{_HEARTBEAT}\nUSB_PULSE_TEST event=ACK\nUSB_PULSE_TEST event=STOP final_left_cmd=0.000 final_right_cmd=0.000 physical_output_active=false\n"
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
    """표준 summary.md/json 존재 + ready=False 를 검사하고 파싱된 dict 반환.

    Assert the standard summary.md/json exist and stay guarded (ready False),
    returning the parsed JSON for further per-mode assertions.
    """
    assert (out_dir / "summary.md").exists()
    assert (out_dir / "summary.json").exists()
    data = json.loads((out_dir / "summary.json").read_text())
    assert data["ready_for_full_path_following"] is False
    return data


# ── calibrate-turn argv (항상 --imu-angle-compare true) / calibrate-turn argv ──


def test_calibrate_turn_argv_always_enables_imu_angle_compare() -> None:
    """calibrate-turn argv 는 항상 --imu-angle-compare true 를 포함(BMI160 yaw 정의 컴파일 트리거) / Always enables IMU angle-compare."""
    argv = cli.build_calibrate_turn_argv(
        script="legacy/stage_scripts/run_guarded_pulse_calibration.sh",
        port="/dev/ttyACM0",
        mode="turn_left",
        target_angle_deg=90.0,
        angle_tolerance_deg=10.0,
        save_turn_calibration="true",
        turn_calibration_out="out.json",
        out_dir="od",
    )
    assert argv[0] == "bash"
    assert argv[1].endswith("run_guarded_pulse_calibration.sh")
    # The IMU flag is what makes the launcher compile the BMI160 yaw defines.
    assert argv[argv.index("--imu-angle-compare") + 1] == "true"
    assert argv[argv.index("--mode") + 1] == "turn_left"
    assert argv[argv.index("--save-turn-calibration") + 1] == "true"
    assert not any("smooth_turn_connector" in part.lower() for part in argv)


def test_calibrate_turn_argv_accepts_direction_command_values() -> None:
    """b-cmd/pulse-ms 가 --cmd-list/--pulse-ms-list 로 전달됨 / Direction command values reach the argv lists."""
    argv = cli.build_calibrate_turn_argv(
        script="legacy/stage_scripts/run_guarded_pulse_calibration.sh",
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
    """--print-cmd 는 서브프로세스 미실행, argv 출력 + COMMAND_PRINTED 요약 / --print-cmd prints argv, runs nothing."""
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
    assert "run_guarded_pulse_calibration.sh" in out
    assert "turn_right" in out
    assert "--cmd-list 0.08" in out
    assert "--pulse-ms-list 250" in out
    data = _assert_standard_summary(tmp_path)
    assert data["mode"] == "calibrate-turn"
    assert data["reason"] == "COMMAND_PRINTED"


# ── 포트 해석 + 통합 모드 어휘 / Port resolution and unified mode vocabulary ──


def test_resolve_port_explicit_wins(monkeypatch: pytest.MonkeyPatch) -> None:
    """명시 --port 가 env/자동탐지를 이긴다 / Explicit port beats env and auto-detect."""
    monkeypatch.setattr(cli, "arduino_cli_openrb_port", lambda: "/dev/cu.usbmodem212101")
    resolved = cli.resolve_port("/dev/custom", env={"PORT": "/dev/env"}, system_name="Darwin")
    assert resolved == {"port": "/dev/custom", "source": "explicit"}


def test_resolve_port_env_used(monkeypatch: pytest.MonkeyPatch) -> None:
    """명시 포트 없으면 PORT 환경변수를 사용 / Falls back to the PORT env var when no explicit port."""
    monkeypatch.setattr(cli, "arduino_cli_openrb_port", lambda: "/dev/cu.usbmodem212101")
    resolved = cli.resolve_port(None, env={"PORT": "/dev/env"}, system_name="Darwin")
    assert resolved == {"port": "/dev/env", "source": "env"}


def test_resolve_port_uses_arduino_openrb_on_macos(monkeypatch: pytest.MonkeyPatch) -> None:
    """macOS 에서 arduino-cli OpenRB 포트를 자동 사용 / On macOS, auto-uses the arduino-cli OpenRB port."""
    monkeypatch.setattr(cli, "arduino_cli_openrb_port", lambda: "/dev/cu.usbmodem212101")
    resolved = cli.resolve_port(None, env={}, system_name="Darwin")
    assert resolved == {"port": "/dev/cu.usbmodem212101", "source": "arduino_cli"}


def test_resolve_port_macos_does_not_default_to_ttyacm0(monkeypatch: pytest.MonkeyPatch) -> None:
    """macOS 는 리눅스용 /dev/ttyACM0 으로 잘못 기본값을 잡지 않는다 / macOS never defaults to ttyACM0."""
    monkeypatch.setattr(cli, "arduino_cli_openrb_port", lambda: None)
    resolved = cli.resolve_port(None, env={}, system_name="Darwin")
    assert resolved == {"port": None, "source": "none"}


def test_missing_port_writes_summary_without_traceback(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """포트 미탐지 시 트레이스백 없이 SERIAL_PORT_NOT_FOUND 요약(탐지 목록 포함) / Missing port -> clean summary."""
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
    """CLI 도움말은 기능 모드 이름만 노출하고 옛 Stage/station-drive 용어는 없다 / Help uses functional names only."""
    help_text = cli.build_parser().format_help()
    assert "gps-wait" in help_text
    assert "rc-input-diagnose" in help_text
    assert "manual-rc" in help_text
    assert "manual-control" in help_text
    assert "station-hw-diagnose" in help_text
    assert "station-hw-manual" in help_text
    assert "usb-pulse-test" in help_text
    assert "station-drive" not in help_text
    assert "station-manual" not in help_text
    assert "guarded-pulse-ready" in help_text
    assert "calibrate-turn" in help_text
    assert "calibration-check" in help_text
    for old_term in ("Stage" + "20", "Stage" + "16", "Stage" + "35", "Stage" + "36", "stage" + "20-imu"):
        assert old_term not in help_text


def test_shell_help_uses_functional_mode_names() -> None:
    """셸 런처 --help 도 동일한 기능 모드 어휘를 노출(파이썬과 일관) / Shell launcher help mirrors the functional vocabulary."""
    completed = subprocess.run(
        ["bash", "scripts/run_physical_path_planner.sh", "--help"],
        check=False,
        capture_output=True,
        text=True,
    )
    assert completed.returncode == 0
    help_text = completed.stdout
    assert "gps-wait" in help_text
    assert "rc-input-diagnose" in help_text
    assert "manual-rc" in help_text
    assert "manual-control" in help_text
    assert "station-hw-diagnose" in help_text
    assert "station-hw-manual" in help_text
    assert "usb-pulse-test" in help_text
    assert "station-drive" not in help_text
    assert "station-manual" not in help_text
    assert "guarded-pulse-ready" in help_text
    assert "calibration-check" in help_text
    for old_term in ("Stage" + "20", "Stage" + "16", "Stage" + "35", "Stage" + "36"):
        assert old_term not in help_text


def test_shell_station_hw_subcommand_help_is_no_hardware() -> None:
    """station-hw 서브커맨드 --help 는 하드웨어를 열지 않고 사용법만 출력 / station-hw help opens no hardware."""
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
        for old_term in ("Stage" + "20", "Stage" + "16"):
            assert old_term not in completed.stdout


# ── guarded-pulse-ready · usb-pulse-test · usb-drive-live (펌웨어 플래그/프로토콜) ──


def test_guarded_pulse_ready_print_cmd_contains_imu_flags(
    capsys: pytest.CaptureFixture[str], tmp_path: Path
) -> None:
    """guarded-pulse-ready --print-cmd 는 IMU_ENABLE/IMU_YAW_DIAG 플래그 포함 / Prints IMU enable/diag flags."""
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
    """usb-pulse-test 사람용 명령 출력이 정확한 경계 A/B/ms(전/후/좌/우)를 찍는다 / Exact bounded A/B mapping."""
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
    assert ("stage" + "20") not in json.dumps(data).lower()


def test_usb_pulse_test_raw_print_cmd_uses_usb_pulse_protocol(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """raw --print-cmd 는 USB_PULSE_TEST ARM/CMD/STOP 와이어 프로토콜을 출력 / Raw print-cmd emits the wire protocol."""
    rc = cli.main(["usb-pulse-test", "--print-cmd", "--out-dir", str(tmp_path)])
    assert rc == 0
    out = capsys.readouterr().out
    assert "USB_PULSE_TEST_ARM seq=1" in out
    assert "USB_PULSE_TEST_CMD seq=1 a=0.300 b=0.000 ms=800" in out
    assert "USB_PULSE_TEST_STOP seq=1" in out
    assert "USB_PULSE_TEST" in out


def test_usb_pulse_test_plan_preserves_physical_ab_mapping() -> None:
    """usb_pulse_test_plan 이 프리미티브별 물리 a/b/ms 값을 정확히 보존 / Plan preserves per-primitive A/B/ms."""
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
    """single/sequence 별칭이 프리미티브 목록으로 정확히 확장 / single/sequence aliases expand to primitive lists."""
    assert [item["primitive"] for item in cli.usb_pulse_test_plan(single="turn_left")] == ["left"]
    assert [item["primitive"] for item in cli.usb_pulse_test_plan(sequence="forward,left,right")] == [
        "forward",
        "left",
        "right",
    ]


def test_usb_pulse_test_default_does_not_require_rc_input() -> None:
    """usb-pulse-test 기본은 RC 입력 불요, max_ms=1000 / Defaults require no RC input, max_ms=1000."""
    parser = cli.build_parser()
    args = parser.parse_args(["usb-pulse-test"])
    assert args.require_rc_input == "false"
    assert args.max_ms == 1000


def test_usb_pulse_test_firmware_flags_include_rc_bypass() -> None:
    """펌웨어 플래그가 RC 우회 + 경로추종 비활성(감독 펄스 전용) 포함 / Flags bypass RC and disable path-following."""
    flags = cli.usb_pulse_test_firmware_flags(max_ms=1000)
    assert "MAC_PHYSICAL_SUPERVISED=1" in flags
    assert "USB_PULSE_TEST_GUARDED=1" in flags
    assert "USB_PULSE_TEST_IGNORE_RC_INPUT=1" in flags
    assert "USB_DRIVE_LIVE_ENABLE=1" in flags
    assert "IMU_ENABLE=1" in flags
    assert "PHYSICAL_PATH_FOLLOWING_ENABLE=0" in flags
    assert "PATH_FOLLOWING_HC12_ENABLED=0" in flags


def test_usb_drive_live_print_command_uses_continuous_protocol(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """usb-drive-live 출력이 연속 SET(ttl 포함)/STOP 프로토콜을 찍는다 / Emits the continuous SET(ttl)/STOP protocol."""
    rc = cli.main(
        [
            "usb-drive-live",
            "--a",
            "0.30",
            "--b",
            "0.00",
            "--duration-s",
            "2",
            "--print-command",
            "true",
            "--out-dir",
            str(tmp_path),
        ]
    )
    assert rc == 0
    out = capsys.readouterr().out
    assert "USB_DRIVE_LIVE_SET seq=1 a=0.300 b=0.000 duration_ms=2000 ttl_ms=350" in out
    assert "USB_DRIVE_LIVE_STOP seq=1" in out
    data = _assert_standard_summary(tmp_path)
    assert data["mode"] == "usb-drive-live"


def test_usb_drive_live_summary_does_not_require_rc_ok() -> None:
    """usb-drive-live 요약은 rc_ok 없이도 성공 판정(감독 모드) / Summary succeeds without rc_ok in supervised mode."""
    rows = cli.telemetry.parse_usbdbg_rows(
        "USB_DRIVE_LIVE event=ACTIVE rc_ok=false neutral_ok=false physical_output_active=true\n"
        "USB_DRIVE_LIVE event=STOP rc_ok=false neutral_ok=false final_left_cmd=0.0 "
        "final_right_cmd=0.0 physical_output_active=false\n"
    )
    summary = cli.usb_drive_live_summary(rows, a_cmd=0.3, b_cmd=0.0, duration_s=1.0)
    assert summary["success"] is True
    assert summary["reason"] == "OK"


def test_usb_drive_live_firmware_flags_are_bounded_and_non_path_following() -> None:
    """플래그가 최대 지속시간 경계 + 경로추종 비활성(안전) 을 명시 / Flags bound max duration and disable path-following."""
    flags = cli.usb_drive_live_firmware_flags(max_duration_ms=3000, update_timeout_ms=350)
    assert "MAC_PHYSICAL_SUPERVISED=1" in flags
    assert "USB_PULSE_TEST_GUARDED=1" in flags
    assert "USB_DRIVE_LIVE_ENABLE=1" in flags
    assert "USB_DRIVE_LIVE_IGNORE_RC_INPUT=1" in flags
    assert "USB_DRIVE_LIVE_MAX_DURATION_MS=3000" in flags
    assert "IMU_ENABLE=1" in flags
    assert "PHYSICAL_PATH_FOLLOWING_ENABLE=0" in flags
    assert "PATH_FOLLOWING_HC12_ENABLED=0" in flags


# ── manual-rc / manual-control 펌웨어 플래그 + PPM 매핑 / Manual RC/control flags + PPM mapping ──


def test_manual_rc_recovery_flags_pin_old_known_good_mapping() -> None:
    """manual-rc 복구 플래그가 옛 known-good 부호/스왑 매핑을 고정(IMU 미사용) / Pins the old known-good mapping."""
    flags = cli.manual_rc_recovery_flags(mode_channel_index=4)
    assert "MANUAL_RC_RECOVERY=1" in flags
    assert "MANUAL_FORWARD_SIGN=-1" in flags
    assert "MANUAL_TURN_SIGN=1" in flags
    assert "MOTOR_OUTPUT_SWAP_LR=0" in flags
    assert "DRIVE_CALIBRATION_ENABLE=0" in flags
    assert "MODE_CHANNEL_INDEX=4" in flags
    assert "PHYSICAL_PATH_FOLLOWING_ENABLE=0" in flags
    assert "PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0" in flags
    assert "IMU_ENABLE=1" not in flags


def test_manual_control_firmware_flags_select_old_ppm_path() -> None:
    """old-working-ppm 프로파일이 옛 PPM 경로(RISING/3000us/min800, IMU off)를 선택 / Selects the old PPM decode path."""
    flags = cli.manual_control_firmware_flags(
        profile=cli.MANUAL_CONTROL_OLD_WORKING_PPM_PROFILE,
        mode_channel_index=4,
    )
    assert "MANUAL_CONTROL_PPM=1" in flags
    assert "MANUAL_RC_RECOVERY=1" not in flags
    assert "MODE_CHANNEL_INDEX=4" in flags
    assert "MANUAL_FORWARD_SIGN=-1" in flags
    assert "MANUAL_TURN_SIGN=1" in flags
    assert "MOTOR_OUTPUT_SWAP_LR=0" in flags
    assert "DRIVE_CALIBRATION_ENABLE=0" in flags
    assert "PPM_INTERRUPT_EDGE_FALLING=0" in flags
    assert "PPM_SYNC_THRESHOLD_US=3000" in flags
    assert "PPM_CAPTURE_MIN_US_VALUE=800" in flags
    assert "IMU_ENABLE=0" in flags
    assert "IMU_YAW_DIAG" not in flags
    assert "PATH_FOLLOWING_HC12_ENABLED=0" in flags
    assert cli.manual_control_build_path(cli.MANUAL_CONTROL_OLD_WORKING_PPM_PROFILE) == (
        "/private/tmp/openrb-manual-forward-neg-turn-pos"
    )
    assert "STAGE" not in flags


def test_manual_control_default_profile_uses_rc_mix_decoder() -> None:
    """기본 rc-mix-ppm 프로파일(FALLING/4000us/min0, IMU off)을 선택 / Default rc-mix-ppm profile decode settings."""
    flags = cli.manual_control_firmware_flags(mode_channel_index=4)
    assert cli.MANUAL_CONTROL_DEFAULT_PROFILE == "rc-mix-ppm"
    assert "MANUAL_FORWARD_SIGN=-1" in flags
    assert "MANUAL_TURN_SIGN=1" in flags
    assert "MOTOR_OUTPUT_SWAP_LR=0" in flags
    assert "DRIVE_CALIBRATION_ENABLE=0" in flags
    assert "PPM_INTERRUPT_EDGE_FALLING=1" in flags
    assert "PPM_SYNC_THRESHOLD_US=4000" in flags
    assert "PPM_CAPTURE_MIN_US_VALUE=0" in flags
    assert "MODE_CHANNEL_INDEX=4" in flags
    assert "MANUAL_CONTROL_PPM=1" in flags
    assert "IMU_ENABLE=0" in flags
    assert "IMU_YAW_DIAG" not in flags
    assert "PATH_FOLLOWING_HC12_ENABLED=0" in flags
    assert cli.manual_control_build_path(cli.MANUAL_CONTROL_RC_MIX_PPM_PROFILE) == (
        "/private/tmp/openrb-manual-rc-mix-ppm"
    )


def test_manual_control_full_telemetry_profile_disables_imu_like_known_good() -> None:
    """full-telemetry 프로파일도 known-good 처럼 IMU 를 끈다 / Full-telemetry profile keeps IMU off like known-good."""
    flags = cli.manual_control_firmware_flags(
        profile=cli.MANUAL_CONTROL_FULL_TELEMETRY_PPM_PROFILE,
        mode_channel_index=4,
    )
    assert "MANUAL_CONTROL_PPM=1" in flags
    assert "MODE_CHANNEL_INDEX=4" in flags
    assert "IMU_ENABLE=0" in flags
    assert "IMU_YAW_DIAG" not in flags
    assert "PATH_FOLLOWING_HC12_ENABLED=0" in flags


def test_integrated_ppm_decoder_matches_old_moving_controller_edge() -> None:
    """펌웨어 소스의 통합 PPM 디코더가 옛 이동 컨트롤러 엣지/임계 상수와 일치 / Firmware PPM decoder matches old edge/thresholds."""
    source = Path("firmware/openrb_robot_controller/openrb_robot_controller.ino").read_text()
    assert "#define PPM_SYNC_THRESHOLD_US 3000" in source
    assert "constexpr uint16_t PPM_SYNC_US = PPM_SYNC_THRESHOLD_US;" in source
    assert 'constexpr const char *PPM_INTERRUPT_EDGE_NAME = PPM_INTERRUPT_EDGE_FALLING_ENABLED ? "FALLING" : "RISING";' in source
    assert "attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmISR, FALLING);" in source
    assert "attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmISR, RISING);" in source
    assert "if (ppmIndex == 0 && width < PPM_CAPTURE_MIN_US)" in source
    assert 'ppmDecodeReason = "PPM_SYNC_ONLY_NO_CHANNELS";' in source
    assert "constexpr bool MOTOR_TRACE_ENABLED" in source


def test_manual_control_ppm_ch1_maps_to_physical_b() -> None:
    """PPM CH1(조향)이 물리 B 로 매핑(좌=+1, 우=-1) / PPM CH1 steer maps to physical B."""
    low_left = cli.manual_control_mapping(steer_norm=-1.0, throttle_norm=0.0)
    high_right = cli.manual_control_mapping(steer_norm=1.0, throttle_norm=0.0)
    assert low_left["physical_a_cmd"] == pytest.approx(0.0)
    assert low_left["physical_b_cmd"] == pytest.approx(1.0)
    assert high_right["physical_b_cmd"] == pytest.approx(-1.0)


def test_manual_control_ppm_ch2_maps_to_physical_a() -> None:
    """PPM CH2(스로틀)이 물리 A 로 매핑(전진=+1, 후진=-1) / PPM CH2 throttle maps to physical A."""
    throttle_low_forward = cli.manual_control_mapping(steer_norm=0.0, throttle_norm=-1.0)
    throttle_high_backward = cli.manual_control_mapping(steer_norm=0.0, throttle_norm=1.0)
    assert throttle_low_forward["physical_a_cmd"] == pytest.approx(1.0)
    assert throttle_low_forward["physical_b_cmd"] == pytest.approx(0.0)
    assert throttle_high_backward["physical_a_cmd"] == pytest.approx(-1.0)


# ── manual-control 텔레메트리 평가 + 상태 라인 / manual-control evaluation + status line ──


def test_manual_control_evaluates_old_known_good_telemetry_as_pass() -> None:
    """옛 known-good 텔레메트리 행을 PASS 로 평가(GPS/IMU/경로패키지 불요) / Known-good telemetry evaluates to PASS."""
    rows = [
        {
            "mode": "MANUAL",
            "control_source": "RC_MANUAL",
            "rc_ok": "true",
            "auto_sw": "false",
            "ppm_age_ms": "12",
            "steer_us": "1000",
            "throttle_us": "1309",
            "mode_us": "1000",
            "manual_forward_sign": "-1.0",
            "manual_turn_sign": "1.0",
            "old_angle_remap_active": "false",
            "control_source": "RC_MANUAL",
            "physical_a_cmd": "0.191",
            "physical_b_cmd": "0.809",
            "final_left_cmd": "-0.618",
            "final_right_cmd": "1.000",
            "motor_write_called": "true",
            "physical_output_active": "true",
            "physical_a_role": "throttle",
            "physical_b_role": "turn",
            "wheel_to_physical_mapping": "diff_to_throttle_turn",
            "gps_block_reason": "NO_LOCATION",
            "gps_chars": "0",
        }
    ]
    summary = cli.evaluate_manual_control_rows(rows)
    assert summary["manual_control_ok"] is True
    assert summary["reason"] == "MANUAL_CONTROL_PASS"
    assert summary["manual_switch"] == "MANUAL"
    assert summary["mode_us_latest"] == "1000"
    assert summary["gps_status_available"] is True
    assert summary["imu_status_available"] is False
    assert summary["last_current_lat"] == "NA"
    assert summary["last_current_lon"] == "NA"
    assert summary["last_imu_yaw"] == "NA"
    assert summary["control_source_rc_manual_seen"] is True
    assert summary["gps_required"] is False
    assert summary["imu_required"] is False
    assert summary["path_package_required"] is False


def test_manual_control_unstable_ppm_does_not_pass_from_one_good_row() -> None:
    """단 한 개의 좋은 행으로는 PASS 불가; 불안정 PPM 은 INVALID 로 분류 / One good row can't pass unstable PPM."""
    rows = [
        {
            "manual_control": "true",
            "manual_control_ppm": "true",
            "mode": "FAILSAFE",
            "control_source": "STOP",
            "rc_ok": "false",
            "rc_input_detected": "true",
            "steer_us": "180",
            "throttle_us": "1493",
            "mode_us": "1548",
            "ppm_decode_reason": "OK",
        }
        for _ in range(8)
    ]
    rows.append(
        {
            "manual_control": "true",
            "manual_control_ppm": "true",
            "mode": "MANUAL",
            "control_source": "RC_MANUAL",
            "rc_ok": "true",
            "rc_input_detected": "true",
            "steer_us": "1001",
            "throttle_us": "1001",
            "mode_us": "1001",
            "physical_a_cmd": "0.300",
            "physical_b_cmd": "0.260",
            "final_left_cmd": "0.040",
            "final_right_cmd": "0.560",
            "motor_write_called": "true",
            "physical_output_active": "true",
            "ppm_decode_reason": "OK",
        }
    )
    summary = cli.evaluate_manual_control_rows(rows)
    assert summary["manual_control_ok"] is False
    assert summary["reason"] == "PPM_CHANNELS_PRESENT_BUT_INVALID"
    assert summary["rc_ok_rows"] == 1
    assert summary["rc_bad_rows"] == 8
    assert summary["ppm_signal_stable"] is False


def test_manual_control_all_zero_ppm_reports_absent() -> None:
    """전(全) 0 PPM 은 PPM_INPUT_ABSENT + D6 핀 안내로 보고 / All-zero PPM reports absent with D6 hint."""
    summary = cli.evaluate_manual_control_rows(cli.telemetry.parse_usbdbg_rows(_RC_ABSENT_LOG))
    assert summary["reason"] == "PPM_INPUT_ABSENT"
    assert summary["manual_switch"] == "UNKNOWN_PPM_ABSENT"
    assert summary["rc_input_detected"] is False
    assert summary["ppm_input_pin"] == "D6"
    assert "CH1" in summary["next_recommended_action"]


def test_manual_control_no_frame_captured_does_not_report_missing_mode_channel() -> None:
    """프레임 미포착 시 '모드채널 없음'이 아니라 PPM_INPUT_ABSENT 로 정정 / No frame -> absent, not missing-mode-channel."""
    rows = [
        {
            "manual_control": "true",
            "manual_control_ppm": "true",
            "ppm_interrupt_edge": "RISING",
            "ppm_decode_reason": "NO_PPM_FRAME",
            "rc_input_detected": "true",
            "rc_ok": "false",
            "mode": "FAILSAFE",
            "auto_sw": "false",
            "manual_switch": "UNKNOWN_MODE_CHANNEL_MISSING",
            "mode_decode_reason": "NO_MODE_CHANNEL",
            "ppm_frame_count": "0",
            "ppm_last_channel_count": "0",
            "steer_us": "0",
            "throttle_us": "0",
            "mode_us": "0",
            "control_source": "STOP",
        }
    ]
    line = cli.format_manual_control_status(elapsed_s=1, rows=rows)
    summary = cli.evaluate_manual_control_rows(rows)
    assert summary["manual_control_ok"] is False
    assert summary["reason"] == "PPM_INPUT_ABSENT"
    assert summary["manual_switch"] == "UNKNOWN_PPM_ABSENT"
    assert summary["mode_decode_reason"] == "PPM_INPUT_ABSENT"
    assert "mode_decode_reason=PPM_INPUT_ABSENT" in line


def test_manual_control_sync_only_no_channels_is_precise_reason() -> None:
    """싱크만 있고 채널 없음은 PPM_SYNC_ONLY_NO_CHANNELS 로 정밀 분류 / Sync-only maps to its precise reason."""
    rows = [
        {
            "manual_control": "false",
            "manual_control_ppm": "true",
            "ppm_interrupt_edge": "FALLING",
            "ppm_decode_reason": "PPM_SYNC_ONLY_NO_CHANNELS",
            "ppm_edge_count": "62",
            "ppm_sync_count": "61",
            "ppm_last_width_us": "19980",
            "ppm_min_width_us": "19970",
            "ppm_max_width_us": "20010",
            "rc_input_detected": "false",
            "rc_ok": "false",
            "mode": "FAILSAFE",
            "manual_switch": "UNKNOWN_PPM_SYNC_ONLY",
            "mode_decode_reason": "PPM_SYNC_ONLY_NO_CHANNELS",
            "ppm_frame_count": "0",
            "ppm_last_channel_count": "0",
            "steer_us": "0",
            "throttle_us": "0",
            "mode_us": "0",
            "control_source": "STOP",
        }
    ]
    line = cli.format_manual_control_status(elapsed_s=3, rows=rows)
    summary = cli.evaluate_manual_control_rows(rows, expected_ppm_interrupt_edge="FALLING")
    assert summary["manual_control_ok"] is False
    assert summary["reason"] == "PPM_SYNC_ONLY_NO_CHANNELS"
    assert summary["manual_switch"] == "UNKNOWN_PPM_SYNC_ONLY"
    assert summary["ppm_sync_only_seen"] is True
    assert summary["rc_input_detected"] is False
    assert summary["ppm_last_width_us_latest"] == "19980"
    assert "ppm_sync_count=61" in line
    assert "mode_decode_reason=PPM_SYNC_ONLY_NO_CHANNELS" in line


def test_manual_control_mode_missing_requires_captured_mode_channel_slot() -> None:
    """채널은 잡혔지만 모드 슬롯이 0이면 MODE_CHANNEL_MISSING / Captured frame but empty mode slot -> mode-channel-missing."""
    rows = [
        {
            "manual_control": "true",
            "manual_control_ppm": "true",
            "ppm_interrupt_edge": "RISING",
            "ppm_decode_reason": "OK",
            "rc_input_detected": "true",
            "rc_ok": "false",
            "mode": "FAILSAFE",
            "ppm_frame_count": "3",
            "ppm_last_channel_count": "5",
            "steer_us": "1504",
            "throttle_us": "1500",
            "mode_us": "0",
            "control_source": "STOP",
        }
    ]
    summary = cli.evaluate_manual_control_rows(rows)
    assert summary["reason"] == "MODE_CHANNEL_MISSING"
    assert summary["manual_switch"] == "UNKNOWN_MODE_CHANNEL_MISSING"
    assert summary["mode_decode_reason"] == "NO_MODE_CHANNEL"


def test_manual_control_ppm_edge_mismatch_classifies_separately() -> None:
    """기대 엣지와 다른 엣지로 프레임 없음이면 PPM_EDGE_MISMATCH 로 분리 분류 / Edge mismatch classified separately."""
    rows = [
        {
            "manual_control": "true",
            "manual_control_ppm": "true",
            "ppm_interrupt_edge": "FALLING",
            "ppm_decode_reason": "NO_PPM_FRAME",
            "rc_input_detected": "false",
            "rc_ok": "false",
            "mode": "FAILSAFE",
            "ppm_frame_count": "0",
            "ppm_last_channel_count": "0",
            "steer_us": "0",
            "throttle_us": "0",
            "mode_us": "0",
            "control_source": "STOP",
        }
    ]
    summary = cli.evaluate_manual_control_rows(rows, expected_ppm_interrupt_edge="RISING")
    assert summary["reason"] == "PPM_EDGE_MISMATCH"
    assert summary["manual_switch"] == "UNKNOWN_PPM_EDGE_MISMATCH"
    assert summary["ppm_edge_mismatch_seen"] is True
    assert summary["expected_ppm_interrupt_edge"] == "RISING"


def test_manual_control_status_line_includes_full_telemetry_snapshot() -> None:
    """상태 라인이 RC/PPM/모터/GPS/IMU 전 필드 스냅샷을 포함 / Status line includes the full telemetry snapshot."""
    rows = [
        {
            "rc_input_detected": "true",
            "rc_ok": "true",
            "mode": "MANUAL",
            "auto_sw": "false",
            "ppm_interrupt_edge": "RISING",
            "ppm_decode_reason": "OK",
            "ppm_frame_count": "12",
            "ppm_last_channel_count": "8",
            "ppm_short_rejects": "0",
            "ppm_long_rejects": "0",
            "ppm_last_rejected_us": "0",
            "steer_us": "1001",
            "throttle_us": "1001",
            "mode_us": "1001",
            "physical_a_cmd": "0.300",
            "physical_b_cmd": "0.260",
            "control_source": "RC_MANUAL",
            "motor_write_called": "true",
            "physical_output_active": "true",
            "final_left_cmd": "0.040",
            "final_right_cmd": "0.560",
            "gps_block_reason": "OK",
            "current_lat": "35.5705010",
            "current_lon": "129.1872696",
            "gps_sats": "9",
            "gps_hdop": "1.20",
            "imu_present": "true",
            "imu_relative_yaw_deg": "12.5",
            "imu_heading_block_reason": "OK",
        }
    ]
    line = cli.format_manual_control_status(elapsed_s=7, rows=rows)
    for field in [
        "rc_input_detected=true",
        "rc_ok=true",
        "mode=MANUAL",
        "auto_sw=false",
        "manual_switch=MANUAL",
        "mode_decode_reason=MODE_CHANNEL_MANUAL",
        "ppm_interrupt_edge=RISING",
        "ppm_decode_reason=OK",
        "ppm_frame_count=12",
        "ppm_last_channel_count=8",
        "ppm_short_rejects=0",
        "ppm_long_rejects=0",
        "ppm_last_rejected_us=0",
        "steer_us=1001",
        "throttle_us=1001",
        "mode_us=1001",
        "physical_a_cmd=0.300",
        "physical_b_cmd=0.260",
        "control_source=RC_MANUAL",
        "motor_write_called=true",
        "physical_output_active=true",
        "final_left_cmd=0.040",
        "final_right_cmd=0.560",
        "gps_block_reason=OK",
        "current_lat=35.5705010",
        "current_lon=129.1872696",
        "gps_sats=9",
        "gps_hdop=1.20",
        "imu_present=true",
        "imu_relative_yaw_deg=12.5",
        "imu_heading_block_reason=OK",
    ]:
        assert field in line


def test_manual_control_status_line_uses_raw_mode_channel_fallback() -> None:
    """mode_us 없을 때 raw_mode_channel_us 로 폴백해 MANUAL 판정 / Falls back to raw mode-channel µs for the switch."""
    rows = [
        {
            "rc_input_detected": "true",
            "rc_ok": "true",
            "mode": "MANUAL",
            "auto_sw": "false",
            "steer_us": "1504",
            "throttle_us": "1500",
            "raw_mode_channel_us": "1001",
            "control_source": "RC_MANUAL",
        }
    ]
    line = cli.format_manual_control_status(elapsed_s=1, rows=rows)
    summary = cli.evaluate_manual_control_rows(rows)
    assert "manual_switch=MANUAL" in line
    assert "mode_decode_reason=MODE_CHANNEL_MANUAL" in line
    assert "mode_us=1001" in line
    assert summary["manual_switch"] == "MANUAL"
    assert summary["mode_us_latest"] == "1001"


def test_manual_control_short_ppm_pulses_classify_as_invalid_channels() -> None:
    """짧은 펄스로 유효 프레임 없음은 채널-무효(present-but-invalid)로 분류 / Short pulses classify as invalid channels."""
    rows = [
        {
            "manual_control": "true",
            "manual_control_ppm": "true",
            "ppm_interrupt_edge": "RISING",
            "ppm_decode_reason": "PPM_SHORT_PULSE_NO_VALID_FRAME",
            "ppm_short_rejects": "27",
            "ppm_last_rejected_us": "204",
            "rc_input_detected": "false",
            "rc_ok": "false",
            "mode": "FAILSAFE",
            "manual_switch": "UNKNOWN_PPM_INVALID",
            "mode_decode_reason": "PPM_SHORT_PULSE_NO_VALID_FRAME",
            "steer_us": "0",
            "throttle_us": "0",
            "mode_us": "0",
            "control_source": "STOP",
        }
    ]
    line = cli.format_manual_control_status(elapsed_s=4, rows=rows)
    summary = cli.evaluate_manual_control_rows(rows)
    assert "manual_switch=UNKNOWN_PPM_INVALID" in line
    assert "ppm_decode_reason=PPM_SHORT_PULSE_NO_VALID_FRAME" in line
    assert "ppm_last_rejected_us=204" in line
    assert summary["reason"] == "PPM_CHANNELS_PRESENT_BUT_INVALID"
    assert summary["ppm_decode_reason_latest"] == "PPM_SHORT_PULSE_NO_VALID_FRAME"


def test_manual_control_no_usbdbg_rows_classifies_without_all_na() -> None:
    """USBDBG 행이 아예 없으면 NO_USBDBG_TELEMETRY 로 분류(전부 NA 남발 금지) / No rows -> explicit no-telemetry reason."""
    summary = cli.evaluate_manual_control_rows([])
    assert summary["reason"] == "NO_USBDBG_TELEMETRY"
    assert summary["manual_switch"] == "UNKNOWN_NO_USBDBG_TELEMETRY"
    assert summary["mode_decode_reason"] == "NO_USBDBG_TELEMETRY"
    assert summary["gps_status_available"] is False
    assert summary["imu_status_available"] is False
    line = cli.format_manual_control_status(elapsed_s=0, rows=[])
    assert "mode_decode_reason=NO_USBDBG_TELEMETRY" in line


def test_manual_control_print_cmd_writes_summary(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """manual-control --print-cmd 가 프로파일/플래그/빌드경로 출력 + 표준 요약 기록 / print-cmd prints flags and writes summary."""
    rc = cli.main(["manual-control", "--print-cmd", "--out-dir", str(tmp_path)])
    assert rc == 0
    out = capsys.readouterr().out
    assert "manual_control_profile=rc-mix-ppm" in out
    assert "PPM_INTERRUPT_EDGE_FALLING=1" in out
    assert "PPM_SYNC_THRESHOLD_US=4000" in out
    assert "MANUAL_CONTROL_PPM=1" in out
    assert "IMU_ENABLE=0" in out
    assert "IMU_YAW_DIAG" not in out
    assert "/private/tmp/openrb-manual-rc-mix-ppm" in out
    assert "OpenRB D6" in out
    data = _assert_standard_summary(tmp_path)
    assert data["mode"] == "manual-control"
    assert data["profile"] == "rc-mix-ppm"
    assert data["ppm_input_pin"] == "D6"


# ── usb-pulse-test 결과 분류 + 옛 별칭 경고 / usb-pulse-test classification + deprecated aliases ──


def test_usb_pulse_test_classifies_no_command_sent_waiting_for_user() -> None:
    """사용자 중단 + 명령 미전송 -> WAITING_FOR_USER_ENTER / User-aborted with no command -> waiting-for-user."""
    result, reason, action = cli.station_drive_classification([], user_aborted=True)
    assert result == "WAITING_FOR_USER_ENTER"
    assert reason == "WAITING_FOR_USER_ENTER"
    assert "Press Enter" in action


def test_usb_pulse_test_classifies_telemetry_motion_but_user_none() -> None:
    """텔레메트리는 출력 활성인데 사용자가 '움직임 없음' 보고 -> 별도 사유 / Telemetry active but user saw none."""
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
    """사용자 움직임 확인 시 PASS, 사유 문자열에 옛 'Stage' 잔재 없음 / User-confirmed motion passes with clean names."""
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
    """옛 station-drive 별칭은 경고 후 usb-pulse-test 로 매핑 / Deprecated station-drive warns and maps to usb-pulse-test."""
    rc = cli.main(["station-drive", "--print-command", "true", "--out-dir", str(tmp_path)])
    assert rc == 0
    out = capsys.readouterr().out
    assert "Deprecated alias: use usb-pulse-test." in out
    data = _assert_standard_summary(tmp_path)
    assert data["mode"] == "usb-pulse-test"


def test_station_manual_alias_warns_and_maps_to_usb_pulse_test(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """옛 station-manual 별칭도 경고 후 usb-pulse-test 로 매핑 / Deprecated station-manual warns and maps too."""
    rc = cli.main(["station-manual", "--print-command", "true", "--out-dir", str(tmp_path)])
    assert rc == 0
    out = capsys.readouterr().out
    assert "Deprecated alias: use usb-pulse-test." in out
    data = _assert_standard_summary(tmp_path)
    assert data["mode"] == "usb-pulse-test"


# ── station-hw 진단/수동: 링크·파서·데드맨·E-stop·A/B 매핑 / station-hw diagnose & manual ──


def test_station_hw_diagnose_classifies_absent_link() -> None:
    """스테이션 링크 무신호 -> STATION_HW_LINK_ABSENT / No station link -> link-absent classification."""
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
    """수신은 되나 파싱 실패 -> WRONG_STATION_FRAME_PARSER(전송/프로토콜/파서 명시) / RX-but-parse-fail names the wrong parser."""
    rows = [
        {
            "station_rx_count": "5",
            "station_frame_count": "0",
            "station_parse_ok_count": "0",
            "station_parse_error_count": "5",
            "station_manual_valid": "false",
            "station_deadman": "false",
            "station_estop": "false",
        }
    ]
    summary = cli.evaluate_station_hw_rows(rows, mode="station-hw-diagnose")
    assert summary["reason"] == "WRONG_STATION_FRAME_PARSER"
    assert summary["station_link_seen"] is True
    assert summary["station_parse_error_count"] == 5
    assert summary["station_transport"] == "station_hardware_serial"
    assert summary["station_protocol"] == "auto"
    assert summary["station_parser"] == "auto_station_manual"


def test_station_hw_diagnose_classifies_deadman_false() -> None:
    """유효 프레임이지만 데드맨 비활성 -> STATION_HW_DEADMAN_NOT_ACTIVE / Deadman off -> deadman-not-active."""
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
    """E-stop 활성 -> STATION_HW_ESTOP_ACTIVE / E-stop asserted -> estop-active classification."""
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
    """station-hw-manual 이 forward/turn 을 물리 A/B 로 매핑하고 PASS(RC 불요) / Maps forward/turn to A/B and passes."""
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
    """station-hw-manual PASS 는 GPS/IMU/RC 없이도 성립 / Manual PASS requires no GPS, IMU, or RC."""
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
    """유효 A/B 지만 모터 미출력: manual=OUTPUT_BLOCKED(실패) vs diagnose=READY(성공) / Valid A/B, no motor: manual blocked, diagnose ready."""
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
    """상태 라인이 스테이션 파이프라인 필드(rx/frame/deadman/A/B)를 담고 hc12 잔재 없음 / Status line carries station pipeline fields."""
    rows = [
        {
            "station_rx_count": "3",
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
    assert "station_rx_count=3" in line
    assert "hc12_rx_count" not in line
    assert "reason_so_far=STATION_HW_MANUAL_READY" in line


def test_station_hw_firmware_flags_enable_ab_mapping() -> None:
    """station-hw-manual 펌웨어 플래그가 A/B 매핑 + RC 무시를 활성 / Flags enable A/B mapping and ignore RC."""
    flags = cli.station_hw_manual_firmware_flags()
    assert "STATION_HW_MANUAL_ENABLE=1" in flags
    assert "STATION_HW_MANUAL_A_B_MAPPING=1" in flags
    assert "STATION_HW_MANUAL_IGNORE_RC_INPUT=1" in flags


def test_station_hw_diagnose_print_command_writes_summary(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """station-hw-diagnose --print-command 가 안내 출력 + 표준 요약 기록 / print-command prints guide and writes summary."""
    rc = cli.main(["station-hw-diagnose", "--print-command", "true", "--out-dir", str(tmp_path)])
    assert rc == 0
    out = capsys.readouterr().out
    assert "STATION HARDWARE DIAGNOSE" in out
    summary = _assert_standard_summary(tmp_path)
    assert summary["mode"] == "station-hw-diagnose"


def test_station_hw_manual_default_duration_is_continuous() -> None:
    """station-hw-manual 기본 duration=0(연속), diagnose 는 유한 / Manual defaults to continuous, diagnose to finite."""
    parser = cli.build_parser()
    manual_args = parser.parse_args(["station-hw-manual"])
    diagnose_args = parser.parse_args(["station-hw-diagnose"])
    assert manual_args.duration_s == 0.0
    assert diagnose_args.duration_s > 0.0


def test_manual_control_default_duration_is_continuous() -> None:
    """manual-control 기본 duration=0(연속), profile=rc-mix-ppm / Manual-control defaults to continuous, rc-mix-ppm."""
    parser = cli.build_parser()
    args = parser.parse_args(["manual-control"])
    assert args.duration_s == 0.0
    assert args.profile == "rc-mix-ppm"


def test_station_hw_firmware_loop_stops_on_stale_deadman_or_estop() -> None:
    """펌웨어 루프가 오래된 데드맨/E-stop 에서 스테이션 수동 출력을 멈추는 소스 확인 / Firmware halts on stale deadman/estop."""
    source = Path("firmware/openrb_robot_controller/openrb_robot_controller.ino").read_text()
    assert "bool stationManualActive = stationManualFresh && stationManual.deadman && !stationEstop;" in source
    assert "if (stationEstop) {" in source
    assert "} else if (stationManualActive) {" in source
    assert "currentControlSource = CONTROL_SOURCE_STATION_MANUAL;" in source
    assert "applyStationManualCommand();" in source


def test_station_hw_firmware_supports_auto_station_manual_parser() -> None:
    """펌웨어가 MANUAL/AB/THROTTLE/STEER 자동 파서 + raw 프레임 덤프를 지원 / Firmware supports the auto station parser."""
    source = Path("firmware/openrb_robot_controller/openrb_robot_controller.ino").read_text()
    assert "parseStationManualLineAuto" in source
    assert 'command == "MANUAL"' in source
    assert 'command == "AB"' in source
    assert 'key == "THROTTLE"' in source
    assert 'key == "STEER"' in source
    assert "printStationRawFrameDump" in source


def test_station_hw_raw_frame_dumps_are_written(tmp_path: Path) -> None:
    """raw 스테이션 프레임(디코드/hex)이 파일로 정확히 기록됨 / Raw station frames (decoded + hex) are written to files."""
    rows = [
        {
            "station_raw_frame": "MANUAL%2Cnot_matching%2C%23",
            "station_raw_frame_hex": "4D414E55414C",
        },
        {
            "station_raw_frame": "A%3D0.1,B%3D0.2",
            "station_raw_frame_hex": "413D302E31",
        },
    ]
    count = cli.write_station_raw_frame_dumps(tmp_path, rows)
    assert count == 2
    assert (tmp_path / "raw_station_frames.txt").read_text().splitlines() == [
        "MANUAL,not_matching,#",
        "A=0.1,B=0.2",
    ]
    assert (tmp_path / "raw_station_frames_hex.txt").read_text().splitlines() == [
        "4D414E55414C",
        "413D302E31",
    ]


def test_station_hw_monitor_ctrl_c_writes_summary_without_traceback(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """모니터 중 Ctrl-C 는 트레이스백 없이 USER_ABORTED 요약 + CSV/로그 기록 / Ctrl-C -> clean USER_ABORTED summary."""
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
    """문서(README/필드매뉴얼)가 통합 런처만 안내하고 옛 stage 스크립트를 참조하지 않음 / Docs point only at the unified launcher."""
    readme = Path("README.md").read_text()
    ppp_readme = Path("docs/README_physical_path_planning.md").read_text()
    field_manual = Path("docs/field_test_manual.md").read_text()
    for text in (readme, ppp_readme, field_manual):
        assert "scripts/run_physical_path_planner.sh" in text
        assert "legacy/stage_scripts/run_" + "stage" not in text
    assert "RC_INPUT_ABSENT" in ppp_readme
    assert "RC_INPUT_ABSENT" in field_manual


# ── preview (시리얼/모션 없음, 기하 전용) / preview (no serial, no motion) ──


def test_preview_writes_guarded_summary(tmp_path: Path) -> None:
    """preview 가 커버리지 계획 요약(세그먼트/레인 수, path_shape)을 가드된 상태로 기록 / Preview writes a guarded coverage summary."""
    rc = cli.main(
        ["preview", *_PREVIEW_GOAL, "--workspace-width-m", "2", "--no-png", "--out-dir", str(tmp_path)]
    )
    assert rc == 0
    data = json.loads((tmp_path / "preview_summary.json").read_text())
    assert data["ready_for_full_path_following"] is False
    assert data["segment_count"] >= 1
    assert data["lane_count"] >= 1
    assert data["path_shape"] == "coverage_lawnmower"
    summary = _assert_standard_summary(tmp_path)
    assert summary["mode"] == "preview"


def test_preview_direct_line_needs_no_width(tmp_path: Path) -> None:
    """direct_line 모양은 워크스페이스 폭 없이도 성공 / A direct_line path needs no workspace width."""
    rc = cli.main(
        ["preview", *_PREVIEW_GOAL, "--path-shape", "direct_line", "--no-png", "--out-dir", str(tmp_path)]
    )
    assert rc == 0
    data = json.loads((tmp_path / "preview_summary.json").read_text())
    assert data["path_shape"] == "direct_line"


def test_preview_diagonal_without_width_fails_cleanly(tmp_path: Path) -> None:
    """대각선 목표에 폭 없이 serpentine 요청 -> PLAN_INPUT_INVALID 로 깔끔히 실패 / Diagonal serpentine without width fails cleanly."""
    # A->B is the diagonal, so a serpentine plan requires a workspace width.
    rc = cli.main(["preview", *_PREVIEW_GOAL, "--no-png", "--out-dir", str(tmp_path)])
    assert rc == 2
    assert not (tmp_path / "preview_summary.json").exists()
    summary = _assert_standard_summary(tmp_path)
    assert summary["reason"] == "PLAN_INPUT_INVALID"


def test_preview_captures_live_gps_start_from_log_when_omitted(tmp_path: Path) -> None:
    """시작 좌표 생략 시 로그의 라이브 GPS fix 를 시작점으로 캡처 / Omitted start captures live GPS from the log."""
    log = tmp_path / "gps.log"
    log.write_text(
        "USBDBG event=HEARTBEAT gps_chars=120 gps_solution_valid=false gps_ready=false "
        "last_rmc_status=V last_gga_fix_quality=0 gps_sats=0 current_lat=NA current_lon=NA\n"
        "USBDBG event=HEARTBEAT gps_chars=840 gps_solution_valid=true gps_ready=true "
        "current_lat=35.5709000 current_lon=129.1871000 "
        "gps_block_reason=OK gps_sats=7 gps_hdop=1.20\n"
    )
    rc = cli.main(
        [
            "preview",
            "--from-log",
            str(log),
            "--goal-mode",
            "relative_enu",
            "--goal-east-m",
            "4.0",
            "--goal-north-m",
            "-1.2",
            "--workspace-width-m",
            "1.2",
            "--step-spacing-m",
            "0.25",
            "--no-png",
            "--out-dir",
            str(tmp_path),
        ]
    )
    assert rc == 0
    summary = _assert_standard_summary(tmp_path)
    assert summary["start_source"] == "live_gps"
    assert summary["current_lat"] == pytest.approx(35.5709)
    assert summary["current_lon"] == pytest.approx(129.1871)
    assert summary["gps_cached_used"] is False
    assert summary["gps_ready"] is True
    assert summary["best_sats"] == pytest.approx(7)


def test_preview_uses_fresh_cached_gps_when_live_gps_missing(tmp_path: Path) -> None:
    """라이브 fix 없으면 신선한 캐시 GPS 를 시작점으로 사용 / Uses fresh cached GPS when the live fix is missing."""
    log = tmp_path / "gps.log"
    log.write_text(
        "USBDBG event=HEARTBEAT current_lat=NA current_lon=NA gps_cached_lat=35.5709000 "
        "gps_cached_lon=129.1871000 gps_cached_age_ms=5000 gps_block_reason=BAD_HDOP\n"
    )
    rc = cli.main(
        [
            "preview",
            "--from-log",
            str(log),
            "--goal-mode",
            "relative_enu",
            "--goal-east-m",
            "4.0",
            "--goal-north-m",
            "-1.2",
            "--workspace-width-m",
            "1.2",
            "--step-spacing-m",
            "0.25",
            "--no-png",
            "--out-dir",
            str(tmp_path),
        ]
    )
    assert rc == 0
    summary = _assert_standard_summary(tmp_path)
    assert summary["start_source"] == "cached_gps"
    assert summary["gps_cached_used"] is True


def test_preview_without_live_or_cached_gps_fails_cleanly(tmp_path: Path) -> None:
    """라이브도 캐시도 없으면 NO_USABLE_START_GPS 로 안내와 함께 실패 / No live/cached GPS fails cleanly with guidance."""
    log = tmp_path / "gps.log"
    log.write_text("USBDBG event=HEARTBEAT current_lat=NA current_lon=NA gps_block_reason=NO_LOCATION\n")
    rc = cli.main(
        [
            "preview",
            "--from-log",
            str(log),
            "--goal-mode",
            "relative_enu",
            "--goal-east-m",
            "4.0",
            "--goal-north-m",
            "-1.2",
            "--workspace-width-m",
            "1.2",
            "--step-spacing-m",
            "0.25",
            "--no-png",
            "--out-dir",
            str(tmp_path),
        ]
    )
    assert rc == 2
    summary = _assert_standard_summary(tmp_path)
    assert summary["reason"] == "NO_USABLE_START_GPS"
    assert "pass --start-lat and --start-lon" in summary["next_recommended_action"]


# ── gps-wait: 타임아웃/성공/파서불일치/중단/펌웨어 프로파일 / gps-wait lifecycle ──


def test_gps_wait_timeout_writes_best_observed_values(tmp_path: Path) -> None:
    """gps-wait 타임아웃 시 관측된 최선값(sats/hdop/imu)을 요약에 기록 / Timeout records best-observed GPS/IMU values."""
    log = tmp_path / "gps.log"
    log.write_text(
        "USBDBG event=HEARTBEAT gps_chars=100 gps_solution_valid=false gps_ready=false "
        "last_rmc_status=V last_gga_fix_quality=0 gps_sats=0 gps_hdop=NA current_lat=NA current_lon=NA\n"
        "USBDBG event=HEARTBEAT gps_chars=700 gps_solution_valid=false gps_ready=false "
        "last_rmc_status=V last_gga_fix_quality=0 gps_sats=3 gps_hdop=4.8 current_lat=NA current_lon=NA "
        "imu_present=true imu_relative_yaw_deg=2.5\n"
    )
    rc = cli.main(["gps-wait", "--from-log", str(log), "--timeout-s", "300", "--out-dir", str(tmp_path)])
    assert rc == 2
    summary = _assert_standard_summary(tmp_path)
    assert summary["reason"] == "GPS_WAIT_TIMEOUT"
    assert summary["best_sats"] == pytest.approx(3)
    assert summary["best_hdop"] == pytest.approx(4.8)
    assert summary["imu_present"] is True
    assert (tmp_path / "gps_wait.csv").exists()


def test_gps_wait_success_detects_delayed_fix(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """지연 도착한 유효 fix 를 감지해 GPS_READY 로 성공 / Detects a delayed fix and succeeds as GPS_READY."""
    monkeypatch.setattr(cli, "DEFAULT_GPS_CACHE", tmp_path / "gps_cache" / "latest_start.json")
    log = tmp_path / "gps.log"
    log.write_text(
        "USBDBG event=HEARTBEAT gps_chars=100 gps_solution_valid=false gps_ready=false "
        "last_rmc_status=V last_gga_fix_quality=0 gps_sats=0 current_lat=NA current_lon=NA\n"
        "USBDBG event=HEARTBEAT gps_chars=900 gps_solution_valid=true gps_ready=true "
        "last_rmc_status=A last_gga_fix_quality=1 gps_sats=6 gps_hdop=1.8 "
        "current_lat=35.5709000 current_lon=129.1871000 imu_present=true imu_relative_yaw_deg=3.0\n"
    )
    rc = cli.main(["gps-wait", "--from-log", str(log), "--timeout-s", "300", "--out-dir", str(tmp_path)])
    assert rc == 0
    summary = _assert_standard_summary(tmp_path)
    assert summary["reason"] == "GPS_READY"
    assert summary["current_lat"] == pytest.approx(35.5709)
    assert summary["current_lon"] == pytest.approx(129.1871)


def test_gps_wait_parse_mismatch_is_clear(tmp_path: Path) -> None:
    """파싱 불가한 텔레메트리는 WRONG_FIRMWARE_PROFILE... 로 명확히 진단 + raw 샘플 저장 / Unparseable lines diagnosed clearly."""
    log = tmp_path / "gps.log"
    log.write_text("RAW_GPS_LINE without key value telemetry\nanother unparsed sample\n")
    rc = cli.main(["gps-wait", "--from-log", str(log), "--timeout-s", "300", "--out-dir", str(tmp_path)])
    assert rc == 2
    summary = _assert_standard_summary(tmp_path)
    assert summary["reason"] == "WRONG_FIRMWARE_PROFILE_OR_TELEMETRY_PARSE_MISMATCH"
    assert (tmp_path / "raw_gps_samples.txt").exists()


def test_gps_wait_keyboard_interrupt_writes_summary(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """gps-wait 중 Ctrl-C 는 USER_ABORTED 요약 + raw 로그를 남기고 rc=130 / Ctrl-C -> USER_ABORTED summary, rc 130."""
    port = tmp_path / "ttyUSB-test"
    port.touch()

    class InterruptingSerial:
        def __init__(self, *args, **kwargs) -> None:
            pass

        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, tb) -> bool:
            return False

        def readline(self) -> bytes:
            raise KeyboardInterrupt

    monkeypatch.setitem(sys.modules, "serial", types.SimpleNamespace(Serial=InterruptingSerial))
    rc = cli.main([
        "gps-wait",
        "--port",
        str(port),
        "--upload",
        "false",
        "--timeout-s",
        "300",
        "--out-dir",
        str(tmp_path),
    ])
    assert rc == 130
    summary = _assert_standard_summary(tmp_path)
    assert summary["reason"] == "USER_ABORTED"
    assert (tmp_path / "raw_usbdbg.log").exists()


def test_gps_wait_uploads_mac_physical_supervised_profile(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """포트 지정 gps-wait 는 MAC_PHYSICAL_SUPERVISED 프로파일을 업로드 후 GPS_READY / Uploads the supervised profile then GPS_READY."""
    port = tmp_path / "ttyUSB-test"
    port.touch()
    compile_flags: list[str] = []

    class Completed:
        returncode = 0

    def fake_run(cmd, check=False, **kwargs):
        joined = " ".join(str(part) for part in cmd)
        if "compiler.cpp.extra_flags=" in joined:
            compile_flags.append(joined)
        return Completed()

    class ReadySerial:
        def __init__(self, *args, **kwargs) -> None:
            self.lines = [
                b"USB_PULSE_TEST event=HEARTBEAT firmware_profile=MAC_PHYSICAL_SUPERVISED "
                b"gps_chars=1200 gps_solution_valid=true gps_ready=true gps_sats=6 gps_hdop=1.8 "
                b"current_lat=35.5709000 current_lon=129.1871000 imu_present=true "
                b"imu_relative_yaw_deg=3.0\n"
            ]

        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, tb) -> bool:
            return False

        def readline(self) -> bytes:
            return self.lines.pop(0) if self.lines else b""

    monkeypatch.setattr(subprocess, "run", fake_run)
    monkeypatch.setitem(sys.modules, "serial", types.SimpleNamespace(Serial=ReadySerial))
    rc = cli.main(["gps-wait", "--port", str(port), "--timeout-s", "5", "--out-dir", str(tmp_path)])
    assert rc == 0
    assert compile_flags
    assert "MAC_PHYSICAL_SUPERVISED=1" in compile_flags[0]
    summary = _assert_standard_summary(tmp_path)
    assert summary["firmware_profile"] == "MAC_PHYSICAL_SUPERVISED"
    assert summary["reason"] == "GPS_READY"


def test_gps_wait_serial_disconnect_writes_summary(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """시리얼 끊김(OSError)은 SERIAL_DISCONNECT 요약으로 귀결(트레이스백 없음) / Serial disconnect -> clean summary."""
    port = tmp_path / "ttyUSB-test"
    port.touch()

    class DisconnectingSerial:
        def __init__(self, *args, **kwargs) -> None:
            pass

        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, tb) -> bool:
            return False

        def readline(self) -> bytes:
            raise OSError("device disconnected")

    monkeypatch.setitem(sys.modules, "serial", types.SimpleNamespace(Serial=DisconnectingSerial))
    rc = cli.main([
        "gps-wait",
        "--port",
        str(port),
        "--upload",
        "false",
        "--timeout-s",
        "5",
        "--out-dir",
        str(tmp_path),
    ])
    assert rc == 2
    summary = _assert_standard_summary(tmp_path)
    assert summary["reason"] == "SERIAL_DISCONNECT"
    assert (tmp_path / "raw_usbdbg.log").exists()


def test_preview_live_gps_start_uses_mac_physical_supervised_profile(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """포트 지정 preview 도 감독 프로파일을 업로드하고 start_source=live_gps / Port preview uploads supervised profile, live GPS start."""
    port = tmp_path / "ttyUSB-test"
    port.touch()
    compile_flags: list[str] = []

    class Completed:
        returncode = 0

    def fake_run(cmd, check=False, **kwargs):
        joined = " ".join(str(part) for part in cmd)
        if "compiler.cpp.extra_flags=" in joined:
            compile_flags.append(joined)
        return Completed()

    class ReadySerial:
        def __init__(self, *args, **kwargs) -> None:
            self.lines = [
                b"USB_PULSE_TEST event=HEARTBEAT firmware_profile=MAC_PHYSICAL_SUPERVISED "
                b"gps_chars=1200 gps_solution_valid=true gps_ready=true gps_sats=6 gps_hdop=1.8 "
                b"current_lat=35.5709000 current_lon=129.1871000 imu_present=true "
                b"imu_relative_yaw_deg=3.0\n"
            ]

        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, tb) -> bool:
            return False

        def readline(self) -> bytes:
            return self.lines.pop(0) if self.lines else b""

    monkeypatch.setattr(subprocess, "run", fake_run)
    monkeypatch.setitem(sys.modules, "serial", types.SimpleNamespace(Serial=ReadySerial))
    rc = cli.main([
        "preview",
        "--port",
        str(port),
        "--goal-mode",
        "relative_enu",
        "--goal-east-m",
        "4.0",
        "--goal-north-m",
        "-1.2",
        "--workspace-width-m",
        "1.2",
        "--step-spacing-m",
        "0.25",
        "--no-png",
        "--out-dir",
        str(tmp_path),
    ])
    assert rc == 0
    assert compile_flags and "MAC_PHYSICAL_SUPERVISED=1" in compile_flags[0]
    summary = _assert_standard_summary(tmp_path)
    assert summary["firmware_profile"] == "MAC_PHYSICAL_SUPERVISED"
    assert summary["start_source"] == "live_gps"


# ── run / execute-plan: --print-plan 은 시리얼 미개방 유지 / run/execute-plan --print-plan stays no-serial ──


def test_run_print_plan_builds_without_opening_serial(tmp_path: Path) -> None:
    """run --print-plan 은 시리얼 없이 계획을 만들고 PLAN_PRINTED 요약 / run --print-plan builds plan without serial."""
    rc = cli.main(
        ["run", *_PREVIEW_GOAL, "--workspace-width-m", "2", "--print-plan", "--out-dir", str(tmp_path)]
    )
    assert rc == 0
    plan = json.loads((tmp_path / "plan.json").read_text())
    assert plan["ready_for_full_path_following"] is False
    assert plan["segment_count"] >= 1
    summary = _assert_standard_summary(tmp_path)
    assert summary["reason"] == "PLAN_PRINTED"


def test_run_print_plan_waits_for_gps_before_planning(tmp_path: Path) -> None:
    """run --print-plan 은 계획 전 GPS 준비를 기다려 라이브 시작점을 확보 / Waits for GPS before planning."""
    log = tmp_path / "gps.log"
    log.write_text(
        "USBDBG event=HEARTBEAT gps_chars=100 gps_solution_valid=false gps_ready=false "
        "last_rmc_status=V last_gga_fix_quality=0 gps_sats=0 current_lat=NA current_lon=NA\n"
        "USBDBG event=HEARTBEAT gps_chars=900 gps_solution_valid=true gps_ready=true "
        "last_rmc_status=A last_gga_fix_quality=1 gps_sats=6 gps_hdop=1.8 "
        "current_lat=35.5709000 current_lon=129.1871000\n"
    )
    rc = cli.main(
        [
            "run",
            "--from-log",
            str(log),
            "--goal-mode",
            "relative_enu",
            "--goal-east-m",
            "4.0",
            "--goal-north-m",
            "-1.2",
            "--workspace-width-m",
            "1.2",
            "--step-spacing-m",
            "0.25",
            "--print-plan",
            "--out-dir",
            str(tmp_path),
        ]
    )
    assert rc == 0
    summary = _assert_standard_summary(tmp_path)
    assert summary["reason"] == "PLAN_PRINTED"
    assert summary["start_source"] == "live_gps"
    assert summary["gps_ready"] is True


def test_execute_plan_is_an_alias_of_run(tmp_path: Path) -> None:
    """execute-plan 은 run 의 별칭(같은 계획 산출) / execute-plan is an alias of run."""
    rc = cli.main(
        ["execute-plan", *_PREVIEW_GOAL, "--workspace-width-m", "2", "--print-plan", "--out-dir", str(tmp_path)]
    )
    assert rc == 0
    assert (tmp_path / "plan.json").exists()


def test_execute_plan_chunking_defaults_are_safe() -> None:
    """execute-plan 청킹/보정 기본값이 안전 범위(chunk/max/heading-hold 등) / Chunking defaults are safe."""
    parser = cli.build_parser()
    args = parser.parse_args(["execute-plan"])
    assert args.live_chunk_ms == 700
    assert args.max_segment_chunks == 20
    assert args.max_ms == 1000
    assert args.gps_degradation_policy == "continue"
    assert args.imu_heading_hold == "true"
    assert args.cross_track_correction == "true"


def test_execute_plan_can_load_existing_plan_dir_without_start_coords(tmp_path: Path) -> None:
    """기존 plan-dir 을 시작 좌표 없이 로드(start_source=plan_dir) / Loads an existing plan-dir without start coords."""
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
    summary = _assert_standard_summary(out_dir)
    assert summary["start_source"] == "plan_dir"
    assert summary["current_lat"] == pytest.approx(35.1)
    assert summary["current_lon"] == pytest.approx(129.1)
    assert summary["rc_ignored_for_usb_supervised"] is True


def test_execute_plan_loads_default_motion_calibration(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """기본 위치의 승인된 모션 캘리브레이션을 로드해 각도-보정/연속 드라이브로 계획 / Loads default motion calibration."""
    monkeypatch.chdir(tmp_path)
    cal_dir = tmp_path / "outputs" / "physical_path_planning" / "calibration"
    cal_dir.mkdir(parents=True)
    (cal_dir / "motion_calibration.json").write_text(
        json.dumps(
            {
                "forward": {"a": 0.30, "b": 0.0, "ms": 900, "approved_by_user": True},
                "backward": {"a": -0.08, "b": 0.0, "ms": 350, "approved_by_user": True},
                "turn_left_90": {
                    "a": 0.0,
                    "b": 0.24,
                    "ms": 1300,
                    "target_angle_deg": 90,
                    "approved_by_user": True,
                },
                "turn_right_90": {
                    "a": 0.0,
                    "b": -0.12,
                    "ms": 850,
                    "target_angle_deg": 90,
                    "approved_by_user": True,
                },
            }
        )
    )
    out_dir = tmp_path / "execute"
    rc = cli.main(
        [
            "execute-plan",
            *_PREVIEW_GOAL,
            "--workspace-width-m",
            "2",
            "--print-plan",
            "--out-dir",
            str(out_dir),
        ]
    )
    assert rc == 0
    summary = _assert_standard_summary(out_dir)
    assert summary["motion_calibration_loaded"] is True
    assert summary["connector_mode_effective"] == "angle_calibrated"
    assert summary["continuous_drive_used"] is True
    plan = json.loads((out_dir / "plan.json").read_text())
    primitives = plan["primitives"]
    assert any(p["primitive_type"] == "move_forward" and p["pulse_ms"] == 900 for p in primitives)
    assert any(p["primitive_type"] == "move_backward" and p["pulse_ms"] == 350 for p in primitives)


# ── diagnose --from-log (시리얼 없음) / diagnose --from-log (no serial) ──


def test_diagnose_from_log_summarizes_telemetry(tmp_path: Path) -> None:
    """저장된 로그를 파싱해 행/하트비트/마지막 GPS 차단사유를 요약 / Summarizes a saved log (rows, heartbeats, block reason)."""
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
    """manual-rc --print-cmd 가 옛 known-good RC 경로 표식과 함께 표준 요약 기록 / manual-rc print-cmd writes standard summary."""
    rc = cli.main(["manual-rc", "--print-cmd", "--out-dir", str(tmp_path)])
    assert rc == 0
    data = _assert_standard_summary(tmp_path)
    assert data["mode"] == "manual-rc"
    assert data["rc_input_mode_requested"] == "old_known_good"
    assert data["rc_input_mode_effective"] == "ppm_old_known_good"
    assert data["old_known_good_rc_path"] is True


def test_rc_input_diagnose_absent_from_log(tmp_path: Path) -> None:
    """무신호 PPM 프로브 로그 -> RC_INPUT_ABSENT / An empty PPM probe log classifies as RC_INPUT_ABSENT."""
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
    """유효 다채널 PPM 로그 -> RC_CHANNELS_PRESENT_AND_VALID / A valid multi-channel PPM log passes."""
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
    """rc-input-diagnose --print-cmd 가 PPM 프로브 명령 출력 + 표준 요약 / print-cmd prints the PPM probe command."""
    rc = cli.main(["rc-input-diagnose", "--print-cmd", "--out-dir", str(tmp_path)])
    assert rc == 0
    out = capsys.readouterr().out
    assert "ppm_channel_map_probe" in out
    data = _assert_standard_summary(tmp_path)
    assert data["mode"] == "rc-input-diagnose"


def test_manual_rc_upload_success_and_rc_absent_summary(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    """업로드는 성공하되 검증이 RC 부재면 upload_success=True + validation_success=False / Upload ok but RC-absent validation."""
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
    """diagnose_summary 는 순수(부수효과 없음) + 이벤트 카운트/가드된 ready=False / diagnose_summary is pure and guarded."""
    log = tmp_path / "serial.log"
    log.write_text(_LOG)
    rows = cli.load_rows_from_log(log)
    summary = cli.diagnose_summary(rows)
    assert summary["event_counts"]["HEARTBEAT"] == 1
    assert summary["event_counts"]["ACK"] == 1
    assert summary["last_imu_relative_yaw_deg"] == "3.500000"
    assert summary["ready_for_full_path_following"] is False


# ── align-heading / initial-heading-align / reset-motion-calibration ──


def _build_preview_plan_dir(tmp_path: Path) -> Path:
    """preview 를 돌려 정렬 테스트가 소비할 plan-dir 을 만든다 / Build a preview plan-dir for alignment tests."""
    plan_dir = tmp_path / "preview"
    rc = cli.main(
        ["preview", *_PREVIEW_GOAL, "--workspace-width-m", "2", "--no-png", "--out-dir", str(plan_dir)]
    )
    assert rc == 0
    return plan_dir


def test_align_heading_skip_writes_artifacts_without_serial(tmp_path: Path) -> None:
    """strategy=skip 은 시리얼 없이 아티팩트 기록 + ready_for_execute_plan=True / skip writes artifacts without serial."""
    plan_dir = _build_preview_plan_dir(tmp_path)
    out_dir = tmp_path / "align"
    rc = cli.main(
        ["align-heading", "--plan-dir", str(plan_dir), "--strategy", "skip", "--out-dir", str(out_dir)]
    )
    assert rc == 0
    summary = json.loads((out_dir / "align_heading_summary.json").read_text())
    assert summary["mode"] == "align-heading"
    assert summary["strategy"] == "skip"
    assert summary["reason"] == "ALIGNMENT_SKIPPED"
    assert summary["alignment_success"] is True
    assert summary["ready_for_execute_plan"] is True
    assert summary["ready_for_full_path_following"] is False
    assert (out_dir / "align_heading_trace.csv").exists()
    assert (out_dir / "raw_usbdbg.log").exists()
    _assert_standard_summary(out_dir)
    for old_term in ("stage" + "20", "stage" + "16"):
        assert old_term not in json.dumps(summary).lower()


def test_align_heading_requires_plan_dir(tmp_path: Path) -> None:
    """plan-dir 없이 align-heading 은 ALIGN_PLAN_DIR_REQUIRED 로 거부 / align-heading requires a plan-dir."""
    out_dir = tmp_path / "align"
    rc = cli.main(["align-heading", "--strategy", "skip", "--out-dir", str(out_dir)])
    assert rc == 2
    summary = _assert_standard_summary(out_dir)
    assert summary["reason"] == "ALIGN_PLAN_DIR_REQUIRED"


def test_align_heading_plan_dir_without_segments_fails_cleanly(tmp_path: Path) -> None:
    """세그먼트 없는 plan-dir 은 PLAN_HAS_NO_SEGMENTS 로 깔끔히 실패 / A segment-less plan-dir fails cleanly."""
    plan_dir = tmp_path / "preview"
    plan_dir.mkdir()
    (plan_dir / "preview_summary.json").write_text(json.dumps({"segments": []}))
    out_dir = tmp_path / "align"
    rc = cli.main(["align-heading", "--plan-dir", str(plan_dir), "--strategy", "skip", "--out-dir", str(out_dir)])
    assert rc == 2
    summary = _assert_standard_summary(out_dir)
    assert summary["reason"] == "PLAN_HAS_NO_SEGMENTS"


def test_execute_plan_default_initial_heading_align_is_none() -> None:
    """execute-plan/run 의 초기 헤딩정렬 기본은 none / execute-plan and run default initial-heading-align to none."""
    parser = cli.build_parser()
    assert parser.parse_args(["execute-plan"]).initial_heading_align == "none"
    assert parser.parse_args(["run"]).initial_heading_align == "none"


def test_auto_relative_run_defaults_initial_heading_align_to_gps_probe() -> None:
    """auto-relative-run 은 초기 헤딩정렬 기본이 gps_probe / auto-relative-run defaults to gps_probe alignment."""
    parser = cli.build_parser()
    assert parser.parse_args(["auto-relative-run"]).initial_heading_align == "gps_probe"


def test_run_align_tunables_have_documented_defaults() -> None:
    """정렬 튜너블(허용오차/프로브 A/지속/최소거리)이 문서화된 기본값 유지 / Alignment tunables keep documented defaults."""
    args = cli.build_parser().parse_args(["execute-plan"])
    assert args.align_heading_tolerance_deg == pytest.approx(8.0)
    assert args.align_probe_a == pytest.approx(0.25)
    assert args.align_probe_duration_s == pytest.approx(1.0)
    assert args.align_min_probe_distance_m == pytest.approx(0.30)


def test_execute_plan_print_plan_with_gps_probe_align_stays_no_serial(tmp_path: Path) -> None:
    """정렬 전략을 요청해도 --print-plan 은 정렬/시리얼 전에 단락(short-circuit) / print-plan short-circuits before alignment."""
    # --print-plan must short-circuit before any alignment / serial work even when
    # an alignment strategy is requested.
    rc = cli.main(
        [
            "execute-plan",
            *_PREVIEW_GOAL,
            "--workspace-width-m",
            "2",
            "--initial-heading-align",
            "gps_probe",
            "--print-plan",
            "--out-dir",
            str(tmp_path),
        ]
    )
    assert rc == 0
    assert (tmp_path / "plan.json").exists()
    summary = _assert_standard_summary(tmp_path)
    assert summary["reason"] == "PLAN_PRINTED"
    assert not (tmp_path / "alignment").exists()


def test_run_initial_alignment_none_preserves_start_yaw(tmp_path: Path) -> None:
    """정렬=none 이면 정렬을 수행하지 않고 주어진 start_yaw 를 그대로 보존 / align=none preserves the given start yaw."""
    args = types.SimpleNamespace(initial_heading_align="none", start_yaw_deg=12.5)
    raw_lines: list[str] = []
    result = cli._run_initial_alignment(
        None, args, segments=[{"target_heading_deg": 90.0}], out_dir=tmp_path, raw_lines=raw_lines
    )
    assert result["performed"] is False
    assert result["ok"] is True
    assert result["aligned_yaw_deg"] == 12.5
    assert result["abort_reason"] is None
    assert result["summary"] is None
    assert raw_lines == []
    assert not (tmp_path / "alignment").exists()


def test_reset_motion_calibration_backs_up_and_clears_existing(tmp_path: Path) -> None:
    """기존 캘리브레이션을 타임스탬프 백업 후 삭제(재캘리브 준비) / Backs up then clears existing calibration."""
    cal_path = tmp_path / "cal" / "motion_calibration.json"
    cal_path.parent.mkdir(parents=True)
    cal_path.write_text(json.dumps({"forward": {"a": 0.30, "b": 0.0, "ms": 900, "approved_by_user": True}}))
    out_dir = tmp_path / "reset"
    rc = cli.main(
        ["reset-motion-calibration", "--calibration-out", str(cal_path), "--out-dir", str(out_dir)]
    )
    assert rc == 0
    summary = json.loads((out_dir / "reset_motion_calibration_summary.json").read_text())
    assert summary["mode"] == "reset-motion-calibration"
    assert summary["reason"] == "CALIBRATION_RESET"
    assert summary["existing_calibration_found"] is True
    assert summary["backup_path"] != "NONE"
    assert summary["ready_for_full_path_following"] is False
    assert not cal_path.exists()  # original cleared for a fresh recalibration
    assert Path(summary["backup_path"]).exists()  # prior values preserved
    _assert_standard_summary(out_dir)
    for old_term in ("stage" + "20", "stage" + "16"):
        assert old_term not in json.dumps(summary).lower()


def test_reset_motion_calibration_no_existing_is_clean(tmp_path: Path) -> None:
    """기존 파일이 없으면 NO_EXISTING_CALIBRATION + 백업 NONE 로 깔끔히 처리 / No existing file is handled cleanly."""
    cal_path = tmp_path / "cal" / "motion_calibration.json"  # never created
    out_dir = tmp_path / "reset"
    rc = cli.main(
        ["reset-motion-calibration", "--calibration-out", str(cal_path), "--out-dir", str(out_dir)]
    )
    assert rc == 0
    summary = json.loads((out_dir / "reset_motion_calibration_summary.json").read_text())
    assert summary["reason"] == "NO_EXISTING_CALIBRATION"
    assert summary["existing_calibration_found"] is False
    assert summary["backup_path"] == "NONE"


def test_tune_motion_reset_calibration_backs_up_before_session(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """tune-motion --reset-calibration 은 후보 세션 전에 백업+초기화 / tune-motion resets (with backup) before the session."""
    cal_path = tmp_path / "cal" / "motion_calibration.json"
    cal_path.parent.mkdir(parents=True)
    cal_path.write_text(json.dumps({"forward": {"a": 0.30, "b": 0.0, "ms": 900, "approved_by_user": True}}))
    out_dir = tmp_path / "tune"
    rc = cli.main(
        [
            "tune-motion",
            "--primitive",
            "forward",
            "--reset-calibration",
            "true",
            "--print-candidate",
            "true",
            "--calibration-out",
            str(cal_path),
            "--out-dir",
            str(out_dir),
        ]
    )
    assert rc == 0
    out = capsys.readouterr().out
    assert "calibration backed up:" in out
    assert "calibration reset:" in out
    assert not cal_path.exists()  # cleared before the printed candidate session
    backups = list(cal_path.parent.glob("motion_calibration.backup_*.json"))
    assert backups, "a timestamped backup should remain after reset"


def test_help_includes_alignment_and_reset_modes() -> None:
    """도움말에 align-heading/reset-motion-calibration 모드가 노출(옛 Stage 용어 없음) / Help lists alignment/reset modes."""
    help_text = cli.build_parser().format_help()
    assert "align-heading" in help_text
    assert "reset-motion-calibration" in help_text
    for old_term in ("Stage" + "20", "Stage" + "16"):
        assert old_term not in help_text
