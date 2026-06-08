from __future__ import annotations

import subprocess
from pathlib import Path

from tools import check_guarded_crawl_log


REPO = Path(__file__).resolve().parents[1]
SCRIPT = REPO / "scripts" / "run_guarded_motor_sanity_crawl.sh"


def test_stage15_script_compile_flags_are_guarded() -> None:
    text = SCRIPT.read_text(encoding="utf-8")
    assert "STAGE15_GUARDED_CRAWL_TEST=1" in text
    assert "PHYSICAL_PATH_FOLLOWING_ENABLE=0" in text
    assert "PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0" in text
    assert "PATH_FOLLOWING_DRYRUN=0" in text
    assert "GROUND_CRAWL_TEST_MODE=0" in text
    assert "AUTO_MOTION_ARMED=0" in text


def test_stage15_script_refuses_long_pulse_before_compile(tmp_path: Path) -> None:
    result = subprocess.run(
        [
            "bash",
            str(SCRIPT),
            "--port",
            "/dev/null",
            "--pulse-ms",
            "301",
            "--out-dir",
            str(tmp_path),
        ],
        cwd=REPO,
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 2
    assert "--dangerously-allow-longer-pulse" in result.stderr
    assert "arduino-cli compile" not in result.stdout


def test_stage15_script_refuses_high_cmd_before_compile(tmp_path: Path) -> None:
    result = subprocess.run(
        [
            "bash",
            str(SCRIPT),
            "--port",
            "/dev/null",
            "--max-cmd",
            "0.11",
            "--out-dir",
            str(tmp_path),
        ],
        cwd=REPO,
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 2
    assert "--dangerously-allow-higher-cmd" in result.stderr
    assert "arduino-cli compile" not in result.stdout


def test_guarded_crawl_checker_passes_safe_log() -> None:
    log = "\n".join(
        [
            "STAGE15 stage15_guarded_crawl=true event=pulse_start path_following_enable=false "
            "path_package_used=false virtual_controller_used=false autonomous_mode=false "
            "pulse_type=test_forward_pulse pulse_cmd_left=0.060 pulse_cmd_right=0.060 "
            "pulse_ms=200 physical_output_active=true stop_confirmed=false rc_ok=true neutral_ok=true "
            "final_left_cmd=0.060 final_right_cmd=0.060",
            "STAGE15 stage15_guarded_crawl=true event=pulse_stop path_following_enable=false "
            "path_package_used=false virtual_controller_used=false autonomous_mode=false "
            "pulse_type=test_stop pulse_cmd_left=0.000 pulse_cmd_right=0.000 pulse_ms=200 "
            "physical_output_active=false stop_confirmed=true final_left_cmd=0.000 final_right_cmd=0.000",
            "STAGE15 stage15_guarded_crawl=true event=pulse_start path_following_enable=false "
            "path_package_used=false virtual_controller_used=false autonomous_mode=false "
            "pulse_type=test_rotate_left_pulse pulse_cmd_left=-0.060 pulse_cmd_right=0.060 "
            "pulse_ms=200 physical_output_active=true stop_confirmed=false rc_ok=true neutral_ok=true "
            "final_left_cmd=-0.060 final_right_cmd=0.060",
            "STAGE15 stage15_guarded_crawl=true event=pulse_stop path_following_enable=false "
            "path_package_used=false virtual_controller_used=false autonomous_mode=false "
            "pulse_type=test_stop pulse_cmd_left=0.000 pulse_cmd_right=0.000 pulse_ms=200 "
            "physical_output_active=false stop_confirmed=true final_left_cmd=0.000 final_right_cmd=0.000",
        ]
    )
    result = check_guarded_crawl_log.evaluate_log_text(log, max_cmd=0.08, max_duration_ms=300)
    assert result["verdict"] == "PASS"
    assert result["motor_command_generated"] is False


def test_guarded_crawl_checker_fails_continuous_output() -> None:
    log = (
        "STAGE15 stage15_guarded_crawl=true event=pulse_start path_following_enable=false "
        "path_package_used=false virtual_controller_used=false autonomous_mode=false "
        "pulse_type=test_forward_pulse pulse_cmd_left=0.060 pulse_cmd_right=0.060 "
        "pulse_ms=200 physical_output_active=true final_left_cmd=0.060 final_right_cmd=0.060\n"
    )
    result = check_guarded_crawl_log.evaluate_log_text(log, max_cmd=0.08, max_duration_ms=300)
    assert result["verdict"] == "FAIL"
    assert "NO_STOP_CONFIRMATION_AFTER_LAST_PULSE" in result["reasons"]


def test_guarded_crawl_checker_fails_path_following_active() -> None:
    log = "\n".join(
        [
            "STAGE15 stage15_guarded_crawl=true event=pulse_start path_following_enable=true "
            "path_package_used=false virtual_controller_used=false autonomous_mode=false "
            "pulse_type=test_forward_pulse pulse_cmd_left=0.060 pulse_cmd_right=0.060 "
            "pulse_ms=200 physical_output_active=true final_left_cmd=0.060 final_right_cmd=0.060",
            "STAGE15 stage15_guarded_crawl=true event=pulse_stop path_following_enable=true "
            "path_package_used=false virtual_controller_used=false autonomous_mode=false "
            "pulse_type=test_stop pulse_cmd_left=0.000 pulse_cmd_right=0.000 pulse_ms=200 "
            "physical_output_active=false stop_confirmed=true final_left_cmd=0.000 final_right_cmd=0.000",
        ]
    )
    result = check_guarded_crawl_log.evaluate_log_text(log, max_cmd=0.08, max_duration_ms=300)
    assert result["verdict"] == "FAIL"
    assert "PATH_OR_AUTONOMOUS_FIELD_ACTIVE" in result["reasons"]
