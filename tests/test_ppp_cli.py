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
from pathlib import Path

import pytest

from tools.physical_path_planning import cli

_HEARTBEAT = (
    "STAGE20 event=HEARTBEAT stage20_physical_ab_guarded_crawl=true rc_ok=true "
    "neutral_ok=true physical_output_active=false gps_block_reason=OK gps_sats=9 "
    "gps_hdop=1.20 imu_relative_yaw_deg=3.5"
)
_LOG = f"{_HEARTBEAT}\nSTAGE20 event=ACK\nSTAGE20 event=STOP final_left_cmd=0.000 final_right_cmd=0.000 physical_output_active=false\n"

_PREVIEW_GOAL = [
    "--start-lat", "35.1",
    "--start-lon", "129.1",
    "--goal-mode", "bearing_distance",
    "--goal-bearing-deg", "90",
    "--goal-distance-m", "6",
]


# --- calibrate-turn argv (always --imu-angle-compare true) --------------------


def test_calibrate_turn_argv_always_enables_imu_angle_compare() -> None:
    argv = cli.build_calibrate_turn_argv(
        script="scripts/run_stage20_physical_ab_probe.sh",
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


def test_calibrate_turn_print_cmd_does_not_invoke_subprocess(capsys: pytest.CaptureFixture[str], tmp_path: Path) -> None:
    rc = cli.main(
        ["calibrate-turn", "--print-cmd", "--mode", "turn_right", "--out-dir", str(tmp_path)]
    )
    assert rc == 0
    out = capsys.readouterr().out
    assert "--imu-angle-compare true" in out
    assert "run_stage20_physical_ab_probe.sh" in out
    assert "turn_right" in out


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


# --- run / execute-plan: --print-plan stays no-serial -------------------------


def test_run_print_plan_builds_without_opening_serial(tmp_path: Path) -> None:
    rc = cli.main(
        ["run", *_PREVIEW_GOAL, "--workspace-width-m", "2", "--print-plan", "--out-dir", str(tmp_path)]
    )
    assert rc == 0
    plan = json.loads((tmp_path / "plan.json").read_text())
    assert plan["ready_for_full_path_following"] is False
    assert plan["segment_count"] >= 1


def test_execute_plan_is_an_alias_of_run(tmp_path: Path) -> None:
    rc = cli.main(
        ["execute-plan", *_PREVIEW_GOAL, "--workspace-width-m", "2", "--print-plan", "--out-dir", str(tmp_path)]
    )
    assert rc == 0
    assert (tmp_path / "plan.json").exists()


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


def test_diagnose_summary_is_pure_and_guarded(tmp_path: Path) -> None:
    log = tmp_path / "serial.log"
    log.write_text(_LOG)
    rows = cli.load_rows_from_log(log)
    summary = cli.diagnose_summary(rows)
    assert summary["event_counts"]["HEARTBEAT"] == 1
    assert summary["event_counts"]["ACK"] == 1
    assert summary["last_imu_relative_yaw_deg"] == "3.500000"
    assert summary["ready_for_full_path_following"] is False
