import json
from pathlib import Path

import pytest

from tools.physical_path_planning import cli


_RELATIVE_PREVIEW_ARGS = [
    "preview",
    "--start-lat",
    "35.5709000",
    "--start-lon",
    "129.1871000",
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
]


def test_relative_enu_field_config_resolves_a_origin_and_b_offset(tmp_path: Path) -> None:
    rc = cli.main([*_RELATIVE_PREVIEW_ARGS, "--out-dir", str(tmp_path)])
    assert rc == 0
    field = json.loads((tmp_path / "field_config_resolved.json").read_text())
    assert field["start_x_m"] == 0.0
    assert field["start_y_m"] == 0.0
    assert field["goal_mode"] == "relative_enu"
    assert field["goal_east_m"] == 4.0
    assert field["goal_north_m"] == -1.2
    assert field["resolved_goal_x_m"] == 4.0
    assert field["resolved_goal_y_m"] == -1.2
    assert field["workspace_width_m"] == 1.2
    assert field["step_spacing_m"] == 0.25
    assert field["expected_lane_count"] > 1
    assert (tmp_path / "planned_segments.csv").exists()
    assert (tmp_path / "planned_primitives.csv").exists()


def test_preview_5m_relative_enu_resolves_a_origin_and_5m_east_goal(tmp_path: Path) -> None:
    # The documented 5m field experiment: A=(0,0), B=(5,0), width 1.5m.
    rc = cli.main(
        [
            "preview",
            "--start-lat", "35.5709000",
            "--start-lon", "129.1871000",
            "--goal-mode", "relative_enu",
            "--goal-east-m", "5.0",
            "--goal-north-m", "0.0",
            "--workspace-width-m", "1.5",
            "--step-spacing-m", "0.30",
            "--path-shape", "diagonal_rectangle_serpentine",
            "--no-png",
            "--out-dir", str(tmp_path),
        ]
    )
    assert rc == 0
    field = json.loads((tmp_path / "field_config_resolved.json").read_text())
    assert field["start_x_m"] == 0.0
    assert field["start_y_m"] == 0.0
    assert field["resolved_goal_x_m"] == pytest.approx(5.0)
    assert field["resolved_goal_y_m"] == pytest.approx(0.0)
    assert field["workspace_width_m"] == 1.5
    assert field["step_spacing_m"] == 0.30
    assert field["lane_count"] > 1
    assert field["segment_count"] > 1


def test_preview_summary_includes_resolved_field_config(tmp_path: Path) -> None:
    rc = cli.main([*_RELATIVE_PREVIEW_ARGS, "--out-dir", str(tmp_path)])
    assert rc == 0
    summary = json.loads((tmp_path / "summary.json").read_text())
    assert summary["field_config"]["start_lat"] == 35.5709
    assert summary["field_config"]["workspace_width_m"] == 1.2
    assert summary["field_config"]["path_shape"] == "coverage_lawnmower"


def test_execute_plan_loads_field_config_from_plan_dir(tmp_path: Path) -> None:
    plan_dir = tmp_path / "preview"
    out_dir = tmp_path / "execute"
    rc = cli.main([*_RELATIVE_PREVIEW_ARGS, "--out-dir", str(plan_dir)])
    assert rc == 0
    rc = cli.main(["execute-plan", "--plan-dir", str(plan_dir), "--print-plan", "--out-dir", str(out_dir)])
    assert rc == 0
    field = json.loads((out_dir / "field_config_resolved.json").read_text())
    assert field["start_source"] == "plan_dir"
    assert field["resolved_goal_x_m"] == 4.0
    assert field["resolved_goal_y_m"] == -1.2


def test_preview_and_run_print_same_resolved_field_config(tmp_path: Path, capsys) -> None:
    preview_dir = tmp_path / "preview"
    run_dir = tmp_path / "run"
    rc = cli.main([*_RELATIVE_PREVIEW_ARGS, "--print-field-config", "true", "--out-dir", str(preview_dir)])
    assert rc == 0
    preview_out = capsys.readouterr().out
    rc = cli.main(
        [
            "execute-plan",
            "--plan-dir",
            str(preview_dir),
            "--print-plan",
            "--print-field-config",
            "true",
            "--out-dir",
            str(run_dir),
        ]
    )
    assert rc == 0
    run_out = capsys.readouterr().out
    for expected in (
        "Field configuration:",
        "goal_mode=relative_enu",
        "resolved_goal_x_m=4.0",
        "resolved_goal_y_m=-1.2",
        "workspace_width_m=1.2",
        "step_spacing_m=0.25",
    ):
        assert expected in preview_out
        assert expected in run_out
