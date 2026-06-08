from __future__ import annotations

import json
from pathlib import Path

import pytest

from tools.physical_path_planning import cli, preview


def _coverage_preview_args(out_dir: Path) -> list[str]:
    return [
        "preview",
        "--start-lat",
        "35.5709000",
        "--start-lon",
        "129.1871000",
        "--goal-mode",
        "relative_enu",
        "--goal-east-m",
        "1.2",
        "--goal-north-m",
        "1.2",
        "--workspace-width-m",
        "1.2",
        "--step-spacing-m",
        "0.4",
        "--out-dir",
        str(out_dir),
    ]


def test_coverage_lawnmower_relative_enu_creates_alternating_lanes() -> None:
    plan = preview.build_preview(
        start_lat=35.5709,
        start_lon=129.1871,
        goal_mode="relative_enu",
        goal_east_m=1.2,
        goal_north_m=1.2,
        workspace_width_m=1.2,
        step_spacing_m=0.4,
    )

    assert plan["path_shape"] == "coverage_lawnmower"
    lanes = [segment for segment in plan["segments"] if str(segment["segment_type"]).endswith("_lane")]
    connectors = [segment for segment in plan["segments"] if segment["segment_type"] == "path_connector"]
    assert len(lanes) == 4
    assert len(connectors) == 3
    assert lanes[0]["start_x_m"] == pytest.approx(0.0)
    assert lanes[0]["start_y_m"] == pytest.approx(0.0)
    assert lanes[0]["end_x_m"] == pytest.approx(1.2)
    assert lanes[0]["end_y_m"] == pytest.approx(0.0)
    assert lanes[1]["start_x_m"] == pytest.approx(1.2)
    assert lanes[1]["end_x_m"] == pytest.approx(0.0)
    assert lanes[2]["start_x_m"] == pytest.approx(0.0)
    assert lanes[2]["end_x_m"] == pytest.approx(1.2)
    assert lanes[3]["start_x_m"] == pytest.approx(1.2)
    assert lanes[3]["end_x_m"] == pytest.approx(0.0)
    for lane in lanes:
        assert lane["start_x_m"] == pytest.approx(lane["end_x_m"]) or lane["start_y_m"] == pytest.approx(lane["end_y_m"])
    assert plan["expected_sweep_style"] == "lawnmower_ㄹ"


def test_coverage_lawnmower_preview_writes_required_artifacts(tmp_path: Path) -> None:
    rc = cli.main(_coverage_preview_args(tmp_path))
    assert rc == 0

    assert (tmp_path / "preview_current_goal_rectangle_path.png").exists()
    assert (tmp_path / "preview_overview.png").exists()
    assert (tmp_path / "field_config_resolved.json").exists()
    assert (tmp_path / "planned_segments.csv").exists()
    assert (tmp_path / "planned_segments.json").exists()
    assert (tmp_path / "summary.md").exists()
    assert (tmp_path / "summary.json").exists()
    field = json.loads((tmp_path / "field_config_resolved.json").read_text())
    assert field["path_shape"] == "coverage_lawnmower"
    assert field["goal_x_m"] == pytest.approx(1.2)
    assert field["goal_y_m"] == pytest.approx(1.2)
    assert field["connector_count"] == 3
    assert field["expected_sweep_style"] == "lawnmower_ㄹ"
    assert "preview_current_goal_rectangle_path" in field["image_paths"]


def test_inspect_plan_detects_missing_preview_images(tmp_path: Path) -> None:
    plan_dir = tmp_path / "preview"
    rc = cli.main(_coverage_preview_args(plan_dir))
    assert rc == 0
    (plan_dir / "preview_overview.png").unlink()

    rc = cli.main(["inspect-plan", "--plan-dir", str(plan_dir)])

    assert rc == 1
    summary = json.loads((plan_dir / "summary.json").read_text())
    assert summary["mode"] == "inspect-plan"
    assert summary["reason"] == "PREVIEW_IMAGE_MISSING"
    assert any("preview_overview.png" in path for path in summary["missing_preview_images"])


def test_execute_plan_print_plan_uses_saved_planned_segments(tmp_path: Path) -> None:
    plan_dir = tmp_path / "preview"
    out_dir = tmp_path / "execute"
    rc = cli.main(_coverage_preview_args(plan_dir))
    assert rc == 0
    saved_segments = json.loads((plan_dir / "planned_segments.json").read_text())
    first_only = [saved_segments[0]]
    (plan_dir / "planned_segments.json").write_text(json.dumps(first_only, indent=2) + "\n")

    rc = cli.main(["execute-plan", "--plan-dir", str(plan_dir), "--print-plan", "--out-dir", str(out_dir)])

    assert rc == 0
    summary = json.loads((out_dir / "summary.json").read_text())
    assert summary["start_source"] == "plan_dir"
    assert summary["segment_count"] == 1
    assert summary["field_config"]["path_shape"] == "coverage_lawnmower"


def test_diagonal_rectangle_serpentine_available_but_not_default(capsys: pytest.CaptureFixture[str]) -> None:
    default_plan = preview.build_preview(
        start_lat=35.5709,
        start_lon=129.1871,
        goal_mode="relative_enu",
        goal_east_m=1.2,
        goal_north_m=1.2,
        workspace_width_m=1.2,
        step_spacing_m=0.4,
    )
    assert default_plan["path_shape"] == "coverage_lawnmower"

    parser = cli.build_parser()
    args = parser.parse_args(
        [
            "preview",
            "--start-lat",
            "35.5709000",
            "--start-lon",
            "129.1871000",
            "--goal-mode",
            "relative_enu",
            "--goal-east-m",
            "1.2",
            "--goal-north-m",
            "1.2",
            "--workspace-width-m",
            "1.2",
            "--step-spacing-m",
            "0.4",
            "--path-shape",
            "diagonal_rectangle_serpentine",
        ]
    )
    cli.resolve_plan(args, cli.resolve_calibration(args))
    out = capsys.readouterr().out
    assert "diagonal_rectangle_serpentine follows the A-B diagonal frame" in out
