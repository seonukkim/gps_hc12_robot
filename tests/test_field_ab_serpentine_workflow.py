from __future__ import annotations

import csv
import json
from pathlib import Path

from tools import capture_field_ab_points
from tools import field_ab_to_serpentine
from tools import path_no_motion_validation


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
        "gps_ok=true bmi160_ok=true rc_manual_ok=true x_m=8.0 y_m=0.0 heading_deg=0 physical_output_active=false\n",
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
            "--out-dir",
            str(tmp_path / "validation"),
        ]
    ) == 0

    summary = (tmp_path / "validation" / "summary.md").read_text(encoding="utf-8")
    assert "gps_ok: `True`" in summary
    assert "bmi160_ok: `True`" in summary
    assert "rc_manual_ok: `True`" in summary
    assert "ready_for_outdoor_no_motion_validation: `True`" in summary
    with (tmp_path / "validation" / "no_motion_validation.csv").open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert rows
    assert {row["motor_command_generated"] for row in rows} == {"False"}
    assert {row["physical_output_active"] for row in rows} == {"False"}
