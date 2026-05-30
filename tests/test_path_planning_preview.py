import csv

import pytest

from gps_coverage_core.geo import GeoPoint
from tools import path_planning_preview


def test_build_waypoints_includes_start_spacing_and_goal() -> None:
    start = GeoPoint(lat=35.571083, lon=129.187290)
    goal = GeoPoint(lat=35.570932, lon=129.187338)

    waypoints = path_planning_preview.build_waypoints(
        start=start,
        goal=goal,
        spacing_m=2.0,
    )

    assert len(waypoints) > 2
    assert waypoints[0]["segment_type"] == "start"
    assert waypoints[-1]["segment_type"] == "goal"
    assert waypoints[0]["distance_from_start_m"] == pytest.approx(0.0)
    assert float(waypoints[-1]["distance_from_start_m"]) > 2.0
    assert float(waypoints[-1]["lat"]) == pytest.approx(goal.lat, abs=1e-7)
    assert float(waypoints[-1]["lon"]) == pytest.approx(goal.lon, abs=1e-7)


def test_path_planning_preview_writes_outputs(tmp_path) -> None:
    result = path_planning_preview.main(
        [
            "--start-lat",
            "35.571083",
            "--start-lon",
            "129.187290",
            "--goal-lat",
            "35.570932",
            "--goal-lon",
            "129.187338",
            "--spacing-m",
            "2.0",
            "--out-dir",
            str(tmp_path),
        ]
    )

    assert result == 0
    csv_path = tmp_path / "waypoints.csv"
    summary_path = tmp_path / "summary.md"
    assert csv_path.exists()
    assert summary_path.exists()

    summary = summary_path.read_text(encoding="utf-8")
    assert "preview-only" in summary
    assert "No HC-12 commands are sent" in summary
    assert "No rover motor commands are generated" in summary
    assert "heading/course estimation" in summary

    with csv_path.open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert rows[0]["segment_type"] == "start"
    assert rows[-1]["segment_type"] == "goal"
    assert set(rows[0]) == {
        "index",
        "lat",
        "lon",
        "x_m",
        "y_m",
        "distance_from_start_m",
        "segment_type",
    }
