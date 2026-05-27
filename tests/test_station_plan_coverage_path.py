import csv
import json

from scripts.station import plan_coverage_path


def test_parse_lat_lon() -> None:
    assert plan_coverage_path.parse_lat_lon("35.1,129.2") == {"lat": 35.1, "lon": 129.2}


def test_plan_coverage_path_writes_required_outputs(tmp_path) -> None:
    result = plan_coverage_path.main(
        [
            "--point-a",
            "35.123456,129.123456",
            "--point-b",
            "35.123456,129.124556",
            "--sweep-width-m",
            "10",
            "--lane-spacing-m",
            "5",
            "--speed-mps",
            "0.4",
            "--mission-name",
            "pytest_mission",
            "--output-dir",
            str(tmp_path),
        ]
    )

    assert result == 0

    mission_dir = tmp_path / "pytest_mission"
    json_path = mission_dir / "mission.json"
    csv_path = mission_dir / "mission.csv"
    preview_path = mission_dir / "preview.png"

    assert json_path.exists()
    assert csv_path.exists()
    assert preview_path.exists()

    mission = json.loads(json_path.read_text(encoding="utf-8"))
    assert mission["metadata"]["dry_run"] is True
    assert mission["metadata"]["sends_rover_commands"] is False
    assert mission["local_origin"] == {
        "lat": 35.123456,
        "lon": 129.123456,
        "description": "point_a",
    }
    assert mission["summary"]["lane_count"] == 3
    assert mission["summary"]["waypoint_count"] == 6
    assert len(mission["coverage_boundary"]) == 5
    assert mission["safety"] == [
        "PC/Mac-side dry-run only",
        "no serial port opened",
        "no HC-12 frames sent",
        "no rover commands generated",
    ]

    with csv_path.open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert len(rows) == 6
    assert set(rows[0]) >= {"index", "lat", "lon", "x_m", "y_m", "segment_type"}
    assert rows[0]["index"] == "0"
    assert rows[0]["segment_type"] == "lane_start"
    assert rows[1]["segment_type"] == "lane_end"
    assert rows[0]["speed_mps"] == "0.4"
