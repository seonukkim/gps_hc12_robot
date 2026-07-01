"""스테이션 커버리지 경로 계획 CLI 계약 테스트 / Contract test for the station coverage-path planning CLI.

목적/역할:
    `scripts.station.plan_coverage_path` CLI가 두 코너점(A,B)으로 미션 산출물
    (mission.json / mission.csv / preview.png)을 만드는 계약을 잠근다. 기본 모드는
    corner-rectangle(코너 직사각형), 대안은 baseline-width(스윕폭 지정)이다.

시스템 내 위치:
    planner.py 라우팅 코어를 감싸 파일 산출물을 만드는 상류 CLI다. 이 CLI는 **PC측 dry-run
    전용**이라 시리얼 포트를 열지 않고 로버 명령을 보내지 않는다 — 그 안전 메타/문구를 검증한다.

핵심 개념·불변식:
    - mission.json: metadata(dry_run=True, sends_rover_commands=False, planner_mode),
      local_origin, input_points_local(A=원점 0,0 / B=반대 코너), waypoints, coverage_boundary(5점),
      safety 배너 목록. 이 스키마 전체가 계약이다.
    - 첫 웨이포인트는 A(0,0), 마지막은 B의 로컬 좌표와 정확히 일치.
    - CSV 헤더는 최소 요구 키 집합을 포함하고 행 수는 waypoint_count와 일치.
    - baseline-width 모드는 --sweep-width-m 필수(누락 시 SystemExit).

리팩토링 노트:
    JSON/CSV 스키마·safety 문구를 바꾸면 이 테스트의 상세 단언이 대거 깨진다(의도된 잠금).

Contract test: the plan_coverage_path CLI turns two corner points (A, B) into mission artifacts
(mission.json / mission.csv / preview.png). Default planner_mode is corner-rectangle; baseline-width
is the sweep-width alternative. The CLI is a PC-side dry-run only (no serial port, no rover
commands) — the test asserts that safety metadata/banner plus the full JSON schema, CSV header, A=
origin / last=B invariants, and the baseline-width required-argument rule.
"""

import csv
import json

import pytest

from scripts.station import plan_coverage_path


def test_parse_lat_lon() -> None:
    """"LAT,LON" 문자열이 {lat, lon} dict로 파싱됨을 검증. / A "LAT,LON" string parses into a {lat, lon} dict."""
    assert plan_coverage_path.parse_lat_lon("35.1,129.2") == {"lat": 35.1, "lon": 129.2}


def test_plan_coverage_path_writes_required_outputs(tmp_path) -> None:
    """corner-rectangle 미션의 JSON/CSV/PNG 산출과 스키마·안전메타·A→B 불변식 전체 검증. / Full corner-rectangle mission: JSON/CSV/PNG outputs plus schema, safety metadata, and A->B invariants."""
    result = plan_coverage_path.main(
        [
            "--point-a",
            "35.123456,129.123456",
            "--point-b",
            "35.123636,129.123756",
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
    assert mission["metadata"]["planner_mode"] == "corner-rectangle"
    assert mission["inputs"]["sweep_width_m"] is None
    assert mission["summary"]["sweep_width_m"] is None
    assert mission["summary"]["rectangle_x_extent_m"] > 0
    assert mission["summary"]["rectangle_y_extent_m"] > 0
    assert mission["local_origin"] == {
        "lat": 35.123456,
        "lon": 129.123456,
        "description": "point_a",
    }
    assert mission["input_points_local"]["point_a"] == {
        "x_m": 0.0,
        "y_m": 0.0,
        "role": "start corner",
    }
    point_b_local = mission["input_points_local"]["point_b"]
    assert point_b_local["role"] == "opposite/end corner"
    assert mission["summary"]["lane_count"] >= 2
    assert mission["summary"]["waypoint_count"] == len(mission["waypoints"])
    assert mission["waypoints"][0]["x_m"] == 0.0
    assert mission["waypoints"][0]["y_m"] == 0.0
    assert mission["waypoints"][-1]["x_m"] == point_b_local["x_m"]
    assert mission["waypoints"][-1]["y_m"] == point_b_local["y_m"]
    assert len(mission["coverage_boundary"]) == 5
    assert mission["safety"] == [
        "PC/Mac-side dry-run only",
        "no serial port opened",
        "no HC-12 frames sent",
        "no rover commands generated",
    ]

    with csv_path.open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert len(rows) == mission["summary"]["waypoint_count"]
    assert set(rows[0]) >= {"index", "lat", "lon", "x_m", "y_m", "segment_type", "notes"}
    assert rows[0]["index"] == "0"
    assert rows[0]["segment_type"] == "lane_start"
    assert rows[1]["segment_type"] == "lane_end"
    assert rows[0]["speed_mps"] == "0.4"
    assert float(rows[-1]["x_m"]) == point_b_local["x_m"]
    assert float(rows[-1]["y_m"]) == point_b_local["y_m"]


def test_plan_coverage_path_baseline_width_mode_uses_sweep_width(tmp_path) -> None:
    """baseline-width 모드가 planner_mode와 입력 sweep_width_m를 미션에 반영함을 검증. / baseline-width mode records planner_mode and the input sweep_width_m in the mission."""
    result = plan_coverage_path.main(
        [
            "--planner-mode",
            "baseline-width",
            "--point-a",
            "35.123456,129.123456",
            "--point-b",
            "35.123456,129.124556",
            "--sweep-width-m",
            "10",
            "--lane-spacing-m",
            "5",
            "--mission-name",
            "baseline_mission",
            "--output-dir",
            str(tmp_path),
        ]
    )

    assert result == 0
    mission = json.loads((tmp_path / "baseline_mission" / "mission.json").read_text())
    assert mission["metadata"]["planner_mode"] == "baseline-width"
    assert mission["inputs"]["sweep_width_m"] == 10


def test_plan_coverage_path_baseline_width_requires_sweep_width() -> None:
    """baseline-width 모드에서 --sweep-width-m 누락 시 SystemExit 발생 검증. / Omitting --sweep-width-m in baseline-width mode raises SystemExit."""
    with pytest.raises(SystemExit, match="--sweep-width-m is required"):
        plan_coverage_path.main(
            [
                "--planner-mode",
                "baseline-width",
                "--point-a",
                "35.123456,129.123456",
                "--point-b",
                "35.123456,129.124556",
                "--lane-spacing-m",
                "5",
                "--mission-name",
                "missing_sweep_width",
            ]
        )
