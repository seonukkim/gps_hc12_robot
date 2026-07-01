"""시작→목표 경로 미리보기 도구 계약 테스트 / Contract test for the start->goal path-preview tool.

목적/역할:
    `tools.path_planning_preview`가 시작/목표 위경도 두 점 사이 직선을 일정 간격 웨이포인트로
    나누고, CSV/Markdown 산출물을 만드는지 잠근다. 이 도구는 **미리보기 전용**이라 HC-12나
    로버 명령을 보내지 않으며, summary가 그 안전 배너를 명시하는지도 검증한다.

시스템 내 위치:
    coverage 파이프라인의 "오프라인 미리보기" 도구다. 코어 geo 유틸(GeoPoint 등)로 좌표를
    변환한다. 이 테스트는 build_waypoints 계약과 main()의 파일 산출/헤더 계약을 함께 고정한다.

핵심 개념·불변식:
    - 첫 웨이포인트 segment_type="start"(거리 0), 마지막 "goal"(정확히 목표 좌표).
    - CSV 헤더 키 집합은 산출물 계약 — CSV_FIELDS와 함께 바뀌어야 한다.
    - summary는 "preview-only" 및 무전송 배너 문구를 반드시 포함(안전 불변식).

리팩토링 노트:
    도구가 미리보기 전용이라는 문구/헤더를 바꾸면 이 테스트의 문자열 단언이 먼저 깨진다.

Contract test: path_planning_preview splits the straight line between two lat/lon points into evenly
spaced waypoints and writes CSV + Markdown. It is preview-only (sends no HC-12 / rover commands);
the test also asserts the summary carries that safety banner. Locks build_waypoints (first="start"
at distance 0, last="goal" at the goal coords) and main()'s output files + CSV header key set.
"""

import csv

import pytest

from gps_coverage_core.geo import GeoPoint
from tools import path_planning_preview


def test_build_waypoints_includes_start_spacing_and_goal() -> None:
    """웨이포인트가 start(거리0)로 시작해 goal(목표좌표)로 끝나며 중간점을 가짐. / Waypoints start at "start" (dist 0) and end at "goal" (exact goal), with points in between."""
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
    """main()이 CSV/summary를 쓰고, summary 안전 배너와 CSV 헤더 계약을 지킴. / main() writes CSV + summary; summary carries the preview-only/no-send banner and CSV header matches the contract."""
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
