"""커버리지 경로 계획 코어 계약 테스트 / Contract test for the coverage-path planning core.

목적/역할:
    `gps_coverage_core.planner`의 라우팅 코어를 잠근다. 세 가지 경로 생성기와 좌표 변환의
    핵심 불변식을 검증한다: (1) 근사 좌표 왕복(latlon_to_xy/xy_to_latlon), (2) 사행
    (boustrophedon) 레인의 교대 방향과 간격, (3) 레인 오프셋이 항상 경계를 포함,
    (4) 코너-직사각형 경로가 정확히 B에서 끝나고 필요 시 final_connector를 덧붙임,
    (5) 불량 입력(간격/속도 0, A==B)의 ValueError 거부, (6) 웨이포인트 dict 키 집합.

시스템 내 위치:
    planner.py는 파이프라인 맨 앞 "경로 계획" 단계다. 상류 CLI(plan_coverage_path)와
    미리보기 도구가 여기 함수를 호출하므로, 반환 dict 키·경로 형태가 계약이다.

핵심 개념·불변식:
    - 로컬 프레임: x=East, y=North. 사행은 레인 홀짝에 따라 시작/끝을 뒤집어 왕복한다.
    - lane_offsets_*는 near/far 경계를 항상 포함해 커버리지 끝단 누락을 막는다.
    - 좌표 왕복 허용 오차 1e-9도, 간격 검증 허용 오차 ±0.05 m.

리팩토링 노트:
    planner의 근사 좌표계는 geo.py의 측지선 정밀 좌표와 다르다(섞지 말 것). 반환 dict 키를
    바꾸면 이 테스트와 상류 CSV/미리보기가 함께 깨진다.

Contract test for planner.py (the routing core): approximate coord round-trip, alternating
serpentine lane direction + spacing, lane offsets always including the boundary, corner-rectangle
paths ending exactly at B (with a final_connector when needed), ValueError rejection of bad inputs
(zero spacing/speed, identical A/B), and the waypoint-dict key set. planner uses a fast
equirectangular frame (x=East, y=North) distinct from geo.py's precise geodesics.
"""

import math

import pytest

from gps_coverage_core.planner import (
    generate_corner_rectangle_path_local,
    generate_coverage_path,
    generate_lawnmower_path,
    lane_offsets_for_extent,
    lane_offsets_for_sweep_width,
    latlon_to_xy,
    xy_to_latlon,
)


# ── 테스트 픽스처: A/B 기준점 / Test fixtures: reference points A and B ──
# 부산 인근, 경도만 다른 두 점(≈100 m 동서 간격)으로 대부분의 경로 테스트를 구동한다.
# / Two points near Busan differing only in longitude (~100 m E-W) drive most path tests.
POINT_A = {"lat": 35.123456, "lon": 129.123456}
POINT_B = {"lat": 35.123456, "lon": 129.124556}


# ── 헬퍼: 레인별 웨이포인트 그룹핑 / Helper: group waypoints by lane index ──
def _lanes(path: list[dict[str, float | int]]) -> dict[int, list[dict[str, float | int]]]:
    """경로를 lane 인덱스별 웨이포인트 목록으로 묶는다. / Group a path into {lane: [waypoints]} for per-lane assertions."""
    grouped: dict[int, list[dict[str, float | int]]] = {}
    for waypoint in path:
        grouped.setdefault(int(waypoint["lane"]), []).append(waypoint)
    return grouped


def test_latlon_xy_roundtrip() -> None:
    """근사 좌표 변환의 위경도⇄미터 왕복 보존(≤1e-9도) 검증. / Approximate latlon<->xy round-trips within 1e-9 deg."""
    lat, lon = 35.123956, 129.123956

    x_m, y_m = latlon_to_xy(lat, lon, POINT_A["lat"], POINT_A["lon"])
    roundtrip_lat, roundtrip_lon = xy_to_latlon(x_m, y_m, POINT_A["lat"], POINT_A["lon"])

    assert roundtrip_lat == pytest.approx(lat, abs=1e-9)
    assert roundtrip_lon == pytest.approx(lon, abs=1e-9)


def test_path_has_two_waypoints_per_default_lane() -> None:
    """기본 4레인 경로: 레인당 2점, order 연속, 키 집합 고정 검증. / Default lawnmower: 2 pts/lane, sequential order, fixed key set."""
    path = generate_lawnmower_path(POINT_A, POINT_B, spacing_m=5.0)

    assert len(path) == 2 * 4
    assert [waypoint["order"] for waypoint in path] == list(range(len(path)))
    assert all(set(waypoint) == {"lat", "lon", "x", "y", "lane", "order"} for waypoint in path)


def test_lanes_alternate_direction() -> None:
    """사행 규칙: 짝수 레인은 +x, 홀수 레인은 -x 방향으로 진행. / Serpentine: even lanes go +x, odd lanes go -x (back-and-forth)."""
    path = generate_lawnmower_path(POINT_A, POINT_B, spacing_m=5.0, num_lanes=3)
    lanes = _lanes(path)

    assert float(lanes[0][0]["x"]) < float(lanes[0][1]["x"])
    assert float(lanes[1][0]["x"]) > float(lanes[1][1]["x"])
    assert float(lanes[2][0]["x"]) < float(lanes[2][1]["x"])


def test_spacing_is_approximately_correct() -> None:
    """인접 레인 간 수직 간격이 요청한 spacing과 ±0.05 m 이내 일치 검증. / Adjacent-lane gap matches requested spacing within ±0.05 m."""
    spacing_m = 7.5
    path = generate_lawnmower_path(POINT_A, POINT_B, spacing_m=spacing_m, num_lanes=3)
    lanes = _lanes(path)

    a_like_points = [min(lanes[lane], key=lambda waypoint: float(waypoint["x"])) for lane in range(3)]
    distances = [
        math.hypot(
            float(curr["x"]) - float(prev["x"]),
            float(curr["y"]) - float(prev["y"]),
        )
        for prev, curr in zip(a_like_points, a_like_points[1:])
    ]

    assert distances == pytest.approx([spacing_m, spacing_m], abs=0.05)


def test_invalid_spacing_raises_value_error() -> None:
    """간격 0(비양수)은 ValueError로 거부됨을 검증. / Zero (non-positive) spacing is rejected with ValueError."""
    with pytest.raises(ValueError, match="positive"):
        generate_lawnmower_path(POINT_A, POINT_B, spacing_m=0.0)


def test_identical_points_raise_value_error() -> None:
    """A==B(길이 0 미션)는 ValueError로 조기 거부됨을 검증. / Identical A/B (zero-length mission) is rejected early."""
    with pytest.raises(ValueError, match="must not be identical"):
        generate_lawnmower_path(POINT_A, POINT_A, spacing_m=5.0)


def test_lane_offsets_include_far_sweep_edge() -> None:
    """스윕폭 오프셋이 간격 배수로 안 나눠져도 far edge(12 m)를 포함함을 검증. / Sweep-width offsets always include the far edge even when not a spacing multiple."""
    assert lane_offsets_for_sweep_width(12.0, 5.0) == pytest.approx(
        [0.0, 5.0, 10.0, 12.0]
    )


def test_corner_rectangle_offsets_include_boundary() -> None:
    """extent 오프셋이 경계를 포함(정확히 나뉘면 중복 없이, 아니면 잔여 추가). / Extent offsets include the boundary — no dup when exact, remainder appended otherwise."""
    assert lane_offsets_for_extent(20.0, 5.0) == pytest.approx(
        [0.0, 5.0, 10.0, 15.0, 20.0]
    )
    assert lane_offsets_for_extent(22.0, 5.0) == pytest.approx(
        [0.0, 5.0, 10.0, 15.0, 20.0, 22.0]
    )


def test_corner_rectangle_path_starts_at_a_and_ends_at_b_when_possible() -> None:
    """세로변이 간격으로 딱 나뉘면 경로는 A(0,0)에서 시작해 B에서 끝난다. / When the Y-extent divides evenly, the path starts at A(0,0) and ends exactly at B."""
    path = generate_corner_rectangle_path_local(27.0, 20.0, lane_spacing_m=5.0)

    assert (path[0]["x_m"], path[0]["y_m"]) == pytest.approx((0.0, 0.0))
    assert (path[-1]["x_m"], path[-1]["y_m"]) == pytest.approx((27.0, 20.0))
    assert path[-1]["notes"] == "point B final target"
    assert [point["offset_m"] for point in path[::2]] == pytest.approx(
        [0.0, 5.0, 10.0, 15.0, 20.0]
    )


def test_corner_rectangle_path_adds_final_connector_when_needed() -> None:
    """세로변에 잔여가 있으면 final_connector 세그먼트로 정확히 B에서 끝냄. / A leftover Y-remainder appends a final_connector so the path still ends exactly at B."""
    path = generate_corner_rectangle_path_local(27.0, 22.0, lane_spacing_m=5.0)

    assert [point["offset_m"] for point in path[::2]][:6] == pytest.approx(
        [0.0, 5.0, 10.0, 15.0, 20.0, 22.0]
    )
    assert (path[-1]["x_m"], path[-1]["y_m"]) == pytest.approx((27.0, 22.0))
    assert path[-1]["segment_type"] == "final_connector"
    assert "point B" in str(path[-1]["notes"])


def test_coverage_path_uses_sweep_width_and_alternates_direction() -> None:
    """스윕폭 기반 커버리지: 레인 수·교대 방향·far edge 오프셋·균일 속도 검증. / Sweep-width coverage: lane count, alternating direction, far-edge offset, uniform speed."""
    path = generate_coverage_path(
        POINT_A,
        POINT_B,
        sweep_width_m=12.0,
        lane_spacing_m=5.0,
        speed_mps=0.4,
    )
    lanes = _lanes(path)

    assert len(path) == 8
    assert set(lanes) == {0, 1, 2, 3}
    assert [waypoint["order"] for waypoint in path] == list(range(len(path)))
    assert all(waypoint["speed_mps"] == pytest.approx(0.4) for waypoint in path)
    assert float(lanes[0][0]["x_m"]) < float(lanes[0][1]["x_m"])
    assert float(lanes[1][0]["x_m"]) > float(lanes[1][1]["x_m"])
    assert float(lanes[3][0]["offset_m"]) == pytest.approx(12.0)


def test_coverage_path_rejects_invalid_inputs() -> None:
    """스윕폭·레인간격·속도 각각의 비양수 입력이 고유 메시지로 거부됨을 검증. / Non-positive sweep width / lane spacing / speed each raise their own ValueError."""
    with pytest.raises(ValueError, match="sweep_width_m must be positive"):
        generate_coverage_path(POINT_A, POINT_B, sweep_width_m=0.0, lane_spacing_m=5.0)

    with pytest.raises(ValueError, match="lane_spacing_m must be positive"):
        generate_coverage_path(POINT_A, POINT_B, sweep_width_m=10.0, lane_spacing_m=0.0)

    with pytest.raises(ValueError, match="speed_mps must be positive"):
        generate_coverage_path(POINT_A, POINT_B, 10.0, 5.0, speed_mps=0.0)
