import math

import pytest

from gps_coverage_core.planner import (
    generate_coverage_path,
    generate_lawnmower_path,
    lane_offsets_for_sweep_width,
    latlon_to_xy,
    xy_to_latlon,
)


POINT_A = {"lat": 35.123456, "lon": 129.123456}
POINT_B = {"lat": 35.123456, "lon": 129.124556}


def _lanes(path: list[dict[str, float | int]]) -> dict[int, list[dict[str, float | int]]]:
    grouped: dict[int, list[dict[str, float | int]]] = {}
    for waypoint in path:
        grouped.setdefault(int(waypoint["lane"]), []).append(waypoint)
    return grouped


def test_latlon_xy_roundtrip() -> None:
    lat, lon = 35.123956, 129.123956

    x_m, y_m = latlon_to_xy(lat, lon, POINT_A["lat"], POINT_A["lon"])
    roundtrip_lat, roundtrip_lon = xy_to_latlon(x_m, y_m, POINT_A["lat"], POINT_A["lon"])

    assert roundtrip_lat == pytest.approx(lat, abs=1e-9)
    assert roundtrip_lon == pytest.approx(lon, abs=1e-9)


def test_path_has_two_waypoints_per_default_lane() -> None:
    path = generate_lawnmower_path(POINT_A, POINT_B, spacing_m=5.0)

    assert len(path) == 2 * 4
    assert [waypoint["order"] for waypoint in path] == list(range(len(path)))
    assert all(set(waypoint) == {"lat", "lon", "x", "y", "lane", "order"} for waypoint in path)


def test_lanes_alternate_direction() -> None:
    path = generate_lawnmower_path(POINT_A, POINT_B, spacing_m=5.0, num_lanes=3)
    lanes = _lanes(path)

    assert float(lanes[0][0]["x"]) < float(lanes[0][1]["x"])
    assert float(lanes[1][0]["x"]) > float(lanes[1][1]["x"])
    assert float(lanes[2][0]["x"]) < float(lanes[2][1]["x"])


def test_spacing_is_approximately_correct() -> None:
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
    with pytest.raises(ValueError, match="positive"):
        generate_lawnmower_path(POINT_A, POINT_B, spacing_m=0.0)


def test_identical_points_raise_value_error() -> None:
    with pytest.raises(ValueError, match="must not be identical"):
        generate_lawnmower_path(POINT_A, POINT_A, spacing_m=5.0)


def test_lane_offsets_include_far_sweep_edge() -> None:
    assert lane_offsets_for_sweep_width(12.0, 5.0) == pytest.approx(
        [0.0, 5.0, 10.0, 12.0]
    )


def test_coverage_path_uses_sweep_width_and_alternates_direction() -> None:
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
    with pytest.raises(ValueError, match="sweep_width_m must be positive"):
        generate_coverage_path(POINT_A, POINT_B, sweep_width_m=0.0, lane_spacing_m=5.0)

    with pytest.raises(ValueError, match="lane_spacing_m must be positive"):
        generate_coverage_path(POINT_A, POINT_B, sweep_width_m=10.0, lane_spacing_m=0.0)

    with pytest.raises(ValueError, match="speed_mps must be positive"):
        generate_coverage_path(POINT_A, POINT_B, 10.0, 5.0, speed_mps=0.0)
