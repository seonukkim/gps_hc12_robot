import pytest

from gps_coverage_core.geo import GeoPoint
from gps_coverage_core.planner import generate_lawnmower_path


def test_invalid_spacing() -> None:
    with pytest.raises(ValueError, match="positive"):
        generate_lawnmower_path(
            GeoPoint(lat=35.123456, lon=129.123456),
            GeoPoint(lat=35.124000, lon=129.124000),
            0.0,
        )


def test_too_small_area() -> None:
    with pytest.raises(ValueError, match="too small"):
        generate_lawnmower_path(
            GeoPoint(lat=35.123456, lon=129.123456),
            GeoPoint(lat=35.123456, lon=129.123456),
            5.0,
        )


def test_positive_waypoint_count() -> None:
    path = generate_lawnmower_path(
        GeoPoint(lat=35.123456, lon=129.123456),
        GeoPoint(lat=35.124156, lon=129.124256),
        5.0,
    )

    assert len(path) >= 4
