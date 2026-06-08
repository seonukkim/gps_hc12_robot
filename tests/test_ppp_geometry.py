"""Geometry contract tests for the consolidated path-planning package.

These lock the A->B-diagonal rectangle guards and the four goal modes against
``tools.physical_path_planning.geometry``.
"""
from __future__ import annotations

import pytest

from tools.physical_path_planning import geometry


def test_rectangle_requires_positive_width() -> None:
    with pytest.raises(ValueError, match="workspace width is required"):
        geometry.build_rectangle_from_diagonal_and_width(
            start_lat=35.0,
            start_lon=129.0,
            goal_lat=35.00008,
            goal_lon=129.00008,
            width_m=0.0,
            step_spacing_m=0.5,
        )


def test_rectangle_requires_diagonal_longer_than_width() -> None:
    with pytest.raises(ValueError, match="A-B diagonal length must be larger"):
        geometry.build_rectangle_from_diagonal_and_width(
            start_lat=35.0,
            start_lon=129.0,
            goal_lat=35.000001,
            goal_lon=129.000001,
            width_m=10.0,
            step_spacing_m=0.5,
        )


def test_goal_mode_absolute() -> None:
    goal, context = geometry.resolve_goal_point(
        start_lat=35.0,
        start_lon=129.0,
        goal_mode="absolute",
        goal_lat=35.00008,
        goal_lon=129.00008,
    )
    assert goal.lat == pytest.approx(35.00008)
    assert goal.lon == pytest.approx(129.00008)
    assert context["goal_mode"] == "absolute"


def test_goal_mode_relative_enu() -> None:
    goal, _ = geometry.resolve_goal_point(
        start_lat=35.0,
        start_lon=129.0,
        goal_mode="relative_enu",
        goal_east_m=4.0,
        goal_north_m=-1.2,
    )
    x, y = geometry.goal_to_local(35.0, 129.0, goal.lat, goal.lon)
    assert x == pytest.approx(4.0, abs=1e-6)
    assert y == pytest.approx(-1.2, abs=1e-6)


def test_goal_mode_relative_latlon() -> None:
    goal, _ = geometry.resolve_goal_point(
        start_lat=35.0,
        start_lon=129.0,
        goal_mode="relative_latlon",
        goal_dlat=0.000010,
        goal_dlon=0.000030,
    )
    assert goal.lat == pytest.approx(35.000010)
    assert goal.lon == pytest.approx(129.000030)


def test_goal_mode_bearing_distance() -> None:
    goal, _ = geometry.resolve_goal_point(
        start_lat=35.0,
        start_lon=129.0,
        goal_mode="bearing_distance",
        goal_bearing_deg=90.0,
        goal_distance_m=4.0,
    )
    x, y = geometry.goal_to_local(35.0, 129.0, goal.lat, goal.lon)
    assert x == pytest.approx(4.0, abs=1e-6)
    assert y == pytest.approx(0.0, abs=1e-6)
