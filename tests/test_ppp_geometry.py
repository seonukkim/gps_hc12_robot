"""Geometry contract tests for the consolidated path-planning package.

These lock the A->B-diagonal rectangle guards and the four goal modes against
``tools.physical_path_planning.geometry``.

목적/역할 (KO):
    통합 경로 계획 패키지의 기하(geometry) 계약을 고정한다. 두 축을 잠근다:
    (1) A->B 대각선 + 폭으로 직사각형을 만드는 빌더의 입력 가드(폭이 양수여야
    하고, A-B 대각선이 폭보다 길어야 함), (2) 네 가지 목표점(goal) 지정 모드가
    같은 시작점에 대해 기대한 지점을 산출하는지.

핵심 계약·불변식 (KO):
    - ``build_rectangle_from_diagonal_and_width``: 폭 0(또는 미지정) -> "workspace
      width is required", A-B 대각 길이 <= 폭 -> "A-B diagonal length must be
      larger" 로 ``ValueError``.
    - ``resolve_goal_point`` 의 네 모드:
        * ``absolute``       -- 절대 위/경도를 그대로 목표로.
        * ``relative_enu``   -- 동/북(m) 오프셋 -> 로컬 (x, y) 로 왕복 검증.
        * ``relative_latlon``-- 위/경도 증분(delta) 을 시작점에 더함.
        * ``bearing_distance`` -- 방위각·거리 -> 로컬 (x, y) 로 왕복 검증.
      ``goal_to_local`` 로 다시 로컬 좌표로 되돌려 대칭성을 확인한다.

Purpose (EN):
    Locks the geometry contract: the A->B-diagonal rectangle builder's input
    guards (width must be positive; the A-B diagonal must exceed the width) and
    the four ``resolve_goal_point`` modes (absolute / relative_enu /
    relative_latlon / bearing_distance), round-tripping through ``goal_to_local``
    where an ENU/bearing offset is expected.
"""
from __future__ import annotations

import pytest

from tools.physical_path_planning import geometry


def test_rectangle_requires_positive_width() -> None:
    """폭이 0/미지정이면 직사각형 빌더가 거부한다 / the rectangle builder rejects a
    zero (or absent) workspace width."""
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
    """A-B 대각 길이가 폭보다 짧으면(축퇴 형상) 거부한다 / rejects a degenerate shape
    where the A-B diagonal is not longer than the width."""
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
    """absolute 모드: 준 위/경도를 그대로 목표점으로 반환 / absolute mode returns the
    supplied lat/lon verbatim as the goal point."""
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
    """relative_enu 모드: 동/북(m) 오프셋이 로컬 (x, y) 로 왕복 일치 / relative_enu
    mode: an east/north (m) offset round-trips to local ``(x, y)``."""
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
    """relative_latlon 모드: 위/경도 증분을 시작점에 더한다 / relative_latlon mode
    adds a lat/lon delta to the start point."""
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
    """bearing_distance 모드: 방위 90도·거리 4m -> 정확히 동쪽 4m 지점 / bearing_
    distance mode: bearing 90deg, distance 4m maps to exactly 4m due east."""
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
