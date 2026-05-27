from __future__ import annotations

import math
from collections.abc import Mapping
from typing import Any

EARTH_RADIUS_M = 6_371_000.0
DEFAULT_NUM_LANES = 4


def latlon_to_xy(lat: float, lon: float, lat0: float, lon0: float) -> tuple[float, float]:
    """Convert latitude/longitude to local x/y meters using an equirectangular approximation."""
    lat0_rad = math.radians(lat0)
    x_m = EARTH_RADIUS_M * math.radians(lon - lon0) * math.cos(lat0_rad)
    y_m = EARTH_RADIUS_M * math.radians(lat - lat0)
    return x_m, y_m


def xy_to_latlon(x: float, y: float, lat0: float, lon0: float) -> tuple[float, float]:
    """Convert local x/y meters back to latitude/longitude."""
    lat0_rad = math.radians(lat0)
    lon_scale = math.cos(lat0_rad)
    if math.isclose(lon_scale, 0.0, abs_tol=1e-12):
        raise ValueError("reference latitude is too close to the poles")

    lat = lat0 + math.degrees(y / EARTH_RADIUS_M)
    lon = lon0 + math.degrees(x / (EARTH_RADIUS_M * lon_scale))
    return lat, lon


def _coerce_point(point: Mapping[str, Any] | Any, name: str) -> tuple[float, float]:
    if isinstance(point, Mapping):
        try:
            return float(point["lat"]), float(point["lon"])
        except KeyError as exc:
            raise ValueError(f"{name} must contain 'lat' and 'lon'") from exc

    try:
        return float(getattr(point, "lat")), float(getattr(point, "lon"))
    except AttributeError as exc:
        raise TypeError(f"{name} must provide lat/lon values") from exc


def lane_offsets_for_sweep_width(sweep_width_m: float, lane_spacing_m: float) -> list[float]:
    """Return lane offsets that include the near and far edges of the sweep width."""
    if sweep_width_m <= 0:
        raise ValueError("sweep_width_m must be positive")
    if lane_spacing_m <= 0:
        raise ValueError("lane_spacing_m must be positive")

    offsets = [0.0]
    next_offset = lane_spacing_m
    while next_offset < sweep_width_m and not math.isclose(
        next_offset, sweep_width_m, abs_tol=1e-9
    ):
        offsets.append(next_offset)
        next_offset += lane_spacing_m

    if not math.isclose(offsets[-1], sweep_width_m, abs_tol=1e-9):
        offsets.append(sweep_width_m)

    return offsets


def generate_coverage_path(
    point_a: Mapping[str, Any] | Any,
    point_b: Mapping[str, Any] | Any,
    sweep_width_m: float,
    lane_spacing_m: float,
    speed_mps: float | None = None,
) -> list[dict[str, float | int]]:
    """Generate a dry-run boustrophedon coverage path for an A/B baseline and sweep width."""
    if speed_mps is not None and speed_mps <= 0:
        raise ValueError("speed_mps must be positive when provided")

    lat_a, lon_a = _coerce_point(point_a, "point_a")
    lat_b, lon_b = _coerce_point(point_b, "point_b")

    bx_m, by_m = latlon_to_xy(lat_b, lon_b, lat_a, lon_a)
    lane_length_m = math.hypot(bx_m, by_m)
    if math.isclose(lane_length_m, 0.0, abs_tol=1e-9):
        raise ValueError("point_a and point_b must not be identical")

    ux = bx_m / lane_length_m
    uy = by_m / lane_length_m
    perp_x = -uy
    perp_y = ux

    waypoints: list[dict[str, float | int]] = []
    order = 0
    for lane, offset_m in enumerate(lane_offsets_for_sweep_width(sweep_width_m, lane_spacing_m)):
        offset_x = perp_x * offset_m
        offset_y = perp_y * offset_m

        lane_start = (offset_x, offset_y)
        lane_end = (bx_m + offset_x, by_m + offset_y)
        endpoints = (lane_start, lane_end) if lane % 2 == 0 else (lane_end, lane_start)

        for x_m, y_m in endpoints:
            lat, lon = xy_to_latlon(x_m, y_m, lat_a, lon_a)
            waypoint: dict[str, float | int] = {
                "order": order,
                "lane": lane,
                "lat": lat,
                "lon": lon,
                "x_m": x_m,
                "y_m": y_m,
                "offset_m": offset_m,
            }
            if speed_mps is not None:
                waypoint["speed_mps"] = speed_mps
            waypoints.append(waypoint)
            order += 1

    return waypoints


def generate_lawnmower_path(
    point_a: Mapping[str, Any] | Any,
    point_b: Mapping[str, Any] | Any,
    spacing_m: float,
    num_lanes: int | None = None,
) -> list[dict[str, float | int]]:
    """Generate a simple back-and-forth coverage path from the line segment A->B."""
    if spacing_m <= 0:
        raise ValueError("spacing_m must be positive")

    lane_count = DEFAULT_NUM_LANES if num_lanes is None else num_lanes
    if isinstance(lane_count, bool) or not isinstance(lane_count, int) or lane_count <= 0:
        raise ValueError("num_lanes must be a positive integer")

    lat_a, lon_a = _coerce_point(point_a, "point_a")
    lat_b, lon_b = _coerce_point(point_b, "point_b")

    bx_m, by_m = latlon_to_xy(lat_b, lon_b, lat_a, lon_a)
    lane_length_m = math.hypot(bx_m, by_m)
    if math.isclose(lane_length_m, 0.0, abs_tol=1e-9):
        raise ValueError("point_a and point_b must not be identical")

    ux = bx_m / lane_length_m
    uy = by_m / lane_length_m
    perp_x = -uy
    perp_y = ux

    waypoints: list[dict[str, float | int]] = []
    order = 0
    for lane in range(lane_count):
        offset_m = lane * spacing_m
        offset_x = perp_x * offset_m
        offset_y = perp_y * offset_m

        lane_start = (offset_x, offset_y)
        lane_end = (bx_m + offset_x, by_m + offset_y)
        endpoints = (lane_start, lane_end) if lane % 2 == 0 else (lane_end, lane_start)

        for x_m, y_m in endpoints:
            lat, lon = xy_to_latlon(x_m, y_m, lat_a, lon_a)
            waypoints.append(
                {
                    "lat": lat,
                    "lon": lon,
                    "x": x_m,
                    "y": y_m,
                    "lane": lane,
                    "order": order,
                }
            )
            order += 1

    return waypoints
