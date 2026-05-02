from __future__ import annotations

from typing import List

from .geo import GeoPoint, LocalPoint, latlon_to_local, local_to_latlon


def _axis_values(start: float, stop: float, step: float) -> list[float]:
    values = [start]
    current = start
    while current + step < stop:
        current += step
        values.append(current)
    if values[-1] != stop:
        values.append(stop)
    return values


def generate_lawnmower_path(point_a: GeoPoint, point_b: GeoPoint, spacing_m: float) -> List[GeoPoint]:
    """Generate a simple axis-aligned lawnmower path between two diagonal corners."""
    if spacing_m <= 0:
        raise ValueError("spacing_m must be positive")

    diagonal = latlon_to_local(point_a, point_b)
    min_x, max_x = sorted((0.0, diagonal.x_m))
    min_y, max_y = sorted((0.0, diagonal.y_m))
    width_m = max_x - min_x
    height_m = max_y - min_y

    if width_m < 1e-3 or height_m < 1e-3:
        raise ValueError("area is too small for lawnmower planning")

    y_values = _axis_values(min_y, max_y, spacing_m)
    waypoints: list[GeoPoint] = []
    for index, y_m in enumerate(y_values):
        x_values = (min_x, max_x) if index % 2 == 0 else (max_x, min_x)
        for x_m in x_values:
            waypoints.append(local_to_latlon(point_a, LocalPoint(x_m=x_m, y_m=y_m)))
    return waypoints
