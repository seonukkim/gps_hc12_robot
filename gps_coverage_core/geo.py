from __future__ import annotations

import math
from dataclasses import dataclass

from geographiclib.geodesic import Geodesic


@dataclass(frozen=True)
class GeoPoint:
    lat: float
    lon: float


@dataclass(frozen=True)
class LocalPoint:
    x_m: float
    y_m: float


def latlon_to_local(origin: GeoPoint, point: GeoPoint) -> LocalPoint:
    """Convert WGS84 latitude/longitude to local East/North meters."""
    inverse = Geodesic.WGS84.Inverse(origin.lat, origin.lon, point.lat, point.lon)
    distance_m = inverse["s12"]
    azimuth_rad = math.radians(inverse["azi1"])
    x_m = math.sin(azimuth_rad) * distance_m
    y_m = math.cos(azimuth_rad) * distance_m
    return LocalPoint(x_m=x_m, y_m=y_m)


def local_to_latlon(origin: GeoPoint, point: LocalPoint) -> GeoPoint:
    """Convert local East/North meters into WGS84 latitude/longitude."""
    distance_m = math.hypot(point.x_m, point.y_m)
    if distance_m == 0:
        return origin

    azimuth_deg = math.degrees(math.atan2(point.x_m, point.y_m))
    direct = Geodesic.WGS84.Direct(origin.lat, origin.lon, azimuth_deg, distance_m)
    return GeoPoint(lat=direct["lat2"], lon=direct["lon2"])
