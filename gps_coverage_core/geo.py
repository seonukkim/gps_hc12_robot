"""정밀 측지 좌표 변환 / High-accuracy geodesic coordinate conversion.

목적/역할:
    WGS84 위/경도와 로컬 East/North 미터(ENU 평면) 사이를 **측지선(geodesic) 기반**으로 정확히
    변환한다. `geographiclib`의 Karney 알고리즘을 사용하므로 planner.py의 등거리(equirectangular)
    근사보다 오차가 작고, 긴 거리·고위도에서도 신뢰할 수 있다.

시스템 내 위치:
    nmea.py가 `GeoPoint`를 import 해 파싱 결과를 담고, 패키지 `__init__`이 네 심볼
    (GeoPoint, LocalPoint, latlon_to_local, local_to_latlon)을 재수출한다. 파이프라인에서 "정밀이
    필요한" 좌표 변환 지점이며, 빠른 근사가 필요한 경로 계획은 planner.py의 latlon_to_xy를 쓴다.

핵심 개념·불변식:
    - 로컬 프레임 규약: x_m = East(동), y_m = North(북). 방위각(azimuth)은 북 기준 시계방향(도).
    - `local_to_latlon`은 거리 0일 때 원점을 그대로 반환한다(atan2(0,0) 미정의 회피). 이 가드는
      깨지 말 것 — 불변식이다.
    - 두 dataclass는 `frozen=True`라 불변(해시 가능). 좌표를 바꾸려면 새 인스턴스를 만들어야 한다.

리팩토링 노트:
    planner.py의 근사 좌표계와 결과가 미세하게 다르므로 둘을 섞어 쓰지 말 것. 정밀도가 중요하면 이
    모듈, 속도/단순성이 중요하면 planner.py를 사용한다.

Purpose: exact WGS84 lat/lon <-> local East/North meter conversion via geographiclib (Karney
geodesics), more accurate than planner.py's equirectangular approximation. Imported by nmea.py and
re-exported by the package. Local-frame convention: x_m=East, y_m=North; azimuth is degrees
clockwise from North. ``local_to_latlon`` returns the origin unchanged at zero distance (avoids
undefined atan2(0,0)) — keep that guard. Both dataclasses are frozen (immutable/hashable). Do not
mix this with planner's approximate coordinates.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from geographiclib.geodesic import Geodesic


@dataclass(frozen=True)
class GeoPoint:
    """WGS84 지리 좌표(위도/경도, 도 단위). / WGS84 geographic point (lat/lon in degrees)."""

    lat: float
    lon: float


@dataclass(frozen=True)
class LocalPoint:
    """로컬 ENU 평면 좌표(East=x_m, North=y_m, 미터). / Local ENU point (x_m=East, y_m=North, meters)."""

    x_m: float
    y_m: float


def latlon_to_local(origin: GeoPoint, point: GeoPoint) -> LocalPoint:
    """원점 기준 로컬 East/North 미터로 변환. / Convert WGS84 lat/lon to local East/North meters.

    측지선 역해(inverse)로 원점→점의 거리 s12와 초기 방위각 azi1을 구한 뒤 East/North 성분으로 분해한다.
    Uses the geodesic inverse to get distance + initial azimuth, then splits into East/North.

    인자/Args: origin=로컬 프레임 원점, point=변환 대상. 반환/Returns: LocalPoint(x_m=East, y_m=North).
    부수효과 없음(순수 함수). / Pure function, no side effects.
    """
    inverse = Geodesic.WGS84.Inverse(origin.lat, origin.lon, point.lat, point.lon)
    distance_m = inverse["s12"]
    # azi1은 북 기준 방위각(도)이라 sin→East, cos→North 로 분해한다.
    # / azi1 is azimuth clockwise from North, so sin->East and cos->North.
    azimuth_rad = math.radians(inverse["azi1"])
    x_m = math.sin(azimuth_rad) * distance_m
    y_m = math.cos(azimuth_rad) * distance_m
    return LocalPoint(x_m=x_m, y_m=y_m)


def local_to_latlon(origin: GeoPoint, point: LocalPoint) -> GeoPoint:
    """로컬 East/North 미터를 WGS84 위/경도로 역변환. / Convert local East/North meters to WGS84 lat/lon.

    `latlon_to_local`의 역연산: 원점에서 거리 hypot(x,y)와 방위각 atan2(East,North)를 구해 측지선
    직접해(direct)로 도착점을 얻는다. / Inverse of ``latlon_to_local`` via the geodesic direct solution.

    인자/Args: origin=원점, point=로컬 오프셋. 반환/Returns: 절대 GeoPoint. 순수 함수.
    """
    distance_m = math.hypot(point.x_m, point.y_m)
    # 거리 0이면 방위각이 미정의(atan2(0,0))이므로 원점을 그대로 반환 — 불변식, 삭제 금지.
    # / Zero distance => azimuth undefined; return origin unchanged. Invariant — do not remove.
    if distance_m == 0:
        return origin

    # atan2(East, North): 북 기준 시계방향 방위각(도). 인자 순서(x 먼저)는 규약이므로 뒤집지 말 것.
    # / atan2(East, North) yields azimuth clockwise from North; argument order is intentional.
    azimuth_deg = math.degrees(math.atan2(point.x_m, point.y_m))
    direct = Geodesic.WGS84.Direct(origin.lat, origin.lon, azimuth_deg, distance_m)
    return GeoPoint(lat=direct["lat2"], lon=direct["lon2"])
