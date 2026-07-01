"""커버리지 경로 계획 코어 / Coverage-path planning core (routing).

목적/역할:
    이 패키지의 **라우팅 코어**다. 두 코너/기준점(A, B)과 레인 간격·스윕 폭 같은 파라미터로부터
    로버가 밭·구역을 빠짐없이 훑는 사행(보استرофедон, boustrophedon = 왕복 지그재그) 웨이포인트
    목록을 생성한다. 좌표 변환은 등거리(equirectangular) 근사(latlon_to_xy/xy_to_latlon)를 써서 빠르고
    단순하게 로컬 미터 평면에서 기하 계산을 한 뒤 다시 위/경도로 되돌린다.

시스템 내 위치:
    - 상류(입력): `scripts/station/plan_coverage_path.py`(CLI 경로 계획), `tools/path_preview.py`,
      `tools/path_planning_preview.py`, 분석 스크립트(`_figure_common.py`), 스테이션 목 미션 도구, 테스트가
      여기 함수를 호출한다. 패키지 `__init__`이 generate_lawnmower_path, latlon_to_xy, xy_to_latlon을 재수출.
    - 하류(출력): 생성된 웨이포인트 dict 목록이 미리보기 그림, CSV, 그리고 로버로 보낼 미션의 기반이 된다.
    파이프라인상 맨 앞 "경로 계획" 단계이며, 이후 protocol.py가 이를 프레임으로 전송한다.

핵심 개념·불변식:
    - 로컬 프레임 규약: 여기서 x=East, y=North(planner 내부). 사행은 레인 인덱스의 홀짝에 따라 시작/끝을
      뒤집어(왕복) 이어붙인다 — order 필드가 방문 순서를 준다.
    - 세 종류의 생성기: (1) generate_lawnmower_path = 고정 레인 수의 단순 왕복, (2) generate_coverage_path
      = 스윕 폭을 레인 간격으로 채우는 커버리지, (3) generate_corner_rectangle_path(_local) = A·B를 대각
      코너로 하는 직사각형 내부를 훑되 정확히 B에서 끝나도록 필요 시 final_connector를 덧붙인다.
    - 레인 오프셋 계산(lane_offsets_*)은 **경계(near/far edge)를 항상 포함**한다. 이 불변식이 커버리지의
      끝단 누락을 막는다 — 함부로 바꾸지 말 것. 부동소수 비교는 math.isclose로 경계 중복을 방지.
    - A==B(길이 0)나 0인 사각형 변, 음수/0 간격·속도는 ValueError로 거부(불량 미션 조기 차단).

리팩토링 노트:
    geo.py는 측지선 기반 정밀 변환을 제공하는 별도 좌표계다 — 이 파일의 근사와 결과가 미세하게 다르니
    섞지 말 것(정밀도가 필요하면 geo, 계획·미리보기는 planner). 반환 dict의 키 집합은 상류 CSV 헤더 및
    미리보기 코드와 결합되어 있으므로 키를 바꾸면 그쪽도 함께 수정해야 한다.

Purpose: the ROUTING CORE of this package. From two corner/reference points (A, B) plus lane
spacing / sweep width it produces the serpentine (boustrophedon = back-and-forth zig-zag) waypoint
list that lets the rover cover an area with no gaps. Geometry is done on a fast equirectangular
local-meter plane (``latlon_to_xy`` / ``xy_to_latlon``) then mapped back to lat/lon.

System placement: consumed upstream by ``scripts/station/plan_coverage_path.py`` (the path-planning
CLI), ``tools/path_preview.py``, analysis scripts, station mock-mission tools and tests; the package
``__init__`` re-exports ``generate_lawnmower_path`` / ``latlon_to_xy`` / ``xy_to_latlon``. Its output
(lists of waypoint dicts) feeds preview figures, CSVs and the rover mission that protocol.py then
transmits. This is the first stage of the pipeline.

Key invariants: internal local frame is x=East, y=North; alternate lanes are reversed to make the
sweep serpentine (the ``order`` field gives visit order). Three generators exist — a fixed-lane-count
lawnmower, a sweep-width coverage path, and a corner-rectangle path that appends a ``final_connector``
so it ends exactly at B. Lane-offset helpers **always include the near/far edges** so coverage does
not miss the boundary — keep that; float comparisons use ``math.isclose`` to avoid duplicate edges.
Degenerate input (A==B, zero rectangle side, non-positive spacing/speed) raises ``ValueError``.

Refactoring note: geo.py is a *separate*, geodesic (higher-accuracy) coordinate system — do not mix
it with this file's approximation. The returned dict key sets are coupled to upstream CSV headers and
preview code, so renaming keys means updating those too.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from typing import Any

# 지구 평균 반경(구 근사). 등거리 변환의 스케일 기준. / Mean Earth radius (sphere approx.); scale basis for the equirectangular conversion.
EARTH_RADIUS_M = 6_371_000.0
# num_lanes 미지정 시 lawnmower 기본 레인 수. / Default lane count for the lawnmower path when num_lanes is omitted.
DEFAULT_NUM_LANES = 4


# ── 등거리 좌표 변환 / Equirectangular coordinate conversion ──
def latlon_to_xy(lat: float, lon: float, lat0: float, lon0: float) -> tuple[float, float]:
    """위/경도를 기준점(lat0,lon0) 기준 로컬 x/y 미터로 변환(등거리 근사). / Lat/lon -> local x/y meters via equirectangular approximation.

    인자/Args: lat,lon=대상, lat0,lon0=로컬 원점(보통 A). 반환/Returns: (x=East m, y=North m).
    소규모 밭 수준에서 충분히 정확하며 geo.py의 측지선 변환보다 빠르다.
    Accurate enough at field scale and faster than geo.py's geodesic conversion.
    """
    lat0_rad = math.radians(lat0)
    # 경도 1도의 실제 거리는 위도에 따라 cos(lat0)만큼 줄어들므로 x에 이 스케일을 곱한다.
    # / One degree of longitude spans less distance at higher latitude, hence the cos(lat0) scale on x.
    x_m = EARTH_RADIUS_M * math.radians(lon - lon0) * math.cos(lat0_rad)
    y_m = EARTH_RADIUS_M * math.radians(lat - lat0)
    return x_m, y_m


def xy_to_latlon(x: float, y: float, lat0: float, lon0: float) -> tuple[float, float]:
    """로컬 x/y 미터를 다시 위/경도로 역변환. / Convert local x/y meters back to lat/lon (inverse of latlon_to_xy).

    극점 부근(cos(lat0)≈0)에서는 경도 스케일이 0에 수렴해 나눗셈이 폭발하므로 ValueError로 거부.
    Near the poles (cos(lat0)≈0) the longitude scale collapses, so this raises ValueError.
    """
    lat0_rad = math.radians(lat0)
    lon_scale = math.cos(lat0_rad)
    # 0으로 나누기(극점) 방지 가드 — 이 프로젝트의 운용 위도에서는 도달하지 않지만 안전상 유지.
    # / Guard against division by zero at the poles; unreachable in normal ops but kept for safety.
    if math.isclose(lon_scale, 0.0, abs_tol=1e-12):
        raise ValueError("reference latitude is too close to the poles")

    lat = lat0 + math.degrees(y / EARTH_RADIUS_M)
    lon = lon0 + math.degrees(x / (EARTH_RADIUS_M * lon_scale))
    return lat, lon


def _coerce_point(point: Mapping[str, Any] | Any, name: str) -> tuple[float, float]:
    """dict 또는 lat/lon 속성 객체에서 (lat, lon)을 유연하게 추출. / Extract (lat, lon) from a dict or an object with lat/lon attributes.

    매핑이면 키로, 그 외에는 속성으로 읽어 float로 강제한다. 키/속성이 없으면 명확한 오류를 던진다.
    Lets callers pass either a mapping or a GeoPoint-like object. Raises a clear error if lat/lon absent.
    """
    if isinstance(point, Mapping):
        try:
            return float(point["lat"]), float(point["lon"])
        except KeyError as exc:
            raise ValueError(f"{name} must contain 'lat' and 'lon'") from exc

    try:
        return float(getattr(point, "lat")), float(getattr(point, "lon"))
    except AttributeError as exc:
        raise TypeError(f"{name} must provide lat/lon values") from exc


# ── 레인 오프셋 계산 / Lane-offset computation ──
def lane_offsets_for_sweep_width(sweep_width_m: float, lane_spacing_m: float) -> list[float]:
    """스윕 폭을 채우는 레인 오프셋 목록(양 끝 경계 포함). / Lane offsets covering the sweep width, including near & far edges.

    0에서 시작해 lane_spacing 간격으로 쌓되, 마지막에 sweep_width 경계를 반드시 포함한다(끝단 누락 방지).
    간격·폭은 양수여야 하며 아니면 ValueError. / Starts at 0, steps by spacing, always ends exactly at
    the boundary so the far edge is not skipped. Positive inputs required.
    """
    if sweep_width_m <= 0:
        raise ValueError("sweep_width_m must be positive")
    if lane_spacing_m <= 0:
        raise ValueError("lane_spacing_m must be positive")

    offsets = [0.0]
    next_offset = lane_spacing_m
    # 경계와 거의 같은 지점에서 멈춰 중복 오프셋을 막는다(isclose로 부동소수 오차 흡수).
    # / Stop just before the boundary to avoid a duplicate offset; isclose absorbs float error.
    while next_offset < sweep_width_m and not math.isclose(
        next_offset, sweep_width_m, abs_tol=1e-9
    ):
        offsets.append(next_offset)
        next_offset += lane_spacing_m

    # 마지막 오프셋이 경계와 다르면 경계를 명시적으로 추가 — far edge 커버 보장(핵심 불변식).
    # / If the last offset isn't the boundary, append it so the far edge is covered (key invariant).
    if not math.isclose(offsets[-1], sweep_width_m, abs_tol=1e-9):
        offsets.append(sweep_width_m)

    return offsets


def lane_offsets_for_extent(extent_m: float, lane_spacing_m: float) -> list[float]:
    """0부터 extent까지의 오프셋(경계 항상 포함). / Offsets from zero to extent, always including the boundary.

    sweep-width 버전과 동일 로직의 별칭(의미상 '폭' 대신 '변 길이'). / Alias of the sweep-width helper.
    """
    return lane_offsets_for_sweep_width(extent_m, lane_spacing_m)


def _signed_lane_offsets(extent_m: float, lane_spacing_m: float) -> list[float]:
    """부호 있는 오프셋 목록(extent 방향 유지). / Signed offsets preserving the direction of ``extent``.

    extent가 음수면 오프셋도 음수로 뒤집는다 — 직사각형이 원점 반대편에 있을 때 방향을 맞추기 위함.
    / Negative extent flips offsets negative so the rectangle can lie on either side of the origin.
    """
    sign = -1.0 if extent_m < 0.0 else 1.0
    return [sign * offset for offset in lane_offsets_for_extent(abs(extent_m), lane_spacing_m)]


# ── 경로 생성기 / Path generators ──
def generate_corner_rectangle_path_local(
    end_x_m: float,
    end_y_m: float,
    lane_spacing_m: float,
    speed_mps: float | None = None,
) -> list[dict[str, float | int | str]]:
    """로컬 A=(0,0)~대각 코너 B=(end_x,end_y) 직사각형을 훑는 사행 경로. / Boustrophedon path across the local rectangle from A=(0,0) to opposite corner B.

    레인은 y축(폭) 방향으로 쌓이고 각 레인은 x=0↔end_x를 왕복한다. 경로가 정확히 B에서 끝나지 않으면
    final_connector 웨이포인트를 덧붙인다. 반환/Returns: order·lane·좌표·segment_type을 담은 dict 목록.
    간격/속도 비양수, 0인 사각형 변은 ValueError.
    Lanes stack along y; each lane sweeps x=0<->end_x. Appends a final connector so the path ends
    exactly at B. Returns a list of waypoint dicts.
    """
    if lane_spacing_m <= 0:
        raise ValueError("lane_spacing_m must be positive")
    if speed_mps is not None and speed_mps <= 0:
        raise ValueError("speed_mps must be positive when provided")
    # 변 하나라도 0이면 직사각형이 성립하지 않음 → 잘못된 A/B 입력을 조기 차단.
    # / A zero-length side means no rectangle; reject bad A/B input early.
    if math.isclose(end_x_m, 0.0, abs_tol=1e-9) or math.isclose(end_y_m, 0.0, abs_tol=1e-9):
        raise ValueError("point_a and point_b must define a non-zero rectangle")

    offsets = _signed_lane_offsets(end_y_m, lane_spacing_m)
    waypoints: list[dict[str, float | int | str]] = []
    order = 0
    for lane, y_m in enumerate(offsets):
        endpoints = ((0.0, y_m), (end_x_m, y_m))
        # 홀수 레인은 시작/끝을 뒤집어 이전 레인 끝에서 이어지게 한다 → 왕복(사행) 패턴.
        # / Reverse odd lanes so each continues from the previous lane's end — the serpentine sweep.
        if lane % 2 == 1:
            endpoints = (endpoints[1], endpoints[0])

        for point_index, (x_m, lane_y_m) in enumerate(endpoints):
            waypoint: dict[str, float | int | str] = {
                "order": order,
                "lane": lane,
                "x_m": x_m,
                "y_m": lane_y_m,
                "offset_m": abs(lane_y_m),
                "segment_type": "lane_start" if point_index == 0 else "lane_end",
                "notes": "",
            }
            if speed_mps is not None:
                waypoint["speed_mps"] = speed_mps
            waypoints.append(waypoint)
            order += 1

    # 마지막 웨이포인트가 코너 B와 정확히 일치하지 않으면 연결 구간을 추가해 B에서 끝나도록 보정.
    # / If the last waypoint isn't exactly at corner B, add a connector so the path terminates at B.
    last = waypoints[-1]
    if not (
        math.isclose(float(last["x_m"]), end_x_m, abs_tol=1e-9)
        and math.isclose(float(last["y_m"]), end_y_m, abs_tol=1e-9)
    ):
        connector: dict[str, float | int | str] = {
            "order": order,
            "lane": int(last["lane"]),
            "x_m": end_x_m,
            "y_m": end_y_m,
            "offset_m": abs(end_y_m),
            "segment_type": "final_connector",
            "notes": "final connector added to end exactly at point B",
        }
        if speed_mps is not None:
            connector["speed_mps"] = speed_mps
        waypoints.append(connector)
    else:
        last["notes"] = "point B final target"

    return waypoints


def generate_corner_rectangle_path(
    point_a: Mapping[str, Any] | Any,
    point_b: Mapping[str, Any] | Any,
    lane_spacing_m: float,
    speed_mps: float | None = None,
) -> list[dict[str, float | int | str]]:
    """A·B(위경도)를 대각 코너로 하는 직사각형 커버리지 경로(위경도 포함). / Coverage path over the lat/lon rectangle whose diagonal corners are A and B.

    A를 로컬 원점으로 삼아 로컬 버전을 호출한 뒤, 각 웨이포인트에 lat/lon을 다시 채워 넣는다.
    'dry-run'(실제 주행 전 미리보기)용이며 A는 (0,0)에 대응한다.
    Calls the local generator with A as origin, then back-fills lat/lon on every waypoint.
    """
    lat_a, lon_a = _coerce_point(point_a, "point_a")
    lat_b, lon_b = _coerce_point(point_b, "point_b")
    end_x_m, end_y_m = latlon_to_xy(lat_b, lon_b, lat_a, lon_a)

    waypoints = generate_corner_rectangle_path_local(
        end_x_m=end_x_m,
        end_y_m=end_y_m,
        lane_spacing_m=lane_spacing_m,
        speed_mps=speed_mps,
    )
    # 로컬 미터 웨이포인트를 다시 위/경도로 환원해 상류(미리보기/CSV)가 지도에 표시할 수 있게 한다.
    # / Map local-meter waypoints back to lat/lon so upstream preview/CSV can plot them on a map.
    for waypoint in waypoints:
        lat, lon = xy_to_latlon(float(waypoint["x_m"]), float(waypoint["y_m"]), lat_a, lon_a)
        waypoint["lat"] = lat
        waypoint["lon"] = lon
    return waypoints


def generate_coverage_path(
    point_a: Mapping[str, Any] | Any,
    point_b: Mapping[str, Any] | Any,
    sweep_width_m: float,
    lane_spacing_m: float,
    speed_mps: float | None = None,
) -> list[dict[str, float | int]]:
    """A→B 기준선과 스윕 폭으로 만드는 사행 커버리지 경로. / Boustrophedon coverage path from an A->B baseline and a sweep width.

    A→B가 레인의 진행 방향(길이)이 되고, 그에 수직으로 lane_spacing 간격만큼 sweep_width까지 레인을
    복제한다. 각 레인은 왕복으로 이어지며 웨이포인트에 위경도·로컬미터·offset이 담긴다.
    A->B is the lane direction; lanes are replicated perpendicular to it up to the sweep width.
    Returns waypoint dicts with lat/lon, local meters, lane index and offset.
    """
    if speed_mps is not None and speed_mps <= 0:
        raise ValueError("speed_mps must be positive when provided")

    lat_a, lon_a = _coerce_point(point_a, "point_a")
    lat_b, lon_b = _coerce_point(point_b, "point_b")

    bx_m, by_m = latlon_to_xy(lat_b, lon_b, lat_a, lon_a)
    lane_length_m = math.hypot(bx_m, by_m)
    # A==B이면 방향 벡터를 정규화할 수 없다(0으로 나눔) → 동일 지점 입력 거부.
    # / A==B leaves no direction to normalize (division by zero); reject identical points.
    if math.isclose(lane_length_m, 0.0, abs_tol=1e-9):
        raise ValueError("point_a and point_b must not be identical")

    # (ux,uy)=A→B 단위벡터, (perp_x,perp_y)=이를 90° 회전한 법선 → 레인을 옆으로 밀 방향.
    # / (ux,uy) is the unit A->B vector; (perp_x,perp_y) is it rotated 90°, the sideways lane-shift axis.
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
        # 짝수 레인은 start→end, 홀수 레인은 뒤집어 end→start로 이어붙여 왕복(사행)을 만든다.
        # / Even lanes go start->end, odd lanes end->start, producing the back-and-forth serpentine.
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
    """A→B 선분에서 고정 레인 수의 단순 왕복(잔디깎기) 경로 생성. / Simple fixed-lane back-and-forth ("lawnmower") path from segment A->B.

    coverage 버전과 기하는 같지만 스윕 폭 대신 레인 **개수**(num_lanes, 기본 4)를 직접 지정한다.
    이 함수가 패키지에서 재수출되는 대표 진입점(path_preview, mock mission, 분석 스크립트가 사용).
    반환/Returns: lat/lon, x, y, lane, order 키를 가진 웨이포인트 dict 목록. spacing/num_lanes 비양수는 ValueError.
    Same geometry as the coverage version but driven by a lane COUNT rather than a sweep width; this is
    the re-exported entry point used by previews, mock missions and analysis scripts.
    """
    if spacing_m <= 0:
        raise ValueError("spacing_m must be positive")

    lane_count = DEFAULT_NUM_LANES if num_lanes is None else num_lanes
    # bool은 int의 하위형이라 True가 개수 1로 새어들 수 있어 명시적으로 함께 거부한다.
    # / bool subclasses int, so True could slip in as count 1 — reject it explicitly.
    if isinstance(lane_count, bool) or not isinstance(lane_count, int) or lane_count <= 0:
        raise ValueError("num_lanes must be a positive integer")

    lat_a, lon_a = _coerce_point(point_a, "point_a")
    lat_b, lon_b = _coerce_point(point_b, "point_b")

    bx_m, by_m = latlon_to_xy(lat_b, lon_b, lat_a, lon_a)
    lane_length_m = math.hypot(bx_m, by_m)
    # A==B면 진행 방향을 정할 수 없어 정규화 불가 → 동일 지점 거부(coverage와 동일 가드).
    # / A==B gives no direction to normalize; reject identical points (same guard as coverage).
    if math.isclose(lane_length_m, 0.0, abs_tol=1e-9):
        raise ValueError("point_a and point_b must not be identical")

    # (perp_x,perp_y)는 A→B에 수직인 단위벡터로, 레인을 옆으로 spacing만큼 밀어내는 방향이다.
    # / (perp_x,perp_y) is the unit vector perpendicular to A->B — the direction lanes are offset by spacing.
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
        # 짝수 레인 start→end, 홀수 레인 end→start → 왕복(사행) 이동 순서. / Alternate direction per lane for the serpentine sweep.
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
