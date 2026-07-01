"""사각형 커버리지 기하학 · 목표/시작점 해석 · 무상태 제어 수학.
Rectangle coverage geometry, goal/start resolution, and stateless control math.

목적/역할 (Purpose):
    로버가 밭을 ㄹ자(lawnmower/boustrophedon)로 훑는 경로의 **순수 기하 계산**을
    담당한다. 위경도(GPS) 시작·목표점을 로컬 ENU(동쪽=+X, 북쪽=+Y) 좌표로 바꾸고,
    커버리지 사각형을 세운 뒤, 레인(lane)·코너 연결부(connector)·주행 프리미티브
    (primitive)를 만든다. 시리얼/펌웨어/실제 모터 제어는 전혀 하지 않는다.

    Pure geometry for lawnmower/boustrophedon field coverage: convert GPS
    start/goal to local ENU (East=+X, North=+Y), build the coverage rectangle,
    then emit lanes, corner connectors, and drive primitives. No serial, no
    firmware, no motion here.

시스템 내 위치 (Where it sits):
    :mod:`preview`, :mod:`controller`, :mod:`alignment`, :mod:`tuning` 등 상위
    모듈이 이 파일의 leaf 함수를 import 한다(이 파일은 아무 것도 되-import 하지
    않아 순환이 없다). :mod:`gps_coverage_core.geo` 의 좌표 변환과
    :mod:`tools.physical_path_planning.calibration` 의 캘리브레이션 해석기를 쓴다.
    파이프라인 상 "계획 생성"의 최하단 수학 계층이다.

    Leaf math layer imported by :mod:`preview`, :mod:`controller`,
    :mod:`alignment`, :mod:`tuning`. Depends on :mod:`gps_coverage_core.geo`
    (coord transforms) and :mod:`calibration` (resolver); imports nothing back.

핵심 개념·불변식 (Key concepts / invariants):
    * ``target_heading_deg`` 는 **주행 방향**(진행 방향)의 ENU 각도이고,
      ``body_heading_deg`` 는 **차체가 향하는 방향**이다. 후진(backward) 레인에서는
      차체가 진행 방향의 반대(+180°)를 향하므로 둘이 다르다. 회전(turn)은 항상
      차체 헤딩(body heading)을 기준으로 정렬한다.
    * 각도는 ``atan2(north, east)`` 규약 — 동쪽 0°, 반시계(CCW) 증가.
    * 모든 산출 dict 는 ``ready_for_full_path_following=False`` 를 달고 나간다.
      (이 계층은 "완전 경로추종 준비 완료"를 절대 주장하지 않는다 — 안전 불변식.)

    ``target_heading_deg`` = travel direction (ENU); ``body_heading_deg`` = where
    the chassis points. On backward lanes the body faces travel+180°, so they
    differ; turns always align the BODY. Angles use ``atan2(north, east)`` (East
    0°, CCW+). Every emitted dict carries ``ready_for_full_path_following=False``.

사용법/진입점 (Entry points):
    ``coverage_lawnmower`` 모드는 :func:`build_axis_aligned_lawnmower_workspace`
    로 축정렬 사각형을 만든 뒤 :func:`build_serpentine_segments` 로 레인/코너를
    생성한다. ``diagonal_rectangle_serpentine`` 은 명시적 A→B 대각선 프레임
    (:func:`build_rectangle_from_diagonal_and_width`). ``direct_line`` 은
    :func:`build_direct_segments` 로 단일 직선. 제어 법칙 헬퍼
    (:func:`projection_metrics`, :func:`compute_b_correction`)는 무상태이며
    연속-모션 컨트롤러가 재사용한다.

리팩토링 노트 (Refactoring notes):
    코너 분해(turn_step_turn)가 가장 미묘한 부분이다 — 아래 해당 섹션 배너 참고.
    연결부 스타일을 늘릴 때는 ``CONNECTOR_STYLES`` 와
    :func:`build_serpentine_segments` 의 분기를 함께 고쳐야 한다. body/target
    헤딩 구분을 깨뜨리면 회전 방향이 조용히 틀어진다.

    Corner decomposition (turn_step_turn) is the subtle part — see its section
    banner. Adding a connector style means updating both ``CONNECTOR_STYLES`` and
    the branch in :func:`build_serpentine_segments`; breaking the body/target
    heading distinction silently flips turn directions.
"""
from __future__ import annotations

import math
from typing import Sequence

from gps_coverage_core.geo import GeoPoint, LocalPoint, latlon_to_local, local_to_latlon
from tools.physical_path_planning import calibration as calibration_resolver

# ── 상수·별칭·폴백 캘리브레이션 / Constants, aliases, fallback calibration ──
# 캘리브레이션이 하나도 주어지지 않을 때 쓰는 안전한 기본값. 프리뷰/미리보기가
# 캘리브레이션 파일 없이도 동작하도록 "전부 없음" 해석 결과를 미리 만들어 둔다.
# Safe default used when no calibration is supplied so preview works file-less.
FALLBACK_RESOLVED_CALIBRATION = calibration_resolver.resolve_physical_calibration(
    motion_calibration_json=None,
    fine_calibration_json=None,
    turn_calibration_json=None,
    turn_angle_calibration_json=None,
    smooth_turn_calibration_json=None,
    calibration_mode="repeated_pulses",
)

COVERAGE_LAWNMOWER = "coverage_lawnmower"
DIAGONAL_RECTANGLE_SERPENTINE = "diagonal_rectangle_serpentine"
DIRECT_LINE = "direct_line"
PATH_SHAPE_ALIASES = {
    "coverage_lawnmower": COVERAGE_LAWNMOWER,
    "coverage_serpentine": COVERAGE_LAWNMOWER,
    "lawnmower": COVERAGE_LAWNMOWER,
    "boustrophedon": COVERAGE_LAWNMOWER,
    "l_shape": COVERAGE_LAWNMOWER,
    "diagonal_rectangle_serpentine": DIAGONAL_RECTANGLE_SERPENTINE,
    "direct_line": DIRECT_LINE,
}

# How lane-to-lane transitions are planned. ``turn_step_turn`` (default) is the
# physically drivable ㄹ corner: pivot ~90 deg, drive the step-over distance as
# a short straight (forward after forward lanes, reverse after backward lanes),
# pivot ~90 deg back. ``single_turn`` is the legacy one-pivot connector whose
# 1.2 m sideways translation was never actually driven.
CONNECTOR_STYLE_TURN_STEP_TURN = "turn_step_turn"
CONNECTOR_STYLE_SINGLE_TURN = "single_turn"
CONNECTOR_STYLES = {CONNECTOR_STYLE_TURN_STEP_TURN, CONNECTOR_STYLE_SINGLE_TURN}
DEFAULT_CONNECTOR_STYLE = CONNECTOR_STYLE_TURN_STEP_TURN

FULL_LANE_SEGMENT_TYPES = {"forward_lane", "backward_lane"}
CONNECTOR_SEGMENT_TYPES = {"connector_turn", "path_connector"}


def canonical_path_shape(path_shape: str) -> str:
    """별칭을 정규 path_shape 로 변환 / Normalize any alias to a canonical path_shape.

    미지원 값이면 ``ValueError``. lawnmower/boustrophedon/l_shape 등은 모두
    ``coverage_lawnmower`` 로 합쳐진다.
    Raises ``ValueError`` for unknown shapes.
    """
    normalized = path_shape.strip().lower()
    if normalized not in PATH_SHAPE_ALIASES:
        raise ValueError(f"unsupported path_shape: {path_shape}")
    return PATH_SHAPE_ALIASES[normalized]


def _resolved_or_fallback(calibration: dict[str, object] | None) -> dict[str, object]:
    """주어진 캘리브레이션 없으면 폴백 사용 / Use fallback calibration when None."""
    return calibration if calibration is not None else FALLBACK_RESOLVED_CALIBRATION


def _motion_calibrated(calibration: dict[str, object] | None, direction: str) -> dict[str, object]:
    """전/후진 프리미티브의 해석된 A/B/ms 를 반환 / Resolved fwd/back drive primitive."""
    name = "forward" if direction == "forward" else "backward"
    return calibration_resolver.planner_primitive(_resolved_or_fallback(calibration), name)


def _turn_calibrated(calibration: dict[str, object] | None, direction: str) -> dict[str, object]:
    """좌/우 회전 연결부 프리미티브를 반환 / Resolved left/right connector turn primitive."""
    return calibration_resolver.connector_primitive(_resolved_or_fallback(calibration), direction)


def clamp(value: float, low: float, high: float) -> float:
    """값을 [low, high] 로 제한 / Clamp value into the inclusive range."""
    return max(low, min(high, value))


def wrap_deg(angle_deg: float) -> float:
    """각도를 (-180, 180] 로 래핑 / Wrap an angle into (-180, 180] degrees."""
    while angle_deg > 180.0:
        angle_deg -= 360.0
    while angle_deg < -180.0:
        angle_deg += 360.0
    return angle_deg


# ── 위경도 검증 & 목표점 해석 / Lat-lon validation & goal resolution ──


def validate_lat_lon(lat: float, lon: float, label: str) -> None:
    """위경도 범위 검증 / Validate lat in [-90,90] and lon in [-180,180], else raise."""
    if not -90.0 <= lat <= 90.0:
        raise ValueError(f"{label} latitude must be between -90 and 90")
    if not -180.0 <= lon <= 180.0:
        raise ValueError(f"{label} longitude must be between -180 and 180")


def goal_to_local(start_lat: float, start_lon: float, goal_lat: float, goal_lon: float) -> tuple[float, float]:
    """시작점 기준 목표점의 로컬 ENU (동, 북) 미터 오프셋 / Goal as local ENU (east, north) m.

    시작점을 원점으로 하는 로컬 평면 좌표를 돌려준다. 이후 모든 사각형 기하가
    이 (x=동, y=북) 규약 위에서 계산된다.
    Returns ``(x_m=east, y_m=north)`` relative to the start point.
    """
    validate_lat_lon(start_lat, start_lon, "start")
    validate_lat_lon(goal_lat, goal_lon, "goal")
    local = latlon_to_local(GeoPoint(start_lat, start_lon), GeoPoint(goal_lat, goal_lon))
    return local.x_m, local.y_m


def resolve_goal_point(
    *,
    start_lat: float,
    start_lon: float,
    goal_mode: str,
    goal_lat: float | None = None,
    goal_lon: float | None = None,
    goal_east_m: float | None = None,
    goal_north_m: float | None = None,
    goal_dlat: float | None = None,
    goal_dlon: float | None = None,
    goal_bearing_deg: float | None = None,
    goal_distance_m: float | None = None,
) -> tuple[GeoPoint, dict[str, object]]:
    """네 가지 목표 지정 방식을 하나의 GeoPoint 로 해석 / Resolve any goal spec to a GeoPoint.

    지원 모드(``goal_mode``): ``absolute`` (위경도), ``relative_enu`` (동/북 미터),
    ``relative_latlon`` (Δ위/Δ경도), ``bearing_distance`` (방위각°+거리 m). 각 모드는
    필요한 인자가 빠지면 CLI 플래그 이름을 담은 ``ValueError`` 를 던진다.
    반환: ``(목표 GeoPoint, 요약용 context dict)`` — context 는 어떤 입력이
    쓰였는지 기록해 프리뷰/로그에 남긴다.

    Resolves the four ``goal_mode`` variants (absolute / relative_enu /
    relative_latlon / bearing_distance) into ``(GeoPoint, context)``. Missing
    required args raise a ``ValueError`` naming the CLI flag; ``context`` records
    which inputs were used for previews/logs.
    """
    validate_lat_lon(start_lat, start_lon, "start")
    start = GeoPoint(start_lat, start_lon)
    context: dict[str, object] = {"goal_mode": goal_mode}
    if goal_mode == "absolute":
        if goal_lat is None or goal_lon is None:
            raise ValueError("--goal-lat and --goal-lon are required for --goal-mode absolute")
        validate_lat_lon(goal_lat, goal_lon, "goal")
        point = GeoPoint(goal_lat, goal_lon)
        context.update({"goal_lat": goal_lat, "goal_lon": goal_lon})
        return point, context
    if goal_mode == "relative_enu":
        if goal_east_m is None or goal_north_m is None:
            raise ValueError("--goal-east-m and --goal-north-m are required for --goal-mode relative_enu")
        point = local_to_latlon(start, LocalPoint(goal_east_m, goal_north_m))
        context.update({"goal_east_m": goal_east_m, "goal_north_m": goal_north_m})
        return point, context
    if goal_mode == "relative_latlon":
        if goal_dlat is None or goal_dlon is None:
            raise ValueError("--goal-dlat and --goal-dlon are required for --goal-mode relative_latlon")
        point = GeoPoint(start_lat + goal_dlat, start_lon + goal_dlon)
        validate_lat_lon(point.lat, point.lon, "goal")
        context.update({"goal_dlat": goal_dlat, "goal_dlon": goal_dlon})
        return point, context
    if goal_mode == "bearing_distance":
        if goal_bearing_deg is None or goal_distance_m is None:
            raise ValueError("--goal-bearing-deg and --goal-distance-m are required for --goal-mode bearing_distance")
        if goal_distance_m <= 0:
            raise ValueError("--goal-distance-m must be > 0")
        bearing_rad = math.radians(goal_bearing_deg)
        point = local_to_latlon(
            start,
            LocalPoint(
                x_m=math.sin(bearing_rad) * goal_distance_m,
                y_m=math.cos(bearing_rad) * goal_distance_m,
            ),
        )
        context.update({"goal_bearing_deg": goal_bearing_deg, "goal_distance_m": goal_distance_m})
        return point, context
    raise ValueError(f"unsupported goal_mode: {goal_mode}")


# ── 2D 벡터 헬퍼 / 2D vector helpers ──


def _unit(x: float, y: float) -> tuple[float, float]:
    """단위 벡터로 정규화 / Normalize to a unit vector; raise on zero length."""
    length = math.hypot(x, y)
    if length <= 1e-12:
        raise ValueError("zero-length vector")
    return x / length, y / length


def _rot_cw(x: float, y: float) -> tuple[float, float]:
    """시계방향 90° 회전 / Rotate a vector 90° clockwise."""
    return y, -x


def _rot_ccw(x: float, y: float) -> tuple[float, float]:
    """반시계방향 90° 회전 / Rotate a vector 90° counter-clockwise."""
    return -y, x


# ── 커버리지 사각형 구성 / Coverage-rectangle construction ──


def build_rectangle_from_diagonal_and_width(
    *,
    start_lat: float,
    start_lon: float,
    goal_lat: float,
    goal_lon: float,
    width_m: float,
    step_spacing_m: float,
    diagonal_orientation: str = "A_top_left_to_B_bottom_right",
) -> dict[str, object]:
    """A→B 를 사각형의 **대각선**으로 보고 커버리지 프레임을 구성 / Rectangle from A-B diagonal.

    ``coverage_lawnmower`` 와 달리 A→B 가 레인 축이 아니라 사각형의 대각선이다.
    폭(``width_m``)이 반드시 필요하며, 대각선과 폭으로부터 긴 축(길이)과 짧은
    축(폭)을 삼각함수로 복원한다. ``diagonal_orientation`` 은 사각형이 대각선의
    어느 쪽으로 펼쳐지는지(좌상→우하 / 좌하→우상)를 결정한다.
    반환: 코너·축 단위벡터·헤딩·치수를 담은 workspace dict.

    Treats A-B as the rectangle's diagonal (not the lane axis): given the
    diagonal and ``width_m``, recovers the long (length) and short (width) axes
    trigonometrically. ``diagonal_orientation`` selects which side of the
    diagonal the rectangle unfolds toward. Returns a workspace dict of corners,
    axis unit vectors, headings, and dimensions.
    """
    if width_m <= 0:
        raise ValueError("workspace width is required because A-B is a diagonal, not a direct path")
    if step_spacing_m <= 0:
        raise ValueError("step_spacing_m must be > 0")
    bx, by = goal_to_local(start_lat, start_lon, goal_lat, goal_lon)
    diagonal_length = math.hypot(bx, by)
    # 대각선이 폭보다 길어야 직각삼각형(길이²=대각선²-폭²)이 성립한다.
    # Diagonal must exceed width so length = sqrt(diag^2 - width^2) is real.
    if diagonal_length <= width_m:
        raise ValueError("A-B diagonal length must be larger than workspace-width-m.")
    length_m = math.sqrt(diagonal_length * diagonal_length - width_m * width_m)
    diagonal_unit = _unit(bx, by)
    angle = math.atan2(width_m, length_m)
    if diagonal_orientation == "A_top_left_to_B_bottom_right":
        ux = math.cos(angle) * diagonal_unit[0] + math.sin(angle) * _rot_ccw(*diagonal_unit)[0]
        uy = math.cos(angle) * diagonal_unit[1] + math.sin(angle) * _rot_ccw(*diagonal_unit)[1]
        vx, vy = _rot_cw(ux, uy)
    elif diagonal_orientation == "A_bottom_left_to_B_top_right":
        ux = math.cos(angle) * diagonal_unit[0] + math.sin(angle) * _rot_cw(*diagonal_unit)[0]
        uy = math.cos(angle) * diagonal_unit[1] + math.sin(angle) * _rot_cw(*diagonal_unit)[1]
        vx, vy = _rot_ccw(ux, uy)
    else:
        raise ValueError("unsupported diagonal_orientation")
    # Re-normalize to protect against tiny floating-point drift.
    ux, uy = _unit(ux, uy)
    vx, vy = _unit(vx, vy)
    c_x = length_m * ux
    c_y = length_m * uy
    d_x = width_m * vx
    d_y = width_m * vy
    b_reconstructed_x = c_x + d_x
    b_reconstructed_y = c_y + d_y
    x_axis_heading = math.degrees(math.atan2(uy, ux))
    y_axis_heading = math.degrees(math.atan2(vy, vx))
    origin = GeoPoint(start_lat, start_lon)
    c_geo = local_to_latlon(origin, LocalPoint(c_x, c_y))
    d_geo = local_to_latlon(origin, LocalPoint(d_x, d_y))
    b_reconstructed_geo = local_to_latlon(origin, LocalPoint(b_reconstructed_x, b_reconstructed_y))
    fallback_x, fallback_y = goal_to_local(start_lat, start_lon, goal_lat, goal_lon)
    return {
        "local_frame_origin_lat": start_lat,
        "local_frame_origin_lon": start_lon,
        "local_frame_origin_latlon": {"lat": start_lat, "lon": start_lon},
        "diagonal_orientation": diagonal_orientation,
        "A_corner": {"x_m": 0.0, "y_m": 0.0, "lat": start_lat, "lon": start_lon},
        "B_corner": {"x_m": bx, "y_m": by, "lat": goal_lat, "lon": goal_lon},
        "B_reconstructed": {"x_m": b_reconstructed_x, "y_m": b_reconstructed_y, "lat": b_reconstructed_geo.lat, "lon": b_reconstructed_geo.lon},
        "C_corner": {"x_m": c_x, "y_m": c_y, "lat": c_geo.lat, "lon": c_geo.lon},
        "D_corner": {"x_m": d_x, "y_m": d_y, "lat": d_geo.lat, "lon": d_geo.lon},
        "A_prime_top_left": {"x_m": 0.0, "y_m": 0.0, "lat": start_lat, "lon": start_lon},
        "B_prime_bottom_right": {"x_m": bx, "y_m": by, "lat": goal_lat, "lon": goal_lon},
        "long_axis_unit": {"x": ux, "y": uy},
        "short_axis_unit": {"x": vx, "y": vy},
        "local_x_axis_heading_deg": x_axis_heading,
        "local_y_axis_heading_deg": y_axis_heading,
        "workspace_length_m": length_m,
        "workspace_width_m": width_m,
        "diagonal_length_m": diagonal_length,
        "step_spacing_m": step_spacing_m,
        "ready_for_full_path_following": False,
    }


def build_axis_aligned_lawnmower_workspace(
    *,
    start_lat: float,
    start_lon: float,
    goal_lat: float,
    goal_lon: float,
    width_m: float,
    step_spacing_m: float,
) -> dict[str, object]:
    """ㄹ자 스윕용 **축정렬** 로컬-ENU 커버리지 사각형 구성 / Axis-aligned lawnmower rectangle.

    시작→목표 벡터의 **우세 성분**(더 큰 쪽)이 레인(주행) 방향이 되고, 나머지
    로컬 축이 커버리지 폭 방향이 된다. 폭의 부호는 목표 오프셋의 부호를 따라가서,
    흔한 상대-ENU 필드 명령을 직관적으로 만든다: 예) ``goal-east=4,
    goal-north=-1.2, width=1.2`` 는 동/서 레인을 훑으며 남쪽으로 스텝한다.
    :func:`build_rectangle_from_diagonal_and_width` 와 달리 축이 항상 로컬
    동/북에 정렬되므로 회전 없이 격자 커버리지가 된다.

    The dominant start->goal component becomes the lane direction; the other
    local axis is the coverage-width direction, taking the sign of the goal
    offset. This keeps common relative-ENU commands intuitive (e.g.
    ``goal-east=4, goal-north=-1.2, width=1.2`` sweeps E/W lanes, steps south).
    Unlike the diagonal builder, axes stay aligned to local East/North.
    """
    if width_m <= 0:
        raise ValueError("workspace_width_m must be > 0")
    if step_spacing_m <= 0:
        raise ValueError("step_spacing_m must be > 0")
    goal_x, goal_y = goal_to_local(start_lat, start_lon, goal_lat, goal_lon)
    if math.hypot(goal_x, goal_y) <= 1e-9:
        raise ValueError("goal distance must be > 0")
    # 더 큰 성분을 레인 축으로 선택. |동|>=|북| 이면 동/서로 훑고 남/북으로 스텝.
    # Pick the larger component as the lane axis; |E|>=|N| => E/W lanes.
    if abs(goal_x) >= abs(goal_y):
        length = goal_x
        if math.isclose(length, 0.0, abs_tol=1e-9):
            raise ValueError("coverage_lawnmower requires a nonzero lane length")
        width_sign = -1.0 if goal_y < 0 else 1.0
        ux, uy = (1.0 if length >= 0 else -1.0), 0.0
        vx, vy = 0.0, width_sign
        length_m = abs(length)
    else:
        length = goal_y
        width_sign = -1.0 if goal_x < 0 else 1.0
        ux, uy = 0.0, (1.0 if length >= 0 else -1.0)
        vx, vy = width_sign, 0.0
        length_m = abs(length)
    c_x = length_m * ux
    c_y = length_m * uy
    d_x = width_m * vx
    d_y = width_m * vy
    b_reconstructed_x = c_x + d_x
    b_reconstructed_y = c_y + d_y
    origin = GeoPoint(start_lat, start_lon)
    c_geo = local_to_latlon(origin, LocalPoint(c_x, c_y))
    d_geo = local_to_latlon(origin, LocalPoint(d_x, d_y))
    b_reconstructed_geo = local_to_latlon(origin, LocalPoint(b_reconstructed_x, b_reconstructed_y))
    diagonal_length = math.hypot(goal_x, goal_y)
    return {
        "local_frame_origin_lat": start_lat,
        "local_frame_origin_lon": start_lon,
        "local_frame_origin_latlon": {"lat": start_lat, "lon": start_lon},
        "diagonal_orientation": "axis_aligned_local_enu",
        "A_corner": {"x_m": 0.0, "y_m": 0.0, "lat": start_lat, "lon": start_lon},
        "B_corner": {"x_m": goal_x, "y_m": goal_y, "lat": goal_lat, "lon": goal_lon},
        "B_reconstructed": {
            "x_m": b_reconstructed_x,
            "y_m": b_reconstructed_y,
            "lat": b_reconstructed_geo.lat,
            "lon": b_reconstructed_geo.lon,
        },
        "C_corner": {"x_m": c_x, "y_m": c_y, "lat": c_geo.lat, "lon": c_geo.lon},
        "D_corner": {"x_m": d_x, "y_m": d_y, "lat": d_geo.lat, "lon": d_geo.lon},
        "A_prime_top_left": {"x_m": 0.0, "y_m": 0.0, "lat": start_lat, "lon": start_lon},
        "B_prime_bottom_right": {
            "x_m": b_reconstructed_x,
            "y_m": b_reconstructed_y,
            "lat": b_reconstructed_geo.lat,
            "lon": b_reconstructed_geo.lon,
        },
        "long_axis_unit": {"x": ux, "y": uy},
        "short_axis_unit": {"x": vx, "y": vy},
        "local_x_axis_heading_deg": math.degrees(math.atan2(uy, ux)),
        "local_y_axis_heading_deg": math.degrees(math.atan2(vy, vx)),
        "workspace_length_m": length_m,
        "workspace_width_m": width_m,
        "diagonal_length_m": diagonal_length,
        "step_spacing_m": step_spacing_m,
        "coverage_area_estimate_m2": length_m * width_m,
        "expected_sweep_style": "lawnmower_ㄹ",
        "ready_for_full_path_following": False,
    }


# ── 레인/코너/프리미티브 생성 / Lane, connector & primitive generation ──


def _track_offsets(width_m: float, step_spacing_m: float) -> list[float]:
    """폭을 스텝 간격으로 나눈 레인 오프셋 목록 / Lane offsets across the width.

    0 에서 시작해 ``step_spacing_m`` 씩 증가하며, 마지막 오프셋이 폭에 정확히
    닿지 않으면 ``width_m`` 를 끝에 덧붙여 가장자리까지 커버되도록 한다.
    Always includes 0 and ``width_m`` so both edges are covered.
    """
    offsets = [0.0]
    value = step_spacing_m
    while value < width_m and not math.isclose(value, width_m, abs_tol=1e-9):
        offsets.append(value)
        value += step_spacing_m
    if not math.isclose(offsets[-1], width_m, abs_tol=1e-9):
        offsets.append(width_m)
    return offsets


def _point_on_rectangle(workspace: dict[str, object], along_m: float, across_m: float) -> tuple[float, float]:
    """(따라/가로) 좌표를 로컬 (x,y) 로 변환 / Map (along, across) to local (x, y).

    긴 축 단위벡터 u 와 짧은 축 단위벡터 v 로 ``along*u + across*v`` 를 계산한다.
    Uses the workspace long/short axis unit vectors.
    """
    u = workspace["long_axis_unit"]  # type: ignore[index]
    v = workspace["short_axis_unit"]  # type: ignore[index]
    ux = float(u["x"])  # type: ignore[index]
    uy = float(u["y"])  # type: ignore[index]
    vx = float(v["x"])  # type: ignore[index]
    vy = float(v["y"])  # type: ignore[index]
    return along_m * ux + across_m * vx, along_m * uy + across_m * vy


def _turn_repeat_count(
    calibration: dict[str, object] | None, direction: str, turn_angle_deg: float
) -> int:
    """코너 회전 1회의 계획 펄스 수 / Planned pulse count for one connector turn.

    반복-펄스(repeated_pulses) 모드는 고정 펄스 수를 그대로 쓴다. 각도 캘리브레이션
    모드는 ``ceil(|각도| / 펄스당 target_angle_deg)`` 로 예산을 잡아, turn_*_90
    키에 저장된 작은 회전 펄스가 코너마다 1회가 아니라 2~4회로 계획되게 한다.
    (프리뷰/프리미티브 표시에만 쓰이는 계획값 — 실제 정지 조건은 IMU 피드백.)

    Repeated-pulse mode keeps its fixed count; angle-calibrated mode budgets
    ``ceil(|angle| / per-pulse target_angle_deg)``. Planning value for
    primitives/preview only; runtime stop is IMU-driven.
    """
    resolved = _resolved_or_fallback(calibration)
    if str(resolved.get("connector_mode_effective", "repeated_pulses")) == "repeated_pulses":
        key = "left_fixed_pulses" if direction == "left" else "right_fixed_pulses"
        return int(resolved.get(key, 12))
    turn = _turn_calibrated(calibration, direction)
    per_pulse = turn.get("target_angle_deg")
    if per_pulse is None or float(per_pulse) <= 0.0:
        return 1
    return max(1, math.ceil(abs(turn_angle_deg) / float(per_pulse) - 1e-9))


def _connector_turn_segment(
    *,
    segment_index: int,
    x: float,
    y: float,
    body_before_deg: float,
    body_after_deg: float,
) -> dict[str, object]:
    """제자리 회전(길이 0) 세그먼트 / Zero-length in-place pivot segment.

    차체 헤딩을 ``body_before_deg`` 에서 ``body_after_deg`` 로 돌린다. 회전각의
    부호로 좌/우 회전을 정하고, 시작=끝 좌표(길이 0)로 순수 피벗을 표현한다.
    Rotates body heading before->after; sign picks left/right, start==end.
    """
    turn_angle = wrap_deg(body_after_deg - body_before_deg)
    return {
        "segment_index": segment_index,
        "segment_type": "connector_turn",
        "connector_kind": "turn_in_place",
        "start_x_m": x,
        "start_y_m": y,
        "end_x_m": x,
        "end_y_m": y,
        "target_heading_deg": body_after_deg,
        "body_heading_deg": body_after_deg,
        "turn_angle_deg": turn_angle,
        "expected_motion_direction": "turn_left" if turn_angle >= 0.0 else "turn_right",
        "tool_active": False,
        "length_m": 0.0,
        "pulse_budget": 1,
    }


def _turn_primitive_for_segment(
    calibration: dict[str, object] | None,
    segment: dict[str, object],
    primitive_index: int,
) -> dict[str, object]:
    """회전 세그먼트를 실행 프리미티브(a/b/ms/repeat)로 변환 / Turn segment -> drive primitive.

    세그먼트의 ``expected_motion_direction`` 으로 좌/우를 골라 해당 캘리브레이션의
    A/B/pulse_ms 를 채우고, :func:`_turn_repeat_count` 로 반복 횟수를 계산한다.
    Fills calibrated a/b/pulse_ms and the planned repeat count for the turn.
    """
    direction = "left" if str(segment["expected_motion_direction"]) == "turn_left" else "right"
    turn = _turn_calibrated(calibration, direction)
    return {
        "primitive_index": primitive_index,
        "segment_index": segment["segment_index"],
        "primitive_type": str(segment["expected_motion_direction"]),
        "a_cmd": turn["a_cmd"],
        "b_cmd": turn["b_cmd"],
        "pulse_ms": turn["pulse_ms"],
        "target_heading_deg": segment["target_heading_deg"],
        "turn_angle_deg": segment.get("turn_angle_deg", "NA"),
        "calibration_source": turn["calibration_source"],
        "connector_mode": turn.get(
            "connector_mode",
            _resolved_or_fallback(calibration).get("connector_mode_effective", "repeated_pulses"),
        ),
        "repeat_count": _turn_repeat_count(
            calibration, direction, float(segment.get("turn_angle_deg", 90.0) or 90.0)
        ),
        "ready_for_full_path_following": False,
    }


def build_serpentine_segments(
    workspace: dict[str, object],
    *,
    max_segment_pulses: int,
    nominal_forward_pulse_m: float,
    calibration: dict[str, object] | None = None,
    connector_style: str = DEFAULT_CONNECTOR_STYLE,
) -> tuple[list[dict[str, object]], list[dict[str, object]], list[dict[str, object]]]:
    """커버리지 사각형을 ㄹ자 세그먼트+프리미티브+경로점으로 전개 / Expand rectangle into ㄹ sweep.

    레인을 번갈아 전진/후진으로 깔고(홀짝), 레인 사이를 ``connector_style`` 에 따라
    연결한다. 반환: ``(segments, primitives, tool_path)`` 세 리스트.

    핵심 불변식 (invariant): 전진 레인은 차체가 진행 방향을, 후진 레인은 그 반대
    (+180°)를 향한다. 코너 회전은 항상 **차체 헤딩(body_heading_deg)** 기준으로
    정렬해야 다음 레인이 올바른 자세로 시작한다.

    연결부 스타일:
        * ``turn_step_turn`` (기본): 물리적으로 주행 가능한 ㄹ 코너 —
          회전→스텝오버 직선→회전 3-분해. 아래 섹션 배너 참고.
        * ``single_turn`` (레거시): 한 번의 피벗으로 옆 레인으로 넘어가는 방식으로,
          실제로는 주행되지 않던 1.2m 측면 이동을 가정한다.

    Expands the coverage rectangle into an alternating forward/backward ㄹ sweep,
    joining lanes per ``connector_style``. Returns ``(segments, primitives,
    tool_path)``. Invariant: forward lanes face travel, backward lanes face
    travel+180°; corner turns align the BODY heading. ``turn_step_turn`` (default)
    is the drivable 3-part corner (see banner below); ``single_turn`` is legacy.
    """
    if connector_style not in CONNECTOR_STYLES:
        raise ValueError(f"unsupported connector_style: {connector_style}")
    length_m = float(workspace["workspace_length_m"])
    width_m = float(workspace["workspace_width_m"])
    step_spacing_m = float(workspace["step_spacing_m"])
    offsets = _track_offsets(width_m, step_spacing_m)
    segments: list[dict[str, object]] = []
    primitives: list[dict[str, object]] = []
    segment_index = 0
    primitive_index = 0
    # ── 레인 루프: 홀짝으로 전/후진 교대 / Lane loop: alternate fwd/back by parity ──
    for lane_index, offset in enumerate(offsets):
        forward = lane_index % 2 == 0
        start_along = 0.0 if forward else length_m
        end_along = length_m if forward else 0.0
        sx, sy = _point_on_rectangle(workspace, start_along, offset)
        ex, ey = _point_on_rectangle(workspace, end_along, offset)
        heading = math.degrees(math.atan2(ey - sy, ex - sx))
        # 전진 레인은 차체가 진행 방향을, 후진(역주행) 레인은 반대(+180°)를 향한다.
        # 회전은 항상 이 body_heading 기준으로 정렬한다 — 핵심 불변식.
        # The body faces the travel direction on forward lanes and the opposite
        # way on backward (reverse-driven) lanes; turns align the BODY.
        body_heading = heading if forward else wrap_deg(heading + 180.0)
        segment_index += 1
        primitive_index += 1
        direction = "forward" if forward else "backward"
        pulse_budget = max(1, min(max_segment_pulses, math.ceil(length_m / max(nominal_forward_pulse_m, 0.05))))
        segments.append({
            "segment_index": segment_index,
            "segment_type": f"{direction}_lane",
            "start_x_m": sx,
            "start_y_m": sy,
            "end_x_m": ex,
            "end_y_m": ey,
            "target_heading_deg": heading,
            "body_heading_deg": body_heading,
            "expected_motion_direction": direction,
            "tool_active": True,
            "length_m": length_m,
            "pulse_budget": pulse_budget,
        })
        primitive_name = "move_forward" if forward else "move_backward"
        calibrated = _motion_calibrated(calibration, direction)
        primitives.append({
            "primitive_index": primitive_index,
            "segment_index": segment_index,
            "primitive_type": primitive_name,
            "a_cmd": calibrated["a_cmd"],
            "b_cmd": calibrated["b_cmd"],
            "pulse_ms": calibrated["pulse_ms"],
            "target_heading_deg": heading,
            "calibration_source": calibrated["calibration_source"],
            "connector_mode": "lane",
            "repeat_count": pulse_budget,
            "ready_for_full_path_following": False,
        })
        if lane_index + 1 < len(offsets):
            next_offset = offsets[lane_index + 1]
            csx, csy = ex, ey
            cex, cey = _point_on_rectangle(workspace, end_along, next_offset)
            connector_heading = math.degrees(math.atan2(cey - csy, cex - csx))
            # ══════════════════════════════════════════════════════════════
            # ㄹ 코너 3-분해 (회전→스텝→회전) / ㄹ corner: turn → step → turn
            # ──────────────────────────────────────────────────────────────
            # 이 파일에서 가장 미묘한 부분. 한 레인 끝에서 옆 레인 시작으로
            # 넘어가는 코너를 물리적으로 주행 가능한 3개 세그먼트로 쪼갠다:
            #   1) turn_in : 차체를 스텝오버 방향으로 피벗
            #   2) step    : 스텝 간격만큼 짧은 직선 주행. 전진 레인 뒤엔 전진,
            #      후진 레인 뒤엔 후진으로 밀어서 다음 레인이 이미 올바른 차체
            #      자세로 시작되게 한다(추가 회전 절약).
            #   3) turn_out: 다음 레인의 body_heading 으로 피벗
            # 함정: step 방향을 전/후진으로 맞추지 않으면 다음 레인 진입 자세가
            # 틀어져 헤딩이 조용히 어긋난다.
            # The subtlest block here. Split each lane-to-lane corner into three
            # drivable segments: (1) pivot into the step-over direction, (2) a
            # short straight of one step spacing — forward after forward lanes,
            # reverse after backward lanes so the next full lane already starts
            # in the correct body pose, (3) pivot to the next lane's body
            # heading. Gotcha: mismatching the step's fwd/back sense skews the
            # next lane's entry pose and silently drifts heading.
            if connector_style == CONNECTOR_STYLE_TURN_STEP_TURN:
                # Drivable ㄹ corner: pivot, drive the step-over as a short
                # straight (forward after forward lanes, reverse after backward
                # lanes so the next full lane starts with the body already
                # aligned), pivot back to the next lane's body heading.
                step_direction = "forward" if forward else "backward"
                step_body_heading = (
                    connector_heading
                    if step_direction == "forward"
                    else wrap_deg(connector_heading + 180.0)
                )
                next_forward = (lane_index + 1) % 2 == 0
                # Next lane runs the opposite along-direction at next_offset.
                n_start_along = 0.0 if next_forward else length_m
                n_end_along = length_m if next_forward else 0.0
                nsx, nsy = _point_on_rectangle(workspace, n_start_along, next_offset)
                nex, ney = _point_on_rectangle(workspace, n_end_along, next_offset)
                next_travel_heading = math.degrees(math.atan2(ney - nsy, nex - nsx))
                next_body_heading = (
                    next_travel_heading if next_forward else wrap_deg(next_travel_heading + 180.0)
                )
                segment_index += 1
                primitive_index += 1
                # (1) turn_in: 현재 차체 → 스텝오버 방향으로 피벗 / pivot into step-over.
                turn_in = _connector_turn_segment(
                    segment_index=segment_index,
                    x=csx,
                    y=csy,
                    body_before_deg=body_heading,
                    body_after_deg=step_body_heading,
                )
                segments.append(turn_in)
                primitives.append(
                    _turn_primitive_for_segment(calibration, turn_in, primitive_index)
                )
                segment_index += 1
                primitive_index += 1
                # (2) step: 스텝 간격만큼 짧은 직선 / short straight of one step spacing.
                step_length = abs(next_offset - offset)
                step_budget = max(
                    1,
                    min(max_segment_pulses, math.ceil(step_length / max(nominal_forward_pulse_m, 0.05))),
                )
                segments.append({
                    "segment_index": segment_index,
                    "segment_type": "step_lane",
                    "connector_kind": "step_over",
                    "start_x_m": csx,
                    "start_y_m": csy,
                    "end_x_m": cex,
                    "end_y_m": cey,
                    "target_heading_deg": connector_heading,
                    "body_heading_deg": step_body_heading,
                    "expected_motion_direction": step_direction,
                    "tool_active": False,
                    "length_m": step_length,
                    "pulse_budget": step_budget,
                })
                step_calibrated = _motion_calibrated(calibration, step_direction)
                primitives.append({
                    "primitive_index": primitive_index,
                    "segment_index": segment_index,
                    "primitive_type": "move_forward" if step_direction == "forward" else "move_backward",
                    "a_cmd": step_calibrated["a_cmd"],
                    "b_cmd": step_calibrated["b_cmd"],
                    "pulse_ms": step_calibrated["pulse_ms"],
                    "target_heading_deg": connector_heading,
                    "calibration_source": step_calibrated["calibration_source"],
                    "connector_mode": "step_lane",
                    "repeat_count": step_budget,
                    "ready_for_full_path_following": False,
                })
                segment_index += 1
                primitive_index += 1
                # (3) turn_out: 다음 레인의 body_heading 으로 피벗 / pivot to next lane pose.
                turn_out = _connector_turn_segment(
                    segment_index=segment_index,
                    x=cex,
                    y=cey,
                    body_before_deg=step_body_heading,
                    body_after_deg=next_body_heading,
                )
                segments.append(turn_out)
                primitives.append(
                    _turn_primitive_for_segment(calibration, turn_out, primitive_index)
                )
            else:
                # 레거시 single_turn: 한 번의 피벗 연결부. 여기서 가정하는 측면
                # 이동(≈1.2m)은 실제로 주행된 적이 없다 — 신규 계획엔 쓰지 말 것.
                # Legacy single_turn: one-pivot connector; its sideways
                # translation was never actually driven. Prefer turn_step_turn.
                segment_index += 1
                primitive_index += 1
                turn_direction = "turn_left" if forward else "turn_right"
                segments.append({
                    "segment_index": segment_index,
                    "segment_type": "path_connector",
                    "connector_kind": "path_connector",
                    "start_x_m": csx,
                    "start_y_m": csy,
                    "end_x_m": cex,
                    "end_y_m": cey,
                    "target_heading_deg": connector_heading,
                    "expected_motion_direction": turn_direction,
                    "tool_active": False,
                    "length_m": abs(next_offset - offset),
                    "pulse_budget": 1,
                })
                connector_direction = "left" if turn_direction == "turn_left" else "right"
                turn = _turn_calibrated(calibration, connector_direction)
                repeat_count = 1
                if str(_resolved_or_fallback(calibration).get("connector_mode_effective", "repeated_pulses")) == "repeated_pulses":
                    repeat_count = int(_resolved_or_fallback(calibration).get("left_fixed_pulses" if connector_direction == "left" else "right_fixed_pulses", 12))
                primitives.append({
                    "primitive_index": primitive_index,
                    "segment_index": segment_index,
                    "primitive_type": turn_direction,
                    "a_cmd": turn["a_cmd"],
                    "b_cmd": turn["b_cmd"],
                    "pulse_ms": turn["pulse_ms"],
                    "target_heading_deg": connector_heading,
                    "calibration_source": turn["calibration_source"],
                    "connector_mode": turn.get("connector_mode", _resolved_or_fallback(calibration).get("connector_mode_effective", "repeated_pulses")),
                    "repeat_count": repeat_count,
                    "ready_for_full_path_following": False,
                })
    # ── 세그먼트 → 위경도 경로점 열거 / Flatten segments to lat-lon path points ──
    tool_path = []
    point_index = 0
    origin = GeoPoint(float(workspace["local_frame_origin_lat"]), float(workspace["local_frame_origin_lon"]))
    for segment in segments:
        for point_type, x_key, y_key in (("start", "start_x_m", "start_y_m"), ("end", "end_x_m", "end_y_m")):
            point_index += 1
            x = float(segment[x_key])
            y = float(segment[y_key])
            geo = local_to_latlon(origin, LocalPoint(x, y))
            tool_path.append({
                "point_index": point_index,
                "segment_index": segment["segment_index"],
                "point_type": point_type,
                "x_m": x,
                "y_m": y,
                "lat": geo.lat,
                "lon": geo.lon,
            })
    return segments, primitives, tool_path


def build_direct_segments(
    *,
    start_lat: float,
    start_lon: float,
    goal_lat: float,
    goal_lon: float,
    max_segment_pulses: int,
    nominal_forward_pulse_m: float,
    calibration: dict[str, object] | None = None,
) -> tuple[list[dict[str, object]], list[dict[str, object]], float]:
    """A→B 단일 직선(탈출용) 세그먼트 / Single straight A->B segment (escape hatch).

    커버리지 없이 시작점에서 목표점까지 한 개의 전진 레인을 만든다.
    반환: ``(segments, primitives, distance_m)``. 시작=목표이면 ``ValueError``.
    Builds one forward lane start->goal; returns ``(segments, primitives,
    distance)``. Raises if start and goal coincide.
    """
    goal_x, goal_y = goal_to_local(start_lat, start_lon, goal_lat, goal_lon)
    distance = math.hypot(goal_x, goal_y)
    if distance <= 1e-6:
        raise ValueError("start and goal must not be identical")
    heading = math.degrees(math.atan2(goal_y, goal_x))
    pulse_budget = max(1, min(max_segment_pulses, math.ceil(distance / max(nominal_forward_pulse_m, 0.05))))
    segment = {
        "segment_index": 1,
        "segment_type": "forward_lane",
        "start_x_m": 0.0,
        "start_y_m": 0.0,
        "end_x_m": goal_x,
        "end_y_m": goal_y,
        "target_heading_deg": heading,
        "expected_motion_direction": "forward",
        "tool_active": True,
        "length_m": distance,
        "pulse_budget": pulse_budget,
    }
    calibrated = _motion_calibrated(calibration, "forward")
    primitive = {
        "primitive_index": 1,
        "segment_index": 1,
        "primitive_type": "move_forward",
        "a_cmd": calibrated["a_cmd"],
        "b_cmd": calibrated["b_cmd"],
        "pulse_ms": calibrated["pulse_ms"],
        "target_heading_deg": heading,
        "calibration_source": calibrated["calibration_source"],
        "connector_mode": "lane",
        "repeat_count": pulse_budget,
        "ready_for_full_path_following": False,
    }
    return [segment], [primitive], distance


def primitives_from_segments(
    segments: Sequence[dict[str, object]],
    calibration: dict[str, object],
) -> list[dict[str, object]]:
    """이미 계획된 세그먼트 리스트를 실행 프리미티브로 재변환 / Rebuild primitives from segments.

    저장/재로드된 세그먼트에서 프리미티브를 다시 만들 때 쓴다. 연결부 세그먼트는
    좌/우 회전으로, 나머지는 전/후진 이동으로 분류하고, 회전각이 없으면 좌=+90°,
    우=-90° 로 기본값을 준다. :func:`build_serpentine_segments` 와 프리미티브
    형식이 일치해야 한다(리팩토링 시 함께 유지).
    Reconstructs primitives from a stored segment list; keep its primitive
    schema in sync with :func:`build_serpentine_segments`.
    """
    primitives: list[dict[str, object]] = []
    for primitive_index, segment in enumerate(segments, start=1):
        segment_type = str(segment.get("segment_type", ""))
        expected = str(segment.get("expected_motion_direction", "forward"))
        if segment_type in CONNECTOR_SEGMENT_TYPES:
            direction = "left" if expected == "turn_left" else "right"
            calibrated = _turn_calibrated(calibration, direction)
            turn_angle = segment.get("turn_angle_deg")
            requested_angle = (
                float(turn_angle)
                if turn_angle not in (None, "", "NA")
                else (90.0 if direction == "left" else -90.0)
            )
            repeat_count = _turn_repeat_count(calibration, direction, requested_angle)
            primitive_type = "turn_left" if direction == "left" else "turn_right"
            connector_mode = calibrated.get("connector_mode", calibration.get("connector_mode_effective", "repeated_pulses"))
        else:
            direction = "backward" if expected == "backward" or segment_type.startswith("backward") else "forward"
            calibrated = _motion_calibrated(calibration, direction)
            repeat_count = int(segment.get("pulse_budget", 1))
            primitive_type = "move_backward" if direction == "backward" else "move_forward"
            connector_mode = "step_lane" if segment_type == "step_lane" else "lane"
        primitives.append({
            "primitive_index": primitive_index,
            "segment_index": segment.get("segment_index", primitive_index),
            "primitive_type": primitive_type,
            "a_cmd": calibrated["a_cmd"],
            "b_cmd": calibrated["b_cmd"],
            "pulse_ms": calibrated["pulse_ms"],
            "target_heading_deg": segment.get("target_heading_deg", "NA"),
            "calibration_source": calibrated["calibration_source"],
            "connector_mode": connector_mode,
            "repeat_count": repeat_count,
            "ready_for_full_path_following": False,
        })
    return primitives


def planned_path_points(segments: Sequence[dict[str, object]], start: GeoPoint) -> list[dict[str, object]]:
    """각 세그먼트의 시작/끝점을 위경도 경로점으로 나열 / Segment endpoints as lat-lon points.

    로컬 (x,y)를 시작점 기준 위경도로 되돌려 프리뷰/로그용 점 목록을 만든다.
    Maps each segment start/end from local (x,y) back to lat-lon.
    """
    points = []
    point_index = 0
    for segment in segments:
        for x_key, y_key, label in (("start_x_m", "start_y_m", "start"), ("end_x_m", "end_y_m", "goal")):
            point_index += 1
            point = local_to_latlon(start, LocalPoint(float(segment[x_key]), float(segment[y_key])))
            points.append({
                "point_index": point_index,
                "segment_index": segment["segment_index"],
                "point_type": label,
                "x_m": segment[x_key],
                "y_m": segment[y_key],
                "lat": point.lat,
                "lon": point.lon,
            })
    return points


# ── 무상태 제어 법칙 헬퍼 (컨트롤러가 재사용) / Stateless control-law helpers ──


def projection_metrics(segment: dict[str, object], x: float, y: float) -> tuple[float, float, float]:
    """현재 위치를 세그먼트에 투영 / Project (x, y) onto the segment line.

    반환: ``(along_m, signed_cross_m, perp_dist_m)`` — 세그먼트 시작에서의 진행
    거리, 부호 있는 횡오차(cross-track; 좌+/우-), 투영점까지의 수직 거리.
    투영 매개변수 t 는 [0,1] 로 clamp 되어 세그먼트 밖으로 나가지 않는다.
    길이 0 세그먼트는 시작점까지 거리로 안전 처리한다. 연속-모션 컨트롤러가
    횡오차 보정에 쓰는 무상태 계산.

    Returns ``(along_m, signed_cross_m, perp_dist_m)``: progress from the start,
    signed cross-track (left+/right-), and perpendicular distance. The
    projection parameter is clamped to [0,1]; zero-length segments degrade to
    distance-from-start. Stateless; reused by the continuous-motion controller.
    """
    sx = float(segment["start_x_m"])
    sy = float(segment["start_y_m"])
    ex = float(segment["end_x_m"])
    ey = float(segment["end_y_m"])
    dx = ex - sx
    dy = ey - sy
    length = math.hypot(dx, dy)
    if length <= 1e-9:
        return 0.0, math.hypot(x - sx, y - sy), 0.0
    raw_t = ((x - sx) * dx + (y - sy) * dy) / (length * length)
    t = clamp(raw_t, 0.0, 1.0)
    proj_x = sx + t * dx
    proj_y = sy + t * dy
    signed_cross = ((x - sx) * dy - (y - sy) * dx) / length
    return length * t, signed_cross, math.hypot(x - proj_x, y - proj_y)


def compute_b_correction(
    *,
    heading_error_deg: float,
    cross_track_error_m: float,
    k_heading: float = 0.006,
    k_cte: float = 0.25,
    max_heading_b: float = 0.08,
    max_cte_b: float = 0.04,
) -> tuple[float, float, float]:
    """헤딩+횡오차를 조향 B 보정으로 변환 / Blend heading+cross-track into a B steering nudge.

    비례(P) 제어: 헤딩 성분과 횡오차(CTE) 성분을 각각 게인·상한으로 clamp 한 뒤
    합치고, 최종 B 는 하드 상한 ±0.08 로 다시 clamp 한다(모터 안전 한계).
    반환: ``(b_correction, heading_component, cte_component)`` — 디버깅용으로
    성분도 함께 돌려준다. 무상태이므로 컨트롤러 루프가 매 틱 호출한다.

    Proportional blend of a heading term and a cross-track term, each clamped to
    its gain/limit, summed, then hard-clamped to ±0.08 (motor safety). Returns
    the total plus both components for debugging. Stateless per control tick.
    """
    heading_component = clamp(k_heading * heading_error_deg, -max_heading_b, max_heading_b)
    cte_component = clamp(k_cte * cross_track_error_m, -max_cte_b, max_cte_b)
    return clamp(heading_component + cte_component, -0.08, 0.08), heading_component, cte_component


def gps_policy_action(gps_degraded: bool, policy: str) -> str:
    """GPS 열화 시 정책 결정 / Map degraded-GPS state + policy to continue/pause/abort.

    GPS 가 정상이면 항상 ``continue``. 열화 시 ``abort``/``pause`` 정책이면 그대로,
    그 외엔 ``continue`` (알 수 없는 정책은 보수적으로 계속 진행이 아니라 기본값).
    Normal GPS => continue; degraded => honor abort/pause, else continue.
    """
    if not gps_degraded:
        return "continue"
    if policy == "abort":
        return "abort"
    if policy == "pause":
        return "pause"
    return "continue"


def manual_override_action(detected: bool, mode: str) -> str:
    """수동 개입 감지 시 동작 결정 / Map manual-override detection + mode to an action.

    :func:`gps_policy_action` 와 같은 형태 — 미감지면 ``continue``, 감지 시
    ``abort``/``pause`` 모드면 그대로, 그 외엔 ``continue``.
    Mirrors :func:`gps_policy_action`: not detected => continue.
    """
    if not detected:
        return "continue"
    if mode == "abort":
        return "abort"
    if mode == "pause":
        return "pause"
    return "continue"
