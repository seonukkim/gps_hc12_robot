"""Rectangle-from-diagonal serpentine coverage geometry, goal/start resolution, and
the stateless control math (cross-track projection + heading/CTE correction).

A->B is treated as the rectangle *diagonal*; ``--path-shape direct_line`` uses
:func:`build_direct_segments`. The control-law helpers (:func:`projection_metrics`,
:func:`compute_b_correction`) are stateless and reused by the continuous-motion
controller. ``ready_for_full_path_following`` stays False on every emitted primitive.
"""
from __future__ import annotations

import math
from typing import Sequence

from gps_coverage_core.geo import GeoPoint, LocalPoint, latlon_to_local, local_to_latlon
from tools.physical_path_planning import calibration as calibration_resolver

FALLBACK_RESOLVED_CALIBRATION = calibration_resolver.resolve_physical_calibration(
    motion_calibration_json=None,
    fine_calibration_json=None,
    turn_calibration_json=None,
    turn_angle_calibration_json=None,
    smooth_turn_calibration_json=None,
    calibration_mode="repeated_pulses",
)


def _resolved_or_fallback(calibration: dict[str, object] | None) -> dict[str, object]:
    return calibration if calibration is not None else FALLBACK_RESOLVED_CALIBRATION


def _motion_calibrated(calibration: dict[str, object] | None, direction: str) -> dict[str, object]:
    name = "forward" if direction == "forward" else "backward"
    return calibration_resolver.planner_primitive(_resolved_or_fallback(calibration), name)


def _turn_calibrated(calibration: dict[str, object] | None, direction: str) -> dict[str, object]:
    return calibration_resolver.connector_primitive(_resolved_or_fallback(calibration), direction)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_deg(angle_deg: float) -> float:
    while angle_deg > 180.0:
        angle_deg -= 360.0
    while angle_deg < -180.0:
        angle_deg += 360.0
    return angle_deg


def validate_lat_lon(lat: float, lon: float, label: str) -> None:
    if not -90.0 <= lat <= 90.0:
        raise ValueError(f"{label} latitude must be between -90 and 90")
    if not -180.0 <= lon <= 180.0:
        raise ValueError(f"{label} longitude must be between -180 and 180")


def goal_to_local(start_lat: float, start_lon: float, goal_lat: float, goal_lon: float) -> tuple[float, float]:
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


def _unit(x: float, y: float) -> tuple[float, float]:
    length = math.hypot(x, y)
    if length <= 1e-12:
        raise ValueError("zero-length vector")
    return x / length, y / length


def _rot_cw(x: float, y: float) -> tuple[float, float]:
    return y, -x


def _rot_ccw(x: float, y: float) -> tuple[float, float]:
    return -y, x


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
    if width_m <= 0:
        raise ValueError("workspace width is required because A-B is a diagonal, not a direct path")
    if step_spacing_m <= 0:
        raise ValueError("step_spacing_m must be > 0")
    bx, by = goal_to_local(start_lat, start_lon, goal_lat, goal_lon)
    diagonal_length = math.hypot(bx, by)
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


def _track_offsets(width_m: float, step_spacing_m: float) -> list[float]:
    offsets = [0.0]
    value = step_spacing_m
    while value < width_m and not math.isclose(value, width_m, abs_tol=1e-9):
        offsets.append(value)
        value += step_spacing_m
    if not math.isclose(offsets[-1], width_m, abs_tol=1e-9):
        offsets.append(width_m)
    return offsets


def _point_on_rectangle(workspace: dict[str, object], along_m: float, across_m: float) -> tuple[float, float]:
    u = workspace["long_axis_unit"]  # type: ignore[index]
    v = workspace["short_axis_unit"]  # type: ignore[index]
    ux = float(u["x"])  # type: ignore[index]
    uy = float(u["y"])  # type: ignore[index]
    vx = float(v["x"])  # type: ignore[index]
    vy = float(v["y"])  # type: ignore[index]
    return along_m * ux + across_m * vx, along_m * uy + across_m * vy


def build_serpentine_segments(
    workspace: dict[str, object],
    *,
    max_segment_pulses: int,
    nominal_forward_pulse_m: float,
    calibration: dict[str, object] | None = None,
) -> tuple[list[dict[str, object]], list[dict[str, object]], list[dict[str, object]]]:
    length_m = float(workspace["workspace_length_m"])
    width_m = float(workspace["workspace_width_m"])
    step_spacing_m = float(workspace["step_spacing_m"])
    offsets = _track_offsets(width_m, step_spacing_m)
    segments: list[dict[str, object]] = []
    primitives: list[dict[str, object]] = []
    segment_index = 0
    primitive_index = 0
    for lane_index, offset in enumerate(offsets):
        forward = lane_index % 2 == 0
        start_along = 0.0 if forward else length_m
        end_along = length_m if forward else 0.0
        sx, sy = _point_on_rectangle(workspace, start_along, offset)
        ex, ey = _point_on_rectangle(workspace, end_along, offset)
        heading = math.degrees(math.atan2(ey - sy, ex - sx))
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
            segment_index += 1
            primitive_index += 1
            connector_heading = math.degrees(math.atan2(cey - csy, cex - csx))
            turn_direction = "turn_left" if forward else "turn_right"
            segments.append({
                "segment_index": segment_index,
                "segment_type": "connector_turn",
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
    primitives: list[dict[str, object]] = []
    for primitive_index, segment in enumerate(segments, start=1):
        segment_type = str(segment.get("segment_type", ""))
        expected = str(segment.get("expected_motion_direction", "forward"))
        if segment_type == "connector_turn":
            direction = "left" if expected == "turn_left" else "right"
            calibrated = _turn_calibrated(calibration, direction)
            repeat_count = 1
            if str(calibration.get("connector_mode_effective", "repeated_pulses")) == "repeated_pulses":
                repeat_count = int(calibration.get("left_fixed_pulses" if direction == "left" else "right_fixed_pulses", 12))
            primitive_type = "turn_left" if direction == "left" else "turn_right"
            connector_mode = calibrated.get("connector_mode", calibration.get("connector_mode_effective", "repeated_pulses"))
        else:
            direction = "backward" if expected == "backward" or segment_type.startswith("backward") else "forward"
            calibrated = _motion_calibrated(calibration, direction)
            repeat_count = int(segment.get("pulse_budget", 1))
            primitive_type = "move_backward" if direction == "backward" else "move_forward"
            connector_mode = "lane"
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


def projection_metrics(segment: dict[str, object], x: float, y: float) -> tuple[float, float, float]:
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
    heading_component = clamp(k_heading * heading_error_deg, -max_heading_b, max_heading_b)
    cte_component = clamp(k_cte * cross_track_error_m, -max_cte_b, max_cte_b)
    return clamp(heading_component + cte_component, -0.08, 0.08), heading_component, cte_component


def gps_policy_action(gps_degraded: bool, policy: str) -> str:
    if not gps_degraded:
        return "continue"
    if policy == "abort":
        return "abort"
    if policy == "pause":
        return "pause"
    return "continue"


def manual_override_action(detected: bool, mode: str) -> str:
    if not detected:
        return "continue"
    if mode == "abort":
        return "abort"
    if mode == "pause":
        return "pause"
    return "continue"
