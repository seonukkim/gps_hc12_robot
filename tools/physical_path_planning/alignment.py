"""Initial heading alignment for supervised physical path execution.

Before the rover follows the first planned lane it must be pointed at that lane's
target heading. IMU relative yaw alone is **not** an absolute ENU heading, so the
absolute current heading is recovered from a short bounded GPS probe (drive
forward a little, measure the GPS displacement). The IMU yaw is then used only as
*turn feedback* to rotate by the shortest signed heading error and stop when the
remaining error is within tolerance.

This module is a peer of :mod:`controller`: it reuses the leaf math
(:mod:`geometry`), telemetry accessors (:mod:`telemetry`), and the serial-facing
bounded executor (:mod:`executor`). The CLI wires it; nothing in the package
imports it back, so there is no cycle.

Three strategies:

* ``gps_probe`` -- automatic: probe forward, estimate heading from GPS
  displacement, rotate with IMU feedback.
* ``user_confirmed`` -- the operator physically points the rover at the A->B
  first lane and presses Enter; the current IMU yaw is captured as the lane
  reference (no probe, no automatic turn).
* ``skip`` -- no alignment is performed.

Every emitted result carries ``ready_for_full_path_following=False`` via
:func:`checks.assert_not_ready_for_full_path_following`.
"""
from __future__ import annotations

import math
import time
from typing import Callable, Sequence

from tools.physical_path_planning import checks, executor, geometry, telemetry

DEFAULT_PROBE_A = 0.25
DEFAULT_PROBE_DURATION_S = 1.0
DEFAULT_MIN_PROBE_DISTANCE_M = 0.30
DEFAULT_HEADING_TOLERANCE_DEG = 8.0
DEFAULT_TURN_B_LEFT = 0.24
DEFAULT_TURN_B_RIGHT = -0.12
DEFAULT_MAX_TURN_DURATION_S = 3.0
DEFAULT_TURN_UPDATE_HZ = 8.0
DEFAULT_TURN_TTL_MS = 350
DEFAULT_TURN_CHUNK_MS = 300
DEFAULT_EVENT_TIMEOUT_S = 3.0
DEFAULT_HEARTBEAT_TIMEOUT_S = 5.0

ALIGNMENT_STRATEGIES = ("gps_probe", "user_confirmed", "skip")


# --- Pure helpers (no serial; directly unit-testable) -------------------------


def first_segment_target_heading(segments: Sequence[dict[str, object]]) -> float:
    """Target heading (ENU degrees) the first planned lane must be aligned to."""
    if not segments:
        raise ValueError("no planned segments: cannot determine target heading")
    return float(segments[0]["target_heading_deg"])


def estimate_heading_deg(
    start_lat: float, start_lon: float, end_lat: float, end_lon: float
) -> tuple[float, float]:
    """Estimate ENU heading + displacement from a start->end GPS fix pair.

    Returns ``(heading_deg, distance_m)`` where ``heading_deg`` is measured CCW
    from East (``atan2(d_north, d_east)``) and matches the planner's segment
    ``target_heading_deg`` convention. Reuses :func:`geometry.goal_to_local`,
    which returns local ``(east, north)`` metres.
    """
    east_m, north_m = geometry.goal_to_local(start_lat, start_lon, end_lat, end_lon)
    distance_m = math.hypot(east_m, north_m)
    heading_deg = math.degrees(math.atan2(north_m, east_m))
    return heading_deg, distance_m


def shortest_heading_error_deg(target_heading_deg: float, current_heading_deg: float) -> float:
    """Shortest signed angular error (target - current), wrapped to [-180, 180]."""
    return geometry.wrap_deg(target_heading_deg - current_heading_deg)


def select_turn_direction(heading_error_deg: float) -> str:
    """Turn ``left`` (B>0, CCW raises ENU heading) for a positive error else ``right``."""
    return "left" if heading_error_deg > 0.0 else "right"


def turn_b_command(turn_direction: str, turn_b_left: float, turn_b_right: float) -> float:
    """Resolve the bounded B steering command for the chosen turn direction."""
    return float(turn_b_left) if turn_direction == "left" else float(turn_b_right)


def remaining_heading_error_deg(initial_heading_error_deg: float, yaw_delta_deg: float) -> float:
    """Signed error left after rotating ``|yaw_delta|`` in the error's direction.

    The turn direction is chosen from the (GPS-derived) heading-error sign and the
    IMU yaw magnitude measures how far we have actually rotated, so the applied
    rotation is ``copysign(|yaw_delta|, initial_error)``.
    """
    applied = math.copysign(abs(yaw_delta_deg), initial_heading_error_deg)
    return geometry.wrap_deg(initial_heading_error_deg - applied)


def within_tolerance(heading_error_deg: float, heading_tolerance_deg: float) -> bool:
    """True when the heading error magnitude is within tolerance."""
    return abs(heading_error_deg) <= abs(heading_tolerance_deg)


def plan_alignment(
    *,
    target_heading_deg: float,
    current_heading_deg: float,
    probe_distance_m: float,
    min_probe_distance_m: float,
    heading_tolerance_deg: float,
    turn_b_left: float,
    turn_b_right: float,
) -> dict[str, object]:
    """Pure alignment decision from a measured heading + probe displacement.

    Returns a dict describing whether a turn is needed, its direction/command, and
    the signed initial heading error -- or a ``PROBE_GPS_DISPLACEMENT_TOO_SMALL``
    failure when the probe did not move far enough to trust the GPS heading.
    """
    if probe_distance_m < min_probe_distance_m:
        return {
            "ok": False,
            "reason": "PROBE_GPS_DISPLACEMENT_TOO_SMALL",
            "initial_heading_error_deg": None,
            "needs_turn": False,
            "turn_direction": "none",
            "turn_b_cmd": 0.0,
        }
    error = shortest_heading_error_deg(target_heading_deg, current_heading_deg)
    if within_tolerance(error, heading_tolerance_deg):
        return {
            "ok": True,
            "reason": "ALREADY_ALIGNED",
            "initial_heading_error_deg": error,
            "needs_turn": False,
            "turn_direction": "none",
            "turn_b_cmd": 0.0,
        }
    direction = select_turn_direction(error)
    return {
        "ok": True,
        "reason": "TURN_REQUIRED",
        "initial_heading_error_deg": error,
        "needs_turn": True,
        "turn_direction": direction,
        "turn_b_cmd": turn_b_command(direction, turn_b_left, turn_b_right),
    }


# --- Result shaping -----------------------------------------------------------


def _base_result(strategy: str, target_heading_deg: float | None) -> dict[str, object]:
    return {
        "mode": "align-heading",
        "strategy": strategy,
        "target_heading_source": "first_segment",
        "target_heading_deg": round(target_heading_deg, 3) if target_heading_deg is not None else None,
        "probe_start_lat": None,
        "probe_start_lon": None,
        "probe_end_lat": None,
        "probe_end_lon": None,
        "probe_distance_m": None,
        "estimated_current_heading_deg": None,
        "initial_heading_error_deg": None,
        "final_heading_error_deg": None,
        "turn_direction": "none",
        "turn_b_cmd": 0.0,
        "turn_duration_ms": 0,
        "aligned_yaw_deg": None,
        "alignment_success": False,
        "ready_for_execute_plan": False,
        "success": False,
        "reason": "NOT_RUN",
        "ready_for_full_path_following": False,
    }


def _finalize(result: dict[str, object]) -> dict[str, object]:
    """Stamp success flags and assert the never-ready guard before returning."""
    result["success"] = bool(result.get("alignment_success"))
    checks.assert_not_ready_for_full_path_following(result)
    return result


# --- Serial-facing routine ----------------------------------------------------


def _wait_for_heartbeat(
    handle: object, raw_lines: list[str], timeout_s: float, *, verbose_raw: bool
) -> dict[str, str] | None:
    return executor.wait_for_row(
        handle,
        raw_lines,
        lambda row: telemetry.event(row) == "HEARTBEAT",
        timeout_s,
        verbose_raw=verbose_raw,
    )


def _row_lat_lon(row: dict[str, str] | None) -> tuple[float | None, float | None]:
    if row is None:
        return None, None
    lat = telemetry._optional_float(row.get("current_lat", row.get("gps_lat")))
    lon = telemetry._optional_float(row.get("current_lon", row.get("gps_lon")))
    return lat, lon


def _execute_alignment_turn(
    handle: object,
    *,
    turn_b_cmd: float,
    yaw_turn_start: float,
    initial_heading_error_deg: float,
    heading_tolerance_deg: float,
    max_turn_duration_s: float,
    update_hz: float,
    ttl_ms: int,
    chunk_ms: int,
    event_timeout_s: float,
    raw_lines: list[str],
    trace: list[dict[str, str]],
    verbose_raw: bool,
) -> dict[str, object]:
    """Rotate with IMU yaw feedback; stop when remaining error is within tolerance.

    Issues bounded ``USB_DRIVE_LIVE_SET`` turn setpoints (A=0, B=turn_b_cmd) and a
    final ``USB_DRIVE_LIVE_STOP``, exactly like :func:`executor.send_live_drive`,
    but breaks early on the IMU stop condition rather than a fixed duration.
    """
    seq = 1
    duration_ms = max(1, int(chunk_ms))
    update_period_s = 1.0 / max(1.0, float(update_hz))
    start_time = time.monotonic()
    deadline = start_time + max(0.0, float(max_turn_duration_s))
    yaw_latest = yaw_turn_start
    remaining = initial_heading_error_deg
    timed_out = False
    rejected = False
    while True:
        if time.monotonic() >= deadline:
            timed_out = True
            break
        executor.write_command(
            handle,
            (
                f"USB_DRIVE_LIVE_SET seq={seq} a=0.000 b={float(turn_b_cmd):.3f} "
                f"duration_ms={duration_ms} ttl_ms={int(ttl_ms)}"
            ),
        )
        row = executor.wait_for_row(
            handle,
            raw_lines,
            lambda r: telemetry.imu_relative_yaw_deg(r) is not None
            or telemetry.event(r) in {"ACTIVE", "REJECT"},
            min(update_period_s, event_timeout_s),
            verbose_raw=verbose_raw,
        )
        if row is not None:
            trace.append(row)
            if telemetry.event(row) == "REJECT":
                rejected = True
                break
            yaw = telemetry.imu_relative_yaw_deg(row)
            if yaw is not None:
                yaw_latest = yaw
                yaw_delta = geometry.wrap_deg(yaw - yaw_turn_start)
                remaining = remaining_heading_error_deg(initial_heading_error_deg, yaw_delta)
                if within_tolerance(remaining, heading_tolerance_deg):
                    break
        else:
            time.sleep(min(update_period_s, 0.05))
    executor.write_command(handle, f"USB_DRIVE_LIVE_STOP seq={seq}")
    executor.wait_for_event(
        handle, raw_lines, executor.STOP_CONFIRM_EVENTS, event_timeout_s, verbose_raw=verbose_raw
    )
    turn_duration_ms = int((time.monotonic() - start_time) * 1000.0)
    return {
        "yaw_final": yaw_latest,
        "final_heading_error_deg": remaining,
        "turn_duration_ms": turn_duration_ms,
        "timed_out": timed_out,
        "rejected": rejected,
    }


def align_heading(
    handle: object,
    *,
    segments: Sequence[dict[str, object]],
    strategy: str = "gps_probe",
    target_heading_source: str = "first_segment",
    probe_a: float = DEFAULT_PROBE_A,
    probe_duration_s: float = DEFAULT_PROBE_DURATION_S,
    min_probe_distance_m: float = DEFAULT_MIN_PROBE_DISTANCE_M,
    heading_tolerance_deg: float = DEFAULT_HEADING_TOLERANCE_DEG,
    turn_b_left: float = DEFAULT_TURN_B_LEFT,
    turn_b_right: float = DEFAULT_TURN_B_RIGHT,
    max_turn_duration_s: float = DEFAULT_MAX_TURN_DURATION_S,
    update_hz: float = DEFAULT_TURN_UPDATE_HZ,
    ttl_ms: int = DEFAULT_TURN_TTL_MS,
    chunk_ms: int = DEFAULT_TURN_CHUNK_MS,
    event_timeout_s: float = DEFAULT_EVENT_TIMEOUT_S,
    heartbeat_timeout_s: float = DEFAULT_HEARTBEAT_TIMEOUT_S,
    input_fn: Callable[[str], str] = input,
    raw_lines: list[str] | None = None,
    verbose_raw: bool = True,
) -> tuple[dict[str, object], list[dict[str, str]]]:
    """Align the rover to the first lane heading; return ``(result, trace_rows)``.

    Never raises on the expected field faults (no GPS, no IMU, tiny probe): each
    sets ``reason`` and returns a guarded result with ``alignment_success=False``
    so the caller can still write a summary and decide whether to abort.
    """
    if raw_lines is None:
        raw_lines = []
    trace: list[dict[str, str]] = []
    target_heading = first_segment_target_heading(segments)
    result = _base_result(strategy, target_heading)
    result["target_heading_source"] = target_heading_source

    if strategy == "skip":
        result["reason"] = "ALIGNMENT_SKIPPED"
        result["alignment_success"] = True
        result["ready_for_execute_plan"] = True
        return _finalize(result), trace

    if strategy == "user_confirmed":
        input_fn(
            "Point the rover toward the A->B first lane heading, then press Enter to capture IMU yaw..."
        )
        heartbeat = _wait_for_heartbeat(handle, raw_lines, heartbeat_timeout_s, verbose_raw=verbose_raw)
        if heartbeat is not None:
            trace.append(heartbeat)
        yaw = telemetry.imu_relative_yaw_deg(heartbeat) if heartbeat else None
        result["estimated_current_heading_deg"] = round(target_heading, 3)
        result["initial_heading_error_deg"] = 0.0
        result["final_heading_error_deg"] = 0.0
        result["turn_direction"] = "user_confirmed"
        result["aligned_yaw_deg"] = round(yaw, 3) if yaw is not None else None
        result["alignment_success"] = True
        result["ready_for_execute_plan"] = True
        result["reason"] = "USER_CONFIRMED_HEADING" if yaw is not None else "USER_CONFIRMED_HEADING_NO_IMU"
        return _finalize(result), trace

    if strategy != "gps_probe":
        result["reason"] = "UNKNOWN_ALIGNMENT_STRATEGY"
        return _finalize(result), trace

    # gps_probe: 1) read GPS+IMU before the probe.
    before = _wait_for_heartbeat(handle, raw_lines, heartbeat_timeout_s, verbose_raw=verbose_raw)
    if before is not None:
        trace.append(before)
    start_lat, start_lon = _row_lat_lon(before)
    if start_lat is None or start_lon is None:
        result["reason"] = "PROBE_GPS_UNAVAILABLE_BEFORE"
        return _finalize(result), trace
    result["probe_start_lat"] = round(start_lat, 7)
    result["probe_start_lon"] = round(start_lon, 7)

    # 2) Short bounded forward probe (usb-drive-live style; A only, B neutral).
    probe_rows = executor.send_live_drive(
        handle,
        seq=1,
        duration_s=float(probe_duration_s),
        update_hz=float(update_hz),
        ttl_ms=int(ttl_ms),
        command_fn=lambda _row: (float(probe_a), 0.0),
        raw_lines=raw_lines,
        event_timeout_s=float(event_timeout_s),
        verbose_raw=verbose_raw,
    )
    trace.extend(probe_rows)

    # 3) Read GPS after the probe.
    after = _wait_for_heartbeat(handle, raw_lines, heartbeat_timeout_s, verbose_raw=verbose_raw)
    if after is not None:
        trace.append(after)
    end_lat, end_lon = _row_lat_lon(after)
    if end_lat is None or end_lon is None:
        result["reason"] = "PROBE_GPS_UNAVAILABLE_AFTER"
        return _finalize(result), trace
    result["probe_end_lat"] = round(end_lat, 7)
    result["probe_end_lon"] = round(end_lon, 7)

    # 4) Estimate the absolute current heading from the GPS displacement.
    current_heading, distance_m = estimate_heading_deg(start_lat, start_lon, end_lat, end_lon)
    result["probe_distance_m"] = round(distance_m, 4)
    result["estimated_current_heading_deg"] = round(current_heading, 3)

    decision = plan_alignment(
        target_heading_deg=target_heading,
        current_heading_deg=current_heading,
        probe_distance_m=distance_m,
        min_probe_distance_m=float(min_probe_distance_m),
        heading_tolerance_deg=float(heading_tolerance_deg),
        turn_b_left=float(turn_b_left),
        turn_b_right=float(turn_b_right),
    )
    if not decision["ok"]:
        result["reason"] = str(decision["reason"])
        result["next_recommended_action"] = (
            "Increase --probe-duration-s/--probe-a so the GPS probe moves at least "
            "--min-probe-distance-m, or use --strategy user_confirmed."
        )
        return _finalize(result), trace

    initial_error = float(decision["initial_heading_error_deg"])
    result["initial_heading_error_deg"] = round(initial_error, 3)
    yaw_after_probe = telemetry.imu_relative_yaw_deg(after)

    if not decision["needs_turn"]:
        result["final_heading_error_deg"] = round(initial_error, 3)
        result["aligned_yaw_deg"] = round(yaw_after_probe, 3) if yaw_after_probe is not None else None
        result["alignment_success"] = True
        result["ready_for_execute_plan"] = True
        result["reason"] = "ALREADY_ALIGNED"
        return _finalize(result), trace

    # 5) A turn is required: IMU yaw is mandatory as turn feedback for gps_probe.
    if yaw_after_probe is None:
        result["reason"] = "IMU_UNAVAILABLE_FOR_ALIGNMENT"
        result["next_recommended_action"] = (
            "Enable IMU telemetry (BMI160) or use --strategy user_confirmed to point the rover manually."
        )
        return _finalize(result), trace

    turn_direction = str(decision["turn_direction"])
    turn_b_cmd = float(decision["turn_b_cmd"])
    result["turn_direction"] = turn_direction
    result["turn_b_cmd"] = round(turn_b_cmd, 3)

    turn = _execute_alignment_turn(
        handle,
        turn_b_cmd=turn_b_cmd,
        yaw_turn_start=float(yaw_after_probe),
        initial_heading_error_deg=initial_error,
        heading_tolerance_deg=float(heading_tolerance_deg),
        max_turn_duration_s=float(max_turn_duration_s),
        update_hz=float(update_hz),
        ttl_ms=int(ttl_ms),
        chunk_ms=int(chunk_ms),
        event_timeout_s=float(event_timeout_s),
        raw_lines=raw_lines,
        trace=trace,
        verbose_raw=verbose_raw,
    )
    final_error = float(turn["final_heading_error_deg"])
    result["final_heading_error_deg"] = round(final_error, 3)
    result["turn_duration_ms"] = int(turn["turn_duration_ms"])
    result["aligned_yaw_deg"] = round(float(turn["yaw_final"]), 3)
    success = within_tolerance(final_error, heading_tolerance_deg) and not turn["rejected"]
    result["alignment_success"] = success
    result["ready_for_execute_plan"] = success
    if turn["rejected"]:
        result["reason"] = "ALIGN_TURN_REJECTED"
    elif success:
        result["reason"] = "ALIGNED"
    elif turn["timed_out"]:
        result["reason"] = "ALIGN_TURN_TIMEOUT_BEFORE_TOLERANCE"
    else:
        result["reason"] = "ALIGN_TURN_INCOMPLETE"
    return _finalize(result), trace
