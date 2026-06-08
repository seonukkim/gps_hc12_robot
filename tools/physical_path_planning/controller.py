"""Continuous-motion controller: the supervised pulse loop that follows a planned
serpentine/direct path toward the goal.

This is the one genuinely new module in the consolidation, but it invents no new
control law. Each step REUSES the already-tested pieces:

* :func:`executor.send_pulse` issues the guarded ARM->command->STOP pulse;
* :func:`geometry.projection_metrics` + :func:`geometry.compute_b_correction`
  produce the steering correction -- applied to the **B axis only**, clamped to
  +/-0.08; the forward A command and pulse duration come straight from
  calibration and are **never lowered** by the controller;
* :func:`geometry.gps_policy_action` decides what to do when GPS degrades.

New (loop-level) behavior layered on top of those primitives:

* IMU yaw is sampled on every heartbeat to hold the segment's target heading
  between pulses (``current_heading`` is the segment heading plus the measured
  yaw delta from the start yaw).
* GPS degraded (``BAD_HDOP`` / no fix / cached) -> by default *continue
  dead-reckoned* from the last good fix and flag ``gps_degraded=True``; it does
  not silently stop. The policy is configurable (continue/pause/abort).
* Serial disconnect mid-loop -> hard abort (no traceback): the loop records
  ``abort_reason`` and stops.
* ``RC_INVALID`` reported during the active pulse -> hard abort.
* RC sticks not neutral before a pulse -> wait up to ``rc_neutral_wait_s``
  (default 5s) for neutral; if it never settles, abort rather than pulse.
* Connectors prefer angle-calibrated turns; when no turn-angle calibration
  exists the resolver's ``repeated_pulses`` fallback is used and
  ``fallback_to_repeated_pulses`` is surfaced. The controller never invents 15
  degree micro-turns as a preferred connector.

Every emitted summary is routed through
:func:`checks.assert_not_ready_for_full_path_following`, so the controller can
never claim full-path-following readiness.
"""
from __future__ import annotations

import math
from typing import Sequence

from tools.physical_path_planning import checks, executor, geometry, safety, telemetry

DEFAULT_EVENT_TIMEOUT_S = 3.0
DEFAULT_HEARTBEAT_TIMEOUT_S = 3.0
DEFAULT_RC_NEUTRAL_WAIT_S = 5.0
DEFAULT_GPS_DEGRADATION_POLICY = "continue"
DEFAULT_MANUAL_OVERRIDE_MODE = "abort"
_ZERO_TOLERANCE = 1e-9
DEFAULT_PATH_CONTROL_MODE = "gps_imu_closed_loop"
PATH_CONTROL_MODES = {"open_loop_chunks", "imu_heading", "gps_imu_closed_loop"}

# Abort reason raised when the operator flips the physical mode switch back to
# MANUAL during an AUTO-triggered run (auto-relative-run).
MANUAL_SWITCH_ABORT_REASON = "USER_SWITCHED_TO_MANUAL"


# --- Pure decision helpers (no serial; directly unit-testable) ----------------


def _row_lat_lon(row: dict[str, str] | None) -> tuple[float | None, float | None]:
    """Latitude/longitude from a telemetry row, accepting ``current_*``/``gps_*``."""
    if row is None:
        return None, None
    lat = telemetry._optional_float(row.get("current_lat", row.get("gps_lat")))
    lon = telemetry._optional_float(row.get("current_lon", row.get("gps_lon")))
    return lat, lon


def mode_switch_state(row: dict[str, str] | None) -> str:
    """Physical PPM mode switch state from a heartbeat row: AUTO / MANUAL / ABSENT.

    Prefers the firmware's ``mode_switch=AUTO|MANUAL`` string; falls back to the
    ``auto_sw`` boolean gated by ``mode_channel_present``. Returns ``ABSENT`` when
    no usable mode channel is reported (no PPM receiver), which lets the caller
    offer the keyboard-start fallback instead of waiting forever for a switch.
    """
    if row is None:
        return "ABSENT"
    label = str(row.get("mode_switch", "")).strip().upper()
    if label in {"AUTO", "MANUAL"}:
        return label
    present_value = row.get("mode_channel_present")
    if present_value is not None and not telemetry._parse_bool(present_value, default=False):
        return "ABSENT"
    auto_value = row.get("auto_sw")
    if auto_value is None:
        return "ABSENT"
    return "AUTO" if telemetry._parse_bool(auto_value, default=False) else "MANUAL"


def dead_reckon_gps(row: dict[str, str] | None, cache: dict[str, object]) -> dict[str, object]:
    """Resolve usable lat/lon for this step, dead-reckoning from cache when degraded.

    Mirrors the legacy stage35 ``_gps_fields`` semantics: a fresh fix updates the
    cache and clears degradation; a missing/blocked fix falls back to the last
    cached fix (``gps_cached_used``); ``BAD_HDOP`` or any cache use marks
    ``gps_degraded`` so the caller can flag the step without stopping.
    """
    lat, lon = _row_lat_lon(row)
    reason = (row or {}).get("gps_block_reason", "NA")
    cached_used = False
    recovered = False
    if lat is not None and lon is not None:
        recovered = bool(cache.get("degraded"))
        cache["lat"] = lat
        cache["lon"] = lon
        cache["degraded"] = False
    elif cache.get("lat") is not None and cache.get("lon") is not None:
        lat = float(cache["lat"])
        lon = float(cache["lon"])
        cached_used = True
        cache["degraded"] = True
    else:
        cache["degraded"] = True
    degraded = reason == "BAD_HDOP" or cached_used or lat is None or lon is None
    return {
        "lat": lat,
        "lon": lon,
        "gps_cached_used": cached_used,
        "gps_degraded": degraded,
        "gps_recovered": recovered,
    }


def current_heading_deg(
    target_heading_deg: float, yaw: float | None, start_yaw_deg: float | None
) -> float:
    """Segment heading adjusted by the measured yaw drift since the start yaw."""
    if yaw is None or start_yaw_deg is None:
        return target_heading_deg
    return geometry.wrap_deg(target_heading_deg + geometry.wrap_deg(yaw - start_yaw_deg))


def reference_yaw_for_segment(
    heartbeat_row: dict[str, str] | None,
    *,
    provided_start_yaw: float | None,
    use_provided: bool,
) -> float | None:
    """Resolve the IMU yaw reference a lane holds its heading against.

    Heading-hold measures drift away from the orientation the robot had when it
    started the lane, so the reference must be re-captured per lane (after each
    connector turn the absolute yaw has changed by ~90 degrees).

    * ``use_provided`` (the first lane) with an explicit ``--start-yaw-deg`` keeps
      that operator-supplied reference.
    * Otherwise the lane's first heartbeat yaw becomes the reference -- this is
      what makes IMU heading correction live when no ``--start-yaw-deg`` is given
      (the field default), instead of silently producing a zero heading error.
    * When no IMU yaw is available the provided value (possibly ``None``) is used,
      which disables heading correction for that lane rather than guessing.
    """
    if use_provided and provided_start_yaw is not None:
        return provided_start_yaw
    yaw = telemetry.imu_relative_yaw_deg(heartbeat_row) if heartbeat_row else None
    if yaw is not None:
        return yaw
    return provided_start_yaw


def pulse_correction(
    *,
    segment: dict[str, object],
    x: float,
    y: float,
    target_heading_deg: float,
    yaw: float | None,
    start_yaw_deg: float | None,
    is_connector: bool = False,
    base_b_cmd: float = 0.0,
    connector_b_cmd: float = 0.0,
    imu_heading_hold: bool = True,
    cross_track_correction: bool = True,
    path_control_mode: str = DEFAULT_PATH_CONTROL_MODE,
    k_heading: float = 0.006,
    k_cross_track: float = 0.20,
    max_correction_b: float = 0.08,
) -> dict[str, float]:
    """Compute the per-pulse steering correction (B axis only).

    For lane segments the B command is the heading+cross-track correction from
    :func:`geometry.compute_b_correction` (clamped to +/-0.08). For connectors the
    B command is the calibrated turn value and the correction components are zero
    (a connector is a deliberate turn, not a steering nudge).
    """
    if path_control_mode not in PATH_CONTROL_MODES:
        path_control_mode = DEFAULT_PATH_CONTROL_MODE
    heading_enabled = imu_heading_hold and path_control_mode in {"imu_heading", "gps_imu_closed_loop"}
    cte_enabled = cross_track_correction and path_control_mode == "gps_imu_closed_loop"
    heading = current_heading_deg(target_heading_deg, yaw if heading_enabled else None, start_yaw_deg)
    heading_error = geometry.wrap_deg(target_heading_deg - heading)
    along, signed_cte, _ = geometry.projection_metrics(segment, x, y)
    if not heading_enabled:
        heading_error = 0.0
    if not cte_enabled:
        signed_cte = 0.0
    b_heading = float(k_heading) * heading_error
    b_cte = float(k_cross_track) * signed_cte
    correction = geometry.clamp(b_heading + b_cte, -abs(max_correction_b), abs(max_correction_b))
    if is_connector:
        b_cmd = float(connector_b_cmd)
        b_heading = 0.0
        b_cte = 0.0
    else:
        b_cmd = geometry.clamp(
            float(base_b_cmd) + correction,
            -abs(max_correction_b),
            abs(max_correction_b),
        )
    if is_connector:
        correction_source = "connector_calibration"
    elif path_control_mode == "open_loop_chunks":
        correction_source = "open_loop"
    elif path_control_mode == "imu_heading":
        correction_source = "imu_heading"
    else:
        correction_source = "gps_imu"
    return {
        "current_heading_deg": heading,
        "heading_error_deg": heading_error,
        "cross_track_error_m": signed_cte,
        "along_track_progress_m": along,
        "remaining_distance_m": max(0.0, float(segment.get("length_m", 0.0)) - along),
        "b_cmd": b_cmd,
        "b_heading_component": b_heading,
        "b_cte_component": b_cte,
        "correction_source": correction_source,
    }


def connector_command(
    resolved_calibration: dict[str, object], direction: str
) -> dict[str, object]:
    """Select the connector turn primitive, surfacing the repeated-pulses fallback.

    Prefers the angle-calibrated 90 degree turn; when no turn-angle calibration
    exists the resolver's ``repeated_pulses`` fallback is used and
    ``fallback_to_repeated_pulses`` is reported True so the caller (and operator)
    can see that connectors are uncalibrated.
    """
    primitive = geometry._turn_calibrated(resolved_calibration, direction)
    effective = str(resolved_calibration.get("connector_mode_effective", "repeated_pulses"))
    fallback = effective == "repeated_pulses"
    return {
        "a_cmd": float(primitive["a_cmd"]),
        "b_cmd": float(primitive["b_cmd"]),
        "pulse_ms": int(primitive["pulse_ms"]),
        "calibration_source": str(primitive.get("calibration_source", "unknown")),
        "connector_mode": effective,
        "fallback_to_repeated_pulses": fallback,
    }


def manual_override_detected(row: dict[str, str] | None) -> bool:
    """True => the operator has taken manual RC control (sticks off neutral / RC source)."""
    if row is None:
        return False
    if telemetry._parse_bool(row.get("neutral_ok"), default=True) is False:
        return True
    for key in ("manual_forward_cmd", "manual_turn_cmd"):
        value = telemetry._optional_float(row.get(key))
        if value is not None and abs(value) > 1e-4:
            return True
    if row.get("control_source") == "RC_MANUAL" and telemetry.physical_output_active(row):
        return True
    return False


def rc_ignored_for_usb_supervised(row: dict[str, str] | None) -> bool:
    """True when the active Mac USB motion firmware explicitly ignores RC input."""
    if row is None:
        return False
    return any(
        telemetry._parse_bool(row.get(key))
        for key in (
            "usb_pulse_test_ignore_rc_input",
            "usb_drive_live_ignore_rc_input",
            "usb_drive_live_mode",
            "usb_pulse_test_mode",
            "mac_physical_supervised",
        )
    ) or str(row.get("firmware_profile", "")).upper() == "MAC_PHYSICAL_SUPERVISED"


def rc_warning_for_usb_supervised(row: dict[str, str] | None) -> str:
    """Return non-fatal RC warning for USB-supervised modes."""
    if row is None or not rc_ignored_for_usb_supervised(row):
        return "NONE"
    if telemetry._parse_bool(row.get("rc_ok")) is not True:
        return "RC_NOT_OK_IGNORED_FOR_MAC_USB_SUPERVISED_MODE"
    if telemetry._parse_bool(row.get("neutral_ok"), default=True) is not True:
        return "RC_NOT_NEUTRAL_IGNORED_FOR_MAC_USB_SUPERVISED_MODE"
    return "NONE"


_USB_PULSE_COMPAT_MODE_KEY = "stage" + "20_physical_ab_guarded_crawl"
_USB_PULSE_COMPAT_READY_KEY = "stage" + "20_firmware_ready"


def guarded_pulse_compatible(row: dict[str, str]) -> bool:
    """True => the firmware heartbeat exposes guarded physical A/B pulse mode."""
    return (
        telemetry._parse_bool(row.get("usb_pulse_test_mode")) is True
        or telemetry._parse_bool(row.get(_USB_PULSE_COMPAT_MODE_KEY)) is True
        or (
            (
                telemetry._parse_bool(row.get("usb_pulse_ready")) is True
                or telemetry._parse_bool(row.get(_USB_PULSE_COMPAT_READY_KEY)) is True
            )
            and row.get("physical_a_role", "throttle") == "throttle"
            and row.get("physical_b_role", "turn") == "turn"
        )
    )


def pulse_block_reason(pulse_rows: Sequence[dict[str, str]]) -> str | None:
    """Return the abort/invalid reason for a completed pulse window, else None.

    Ordered so the most safety-critical condition wins: an RC_INVALID reject
    aborts the run; a missing ACK/STOP handshake, output still active after STOP,
    or nonzero final wheel commands each invalidate the pulse.
    """
    if safety.rc_invalid_abort(pulse_rows):
        return "RC_INVALID"
    missing = safety.missing_ack_or_stop_abort(pulse_rows)
    if missing is not None:
        return missing
    if safety.output_active_after_stop(pulse_rows):
        return "OUTPUT_ACTIVE_AFTER_STOP"
    if safety.nonzero_final_cmd(pulse_rows):
        return "FINAL_COMMANDS_NONZERO"
    return None


def live_drive_block_reason(rows: Sequence[dict[str, str]]) -> str | None:
    if safety.rc_invalid_abort(rows):
        return "RC_INVALID"
    if any(telemetry.event(row) == "REJECT" for row in rows):
        return safety.latest_reject_reason(rows)
    if not any(telemetry.event(row) == "ACTIVE" for row in rows):
        return "ACTIVE_MISSING"
    if not any(telemetry.event(row) in safety.STOP_EVENTS for row in rows):
        return "STOP_MISSING"
    if safety.output_active_after_stop(rows):
        return "OUTPUT_ACTIVE_AFTER_STOP"
    if safety.nonzero_final_cmd(rows):
        return "FINAL_COMMANDS_NONZERO"
    return None


def _last_row_value(rows: Sequence[dict[str, str]], key: str) -> str | None:
    for row in reversed(rows):
        value = row.get(key)
        if value not in (None, "", "NA"):
            return value
    return None


def _final_zero(rows: Sequence[dict[str, str]]) -> bool:
    left = telemetry._optional_float(_last_row_value(rows, "final_left_cmd"))
    right = telemetry._optional_float(_last_row_value(rows, "final_right_cmd"))
    if left is None and right is None:
        return False
    return abs(left or 0.0) <= 1e-3 and abs(right or 0.0) <= 1e-3


def _segment_point(segment: dict[str, object], along_m: float) -> tuple[float, float]:
    sx = float(segment["start_x_m"])
    sy = float(segment["start_y_m"])
    ex = float(segment["end_x_m"])
    ey = float(segment["end_y_m"])
    length = math.hypot(ex - sx, ey - sy)
    if length <= 1e-9:
        return sx, sy
    t = geometry.clamp(along_m / length, 0.0, 1.0)
    return sx + (ex - sx) * t, sy + (ey - sy) * t


def _dead_reckon_pose_along_segment(
    *,
    segment: dict[str, object],
    x_m: float,
    y_m: float,
    advance_m: float,
) -> tuple[float, float]:
    along, signed_cte, _ = geometry.projection_metrics(segment, x_m, y_m)
    new_along = min(float(segment.get("length_m", 0.0)), along + max(0.0, advance_m))
    px, py = _segment_point(segment, new_along)
    sx = float(segment["start_x_m"])
    sy = float(segment["start_y_m"])
    ex = float(segment["end_x_m"])
    ey = float(segment["end_y_m"])
    length = math.hypot(ex - sx, ey - sy)
    if length <= 1e-9:
        return px, py
    # Signed cross-track uses the left-normal convention from projection_metrics.
    nx = -(ey - sy) / length
    ny = (ex - sx) / length
    return px + nx * signed_cte, py + ny * signed_cte


def _pose_from_gps_or_dead_reckon(
    *,
    row: dict[str, str] | None,
    gps: dict[str, object],
    pose_state: dict[str, object],
    segment: dict[str, object],
    start_lat: float,
    start_lon: float,
    advance_m: float = 0.0,
    gps_reanchor: bool = True,
) -> dict[str, object]:
    lat = gps.get("lat")
    lon = gps.get("lon")
    gps_valid = bool(lat is not None and lon is not None and not gps.get("gps_degraded"))
    gps_reanchored = False
    if gps_valid and gps_reanchor:
        x, y = geometry.goal_to_local(start_lat, start_lon, float(lat), float(lon))
        gps_reanchored = bool(pose_state.get("gps_degraded") or pose_state.get("source") != "gps")
        pose_state.update({"x": x, "y": y, "source": "gps", "gps_degraded": False})
    elif gps_valid and pose_state.get("x") is None:
        x, y = geometry.goal_to_local(start_lat, start_lon, float(lat), float(lon))
        pose_state.update({"x": x, "y": y, "source": "gps", "gps_degraded": False})
    else:
        x = float(pose_state.get("x", float(segment.get("start_x_m", 0.0))) or 0.0)
        y = float(pose_state.get("y", float(segment.get("start_y_m", 0.0))) or 0.0)
        if advance_m > 0.0:
            x, y = _dead_reckon_pose_along_segment(
                segment=segment,
                x_m=x,
                y_m=y,
                advance_m=advance_m,
            )
        pose_state.update({"x": x, "y": y, "source": "dead_reckoning", "gps_degraded": True})
    return {
        "x": float(pose_state.get("x", 0.0)),
        "y": float(pose_state.get("y", 0.0)),
        "gps_valid": gps_valid,
        "gps_reanchored": gps_reanchored,
        "source": pose_state.get("source", "dead_reckoning"),
    }


def planned_pulse(
    *, seq: int, a_cmd: float, b_cmd: float, pulse_ms: int
) -> dict[str, object]:
    """Build the ARM/CMD/STOP command texts for :func:`executor.send_pulse`."""
    return {
        "arm_command_text": f"USB_PULSE_TEST_ARM seq={seq}",
        "command_text": (
            f"USB_PULSE_TEST_CMD seq={seq} a={a_cmd:.3f} b={b_cmd:.3f} ms={int(pulse_ms)}"
        ),
        "stop_command_text": f"USB_PULSE_TEST_STOP seq={seq}",
        "pulse_ms": int(pulse_ms),
        "force_stop_command": True,
    }


# --- Serial-facing waits (small; exercised with a fake handle) -----------------


def _is_guarded_pulse_heartbeat(row: dict[str, str]) -> bool:
    return telemetry.event(row) == "HEARTBEAT" and guarded_pulse_compatible(row)


def wait_for_guarded_pulse_heartbeat(
    handle: object, raw_lines: list[str], timeout_s: float
) -> dict[str, str] | None:
    return executor.wait_for_row(handle, raw_lines, _is_guarded_pulse_heartbeat, timeout_s)


def wait_for_neutral_rc(
    handle: object, raw_lines: list[str], timeout_s: float
) -> dict[str, str] | None:
    """Wait for a guarded pulse heartbeat whose RC sticks are neutral and ready."""
    return executor.wait_for_row(
        handle,
        raw_lines,
        lambda row: _is_guarded_pulse_heartbeat(row) and not safety.rc_neutral_wait(row),
        timeout_s,
    )


# --- Execution row + summary --------------------------------------------------


def build_execution_row(
    *,
    segment: dict[str, object],
    primitive_index: int,
    after_row: dict[str, str] | None,
    pulse_rows: Sequence[dict[str, str]],
    start_lat: float,
    start_lon: float,
    goal_lat: float,
    goal_lon: float,
    start_yaw_deg: float | None,
    target_heading_deg: float,
    a_cmd: float,
    correction: dict[str, float],
    pulse_ms: int,
    gps: dict[str, object],
    calibration_source: str,
    connector_mode: str,
    block_reason_override: str | None = None,
    drive_mode: str = "pulse",
    chunk_index: int | None = None,
    live_chunk_ms: int | None = None,
    max_ms: int | None = None,
    path_control_mode: str = DEFAULT_PATH_CONTROL_MODE,
    pose: dict[str, object] | None = None,
    base_a_cmd: float | None = None,
    b_trim: float = 0.0,
) -> dict[str, object]:
    if drive_mode == "continuous":
        block_reason = block_reason_override
    else:
        block_reason = block_reason_override if block_reason_override is not None else pulse_block_reason(pulse_rows)
    yaw = telemetry.imu_relative_yaw_deg(after_row) if after_row else None
    rc_ignored = rc_ignored_for_usb_supervised(after_row)
    invalid_reason = block_reason or "NONE"
    offending_duration_ms = "NA"
    if invalid_reason == "USB_DRIVE_LIVE_DURATION_EXCEEDS_MAX":
        invalid_reason = "HOST_SENT_DURATION_OVER_MAX"
        offending_duration_ms = int(pulse_ms)
    pose = pose or {}
    current_x = float(pose.get("x", 0.0))
    current_y = float(pose.get("y", 0.0))
    segment_start_x = float(segment.get("start_x_m", 0.0))
    segment_start_y = float(segment.get("start_y_m", 0.0))
    segment_end_x = float(segment.get("end_x_m", 0.0))
    segment_end_y = float(segment.get("end_y_m", 0.0))
    target_x = segment_end_x
    target_y = segment_end_y
    return {
        "row_type": "pulse",
        "segment_index": segment["segment_index"],
        "primitive_index": primitive_index,
        "chunk_index": chunk_index if chunk_index is not None else primitive_index,
        "path_control_mode": path_control_mode,
        "segment_type": segment["segment_type"],
        "start_lat": f"{start_lat:.7f}",
        "start_lon": f"{start_lon:.7f}",
        "goal_lat": f"{goal_lat:.7f}",
        "goal_lon": f"{goal_lon:.7f}",
        "current_lat": telemetry._fmt(gps["lat"], 7) if gps["lat"] is not None else "NA",
        "current_lon": telemetry._fmt(gps["lon"], 7) if gps["lon"] is not None else "NA",
        "gps_block_reason": (after_row or {}).get("gps_block_reason", "NA"),
        "firmware_profile": (after_row or {}).get("firmware_profile", "NA"),
        "gps_valid": bool(pose.get("gps_valid", False)),
        "gps_degraded": gps["gps_degraded"],
        "gps_reanchored": bool(pose.get("gps_reanchored", False)),
        "gps_cached_used": gps["gps_cached_used"],
        "imu_relative_yaw_deg": telemetry._fmt(yaw),
        "imu_yaw_deg": telemetry._fmt(yaw),
        "current_x_m": telemetry._fmt(current_x),
        "current_y_m": telemetry._fmt(current_y),
        "target_x_m": telemetry._fmt(target_x),
        "target_y_m": telemetry._fmt(target_y),
        "segment_start_x_m": telemetry._fmt(segment_start_x),
        "segment_start_y_m": telemetry._fmt(segment_start_y),
        "segment_end_x_m": telemetry._fmt(segment_end_x),
        "segment_end_y_m": telemetry._fmt(segment_end_y),
        "current_heading_deg": telemetry._fmt(correction["current_heading_deg"]),
        "target_heading_deg": telemetry._fmt(target_heading_deg),
        "heading_error_deg": telemetry._fmt(correction["heading_error_deg"]),
        "cross_track_error_m": telemetry._fmt(correction["cross_track_error_m"]),
        "along_track_progress_m": telemetry._fmt(correction["along_track_progress_m"]),
        "remaining_distance_m": telemetry._fmt(correction["remaining_distance_m"]),
        "a_cmd": f"{a_cmd:.3f}",
        "b_cmd": f"{correction['b_cmd']:.3f}",
        "base_a_cmd": f"{(base_a_cmd if base_a_cmd is not None else a_cmd):.3f}",
        "b_trim": f"{b_trim:.3f}",
        "b_heading_component": f"{correction['b_heading_component']:.3f}",
        "b_cte_component": f"{correction['b_cte_component']:.3f}",
        "b_heading_correction": f"{correction['b_heading_component']:.3f}",
        "b_cross_track_correction": f"{correction['b_cte_component']:.3f}",
        "final_a_cmd": f"{a_cmd:.3f}",
        "final_b_cmd": f"{correction['b_cmd']:.3f}",
        "pulse_ms": int(pulse_ms),
        "live_chunk_ms": live_chunk_ms if live_chunk_ms is not None else "NA",
        "max_ms": max_ms if max_ms is not None else "NA",
        "offending_duration_ms": offending_duration_ms,
        "ack_seen": any(telemetry.event(r) == "ACK" for r in pulse_rows),
        "active_seen": any(telemetry.event(r) == "ACTIVE" for r in pulse_rows),
        "stop_seen": any(telemetry.event(r) in safety.STOP_EVENTS for r in pulse_rows),
        "final_zero": _final_zero(pulse_rows),
        "manual_override_detected": False if rc_ignored else manual_override_detected(after_row),
        "rc_ignored_for_usb_supervised": rc_ignored,
        "rc_warning": rc_warning_for_usb_supervised(after_row),
        "calibration_source": calibration_source,
        "connector_mode": connector_mode,
        "drive_mode": drive_mode,
        "correction_source": str(correction.get("correction_source", "unknown")),
        "valid_pulse": block_reason is None,
        "invalid_reason": invalid_reason,
        "ready_for_full_path_following": False,
    }


def build_controller_summary(
    rows: Sequence[dict[str, object]],
    *,
    start_lat: float,
    start_lon: float,
    goal_lat: float,
    goal_lon: float,
    goal_distance_m: float,
    fallback_to_repeated_pulses: bool,
    abort_reason: str = "NONE",
) -> dict[str, object]:
    """Build the guarded controller run summary (routed through the readiness check)."""
    pulse_rows = [r for r in rows if r.get("row_type") == "pulse"]
    valid = sum(1 for r in pulse_rows if r.get("valid_pulse") is True)
    continuous_drive_count = sum(1 for r in pulse_rows if r.get("drive_mode") == "continuous")
    rc_warning_count = sum(1 for r in pulse_rows if r.get("rc_warning") not in (None, "", "NONE"))
    cross_track_correction_used_count = sum(
        1
        for r in pulse_rows
        if abs(float(r.get("b_cte_component") or 0.0)) > _ZERO_TOLERANCE
    )
    imu_heading_used_count = sum(
        1
        for r in pulse_rows
        if r.get("imu_relative_yaw_deg") not in (None, "", "NA")
    )
    offending_durations = [
        int(r["offending_duration_ms"])
        for r in pulse_rows
        if str(r.get("offending_duration_ms", "NA")) not in {"", "NA"}
    ]
    max_ms_values = [
        int(r["max_ms"])
        for r in pulse_rows
        if str(r.get("max_ms", "NA")) not in {"", "NA"}
    ]
    heading_errors = [
        abs(float(r["heading_error_deg"]))
        for r in pulse_rows
        if str(r.get("heading_error_deg", "NA")) not in {"", "NA"}
    ]
    cross_track_errors = [
        abs(float(r["cross_track_error_m"]))
        for r in pulse_rows
        if str(r.get("cross_track_error_m", "NA")) not in {"", "NA"}
    ]
    completed_segment_count = len(
        {
            r.get("segment_index")
            for r in pulse_rows
            if r.get("valid_pulse") is True
        }
    )
    path_control_modes = [
        str(r.get("path_control_mode"))
        for r in pulse_rows
        if r.get("path_control_mode") not in (None, "", "NA")
    ]
    path_control_mode = path_control_modes[-1] if path_control_modes else DEFAULT_PATH_CONTROL_MODE
    # "enabled" is the configured intent (closed-loop modes); "applied" is the
    # runtime evidence that a nonzero steering correction actually reached B.
    correction_enabled = path_control_mode in {"imu_heading", "gps_imu_closed_loop"}
    correction_applied = any(
        abs(float(r.get("b_heading_component") or 0.0)) > _ZERO_TOLERANCE
        or abs(float(r.get("b_cte_component") or 0.0)) > _ZERO_TOLERANCE
        for r in pulse_rows
    )
    final_remaining = [
        float(r["remaining_distance_m"])
        for r in pulse_rows
        if str(r.get("remaining_distance_m", "NA")) not in {"", "NA"}
    ]
    summary = {
        "controller_mode": "continuous_motion",
        "path_control_mode": path_control_mode,
        "closed_loop_correction_enabled": correction_enabled,
        "closed_loop_correction_applied": correction_applied,
        "closed_loop_correction_disabled_reason": (
            "OPEN_LOOP_CHUNKS"
            if path_control_mode == "open_loop_chunks"
            else ("NO_NONZERO_HEADING_OR_CROSS_TRACK_ERROR" if not correction_applied else "NONE")
        ),
        "start_lat": start_lat,
        "start_lon": start_lon,
        "goal_lat": goal_lat,
        "goal_lon": goal_lon,
        "goal_distance_m": goal_distance_m,
        "pulse_count": len(pulse_rows),
        "valid_pulse_count": valid,
        "invalid_pulse_count": len(pulse_rows) - valid,
        "segment_count": len({r.get("segment_index") for r in pulse_rows}),
        "chunk_count": len(pulse_rows),
        "valid_chunk_count": valid,
        "completed_segment_count": completed_segment_count,
        "completed_chunk_count": valid,
        "gps_chunk_count": sum(1 for r in pulse_rows if r.get("gps_valid") is True),
        "gps_degraded_count": sum(1 for r in pulse_rows if r.get("gps_degraded") is True),
        "gps_reanchor_count": sum(1 for r in pulse_rows if r.get("gps_reanchored") is True),
        "continuous_drive_used": continuous_drive_count > 0,
        "continuous_drive_count": continuous_drive_count,
        "imu_heading_used_count": imu_heading_used_count,
        "cross_track_correction_used_count": cross_track_correction_used_count,
        "average_abs_heading_error_deg": (
            sum(heading_errors) / len(heading_errors) if heading_errors else "NA"
        ),
        "max_abs_heading_error_deg": max(heading_errors) if heading_errors else "NA",
        "average_abs_cross_track_error_m": (
            sum(cross_track_errors) / len(cross_track_errors) if cross_track_errors else "NA"
        ),
        "max_abs_cross_track_error_m": max(cross_track_errors) if cross_track_errors else "NA",
        "final_distance_to_goal_m": final_remaining[-1] if final_remaining else "NA",
        "rc_ignored_for_usb_supervised": any(r.get("rc_ignored_for_usb_supervised") is True for r in pulse_rows),
        "rc_warning_count": rc_warning_count,
        "offending_duration_ms": offending_durations[-1] if offending_durations else "NA",
        "max_ms": max_ms_values[-1] if max_ms_values else "NA",
        "fallback_to_repeated_pulses": bool(fallback_to_repeated_pulses),
        "abort_reason": abort_reason,
        "aborted": abort_reason != "NONE",
        "ready_for_full_path_following": False,
    }
    return checks.assert_not_ready_for_full_path_following(summary)


# --- The continuous-motion loop -----------------------------------------------


def correction_token(row: dict[str, object]) -> str:
    """One-word correction summary for the per-chunk console line.

    ``dead_reckon`` when GPS is degraded (pose came from IMU + calibrated
    progress, not GPS); otherwise ``both`` / ``cross_track`` / ``heading`` for
    whichever steering components are nonzero, or ``none`` when B was untouched
    (open-loop chunks, or a chunk with zero error).
    """
    if row.get("gps_degraded") is True:
        return "dead_reckon"
    heading = abs(telemetry._optional_float(row.get("b_heading_correction")) or 0.0) > _ZERO_TOLERANCE
    cross_track = abs(telemetry._optional_float(row.get("b_cross_track_correction")) or 0.0) > _ZERO_TOLERANCE
    if heading and cross_track:
        return "both"
    if cross_track:
        return "cross_track"
    if heading:
        return "heading"
    return "none"


def chunk_status_line(row: dict[str, object], *, a_cmd: float, b_cmd: float) -> str:
    """Render the required one-line per-chunk status string for a built row."""
    imu_ok = row.get("imu_relative_yaw_deg") not in (None, "", "NA")
    return (
        f"seg={row['segment_index']} chunk={row['chunk_index']} "
        f"mode={row['path_control_mode']} "
        f"gps={'DEGRADED' if row.get('gps_degraded') else 'OK'} "
        f"imu={'OK' if imu_ok else 'NA'} "
        f"heading_err={row['heading_error_deg']} cte={row['cross_track_error_m']} "
        f"progress={row['along_track_progress_m']} remaining={row['remaining_distance_m']} "
        f"A={a_cmd:.3f} B={b_cmd:.3f} correction={correction_token(row)}"
    )


def _segment_pulse_budget(
    segment: dict[str, object],
    resolved_calibration: dict[str, object],
    *,
    left_fixed_pulses: int,
    right_fixed_pulses: int,
) -> tuple[int, bool, str]:
    """Return (pulse_budget, is_connector, direction) for a planned segment."""
    if str(segment["segment_type"]) == "connector_turn":
        direction = "left" if str(segment["expected_motion_direction"]) == "turn_left" else "right"
        if str(resolved_calibration.get("connector_mode_effective")) == "repeated_pulses":
            budget = int(left_fixed_pulses if direction == "left" else right_fixed_pulses)
        else:
            budget = 1
        return budget, True, direction
    return int(segment.get("pulse_budget", 1)), False, "forward"


def run_controller(
    handle: object,
    *,
    segments: Sequence[dict[str, object]],
    resolved_calibration: dict[str, object],
    start_lat: float,
    start_lon: float,
    start_yaw_deg: float | None,
    goal_lat: float,
    goal_lon: float,
    event_timeout_s: float = DEFAULT_EVENT_TIMEOUT_S,
    heartbeat_timeout_s: float = DEFAULT_HEARTBEAT_TIMEOUT_S,
    rc_neutral_wait_s: float = DEFAULT_RC_NEUTRAL_WAIT_S,
    gps_degradation_policy: str = DEFAULT_GPS_DEGRADATION_POLICY,
    manual_override_mode: str = DEFAULT_MANUAL_OVERRIDE_MODE,
    left_fixed_pulses: int = 12,
    right_fixed_pulses: int = 12,
    straight_motion_mode: str = "pulse",
    live_update_hz: float = 8.0,
    live_ttl_ms: int = 350,
    live_chunk_ms: int = 700,
    max_segment_chunks: int = 20,
    live_max_ms: int = 1000,
    imu_heading_hold: bool = True,
    cross_track_correction: bool = True,
    path_control_mode: str = DEFAULT_PATH_CONTROL_MODE,
    k_heading: float = 0.006,
    k_cross_track: float = 0.20,
    max_correction_b: float = 0.08,
    gps_reanchor: bool = True,
    require_auto_switch: bool = False,
) -> tuple[list[dict[str, object]], list[str], str]:
    """Run the supervised pulse loop over ``segments``; return (rows, raw_lines, abort_reason).

    Never raises on the expected field faults (serial disconnect, RC invalid,
    GPS abort policy, missing heartbeat): each sets ``abort_reason`` and stops the
    loop cleanly so the caller can still write a guarded summary.

    ``require_auto_switch`` makes the loop abort with ``USER_SWITCHED_TO_MANUAL``
    if a heartbeat reports the physical mode switch back in MANUAL -- used by the
    AUTO-switch-triggered ``auto-relative-run`` so flipping the switch stops the
    rover safely (the final STOP/zero handshake still runs).
    """
    rows: list[dict[str, object]] = []
    raw_lines: list[str] = []
    gps_cache: dict[str, object] = {"lat": start_lat, "lon": start_lon, "degraded": False}
    abort_reason = "NONE"
    primitive_index = 0
    effective_live_max_ms = max(1, int(live_max_ms))
    effective_live_chunk_ms = max(1, min(int(live_chunk_ms), effective_live_max_ms))
    effective_max_segment_chunks = max(1, int(max_segment_chunks))
    if path_control_mode not in PATH_CONTROL_MODES:
        path_control_mode = DEFAULT_PATH_CONTROL_MODE
    pose_state: dict[str, object] = {"x": 0.0, "y": 0.0, "source": "plan_start", "gps_degraded": False}
    # Heading-hold reference: honor an explicit --start-yaw-deg for the first lane,
    # then re-capture per lane from each lane's first heartbeat yaw so heading
    # correction stays live without the operator supplying a start yaw.
    provided_start_yaw = start_yaw_deg
    first_lane_pending = True
    if path_control_mode == "open_loop_chunks":
        imu_heading_hold = False
        cross_track_correction = False
    elif path_control_mode == "imu_heading":
        cross_track_correction = False

    for segment in segments:
        segment_ref_yaw: float | None = None
        budget, is_connector, direction = _segment_pulse_budget(
            segment,
            resolved_calibration,
            left_fixed_pulses=left_fixed_pulses,
            right_fixed_pulses=right_fixed_pulses,
        )
        target_heading = float(segment["target_heading_deg"])
        if is_connector:
            connector = connector_command(resolved_calibration, direction)
            a_cmd = float(connector["a_cmd"])
            pulse_ms = int(connector["pulse_ms"])
            calibration_source = str(connector["calibration_source"])
            connector_mode = str(connector["connector_mode"])
            connector_b = float(connector["b_cmd"])
            base_b = 0.0
        else:
            motion = geometry._motion_calibrated(
                resolved_calibration, str(segment["expected_motion_direction"])
            )
            a_cmd = float(motion["a_cmd"])
            pulse_ms = int(motion["pulse_ms"])
            calibration_source = str(motion.get("calibration_source", "unknown"))
            connector_mode = "lane"
            connector_b = 0.0
            base_b = float(motion.get("b_cmd", 0.0))

        if not is_connector and straight_motion_mode == "continuous":
            total_duration_ms = max(1, int(pulse_ms) * max(1, budget))
            bounded_chunk_ms = effective_live_chunk_ms
            chunk_count = max(1, math.ceil(total_duration_ms / bounded_chunk_ms))
            chunk_count = min(chunk_count, effective_max_segment_chunks)
            remaining_ms = total_duration_ms
            chunk_progress_m = float(segment.get("length_m", 0.0)) / max(1, chunk_count)
            for chunk_index in range(1, chunk_count + 1):
                primitive_index += 1
                chunk_ms = min(bounded_chunk_ms, remaining_ms)
                if chunk_index == chunk_count:
                    chunk_ms = min(bounded_chunk_ms, max(1, remaining_ms))
                remaining_ms = max(0, remaining_ms - chunk_ms)
                try:
                    heartbeat = wait_for_guarded_pulse_heartbeat(handle, raw_lines, heartbeat_timeout_s)
                    if heartbeat is None:
                        abort_reason = "NO_GUARDED_PULSE_HEARTBEAT"
                        break
                    usb_supervised_rc_ignored = rc_ignored_for_usb_supervised(heartbeat)
                    if telemetry._parse_bool(heartbeat.get("rc_ok")) is not True and not usb_supervised_rc_ignored:
                        abort_reason = "RC_NOT_OK"
                        break
                    if (
                        not usb_supervised_rc_ignored
                        and manual_override_detected(heartbeat)
                        and manual_override_mode == "abort"
                    ):
                        abort_reason = "MANUAL_OVERRIDE"
                        break
                    if require_auto_switch and mode_switch_state(heartbeat) == "MANUAL":
                        abort_reason = MANUAL_SWITCH_ABORT_REASON
                        break
                    if segment_ref_yaw is None:
                        segment_ref_yaw = reference_yaw_for_segment(
                            heartbeat,
                            provided_start_yaw=provided_start_yaw,
                            use_provided=first_lane_pending,
                        )
                        first_lane_pending = False
                    gps = dead_reckon_gps(heartbeat, gps_cache)
                    gps_action = geometry.gps_policy_action(bool(gps["gps_degraded"]), gps_degradation_policy)
                    if gps_action == "abort":
                        abort_reason = "GPS_DEGRADED"
                        break
                    if gps_action == "pause":
                        continue
                    pose_before = _pose_from_gps_or_dead_reckon(
                        row=heartbeat,
                        gps=gps,
                        pose_state=pose_state,
                        segment=segment,
                        start_lat=start_lat,
                        start_lon=start_lon,
                        gps_reanchor=gps_reanchor,
                    )
                    latest_correction: dict[str, float] | None = None
                    latest_pose: dict[str, object] | None = pose_before
                    chunk_gps_reanchored = bool(pose_before.get("gps_reanchored"))

                    def command_from_row(row: dict[str, str] | None) -> tuple[float, float]:
                        nonlocal latest_correction, latest_pose, chunk_gps_reanchored
                        source = row or heartbeat
                        local_gps = dead_reckon_gps(source, gps_cache)
                        latest_pose = _pose_from_gps_or_dead_reckon(
                            row=source,
                            gps=local_gps,
                            pose_state=pose_state,
                            segment=segment,
                            start_lat=start_lat,
                            start_lon=start_lon,
                            gps_reanchor=gps_reanchor,
                        )
                        chunk_gps_reanchored = chunk_gps_reanchored or bool(latest_pose.get("gps_reanchored"))
                        yaw = telemetry.imu_relative_yaw_deg(source)
                        latest_correction = pulse_correction(
                            segment=segment,
                            x=float(latest_pose["x"]),
                            y=float(latest_pose["y"]),
                            target_heading_deg=target_heading,
                            yaw=yaw,
                            start_yaw_deg=segment_ref_yaw,
                            is_connector=False,
                            base_b_cmd=base_b,
                            imu_heading_hold=imu_heading_hold,
                            cross_track_correction=cross_track_correction,
                            path_control_mode=path_control_mode,
                            k_heading=k_heading,
                            k_cross_track=k_cross_track,
                            max_correction_b=max_correction_b,
                        )
                        return a_cmd, float(latest_correction["b_cmd"])

                    duration_s = max(0.001, chunk_ms / 1000.0)
                    live_rows = executor.send_live_drive(
                        handle,
                        seq=primitive_index,
                        duration_s=duration_s,
                        update_hz=live_update_hz,
                        ttl_ms=live_ttl_ms,
                        command_fn=command_from_row,
                        raw_lines=raw_lines,
                        event_timeout_s=event_timeout_s,
                    )
                    after = wait_for_guarded_pulse_heartbeat(handle, raw_lines, heartbeat_timeout_s) or heartbeat
                except OSError:
                    abort_reason = "SERIAL_DISCONNECT"
                    break

                gps_after = dead_reckon_gps(after, gps_cache)
                pose_after = _pose_from_gps_or_dead_reckon(
                    row=after,
                    gps=gps_after,
                    pose_state=pose_state,
                    segment=segment,
                    start_lat=start_lat,
                    start_lon=start_lon,
                    advance_m=0.0 if not gps_after["gps_degraded"] else chunk_progress_m,
                    gps_reanchor=gps_reanchor,
                )
                chunk_gps_reanchored = chunk_gps_reanchored or bool(pose_after.get("gps_reanchored"))
                pose_after["gps_reanchored"] = chunk_gps_reanchored
                if latest_correction is None:
                    latest_correction = pulse_correction(
                        segment=segment,
                        x=float(pose_after["x"]),
                        y=float(pose_after["y"]),
                        target_heading_deg=target_heading,
                        yaw=telemetry.imu_relative_yaw_deg(after),
                        start_yaw_deg=segment_ref_yaw,
                        is_connector=False,
                        base_b_cmd=base_b,
                        imu_heading_hold=imu_heading_hold,
                        cross_track_correction=cross_track_correction,
                        path_control_mode=path_control_mode,
                        k_heading=k_heading,
                        k_cross_track=k_cross_track,
                        max_correction_b=max_correction_b,
                    )
                    latest_pose = pose_after
                else:
                    # The command callback computed the command from the latest active
                    # telemetry; rows should report the post-chunk pose when available.
                    latest_pose = pose_after
                row = build_execution_row(
                    segment=segment,
                    primitive_index=primitive_index,
                    after_row=after,
                    pulse_rows=live_rows,
                    start_lat=start_lat,
                    start_lon=start_lon,
                    goal_lat=goal_lat,
                    goal_lon=goal_lon,
                    start_yaw_deg=start_yaw_deg,
                    target_heading_deg=target_heading,
                    a_cmd=a_cmd,
                    correction=latest_correction,
                    pulse_ms=chunk_ms,
                    gps=gps_after,
                    calibration_source=calibration_source,
                    connector_mode=connector_mode,
                    block_reason_override=live_drive_block_reason(live_rows),
                    drive_mode="continuous",
                    chunk_index=chunk_index,
                    live_chunk_ms=bounded_chunk_ms,
                    max_ms=effective_live_max_ms,
                    path_control_mode=path_control_mode,
                    pose=latest_pose,
                    base_a_cmd=a_cmd,
                    b_trim=base_b,
                )
                rows.append(row)
                print(chunk_status_line(row, a_cmd=a_cmd, b_cmd=float(latest_correction["b_cmd"])))
                if row["valid_pulse"] is not True:
                    abort_reason = str(row["invalid_reason"])
                    break
                remaining = telemetry._optional_float(row.get("remaining_distance_m")) or 0.0
                if remaining <= max(0.05, chunk_progress_m * 0.5):
                    break
            if abort_reason != "NONE":
                break
            continue

        pulse_chunk_ms_values: list[int] = []
        for _ in range(budget):
            remaining_pulse_ms = int(pulse_ms)
            while remaining_pulse_ms > 0:
                chunk_ms = min(effective_live_max_ms, remaining_pulse_ms)
                pulse_chunk_ms_values.append(chunk_ms)
                remaining_pulse_ms -= chunk_ms
        for chunk_index, pulse_chunk_ms in enumerate(pulse_chunk_ms_values, 1):
            primitive_index += 1
            try:
                heartbeat = wait_for_guarded_pulse_heartbeat(handle, raw_lines, heartbeat_timeout_s)
                if heartbeat is None:
                    abort_reason = "NO_GUARDED_PULSE_HEARTBEAT"
                    break
                usb_supervised_rc_ignored = rc_ignored_for_usb_supervised(heartbeat)
                if telemetry._parse_bool(heartbeat.get("rc_ok")) is not True and not usb_supervised_rc_ignored:
                    abort_reason = "RC_NOT_OK"
                    break
                if (
                    not usb_supervised_rc_ignored
                    and manual_override_detected(heartbeat)
                    and manual_override_mode == "abort"
                ):
                    abort_reason = "MANUAL_OVERRIDE"
                    break
                if require_auto_switch and mode_switch_state(heartbeat) == "MANUAL":
                    abort_reason = MANUAL_SWITCH_ABORT_REASON
                    break
                if not usb_supervised_rc_ignored and safety.rc_neutral_wait(heartbeat):
                    neutral = wait_for_neutral_rc(handle, raw_lines, rc_neutral_wait_s)
                    if neutral is None:
                        abort_reason = "RC_NOT_NEUTRAL"
                        break
                    heartbeat = neutral

                gps = dead_reckon_gps(heartbeat, gps_cache)
                gps_action = geometry.gps_policy_action(
                    bool(gps["gps_degraded"]), gps_degradation_policy
                )
                if gps_action == "abort":
                    abort_reason = "GPS_DEGRADED"
                    break
                if gps_action == "pause":
                    continue

                lat = float(gps["lat"]) if gps["lat"] is not None else start_lat
                lon = float(gps["lon"]) if gps["lon"] is not None else start_lon
                x, y = geometry.goal_to_local(start_lat, start_lon, lat, lon)
                # Connectors hold against the global start yaw (the turn rotates the
                # body); lanes hold against the per-lane captured reference.
                if is_connector:
                    effective_ref_yaw = provided_start_yaw
                else:
                    if segment_ref_yaw is None:
                        segment_ref_yaw = reference_yaw_for_segment(
                            heartbeat,
                            provided_start_yaw=provided_start_yaw,
                            use_provided=first_lane_pending,
                        )
                        first_lane_pending = False
                    effective_ref_yaw = segment_ref_yaw
                yaw = telemetry.imu_relative_yaw_deg(heartbeat)
                correction = pulse_correction(
                    segment=segment,
                    x=x,
                    y=y,
                    target_heading_deg=target_heading,
                    yaw=yaw,
                    start_yaw_deg=effective_ref_yaw,
                    is_connector=is_connector,
                    base_b_cmd=base_b,
                    connector_b_cmd=connector_b,
                    imu_heading_hold=imu_heading_hold,
                    cross_track_correction=cross_track_correction,
                    path_control_mode=path_control_mode,
                    k_heading=k_heading,
                    k_cross_track=k_cross_track,
                    max_correction_b=max_correction_b,
                )
                planned = planned_pulse(
                    seq=primitive_index,
                    a_cmd=a_cmd,
                    b_cmd=float(correction["b_cmd"]),
                    pulse_ms=pulse_chunk_ms,
                )
                pulse_rows = executor.send_pulse(
                    handle, planned, raw_lines, event_timeout_s=event_timeout_s
                )
                after = (
                    wait_for_guarded_pulse_heartbeat(handle, raw_lines, heartbeat_timeout_s)
                    or heartbeat
                )
            except OSError:
                abort_reason = "SERIAL_DISCONNECT"
                break

            gps_after = dead_reckon_gps(after, gps_cache)
            row = build_execution_row(
                segment=segment,
                primitive_index=primitive_index,
                after_row=after,
                pulse_rows=pulse_rows,
                start_lat=start_lat,
                start_lon=start_lon,
                goal_lat=goal_lat,
                goal_lon=goal_lon,
                start_yaw_deg=start_yaw_deg,
                target_heading_deg=target_heading,
                a_cmd=a_cmd,
                correction=correction,
                pulse_ms=pulse_chunk_ms,
                gps=gps_after,
                calibration_source=calibration_source,
                connector_mode=connector_mode,
                chunk_index=chunk_index,
                max_ms=effective_live_max_ms,
                path_control_mode=path_control_mode,
                pose={
                    "x": x,
                    "y": y,
                    "gps_valid": not bool(gps_after["gps_degraded"]),
                    "gps_reanchored": False,
                },
                base_a_cmd=a_cmd,
                b_trim=base_b,
            )
            rows.append(row)
            print(chunk_status_line(row, a_cmd=a_cmd, b_cmd=float(correction["b_cmd"])))
            if row["valid_pulse"] is not True:
                abort_reason = str(row["invalid_reason"])
                break
            heading_error_after = telemetry._optional_float(row.get("heading_error_deg"))
            if (
                is_connector
                and connector_mode != "repeated_pulses"
                and row.get("imu_relative_yaw_deg") not in (None, "", "NA")
                and heading_error_after is not None
                and abs(heading_error_after) <= 10.0
            ):
                break
            progress = telemetry._optional_float(row.get("along_track_progress_m")) or 0.0
            if str(segment["segment_type"]).endswith("_lane") and progress >= float(
                segment.get("length_m", 0.0)
            ):
                break

        if abort_reason != "NONE":
            break
        if rows and rows[-1]["valid_pulse"] is not True:
            break

    return rows, raw_lines, abort_reason
