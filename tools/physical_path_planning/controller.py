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
import time
from typing import Sequence

from tools.physical_path_planning import checks, executor, geometry, safety, telemetry

DEFAULT_EVENT_TIMEOUT_S = 3.0
DEFAULT_HEARTBEAT_TIMEOUT_S = 3.0
DEFAULT_RC_NEUTRAL_WAIT_S = 5.0
DEFAULT_GPS_DEGRADATION_POLICY = "continue"
DEFAULT_MANUAL_OVERRIDE_MODE = "abort"
_ZERO_TOLERANCE = 1e-9
DEFAULT_PATH_CONTROL_MODE = "gps_imu_closed_loop"
PATH_CONTROL_MODES = {
    "open_loop_chunks",
    "imu_heading",
    "gps_imu_closed_loop",
    "stop_correct_go",
}

# Abort reason raised when the operator flips the physical mode switch back to
# MANUAL during an AUTO-triggered run (auto-relative-run).
MANUAL_SWITCH_ABORT_REASON = "USER_SWITCHED_TO_MANUAL"

# stop_correct_go defaults: a discrete move -> stop -> measure -> correct loop
# that only steers (B axis) and turns in place, never lowering the calibrated A.
DEFAULT_MOVE_CHUNK_MS = 700
DEFAULT_SETTLE_AFTER_MOVE_MS = 300
DEFAULT_TELEMETRY_STABILIZE_MS = 500
DEFAULT_HEADING_CORRECTION_THRESHOLD_DEG = 8.0
DEFAULT_HEADING_CORRECTION_TOLERANCE_DEG = 5.0
DEFAULT_CROSS_TRACK_CORRECTION_THRESHOLD_M = 0.35
DEFAULT_HEADING_CORRECTION_B_LEFT = 0.24
DEFAULT_HEADING_CORRECTION_B_RIGHT = -0.12
DEFAULT_MAX_HEADING_CORRECTION_MS = 1800
DEFAULT_HEADING_CORRECTION_CHUNK_MS = 300
SENSOR_TRUST_MODES = {"imu_gps_first", "calibration_fallback"}
DEFAULT_SENSOR_TRUST_MODE = "imu_gps_first"

# Connector turn semantics: the calibrated turn primitive is a PULSE whose real
# per-pulse rotation is target_angle_deg (often 15-45 degrees on this rover,
# despite the turn_*_90 key name). A 90 degree planned corner therefore takes
# repeated pulses, stopped early by IMU yaw feedback when available.
TURN_ANGLE_POLICIES = {"from_json", "assume_90"}
DEFAULT_TURN_ANGLE_POLICY = "from_json"
DEFAULT_MAX_CONNECTOR_PULSES_PER_TURN = 6
DEFAULT_CONNECTOR_TURN_TOLERANCE_DEG = 10.0
# How lane heading errors are referenced. "mission" chains one yaw reference
# across the whole run, so an under-turned connector shows up (and gets
# corrected) on the next lane. "per_lane" is the legacy behavior: each lane
# re-captures its reference yaw, silently absorbing any connector error.
HEADING_REFERENCES = {"mission", "per_lane"}
DEFAULT_HEADING_REFERENCE = "mission"


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
    target_angle = primitive.get("target_angle_deg")
    return {
        "a_cmd": float(primitive["a_cmd"]),
        "b_cmd": float(primitive["b_cmd"]),
        "pulse_ms": int(primitive["pulse_ms"]),
        "calibration_source": str(primitive.get("calibration_source", "unknown")),
        "connector_mode": effective,
        "fallback_to_repeated_pulses": fallback,
        "target_angle_deg": float(target_angle) if target_angle is not None else None,
    }


def connector_turn_angle_deg(segment: dict[str, object]) -> float:
    """Signed rotation a connector segment requests (+left / -right).

    Plans generated after the turn/step decomposition carry an explicit
    ``turn_angle_deg``; older plans fall back to +/-90 from the expected motion
    direction (the lawnmower corner has always been a quarter turn).
    """
    explicit = telemetry._optional_float(segment.get("turn_angle_deg"))
    if explicit is not None:
        return float(explicit)
    return 90.0 if str(segment.get("expected_motion_direction")) == "turn_left" else -90.0


def per_pulse_turn_angle_deg(
    connector: dict[str, object],
    *,
    turn_angle_policy: str = DEFAULT_TURN_ANGLE_POLICY,
    turn_angle_override: float | None = None,
) -> float | None:
    """Rotation one calibrated turn pulse is expected to produce, or None.

    ``None`` means unknown (repeated-pulse twitch calibration); callers then keep
    the fixed-pulse budget. ``assume_90`` reproduces the legacy one-pulse-per-90
    assumption; ``from_json`` (default) trusts the calibration's target_angle_deg.
    """
    if turn_angle_override is not None and float(turn_angle_override) > 0.0:
        return float(turn_angle_override)
    if turn_angle_policy == "assume_90":
        return 90.0
    target = connector.get("target_angle_deg")
    if target is None:
        return None
    angle = abs(float(target))
    return angle if angle > 1e-6 else None


def connector_planned_pulses(requested_angle_deg: float, per_pulse_angle_deg: float | None) -> int:
    """Open-loop pulse count to cover the requested rotation (>= 1)."""
    if per_pulse_angle_deg is None or per_pulse_angle_deg <= 0.0:
        return 1
    return max(1, math.ceil(abs(requested_angle_deg) / per_pulse_angle_deg - 1e-9))


def connector_pulse_budget(
    requested_angle_deg: float,
    per_pulse_angle_deg: float | None,
    *,
    max_pulses: int = DEFAULT_MAX_CONNECTOR_PULSES_PER_TURN,
    imu_available: bool,
) -> int:
    """Pulse cap for one connector turn (anti-rotation-loop guard).

    With IMU feedback the loop may use a couple of extra pulses beyond the
    planned count because each one is verified against measured yaw; without
    IMU it must stop exactly at the open-loop count rather than guess.
    """
    planned = connector_planned_pulses(requested_angle_deg, per_pulse_angle_deg)
    budget = planned + 2 if imu_available else planned
    return max(1, min(int(max_pulses), budget))


def manual_switch_seen(rows: Sequence[dict[str, str]]) -> bool:
    """True when any telemetry row reports the physical mode switch in MANUAL.

    Rows without mode-channel fields (pulse ACK/STOP events) report ABSENT and
    never count, so this only triggers on explicit MANUAL evidence.
    """
    return any(mode_switch_state(row) == "MANUAL" for row in rows if isinstance(row, dict))


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
    max_connector_pulses: int = DEFAULT_MAX_CONNECTOR_PULSES_PER_TURN,
    turn_angle_policy: str = DEFAULT_TURN_ANGLE_POLICY,
    turn_angle_override: float | None = None,
    imu_available: bool = False,
) -> tuple[int, bool, str]:
    """Return (pulse_budget, is_connector, direction) for a planned segment.

    Angle-calibrated connectors budget ``ceil(|requested| / per-pulse angle)``
    pulses from the calibration's target_angle_deg instead of assuming one pulse
    completes the corner; repeated-pulse twitch calibration keeps the fixed count.
    """
    if str(segment["segment_type"]) in {"connector_turn", "path_connector"}:
        direction = "left" if str(segment["expected_motion_direction"]) == "turn_left" else "right"
        if str(resolved_calibration.get("connector_mode_effective")) == "repeated_pulses":
            budget = int(left_fixed_pulses if direction == "left" else right_fixed_pulses)
        else:
            connector = connector_command(resolved_calibration, direction)
            per_pulse = per_pulse_turn_angle_deg(
                connector,
                turn_angle_policy=turn_angle_policy,
                turn_angle_override=turn_angle_override,
            )
            budget = connector_pulse_budget(
                connector_turn_angle_deg(segment),
                per_pulse,
                max_pulses=max_connector_pulses,
                imu_available=imu_available,
            )
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
    max_connector_pulses_per_turn: int = DEFAULT_MAX_CONNECTOR_PULSES_PER_TURN,
    turn_angle_policy: str = DEFAULT_TURN_ANGLE_POLICY,
    turn_angle_override: float | None = None,
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
        connector_yaw_ref: float | None = None
        budget, is_connector, direction = _segment_pulse_budget(
            segment,
            resolved_calibration,
            left_fixed_pulses=left_fixed_pulses,
            right_fixed_pulses=right_fixed_pulses,
            max_connector_pulses=max_connector_pulses_per_turn,
            turn_angle_policy=turn_angle_policy,
            turn_angle_override=turn_angle_override,
        )
        connector_requested_angle = connector_turn_angle_deg(segment) if is_connector else 0.0
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
                if require_auto_switch and manual_switch_seen(live_rows):
                    abort_reason = MANUAL_SWITCH_ABORT_REASON
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
                    if connector_yaw_ref is None:
                        connector_yaw_ref = telemetry.imu_relative_yaw_deg(heartbeat)
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
            if require_auto_switch and manual_switch_seen(pulse_rows):
                abort_reason = MANUAL_SWITCH_ABORT_REASON
                break
            # Connector early-stop: measure the rotation actually applied since
            # the connector started (local yaw reference), not against a global
            # start yaw that is usually absent. Without IMU evidence the loop
            # must run its full angle-derived budget instead of assuming done.
            if is_connector and connector_mode != "repeated_pulses":
                yaw_after_turn = telemetry.imu_relative_yaw_deg(after)
                if connector_yaw_ref is not None and yaw_after_turn is not None:
                    applied_turn = geometry.wrap_deg(yaw_after_turn - connector_yaw_ref)
                    remaining_turn = geometry.wrap_deg(connector_requested_angle - applied_turn)
                    if abs(remaining_turn) <= DEFAULT_CONNECTOR_TURN_TOLERANCE_DEG:
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


# --- stop_correct_go: discrete move -> stop -> measure -> correct loop ---------
#
# A deliberately conservative path-following mode for short field tests. Unlike
# the continuous gps_imu_closed_loop drive, each straight chunk fully stops and
# confirms zero before the rover reads its pose, so heading is corrected by a
# discrete in-place IMU turn (not a continuous B nudge) and cross-track error is
# trimmed onto the *next* chunk's B. It reuses the same primitives as the rest of
# the controller: executor.send_pulse for the guarded move, projection_metrics for
# the geometry, dead_reckon_gps / _pose_from_gps_or_dead_reckon for pose, and
# build_execution_row so the standard summary/CSV path works unchanged.


def stop_correct_go_sensor_source(
    *,
    gps_valid: bool,
    imu_valid: bool,
    trust_mode: str = DEFAULT_SENSOR_TRUST_MODE,
    allow_calibration_fallback: bool = True,
) -> dict[str, object]:
    """Decide which sensor drives this cycle's pose/heading estimate.

    GPS and/or IMU are used whenever available. When *both* are unavailable the
    cycle may only continue on calibrated dead-reckoning if it is allowed --
    either explicitly (``allow_calibration_fallback``) or implicitly by the
    ``calibration_fallback`` trust mode. Otherwise the run aborts
    ``SENSOR_UNAVAILABLE`` rather than driving blind.
    """
    if trust_mode not in SENSOR_TRUST_MODES:
        trust_mode = DEFAULT_SENSOR_TRUST_MODE
    if gps_valid or imu_valid:
        source = "gps_imu" if (gps_valid and imu_valid) else ("gps" if gps_valid else "imu")
        return {"ok": True, "source": source, "fallback_used": False, "reason": "NONE"}
    fallback_allowed = bool(allow_calibration_fallback) or trust_mode == "calibration_fallback"
    if fallback_allowed:
        return {
            "ok": True,
            "source": "calibration",
            "fallback_used": True,
            "reason": "CALIBRATION_DEAD_RECKON",
        }
    return {"ok": False, "source": "none", "fallback_used": False, "reason": "SENSOR_UNAVAILABLE"}


def stop_correct_go_heading_decision(
    *,
    heading_error_deg: float,
    threshold_deg: float,
    imu_valid: bool,
    b_left: float,
    b_right: float,
) -> dict[str, object]:
    """Whether an in-place heading correction is needed, and its turn command.

    The correction is an IMU-feedback turn, so it is only attempted when IMU yaw
    is available; without IMU the rover keeps its heading on the next straight
    chunk instead of guessing a blind rotation.
    """
    needs = bool(imu_valid) and abs(heading_error_deg) > abs(threshold_deg)
    if not needs:
        return {"needs_correction": False, "turn_direction": "none", "correction_b_cmd": 0.0}
    direction = "left" if heading_error_deg > 0.0 else "right"
    return {
        "needs_correction": True,
        "turn_direction": direction,
        "correction_b_cmd": float(b_left) if direction == "left" else float(b_right),
    }


def stop_correct_go_cross_track_trim(
    *,
    cross_track_error_m: float,
    threshold_m: float,
    k_cross_track: float = 0.20,
    max_correction_b: float = 0.08,
) -> float:
    """B trim applied to the *next* chunk when cross-track error exceeds threshold.

    Uses the same sign/gain convention as the continuous closed-loop B correction
    (positive signed cross-track -> positive B -> steer back onto the line),
    clamped to +/-``max_correction_b``. Below threshold the trim is zero.
    """
    if abs(cross_track_error_m) <= abs(threshold_m):
        return 0.0
    return geometry.clamp(
        float(k_cross_track) * float(cross_track_error_m),
        -abs(max_correction_b),
        abs(max_correction_b),
    )


def remaining_turn_error_deg(initial_heading_error_deg: float, yaw_delta_deg: float) -> float:
    """Signed heading error left after rotating ``|yaw_delta|`` toward the target.

    The turn direction comes from the error sign and the IMU yaw magnitude
    measures how far we actually rotated, so the applied rotation is
    ``copysign(|yaw_delta|, initial_error)`` (mirrors the alignment turn math).
    """
    applied = math.copysign(abs(yaw_delta_deg), initial_heading_error_deg)
    return geometry.wrap_deg(initial_heading_error_deg - applied)


def _collect_stabilized_heartbeat(
    handle: object,
    raw_lines: list[str],
    *,
    settle_after_move_ms: int,
    telemetry_stabilize_ms: int,
    heartbeat_timeout_s: float,
    verbose_raw: bool = True,
) -> dict[str, str] | None:
    """Let the motion settle, then read heartbeats over the stabilize window.

    Returns the *last* guarded-pulse heartbeat seen inside the window so pose and
    heading are read from a stationary, settled telemetry sample rather than the
    transient first frame after the move stops.
    """
    if settle_after_move_ms > 0:
        time.sleep(settle_after_move_ms / 1000.0)
    last = wait_for_guarded_pulse_heartbeat(handle, raw_lines, heartbeat_timeout_s)
    deadline = time.monotonic() + max(0.0, telemetry_stabilize_ms / 1000.0)
    while time.monotonic() < deadline:
        budget = max(0.02, min(float(heartbeat_timeout_s), deadline - time.monotonic()))
        nxt = wait_for_guarded_pulse_heartbeat(handle, raw_lines, budget)
        if nxt is None:
            break
        last = nxt
    return last


def _run_stop_correct_go_heading_turn(
    handle: object,
    *,
    correction_b_cmd: float,
    yaw_turn_start: float,
    initial_heading_error_deg: float,
    heading_tolerance_deg: float,
    max_correction_ms: int,
    update_hz: float,
    ttl_ms: int,
    chunk_ms: int,
    event_timeout_s: float,
    raw_lines: list[str],
    verbose_raw: bool = True,
    abort_on_manual_switch: bool = False,
) -> dict[str, object]:
    """Rotate in place with IMU yaw feedback until the heading error is corrected.

    Issues bounded ``USB_DRIVE_LIVE_SET`` turn setpoints (A=0, B=correction) and a
    final ``USB_DRIVE_LIVE_STOP`` (same bounded-deadman pattern as the alignment
    turn and :func:`executor.send_live_drive`), breaking early on the IMU stop
    condition rather than a fixed duration. ``abort_on_manual_switch`` stops the
    turn the moment a telemetry row reports the physical switch back in MANUAL.
    """
    seq = 1
    duration_ms = max(1, int(chunk_ms))
    update_period_s = 1.0 / max(1.0, float(update_hz))
    start_time = time.monotonic()
    deadline = start_time + max(0.0, max_correction_ms / 1000.0)
    yaw_latest = yaw_turn_start
    remaining = initial_heading_error_deg
    timed_out = False
    rejected = False
    manual_switch = False
    while True:
        if time.monotonic() >= deadline:
            timed_out = True
            break
        executor.write_command(
            handle,
            (
                f"USB_DRIVE_LIVE_SET seq={seq} a=0.000 b={float(correction_b_cmd):.3f} "
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
            if telemetry.event(row) == "REJECT":
                rejected = True
                break
            if abort_on_manual_switch and mode_switch_state(row) == "MANUAL":
                manual_switch = True
                break
            yaw = telemetry.imu_relative_yaw_deg(row)
            if yaw is not None:
                yaw_latest = yaw
                yaw_delta = geometry.wrap_deg(yaw - yaw_turn_start)
                remaining = remaining_turn_error_deg(initial_heading_error_deg, yaw_delta)
                if abs(remaining) <= abs(heading_tolerance_deg):
                    break
        else:
            time.sleep(min(update_period_s, 0.05))
    executor.write_command(handle, f"USB_DRIVE_LIVE_STOP seq={seq}")
    executor.wait_for_event(
        handle, raw_lines, executor.STOP_CONFIRM_EVENTS, event_timeout_s, verbose_raw=verbose_raw
    )
    return {
        "yaw_turn_start": yaw_turn_start,
        "yaw_final": yaw_latest,
        "initial_heading_error_deg": initial_heading_error_deg,
        "final_heading_error_deg": remaining,
        "turn_duration_ms": int((time.monotonic() - start_time) * 1000.0),
        "correction_success": abs(remaining) <= abs(heading_tolerance_deg)
        and not rejected
        and not manual_switch,
        "timed_out": timed_out,
        "rejected": rejected,
        "manual_switch": manual_switch,
    }


def stop_correct_go_status_line(row: dict[str, object]) -> str:
    """One-line per-cycle console status for a built stop_correct_go row."""
    imu_ok = bool(row.get("imu_valid"))
    return (
        f"seg={row['segment_index']} chunk={row['chunk_index']} "
        f"phase={row.get('phase', 'move')} mode=stop_correct_go "
        f"sensor={row.get('sensor_source', 'NA')} fallback={str(bool(row.get('fallback_used'))).lower()} "
        f"gps={'OK' if row.get('gps_valid') else 'DEGRADED'} imu={'OK' if imu_ok else 'NA'} "
        f"heading_err={row['heading_error_deg']} cte={row['cross_track_error_m']} "
        f"progress={row['along_track_progress_m']} remaining={row['remaining_distance_m']} "
        f"moveA={row.get('move_a_cmd')} moveB={row.get('move_b_cmd')} "
        f"corrB={row.get('correction_b_cmd')} corr_ok={row.get('correction_success')}"
    )


def _stop_correct_go_preflight(
    handle: object,
    raw_lines: list[str],
    *,
    heartbeat_timeout_s: float,
    rc_neutral_wait_s: float,
    manual_override_mode: str,
    require_auto_switch: bool,
) -> tuple[dict[str, str] | None, str]:
    """Wait for a guarded heartbeat and apply the shared pre-move safety gates.

    Returns ``(heartbeat, abort_reason)`` where ``abort_reason`` is ``"NONE"`` when
    it is safe to move. Mirrors the per-pulse preamble of :func:`run_controller`.
    """
    heartbeat = wait_for_guarded_pulse_heartbeat(handle, raw_lines, heartbeat_timeout_s)
    if heartbeat is None:
        return None, "NO_GUARDED_PULSE_HEARTBEAT"
    rc_ignored = rc_ignored_for_usb_supervised(heartbeat)
    if telemetry._parse_bool(heartbeat.get("rc_ok")) is not True and not rc_ignored:
        return heartbeat, "RC_NOT_OK"
    if not rc_ignored and manual_override_detected(heartbeat) and manual_override_mode == "abort":
        return heartbeat, "MANUAL_OVERRIDE"
    if require_auto_switch and mode_switch_state(heartbeat) == "MANUAL":
        return heartbeat, MANUAL_SWITCH_ABORT_REASON
    if not rc_ignored and safety.rc_neutral_wait(heartbeat):
        neutral = wait_for_neutral_rc(handle, raw_lines, rc_neutral_wait_s)
        if neutral is None:
            return heartbeat, "RC_NOT_NEUTRAL"
        heartbeat = neutral
    return heartbeat, "NONE"


def build_stop_correct_go_summary(
    rows: Sequence[dict[str, object]],
    *,
    start_lat: float,
    start_lon: float,
    goal_lat: float,
    goal_lon: float,
    goal_distance_m: float,
    fallback_to_repeated_pulses: bool,
    sensor_trust_mode: str,
    allow_calibration_fallback: bool,
    abort_reason: str = "NONE",
    heading_reference: str = DEFAULT_HEADING_REFERENCE,
    turn_angle_policy: str = DEFAULT_TURN_ANGLE_POLICY,
    turn_calibration_angles: dict[str, object] | None = None,
) -> dict[str, object]:
    """Controller summary augmented with stop_correct_go-specific counters."""
    summary = build_controller_summary(
        rows,
        start_lat=start_lat,
        start_lon=start_lon,
        goal_lat=goal_lat,
        goal_lon=goal_lon,
        goal_distance_m=goal_distance_m,
        fallback_to_repeated_pulses=fallback_to_repeated_pulses,
        abort_reason=abort_reason,
    )
    cycle_rows = [r for r in rows if r.get("row_type") == "pulse"]
    heading_corrections = [r for r in cycle_rows if int(r.get("correction_duration_ms") or 0) > 0]
    connector_rows = [r for r in cycle_rows if r.get("phase") == "connector"]
    connector_segments = {r.get("segment_index") for r in connector_rows}
    connector_completed_segments = {
        r.get("segment_index")
        for r in connector_rows
        if r.get("connector_turn_completed") is True
    }
    summary.update(
        {
            "path_control_mode": "stop_correct_go",
            "sensor_trust_mode": sensor_trust_mode,
            "allow_calibration_fallback": bool(allow_calibration_fallback),
            "heading_reference": heading_reference,
            "turn_calibration_angle_policy": turn_angle_policy,
            "heading_correction_count": len(heading_corrections),
            "heading_correction_success_count": sum(
                1 for r in heading_corrections if r.get("correction_success") is True
            ),
            "cross_track_trim_applied_count": sum(
                1
                for r in cycle_rows
                if abs(telemetry._optional_float(r.get("move_b_cmd")) or 0.0) > _ZERO_TOLERANCE
            ),
            "sensor_fallback_used_count": sum(1 for r in cycle_rows if r.get("fallback_used") is True),
            "connector_turn_count": len(connector_segments),
            "connector_pulse_count": len(connector_rows),
            "connector_completed_count": len(connector_completed_segments),
            "connector_incomplete_count": len(connector_segments - connector_completed_segments),
            "connector_imu_measured_pulse_count": sum(
                1 for r in connector_rows if r.get("turn_measured_by_imu") is True
            ),
        }
    )
    if turn_calibration_angles:
        summary.update(turn_calibration_angles)
    return checks.assert_not_ready_for_full_path_following(summary)


def run_stop_correct_go(
    handle: object,
    *,
    segments: Sequence[dict[str, object]],
    resolved_calibration: dict[str, object],
    start_lat: float,
    start_lon: float,
    start_yaw_deg: float | None,
    goal_lat: float,
    goal_lon: float,
    move_chunk_ms: int = DEFAULT_MOVE_CHUNK_MS,
    settle_after_move_ms: int = DEFAULT_SETTLE_AFTER_MOVE_MS,
    telemetry_stabilize_ms: int = DEFAULT_TELEMETRY_STABILIZE_MS,
    heading_correction_threshold_deg: float = DEFAULT_HEADING_CORRECTION_THRESHOLD_DEG,
    heading_correction_tolerance_deg: float = DEFAULT_HEADING_CORRECTION_TOLERANCE_DEG,
    cross_track_correction_threshold_m: float = DEFAULT_CROSS_TRACK_CORRECTION_THRESHOLD_M,
    heading_correction_b_left: float = DEFAULT_HEADING_CORRECTION_B_LEFT,
    heading_correction_b_right: float = DEFAULT_HEADING_CORRECTION_B_RIGHT,
    max_heading_correction_ms: int = DEFAULT_MAX_HEADING_CORRECTION_MS,
    heading_correction_chunk_ms: int = DEFAULT_HEADING_CORRECTION_CHUNK_MS,
    sensor_trust_mode: str = DEFAULT_SENSOR_TRUST_MODE,
    allow_calibration_fallback: bool = True,
    event_timeout_s: float = DEFAULT_EVENT_TIMEOUT_S,
    heartbeat_timeout_s: float = DEFAULT_HEARTBEAT_TIMEOUT_S,
    rc_neutral_wait_s: float = DEFAULT_RC_NEUTRAL_WAIT_S,
    gps_degradation_policy: str = DEFAULT_GPS_DEGRADATION_POLICY,
    manual_override_mode: str = DEFAULT_MANUAL_OVERRIDE_MODE,
    left_fixed_pulses: int = 12,
    right_fixed_pulses: int = 12,
    live_update_hz: float = 8.0,
    live_ttl_ms: int = 350,
    max_segment_chunks: int = 20,
    k_cross_track: float = 0.20,
    max_correction_b: float = 0.08,
    gps_reanchor: bool = True,
    require_auto_switch: bool = False,
    verbose_raw: bool = True,
    max_connector_pulses_per_turn: int = DEFAULT_MAX_CONNECTOR_PULSES_PER_TURN,
    connector_turn_tolerance_deg: float = DEFAULT_CONNECTOR_TURN_TOLERANCE_DEG,
    turn_angle_policy: str = DEFAULT_TURN_ANGLE_POLICY,
    turn_angle_override: float | None = None,
    heading_reference: str = DEFAULT_HEADING_REFERENCE,
) -> tuple[list[dict[str, object]], list[str], str]:
    """Run the stop_correct_go loop over ``segments``; return (rows, raw_lines, abort).

    Lane segments advance one bounded guarded chunk at a time, fully stopping and
    confirming zero before reading a settled pose; heading errors over threshold
    are corrected by a discrete IMU turn-in-place and cross-track error is trimmed
    onto the next chunk's B.

    Connector turns pulse the calibrated turn primitive repeatedly until the IMU
    yaw delta (measured from the yaw at the connector's start) reaches the
    segment's requested angle within ``connector_turn_tolerance_deg``, capped by
    ``max_connector_pulses_per_turn``. The per-pulse rotation comes from the
    calibration's ``target_angle_deg`` (``turn_angle_policy='from_json'``) -- a
    turn_*_90 entry that physically turns only ~30 degrees therefore gets ~3
    pulses, not 1. Without IMU yaw the open-loop pulse count derived from the
    same angle is used. Expected field faults set ``abort_reason`` and stop the
    loop cleanly rather than raising.
    """
    rows: list[dict[str, object]] = []
    raw_lines: list[str] = []
    gps_cache: dict[str, object] = {"lat": start_lat, "lon": start_lon, "degraded": False}
    pose_state: dict[str, object] = {"x": 0.0, "y": 0.0, "source": "plan_start", "gps_degraded": False}
    abort_reason = "NONE"
    primitive_index = 0
    provided_start_yaw = start_yaw_deg
    first_lane_pending = True
    if sensor_trust_mode not in SENSOR_TRUST_MODES:
        sensor_trust_mode = DEFAULT_SENSOR_TRUST_MODE
    if heading_reference not in HEADING_REFERENCES:
        heading_reference = DEFAULT_HEADING_REFERENCE
    if turn_angle_policy not in TURN_ANGLE_POLICIES:
        turn_angle_policy = DEFAULT_TURN_ANGLE_POLICY
    effective_max_chunks = max(1, int(max_segment_chunks))
    dead_reckon_advance_m = max(0.01, 0.30 * (float(move_chunk_ms) / 700.0))

    # Mission heading frame: heading = yaw + offset, captured once while the
    # rover is still aligned with the first lane (operator alignment or the
    # initial-heading-align step). Chaining one reference across the whole run
    # keeps a connector under-turn visible as next-lane heading error instead of
    # silently re-zeroing it per lane. The frame may only be captured before the
    # first connector turn; if IMU yaw appears later the run degrades to the
    # legacy per-lane behavior rather than capturing a frame mid-rotation.
    first_lane_heading = next(
        (
            float(s["target_heading_deg"])
            for s in segments
            if str(s.get("segment_type")) not in {"connector_turn", "path_connector"}
        ),
        float(segments[0]["target_heading_deg"]) if segments else 0.0,
    )
    mission_frame_offset: float | None = None
    mission_capture_allowed = True
    if provided_start_yaw is not None:
        mission_frame_offset = geometry.wrap_deg(first_lane_heading - provided_start_yaw)

    def maybe_capture_mission_frame(row: dict[str, str] | None) -> None:
        nonlocal mission_frame_offset
        if (
            heading_reference != "mission"
            or mission_frame_offset is not None
            or not mission_capture_allowed
        ):
            return
        yaw_now = telemetry.imu_relative_yaw_deg(row) if row else None
        if yaw_now is not None:
            mission_frame_offset = geometry.wrap_deg(first_lane_heading - yaw_now)

    def mission_heading(yaw_value: float | None) -> float | None:
        if (
            heading_reference != "mission"
            or yaw_value is None
            or mission_frame_offset is None
        ):
            return None
        return geometry.wrap_deg(yaw_value + mission_frame_offset)

    for segment in segments:
        is_connector = str(segment["segment_type"]) in {"connector_turn", "path_connector"}
        target_heading = float(segment["target_heading_deg"])
        length_m = float(segment.get("length_m", 0.0))

        if is_connector:
            mission_capture_allowed = False
            direction = "left" if str(segment["expected_motion_direction"]) == "turn_left" else "right"
            connector = connector_command(resolved_calibration, direction)
            a_cmd = float(connector["a_cmd"])
            connector_b = float(connector["b_cmd"])
            pulse_ms = int(connector["pulse_ms"])
            calibration_source = str(connector["calibration_source"])
            connector_mode = str(connector["connector_mode"])
            requested_angle = connector_turn_angle_deg(segment)
            per_pulse = per_pulse_turn_angle_deg(
                connector,
                turn_angle_policy=turn_angle_policy,
                turn_angle_override=turn_angle_override,
            )
            planned_pulses = connector_planned_pulses(requested_angle, per_pulse)
            fixed_budget: int | None = None
            if connector_mode == "repeated_pulses":
                fixed_budget = int(
                    left_fixed_pulses if direction == "left" else right_fixed_pulses
                )
            connector_yaw_ref: float | None = None
            applied_delta = 0.0
            remaining_angle = requested_angle
            turn_completed = False
            budget: int | None = fixed_budget
            chunk_index = 0
            while True:
                chunk_index += 1
                primitive_index += 1
                try:
                    heartbeat, gate = _stop_correct_go_preflight(
                        handle,
                        raw_lines,
                        heartbeat_timeout_s=heartbeat_timeout_s,
                        rc_neutral_wait_s=rc_neutral_wait_s,
                        manual_override_mode=manual_override_mode,
                        require_auto_switch=require_auto_switch,
                    )
                    if gate != "NONE":
                        abort_reason = gate
                        break
                    if connector_yaw_ref is None:
                        connector_yaw_ref = telemetry.imu_relative_yaw_deg(heartbeat)
                    if budget is None:
                        # Angle-calibrated turns: with IMU feedback allow a couple of
                        # verified extra pulses; open loop must stop at the planned count.
                        budget = connector_pulse_budget(
                            requested_angle,
                            per_pulse,
                            max_pulses=max_connector_pulses_per_turn,
                            imu_available=connector_yaw_ref is not None,
                        )
                    gps = dead_reckon_gps(heartbeat, gps_cache)
                    planned = planned_pulse(
                        seq=primitive_index, a_cmd=a_cmd, b_cmd=connector_b, pulse_ms=pulse_ms
                    )
                    pulse_rows = executor.send_pulse(
                        handle, planned, raw_lines, event_timeout_s=event_timeout_s, verbose_raw=verbose_raw
                    )
                    manual_after_pulse = require_auto_switch and manual_switch_seen(pulse_rows)
                    after = _collect_stabilized_heartbeat(
                        handle,
                        raw_lines,
                        settle_after_move_ms=settle_after_move_ms,
                        telemetry_stabilize_ms=telemetry_stabilize_ms,
                        heartbeat_timeout_s=heartbeat_timeout_s,
                        verbose_raw=verbose_raw,
                    ) or heartbeat
                except OSError:
                    abort_reason = "SERIAL_DISCONNECT"
                    break
                gps_after = dead_reckon_gps(after, gps_cache)
                lat = float(gps_after["lat"]) if gps_after["lat"] is not None else start_lat
                lon = float(gps_after["lon"]) if gps_after["lon"] is not None else start_lon
                x, y = geometry.goal_to_local(start_lat, start_lon, lat, lon)
                along, signed_cte, _ = geometry.projection_metrics(segment, x, y)
                yaw = telemetry.imu_relative_yaw_deg(after)
                imu_valid = yaw is not None
                # Measure how far the body has actually rotated since the
                # connector started; fall back to assuming the calibrated
                # per-pulse angle when yaw is unavailable.
                if connector_yaw_ref is not None and imu_valid:
                    applied_delta = geometry.wrap_deg(yaw - connector_yaw_ref)
                    remaining_angle = geometry.wrap_deg(requested_angle - applied_delta)
                    turn_completed = abs(remaining_angle) <= abs(connector_turn_tolerance_deg)
                    turn_measured = True
                else:
                    if per_pulse is not None:
                        applied_delta = math.copysign(per_pulse * chunk_index, requested_angle)
                        remaining_angle = geometry.wrap_deg(requested_angle - applied_delta)
                    turn_completed = chunk_index >= planned_pulses if per_pulse is not None else False
                    turn_measured = False
                mission_h = mission_heading(yaw)
                if mission_h is not None:
                    current_heading = mission_h
                else:
                    current_heading = current_heading_deg(target_heading, yaw, provided_start_yaw)
                correction = {
                    "current_heading_deg": current_heading,
                    "heading_error_deg": geometry.wrap_deg(target_heading - current_heading),
                    "cross_track_error_m": signed_cte,
                    "along_track_progress_m": along,
                    "remaining_distance_m": max(0.0, length_m - along),
                    "b_cmd": connector_b,
                    "b_heading_component": 0.0,
                    "b_cte_component": 0.0,
                    "correction_source": "connector_calibration",
                }
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
                    pulse_ms=pulse_ms,
                    gps=gps_after,
                    calibration_source=calibration_source,
                    connector_mode=connector_mode,
                    chunk_index=chunk_index,
                    path_control_mode="stop_correct_go",
                    pose={
                        "x": x,
                        "y": y,
                        "gps_valid": not bool(gps_after["gps_degraded"]),
                        "gps_reanchored": False,
                    },
                    base_a_cmd=a_cmd,
                    b_trim=0.0,
                )
                row.update(
                    {
                        "drive_mode": "stop_correct_go",
                        "phase": "connector",
                        "imu_valid": imu_valid,
                        "move_a_cmd": f"{a_cmd:.3f}",
                        "move_b_cmd": f"{connector_b:.3f}",
                        "correction_b_cmd": "0.000",
                        "correction_duration_ms": 0,
                        "correction_success": "NA",
                        "post_correction_heading_error_deg": telemetry._fmt(correction["heading_error_deg"]),
                        "sensor_source": "imu" if imu_valid else "calibration",
                        "fallback_used": not imu_valid,
                        "heading_reference": heading_reference,
                        "turn_angle_policy": turn_angle_policy,
                        "requested_turn_angle_deg": telemetry._fmt(requested_angle),
                        "calibration_target_angle_deg": (
                            telemetry._fmt(per_pulse) if per_pulse is not None else "NA"
                        ),
                        "turn_pulse_index": chunk_index,
                        "turn_pulse_budget": budget if budget is not None else "NA",
                        "turn_planned_pulses": planned_pulses,
                        "yaw_turn_ref_deg": telemetry._fmt(connector_yaw_ref),
                        "applied_turn_delta_deg": telemetry._fmt(applied_delta),
                        "remaining_turn_error_deg": telemetry._fmt(remaining_angle),
                        "turn_measured_by_imu": turn_measured,
                        "connector_turn_completed": turn_completed,
                    }
                )
                rows.append(row)
                if verbose_raw:
                    print(stop_correct_go_status_line(row))
                if row["valid_pulse"] is not True:
                    abort_reason = str(row["invalid_reason"])
                    break
                if manual_after_pulse:
                    abort_reason = MANUAL_SWITCH_ABORT_REASON
                    break
                if turn_completed or chunk_index >= int(budget or 1):
                    break
            if abort_reason != "NONE":
                break
            continue

        # --- straight lane: discrete move -> stop -> measure -> correct ---------
        motion = geometry._motion_calibrated(
            resolved_calibration, str(segment["expected_motion_direction"])
        )
        a_cmd = float(motion["a_cmd"])
        calibration_source = str(motion.get("calibration_source", "unknown"))
        segment_ref_yaw: float | None = None
        next_b_trim = 0.0
        for chunk_index in range(1, effective_max_chunks + 1):
            primitive_index += 1
            try:
                heartbeat, gate = _stop_correct_go_preflight(
                    handle,
                    raw_lines,
                    heartbeat_timeout_s=heartbeat_timeout_s,
                    rc_neutral_wait_s=rc_neutral_wait_s,
                    manual_override_mode=manual_override_mode,
                    require_auto_switch=require_auto_switch,
                )
                if gate != "NONE":
                    abort_reason = gate
                    break
                maybe_capture_mission_frame(heartbeat)
                if segment_ref_yaw is None:
                    segment_ref_yaw = reference_yaw_for_segment(
                        heartbeat,
                        provided_start_yaw=provided_start_yaw,
                        use_provided=first_lane_pending,
                    )
                    first_lane_pending = False
                gps_before = dead_reckon_gps(heartbeat, gps_cache)
                gps_action = geometry.gps_policy_action(
                    bool(gps_before["gps_degraded"]), gps_degradation_policy
                )
                if gps_action == "abort":
                    abort_reason = "GPS_DEGRADED"
                    break
                if gps_action == "pause":
                    continue
                # MOVE: one bounded guarded chunk (B = cross-track trim from prior cycle).
                move_b = float(next_b_trim)
                planned = planned_pulse(
                    seq=primitive_index, a_cmd=a_cmd, b_cmd=move_b, pulse_ms=int(move_chunk_ms)
                )
                pulse_rows = executor.send_pulse(
                    handle, planned, raw_lines, event_timeout_s=event_timeout_s, verbose_raw=verbose_raw
                )
                manual_after_pulse = require_auto_switch and manual_switch_seen(pulse_rows)
                # SETTLE + STABILIZE telemetry, then read a settled pose.
                stabilized = _collect_stabilized_heartbeat(
                    handle,
                    raw_lines,
                    settle_after_move_ms=settle_after_move_ms,
                    telemetry_stabilize_ms=telemetry_stabilize_ms,
                    heartbeat_timeout_s=heartbeat_timeout_s,
                    verbose_raw=verbose_raw,
                ) or heartbeat
            except OSError:
                abort_reason = "SERIAL_DISCONNECT"
                break

            gps_after = dead_reckon_gps(stabilized, gps_cache)
            yaw = telemetry.imu_relative_yaw_deg(stabilized)
            imu_valid = yaw is not None
            gps_valid = bool(
                gps_after["lat"] is not None
                and gps_after["lon"] is not None
                and not gps_after["gps_degraded"]
            )
            sensor = stop_correct_go_sensor_source(
                gps_valid=gps_valid,
                imu_valid=imu_valid,
                trust_mode=sensor_trust_mode,
                allow_calibration_fallback=allow_calibration_fallback,
            )
            if not sensor["ok"]:
                abort_reason = "SENSOR_UNAVAILABLE"
                break
            pose = _pose_from_gps_or_dead_reckon(
                row=stabilized,
                gps=gps_after,
                pose_state=pose_state,
                segment=segment,
                start_lat=start_lat,
                start_lon=start_lon,
                advance_m=0.0 if gps_valid else dead_reckon_advance_m,
                gps_reanchor=gps_reanchor,
            )
            x = float(pose["x"])
            y = float(pose["y"])
            along, signed_cte, _ = geometry.projection_metrics(segment, x, y)
            mission_h = mission_heading(yaw if imu_valid else None)
            if mission_h is not None:
                current_heading = mission_h
            else:
                current_heading = current_heading_deg(
                    target_heading, yaw if imu_valid else None, segment_ref_yaw
                )
            heading_error = geometry.wrap_deg(target_heading - current_heading)
            remaining_m = max(0.0, length_m - along)

            # HEADING CORRECTION: discrete IMU turn-in-place when over threshold.
            decision = stop_correct_go_heading_decision(
                heading_error_deg=heading_error,
                threshold_deg=heading_correction_threshold_deg,
                imu_valid=imu_valid,
                b_left=heading_correction_b_left,
                b_right=heading_correction_b_right,
            )
            correction_b_cmd = 0.0
            correction_duration_ms = 0
            correction_success: object = "NA"
            post_correction_error = heading_error
            correction_manual_switch = False
            phase = "move"
            if decision["needs_correction"] and not manual_after_pulse:
                phase = "correction"
                try:
                    turn = _run_stop_correct_go_heading_turn(
                        handle,
                        correction_b_cmd=float(decision["correction_b_cmd"]),
                        yaw_turn_start=float(yaw),
                        initial_heading_error_deg=heading_error,
                        heading_tolerance_deg=heading_correction_tolerance_deg,
                        max_correction_ms=max_heading_correction_ms,
                        update_hz=live_update_hz,
                        ttl_ms=live_ttl_ms,
                        chunk_ms=heading_correction_chunk_ms,
                        event_timeout_s=event_timeout_s,
                        raw_lines=raw_lines,
                        verbose_raw=verbose_raw,
                        abort_on_manual_switch=require_auto_switch,
                    )
                except OSError:
                    abort_reason = "SERIAL_DISCONNECT"
                    break
                correction_b_cmd = float(decision["correction_b_cmd"])
                correction_duration_ms = int(turn["turn_duration_ms"])
                correction_success = bool(turn["correction_success"])
                post_correction_error = float(turn["final_heading_error_deg"])
                correction_manual_switch = bool(turn.get("manual_switch"))

            # CROSS-TRACK: trim the next chunk's B when off the line.
            next_b_trim = stop_correct_go_cross_track_trim(
                cross_track_error_m=signed_cte,
                threshold_m=cross_track_correction_threshold_m,
                k_cross_track=k_cross_track,
                max_correction_b=max_correction_b,
            )

            correction = {
                "current_heading_deg": current_heading,
                "heading_error_deg": heading_error,
                "cross_track_error_m": signed_cte,
                "along_track_progress_m": along,
                "remaining_distance_m": remaining_m,
                "b_cmd": move_b,
                "b_heading_component": 0.0,
                "b_cte_component": move_b,
                "correction_source": "stop_correct_go",
            }
            row = build_execution_row(
                segment=segment,
                primitive_index=primitive_index,
                after_row=stabilized,
                pulse_rows=pulse_rows,
                start_lat=start_lat,
                start_lon=start_lon,
                goal_lat=goal_lat,
                goal_lon=goal_lon,
                start_yaw_deg=start_yaw_deg,
                target_heading_deg=target_heading,
                a_cmd=a_cmd,
                correction=correction,
                pulse_ms=int(move_chunk_ms),
                gps=gps_after,
                calibration_source=calibration_source,
                connector_mode="lane",
                chunk_index=chunk_index,
                path_control_mode="stop_correct_go",
                pose={
                    "x": x,
                    "y": y,
                    "gps_valid": gps_valid,
                    "gps_reanchored": bool(pose.get("gps_reanchored")),
                },
                base_a_cmd=a_cmd,
                b_trim=move_b,
            )
            row.update(
                {
                    "drive_mode": "stop_correct_go",
                    "phase": phase,
                    "imu_valid": imu_valid,
                    "move_a_cmd": f"{a_cmd:.3f}",
                    "move_b_cmd": f"{move_b:.3f}",
                    "correction_b_cmd": f"{correction_b_cmd:.3f}",
                    "correction_duration_ms": correction_duration_ms,
                    "correction_success": correction_success,
                    "post_correction_heading_error_deg": telemetry._fmt(post_correction_error),
                    "next_b_trim": f"{next_b_trim:.3f}",
                    "sensor_source": str(sensor["source"]),
                    "fallback_used": bool(sensor["fallback_used"]),
                    "heading_reference": heading_reference,
                    "turn_angle_policy": turn_angle_policy,
                }
            )
            rows.append(row)
            if verbose_raw:
                print(stop_correct_go_status_line(row))
            if row["valid_pulse"] is not True:
                abort_reason = str(row["invalid_reason"])
                break
            if manual_after_pulse or correction_manual_switch:
                abort_reason = MANUAL_SWITCH_ABORT_REASON
                break
            if remaining_m <= max(0.05, dead_reckon_advance_m * 0.5) or along >= length_m:
                break

        if abort_reason != "NONE":
            break
        if rows and rows[-1]["valid_pulse"] is not True:
            break

    return rows, raw_lines, abort_reason
