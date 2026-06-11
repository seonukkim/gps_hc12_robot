"""Physical motion/turn calibration resolver and calibrated-primitive accessors.

Reads the calibration JSON sources (interactive motion, fine motion, turn twitch,
turn angle, smooth connector) in priority order and produces a normalized schema with a
``connector_mode_effective`` plus ``ready_*`` flags. ``ready_for_full_path_following``
is always False.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Sequence


DEFAULT_FINE_CALIBRATION = Path("outputs/" + "stage" + "20_physical_ab_probe/calibration/physical_ab_fine_motion_calibration.json")
DEFAULT_TURN_CALIBRATION = Path("outputs/stage23_turn_calibration/calibration/physical_ab_turn_twitch_calibration.json")
DEFAULT_TURN_ANGLE_CALIBRATION = Path("outputs/stage23_turn_calibration/calibration/physical_ab_turn_angle_calibration.json")
DEFAULT_SMOOTH_TURN_CALIBRATION = Path("outputs/stage36_smooth_turn_connector/calibration/smooth_turn_connector_calibration.json")
DEFAULT_MOTION_CALIBRATION = Path("outputs/physical_path_planning/calibration/motion_calibration.json")

DEFAULT_FORWARD_A_CMD = 0.30
DEFAULT_FORWARD_MS = 800
DEFAULT_BACKWARD_A_CMD = -0.08
DEFAULT_BACKWARD_MS = 300
DEFAULT_TURN_LEFT_B_CMD = 0.26
DEFAULT_TURN_LEFT_MS = 700
DEFAULT_TURN_RIGHT_B_CMD = -0.08
DEFAULT_TURN_RIGHT_MS = 250
DEFAULT_LEFT_FIXED_PULSES = 12
DEFAULT_RIGHT_FIXED_PULSES = 12


def _parse_bool(value: object, default: bool = False) -> bool:
    if value is None:
        return default
    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "y", "ok", "ready"}:
        return True
    if text in {"0", "false", "no", "n", "off", "inactive"}:
        return False
    return default


def _optional_float(value: object) -> float | None:
    if value is None:
        return None
    text = str(value).strip()
    if text.upper() in {"", "NA", "NAN", "NONE", "NULL"}:
        return None
    try:
        parsed = float(text)
    except ValueError:
        return None
    return parsed if math.isfinite(parsed) else None


def _optional_int(value: object) -> int | None:
    parsed = _optional_float(value)
    return int(parsed) if parsed is not None else None


def _load_json_if_present(path: Path | None) -> tuple[dict[str, object], str | None]:
    if path is None or not path.exists():
        return {}, None
    loaded = json.loads(path.read_text(encoding="utf-8"))
    return (loaded if isinstance(loaded, dict) else {}), path.name


def _first_float(data: dict[str, object], keys: Sequence[str]) -> tuple[float | None, str | None]:
    for key in keys:
        parsed = _optional_float(data.get(key))
        if parsed is not None and parsed != 0.0:
            return parsed, key
    return None, None


def _first_int(data: dict[str, object], keys: Sequence[str]) -> tuple[int | None, str | None]:
    for key in keys:
        parsed = _optional_int(data.get(key))
        if parsed is not None and parsed > 0:
            return parsed, key
    return None, None


def _primitive(
    *,
    a: float,
    b: float,
    ms: int,
    source: str,
) -> dict[str, object]:
    return {"a": round(a, 3), "b": round(b, 3), "ms": int(ms), "source": source}


def _approved_entry(data: dict[str, object], key: str) -> dict[str, object] | None:
    entry = data.get(key)
    if not isinstance(entry, dict) or not _parse_bool(entry.get("approved_by_user")):
        return None
    a = _optional_float(entry.get("a_cmd", entry.get("a")))
    b = _optional_float(entry.get("b_cmd", entry.get("b")))
    ms = _optional_int(entry.get("pulse_ms", entry.get("ms")))
    if a is None or b is None or ms is None:
        return None
    return entry


def _interactive_motion_override(
    data: dict[str, object],
    source_name: str | None,
    key: str,
    fallback: dict[str, object],
) -> dict[str, object]:
    entry = _approved_entry(data, key)
    if entry is None:
        return fallback
    a = _optional_float(entry.get("a_cmd", entry.get("a")))
    b = _optional_float(entry.get("b_cmd", entry.get("b")))
    ms = _optional_int(entry.get("pulse_ms", entry.get("ms")))
    if a is None or b is None or ms is None:
        return fallback
    if key == "forward":
        a = abs(a)
    elif key == "backward":
        a = -abs(a)
    elif key == "left":
        b = abs(b)
    elif key == "right":
        b = -abs(b)
    return _primitive(
        a=a,
        b=b,
        ms=ms,
        source=source_name or "interactive_motion_calibration",
    )


def _motion_primitive(
    data: dict[str, object],
    source_name: str | None,
    *,
    direction: str,
) -> dict[str, object]:
    if direction == "forward":
        value, value_key = _first_float(
            data,
            (
                "stage26_forward_a_cmd",
                "stage22_forward_a_cmd",
                "stage21_forward_a_cmd",
                "forward_recommended_crawl_a_cmd",
                "forward_visible_a_cmd",
            ),
        )
        ms, ms_key = _first_int(
            data,
            (
                "stage26_forward_pulse_ms",
                "stage22_forward_pulse_ms",
                "stage21_forward_pulse_ms",
                "forward_recommended_crawl_pulse_ms",
                "forward_visible_pulse_ms",
            ),
        )
        return _primitive(
            a=abs(value) if value is not None else DEFAULT_FORWARD_A_CMD,
            b=0.0,
            ms=ms if ms is not None else DEFAULT_FORWARD_MS,
            source=source_name if source_name and (value_key or ms_key) else "fallback_known_forward",
        )
    value, value_key = _first_float(
        data,
        (
            "stage26_backward_a_cmd",
            "stage21_backward_a_cmd",
            "backward_recommended_crawl_a_cmd",
            "backward_visible_a_cmd",
            "backward_min_a_cmd",
        ),
    )
    ms, ms_key = _first_int(
        data,
        (
            "stage26_backward_pulse_ms",
            "stage21_backward_pulse_ms",
            "backward_recommended_crawl_pulse_ms",
            "backward_visible_pulse_ms",
            "backward_min_pulse_ms",
        ),
    )
    return _primitive(
        a=-abs(value) if value is not None else DEFAULT_BACKWARD_A_CMD,
        b=0.0,
        ms=ms if ms is not None else DEFAULT_BACKWARD_MS,
        source=source_name if source_name and (value_key or ms_key) else "fallback_known_backward",
    )


def _turn_primitive(
    data: dict[str, object],
    source_name: str | None,
    *,
    direction: str,
) -> dict[str, object]:
    if direction == "left":
        value, value_key = _first_float(
            data,
            ("stage26_turn_left_b_cmd", "turn_left_usable_b_cmd", "turn_left_min_b_cmd", "safe_turn_left_cmd"),
        )
        ms, ms_key = _first_int(
            data,
            ("stage26_turn_left_pulse_ms", "turn_left_usable_pulse_ms", "turn_left_min_pulse_ms", "safe_turn_left_pulse_ms"),
        )
        return _primitive(
            a=0.0,
            b=abs(value) if value is not None else DEFAULT_TURN_LEFT_B_CMD,
            ms=ms if ms is not None else DEFAULT_TURN_LEFT_MS,
            source=source_name if source_name and (value_key or ms_key) else "fallback_known_turn_left",
        )
    value, value_key = _first_float(
        data,
        ("stage26_turn_right_b_cmd", "turn_right_small_b_cmd", "turn_right_min_b_cmd", "safe_turn_right_cmd"),
    )
    ms, ms_key = _first_int(
        data,
        ("stage26_turn_right_pulse_ms", "turn_right_small_pulse_ms", "turn_right_min_pulse_ms", "safe_turn_right_pulse_ms"),
    )
    return _primitive(
        a=0.0,
        b=-abs(value) if value is not None else DEFAULT_TURN_RIGHT_B_CMD,
        ms=ms if ms is not None else DEFAULT_TURN_RIGHT_MS,
        source=source_name if source_name and (value_key or ms_key) else "fallback_known_turn_right",
    )


def _angle_turn_entry(data: dict[str, object], source_name: str | None, key: str) -> dict[str, object]:
    entry = data.get(key)
    if not isinstance(entry, dict):
        return {"available": False, "source": source_name or "missing"}
    ready = _parse_bool(entry.get("ready")) and str(entry.get("visual_confirmation", "")).strip().lower() == "yes"
    a = _optional_float(entry.get("a_cmd", entry.get("a")))
    b = _optional_float(entry.get("b_cmd", entry.get("b")))
    ms = _optional_int(entry.get("pulse_ms", entry.get("ms")))
    yaw = _optional_float(entry.get("imu_yaw_delta_deg"))
    if not ready or a is None or b is None or ms is None:
        return {
            "available": False,
            "visual_confirmation": entry.get("visual_confirmation", "unknown"),
            "ready": _parse_bool(entry.get("ready")),
            "source": source_name or "missing",
        }
    return {
        "available": True,
        "a": round(a, 3),
        "b": round(b, 3),
        "ms": ms,
        "imu_yaw_delta_deg": None if yaw is None else round(yaw, 3),
        "visual_confirmation": str(entry.get("visual_confirmation", "unknown")),
        "source": source_name or "missing",
    }


def _interactive_angle_turn_entry(
    data: dict[str, object],
    source_name: str | None,
    key: str,
    fallback: dict[str, object],
) -> dict[str, object]:
    entry = _approved_entry(data, key)
    if entry is None:
        return fallback
    a = _optional_float(entry.get("a_cmd", entry.get("a")))
    b = _optional_float(entry.get("b_cmd", entry.get("b")))
    ms = _optional_int(entry.get("pulse_ms", entry.get("ms")))
    yaw = _optional_float(entry.get("last_imu_yaw_delta_deg", entry.get("imu_yaw_delta_deg")))
    target = _optional_float(entry.get("target_angle_deg"))
    if a is None or b is None or ms is None:
        return fallback
    if key == "turn_left_90":
        b = abs(b)
    else:
        b = -abs(b)
    return {
        "available": True,
        "a": round(a, 3),
        "b": round(b, 3),
        "ms": ms,
        "target_angle_deg": target if target is not None else 90.0,
        "imu_yaw_delta_deg": None if yaw is None else round(yaw, 3),
        "visual_confirmation": str(entry.get("visual_confirmation", "approved")),
        "ready": True,
        "source": source_name or "interactive_motion_calibration",
    }


def _smooth_turn_entry(data: dict[str, object], source_name: str | None, *, direction: str, allow_uncalibrated: bool) -> dict[str, object]:
    prefix = "smooth_left" if direction == "left" else "smooth_right"
    default_b = 0.20 if direction == "left" else -0.08
    b_value = _optional_float(data.get(f"{prefix}_b_cmd"))
    ms = _optional_int(data.get(f"{prefix}_max_ms"))
    ready = _parse_bool(data.get("ready_for_smooth_connectors")) and source_name is not None
    if not ready and not allow_uncalibrated:
        return {"available": False, "source": source_name or "missing"}
    signed_b = abs(b_value) if b_value is not None else abs(default_b)
    if direction == "right":
        signed_b = -signed_b
    return {
        "available": bool(ready or allow_uncalibrated),
        "a": 0.0,
        "b": round(signed_b, 3),
        "ms": ms if ms is not None else 3000,
        "target_angle_deg": _optional_float(data.get(f"{prefix}_target_angle_deg")) or 90.0,
        "angle_tolerance_deg": _optional_float(data.get(f"{prefix}_angle_tolerance_deg")) or 10.0,
        "source": source_name if ready else "uncalibrated_smooth_defaults",
    }


def resolve_physical_calibration(
    *,
    motion_calibration_json: Path | None = None,
    fine_calibration_json: Path | None = DEFAULT_FINE_CALIBRATION,
    turn_calibration_json: Path | None = DEFAULT_TURN_CALIBRATION,
    turn_angle_calibration_json: Path | None = DEFAULT_TURN_ANGLE_CALIBRATION,
    smooth_turn_calibration_json: Path | None = DEFAULT_SMOOTH_TURN_CALIBRATION,
    calibration_mode: str = "auto",
    allow_uncalibrated_smooth: bool = False,
    left_fixed_pulses: int = DEFAULT_LEFT_FIXED_PULSES,
    right_fixed_pulses: int = DEFAULT_RIGHT_FIXED_PULSES,
) -> dict[str, object]:
    motion, motion_source = _load_json_if_present(motion_calibration_json)
    fine, fine_source = _load_json_if_present(fine_calibration_json)
    turn, turn_source = _load_json_if_present(turn_calibration_json)
    angle, angle_source = _load_json_if_present(turn_angle_calibration_json)
    smooth, smooth_source = _load_json_if_present(smooth_turn_calibration_json)

    left_90 = _interactive_angle_turn_entry(
        motion, motion_source, "turn_left_90", _angle_turn_entry(angle, angle_source, "turn_left_90")
    )
    right_90 = _interactive_angle_turn_entry(
        motion, motion_source, "turn_right_90", _angle_turn_entry(angle, angle_source, "turn_right_90")
    )
    angle_ready = bool(left_90.get("available")) and bool(right_90.get("available"))
    smooth_left = _smooth_turn_entry(smooth, smooth_source, direction="left", allow_uncalibrated=allow_uncalibrated_smooth)
    smooth_right = _smooth_turn_entry(smooth, smooth_source, direction="right", allow_uncalibrated=allow_uncalibrated_smooth)
    smooth_ready = bool(smooth_left.get("available")) and bool(smooth_right.get("available")) and (
        bool(smooth.get("ready_for_smooth_connectors")) or allow_uncalibrated_smooth
    )

    if angle_ready:
        recommended = "angle_calibrated"
    elif smooth_ready:
        recommended = "smooth_imu"
    else:
        recommended = "repeated_pulses"

    if calibration_mode == "auto":
        effective = recommended
    elif calibration_mode == "angle_calibrated":
        if not angle_ready:
            raise RuntimeError("angle-calibrated connector requested but physical_ab_turn_angle_calibration.json is missing or incomplete.")
        effective = "angle_calibrated"
    elif calibration_mode == "smooth_imu":
        if not smooth_ready:
            raise RuntimeError("smooth-imu connector requested but smooth_turn_connector_calibration.json is missing or incomplete.")
        effective = "smooth_imu"
    elif calibration_mode == "repeated_pulses":
        effective = "repeated_pulses"
    else:
        raise ValueError(f"unsupported calibration_mode: {calibration_mode}")

    return {
        "forward": _interactive_motion_override(
            motion, motion_source, "forward", _motion_primitive(fine, fine_source, direction="forward")
        ),
        "backward": _interactive_motion_override(
            motion, motion_source, "backward", _motion_primitive(fine, fine_source, direction="backward")
        ),
        "turn_left": _interactive_motion_override(
            motion, motion_source, "left", _turn_primitive(turn, turn_source, direction="left")
        ),
        "turn_right": _interactive_motion_override(
            motion, motion_source, "right", _turn_primitive(turn, turn_source, direction="right")
        ),
        "turn_left_90": left_90,
        "turn_right_90": right_90,
        "smooth_turn_left_90": smooth_left,
        "smooth_turn_right_90": smooth_right,
        "connector_mode_requested": calibration_mode,
        "connector_mode_recommended": recommended,
        "connector_mode_effective": effective,
        "ready_for_angle_based_connectors": angle_ready,
        "angle_based_connector_available": angle_ready,
        "ready_for_smooth_connectors": smooth_ready,
        "smooth_connector_available": smooth_ready,
        "fallback_to_repeated_pulses": effective == "repeated_pulses",
        "left_fixed_pulses": int(left_fixed_pulses),
        "right_fixed_pulses": int(right_fixed_pulses),
        "calibration_files": {
            "motion": str(motion_calibration_json) if motion_calibration_json else None,
            "fine": str(fine_calibration_json) if fine_calibration_json else None,
            "turn_twitch": str(turn_calibration_json) if turn_calibration_json else None,
            "turn_angle": str(turn_angle_calibration_json) if turn_angle_calibration_json else None,
            "smooth_turn": str(smooth_turn_calibration_json) if smooth_turn_calibration_json else None,
        },
        "ready_for_full_path_following": False,
    }


def planner_primitive(calibration: dict[str, object], name: str) -> dict[str, object]:
    key = {
        "move_forward": "forward",
        "move_backward": "backward",
        "turn_left": "turn_left",
        "turn_right": "turn_right",
    }.get(name, name)
    entry = calibration[key]
    if not isinstance(entry, dict):
        raise KeyError(key)
    return {
        "a_cmd": float(entry["a"]),
        "b_cmd": float(entry["b"]),
        "pulse_ms": int(entry["ms"]),
        "calibration_source": str(entry.get("source", "unknown")),
    }


def _is_motion_calibrated(primitive: object) -> bool:
    """A motion primitive is user-calibrated when it is not a known-safe fallback.

    The resolver always returns a usable ``forward``/``backward`` primitive, but
    falls back to the built-in safe values (``source`` prefixed ``fallback_known_``)
    when no real calibration file supplied them. Driving the rover physically should
    use measured calibration, so a fallback source counts as *not calibrated*.
    """
    if not isinstance(primitive, dict):
        return False
    return not str(primitive.get("source", "")).startswith("fallback_known_")


def plan_requires_backward(segments: Sequence[dict[str, object]] | None) -> bool:
    """True when any planned lane drives backward (serpentine return lanes)."""
    for segment in segments or []:
        if str(segment.get("expected_motion_direction", "")) == "backward":
            return True
    return False


def calibration_completeness(
    calibration: dict[str, object],
    *,
    segments: Sequence[dict[str, object]] | None = None,
) -> dict[str, object]:
    """Report which motion primitives are calibrated and which the plan requires.

    ``forward`` is always required to drive. ``backward`` is required only when the
    plan contains a backward (serpentine return) lane. ``turn_left_90`` /
    ``turn_right_90`` are reported but never required: a missing turn-angle
    calibration falls back to repeated fixed pulses, so it cannot block motion.
    """
    left_90 = calibration.get("turn_left_90")
    right_90 = calibration.get("turn_right_90")
    present_and_approved = {
        "forward": _is_motion_calibrated(calibration.get("forward")),
        "backward": _is_motion_calibrated(calibration.get("backward")),
        "turn_left_90": bool(isinstance(left_90, dict) and left_90.get("available")),
        "turn_right_90": bool(isinstance(right_90, dict) and right_90.get("available")),
    }
    needs_backward = plan_requires_backward(segments)
    required = ["forward"] + (["backward"] if needs_backward else [])
    missing_required = [name for name in required if not present_and_approved[name]]
    return {
        "present_and_approved": present_and_approved,
        "required_for_current_plan": required,
        "missing_required": missing_required,
        "plan_requires_backward": needs_backward,
        "can_run_stop_correct_go": not missing_required,
        "ready_for_full_path_following": False,
    }


def connector_primitive(calibration: dict[str, object], direction: str) -> dict[str, object]:
    effective = str(calibration.get("connector_mode_effective", "repeated_pulses"))
    if effective == "angle_calibrated":
        key = "turn_left_90" if direction == "left" else "turn_right_90"
    elif effective == "smooth_imu":
        key = "smooth_turn_left_90" if direction == "left" else "smooth_turn_right_90"
    else:
        key = "turn_left" if direction == "left" else "turn_right"
    entry = calibration[key]
    if not isinstance(entry, dict):
        raise KeyError(key)
    # target_angle_deg is the measured/declared rotation of ONE pulse of this
    # primitive. A turn_*_90 entry may legally hold a much smaller pulse (the
    # operator calibrated a 15-45 degree twitch); the executor must budget
    # repeated pulses / IMU feedback from this value instead of trusting the
    # "90" in the key name. Repeated-pulse twitch entries carry no angle (None).
    target_angle: float | None = None
    if effective in {"angle_calibrated", "smooth_imu"}:
        target_angle = _optional_float(entry.get("target_angle_deg"))
        if target_angle is None:
            # Older turn-angle calibrations carry only the measured IMU yaw
            # delta of one pulse; that measurement beats assuming 90.
            measured = _optional_float(entry.get("imu_yaw_delta_deg"))
            if measured is not None and abs(measured) >= 5.0:
                target_angle = abs(measured)
        if target_angle is None:
            target_angle = 90.0
    return {
        "a_cmd": float(entry["a"]),
        "b_cmd": float(entry["b"]),
        "pulse_ms": int(entry["ms"]),
        "calibration_source": str(entry.get("source", "unknown")),
        "connector_mode": effective,
        "target_angle_deg": target_angle,
    }


TURN_SMALL_PULSE_WARNING = "TURN_CALIBRATION_IS_SMALL_PULSE_NOT_90"
SMALL_PULSE_ANGLE_THRESHOLD_DEG = 60.0


def turn_angle_summary(calibration: dict[str, object]) -> dict[str, object]:
    """Per-direction turn pulse angles plus small-pulse warnings for reports.

    A ``turn_*_90`` entry whose ``target_angle_deg`` is below 60 degrees is a
    small turn pulse, not a one-shot 90 degree turn; connectors then need
    repeated pulses (or IMU feedback) to actually reach the planned corner
    angle, and the warning makes that visible in calibration-check output.
    """
    out: dict[str, object] = {}
    warnings: list[str] = []
    for key in ("turn_left_90", "turn_right_90"):
        entry = calibration.get(key)
        angle: float | None = None
        if isinstance(entry, dict) and entry.get("available"):
            angle = _optional_float(entry.get("target_angle_deg"))
            if angle is None:
                angle = 90.0
            if abs(angle) < SMALL_PULSE_ANGLE_THRESHOLD_DEG:
                warnings.append(f"{TURN_SMALL_PULSE_WARNING}:{key}:target_angle_deg={angle:g}")
        out[f"{key}_target_angle_deg"] = angle if angle is not None else "NA"
    out["turn_angle_warnings"] = warnings
    return out
