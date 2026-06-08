"""Interactive visual/IMU-assisted motion tuning helpers.

This module is deliberately pure: it owns candidate adjustment, IMU yaw-delta
extraction, and approved calibration persistence. The CLI wires these helpers
to the already-confirmed USB bounded-pulse executor.
"""
from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Sequence

from tools.physical_path_planning import calibration, geometry, telemetry

MAX_ABS_A = 0.35
MAX_ABS_B = 0.35
MAX_MS = 3000
MIN_MS = 100
TURN_TARGET_ANGLE_DEG = 90.0
TURN_ANGLE_TOLERANCE_DEG = 10.0

INITIAL_CANDIDATES: dict[str, dict[str, object]] = {
    "forward": {"primitive": "forward", "a": 0.30, "b": 0.0, "ms": 800},
    "backward": {"primitive": "backward", "a": -0.08, "b": 0.0, "ms": 300},
    "left": {"primitive": "left", "a": 0.0, "b": 0.26, "ms": 700},
    "right": {"primitive": "right", "a": 0.0, "b": -0.08, "ms": 250},
    "turn-left-90": {
        "primitive": "turn-left-90",
        "a": 0.0,
        "b": 0.24,
        "ms": 2200,
        "target_angle_deg": TURN_TARGET_ANGLE_DEG,
        "angle_tolerance_deg": TURN_ANGLE_TOLERANCE_DEG,
    },
    "turn-right-90": {
        "primitive": "turn-right-90",
        "a": 0.0,
        "b": -0.12,
        "ms": 2200,
        "target_angle_deg": TURN_TARGET_ANGLE_DEG,
        "angle_tolerance_deg": TURN_ANGLE_TOLERANCE_DEG,
    },
}

ALIASES = {
    "turn_left": "left",
    "turn_right": "right",
    "turn_left_90": "turn-left-90",
    "turn-right_90": "turn-right-90",
    "turn_right_90": "turn-right-90",
}

CALIBRATION_KEYS = {
    "forward": "forward",
    "backward": "backward",
    "left": "left",
    "right": "right",
    "turn-left-90": "turn_left_90",
    "turn-right-90": "turn_right_90",
}


def normalize_primitive(name: str) -> str:
    normalized = name.strip().lower().replace("_", "-")
    return ALIASES.get(normalized, normalized)


def initial_candidate(primitive: str) -> dict[str, object]:
    key = normalize_primitive(primitive)
    if key not in INITIAL_CANDIDATES:
        raise ValueError(f"unsupported tune-motion primitive: {primitive}")
    return dict(INITIAL_CANDIDATES[key])


def clamp_candidate(candidate: dict[str, object]) -> dict[str, object]:
    out = dict(candidate)
    primitive = str(out["primitive"])
    a = geometry.clamp(float(out["a"]), -MAX_ABS_A, MAX_ABS_A)
    b = geometry.clamp(float(out["b"]), -MAX_ABS_B, MAX_ABS_B)
    if primitive == "forward":
        a = abs(a)
    elif primitive == "backward":
        a = -abs(a)
    elif primitive in {"left", "turn-left-90"}:
        b = abs(b)
    elif primitive in {"right", "turn-right-90"}:
        b = -abs(b)
    out["a"] = round(a, 3)
    out["b"] = round(b, 3)
    out["ms"] = int(geometry.clamp(float(out["ms"]), MIN_MS, MAX_MS))
    return out


def yaw_delta_from_rows(rows: Sequence[dict[str, str]]) -> float | None:
    yaws = [
        yaw for yaw in (telemetry.imu_relative_yaw_deg(row) for row in rows)
        if yaw is not None
    ]
    if len(yaws) < 2:
        return None
    return round(geometry.wrap_deg(yaws[-1] - yaws[0]), 3)


def adjust_candidate(
    candidate: dict[str, object],
    feedback: str,
    *,
    yaw_delta_deg: float | None = None,
) -> dict[str, object]:
    """Return the next bounded candidate from simple operator feedback."""
    feedback = feedback.strip().lower()
    out = dict(candidate)
    primitive = str(out["primitive"])
    if feedback in {"retry", "approve"}:
        return clamp_candidate(out)

    target = float(out.get("target_angle_deg", TURN_TARGET_ANGLE_DEG))
    tolerance = float(out.get("angle_tolerance_deg", TURN_ANGLE_TOLERANCE_DEG))
    if primitive in {"turn-left-90", "turn-right-90"} and yaw_delta_deg is not None:
        yaw_abs = abs(float(yaw_delta_deg))
        if yaw_abs < target - tolerance:
            feedback = "weak"
        elif yaw_abs > target + tolerance:
            feedback = "strong"
        else:
            return clamp_candidate(out)
    elif feedback == "good":
        return clamp_candidate(out)

    if primitive == "forward":
        if feedback in {"weak", "too_short", "none"}:
            out["ms"] = int(out["ms"]) + 100
            if int(out["ms"]) > MAX_MS:
                out["a"] = float(out["a"]) + 0.02
        elif feedback in {"strong", "too_long"}:
            out["ms"] = int(out["ms"]) - 100
        elif feedback == "left":
            out["b"] = float(out["b"]) - 0.02
        elif feedback == "right":
            out["b"] = float(out["b"]) + 0.02
    elif primitive == "backward":
        if feedback in {"weak", "too_short", "none"}:
            out["ms"] = int(out["ms"]) + 50
            if int(out["ms"]) > MAX_MS:
                out["a"] = float(out["a"]) - 0.01
        elif feedback in {"strong", "too_long"}:
            out["ms"] = int(out["ms"]) - 50
        elif feedback == "left":
            out["b"] = float(out["b"]) + 0.02
        elif feedback == "right":
            out["b"] = float(out["b"]) - 0.02
    elif primitive in {"left", "right", "turn-left-90", "turn-right-90"}:
        sign = 1.0 if primitive in {"left", "turn-left-90"} else -1.0
        if feedback in {"weak", "too_short", "none"}:
            out["ms"] = int(out["ms"]) + (150 if "90" in primitive else 100)
            if int(out["ms"]) > MAX_MS:
                out["b"] = float(out["b"]) + sign * 0.02
        elif feedback in {"strong", "too_long"}:
            out["ms"] = int(out["ms"]) - (150 if "90" in primitive else 100)
    return clamp_candidate(out)


def approved_calibration_entry(
    candidate: dict[str, object],
    *,
    yaw_delta_deg: float | None = None,
    heading_drift_deg: float | None = None,
) -> dict[str, object]:
    primitive = str(candidate["primitive"])
    entry: dict[str, object] = {
        "a": round(float(candidate["a"]), 3),
        "b": round(float(candidate["b"]), 3),
        "ms": int(candidate["ms"]),
        "approved_by_user": True,
        "source": "interactive_visual_tuning",
    }
    if primitive in {"left", "right"}:
        entry["last_imu_yaw_delta_deg"] = yaw_delta_deg
    if primitive in {"forward", "backward"}:
        entry["heading_drift_deg"] = heading_drift_deg
    if primitive in {"turn-left-90", "turn-right-90"}:
        entry["target_angle_deg"] = float(candidate.get("target_angle_deg", TURN_TARGET_ANGLE_DEG))
        entry["last_imu_yaw_delta_deg"] = yaw_delta_deg
    return entry


def opposite_sign_transient(primitive: str, rows: Sequence[dict[str, str]]) -> bool:
    """Detect a motor-trace sign reversal for forward/backward tuning trials."""
    normalized = normalize_primitive(primitive)
    if normalized not in {"forward", "backward"}:
        return False
    expected_sign = 1.0 if normalized == "forward" else -1.0
    for row in rows:
        if telemetry._parse_bool(row.get("motor_write_called")) is not True:
            continue
        physical_a = telemetry._optional_float(row.get("physical_a_cmd"))
        if physical_a is not None and physical_a * expected_sign < -1e-4:
            return True
        final_left = telemetry._optional_float(row.get("final_left_cmd"))
        final_right = telemetry._optional_float(row.get("final_right_cmd"))
        if final_left is not None and final_right is not None:
            inferred_a = (final_left + final_right) * 0.5
            if inferred_a * expected_sign < -1e-4:
                return True
    return False


def save_approved_calibration(
    path: Path,
    candidate: dict[str, object],
    *,
    yaw_delta_deg: float | None = None,
    heading_drift_deg: float | None = None,
) -> dict[str, object]:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        loaded = json.loads(path.read_text(encoding="utf-8"))
        data = loaded if isinstance(loaded, dict) else {}
    else:
        data = {}
    primitive = str(candidate["primitive"])
    data[CALIBRATION_KEYS[primitive]] = approved_calibration_entry(
        candidate,
        yaw_delta_deg=yaw_delta_deg,
        heading_drift_deg=heading_drift_deg,
    )
    data["ready_for_full_path_following"] = False
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return data


def motion_calibration_path(path: str | None = None) -> Path:
    return Path(path) if path else calibration.DEFAULT_MOTION_CALIBRATION


def backup_calibration(path: Path, *, timestamp: str | None = None) -> Path | None:
    """Copy an existing calibration JSON to a timestamped sibling.

    Returns the backup path, or ``None`` when there is nothing to back up. The
    timestamp is injectable so callers (and tests) get deterministic names.
    """
    if not path.exists():
        return None
    stamp = timestamp or time.strftime("%Y%m%d_%H%M%S")
    backup_path = path.with_name(f"{path.stem}.backup_{stamp}{path.suffix}")
    backup_path.write_text(path.read_text(encoding="utf-8"), encoding="utf-8")
    return backup_path


def reset_calibration(path: Path, *, timestamp: str | None = None) -> tuple[Path | None, bool]:
    """Back up then delete the calibration file before a full recalibration.

    Returns ``(backup_path, removed)``: ``backup_path`` is ``None`` when no prior
    file existed, ``removed`` is ``True`` only when an existing file was deleted.
    Approved ``tune-motion`` values written afterward start from a clean slate
    while the previous calibration is preserved as a timestamped backup.
    """
    backup_path = backup_calibration(path, timestamp=timestamp)
    removed = False
    if path.exists():
        path.unlink()
        removed = True
    return backup_path, removed
