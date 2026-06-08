from __future__ import annotations

import argparse
import csv
import json
import math
import time
from pathlib import Path
from typing import Sequence

try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401

from legacy.stage_tools import stage26_calibrated_primitive_sequence as stage26
from legacy.stage_tools import check_stage26_calibrated_primitive_sequence
from tools import station_path_package_tracker
from tools.path_no_motion_validation import PathPackageResolutionError, resolve_path_package


DEFAULT_FORWARD_A = 0.30
DEFAULT_BACKWARD_A = -0.08
DEFAULT_LEFT_B = 0.26
DEFAULT_RIGHT_B = -0.08
DEFAULT_CORRECTION_PULSE_MS = 250
DEFAULT_MAX_CORRECTION_PULSE_MS = 500
DEFAULT_MAX_STAGE20_MS = 1000
DEFAULT_MAX_ABS_A = 0.35
DEFAULT_MAX_ABS_B = 0.35
DEFAULT_MAX_CORRECTION_B = 0.12
MOTION_REPORTS = {"forward", "backward", "left", "right", "twitch", "none", "unknown"}
CORRECTION_MODES = {"correction_forward_segment", "correction_backward_segment", "straight_forward_segment", "straight_backward_segment"}
CALIBRATED_STRAIGHT_MODES = {"calibrated_forward_once", "calibrated_backward_once"}
CALIBRATED_TURN_MODES = {"calibrated_turn_left_once", "calibrated_turn_right_once"}

STAGE28_FIELDS = (
    "pulse_index",
    "mode",
    "segment_index",
    "current_lat",
    "current_lon",
    "current_x_m",
    "current_y_m",
    "imu_relative_yaw_deg",
    "current_heading_deg",
    "target_heading_deg",
    "heading_error_deg",
    "cross_track_error_m",
    "along_track_progress_m",
    "a_cmd",
    "b_cmd",
    "pulse_ms",
    "b_heading_component",
    "b_cte_component",
    "b_cmd_clamped",
    "arm_seen",
    "ack_seen",
    "stop_seen",
    "reject_seen",
    "rc_invalid_seen",
    "final_left_cmd",
    "final_right_cmd",
    "final_left_cmd_zero",
    "final_right_cmd_zero",
    "physical_output_active_after_stop",
    "user_motion_report",
    "valid_pulse",
    "invalid_reason",
    "ready_for_full_path_following",
)


def _parse_bool(value: object, default: bool = False) -> bool:
    if value is None:
        return default
    return str(value).strip().lower() in {"1", "true", "yes", "y", "ok", "active"}


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


def _latest(rows: Sequence[dict[str, str]], key: str, default: str = "NA") -> str:
    for row in reversed(rows):
        if key in row:
            return row[key]
    return default


def _latest_reject_reason(rows: Sequence[dict[str, str]]) -> str:
    reason = _latest(rows, "stage20_reject_reason", "")
    if reason:
        return reason
    return _latest(rows, "stage16_reject_reason", "NONE")


def wrap_deg(angle_deg: float) -> float:
    return ((angle_deg + 180.0) % 360.0) - 180.0


def unwrap_delta_deg(current_deg: float, zero_deg: float) -> float:
    return wrap_deg(current_deg - zero_deg)


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def segment_heading_deg(start_x: float, start_y: float, end_x: float, end_y: float) -> float:
    return math.degrees(math.atan2(end_y - start_y, end_x - start_x))


def signed_cross_track_error(
    *,
    start_x: float,
    start_y: float,
    end_x: float,
    end_y: float,
    point_x: float,
    point_y: float,
) -> float:
    dx = end_x - start_x
    dy = end_y - start_y
    length = math.hypot(dx, dy)
    if length <= 1e-12:
        return 0.0
    return (dx * (point_y - start_y) - dy * (point_x - start_x)) / length


def along_track_progress(
    *,
    start_x: float,
    start_y: float,
    end_x: float,
    end_y: float,
    point_x: float,
    point_y: float,
) -> float:
    dx = end_x - start_x
    dy = end_y - start_y
    length = math.hypot(dx, dy)
    if length <= 1e-12:
        return 0.0
    return ((point_x - start_x) * dx + (point_y - start_y) * dy) / length


def closed_loop_b_command(
    *,
    heading_error_deg: float,
    cross_track_error_m: float,
    k_heading: float = 0.006,
    k_cte: float = 0.25,
    max_correction_b: float = DEFAULT_MAX_CORRECTION_B,
    backward: bool = False,
    reverse_turn_correction_for_backward: bool = True,
) -> tuple[float, float, float]:
    heading_component = k_heading * heading_error_deg
    cte_component = k_cte * cross_track_error_m
    raw = heading_component + cte_component
    if backward and reverse_turn_correction_for_backward:
        raw *= -1.0
        heading_component *= -1.0
        cte_component *= -1.0
    return clamp(raw, max_correction_b), heading_component, cte_component


def load_calibrated_commands(
    fine_calibration: dict[str, object] | None,
    turn_calibration: dict[str, object] | None,
) -> dict[str, float | int]:
    primitive_map = stage26.build_primitive_map(fine_calibration or {}, turn_calibration or {})
    return {
        "forward_base_a": float(primitive_map["forward"]["a_cmd"]),
        "forward_ms": int(primitive_map["forward"]["pulse_ms"]),
        "backward_base_a": float(primitive_map["backward"]["a_cmd"]),
        "backward_ms": int(primitive_map["backward"]["pulse_ms"]),
        "left_b": float(primitive_map["turn_left"]["b_cmd"]),
        "left_ms": int(primitive_map["turn_left"]["pulse_ms"]),
        "right_b": float(primitive_map["turn_right"]["b_cmd"]),
        "right_ms": int(primitive_map["turn_right"]["pulse_ms"]),
    }


def calibrated_pulse_for_mode(mode: str, commands: dict[str, float | int]) -> tuple[float, float, int]:
    if mode == "calibrated_forward_once":
        return float(commands["forward_base_a"]), 0.0, int(commands["forward_ms"])
    if mode == "calibrated_backward_once":
        return float(commands["backward_base_a"]), 0.0, int(commands["backward_ms"])
    if mode == "calibrated_turn_left_once":
        return 0.0, float(commands["left_b"]), int(commands["left_ms"])
    if mode == "calibrated_turn_right_once":
        return 0.0, float(commands["right_b"]), int(commands["right_ms"])
    raise ValueError(f"mode does not use a calibrated open-loop pulse: {mode}")


def mode_requires_gps_local_pose(mode: str) -> bool:
    return mode in {"correction_forward_segment", "correction_backward_segment", "straight_forward_segment", "straight_backward_segment"}


def mode_requires_imu_ready(mode: str) -> bool:
    return mode in {
        "correction_forward_segment",
        "correction_backward_segment",
        "straight_forward_segment",
        "straight_backward_segment",
        "turn_to_heading",
    }


def effective_pulse_ms(args: argparse.Namespace, mode: str, commands: dict[str, float | int]) -> int:
    if mode in CALIBRATED_STRAIGHT_MODES or mode in CALIBRATED_TURN_MODES:
        _a, _b, pulse_ms = calibrated_pulse_for_mode(mode, commands)
        return pulse_ms
    if args.pulse_ms is None:
        return DEFAULT_CORRECTION_PULSE_MS
    return int(args.pulse_ms)


def gps_local_pose_required_for_args(args: argparse.Namespace) -> bool:
    if args.mode not in CORRECTION_MODES:
        return False
    if str(args.closed_loop_mode).upper() == "IMU_ONLY_HEADING_HOLD":
        return False
    if _parse_bool(args.allow_imu_only, default=False):
        return False
    return True


def _summary_path(path_text: str | None, filename: str) -> Path | None:
    if not path_text:
        return None
    path = Path(path_text)
    return path / filename if path.is_dir() else path


def _load_json_summary(path_text: str | None, filename: str) -> dict[str, object] | None:
    path = _summary_path(path_text, filename)
    if path is None or not path.exists():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def recovery_gate_reasons(args: argparse.Namespace) -> list[str]:
    if args.mode == "dry_run":
        return []
    if _parse_bool(getattr(args, "override_recovery_gates", "false"), default=False):
        return []
    reasons: list[str] = []
    manual = _load_json_summary(getattr(args, "manual_rc_summary", None), "manual_rc_passthrough_summary.json")
    if manual is None or manual.get("manual_rc_passthrough_pass") is not True:
        reasons.append("BLOCKED_MANUAL_RC")
    stage26_summary = getattr(args, "stage26_summary", None)
    if not stage26_summary:
        reasons.append("BLOCKED_STAGE26_VALIDATION_MISSING")
    else:
        try:
            result = check_stage26_calibrated_primitive_sequence.evaluate_path(Path(stage26_summary))
            if result.get("verdict") != "PASS":
                reasons.append("BLOCKED_STAGE26_VALIDATION_FAILED")
        except (FileNotFoundError, OSError, ValueError):
            reasons.append("BLOCKED_STAGE26_VALIDATION_MISSING")
    if mode_requires_imu_ready(args.mode):
        imu = _load_json_summary(getattr(args, "imu_readiness_summary", None), "imu_readiness_summary.json")
        if imu is None or imu.get("imu_status") != "IMU_READY":
            reasons.append("BLOCKED_IMU_NOT_READY")
    if gps_local_pose_required_for_args(args):
        gps = _load_json_summary(getattr(args, "gps_watch_summary", None), "gps_link_and_fix_watch_summary.json")
        if gps is None:
            reasons.append("BLOCKED_GPS_WATCH_MISSING")
        elif gps.get("gps_status") != "GPS_READY_FULL":
            reasons.append(f"BLOCKED_{gps.get('gps_status', 'GPS_NOT_READY')}")
    return reasons


def straight_segments(package: dict[str, object]) -> list[dict[str, object]]:
    segments: list[dict[str, object]] = []
    for primitive in package.get("primitive_sequence", []):
        if not isinstance(primitive, dict):
            continue
        if str(primitive.get("primitive_type", "")).startswith("move"):
            segments.append(primitive)
    return segments


def select_segment(package: dict[str, object], segment_index: int) -> dict[str, object]:
    segments = straight_segments(package)
    if not segments:
        raise ValueError("path package contains no straight move segments")
    if segment_index < 0 or segment_index >= len(segments):
        raise ValueError(f"segment_index {segment_index} outside 0..{len(segments) - 1}")
    return segments[segment_index]


def row_pose_ready(row: dict[str, str]) -> bool:
    return (
        _parse_bool(row.get("stage20_physical_ab_guarded_crawl")) is True
        and _parse_bool(row.get("stage20_firmware_ready")) is True
        and _parse_bool(row.get("rc_ok")) is True
        and _parse_bool(row.get("neutral_ok")) is True
        and _parse_bool(row.get("physical_output_active")) is False
        and row.get("imu_type") == "BMI160"
        and _parse_bool(row.get("imu_present")) is True
        and _parse_bool(row.get("imu_data_plausible")) is True
    )


def row_imu_ready_for_closed_loop(row: dict[str, str]) -> bool:
    return row_pose_ready(row) and _parse_bool(row.get("imu_calibrated")) is True


def local_pose_from_usb_row(package: dict[str, object], row: dict[str, str]) -> tuple[bool, str, float, float, float | None, float | None, float | None]:
    lat = _optional_float(row.get("current_lat", row.get("lat")))
    lon = _optional_float(row.get("current_lon", row.get("lon")))
    if lat is not None and lon is not None:
        local = station_path_package_tracker.lat_lon_to_local(package, lat, lon)
        if local is None:
            return False, "NO_GEOREFERENCE_FOR_LAT_LON_TO_LOCAL", 0.0, 0.0, lat, lon, _optional_float(row.get("imu_relative_yaw_deg"))
        return True, "OK", local[0], local[1], lat, lon, _optional_float(row.get("imu_relative_yaw_deg"))
    local_available, reason, _source, x, y, _heading = station_path_package_tracker.local_pose_from_row(package, row)
    return local_available, reason, x, y, lat, lon, _optional_float(row.get("imu_relative_yaw_deg"))


def compute_control_snapshot(
    *,
    package: dict[str, object],
    row: dict[str, str],
    segment: dict[str, object],
    mode: str,
    segment_index: int,
    pulse_index: int,
    imu_yaw_zero_deg: float,
    initial_path_heading_deg: float,
    commands: dict[str, float | int],
    pulse_ms: int,
    k_heading: float = 0.006,
    k_cte: float = 0.25,
    max_correction_b: float = DEFAULT_MAX_CORRECTION_B,
    reverse_turn_correction_for_backward: bool = True,
    require_local_pose: bool = True,
) -> dict[str, object]:
    local_available, reason, x, y, lat, lon, imu_yaw = local_pose_from_usb_row(package, row)
    if require_local_pose and not local_available:
        raise ValueError(reason)
    if imu_yaw is None:
        raise ValueError("IMU_YAW_UNAVAILABLE")
    sx = float(segment["start_x_m"])
    sy = float(segment["start_y_m"])
    ex = float(segment["end_x_m"])
    ey = float(segment["end_y_m"])
    target_heading = segment_heading_deg(sx, sy, ex, ey)
    current_heading = wrap_deg(initial_path_heading_deg + unwrap_delta_deg(imu_yaw, imu_yaw_zero_deg))
    heading_error = wrap_deg(target_heading - current_heading)
    if local_available:
        cte = signed_cross_track_error(start_x=sx, start_y=sy, end_x=ex, end_y=ey, point_x=x, point_y=y)
        along: float | str = along_track_progress(start_x=sx, start_y=sy, end_x=ex, end_y=ey, point_x=x, point_y=y)
        current_x: float | str = x
        current_y: float | str = y
    else:
        cte = 0.0
        along = ""
        current_x = ""
        current_y = ""
    backward = mode in {"correction_backward_segment", "straight_backward_segment"}
    b_cmd, b_heading, b_cte = closed_loop_b_command(
        heading_error_deg=heading_error,
        cross_track_error_m=cte,
        k_heading=k_heading,
        k_cte=k_cte,
        max_correction_b=max_correction_b,
        backward=backward,
        reverse_turn_correction_for_backward=reverse_turn_correction_for_backward,
    )
    a_cmd = float(commands["backward_base_a"] if backward else commands["forward_base_a"])
    return {
        "pulse_index": pulse_index,
        "mode": mode,
        "segment_index": segment_index,
        "current_lat": "" if lat is None else lat,
        "current_lon": "" if lon is None else lon,
        "current_x_m": current_x,
        "current_y_m": current_y,
        "imu_relative_yaw_deg": imu_yaw,
        "current_heading_deg": current_heading,
        "target_heading_deg": target_heading,
        "heading_error_deg": heading_error,
        "cross_track_error_m": cte,
        "along_track_progress_m": along,
        "a_cmd": a_cmd,
        "b_cmd": b_cmd,
        "pulse_ms": pulse_ms,
        "b_heading_component": b_heading,
        "b_cte_component": b_cte,
        "b_cmd_clamped": abs(b_heading + b_cte) > abs(b_cmd) + 1e-12,
        "arm_command_text": f"STAGE20_ARM seq={pulse_index}",
        "stage20_command_text": f"STAGE20_CMD seq={pulse_index} a={a_cmd:.3f} b={b_cmd:.3f} ms={pulse_ms}",
        "stop_command_text": f"STAGE20_STOP seq={pulse_index}",
        "ready_for_full_path_following": False,
    }


def compute_turn_snapshot(
    *,
    row: dict[str, str],
    mode: str,
    pulse_index: int,
    direction: str,
    start_yaw_deg: float,
    target_angle_deg: float,
    commands: dict[str, float | int],
    pulse_ms: int,
) -> dict[str, object]:
    imu_yaw = _optional_float(row.get("imu_relative_yaw_deg"))
    if imu_yaw is None:
        raise ValueError("IMU_YAW_UNAVAILABLE")
    yaw_delta = unwrap_delta_deg(imu_yaw, start_yaw_deg)
    b_cmd = float(commands["left_b"] if direction == "left" else commands["right_b"])
    return {
        "pulse_index": pulse_index,
        "mode": mode,
        "segment_index": -1,
        "current_lat": row.get("current_lat", ""),
        "current_lon": row.get("current_lon", ""),
        "current_x_m": "",
        "current_y_m": "",
        "imu_relative_yaw_deg": imu_yaw,
        "current_heading_deg": yaw_delta,
        "target_heading_deg": target_angle_deg if direction == "left" else -target_angle_deg,
        "heading_error_deg": wrap_deg((target_angle_deg if direction == "left" else -target_angle_deg) - yaw_delta),
        "cross_track_error_m": "",
        "along_track_progress_m": abs(yaw_delta),
        "a_cmd": 0.0,
        "b_cmd": b_cmd,
        "pulse_ms": pulse_ms,
        "b_heading_component": b_cmd,
        "b_cte_component": 0.0,
        "b_cmd_clamped": False,
        "arm_command_text": f"STAGE20_ARM seq={pulse_index}",
        "stage20_command_text": f"STAGE20_CMD seq={pulse_index} a=0.000 b={b_cmd:.3f} ms={pulse_ms}",
        "stop_command_text": f"STAGE20_STOP seq={pulse_index}",
        "ready_for_full_path_following": False,
    }


def compute_calibrated_snapshot(
    *,
    row: dict[str, str],
    mode: str,
    pulse_index: int,
    commands: dict[str, float | int],
    package: dict[str, object] | None = None,
) -> dict[str, object]:
    a_cmd, b_cmd, pulse_ms = calibrated_pulse_for_mode(mode, commands)
    local_available = False
    reason = "NO_LOCAL_POSE_IN_ROW"
    x: float | str = ""
    y: float | str = ""
    lat: float | str = row.get("current_lat", "")
    lon: float | str = row.get("current_lon", "")
    imu_yaw = _optional_float(row.get("imu_relative_yaw_deg"))
    if package is not None:
        local_available, reason, local_x, local_y, parsed_lat, parsed_lon, _imu_yaw = local_pose_from_usb_row(package, row)
        if local_available:
            x = local_x
            y = local_y
        if parsed_lat is not None:
            lat = parsed_lat
        if parsed_lon is not None:
            lon = parsed_lon
    return {
        "pulse_index": pulse_index,
        "mode": mode,
        "segment_index": -1,
        "current_lat": lat,
        "current_lon": lon,
        "current_x_m": x,
        "current_y_m": y,
        "imu_relative_yaw_deg": "" if imu_yaw is None else imu_yaw,
        "current_heading_deg": "",
        "target_heading_deg": "",
        "heading_error_deg": "",
        "cross_track_error_m": "" if not local_available else 0.0,
        "along_track_progress_m": "",
        "a_cmd": a_cmd,
        "b_cmd": b_cmd,
        "pulse_ms": pulse_ms,
        "b_heading_component": 0.0,
        "b_cte_component": 0.0,
        "b_cmd_clamped": False,
        "local_pose_available": local_available,
        "local_pose_reason": reason,
        "arm_command_text": f"STAGE20_ARM seq={pulse_index}",
        "stage20_command_text": f"STAGE20_CMD seq={pulse_index} a={a_cmd:.3f} b={b_cmd:.3f} ms={pulse_ms}",
        "stop_command_text": f"STAGE20_STOP seq={pulse_index}",
        "ready_for_full_path_following": False,
    }


def _serial_rows(raw_lines: Sequence[str], start_index: int = 0) -> list[dict[str, str]]:
    return station_path_package_tracker.parse_usbdbg_rows("\n".join(raw_lines[start_index:]))


def _wait_for_row(handle: object, raw_lines: list[str], predicate, timeout_s: float) -> dict[str, str] | None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        raw = handle.readline()
        if raw:
            line = raw.decode("utf-8", errors="replace").strip()
            print(line)
            raw_lines.append(line)
            row = _serial_rows([line])[0] if _serial_rows([line]) else {}
            if row and predicate(row):
                return row
    return None


def _wait_for_event(handle: object, raw_lines: list[str], wanted: set[str], timeout_s: float) -> list[dict[str, str]]:
    deadline = time.monotonic() + timeout_s
    start_index = len(raw_lines)
    while time.monotonic() < deadline:
        raw = handle.readline()
        if raw:
            line = raw.decode("utf-8", errors="replace").strip()
            print(line)
            raw_lines.append(line)
            rows = _serial_rows(raw_lines, start_index)
            if any(row.get("event", "").upper() in wanted for row in rows):
                return station_path_package_tracker.parse_usbdbg_rows("\n".join(raw_lines))
    return station_path_package_tracker.parse_usbdbg_rows("\n".join(raw_lines))


def row_from_pulse(snapshot: dict[str, object], rows: Sequence[dict[str, str]], user_motion_report: str) -> dict[str, object]:
    final_left = _optional_float(_latest(rows, "final_left_cmd", "0")) or 0.0
    final_right = _optional_float(_latest(rows, "final_right_cmd", "0")) or 0.0
    reject_reason = _latest_reject_reason(rows)
    reject_seen = any(row.get("event", "").upper() == "REJECT" for row in rows)
    output_after_stop = any(row.get("event", "").upper() == "STOP" and _parse_bool(row.get("physical_output_active")) for row in rows)
    final_left_zero = abs(final_left) <= 1e-9
    final_right_zero = abs(final_right) <= 1e-9
    invalid_reason = "NONE"
    if reject_seen:
        invalid_reason = reject_reason if reject_reason != "NONE" else "REJECT"
    elif reject_reason == "RC_INVALID":
        invalid_reason = "RC_INVALID"
    elif not any(row.get("event", "").upper() == "ACK" for row in rows):
        invalid_reason = "ACK_MISSING"
    elif not any(row.get("event", "").upper() in {"STOP", "PULSE_COMPLETE", "PULSE_DONE"} for row in rows):
        invalid_reason = "STOP_MISSING"
    elif output_after_stop:
        invalid_reason = "OUTPUT_ACTIVE_AFTER_STOP"
    elif not final_left_zero or not final_right_zero:
        invalid_reason = "FINAL_COMMANDS_NONZERO"
    return {
        **snapshot,
        "arm_seen": any(row.get("event", "").upper() == "ARM" for row in rows),
        "ack_seen": any(row.get("event", "").upper() == "ACK" for row in rows),
        "stop_seen": any(row.get("event", "").upper() in {"STOP", "PULSE_COMPLETE", "PULSE_DONE"} for row in rows),
        "reject_seen": reject_seen,
        "rc_invalid_seen": reject_reason == "RC_INVALID",
        "final_left_cmd": f"{final_left:.3f}",
        "final_right_cmd": f"{final_right:.3f}",
        "final_left_cmd_zero": final_left_zero,
        "final_right_cmd_zero": final_right_zero,
        "physical_output_active_after_stop": output_after_stop,
        "user_motion_report": user_motion_report,
        "valid_pulse": invalid_reason == "NONE",
        "invalid_reason": invalid_reason,
        "ready_for_full_path_following": False,
    }


def _prompt_choice(prompt: str, choices: set[str]) -> str:
    while True:
        answer = input(prompt).strip().lower()
        if answer in choices:
            return answer
        print("Use one of: " + ", ".join(sorted(choices)))


def _write_csv(path: Path, rows: Sequence[dict[str, object]], fields: Sequence[str]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in fields})


def build_summary(rows: Sequence[dict[str, object]], *, mode: str) -> dict[str, object]:
    valid_rows = [row for row in rows if row.get("valid_pulse") is True]
    reject_count = sum(1 for row in rows if row.get("reject_seen") is True)
    rc_invalid_count = sum(1 for row in rows if row.get("rc_invalid_seen") is True)
    heading_errors = [abs(float(row["heading_error_deg"])) for row in valid_rows if row.get("heading_error_deg") not in {"", "NA"}]
    ctes = [abs(float(row["cross_track_error_m"])) for row in valid_rows if row.get("cross_track_error_m") not in {"", "NA"}]
    start_x = rows[0].get("current_x_m", "") if rows else ""
    start_y = rows[0].get("current_y_m", "") if rows else ""
    end_x = rows[-1].get("current_x_m", "") if rows else ""
    end_y = rows[-1].get("current_y_m", "") if rows else ""
    net_progress = ""
    if rows and rows[0].get("along_track_progress_m") not in {"", "NA"} and rows[-1].get("along_track_progress_m") not in {"", "NA"}:
        net_progress = float(rows[-1]["along_track_progress_m"]) - float(rows[0]["along_track_progress_m"])
    final = rows[-1] if rows else {}
    return {
        "mode": mode,
        "pulse_count": len(rows),
        "valid_pulse_count": len(valid_rows),
        "reject_count": reject_count,
        "rc_invalid_count": rc_invalid_count,
        "final_stop_confirmed": bool(rows) and final.get("stop_seen") is True and final.get("final_left_cmd_zero") is True and final.get("final_right_cmd_zero") is True,
        "average_abs_heading_error_deg": "" if not heading_errors else sum(heading_errors) / len(heading_errors),
        "average_abs_cross_track_error_m": "" if not ctes else sum(ctes) / len(ctes),
        "start_x_m": start_x,
        "start_y_m": start_y,
        "end_x_m": end_x,
        "end_y_m": end_y,
        "net_progress_m": net_progress,
        "ready_for_stage29_one_block": bool(valid_rows) and len(valid_rows) == len(rows) and reject_count == 0,
        "ready_for_full_path_following": False,
    }


def _write_summary(path: Path, summary: dict[str, object]) -> None:
    lines = [
        "# Stage 28 Closed-Loop Segment",
        "",
        "USB-supervised bounded Stage20 pulses only. Not full autonomous path following.",
        "",
    ]
    lines.extend(f"- {key}: `{value}`" for key, value in summary.items())
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run_stage28(args: argparse.Namespace) -> int:
    selected = resolve_path_package(args.path_package)
    package = json.loads(selected.read_text(encoding="utf-8"))
    fine = json.loads(Path(args.fine_calibration_json).read_text(encoding="utf-8"))
    turn = json.loads(Path(args.turn_calibration_json).read_text(encoding="utf-8"))
    commands = load_calibrated_commands(fine, turn)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    debug = {
        "selected_path_package": str(selected),
        "mode": args.mode,
        "commands": commands,
        "effective_pulse_ms": effective_pulse_ms(args, args.mode, commands),
        "k_heading": args.k_heading,
        "k_cte": args.k_cte,
        "max_correction_b": args.max_correction_b,
        "ready_for_full_path_following": False,
    }
    (out_dir / "controller_debug.json").write_text(json.dumps(debug, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    if args.mode == "dry_run":
        summary = build_summary([], mode=args.mode)
        _write_summary(out_dir / "summary.md", summary)
        _write_csv(out_dir / "stage28_closed_loop_segment.csv", [], STAGE28_FIELDS)
        _write_csv(out_dir / "path_trace.csv", [], ("pulse_index", "current_x_m", "current_y_m", "along_track_progress_m"))
        print("Stage28 dry run complete.")
        print(f"controller_debug_json={out_dir / 'controller_debug.json'}")
        print("ready_for_full_path_following=false")
        return 0

    gate_reasons = recovery_gate_reasons(args)
    if gate_reasons:
        raise RuntimeError("RECOVERY_GATES_NOT_PASSED:" + ",".join(gate_reasons))

    try:
        import serial  # type: ignore[import-not-found]
    except ImportError as exc:
        raise RuntimeError("pyserial is not available; cannot run Stage28 closed loop") from exc

    selected_pulse_ms = effective_pulse_ms(args, args.mode, commands)
    if args.mode in CORRECTION_MODES and selected_pulse_ms > DEFAULT_MAX_CORRECTION_PULSE_MS:
        raise SystemExit("--pulse-ms must be <= 500 for Stage28 correction modes")
    if selected_pulse_ms > DEFAULT_MAX_STAGE20_MS:
        raise SystemExit("--pulse-ms/calibrated pulse must be <= 1000 for Stage28")

    rows: list[dict[str, object]] = []
    raw_lines: list[str] = []
    require_enter = _parse_bool(args.require_enter, default=True)
    require_each = _parse_bool(args.require_enter_each_pulse, default=True)
    interactive = _parse_bool(args.interactive_visible_motion, default=True)

    with serial.Serial(args.port, baudrate=115200, timeout=0.5) as handle:
        first_timeout = max(args.heartbeat_timeout_s, args.imu_calibration_timeout_s if mode_requires_imu_ready(args.mode) else 0.0)
        first_predicate = row_imu_ready_for_closed_loop if mode_requires_imu_ready(args.mode) else row_pose_ready
        first_row = _wait_for_row(handle, raw_lines, first_predicate, first_timeout)
        if first_row is None:
            raise RuntimeError("Stage28 GPS/IMU/RC precondition failed")
        first_local, reason, _x, _y, _lat, _lon, first_yaw = local_pose_from_usb_row(package, first_row)
        if gps_local_pose_required_for_args(args) and not first_local:
            raise RuntimeError(reason)
        if mode_requires_imu_ready(args.mode):
            if first_yaw is None:
                raise RuntimeError("IMU_YAW_UNAVAILABLE")
            if not row_imu_ready_for_closed_loop(first_row):
                raise RuntimeError(first_row.get("imu_heading_block_reason", "IMU_NOT_READY"))
        if require_enter:
            input(f"Press Enter to start Stage28 mode={args.mode}. ")

        if args.mode in CALIBRATED_STRAIGHT_MODES or args.mode in CALIBRATED_TURN_MODES:
            for pulse_index in range(1, args.max_pulses + 1):
                if pulse_index > 1:
                    break
                pulse_start = len(raw_lines)
                pose_row = _wait_for_row(handle, raw_lines, row_pose_ready, args.heartbeat_timeout_s)
                if pose_row is None:
                    raise RuntimeError("Stage28 precondition row unavailable before calibrated pulse")
                snapshot = compute_calibrated_snapshot(
                    row=pose_row,
                    mode=args.mode,
                    pulse_index=pulse_index,
                    commands=commands,
                    package=package,
                )
                if require_each:
                    input(f"Press Enter for Stage28 calibrated pulse {pulse_index}: {snapshot['stage20_command_text']} ")
                handle.write((str(snapshot["arm_command_text"]) + "\n").encode("ascii"))
                handle.flush()
                _wait_for_event(handle, raw_lines, {"ARM", "REJECT"}, args.event_timeout_s)
                handle.write((str(snapshot["stage20_command_text"]) + "\n").encode("ascii"))
                handle.flush()
                _wait_for_event(handle, raw_lines, {"ACK", "REJECT"}, args.event_timeout_s)
                pulse_ms = int(snapshot["pulse_ms"])
                _wait_for_event(handle, raw_lines, {"STOP", "PULSE_COMPLETE", "PULSE_DONE"}, max(args.event_timeout_s, pulse_ms / 1000.0 + 1.0))
                handle.write((str(snapshot["stop_command_text"]) + "\n").encode("ascii"))
                handle.flush()
                _wait_for_event(handle, raw_lines, {"STOP"}, args.event_timeout_s)
                pulse_rows = _serial_rows(raw_lines, pulse_start)
                motion = _prompt_choice("Observed motion? [forward/backward/left/right/twitch/none/unknown] ", MOTION_REPORTS) if interactive else "unknown"
                row = row_from_pulse(snapshot, pulse_rows, motion)
                rows.append(row)
                if row["valid_pulse"] is not True:
                    break
        elif args.mode in CORRECTION_MODES:
            segment = select_segment(package, args.segment_index)
            initial_path_heading = segment_heading_deg(
                float(segment["start_x_m"]),
                float(segment["start_y_m"]),
                float(segment["end_x_m"]),
                float(segment["end_y_m"]),
            )
            for pulse_index in range(1, args.max_pulses + 1):
                pulse_start = len(raw_lines)
                pose_row = _wait_for_row(handle, raw_lines, row_pose_ready, args.heartbeat_timeout_s)
                if pose_row is None:
                    raise RuntimeError("Stage28 pose row unavailable before pulse")
                snapshot = compute_control_snapshot(
                    package=package,
                    row=pose_row,
                    segment=segment,
                    mode=args.mode,
                    segment_index=args.segment_index,
                    pulse_index=pulse_index,
                    imu_yaw_zero_deg=first_yaw,
                    initial_path_heading_deg=initial_path_heading,
                    commands=commands,
                    pulse_ms=selected_pulse_ms,
                    k_heading=args.k_heading,
                    k_cte=args.k_cte,
                    max_correction_b=args.max_correction_b,
                    reverse_turn_correction_for_backward=_parse_bool(args.reverse_turn_correction_for_backward, default=True),
                    require_local_pose=gps_local_pose_required_for_args(args),
                )
                if require_each:
                    input(f"Press Enter for Stage28 pulse {pulse_index}: {snapshot['stage20_command_text']} ")
                handle.write((str(snapshot["arm_command_text"]) + "\n").encode("ascii"))
                handle.flush()
                _wait_for_event(handle, raw_lines, {"ARM", "REJECT"}, args.event_timeout_s)
                handle.write((str(snapshot["stage20_command_text"]) + "\n").encode("ascii"))
                handle.flush()
                _wait_for_event(handle, raw_lines, {"ACK", "REJECT"}, args.event_timeout_s)
                _wait_for_event(handle, raw_lines, {"STOP", "PULSE_COMPLETE", "PULSE_DONE"}, max(args.event_timeout_s, selected_pulse_ms / 1000.0 + 1.0))
                handle.write((str(snapshot["stop_command_text"]) + "\n").encode("ascii"))
                handle.flush()
                _wait_for_event(handle, raw_lines, {"STOP"}, args.event_timeout_s)
                pulse_rows = _serial_rows(raw_lines, pulse_start)
                motion = _prompt_choice("Observed motion? [forward/backward/left/right/twitch/none/unknown] ", MOTION_REPORTS) if interactive else "unknown"
                row = row_from_pulse(snapshot, pulse_rows, motion)
                rows.append(row)
                if row["valid_pulse"] is not True:
                    break
        elif args.mode == "turn_to_heading":
            start_yaw = first_yaw
            for pulse_index in range(1, args.max_pulses + 1):
                pulse_start = len(raw_lines)
                pose_row = _wait_for_row(handle, raw_lines, row_pose_ready, args.heartbeat_timeout_s)
                if pose_row is None:
                    raise RuntimeError("Stage28 pose row unavailable before turn pulse")
                snapshot = compute_turn_snapshot(
                    row=pose_row,
                    mode=args.mode,
                    pulse_index=pulse_index,
                    direction=args.direction,
                    start_yaw_deg=start_yaw,
                    target_angle_deg=args.target_angle_deg,
                    commands=commands,
                    pulse_ms=selected_pulse_ms,
                )
                if abs(float(snapshot["along_track_progress_m"])) >= max(0.0, args.target_angle_deg - args.angle_tolerance_deg):
                    break
                if require_each:
                    input(f"Press Enter for Stage28 turn pulse {pulse_index}: {snapshot['stage20_command_text']} ")
                handle.write((str(snapshot["arm_command_text"]) + "\n").encode("ascii"))
                handle.flush()
                _wait_for_event(handle, raw_lines, {"ARM", "REJECT"}, args.event_timeout_s)
                handle.write((str(snapshot["stage20_command_text"]) + "\n").encode("ascii"))
                handle.flush()
                _wait_for_event(handle, raw_lines, {"ACK", "REJECT"}, args.event_timeout_s)
                _wait_for_event(handle, raw_lines, {"STOP", "PULSE_COMPLETE", "PULSE_DONE"}, max(args.event_timeout_s, selected_pulse_ms / 1000.0 + 1.0))
                handle.write((str(snapshot["stop_command_text"]) + "\n").encode("ascii"))
                handle.flush()
                _wait_for_event(handle, raw_lines, {"STOP"}, args.event_timeout_s)
                pulse_rows = _serial_rows(raw_lines, pulse_start)
                motion = _prompt_choice("Observed motion? [forward/backward/left/right/twitch/none/unknown] ", MOTION_REPORTS) if interactive else "unknown"
                row = row_from_pulse(snapshot, pulse_rows, motion)
                rows.append(row)
                if row["valid_pulse"] is not True:
                    break
        else:
            raise SystemExit("one_block is reserved; run straight/turn modes first")

    _write_csv(out_dir / "stage28_closed_loop_segment.csv", rows, STAGE28_FIELDS)
    trace_fields = ("pulse_index", "current_x_m", "current_y_m", "along_track_progress_m")
    _write_csv(out_dir / "path_trace.csv", rows, trace_fields)
    (out_dir / "raw_usbdbg.log").write_text("\n".join(raw_lines) + ("\n" if raw_lines else ""), encoding="utf-8")
    _write_summary(out_dir / "summary.md", build_summary(rows, mode=args.mode))
    print("Stage28 closed-loop segment complete.")
    print(f"stage28_csv={out_dir / 'stage28_closed_loop_segment.csv'}")
    print(f"raw_usbdbg_log={out_dir / 'raw_usbdbg.log'}")
    print(f"summary_md={out_dir / 'summary.md'}")
    print("ready_for_full_path_following=false")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Stage28 supervised closed-loop segment runner.")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--path-package", default="latest")
    parser.add_argument("--fine-calibration-json", required=True)
    parser.add_argument("--turn-calibration-json", required=True)
    parser.add_argument(
        "--mode",
        choices=(
            "dry_run",
            "calibrated_forward_once",
            "calibrated_backward_once",
            "calibrated_turn_left_once",
            "calibrated_turn_right_once",
            "correction_forward_segment",
            "correction_backward_segment",
            "straight_forward_segment",
            "straight_backward_segment",
            "turn_to_heading",
            "one_block",
        ),
        required=True,
    )
    parser.add_argument("--segment-index", type=int, default=0)
    parser.add_argument("--direction", choices=("left", "right"), default="left")
    parser.add_argument("--target-angle-deg", type=float, default=90.0)
    parser.add_argument("--angle-tolerance-deg", type=float, default=15.0)
    parser.add_argument("--max-pulses", type=int, default=5)
    parser.add_argument("--pulse-ms", type=int, default=None)
    parser.add_argument("--k-heading", type=float, default=0.006)
    parser.add_argument("--k-cte", type=float, default=0.25)
    parser.add_argument("--max-correction-b", type=float, default=DEFAULT_MAX_CORRECTION_B)
    parser.add_argument("--reverse-turn-correction-for-backward", default="true")
    parser.add_argument("--closed-loop-mode", choices=("GPS_IMU_CLOSED_LOOP", "IMU_ONLY_HEADING_HOLD"), default="GPS_IMU_CLOSED_LOOP")
    parser.add_argument("--allow-imu-only", default="false")
    parser.add_argument("--imu-calibration-timeout-s", type=float, default=30.0)
    parser.add_argument("--manual-rc-summary")
    parser.add_argument("--gps-watch-summary")
    parser.add_argument("--imu-readiness-summary")
    parser.add_argument("--stage26-summary")
    parser.add_argument("--override-recovery-gates", default="false")
    parser.add_argument("--require-enter", default="true")
    parser.add_argument("--require-enter-each-pulse", default="true")
    parser.add_argument("--interactive-visible-motion", default="true")
    parser.add_argument("--heartbeat-timeout-s", type=float, default=10.0)
    parser.add_argument("--event-timeout-s", type=float, default=5.0)
    parser.add_argument("--out-dir", required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.max_pulses <= 0:
        raise SystemExit("--max-pulses must be > 0")
    if args.pulse_ms is not None and args.pulse_ms <= 0:
        raise SystemExit("--pulse-ms must be > 0")
    if args.pulse_ms is not None and args.pulse_ms > DEFAULT_MAX_STAGE20_MS:
        raise SystemExit("--pulse-ms must be <= 1000")
    try:
        return run_stage28(args)
    except PathPackageResolutionError as exc:
        print(f"provided_path_package={exc.provided}")
        print("file_exists=false")
        print("nearest_candidates:")
        for candidate in exc.candidates:
            print(f"- {candidate}")
        return 2
    except RuntimeError as exc:
        print(f"reason={exc}")
        print("ready_for_full_path_following=false")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
