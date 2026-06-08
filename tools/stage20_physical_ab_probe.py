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

from tools import station_path_package_tracker


DEFAULT_FORWARD_SIGN = 1.0
DEFAULT_TURN_SIGN = 1.0
DEFAULT_MAX_ABS_A = 0.25
DEFAULT_MAX_ABS_B = 0.25
DEFAULT_MAX_MS = 800
INVALID_REJECT_REASONS = {"LATCHED_STOP", "RC_INVALID"}

PROBE_FIELDS = (
    "trial_index",
    "mode",
    "cmd",
    "pulse_ms",
    "command_axis",
    "axis_limit_used",
    "max_abs_a",
    "max_abs_b",
    "command_within_axis_limit",
    "seq",
    "forward_sign",
    "turn_sign",
    "arm_command_text",
    "stage20_command_text",
    "stop_command_text",
    "requested_a_cmd",
    "requested_b_cmd",
    "physical_a_cmd",
    "physical_b_cmd",
    "manual_equivalent_forward_cmd",
    "manual_equivalent_turn_cmd",
    "final_left_cmd",
    "final_right_cmd",
    "motor_write_called",
    "motor_backend",
    "motor_enable_state",
    "pwm_or_dynamixel_write_status",
    "wheel_to_physical_mapping",
    "firmware_active_target_source",
    "physical_path_following_enable",
    "allow_motor_output",
    "physical_output_active_seen",
    "physical_output_active_after_stop",
    "valid_trial",
    "invalid_reason",
    "rc_ok_before_cmd",
    "neutral_ok_before_cmd",
    "arm_ack_seen",
    "ack_seen",
    "stop_seen",
    "reject_seen",
    "reject_reason",
    "left_wheel_user_report",
    "right_wheel_user_report",
    "body_motion_user_report",
    "turn_angle_report",
    "imu_angle_compare_enabled",
    "yaw_before_deg",
    "yaw_after_deg",
    "yaw_delta_deg",
    "abs_yaw_delta_deg",
    "target_angle_deg",
    "angle_tolerance_deg",
    "error_to_target_deg",
    "visual_angle_confirmation",
    "turn_calibration_saved",
    "turn_calibration_out",
    "motion_class",
    "visible_motion_confirmed",
    "stop_after_big_turn",
    "motor_power_or_mapping_suspect",
    "next_action",
    "ready_for_full_path_following",
)

WHEEL_RESPONSES = {"yes", "no", "twitch", "unknown"}
BODY_RESPONSES = {"none", "forward", "backward", "left", "right", "twitch", "unknown"}
TURN_ANGLE_RESPONSES = {"none", "twitch", "small_under_15deg", "medium_15_45deg", "big_over_45deg", "unknown"}
VISUAL_ANGLE_CONFIRMATIONS = {"yes", "under", "over", "unknown"}
VISIBLE_WHEEL = {"yes", "twitch"}
VISIBLE_BODY = {"forward", "backward", "left", "right", "twitch"}


def _parse_bool(value: object, default: bool = False) -> bool:
    if value is None:
        return default
    return str(value).strip().lower() in {"1", "true", "yes", "y", "ok", "active"}


def _parse_float(value: object, default: float = 0.0) -> float:
    try:
        return float(str(value).strip())
    except (TypeError, ValueError):
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


def _fmt_float(value: float | None) -> str:
    return "NA" if value is None else f"{value:.3f}"


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


def parse_float_list(text: str, *, max_value: float | None = None) -> list[float]:
    values: list[float] = []
    for item in text.split(","):
        stripped = item.strip()
        if not stripped:
            continue
        value = float(stripped)
        if value <= 0 or (max_value is not None and value > max_value):
            if max_value is None:
                raise ValueError(f"value {value} exceeds safe limit")
            raise ValueError(f"value {value} exceeds safe limit {max_value}")
        values.append(value)
    if not values:
        raise ValueError("empty command list")
    return values


def parse_int_list(text: str, *, max_value: int) -> list[int]:
    values: list[int] = []
    for item in text.split(","):
        stripped = item.strip()
        if not stripped:
            continue
        value = int(float(stripped))
        if value <= 0 or value > max_value:
            raise ValueError(f"value {value} exceeds safe limit {max_value}")
        values.append(value)
    if not values:
        raise ValueError("empty pulse list")
    return values


def command_for_probe_mode(
    mode: str,
    cmd: float,
    *,
    forward_sign: float = DEFAULT_FORWARD_SIGN,
    turn_sign: float = DEFAULT_TURN_SIGN,
) -> tuple[float, float]:
    if mode == "forward":
        return forward_sign * cmd, 0.0
    if mode == "backward":
        return -forward_sign * cmd, 0.0
    if mode == "turn_left":
        return 0.0, turn_sign * cmd
    if mode == "turn_right":
        return 0.0, -turn_sign * cmd
    if mode == "a_sweep":
        return forward_sign * cmd, 0.0
    if mode == "b_sweep":
        return 0.0, turn_sign * cmd
    raise ValueError(f"unsupported Stage20 probe mode: {mode}")


def command_axis_for_probe_mode(mode: str) -> str:
    if mode in {"forward", "backward", "a_sweep"}:
        return "A"
    if mode in {"turn_left", "turn_right", "b_sweep"}:
        return "B"
    raise ValueError(f"unsupported Stage20 probe mode: {mode}")


def axis_limit_for_probe_mode(mode: str, *, max_abs_a: float, max_abs_b: float) -> float:
    return max_abs_a if command_axis_for_probe_mode(mode) == "A" else max_abs_b


def command_within_axis_limit(mode: str, cmd: float, *, max_abs_a: float, max_abs_b: float) -> bool:
    return cmd <= axis_limit_for_probe_mode(mode, max_abs_a=max_abs_a, max_abs_b=max_abs_b) + 1e-9


def validate_command_axis_limit(mode: str, cmd: float, *, max_abs_a: float, max_abs_b: float) -> None:
    if command_within_axis_limit(mode, cmd, max_abs_a=max_abs_a, max_abs_b=max_abs_b):
        return
    axis = command_axis_for_probe_mode(mode)
    limit = axis_limit_for_probe_mode(mode, max_abs_a=max_abs_a, max_abs_b=max_abs_b)
    raise ValueError(f"value {cmd} exceeds safe {axis}-axis limit {limit}")


def modes_for_selection(mode: str) -> list[str]:
    if mode == "all":
        return ["forward", "backward", "turn_left", "turn_right", "a_sweep", "b_sweep"]
    return [mode]


def planned_probe_commands(
    *,
    seq: int,
    mode: str,
    cmd: float,
    pulse_ms: int,
    forward_sign: float = DEFAULT_FORWARD_SIGN,
    turn_sign: float = DEFAULT_TURN_SIGN,
    max_abs_a: float = DEFAULT_MAX_ABS_A,
    max_abs_b: float = DEFAULT_MAX_ABS_B,
) -> dict[str, object]:
    validate_command_axis_limit(mode, cmd, max_abs_a=max_abs_a, max_abs_b=max_abs_b)
    a_cmd, b_cmd = command_for_probe_mode(mode, cmd, forward_sign=forward_sign, turn_sign=turn_sign)
    command_axis = command_axis_for_probe_mode(mode)
    axis_limit = axis_limit_for_probe_mode(mode, max_abs_a=max_abs_a, max_abs_b=max_abs_b)
    return {
        "seq": seq,
        "mode": mode,
        "cmd": cmd,
        "pulse_ms": pulse_ms,
        "command_axis": command_axis,
        "axis_limit_used": axis_limit,
        "max_abs_a": max_abs_a,
        "max_abs_b": max_abs_b,
        "command_within_axis_limit": True,
        "forward_sign": forward_sign,
        "turn_sign": turn_sign,
        "requested_a_cmd": a_cmd,
        "requested_b_cmd": b_cmd,
        "arm_command_text": f"STAGE20_ARM seq={seq}",
        "stage20_command_text": f"STAGE20_CMD seq={seq} a={a_cmd:.3f} b={b_cmd:.3f} ms={pulse_ms}",
        "stop_command_text": f"STAGE20_STOP seq={seq}",
    }


def visible_motion(left: str, right: str, body: str) -> bool:
    return left in VISIBLE_WHEEL or right in VISIBLE_WHEEL or body in VISIBLE_BODY


def classify_motion(left: str, right: str, body: str) -> str:
    if body in {"forward", "backward", "left", "right"}:
        return "visible"
    if body == "twitch" or left in VISIBLE_WHEEL or right in VISIBLE_WHEEL:
        return "twitch"
    if body == "unknown" or left == "unknown" or right == "unknown":
        return "unknown"
    return "none"


def is_turn_mode(mode: object) -> bool:
    return str(mode) in {"turn_left", "turn_right", "b_sweep"}


def stop_search_after_user_report(row: dict[str, object]) -> bool:
    return (
        row.get("body_motion_user_report") in {"twitch", "left", "right"}
        or row.get("turn_angle_report") in {"twitch", "small_under_15deg"}
        or row.get("visible_motion_confirmed") is True
    )


def _serial_rows(text: str) -> list[dict[str, str]]:
    return station_path_package_tracker.parse_usbdbg_rows(text)


def _wait_for_event(handle: object, raw_lines: list[str], wanted: set[str], timeout_s: float) -> list[dict[str, str]]:
    deadline = time.monotonic() + timeout_s
    start_index = len(raw_lines)
    while time.monotonic() < deadline:
        raw = handle.readline()
        if raw:
            line = raw.decode("utf-8", errors="replace").strip()
            print(line)
            raw_lines.append(line)
            rows = _serial_rows("\n".join(raw_lines[start_index:]))
            if any(row.get("event", "").upper() in wanted for row in rows):
                return _serial_rows("\n".join(raw_lines))
    return _serial_rows("\n".join(raw_lines))


def _row_is_stable_precondition(row: dict[str, str]) -> bool:
    return (
        row.get("event", "").upper() == "HEARTBEAT"
        and _parse_bool(row.get("rc_ok")) is True
        and _parse_bool(row.get("neutral_ok")) is True
        and _parse_bool(row.get("physical_output_active")) is False
    )


def stable_precondition_met(rows: Sequence[dict[str, str]], *, consecutive: int = 3) -> bool:
    count = 0
    for row in reversed(rows):
        if _row_is_stable_precondition(row):
            count += 1
            if count >= consecutive:
                return True
            continue
        if row.get("event", "").upper() == "HEARTBEAT":
            count = 0
    return False


def latest_stable_heartbeat(rows: Sequence[dict[str, str]]) -> dict[str, str] | None:
    for row in reversed(rows):
        if _row_is_stable_precondition(row):
            return row
    return None


def row_yaw_deg(row: dict[str, str] | None) -> float | None:
    if row is None:
        return None
    return _optional_float(row.get("imu_relative_yaw_deg"))


def unwrap_yaw_delta_deg(yaw_after_deg: float, yaw_before_deg: float) -> float:
    delta = yaw_after_deg - yaw_before_deg
    while delta > 180.0:
        delta -= 360.0
    while delta < -180.0:
        delta += 360.0
    return delta


def angle_comparison_from_yaw(
    *,
    yaw_before_deg: float | None,
    yaw_after_deg: float | None,
    target_angle_deg: float,
    angle_tolerance_deg: float,
) -> dict[str, float | None]:
    yaw_delta = (
        unwrap_yaw_delta_deg(yaw_after_deg, yaw_before_deg)
        if yaw_before_deg is not None and yaw_after_deg is not None
        else None
    )
    abs_delta = abs(yaw_delta) if yaw_delta is not None else None
    error = abs_delta - target_angle_deg if abs_delta is not None else None
    return {
        "yaw_before_deg": yaw_before_deg,
        "yaw_after_deg": yaw_after_deg,
        "yaw_delta_deg": yaw_delta,
        "abs_yaw_delta_deg": abs_delta,
        "target_angle_deg": target_angle_deg,
        "angle_tolerance_deg": angle_tolerance_deg,
        "error_to_target_deg": error,
    }


def angle_comparison_from_rows(
    *,
    before_rows: Sequence[dict[str, str]],
    after_rows: Sequence[dict[str, str]],
    target_angle_deg: float,
    angle_tolerance_deg: float,
) -> dict[str, float | None]:
    return angle_comparison_from_yaw(
        yaw_before_deg=row_yaw_deg(latest_stable_heartbeat(before_rows)),
        yaw_after_deg=row_yaw_deg(latest_stable_heartbeat(after_rows)),
        target_angle_deg=target_angle_deg,
        angle_tolerance_deg=angle_tolerance_deg,
    )


def _print_angle_comparison(angle: dict[str, float | None]) -> None:
    for key in (
        "yaw_before_deg",
        "yaw_after_deg",
        "yaw_delta_deg",
        "abs_yaw_delta_deg",
        "target_angle_deg",
        "error_to_target_deg",
    ):
        print(f"{key}={_fmt_float(angle.get(key))}")


def _wait_for_stable_precondition(
    handle: object,
    raw_lines: list[str],
    *,
    consecutive: int,
    timeout_s: float,
) -> list[dict[str, str]]:
    deadline = time.monotonic() + timeout_s
    start_index = len(raw_lines)
    while time.monotonic() < deadline:
        rows = _serial_rows("\n".join(raw_lines[start_index:]))
        if stable_precondition_met(rows, consecutive=consecutive):
            return rows
        raw = handle.readline()
        if raw:
            line = raw.decode("utf-8", errors="replace").strip()
            print(line)
            raw_lines.append(line)
    return _serial_rows("\n".join(raw_lines[start_index:]))


def _prompt_choice(prompt: str, choices: set[str]) -> str:
    while True:
        response = input(prompt).strip().lower()
        if response in choices:
            return response
        print("Use one of: " + ", ".join(sorted(choices)))


def _calibration_float(value: object) -> float:
    parsed = _optional_float(value)
    return 0.0 if parsed is None else round(parsed, 3)


def turn_calibration_key(mode: object, target_angle_deg: float) -> str | None:
    target = int(round(abs(target_angle_deg)))
    if str(mode) == "turn_left":
        return f"turn_left_{target}"
    if str(mode) == "turn_right":
        return f"turn_right_{target}"
    return None


def successful_turn_calibration_trial(row: dict[str, object]) -> bool:
    return (
        _parse_bool(row.get("ack_seen")) is True
        and _parse_bool(row.get("stop_seen")) is True
        and _parse_bool(row.get("reject_seen")) is False
        and _parse_bool(row.get("physical_output_active_after_stop")) is False
        and _parse_bool(row.get("valid_trial"), default=True) is True
        and abs(_parse_float(row.get("final_left_cmd"), default=999.0)) <= 1e-9
        and abs(_parse_float(row.get("final_right_cmd"), default=999.0)) <= 1e-9
    )


def turn_calibration_entry(
    *,
    planned: dict[str, object],
    angle: dict[str, float | None],
    visual_confirmation: str,
) -> dict[str, object] | None:
    yaw_delta = angle.get("yaw_delta_deg")
    error = angle.get("error_to_target_deg")
    tolerance = angle.get("angle_tolerance_deg")
    if visual_confirmation != "yes" or yaw_delta is None or error is None or tolerance is None:
        return None
    return {
        "a_cmd": _calibration_float(planned.get("requested_a_cmd")),
        "b_cmd": _calibration_float(planned.get("requested_b_cmd")),
        "pulse_ms": int(planned["pulse_ms"]),
        "imu_yaw_delta_deg": round(yaw_delta, 3),
        "visual_confirmation": visual_confirmation,
        "ready": abs(error) <= tolerance + 1e-9,
    }


def save_turn_calibration_result(
    *,
    path: Path,
    planned: dict[str, object],
    row: dict[str, object],
    angle: dict[str, float | None],
    visual_confirmation: str,
) -> bool:
    key = turn_calibration_key(planned.get("mode"), float(angle.get("target_angle_deg") or 90.0))
    entry = turn_calibration_entry(
        planned=planned,
        angle=angle,
        visual_confirmation=visual_confirmation,
    )
    if key is None or entry is None or not successful_turn_calibration_trial(row):
        return False

    if path.exists():
        try:
            existing = json.loads(path.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            existing = {}
        data = existing if isinstance(existing, dict) else {}
    else:
        data = {}
    data[key] = entry
    data["ready_for_angle_based_connectors"] = (
        isinstance(data.get("turn_left_90"), dict)
        and isinstance(data.get("turn_right_90"), dict)
        and bool(data["turn_left_90"].get("ready"))
        and bool(data["turn_right_90"].get("ready"))
    )
    data["ready_for_full_path_following"] = False

    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return True


def _active_rows(rows: Sequence[dict[str, str]]) -> list[dict[str, str]]:
    return [
        row
        for row in rows
        if row.get("event", "").upper() == "ACK"
        or row.get("stage20_cmd_state", "").upper() == "ACTIVE"
        or row.get("stage16_cmd_state", "").upper() == "ACTIVE"
        or _parse_bool(row.get("physical_output_active"))
    ]


def row_from_trial(
    *,
    trial_index: int,
    planned: dict[str, object],
    rows: Sequence[dict[str, str]],
    left_report: str,
    right_report: str,
    body_report: str,
    turn_angle_report: str = "unknown",
    angle_compare_enabled: bool = False,
    angle: dict[str, float | None] | None = None,
    visual_angle_confirmation: str = "unknown",
    turn_calibration_saved: bool = False,
    turn_calibration_out: str = "",
    stop_after_big_turn: bool = False,
    upper_limit_trial: bool,
) -> dict[str, object]:
    active = _active_rows(rows)
    source = active[-1] if active else (rows[-1] if rows else {})
    final_left = _parse_float(_latest(rows, "final_left_cmd", "0"), default=0.0)
    final_right = _parse_float(_latest(rows, "final_right_cmd", "0"), default=0.0)
    active_source = source.get("active_target_source", _latest(rows, "active_target_source", "NONE"))
    reject_reason = _latest_reject_reason(rows)
    valid_trial = reject_reason not in INVALID_REJECT_REASONS
    output_after_stop = any(
        row.get("event", "").upper() == "STOP" and _parse_bool(row.get("physical_output_active"))
        for row in rows
    )
    motion = visible_motion(left_report, right_report, body_report)
    motion_class = classify_motion(left_report, right_report, body_report)
    suspect = valid_trial and upper_limit_trial and not motion
    return {
        "trial_index": trial_index,
        "mode": planned["mode"],
        "cmd": planned["cmd"],
        "pulse_ms": planned["pulse_ms"],
        "command_axis": planned.get("command_axis", command_axis_for_probe_mode(str(planned["mode"]))),
        "axis_limit_used": planned.get("axis_limit_used", "NA"),
        "max_abs_a": planned.get("max_abs_a", "NA"),
        "max_abs_b": planned.get("max_abs_b", "NA"),
        "command_within_axis_limit": planned.get("command_within_axis_limit", False),
        "seq": planned["seq"],
        "forward_sign": planned["forward_sign"],
        "turn_sign": planned["turn_sign"],
        "arm_command_text": planned["arm_command_text"],
        "stage20_command_text": planned["stage20_command_text"],
        "stop_command_text": planned["stop_command_text"],
        "requested_a_cmd": planned["requested_a_cmd"],
        "requested_b_cmd": planned["requested_b_cmd"],
        "physical_a_cmd": source.get("physical_a_cmd", "NA"),
        "physical_b_cmd": source.get("physical_b_cmd", "NA"),
        "manual_equivalent_forward_cmd": source.get("manual_equivalent_forward_cmd", "NA"),
        "manual_equivalent_turn_cmd": source.get("manual_equivalent_turn_cmd", "NA"),
        "final_left_cmd": f"{final_left:.3f}",
        "final_right_cmd": f"{final_right:.3f}",
        "motor_write_called": any(_parse_bool(row.get("motor_write_called")) for row in active),
        "motor_backend": source.get("motor_backend", "NA"),
        "motor_enable_state": source.get("motor_enable_state", "NA"),
        "pwm_or_dynamixel_write_status": source.get("pwm_or_dynamixel_write_status", "NA"),
        "wheel_to_physical_mapping": source.get("wheel_to_physical_mapping", "NA"),
        "firmware_active_target_source": active_source,
        "physical_path_following_enable": _latest(rows, "physical_path_following_enable", "false"),
        "allow_motor_output": _latest(rows, "allow_motor_output", "false"),
        "physical_output_active_seen": any(_parse_bool(row.get("physical_output_active")) for row in rows),
        "physical_output_active_after_stop": output_after_stop,
        "valid_trial": valid_trial,
        "invalid_reason": "NONE" if valid_trial else reject_reason,
        "rc_ok_before_cmd": _latest(rows, "rc_ok", "NA"),
        "neutral_ok_before_cmd": _latest(rows, "neutral_ok", "NA"),
        "arm_ack_seen": any(row.get("event", "").upper() == "ARM" for row in rows),
        "ack_seen": any(row.get("event", "").upper() == "ACK" for row in rows),
        "stop_seen": any(row.get("event", "").upper() == "STOP" for row in rows),
        "reject_seen": any(row.get("event", "").upper() == "REJECT" for row in rows),
        "reject_reason": reject_reason,
        "left_wheel_user_report": left_report,
        "right_wheel_user_report": right_report,
        "body_motion_user_report": body_report,
        "turn_angle_report": turn_angle_report,
        "imu_angle_compare_enabled": angle_compare_enabled,
        "yaw_before_deg": _fmt_float(None if angle is None else angle.get("yaw_before_deg")),
        "yaw_after_deg": _fmt_float(None if angle is None else angle.get("yaw_after_deg")),
        "yaw_delta_deg": _fmt_float(None if angle is None else angle.get("yaw_delta_deg")),
        "abs_yaw_delta_deg": _fmt_float(None if angle is None else angle.get("abs_yaw_delta_deg")),
        "target_angle_deg": _fmt_float(None if angle is None else angle.get("target_angle_deg")),
        "angle_tolerance_deg": _fmt_float(None if angle is None else angle.get("angle_tolerance_deg")),
        "error_to_target_deg": _fmt_float(None if angle is None else angle.get("error_to_target_deg")),
        "visual_angle_confirmation": visual_angle_confirmation,
        "turn_calibration_saved": turn_calibration_saved,
        "turn_calibration_out": turn_calibration_out,
        "motion_class": motion_class,
        "visible_motion_confirmed": motion,
        "stop_after_big_turn": stop_after_big_turn,
        "motor_power_or_mapping_suspect": suspect,
        "next_action": "check Stage20 A/B signs or motor output backend" if suspect else "OK",
        "ready_for_full_path_following": False,
    }


def _write_csv(path: Path, rows: Sequence[dict[str, object]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=PROBE_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in PROBE_FIELDS})


def imu_yaw_block_reason(imu_angle_compare: bool, yaw_before_deg: float | None) -> str | None:
    """Return a block reason when IMU yaw is required but unavailable, else None.

    With ``--imu-angle-compare`` the turn-angle calibration needs a valid 'before'
    yaw; if the IMU never produced one we cannot compute an angle, so the probe
    must stop cleanly -- no traceback and no calibration JSON written.
    """
    if imu_angle_compare and yaw_before_deg is None:
        return "IMU_YAW_NOT_AVAILABLE"
    return None


def build_summary(
    rows: Sequence[dict[str, object]],
    imu_heading_block_reason: str = "NONE",
) -> dict[str, object]:
    visible_rows = [row for row in rows if row.get("visible_motion_confirmed") is True]
    first_visible = visible_rows[0] if visible_rows else {}
    suspect = bool(rows) and not visible_rows and rows[-1].get("motor_power_or_mapping_suspect") is True
    invalid_count = sum(1 for row in rows if row.get("valid_trial") is False)
    big_turn_seen = any(
        row.get("turn_angle_report") == "big_over_45deg"
        for row in rows
    )
    return {
        "stage20_heartbeat_seen": bool(rows),
        "arm_success_count": sum(1 for row in rows if row.get("arm_ack_seen") is True),
        "ack_count": sum(1 for row in rows if row.get("ack_seen") is True),
        "stop_count": sum(1 for row in rows if row.get("stop_seen") is True),
        "invalid_trial_count": invalid_count,
        "command_axes": ",".join(sorted({str(row.get("command_axis", "NA")) for row in rows})) if rows else "NA",
        "axis_limits_used": ",".join(sorted({str(row.get("axis_limit_used", "NA")) for row in rows})) if rows else "NA",
        "max_abs_a": rows[0].get("max_abs_a", DEFAULT_MAX_ABS_A) if rows else DEFAULT_MAX_ABS_A,
        "max_abs_b": rows[0].get("max_abs_b", DEFAULT_MAX_ABS_B) if rows else DEFAULT_MAX_ABS_B,
        "first_visible_motion_mode": first_visible.get("mode", "NA"),
        "first_visible_motion_a_cmd": first_visible.get("requested_a_cmd", "NA"),
        "first_visible_motion_b_cmd": first_visible.get("requested_b_cmd", "NA"),
        "first_visible_motion_ms": first_visible.get("pulse_ms", "NA"),
        "forward_sign": rows[0].get("forward_sign", DEFAULT_FORWARD_SIGN) if rows else DEFAULT_FORWARD_SIGN,
        "turn_sign": rows[0].get("turn_sign", DEFAULT_TURN_SIGN) if rows else DEFAULT_TURN_SIGN,
        "motor_power_or_mapping_suspect": suspect,
        "forward_threshold_unresolved": suspect,
        "forward_specific_deadband_or_sign_suspect": suspect,
        "big_turn_seen": big_turn_seen,
        "next_action": "reduce turn command before any path correction" if big_turn_seen else ("check Stage20 A/B signs or motor output backend" if suspect else "OK"),
        "final_stop_confirmed": bool(rows) and rows[-1].get("stop_seen") is True,
        "imu_heading_block_reason": imu_heading_block_reason,
        "ready_for_full_path_following": False,
    }


def _write_summary(path: Path, summary: dict[str, object]) -> None:
    lines = [
        "# Stage 20 Physical A/B Probe",
        "",
        "Direct throttle/turn manual-equivalent guarded probe only. Not full path following.",
        "",
    ]
    for key, value in summary.items():
        lines.append(f"- {key}: `{value}`")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run_probe(args: argparse.Namespace) -> int:
    try:
        import serial  # type: ignore[import-not-found]
    except ImportError as exc:
        raise RuntimeError("pyserial is not available; cannot run Stage 20 physical A/B probe") from exc

    cmd_values = parse_float_list(args.cmd_list, max_value=max(args.max_abs_a, args.max_abs_b))
    pulse_values = parse_int_list(args.pulse_ms_list, max_value=args.max_ms)
    selected_modes = modes_for_selection(args.mode)
    for mode in selected_modes:
        for cmd in cmd_values:
            validate_command_axis_limit(mode, cmd, max_abs_a=args.max_abs_a, max_abs_b=args.max_abs_b)
    require_enter = _parse_bool(args.require_enter, default=True)
    continue_after_visible = _parse_bool(args.continue_after_visible, default=False)
    interactive = _parse_bool(args.interactive_visible_motion, default=True)
    imu_angle_compare = _parse_bool(args.imu_angle_compare, default=False)
    save_turn_calibration = _parse_bool(args.save_turn_calibration, default=False)
    turn_calibration_out = Path(args.turn_calibration_out)
    rows: list[dict[str, object]] = []
    raw_lines: list[str] = []
    trial_index = 0
    visible_found = False
    abort_requested = False
    imu_heading_block_reason = "NONE"
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    with serial.Serial(args.port, baudrate=115200, timeout=0.5) as handle:
        _wait_for_event(handle, raw_lines, {"HEARTBEAT", "STOP"}, args.heartbeat_timeout_s)
        for mode in selected_modes:
            for cmd in cmd_values:
                for pulse_ms in pulse_values:
                    trial_index += 1
                    planned = planned_probe_commands(
                        seq=trial_index,
                        mode=mode,
                        cmd=cmd,
                        pulse_ms=pulse_ms,
                        forward_sign=args.forward_sign,
                        turn_sign=args.turn_sign,
                        max_abs_a=args.max_abs_a,
                        max_abs_b=args.max_abs_b,
                    )
                    if require_enter:
                        input(
                            "Place rover at the desired starting orientation, then press Enter. "
                            f"Stage20 {mode} cmd={cmd:.3f} pulse_ms={pulse_ms}. "
                        )
                    precondition_rows = _wait_for_stable_precondition(
                        handle,
                        raw_lines,
                        consecutive=args.stable_heartbeat_count,
                        timeout_s=args.precondition_timeout_s,
                    )
                    if not stable_precondition_met(precondition_rows, consecutive=args.stable_heartbeat_count):
                        raise RuntimeError("Stage20 precondition not stable before pulse; aborting")
                    angle: dict[str, float | None] | None = None
                    visual_angle_confirmation = "unknown"
                    if imu_angle_compare:
                        yaw_before = row_yaw_deg(latest_stable_heartbeat(precondition_rows))
                        block_reason = imu_yaw_block_reason(imu_angle_compare, yaw_before)
                        if block_reason is not None:
                            imu_heading_block_reason = block_reason
                            print(
                                "Stage20 IMU yaw not available before pulse; stopping cleanly "
                                f"(imu_heading_block_reason={block_reason}, no turn-angle calibration written)."
                            )
                            abort_requested = True
                            break
                        angle = angle_comparison_from_yaw(
                            yaw_before_deg=yaw_before,
                            yaw_after_deg=None,
                            target_angle_deg=args.target_angle_deg,
                            angle_tolerance_deg=args.angle_tolerance_deg,
                        )
                    axis_limit = axis_limit_for_probe_mode(mode, max_abs_a=args.max_abs_a, max_abs_b=args.max_abs_b)
                    upper_limit_trial = cmd >= axis_limit and pulse_ms >= args.max_ms
                    handle.write((str(planned["arm_command_text"]) + "\n").encode("ascii"))
                    handle.flush()
                    arm_rows = _wait_for_event(handle, raw_lines, {"ARM", "REJECT"}, args.event_timeout_s)
                    if not any(row.get("event", "").upper() == "ARM" for row in arm_rows):
                        rows.append(row_from_trial(
                            trial_index=trial_index,
                            planned=planned,
                            rows=arm_rows,
                            left_report="unknown",
                            right_report="unknown",
                            body_report="unknown",
                            turn_angle_report="unknown",
                            angle_compare_enabled=imu_angle_compare,
                            angle=angle,
                            visual_angle_confirmation=visual_angle_confirmation,
                            turn_calibration_saved=False,
                            turn_calibration_out=str(turn_calibration_out) if save_turn_calibration else "",
                            stop_after_big_turn=_parse_bool(args.stop_after_big_turn, default=True),
                            upper_limit_trial=upper_limit_trial,
                        ))
                        if rows[-1]["invalid_reason"] in INVALID_REJECT_REASONS:
                            abort_requested = True
                            visible_found = True
                            break
                        continue
                    handle.write((str(planned["stage20_command_text"]) + "\n").encode("ascii"))
                    handle.flush()
                    _wait_for_event(handle, raw_lines, {"ACK", "REJECT"}, args.event_timeout_s)
                    _wait_for_event(handle, raw_lines, {"STOP"}, max(args.event_timeout_s, pulse_ms / 1000.0 + 1.0))
                    handle.write((str(planned["stop_command_text"]) + "\n").encode("ascii"))
                    handle.flush()
                    final_rows = _wait_for_event(handle, raw_lines, {"STOP"}, args.event_timeout_s)
                    if imu_angle_compare:
                        final_left = _parse_float(_latest(final_rows, "final_left_cmd", "0"), default=0.0)
                        final_right = _parse_float(_latest(final_rows, "final_right_cmd", "0"), default=0.0)
                        output_after_stop = any(
                            row.get("event", "").upper() == "STOP" and _parse_bool(row.get("physical_output_active"))
                            for row in final_rows
                        )
                        yaw_before = None if angle is None else angle.get("yaw_before_deg")
                        yaw_after = None
                        if abs(final_left) <= 1e-9 and abs(final_right) <= 1e-9 and not output_after_stop:
                            after_rows = _wait_for_stable_precondition(
                                handle,
                                raw_lines,
                                consecutive=args.stable_heartbeat_count,
                                timeout_s=args.precondition_timeout_s,
                            )
                            yaw_after = row_yaw_deg(latest_stable_heartbeat(after_rows))
                            if yaw_after is None:
                                imu_heading_block_reason = "IMU_YAW_NOT_AVAILABLE"
                        angle = angle_comparison_from_yaw(
                            yaw_before_deg=yaw_before,
                            yaw_after_deg=yaw_after,
                            target_angle_deg=args.target_angle_deg,
                            angle_tolerance_deg=args.angle_tolerance_deg,
                        )
                        _print_angle_comparison(angle)
                        visual_angle_confirmation = (
                            _prompt_choice(
                                "Visual angle confirmation? [yes/under/over/unknown] ",
                                VISUAL_ANGLE_CONFIRMATIONS,
                            )
                            if interactive
                            else "unknown"
                        )
                    if interactive:
                        body_report = _prompt_choice(
                            "Did rover body move? [none/forward/backward/left/right/twitch/unknown] ",
                            BODY_RESPONSES,
                        )
                        if is_turn_mode(mode):
                            turn_angle_report = _prompt_choice(
                                "Did rover rotate angle? [none/twitch/small_under_15deg/medium_15_45deg/big_over_45deg/unknown] ",
                                TURN_ANGLE_RESPONSES,
                            )
                        else:
                            turn_angle_report = "none"
                        left_report = _prompt_choice("Did left wheel move? [yes/no/twitch/unknown] ", WHEEL_RESPONSES)
                        right_report = _prompt_choice("Did right wheel move? [yes/no/twitch/unknown] ", WHEEL_RESPONSES)
                    else:
                        left_report = "unknown"
                        right_report = "unknown"
                        body_report = "unknown"
                        turn_angle_report = "unknown"
                    row = row_from_trial(
                        trial_index=trial_index,
                        planned=planned,
                        rows=final_rows,
                        left_report=left_report,
                        right_report=right_report,
                        body_report=body_report,
                        turn_angle_report=turn_angle_report,
                        angle_compare_enabled=imu_angle_compare,
                        angle=angle,
                        visual_angle_confirmation=visual_angle_confirmation,
                        turn_calibration_saved=False,
                        turn_calibration_out=str(turn_calibration_out) if save_turn_calibration else "",
                        stop_after_big_turn=_parse_bool(args.stop_after_big_turn, default=True),
                        upper_limit_trial=upper_limit_trial,
                    )
                    rows.append(row)
                    if row["physical_output_active_after_stop"] is True:
                        raise RuntimeError("Stage20 output remained active after STOP; aborting")
                    if row["final_left_cmd"] != "0.000" or row["final_right_cmd"] != "0.000":
                        raise RuntimeError("Stage20 final wheel commands were nonzero; aborting")
                    if row["ack_seen"] is True and row["stop_seen"] is not True:
                        raise RuntimeError("Stage20 ACK was seen without STOP; aborting")
                    if imu_angle_compare and save_turn_calibration and angle is not None:
                        saved = save_turn_calibration_result(
                            path=turn_calibration_out,
                            planned=planned,
                            row=row,
                            angle=angle,
                            visual_confirmation=visual_angle_confirmation,
                        )
                        row["turn_calibration_saved"] = saved
                        if saved:
                            print(f"turn_calibration_saved={turn_calibration_out}")
                    if row["invalid_reason"] in INVALID_REJECT_REASONS:
                        abort_requested = True
                        visible_found = True
                        break
                    visible_found = visible_found or stop_search_after_user_report(row)
                    if is_turn_mode(mode) and row["turn_angle_report"] == "big_over_45deg" and _parse_bool(args.stop_after_big_turn, default=True):
                        visible_found = True
                    if visible_found and not continue_after_visible:
                        break
                    if abort_requested:
                        break
                if abort_requested:
                    break
                if visible_found and not continue_after_visible:
                    break
            if abort_requested:
                break
            if visible_found and not continue_after_visible:
                break

    _write_csv(out_dir / "stage20_physical_ab_probe.csv", rows)
    (out_dir / "raw_usbdbg.log").write_text("\n".join(raw_lines) + ("\n" if raw_lines else ""), encoding="utf-8")
    _write_summary(out_dir / "summary.md", build_summary(rows, imu_heading_block_reason))
    print("Stage 20 physical A/B probe complete.")
    print(f"stage20_physical_ab_probe_csv={out_dir / 'stage20_physical_ab_probe.csv'}")
    print(f"raw_usbdbg_log={out_dir / 'raw_usbdbg.log'}")
    print(f"summary_md={out_dir / 'summary.md'}")
    print(f"imu_heading_block_reason={imu_heading_block_reason}")
    print("ready_for_full_path_following=false")
    return 3 if imu_heading_block_reason != "NONE" else 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Stage 20 direct physical A/B manual-equivalent guarded probe.")
    parser.add_argument("--port", required=True)
    parser.add_argument("--mode", choices=("forward", "backward", "turn_left", "turn_right", "a_sweep", "b_sweep", "all"), default="all")
    parser.add_argument("--cmd-list", default="0.08,0.12,0.16,0.20,0.25")
    parser.add_argument("--pulse-ms-list", default="300,500,800")
    parser.add_argument("--max-abs-a", type=float, default=DEFAULT_MAX_ABS_A)
    parser.add_argument("--max-abs-b", type=float, default=DEFAULT_MAX_ABS_B)
    parser.add_argument("--max-ms", type=int, default=DEFAULT_MAX_MS)
    parser.add_argument("--forward-sign", type=float, default=DEFAULT_FORWARD_SIGN)
    parser.add_argument("--turn-sign", type=float, default=DEFAULT_TURN_SIGN)
    parser.add_argument("--stop-after-big-turn", default="true")
    parser.add_argument("--imu-angle-compare", default="false")
    parser.add_argument("--target-angle-deg", type=float, default=90.0)
    parser.add_argument("--angle-tolerance-deg", type=float, default=10.0)
    parser.add_argument("--save-turn-calibration", default="false")
    parser.add_argument(
        "--turn-calibration-out",
        default="outputs/stage23_turn_calibration/calibration/physical_ab_turn_angle_calibration.json",
    )
    parser.add_argument("--require-enter", default="true")
    parser.add_argument("--interactive-visible-motion", default="true")
    parser.add_argument("--continue-after-visible", default="false")
    parser.add_argument("--heartbeat-timeout-s", type=float, default=10.0)
    parser.add_argument("--event-timeout-s", type=float, default=5.0)
    parser.add_argument("--stable-heartbeat-count", type=int, default=3)
    parser.add_argument("--precondition-timeout-s", type=float, default=5.0)
    parser.add_argument("--out-dir", required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    return run_probe(build_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
