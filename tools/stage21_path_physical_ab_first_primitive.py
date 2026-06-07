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

from tools import stage20_physical_ab_probe, station_path_package_tracker, station_virtual_path_controller
from tools.path_no_motion_validation import PathPackageResolutionError, resolve_path_package


STAGE21_FIELDS = (
    "row_index",
    "dry_run",
    "path_package_loaded",
    "station_package_target_source",
    "pose_mode",
    "local_pose_available",
    "firmware_active_target_source",
    "physical_path_following_enable",
    "allow_motor_output",
    "active_primitive_index",
    "active_tool_segment_id",
    "target_distance_m",
    "target_bearing_deg",
    "cross_track_error_m",
    "virtual_forward_cmd",
    "virtual_turn_cmd",
    "virtual_left_cmd",
    "virtual_right_cmd",
    "forward_sign",
    "turn_sign",
    "calibration_json",
    "fine_calibration_json",
    "use_recommended_crawl_command",
    "min_effective_a",
    "min_effective_b",
    "max_abs_a",
    "max_abs_b",
    "max_ms",
    "stage21_forward_a_cmd",
    "stage21_forward_pulse_ms",
    "stage21_backward_a_cmd",
    "stage21_backward_pulse_ms",
    "ready_for_stage21_forward",
    "ready_for_stage21_backward",
    "forward_threshold_unresolved",
    "stage21_command_blocked_reason",
    "motion_class_source",
    "b_forced_zero",
    "max_forward_cmd",
    "max_backward_cmd",
    "max_turn_left_cmd",
    "max_turn_right_cmd",
    "straight_only",
    "turn_disabled",
    "diagnostic_only_straight_primitive",
    "stage20_a_cmd",
    "stage20_b_cmd",
    "stage20_ms",
    "command_within_firmware_limits",
    "stage20_command_text",
    "arm_command_text",
    "stop_command_text",
    "arm_ack_seen",
    "ack_seen",
    "stop_seen",
    "reject_seen",
    "reject_reason",
    "physical_output_active_seen",
    "physical_output_active_after_stop",
    "final_left_cmd",
    "final_right_cmd",
    "final_left_cmd_zero",
    "final_right_cmd_zero",
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


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _latest(rows: Sequence[dict[str, str]], key: str, default: str = "NA") -> str:
    for row in reversed(rows):
        if key in row:
            return row[key]
    return default


def parse_stage20_ready(rows: Sequence[dict[str, str]]) -> tuple[bool, bool]:
    heartbeat_seen = False
    for row in rows:
        if row.get("event", "").upper() != "HEARTBEAT":
            continue
        if _parse_bool(row.get("stage20_physical_ab_guarded_crawl")) is not True:
            continue
        heartbeat_seen = True
        if (
            _parse_bool(row.get("stage20_firmware_ready")) is True
            and _parse_bool(row.get("physical_path_following_enable")) is False
            and _parse_bool(row.get("allow_motor_output")) is False
        ):
            return True, heartbeat_seen
    return False, heartbeat_seen


def apply_min_effective(value: float, *, minimum: float, limit: float) -> float:
    bounded = _clamp(value, limit)
    if abs(bounded) < 1e-12:
        return 0.0
    if abs(bounded) < minimum:
        return math.copysign(min(minimum, limit), bounded)
    return bounded


def load_directional_calibration(path: Path | None) -> dict[str, object]:
    if path is None:
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def load_fine_calibration(path: Path | None) -> dict[str, object]:
    if path is None:
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def _cal_float(calibration: dict[str, object], key: str, default: float) -> float:
    value = _optional_float(calibration.get(key))
    return default if value is None else value


def physical_ab_from_virtual(
    virtual_row: dict[str, object],
    *,
    forward_sign: float = stage20_physical_ab_probe.DEFAULT_FORWARD_SIGN,
    turn_sign: float = stage20_physical_ab_probe.DEFAULT_TURN_SIGN,
    min_effective_a: float = 0.08,
    min_effective_b: float = 0.08,
    max_abs_a: float = stage20_physical_ab_probe.DEFAULT_MAX_ABS_A,
    max_abs_b: float = stage20_physical_ab_probe.DEFAULT_MAX_ABS_B,
    calibration: dict[str, object] | None = None,
    straight_only: bool = False,
    turn_disabled: bool = False,
    max_forward_cmd: float | None = None,
    max_backward_cmd: float | None = None,
    max_turn_left_cmd: float | None = None,
    max_turn_right_cmd: float | None = None,
    fine_calibration: dict[str, object] | None = None,
    use_recommended_crawl_command: bool = True,
) -> tuple[float, float]:
    calibration = calibration or {}
    fine_calibration = fine_calibration or {}
    if calibration:
        forward_sign = _cal_float(calibration, "forward_sign", forward_sign)
        turn_sign = _cal_float(calibration, "turn_sign", turn_sign)
        if max_forward_cmd is None:
            max_forward_cmd = _cal_float(calibration, "max_forward_cmd", max_abs_a)
        if max_backward_cmd is None:
            max_backward_cmd = _cal_float(calibration, "max_backward_cmd", max_abs_a)
        if max_turn_left_cmd is None:
            max_turn_left_cmd = _cal_float(calibration, "max_turn_left_cmd", max_abs_b)
        if max_turn_right_cmd is None:
            max_turn_right_cmd = _cal_float(calibration, "max_turn_right_cmd", max_abs_b)
    max_forward_cmd = max_forward_cmd if max_forward_cmd is not None else max_abs_a
    max_backward_cmd = max_backward_cmd if max_backward_cmd is not None else max_abs_a
    max_turn_left_cmd = max_turn_left_cmd if max_turn_left_cmd is not None else max_abs_b
    max_turn_right_cmd = max_turn_right_cmd if max_turn_right_cmd is not None else max_abs_b
    forward = _optional_float(virtual_row.get("virtual_forward_cmd")) or 0.0
    turn = _optional_float(virtual_row.get("virtual_turn_cmd")) or 0.0
    if fine_calibration and use_recommended_crawl_command:
        if forward > 0:
            if fine_calibration.get("ready_for_stage21_forward") is False:
                return 0.0, 0.0
            a_cmd = _cal_float(fine_calibration, "stage21_forward_a_cmd", 0.0)
        elif forward < 0:
            if fine_calibration.get("ready_for_stage21_backward") is False:
                return 0.0, 0.0
            a_cmd = _cal_float(fine_calibration, "stage21_backward_a_cmd", 0.0)
        else:
            a_cmd = 0.0
        b_cmd = 0.0 if straight_only or turn_disabled or fine_calibration.get("turn_disabled") is True else 0.0
    elif calibration:
        if forward > 0:
            forward_min = abs(_cal_float(calibration, "forward_min_a_cmd", min_effective_a))
            a_cmd = apply_min_effective(forward_sign * forward, minimum=forward_min, limit=max_forward_cmd)
        elif forward < 0:
            backward_min = abs(_cal_float(calibration, "backward_min_a_cmd", min_effective_a))
            a_cmd = -apply_min_effective(abs(forward), minimum=backward_min, limit=max_backward_cmd)
        else:
            a_cmd = 0.0
        turning_ready = bool(calibration.get("ready_for_stage21_turning") is True)
        if straight_only or turn_disabled or not turning_ready:
            b_cmd = 0.0
        elif turn > 0:
            turn_left_min = abs(_cal_float(calibration, "turn_left_min_b_cmd", min_effective_b))
            b_cmd = apply_min_effective(turn_sign * turn, minimum=turn_left_min, limit=max_turn_left_cmd)
        elif turn < 0:
            turn_right_min = abs(_cal_float(calibration, "turn_right_min_b_cmd", min_effective_b))
            b_cmd = -apply_min_effective(abs(turn), minimum=turn_right_min, limit=max_turn_right_cmd)
        else:
            b_cmd = 0.0
    else:
        a_cmd = apply_min_effective(forward_sign * forward, minimum=min_effective_a, limit=max_abs_a)
        b_cmd = 0.0 if straight_only or turn_disabled else apply_min_effective(turn_sign * turn, minimum=min_effective_b, limit=max_abs_b)
    return a_cmd, b_cmd


def pulse_ms_from_calibration(
    virtual_row: dict[str, object],
    *,
    explicit_pulse_ms: int | None,
    fine_calibration: dict[str, object] | None,
    use_recommended_crawl_command: bool,
    default_pulse_ms: int = 300,
) -> int:
    if explicit_pulse_ms is not None:
        return explicit_pulse_ms
    fine_calibration = fine_calibration or {}
    forward = _optional_float(virtual_row.get("virtual_forward_cmd")) or 0.0
    if fine_calibration and use_recommended_crawl_command:
        if forward > 0:
            return int(_cal_float(fine_calibration, "stage21_forward_pulse_ms", default_pulse_ms))
        if forward < 0:
            return int(_cal_float(fine_calibration, "stage21_backward_pulse_ms", default_pulse_ms))
    return default_pulse_ms


def planned_stage21_command(
    *,
    seq: int,
    virtual_row: dict[str, object],
    pulse_ms: int | None,
    forward_sign: float = stage20_physical_ab_probe.DEFAULT_FORWARD_SIGN,
    turn_sign: float = stage20_physical_ab_probe.DEFAULT_TURN_SIGN,
    min_effective_a: float = 0.08,
    min_effective_b: float = 0.08,
    max_abs_a: float = stage20_physical_ab_probe.DEFAULT_MAX_ABS_A,
    max_abs_b: float = stage20_physical_ab_probe.DEFAULT_MAX_ABS_B,
    calibration: dict[str, object] | None = None,
    calibration_json: str = "",
    straight_only: bool = False,
    turn_disabled: bool = False,
    max_forward_cmd: float | None = None,
    max_backward_cmd: float | None = None,
    max_turn_left_cmd: float | None = None,
    max_turn_right_cmd: float | None = None,
    fine_calibration: dict[str, object] | None = None,
    fine_calibration_json: str = "",
    use_recommended_crawl_command: bool = True,
    max_ms: int = 800,
) -> dict[str, object]:
    a_cmd, b_cmd = physical_ab_from_virtual(
        virtual_row,
        forward_sign=forward_sign,
        turn_sign=turn_sign,
        min_effective_a=min_effective_a,
        min_effective_b=min_effective_b,
        max_abs_a=max_abs_a,
        max_abs_b=max_abs_b,
        calibration=calibration,
        straight_only=straight_only,
        turn_disabled=turn_disabled,
        max_forward_cmd=max_forward_cmd,
        max_backward_cmd=max_backward_cmd,
        max_turn_left_cmd=max_turn_left_cmd,
        max_turn_right_cmd=max_turn_right_cmd,
        fine_calibration=fine_calibration,
        use_recommended_crawl_command=use_recommended_crawl_command,
    )
    actual_pulse_ms = pulse_ms_from_calibration(
        virtual_row,
        explicit_pulse_ms=pulse_ms,
        fine_calibration=fine_calibration,
        use_recommended_crawl_command=use_recommended_crawl_command,
    )
    forward = _optional_float(virtual_row.get("virtual_forward_cmd")) or 0.0
    blocked_reason = "NONE"
    fine = fine_calibration or {}
    if fine and use_recommended_crawl_command:
        if forward > 0 and fine.get("ready_for_stage21_forward") is False:
            blocked_reason = "FORWARD_THRESHOLD_UNRESOLVED"
        elif forward < 0 and fine.get("ready_for_stage21_backward") is False:
            blocked_reason = "BACKWARD_THRESHOLD_UNRESOLVED"
    command_within_firmware_limits = abs(a_cmd) <= max_abs_a + 1e-12 and abs(b_cmd) <= max_abs_b + 1e-12 and actual_pulse_ms <= max_ms
    if blocked_reason == "NONE" and not command_within_firmware_limits:
        if abs(a_cmd) > max_abs_a + 1e-12 or abs(b_cmd) > max_abs_b + 1e-12:
            blocked_reason = "COMMAND_EXCEEDS_FIRMWARE_LIMIT"
        elif actual_pulse_ms > max_ms:
            blocked_reason = "PULSE_EXCEEDS_FIRMWARE_LIMIT"
    b_forced_zero = (straight_only or turn_disabled or (fine_calibration or {}).get("turn_disabled") is True) and abs(b_cmd) <= 1e-12
    return {
        "seq": seq,
        "forward_sign": forward_sign,
        "turn_sign": turn_sign,
        "calibration_json": calibration_json,
        "fine_calibration_json": fine_calibration_json,
        "use_recommended_crawl_command": use_recommended_crawl_command,
        "min_effective_a": min_effective_a,
        "min_effective_b": min_effective_b,
        "max_abs_a": max_abs_a,
        "max_abs_b": max_abs_b,
        "max_ms": max_ms,
        "stage21_forward_a_cmd": (fine_calibration or {}).get("stage21_forward_a_cmd", "NA"),
        "stage21_forward_pulse_ms": (fine_calibration or {}).get("stage21_forward_pulse_ms", "NA"),
        "stage21_backward_a_cmd": (fine_calibration or {}).get("stage21_backward_a_cmd", "NA"),
        "stage21_backward_pulse_ms": (fine_calibration or {}).get("stage21_backward_pulse_ms", "NA"),
        "ready_for_stage21_forward": (fine_calibration or {}).get("ready_for_stage21_forward", "NA"),
        "ready_for_stage21_backward": (fine_calibration or {}).get("ready_for_stage21_backward", "NA"),
        "forward_threshold_unresolved": (fine_calibration or {}).get("forward_threshold_unresolved", "NA"),
        "stage21_command_blocked_reason": blocked_reason,
        "motion_class_source": (fine_calibration or {}).get("motion_class_source", "NA"),
        "b_forced_zero": b_forced_zero,
        "max_forward_cmd": max_forward_cmd if max_forward_cmd is not None else max_abs_a,
        "max_backward_cmd": max_backward_cmd if max_backward_cmd is not None else max_abs_a,
        "max_turn_left_cmd": max_turn_left_cmd if max_turn_left_cmd is not None else max_abs_b,
        "max_turn_right_cmd": max_turn_right_cmd if max_turn_right_cmd is not None else max_abs_b,
        "straight_only": straight_only,
        "turn_disabled": turn_disabled,
        "diagnostic_only_straight_primitive": straight_only or turn_disabled,
        "stage20_a_cmd": a_cmd,
        "stage20_b_cmd": b_cmd,
        "stage20_ms": actual_pulse_ms,
        "command_within_firmware_limits": command_within_firmware_limits,
        "arm_command_text": f"STAGE20_ARM seq={seq}",
        "stage20_command_text": f"STAGE20_CMD seq={seq} a={a_cmd:.3f} b={b_cmd:.3f} ms={actual_pulse_ms}",
        "stop_command_text": f"STAGE20_STOP seq={seq}",
    }


def _target_row(
    package: dict[str, object],
    usbdbg_text: str,
    *,
    pose_mode: str,
    current_x: float | None,
    current_y: float | None,
    current_heading_deg: float | None,
) -> dict[str, object]:
    if pose_mode == "manual_local":
        if current_x is None or current_y is None:
            return station_path_package_tracker.diagnostic_status(
                mode="stage21_path_physical_ab_first_primitive",
                row_index=0,
                reason="MANUAL_LOCAL_POSE_REQUIRED",
                firmware_active_target_source="unknown",
            )
        return station_path_package_tracker.compute_target_status(
            package,
            current_x=current_x,
            current_y=current_y,
            current_heading_deg=current_heading_deg,
            mode="stage21_path_physical_ab_first_primitive",
            firmware_active_target_source="station_physical_ab_guarded",
            local_pose_source="manual_local",
        )
    rows = station_path_package_tracker.build_rows_from_replay(
        package,
        usbdbg_text,
        "stage21_path_physical_ab_first_primitive",
    )
    return rows[0] if rows else station_path_package_tracker.diagnostic_status(
        mode="stage21_path_physical_ab_first_primitive",
        row_index=0,
        reason="NO_USBDBG_ROWS_READ",
        firmware_active_target_source="unknown",
    )


def build_stage21_row(
    package: dict[str, object],
    usbdbg_text: str,
    *,
    dry_run: bool,
    pose_mode: str,
    current_x: float | None,
    current_y: float | None,
    current_heading_deg: float | None,
    pulse_ms: int | None,
    seq: int = 1,
    forward_sign: float = stage20_physical_ab_probe.DEFAULT_FORWARD_SIGN,
    turn_sign: float = stage20_physical_ab_probe.DEFAULT_TURN_SIGN,
    min_effective_a: float = 0.08,
    min_effective_b: float = 0.08,
    max_abs_a: float = stage20_physical_ab_probe.DEFAULT_MAX_ABS_A,
    max_abs_b: float = stage20_physical_ab_probe.DEFAULT_MAX_ABS_B,
    calibration: dict[str, object] | None = None,
    calibration_json: str = "",
    straight_only: bool = False,
    turn_disabled: bool = False,
    max_forward_cmd: float | None = None,
    max_backward_cmd: float | None = None,
    max_turn_left_cmd: float | None = None,
    max_turn_right_cmd: float | None = None,
    fine_calibration: dict[str, object] | None = None,
    fine_calibration_json: str = "",
    use_recommended_crawl_command: bool = True,
    max_ms: int = 800,
) -> dict[str, object]:
    usb_rows = station_path_package_tracker.parse_usbdbg_rows(usbdbg_text)
    target = _target_row(
        package,
        usbdbg_text,
        pose_mode=pose_mode,
        current_x=current_x,
        current_y=current_y,
        current_heading_deg=current_heading_deg,
    )
    virtual = station_virtual_path_controller.compute_virtual_control(
        target,
        max_virtual_forward_cmd=max_abs_a,
        max_virtual_turn_cmd=max_abs_b,
    )
    command = planned_stage21_command(
        seq=seq,
        virtual_row=virtual,
        pulse_ms=pulse_ms,
        forward_sign=forward_sign,
        turn_sign=turn_sign,
        min_effective_a=min_effective_a,
        min_effective_b=min_effective_b,
        max_abs_a=max_abs_a,
        max_abs_b=max_abs_b,
        calibration=calibration,
        calibration_json=calibration_json,
        straight_only=straight_only,
        turn_disabled=turn_disabled,
        max_forward_cmd=max_forward_cmd,
        max_backward_cmd=max_backward_cmd,
        max_turn_left_cmd=max_turn_left_cmd,
        max_turn_right_cmd=max_turn_right_cmd,
        fine_calibration=fine_calibration,
        fine_calibration_json=fine_calibration_json,
        use_recommended_crawl_command=use_recommended_crawl_command,
        max_ms=max_ms,
    )
    ack_seen = any(row.get("event", "").upper() == "ACK" for row in usb_rows)
    arm_ack_seen = any(row.get("event", "").upper() == "ARM" for row in usb_rows)
    stop_seen = any(row.get("event", "").upper() == "STOP" for row in usb_rows)
    reject_seen = any(row.get("event", "").upper() == "REJECT" for row in usb_rows)
    final_left = _optional_float(_latest(usb_rows, "final_left_cmd", "0.0")) or 0.0
    final_right = _optional_float(_latest(usb_rows, "final_right_cmd", "0.0")) or 0.0
    physical_active_after_stop = any(
        row.get("event", "").upper() == "STOP" and _parse_bool(row.get("physical_output_active"))
        for row in usb_rows
    )
    physical_seen = any(_parse_bool(row.get("physical_output_active")) for row in usb_rows)
    active_source = _latest(usb_rows, "active_target_source", "unknown")
    for row in usb_rows:
        if row.get("event", "").upper() == "ACK" or row.get("stage20_cmd_state", "").upper() == "ACTIVE":
            active_source = row.get("active_target_source", active_source)
    return {
        "row_index": 0,
        "dry_run": dry_run,
        "path_package_loaded": True,
        "station_package_target_source": "path_package",
        "pose_mode": pose_mode,
        "local_pose_available": target.get("local_pose_available") is True,
        "firmware_active_target_source": active_source,
        "physical_path_following_enable": _latest(usb_rows, "physical_path_following_enable", "false"),
        "allow_motor_output": _latest(usb_rows, "allow_motor_output", "false"),
        "active_primitive_index": target.get("active_primitive_index", "NA"),
        "active_tool_segment_id": target.get("active_tool_segment_id", "NA"),
        "target_distance_m": target.get("target_distance_m", "NA"),
        "target_bearing_deg": target.get("target_bearing_deg", "NA"),
        "cross_track_error_m": target.get("cross_track_error_m", "NA"),
        "virtual_forward_cmd": virtual.get("virtual_forward_cmd", 0.0),
        "virtual_turn_cmd": virtual.get("virtual_turn_cmd", 0.0),
        "virtual_left_cmd": virtual.get("virtual_left_cmd", 0.0),
        "virtual_right_cmd": virtual.get("virtual_right_cmd", 0.0),
        **command,
        "arm_ack_seen": arm_ack_seen,
        "ack_seen": ack_seen,
        "stop_seen": stop_seen,
        "reject_seen": reject_seen,
        "reject_reason": _latest(usb_rows, "stage16_reject_reason", "NONE"),
        "physical_output_active_seen": physical_seen,
        "physical_output_active_after_stop": physical_active_after_stop,
        "final_left_cmd": f"{final_left:.3f}",
        "final_right_cmd": f"{final_right:.3f}",
        "final_left_cmd_zero": abs(final_left) <= 1e-9,
        "final_right_cmd_zero": abs(final_right) <= 1e-9,
        "ready_for_full_path_following": False,
    }


def _write_csv(path: Path, rows: Sequence[dict[str, object]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=STAGE21_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in STAGE21_FIELDS})


def _write_summary(path: Path, selected_path: Path, rows: Sequence[dict[str, object]]) -> None:
    first = rows[0] if rows else {}
    lines = [
        "# Stage 21 Path Physical A/B First Primitive",
        "",
        "Single bounded path-package primitive using Stage20 physical A/B commands. Not full path following.",
        "",
        f"- selected_path_package: `{selected_path}`",
        f"- path_package_loaded: `{bool(rows)}`",
        f"- station_package_target_source: `path_package`",
        f"- active_primitive_index: `{first.get('active_primitive_index', 'NA')}`",
        f"- target_distance_m: `{first.get('target_distance_m', 'NA')}`",
        f"- target_bearing_deg: `{first.get('target_bearing_deg', 'NA')}`",
        f"- virtual_forward_cmd: `{first.get('virtual_forward_cmd', 0.0)}`",
        f"- virtual_turn_cmd: `{first.get('virtual_turn_cmd', 0.0)}`",
        f"- stage20_a_cmd: `{first.get('stage20_a_cmd', 0.0)}`",
        f"- stage20_b_cmd: `{first.get('stage20_b_cmd', 0.0)}`",
        f"- stage20_ms: `{first.get('stage20_ms', 'NA')}`",
        f"- max_abs_a: `{first.get('max_abs_a', 'NA')}`",
        f"- max_abs_b: `{first.get('max_abs_b', 'NA')}`",
        f"- max_ms: `{first.get('max_ms', 'NA')}`",
        f"- command_within_firmware_limits: `{first.get('command_within_firmware_limits', 'NA')}`",
        f"- calibration_json: `{first.get('calibration_json', '')}`",
        f"- fine_calibration_json: `{first.get('fine_calibration_json', '')}`",
        f"- stage21_forward_a_cmd: `{first.get('stage21_forward_a_cmd', 'NA')}`",
        f"- stage21_forward_pulse_ms: `{first.get('stage21_forward_pulse_ms', 'NA')}`",
        f"- stage21_backward_a_cmd: `{first.get('stage21_backward_a_cmd', 'NA')}`",
        f"- stage21_backward_pulse_ms: `{first.get('stage21_backward_pulse_ms', 'NA')}`",
        f"- ready_for_stage21_forward: `{first.get('ready_for_stage21_forward', 'NA')}`",
        f"- ready_for_stage21_backward: `{first.get('ready_for_stage21_backward', 'NA')}`",
        f"- forward_threshold_unresolved: `{first.get('forward_threshold_unresolved', 'NA')}`",
        f"- stage21_command_blocked_reason: `{first.get('stage21_command_blocked_reason', 'NONE')}`",
        f"- motion_class_source: `{first.get('motion_class_source', 'NA')}`",
        f"- straight_only: `{first.get('straight_only', False)}`",
        f"- turn_disabled: `{first.get('turn_disabled', False)}`",
        f"- diagnostic_only_straight_primitive: `{first.get('diagnostic_only_straight_primitive', False)}`",
        f"- B forced zero: `{first.get('b_forced_zero', False)}`",
        f"- final_left_cmd_zero: `{first.get('final_left_cmd_zero', False)}`",
        f"- final_right_cmd_zero: `{first.get('final_right_cmd_zero', False)}`",
        "- ready_for_full_path_following: `False`",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _wait_for_event(handle: object, raw_lines: list[str], wanted: set[str], timeout_s: float) -> list[dict[str, str]]:
    deadline = time.monotonic() + timeout_s
    start_index = len(raw_lines)
    while time.monotonic() < deadline:
        raw = handle.readline()
        if raw:
            line = raw.decode("utf-8", errors="replace").strip()
            print(line)
            raw_lines.append(line)
            new_rows = station_path_package_tracker.parse_usbdbg_rows("\n".join(raw_lines[start_index:]))
            if any(row.get("event", "").upper() in wanted for row in new_rows):
                return station_path_package_tracker.parse_usbdbg_rows("\n".join(raw_lines))
    return station_path_package_tracker.parse_usbdbg_rows("\n".join(raw_lines))


def _read_serial_session(args: argparse.Namespace, package: dict[str, object]) -> tuple[str, list[dict[str, object]]]:
    try:
        import serial  # type: ignore[import-not-found]
    except ImportError as exc:
        raise RuntimeError("pyserial is not available; cannot run Stage 21 physical A/B first primitive") from exc
    raw_lines: list[str] = []
    dry_run = _parse_bool(args.dry_run, default=True)
    require_enter = _parse_bool(args.require_enter, default=True)
    calibration_path = Path(args.calibration_json) if args.calibration_json else None
    calibration = load_directional_calibration(calibration_path)
    fine_calibration_path = Path(args.fine_calibration_json) if args.fine_calibration_json else None
    fine_calibration = load_fine_calibration(fine_calibration_path)
    straight_only = _parse_bool(args.straight_only, default=False)
    turn_disabled = _parse_bool(args.turn_disabled, default=False)
    use_recommended = _parse_bool(args.use_recommended_crawl_command, default=True)
    calibration_json = "" if calibration_path is None else str(calibration_path)
    fine_calibration_json = "" if fine_calibration_path is None else str(fine_calibration_path)
    with serial.Serial(args.port, baudrate=115200, timeout=0.5) as handle:
        ready_rows = _wait_for_event(handle, raw_lines, {"HEARTBEAT", "STOP"}, min(10.0, max(args.duration_s, 0.1)))
        ready, _heartbeat = parse_stage20_ready(ready_rows)
        if _parse_bool(args.require_stage20_ready, default=True) and not ready:
            raise RuntimeError("FIRMWARE_NOT_READY_NEEDS_UPLOAD")
        proposed = build_stage21_row(
            package,
            "\n".join(raw_lines),
            dry_run=dry_run,
            pose_mode=args.pose_mode,
            current_x=args.current_x,
            current_y=args.current_y,
            current_heading_deg=args.current_heading_deg,
            pulse_ms=args.pulse_ms,
            seq=1,
            forward_sign=args.forward_sign,
            turn_sign=args.turn_sign,
            min_effective_a=args.min_effective_a,
            min_effective_b=args.min_effective_b,
            max_abs_a=args.max_abs_a,
            max_abs_b=args.max_abs_b,
            calibration=calibration,
            calibration_json=calibration_json,
            straight_only=straight_only,
            turn_disabled=turn_disabled,
            max_forward_cmd=args.max_forward_cmd,
            max_backward_cmd=args.max_backward_cmd,
            max_turn_left_cmd=args.max_turn_left_cmd,
            max_turn_right_cmd=args.max_turn_right_cmd,
            fine_calibration=fine_calibration,
            fine_calibration_json=fine_calibration_json,
            use_recommended_crawl_command=use_recommended,
            max_ms=args.max_ms,
        )
        if dry_run:
            return "\n".join(raw_lines), [proposed]
        if proposed["local_pose_available"] is not True:
            return "\n".join(raw_lines), [proposed]
        if proposed["stage21_command_blocked_reason"] != "NONE" or proposed["command_within_firmware_limits"] is not True:
            return "\n".join(raw_lines), [proposed]
        if require_enter:
            input(f"Press Enter to send Stage21 first primitive pulse: {proposed['stage20_command_text']} ")
        handle.write((str(proposed["arm_command_text"]) + "\n").encode("ascii"))
        handle.flush()
        _wait_for_event(handle, raw_lines, {"ARM", "REJECT"}, 5.0)
        armed = build_stage21_row(
            package,
            "\n".join(raw_lines),
            dry_run=False,
            pose_mode=args.pose_mode,
            current_x=args.current_x,
            current_y=args.current_y,
            current_heading_deg=args.current_heading_deg,
            pulse_ms=args.pulse_ms,
            seq=1,
            forward_sign=args.forward_sign,
            turn_sign=args.turn_sign,
            min_effective_a=args.min_effective_a,
            min_effective_b=args.min_effective_b,
            max_abs_a=args.max_abs_a,
            max_abs_b=args.max_abs_b,
            calibration=calibration,
            calibration_json=calibration_json,
            straight_only=straight_only,
            turn_disabled=turn_disabled,
            max_forward_cmd=args.max_forward_cmd,
            max_backward_cmd=args.max_backward_cmd,
            max_turn_left_cmd=args.max_turn_left_cmd,
            max_turn_right_cmd=args.max_turn_right_cmd,
            fine_calibration=fine_calibration,
            fine_calibration_json=fine_calibration_json,
            use_recommended_crawl_command=use_recommended,
            max_ms=args.max_ms,
        )
        if armed["arm_ack_seen"] is not True:
            return "\n".join(raw_lines), [armed]
        if armed["stage21_command_blocked_reason"] != "NONE" or armed["command_within_firmware_limits"] is not True:
            return "\n".join(raw_lines), [armed]
        handle.write((str(armed["stage20_command_text"]) + "\n").encode("ascii"))
        handle.flush()
        _wait_for_event(handle, raw_lines, {"ACK", "REJECT"}, 5.0)
        effective_pulse_ms = int(armed["stage20_ms"])
        _wait_for_event(handle, raw_lines, {"STOP"}, max(5.0, effective_pulse_ms / 1000.0 + 1.0))
        handle.write((str(armed["stop_command_text"]) + "\n").encode("ascii"))
        handle.flush()
        _wait_for_event(handle, raw_lines, {"STOP"}, 5.0)
        completed = build_stage21_row(
            package,
            "\n".join(raw_lines),
            dry_run=False,
            pose_mode=args.pose_mode,
            current_x=args.current_x,
            current_y=args.current_y,
            current_heading_deg=args.current_heading_deg,
            pulse_ms=args.pulse_ms,
            seq=1,
            forward_sign=args.forward_sign,
            turn_sign=args.turn_sign,
            min_effective_a=args.min_effective_a,
            min_effective_b=args.min_effective_b,
            max_abs_a=args.max_abs_a,
            max_abs_b=args.max_abs_b,
            calibration=calibration,
            calibration_json=calibration_json,
            straight_only=straight_only,
            turn_disabled=turn_disabled,
            max_forward_cmd=args.max_forward_cmd,
            max_backward_cmd=args.max_backward_cmd,
            max_turn_left_cmd=args.max_turn_left_cmd,
            max_turn_right_cmd=args.max_turn_right_cmd,
            fine_calibration=fine_calibration,
            fine_calibration_json=fine_calibration_json,
            use_recommended_crawl_command=use_recommended,
            max_ms=args.max_ms,
        )
        if completed["final_left_cmd_zero"] is not True or completed["final_right_cmd_zero"] is not True:
            raise RuntimeError("Stage21 final wheel commands were nonzero; aborting")
        if completed["physical_output_active_after_stop"] is True:
            raise RuntimeError("Stage21 output remained active after STOP; aborting")
        return "\n".join(raw_lines), [completed]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Stage 21 path-package first primitive using physical A/B commands.")
    parser.add_argument("--path-package", default="latest")
    parser.add_argument("--port", required=True)
    parser.add_argument("--pose-mode", choices=("manual_local", "gps_georef"), default="gps_georef")
    parser.add_argument("--current-x", type=float)
    parser.add_argument("--current-y", type=float)
    parser.add_argument("--current-heading-deg", type=float)
    parser.add_argument("--forward-sign", type=float, default=stage20_physical_ab_probe.DEFAULT_FORWARD_SIGN)
    parser.add_argument("--turn-sign", type=float, default=stage20_physical_ab_probe.DEFAULT_TURN_SIGN)
    parser.add_argument("--calibration-json", default="")
    parser.add_argument("--fine-calibration-json", default="")
    parser.add_argument("--use-recommended-crawl-command", default="true")
    parser.add_argument("--straight-only", default="false")
    parser.add_argument("--turn-disabled", default="false")
    parser.add_argument("--min-effective-a", type=float, default=0.08)
    parser.add_argument("--min-effective-b", type=float, default=0.08)
    parser.add_argument("--max-abs-a", type=float, default=stage20_physical_ab_probe.DEFAULT_MAX_ABS_A)
    parser.add_argument("--max-abs-b", type=float, default=stage20_physical_ab_probe.DEFAULT_MAX_ABS_B)
    parser.add_argument("--max-forward-cmd", type=float)
    parser.add_argument("--max-backward-cmd", type=float)
    parser.add_argument("--max-turn-left-cmd", type=float)
    parser.add_argument("--max-turn-right-cmd", type=float)
    parser.add_argument("--max-ms", type=int, default=800)
    parser.add_argument("--pulse-ms", type=int)
    parser.add_argument("--duration-s", type=float, default=30.0)
    parser.add_argument("--require-enter", default="true")
    parser.add_argument("--require-stage20-ready", default="true")
    parser.add_argument("--dry-run", default="true")
    parser.add_argument("--out-dir", required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.max_abs_a > 0.35 or args.max_abs_b > 0.35:
        raise SystemExit("--max-abs-a/b > 0.35 is not allowed in Stage21 wrapper")
    if args.max_ms <= 0 or args.max_ms > 1000:
        raise SystemExit("--max-ms must be > 0 and <= 1000 in Stage21 wrapper")
    if args.pulse_ms is not None and args.pulse_ms > args.max_ms:
        raise SystemExit("--pulse-ms cannot exceed --max-ms in Stage21 wrapper")
    try:
        selected = resolve_path_package(args.path_package)
    except PathPackageResolutionError as exc:
        print(f"provided_path_package={exc.provided}")
        print("file_exists=false")
        print("nearest_candidates:")
        for candidate in exc.candidates:
            print(f"- {candidate}")
        return 2
    package = json.loads(selected.read_text(encoding="utf-8"))
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    try:
        raw_text, rows = _read_serial_session(args, package)
    except RuntimeError as exc:
        print(f"reason={exc}")
        print("ready_for_full_path_following=false")
        return 2
    (out_dir / "raw_usbdbg.log").write_text(raw_text + ("\n" if raw_text else ""), encoding="utf-8")
    _write_csv(out_dir / "stage21_path_physical_ab_first_primitive.csv", rows)
    _write_summary(out_dir / "summary.md", selected, rows)
    print("Stage 21 path physical A/B first primitive complete.")
    print(f"selected_path_package={selected}")
    print(f"stage21_csv={out_dir / 'stage21_path_physical_ab_first_primitive.csv'}")
    print(f"raw_usbdbg_log={out_dir / 'raw_usbdbg.log'}")
    print(f"summary_md={out_dir / 'summary.md'}")
    print("ready_for_full_path_following=false")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
