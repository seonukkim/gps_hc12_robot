"""Unified physical-path-planning CLI: one field-facing entrypoint.

    diagnose              read-only telemetry summary.
    gps-wait              wait for GPS cold/warm start.
    manual-rc             restore and validate manual RC passthrough.
    manual-control        upload and monitor PPM physical manual control.
    guarded-pulse-ready   upload/check IMU-enabled guarded pulse firmware.
    station-hw-diagnose  read-only physical station hardware link diagnostic.
    station-hw-manual    physical station hardware manual rover control.
    usb-pulse-test       laptop USB bounded A/B rover pulse test.
    calibrate-turn        run guarded pulse turn-angle calibration.
    preview               build + render the rectangle coverage plan.
    execute-plan / run    execute a planned path with guarded pulses.

Every summary is routed through ``checks.assert_not_ready_for_full_path_following``
so no mode can ever claim full-path-following readiness. The hardware modes open
serial only when actually invoked; ``--print-plan`` / ``--print-cmd`` /
``--from-log`` give fully no-hardware paths for previewing exactly what would run.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import platform
import shlex
import subprocess
import sys
import time
from urllib.parse import unquote
from pathlib import Path
from typing import Sequence

from tools.physical_path_planning import calibration, checks, controller, executor, preview, safety, telemetry, tuning

DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = 115200
DEFAULT_GUARDED_PULSE_CALIBRATION_SCRIPT = "legacy/stage_scripts/run_guarded_pulse_calibration.sh"
DEFAULT_MANUAL_RC_UPLOAD_SCRIPT = "legacy/stage_scripts/upload_manual_rc_recovery_firmware.sh"
DEFAULT_MANUAL_RC_VALIDATE_SCRIPT = "legacy/stage_scripts/run_manual_rc_passthrough_validation.sh"
DEFAULT_RC_INPUT_DIAGNOSE_SKETCH = "firmware/ppm_channel_map_probe"
DEFAULT_TURN_CALIBRATION_OUT = (
    "outputs/stage23_turn_calibration/calibration/physical_ab_turn_angle_calibration.json"
)
DEFAULT_GPS_CACHE = Path("outputs/physical_path_planning/gps_cache/latest_start.json")
RC_INPUT_ABSENT_ACTION = (
    "Check RC receiver power; check receiver signal wire to OpenRB RC input; "
    "check whether receiver output mode is PPM/SBUS/PWM and firmware input mode matches; "
    "check mode channel index / channel mapping; check transmitter-receiver binding; "
    "if using individual PWM channels instead of PPM, firmware must read the correct pins; "
    "then run manual-rc --diagnose-only true after changing wiring or binding."
)
PPM_INPUT_ABSENT_ACTION = (
    "PPM input is absent. Expected wiring: signal -> OpenRB D6; CH1 steering; "
    "CH2 throttle; CH5 mode/manual-auto switch. Check station/controller power, "
    "PPM output mode, transmitter binding, and the D6 signal wire."
)
NO_USABLE_START_GPS_ACTION = (
    "No usable current or fresh cached GPS coordinate was available for the plan start. "
    "Move outside and wait longer for GPS, or pass --start-lat and --start-lon."
)
GPS_WAIT_TIMEOUT_ACTION = (
    "GPS characters are being monitored but no usable fix was reached before timeout. "
    "Move outdoors, wait longer for cold start, or pass --start-lat and --start-lon."
)
USB_PULSE_TEST_SEQUENCE = (
    {"primitive": "forward", "a": calibration.DEFAULT_FORWARD_A_CMD, "b": 0.0, "ms": calibration.DEFAULT_FORWARD_MS},
    {"primitive": "backward", "a": calibration.DEFAULT_BACKWARD_A_CMD, "b": 0.0, "ms": calibration.DEFAULT_BACKWARD_MS},
    {"primitive": "left", "a": 0.0, "b": calibration.DEFAULT_TURN_LEFT_B_CMD, "ms": calibration.DEFAULT_TURN_LEFT_MS},
    {"primitive": "right", "a": 0.0, "b": calibration.DEFAULT_TURN_RIGHT_B_CMD, "ms": calibration.DEFAULT_TURN_RIGHT_MS},
)
USB_PULSE_TEST_ALIASES = {
    "forward": "forward",
    "backward": "backward",
    "left": "left",
    "right": "right",
    "turn_left": "left",
    "turn_right": "right",
}


# --- Pure, no-hardware helpers (directly unit-testable) -----------------------


_COMPAT_GUARDED_MODE_KEY = "stage" + "20_physical_ab_guarded_crawl"
_COMPAT_GUARDED_READY_KEY = "stage" + "20_firmware_ready"
_COMPAT_GUARDED_STATE_KEY = "stage" + "20_cmd_state"
_COMPAT_GUARDED_STATE_FALLBACK_KEY = "stage" + "16_cmd_state"


def guarded_pulse_imu_flags(*, max_abs_a: float = 0.35, max_abs_b: float = 0.35, max_ms: int = 1500) -> str:
    return (
        "-DUSB_PULSE_TEST_GUARDED=1 "
        f"-DUSB_PULSE_TEST_MAX_ABS_A={max_abs_a} "
        f"-DUSB_PULSE_TEST_MAX_ABS_B={max_abs_b} "
        f"-DUSB_PULSE_TEST_MAX_MS={max_ms} "
        "-DUSB_PULSE_TEST_IGNORE_RC_INPUT=1 "
        "-DUSB_DRIVE_LIVE_ENABLE=1 "
        "-DIMU_ENABLE=1 "
        "-DIMU_YAW_DIAG=1 "
        "-DPHYSICAL_PATH_FOLLOWING_ENABLE=0 "
        "-DPATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0 "
        "-DPATH_FOLLOWING_DRYRUN=0 "
        "-DPATH_FOLLOWING_HC12_ENABLED=0 "
        "-DGROUND_CRAWL_TEST_MODE=0 "
        "-DAUTO_MOTION_ARMED=0"
    )


def guarded_pulse_firmware_flags(
    *,
    max_abs_a: float = 0.35,
    max_abs_b: float = 0.35,
    max_ms: int = 1500,
    ignore_rc_input_for_usb_command: bool = False,
) -> str:
    flags = guarded_pulse_imu_flags(max_abs_a=max_abs_a, max_abs_b=max_abs_b, max_ms=max_ms)
    if ignore_rc_input_for_usb_command:
        flags += " -DSTATION_MANUAL_IGNORE_RC_INPUT=1"
    return flags


def usb_pulse_test_firmware_flags(
    *,
    max_abs_a: float = 0.35,
    max_abs_b: float = 0.35,
    max_ms: int = 1000,
) -> str:
    return (
        "-DUSB_PULSE_TEST_GUARDED=1 "
        "-DUSB_PULSE_TEST_IGNORE_RC_INPUT=1 "
        f"-DUSB_PULSE_TEST_MAX_ABS_A={max_abs_a} "
        f"-DUSB_PULSE_TEST_MAX_ABS_B={max_abs_b} "
        f"-DUSB_PULSE_TEST_MAX_MS={max_ms} "
        "-DPHYSICAL_PATH_FOLLOWING_ENABLE=0 "
        "-DPATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0 "
        "-DPATH_FOLLOWING_DRYRUN=0 "
        "-DPATH_FOLLOWING_HC12_ENABLED=0 "
        "-DGROUND_CRAWL_TEST_MODE=0 "
        "-DAUTO_MOTION_ARMED=0"
    )


def usb_drive_live_firmware_flags(
    *,
    max_abs_a: float = 0.35,
    max_abs_b: float = 0.35,
    max_duration_ms: int = 3000,
    update_timeout_ms: int = 350,
) -> str:
    return (
        "-DUSB_DRIVE_LIVE_ENABLE=1 "
        "-DUSB_DRIVE_LIVE_IGNORE_RC_INPUT=1 "
        f"-DUSB_DRIVE_LIVE_MAX_ABS_A={max_abs_a} "
        f"-DUSB_DRIVE_LIVE_MAX_ABS_B={max_abs_b} "
        f"-DUSB_DRIVE_LIVE_MAX_DURATION_MS={max_duration_ms} "
        f"-DUSB_DRIVE_LIVE_UPDATE_TIMEOUT_MS={update_timeout_ms} "
        "-DIMU_ENABLE=1 "
        "-DIMU_YAW_DIAG=1 "
        "-DPHYSICAL_PATH_FOLLOWING_ENABLE=0 "
        "-DPATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0 "
        "-DPATH_FOLLOWING_DRYRUN=0 "
        "-DPATH_FOLLOWING_HC12_ENABLED=0 "
        "-DGROUND_CRAWL_TEST_MODE=0 "
        "-DAUTO_MOTION_ARMED=0"
    )


def station_hw_manual_firmware_flags() -> str:
    return (
        "-DSTATION_HW_MANUAL_ENABLE=1 "
        "-DSTATION_HW_MANUAL_A_B_MAPPING=1 "
        "-DSTATION_HW_MANUAL_IGNORE_RC_INPUT=1 "
        "-DPHYSICAL_PATH_FOLLOWING_ENABLE=0 "
        "-DPATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0 "
        "-DPATH_FOLLOWING_DRYRUN=0 "
        "-DGROUND_CRAWL_TEST_MODE=0 "
        "-DAUTO_MOTION_ARMED=0"
    )


def station_hw_diagnose_firmware_flags() -> str:
    return station_hw_manual_firmware_flags() + " -DSTATION_HW_MANUAL_DIAGNOSE_ONLY=1"


def manual_rc_recovery_flags(*, mode_channel_index: int | None = None) -> str:
    flags = (
        "-DMANUAL_RC_RECOVERY=1 "
        "-DMANUAL_FORWARD_SIGN=-1 "
        "-DMANUAL_TURN_SIGN=1 "
        "-DMOTOR_OUTPUT_SWAP_LR=0 "
        "-DDRIVE_CALIBRATION_ENABLE=0 "
        "-DPHYSICAL_PATH_FOLLOWING_ENABLE=0 "
        "-DPATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0 "
        "-DPATH_FOLLOWING_DRYRUN=0 "
        "-DPATH_FOLLOWING_HC12_ENABLED=0 "
        "-DGROUND_CRAWL_TEST_MODE=0 "
        "-DAUTO_MOTION_ARMED=0"
    )
    if mode_channel_index is not None:
        flags += f" -DMODE_CHANNEL_INDEX={mode_channel_index}"
    return flags


def manual_control_firmware_flags(*, mode_channel_index: int | None = 4) -> str:
    flags = (
        "-DMANUAL_CONTROL_PPM=1 "
        "-DMANUAL_FORWARD_SIGN=-1 "
        "-DMANUAL_TURN_SIGN=1 "
        "-DMOTOR_OUTPUT_SWAP_LR=0 "
        "-DDRIVE_CALIBRATION_ENABLE=0 "
        "-DIMU_ENABLE=1 "
        "-DIMU_YAW_DIAG=1 "
        "-DPHYSICAL_PATH_FOLLOWING_ENABLE=0 "
        "-DPATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0 "
        "-DPATH_FOLLOWING_DRYRUN=0 "
        "-DPATH_FOLLOWING_HC12_ENABLED=0 "
        "-DGROUND_CRAWL_TEST_MODE=0 "
        "-DAUTO_MOTION_ARMED=0"
    )
    if mode_channel_index is not None:
        flags += f" -DMODE_CHANNEL_INDEX={mode_channel_index}"
    return flags


def manual_control_mapping(
    *,
    steer_norm: float,
    throttle_norm: float,
    forward_sign: float = -1.0,
    turn_sign: float = 1.0,
) -> dict[str, float]:
    """Return the physical A/B commands selected by the old PPM manual path.

    The PPM wiring is CH1 steering, CH2 throttle, CH5 mode. The old working
    controller used ``MANUAL_FORWARD_SIGN=-1`` and ``MANUAL_TURN_SIGN=1`` before
    the logical-wheel-to-physical A/B conversion. This helper keeps the tested
    sign contract explicit without touching path-planning logic.
    """
    physical_a = max(-1.0, min(1.0, forward_sign * throttle_norm))
    physical_b = max(-1.0, min(1.0, -turn_sign * steer_norm))
    return {"physical_a_cmd": physical_a, "physical_b_cmd": physical_b}


def _row_input_zero(row: dict[str, str]) -> bool:
    keys = [f"raw_ch{i}_us" for i in range(1, 9)] + [
        "steer_us",
        "throttle_us",
        "mode_us",
        "raw_mode_channel_us",
    ]
    present = [key for key in keys if key in row]
    if not present:
        return False
    return all(abs(telemetry._optional_float(row.get(key)) or 0.0) <= 1e-3 for key in present)


def _row_input_nonzero(row: dict[str, str]) -> bool:
    keys = [f"raw_ch{i}_us" for i in range(1, 9)] + [
        "steer_us",
        "throttle_us",
        "mode_us",
        "raw_mode_channel_us",
    ]
    return any(abs(telemetry._optional_float(row.get(key)) or 0.0) > 1e-3 for key in keys)


def _row_rc_input_detected(row: dict[str, str]) -> bool:
    return telemetry._parse_bool(row.get("rc_input_detected"), default=False) or _row_input_nonzero(row)


def manual_control_mode_decode(row: dict[str, str]) -> tuple[str, str]:
    if not row:
        return "UNKNOWN_NO_USBDBG_TELEMETRY", "NO_USBDBG_TELEMETRY"
    row_manual_switch = _row_value(row, ["manual_switch"], default="")
    row_mode_decode_reason = _row_value(row, ["mode_decode_reason"], default="")
    if row_manual_switch and row_mode_decode_reason:
        return row_manual_switch, row_mode_decode_reason
    if not _row_rc_input_detected(row):
        return "UNKNOWN_PPM_ABSENT", "PPM_INPUT_ABSENT"
    mode_us = telemetry._optional_float(row.get("mode_us") or row.get("raw_mode_channel_us"))
    if mode_us is None or mode_us <= 0.0:
        return "UNKNOWN_MODE_CHANNEL_MISSING", "NO_MODE_CHANNEL"
    if mode_us < 900.0 or mode_us > 2100.0:
        return "UNKNOWN_MODE_CHANNEL_INVALID", "MODE_CHANNEL_OUT_OF_RANGE"
    if mode_us > 1600.0:
        return "AUTO", "MODE_CHANNEL_AUTO"
    return "MANUAL", "MODE_CHANNEL_MANUAL"


def _latest_manual_control_row(rows: Sequence[dict[str, str]]) -> dict[str, str]:
    status_keys = {
        "manual_control",
        "manual_control_ppm",
        "rc_input_detected",
        "mode",
        "auto_sw",
        "rc_ok",
        "steer_us",
        "throttle_us",
        "mode_us",
        "raw_mode_channel_us",
        *[f"raw_ch{i}_us" for i in range(1, 9)],
        "ppm_interrupt_edge",
        "ppm_decode_reason",
        "ppm_frame_count",
        "ppm_last_channel_count",
        "ppm_short_rejects",
        "ppm_long_rejects",
        "ppm_last_rejected_us",
        "gps_block_reason",
        "gps_sats",
        "gps_hdop",
        "current_lat",
        "current_lon",
        "gps_cached_lat",
        "gps_cached_lon",
        "imu_present",
        "imu_relative_yaw_deg",
        "imu_heading_block_reason",
    }
    for row in reversed(rows):
        if any(key in row for key in status_keys):
            return row
    return {}


def _row_value(row: dict[str, str], keys: Sequence[str], default: str = "NA") -> str:
    for key in keys:
        value = row.get(key)
        if value is not None and str(value).strip().upper() not in {"", "NA", "NAN", "NONE", "NULL"}:
            return str(value)
    return default


def _latest_non_na(rows: Sequence[dict[str, str]], keys: Sequence[str], default: str = "NA") -> str:
    for row in reversed(rows):
        value = _row_value(row, keys, default="")
        if value:
            return value
    return default


def format_manual_control_status(*, elapsed_s: int, rows: Sequence[dict[str, str]]) -> str:
    last = _latest_manual_control_row(rows)
    manual_switch, mode_decode_reason = manual_control_mode_decode(last)
    mode_us = _row_value(last, ["mode_us", "raw_mode_channel_us"])
    current_lat = _row_value(last, ["current_lat", "gps_cached_lat", "gps_lat", "current_gps_lat"])
    current_lon = _row_value(last, ["current_lon", "gps_cached_lon", "gps_lon", "current_gps_lon"])
    fields = {
        "elapsed_s": str(elapsed_s),
        "rc_input_detected": str(_row_rc_input_detected(last)).lower() if last else "false",
        "rc_ok": last.get("rc_ok", "NA"),
        "mode": last.get("mode", "NA"),
        "auto_sw": last.get("auto_sw", "NA"),
        "manual_switch": manual_switch,
        "mode_decode_reason": mode_decode_reason,
        "ppm_interrupt_edge": last.get("ppm_interrupt_edge", "NA"),
        "ppm_decode_reason": last.get("ppm_decode_reason", "NA"),
        "ppm_frame_count": last.get("ppm_frame_count", "NA"),
        "ppm_last_channel_count": last.get("ppm_last_channel_count", "NA"),
        "ppm_short_rejects": last.get("ppm_short_rejects", "NA"),
        "ppm_long_rejects": last.get("ppm_long_rejects", "NA"),
        "ppm_last_rejected_us": last.get("ppm_last_rejected_us", "NA"),
        "steer_us": last.get("steer_us", "NA"),
        "throttle_us": last.get("throttle_us", "NA"),
        "mode_us": mode_us,
        "physical_a_cmd": last.get("physical_a_cmd", "NA"),
        "physical_b_cmd": last.get("physical_b_cmd", "NA"),
        "control_source": last.get("control_source", "NA"),
        "motor_write_called": last.get("motor_write_called", "NA"),
        "physical_output_active": last.get("physical_output_active", "NA"),
        "final_left_cmd": last.get("final_left_cmd", "NA"),
        "final_right_cmd": last.get("final_right_cmd", "NA"),
        "gps_block_reason": last.get("gps_block_reason", "NA"),
        "current_lat": current_lat,
        "current_lon": current_lon,
        "gps_sats": last.get("gps_sats", "NA"),
        "gps_hdop": last.get("gps_hdop", "NA"),
        "imu_present": last.get("imu_present", "NA"),
        "imu_relative_yaw_deg": last.get("imu_relative_yaw_deg", "NA"),
        "imu_heading_block_reason": last.get("imu_heading_block_reason", "NA"),
    }
    return " ".join(f"{key}={value}" for key, value in fields.items())


def evaluate_manual_control_rows(rows: Sequence[dict[str, str]]) -> dict[str, object]:
    rows_with_input = [
        row for row in rows
        if any(
            key in row
            for key in [f"raw_ch{i}_us" for i in range(1, 9)]
            + ["steer_us", "throttle_us", "mode_us", "raw_mode_channel_us"]
        )
    ]
    zero_rows = [row for row in rows_with_input if _row_input_zero(row)]
    rc_input_detected = any(_row_rc_input_detected(row) for row in rows)
    input_absent = bool(rows_with_input) and len(zero_rows) / max(1, len(rows_with_input)) >= 0.8 and not rc_input_detected
    rc_ok_seen = any(telemetry._parse_bool(row.get("rc_ok")) is True for row in rows)
    manual_mode_seen = any(row.get("mode") == "MANUAL" for row in rows)
    rc_manual_seen = any(row.get("control_source") == "RC_MANUAL" for row in rows)
    physical_a_nonzero = any(abs(telemetry._optional_float(row.get("physical_a_cmd")) or 0.0) > 1e-3 for row in rows)
    physical_b_nonzero = any(abs(telemetry._optional_float(row.get("physical_b_cmd")) or 0.0) > 1e-3 for row in rows)
    final_motor_nonzero = any(
        abs(telemetry._optional_float(row.get("final_left_cmd")) or 0.0) > 1e-3
        or abs(telemetry._optional_float(row.get("final_right_cmd")) or 0.0) > 1e-3
        for row in rows
    )
    motor_write_seen = any(telemetry._parse_bool(row.get("motor_write_called")) is True for row in rows)
    physical_output_seen = any(telemetry.physical_output_active(row) for row in rows)
    rc_status_rows = [
        row for row in rows
        if telemetry._parse_bool(row.get("rc_ok")) is not None
    ]
    rc_ok_rows = sum(1 for row in rc_status_rows if telemetry._parse_bool(row.get("rc_ok")) is True)
    rc_bad_rows = sum(1 for row in rc_status_rows if telemetry._parse_bool(row.get("rc_ok")) is False)
    rc_ok_ratio = (rc_ok_rows / len(rc_status_rows)) if rc_status_rows else 0.0
    ppm_signal_stable = not rc_status_rows or len(rc_status_rows) < 3 or rc_ok_ratio >= 0.6
    pass_ready = (
        rc_ok_seen
        and manual_mode_seen
        and rc_manual_seen
        and final_motor_nonzero
        and (motor_write_seen or physical_output_seen)
        and ppm_signal_stable
    )
    last = _latest_manual_control_row(rows)
    manual_switch, mode_decode_reason = manual_control_mode_decode(last)
    ppm_decode_reason_latest = _latest_non_na(rows, ["ppm_decode_reason"])
    ppm_invalid_decode_seen = any(
        str(row.get("ppm_decode_reason", "")).startswith("PPM_")
        and row.get("ppm_decode_reason") not in {"OK", "PPM_FRAME_STALE"}
        for row in rows
    )
    gps_status_available = _latest_non_na(
        rows,
        ["gps_block_reason", "gps_sats", "gps_hdop", "current_lat", "current_lon", "gps_cached_lat", "gps_cached_lon"],
    ) != "NA"
    imu_status_available = _latest_non_na(
        rows,
        ["imu_present", "imu_relative_yaw_deg", "imu_heading_block_reason"],
    ) != "NA"
    if pass_ready:
        reason = "MANUAL_CONTROL_PASS"
    elif not rows or not last:
        reason = "NO_USBDBG_TELEMETRY"
    elif rc_ok_seen and not ppm_signal_stable:
        reason = "PPM_CHANNELS_PRESENT_BUT_INVALID"
    elif ppm_invalid_decode_seen and not rc_ok_seen:
        reason = "PPM_CHANNELS_PRESENT_BUT_INVALID"
    elif input_absent:
        reason = "PPM_INPUT_ABSENT"
    elif mode_decode_reason == "NO_MODE_CHANNEL":
        reason = "MODE_CHANNEL_MISSING"
    elif rc_input_detected and not rc_ok_seen:
        reason = "PPM_CHANNELS_PRESENT_BUT_INVALID"
    elif rc_ok_seen and not manual_mode_seen:
        reason = "MANUAL_CONTROL_READY"
    elif (physical_a_nonzero or physical_b_nonzero) and not final_motor_nonzero:
        reason = "MOTOR_OUTPUT_BLOCKED"
    else:
        reason = "MANUAL_CONTROL_READY"
    next_action = {
        "MANUAL_CONTROL_PASS": "PPM manual control is verified.",
        "MANUAL_CONTROL_READY": "PPM telemetry is present; set CH5 to MANUAL and move the sticks to verify output.",
        "PPM_INPUT_ABSENT": PPM_INPUT_ABSENT_ACTION,
        "PPM_CHANNELS_PRESENT_BUT_INVALID": "PPM is present but unstable or invalid; charge/check the station controller battery, receiver power, D6 signal wire, shared ground, channel order, and pulse widths.",
        "MODE_CHANNEL_MISSING": "PPM steering/throttle are present but CH5 mode is missing; verify the receiver mode channel.",
        "MOTOR_OUTPUT_BLOCKED": "Manual A/B commands changed, but final motor output stayed zero; inspect manual control priority and motor gating.",
        "NO_USBDBG_TELEMETRY": "No USBDBG rows were parsed; check USB serial, firmware mode, baud rate, and --verbose-raw output.",
    }.get(reason, "Move the physical station/controller during the monitor window and inspect summary telemetry.")
    return {
        "mode": "manual-control",
        "success": pass_ready,
        "manual_control_ok": pass_ready,
        "reason": reason,
        "manual_switch": manual_switch,
        "mode_decode_reason": mode_decode_reason,
        "ppm_decode_reason_latest": ppm_decode_reason_latest,
        "mode_us_latest": _latest_non_na(rows, ["mode_us", "raw_mode_channel_us"]),
        "rc_input_detected": rc_input_detected,
        "rc_ok_rows": rc_ok_rows,
        "rc_bad_rows": rc_bad_rows,
        "rc_ok_ratio": round(rc_ok_ratio, 3),
        "ppm_signal_stable": ppm_signal_stable,
        "ppm_input_pin": "D6",
        "steer_channel": "CH1",
        "throttle_channel": "CH2",
        "mode_channel": "CH5",
        "rc_ok_seen": rc_ok_seen,
        "gps_status_available": gps_status_available,
        "imu_status_available": imu_status_available,
        "last_current_lat": _latest_non_na(rows, ["current_lat", "gps_cached_lat", "gps_lat", "current_gps_lat"]),
        "last_current_lon": _latest_non_na(rows, ["current_lon", "gps_cached_lon", "gps_lon", "current_gps_lon"]),
        "last_imu_yaw": _latest_non_na(rows, ["imu_relative_yaw_deg"]),
        "manual_mode_seen": manual_mode_seen,
        "control_source_rc_manual_seen": rc_manual_seen,
        "physical_a_nonzero_seen": physical_a_nonzero,
        "physical_b_nonzero_seen": physical_b_nonzero,
        "final_motor_nonzero_seen": final_motor_nonzero,
        "motor_write_called_seen": motor_write_seen,
        "physical_output_active_seen": physical_output_seen,
        "gps_required": False,
        "imu_required": False,
        "path_package_required": False,
        "station_frame_parser_required": False,
        "hc12_required": False,
        "physical_a_role": "throttle",
        "physical_b_role": "turn",
        "wheel_to_physical_mapping": "physical_ab_manual_equivalent",
        "next_recommended_action": next_action,
        "ready_for_full_path_following": False,
    }


def evaluate_rc_input_diagnose_rows(rows: Sequence[dict[str, str]]) -> dict[str, object]:
    frame_counts = [int(float(row.get("frames", "0") or 0)) for row in rows if "frames" in row]
    invalid_counts = [int(float(row.get("invalid_frames", "0") or 0)) for row in rows if "invalid_frames" in row]
    total_frames = sum(frame_counts)
    total_invalid = sum(invalid_counts)
    ppm_header_seen = any("ppm_pin" in row or "channel_count" in row for row in rows)
    event_frames = [
        row for row in rows
        if any(f"ch{i}_us" in row for i in range(1, 9))
    ]
    nonzero_channels = [
        (key, value)
        for row in rows
        for key, value in row.items()
        if key.startswith("ch") and key.endswith("_us")
        if telemetry._optional_float(value) is not None and abs(float(value)) > 1e-3
    ]
    any_ppm_signal = total_frames > 0 or bool(nonzero_channels)
    valid_ppm_signal = any_ppm_signal and total_invalid < max(1, total_frames)
    if not rows:
        reason = "SERIAL_ERROR"
    elif not any_ppm_signal:
        reason = "RC_INPUT_ABSENT"
    elif not valid_ppm_signal:
        reason = "RC_CHANNELS_PRESENT_BUT_INVALID"
    else:
        reason = "RC_CHANNELS_PRESENT_AND_VALID"
    signal_class = "RC_INPUT_PRESENT_PPM" if any_ppm_signal else "RC_INPUT_ABSENT"
    next_action = {
        "SERIAL_ERROR": "Check USB serial connection and rerun rc-input-diagnose.",
        "RC_INPUT_ABSENT": RC_INPUT_ABSENT_ACTION,
        "RC_CHANNELS_PRESENT_BUT_INVALID": (
            "PPM frames were seen but invalid or incomplete; check receiver output mode, "
            "signal wiring, and whether the receiver is configured for PPM on the OpenRB input pin."
        ),
        "RC_CHANNELS_PRESENT_AND_VALID": (
            "RC input frames are present. Run manual-rc and verify MANUAL mode stick passthrough."
        ),
    }.get(reason, "Inspect RC input telemetry.")
    return {
        "mode": "rc-input-diagnose",
        "success": reason == "RC_CHANNELS_PRESENT_AND_VALID",
        "reason": reason,
        "rc_input_classification": reason,
        "rc_input_signal_class": signal_class,
        "rc_input_detected": any_ppm_signal,
        "rc_input_present_ppm": any_ppm_signal,
        "rc_input_present_pwm": False,
        "rc_input_present_sbus": False,
        "ppm_header_seen": ppm_header_seen,
        "ppm_event_row_count": len(event_frames),
        "ppm_total_frames": total_frames,
        "ppm_invalid_frames": total_invalid,
        "raw_channel_nonzero_seen": bool(nonzero_channels),
        "next_recommended_action": next_action,
        "ready_for_full_path_following": False,
    }


def _station_value_present(value: object) -> bool:
    text = str(value or "").strip().upper()
    return text not in {"", "NA", "NAN", "NONE", "NULL"}


def _station_hw_link_row(row: dict[str, str]) -> bool:
    if telemetry._parse_bool(row.get("station_link_seen")) is True:
        return True
    if _station_value_present(row.get("station_seq")) or _station_value_present(row.get("station_age_ms")):
        return True
    rx_count = telemetry._optional_float(row.get("station_rx_count"))
    if rx_count is None:
        rx_count = telemetry._optional_float(row.get("hc12_rx_count"))
    return rx_count is not None and rx_count > 0


def _station_hw_float_seen(rows: Sequence[dict[str, str]], *keys: str) -> bool:
    for row in rows:
        for key in keys:
            value = telemetry._optional_float(row.get(key))
            if value is not None and abs(value) > 1e-6:
                return True
    return False


def _station_last_present(last: dict[str, str], key: str, default: object = "NA") -> object:
    value = last.get(key)
    return value if _station_value_present(value) else default


def evaluate_station_hw_rows(rows: Sequence[dict[str, str]], *, mode: str) -> dict[str, object]:
    link_rows = [row for row in rows if _station_hw_link_row(row)]
    manual_valid_rows = [row for row in rows if telemetry._parse_bool(row.get("station_manual_valid")) is True]
    deadman_rows = [row for row in rows if telemetry._parse_bool(row.get("station_deadman")) is True]
    estop_rows = [row for row in rows if telemetry._parse_bool(row.get("station_estop")) is True]
    motor_rows = [
        row for row in rows
        if telemetry._parse_bool(row.get("motor_write_called")) is True
        or telemetry.physical_output_active(row)
        or abs(telemetry._optional_float(row.get("final_left_cmd")) or 0.0) > 1e-6
        or abs(telemetry._optional_float(row.get("final_right_cmd")) or 0.0) > 1e-6
    ]
    parsed_ok_rows = sum(
        1 for row in rows
        if telemetry._parse_bool(row.get("station_parse_ok")) is True
        or telemetry._parse_bool(row.get("station_manual_valid")) is True
    )
    parsed_error_rows = sum(
        1 for row in rows
        if telemetry._parse_bool(row.get("station_parse_error")) is True
    )
    last = link_rows[-1] if link_rows else (rows[-1] if rows else {})
    parse_ok_count = max(
        parsed_ok_rows,
        int(telemetry._optional_float(last.get("station_parse_ok_count")) or 0),
    )
    parse_error_count = max(
        parsed_error_rows,
        int(telemetry._optional_float(last.get("station_parse_error_count")) or 0),
    )
    station_frame_count = max(
        len(link_rows),
        int(telemetry._optional_float(last.get("station_frame_count")) or 0),
        int(telemetry._optional_float(last.get("station_rx_count")) or 0),
        int(telemetry._optional_float(last.get("hc12_rx_count")) or 0),
    )
    station_link_seen = station_frame_count > 0
    station_physical_a_nonzero_seen = _station_hw_float_seen(
        rows, "station_physical_a_cmd", "station_a_cmd", "station_forward_cmd"
    )
    station_physical_b_nonzero_seen = _station_hw_float_seen(
        rows, "station_physical_b_cmd", "station_b_cmd", "station_turn_cmd"
    )
    if not station_link_seen:
        reason = "STATION_HW_LINK_ABSENT"
        success = False
        next_action = (
            "Check station hardware power, station transport wiring, station baud/settings, "
            "and whether the rover firmware has station hardware manual mode enabled."
        )
    elif estop_rows:
        reason = "STATION_HW_ESTOP_ACTIVE"
        success = False
        next_action = "Release station hardware emergency stop and rerun station-hw-diagnose."
    elif link_rows and not manual_valid_rows:
        reason = "WRONG_STATION_FRAME_PARSER"
        success = False
        next_action = (
            "Station bytes are arriving but no station manual frame parsed. Inspect "
            "raw_station_frames.txt and raw_station_frames_hex.txt, then compare the "
            "physical station output against the rover station parser."
        )
    elif not deadman_rows:
        reason = "STATION_HW_DEADMAN_NOT_ACTIVE"
        success = False
        next_action = "Hold the station hardware deadman control while moving the station input."
    elif motor_rows:
        reason = "STATION_HW_MANUAL_PASS"
        success = True
        next_action = "Station hardware manual control is passing; continue only with bounded supervised tests."
    elif station_physical_a_nonzero_seen or station_physical_b_nonzero_seen:
        if mode == "station-hw-manual":
            reason = "STATION_HW_MANUAL_OUTPUT_BLOCKED"
            success = False
            next_action = (
                "Station hardware A/B commands changed but rover motor output did not respond; "
                "compare against usb-pulse-test, then inspect station manual control-source gating."
            )
        else:
            reason = "STATION_HW_MANUAL_READY"
            success = True
            next_action = "Station hardware commands are valid. Run station-hw-manual to verify motor output if needed."
    else:
        reason = "STATION_HW_MANUAL_VALID"
        success = mode == "station-hw-diagnose"
        next_action = "Station frames are valid. Move the station hardware input while holding deadman to verify A/B commands."
    return checks.assert_not_ready_for_full_path_following({
        "mode": mode,
        "success": success,
        "reason": reason,
        "station_hw_result": reason,
        "station_link_seen": station_link_seen,
        "station_frame_count": station_frame_count,
        "station_parse_ok_count": parse_ok_count,
        "station_parse_error_count": parse_error_count,
        "station_transport": _station_last_present(last, "station_transport", "station_hardware_serial"),
        "station_protocol": _station_last_present(last, "station_protocol", "auto"),
        "station_parser": _station_last_present(last, "station_parser", "auto_station_manual"),
        "station_last_frame_age_ms": last.get("station_age_ms", "NA"),
        "station_seq": last.get("station_seq", "NA"),
        "station_manual_valid": bool(manual_valid_rows),
        "station_manual_valid_seen": bool(manual_valid_rows),
        "station_deadman": bool(deadman_rows),
        "station_deadman_seen": bool(deadman_rows),
        "station_estop": bool(estop_rows),
        "station_estop_seen": bool(estop_rows),
        "station_a_cmd": last.get("station_a_cmd", last.get("station_physical_a_cmd", "NA")),
        "station_b_cmd": last.get("station_b_cmd", last.get("station_physical_b_cmd", "NA")),
        "station_forward_cmd": last.get("station_forward_cmd", "NA"),
        "station_turn_cmd": last.get("station_turn_cmd", "NA"),
        "station_physical_a_cmd": last.get("station_physical_a_cmd", "NA"),
        "station_physical_b_cmd": last.get("station_physical_b_cmd", "NA"),
        "station_physical_a_nonzero_seen": station_physical_a_nonzero_seen,
        "station_physical_b_nonzero_seen": station_physical_b_nonzero_seen,
        "active_control_source_candidate": "STATION_HW_MANUAL" if bool(manual_valid_rows) else "STOP",
        "station_rx_count": last.get("station_rx_count", last.get("hc12_rx_count", station_frame_count)),
        "motor_write_called_seen": any(telemetry._parse_bool(row.get("motor_write_called")) is True for row in rows),
        "physical_output_active_seen": any(telemetry.physical_output_active(row) for row in rows),
        "final_motor_nonzero_seen": any(
            abs(telemetry._optional_float(row.get("final_left_cmd")) or 0.0) > 1e-6
            or abs(telemetry._optional_float(row.get("final_right_cmd")) or 0.0) > 1e-6
            for row in rows
        ),
        "rc_input_required": False,
        "gps_required": False,
        "imu_required": False,
        "physical_a_role": "throttle",
        "physical_b_role": "turn",
        "wheel_to_physical_mapping": "physical_ab_manual_equivalent",
        "next_recommended_action": next_action,
        "ready_for_full_path_following": False,
    })


def _station_hw_status_line(summary: dict[str, object], *, elapsed_s: float) -> str:
    return (
        f"elapsed_s={elapsed_s:.0f} "
        f"station_link_seen={str(summary.get('station_link_seen', False)).lower()} "
        f"station_frame_count={summary.get('station_frame_count', 0)} "
        f"station_parse_ok_count={summary.get('station_parse_ok_count', 0)} "
        f"station_parse_error_count={summary.get('station_parse_error_count', 0)} "
        f"station_deadman={str(summary.get('station_deadman', False)).lower()} "
        f"station_estop={str(summary.get('station_estop', False)).lower()} "
        f"station_manual_valid={str(summary.get('station_manual_valid', False)).lower()} "
        f"station_physical_a_cmd={summary.get('station_physical_a_cmd', 'NA')} "
        f"station_physical_b_cmd={summary.get('station_physical_b_cmd', 'NA')} "
        f"station_rx_count={summary.get('station_rx_count', 'NA')} "
        f"station_transport={summary.get('station_transport', 'NA')} "
        f"station_parser={summary.get('station_parser', 'NA')} "
        f"motor_write_called={str(summary.get('motor_write_called_seen', False)).lower()} "
        f"physical_output_active={str(summary.get('physical_output_active_seen', False)).lower()} "
        f"reason_so_far={summary.get('reason', 'NA')}"
    )


def station_hw_status_line(rows: Sequence[dict[str, str]], *, mode: str, elapsed_s: float) -> str:
    return _station_hw_status_line(evaluate_station_hw_rows(rows, mode=mode), elapsed_s=elapsed_s)


def _station_raw_frame_dumps(rows: Sequence[dict[str, str]]) -> list[tuple[str, str]]:
    dumps: list[tuple[str, str]] = []
    seen: set[tuple[str, str]] = set()
    for row in rows:
        raw_text = row.get("station_raw_frame")
        raw_hex = row.get("station_raw_frame_hex")
        if not _station_value_present(raw_text) and not _station_value_present(raw_hex):
            continue
        decoded = unquote(str(raw_text or ""))
        hex_text = str(raw_hex or "")
        key = (decoded, hex_text)
        if key in seen:
            continue
        seen.add(key)
        dumps.append(key)
        if len(dumps) >= 20:
            break
    return dumps


def write_station_raw_frame_dumps(out_dir: Path, rows: Sequence[dict[str, str]]) -> int:
    dumps = _station_raw_frame_dumps(rows)
    if not dumps:
        return 0
    (out_dir / "raw_station_frames.txt").write_text(
        "\n".join(raw for raw, _ in dumps) + "\n",
        encoding="utf-8",
    )
    (out_dir / "raw_station_frames_hex.txt").write_text(
        "\n".join(raw_hex for _, raw_hex in dumps) + "\n",
        encoding="utf-8",
    )
    return len(dumps)


def arduino_cli_openrb_port() -> str | None:
    try:
        completed = subprocess.run(
            ["arduino-cli", "board", "list"],
            check=False,
            capture_output=True,
            text=True,
            timeout=5,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    for line in completed.stdout.splitlines():
        if "OpenRB-150" in line:
            parts = line.split()
            return parts[0] if parts else None
    return None


def detected_serial_ports() -> list[str]:
    parent = Path("/dev")
    if not parent.exists():
        return []
    patterns = (
        "cu.usbmodem*",
        "tty.usbmodem*",
        "ttyACM*",
        "ttyUSB*",
        "cu.usbserial*",
        "tty.usbserial*",
    )
    ports: list[str] = []
    for pattern in patterns:
        ports.extend(str(path) for path in parent.glob(pattern))
    return sorted(set(ports))


def resolve_port(
    explicit_port: str | None,
    *,
    env: dict[str, str] | None = None,
    system_name: str | None = None,
) -> dict[str, object]:
    env = os.environ if env is None else env
    system_name = platform.system() if system_name is None else system_name
    if explicit_port:
        return {"port": explicit_port, "source": "explicit"}
    env_port = env.get("PORT", "")
    if env_port:
        return {"port": env_port, "source": "env"}
    detected = arduino_cli_openrb_port()
    if detected:
        return {"port": detected, "source": "arduino_cli"}
    if system_name == "Linux" and Path(DEFAULT_PORT).exists():
        return {"port": DEFAULT_PORT, "source": "linux_default"}
    return {"port": None, "source": "none"}


def write_summary_files(out_dir: str | Path, summary: dict[str, object], *, title: str) -> dict[str, object]:
    path = Path(out_dir)
    path.mkdir(parents=True, exist_ok=True)
    normalized = dict(summary)
    normalized.setdefault("success", normalized.get("reason") == "OK")
    normalized["ready_for_full_path_following"] = False
    normalized = checks.assert_not_ready_for_full_path_following(normalized)
    _write_json(path / "summary.json", normalized)
    lines = [f"# {title}", ""]
    lines.extend(f"- {key}: `{value}`" for key, value in normalized.items())
    (path / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return normalized


def write_failure_summary(
    out_dir: str | Path | None,
    *,
    reason: str,
    attempted_port: str | None = None,
    mode: str | None = None,
    next_recommended_action: str = "Check the requested serial port or connect OpenRB-150 and retry.",
) -> None:
    if out_dir is None:
        return
    path = Path(out_dir)
    path.mkdir(parents=True, exist_ok=True)
    payload = {
        "mode": mode or "unknown",
        "reason": reason,
        "success": False,
        "attempted_port": attempted_port,
        "detected_ports": detected_serial_ports(),
        "next_recommended_action": next_recommended_action,
        "ready_for_full_path_following": False,
    }
    write_summary_files(path, payload, title="Physical Path Planner")


def ensure_port(args: argparse.Namespace) -> bool:
    if getattr(args, "from_log", None):
        return True
    resolved = resolve_port(getattr(args, "port", None))
    args.port = resolved["port"]
    args.port_source = resolved["source"]
    if args.port is None or not Path(str(args.port)).exists():
        write_failure_summary(
            getattr(args, "out_dir", None),
            reason="SERIAL_PORT_NOT_FOUND",
            attempted_port=None if args.port is None else str(args.port),
            mode=getattr(args, "mode", None),
        )
        print(f"reason=SERIAL_PORT_NOT_FOUND attempted_port={args.port} detected_ports={detected_serial_ports()}")
        print("ready_for_full_path_following=false")
        return False
    print(f"resolved_port={args.port}")
    print(f"port_source={args.port_source}")
    return True


def printable_port(explicit_port: str | None) -> str:
    if explicit_port:
        return explicit_port
    resolved = resolve_port(None)
    return str(resolved["port"] or "$PORT")


def build_calibrate_turn_argv(
    *,
    script: str,
    port: str,
    mode: str,
    target_angle_deg: float,
    angle_tolerance_deg: float,
    save_turn_calibration: str,
    turn_calibration_out: str,
    out_dir: str,
    b_cmd: float | None = None,
    pulse_ms: int | None = None,
    max_abs_b: float = 0.35,
    max_ms: int = 1500,
) -> list[str]:
    """Build the argv that shells out to guarded pulse calibration.

    Always passes ``--imu-angle-compare true`` -- that is what makes the launcher
    append ``-DIMU_ENABLE=1 -DIMU_YAW_DIAG=1`` and measure before/after yaw.
    """
    argv = [
        "bash",
        str(script),
        "--port",
        str(port),
        "--mode",
        str(mode),
        "--max-abs-b",
        str(max_abs_b),
        "--max-ms",
        str(max_ms),
        "--imu-angle-compare",
        "true",
        "--target-angle-deg",
        str(target_angle_deg),
        "--angle-tolerance-deg",
        str(angle_tolerance_deg),
        "--save-turn-calibration",
        str(save_turn_calibration),
        "--turn-calibration-out",
        str(turn_calibration_out),
        "--out-dir",
        str(out_dir),
    ]
    if b_cmd is not None:
        argv.extend(["--cmd-list", str(abs(b_cmd))])
    if pulse_ms is not None:
        argv.extend(["--pulse-ms-list", str(pulse_ms)])
    return argv


def resolve_calibration(args: argparse.Namespace) -> dict[str, object]:
    """Resolve calibration honoring real on-disk files, with explicit overrides.

    Unspecified ``--*-calibration-json`` flags fall back to the resolver's default
    on-disk paths (so genuine calibration is used when present); a missing file
    degrades to the repeated-pulses fallback and never raises.
    """
    kwargs: dict[str, object] = {"calibration_mode": args.calibration_mode}
    for flag, key in (
        ("motion_calibration_json", "motion_calibration_json"),
        ("fine_calibration_json", "fine_calibration_json"),
        ("turn_calibration_json", "turn_calibration_json"),
        ("turn_angle_calibration_json", "turn_angle_calibration_json"),
        ("smooth_turn_calibration_json", "smooth_turn_calibration_json"),
    ):
        value = getattr(args, flag)
        if value is not None:
            kwargs[key] = Path(value)
    return calibration.resolve_physical_calibration(**kwargs)


def motion_calibration_loaded(calibration_dict: dict[str, object]) -> bool:
    files = calibration_dict.get("calibration_files")
    if not isinstance(files, dict):
        return False
    motion_path = files.get("motion")
    return bool(motion_path and Path(str(motion_path)).exists())


def _lat_lon_from_row(row: dict[str, str], lat_keys: Sequence[str], lon_keys: Sequence[str]) -> tuple[float | None, float | None]:
    lat = None
    lon = None
    for key in lat_keys:
        lat = telemetry._optional_float(row.get(key))
        if lat is not None:
            break
    for key in lon_keys:
        lon = telemetry._optional_float(row.get(key))
        if lon is not None:
            break
    return lat, lon


def _fresh_cached_gps(row: dict[str, str], max_age_ms: int) -> bool:
    age = telemetry._optional_float(row.get("gps_cached_age_ms", row.get("gps_age_ms")))
    if age is not None:
        return age <= max_age_ms
    return telemetry._parse_bool(row.get("gps_location_fresh"), default=False)


def gps_snapshot(rows: Sequence[dict[str, str]], *, min_sats: float = 5.0, max_hdop: float = 2.5) -> dict[str, object]:
    """Summarize cold-start GPS state from parsed telemetry rows."""
    best_sats: float | None = None
    best_hdop: float | None = None
    best_lat: float | None = None
    best_lon: float | None = None
    best_ready_row: dict[str, str] | None = None
    last = rows[-1] if rows else {}
    for row in rows:
        sats = telemetry._optional_float(row.get("gps_sats"))
        hdop = telemetry._optional_float(row.get("gps_hdop"))
        lat, lon = _lat_lon_from_row(
            row,
            ("current_lat", "gps_lat", "current_gps_lat"),
            ("current_lon", "gps_lon", "current_gps_lon"),
        )
        if sats is not None and (best_sats is None or sats > best_sats):
            best_sats = sats
        if hdop is not None and (best_hdop is None or hdop < best_hdop):
            best_hdop = hdop
        if lat is not None and lon is not None:
            best_lat = lat
            best_lon = lon
            if (
                (sats is None or sats >= min_sats)
                and (hdop is None or hdop <= max_hdop)
                and (
                    telemetry._parse_bool(row.get("gps_ready"))
                    or telemetry._parse_bool(row.get("gps_solution_valid"))
                    or str(row.get("gps_block_reason", "")).upper() == "OK"
                )
            ):
                best_ready_row = row
    current_lat, current_lon = _lat_lon_from_row(
        last,
        ("current_lat", "gps_lat", "current_gps_lat"),
        ("current_lon", "gps_lon", "current_gps_lon"),
    )
    sats = telemetry._optional_float(last.get("gps_sats"))
    hdop = telemetry._optional_float(last.get("gps_hdop"))
    gps_ready = best_ready_row is not None
    return {
        "gps_ready": gps_ready,
        "gps_solution_valid": telemetry._parse_bool(last.get("gps_solution_valid")),
        "gps_chars": last.get("gps_chars", "NA"),
        "current_lat": current_lat,
        "current_lon": current_lon,
        "gps_sats": sats,
        "gps_hdop": hdop,
        "best_sats": best_sats,
        "best_hdop": best_hdop,
        "best_lat": best_lat,
        "best_lon": best_lon,
        "last_rmc_status": last.get("last_rmc_status", "NA"),
        "last_gga_fix_quality": last.get("last_gga_fix_quality", "NA"),
        "gps_block_reason": last.get("gps_block_reason", "NA"),
        "imu_present": telemetry._parse_bool(last.get("imu_present")),
        "imu_relative_yaw_deg": last.get("imu_relative_yaw_deg", "NA"),
        "ready_row": best_ready_row,
    }


def _gps_status_line(elapsed_s: float, snapshot: dict[str, object]) -> str:
    return (
        f"elapsed_s={elapsed_s:.0f} "
        f"gps_chars={snapshot['gps_chars']} "
        f"gps_ready={str(snapshot['gps_ready']).lower()} "
        f"gps_solution_valid={str(snapshot['gps_solution_valid']).lower()} "
        f"current_lat={telemetry._fmt(snapshot['current_lat'], 7) if snapshot['current_lat'] is not None else 'NA'} "
        f"current_lon={telemetry._fmt(snapshot['current_lon'], 7) if snapshot['current_lon'] is not None else 'NA'} "
        f"gps_sats={telemetry._fmt(snapshot['gps_sats'], 0) if snapshot['gps_sats'] is not None else 'NA'} "
        f"gps_hdop={telemetry._fmt(snapshot['gps_hdop'], 2) if snapshot['gps_hdop'] is not None else 'NA'} "
        f"last_rmc_status={snapshot['last_rmc_status']} "
        f"last_gga_fix_quality={snapshot['last_gga_fix_quality']} "
        f"best_sats={telemetry._fmt(snapshot['best_sats'], 0) if snapshot['best_sats'] is not None else 'NA'} "
        f"best_hdop={telemetry._fmt(snapshot['best_hdop'], 2) if snapshot['best_hdop'] is not None else 'NA'} "
        f"best_lat={telemetry._fmt(snapshot['best_lat'], 7) if snapshot['best_lat'] is not None else 'NA'} "
        f"best_lon={telemetry._fmt(snapshot['best_lon'], 7) if snapshot['best_lon'] is not None else 'NA'} "
        f"imu_present={str(snapshot['imu_present']).lower()} "
        f"imu_relative_yaw_deg={snapshot['imu_relative_yaw_deg']}"
    )


def write_gps_cache(snapshot: dict[str, object]) -> None:
    lat = snapshot.get("best_lat")
    lon = snapshot.get("best_lon")
    if lat is None or lon is None:
        return
    DEFAULT_GPS_CACHE.parent.mkdir(parents=True, exist_ok=True)
    _write_json(
        DEFAULT_GPS_CACHE,
        {
            "start_lat": lat,
            "start_lon": lon,
            "timestamp_s": time.time(),
            "gps_sats": snapshot.get("best_sats"),
            "gps_hdop": snapshot.get("best_hdop"),
            "source": "gps-wait",
            "ready_for_full_path_following": False,
        },
    )


def load_cached_start(max_age_s: float) -> dict[str, object] | None:
    if not DEFAULT_GPS_CACHE.exists():
        return None
    try:
        data = json.loads(DEFAULT_GPS_CACHE.read_text())
    except (OSError, json.JSONDecodeError):
        return None
    timestamp_s = telemetry._optional_float(data.get("timestamp_s"))
    if timestamp_s is None or time.time() - timestamp_s > max_age_s:
        return None
    lat = telemetry._optional_float(data.get("start_lat"))
    lon = telemetry._optional_float(data.get("start_lon"))
    if lat is None or lon is None:
        return None
    return {
        "start_lat": lat,
        "start_lon": lon,
        "start_source": "cached_gps",
        "start_gps_block_reason": "CACHE",
        "start_gps_sats": data.get("gps_sats", "NA"),
        "start_gps_hdop": data.get("gps_hdop", "NA"),
        "gps_cached_used": True,
        "gps_wait_snapshot": {
            "gps_ready": True,
            "gps_solution_valid": True,
            "current_lat": lat,
            "current_lon": lon,
            "gps_sats": data.get("gps_sats"),
            "gps_hdop": data.get("gps_hdop"),
            "best_sats": data.get("gps_sats"),
            "best_hdop": data.get("gps_hdop"),
            "best_lat": lat,
            "best_lon": lon,
            "last_rmc_status": data.get("last_rmc_status", "CACHE"),
            "last_gga_fix_quality": data.get("last_gga_fix_quality", "CACHE"),
            "imu_present": False,
            "imu_relative_yaw_deg": "NA",
        },
    }


def resolve_start_gps_from_rows(
    rows: Sequence[dict[str, str]],
    *,
    start_mode: str,
    cached_start_max_age_ms: int,
    min_sats: float = 5.0,
    max_hdop: float = 2.5,
) -> dict[str, object] | None:
    """Resolve the preview start coordinate from live/current or fresh cached GPS."""
    if start_mode not in {"live_gps", "cached_gps"}:
        return None
    snapshot = gps_snapshot(rows, min_sats=min_sats, max_hdop=max_hdop)
    ready_row = snapshot.get("ready_row")
    if start_mode == "live_gps" and isinstance(ready_row, dict):
        lat, lon = _lat_lon_from_row(
            ready_row,
            ("current_lat", "gps_lat", "current_gps_lat"),
            ("current_lon", "gps_lon", "current_gps_lon"),
        )
        if lat is not None and lon is not None:
            return {
                "start_lat": lat,
                "start_lon": lon,
                "start_source": "live_gps",
                "start_gps_block_reason": ready_row.get("gps_block_reason", "NA"),
                "start_gps_sats": ready_row.get("gps_sats", "NA"),
                "start_gps_hdop": ready_row.get("gps_hdop", "NA"),
                "gps_cached_used": False,
                "gps_wait_snapshot": {k: v for k, v in snapshot.items() if k != "ready_row"},
            }
    for row in reversed(rows):
        lat, lon = _lat_lon_from_row(
            row,
            ("gps_cached_lat", "gps_lat", "current_lat", "current_gps_lat"),
            ("gps_cached_lon", "gps_lon", "current_lon", "current_gps_lon"),
        )
        if lat is not None and lon is not None and _fresh_cached_gps(row, cached_start_max_age_ms):
            return {
                "start_lat": lat,
                "start_lon": lon,
                "start_source": "cached_gps",
                "start_gps_block_reason": row.get("gps_block_reason", "NA"),
                "start_gps_sats": row.get("gps_sats", "NA"),
                "start_gps_hdop": row.get("gps_hdop", "NA"),
                "gps_cached_used": True,
                "gps_wait_snapshot": {k: v for k, v in snapshot.items() if k != "ready_row"},
            }
    return None


def resolve_start_for_preview(args: argparse.Namespace) -> tuple[dict[str, object] | None, list[str]]:
    """Resolve preview start coordinates, optionally opening serial for live GPS."""
    if args.start_lat is not None and args.start_lon is not None:
        return {
            "start_lat": float(args.start_lat),
            "start_lon": float(args.start_lon),
            "start_source": "explicit",
            "start_gps_block_reason": "NA",
            "start_gps_sats": "NA",
            "start_gps_hdop": "NA",
            "gps_cached_used": False,
            "gps_wait_elapsed_s": 0.0,
        }, []
    if getattr(args, "start_mode", "live_gps") == "explicit":
        return None, []

    raw_lines: list[str] = []
    min_sats = float(getattr(args, "gps_min_sats", 5))
    max_hdop = float(getattr(args, "gps_max_hdop", 2.5))
    max_cache_s = float(getattr(args, "max_cached_start_age_s", 600))
    allow_cache = str(getattr(args, "allow_cached_start", "true")).lower() != "false"
    if getattr(args, "from_log", None):
        log_path = Path(args.from_log)
        raw_lines = log_path.read_text(encoding="utf-8").splitlines()
        rows = telemetry.parse_usbdbg_rows("\n".join(raw_lines))
        resolved = resolve_start_gps_from_rows(
            rows,
            start_mode=args.start_mode,
            cached_start_max_age_ms=args.cached_start_max_age_ms,
            min_sats=min_sats,
            max_hdop=max_hdop,
        )
        if resolved is not None:
            resolved["gps_wait_elapsed_s"] = 0.0
        return resolved, raw_lines

    if not ensure_port(args):
        return None, []
    import serial

    if not telemetry._parse_bool(getattr(args, "wait_gps", "true"), default=True):
        deadline = time.monotonic() + 0.5
    else:
        deadline = time.monotonic() + float(getattr(args, "gps_timeout_s", args.start_timeout_s))
    status_interval_s = float(getattr(args, "gps_status_interval_s", 2.0))
    next_status = time.monotonic()
    start_monotonic = time.monotonic()
    rows: list[dict[str, str]] = []
    with serial.Serial(args.port, baudrate=args.baud, timeout=0.5) as handle:
        while time.monotonic() < deadline:
            raw = handle.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            raw_lines.append(line)
            rows = telemetry.parse_usbdbg_rows("\n".join(raw_lines))
            resolved = resolve_start_gps_from_rows(
                rows,
                start_mode=args.start_mode,
                cached_start_max_age_ms=args.cached_start_max_age_ms,
                min_sats=min_sats,
                max_hdop=max_hdop,
            )
            if resolved is not None:
                resolved["gps_wait_elapsed_s"] = time.monotonic() - start_monotonic
                return resolved, raw_lines
            if time.monotonic() >= next_status:
                snapshot = gps_snapshot(rows, min_sats=min_sats, max_hdop=max_hdop)
                print(_gps_status_line(time.monotonic() - start_monotonic, snapshot))
                next_status = time.monotonic() + status_interval_s
    if allow_cache:
        cached = load_cached_start(max_cache_s)
        if cached is not None:
            cached["gps_wait_elapsed_s"] = time.monotonic() - start_monotonic
            return cached, raw_lines
    return None, raw_lines


def resolve_plan(args: argparse.Namespace, calibration_dict: dict[str, object]) -> dict[str, object]:
    """Build the no-motion plan (segments + goal) shared by preview and run."""
    return preview.build_preview(
        start_lat=args.start_lat,
        start_lon=args.start_lon,
        goal_mode=args.goal_mode,
        goal_lat=args.goal_lat,
        goal_lon=args.goal_lon,
        goal_east_m=args.goal_east_m,
        goal_north_m=args.goal_north_m,
        goal_dlat=args.goal_dlat,
        goal_dlon=args.goal_dlon,
        goal_bearing_deg=args.goal_bearing_deg,
        goal_distance_m=args.goal_distance_m,
        path_shape=args.path_shape,
        workspace_width_m=args.workspace_width_m,
        step_spacing_m=args.step_spacing_m,
        diagonal_orientation=args.diagonal_orientation,
        max_segment_pulses=args.max_segment_pulses,
        nominal_forward_pulse_m=args.nominal_forward_pulse_m,
        calibration=calibration_dict,
    )


def load_rows_from_log(path: Path) -> list[dict[str, str]]:
    """Parse USBDBG telemetry rows from a saved serial log (no serial needed)."""
    return telemetry.parse_usbdbg_rows(path.read_text())


def load_planner_config(path: Path) -> dict[str, object]:
    """Load a shipped JSON config, dropping ``_``-prefixed comment keys.

    Used for the ``configs/*.json`` starting points. A ``field_rectangle_example``
    config loads with keys that are exactly :func:`preview.build_preview` kwargs,
    so ``build_preview(**load_planner_config(path))`` runs it directly.
    """
    data = json.loads(path.read_text())
    if not isinstance(data, dict):
        raise ValueError(f"config {path} must be a JSON object")
    return {key: value for key, value in data.items() if not key.startswith("_")}


def diagnose_summary(rows: Sequence[dict[str, str]]) -> dict[str, object]:
    """Summarize telemetry rows into a read-only, never-ready diagnostic dict."""
    heartbeats = [r for r in rows if telemetry.event(r) == "HEARTBEAT"]
    last = heartbeats[-1] if heartbeats else None
    event_counts: dict[str, int] = {}
    for row in rows:
        name = telemetry.event(row) or "NONE"
        event_counts[name] = event_counts.get(name, 0) + 1
    summary: dict[str, object] = {
        "mode": "diagnose",
        "row_count": len(rows),
        "heartbeat_count": len(heartbeats),
        "event_counts": event_counts,
        "guarded_pulse_compatible": controller.guarded_pulse_compatible(last) if last else False,
        "physical_output_active": telemetry.physical_output_active(last) if last else False,
        "last_gps_block_reason": telemetry.gps_block_reason(last) if last else "NA",
        "last_gps_sats": telemetry._fmt(telemetry.gps_sats(last)) if last else "NA",
        "last_gps_hdop": telemetry._fmt(telemetry.gps_hdop(last)) if last else "NA",
        "last_imu_relative_yaw_deg": (
            telemetry._fmt(telemetry.imu_relative_yaw_deg(last)) if last else "NA"
        ),
        "ready_for_full_path_following": False,
    }
    return checks.assert_not_ready_for_full_path_following(summary)


def guarded_pulse_ready_summary(rows: Sequence[dict[str, str]]) -> dict[str, object]:
    heartbeats = [row for row in rows if telemetry.event(row) == "HEARTBEAT"]
    last = heartbeats[-1] if heartbeats else {}
    guarded_seen = any(
        row.get("usb_pulse_test_mode") == "true" or row.get(_COMPAT_GUARDED_MODE_KEY) == "true"
        for row in rows
    )
    firmware_ready = any(
        row.get("usb_pulse_ready") == "true" or row.get(_COMPAT_GUARDED_READY_KEY) == "true"
        for row in rows
    )
    imu_enabled = any(row.get("imu_enabled") == "true" for row in rows)
    imu_present = any(row.get("imu_present") == "true" for row in rows)
    imu_bmi160 = any(row.get("imu_type") == "BMI160" for row in rows)
    yaw_seen = any(row.get("imu_relative_yaw_deg", "NA").upper() not in {"", "NA", "NAN", "NONE"} for row in rows)
    rc_ok = any(row.get("rc_ok") == "true" for row in rows)
    neutral_ok = any(row.get("neutral_ok") == "true" for row in rows)
    ready = guarded_seen and firmware_ready and imu_enabled and imu_present and imu_bmi160 and yaw_seen and rc_ok and neutral_ok
    reasons: list[str] = []
    if not guarded_seen:
        reasons.append("GUARDED_PULSE_HEARTBEAT_NOT_SEEN")
    if not firmware_ready:
        reasons.append("GUARDED_PULSE_READY_FALSE")
    if not imu_enabled:
        reasons.append("IMU_NOT_ENABLED")
    if not imu_present:
        reasons.append("IMU_NOT_PRESENT")
    if not imu_bmi160:
        reasons.append("BMI160_NOT_SEEN")
    if not yaw_seen:
        reasons.append("IMU_YAW_NOT_AVAILABLE")
    if not rc_ok:
        reasons.append("RC_NOT_OK")
    if not neutral_ok:
        reasons.append("NEUTRAL_NOT_OK")
    return checks.assert_not_ready_for_full_path_following({
        "mode": "guarded-pulse-ready",
        "success": ready,
        "guarded_pulse_ready": ready,
        "guarded_pulse_heartbeat_seen": guarded_seen,
        "turn_angle_calibration_ready": ready,
        "imu_enabled": imu_enabled,
        "imu_present": imu_present,
        "imu_type": last.get("imu_type", "NA"),
        "imu_relative_yaw_available": yaw_seen,
        "rc_ok": rc_ok,
        "neutral_ok": neutral_ok,
        "reason": "OK" if ready else ",".join(reasons),
        "next_recommended_action": (
            "Guarded pulse firmware is ready for turn calibration or usb-pulse-test validation."
            if ready else
            "Upload/check IMU-enabled guarded pulse firmware and inspect heartbeat, IMU, RC, and neutral fields."
        ),
        "ready_for_full_path_following": False,
    })


# --- Output writers -----------------------------------------------------------


def _write_json(path: Path, obj: object) -> None:
    path.write_text(json.dumps(obj, indent=2, default=str) + "\n")


def _write_rows_csv(path: Path, rows: Sequence[dict[str, object]]) -> None:
    if not rows:
        path.write_text("")
        return
    fieldnames: list[str] = []
    for row in rows:
        for key in row.keys():
            if key not in fieldnames:
                fieldnames.append(key)
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def _write_raw_log(path: Path, lines: Sequence[str]) -> None:
    path.write_text("\n".join(lines) + ("\n" if lines else ""))


def _fail(message: str) -> int:
    print(f"ABORT: {message}", file=sys.stderr)
    return 2


def _fail_with_summary(args: argparse.Namespace, *, reason: str, message: str) -> int:
    out_dir = getattr(args, "out_dir", None)
    if out_dir is not None:
        write_summary_files(
            out_dir,
            {
                "mode": getattr(args, "mode", "unknown"),
                "success": False,
                "reason": reason,
                "message": message,
                "next_recommended_action": "Fix the reported input or configuration and rerun the same command.",
                "ready_for_full_path_following": False,
            },
            title="Physical Path Planner",
        )
    return _fail(message)


# --- Mode handlers ------------------------------------------------------------


def cmd_preview(args: argparse.Namespace) -> int:
    cal = resolve_calibration(args)
    start, raw_start_lines = resolve_start_for_preview(args)
    if start is None:
        out_dir = Path(args.out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        if raw_start_lines:
            _write_raw_log(out_dir / "preview_start_usbdbg.log", raw_start_lines)
        raw_rows = telemetry.parse_usbdbg_rows("\n".join(raw_start_lines))
        snapshot = gps_snapshot(
            raw_rows,
            min_sats=float(getattr(args, "gps_min_sats", 5.0)),
            max_hdop=float(getattr(args, "gps_max_hdop", 2.5)),
        )
        summary = {
            "mode": "preview",
            "success": False,
            "reason": "NO_USABLE_START_GPS",
            "message": NO_USABLE_START_GPS_ACTION,
            "next_recommended_action": NO_USABLE_START_GPS_ACTION,
            "start_mode": getattr(args, "start_mode", "live_gps"),
            "start_source": "none",
            "gps_wait_enabled": telemetry._parse_bool(getattr(args, "wait_gps", "true"), default=True),
            "gps_wait_timeout_s": float(getattr(args, "gps_timeout_s", getattr(args, "start_timeout_s", 0.0))),
            "gps_wait_elapsed_s": float(getattr(args, "gps_timeout_s", getattr(args, "start_timeout_s", 0.0))),
            **{k: v for k, v in snapshot.items() if k != "ready_row"},
            "motion_calibration_loaded": motion_calibration_loaded(cal),
            "ready_for_full_path_following": False,
        }
        write_summary_files(out_dir, summary, title="Physical Path Planner Preview")
        print(f"preview: reason=NO_USABLE_START_GPS. {NO_USABLE_START_GPS_ACTION}")
        return 2
    args.start_lat = float(start["start_lat"])
    args.start_lon = float(start["start_lon"])
    try:
        plan = resolve_plan(args, cal)
    except ValueError as exc:
        return _fail_with_summary(args, reason="PLAN_INPUT_INVALID", message=str(exc))
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    if raw_start_lines:
        _write_raw_log(out_dir / "preview_start_usbdbg.log", raw_start_lines)
    summary = {
        **plan,
        "mode": "preview",
        "success": True,
        "reason": "OK",
        "start_mode": getattr(args, "start_mode", "live_gps"),
        "start_source": start["start_source"],
        "current_lat": start["start_lat"],
        "current_lon": start["start_lon"],
        "start_gps_block_reason": start["start_gps_block_reason"],
        "start_gps_sats": start["start_gps_sats"],
        "start_gps_hdop": start["start_gps_hdop"],
        "gps_cached_used": start["gps_cached_used"],
        "gps_wait_enabled": telemetry._parse_bool(getattr(args, "wait_gps", "true"), default=True),
        "gps_wait_timeout_s": float(getattr(args, "gps_timeout_s", getattr(args, "start_timeout_s", 0.0))),
        "gps_wait_elapsed_s": start.get("gps_wait_elapsed_s", 0.0),
        **dict(start.get("gps_wait_snapshot", {})),
        "motion_calibration_loaded": motion_calibration_loaded(cal),
        "connector_mode_effective": cal.get("connector_mode_effective", plan.get("connector_mode_effective")),
        "next_recommended_action": f"Inspect {out_dir / 'summary.md'} and preview outputs before execute-plan or run.",
        "ready_for_full_path_following": False,
    }
    _write_json(out_dir / "preview_summary.json", summary)
    write_summary_files(out_dir, summary, title="Physical Path Planner Preview")
    if args.png:
        png = preview.write_preview_png(
            out_dir / "preview.png",
            plan["segments"],  # type: ignore[arg-type]
            float(plan["start_lat"]),
            float(plan["start_lon"]),
            float(plan["goal_lat"]),
            float(plan["goal_lon"]),
            plan["workspace"],  # type: ignore[arg-type]
        )
        if png is None:
            print("preview: matplotlib unavailable; skipped PNG render")
    print(
        f"preview: {plan['segment_count']} segments, "
        f"{plan['lane_count']} lanes, goal_distance_m={float(plan['goal_distance_m']):.3f} -> {out_dir}"
    )
    return 0


def _gps_wait_summary(
    rows: Sequence[dict[str, str]],
    *,
    mode: str,
    elapsed_s: float,
    timeout_s: float,
    min_sats: float,
    max_hdop: float,
) -> dict[str, object]:
    snapshot = gps_snapshot(rows, min_sats=min_sats, max_hdop=max_hdop)
    summary = {k: v for k, v in snapshot.items() if k != "ready_row"}
    success = bool(snapshot["gps_ready"])
    summary.update(
        {
            "mode": mode,
            "success": success,
            "reason": "GPS_READY" if success else "GPS_WAIT_TIMEOUT",
            "gps_wait_enabled": True,
            "gps_wait_timeout_s": timeout_s,
            "gps_wait_elapsed_s": elapsed_s,
            "gps_min_sats": min_sats,
            "gps_max_hdop": max_hdop,
            "next_recommended_action": (
                "Use preview/run now that a start coordinate is available."
                if success else GPS_WAIT_TIMEOUT_ACTION
            ),
            "ready_for_full_path_following": False,
        }
    )
    return checks.assert_not_ready_for_full_path_following(summary)


def cmd_gps_wait(args: argparse.Namespace) -> int:
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    raw_lines: list[str] = []
    rows: list[dict[str, str]] = []
    start = time.monotonic()
    if getattr(args, "from_log", None):
        raw_lines = Path(args.from_log).read_text(encoding="utf-8").splitlines()
        rows = telemetry.parse_usbdbg_rows("\n".join(raw_lines))
        elapsed_s = 0.0
    else:
        if not ensure_port(args):
            return 2
        import serial

        deadline = start + float(args.timeout_s)
        next_status = start
        with serial.Serial(args.port, baudrate=args.baud, timeout=0.5) as handle:
            while time.monotonic() < deadline:
                raw = handle.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                raw_lines.append(line)
                parsed = telemetry.parse_usbdbg_rows(line)
                if parsed:
                    rows.extend(parsed)
                snapshot = gps_snapshot(rows, min_sats=args.min_sats, max_hdop=args.max_hdop)
                if time.monotonic() >= next_status:
                    print(_gps_status_line(time.monotonic() - start, snapshot))
                    next_status = time.monotonic() + float(args.status_interval_s)
                if snapshot["gps_ready"]:
                    break
        elapsed_s = time.monotonic() - start
    summary = _gps_wait_summary(
        rows,
        mode="gps-wait",
        elapsed_s=elapsed_s,
        timeout_s=float(args.timeout_s),
        min_sats=float(args.min_sats),
        max_hdop=float(args.max_hdop),
    )
    _write_raw_log(out_dir / "raw_usbdbg.log", raw_lines)
    _write_rows_csv(out_dir / "gps_wait.csv", rows)
    write_summary_files(out_dir, summary, title="GPS Wait")
    if summary["success"] is True:
        write_gps_cache(gps_snapshot(rows, min_sats=args.min_sats, max_hdop=args.max_hdop))
    print(
        f"gps-wait: reason={summary['reason']} "
        f"best_sats={summary['best_sats']} best_hdop={summary['best_hdop']} -> {out_dir}"
    )
    return 0 if summary["success"] is True else 2


def cmd_calibrate_turn(args: argparse.Namespace) -> int:
    if args.print_cmd:
        args.port = printable_port(args.port)
    if not args.print_cmd and not ensure_port(args):
        return 2
    mode = args.mode
    if args.direction:
        mode = "turn_left" if args.direction == "left" else "turn_right"
    argv = build_calibrate_turn_argv(
        script=args.script,
        port=args.port,
        mode=mode,
        b_cmd=args.b_cmd,
        pulse_ms=args.pulse_ms,
        max_abs_b=args.max_abs_b,
        max_ms=args.max_ms,
        target_angle_deg=args.target_angle_deg,
        angle_tolerance_deg=args.angle_tolerance_deg,
        save_turn_calibration=args.save_turn_calibration,
        turn_calibration_out=args.turn_calibration_out,
        out_dir=args.out_dir,
    )
    printable = " ".join(shlex.quote(part) for part in argv)
    if args.print_cmd:
        print(printable)
        write_summary_files(
            args.out_dir,
            {
                "mode": "calibrate-turn",
                "success": True,
                "reason": "COMMAND_PRINTED",
                "command": printable,
                "turn_angle_calibration_ready": False,
                "next_recommended_action": "Run without --print-cmd only when ready to upload firmware and calibrate physically.",
                "ready_for_full_path_following": False,
            },
            title="Turn Angle Calibration",
        )
        return 0
    print(f"calibrate-turn: invoking {printable}")
    completed = subprocess.run(argv, check=False)
    write_summary_files(
        args.out_dir,
        {
            "mode": "calibrate-turn",
            "success": completed.returncode == 0,
            "reason": "OK" if completed.returncode == 0 else "TURN_CALIBRATION_FAILED",
            "returncode": completed.returncode,
            "command": printable,
            "turn_angle_calibration_ready": completed.returncode == 0,
            "next_recommended_action": (
                "Inspect the calibration output summary and turn calibration JSON."
                if completed.returncode == 0
                else "Inspect raw logs, IMU yaw availability, ACK/STOP, and visual confirmation."
            ),
            "ready_for_full_path_following": False,
        },
        title="Turn Angle Calibration",
    )
    return completed.returncode


def cmd_manual_rc(args: argparse.Namespace) -> int:
    if args.print_cmd:
        args.port = printable_port(args.port)
    if not args.print_cmd and not ensure_port(args):
        return 2
    if args.diagnose_only == "true" and args.upload == "true":
        print("diagnose-only requested; skipping upload")
        args.upload = "false"
    upload_enabled = args.upload in {"true", "auto"}
    validate_enabled = args.validate == "true"
    upload_cmd = ["bash", args.upload_script, "--port", str(args.port)]
    upload_cmd.extend(["--rc-input-mode", str(args.rc_input_mode)])
    if args.mode_channel_index is not None:
        upload_cmd.extend(["--mode-channel-index", str(args.mode_channel_index)])
    upload_cmd.extend(["--steer-channel-index", str(args.steer_channel_index)])
    upload_cmd.extend(["--throttle-channel-index", str(args.throttle_channel_index)])
    validate_cmd = [
        "bash",
        args.validate_script,
        "--port",
        str(args.port),
        "--duration-s",
        str(args.duration_s),
        "--out-dir",
        str(args.out_dir),
        "--upload",
        "false",
    ]
    if args.log:
        validate_cmd.extend(["--log", str(args.log)])
    if args.diagnose_only == "true":
        validate_cmd.extend(["--diagnose-only", "true"])
    mapping_warning = "NONE"
    if args.rc_input_mode not in {"auto", "old_known_good", "ppm"}:
        mapping_warning = "RC_INPUT_MODE_FLAG_NOT_IMPLEMENTED"
    elif args.steer_channel_index != 0 or args.throttle_channel_index != 1:
        mapping_warning = "RC_STEER_THROTTLE_CHANNEL_FLAGS_NOT_IMPLEMENTED"
    mapping_summary = {
        "rc_input_mode_requested": args.rc_input_mode,
        "rc_input_mode_effective": "ppm_old_known_good" if args.rc_input_mode in {"auto", "old_known_good", "ppm"} else "unsupported",
        "mode_channel_index": args.mode_channel_index,
        "steer_channel_index": args.steer_channel_index,
        "throttle_channel_index": args.throttle_channel_index,
        "old_known_good_rc_path": args.rc_input_mode in {"auto", "old_known_good", "ppm"},
        "manual_forward_sign": -1,
        "manual_turn_sign": 1,
        "motor_output_swap_lr": 0,
        "drive_calibration_enable": 0,
        "manual_mode_threshold_us": args.manual_mode_threshold_us,
        "rc_mapping_flags_effective": mapping_warning == "NONE",
        "rc_mapping_warning": mapping_warning,
    }
    if args.print_cmd:
        if upload_enabled:
            print(" ".join(shlex.quote(part) for part in upload_cmd))
        if validate_enabled:
            print(" ".join(shlex.quote(part) for part in validate_cmd))
        print("ready_for_full_path_following=false")
        write_summary_files(
            args.out_dir,
            {
                "mode": "manual-rc",
                "success": True,
                "reason": "COMMAND_PRINTED",
                "upload_success": False,
                "validation_success": False,
                **mapping_summary,
                "next_recommended_action": "Run without --print-cmd when ready to upload or validate manual RC telemetry.",
                "ready_for_full_path_following": False,
            },
            title="Manual RC Diagnostic",
        )
        return 0
    print("Manual RC recovery")
    print("RC transmitter ON; MANUAL / AUTO OFF")
    print("Sequence: neutral 5s, slight forward, neutral, slight backward, neutral, slight left/right steering, neutral")
    if args.print_rc_mapping == "true":
        for key, value in mapping_summary.items():
            print(f"{key}={value}")
    print(f"manual_rc_recovery_flags={manual_rc_recovery_flags(mode_channel_index=args.mode_channel_index)}")
    upload_success = False
    if upload_enabled:
        completed = subprocess.run(upload_cmd, check=False)
        if completed.returncode != 0:
            write_summary_files(
                args.out_dir,
                {
                    "mode": "manual-rc",
                    "success": False,
                    "reason": "MANUAL_RC_UPLOAD_FAILED",
                    "upload_success": False,
                    "validation_success": False,
                    "returncode": completed.returncode,
                    **mapping_summary,
                    "next_recommended_action": "Check Arduino CLI, OpenRB port, and compile/upload output.",
                    "ready_for_full_path_following": False,
                },
                title="Manual RC Diagnostic",
            )
            return completed.returncode
        upload_success = True
    else:
        upload_success = args.upload == "false"
    if validate_enabled:
        completed = subprocess.run(validate_cmd, check=False)
        summary_path = Path(args.out_dir) / "summary.json"
        validation_summary: dict[str, object]
        if summary_path.exists():
            loaded = json.loads(summary_path.read_text())
            validation_summary = loaded if isinstance(loaded, dict) else {}
        else:
            validation_summary = {
                "reason": "MANUAL_RC_VALIDATION_FAILED",
                "manual_rc_passthrough_ok": False,
                "validation_success": False,
            }
        merged = {
            "mode": "manual-rc",
            "success": completed.returncode == 0 and validation_summary.get("manual_rc_passthrough_ok") is True,
            "upload_success": upload_success,
            "validation_success": completed.returncode == 0 and validation_summary.get("manual_rc_passthrough_ok") is True,
            **mapping_summary,
            **validation_summary,
            "ready_for_full_path_following": False,
        }
        merged.setdefault("reason", "OK" if merged["success"] else "MANUAL_RC_VALIDATION_FAILED")
        if merged.get("reason") == "RC_INPUT_ABSENT":
            merged["next_recommended_action"] = RC_INPUT_ABSENT_ACTION
        merged.setdefault("next_recommended_action", "Inspect manual RC telemetry and wiring before rerunning.")
        write_summary_files(args.out_dir, merged, title="Manual RC Diagnostic")
        return 0 if merged["success"] is True else 2
    write_summary_files(
        args.out_dir,
        {
            "mode": "manual-rc",
            "success": upload_success,
            "reason": "OK" if upload_success else "NO_UPLOAD_OR_VALIDATION_REQUESTED",
            "upload_success": upload_success,
            "validation_success": False,
            "manual_rc_passthrough_ok": False,
            **mapping_summary,
            "next_recommended_action": "Run manual-rc --upload false --validate true to diagnose receiver input.",
            "ready_for_full_path_following": False,
        },
        title="Manual RC Diagnostic",
    )
    return 0 if upload_success else 2


def cmd_manual_control(args: argparse.Namespace) -> int:
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    if args.print_cmd:
        args.port = printable_port(args.port)
    if not args.print_cmd and not args.from_log and not ensure_port(args):
        return 2

    flags = manual_control_firmware_flags(mode_channel_index=args.mode_channel_index)
    compile_cmd = [
        "arduino-cli",
        "compile",
        "--fqbn",
        "OpenRB-150:samd:OpenRB-150",
        "--build-path",
        "/private/tmp/openrb-manual-control-ppm",
        "--build-property",
        f"compiler.cpp.extra_flags={flags}",
        "firmware/openrb_robot_controller",
    ]
    upload_cmd = [
        "arduino-cli",
        "upload",
        "-p",
        str(args.port),
        "--fqbn",
        "OpenRB-150:samd:OpenRB-150",
        "--build-path",
        "/private/tmp/openrb-manual-control-ppm",
        "firmware/openrb_robot_controller",
    ]
    config = {
        "mode": "manual-control",
        "rc_input_mode": "ppm",
        "ppm_input_pin": "D6",
        "steer_channel": "CH1",
        "throttle_channel": "CH2",
        "mode_channel": "CH5",
        "mode_channel_index": args.mode_channel_index,
        "manual_forward_sign": -1,
        "manual_turn_sign": 1,
        "gps_required": False,
        "imu_required": False,
        "path_package_required": False,
        "station_frame_parser_required": False,
        "hc12_required": False,
        "ready_for_full_path_following": False,
    }
    _write_json(out_dir / "manual_control_config.json", config)
    if args.print_cmd:
        if args.upload in {"true", "auto"}:
            print(" ".join(shlex.quote(part) for part in compile_cmd))
            print(" ".join(shlex.quote(part) for part in upload_cmd))
        print(f"manual_control_firmware_flags={flags}")
        print("PPM wiring: signal -> OpenRB D6; CH1 steering; CH2 throttle; CH5 mode/manual-auto.")
        print("ready_for_full_path_following=false")
        write_summary_files(
            out_dir,
            {
                **config,
                "success": True,
                "reason": "COMMAND_PRINTED",
                "manual_control_ok": False,
                "next_recommended_action": "Run without --print-cmd when ready to upload and monitor PPM manual control.",
            },
            title="Manual Control",
        )
        return 0

    raw_lines: list[str] = []
    if args.from_log:
        raw_lines = Path(args.from_log).read_text(encoding="utf-8").splitlines()
    else:
        if args.upload in {"true", "auto"}:
            completed = subprocess.run(compile_cmd, check=False)
            if completed.returncode != 0:
                write_summary_files(
                    out_dir,
                    {
                        **config,
                        "success": False,
                        "reason": "MANUAL_CONTROL_COMPILE_FAILED",
                        "returncode": completed.returncode,
                        "next_recommended_action": "Inspect Arduino compile output for the PPM manual control firmware.",
                    },
                    title="Manual Control",
                )
                return completed.returncode
            completed = subprocess.run(upload_cmd, check=False)
            if completed.returncode != 0:
                write_summary_files(
                    out_dir,
                    {
                        **config,
                        "success": False,
                        "reason": "MANUAL_CONTROL_UPLOAD_FAILED",
                        "returncode": completed.returncode,
                        "next_recommended_action": "Check OpenRB port and upload output.",
                    },
                    title="Manual Control",
                )
                return completed.returncode
        if args.validate == "false":
            write_summary_files(
                out_dir,
                {
                    **config,
                    "success": True,
                    "reason": "UPLOAD_ONLY",
                    "manual_control_ok": False,
                    "next_recommended_action": "Run manual-control --upload false --validate true to monitor PPM control.",
                },
                title="Manual Control",
            )
            return 0
        import serial

        print("Manual control: PPM input on OpenRB D6.")
        print("Expected mapping: CH1 steering -> physical B, CH2 throttle -> physical A, CH5 mode/manual-auto.")
        print("GPS/IMU status remains visible as telemetry only; it does not gate manual motor output.")
        print("Set mode to MANUAL / AUTO OFF and move the physical station/controller.")
        if args.duration_s <= 0:
            print("Monitor runs until Ctrl-C.")
        last_status_s = -1
        start_s = time.monotonic()
        deadline = None if args.duration_s <= 0 else start_s + args.duration_s
        try:
            with serial.Serial(args.port, baudrate=args.baud, timeout=0.2) as handle:
                while deadline is None or time.monotonic() < deadline:
                    raw = handle.readline()
                    if raw:
                        line = raw.decode("utf-8", errors="replace").strip()
                        raw_lines.append(line)
                        if args.verbose_raw == "true":
                            print(line)
                    elapsed_s = int(time.monotonic() - start_s)
                    if elapsed_s != last_status_s:
                        last_status_s = elapsed_s
                        rows = telemetry.parse_usbdbg_rows("\n".join(raw_lines[-200:]))
                        print(format_manual_control_status(elapsed_s=elapsed_s, rows=rows))
        except KeyboardInterrupt:
            print("User aborted manual-control monitor; writing summaries.")
        except (OSError, serial.serialutil.SerialException) as exc:
            print(f"manual-control serial error: {exc}")
            raw_lines.append(f"SERIAL_ERROR error={str(exc).replace(' ', '_')}")

    rows = telemetry.parse_usbdbg_rows("\n".join(raw_lines))
    summary = {**config, **evaluate_manual_control_rows(rows)}
    if summary.get("reason") == "PPM_INPUT_ABSENT":
        print("reason=PPM_INPUT_ABSENT")
        print("Expected wiring: signal -> OpenRB D6; CH1 steering; CH2 throttle; CH5 mode/manual-auto.")
    _write_raw_log(out_dir / "raw_usbdbg.log", raw_lines)
    _write_rows_csv(out_dir / "manual_control.csv", rows)
    _write_json(out_dir / "manual_control_summary.json", summary)
    write_summary_files(out_dir, summary, title="Manual Control")
    print(f"manual_control_ok={str(summary['manual_control_ok']).lower()}")
    print(f"reason={summary['reason']}")
    print("ready_for_full_path_following=false")
    return 0 if summary["manual_control_ok"] is True else 2


def cmd_rc_input_diagnose(args: argparse.Namespace) -> int:
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    compile_cmd = [
        "arduino-cli",
        "compile",
        "--fqbn",
        "OpenRB-150:samd:OpenRB-150",
        "--build-path",
        "/private/tmp/openrb-rc-input-diagnose",
        args.sketch,
    ]
    upload_cmd = [
        "arduino-cli",
        "upload",
        "-p",
        printable_port(args.port),
        "--fqbn",
        "OpenRB-150:samd:OpenRB-150",
        "--build-path",
        "/private/tmp/openrb-rc-input-diagnose",
        args.sketch,
    ]
    if args.print_cmd:
        print(" ".join(shlex.quote(part) for part in compile_cmd))
        print(" ".join(shlex.quote(part) for part in upload_cmd))
        print("ready_for_full_path_following=false")
        write_summary_files(
            out_dir,
            {
                "mode": "rc-input-diagnose",
                "success": True,
                "reason": "COMMAND_PRINTED",
                "probe": "ppm_channel_map_probe",
                "motors_enabled": False,
                "next_recommended_action": "Run without --print-cmd to upload/read the RC input diagnostic firmware.",
                "ready_for_full_path_following": False,
            },
            title="RC Input Diagnose",
        )
        return 0
    raw_lines: list[str] = []
    if args.from_log:
        raw_lines = Path(args.from_log).read_text(encoding="utf-8").splitlines()
    else:
        if not ensure_port(args):
            return 2
        if args.upload in {"true", "auto"}:
            completed = subprocess.run(compile_cmd, check=False)
            if completed.returncode != 0:
                write_summary_files(
                    out_dir,
                    {
                        "mode": "rc-input-diagnose",
                        "success": False,
                        "reason": "RC_INPUT_DIAGNOSE_COMPILE_FAILED",
                        "returncode": completed.returncode,
                        "next_recommended_action": "Inspect Arduino compile output for the read-only PPM probe.",
                        "ready_for_full_path_following": False,
                    },
                    title="RC Input Diagnose",
                )
                return completed.returncode
            completed = subprocess.run(upload_cmd, check=False)
            if completed.returncode != 0:
                write_summary_files(
                    out_dir,
                    {
                        "mode": "rc-input-diagnose",
                        "success": False,
                        "reason": "RC_INPUT_DIAGNOSE_UPLOAD_FAILED",
                        "returncode": completed.returncode,
                        "next_recommended_action": "Check OpenRB port and upload mode, then retry rc-input-diagnose.",
                        "ready_for_full_path_following": False,
                    },
                    title="RC Input Diagnose",
                )
                return completed.returncode
        import serial

        print("RC input diagnose: read-only PPM channel probe; motors/GPS/HC-12 disabled.")
        deadline = time.monotonic() + args.duration_s
        with serial.Serial(args.port, baudrate=args.baud, timeout=0.5) as handle:
            while time.monotonic() < deadline:
                raw = handle.readline()
                if raw:
                    line = raw.decode("utf-8", errors="replace").strip()
                    print(line)
                    raw_lines.append(line)
    rows = telemetry.parse_usbdbg_rows("\n".join(raw_lines))
    summary = evaluate_rc_input_diagnose_rows(rows)
    _write_raw_log(out_dir / "raw_usbdbg.log", raw_lines)
    _write_rows_csv(out_dir / "rc_input_diagnose.csv", rows)
    _write_json(out_dir / "rc_input_diagnose_summary.json", summary)
    write_summary_files(out_dir, summary, title="RC Input Diagnose")
    print(f"rc_input_classification={summary['rc_input_classification']}")
    print("ready_for_full_path_following=false")
    return 0 if summary["success"] is True else 2


def cmd_guarded_pulse_ready(args: argparse.Namespace) -> int:
    if getattr(args, "deprecated_alias", False):
        print("Deprecated alias: use guarded-pulse-ready.")
    if args.print_cmd:
        args.port = printable_port(args.port)
    if not args.print_cmd and not ensure_port(args):
        return 2
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    flags = guarded_pulse_firmware_flags(max_abs_a=args.max_abs_a, max_abs_b=args.max_abs_b, max_ms=args.max_ms)
    compile_cmd = [
        "arduino-cli",
        "compile",
        "--fqbn",
        "OpenRB-150:samd:OpenRB-150",
        "--build-path",
        "/private/tmp/openrb-guarded-pulse-ready",
        "--build-property",
        f"compiler.cpp.extra_flags={flags}",
        "firmware/openrb_robot_controller",
    ]
    upload_cmd = [
        "arduino-cli",
        "upload",
        "-p",
        str(args.port),
        "--fqbn",
        "OpenRB-150:samd:OpenRB-150",
        "--build-path",
        "/private/tmp/openrb-guarded-pulse-ready",
        "firmware/openrb_robot_controller",
    ]
    if args.print_cmd:
        if args.upload in {"true", "auto"}:
            print(" ".join(shlex.quote(part) for part in compile_cmd))
            print(" ".join(shlex.quote(part) for part in upload_cmd))
        print(f"guarded_pulse_flags={flags}")
        print("ready_for_full_path_following=false")
        write_summary_files(
            out_dir,
            {
                "mode": "guarded-pulse-ready",
                "success": True,
                "reason": "COMMAND_PRINTED",
                "guarded_pulse_ready": False,
                "guarded_pulse_heartbeat_seen": False,
                "turn_angle_calibration_ready": False,
                "next_recommended_action": "Run without --print-cmd only when ready to upload/check guarded pulse firmware.",
                "ready_for_full_path_following": False,
            },
            title="IMU-Enabled Guarded Pulse Firmware",
        )
        return 0
    if args.upload in {"true", "auto"}:
        completed = subprocess.run(compile_cmd, check=False)
        if completed.returncode != 0:
            return completed.returncode
        completed = subprocess.run(upload_cmd, check=False)
        if completed.returncode != 0:
            return completed.returncode
    import serial

    raw_lines: list[str] = []
    with serial.Serial(args.port, baudrate=args.baud, timeout=0.5) as handle:
        deadline = time.monotonic() + args.duration_s
        while time.monotonic() < deadline:
            raw = handle.readline()
            if raw:
                line = raw.decode("utf-8", errors="replace").strip()
                print(line)
                raw_lines.append(line)
    rows = telemetry.parse_usbdbg_rows("\n".join(raw_lines))
    summary = guarded_pulse_ready_summary(rows)
    _write_raw_log(out_dir / "raw_usbdbg.log", raw_lines)
    _write_rows_csv(out_dir / "guarded_pulse_readiness.csv", rows)
    _write_json(out_dir / "guarded_pulse_readiness_summary.json", summary)
    write_summary_files(out_dir, summary, title="IMU-Enabled Guarded Pulse Firmware")
    print(f"guarded_pulse_ready={str(summary['guarded_pulse_ready']).lower()}")
    print("ready_for_full_path_following=false")
    return 0 if summary["guarded_pulse_ready"] is True else 2


def _station_drive_name(name: str) -> str:
    normalized = USB_PULSE_TEST_ALIASES.get(name.strip().lower())
    if normalized is None:
        raise ValueError(f"unknown usb-pulse-test primitive: {name}")
    return normalized


def station_drive_plan(*, sequence: str | None = None, single: str | None = None) -> list[dict[str, object]]:
    requested = [_station_drive_name(item["primitive"]) for item in USB_PULSE_TEST_SEQUENCE]
    if sequence:
        requested = [_station_drive_name(part) for part in sequence.split(",") if part.strip()]
    if single:
        requested = [_station_drive_name(single)]
    by_name = {str(item["primitive"]): item for item in USB_PULSE_TEST_SEQUENCE}
    planned: list[dict[str, object]] = []
    for index, name in enumerate(requested, start=1):
        primitive = by_name[name]
        a_cmd = float(primitive["a"])
        b_cmd = float(primitive["b"])
        pulse_ms = int(primitive["ms"])
        planned.append(
            {
                **primitive,
                "seq": index,
                "a_cmd": a_cmd,
                "b_cmd": b_cmd,
                "pulse_ms": pulse_ms,
                "arm_command_text": f"USB_PULSE_TEST_ARM seq={index}",
                "usb_pulse_test_command_text": (
                    f"USB_PULSE_TEST_CMD seq={index} a={a_cmd:.3f} b={b_cmd:.3f} ms={pulse_ms}"
                ),
                "command_text": f"USB_PULSE_TEST_CMD seq={index} a={a_cmd:.3f} b={b_cmd:.3f} ms={pulse_ms}",
                "stop_command_text": f"USB_PULSE_TEST_STOP seq={index}",
            }
        )
    return planned


def usb_pulse_test_plan(*, sequence: str | None = None, single: str | None = None) -> list[dict[str, object]]:
    return station_drive_plan(sequence=sequence, single=single)


def station_drive_display_block(item: dict[str, object]) -> str:
    label = {
        "forward": "FORWARD",
        "backward": "BACKWARD",
        "left": "LEFT",
        "right": "RIGHT",
    }.get(str(item["primitive"]), str(item["primitive"]).upper())
    return f"{label}:\nA={float(item['a']):+0.3f} B={float(item['b']):+0.3f} ms={int(item['ms'])}"


def station_drive_console_line(item: dict[str, object]) -> str:
    return f"{str(item['primitive']).upper()}: A={float(item['a']):+0.2f} B={float(item['b']):+0.2f} {int(item['ms'])}ms"


def station_drive_clean_plan(planned: Sequence[dict[str, object]]) -> list[dict[str, object]]:
    return [
        {
            "primitive": item["primitive"],
            "a_cmd": item["a"],
            "b_cmd": item["b"],
            "pulse_ms": item["ms"],
        }
        for item in planned
    ]


def station_drive_compatible(row: dict[str, str]) -> bool:
    clean_ready = (
        telemetry._parse_bool(row.get("usb_pulse_test_mode")) is True
        and telemetry._parse_bool(row.get("usb_pulse_test_ready")) is True
    )
    return clean_ready or controller.guarded_pulse_compatible(row)


def station_drive_event_counts(rows: Sequence[dict[str, object]]) -> dict[str, int]:
    def count_bool(key: str) -> int:
        return sum(1 for row in rows if row.get(key) is True)

    return {
        "command_sent_count": sum(1 for row in rows if row.get("command_sent") is True),
        "ack_count": count_bool("ack_seen"),
        "active_count": count_bool("active_seen"),
        "stop_count": count_bool("stop_seen"),
        "reject_count": count_bool("reject_seen"),
        "rc_invalid_count": sum(1 for row in rows if row.get("reject_reason") == "RC_INVALID"),
        "motor_write_called_count": count_bool("motor_write_called_seen"),
        "physical_output_active_count": count_bool("physical_output_active_seen"),
        "final_zero_count": count_bool("final_zero"),
        "user_observed_motion_count": sum(
            1 for row in rows if str(row.get("user_motion_report", "")).lower() in {"forward", "backward", "left", "right"}
        ),
    }


def station_drive_classification(rows: Sequence[dict[str, object]], *, user_aborted: bool = False) -> tuple[str, str, str]:
    counts = station_drive_event_counts(rows)
    if any(row.get("wrong_firmware_manual_rc_recovery") is True for row in rows):
        return (
            "WRONG_FIRMWARE_MANUAL_RC_RECOVERY",
            "WRONG_FIRMWARE_MANUAL_RC_RECOVERY",
            "Upload usb-pulse-test firmware; manual-rc firmware reads receiver passthrough and is not usb-pulse-test.",
        )
    if counts["command_sent_count"] == 0:
        return (
            "WAITING_FOR_USER_ENTER",
            "WAITING_FOR_USER_ENTER",
            "No usb-pulse-test command was sent. Press Enter at a command prompt to run bounded USB pulses.",
        )
    if counts["reject_count"] > 0:
        if counts["rc_invalid_count"] > 0:
            return (
                "BUG_USB_PULSE_TEST_STILL_REQUIRES_RC_INPUT",
                "BUG_USB_PULSE_TEST_STILL_REQUIRES_RC_INPUT",
                "usb-pulse-test must ignore absent RC input. Re-upload usb-pulse-test firmware and inspect reject telemetry.",
            )
        return (
            "COMMAND_SENT_NO_ACK",
            "COMMAND_SENT_NO_ACK",
            "Inspect raw_usbdbg.log for the usb-pulse-test reject reason and command limits.",
        )
    if counts["ack_count"] < counts["command_sent_count"]:
        return (
            "COMMAND_SENT_NO_ACK",
            "COMMAND_SENT_NO_ACK",
            "Confirm usb-pulse-test firmware received the USB pulse command and emitted ACK.",
        )
    if counts["active_count"] < counts["command_sent_count"]:
        return (
            "COMMAND_ACKED_NO_ACTIVE",
            "COMMAND_ACKED_NO_ACTIVE",
            "ACK was seen but ACTIVE was missing; inspect firmware output gating.",
        )
    if counts["stop_count"] < counts["command_sent_count"]:
        return (
            "COMMAND_ACTIVE_NO_STOP",
            "COMMAND_ACTIVE_NO_STOP",
            "ACTIVE was seen but STOP was missing; inspect serial timing and STOP telemetry.",
        )
    if counts["motor_write_called_count"] == 0 and counts["physical_output_active_count"] == 0:
        return (
            "MOTOR_OUTPUT_BLOCKED",
            "MOTOR_OUTPUT_BLOCKED",
            "ACK/ACTIVE were seen but no motor write or output-active telemetry appeared.",
        )
    if any(
        row.get("valid_pulse") is True
        and (
            row.get("telemetry_motion_seen") is True
            or row.get("motor_write_called_seen") is True
            or row.get("physical_output_active_seen") is True
        )
        and str(row.get("user_motion_report", "")).lower() == "none"
        for row in rows
    ):
        return (
            "TELEMETRY_OUTPUT_ACTIVE_BUT_USER_SAW_NONE",
            "TELEMETRY_OUTPUT_ACTIVE_BUT_USER_SAW_NONE",
            "Telemetry says output occurred; inspect wheels, drivetrain load, and whether the rover was able to move.",
        )
    valid_rows = [row for row in rows if row.get("valid_pulse") is True and row.get("skipped") is not True]
    if valid_rows and all(
        str(row.get("user_motion_report", "")).lower() in {"forward", "backward", "left", "right", "twitch", "unknown", "not_asked"}
        for row in valid_rows
    ):
        return (
            "USB_PULSE_TEST_PASS",
            "USB_PULSE_TEST_PASS",
            "usb-pulse-test A/B control passed; RC receiver passthrough remains a separate mode.",
        )
    return (
        "COMMAND_SENT_NO_ACK",
        "COMMAND_SENT_NO_ACK",
        "Inspect ACK/ACTIVE/STOP/final-zero fields and raw_usbdbg.log.",
    )


def _station_drive_latest_state(row: dict[str, str]) -> str:
    return (
        row.get("usb_pulse_test_cmd_state")
        or row.get("station_drive_cmd_state")
        or row.get(_COMPAT_GUARDED_STATE_KEY)
        or row.get(_COMPAT_GUARDED_STATE_FALLBACK_KEY)
        or ""
    )


def cmd_usb_pulse_test(args: argparse.Namespace) -> int:
    if getattr(args, "deprecated_station_manual_alias", False):
        print("Deprecated alias: use usb-pulse-test.")
    if getattr(args, "deprecated_station_drive_alias", False):
        print("Deprecated alias: use usb-pulse-test.")
    planned = station_drive_plan(sequence=getattr(args, "sequence", None), single=getattr(args, "single", None))
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    if args.print_command == "true":
        print("\n\n".join(station_drive_display_block(item) for item in planned))
        summary = {
            "mode": "usb-pulse-test",
            "success": True,
            "reason": "COMMAND_PRINTED",
            "usb_pulse_test_result": "COMMAND_PRINTED",
            "rc_input_required": False,
            "rc_input_ignored": True,
            "gps_required": False,
            "imu_required": False,
            "pulse_count": len(planned),
            "planned_pulses": station_drive_clean_plan(planned),
            "physical_a_role": "throttle",
            "physical_b_role": "turn",
            "wheel_to_physical_mapping": "physical_ab_manual_equivalent",
            "next_recommended_action": "Run usb-pulse-test without --print-command only when ready for bounded USB pulse control.",
            "ready_for_full_path_following": False,
        }
        _write_json(out_dir / "usb_pulse_test_plan.json", planned)
        write_summary_files(out_dir, summary, title="USB Pulse Test")
        return 0
    if args.print_cmd:
        for item in planned:
            print(item["arm_command_text"])
            print(item["usb_pulse_test_command_text"])
            print(item["stop_command_text"])
        summary = {
            "mode": "usb-pulse-test",
            "success": True,
            "reason": "COMMAND_PRINTED",
            "usb_pulse_test_result": "COMMAND_PRINTED",
            "rc_input_required": False,
            "rc_input_ignored": True,
            "gps_required": False,
            "imu_required": False,
            "pulse_count": len(planned),
            "planned_pulses": station_drive_clean_plan(planned),
            "next_recommended_action": "Run without --print-cmd only when ready for bounded USB pulse control.",
            "ready_for_full_path_following": False,
        }
        _write_json(out_dir / "usb_pulse_test_plan.json", planned)
        write_summary_files(out_dir, summary, title="USB Pulse Test")
        return 0
    if not ensure_port(args):
        return 2
    if args.upload in {"true", "auto"}:
        flags = usb_pulse_test_firmware_flags(max_abs_a=args.max_abs_a, max_abs_b=args.max_abs_b, max_ms=args.max_ms)
        compile_cmd = [
            "arduino-cli",
            "compile",
            "--fqbn",
            "OpenRB-150:samd:OpenRB-150",
            "--build-path",
            "/private/tmp/openrb-usb-pulse-test",
            "--build-property",
            f"compiler.cpp.extra_flags={flags}",
            "firmware/openrb_robot_controller",
        ]
        upload_cmd = [
            "arduino-cli",
            "upload",
            "-p",
            str(args.port),
            "--fqbn",
            "OpenRB-150:samd:OpenRB-150",
            "--build-path",
            "/private/tmp/openrb-usb-pulse-test",
            "firmware/openrb_robot_controller",
        ]
        completed = subprocess.run(compile_cmd, check=False)
        if completed.returncode != 0:
            write_summary_files(
                out_dir,
                {
                    "mode": "usb-pulse-test",
                    "success": False,
                    "reason": "USB_PULSE_TEST_FIRMWARE_COMPILE_FAILED",
                    "returncode": completed.returncode,
                    "next_recommended_action": "Inspect Arduino compile output before retrying usb-pulse-test.",
                    "ready_for_full_path_following": False,
                },
                title="USB Pulse Test",
            )
            return completed.returncode
        completed = subprocess.run(upload_cmd, check=False)
        if completed.returncode != 0:
            write_summary_files(
                out_dir,
                {
                    "mode": "usb-pulse-test",
                    "success": False,
                    "reason": "USB_PULSE_TEST_FIRMWARE_UPLOAD_FAILED",
                    "returncode": completed.returncode,
                    "next_recommended_action": "Check OpenRB port and upload mode before retrying usb-pulse-test.",
                    "ready_for_full_path_following": False,
                },
                title="USB Pulse Test",
            )
            return completed.returncode
    import serial

    raw_lines: list[str] = []
    rows: list[dict[str, object]] = []
    invalid_count = 0
    user_aborted = False
    print(f"resolved_port={args.port}")
    print("firmware_mode=usb_pulse_test")
    print("usb_pulse_test_ignore_rc_input=true")
    print("USB pulse-test command plan:")
    for item in planned:
        print(f"  {station_drive_console_line(item)}")
    with serial.Serial(args.port, baudrate=args.baud, timeout=0.5) as handle:
        for item in planned:
            print(f"Ready to send {station_drive_console_line(item)}")
            heartbeat = executor.wait_for_row(
                handle,
                raw_lines,
                lambda row: telemetry.event(row) == "HEARTBEAT" and station_drive_compatible(row),
                args.heartbeat_timeout_s,
                verbose_raw=args.verbose_raw == "true",
            )
            print(f"heartbeat ready: {str(heartbeat is not None).lower()}")
            if heartbeat and telemetry._parse_bool(heartbeat.get("manual_rc_recovery")) is True:
                rows.append({
                    "seq": item["seq"],
                    "primitive": item["primitive"],
                    "a_cmd": item["a"],
                    "b_cmd": item["b"],
                    "pulse_ms": item["ms"],
                    "command_sent": False,
                    "wrong_firmware_manual_rc_recovery": True,
                    "valid_pulse": False,
                    "invalid_reason": "WRONG_FIRMWARE_MANUAL_RC_RECOVERY",
                    "ready_for_full_path_following": False,
                })
                break
            if heartbeat is None:
                rows.append({
                    "seq": item["seq"],
                    "primitive": item["primitive"],
                    "a_cmd": item["a"],
                    "b_cmd": item["b"],
                    "pulse_ms": item["ms"],
                    "command_sent": False,
                    "skipped": False,
                    "valid_pulse": False,
                    "invalid_reason": "USB_PULSE_TEST_HEARTBEAT_MISSING",
                    "ready_for_full_path_following": False,
                })
                if args.abort_on_invalid == "true":
                    break
                continue
            if args.require_enter == "true":
                response = input("Press Enter to send, or type skip/abort: ").strip().lower()
                if response == "skip":
                    rows.append({
                        "seq": item["seq"],
                        "primitive": item["primitive"],
                        "a_cmd": item["a"],
                        "b_cmd": item["b"],
                        "pulse_ms": item["ms"],
                        "command_sent": False,
                        "skipped": True,
                        "valid_pulse": False,
                        "invalid_reason": "SKIPPED_BY_USER",
                        "ready_for_full_path_following": False,
                    })
                    print("skipped=true")
                    continue
                if response == "abort":
                    user_aborted = True
                    print("aborted_by_user=true")
                    break
            else:
                for remaining in (3, 2, 1):
                    print(f"sending in {remaining}...")
                    time.sleep(1.0)
            print("command sent")
            pulse_rows = executor.send_pulse(
                handle,
                item,
                raw_lines,
                event_timeout_s=args.event_timeout_s,
                verbose_raw=args.verbose_raw == "true",
            )
            invalid_reason = controller.pulse_block_reason(pulse_rows)
            if invalid_reason is not None:
                invalid_count += 1
            visual = "not_asked"
            if args.interactive_visible_motion == "true":
                visual = input("Observed motion [forward/backward/left/right/twitch/none/unknown]: ").strip() or "unknown"
            last = pulse_rows[-1] if pulse_rows else {}
            reject_reason = safety.latest_reject_reason(pulse_rows) if pulse_rows else "NONE"
            ack_seen = any(telemetry.event(row) == "ACK" for row in pulse_rows)
            active_seen = any(telemetry.event(row) == "ACTIVE" or _station_drive_latest_state(row) == "ACTIVE" for row in pulse_rows)
            stop_seen = any(telemetry.event(row) in {"STOP", "PULSE_COMPLETE", "PULSE_DONE"} for row in pulse_rows)
            motor_write_called_seen = any(telemetry._parse_bool(row.get("motor_write_called")) is True for row in pulse_rows)
            physical_output_active_seen = any(telemetry.physical_output_active(row) for row in pulse_rows)
            final_left = telemetry._optional_float(last.get("final_left_cmd")) if last else None
            final_right = telemetry._optional_float(last.get("final_right_cmd")) if last else None
            final_zero = (final_left is not None and final_right is not None and abs(final_left) <= 1e-6 and abs(final_right) <= 1e-6)
            print(f"ACK seen: {str(ack_seen).lower()}")
            print(f"ACTIVE seen: {str(active_seen).lower()}")
            print(f"STOP seen: {str(stop_seen).lower()}")
            print(f"final zero: {str(final_zero).lower()}")
            print(f"observed_motion={visual}")
            rows.append({
                "seq": item["seq"],
                "primitive": item["primitive"],
                "a_cmd": item["a"],
                "b_cmd": item["b"],
                "pulse_ms": item["ms"],
                "arm_command_text": item["arm_command_text"],
                "usb_pulse_test_command_text": item["usb_pulse_test_command_text"],
                "stop_command_text": item["stop_command_text"],
                "command_sent": True,
                "ack_seen": ack_seen,
                "active_seen": active_seen,
                "stop_seen": stop_seen,
                "reject_seen": any(telemetry.event(row) == "REJECT" for row in pulse_rows),
                "reject_reason": reject_reason,
                "motor_write_called_seen": motor_write_called_seen,
                "physical_output_active_seen": physical_output_active_seen,
                "telemetry_motion_seen": motor_write_called_seen or physical_output_active_seen,
                "invalid_reason": invalid_reason or "OK",
                "valid_pulse": invalid_reason is None,
                "final_left_cmd": last.get("final_left_cmd", "NA"),
                "final_right_cmd": last.get("final_right_cmd", "NA"),
                "final_zero": final_zero,
                "physical_output_active_after_stop": last.get("physical_output_active", "NA"),
                "user_motion_report": visual,
                "ready_for_full_path_following": False,
            })
            if invalid_reason is not None and args.abort_on_invalid == "true":
                break
    _write_raw_log(out_dir / "raw_usbdbg.log", raw_lines)
    _write_rows_csv(out_dir / "usb_pulse_test_validation.csv", rows)
    result, reason, next_action = station_drive_classification(rows, user_aborted=user_aborted)
    counts = station_drive_event_counts(rows)
    success = result == "USB_PULSE_TEST_PASS"
    summary = {
        "mode": "usb-pulse-test",
        "success": success,
        "reason": reason,
        "usb_pulse_test_result": result,
        "pulse_count": len(planned),
        "completed_pulse_count": len(rows),
        "invalid_pulse_count": invalid_count,
        **counts,
        "observed_motions": [row.get("user_motion_report", "") for row in rows if row.get("command_sent") is True],
        "rc_input_required": False,
        "rc_input_ignored": True,
        "gps_required": False,
        "imu_required": False,
        "physical_a_role": "throttle",
        "physical_b_role": "turn",
        "wheel_to_physical_mapping": "physical_ab_manual_equivalent",
        "next_recommended_action": next_action,
        "ready_for_full_path_following": False,
    }
    write_summary_files(out_dir, summary, title="USB Pulse Test")
    print(f"usb_pulse_test_success={str(success).lower()}")
    print("ready_for_full_path_following=false")
    return 0 if success else 2


def tune_motion_planned_command(candidate: dict[str, object], *, seq: int) -> dict[str, object]:
    a_cmd = float(candidate["a"])
    b_cmd = float(candidate["b"])
    pulse_ms = int(candidate["ms"])
    return {
        "seq": seq,
        "primitive": candidate["primitive"],
        "a_cmd": a_cmd,
        "b_cmd": b_cmd,
        "pulse_ms": pulse_ms,
        "arm_command_text": f"USB_PULSE_TEST_ARM seq={seq}",
        "command_text": f"USB_PULSE_TEST_CMD seq={seq} a={a_cmd:.3f} b={b_cmd:.3f} ms={pulse_ms}",
        "stop_command_text": f"USB_PULSE_TEST_STOP seq={seq}",
    }


def tune_motion_trial_row(
    *,
    trial_index: int,
    candidate: dict[str, object],
    feedback: str,
    pulse_rows: Sequence[dict[str, str]],
    invalid_reason: str | None,
    yaw_delta_deg: float | None,
    opposite_sign_transient: bool = False,
) -> dict[str, object]:
    last = pulse_rows[-1] if pulse_rows else {}
    final_left = telemetry._optional_float(last.get("final_left_cmd")) if last else None
    final_right = telemetry._optional_float(last.get("final_right_cmd")) if last else None
    final_zero = (
        final_left is not None
        and final_right is not None
        and abs(final_left) <= 1e-6
        and abs(final_right) <= 1e-6
    )
    return {
        "trial_index": trial_index,
        "primitive": candidate["primitive"],
        "a_cmd": f"{float(candidate['a']):.3f}",
        "b_cmd": f"{float(candidate['b']):.3f}",
        "pulse_ms": int(candidate["ms"]),
        "target_angle_deg": candidate.get("target_angle_deg", "NA"),
        "imu_yaw_delta_deg": "NA" if yaw_delta_deg is None else f"{yaw_delta_deg:.3f}",
        "feedback": feedback,
        "ack_seen": any(telemetry.event(row) == "ACK" for row in pulse_rows),
        "active_seen": any(telemetry.event(row) == "ACTIVE" or _station_drive_latest_state(row) == "ACTIVE" for row in pulse_rows),
        "stop_seen": any(telemetry.event(row) in safety.STOP_EVENTS for row in pulse_rows),
        "reject_seen": any(telemetry.event(row) == "REJECT" for row in pulse_rows),
        "opposite_sign_transient": opposite_sign_transient,
        "final_left_cmd": last.get("final_left_cmd", "NA"),
        "final_right_cmd": last.get("final_right_cmd", "NA"),
        "final_zero": final_zero,
        "valid_pulse": invalid_reason is None,
        "invalid_reason": invalid_reason or "OK",
        "ready_for_full_path_following": False,
    }


def tune_motion_summary(
    rows: Sequence[dict[str, object]],
    *,
    primitive: str,
    candidate: dict[str, object],
    approved: bool,
    reason: str,
    calibration_out: Path,
) -> dict[str, object]:
    summary = {
        "mode": "tune-motion",
        "success": approved,
        "reason": reason,
        "primitive": primitive,
        "trial_count": len(rows),
        "actual_pulse_count": sum(1 for row in rows if row.get("valid_pulse") in {True, False}),
        "opposite_sign_transient_count": sum(1 for row in rows if row.get("opposite_sign_transient") is True),
        "approved_candidate": {
            "a": round(float(candidate["a"]), 3),
            "b": round(float(candidate["b"]), 3),
            "ms": int(candidate["ms"]),
            **(
                {"target_angle_deg": float(candidate["target_angle_deg"])}
                if "target_angle_deg" in candidate else {}
            ),
        },
        "calibration_out": str(calibration_out),
        "final_zero_required": True,
        "observed_distance_m_required": False,
        "next_recommended_action": (
            "Use execute-plan or run; approved motion calibration will be loaded automatically."
            if approved else
            "Rerun tune-motion and approve only after ACK/ACTIVE/STOP/final-zero and visual behavior are acceptable."
        ),
        "ready_for_full_path_following": False,
    }
    return checks.assert_not_ready_for_full_path_following(summary)


def _upload_usb_pulse_test_firmware(args: argparse.Namespace, out_dir: Path, *, title: str) -> int:
    flags = usb_pulse_test_firmware_flags(max_abs_a=args.max_abs_a, max_abs_b=args.max_abs_b, max_ms=args.max_ms)
    build_path = "/private/tmp/openrb-tune-motion" if title == "Tune Motion" else "/private/tmp/openrb-usb-pulse-test"
    compile_cmd = [
        "arduino-cli",
        "compile",
        "--fqbn",
        "OpenRB-150:samd:OpenRB-150",
        "--build-path",
        build_path,
        "--build-property",
        f"compiler.cpp.extra_flags={flags}",
        "firmware/openrb_robot_controller",
    ]
    upload_cmd = [
        "arduino-cli",
        "upload",
        "-p",
        str(args.port),
        "--fqbn",
        "OpenRB-150:samd:OpenRB-150",
        "--build-path",
        build_path,
        "firmware/openrb_robot_controller",
    ]
    completed = subprocess.run(compile_cmd, check=False)
    if completed.returncode != 0:
        write_summary_files(
            out_dir,
            {
                "mode": "tune-motion",
                "success": False,
                "reason": "USB_PULSE_TEST_FIRMWARE_COMPILE_FAILED",
                "returncode": completed.returncode,
                "next_recommended_action": "Inspect Arduino compile output before retrying tune-motion.",
                "ready_for_full_path_following": False,
            },
            title=title,
        )
        return completed.returncode
    completed = subprocess.run(upload_cmd, check=False)
    if completed.returncode != 0:
        write_summary_files(
            out_dir,
            {
                "mode": "tune-motion",
                "success": False,
                "reason": "USB_PULSE_TEST_FIRMWARE_UPLOAD_FAILED",
                "returncode": completed.returncode,
                "next_recommended_action": "Check OpenRB port and upload mode before retrying tune-motion.",
                "ready_for_full_path_following": False,
            },
            title=title,
        )
        return completed.returncode
    return 0


def _upload_usb_drive_live_firmware(args: argparse.Namespace, out_dir: Path) -> int:
    flags = usb_drive_live_firmware_flags(
        max_abs_a=args.max_abs_a,
        max_abs_b=args.max_abs_b,
        max_duration_ms=int(args.max_duration_s * 1000.0),
        update_timeout_ms=args.ttl_ms,
    )
    build_path = "/private/tmp/openrb-usb-drive-live"
    compile_cmd = [
        "arduino-cli",
        "compile",
        "--fqbn",
        "OpenRB-150:samd:OpenRB-150",
        "--build-path",
        build_path,
        "--build-property",
        f"compiler.cpp.extra_flags={flags}",
        "firmware/openrb_robot_controller",
    ]
    upload_cmd = [
        "arduino-cli",
        "upload",
        "-p",
        str(args.port),
        "--fqbn",
        "OpenRB-150:samd:OpenRB-150",
        "--build-path",
        build_path,
        "firmware/openrb_robot_controller",
    ]
    completed = subprocess.run(compile_cmd, check=False)
    if completed.returncode != 0:
        write_summary_files(
            out_dir,
            {
                "mode": "usb-drive-live",
                "success": False,
                "reason": "USB_DRIVE_LIVE_FIRMWARE_COMPILE_FAILED",
                "returncode": completed.returncode,
                "ready_for_full_path_following": False,
            },
            title="USB Drive Live",
        )
        return completed.returncode
    completed = subprocess.run(upload_cmd, check=False)
    if completed.returncode != 0:
        write_summary_files(
            out_dir,
            {
                "mode": "usb-drive-live",
                "success": False,
                "reason": "USB_DRIVE_LIVE_FIRMWARE_UPLOAD_FAILED",
                "returncode": completed.returncode,
                "ready_for_full_path_following": False,
            },
            title="USB Drive Live",
        )
        return completed.returncode
    return 0


def usb_drive_live_summary(rows: Sequence[dict[str, str]], *, a_cmd: float, b_cmd: float, duration_s: float) -> dict[str, object]:
    reject_seen = any(telemetry.event(row) == "REJECT" for row in rows)
    stop_seen = any(telemetry.event(row) in safety.STOP_EVENTS for row in rows)
    trace_rows = [row for row in rows if "physical_a_cmd" in row and "motor_write_called" in row]
    motor_write_seen = any(telemetry._parse_bool(row.get("motor_write_called")) is True for row in trace_rows)
    output_active_seen = any(telemetry.physical_output_active(row) for row in rows)
    final_nonzero = safety.nonzero_final_cmd(rows)
    success = not reject_seen and stop_seen and not final_nonzero
    reason = "OK" if success else (
        "REJECT" if reject_seen else
        "STOP_MISSING" if not stop_seen else
        "FINAL_COMMANDS_NONZERO"
    )
    summary = {
        "mode": "usb-drive-live",
        "success": success,
        "reason": reason,
        "a_cmd": round(a_cmd, 3),
        "b_cmd": round(b_cmd, 3),
        "duration_s": duration_s,
        "setpoint_update_count": sum(1 for row in rows if telemetry.event(row) == "ACTIVE"),
        "motor_trace_count": len(trace_rows),
        "motor_write_called_seen": motor_write_seen,
        "physical_output_active_seen": output_active_seen,
        "stop_seen": stop_seen,
        "final_zero": not final_nonzero,
        "next_recommended_action": (
            "Use tune-motion or execute-plan after confirming smooth motion."
            if success else
            "Inspect raw_usbdbg.log and motor trace rows before retrying live drive."
        ),
        "ready_for_full_path_following": False,
    }
    return checks.assert_not_ready_for_full_path_following(summary)


def cmd_usb_drive_live(args: argparse.Namespace) -> int:
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    if abs(float(args.a)) > args.max_abs_a or abs(float(args.b)) > args.max_abs_b:
        return _fail_with_summary(args, reason="USB_DRIVE_LIVE_COMMAND_EXCEEDS_MAX", message="--a/--b exceed live-drive bounds")
    if args.duration_s <= 0 or args.duration_s > args.max_duration_s:
        return _fail_with_summary(args, reason="USB_DRIVE_LIVE_DURATION_EXCEEDS_MAX", message="--duration-s must be >0 and <= --max-duration-s")
    if args.print_command == "true":
        print(
            f"USB_DRIVE_LIVE_SET seq=1 a={float(args.a):.3f} b={float(args.b):.3f} "
            f"duration_ms={int(args.duration_s * 1000.0)} ttl_ms={int(args.ttl_ms)}"
        )
        print("USB_DRIVE_LIVE_STOP seq=1")
        summary = {
            "mode": "usb-drive-live",
            "success": True,
            "reason": "COMMAND_PRINTED",
            "a_cmd": round(float(args.a), 3),
            "b_cmd": round(float(args.b), 3),
            "duration_s": args.duration_s,
            "ready_for_full_path_following": False,
        }
        write_summary_files(out_dir, summary, title="USB Drive Live")
        return 0
    if not ensure_port(args):
        return 2
    if args.upload in {"true", "auto"}:
        uploaded = _upload_usb_drive_live_firmware(args, out_dir)
        if uploaded != 0:
            return uploaded

    import serial

    raw_lines: list[str] = []
    rows: list[dict[str, str]] = []
    try:
        with serial.Serial(args.port, baudrate=args.baud, timeout=0.5) as handle:
            print(f"resolved_port={args.port}")
            print(f"usb_drive_live A={float(args.a):+0.3f} B={float(args.b):+0.3f} duration_s={float(args.duration_s):.2f}")
            rows = executor.send_live_drive(
                handle,
                seq=1,
                duration_s=float(args.duration_s),
                update_hz=float(args.update_hz),
                ttl_ms=int(args.ttl_ms),
                command_fn=lambda _row: (float(args.a), float(args.b)),
                raw_lines=raw_lines,
                event_timeout_s=float(args.event_timeout_s),
                verbose_raw=args.verbose_raw == "true",
            )
    except KeyboardInterrupt:
        rows = telemetry.parse_usbdbg_rows("\n".join(raw_lines))
    except OSError:
        rows = telemetry.parse_usbdbg_rows("\n".join(raw_lines))
        summary = {
            "mode": "usb-drive-live",
            "success": False,
            "reason": "SERIAL_DISCONNECT",
            "ready_for_full_path_following": False,
        }
        _write_raw_log(out_dir / "raw_usbdbg.log", raw_lines)
        _write_rows_csv(out_dir / "usb_drive_live_rows.csv", rows)
        write_summary_files(out_dir, summary, title="USB Drive Live")
        return 2
    _write_raw_log(out_dir / "raw_usbdbg.log", raw_lines)
    _write_rows_csv(out_dir / "usb_drive_live_rows.csv", rows)
    summary = usb_drive_live_summary(rows, a_cmd=float(args.a), b_cmd=float(args.b), duration_s=float(args.duration_s))
    write_summary_files(out_dir, summary, title="USB Drive Live")
    print(f"usb_drive_live_success={str(summary['success']).lower()}")
    print(f"reason={summary['reason']}")
    print("ready_for_full_path_following=false")
    return 0 if summary["success"] is True else 2


def cmd_tune_motion(args: argparse.Namespace) -> int:
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    primitive = tuning.normalize_primitive(args.primitive)
    candidate = tuning.initial_candidate(primitive)
    calibration_out = tuning.motion_calibration_path(args.calibration_out)
    if args.print_candidate == "true":
        summary = tune_motion_summary(
            [],
            primitive=primitive,
            candidate=candidate,
            approved=False,
            reason="CANDIDATE_PRINTED",
            calibration_out=calibration_out,
        )
        summary["success"] = True
        _write_json(out_dir / "tune_motion_candidate.json", candidate)
        write_summary_files(out_dir, summary, title="Tune Motion")
        print(
            f"{primitive}: A={float(candidate['a']):+0.3f} "
            f"B={float(candidate['b']):+0.3f} ms={int(candidate['ms'])}"
        )
        return 0
    if not ensure_port(args):
        return 2
    if args.upload in {"true", "auto"}:
        uploaded = _upload_usb_pulse_test_firmware(args, out_dir, title="Tune Motion")
        if uploaded != 0:
            return uploaded

    import serial

    raw_lines: list[str] = []
    trial_rows: list[dict[str, object]] = []
    approved = False
    reason = "NOT_APPROVED"
    try:
        with serial.Serial(args.port, baudrate=args.baud, timeout=0.5) as handle:
            for trial_index in range(1, args.max_iterations + 1):
                candidate = tuning.clamp_candidate(candidate)
                print(
                    f"candidate trial={trial_index} primitive={primitive} "
                    f"A={float(candidate['a']):+0.3f} B={float(candidate['b']):+0.3f} ms={int(candidate['ms'])}"
                )
                if "target_angle_deg" in candidate:
                    print(f"target_angle_deg={float(candidate['target_angle_deg']):.1f}")
                heartbeat = executor.wait_for_row(
                    handle,
                    raw_lines,
                    lambda row: telemetry.event(row) == "HEARTBEAT" and station_drive_compatible(row),
                    args.heartbeat_timeout_s,
                    verbose_raw=args.verbose_raw == "true",
                )
                print(f"heartbeat ready: {str(heartbeat is not None).lower()}")
                if heartbeat is None:
                    reason = "USB_PULSE_TEST_HEARTBEAT_MISSING"
                    break
                if args.require_enter == "true":
                    response = input("Press Enter to send, or type abort: ").strip().lower()
                    if response == "abort":
                        reason = "USER_ABORTED"
                        break
                planned = tune_motion_planned_command(candidate, seq=trial_index)
                print("command sent")
                pulse_rows = executor.send_pulse(
                    handle,
                    planned,
                    raw_lines,
                    event_timeout_s=args.event_timeout_s,
                    verbose_raw=args.verbose_raw == "true",
                )
                invalid_reason = controller.pulse_block_reason(pulse_rows)
                yaw_delta = tuning.yaw_delta_from_rows(pulse_rows)
                opposite = tuning.opposite_sign_transient(primitive, pulse_rows)
                if yaw_delta is not None:
                    print(f"imu_yaw_delta_deg={yaw_delta:.3f}")
                if opposite:
                    print("opposite_sign_transient=true")
                if invalid_reason is not None:
                    reason = invalid_reason
                    trial_rows.append(
                        tune_motion_trial_row(
                            trial_index=trial_index,
                            candidate=candidate,
                            feedback="invalid",
                            pulse_rows=pulse_rows,
                            invalid_reason=invalid_reason,
                            yaw_delta_deg=yaw_delta,
                            opposite_sign_transient=opposite,
                        )
                    )
                    break
                feedback = input(
                    "observed? [good/weak/strong/too_short/too_long/left/right/none/retry/approve/abort]: "
                ).strip().lower() or "retry"
                trial_rows.append(
                    tune_motion_trial_row(
                        trial_index=trial_index,
                        candidate=candidate,
                        feedback=feedback,
                        pulse_rows=pulse_rows,
                        invalid_reason=invalid_reason,
                        yaw_delta_deg=yaw_delta,
                        opposite_sign_transient=opposite,
                    )
                )
                if feedback == "abort":
                    reason = "USER_ABORTED"
                    break
                if feedback == "approve":
                    if opposite:
                        reason = "OPPOSITE_SIGN_TRANSIENT"
                        break
                    tuning.save_approved_calibration(
                        calibration_out,
                        candidate,
                        yaw_delta_deg=yaw_delta,
                        heading_drift_deg=yaw_delta if primitive in {"forward", "backward"} else None,
                    )
                    approved = True
                    reason = "APPROVED"
                    break
                candidate = tuning.adjust_candidate(candidate, feedback, yaw_delta_deg=yaw_delta)
    except KeyboardInterrupt:
        reason = "USER_ABORTED"
    except OSError:
        reason = "SERIAL_DISCONNECT"

    _write_raw_log(out_dir / "raw_usbdbg.log", raw_lines)
    _write_rows_csv(out_dir / "tune_motion_trials.csv", trial_rows)
    summary = tune_motion_summary(
        trial_rows,
        primitive=primitive,
        candidate=candidate,
        approved=approved,
        reason=reason,
        calibration_out=calibration_out,
    )
    write_summary_files(out_dir, summary, title="Tune Motion")
    print(f"tune_motion_success={str(approved).lower()}")
    print(f"reason={reason}")
    print("ready_for_full_path_following=false")
    return 0 if approved else 2


def _station_hw_compile_upload_cmds(args: argparse.Namespace, *, diagnose_only: bool) -> tuple[list[str], list[str], str]:
    flags = station_hw_diagnose_firmware_flags() if diagnose_only else station_hw_manual_firmware_flags()
    build_path = "/private/tmp/openrb-station-hw-diagnose" if diagnose_only else "/private/tmp/openrb-station-hw-manual"
    compile_cmd = [
        "arduino-cli",
        "compile",
        "--fqbn",
        "OpenRB-150:samd:OpenRB-150",
        "--build-path",
        build_path,
        "--build-property",
        f"compiler.cpp.extra_flags={flags}",
        "firmware/openrb_robot_controller",
    ]
    upload_cmd = [
        "arduino-cli",
        "upload",
        "-p",
        str(args.port),
        "--fqbn",
        "OpenRB-150:samd:OpenRB-150",
        "--build-path",
        build_path,
        "firmware/openrb_robot_controller",
    ]
    return compile_cmd, upload_cmd, flags


def _read_station_hw_rows(args: argparse.Namespace, *, title: str, csv_name: str, summary_name: str, mode: str) -> int:
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    raw_lines: list[str] = []
    user_aborted = False
    serial_error: str | None = None
    if args.from_log:
        raw_lines = Path(args.from_log).read_text(encoding="utf-8").splitlines()
    else:
        if not ensure_port(args):
            return 2
        compile_cmd, upload_cmd, flags = _station_hw_compile_upload_cmds(
            args,
            diagnose_only=(mode == "station-hw-diagnose"),
        )
        if args.upload in {"true", "auto"}:
            print(f"station_hw_firmware_flags={flags}")
            completed = subprocess.run(compile_cmd, check=False)
            if completed.returncode != 0:
                write_summary_files(
                    out_dir,
                    {
                        "mode": mode,
                        "success": False,
                        "reason": "STATION_HW_FIRMWARE_COMPILE_FAILED",
                        "returncode": completed.returncode,
                        "next_recommended_action": "Inspect Arduino compile output before retrying station hardware mode.",
                        "ready_for_full_path_following": False,
                    },
                    title=title,
                )
                return completed.returncode
            completed = subprocess.run(upload_cmd, check=False)
            if completed.returncode != 0:
                write_summary_files(
                    out_dir,
                    {
                        "mode": mode,
                        "success": False,
                        "reason": "STATION_HW_FIRMWARE_UPLOAD_FAILED",
                        "returncode": completed.returncode,
                        "next_recommended_action": "Check OpenRB port and upload mode before retrying station hardware mode.",
                        "ready_for_full_path_following": False,
                    },
                    title=title,
                )
                return completed.returncode
        import serial

        print(f"{mode}: monitoring physical station hardware frames")
        print("rc_receiver_required=false gps_required=false imu_required=false")
        print("station input mapping: throttle -> physical A, steering -> physical B")
        print("station_transport=station_hardware_serial")
        print("station_protocol=auto")
        print("station_parser=auto_station_manual")
        duration_s = float(args.duration_s)
        deadline = None if duration_s <= 0 else time.monotonic() + duration_s
        started = time.monotonic()
        next_status_s = 0.0
        no_link_notice_printed = False
        if deadline is None:
            print("duration=continuous_until_ctrl_c")
        try:
            with serial.Serial(args.port, baudrate=args.baud, timeout=0.2) as handle:
                while deadline is None or time.monotonic() < deadline:
                    raw = handle.readline()
                    if raw:
                        line = raw.decode("utf-8", errors="replace").strip()
                        raw_lines.append(line)
                        if args.verbose_raw == "true":
                            print(line)
                    elapsed_s = time.monotonic() - started
                    if elapsed_s >= next_status_s:
                        rows_now = telemetry.parse_usbdbg_rows("\n".join(raw_lines))
                        summary_now = evaluate_station_hw_rows(rows_now, mode=mode)
                        print(_station_hw_status_line(summary_now, elapsed_s=elapsed_s))
                        if (
                            elapsed_s >= 3.0
                            and not no_link_notice_printed
                            and summary_now.get("station_link_seen") is not True
                        ):
                            print("No station hardware frames received yet.")
                            no_link_notice_printed = True
                        if summary_now.get("reason") == "STATION_HW_DEADMAN_NOT_ACTIVE":
                            print("Station frames received, but deadman is not active.")
                        elif summary_now.get("reason") == "STATION_HW_ESTOP_ACTIVE":
                            print("Station estop active.")
                        elif (
                            summary_now.get("station_physical_a_nonzero_seen") is True
                            or summary_now.get("station_physical_b_nonzero_seen") is True
                        ):
                            print(
                                f"A={summary_now.get('station_physical_a_cmd', 'NA')} "
                                f"B={summary_now.get('station_physical_b_cmd', 'NA')}"
                            )
                        if summary_now.get("motor_write_called_seen") is True or summary_now.get("physical_output_active_seen") is True:
                            print(
                                "motor_write_called="
                                f"{str(summary_now.get('motor_write_called_seen', False)).lower()} "
                                "physical_output_active="
                                f"{str(summary_now.get('physical_output_active_seen', False)).lower()}"
                            )
                        next_status_s += 1.0
        except KeyboardInterrupt:
            user_aborted = True
            print("User aborted station hardware monitor; writing summaries.")
        except (OSError, serial.serialutil.SerialException) as exc:
            serial_error = str(exc)
            print(f"station hardware serial error: {exc}")
    rows = telemetry.parse_usbdbg_rows("\n".join(raw_lines))
    summary = evaluate_station_hw_rows(rows, mode=mode)
    if user_aborted:
        summary = dict(summary)
        summary["success"] = False
        summary["station_hw_result_before_abort"] = summary.get("station_hw_result")
        summary["reason_before_abort"] = summary.get("reason")
        summary["reason"] = "USER_ABORTED"
        summary["station_hw_result"] = "USER_ABORTED"
        summary["user_aborted"] = True
        summary["next_recommended_action"] = "Rerun station-hw-diagnose or station-hw-manual after checking the station hardware state."
        summary = checks.assert_not_ready_for_full_path_following(summary)
    elif serial_error is not None:
        summary = dict(summary)
        summary["success"] = False
        summary["station_hw_result_before_serial_error"] = summary.get("station_hw_result")
        summary["reason_before_serial_error"] = summary.get("reason")
        summary["reason"] = "SERIAL_ERROR"
        summary["station_hw_result"] = "SERIAL_ERROR"
        summary["serial_error"] = serial_error
        summary["next_recommended_action"] = "Check the OpenRB USB cable/port and rerun the station hardware monitor."
        summary = checks.assert_not_ready_for_full_path_following(summary)
    _write_raw_log(out_dir / "raw_usbdbg.log", raw_lines)
    _write_rows_csv(out_dir / csv_name, rows)
    raw_dump_count = write_station_raw_frame_dumps(out_dir, rows)
    if raw_dump_count:
        summary = dict(summary)
        summary["station_raw_frame_dump_count"] = raw_dump_count
        summary["raw_station_frames"] = "raw_station_frames.txt"
        summary["raw_station_frames_hex"] = "raw_station_frames_hex.txt"
        summary = checks.assert_not_ready_for_full_path_following(summary)
        if summary.get("station_parse_ok_count") == 0 and summary.get("station_parse_error_count", 0):
            print("Station frames are arriving but parser does not match. See raw_station_frames.txt.")
    _write_json(out_dir / summary_name, summary)
    write_summary_files(out_dir, summary, title=title)
    print(f"station_link_seen={str(summary['station_link_seen']).lower()}")
    print(f"station_hw_result={summary['station_hw_result']}")
    print("ready_for_full_path_following=false")
    if user_aborted:
        return 130
    return 0 if summary["success"] is True else 2


def cmd_station_hw_diagnose(args: argparse.Namespace) -> int:
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    compile_cmd, upload_cmd, flags = _station_hw_compile_upload_cmds(args, diagnose_only=True)
    if args.print_cmd or args.print_command == "true":
        print("STATION HARDWARE DIAGNOSE:")
        print("No motor commands are sent. The rover only reports station hardware frames.")
        print("Expected station input mapping: throttle -> physical A, steering -> physical B")
        print(f"station_hw_firmware_flags={flags}")
        if args.print_cmd:
            print(" ".join(shlex.quote(part) for part in compile_cmd))
            print(" ".join(shlex.quote(part) for part in upload_cmd))
        write_summary_files(
            out_dir,
            {
                "mode": "station-hw-diagnose",
                "success": True,
                "reason": "COMMAND_PRINTED",
                "station_hw_result": "COMMAND_PRINTED",
                "motors_enabled": False,
                "rc_input_required": False,
                "gps_required": False,
                "imu_required": False,
                "physical_a_role": "throttle",
                "physical_b_role": "turn",
                "wheel_to_physical_mapping": "physical_ab_manual_equivalent",
                "next_recommended_action": "Run without print options to monitor physical station hardware frames.",
                "ready_for_full_path_following": False,
            },
            title="Station Hardware Diagnose",
        )
        return 0
    return _read_station_hw_rows(
        args,
        title="Station Hardware Diagnose",
        csv_name="station_hw_diagnose.csv",
        summary_name="station_hw_diagnose_summary.json",
        mode="station-hw-diagnose",
    )


def cmd_station_hw_manual(args: argparse.Namespace) -> int:
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    compile_cmd, upload_cmd, flags = _station_hw_compile_upload_cmds(args, diagnose_only=False)
    if args.print_cmd:
        print(" ".join(shlex.quote(part) for part in compile_cmd))
        print(" ".join(shlex.quote(part) for part in upload_cmd))
        print(f"station_hw_firmware_flags={flags}")
        write_summary_files(
            out_dir,
            {
                "mode": "station-hw-manual",
                "success": True,
                "reason": "COMMAND_PRINTED",
                "station_hw_result": "COMMAND_PRINTED",
                "rc_input_required": False,
                "gps_required": False,
                "imu_required": False,
                "physical_a_role": "throttle",
                "physical_b_role": "turn",
                "wheel_to_physical_mapping": "physical_ab_manual_equivalent",
                "next_recommended_action": "Run without --print-cmd to upload/verify station hardware manual firmware and monitor output.",
                "ready_for_full_path_following": False,
            },
            title="Station Hardware Manual",
        )
        return 0
    print("Station hardware manual control")
    print("Operator: turn on station hardware, release estop, hold deadman, then move station throttle/steering.")
    return _read_station_hw_rows(
        args,
        title="Station Hardware Manual",
        csv_name="station_hw_manual.csv",
        summary_name="station_hw_manual_summary.json",
        mode="station-hw-manual",
    )


def cmd_run(args: argparse.Namespace) -> int:
    cal = resolve_calibration(args)
    if getattr(args, "plan_dir", None):
        plan_dir = Path(args.plan_dir)
        candidates = [plan_dir / "preview_summary.json", plan_dir / "plan.json"]
        plan_path = next((path for path in candidates if path.exists()), None)
        if plan_path is None:
            return _fail_with_summary(
                args,
                reason="PLAN_DIR_MISSING_PLAN",
                message=f"--plan-dir must contain preview_summary.json or plan.json: {plan_dir}",
            )
        plan = json.loads(plan_path.read_text())
    else:
        if args.start_lat is None or args.start_lon is None:
            start, raw_start_lines = resolve_start_for_preview(args)
            if start is None:
                out_dir = Path(args.out_dir)
                out_dir.mkdir(parents=True, exist_ok=True)
                if raw_start_lines:
                    _write_raw_log(out_dir / "run_start_usbdbg.log", raw_start_lines)
                raw_rows = telemetry.parse_usbdbg_rows("\n".join(raw_start_lines))
                snapshot = gps_snapshot(
                    raw_rows,
                    min_sats=float(getattr(args, "gps_min_sats", 5.0)),
                    max_hdop=float(getattr(args, "gps_max_hdop", 2.5)),
                )
                write_summary_files(
                    out_dir,
                    {
                        "mode": args.mode,
                        "success": False,
                        "reason": "NO_USABLE_START_GPS",
                        "message": NO_USABLE_START_GPS_ACTION,
                        "next_recommended_action": NO_USABLE_START_GPS_ACTION,
                        "start_mode": getattr(args, "start_mode", "live_gps"),
                        "start_source": "none",
                        "gps_wait_enabled": telemetry._parse_bool(getattr(args, "wait_gps", "true"), default=True),
                        "gps_wait_timeout_s": float(getattr(args, "gps_timeout_s", getattr(args, "start_timeout_s", 0.0))),
                        "gps_wait_elapsed_s": float(getattr(args, "gps_timeout_s", getattr(args, "start_timeout_s", 0.0))),
                        **{k: v for k, v in snapshot.items() if k != "ready_row"},
                        "motion_calibration_loaded": motion_calibration_loaded(cal),
                        "ready_for_full_path_following": False,
                    },
                    title="Physical Path Planner Run",
                )
                return 2
            args.start_lat = float(start["start_lat"])
            args.start_lon = float(start["start_lon"])
        try:
            plan = resolve_plan(args, cal)
        except ValueError as exc:
            return _fail_with_summary(args, reason="PLAN_INPUT_INVALID", message=str(exc))
        if "start" in locals():
            plan.update(
                {
                    "start_mode": getattr(args, "start_mode", "live_gps"),
                    "start_source": start["start_source"],
                    "current_lat": start["start_lat"],
                    "current_lon": start["start_lon"],
                    "gps_wait_enabled": telemetry._parse_bool(getattr(args, "wait_gps", "true"), default=True),
                    "gps_wait_timeout_s": float(getattr(args, "gps_timeout_s", getattr(args, "start_timeout_s", 0.0))),
                    "gps_wait_elapsed_s": start.get("gps_wait_elapsed_s", 0.0),
                    **dict(start.get("gps_wait_snapshot", {})),
                }
            )
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    if args.print_plan:
        _write_json(out_dir / "plan.json", plan)
        start_source = str(plan.get("start_source", "plan_dir" if getattr(args, "plan_dir", None) else "explicit"))
        summary = {
            **plan,
            "mode": args.mode,
            "success": True,
            "reason": "PLAN_PRINTED",
            "start_source": start_source,
            "current_lat": plan["start_lat"],
            "current_lon": plan["start_lon"],
            "start_mode": plan.get("start_mode", getattr(args, "start_mode", "explicit")),
            "gps_wait_enabled": plan.get("gps_wait_enabled", False),
            "gps_wait_timeout_s": plan.get("gps_wait_timeout_s", 0.0),
            "gps_wait_elapsed_s": plan.get("gps_wait_elapsed_s", 0.0),
            "gps_ready": plan.get("gps_ready", "NA"),
            "gps_solution_valid": plan.get("gps_solution_valid", "NA"),
            "gps_sats": plan.get("gps_sats", "NA"),
            "gps_hdop": plan.get("gps_hdop", "NA"),
            "best_sats": plan.get("best_sats", "NA"),
            "best_hdop": plan.get("best_hdop", "NA"),
            "best_lat": plan.get("best_lat", "NA"),
            "best_lon": plan.get("best_lon", "NA"),
            "last_rmc_status": plan.get("last_rmc_status", "NA"),
            "last_gga_fix_quality": plan.get("last_gga_fix_quality", "NA"),
            "imu_present": plan.get("imu_present", "NA"),
            "imu_relative_yaw_deg": plan.get("imu_relative_yaw_deg", "NA"),
            "motion_calibration_loaded": motion_calibration_loaded(cal),
            "connector_mode_effective": cal.get("connector_mode_effective", plan.get("connector_mode_effective")),
            "continuous_drive_used": args.straight_motion_mode == "continuous",
            "gps_degraded_count": 0,
            "imu_heading_used_count": 0,
            "next_recommended_action": "Inspect summary.md and plan.json before running physical execution.",
            "ready_for_full_path_following": False,
        }
        write_summary_files(out_dir, summary, title="Physical Path Planner Plan")
        print(
            f"run --print-plan: {plan['segment_count']} segments, "
            f"fallback_to_repeated_pulses={cal['fallback_to_repeated_pulses']} "
            f"-> {out_dir}/plan.json (no serial opened)"
        )
        return 0
    if not ensure_port(args):
        return 2

    import serial  # local import: preview/diagnose --from-log never need pyserial

    handle = serial.Serial(args.port, baudrate=args.baud, timeout=0.5)
    try:
        rows, raw_lines, abort_reason = controller.run_controller(
            handle,
            segments=plan["segments"],  # type: ignore[arg-type]
            resolved_calibration=cal,
            start_lat=float(plan["start_lat"]),
            start_lon=float(plan["start_lon"]),
            start_yaw_deg=args.start_yaw_deg,
            goal_lat=float(plan["goal_lat"]),
            goal_lon=float(plan["goal_lon"]),
            event_timeout_s=args.event_timeout_s,
            heartbeat_timeout_s=args.heartbeat_timeout_s,
            rc_neutral_wait_s=args.rc_neutral_wait_s,
            gps_degradation_policy=args.gps_degradation_policy,
            manual_override_mode=args.manual_override_mode,
            left_fixed_pulses=args.left_fixed_pulses,
            right_fixed_pulses=args.right_fixed_pulses,
            straight_motion_mode=args.straight_motion_mode,
            live_update_hz=args.live_update_hz,
            live_ttl_ms=args.live_ttl_ms,
        )
    finally:
        handle.close()

    summary = controller.build_controller_summary(
        rows,
        start_lat=float(plan["start_lat"]),
        start_lon=float(plan["start_lon"]),
        goal_lat=float(plan["goal_lat"]),
        goal_lon=float(plan["goal_lon"]),
        goal_distance_m=float(plan["goal_distance_m"]),
        fallback_to_repeated_pulses=bool(cal["fallback_to_repeated_pulses"]),
        abort_reason=abort_reason,
    )
    summary = {
        **summary,
        "mode": args.mode,
        "success": summary.get("aborted") is False,
        "reason": "OK" if summary.get("aborted") is False else str(abort_reason),
        "start_source": str(plan.get("start_source", "plan_dir" if getattr(args, "plan_dir", None) else "explicit")),
        "current_lat": plan["start_lat"],
        "current_lon": plan["start_lon"],
        "start_mode": plan.get("start_mode", getattr(args, "start_mode", "explicit")),
        "gps_wait_enabled": plan.get("gps_wait_enabled", False),
        "gps_wait_timeout_s": plan.get("gps_wait_timeout_s", 0.0),
        "gps_wait_elapsed_s": plan.get("gps_wait_elapsed_s", 0.0),
        "gps_ready": plan.get("gps_ready", "NA"),
        "gps_solution_valid": plan.get("gps_solution_valid", "NA"),
        "gps_sats": plan.get("gps_sats", "NA"),
        "gps_hdop": plan.get("gps_hdop", "NA"),
        "best_sats": plan.get("best_sats", "NA"),
        "best_hdop": plan.get("best_hdop", "NA"),
        "best_lat": plan.get("best_lat", "NA"),
        "best_lon": plan.get("best_lon", "NA"),
        "last_rmc_status": plan.get("last_rmc_status", "NA"),
        "last_gga_fix_quality": plan.get("last_gga_fix_quality", "NA"),
        "imu_present": plan.get("imu_present", "NA"),
        "imu_relative_yaw_deg": plan.get("imu_relative_yaw_deg", "NA"),
        "motion_calibration_loaded": motion_calibration_loaded(cal),
        "connector_mode_effective": cal.get("connector_mode_effective", plan.get("connector_mode_effective")),
        "continuous_drive_used": summary.get("continuous_drive_used", args.straight_motion_mode == "continuous"),
        "next_recommended_action": (
            "Inspect path trace and run summary before any longer test."
            if summary.get("aborted") is False else
            "Inspect abort reason, raw USB log, and final motor command fields."
        ),
        "ready_for_full_path_following": False,
    }
    _write_json(out_dir / "run_summary.json", summary)
    write_summary_files(out_dir, summary, title="Physical Path Planner Run")
    _write_rows_csv(out_dir / "run_rows.csv", rows)
    _write_raw_log(out_dir / "run_serial.log", raw_lines)
    print(
        f"run: abort_reason={abort_reason}, pulses={summary['pulse_count']}, "
        f"valid={summary['valid_pulse_count']} -> {out_dir}"
    )
    return 1 if summary["aborted"] else 0


def cmd_diagnose(args: argparse.Namespace) -> int:
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    if args.from_log:
        log_path = Path(args.from_log)
        raw_lines = log_path.read_text().splitlines()
        rows = load_rows_from_log(log_path)
    else:
        if not ensure_port(args):
            return 2
        import serial  # local import: --from-log path never needs pyserial

        raw_lines = []
        handle = serial.Serial(args.port, baudrate=args.baud, timeout=0.5)
        try:
            deadline = time.monotonic() + args.duration_s
            while time.monotonic() < deadline:
                raw = handle.readline()
                if raw:
                    line = raw.decode("utf-8", errors="replace").strip()
                    print(line)
                    raw_lines.append(line)
        finally:
            handle.close()
        rows = telemetry.parse_usbdbg_rows("\n".join(raw_lines))

    summary = diagnose_summary(rows)
    summary = {
        **summary,
        "success": True,
        "reason": "OK",
        "next_recommended_action": "Inspect summary.md for GPS, IMU, RC, and guarded pulse heartbeat status.",
        "ready_for_full_path_following": False,
    }
    _write_json(out_dir / "diagnose_summary.json", summary)
    write_summary_files(out_dir, summary, title="Physical Path Planner Diagnose")
    if raw_lines:
        _write_raw_log(out_dir / "diagnose_serial.log", raw_lines)
    print(
        f"diagnose: {summary['row_count']} rows, {summary['heartbeat_count']} heartbeats, "
        f"last_gps_block_reason={summary['last_gps_block_reason']} -> {out_dir}"
    )
    return 0


# --- Argument parser ----------------------------------------------------------


def _add_goal_arguments(parser: argparse.ArgumentParser, *, require_start: bool = True) -> None:
    parser.add_argument("--start-lat", type=float, required=require_start)
    parser.add_argument("--start-lon", type=float, required=require_start)
    parser.add_argument(
        "--goal-mode",
        choices=["absolute", "relative_enu", "relative_latlon", "bearing_distance"],
        default="absolute",
    )
    parser.add_argument("--goal-lat", type=float, default=None)
    parser.add_argument("--goal-lon", type=float, default=None)
    parser.add_argument("--goal-east-m", type=float, default=None)
    parser.add_argument("--goal-north-m", type=float, default=None)
    parser.add_argument("--goal-dlat", type=float, default=None)
    parser.add_argument("--goal-dlon", type=float, default=None)
    parser.add_argument("--goal-bearing-deg", type=float, default=None)
    parser.add_argument("--goal-distance-m", type=float, default=None)
    parser.add_argument(
        "--path-shape",
        choices=["diagonal_rectangle_serpentine", "direct_line"],
        default="diagonal_rectangle_serpentine",
    )
    parser.add_argument("--workspace-width-m", type=float, default=None)
    parser.add_argument("--step-spacing-m", type=float, default=0.5)
    parser.add_argument(
        "--diagonal-orientation", default="A_top_left_to_B_bottom_right"
    )
    parser.add_argument("--max-segment-pulses", type=int, default=8)
    parser.add_argument("--nominal-forward-pulse-m", type=float, default=0.30)


def _add_calibration_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--calibration-mode", default="auto")
    parser.add_argument("--motion-calibration-json", default=str(calibration.DEFAULT_MOTION_CALIBRATION))
    parser.add_argument("--fine-calibration-json", default=None)
    parser.add_argument("--turn-calibration-json", default=None)
    parser.add_argument("--turn-angle-calibration-json", default=None)
    parser.add_argument("--smooth-turn-calibration-json", default=None)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="physical_path_planner",
        description="Unified physical rover tools: station hardware manual, USB pulse tests, diagnostics, calibration, and supervised planning.",
    )
    sub = parser.add_subparsers(
        dest="mode",
        required=True,
        metavar="{diagnose,gps-wait,rc-input-diagnose,manual-rc,manual-control,station-hw-diagnose,station-hw-manual,usb-pulse-test,usb-drive-live,tune-motion,guarded-pulse-ready,calibrate-turn,preview,execute-plan,run}",
    )

    gps_p = sub.add_parser("gps-wait", help="wait for usable GPS start fix; no motion")
    gps_p.add_argument("--port", default=None)
    gps_p.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    gps_p.add_argument("--from-log", default=None)
    gps_p.add_argument("--timeout-s", type=float, default=300.0)
    gps_p.add_argument("--status-interval-s", type=float, default=2.0)
    gps_p.add_argument("--min-sats", type=float, default=5.0)
    gps_p.add_argument("--max-hdop", type=float, default=2.5)
    gps_p.add_argument("--out-dir", default="outputs/physical_path_planning/gps_wait")
    gps_p.set_defaults(handler=cmd_gps_wait)

    preview_p = sub.add_parser("preview", help="build + render the plan (captures live/cached GPS start when omitted)")
    _add_goal_arguments(preview_p, require_start=False)
    _add_calibration_arguments(preview_p)
    preview_p.add_argument("--port", default=None)
    preview_p.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    preview_p.add_argument(
        "--start-mode",
        choices=["live_gps", "cached_gps", "explicit"],
        default="live_gps",
        help="how to resolve the plan start when --start-lat/--start-lon are omitted",
    )
    preview_p.add_argument("--start-timeout-s", type=float, default=120.0)
    preview_p.add_argument("--wait-gps", choices=["true", "false"], default="true")
    preview_p.add_argument("--gps-timeout-s", type=float, default=300.0)
    preview_p.add_argument("--gps-status-interval-s", type=float, default=2.0)
    preview_p.add_argument("--gps-min-sats", type=float, default=5.0)
    preview_p.add_argument("--gps-max-hdop", type=float, default=2.5)
    preview_p.add_argument("--allow-cached-start", choices=["true", "false"], default="true")
    preview_p.add_argument("--max-cached-start-age-s", type=float, default=600.0)
    preview_p.add_argument("--cached-start-max-age-ms", type=int, default=10000)
    preview_p.add_argument("--from-log", default=None, help="parse saved telemetry for start GPS instead of opening serial")
    preview_p.add_argument("--out-dir", default="outputs/physical_path_planning/preview")
    preview_p.add_argument("--png", dest="png", action="store_true", default=True)
    preview_p.add_argument("--no-png", dest="png", action="store_false")
    preview_p.set_defaults(handler=cmd_preview)

    cal_p = sub.add_parser(
        "calibrate-turn",
        help="run guarded pulse turn angle calibration",
    )
    cal_p.add_argument("--port", default=None)
    cal_p.add_argument("--direction", choices=["left", "right"], default=None)
    cal_p.add_argument("--mode", default="turn_left")
    cal_p.add_argument("--b-cmd", type=float, default=None)
    cal_p.add_argument("--pulse-ms", type=int, default=None)
    cal_p.add_argument("--max-abs-b", type=float, default=0.35)
    cal_p.add_argument("--max-ms", type=int, default=1500)
    cal_p.add_argument("--upload", choices=["true", "false", "auto"], default="auto")
    cal_p.add_argument("--target-angle-deg", type=float, default=90.0)
    cal_p.add_argument("--angle-tolerance-deg", type=float, default=10.0)
    cal_p.add_argument("--save-turn-calibration", default="true")
    cal_p.add_argument("--turn-calibration-out", default=DEFAULT_TURN_CALIBRATION_OUT)
    cal_p.add_argument("--out-dir", default="outputs/physical_path_planning/calibration")
    cal_p.add_argument("--script", default=DEFAULT_GUARDED_PULSE_CALIBRATION_SCRIPT)
    cal_p.add_argument(
        "--print-cmd",
        action="store_true",
        help="print the shell-out command and exit (no firmware, no serial)",
    )
    cal_p.set_defaults(handler=cmd_calibrate_turn)

    rc_diag_p = sub.add_parser(
        "rc-input-diagnose",
        help="upload/read the read-only RC input channel diagnostic",
    )
    rc_diag_p.add_argument("--port", default=None)
    rc_diag_p.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    rc_diag_p.add_argument("--duration-s", type=float, default=20.0)
    rc_diag_p.add_argument("--upload", choices=["true", "false", "auto"], default="true")
    rc_diag_p.add_argument("--from-log", default=None)
    rc_diag_p.add_argument("--out-dir", default="outputs/physical_path_planning/rc_input_diagnose")
    rc_diag_p.add_argument("--sketch", default=DEFAULT_RC_INPUT_DIAGNOSE_SKETCH)
    rc_diag_p.add_argument(
        "--print-cmd",
        action="store_true",
        help="print read-only probe upload commands and exit",
    )
    rc_diag_p.set_defaults(handler=cmd_rc_input_diagnose)

    manual_p = sub.add_parser("manual-rc", help="upload and validate manual RC recovery")
    manual_p.add_argument("--port", default=None)
    manual_p.add_argument("--upload", choices=["true", "false", "auto"], default="true")
    manual_p.add_argument("--validate", choices=["true", "false"], default="true")
    manual_p.add_argument("--diagnose-only", choices=["true", "false"], default="false")
    manual_p.add_argument("--duration-s", type=float, default=45.0)
    manual_p.add_argument("--log", default=None)
    manual_p.add_argument("--rc-input-mode", choices=["auto", "old_known_good", "ppm", "pwm", "sbus"], default="old_known_good")
    manual_p.add_argument("--mode-channel-index", type=int, default=4)
    manual_p.add_argument("--steer-channel-index", type=int, default=0)
    manual_p.add_argument("--throttle-channel-index", type=int, default=1)
    manual_p.add_argument("--manual-mode-threshold-us", type=int, default=None)
    manual_p.add_argument("--print-rc-mapping", choices=["true", "false"], default="false")
    manual_p.add_argument("--out-dir", default="outputs/physical_path_planning/manual_rc")
    manual_p.add_argument("--upload-script", default=DEFAULT_MANUAL_RC_UPLOAD_SCRIPT)
    manual_p.add_argument("--validate-script", default=DEFAULT_MANUAL_RC_VALIDATE_SCRIPT)
    manual_p.add_argument(
        "--print-cmd",
        action="store_true",
        help="print upload/validation commands and exit",
    )
    manual_p.set_defaults(handler=cmd_manual_rc)

    manual_control_p = sub.add_parser(
        "manual-control",
        help="upload and monitor PPM physical manual control with full telemetry display",
    )
    manual_control_p.add_argument("--port", default=None)
    manual_control_p.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    manual_control_p.add_argument("--upload", choices=["true", "false", "auto"], default="true")
    manual_control_p.add_argument("--validate", choices=["true", "false"], default="true")
    manual_control_p.add_argument("--duration-s", type=float, default=0.0)
    manual_control_p.add_argument("--from-log", default=None)
    manual_control_p.add_argument("--mode-channel-index", type=int, default=4)
    manual_control_p.add_argument("--verbose-raw", choices=["true", "false"], default="false")
    manual_control_p.add_argument("--out-dir", default="outputs/physical_path_planning/manual_control")
    manual_control_p.add_argument(
        "--print-cmd",
        action="store_true",
        help="print upload commands and exit",
    )
    manual_control_p.set_defaults(handler=cmd_manual_control)

    def add_station_hw_parser(name: str, *, diagnose_only: bool) -> None:
        station_p = sub.add_parser(
            name,
            help=(
                "read-only physical station hardware link diagnostic"
                if diagnose_only else
                "deprecated serial-frame hardware monitor; use manual-control for PPM control"
            ),
        )
        station_p.add_argument("--port", default=None)
        station_p.add_argument("--baud", type=int, default=DEFAULT_BAUD)
        station_p.add_argument(
            "--duration-s",
            type=float,
            default=20.0 if diagnose_only else 0.0,
            help=(
                "monitor duration in seconds"
                if diagnose_only else
                "monitor duration in seconds; <=0 means continuous until Ctrl-C"
            ),
        )
        station_p.add_argument("--upload", choices=["true", "false", "auto"], default="auto")
        station_p.add_argument("--from-log", default=None)
        station_p.add_argument("--verbose-raw", choices=["true", "false"], default="false")
        station_p.add_argument(
            "--print-cmd",
            action="store_true",
            help="print firmware commands and exit",
        )
        station_p.add_argument("--print-command", choices=["true", "false"], default="false")
        station_p.add_argument(
            "--out-dir",
            default=(
                "outputs/physical_path_planning/station_hw_diagnose"
                if diagnose_only else
                "outputs/physical_path_planning/station_hw_manual"
            ),
        )
        station_p.set_defaults(handler=cmd_station_hw_diagnose if diagnose_only else cmd_station_hw_manual)

    add_station_hw_parser("station-hw-diagnose", diagnose_only=True)
    add_station_hw_parser("station-hw-manual", diagnose_only=False)

    station_drive_p = sub.add_parser(
        "usb-pulse-test",
        help="laptop USB bounded A/B pulse motor validation",
    )
    station_drive_p.add_argument("--port", default=None)
    station_drive_p.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    station_drive_p.add_argument("--upload", choices=["true", "false", "auto"], default="auto")
    station_drive_p.add_argument("--max-abs-a", type=float, default=0.35)
    station_drive_p.add_argument("--max-abs-b", type=float, default=0.35)
    station_drive_p.add_argument("--max-ms", type=int, default=1000)
    station_drive_p.add_argument("--event-timeout-s", type=float, default=controller.DEFAULT_EVENT_TIMEOUT_S)
    station_drive_p.add_argument("--heartbeat-timeout-s", type=float, default=controller.DEFAULT_HEARTBEAT_TIMEOUT_S)
    station_drive_p.add_argument("--single", choices=["forward", "backward", "left", "right", "turn_left", "turn_right"], default=None)
    station_drive_p.add_argument("--sequence", default=None)
    station_drive_p.add_argument("--require-rc-input", choices=["true", "false"], default="false")
    station_drive_p.add_argument("--require-enter", choices=["true", "false"], default="true")
    station_drive_p.add_argument("--interactive-visible-motion", choices=["true", "false"], default="true")
    station_drive_p.add_argument("--abort-on-invalid", choices=["true", "false"], default="true")
    station_drive_p.add_argument("--verbose-raw", choices=["true", "false"], default="false")
    station_drive_p.add_argument("--out-dir", default="outputs/physical_path_planning/usb_pulse_test")
    station_drive_p.add_argument(
        "--print-cmd",
        action="store_true",
        help="print bounded USB pulse serial commands and exit",
    )
    station_drive_p.add_argument("--print-command", choices=["true", "false"], default="false")
    station_drive_p.set_defaults(handler=cmd_usb_pulse_test)

    live_p = sub.add_parser(
        "usb-drive-live",
        help="continuous laptop USB A/B setpoint drive with firmware deadman",
    )
    live_p.add_argument("--port", default=None)
    live_p.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    live_p.add_argument("--a", type=float, required=True)
    live_p.add_argument("--b", type=float, required=True)
    live_p.add_argument("--duration-s", type=float, required=True)
    live_p.add_argument("--update-hz", type=float, default=8.0)
    live_p.add_argument("--ttl-ms", type=int, default=350)
    live_p.add_argument("--max-abs-a", type=float, default=0.35)
    live_p.add_argument("--max-abs-b", type=float, default=0.35)
    live_p.add_argument("--max-duration-s", type=float, default=3.0)
    live_p.add_argument("--event-timeout-s", type=float, default=controller.DEFAULT_EVENT_TIMEOUT_S)
    live_p.add_argument("--upload", choices=["true", "false", "auto"], default="auto")
    live_p.add_argument("--verbose-raw", choices=["true", "false"], default="false")
    live_p.add_argument("--print-command", choices=["true", "false"], default="false")
    live_p.add_argument("--out-dir", default="outputs/physical_path_planning/usb_drive_live")
    live_p.set_defaults(handler=cmd_usb_drive_live)

    tune_p = sub.add_parser(
        "tune-motion",
        help="interactive visual/IMU-assisted USB pulse calibration",
    )
    tune_p.add_argument("--port", default=None)
    tune_p.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    tune_p.add_argument(
        "--primitive",
        choices=["forward", "backward", "left", "right", "turn-left-90", "turn-right-90"],
        required=True,
    )
    tune_p.add_argument("--upload", choices=["true", "false", "auto"], default="auto")
    tune_p.add_argument("--max-abs-a", type=float, default=tuning.MAX_ABS_A)
    tune_p.add_argument("--max-abs-b", type=float, default=tuning.MAX_ABS_B)
    tune_p.add_argument("--max-ms", type=int, default=tuning.MAX_MS)
    tune_p.add_argument("--event-timeout-s", type=float, default=controller.DEFAULT_EVENT_TIMEOUT_S)
    tune_p.add_argument("--heartbeat-timeout-s", type=float, default=controller.DEFAULT_HEARTBEAT_TIMEOUT_S)
    tune_p.add_argument("--require-enter", choices=["true", "false"], default="true")
    tune_p.add_argument("--max-iterations", type=int, default=12)
    tune_p.add_argument("--verbose-raw", choices=["true", "false"], default="false")
    tune_p.add_argument("--print-candidate", choices=["true", "false"], default="false")
    tune_p.add_argument("--calibration-out", default=str(calibration.DEFAULT_MOTION_CALIBRATION))
    tune_p.add_argument("--out-dir", default="outputs/physical_path_planning/tune_motion")
    tune_p.set_defaults(handler=cmd_tune_motion)

    def add_guarded_pulse_ready_parser(name: str) -> None:
        guarded_p = sub.add_parser(
            name,
            help="upload/check IMU-enabled guarded pulse firmware",
        )
        guarded_p.add_argument("--port", default=None)
        guarded_p.add_argument("--baud", type=int, default=DEFAULT_BAUD)
        guarded_p.add_argument("--upload", choices=["true", "false", "auto"], default="true")
        guarded_p.add_argument("--duration-s", type=float, default=15.0)
        guarded_p.add_argument("--max-abs-a", type=float, default=0.35)
        guarded_p.add_argument("--max-abs-b", type=float, default=0.35)
        guarded_p.add_argument("--max-ms", type=int, default=1500)
        guarded_p.add_argument("--out-dir", default="outputs/physical_path_planning/guarded_pulse_ready")
        guarded_p.add_argument(
            "--print-cmd",
            action="store_true",
            help="print firmware commands and exit",
        )
        guarded_p.set_defaults(handler=cmd_guarded_pulse_ready, deprecated_alias=False)

    add_guarded_pulse_ready_parser("guarded-pulse-ready")

    for name in ("run", "execute-plan"):
        run_p = sub.add_parser(name, help="drive the continuous-motion controller over a plan")
        _add_goal_arguments(run_p, require_start=False)
        _add_calibration_arguments(run_p)
        run_p.add_argument("--plan-dir", default=None)
        run_p.add_argument("--port", default=None)
        run_p.add_argument("--baud", type=int, default=DEFAULT_BAUD)
        run_p.add_argument(
            "--start-mode",
            choices=["live_gps", "cached_gps", "explicit"],
            default="live_gps",
        )
        run_p.add_argument("--wait-gps", choices=["true", "false"], default="true")
        run_p.add_argument("--gps-timeout-s", type=float, default=300.0)
        run_p.add_argument("--gps-status-interval-s", type=float, default=2.0)
        run_p.add_argument("--gps-min-sats", type=float, default=5.0)
        run_p.add_argument("--gps-max-hdop", type=float, default=2.5)
        run_p.add_argument("--allow-cached-start", choices=["true", "false"], default="true")
        run_p.add_argument("--max-cached-start-age-s", type=float, default=600.0)
        run_p.add_argument("--start-timeout-s", type=float, default=120.0)
        run_p.add_argument("--cached-start-max-age-ms", type=int, default=10000)
        run_p.add_argument("--from-log", default=None)
        run_p.add_argument("--start-yaw-deg", type=float, default=None)
        run_p.add_argument("--event-timeout-s", type=float, default=controller.DEFAULT_EVENT_TIMEOUT_S)
        run_p.add_argument(
            "--heartbeat-timeout-s", type=float, default=controller.DEFAULT_HEARTBEAT_TIMEOUT_S
        )
        run_p.add_argument(
            "--rc-neutral-wait-s", type=float, default=controller.DEFAULT_RC_NEUTRAL_WAIT_S
        )
        run_p.add_argument(
            "--gps-degradation-policy",
            choices=["continue", "pause", "abort"],
            default=controller.DEFAULT_GPS_DEGRADATION_POLICY,
        )
        run_p.add_argument(
            "--manual-override-mode",
            choices=["abort", "warn", "continue"],
            default=controller.DEFAULT_MANUAL_OVERRIDE_MODE,
        )
        run_p.add_argument("--left-fixed-pulses", type=int, default=12)
        run_p.add_argument("--right-fixed-pulses", type=int, default=12)
        run_p.add_argument("--straight-motion-mode", choices=["continuous", "pulse"], default="continuous")
        run_p.add_argument("--live-update-hz", type=float, default=8.0)
        run_p.add_argument("--live-ttl-ms", type=int, default=350)
        run_p.add_argument("--out-dir", default="outputs/physical_path_planning/run")
        run_p.add_argument(
            "--print-plan",
            action="store_true",
            help="build + write the plan and exit (no serial opened)",
        )
        run_p.set_defaults(handler=cmd_run)

    diag_p = sub.add_parser("diagnose", help="read-only telemetry summary (live port or --from-log)")
    diag_p.add_argument("--port", default=None)
    diag_p.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    diag_p.add_argument("--from-log", default=None, help="parse a saved serial log instead of a port")
    diag_p.add_argument("--duration-s", type=float, default=5.0)
    diag_p.add_argument("--out-dir", default="outputs/physical_path_planning/diagnose")
    diag_p.set_defaults(handler=cmd_diagnose)

    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    normalized_argv = list(sys.argv[1:] if argv is None else argv)
    deprecated_station_manual_alias = False
    deprecated_station_drive_alias = False
    if normalized_argv and normalized_argv[0] == "station-manual":
        normalized_argv[0] = "usb-pulse-test"
        deprecated_station_manual_alias = True
    if normalized_argv and normalized_argv[0] == "station-drive":
        normalized_argv[0] = "usb-pulse-test"
        deprecated_station_drive_alias = True
    args = parser.parse_args(normalized_argv)
    if deprecated_station_manual_alias:
        args.deprecated_station_manual_alias = True
    if deprecated_station_drive_alias:
        args.deprecated_station_drive_alias = True
    return int(args.handler(args))


if __name__ == "__main__":
    raise SystemExit(main())
