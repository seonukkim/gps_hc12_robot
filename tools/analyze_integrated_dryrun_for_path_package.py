from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path
from typing import Sequence


def _parse_bool(value: object) -> bool:
    return str(value).strip().lower() in {"1", "true", "ok", "yes", "ready"}


def _parse_float(value: object) -> float | None:
    text = str(value).strip()
    if text.upper() in {"", "NA", "NAN", "NONE", "NULL"}:
        return None
    try:
        parsed = float(text)
    except ValueError:
        return None
    return parsed if math.isfinite(parsed) else None


def parse_log_rows(text: str) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    for line in text.splitlines():
        values = {key: value for key, value in re.findall(r"([A-Za-z0-9_]+)=([^,\s]+)", line)}
        if values:
            rows.append(values)
    return rows


def _last_values(rows: Sequence[dict[str, str]]) -> dict[str, str]:
    merged: dict[str, str] = {}
    for row in rows:
        merged.update(row)
    return merged


def analyze_rows(rows: Sequence[dict[str, str]]) -> dict[str, object]:
    values = _last_values(rows)
    gps_sats = _parse_float(values.get("gps_sats", "NA"))
    gps_hdop = _parse_float(values.get("gps_hdop", "NA"))
    gps_position_ok = (
        values.get("gps_block_reason") == "OK"
        and values.get("position_source") == "gps"
        and gps_sats is not None
        and gps_sats >= 4
        and gps_hdop is not None
        and gps_hdop <= 3.0
    )
    imu_ok = (
        values.get("imu_type") == "BMI160"
        and _parse_bool(values.get("imu_data_plausible"))
        and _parse_bool(values.get("imu_calibrated", "true"))
    )
    motor_safety_ok = (
        not _parse_bool(values.get("physical_compile_gate"))
        and not _parse_bool(values.get("physical_path_following_enable"))
        and not _parse_bool(values.get("allow_motor_output"))
        and not _parse_bool(values.get("physical_output_active"))
    )
    gps_course_available = _parse_float(values.get("gps_course_deg", "NA")) is not None or _parse_bool(
        values.get("gps_course_output_valid")
    )
    gps_course_block_reason = values.get("gps_course_output_block_reason", "")
    path_block_reason = values.get("path_following_block_reason", "")
    if gps_course_available:
        gps_course_status = "AVAILABLE"
        gps_course_skip_reason = ""
    elif gps_course_block_reason == "NO_ACCEPTED_COURSE_YET" or path_block_reason == "NO_HEADING":
        gps_course_status = "SKIPPED_NO_MOTION_OR_TETHERED"
        gps_course_skip_reason = gps_course_block_reason or path_block_reason
    else:
        gps_course_status = "NOT_AVAILABLE"
        gps_course_skip_reason = gps_course_block_reason or path_block_reason or "UNKNOWN"

    active_target_source = values.get("active_target_source", "unknown")
    current_target_source_is_compile_time = active_target_source == "compile_time"
    live_path_package_connected = active_target_source not in {"compile_time", "unknown", ""}
    if current_target_source_is_compile_time:
        recommendation = (
            "Do not proceed to motor tests. Validate path package offline and implement "
            "package-to-firmware/station target bridge."
        )
        reason = "ACTIVE_TARGET_SOURCE_COMPILE_TIME"
    elif not live_path_package_connected:
        recommendation = "Do not proceed to motor tests. Live target source is not a generated path package."
        reason = "ACTIVE_TARGET_SOURCE_UNKNOWN"
    else:
        recommendation = "Live target source is not compile_time; continue no-motion validation only."
        reason = "OK"

    return {
        "gps_position_ok": gps_position_ok,
        "gps_sats": gps_sats,
        "gps_hdop": gps_hdop,
        "position_source": values.get("position_source", "unknown"),
        "imu_ok": imu_ok,
        "imu_type": values.get("imu_type", ""),
        "imu_data_plausible": _parse_bool(values.get("imu_data_plausible")),
        "imu_calibrated": _parse_bool(values.get("imu_calibrated", "true")),
        "motor_safety_ok": motor_safety_ok,
        "physical_output_active": _parse_bool(values.get("physical_output_active")),
        "gps_course_status": gps_course_status,
        "gps_course_skip_reason": gps_course_skip_reason,
        "gps_course_output_block_reason": gps_course_block_reason,
        "path_following_block_reason": path_block_reason,
        "active_target_source": active_target_source,
        "live_path_package_connected": live_path_package_connected,
        "current_target_source_is_compile_time": current_target_source_is_compile_time,
        "reason": reason,
        "recommendation": recommendation,
        "motor_command_generated": False,
    }


def analyze_files(paths: Sequence[Path]) -> dict[str, object]:
    rows: list[dict[str, str]] = []
    for path in paths:
        rows.extend(parse_log_rows(path.read_text(encoding="utf-8", errors="replace")))
    result = analyze_rows(rows)
    result["log_paths"] = [str(path) for path in paths]
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Analyze integrated dry-run logs for Stage 11 path-package target linkage.")
    parser.add_argument("logs", nargs="+")
    parser.add_argument("--json-out")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    paths = [Path(arg) for arg in args.logs]
    missing = [path for path in paths if not path.exists()]
    if missing:
        for path in missing:
            print(f"missing_log={path}")
        return 2
    result = analyze_files(paths)
    if args.json_out:
        Path(args.json_out).write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    for key in (
        "gps_position_ok",
        "imu_ok",
        "motor_safety_ok",
        "gps_course_status",
        "gps_course_skip_reason",
        "active_target_source",
        "live_path_package_connected",
        "current_target_source_is_compile_time",
        "reason",
        "recommendation",
        "motor_command_generated",
    ):
        print(f"{key}={result[key]}")
    return 1 if result["current_target_source_is_compile_time"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
