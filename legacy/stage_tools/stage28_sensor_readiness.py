from __future__ import annotations

import argparse
import json
import math
import time
from collections import Counter
from pathlib import Path
from typing import Sequence

try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401

from tools import station_path_package_tracker
from tools.path_no_motion_validation import PathPackageResolutionError, resolve_path_package


def _parse_bool(value: object) -> bool | None:
    if value is None:
        return None
    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "ok", "active"}:
        return True
    if text in {"0", "false", "no", "off", "inactive"}:
        return False
    return None


def _parse_float(value: object) -> float | None:
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


def _range(values: Sequence[float]) -> dict[str, float | None]:
    return {"min": min(values) if values else None, "max": max(values) if values else None}


def parse_rows(text: str) -> list[dict[str, str]]:
    return station_path_package_tracker.parse_usbdbg_rows(text)


def summarize_rows(rows: Sequence[dict[str, str]], package: dict[str, object] | None = None) -> dict[str, object]:
    gps_reasons = Counter(row.get("gps_block_reason", "NA") for row in rows)
    position_sources = Counter(row.get("position_source", "NA") for row in rows)
    gps_sats = [value for row in rows if (value := _parse_float(row.get("gps_sats"))) is not None]
    gps_hdop = [value for row in rows if (value := _parse_float(row.get("gps_hdop"))) is not None]
    yaw_values = [value for row in rows if (value := _parse_float(row.get("imu_relative_yaw_deg"))) is not None]
    lat_lon_rows = [
        row for row in rows
        if _parse_float(row.get("current_lat")) is not None and _parse_float(row.get("current_lon")) is not None
    ]
    local_pose_count = 0
    if package is not None:
        for row in lat_lon_rows:
            lat = _parse_float(row.get("current_lat"))
            lon = _parse_float(row.get("current_lon"))
            if lat is not None and lon is not None and station_path_package_tracker.lat_lon_to_local(package, lat, lon) is not None:
                local_pose_count += 1

    imu_rows = [
        row for row in rows
        if row.get("imu_type") == "BMI160"
        and _parse_bool(row.get("imu_present")) is True
        and _parse_bool(row.get("imu_data_plausible")) is True
    ]
    imu_ready_rows = [row for row in imu_rows if _parse_bool(row.get("imu_calibrated")) is True]
    imu_calibrating_rows = [
        row for row in imu_rows
        if _parse_bool(row.get("imu_calibrated")) is False
        or row.get("imu_heading_block_reason") == "CALIBRATING"
    ]
    rc_ok_count = sum(1 for row in rows if _parse_bool(row.get("rc_ok")) is True)
    neutral_ok_count = sum(1 for row in rows if _parse_bool(row.get("neutral_ok")) is True)
    physical_output_active_count = sum(1 for row in rows if _parse_bool(row.get("physical_output_active")) is True)
    final_nonzero_count = sum(
        1 for row in rows
        if abs(_parse_float(row.get("final_left_cmd")) or 0.0) > 1e-9
        or abs(_parse_float(row.get("final_right_cmd")) or 0.0) > 1e-9
    )
    gps_ok = any(row.get("gps_block_reason") == "OK" and row.get("position_source") == "gps" for row in rows)
    local_pose_available = local_pose_count > 0
    imu_ready = bool(imu_ready_rows)
    rc_ready = rc_ok_count > 0 and neutral_ok_count > 0
    explicit_gps_no_location = any(row.get("gps_block_reason") == "NO_LOCATION" for row in rows)

    gps_update_rows = [
        row for row in rows
        if any(key in row for key in ("gps_block_reason", "position_source", "gps_chars", "gps_sats", "gps_hdop", "current_lat", "current_lon"))
    ]
    if physical_output_active_count > 0 or final_nonzero_count > 0:
        readiness = "BLOCKED_MOTOR_OUTPUT"
    elif not rc_ready:
        readiness = "BLOCKED_MANUAL_RC"
    elif imu_calibrating_rows and not imu_ready:
        readiness = "BLOCKED_IMU_CALIBRATING"
    elif gps_ok and local_pose_available and imu_ready and rc_ready:
        readiness = "READY_GPS_IMU_CLOSED_LOOP"
    elif imu_ready and rc_ready:
        readiness = "READY_IMU_ONLY_HEADING"
    elif rc_ready and rows:
        readiness = "READY_MANUAL_RC_ONLY"
    elif not gps_update_rows:
        readiness = "BLOCKED_GPS_NO_DATA"
    elif explicit_gps_no_location:
        readiness = "BLOCKED_GPS_WAIT_LONGER"
    else:
        readiness = "READY_OPEN_LOOP_WITH_SENSORS" if rc_ready else "BLOCKED_MANUAL_RC"

    return {
        "row_count": len(rows),
        "gps_block_reason_counts": dict(gps_reasons),
        "position_source_counts": dict(position_sources),
        "gps_sats_range": _range(gps_sats),
        "gps_hdop_range": _range(gps_hdop),
        "current_lat_lon_available_count": len(lat_lon_rows),
        "local_pose_available": local_pose_available,
        "local_pose_available_count": local_pose_count,
        "imu_type": next((row.get("imu_type") for row in reversed(rows) if row.get("imu_type")), "NA"),
        "imu_present": any(_parse_bool(row.get("imu_present")) is True for row in rows),
        "imu_data_plausible": any(_parse_bool(row.get("imu_data_plausible")) is True for row in rows),
        "imu_calibrated": imu_ready,
        "imu_heading_block_reason": next((row.get("imu_heading_block_reason") for row in reversed(rows) if row.get("imu_heading_block_reason")), "NA"),
        "imu_relative_yaw_deg_range": _range(yaw_values),
        "rc_ok_count": rc_ok_count,
        "neutral_ok_count": neutral_ok_count,
        "physical_output_active_count": physical_output_active_count,
        "final_nonzero_count": final_nonzero_count,
        "sensor_readiness": readiness,
        "motor_command_generated": False,
        "ready_for_full_path_following": False,
    }


def write_summary(out_dir: Path, summary: dict[str, object]) -> None:
    (out_dir / "sensor_readiness_summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    lines = ["# Stage 28A Sensor Readiness", "", "No motor commands are sent.", ""]
    lines.extend(f"- {key}: `{value}`" for key, value in summary.items())
    (out_dir / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def run(args: argparse.Namespace) -> int:
    package = None
    if args.path_package:
        try:
            selected = resolve_path_package(args.path_package)
            package = json.loads(selected.read_text(encoding="utf-8"))
        except PathPackageResolutionError:
            package = None
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    raw_lines: list[str] = []
    if args.log:
        raw_text = Path(args.log).read_text(encoding="utf-8")
        raw_lines = raw_text.splitlines()
    else:
        try:
            import serial  # type: ignore[import-not-found]
        except ImportError as exc:
            raise RuntimeError("pyserial is not available; cannot monitor Stage28 sensors") from exc
        deadline = time.monotonic() + args.duration_s
        with serial.Serial(args.port, baudrate=115200, timeout=0.5) as handle:
            while time.monotonic() < deadline:
                raw = handle.readline()
                if raw:
                    line = raw.decode("utf-8", errors="replace").strip()
                    print(line)
                    raw_lines.append(line)
    rows = parse_rows("\n".join(raw_lines))
    (out_dir / "raw_usbdbg.log").write_text("\n".join(raw_lines) + ("\n" if raw_lines else ""), encoding="utf-8")
    summary = summarize_rows(rows, package)
    write_summary(out_dir, summary)
    print(f"sensor_readiness={summary['sensor_readiness']}")
    print("motor_command_generated=false")
    print("ready_for_full_path_following=false")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Stage28A sensor readiness monitor. No movement.")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--path-package", default="latest")
    parser.add_argument("--duration-s", type=float, default=30.0)
    parser.add_argument("--log")
    parser.add_argument("--out-dir", required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    try:
        return run(build_parser().parse_args(argv))
    except RuntimeError as exc:
        print(f"reason={exc}")
        print("motor_command_generated=false")
        print("ready_for_full_path_following=false")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
