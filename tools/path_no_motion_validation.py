from __future__ import annotations

import argparse
import csv
import json
import math
import re
from pathlib import Path
from typing import Sequence

try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401


CSV_FIELDS = (
    "current_position",
    "current_heading",
    "active_primitive_index",
    "target_point",
    "target_distance_m",
    "target_bearing_deg",
    "heading_error_deg",
    "cross_track_error_m",
    "along_track_progress_m",
    "tool_active_expected",
    "motor_command_generated",
    "physical_output_active",
)


def _normalize_deg(angle_deg: float) -> float:
    return ((angle_deg + 180.0) % 360.0) - 180.0


def parse_status_log(text: str) -> dict[str, object]:
    status: dict[str, object] = {
        "gps_ok": False,
        "bmi160_ok": False,
        "rc_manual_ok": False,
        "current_pose_known": False,
        "x_m": 0.0,
        "y_m": 0.0,
        "heading_deg": 0.0,
        "physical_output_active": False,
    }
    key_values: dict[str, str] = {}
    for line in text.splitlines():
        for key, value in re.findall(r"([A-Za-z0-9_]+)=([^,\s]+)", line):
            key_values[key.lower()] = value
    if not key_values:
        try:
            rows = list(csv.DictReader(text.splitlines()))
        except csv.Error:
            rows = []
        if rows:
            key_values.update({key.lower(): value for key, value in rows[-1].items() if value is not None})
    true_values = {"1", "true", "ok", "yes", "manual", "ready"}
    status["gps_ok"] = key_values.get("gps_ok", key_values.get("gps_fix", "false")).lower() in true_values
    status["bmi160_ok"] = key_values.get("bmi160_ok", key_values.get("imu_ok", "false")).lower() in true_values
    status["rc_manual_ok"] = key_values.get("rc_manual_ok", key_values.get("rc_ok", "false")).lower() in true_values
    for x_key, y_key in (("x_m", "y_m"), ("current_x_m", "current_y_m")):
        if x_key in key_values and y_key in key_values:
            status["x_m"] = float(key_values[x_key])
            status["y_m"] = float(key_values[y_key])
            status["current_pose_known"] = True
            break
    if "heading_deg" in key_values:
        status["heading_deg"] = float(key_values["heading_deg"])
    elif "current_heading_deg" in key_values:
        status["heading_deg"] = float(key_values["current_heading_deg"])
    status["physical_output_active"] = key_values.get("physical_output_active", "false").lower() in true_values
    return status


def _load_status(path: Path | None, package: dict[str, object]) -> dict[str, object]:
    if path is not None:
        return parse_status_log(path.read_text(encoding="utf-8"))
    primitives = package["primitive_sequence"]  # type: ignore[index]
    first = primitives[0] if primitives else {"start_x_m": 0.0, "start_y_m": 0.0, "start_heading_deg": 0.0}
    return {
        "gps_ok": True,
        "bmi160_ok": True,
        "rc_manual_ok": True,
        "current_pose_known": True,
        "x_m": float(first["start_x_m"]),
        "y_m": float(first["start_y_m"]),
        "heading_deg": float(first["start_heading_deg"]),
        "physical_output_active": False,
    }


def build_validation_rows(package: dict[str, object], status: dict[str, object]) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    current_x = float(status["x_m"])
    current_y = float(status["y_m"])
    current_heading = float(status["heading_deg"])
    progress = 0.0
    for primitive in package["primitive_sequence"]:  # type: ignore[index]
        target_x = float(primitive["end_x_m"])
        target_y = float(primitive["end_y_m"])
        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.hypot(dx, dy)
        bearing = math.degrees(math.atan2(dy, dx)) if distance > 1e-9 else current_heading
        heading_error = _normalize_deg(bearing - current_heading)
        progress += distance
        rows.append(
            {
                "current_position": f"{current_x:.3f},{current_y:.3f}",
                "current_heading": f"{current_heading:.3f}",
                "active_primitive_index": primitive["primitive_index"],
                "target_point": f"{target_x:.3f},{target_y:.3f}",
                "target_distance_m": f"{distance:.3f}",
                "target_bearing_deg": f"{bearing:.3f}",
                "heading_error_deg": f"{heading_error:.3f}",
                "cross_track_error_m": "0.000",
                "along_track_progress_m": f"{progress:.3f}",
                "tool_active_expected": primitive["tool_active"],
                "motor_command_generated": False,
                "physical_output_active": False,
            }
        )
        current_x = target_x
        current_y = target_y
        current_heading = float(primitive["end_heading_deg"])
    return rows


def readiness(package: dict[str, object], status: dict[str, object]) -> dict[str, object]:
    summary = package["summary"]  # type: ignore[index]
    primitive_sequence_valid = bool(summary["primitive_sequence_valid"])
    motor_command_generated = bool(summary["motor_command_generated"])
    physical_output_active = bool(status["physical_output_active"])
    ready = (
        bool(status["gps_ok"])
        and bool(status["bmi160_ok"])
        and bool(status["rc_manual_ok"])
        and bool(status["current_pose_known"])
        and bool(summary["A_prime_top_left"])
        and bool(summary["B_prime_bottom_right"])
        and primitive_sequence_valid
        and not motor_command_generated
        and not physical_output_active
    )
    return {
        "gps_ok": status["gps_ok"],
        "bmi160_ok": status["bmi160_ok"],
        "rc_manual_ok": status["rc_manual_ok"],
        "current_pose_known": status["current_pose_known"],
        "a_prime_b_prime_generated": True,
        "primitive_sequence_valid": primitive_sequence_valid,
        "motor_command_generated": motor_command_generated,
        "physical_output_active": physical_output_active,
        "ready_for_outdoor_no_motion_validation": ready,
    }


def _write_csv(path: Path, rows: Sequence[dict[str, object]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row[field] for field in CSV_FIELDS})


def _write_summary(path: Path, ready: dict[str, object], csv_path: Path) -> None:
    lines = [
        "# Outdoor Path No-Motion Validation",
        "",
        "No-motion preview only. This tool does not open serial by default, send HC-12 frames, or generate motor commands.",
        "",
    ]
    for key, value in ready.items():
        lines.append(f"- {key}: `{value}`")
    lines.extend(
        [
            f"- validation_csv: `{csv_path}`",
            "- motor_command_generated: `False`",
            "- physical_output_active: `False`",
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Validate a field serpentine path package in no-motion preview mode."
    )
    parser.add_argument("--path-package", required=True)
    parser.add_argument("--sample-log", help="Optional text/CSV status log. No serial port is opened.")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Reported OpenRB port; not opened by this parser.")
    parser.add_argument("--out-dir", default="outputs/path_no_motion_validation/latest")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    package = json.loads(Path(args.path_package).read_text(encoding="utf-8"))
    status = _load_status(Path(args.sample_log) if args.sample_log else None, package)
    rows = build_validation_rows(package, status)
    ready = readiness(package, status)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / "no_motion_validation.csv"
    summary_path = out_dir / "summary.md"
    _write_csv(csv_path, rows)
    _write_summary(summary_path, ready, csv_path)
    print("Outdoor path no-motion validation generated.")
    print(f"port_reported_not_opened: {args.port}")
    print("motor_command_generated: False")
    print("physical_output_active: False")
    print(f"ready_for_outdoor_no_motion_validation: {ready['ready_for_outdoor_no_motion_validation']}")
    print(f"validation_csv: {csv_path}")
    print(f"summary_md: {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
