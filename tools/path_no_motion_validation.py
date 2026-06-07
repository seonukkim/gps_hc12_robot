from __future__ import annotations

import argparse
import csv
import json
import math
import re
import time
from pathlib import Path
from typing import Sequence

try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401


CONCISE_CSV_FIELDS = (
    "validation_mode",
    "serial_opened",
    "selected_path_package",
    "path_package_loaded",
    "gps_status",
    "gps_sats",
    "gps_hdop",
    "position_source",
    "current_lat",
    "current_lon",
    "current_x_m",
    "current_y_m",
    "imu_status",
    "imu_type",
    "imu_chip_id",
    "imu_data_plausible",
    "rc_status",
    "heading_status",
    "active_target_source",
    "live_path_package_connected",
    "active_primitive_index",
    "target_distance_m",
    "target_bearing_deg",
    "cross_track_error_m",
    "heading_error_deg",
    "motor_command_generated",
    "physical_output_active",
    "ready_for_outdoor_no_motion_validation",
    "reason",
    "next_action",
)

VERBOSE_EXTRA_FIELDS = (
    "current_heading",
    "target_point",
    "along_track_progress_m",
    "tool_active_expected",
)


class PathPackageResolutionError(Exception):
    def __init__(self, message: str, *, provided: str, candidates: Sequence[Path]) -> None:
        super().__init__(message)
        self.provided = provided
        self.candidates = list(candidates)


def _parse_bool(value: object, default: bool = False) -> bool:
    if value is None:
        return default
    return str(value).strip().lower() in {"1", "true", "ok", "yes", "manual", "ready"}


def _parse_optional_float(value: object) -> float | None:
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


def _parse_optional_int(value: object) -> int | None:
    parsed = _parse_optional_float(value)
    return int(parsed) if parsed is not None else None


def _format_optional(value: object) -> object:
    return "" if value is None else value


def _normalize_deg(angle_deg: float) -> float:
    return ((angle_deg + 180.0) % 360.0) - 180.0


def _dedupe_existing(paths: Sequence[Path]) -> list[Path]:
    seen: set[Path] = set()
    result: list[Path] = []
    for path in paths:
        if not path.exists():
            continue
        resolved = path.resolve()
        if resolved not in seen:
            seen.add(resolved)
            result.append(path)
    return result


def discover_path_package_candidates(base_dir: Path | None = None) -> list[Path]:
    base = base_dir or Path.cwd()
    candidates: list[Path] = []
    candidates.append(base / "outputs/field_ab_serpentine/latest/path_package.json")
    candidates.extend(
        sorted(
            (base / "outputs/field_runs").glob("*/field_ab_serpentine/path_package.json"),
            key=lambda path: path.stat().st_mtime if path.exists() else 0.0,
            reverse=True,
        )
    )
    outputs_dir = base / "outputs"
    if outputs_dir.exists():
        candidates.extend(
            sorted(
                outputs_dir.rglob("path_package.json"),
                key=lambda path: path.stat().st_mtime if path.exists() else 0.0,
                reverse=True,
            )
        )
    return _dedupe_existing(candidates)


def resolve_path_package(path_arg: str, base_dir: Path | None = None) -> Path:
    base = base_dir or Path.cwd()
    candidates = discover_path_package_candidates(base)
    if path_arg == "latest":
        if candidates:
            return candidates[0]
        raise PathPackageResolutionError(
            "No path_package.json found. Run tools/field_ab_to_serpentine.py first.",
            provided=path_arg,
            candidates=[],
        )
    provided = Path(path_arg).expanduser()
    if not provided.is_absolute():
        provided = base / provided
    if provided.exists():
        return provided
    raise PathPackageResolutionError(
        "Provided path_package.json does not exist.",
        provided=path_arg,
        candidates=candidates[:5],
    )


def parse_status_log(text: str) -> dict[str, object]:
    status: dict[str, object] = {
        "gps_ok": False,
        "bmi160_ok": False,
        "rc_manual_ok": False,
        "current_pose_known": False,
        "position_source": "unknown",
        "gps_sats": None,
        "gps_hdop": None,
        "current_lat": None,
        "current_lon": None,
        "x_m": 0.0,
        "y_m": 0.0,
        "heading_deg": None,
        "heading_ready": False,
        "gps_course_deg": None,
        "course_displacement_m": None,
        "gps_course_min_displacement_m": None,
        "imu_heading_diag": None,
        "imu_type": "",
        "imu_chip_id": "",
        "imu_data_plausible": False,
        "physical_output_active": False,
        "active_target_source": "unknown",
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

    status["gps_ok"] = _parse_bool(key_values.get("gps_ok", key_values.get("gps_fix")))
    status["bmi160_ok"] = _parse_bool(key_values.get("bmi160_ok", key_values.get("imu_ok")))
    status["rc_manual_ok"] = _parse_bool(key_values.get("rc_manual_ok", key_values.get("rc_ok")))
    status["imu_data_plausible"] = _parse_bool(key_values.get("imu_data_plausible", key_values.get("imu_ok")))
    status["imu_type"] = key_values.get("imu_type", "")
    status["imu_chip_id"] = key_values.get("imu_chip_id", "")
    if "position_source" in key_values:
        status["position_source"] = key_values["position_source"].lower()
    elif bool(status["gps_ok"]):
        status["position_source"] = "gps"

    status["gps_sats"] = _parse_optional_int(key_values.get("gps_sats", key_values.get("sats")))
    status["gps_hdop"] = _parse_optional_float(key_values.get("gps_hdop", key_values.get("hdop")))
    status["current_lat"] = _parse_optional_float(key_values.get("current_lat", key_values.get("lat")))
    status["current_lon"] = _parse_optional_float(key_values.get("current_lon", key_values.get("lon")))

    for x_key, y_key in (("x_m", "y_m"), ("current_x_m", "current_y_m")):
        if x_key in key_values and y_key in key_values:
            status["x_m"] = float(key_values[x_key])
            status["y_m"] = float(key_values[y_key])
            status["current_pose_known"] = True
            break
    if status["current_lat"] is not None and status["current_lon"] is not None:
        status["current_pose_known"] = True

    if "heading_deg" in key_values:
        status["heading_deg"] = _parse_optional_float(key_values["heading_deg"])
    elif "current_heading_deg" in key_values:
        status["heading_deg"] = _parse_optional_float(key_values["current_heading_deg"])
    status["heading_ready"] = _parse_bool(
        key_values.get("heading_ready", key_values.get("gps_course_output_valid"))
    )
    status["gps_course_deg"] = _parse_optional_float(
        key_values.get("gps_course_deg", key_values.get("gps_course_output_deg"))
    )
    status["course_displacement_m"] = _parse_optional_float(key_values.get("course_displacement_m"))
    status["gps_course_min_displacement_m"] = _parse_optional_float(key_values.get("gps_course_min_displacement_m"))
    status["imu_heading_diag"] = _parse_optional_float(
        key_values.get("imu_heading_diag", key_values.get("imu_relative_yaw_deg", key_values.get("yaw_deg")))
    )
    status["physical_output_active"] = _parse_bool(key_values.get("physical_output_active"))
    status["active_target_source"] = key_values.get("active_target_source", "unknown")
    return status


def package_preview_status(package: dict[str, object]) -> dict[str, object]:
    primitives = package["primitive_sequence"]  # type: ignore[index]
    first = primitives[0] if primitives else {"start_x_m": 0.0, "start_y_m": 0.0, "start_heading_deg": 0.0}
    return {
        "gps_ok": False,
        "bmi160_ok": False,
        "rc_manual_ok": False,
        "current_pose_known": True,
        "position_source": "package_preview",
        "gps_sats": None,
        "gps_hdop": None,
        "current_lat": None,
        "current_lon": None,
        "x_m": float(first["start_x_m"]),
        "y_m": float(first["start_y_m"]),
        "heading_deg": float(first["start_heading_deg"]),
        "heading_ready": False,
        "gps_course_deg": None,
        "course_displacement_m": None,
        "gps_course_min_displacement_m": None,
        "imu_heading_diag": float(first["start_heading_deg"]),
        "imu_type": "",
        "imu_chip_id": "",
        "imu_data_plausible": False,
        "physical_output_active": False,
        "active_target_source": "package_preview",
    }


def read_live_serial_status(port: str, duration_s: float, baud: int = 115200) -> dict[str, object]:
    try:
        import serial  # type: ignore[import-not-found]
    except ImportError as exc:
        raise RuntimeError("pyserial is not available; cannot open live serial") from exc
    try:
        with serial.Serial(port, baudrate=baud, timeout=1.0) as handle:
            lines: list[str] = []
            deadline = time.monotonic() + max(duration_s, 0.1)
            while time.monotonic() < deadline:
                raw = handle.readline()
                if raw:
                    lines.append(raw.decode("utf-8", errors="replace").strip())
    except Exception as exc:  # pragma: no cover - exact exception varies by platform.
        raise RuntimeError(f"could not open/read serial port {port}: {exc}") from exc
    return parse_status_log("\n".join(lines))


def _gps_status(
    status: dict[str, object],
    *,
    validation_mode: str,
    min_sats: int,
    max_hdop: float,
    no_motion_gps_mode: str,
) -> str:
    if validation_mode == "package_check" and status.get("position_source") == "package_preview":
        return "NOT_CHECKED_PACKAGE_ONLY"
    if no_motion_gps_mode == "require_course":
        if not bool(status["gps_ok"]) or status.get("gps_course_deg") is None:
            return "FAIL"
        return "OK"
    if status.get("position_source") != "gps" or not bool(status["current_pose_known"]):
        return "FAIL"
    sats = status.get("gps_sats")
    hdop = status.get("gps_hdop")
    if not isinstance(sats, int) or not isinstance(hdop, float):
        return "WAIT"
    if sats >= min_sats and hdop <= max_hdop:
        return "OK"
    return "WAIT"


def _status_label(ok: object) -> str:
    return "OK" if bool(ok) else "WAIT"


def _heading_status(status: dict[str, object], no_motion_gps_mode: str) -> str:
    if no_motion_gps_mode == "require_course":
        return "OK" if bool(status.get("heading_ready")) else "FAIL"
    if status.get("gps_course_deg") is None:
        displacement = status.get("course_displacement_m")
        minimum = status.get("gps_course_min_displacement_m")
        if isinstance(displacement, float) and isinstance(minimum, float) and displacement < minimum:
            return "WAITING_FOR_MOTION_OR_DIAG_ONLY"
        return "WAITING_FOR_MOTION_OR_DIAG_ONLY"
    return "DIAG_ONLY"


def build_target_rows(
    package: dict[str, object],
    status: dict[str, object],
    ready: dict[str, object],
    *,
    concise: bool,
) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    current_x = float(status["x_m"])
    current_y = float(status["y_m"])
    current_heading = status.get("heading_deg")
    heading_available = isinstance(current_heading, float)
    progress = 0.0
    for primitive in package["primitive_sequence"]:  # type: ignore[index]
        target_x = float(primitive["end_x_m"])
        target_y = float(primitive["end_y_m"])
        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.hypot(dx, dy)
        bearing = math.degrees(math.atan2(dy, dx)) if distance > 1e-9 else (
            float(current_heading) if heading_available else 0.0
        )
        progress += distance
        heading_error = (
            f"{_normalize_deg(bearing - float(current_heading)):.3f}" if heading_available else "NA_DIAG_ONLY"
        )
        row = {
            "validation_mode": ready["validation_mode"],
            "serial_opened": ready["serial_opened"],
            "selected_path_package": ready["selected_path_package"],
            "path_package_loaded": ready["path_package_loaded"],
            "gps_status": ready["gps_status"],
            "gps_sats": _format_optional(status.get("gps_sats")),
            "gps_hdop": _format_optional(status.get("gps_hdop")),
            "position_source": status.get("position_source", "unknown"),
            "current_lat": _format_optional(status.get("current_lat")),
            "current_lon": _format_optional(status.get("current_lon")),
            "current_x_m": f"{current_x:.3f}",
            "current_y_m": f"{current_y:.3f}",
            "imu_status": ready["imu_status"],
            "imu_type": _format_optional(status.get("imu_type")),
            "imu_chip_id": _format_optional(status.get("imu_chip_id")),
            "imu_data_plausible": ready["imu_data_plausible"],
            "rc_status": ready["rc_status"],
            "heading_status": ready["heading_status"],
            "active_target_source": ready["active_target_source"],
            "live_path_package_connected": ready["live_path_package_connected"],
            "active_primitive_index": primitive["primitive_index"],
            "target_distance_m": f"{distance:.3f}",
            "target_bearing_deg": f"{bearing:.3f}",
            "cross_track_error_m": "0.000",
            "heading_error_deg": heading_error,
            "motor_command_generated": False,
            "physical_output_active": ready["physical_output_active"],
            "ready_for_outdoor_no_motion_validation": ready["ready_for_outdoor_no_motion_validation"],
            "reason": ready["reason"],
            "next_action": ready["next_action"],
        }
        if not concise:
            row |= {
                "current_heading": "" if not heading_available else f"{float(current_heading):.3f}",
                "target_point": f"{target_x:.3f},{target_y:.3f}",
                "along_track_progress_m": f"{progress:.3f}",
                "tool_active_expected": primitive["tool_active"],
            }
        rows.append(row)
        current_x = target_x
        current_y = target_y
        current_heading = primitive["end_heading_deg"] if heading_available else None
    return rows


def _first_target_values(rows: Sequence[dict[str, object]]) -> tuple[float | None, float | None]:
    if not rows:
        return None, None
    return (
        _parse_optional_float(rows[0].get("target_distance_m")),
        _parse_optional_float(rows[0].get("target_bearing_deg")),
    )


def build_readiness(
    package: dict[str, object],
    status: dict[str, object],
    rows: Sequence[dict[str, object]],
    *,
    validation_mode: str,
    serial_opened: bool,
    selected_path_package: Path,
    min_sats: int,
    max_hdop: float,
    no_motion_gps_mode: str,
) -> dict[str, object]:
    summary = package["summary"]  # type: ignore[index]
    primitive_sequence_valid = bool(summary["primitive_sequence_valid"])
    motor_command_generated = bool(summary["motor_command_generated"])
    physical_output_active = bool(status["physical_output_active"])
    target_distance, target_bearing = _first_target_values(rows)
    gps_status = _gps_status(
        status,
        validation_mode=validation_mode,
        min_sats=min_sats,
        max_hdop=max_hdop,
        no_motion_gps_mode=no_motion_gps_mode,
    )
    ready = (
        gps_status == "OK"
        and target_distance is not None
        and target_bearing is not None
        and primitive_sequence_valid
        and not motor_command_generated
        and not physical_output_active
        and (validation_mode != "live_serial" or serial_opened)
    )
    active_target_source = str(status.get("active_target_source", "unknown"))
    current_target_source_is_compile_time = active_target_source == "compile_time"
    live_path_package_connected: bool | str = "not_checked_package_only"
    reason = "OK"
    if validation_mode == "live_serial":
        live_path_package_connected = active_target_source not in {"compile_time", "unknown", ""}
        if current_target_source_is_compile_time:
            reason = "ACTIVE_TARGET_SOURCE_COMPILE_TIME"
            ready = False
        elif not live_path_package_connected:
            reason = "ACTIVE_TARGET_SOURCE_UNKNOWN"
            ready = False
    next_action = "Proceed with no-motion target preview."
    if validation_mode == "package_check" and status.get("position_source") == "package_preview":
        next_action = "Package-only check complete; run live_serial with --duration-s for outdoor GPS target validation."
    elif gps_status == "WAIT":
        next_action = f"Wait for GPS quality: sats >= {min_sats}, hdop <= {max_hdop:g}."
    elif gps_status == "FAIL":
        next_action = "Acquire GPS position before target validation."
    elif physical_output_active or motor_command_generated:
        next_action = "Stop: motor or physical output is active."
    elif target_distance is None or target_bearing is None:
        next_action = "Regenerate the path package; target distance/bearing are NA."
    elif reason == "ACTIVE_TARGET_SOURCE_COMPILE_TIME":
        next_action = "Do not proceed to motor tests. Validate path package offline and implement package-to-firmware/station target bridge."
    elif reason == "ACTIVE_TARGET_SOURCE_UNKNOWN":
        next_action = "Do not proceed to motor tests. Live target source is not a generated path package."

    return {
        "validation_mode": validation_mode,
        "serial_opened": serial_opened,
        "selected_path_package": str(selected_path_package),
        "path_package_loaded": True,
        "gps_status": gps_status,
        "gps_sats": status.get("gps_sats"),
        "gps_hdop": status.get("gps_hdop"),
        "position_source": status.get("position_source", "unknown"),
        "current_lat": status.get("current_lat"),
        "current_lon": status.get("current_lon"),
        "imu_status": _status_label(status["bmi160_ok"]),
        "imu_type": status.get("imu_type", ""),
        "imu_chip_id": status.get("imu_chip_id", ""),
        "imu_data_plausible": bool(status.get("imu_data_plausible")),
        "rc_status": _status_label(status["rc_manual_ok"]),
        "heading_status": _heading_status(status, no_motion_gps_mode),
        "active_target_source": active_target_source,
        "live_path_package_connected": live_path_package_connected,
        "current_target_source_is_compile_time": current_target_source_is_compile_time,
        "active_primitive_index": rows[0]["active_primitive_index"] if rows else "NA",
        "target_distance_m": "NA" if target_distance is None else f"{target_distance:.3f}",
        "target_bearing_deg": "NA" if target_bearing is None else f"{target_bearing:.3f}",
        "cross_track_error_m": rows[0]["cross_track_error_m"] if rows else "NA",
        "heading_error_deg": rows[0]["heading_error_deg"] if rows else "NA_DIAG_ONLY",
        "motor_command_generated": motor_command_generated,
        "physical_output_active": physical_output_active,
        "ready_for_outdoor_no_motion_validation": ready,
        "reason": reason,
        "next_action": next_action,
    }


def _write_csv(path: Path, rows: Sequence[dict[str, object]], fields: Sequence[str]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in fields})


def _write_summary(path: Path, ready: dict[str, object], csv_path: Path, *, concise: bool) -> None:
    lines = [
        "# Outdoor Path No-Motion Validation",
        "",
        "No-motion preview only. This tool does not generate motor commands.",
        "",
    ]
    if concise:
        keys = [
            "validation_mode",
            "serial_opened",
            "selected_path_package",
            "path_package_loaded",
            "gps_status",
            "gps_sats",
            "gps_hdop",
            "position_source",
            "current_lat",
            "current_lon",
            "imu_status",
            "imu_type",
            "imu_chip_id",
            "imu_data_plausible",
            "rc_status",
            "heading_status",
            "active_target_source",
            "live_path_package_connected",
            "active_primitive_index",
            "target_distance_m",
            "target_bearing_deg",
            "cross_track_error_m",
            "heading_error_deg",
            "motor_command_generated",
            "physical_output_active",
            "ready_for_outdoor_no_motion_validation",
            "reason",
            "next_action",
        ]
    else:
        keys = list(ready.keys())
    for key in keys:
        lines.append(f"- {key}: `{ready.get(key, '')}`")
    lines.append(f"- validation_csv: `{csv_path}`")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_failure_summary(
    out_dir: Path,
    *,
    validation_mode: str,
    selected_path_package: Path | None,
    serial_opened: bool,
    reason: str,
) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    lines = [
        "# Outdoor Path No-Motion Validation",
        "",
        "- validation_mode: `" + validation_mode + "`",
        f"- serial_opened: `{serial_opened}`",
        f"- selected_path_package: `{'' if selected_path_package is None else selected_path_package}`",
        "- path_package_loaded: `" + str(selected_path_package is not None) + "`",
        "- motor_command_generated: `False`",
        "- physical_output_active: `False`",
        "- ready_for_outdoor_no_motion_validation: `False`",
        f"- next_action: `{reason}`",
    ]
    (out_dir / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def _effective_mode(mode: str, port: str | None, duration_s: float) -> str:
    if mode != "auto":
        return mode
    if port and duration_s > 0:
        return "live_serial"
    return "package_check"


def _print_package_error(error: PathPackageResolutionError) -> None:
    print(f"provided_path_package={error.provided}")
    print("file_exists=false")
    print(f"error={error}")
    print("nearest_candidates:")
    if error.candidates:
        for candidate in error.candidates:
            print(f"- {candidate}")
    else:
        print("- none")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Validate a field serpentine path package in no-motion preview mode."
    )
    parser.add_argument("--path-package", required=True, help="Path to path_package.json, or 'latest'.")
    parser.add_argument("--sample-log", help="Optional text/CSV status log. No serial port is opened.")
    parser.add_argument("--port", default=None, help="OpenRB port for live_serial mode.")
    parser.add_argument("--mode", choices=("package_check", "live_serial", "auto"), default="auto")
    parser.add_argument("--duration-s", type=float, default=0.0)
    parser.add_argument("--out-dir", default="outputs/path_no_motion_validation/latest")
    parser.add_argument("--concise", choices=("true", "false"), default="true")
    parser.add_argument("--no-motion-gps-mode", choices=("position_only", "require_course", "course_required"), default="position_only")
    parser.add_argument("--min-sats", type=int, default=4)
    parser.add_argument("--max-hdop", type=float, default=3.0)
    parser.add_argument("--verbose", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    out_dir = Path(args.out_dir)
    try:
        selected_path_package = resolve_path_package(args.path_package)
    except PathPackageResolutionError as error:
        _print_package_error(error)
        _write_failure_summary(
            out_dir,
            validation_mode=args.mode,
            selected_path_package=None,
            serial_opened=False,
            reason="No path_package.json found. Run tools/field_ab_to_serpentine.py first.",
        )
        return 2

    no_motion_gps_mode = "require_course" if args.no_motion_gps_mode == "course_required" else args.no_motion_gps_mode
    validation_mode = _effective_mode(args.mode, args.port, args.duration_s)
    package = json.loads(selected_path_package.read_text(encoding="utf-8"))
    serial_opened = False
    try:
        if validation_mode == "live_serial":
            if not args.port:
                raise RuntimeError("--port is required in live_serial mode")
            status = read_live_serial_status(args.port, args.duration_s if args.duration_s > 0 else 120.0)
            serial_opened = True
        elif args.sample_log:
            status = parse_status_log(Path(args.sample_log).read_text(encoding="utf-8"))
        else:
            status = package_preview_status(package)
    except RuntimeError as exc:
        print(f"validation_mode={validation_mode}")
        print("serial_opened=false")
        print(f"selected_path_package={selected_path_package}")
        print(f"error={exc}")
        _write_failure_summary(
            out_dir,
            validation_mode=validation_mode,
            selected_path_package=selected_path_package,
            serial_opened=False,
            reason=str(exc),
        )
        return 2

    preview_ready = {
        "validation_mode": validation_mode,
        "serial_opened": serial_opened,
        "selected_path_package": str(selected_path_package),
        "path_package_loaded": True,
        "gps_status": "",
        "imu_status": "",
        "imu_data_plausible": False,
        "rc_status": "",
        "heading_status": "",
        "active_target_source": "",
        "live_path_package_connected": "not_checked_package_only",
        "active_primitive_index": "",
        "target_distance_m": "",
        "target_bearing_deg": "",
        "cross_track_error_m": "",
        "heading_error_deg": "",
        "motor_command_generated": bool(package["summary"]["motor_command_generated"]),  # type: ignore[index]
        "physical_output_active": bool(status["physical_output_active"]),
        "ready_for_outdoor_no_motion_validation": False,
        "reason": "",
        "next_action": "",
    }
    initial_rows = build_target_rows(package, status, preview_ready, concise=True)
    ready = build_readiness(
        package,
        status,
        initial_rows,
        validation_mode=validation_mode,
        serial_opened=serial_opened,
        selected_path_package=selected_path_package,
        min_sats=args.min_sats,
        max_hdop=args.max_hdop,
        no_motion_gps_mode=no_motion_gps_mode,
    )
    concise = args.concise == "true" and not args.verbose
    rows = build_target_rows(package, status, ready, concise=concise)
    fields = CONCISE_CSV_FIELDS if concise else CONCISE_CSV_FIELDS + VERBOSE_EXTRA_FIELDS
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / "no_motion_validation.csv"
    summary_path = out_dir / "summary.md"
    _write_csv(csv_path, rows, fields)
    _write_summary(summary_path, ready, csv_path, concise=concise)

    print("Outdoor path no-motion validation generated.")
    print(f"validation_mode={validation_mode}")
    print(f"serial_opened={str(serial_opened).lower()}")
    if validation_mode == "package_check" and args.port:
        print(f"port_reported_not_opened={args.port}")
        print("reason=package_check_does_not_use_serial")
    print(f"selected_path_package={selected_path_package}")
    print("motor_command_generated=false")
    print(f"physical_output_active={str(bool(status['physical_output_active'])).lower()}")
    print(f"gps_status={ready['gps_status']}")
    print(f"heading_status={ready['heading_status']}")
    print(f"ready_for_outdoor_no_motion_validation={ready['ready_for_outdoor_no_motion_validation']}")
    print(f"next_action={ready['next_action']}")
    print(f"validation_csv={csv_path}")
    print(f"summary_md={summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
