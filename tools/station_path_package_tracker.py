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

from tools.path_no_motion_validation import PathPackageResolutionError, resolve_path_package


STATUS_FIELDS = (
    "row_index",
    "mode",
    "station_package_target_source",
    "firmware_active_target_source",
    "firmware_still_compile_time",
    "local_pose_available",
    "local_pose_source",
    "reason",
    "current_x_m",
    "current_y_m",
    "current_heading_deg",
    "current_lat",
    "current_lon",
    "active_primitive_index",
    "active_tool_segment_id",
    "target_x_m",
    "target_y_m",
    "target_distance_m",
    "target_bearing_deg",
    "cross_track_error_m",
    "along_track_progress_m",
    "heading_error_deg",
    "tool_active_expected",
    "coverage_contributes",
    "motor_command_generated",
    "physical_output_active",
)


def _parse_bool(value: object) -> bool:
    return str(value).strip().lower() in {"1", "true", "ok", "yes", "ready"}


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


def _normalize_deg(angle_deg: float) -> float:
    return ((angle_deg + 180.0) % 360.0) - 180.0


def parse_usbdbg_rows(text: str) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    for line in text.splitlines():
        values = {key.lower(): value for key, value in re.findall(r"([A-Za-z0-9_]+)=([^,\s]+)", line)}
        if values:
            rows.append(values)
    return rows


def _latest_firmware_target_source(rows: Sequence[dict[str, str]]) -> str:
    for row in reversed(rows):
        if "active_target_source" in row:
            return row["active_target_source"]
    return "unknown"


def _georeference(package: dict[str, object]) -> dict[str, float] | None:
    containers = [
        package,
        package.get("georeference", {}),
        package.get("normalized_workspace", {}),
        package.get("summary", {}),
    ]
    for container in containers:
        if not isinstance(container, dict):
            continue
        origin_lat = _optional_float(container.get("origin_lat", container.get("origin_lat_deg")))
        origin_lon = _optional_float(container.get("origin_lon", container.get("origin_lon_deg")))
        bearing = _optional_float(container.get("x_axis_bearing_deg"))
        meters_per_lat = _optional_float(container.get("meters_per_lat", container.get("meters_per_deg_lat")))
        meters_per_lon = _optional_float(container.get("meters_per_lon", container.get("meters_per_deg_lon")))
        if None not in (origin_lat, origin_lon, bearing, meters_per_lat, meters_per_lon):
            return {
                "origin_lat": float(origin_lat),
                "origin_lon": float(origin_lon),
                "x_axis_bearing_deg": float(bearing),
                "meters_per_lat": float(meters_per_lat),
                "meters_per_lon": float(meters_per_lon),
            }
    return None


def lat_lon_to_local(package: dict[str, object], lat: float, lon: float) -> tuple[float, float] | None:
    georef = _georeference(package)
    if georef is None:
        return None
    east_m = (lon - georef["origin_lon"]) * georef["meters_per_lon"]
    north_m = (lat - georef["origin_lat"]) * georef["meters_per_lat"]
    theta = math.radians(georef["x_axis_bearing_deg"])
    local_x = east_m * math.sin(theta) + north_m * math.cos(theta)
    local_y = -east_m * math.cos(theta) + north_m * math.sin(theta)
    return local_x, local_y


def local_pose_from_row(package: dict[str, object], row: dict[str, str]) -> tuple[bool, str, str, float, float, float | None]:
    for x_key, y_key in (("current_x_m", "current_y_m"), ("x_m", "y_m")):
        if x_key in row and y_key in row:
            heading = _optional_float(row.get("current_heading_deg", row.get("heading_deg")))
            return True, "OK", "local_xy", float(row[x_key]), float(row[y_key]), heading
    lat = _optional_float(row.get("current_lat", row.get("lat")))
    lon = _optional_float(row.get("current_lon", row.get("lon")))
    if lat is not None and lon is not None:
        local = lat_lon_to_local(package, lat, lon)
        if local is None:
            return False, "NO_GEOREFERENCE_FOR_LAT_LON_TO_LOCAL", "none", 0.0, 0.0, None
        heading = _optional_float(row.get("current_heading_deg", row.get("heading_deg")))
        return True, "OK", "gps_georeference", local[0], local[1], heading
    return False, "NO_LOCAL_POSE_IN_ROW", "none", 0.0, 0.0, None


def _project_to_move(row: dict[str, object], x: float, y: float) -> tuple[float, float, float]:
    sx = float(row["start_x_m"])
    sy = float(row["start_y_m"])
    ex = float(row["end_x_m"])
    ey = float(row["end_y_m"])
    dx = ex - sx
    dy = ey - sy
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-12:
        return 0.0, math.hypot(x - sx, y - sy), 0.0
    raw_t = ((x - sx) * dx + (y - sy) * dy) / length_sq
    t = max(0.0, min(1.0, raw_t))
    proj_x = sx + t * dx
    proj_y = sy + t * dy
    return t, math.hypot(x - proj_x, y - proj_y), math.sqrt(length_sq) * t


def compute_target_status(
    package: dict[str, object],
    *,
    current_x: float,
    current_y: float,
    current_heading_deg: float | None,
    mode: str,
    row_index: int = 0,
    firmware_active_target_source: str = "unknown",
    current_lat: float | None = None,
    current_lon: float | None = None,
    local_pose_source: str = "manual_local",
) -> dict[str, object]:
    primitives = list(package["primitive_sequence"])  # type: ignore[index]
    best_index = 0
    best_score = float("inf")
    best_t = 0.0
    best_cross_track = 0.0
    best_along = 0.0
    for index, primitive in enumerate(primitives):
        ptype = str(primitive["primitive_type"])
        if ptype.startswith("move"):
            t, cross, along = _project_to_move(primitive, current_x, current_y)
            score = cross
        else:
            cross = math.hypot(current_x - float(primitive["start_x_m"]), current_y - float(primitive["start_y_m"]))
            along = 0.0
            t = 0.0
            score = cross + 0.05
        if score < best_score:
            best_index = index
            best_score = score
            best_t = t
            best_cross_track = cross
            best_along = along
    if best_t >= 0.995 and best_index + 1 < len(primitives):
        best_index += 1
        best = primitives[best_index]
        if str(best["primitive_type"]).startswith("move"):
            best_t, best_cross_track, best_along = _project_to_move(best, current_x, current_y)
        else:
            best_t = 0.0
            best_cross_track = math.hypot(current_x - float(best["start_x_m"]), current_y - float(best["start_y_m"]))
            best_along = 0.0
    primitive = primitives[best_index]
    target_x = float(primitive["end_x_m"])
    target_y = float(primitive["end_y_m"])
    dx = target_x - current_x
    dy = target_y - current_y
    target_distance = math.hypot(dx, dy)
    target_bearing = math.degrees(math.atan2(dy, dx)) if target_distance > 1e-9 else (
        float(current_heading_deg) if current_heading_deg is not None else 0.0
    )
    heading_error: str | float
    if current_heading_deg is None:
        heading_error = "NA_DIAG_ONLY"
    else:
        heading_error = _normalize_deg(target_bearing - current_heading_deg)
    return {
        "row_index": row_index,
        "mode": mode,
        "station_package_target_source": "path_package",
        "firmware_active_target_source": firmware_active_target_source,
        "firmware_still_compile_time": firmware_active_target_source == "compile_time",
        "local_pose_available": True,
        "local_pose_source": local_pose_source,
        "reason": "OK",
        "current_x_m": current_x,
        "current_y_m": current_y,
        "current_heading_deg": "" if current_heading_deg is None else current_heading_deg,
        "current_lat": "" if current_lat is None else current_lat,
        "current_lon": "" if current_lon is None else current_lon,
        "active_primitive_index": primitive["primitive_index"],
        "active_tool_segment_id": primitive.get("associated_tool_segment_id", ""),
        "target_x_m": target_x,
        "target_y_m": target_y,
        "target_distance_m": target_distance,
        "target_bearing_deg": target_bearing,
        "cross_track_error_m": best_cross_track,
        "along_track_progress_m": best_along,
        "heading_error_deg": heading_error,
        "tool_active_expected": primitive["tool_active"],
        "coverage_contributes": primitive["coverage_contributes"],
        "motor_command_generated": False,
        "physical_output_active": False,
    }


def diagnostic_status(
    *,
    mode: str,
    row_index: int,
    reason: str,
    firmware_active_target_source: str = "unknown",
    current_lat: float | None = None,
    current_lon: float | None = None,
) -> dict[str, object]:
    return {
        "row_index": row_index,
        "mode": mode,
        "station_package_target_source": "path_package",
        "firmware_active_target_source": firmware_active_target_source,
        "firmware_still_compile_time": firmware_active_target_source == "compile_time",
        "local_pose_available": False,
        "local_pose_source": "none",
        "reason": reason,
        "current_x_m": "",
        "current_y_m": "",
        "current_heading_deg": "",
        "current_lat": "" if current_lat is None else current_lat,
        "current_lon": "" if current_lon is None else current_lon,
        "active_primitive_index": "NA",
        "active_tool_segment_id": "NA",
        "target_x_m": "NA",
        "target_y_m": "NA",
        "target_distance_m": "NA",
        "target_bearing_deg": "NA",
        "cross_track_error_m": "NA",
        "along_track_progress_m": "NA",
        "heading_error_deg": "NA_DIAG_ONLY",
        "tool_active_expected": "NA",
        "coverage_contributes": "NA",
        "motor_command_generated": False,
        "physical_output_active": False,
    }


def _summary_from_rows(package: dict[str, object], selected_path: Path, rows: Sequence[dict[str, object]]) -> dict[str, object]:
    first = rows[0] if rows else diagnostic_status(mode="unknown", row_index=0, reason="NO_ROWS")
    primitive_sequence_valid = bool(package["summary"]["primitive_sequence_valid"])  # type: ignore[index]
    target_distance = _optional_float(first.get("target_distance_m"))
    target_bearing = _optional_float(first.get("target_bearing_deg"))
    ready = (
        primitive_sequence_valid
        and bool(first["local_pose_available"])
        and target_distance is not None
        and target_bearing is not None
        and first["motor_command_generated"] is False
    )
    return {
        "selected_path_package": str(selected_path),
        "path_package_loaded": True,
        "station_package_target_source": "path_package",
        "firmware_active_target_source": first["firmware_active_target_source"],
        "firmware_still_compile_time": first["firmware_still_compile_time"],
        "primitive_sequence_valid": primitive_sequence_valid,
        "local_pose_available": first["local_pose_available"],
        "local_pose_source": first["local_pose_source"],
        "reason": first["reason"],
        "active_primitive_index": first["active_primitive_index"],
        "active_tool_segment_id": first["active_tool_segment_id"],
        "target_distance_m": first["target_distance_m"],
        "target_bearing_deg": first["target_bearing_deg"],
        "cross_track_error_m": first["cross_track_error_m"],
        "tool_active_expected": first["tool_active_expected"],
        "motor_command_generated": False,
        "physical_output_active": False,
        "ready_for_station_side_target_preview": ready,
        "ready_for_motor_test": False,
        "next_action": "Station-side target preview ready; motor tests remain prohibited."
        if ready
        else "Provide local pose or georeferenced A/B package before station-side target preview.",
    }


def _write_csv(path: Path, rows: Sequence[dict[str, object]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=STATUS_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in STATUS_FIELDS})


def _write_summary(path: Path, summary: dict[str, object]) -> None:
    lines = [
        "# Stage 12 Station-Side Path Package Tracker",
        "",
        "Station-side preview/no-motion only. No serial writes, HC-12 frames, or motor commands are generated.",
        "",
    ]
    for key, value in summary.items():
        lines.append(f"- {key}: `{value}`")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _plot_current_target(path: Path, package: dict[str, object], status: dict[str, object]) -> None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle
    except ImportError:
        return
    workspace = package["normalized_workspace"]  # type: ignore[index]
    tool_path = package["tool_path"]  # type: ignore[index]
    x_min = float(workspace["x_min_m"])  # type: ignore[index]
    x_max = float(workspace["x_max_m"])  # type: ignore[index]
    y_min = float(workspace["y_min_m"])  # type: ignore[index]
    y_max = float(workspace["y_max_m"])  # type: ignore[index]
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.add_patch(Rectangle((x_min, y_min), x_max - x_min, y_max - y_min, fill=False, edgecolor="black"))
    for row in tool_path:
        ax.plot(
            [float(row["tool_start_x_m"]), float(row["tool_end_x_m"])],
            [float(row["tool_start_y_m"]), float(row["tool_end_y_m"])],
            color="tab:orange" if row["tool_active"] else "0.55",
            linestyle="-" if row["tool_active"] else "--",
            linewidth=2.0 if row["tool_active"] else 1.0,
        )
    if status["local_pose_available"] is True:
        ax.scatter([float(status["current_x_m"])], [float(status["current_y_m"])], color="tab:blue", label="current")
        ax.scatter([float(status["target_x_m"])], [float(status["target_y_m"])], color="tab:red", label="target")
        ax.plot(
            [float(status["current_x_m"]), float(status["target_x_m"])],
            [float(status["current_y_m"]), float(status["target_y_m"])],
            color="tab:red",
            linestyle=":",
        )
    ax.set_title("Stage 12 Station Package Target Preview")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.axis("equal")
    ax.grid(True, linestyle="--", alpha=0.3)
    handles, labels = ax.get_legend_handles_labels()
    if labels:
        ax.legend(handles, labels, loc="best")
    ax.text(0.01, 0.01, "No motor commands generated", transform=ax.transAxes, fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def _read_live_usbdbg(port: str, duration_s: float) -> list[dict[str, str]]:
    try:
        import serial  # type: ignore[import-not-found]
    except ImportError as exc:
        raise RuntimeError("pyserial is not available; cannot open live USBDBG") from exc
    lines: list[str] = []
    with serial.Serial(port, baudrate=115200, timeout=1.0) as handle:
        deadline = time.monotonic() + max(duration_s, 0.1)
        while time.monotonic() < deadline:
            raw = handle.readline()
            if raw:
                lines.append(raw.decode("utf-8", errors="replace").strip())
    return parse_usbdbg_rows("\n".join(lines))


def build_rows_from_replay(package: dict[str, object], log_text: str, mode: str) -> list[dict[str, object]]:
    rows = parse_usbdbg_rows(log_text)
    firmware_source = _latest_firmware_target_source(rows)
    output: list[dict[str, object]] = []
    for index, row in enumerate(rows):
        local_available, reason, local_pose_source, x_m, y_m, heading = local_pose_from_row(package, row)
        lat = _optional_float(row.get("current_lat", row.get("lat")))
        lon = _optional_float(row.get("current_lon", row.get("lon")))
        firmware_for_row = row.get("active_target_source", firmware_source)
        if not local_available:
            output.append(
                diagnostic_status(
                    mode=mode,
                    row_index=index,
                    reason=reason,
                    firmware_active_target_source=firmware_for_row,
                    current_lat=lat,
                    current_lon=lon,
                )
            )
            continue
        output.append(
            compute_target_status(
                package,
                current_x=x_m,
                current_y=y_m,
                current_heading_deg=heading,
                mode=mode,
                row_index=index,
                firmware_active_target_source=firmware_for_row,
                current_lat=lat,
                current_lon=lon,
                local_pose_source=local_pose_source,
            )
        )
    if output:
        return output
    return [diagnostic_status(mode=mode, row_index=0, reason="NO_USBDBG_ROWS", firmware_active_target_source="unknown")]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Track station-side targets from path_package.json without motor output.")
    parser.add_argument("--path-package", default="latest")
    parser.add_argument("--mode", choices=("offline_pose", "replay_log", "live_usbdbg"), required=True)
    parser.add_argument("--current-x", type=float)
    parser.add_argument("--current-y", type=float)
    parser.add_argument("--current-heading-deg", type=float)
    parser.add_argument("--log")
    parser.add_argument("--port")
    parser.add_argument("--duration-s", type=float, default=60.0)
    parser.add_argument("--out-dir", required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
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
    if args.mode == "offline_pose":
        if args.current_x is None or args.current_y is None:
            raise SystemExit("--current-x and --current-y are required in offline_pose mode")
        rows = [
            compute_target_status(
                package,
                current_x=float(args.current_x),
                current_y=float(args.current_y),
                current_heading_deg=args.current_heading_deg,
                mode=args.mode,
                firmware_active_target_source="not_checked_offline_pose",
                local_pose_source="manual_local",
            )
        ]
    elif args.mode == "replay_log":
        if not args.log:
            raise SystemExit("--log is required in replay_log mode")
        rows = build_rows_from_replay(package, Path(args.log).read_text(encoding="utf-8", errors="replace"), args.mode)
    else:
        if not args.port:
            raise SystemExit("--port is required in live_usbdbg mode")
        rows = build_rows_from_replay(package, "\n".join(" ".join(f"{k}={v}" for k, v in row.items()) for row in _read_live_usbdbg(args.port, args.duration_s)), args.mode)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / "station_target_status.csv"
    summary_path = out_dir / "summary.md"
    preview_path = out_dir / "preview_current_target.png"
    _write_csv(csv_path, rows)
    summary = _summary_from_rows(package, selected, rows)
    _write_summary(summary_path, summary)
    _plot_current_target(preview_path, package, rows[0])
    print("Stage 12 station-side path package tracker complete.")
    print(f"selected_path_package={selected}")
    print(f"station_target_status_csv={csv_path}")
    print(f"summary_md={summary_path}")
    print(f"preview_current_target_png={preview_path if preview_path.exists() else 'not_generated'}")
    for key in (
        "station_package_target_source",
        "firmware_active_target_source",
        "firmware_still_compile_time",
        "local_pose_available",
        "local_pose_source",
        "active_primitive_index",
        "target_distance_m",
        "target_bearing_deg",
        "ready_for_station_side_target_preview",
        "ready_for_motor_test",
        "motor_command_generated",
        "physical_output_active",
    ):
        print(f"{key}={summary[key]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
