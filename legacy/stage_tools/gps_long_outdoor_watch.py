from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import time
from pathlib import Path
from typing import Sequence

try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401

from tools import station_path_package_tracker
from tools.path_no_motion_validation import PathPackageResolutionError, resolve_path_package


GPS_FIELDS = (
    "row_index",
    "elapsed_s",
    "gps_block_reason",
    "position_source",
    "gps_sats",
    "gps_hdop",
    "current_lat",
    "current_lon",
    "local_x_m",
    "local_y_m",
)


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


def _median(values: Sequence[float]) -> float | None:
    return statistics.median(values) if values else None


def parse_rows(text: str) -> list[dict[str, str]]:
    return station_path_package_tracker.parse_usbdbg_rows(text)


def gps_row_has_update(row: dict[str, str]) -> bool:
    return any(key in row for key in ("gps_block_reason", "gps_sats", "gps_hdop", "current_lat", "current_lon", "position_source"))


def summarize_rows(
    rows: Sequence[dict[str, str]],
    *,
    package: dict[str, object] | None = None,
    requested_duration_s: float = 1200.0,
    elapsed_s: float | None = None,
    min_sats: int = 4,
    max_hdop: float = 3.0,
) -> tuple[dict[str, object], list[dict[str, object]]]:
    elapsed = requested_duration_s if elapsed_s is None else elapsed_s
    gps_rows = [row for row in rows if gps_row_has_update(row)]
    table_rows: list[dict[str, object]] = []
    fix_rows: list[dict[str, str]] = []
    quality_rows: list[dict[str, str]] = []
    local_pose_count = 0
    first_fix_index: int | None = None
    sats_values: list[float] = []
    hdop_values: list[float] = []
    lat_values: list[float] = []
    lon_values: list[float] = []

    for index, row in enumerate(rows):
        sats = _parse_float(row.get("gps_sats"))
        hdop = _parse_float(row.get("gps_hdop"))
        lat = _parse_float(row.get("current_lat"))
        lon = _parse_float(row.get("current_lon"))
        local_x: float | str = ""
        local_y: float | str = ""
        if sats is not None:
            sats_values.append(sats)
        if hdop is not None:
            hdop_values.append(hdop)
        if lat is not None and lon is not None:
            lat_values.append(lat)
            lon_values.append(lon)
            if row.get("gps_block_reason") == "OK" or row.get("position_source") == "gps":
                fix_rows.append(row)
                if first_fix_index is None:
                    first_fix_index = index
            if package is not None:
                local = station_path_package_tracker.lat_lon_to_local(package, lat, lon)
                if local is not None:
                    local_pose_count += 1
                    local_x, local_y = local
        if lat is not None and lon is not None and sats is not None and hdop is not None and sats >= min_sats and hdop <= max_hdop:
            quality_rows.append(row)
        table_rows.append({
            "row_index": index,
            "elapsed_s": "",
            "gps_block_reason": row.get("gps_block_reason", "NA"),
            "position_source": row.get("position_source", "NA"),
            "gps_sats": "" if sats is None else sats,
            "gps_hdop": "" if hdop is None else hdop,
            "current_lat": "" if lat is None else lat,
            "current_lon": "" if lon is None else lon,
            "local_x_m": local_x,
            "local_y_m": local_y,
        })

    fix_present = bool(fix_rows)
    local_pose_available = local_pose_count > 0
    if fix_present and quality_rows and local_pose_available:
        status = "GPS_READY_FULL"
    elif fix_present and quality_rows:
        status = "GPS_FIX_NO_LOCAL_POSE"
    elif fix_present:
        status = "GPS_BLOCKED_BAD_QUALITY"
    elif not gps_rows:
        status = "GPS_BLOCKED_NO_DATA" if elapsed >= requested_duration_s else "GPS_WAITING"
    else:
        status = "GPS_WAITING"

    time_to_first_fix_s = None
    if first_fix_index is not None and rows:
        time_to_first_fix_s = elapsed * (first_fix_index / max(1, len(rows) - 1))

    summary = {
        "requested_duration_s": requested_duration_s,
        "elapsed_s": elapsed,
        "gps_status": status,
        "gps_update_rows": len(gps_rows),
        "gps_fix_rows": len(fix_rows),
        "gps_quality_rows": len(quality_rows),
        "local_pose_available": local_pose_available,
        "local_pose_count": local_pose_count,
        "time_to_first_fix_s": time_to_first_fix_s,
        "gps_sats_min": _range(sats_values)["min"],
        "gps_sats_median": _median(sats_values),
        "gps_sats_max": _range(sats_values)["max"],
        "gps_hdop_min": _range(hdop_values)["min"],
        "gps_hdop_median": _median(hdop_values),
        "gps_hdop_max": _range(hdop_values)["max"],
        "lat_range": _range(lat_values),
        "lon_range": _range(lon_values),
        "coordinate_stability_lat_span": (max(lat_values) - min(lat_values)) if lat_values else None,
        "coordinate_stability_lon_span": (max(lon_values) - min(lon_values)) if lon_values else None,
        "motor_command_generated": False,
        "ready_for_full_path_following": False,
    }
    return summary, table_rows


def write_outputs(out_dir: Path, summary: dict[str, object], table_rows: Sequence[dict[str, object]], raw_lines: Sequence[str]) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "raw_usbdbg.log").write_text("\n".join(raw_lines) + ("\n" if raw_lines else ""), encoding="utf-8")
    with (out_dir / "gps_long_outdoor_watch.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=GPS_FIELDS)
        writer.writeheader()
        for row in table_rows:
            writer.writerow({field: row.get(field, "") for field in GPS_FIELDS})
    (out_dir / "gps_long_outdoor_watch_summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    lines = ["# GPS Long Outdoor Watch", "", "No motion and no motor commands.", ""]
    lines.extend(f"- {key}: `{value}`" for key, value in summary.items())
    (out_dir / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def run(args: argparse.Namespace) -> int:
    if args.duration_s < 600 and not args.allow_short_duration:
        raise SystemExit("--duration-s must be >= 600 unless --allow-short-duration true")
    package = None
    if args.path_package:
        try:
            selected = resolve_path_package(args.path_package)
            package = json.loads(selected.read_text(encoding="utf-8"))
        except PathPackageResolutionError:
            package = None
    raw_lines: list[str] = []
    started = time.monotonic()
    if args.log:
        raw_lines = Path(args.log).read_text(encoding="utf-8").splitlines()
        elapsed = args.duration_s
    else:
        try:
            import serial  # type: ignore[import-not-found]
        except ImportError as exc:
            raise RuntimeError("pyserial is not available; cannot run GPS long watch") from exc
        deadline = started + args.duration_s
        with serial.Serial(args.port, baudrate=115200, timeout=0.5) as handle:
            while time.monotonic() < deadline:
                raw = handle.readline()
                if raw:
                    line = raw.decode("utf-8", errors="replace").strip()
                    print(line)
                    raw_lines.append(line)
        elapsed = time.monotonic() - started
    rows = parse_rows("\n".join(raw_lines))
    summary, table_rows = summarize_rows(
        rows,
        package=package,
        requested_duration_s=args.duration_s,
        elapsed_s=elapsed,
        min_sats=args.min_sats,
        max_hdop=args.max_hdop,
    )
    write_outputs(Path(args.out_dir), summary, table_rows, raw_lines)
    print(f"gps_status={summary['gps_status']}")
    print("motor_command_generated=false")
    print("ready_for_full_path_following=false")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Long outdoor GPS readiness watch. No motion.")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--duration-s", type=float, default=1200.0)
    parser.add_argument("--path-package", default="latest")
    parser.add_argument("--min-sats", type=int, default=4)
    parser.add_argument("--max-hdop", type=float, default=3.0)
    parser.add_argument("--allow-short-duration", default="false")
    parser.add_argument("--log")
    parser.add_argument("--out-dir", required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    args.allow_short_duration = str(args.allow_short_duration).lower() in {"1", "true", "yes"}
    try:
        return run(args)
    except RuntimeError as exc:
        print(f"reason={exc}")
        print("motor_command_generated=false")
        print("ready_for_full_path_following=false")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
