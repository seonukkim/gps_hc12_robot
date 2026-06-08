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


WATCH_FIELDS = (
    "row_index",
    "gps_block_reason",
    "position_source",
    "gps_chars",
    "gps_sats",
    "gps_hdop",
    "current_lat",
    "current_lon",
    "local_x_m",
    "local_y_m",
)
GPS_SERIAL_COUNTER_KEYS = (
    "gps_chars",
    "gps_bytes",
    "gps_byte_count",
    "gps_nmea_count",
    "gps_nmea_sentence_count",
    "nmea_count",
    "nmea_sentence_count",
)


def _float(value: object) -> float | None:
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


def _median(values: Sequence[float]) -> float | None:
    return statistics.median(values) if values else None


def parse_rows(text: str) -> list[dict[str, str]]:
    return station_path_package_tracker.parse_usbdbg_rows(text)


def _row_gps_serial_count(row: dict[str, str]) -> float | None:
    for key in GPS_SERIAL_COUNTER_KEYS:
        parsed = _float(row.get(key))
        if parsed is not None:
            return parsed
    return None


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
    gps_chars = [_row_gps_serial_count(row) for row in rows]
    gps_chars_values = [value for value in gps_chars if value is not None]
    serial_present = bool(gps_chars_values and max(gps_chars_values) > 0)
    gps_update_rows = [
        row for row in rows
        if any(key in row for key in ("gps_block_reason", "position_source", "gps_sats", "gps_hdop", "current_lat", "current_lon", *GPS_SERIAL_COUNTER_KEYS))
    ]
    sats_values = [value for row in rows if (value := _float(row.get("gps_sats"))) is not None]
    hdop_values = [value for row in rows if (value := _float(row.get("gps_hdop"))) is not None]
    fix_rows: list[dict[str, str]] = []
    quality_rows: list[dict[str, str]] = []
    local_pose_count = 0
    table_rows: list[dict[str, object]] = []
    for index, row in enumerate(rows):
        lat = _float(row.get("current_lat"))
        lon = _float(row.get("current_lon"))
        sats = _float(row.get("gps_sats"))
        hdop = _float(row.get("gps_hdop"))
        local_x: float | str = ""
        local_y: float | str = ""
        if lat is not None and lon is not None:
            fix_rows.append(row)
            if sats is not None and hdop is not None and sats >= min_sats and hdop <= max_hdop:
                quality_rows.append(row)
            if package is not None:
                local = station_path_package_tracker.lat_lon_to_local(package, lat, lon)
                if local is not None:
                    local_pose_count += 1
                    local_x, local_y = local
        table_rows.append({
            "row_index": index,
            "gps_block_reason": row.get("gps_block_reason", "NA"),
            "position_source": row.get("position_source", "NA"),
            "gps_chars": "" if (chars := _row_gps_serial_count(row)) is None else chars,
            "gps_sats": "" if sats is None else sats,
            "gps_hdop": "" if hdop is None else hdop,
            "current_lat": "" if lat is None else lat,
            "current_lon": "" if lon is None else lon,
            "local_x_m": local_x,
            "local_y_m": local_y,
        })
    local_required = package is not None
    local_ready = local_pose_count > 0 or not local_required
    if quality_rows and local_ready:
        status = "GPS_READY_FULL"
    elif fix_rows:
        status = "GPS_BAD_QUALITY"
    elif serial_present:
        status = "GPS_SERIAL_PRESENT_NO_FIX"
    elif elapsed < requested_duration_s:
        status = "GPS_WAIT_LONGER"
    elif not gps_update_rows:
        status = "GPS_NO_SERIAL_DATA"
    else:
        status = "GPS_NO_SERIAL_DATA"
    return {
        "requested_duration_s": requested_duration_s,
        "elapsed_s": elapsed,
        "gps_status": status,
        "gps_update_rows": len(gps_update_rows),
        "gps_serial_present": serial_present,
        "gps_chars_min": min(gps_chars_values) if gps_chars_values else None,
        "gps_chars_max": max(gps_chars_values) if gps_chars_values else None,
        "gps_fix_rows": len(fix_rows),
        "gps_quality_rows": len(quality_rows),
        "local_pose_available": local_pose_count > 0,
        "gps_sats_min": min(sats_values) if sats_values else None,
        "gps_sats_median": _median(sats_values),
        "gps_sats_max": max(sats_values) if sats_values else None,
        "gps_hdop_min": min(hdop_values) if hdop_values else None,
        "gps_hdop_median": _median(hdop_values),
        "gps_hdop_max": max(hdop_values) if hdop_values else None,
        "motor_command_generated": False,
        "ready_for_full_path_following": False,
    }, table_rows


def write_outputs(out_dir: Path, summary: dict[str, object], table_rows: Sequence[dict[str, object]], raw_lines: Sequence[str]) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "raw_usbdbg.log").write_text("\n".join(raw_lines) + ("\n" if raw_lines else ""), encoding="utf-8")
    with (out_dir / "gps_link_and_fix_watch.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=WATCH_FIELDS)
        writer.writeheader()
        for row in table_rows:
            writer.writerow({field: row.get(field, "") for field in WATCH_FIELDS})
    (out_dir / "gps_link_and_fix_watch_summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    lines = ["# GPS Link And Fix Watch", "", "No motion and no motor commands.", ""]
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
            raise RuntimeError("pyserial is not available; cannot run GPS watch") from exc
        deadline = started + args.duration_s
        with serial.Serial(args.port, baudrate=115200, timeout=0.5) as handle:
            while time.monotonic() < deadline:
                raw = handle.readline()
                if raw:
                    line = raw.decode("utf-8", errors="replace").strip()
                    print(line)
                    raw_lines.append(line)
        elapsed = time.monotonic() - started
    summary, table_rows = summarize_rows(
        parse_rows("\n".join(raw_lines)),
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
    parser = argparse.ArgumentParser(description="Long GPS link/fix watch. No movement.")
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
