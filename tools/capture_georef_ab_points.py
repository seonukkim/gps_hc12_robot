from __future__ import annotations

import argparse
import csv
import json
import re
import time
from pathlib import Path
from typing import Sequence

try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401


FIELDS = ("point_label", "lat", "lon", "sample_count")


def _parse_rows(text: str) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    for line in text.splitlines():
        values = {key.lower(): value for key, value in re.findall(r"([A-Za-z0-9_]+)=([^,\s]+)", line)}
        if values:
            rows.append(values)
    return rows


def _average_lat_lon(rows: Sequence[dict[str, str]]) -> tuple[float, float, int]:
    coords: list[tuple[float, float]] = []
    for row in rows:
        lat = row.get("current_lat", row.get("lat"))
        lon = row.get("current_lon", row.get("lon"))
        if lat is None or lon is None:
            continue
        coords.append((float(lat), float(lon)))
    if not coords:
        raise ValueError("NO_LAT_LON_SAMPLES")
    return (
        sum(lat for lat, _ in coords) / len(coords),
        sum(lon for _, lon in coords) / len(coords),
        len(coords),
    )


def _parse_range(text: str) -> tuple[int, int]:
    if ":" not in text:
        index = int(text)
        return index, index + 1
    start, end = text.split(":", 1)
    return int(start), int(end)


def _write_outputs(out_dir: Path, *, a: tuple[float, float, int], b: tuple[float, float, int], mode: str) -> dict[str, Path]:
    out_dir.mkdir(parents=True, exist_ok=True)
    data = {
        "capture_mode": mode,
        "points": {
            "A": {"lat": a[0], "lon": a[1], "sample_count": a[2]},
            "B": {"lat": b[0], "lon": b[1], "sample_count": b[2]},
        },
        "georeference_available": True,
        "motor_command_generated": False,
        "physical_output_active": False,
    }
    json_path = out_dir / "field_points_georef.json"
    csv_path = out_dir / "field_points_georef.csv"
    json_path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    with csv_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerow({"point_label": "A", "lat": a[0], "lon": a[1], "sample_count": a[2]})
        writer.writerow({"point_label": "B", "lat": b[0], "lon": b[1], "sample_count": b[2]})
    return {"json": json_path, "csv": csv_path}


def _capture_live(port: str, sample_count: int) -> tuple[tuple[float, float, int], tuple[float, float, int]]:
    try:
        import serial  # type: ignore[import-not-found]
    except ImportError as exc:
        raise RuntimeError("pyserial is not available; cannot read live USBDBG") from exc

    def collect(handle, label: str) -> tuple[float, float, int]:
        input(f"Press Enter to capture {label} average...")
        rows: list[dict[str, str]] = []
        deadline = time.monotonic() + max(30.0, sample_count * 2.0)
        while len(rows) < sample_count and time.monotonic() < deadline:
            raw = handle.readline()
            if not raw:
                continue
            parsed = _parse_rows(raw.decode("utf-8", errors="replace"))
            if parsed:
                rows.extend(parsed)
        return _average_lat_lon(rows)

    with serial.Serial(port, baudrate=115200, timeout=1.0) as handle:
        return collect(handle, "A"), collect(handle, "B")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Capture georeferenced A/B points for Stage 13 no-motion path tracking.")
    parser.add_argument("--mode", choices=("manual_latlon", "replay_log", "live_usbdbg"), required=True)
    parser.add_argument("--a-lat", type=float)
    parser.add_argument("--a-lon", type=float)
    parser.add_argument("--b-lat", type=float)
    parser.add_argument("--b-lon", type=float)
    parser.add_argument("--log")
    parser.add_argument("--a-row-range")
    parser.add_argument("--b-row-range")
    parser.add_argument("--port")
    parser.add_argument("--sample-count", type=int, default=10)
    parser.add_argument("--out-dir", default="outputs/field_georef_capture/latest")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.mode == "manual_latlon":
        missing = [
            name
            for name in ("a_lat", "a_lon", "b_lat", "b_lon")
            if getattr(args, name) is None
        ]
        if missing:
            raise SystemExit("--a-lat/--a-lon/--b-lat/--b-lon are required in manual_latlon mode")
        a = (float(args.a_lat), float(args.a_lon), 1)
        b = (float(args.b_lat), float(args.b_lon), 1)
    elif args.mode == "replay_log":
        if not args.log or not args.a_row_range or not args.b_row_range:
            raise SystemExit("--log, --a-row-range, and --b-row-range are required in replay_log mode")
        rows = _parse_rows(Path(args.log).read_text(encoding="utf-8", errors="replace"))
        a_start, a_end = _parse_range(args.a_row_range)
        b_start, b_end = _parse_range(args.b_row_range)
        a = _average_lat_lon(rows[a_start:a_end])
        b = _average_lat_lon(rows[b_start:b_end])
    else:
        if not args.port:
            raise SystemExit("--port is required in live_usbdbg mode")
        a, b = _capture_live(args.port, args.sample_count)
    outputs = _write_outputs(Path(args.out_dir), a=a, b=b, mode=args.mode)
    print("Georeferenced A/B points captured.")
    print(f"field_points_georef_json={outputs['json']}")
    print(f"field_points_georef_csv={outputs['csv']}")
    print(f"A_lat={a[0]}")
    print(f"A_lon={a[1]}")
    print(f"B_lat={b[0]}")
    print(f"B_lon={b[1]}")
    print("motor_command_generated=false")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
