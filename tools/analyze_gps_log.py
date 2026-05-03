from __future__ import annotations

import argparse
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path


KEY_VALUE_RE = re.compile(r"([A-Za-z0-9_]+)=([^\s]+)")
REQUIRED_FIELDS = (
    "gps_fix",
    "gps_lat",
    "gps_lon",
    "gps_sats",
    "gps_hdop",
    "gps_age_ms",
    "rc_ok",
    "mode",
    "control_source",
)
EARTH_RADIUS_M = 6_371_000.0
MAX_PARSE_WARNINGS = 10


class ParseError(ValueError):
    """Raised when a USBDBG line is missing required fields or values."""


@dataclass(slots=True)
class USBDBGRecord:
    gps_fix: bool
    gps_lat: float | None
    gps_lon: float | None
    gps_sats: int | None
    gps_hdop: float | None
    gps_age_ms: int | None
    rc_ok: bool
    mode: str
    control_source: str


@dataclass(slots=True)
class ParseWarnings:
    shown: int = 0
    suppressed: int = 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Summarize USBDBG GPS health from one or more OpenRB USB debug logs."
    )
    parser.add_argument("paths", nargs="+", help="One or more log files to analyze")
    return parser


def parse_bool(name: str, value: str) -> bool:
    lowered = value.lower()
    if lowered == "true":
        return True
    if lowered == "false":
        return False
    raise ParseError(f"{name} must be true/false, got {value!r}")


def parse_optional_int(name: str, value: str) -> int | None:
    if value == "NA":
        return None
    try:
        return int(value)
    except ValueError as exc:
        raise ParseError(f"{name} must be an integer or NA, got {value!r}") from exc


def parse_optional_float(name: str, value: str) -> float | None:
    if value == "NA":
        return None
    try:
        return float(value)
    except ValueError as exc:
        raise ParseError(f"{name} must be a float or NA, got {value!r}") from exc


def parse_usbdbg_line(text: str) -> USBDBGRecord:
    usbdbg_offset = text.find("USBDBG")
    if usbdbg_offset < 0:
        raise ParseError("line does not contain USBDBG")

    payload = text[usbdbg_offset:]
    fields = dict(KEY_VALUE_RE.findall(payload))
    missing = [field_name for field_name in REQUIRED_FIELDS if field_name not in fields]
    if missing:
        raise ParseError(f"missing fields: {', '.join(missing)}")

    return USBDBGRecord(
        gps_fix=parse_bool("gps_fix", fields["gps_fix"]),
        gps_lat=parse_optional_float("gps_lat", fields["gps_lat"]),
        gps_lon=parse_optional_float("gps_lon", fields["gps_lon"]),
        gps_sats=parse_optional_int("gps_sats", fields["gps_sats"]),
        gps_hdop=parse_optional_float("gps_hdop", fields["gps_hdop"]),
        gps_age_ms=parse_optional_int("gps_age_ms", fields["gps_age_ms"]),
        rc_ok=parse_bool("rc_ok", fields["rc_ok"]),
        mode=fields["mode"],
        control_source=fields["control_source"],
    )


def emit_parse_warning(warnings: ParseWarnings, path: Path, line_number: int, message: str) -> None:
    if warnings.shown < MAX_PARSE_WARNINGS:
        print(f"Warning: {path}:{line_number}: {message}", file=sys.stderr)
        warnings.shown += 1
        return
    warnings.suppressed += 1


def finish_parse_warnings(warnings: ParseWarnings) -> None:
    if warnings.suppressed:
        print(
            f"Warning: suppressed {warnings.suppressed} additional parse warning(s).",
            file=sys.stderr,
        )


def format_int(value: float) -> str:
    return str(int(round(value)))


def format_float(value: float, digits: int = 2) -> str:
    return f"{value:.{digits}f}"


def format_min_mean_max(
    values: list[int | float], *, digits: int = 2, integer_bounds: bool = False
) -> str:
    if not values:
        return "n/a"

    low = min(values)
    high = max(values)
    mean = sum(values) / len(values)

    if integer_bounds:
        return (
            f"{format_int(low)} / {format_float(mean, digits)} / {format_int(high)}"
        )
    return f"{format_float(low, digits)} / {format_float(mean, digits)} / {format_float(high, digits)}"


def equirectangular_distance_m(lat_a: float, lon_a: float, lat_b: float, lon_b: float) -> float:
    lat_a_rad = math.radians(lat_a)
    lat_b_rad = math.radians(lat_b)
    lon_a_rad = math.radians(lon_a)
    lon_b_rad = math.radians(lon_b)
    x = (lon_b_rad - lon_a_rad) * math.cos((lat_a_rad + lat_b_rad) / 2.0)
    y = lat_b_rad - lat_a_rad
    return math.hypot(x, y) * EARTH_RADIUS_M


def main() -> int:
    args = build_parser().parse_args()

    usbdbg_lines_seen = 0
    parsed_records = 0
    malformed_usbdbg_lines = 0
    files_read = 0
    file_errors = 0
    warnings = ParseWarnings()

    gps_fix_true = 0
    sats_values: list[int] = []
    hdop_values: list[float] = []
    age_values: list[int] = []
    fixed_points: list[tuple[float, float]] = []

    for raw_path in args.paths:
        path = Path(raw_path)
        try:
            with path.open("r", encoding="utf-8", errors="replace") as handle:
                files_read += 1
                for line_number, line in enumerate(handle, start=1):
                    if "USBDBG" not in line:
                        continue

                    usbdbg_lines_seen += 1
                    try:
                        record = parse_usbdbg_line(line)
                    except ParseError as exc:
                        malformed_usbdbg_lines += 1
                        emit_parse_warning(warnings, path, line_number, str(exc))
                        continue

                    parsed_records += 1
                    if record.gps_fix:
                        gps_fix_true += 1

                        # Summarize only real fixes so placeholder no-fix values
                        # such as sats=0 or hdop=99.99 do not distort the report.
                        if record.gps_sats is not None:
                            sats_values.append(record.gps_sats)
                        if record.gps_hdop is not None:
                            hdop_values.append(record.gps_hdop)
                        if record.gps_age_ms is not None:
                            age_values.append(record.gps_age_ms)
                        if record.gps_lat is not None and record.gps_lon is not None:
                            fixed_points.append((record.gps_lat, record.gps_lon))
        except FileNotFoundError:
            print(f"Error: file not found: {path}", file=sys.stderr)
            file_errors += 1
        except IsADirectoryError:
            print(f"Error: expected a file, got directory: {path}", file=sys.stderr)
            file_errors += 1
        except OSError as exc:
            print(f"Error: failed to read {path}: {exc}", file=sys.stderr)
            file_errors += 1

    finish_parse_warnings(warnings)

    if files_read == 0:
        print("Error: no readable input files.", file=sys.stderr)
        return 1

    if usbdbg_lines_seen == 0:
        print("Error: no USBDBG lines found in the provided input file(s).", file=sys.stderr)
        return 1

    if parsed_records == 0:
        print("Error: found USBDBG lines, but none could be parsed successfully.", file=sys.stderr)
        return 1

    gps_fix_ratio = gps_fix_true / parsed_records

    print(f"Parsed USBDBG lines: {parsed_records}")
    if malformed_usbdbg_lines:
        print(f"Skipped malformed USBDBG lines: {malformed_usbdbg_lines}")
    if file_errors:
        print(f"Files with read errors: {file_errors}")
    print(f"gps_fix=true: {gps_fix_true}/{parsed_records} ({gps_fix_ratio:.1%})")
    print(f"sats min/mean/max: {format_min_mean_max(sats_values, digits=2, integer_bounds=True)}")
    print(f"hdop min/mean/max: {format_min_mean_max(hdop_values, digits=2)}")
    print(
        "gps_age_ms min/mean/max: "
        f"{format_min_mean_max(age_values, digits=1, integer_bounds=True)}"
    )

    if fixed_points:
        latitudes = [point[0] for point in fixed_points]
        longitudes = [point[1] for point in fixed_points]
        drift_m = equirectangular_distance_m(
            min(latitudes),
            min(longitudes),
            max(latitudes),
            max(longitudes),
        )
        print(f"lat min/max: {format_float(min(latitudes), 6)} / {format_float(max(latitudes), 6)}")
        print(f"lon min/max: {format_float(min(longitudes), 6)} / {format_float(max(longitudes), 6)}")
        print(f"approximate position drift: {format_float(drift_m, 2)} m")
    else:
        print("lat min/max: n/a")
        print("lon min/max: n/a")
        print("approximate position drift: n/a")
        print("WARNING: no GPS fix lines found; lat/lon range and drift are unavailable.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
