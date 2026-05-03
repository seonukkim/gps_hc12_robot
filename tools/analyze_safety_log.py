from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path


KEY_VALUE_RE = re.compile(r"([A-Za-z0-9_]+)=([^\s]+)")
REQUIRED_FIELDS = (
    "mode",
    "rc_ok",
    "auto_sw",
    "ppm_age_ms",
    "steer_us",
    "throttle_us",
    "mode_us",
    "steer_norm",
    "throttle_norm",
    "station_age_ms",
    "station_manual_valid",
    "station_deadman",
    "station_estop",
    "control_source",
    "left_cmd",
    "right_cmd",
    "gps_fix",
)
MAX_PARSE_WARNINGS = 10


class ParseError(ValueError):
    """Raised when a USBDBG line is missing required fields or values."""


@dataclass(slots=True)
class USBDBGRecord:
    mode: str
    rc_ok: bool
    auto_sw: bool
    ppm_age_ms: int | None
    steer_us: int
    throttle_us: int
    mode_us: int
    steer_norm: float
    throttle_norm: float
    station_age_ms: int | None
    station_manual_valid: bool
    station_deadman: bool
    station_estop: bool
    control_source: str
    left_cmd: float
    right_cmd: float
    gps_fix: bool


@dataclass(slots=True)
class ParseWarnings:
    shown: int = 0
    suppressed: int = 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Summarize USBDBG safety behavior from one or more OpenRB logs."
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


def parse_int(name: str, value: str) -> int:
    try:
        return int(value)
    except ValueError as exc:
        raise ParseError(f"{name} must be an integer, got {value!r}") from exc


def parse_float(name: str, value: str) -> float:
    try:
        return float(value)
    except ValueError as exc:
        raise ParseError(f"{name} must be a float, got {value!r}") from exc


def parse_optional_int(name: str, value: str) -> int | None:
    if value == "NA":
        return None
    return parse_int(name, value)


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
        mode=fields["mode"],
        rc_ok=parse_bool("rc_ok", fields["rc_ok"]),
        auto_sw=parse_bool("auto_sw", fields["auto_sw"]),
        ppm_age_ms=parse_optional_int("ppm_age_ms", fields["ppm_age_ms"]),
        steer_us=parse_int("steer_us", fields["steer_us"]),
        throttle_us=parse_int("throttle_us", fields["throttle_us"]),
        mode_us=parse_int("mode_us", fields["mode_us"]),
        steer_norm=parse_float("steer_norm", fields["steer_norm"]),
        throttle_norm=parse_float("throttle_norm", fields["throttle_norm"]),
        station_age_ms=parse_optional_int("station_age_ms", fields["station_age_ms"]),
        station_manual_valid=parse_bool(
            "station_manual_valid", fields["station_manual_valid"]
        ),
        station_deadman=parse_bool("station_deadman", fields["station_deadman"]),
        station_estop=parse_bool("station_estop", fields["station_estop"]),
        control_source=fields["control_source"],
        left_cmd=parse_float("left_cmd", fields["left_cmd"]),
        right_cmd=parse_float("right_cmd", fields["right_cmd"]),
        gps_fix=parse_bool("gps_fix", fields["gps_fix"]),
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


def format_values(values: set[str]) -> str:
    if not values:
        return "n/a"
    return ", ".join(sorted(values))


def format_yes_no(value: bool) -> str:
    return "yes" if value else "no"


def command_is_nonzero(record: USBDBGRecord) -> bool:
    return record.left_cmd != 0.0 or record.right_cmd != 0.0


def main() -> int:
    args = build_parser().parse_args()

    usbdbg_lines_seen = 0
    parsed_records = 0
    malformed_usbdbg_lines = 0
    files_read = 0
    file_errors = 0
    warnings = ParseWarnings()

    modes: set[str] = set()
    control_sources: set[str] = set()
    rc_ok_true = 0
    rc_ok_false = 0
    max_abs_left_cmd = 0.0
    max_abs_right_cmd = 0.0
    nonzero_cmd_while_rc_bad = False
    nonzero_cmd_while_stop = False
    gps_remained_fix = True

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
                    modes.add(record.mode)
                    control_sources.add(record.control_source)
                    if record.rc_ok:
                        rc_ok_true += 1
                    else:
                        rc_ok_false += 1
                    max_abs_left_cmd = max(max_abs_left_cmd, abs(record.left_cmd))
                    max_abs_right_cmd = max(max_abs_right_cmd, abs(record.right_cmd))
                    if not record.gps_fix:
                        gps_remained_fix = False

                    if command_is_nonzero(record):
                        if not record.rc_ok:
                            nonzero_cmd_while_rc_bad = True
                        if record.control_source == "STOP":
                            nonzero_cmd_while_stop = True
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

    safety_pass = not nonzero_cmd_while_stop and not nonzero_cmd_while_rc_bad

    print(f"Parsed USBDBG lines: {parsed_records}")
    if malformed_usbdbg_lines:
        print(f"Skipped malformed USBDBG lines: {malformed_usbdbg_lines}")
    if file_errors:
        print(f"Files with read errors: {file_errors}")
    print(f"Unique modes: {format_values(modes)}")
    print(f"Unique control_source values: {format_values(control_sources)}")
    print(f"rc_ok=true: {rc_ok_true}")
    print(f"rc_ok=false: {rc_ok_false}")
    print(f"Max |left_cmd|: {max_abs_left_cmd:.3f}")
    print(f"Max |right_cmd|: {max_abs_right_cmd:.3f}")
    print(
        "Nonzero motor command while rc_ok=false: "
        f"{format_yes_no(nonzero_cmd_while_rc_bad)}"
    )
    print(
        "Nonzero motor command while control_source=STOP: "
        f"{format_yes_no(nonzero_cmd_while_stop)}"
    )
    print(f"GPS remained fix throughout test: {format_yes_no(gps_remained_fix)}")
    print(f"Safety: {'PASS' if safety_pass else 'FAIL'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
