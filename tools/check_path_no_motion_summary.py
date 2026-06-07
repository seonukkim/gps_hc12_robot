from __future__ import annotations

import argparse
import math
import re
from pathlib import Path
from typing import Sequence


def _parse_summary(text: str) -> dict[str, str]:
    values: dict[str, str] = {}
    for line in text.splitlines():
        match = re.match(r"^- ([A-Za-z0-9_]+): `?(.*?)`?$", line.strip())
        if match:
            values[match.group(1)] = match.group(2).strip("`")
    return values


def _is_true(value: str | None) -> bool:
    return str(value).strip().lower() in {"1", "true", "ok", "yes", "ready"}


def _is_false(value: str | None) -> bool:
    return str(value).strip().lower() in {"0", "false", "no", "off"}


def _finite_float(value: str | None) -> float | None:
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


def evaluate_summary(values: dict[str, str], *, min_sats: int = 4, max_hdop: float = 3.0) -> tuple[str, str]:
    if values.get("validation_mode") == "live_serial" and not _is_true(values.get("serial_opened")):
        return "FAIL", "serial was expected for live validation but was not opened"
    if values.get("validation_mode") == "live_serial" and values.get("live_path_package_connected") == "False":
        return "FAIL", "live target source is not connected to the generated path package"
    if not _is_false(values.get("motor_command_generated")):
        return "FAIL", "motor output is not confirmed disabled"
    if not _is_false(values.get("physical_output_active")):
        return "FAIL", "physical output is active or not reported false"
    if not _is_true(values.get("path_package_loaded")):
        return "FAIL", "load a path_package.json before validation"
    if values.get("position_source") != "gps" or values.get("gps_status") == "FAIL":
        return "FAIL", "wait for a GPS position fix before target preview"
    if not _is_true(values.get("target_distance_finite")) or not _is_true(values.get("target_bearing_finite")):
        return "FAIL", "target distance or bearing is NA"

    sats = _finite_float(values.get("gps_sats"))
    hdop = _finite_float(values.get("gps_hdop"))
    if sats is None or hdop is None:
        return "WAIT", "GPS position exists, but sats/HDOP quality is not reported"
    if sats < min_sats:
        return "WAIT", f"wait for at least {min_sats} GPS satellites"
    if hdop > max_hdop:
        return "WAIT", f"wait for GPS HDOP <= {max_hdop:g}"

    if values.get("heading_status") in {"DIAG_ONLY", "WAITING_FOR_MOTION_OR_DIAG_ONLY"}:
        return "WAIT", "GPS course heading is unavailable while stationary; target distance/bearing remain diagnostic"
    return "PASS", "target preview ready"


def resolve_summary_path(path_arg: str | None, summary_arg: str | None) -> Path:
    if summary_arg:
        return Path(summary_arg)
    if not path_arg:
        raise SystemExit("ERROR: provide an output directory or --summary path.")
    path = Path(path_arg)
    return path / "summary.md" if path.is_dir() else path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Check a concise no-motion path validation summary.")
    parser.add_argument("path", nargs="?", help="Validation output directory or summary.md path.")
    parser.add_argument("--summary", help="Explicit summary.md path.")
    parser.add_argument("--min-sats", type=int, default=4)
    parser.add_argument("--max-hdop", type=float, default=3.0)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    summary_path = resolve_summary_path(args.path, args.summary)
    values = _parse_summary(summary_path.read_text(encoding="utf-8"))
    verdict, action = evaluate_summary(values, min_sats=args.min_sats, max_hdop=args.max_hdop)
    print(f"{verdict}: {action}")
    return 0 if verdict in {"PASS", "WAIT"} else 1


if __name__ == "__main__":
    raise SystemExit(main())
