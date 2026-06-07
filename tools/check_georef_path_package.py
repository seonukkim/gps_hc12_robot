from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Sequence

try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401

from tools.path_no_motion_validation import PathPackageResolutionError, resolve_path_package
from tools.station_path_package_tracker import lat_lon_to_local


def inspect_georef(package: dict[str, object], selected_path: Path) -> dict[str, object]:
    georef = package.get("georeference")
    workspace = package.get("normalized_workspace", {})
    if not isinstance(georef, dict):
        return {
            "selected_path_package": str(selected_path),
            "georeference_available": False,
            "reason": "NO_GEOREFERENCE_IN_PATH_PACKAGE",
            "motor_command_generated": False,
        }
    raw_a_lat = float(georef["raw_A_lat"])
    raw_a_lon = float(georef["raw_A_lon"])
    raw_b_lat = float(georef["raw_B_lat"])
    raw_b_lon = float(georef["raw_B_lon"])
    a_local = lat_lon_to_local(package, raw_a_lat, raw_a_lon)
    b_local = lat_lon_to_local(package, raw_b_lat, raw_b_lon)
    width = float(workspace.get("width_m", 0.0)) if isinstance(workspace, dict) else 0.0
    height = float(workspace.get("height_m", 0.0)) if isinstance(workspace, dict) else 0.0
    sanity_ok = a_local is not None and b_local is not None and width > 0 and height > 0
    return {
        "selected_path_package": str(selected_path),
        "georeference_available": True,
        "origin_lat": georef["origin_lat"],
        "origin_lon": georef["origin_lon"],
        "raw_A_lat": raw_a_lat,
        "raw_A_lon": raw_a_lon,
        "raw_B_lat": raw_b_lat,
        "raw_B_lon": raw_b_lon,
        "A_local_coordinates": a_local,
        "B_local_coordinates": b_local,
        "local_frame_type": georef["local_frame_type"],
        "x_axis_source": georef["x_axis_source"],
        "meters_per_deg_lat": georef["meters_per_deg_lat"],
        "meters_per_deg_lon": georef["meters_per_deg_lon"],
        "expected_rectangle_width_m": width,
        "expected_rectangle_height_m": height,
        "conversion_sanity_check": sanity_ok,
        "motor_command_generated": False,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Check georeference metadata in a path_package.json.")
    parser.add_argument("--path-package", default="latest")
    parser.add_argument("--out-dir")
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
    result = inspect_georef(package, selected)
    for key, value in result.items():
        print(f"{key}={value}")
    if args.out_dir:
        out_dir = Path(args.out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        (out_dir / "georef_path_package_check.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    return 0 if result.get("georeference_available") else 1


if __name__ == "__main__":
    raise SystemExit(main())
