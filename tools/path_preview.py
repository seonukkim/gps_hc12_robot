from __future__ import annotations

import argparse
import datetime as dt
from pathlib import Path

import _bootstrap  # noqa: F401
import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt

from gps_coverage_core.geo import GeoPoint, latlon_to_local
from gps_coverage_core.planner import generate_lawnmower_path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Preview a lawnmower coverage path.")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Unused placeholder for CLI consistency")
    parser.add_argument("--lat-a", type=float, required=True, help="First corner latitude")
    parser.add_argument("--lon-a", type=float, required=True, help="First corner longitude")
    parser.add_argument("--lat-b", type=float, required=True, help="Opposite corner latitude")
    parser.add_argument("--lon-b", type=float, required=True, help="Opposite corner longitude")
    parser.add_argument("--spacing", type=float, required=True, help="Row spacing in meters")
    parser.add_argument(
        "--output",
        default=f"outputs/path_preview_{dt.datetime.now():%Y%m%d_%H%M%S}.png",
        help="Figure output path",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    point_a = GeoPoint(lat=args.lat_a, lon=args.lon_a)
    point_b = GeoPoint(lat=args.lat_b, lon=args.lon_b)
    path = generate_lawnmower_path(point_a, point_b, args.spacing)
    local_points = [latlon_to_local(point_a, point) for point in path]

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    xs = [point.x_m for point in local_points]
    ys = [point.y_m for point in local_points]

    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot(xs, ys, marker="o", linewidth=1.5)
    ax.set_title("Lawnmower Path Preview")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.grid(True, linestyle="--", alpha=0.4)
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)

    print(f"Saved {len(path)} waypoints to preview figure: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
