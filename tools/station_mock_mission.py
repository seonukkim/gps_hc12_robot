from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import _bootstrap  # noqa: F401

from gps_coverage_core.planner import generate_lawnmower_path

CSV_FIELDS = ("order", "lane", "lat", "lon", "x", "y")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Generate a station-side mock coverage mission without HC-12 or ROS."
    )
    parser.add_argument("--a-lat", type=float, required=True, help="Point A latitude")
    parser.add_argument("--a-lon", type=float, required=True, help="Point A longitude")
    parser.add_argument("--b-lat", type=float, required=True, help="Point B latitude")
    parser.add_argument("--b-lon", type=float, required=True, help="Point B longitude")
    parser.add_argument("--spacing-m", type=float, default=8.0, help="Lane spacing in meters")
    parser.add_argument("--num-lanes", type=int, default=4, help="Number of lanes")
    parser.add_argument("--out-dir", default="data/mock_runs", help="Output directory")
    return parser


def save_waypoints_json(path: Path, waypoints: list[dict[str, float | int]]) -> None:
    with path.open("w", encoding="utf-8") as handle:
        json.dump(waypoints, handle, indent=2)
        handle.write("\n")


def save_waypoints_csv(path: Path, waypoints: list[dict[str, float | int]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for waypoint in waypoints:
            writer.writerow({field: waypoint[field] for field in CSV_FIELDS})


def save_path_preview(path: Path, waypoints: list[dict[str, float | int]]) -> bool:
    try:
        import matplotlib

        matplotlib.use("Agg")

        import matplotlib.pyplot as plt
    except ImportError:
        return False

    xs = [float(waypoint["x"]) for waypoint in waypoints]
    ys = [float(waypoint["y"]) for waypoint in waypoints]

    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot(xs, ys, marker="o", linewidth=1.5)
    ax.set_title("Mock Mission Path Preview")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.grid(True, linestyle="--", alpha=0.4)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def build_summary_text(
    point_a: dict[str, float],
    point_b: dict[str, float],
    spacing_m: float,
    num_lanes: int,
    waypoint_count: int,
    outputs: dict[str, str],
    preview_created: bool,
) -> str:
    lines = [
        f"A: {point_a['lat']:.6f}, {point_a['lon']:.6f}",
        f"B: {point_b['lat']:.6f}, {point_b['lon']:.6f}",
        f"spacing_m: {spacing_m:.2f}",
        f"num_lanes: {num_lanes}",
        f"waypoints: {waypoint_count}",
        f"waypoints_json: {outputs['waypoints_json']}",
        f"waypoints_csv: {outputs['waypoints_csv']}",
        f"mission_summary: {outputs['mission_summary']}",
        (
            f"path_preview: {outputs['path_preview']}"
            if preview_created
            else "path_preview: skipped (matplotlib not installed)"
        ),
    ]
    return "\n".join(lines) + "\n"


def main() -> int:
    args = build_parser().parse_args()

    point_a = {"lat": args.a_lat, "lon": args.a_lon}
    point_b = {"lat": args.b_lat, "lon": args.b_lon}

    output_dir = Path(args.out_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    waypoints = generate_lawnmower_path(
        point_a=point_a,
        point_b=point_b,
        spacing_m=args.spacing_m,
        num_lanes=args.num_lanes,
    )

    json_path = output_dir / "waypoints.json"
    csv_path = output_dir / "waypoints.csv"
    summary_path = output_dir / "mission_summary.txt"
    preview_path = output_dir / "path_preview.png"

    save_waypoints_json(json_path, waypoints)
    save_waypoints_csv(csv_path, waypoints)
    preview_created = save_path_preview(preview_path, waypoints)

    outputs = {
        "waypoints_json": str(json_path.resolve()),
        "waypoints_csv": str(csv_path.resolve()),
        "mission_summary": str(summary_path.resolve()),
        "path_preview": str(preview_path.resolve()),
    }
    summary_text = build_summary_text(
        point_a=point_a,
        point_b=point_b,
        spacing_m=args.spacing_m,
        num_lanes=args.num_lanes,
        waypoint_count=len(waypoints),
        outputs=outputs,
        preview_created=preview_created,
    )

    summary_path.write_text(summary_text, encoding="utf-8")
    print(summary_text, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
