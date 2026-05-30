from __future__ import annotations

import argparse
import csv
import datetime as dt
import math
from pathlib import Path
from typing import Sequence

try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401

from gps_coverage_core.geo import GeoPoint, LocalPoint, latlon_to_local, local_to_latlon


CSV_FIELDS = (
    "index",
    "lat",
    "lon",
    "x_m",
    "y_m",
    "distance_from_start_m",
    "segment_type",
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Generate a station-side start/goal path preview. "
            "This is preview-only and sends no HC-12 or rover commands."
        )
    )
    parser.add_argument("--start-lat", type=float, required=True, help="Start latitude")
    parser.add_argument("--start-lon", type=float, required=True, help="Start longitude")
    parser.add_argument("--goal-lat", type=float, required=True, help="Goal latitude")
    parser.add_argument("--goal-lon", type=float, required=True, help="Goal longitude")
    parser.add_argument("--spacing-m", type=float, default=2.0, help="Waypoint spacing in meters")
    parser.add_argument(
        "--out-dir",
        default="outputs/path_preview",
        help="Output directory for waypoints.csv, summary.md, and optional preview.png",
    )
    return parser


def _validate_lat_lon(lat: float, lon: float, label: str) -> None:
    if not -90.0 <= lat <= 90.0:
        raise ValueError(f"{label} latitude must be between -90 and 90")
    if not -180.0 <= lon <= 180.0:
        raise ValueError(f"{label} longitude must be between -180 and 180")


def build_waypoints(
    *,
    start: GeoPoint,
    goal: GeoPoint,
    spacing_m: float,
) -> list[dict[str, float | int | str]]:
    if spacing_m <= 0.0:
        raise ValueError("spacing_m must be positive")
    _validate_lat_lon(start.lat, start.lon, "start")
    _validate_lat_lon(goal.lat, goal.lon, "goal")

    goal_local = latlon_to_local(start, goal)
    total_distance_m = math.hypot(goal_local.x_m, goal_local.y_m)
    if math.isclose(total_distance_m, 0.0, abs_tol=1e-9):
        raise ValueError("start and goal must not be identical")

    unit_x = goal_local.x_m / total_distance_m
    unit_y = goal_local.y_m / total_distance_m
    distances = [0.0]
    next_distance = spacing_m
    while next_distance < total_distance_m and not math.isclose(
        next_distance, total_distance_m, abs_tol=1e-9
    ):
        distances.append(next_distance)
        next_distance += spacing_m
    if not math.isclose(distances[-1], total_distance_m, abs_tol=1e-9):
        distances.append(total_distance_m)

    waypoints: list[dict[str, float | int | str]] = []
    for index, distance_m in enumerate(distances):
        local = LocalPoint(x_m=unit_x * distance_m, y_m=unit_y * distance_m)
        point = local_to_latlon(start, local)
        if index == 0:
            segment_type = "start"
        elif index == len(distances) - 1:
            segment_type = "goal"
        else:
            segment_type = "intermediate"
        waypoints.append(
            {
                "index": index,
                "lat": point.lat,
                "lon": point.lon,
                "x_m": local.x_m,
                "y_m": local.y_m,
                "distance_from_start_m": distance_m,
                "segment_type": segment_type,
            }
        )
    return waypoints


def _write_csv(path: Path, waypoints: Sequence[dict[str, float | int | str]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for waypoint in waypoints:
            writer.writerow({field: waypoint[field] for field in CSV_FIELDS})


def _format_optional_png(path: Path | None) -> str:
    if path is None:
        return "not generated; matplotlib is not available"
    return str(path)


def _write_summary(
    *,
    path: Path,
    start: GeoPoint,
    goal: GeoPoint,
    spacing_m: float,
    waypoints: Sequence[dict[str, float | int | str]],
    csv_path: Path,
    png_path: Path | None,
) -> None:
    total_distance_m = float(waypoints[-1]["distance_from_start_m"])
    lines = [
        "# Station Path Planning Preview",
        "",
        "This artifact is preview-only.",
        "",
        "- No HC-12 commands are sent.",
        "- No rover motor commands are generated.",
        "- No firmware is uploaded or modified.",
        "- This is not autonomous execution.",
        "- Physical path following still requires heading/course estimation and steering validation.",
        "",
        "## Inputs",
        "",
        f"- start: `{start.lat:.7f}, {start.lon:.7f}`",
        f"- goal: `{goal.lat:.7f}, {goal.lon:.7f}`",
        f"- spacing_m: `{spacing_m:.3f}`",
        "",
        "## Summary",
        "",
        f"- total_distance_m: `{total_distance_m:.3f}`",
        f"- waypoint_count: `{len(waypoints)}`",
        f"- generated_at_utc: `{dt.datetime.now(tz=dt.UTC).isoformat()}`",
        f"- waypoints_csv: `{csv_path}`",
        f"- preview_png: `{_format_optional_png(png_path)}`",
        "",
        "## Next Required Validation",
        "",
        "1. Use this output only for station-side visual inspection.",
        "2. Add single-waypoint steering dry-run before physical waypoint following.",
        "3. Add heading/course estimation before any rover follows this path.",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_png(
    *,
    path: Path,
    waypoints: Sequence[dict[str, float | int | str]],
    start: GeoPoint,
    goal: GeoPoint,
) -> Path | None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return None

    xs = [float(waypoint["x_m"]) for waypoint in waypoints]
    ys = [float(waypoint["y_m"]) for waypoint in waypoints]
    fig, ax = plt.subplots(figsize=(7, 5))
    ax.plot(xs, ys, marker="o", linewidth=1.5, label="preview path")
    for waypoint in waypoints:
        ax.annotate(str(waypoint["index"]), (float(waypoint["x_m"]), float(waypoint["y_m"])))
    ax.scatter([xs[0]], [ys[0]], c="green", s=70, label="start")
    ax.scatter([xs[-1]], [ys[-1]], c="red", s=70, label="goal")
    ax.set_title("Station-Side Path Preview Only")
    ax.set_xlabel("East from start (m)")
    ax.set_ylabel("North from start (m)")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.axis("equal")
    ax.legend()
    ax.text(
        0.01,
        0.01,
        "No HC-12, no rover commands, no firmware upload",
        transform=ax.transAxes,
        fontsize=8,
        va="bottom",
    )
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    _ = (start, goal)
    return path


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    start = GeoPoint(lat=args.start_lat, lon=args.start_lon)
    goal = GeoPoint(lat=args.goal_lat, lon=args.goal_lon)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    waypoints = build_waypoints(start=start, goal=goal, spacing_m=args.spacing_m)
    csv_path = out_dir / "waypoints.csv"
    summary_path = out_dir / "summary.md"
    png_candidate = out_dir / "preview.png"

    _write_csv(csv_path, waypoints)
    png_path = _write_png(path=png_candidate, waypoints=waypoints, start=start, goal=goal)
    _write_summary(
        path=summary_path,
        start=start,
        goal=goal,
        spacing_m=args.spacing_m,
        waypoints=waypoints,
        csv_path=csv_path,
        png_path=png_path,
    )

    print("Station path planning preview generated.")
    print("Preview only: no HC-12 commands, no rover motor commands, no firmware upload.")
    print(f"Waypoints CSV: {csv_path}")
    print(f"Summary: {summary_path}")
    if png_path is None:
        print("Preview PNG: skipped because matplotlib is not available")
    else:
        print(f"Preview PNG: {png_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
