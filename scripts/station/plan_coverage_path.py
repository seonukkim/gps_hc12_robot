from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import re
import sys
from pathlib import Path
from typing import Any

ROOT_DIR = Path(__file__).resolve().parents[2]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt

from gps_coverage_core.planner import (
    generate_corner_rectangle_path,
    generate_coverage_path,
    latlon_to_xy,
)

CSV_FIELDS = (
    "index",
    "lat",
    "lon",
    "x_m",
    "y_m",
    "segment_type",
    "lane",
    "offset_m",
    "speed_mps",
    "notes",
)


def parse_lat_lon(text: str) -> dict[str, float]:
    parts = [part.strip() for part in text.split(",")]
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("point must be LAT,LON")

    try:
        lat = float(parts[0])
        lon = float(parts[1])
    except ValueError as exc:
        raise argparse.ArgumentTypeError("point must contain numeric LAT,LON") from exc

    if not -90.0 <= lat <= 90.0:
        raise argparse.ArgumentTypeError("latitude must be between -90 and 90")
    if not -180.0 <= lon <= 180.0:
        raise argparse.ArgumentTypeError("longitude must be between -180 and 180")

    return {"lat": lat, "lon": lon}


def parse_mission_name(text: str) -> str:
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", text):
        raise argparse.ArgumentTypeError(
            "mission name must contain only letters, numbers, dot, underscore, or hyphen"
        )
    return text


def default_mission_name() -> str:
    return f"coverage_{dt.datetime.now(tz=dt.UTC):%Y%m%d_%H%M%S}"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Generate a station-side coverage path dry-run. "
            "This writes mission files only and sends no rover commands."
        )
    )
    parser.add_argument(
        "--point-a",
        type=parse_lat_lon,
        required=True,
        help="Start corner as LAT,LON.",
    )
    parser.add_argument(
        "--point-b",
        type=parse_lat_lon,
        required=True,
        help="Opposite/end corner as LAT,LON in the default corner-rectangle mode.",
    )
    parser.add_argument(
        "--planner-mode",
        choices=("corner-rectangle", "baseline-width"),
        default="corner-rectangle",
        help="Planning geometry. Default: corner-rectangle.",
    )
    parser.add_argument(
        "--sweep-width-m",
        type=float,
        default=None,
        help="Coverage width in meters. Required only for --planner-mode baseline-width.",
    )
    parser.add_argument(
        "--lane-spacing-m",
        type=float,
        required=True,
        help="Lane spacing / sweep interval in meters.",
    )
    parser.add_argument("--speed-mps", type=float, default=None, help="Optional dry-run speed metadata")
    parser.add_argument(
        "--mission-name",
        type=parse_mission_name,
        default=None,
        help="Mission directory name. Defaults to coverage_YYYYmmdd_HHMMSS.",
    )
    parser.add_argument(
        "--output-dir",
        default="outputs/missions",
        help="Base output directory. Default: outputs/missions",
    )
    return parser


def validate_args(args: argparse.Namespace) -> None:
    if args.planner_mode == "baseline-width" and args.sweep_width_m is None:
        raise SystemExit("--sweep-width-m is required when --planner-mode baseline-width")
    if args.planner_mode == "corner-rectangle" and args.sweep_width_m is not None:
        print("warning: --sweep-width-m is ignored in corner-rectangle mode", file=sys.stderr)


def _lane_length_m(point_a: dict[str, float], point_b: dict[str, float]) -> float:
    bx_m, by_m = latlon_to_xy(point_b["lat"], point_b["lon"], point_a["lat"], point_a["lon"])
    return (bx_m**2 + by_m**2) ** 0.5


def _coverage_boundary(waypoints: list[dict[str, float | int | str]]) -> list[tuple[float, float]]:
    lane0 = [waypoint for waypoint in waypoints if int(waypoint["lane"]) == 0]
    max_lane = max(int(waypoint["lane"]) for waypoint in waypoints)
    lane_last = [waypoint for waypoint in waypoints if int(waypoint["lane"]) == max_lane]

    start0, end0 = lane0[0], lane0[1]
    if max_lane % 2 == 0:
        start_last, end_last = lane_last[0], lane_last[1]
    else:
        end_last, start_last = lane_last[0], lane_last[1]

    return [
        (float(start0["x_m"]), float(start0["y_m"])),
        (float(end0["x_m"]), float(end0["y_m"])),
        (float(end_last["x_m"]), float(end_last["y_m"])),
        (float(start_last["x_m"]), float(start_last["y_m"])),
        (float(start0["x_m"]), float(start0["y_m"])),
    ]


def _corner_rectangle_boundary(point_b_local: dict[str, float]) -> list[tuple[float, float]]:
    min_x = min(0.0, point_b_local["x_m"])
    max_x = max(0.0, point_b_local["x_m"])
    min_y = min(0.0, point_b_local["y_m"])
    max_y = max(0.0, point_b_local["y_m"])
    return [
        (min_x, min_y),
        (max_x, min_y),
        (max_x, max_y),
        (min_x, max_y),
        (min_x, min_y),
    ]


def _mission_waypoints(
    waypoints: list[dict[str, float | int | str]],
) -> list[dict[str, float | int | str]]:
    mission_waypoints: list[dict[str, float | int | str]] = []
    for waypoint in waypoints:
        index = int(waypoint["order"])
        segment_type = str(
            waypoint.get("segment_type", "lane_start" if index % 2 == 0 else "lane_end")
        )
        mission_waypoint: dict[str, float | int | str] = {
            "index": index,
            "lat": float(waypoint["lat"]),
            "lon": float(waypoint["lon"]),
            "x_m": float(waypoint["x_m"]),
            "y_m": float(waypoint["y_m"]),
            "segment_type": segment_type,
            "lane": int(waypoint["lane"]),
            "offset_m": float(waypoint["offset_m"]),
            "notes": str(waypoint.get("notes", "")),
        }
        if "speed_mps" in waypoint:
            mission_waypoint["speed_mps"] = float(waypoint["speed_mps"])
        mission_waypoints.append(mission_waypoint)
    return mission_waypoints


def build_mission(
    *,
    mission_name: str,
    point_a: dict[str, float],
    point_b: dict[str, float],
    planner_mode: str,
    sweep_width_m: float | None,
    lane_spacing_m: float,
    speed_mps: float | None,
    waypoints: list[dict[str, float | int | str]],
) -> dict[str, Any]:
    mission_waypoints = _mission_waypoints(waypoints)
    lane_count = len({int(waypoint["lane"]) for waypoint in waypoints})
    point_b_x_m, point_b_y_m = latlon_to_xy(
        point_b["lat"], point_b["lon"], point_a["lat"], point_a["lon"]
    )
    point_b_local = {"x_m": point_b_x_m, "y_m": point_b_y_m}
    if planner_mode == "corner-rectangle":
        boundary_points = _corner_rectangle_boundary(point_b_local)
        lane_length_m = abs(point_b_x_m)
        rectangle_x_extent_m = abs(point_b_x_m)
        rectangle_y_extent_m = abs(point_b_y_m)
    else:
        boundary_points = _coverage_boundary(waypoints)
        lane_length_m = _lane_length_m(point_a, point_b)
        rectangle_x_extent_m = None
        rectangle_y_extent_m = None
    boundary = [{"x_m": x_m, "y_m": y_m} for x_m, y_m in boundary_points]

    return {
        "schema": "station_coverage_path.v1",
        "metadata": {
            "mission_name": mission_name,
            "generated_at": dt.datetime.now(tz=dt.UTC).isoformat(),
            "dry_run": True,
            "sends_rover_commands": False,
            "planner_mode": planner_mode,
        },
        "inputs": {
            "point_a": point_a,
            "point_b": point_b,
            "sweep_width_m": sweep_width_m,
            "lane_spacing_m": lane_spacing_m,
            "speed_mps": speed_mps,
        },
        "local_origin": {
            "lat": point_a["lat"],
            "lon": point_a["lon"],
            "description": "point_a",
        },
        "input_points_local": {
            "point_a": {"x_m": 0.0, "y_m": 0.0, "role": "start corner"},
            "point_b": {
                "x_m": point_b_x_m,
                "y_m": point_b_y_m,
                "role": "opposite/end corner",
            },
        },
        "local_frame": {
            "x_m": "east",
            "y_m": "north",
            "planner_mode": planner_mode,
            "corner_rectangle_rule": "point_a and point_b are opposite corners",
        },
        "summary": {
            "lane_count": lane_count,
            "waypoint_count": len(mission_waypoints),
            "lane_length_m": lane_length_m,
            "sweep_width_m": sweep_width_m,
            "rectangle_x_extent_m": rectangle_x_extent_m,
            "rectangle_y_extent_m": rectangle_y_extent_m,
        },
        "coverage_boundary": boundary,
        "safety": [
            "PC/Mac-side dry-run only",
            "no serial port opened",
            "no HC-12 frames sent",
            "no rover commands generated",
        ],
        "waypoints": mission_waypoints,
    }


def save_mission_json(path: Path, mission: dict[str, Any]) -> None:
    with path.open("w", encoding="utf-8") as handle:
        json.dump(mission, handle, indent=2)
        handle.write("\n")


def save_mission_csv(path: Path, waypoints: list[dict[str, float | int | str]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for waypoint in waypoints:
            writer.writerow({field: waypoint.get(field, "") for field in CSV_FIELDS})


def save_preview_png(path: Path, mission: dict[str, Any]) -> None:
    waypoints = mission["waypoints"]
    xs = [float(waypoint["x_m"]) for waypoint in waypoints]
    ys = [float(waypoint["y_m"]) for waypoint in waypoints]

    boundary = mission["coverage_boundary"]
    bx = [float(point["x_m"]) for point in boundary]
    by = [float(point["y_m"]) for point in boundary]

    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot(bx, by, linestyle="--", color="tab:gray", linewidth=1.2, label="coverage boundary")
    ax.plot(xs, ys, marker="o", linewidth=1.5, label="boustrophedon path")
    point_a_local = mission["input_points_local"]["point_a"]
    point_b_local = mission["input_points_local"]["point_b"]
    ax.scatter(
        [float(point_a_local["x_m"])],
        [float(point_a_local["y_m"])],
        color="tab:green",
        s=70,
        label="Point A start",
        zorder=5,
    )
    ax.scatter(
        [float(point_b_local["x_m"])],
        [float(point_b_local["y_m"])],
        color="tab:red",
        s=70,
        label="Point B final/end corner",
        zorder=5,
    )

    for waypoint in waypoints:
        ax.annotate(
            str(waypoint["index"]),
            (float(waypoint["x_m"]), float(waypoint["y_m"])),
            textcoords="offset points",
            xytext=(4, 4),
            fontsize=8,
        )

    lane_spacing_m = float(mission["inputs"]["lane_spacing_m"])
    if mission["metadata"]["planner_mode"] == "corner-rectangle":
        sweep_extent_m = float(mission["summary"]["rectangle_y_extent_m"])
    else:
        sweep_extent_m = float(mission["summary"]["sweep_width_m"])
    residual_m = sweep_extent_m % lane_spacing_m
    if residual_m > 1e-6:
        spacing_note = f"lane spacing={lane_spacing_m:.2f} m, final residual={residual_m:.2f} m"
    else:
        spacing_note = f"lane spacing={lane_spacing_m:.2f} m"
    ax.text(
        0.02,
        0.02,
        spacing_note,
        transform=ax.transAxes,
        fontsize=9,
        bbox={"boxstyle": "round,pad=0.25", "facecolor": "white", "alpha": 0.75},
    )
    ax.set_title("Station Coverage Path Dry-Run")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def write_outputs(output_dir: Path, mission_name: str, mission: dict[str, Any]) -> dict[str, Path]:
    mission_dir = output_dir / mission_name
    mission_dir.mkdir(parents=True, exist_ok=True)

    paths = {
        "json": mission_dir / "mission.json",
        "csv": mission_dir / "mission.csv",
        "preview": mission_dir / "preview.png",
    }
    save_mission_json(paths["json"], mission)
    save_mission_csv(paths["csv"], mission["waypoints"])
    save_preview_png(paths["preview"], mission)
    return paths


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    validate_args(args)
    mission_name = args.mission_name or default_mission_name()

    if args.planner_mode == "corner-rectangle":
        raw_waypoints = generate_corner_rectangle_path(
            point_a=args.point_a,
            point_b=args.point_b,
            lane_spacing_m=args.lane_spacing_m,
            speed_mps=args.speed_mps,
        )
    else:
        raw_waypoints = generate_coverage_path(
            point_a=args.point_a,
            point_b=args.point_b,
            sweep_width_m=args.sweep_width_m,
            lane_spacing_m=args.lane_spacing_m,
            speed_mps=args.speed_mps,
        )
    mission = build_mission(
        mission_name=mission_name,
        point_a=args.point_a,
        point_b=args.point_b,
        planner_mode=args.planner_mode,
        sweep_width_m=args.sweep_width_m,
        lane_spacing_m=args.lane_spacing_m,
        speed_mps=args.speed_mps,
        waypoints=raw_waypoints,
    )
    paths = write_outputs(Path(args.output_dir), mission_name, mission)

    print("dry_run=true")
    print("sends_rover_commands=false")
    print(f"mission_name={mission_name}")
    print(f"lane_count={mission['summary']['lane_count']}")
    print(f"waypoint_count={mission['summary']['waypoint_count']}")
    print(f"mission_json={paths['json'].resolve()}")
    print(f"mission_csv={paths['csv'].resolve()}")
    print(f"preview_png={paths['preview'].resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
