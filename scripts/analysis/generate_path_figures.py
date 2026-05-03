from __future__ import annotations

import argparse
from pathlib import Path
from typing import Sequence

from _figure_common import (
    COLOR_BLUE,
    COLOR_GRAY,
    COLOR_GREEN,
    COLOR_LIGHT,
    COLOR_NAVY,
    COLOR_ORANGE,
    FigureResult,
    add_output_args,
    add_panel_label,
    add_source_note,
    equalize_xy,
    finalize_axes,
    load_waypoint_dataset,
    save_figure_all,
)

import matplotlib.pyplot as plt

SCRIPT_NAME = "scripts/analysis/generate_path_figures.py"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Generate report-quality A/B region and lawnmower path figures."
    )
    parser.add_argument(
        "--waypoints",
        nargs="*",
        type=Path,
        default=None,
        help="Optional waypoint CSV/JSON files. Defaults to data/mock_runs/**/waypoints.*.",
    )
    add_output_args(parser)
    return parser


def _xy_points(waypoints: Sequence[dict[str, float | int]]) -> tuple[list[float], list[float]]:
    return [float(point["x"]) for point in waypoints], [float(point["y"]) for point in waypoints]


def _lane_endpoints(
    waypoints: Sequence[dict[str, float | int]],
) -> dict[int, list[tuple[float, float]]]:
    lanes: dict[int, list[tuple[float, float]]] = {}
    for waypoint in waypoints:
        lanes.setdefault(int(waypoint["lane"]), []).append(
            (float(waypoint["x"]), float(waypoint["y"]))
        )
    return lanes


def _plot_region(ax, waypoints: Sequence[dict[str, float | int]]) -> None:
    lanes = _lane_endpoints(waypoints)
    first_lane = lanes[min(lanes)]
    last_lane = lanes[max(lanes)]
    corners = [first_lane[0], first_lane[1], last_lane[0], last_lane[1], first_lane[0]]
    xs = [point[0] for point in corners]
    ys = [point[1] for point in corners]
    ax.fill(xs, ys, color=COLOR_LIGHT, alpha=0.95, label="Planar work region")
    ax.plot(xs, ys, color=COLOR_NAVY, linewidth=1.4)


def _generate_ab_region(dataset, output_dirs: Sequence[str | Path] | None) -> FigureResult:
    waypoints = dataset.waypoints
    xs, ys = _xy_points(waypoints)
    fig, ax = plt.subplots(figsize=(7.2, 5.4))
    _plot_region(ax, waypoints)

    ax.scatter([xs[0], xs[1]], [ys[0], ys[1]], s=70, color=[COLOR_GREEN, COLOR_ORANGE], zorder=4)
    ax.annotate("A", (xs[0], ys[0]), xytext=(-15, -16), textcoords="offset points", weight="bold")
    ax.annotate("B", (xs[1], ys[1]), xytext=(9, 8), textcoords="offset points", weight="bold")
    ax.annotate(
        "Operator-selected A/B edge",
        xy=((xs[0] + xs[1]) / 2.0, (ys[0] + ys[1]) / 2.0),
        xytext=(10, -26),
        textcoords="offset points",
        arrowprops={"arrowstyle": "->", "color": COLOR_GRAY, "lw": 1.0},
        color=COLOR_GRAY,
        fontsize=9,
    )
    ax.plot([xs[0], xs[1]], [ys[0], ys[1]], color=COLOR_ORANGE, linewidth=2.0)
    ax.set_title("A/B Region Definition")
    ax.set_xlabel("Local East (m)")
    ax.set_ylabel("Local North (m)")
    add_panel_label(ax, "MOCK MISSION", color=COLOR_GRAY)
    add_source_note(ax, dataset.source_label, mock=dataset.is_mock)
    equalize_xy(ax, xs, ys)
    finalize_axes(ax)
    fig.tight_layout()
    save_figure_all(fig, "fig_ab_region_definition.png", output_dirs)
    return FigureResult(
        filename="fig_ab_region_definition.png",
        script=SCRIPT_NAME,
        data_source=dataset.data_source,
        recommended_use="Planning method section: explain manual A/B point selection and planar rectangular region definition.",
        caption=(
            "Mock planar work-region definition from operator-selected A/B points. The shaded "
            "rectangle and offsets illustrate the current planning assumption, not a validated hull survey."
        ),
    )


def _generate_lawnmower_preview(dataset, output_dirs: Sequence[str | Path] | None) -> FigureResult:
    waypoints = dataset.waypoints
    xs, ys = _xy_points(waypoints)
    fig, ax = plt.subplots(figsize=(7.2, 5.4))
    _plot_region(ax, waypoints)
    ax.plot(xs, ys, marker="o", color=COLOR_BLUE, linewidth=2.0, markersize=5)
    for left, right in zip(waypoints, waypoints[1:]):
        ax.annotate(
            "",
            xy=(float(right["x"]), float(right["y"])),
            xytext=(float(left["x"]), float(left["y"])),
            arrowprops={"arrowstyle": "->", "color": COLOR_BLUE, "lw": 1.1, "shrinkA": 6, "shrinkB": 6},
        )
    ax.set_title("Lawnmower Path Preview")
    ax.set_xlabel("Local East (m)")
    ax.set_ylabel("Local North (m)")
    add_panel_label(ax, "MOCK MISSION", color=COLOR_GRAY)
    add_source_note(ax, dataset.source_label, mock=dataset.is_mock)
    equalize_xy(ax, xs, ys)
    finalize_axes(ax)
    fig.tight_layout()
    save_figure_all(fig, "fig_lawnmower_path_preview.png", output_dirs)
    return FigureResult(
        filename="fig_lawnmower_path_preview.png",
        script=SCRIPT_NAME,
        data_source=dataset.data_source,
        recommended_use="Planning/results section: show the generated coverage pattern before hardware execution.",
        caption=(
            "Mock lawnmower coverage preview generated by the ROS-independent Python planner. "
            "The sequence shows the intended coverage geometry only; it is not an autonomous field result."
        ),
    )


def _generate_waypoint_sequence(dataset, output_dirs: Sequence[str | Path] | None) -> FigureResult:
    waypoints = dataset.waypoints
    xs, ys = _xy_points(waypoints)
    orders = [int(point["order"]) for point in waypoints]
    fig, ax = plt.subplots(figsize=(7.2, 5.4))
    _plot_region(ax, waypoints)
    scatter = ax.scatter(xs, ys, c=orders, cmap="viridis", s=80, edgecolor="white", linewidth=0.8, zorder=4)
    ax.plot(xs, ys, color=COLOR_NAVY, alpha=0.55, linewidth=1.2)
    for waypoint in waypoints:
        ax.annotate(
            str(int(waypoint["order"])),
            (float(waypoint["x"]), float(waypoint["y"])),
            xytext=(6, 6),
            textcoords="offset points",
            fontsize=8,
            color=COLOR_NAVY,
        )
    colorbar = fig.colorbar(scatter, ax=ax, fraction=0.045, pad=0.03)
    colorbar.set_label("Waypoint order")
    ax.set_title("Waypoint Sequence")
    ax.set_xlabel("Local East (m)")
    ax.set_ylabel("Local North (m)")
    add_panel_label(ax, "MOCK MISSION", color=COLOR_GRAY)
    add_source_note(ax, dataset.source_label, mock=dataset.is_mock)
    equalize_xy(ax, xs, ys)
    finalize_axes(ax)
    fig.tight_layout()
    save_figure_all(fig, "fig_waypoint_sequence.png", output_dirs)
    return FigureResult(
        filename="fig_waypoint_sequence.png",
        script=SCRIPT_NAME,
        data_source=dataset.data_source,
        recommended_use="Planner implementation section: document waypoint ordering and alternating lane traversal.",
        caption=(
            "Mock waypoint order for the generated lawnmower path. Labels identify the planner's "
            "sequential output and alternating lane direction."
        ),
    )


def generate(
    *,
    output_dirs: Sequence[str | Path] | None = None,
    waypoint_paths: Sequence[Path] | None = None,
) -> list[FigureResult]:
    dataset = load_waypoint_dataset(waypoint_paths)
    return [
        _generate_ab_region(dataset, output_dirs),
        _generate_lawnmower_preview(dataset, output_dirs),
        _generate_waypoint_sequence(dataset, output_dirs),
    ]


def main() -> int:
    args = build_parser().parse_args()
    results = generate(output_dirs=args.output_dirs, waypoint_paths=args.waypoints)
    for result in results:
        print(f"generated {result.filename} from {result.data_source}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
