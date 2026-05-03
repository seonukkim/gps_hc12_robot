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
    COLOR_RED,
    FigureResult,
    add_output_args,
    save_figure_all,
)

import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Rectangle

SCRIPT_NAME = "scripts/analysis/generate_system_figures.py"
SCHEMATIC_SOURCE = "mock schematic derived from repository architecture, protocol, and safety docs"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Generate report-quality system schematic figures.")
    add_output_args(parser)
    return parser


def _blank_figure(figsize=(9.0, 5.6)):
    fig, ax = plt.subplots(figsize=figsize)
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    return fig, ax


def _box(ax, xy, width, height, text, *, facecolor=COLOR_LIGHT, edgecolor=COLOR_NAVY, fontsize=9):
    patch = FancyBboxPatch(
        xy,
        width,
        height,
        boxstyle="round,pad=0.018,rounding_size=0.018",
        linewidth=1.25,
        edgecolor=edgecolor,
        facecolor=facecolor,
    )
    ax.add_patch(patch)
    ax.text(
        xy[0] + width / 2.0,
        xy[1] + height / 2.0,
        text,
        ha="center",
        va="center",
        fontsize=fontsize,
        color="#202427",
        linespacing=1.25,
    )
    return patch


def _arrow(ax, start, end, *, text: str | None = None, color=COLOR_NAVY, rad=0.0):
    arrow = FancyArrowPatch(
        start,
        end,
        arrowstyle="-|>",
        mutation_scale=13,
        linewidth=1.4,
        color=color,
        connectionstyle=f"arc3,rad={rad}",
    )
    ax.add_patch(arrow)
    if text:
        mid_x = (start[0] + end[0]) / 2.0
        mid_y = (start[1] + end[1]) / 2.0
        ax.text(
            mid_x,
            mid_y + 0.025,
            text,
            ha="center",
            va="center",
            fontsize=8,
            color=color,
            bbox={"facecolor": "white", "edgecolor": "none", "pad": 1.5, "alpha": 0.9},
        )


def _source_footer(ax) -> None:
    ax.text(
        0.01,
        0.02,
        "SCHEMATIC: repository-derived architecture; not a measured performance result.",
        ha="left",
        va="bottom",
        fontsize=8,
        color=COLOR_GRAY,
    )


def _generate_system_overview(output_dirs: Sequence[str | Path] | None) -> FigureResult:
    fig, ax = _blank_figure()
    ax.text(0.5, 0.95, "System Overview", ha="center", va="top", fontsize=15, weight="bold")

    _box(ax, (0.06, 0.62), 0.22, 0.16, "Operator\nmanual A/B selection\nreport review", facecolor="#f3f7fb")
    _box(ax, (0.06, 0.34), 0.22, 0.18, "Ground station\nUbuntu 24.04 + Python\nheartbeat + STOP default", facecolor="#f3f7fb")
    _box(ax, (0.39, 0.47), 0.18, 0.14, "HC-12\nUART radio link", facecolor="#f7f4ed", edgecolor=COLOR_ORANGE)
    _box(ax, (0.68, 0.50), 0.23, 0.19, "Rover controller\nOpenRB-150 firmware\nsafety ownership", facecolor="#f3fbf5", edgecolor=COLOR_GREEN)
    _box(ax, (0.68, 0.22), 0.23, 0.15, "ESC / motor outputs\nwheel-off-ground\nbench testing", facecolor="#fff6f4", edgecolor=COLOR_RED)
    _box(ax, (0.70, 0.76), 0.18, 0.10, "GPS module\nSerial3 telemetry", facecolor="#f3f7fb")
    _box(ax, (0.38, 0.76), 0.18, 0.10, "RC transmitter\nmanual override", facecolor="#f3f7fb")
    ax.add_patch(Rectangle((0.36, 0.10), 0.30, 0.12, facecolor="#e7eef2", edgecolor=COLOR_GRAY, linewidth=1.1))
    ax.text(0.51, 0.16, "Planar hull work area\ncurrent planning assumption", ha="center", va="center", fontsize=9)

    _arrow(ax, (0.17, 0.62), (0.17, 0.52), text="A/B points", color=COLOR_BLUE)
    _arrow(ax, (0.28, 0.43), (0.39, 0.53), text="@TYPE,SEQ,...*CS", color=COLOR_NAVY)
    _arrow(ax, (0.57, 0.54), (0.68, 0.58), text="commands", color=COLOR_NAVY)
    _arrow(ax, (0.68, 0.53), (0.57, 0.50), text="telemetry", color=COLOR_GRAY)
    _arrow(ax, (0.79, 0.50), (0.79, 0.37), text="bounded output", color=COLOR_RED)
    _arrow(ax, (0.79, 0.76), (0.79, 0.69), text="GPS", color=COLOR_BLUE)
    _arrow(ax, (0.47, 0.76), (0.68, 0.64), text="RC priority", color=COLOR_GREEN)
    _source_footer(ax)

    fig.tight_layout()
    save_figure_all(fig, "fig_system_overview.png", output_dirs)
    return FigureResult(
        filename="fig_system_overview.png",
        script=SCRIPT_NAME,
        data_source=SCHEMATIC_SOURCE,
        recommended_use="System architecture section: introduce station, radio, rover, RC, GPS, and safety boundaries.",
        caption=(
            "Repository-derived schematic of the current pre-ROS2 system concept. It shows the "
            "intended station, radio, rover, RC, GPS, and motor boundaries without claiming completed end-to-end autonomy."
        ),
    )


def _generate_control_flow(output_dirs: Sequence[str | Path] | None) -> FigureResult:
    fig, ax = _blank_figure(figsize=(9.2, 5.2))
    ax.text(0.5, 0.94, "Control Flow", ha="center", va="top", fontsize=15, weight="bold")

    boxes = [
        ((0.04, 0.58), "Manual A/B\npoint selection", COLOR_BLUE),
        ((0.23, 0.58), "Planar\nregion model", COLOR_BLUE),
        ((0.42, 0.58), "Lawnmower\npath planner", COLOR_BLUE),
        ((0.61, 0.58), "Waypoint list\npreview/export", COLOR_BLUE),
        ((0.80, 0.58), "Future mission\nexecution", COLOR_GRAY),
    ]
    for xy, text, edge in boxes:
        _box(ax, xy, 0.14, 0.14, text, facecolor="#f5f8fb", edgecolor=edge, fontsize=8.5)
    for left, right in zip(boxes, boxes[1:]):
        _arrow(ax, (left[0][0] + 0.14, left[0][1] + 0.07), (right[0][0], right[0][1] + 0.07), color=COLOR_NAVY)

    _box(
        ax,
        (0.23, 0.27),
        0.18,
        0.15,
        "Station runtime\nheartbeat + STOP\nby default",
        facecolor="#fff8ec",
        edgecolor=COLOR_ORANGE,
        fontsize=8.5,
    )
    _box(
        ax,
        (0.49, 0.27),
        0.18,
        0.15,
        "Rover safety gate\nRC priority\nlink timeout STOP",
        facecolor="#f1fbf4",
        edgecolor=COLOR_GREEN,
        fontsize=8.5,
    )
    _box(
        ax,
        (0.75, 0.27),
        0.16,
        0.15,
        "Motor command\nbounded output",
        facecolor="#fff4f2",
        edgecolor=COLOR_RED,
        fontsize=8.5,
    )
    _arrow(ax, (0.70, 0.58), (0.41, 0.42), text="operator chooses when to run", color=COLOR_GRAY, rad=-0.1)
    _arrow(ax, (0.41, 0.345), (0.49, 0.345), text="validated frames", color=COLOR_NAVY)
    _arrow(ax, (0.67, 0.345), (0.75, 0.345), text="safe source", color=COLOR_NAVY)

    ax.text(
        0.5,
        0.13,
        "Startup behavior remains safe: no live motor-driving AUTO commands are sent by the station unless explicitly changed.",
        ha="center",
        va="center",
        fontsize=9,
        color=COLOR_RED,
    )
    _source_footer(ax)
    fig.tight_layout()
    save_figure_all(fig, "fig_control_flow.png", output_dirs)
    return FigureResult(
        filename="fig_control_flow.png",
        script=SCRIPT_NAME,
        data_source=SCHEMATIC_SOURCE,
        recommended_use="Control architecture section: explain planner-to-station-to-rover flow and safe station defaults.",
        caption=(
            "Control-flow schematic for the current Python prototype. The diagram separates offline "
            "planning from runtime station behavior and highlights heartbeat/STOP defaults."
        ),
    )


def _generate_state_machine(output_dirs: Sequence[str | Path] | None) -> FigureResult:
    fig, ax = _blank_figure(figsize=(8.6, 5.4))
    ax.text(0.5, 0.95, "Safety State Machine", ha="center", va="top", fontsize=15, weight="bold")

    _box(ax, (0.08, 0.56), 0.18, 0.14, "FAILSAFE\nSTOP output", facecolor="#fff4f2", edgecolor=COLOR_RED)
    _box(ax, (0.39, 0.60), 0.18, 0.14, "MANUAL\nRC source", facecolor="#f1fbf4", edgecolor=COLOR_GREEN)
    _box(ax, (0.70, 0.56), 0.18, 0.14, "AUTO_READY\nSTOP until command", facecolor="#fff8ec", edgecolor=COLOR_ORANGE)
    _box(ax, (0.39, 0.27), 0.18, 0.14, "STOP\nneutral output", facecolor="#f5f8fb", edgecolor=COLOR_BLUE)

    _arrow(ax, (0.26, 0.64), (0.39, 0.67), text="RC valid", color=COLOR_GREEN)
    _arrow(ax, (0.39, 0.62), (0.26, 0.59), text="RC lost", color=COLOR_RED)
    _arrow(ax, (0.57, 0.67), (0.70, 0.64), text="auto switch", color=COLOR_ORANGE)
    _arrow(ax, (0.70, 0.59), (0.57, 0.62), text="manual switch", color=COLOR_GREEN)
    _arrow(ax, (0.79, 0.56), (0.52, 0.41), text="no valid command", color=COLOR_BLUE, rad=-0.08)
    _arrow(ax, (0.17, 0.56), (0.43, 0.41), text="failsafe STOP", color=COLOR_RED, rad=0.08)
    _arrow(ax, (0.48, 0.41), (0.48, 0.60), text="RC manual priority", color=COLOR_GREEN)

    ax.text(
        0.5,
        0.14,
        "State labels reflect current USBDBG fields and safety intent. ROS2 orchestration is planned later.",
        ha="center",
        va="center",
        fontsize=9,
        color=COLOR_GRAY,
    )
    _source_footer(ax)
    fig.tight_layout()
    save_figure_all(fig, "fig_state_machine.png", output_dirs)
    return FigureResult(
        filename="fig_state_machine.png",
        script=SCRIPT_NAME,
        data_source=SCHEMATIC_SOURCE,
        recommended_use="Safety design section: summarize rover-side state logic and STOP/failsafe behavior.",
        caption=(
            "Repository-derived safety state-machine schematic based on current USB debug state fields "
            "and documented safety boundaries. It is a design summary, not formal verification."
        ),
    )


def generate(*, output_dirs: Sequence[str | Path] | None = None) -> list[FigureResult]:
    return [
        _generate_system_overview(output_dirs),
        _generate_control_flow(output_dirs),
        _generate_state_machine(output_dirs),
    ]


def main() -> int:
    args = build_parser().parse_args()
    results = generate(output_dirs=args.output_dirs)
    for result in results:
        print(f"generated {result.filename} from {result.data_source}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
