from __future__ import annotations

import argparse
from pathlib import Path
from typing import Sequence

from _figure_common import (
    COLOR_BLUE,
    COLOR_GRAY,
    COLOR_GREEN,
    COLOR_NAVY,
    COLOR_ORANGE,
    FigureResult,
    add_output_args,
    add_panel_label,
    add_source_note,
    equalize_xy,
    finalize_axes,
    fixed_xy_records,
    gps_local_xy,
    load_gps_datasets,
    mock_gps_dataset,
    save_figure_all,
)

import matplotlib.pyplot as plt

SCRIPT_NAME = "scripts/analysis/generate_gps_figures.py"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Generate report-quality GPS validation figures.")
    parser.add_argument(
        "--gps-log",
        nargs="*",
        type=Path,
        default=None,
        help="Optional GPS CSV/log files. Defaults to data/nmea_logs/*.csv and data/gps_logs/*.log.",
    )
    add_output_args(parser)
    return parser


def _choose_dataset(paths: Sequence[Path] | None) -> object:
    datasets = load_gps_datasets(paths)
    if not datasets:
        return mock_gps_dataset()

    def score(dataset) -> tuple[int, int, int, int]:
        records = dataset.records
        fix_values = {record.fix_valid for record in records}
        valid_points = len(fixed_xy_records(records))
        has_fix_transition = int(len(fix_values) > 1)
        return (has_fix_transition, valid_points, len(records), -int(dataset.is_mock))

    return max(datasets, key=score)


def _valid_metric_records(dataset, metric: str):
    valid = []
    for record in dataset.records:
        value = getattr(record, metric)
        if not record.fix_valid or value is None:
            continue
        if metric == "hdop" and value >= 20:
            continue
        valid.append(record)
    return valid


def _generate_fix_timeline(dataset, output_dirs: Sequence[str | Path] | None) -> FigureResult:
    t_s = [record.t_s for record in dataset.records]
    fix = [1 if record.fix_valid else 0 for record in dataset.records]
    fig, ax = plt.subplots(figsize=(8, 3.8))
    ax.step(t_s, fix, where="post", color=COLOR_GREEN, linewidth=2.0)
    ax.fill_between(t_s, fix, step="post", color=COLOR_GREEN, alpha=0.16)
    ax.set_ylim(-0.15, 1.15)
    ax.set_yticks([0, 1])
    ax.set_yticklabels(["No fix", "Fix"])
    ax.set_title("GPS Fix Timeline")
    ax.set_xlabel("Elapsed sample time (s)")
    ax.set_ylabel("Fix status")
    add_panel_label(ax, "MOCK/DEMO" if dataset.is_mock else "REAL LOG", color=COLOR_GRAY)
    add_source_note(ax, dataset.source_label, mock=dataset.is_mock)
    finalize_axes(ax)
    fig.tight_layout()
    save_figure_all(fig, "fig_gps_fix_timeline.png", output_dirs)
    return FigureResult(
        filename="fig_gps_fix_timeline.png",
        script=SCRIPT_NAME,
        data_source=dataset.data_source,
        recommended_use="GPS validation section: show whether the selected log reported fix availability over the capture.",
        caption=(
            "GPS fix status over the selected log. For USB debug captures without per-line timestamps, "
            "elapsed time is reconstructed from sample order and should be treated as nominal."
        ),
    )


def _generate_satellites(dataset, output_dirs: Sequence[str | Path] | None) -> FigureResult:
    records = _valid_metric_records(dataset, "satellites")
    fig, ax = plt.subplots(figsize=(8, 3.8))
    ax.plot(
        [record.t_s for record in records],
        [record.satellites for record in records],
        marker="o",
        markersize=3,
        color=COLOR_BLUE,
    )
    ax.set_title("GPS Satellites vs Time")
    ax.set_xlabel("Elapsed sample time (s)")
    ax.set_ylabel("Satellites")
    ax.set_ylim(bottom=0)
    add_panel_label(ax, "MOCK/DEMO" if dataset.is_mock else "REAL LOG", color=COLOR_GRAY)
    add_source_note(ax, dataset.source_label, mock=dataset.is_mock)
    finalize_axes(ax)
    fig.tight_layout()
    save_figure_all(fig, "fig_gps_satellites_vs_time.png", output_dirs)
    return FigureResult(
        filename="fig_gps_satellites_vs_time.png",
        script=SCRIPT_NAME,
        data_source=dataset.data_source,
        recommended_use="GPS validation section: summarize satellite count stability during valid-fix records.",
        caption=(
            "Satellite count for records with valid GPS fix in the selected log. No-fix placeholder "
            "values are omitted from the plotted series."
        ),
    )


def _generate_hdop(dataset, output_dirs: Sequence[str | Path] | None) -> FigureResult:
    records = _valid_metric_records(dataset, "hdop")
    fig, ax = plt.subplots(figsize=(8, 3.8))
    ax.plot(
        [record.t_s for record in records],
        [record.hdop for record in records],
        marker="o",
        markersize=3,
        color=COLOR_ORANGE,
    )
    ax.axhline(2.5, color=COLOR_GRAY, linewidth=1.0, linestyle=":", label="Reference: HDOP 2.5")
    ax.set_title("GPS HDOP vs Time")
    ax.set_xlabel("Elapsed sample time (s)")
    ax.set_ylabel("HDOP")
    ax.set_ylim(bottom=0)
    ax.legend(loc="upper right")
    add_panel_label(ax, "MOCK/DEMO" if dataset.is_mock else "REAL LOG", color=COLOR_GRAY)
    add_source_note(ax, dataset.source_label, mock=dataset.is_mock)
    finalize_axes(ax)
    fig.tight_layout()
    save_figure_all(fig, "fig_gps_hdop_vs_time.png", output_dirs)
    return FigureResult(
        filename="fig_gps_hdop_vs_time.png",
        script=SCRIPT_NAME,
        data_source=dataset.data_source,
        recommended_use="GPS validation section: report HDOP trend for valid-fix samples without claiming navigation accuracy.",
        caption=(
            "HDOP values for records with valid GPS fix in the selected log. The plot is a log "
            "quality summary and does not by itself validate closed-loop path tracking."
        ),
    )


def _generate_position_scatter(dataset, output_dirs: Sequence[str | Path] | None) -> FigureResult:
    fixed = fixed_xy_records(dataset.records)
    xs, ys = gps_local_xy(dataset.records)
    times = [record.t_s for record in fixed]
    fig, ax = plt.subplots(figsize=(6.6, 5.4))
    scatter = ax.scatter(xs, ys, c=times, cmap="plasma", s=36, edgecolor="white", linewidth=0.4)
    if xs and ys:
        ax.scatter([xs[0]], [ys[0]], marker="s", s=70, color=COLOR_GREEN, edgecolor="white", label="First fix")
        ax.scatter([xs[-1]], [ys[-1]], marker="^", s=80, color=COLOR_NAVY, edgecolor="white", label="Last fix")
    colorbar = fig.colorbar(scatter, ax=ax, fraction=0.045, pad=0.03)
    colorbar.set_label("Elapsed sample time (s)")
    ax.set_title("GPS Position Scatter")
    ax.set_xlabel("Local East from first fix (m)")
    ax.set_ylabel("Local North from first fix (m)")
    ax.legend(loc="best")
    add_panel_label(ax, "MOCK/DEMO" if dataset.is_mock else "REAL LOG", color=COLOR_GRAY)
    add_source_note(ax, dataset.source_label, mock=dataset.is_mock)
    equalize_xy(ax, xs, ys)
    finalize_axes(ax)
    fig.tight_layout()
    save_figure_all(fig, "fig_gps_position_scatter.png", output_dirs)
    return FigureResult(
        filename="fig_gps_position_scatter.png",
        script=SCRIPT_NAME,
        data_source=dataset.data_source,
        recommended_use="GPS validation section: visualize fixed-position spread in local meters.",
        caption=(
            "Local-position scatter for valid GPS fixes, converted relative to the first fixed point. "
            "This summarizes observed spread in the selected log, not hull-surface localization performance."
        ),
    )


def generate(
    *,
    output_dirs: Sequence[str | Path] | None = None,
    gps_paths: Sequence[Path] | None = None,
) -> list[FigureResult]:
    dataset = _choose_dataset(gps_paths)
    return [
        _generate_fix_timeline(dataset, output_dirs),
        _generate_satellites(dataset, output_dirs),
        _generate_hdop(dataset, output_dirs),
        _generate_position_scatter(dataset, output_dirs),
    ]


def main() -> int:
    args = build_parser().parse_args()
    results = generate(output_dirs=args.output_dirs, gps_paths=args.gps_log)
    for result in results:
        print(f"generated {result.filename} from {result.data_source}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
