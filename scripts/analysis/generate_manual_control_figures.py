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
    COLOR_RED,
    FigureResult,
    add_output_args,
    add_panel_label,
    add_source_note,
    finalize_axes,
    load_safety_datasets,
    mock_safety_dataset,
    save_figure_all,
)

import matplotlib.pyplot as plt

SCRIPT_NAME = "scripts/analysis/generate_manual_control_figures.py"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Generate report-quality manual-control and failsafe figures."
    )
    parser.add_argument(
        "--safety-log",
        nargs="*",
        type=Path,
        default=None,
        help=(
            "Optional USBDBG safety log files. Defaults to data/safety_logs/*.log "
            "and data/gps_logs/*.log."
        ),
    )
    add_output_args(parser)
    return parser


def _datasets_or_mock(paths: Sequence[Path] | None):
    datasets = load_safety_datasets(paths)
    return datasets if datasets else [mock_safety_dataset()]


def _choose_manual_dataset(paths: Sequence[Path] | None):
    datasets = _datasets_or_mock(paths)

    def score(dataset) -> tuple[int, int, float, int, int]:
        nonzero = sum(
            1
            for record in dataset.records
            if abs(record.left_cmd) > 0.001 or abs(record.right_cmd) > 0.001
        )
        rc_manual = sum(1 for record in dataset.records if record.control_source == "RC_MANUAL")
        total_command = sum(abs(record.left_cmd) + abs(record.right_cmd) for record in dataset.records)
        safety_log = int("data/safety_logs/" in dataset.source_label)
        return (int(nonzero > 0), nonzero, total_command, safety_log, rc_manual)

    return max(datasets, key=score)


def _choose_failsafe_dataset(paths: Sequence[Path] | None):
    datasets = _datasets_or_mock(paths)

    def score(dataset) -> tuple[int, int, int]:
        failsafe = sum(1 for record in dataset.records if record.mode == "FAILSAFE")
        rc_bad = sum(1 for record in dataset.records if not record.rc_ok)
        stop = sum(1 for record in dataset.records if record.control_source == "STOP")
        return (failsafe + rc_bad, stop, len(dataset.records))

    selected = max(datasets, key=score)
    if not any(record.mode == "FAILSAFE" or not record.rc_ok for record in selected.records):
        return mock_safety_dataset()
    return selected


def _choose_transition_dataset(paths: Sequence[Path] | None):
    datasets = _datasets_or_mock(paths)

    def score(dataset) -> tuple[int, int, int, int]:
        unique_sources = len({record.control_source for record in dataset.records})
        unique_modes = len({record.mode for record in dataset.records})
        safety_log = int("data/safety_logs/" in dataset.source_label)
        return (unique_sources, safety_log, unique_modes, len(dataset.records))

    return max(datasets, key=score)


def _boolean_segments(times: Sequence[float], flags: Sequence[bool]) -> list[tuple[float, float]]:
    segments: list[tuple[float, float]] = []
    if not times:
        return segments
    default_step = times[1] - times[0] if len(times) > 1 else 1.0
    start: float | None = None
    for index, flag in enumerate(flags):
        if flag and start is None:
            start = times[index]
        if start is not None and (not flag or index == len(flags) - 1):
            end_index = index if not flag else index + 1
            if end_index < len(times):
                end = times[end_index]
            else:
                end = times[-1] + default_step
            segments.append((start, max(end - start, default_step)))
            start = None
    return segments


def _generate_manual_timeline(dataset, output_dirs: Sequence[str | Path] | None) -> FigureResult:
    records = dataset.records
    t_s = [record.t_s for record in records]
    fig, ax = plt.subplots(figsize=(8.4, 4.2))
    ax.plot(t_s, [record.throttle_norm for record in records], color=COLOR_BLUE, label="RC throttle")
    ax.plot(t_s, [record.steer_norm for record in records], color=COLOR_ORANGE, label="RC steer")
    ax.plot(t_s, [record.left_cmd for record in records], color=COLOR_GREEN, label="Left motor cmd")
    ax.plot(t_s, [record.right_cmd for record in records], color=COLOR_NAVY, label="Right motor cmd")
    ax.axhline(0.0, color=COLOR_GRAY, linewidth=0.8)
    ax.set_ylim(-1.08, 1.08)
    ax.set_title("Manual Control Timeline")
    ax.set_xlabel("Elapsed sample time (s)")
    ax.set_ylabel("Normalized command")
    ax.legend(loc="upper right", ncols=2)
    add_panel_label(ax, "MOCK/DEMO" if dataset.is_mock else "REAL LOG", color=COLOR_GRAY)
    add_source_note(ax, dataset.source_label, mock=dataset.is_mock)
    finalize_axes(ax)
    fig.tight_layout()
    save_figure_all(fig, "fig_manual_control_timeline.png", output_dirs)
    return FigureResult(
        filename="fig_manual_control_timeline.png",
        script=SCRIPT_NAME,
        data_source=dataset.data_source,
        recommended_use="Safety validation section: show RC/manual input and resulting normalized motor commands.",
        caption=(
            "Manual-control timeline from the selected USB debug log. The figure summarizes logged "
            "commands only and should be discussed with the wheel-off-ground bench-test constraint."
        ),
    )


def _generate_failsafe_timeline(dataset, output_dirs: Sequence[str | Path] | None) -> FigureResult:
    records = dataset.records
    t_s = [record.t_s for record in records]
    lanes = [
        ("rc_ok=false", [not record.rc_ok for record in records], COLOR_RED),
        ("mode=FAILSAFE", [record.mode == "FAILSAFE" for record in records], COLOR_ORANGE),
        ("control_source=STOP", [record.control_source == "STOP" for record in records], COLOR_BLUE),
        (
            "motor_cmd_zero",
            [abs(record.left_cmd) <= 0.001 and abs(record.right_cmd) <= 0.001 for record in records],
            COLOR_GREEN,
        ),
    ]
    fig, ax = plt.subplots(figsize=(8.4, 4.1))
    for y_index, (_, flags, color) in enumerate(lanes):
        ax.broken_barh(
            _boolean_segments(t_s, flags),
            (y_index - 0.34, 0.68),
            facecolors=color,
            edgecolors="none",
            alpha=0.86,
        )
    ax.set_yticks(range(len(lanes)))
    ax.set_yticklabels([name for name, _, _ in lanes])
    ax.set_ylim(-0.65, len(lanes) - 0.35)
    ax.set_title("Failsafe Event Timeline")
    ax.set_xlabel("Elapsed sample time (s)")
    ax.set_ylabel("Logged condition")
    add_panel_label(ax, "MOCK/DEMO" if dataset.is_mock else "REAL LOG", color=COLOR_GRAY)
    add_source_note(ax, dataset.source_label, mock=dataset.is_mock)
    finalize_axes(ax)
    fig.tight_layout()
    save_figure_all(fig, "fig_failsafe_event_timeline.png", output_dirs)
    return FigureResult(
        filename="fig_failsafe_event_timeline.png",
        script=SCRIPT_NAME,
        data_source=dataset.data_source,
        recommended_use="Failsafe validation section: show when invalid RC/failsafe/STOP conditions appear in the log.",
        caption=(
            "Failsafe-related conditions from the selected USB debug log. The bars show logged "
            "state conditions and zero-command intervals; they do not claim full autonomous safety validation."
        ),
    )


def _generate_control_source_transition(dataset, output_dirs: Sequence[str | Path] | None) -> FigureResult:
    records = dataset.records
    t_s = [record.t_s for record in records]
    ordered_sources: list[str] = []
    for record in records:
        if record.control_source not in ordered_sources:
            ordered_sources.append(record.control_source)
    source_index = {source: index for index, source in enumerate(ordered_sources)}
    y_values = [source_index[record.control_source] for record in records]

    fig, ax = plt.subplots(figsize=(8.4, 3.9))
    ax.step(t_s, y_values, where="post", color=COLOR_NAVY, linewidth=2.1)
    ax.scatter(t_s, y_values, color=COLOR_NAVY, s=12, alpha=0.65)
    ax.set_yticks(range(len(ordered_sources)))
    ax.set_yticklabels(ordered_sources)
    ax.set_title("Control Source Transition")
    ax.set_xlabel("Elapsed sample time (s)")
    ax.set_ylabel("Control source")
    add_panel_label(ax, "MOCK/DEMO" if dataset.is_mock else "REAL LOG", color=COLOR_GRAY)
    add_source_note(ax, dataset.source_label, mock=dataset.is_mock)
    finalize_axes(ax)
    fig.tight_layout()
    save_figure_all(fig, "fig_control_source_transition.png", output_dirs)
    return FigureResult(
        filename="fig_control_source_transition.png",
        script=SCRIPT_NAME,
        data_source=dataset.data_source,
        recommended_use="Control and safety section: document transitions among STOP, RC manual, and other logged sources.",
        caption=(
            "Logged control-source transitions over the selected USB debug capture. This figure "
            "documents source selection in the log and should not be described as completed ROS2 autonomy."
        ),
    )


def generate(
    *,
    output_dirs: Sequence[str | Path] | None = None,
    safety_paths: Sequence[Path] | None = None,
) -> list[FigureResult]:
    manual_dataset = _choose_manual_dataset(safety_paths)
    failsafe_dataset = _choose_failsafe_dataset(safety_paths)
    transition_dataset = _choose_transition_dataset(safety_paths)
    return [
        _generate_manual_timeline(manual_dataset, output_dirs),
        _generate_failsafe_timeline(failsafe_dataset, output_dirs),
        _generate_control_source_transition(transition_dataset, output_dirs),
    ]


def main() -> int:
    args = build_parser().parse_args()
    results = generate(output_dirs=args.output_dirs, safety_paths=args.safety_log)
    for result in results:
        print(f"generated {result.filename} from {result.data_source}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
