from __future__ import annotations

import argparse
from pathlib import Path

from _figure_common import add_output_args, write_figure_captions
from generate_gps_figures import generate as generate_gps_figures
from generate_manual_control_figures import generate as generate_manual_control_figures
from generate_path_figures import generate as generate_path_figures
from generate_system_figures import generate as generate_system_figures


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Generate all report-quality project figures.")
    parser.add_argument(
        "--waypoints",
        nargs="*",
        type=Path,
        default=None,
        help="Optional waypoint CSV/JSON files for path figures.",
    )
    parser.add_argument(
        "--gps-log",
        nargs="*",
        type=Path,
        default=None,
        help="Optional GPS CSV/log files for GPS figures.",
    )
    parser.add_argument(
        "--safety-log",
        nargs="*",
        type=Path,
        default=None,
        help="Optional USBDBG safety log files for manual/failsafe figures.",
    )
    parser.add_argument(
        "--no-captions",
        action="store_true",
        help="Generate figures without rewriting docs/figures/generated/figure_captions.md.",
    )
    add_output_args(parser)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    output_dirs = args.output_dirs

    results = []
    results.extend(generate_system_figures(output_dirs=output_dirs))
    results.extend(generate_path_figures(output_dirs=output_dirs, waypoint_paths=args.waypoints))
    results.extend(generate_gps_figures(output_dirs=output_dirs, gps_paths=args.gps_log))
    results.extend(
        generate_manual_control_figures(output_dirs=output_dirs, safety_paths=args.safety_log)
    )

    if not args.no_captions:
        caption_path = write_figure_captions(results)
        print(f"updated {caption_path.relative_to(caption_path.parents[3])}")

    for result in results:
        print(f"generated {result.filename} from {result.data_source}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
