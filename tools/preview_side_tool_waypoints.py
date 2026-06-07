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

from gps_coverage_core.side_tool_planner import SideToolPlanConfig, generate_side_tool_path


CSV_FIELDS = (
    "index",
    "segment_type",
    "lane_index",
    "direction",
    "x_m",
    "y_m",
    "target_x_m",
    "target_y_m",
    "target_bearing_deg",
    "expected_rover_heading_deg",
    "reverse_direction_expected",
    "transition_style",
    "tool_side",
    "tool_x_m",
    "tool_y_m",
    "motor_command_generated",
    "notes",
)


def _normalize_deg(angle_deg: float) -> float:
    return ((angle_deg + 180.0) % 360.0) - 180.0


def _target_bearing_deg(
    current_pose: dict[str, float | int | str | bool],
    next_pose: dict[str, float | int | str | bool] | None,
) -> float | None:
    if next_pose is None:
        return None
    dx = float(next_pose["x_m"]) - float(current_pose["x_m"])
    dy = float(next_pose["y_m"]) - float(current_pose["y_m"])
    if math.hypot(dx, dy) == 0.0:
        return None
    # Planner-frame bearing: 0 deg points along +x, 90 deg points along +y.
    return _normalize_deg(math.degrees(math.atan2(dy, dx)))


def build_waypoint_rows(
    poses: Sequence[dict[str, float | int | str | bool]],
) -> list[dict[str, float | int | str | bool]]:
    rows: list[dict[str, float | int | str | bool]] = []
    for index, pose in enumerate(poses):
        next_pose = poses[index + 1] if index + 1 < len(poses) else None
        target_bearing = _target_bearing_deg(pose, next_pose)
        rows.append(
            {
                "index": pose["index"],
                "segment_type": pose["segment_type"],
                "lane_index": pose["lane_index"],
                "direction": pose["direction"],
                "x_m": pose["x_m"],
                "y_m": pose["y_m"],
                "target_x_m": next_pose["x_m"] if next_pose is not None else "NA",
                "target_y_m": next_pose["y_m"] if next_pose is not None else "NA",
                "target_bearing_deg": f"{target_bearing:.3f}" if target_bearing is not None else "NA",
                "expected_rover_heading_deg": pose["heading_deg"],
                "reverse_direction_expected": pose["direction"] == "reverse",
                "transition_style": pose["transition_style"],
                "tool_side": pose["tool_side"],
                "tool_x_m": pose["tool_x_m"],
                "tool_y_m": pose["tool_y_m"],
                "motor_command_generated": False,
                "notes": pose["notes"],
            }
        )
    return rows


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Export offline side-tool preview poses as target waypoint diagnostics. "
            "This never uploads firmware, sends HC-12 frames, or generates motor commands."
        )
    )
    parser.add_argument("--tool-side", choices=("left", "right"), required=True)
    parser.add_argument("--tool-lateral-offset-m", type=float, required=True)
    parser.add_argument("--tool-width-m", type=float, required=True)
    parser.add_argument("--lane-spacing-m", type=float, required=True)
    parser.add_argument("--row-length-m", type=float, required=True)
    parser.add_argument("--row-count", type=int, required=True)
    parser.add_argument("--start-x-m", type=float, default=0.0)
    parser.add_argument("--start-y-m", type=float, default=0.0)
    parser.add_argument("--start-heading-deg", type=float, default=0.0)
    parser.add_argument("--first-lane-direction", choices=("forward", "reverse"), default="forward")
    parser.add_argument("--transition-style", choices=("side-step-reverse-90",), default="side-step-reverse-90")
    parser.add_argument(
        "--out-dir",
        default="outputs/side_tool_waypoint_preview",
        help="Output directory for side_tool_waypoints.csv and waypoint_summary.md",
    )
    return parser


def _write_csv(path: Path, rows: Sequence[dict[str, float | int | str | bool]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row[field] for field in CSV_FIELDS})


def _write_summary(
    *,
    path: Path,
    config: SideToolPlanConfig,
    rows: Sequence[dict[str, float | int | str | bool]],
    csv_path: Path,
) -> None:
    reverse_count = sum(1 for row in rows if row["reverse_direction_expected"] is True)
    lines = [
        "# Side-Tool Waypoint Preview",
        "",
        "This is an offline no-motion diagnostic artifact.",
        "",
        "- No HC-12 frames are sent.",
        "- No firmware is uploaded or modified.",
        "- No rover motor commands are generated.",
        "- These waypoints are not approved for physical path following.",
        "",
        "## Inputs",
        "",
        f"- tool_side: `{config.tool_side}`",
        f"- tool_lateral_offset_m: `{config.tool_lateral_offset_m:.3f}`",
        f"- tool_width_m: `{config.tool_width_m:.3f}`",
        f"- lane_spacing_m: `{config.lane_spacing_m:.3f}`",
        f"- row_length_m: `{config.row_length_m:.3f}`",
        f"- row_count: `{config.row_count}`",
        f"- first_lane_direction: `{config.first_lane_direction}`",
        f"- transition_style: `{config.transition_style}`",
        "",
        "## Output Semantics",
        "",
        "- `target_bearing_deg` is a planner-frame bearing to the next preview pose.",
        "- `expected_rover_heading_deg` is the geometric pose heading for diagnostics.",
        "- `reverse_direction_expected=true` marks reverse lanes and reverse offset transitions.",
        "- `motor_command_generated` is always `False`.",
        "",
        "## Outputs",
        "",
        f"- waypoint_rows: `{len(rows)}`",
        f"- reverse_expected_rows: `{reverse_count}`",
        f"- generated_at_utc: `{dt.datetime.now(tz=dt.UTC).isoformat()}`",
        f"- csv: `{csv_path}`",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    config = SideToolPlanConfig(
        workspace_mode="axis_width",
        tool_side=args.tool_side,
        tool_lateral_offset_m=args.tool_lateral_offset_m,
        tool_width_m=args.tool_width_m,
        lane_spacing_m=args.lane_spacing_m,
        row_length_m=args.row_length_m,
        row_count=args.row_count,
        start_x_m=args.start_x_m,
        start_y_m=args.start_y_m,
        start_heading_deg=args.start_heading_deg,
        first_lane_direction=args.first_lane_direction,
        transition_style=args.transition_style,
        contamination_mode="off",
        fail_on_contamination_violation=False,
    )
    rows = build_waypoint_rows(generate_side_tool_path(config))
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / "side_tool_waypoints.csv"
    summary_path = out_dir / "waypoint_summary.md"
    _write_csv(csv_path, rows)
    _write_summary(path=summary_path, config=config, rows=rows, csv_path=csv_path)

    print("Side-tool waypoint preview generated.")
    print("Preview only: no HC-12 commands, no rover motor commands, no firmware upload.")
    print(f"CSV: {csv_path}")
    print(f"Summary: {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
