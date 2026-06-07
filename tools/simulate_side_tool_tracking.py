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


CSV_FIELDS = (
    "segment_index",
    "segment_type",
    "lane_index",
    "x_m",
    "y_m",
    "heading_deg",
    "motion_direction",
    "travel_direction_deg",
    "target_x_m",
    "target_y_m",
    "target_bearing_deg",
    "heading_error_deg",
    "cross_track_error_m",
    "along_track_progress_m",
    "virtual_desired_forward_cmd",
    "virtual_desired_turn_cmd",
    "feedback_source",
    "motor_command_generated",
)


def _normalize_deg(angle_deg: float) -> float:
    return ((angle_deg + 180.0) % 360.0) - 180.0


def _float(row: dict[str, str], key: str, default: float = 0.0) -> float:
    value = row.get(key, "")
    if value in {"", "NA", "None"}:
        return default
    return float(value)


def build_tracking_rows(
    preview_rows: Sequence[dict[str, str]],
) -> list[dict[str, str | int | float | bool]]:
    rows: list[dict[str, str | int | float | bool]] = []
    cumulative = 0.0
    for index, row in enumerate(preview_rows):
        next_row = preview_rows[index + 1] if index + 1 < len(preview_rows) else None
        x = _float(row, "x_m")
        y = _float(row, "y_m")
        heading = _float(row, "heading_deg")
        if next_row is None:
            target_x = "NA"
            target_y = "NA"
            target_bearing = "NA"
            heading_error = "NA"
            step_distance = 0.0
        else:
            nx = _float(next_row, "x_m")
            ny = _float(next_row, "y_m")
            dx = nx - x
            dy = ny - y
            step_distance = math.hypot(dx, dy)
            target_x = f"{nx:.3f}"
            target_y = f"{ny:.3f}"
            if step_distance > 0.0:
                bearing = math.degrees(math.atan2(dy, dx))
                target_bearing = f"{bearing:.3f}"
                heading_error = f"{_normalize_deg(bearing - heading):.3f}"
            else:
                target_bearing = "NA"
                heading_error = "NA"
        cumulative += step_distance
        motion = row.get("motion_direction", "forward")
        virtual_forward = 0.10 if motion == "forward" else -0.10
        rows.append(
            {
                "segment_index": row.get("segment_index", row.get("index", index)),
                "segment_type": row.get("segment_type", "unknown"),
                "lane_index": row.get("lane_index", -1),
                "x_m": f"{x:.3f}",
                "y_m": f"{y:.3f}",
                "heading_deg": f"{heading:.3f}",
                "motion_direction": motion,
                "travel_direction_deg": row.get("travel_direction_deg", "NA"),
                "target_x_m": target_x,
                "target_y_m": target_y,
                "target_bearing_deg": target_bearing,
                "heading_error_deg": heading_error,
                "cross_track_error_m": "0.000",
                "along_track_progress_m": f"{cumulative:.3f}",
                "virtual_desired_forward_cmd": f"{virtual_forward:.3f}",
                "virtual_desired_turn_cmd": "0.000",
                "feedback_source": "offline_preview_geometry",
                "motor_command_generated": False,
            }
        )
    return rows


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Simulate side-tool path tracking from an offline preview CSV. "
            "This produces virtual diagnostics only and never generates motor commands."
        )
    )
    parser.add_argument("--path-csv", required=True, help="side_tool_path.csv from side_tool_path_preview.py")
    parser.add_argument("--out-dir", default="outputs/side_tool_tracking_sim")
    return parser


def _read_preview(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def _write_csv(path: Path, rows: Sequence[dict[str, str | int | float | bool]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row[field] for field in CSV_FIELDS})


def _write_summary(path: Path, rows: Sequence[dict[str, str | int | float | bool]], csv_path: Path) -> None:
    lines = [
        "# Side-Tool Tracking Simulation",
        "",
        "This is an offline no-motion simulation artifact.",
        "",
        "- No firmware is uploaded.",
        "- No serial ports are opened.",
        "- No HC-12 frames are sent.",
        "- No rover motor commands are generated.",
        "- `virtual_desired_forward_cmd` and `virtual_desired_turn_cmd` are diagnostic values only.",
        "",
        "## Output",
        "",
        f"- tracking_rows: `{len(rows)}`",
        "- motor_command_generated: `False`",
        "- feedback_tracking_ready: `simulation_only`",
        f"- generated_at_utc: `{dt.datetime.now(tz=dt.UTC).isoformat()}`",
        f"- tracking_errors_csv: `{csv_path}`",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    preview_path = Path(args.path_csv)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    rows = build_tracking_rows(_read_preview(preview_path))
    csv_path = out_dir / "tracking_errors.csv"
    summary_path = out_dir / "summary.md"
    _write_csv(csv_path, rows)
    _write_summary(summary_path, rows, csv_path)
    print("Side-tool tracking simulation generated.")
    print("Simulation only: no rover commands, no motor output, no firmware upload.")
    print(f"CSV: {csv_path}")
    print(f"Summary: {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
