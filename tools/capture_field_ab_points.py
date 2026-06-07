from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
from pathlib import Path
from typing import Sequence

try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401


CSV_FIELDS = ("point_label", "x_m", "y_m", "sample_count")


def _point_from_args_or_prompt(
    *,
    label: str,
    x_value: float | None,
    y_value: float | None,
) -> tuple[float, float]:
    if x_value is not None and y_value is not None:
        return x_value, y_value
    raw = input(f"Enter point {label} as 'x,y' in local meters: ").strip()
    parts = [part.strip() for part in raw.split(",")]
    if len(parts) != 2:
        raise ValueError(f"point {label} must be entered as x,y")
    return float(parts[0]), float(parts[1])


def build_field_points(
    *,
    a_x: float,
    a_y: float,
    b_x: float,
    b_y: float,
) -> dict[str, object]:
    return {
        "capture_mode": "manual_local_meters",
        "generated_at_utc": dt.datetime.now(tz=dt.UTC).isoformat(),
        "points": {
            "A": {"x_m": a_x, "y_m": a_y, "sample_count": 1},
            "B": {"x_m": b_x, "y_m": b_y, "sample_count": 1},
        },
        "path_preview_only": True,
        "motor_command_generated": False,
    }


def write_field_points(out_dir: Path, data: dict[str, object]) -> tuple[Path, Path]:
    out_dir.mkdir(parents=True, exist_ok=True)
    json_path = out_dir / "field_points.json"
    csv_path = out_dir / "field_points.csv"
    json_path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    points = data["points"]  # type: ignore[index]
    with csv_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for label in ("A", "B"):
            point = points[label]  # type: ignore[index]
            writer.writerow(
                {
                    "point_label": label,
                    "x_m": point["x_m"],
                    "y_m": point["y_m"],
                    "sample_count": point["sample_count"],
                }
            )
    return json_path, csv_path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Capture field A/B points for offline path planning. Manual local-meter "
            "input is supported now; no serial ports are opened and no motor commands "
            "are generated."
        )
    )
    parser.add_argument("--a-x", type=float)
    parser.add_argument("--a-y", type=float)
    parser.add_argument("--b-x", type=float)
    parser.add_argument("--b-y", type=float)
    parser.add_argument(
        "--out-dir",
        default="outputs/field_ab_capture/latest",
        help="Directory for field_points.json and field_points.csv",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    a_x, a_y = _point_from_args_or_prompt(label="A", x_value=args.a_x, y_value=args.a_y)
    b_x, b_y = _point_from_args_or_prompt(label="B", x_value=args.b_x, y_value=args.b_y)
    data = build_field_points(a_x=a_x, a_y=a_y, b_x=b_x, b_y=b_y)
    json_path, csv_path = write_field_points(Path(args.out_dir), data)
    print("Field A/B points captured.")
    print("Preview only: no serial monitor, no rover commands, no motor output.")
    print(f"field_points_json: {json_path}")
    print(f"field_points_csv: {csv_path}")
    print("motor_command_generated: False")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
