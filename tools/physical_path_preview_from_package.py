from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Sequence

try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401

from tools.inspect_path_package import ALLOWED_PRIMITIVES, inspect_package
from tools.path_no_motion_validation import PathPackageResolutionError, resolve_path_package


CHECKED_PRIMITIVE_FIELDS = (
    "primitive_index",
    "primitive_type",
    "distance_m",
    "angle_deg",
    "segment_role",
    "start_x_m",
    "start_y_m",
    "start_heading_deg",
    "end_x_m",
    "end_y_m",
    "end_heading_deg",
    "associated_tool_segment_id",
    "tool_active",
    "coverage_contributes",
    "primitive_allowed",
    "motor_command_generated",
)


def _write_csv(path: Path, rows: Sequence[dict[str, object]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CHECKED_PRIMITIVE_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in CHECKED_PRIMITIVE_FIELDS})


def _plot_previews(out_dir: Path, package: dict[str, object]) -> dict[str, Path | None]:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle
    except ImportError:
        return {
            "preview_tool_path.png": None,
            "preview_chassis_path.png": None,
            "preview_primitive_sequence.png": None,
            "preview_approach_then_serpentine.png": None,
        }

    workspace = package["normalized_workspace"]  # type: ignore[index]
    tool_path = package["tool_path"]  # type: ignore[index]
    chassis_path = package.get("chassis_path", [])
    primitives = package["primitive_sequence"]  # type: ignore[index]
    x_min = float(workspace["x_min_m"])  # type: ignore[index]
    x_max = float(workspace["x_max_m"])  # type: ignore[index]
    y_min = float(workspace["y_min_m"])  # type: ignore[index]
    y_max = float(workspace["y_max_m"])  # type: ignore[index]

    def setup(title: str):
        fig, ax = plt.subplots(figsize=(9, 5))
        ax.add_patch(Rectangle((x_min, y_min), x_max - x_min, y_max - y_min, fill=False, edgecolor="black"))
        ax.set_title(title)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.axis("equal")
        ax.grid(True, linestyle="--", alpha=0.3)
        ax.text(0.01, 0.01, "Stage 11 preview only: no motor commands", transform=ax.transAxes, fontsize=8)
        return fig, ax

    outputs: dict[str, Path | None] = {}
    fig, ax = setup("Tool Path From Path Package")
    for row in tool_path:
        active = bool(row["tool_active"])
        ax.plot(
            [float(row["tool_start_x_m"]), float(row["tool_end_x_m"])],
            [float(row["tool_start_y_m"]), float(row["tool_end_y_m"])],
            color="tab:orange" if active else "0.55",
            linestyle="-" if active else "--",
            linewidth=2.4 if active else 1.2,
        )
    path = out_dir / "preview_tool_path.png"
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    outputs[path.name] = path

    fig, ax = setup("Derived Chassis Path From Package")
    for row in chassis_path:  # type: ignore[assignment]
        ax.plot(
            [float(row["chassis_start_x_m"]), float(row["chassis_end_x_m"])],
            [float(row["chassis_start_y_m"]), float(row["chassis_end_y_m"])],
            color="tab:blue",
            linewidth=1.5,
        )
    for row in tool_path:
        ax.plot(
            [float(row["tool_start_x_m"]), float(row["tool_end_x_m"])],
            [float(row["tool_start_y_m"]), float(row["tool_end_y_m"])],
            color="tab:orange",
            alpha=0.25,
            linewidth=1.0,
        )
    path = out_dir / "preview_chassis_path.png"
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    outputs[path.name] = path

    fig, ax = setup("Primitive Sequence From Package")
    for row in primitives:
        sx = float(row["start_x_m"])
        sy = float(row["start_y_m"])
        ex = float(row["end_x_m"])
        ey = float(row["end_y_m"])
        ptype = str(row["primitive_type"])
        if ptype.startswith("move"):
            ax.annotate("", xy=(ex, ey), xytext=(sx, sy), arrowprops={"arrowstyle": "->", "color": "tab:green"})
        else:
            ax.scatter([sx], [sy], color="tab:purple", s=22)
        ax.text(sx, sy, f"P{int(row['primitive_index']):03d}", fontsize=6)
    path = out_dir / "preview_primitive_sequence.png"
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    outputs[path.name] = path

    fig, ax = setup("Approach Then Serpentine From Package")
    for row in primitives:
        if not str(row["primitive_type"]).startswith("move"):
            continue
        role = str(row["segment_role"])
        color = "tab:green" if role.startswith("approach") else "tab:blue"
        ax.plot([float(row["start_x_m"]), float(row["end_x_m"])], [float(row["start_y_m"]), float(row["end_y_m"])], color=color)
    path = out_dir / "preview_approach_then_serpentine.png"
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    outputs[path.name] = path
    return outputs


def _write_summary(path: Path, inspection: dict[str, object], preview_paths: dict[str, Path | None]) -> None:
    validation = inspection["validation"]  # type: ignore[index]
    lines = [
        "# Stage 11 Physical Path Preview From Package",
        "",
        "Software-side no-motion preview only. This does not use GPS course, serial, HC-12, or motor output.",
        "",
        f"- selected_path_package: `{inspection['selected_path_package']}`",
        f"- tool_path_starts_at_A_prime: `{validation['tool_path_starts_at_A_prime']}`",
        f"- tool_path_ends_at_B_prime: `{validation['tool_path_ends_at_B_prime']}`",
        f"- tool_path_continuous: `{validation['tool_path_continuous']}`",
        f"- connectors_inactive: `{validation['connectors_inactive']}`",
        f"- sweep_tracks_active: `{validation['sweep_tracks_active']}`",
        f"- primitive_sequence_allowed: `{validation['primitive_sequence_allowed']}`",
        f"- motor_command_generated: `False`",
        f"- path_preview_only: `True`",
        "",
        "## Outputs",
    ]
    for name, output in preview_paths.items():
        lines.append(f"- {name}: `{output if output is not None else 'not generated'}`")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Generate Stage 11 physical path preview from path_package.json.")
    parser.add_argument("--path-package", default="latest")
    parser.add_argument("--out-dir", default="outputs/stage11_physical_preview/latest")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        selected = resolve_path_package(args.path_package)
    except PathPackageResolutionError as exc:
        print(f"provided_path_package={exc.provided}")
        print("file_exists=false")
        print("nearest_candidates:")
        for candidate in exc.candidates:
            print(f"- {candidate}")
        print("next_action=Run tools/field_ab_to_serpentine.py first.")
        return 2
    package = json.loads(selected.read_text(encoding="utf-8"))
    inspection = inspect_package(package, selected)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    checked_rows = []
    for row in package["primitive_sequence"]:  # type: ignore[index]
        updated = dict(row)
        updated["primitive_allowed"] = row["primitive_type"] in ALLOWED_PRIMITIVES
        updated["motor_command_generated"] = False
        checked_rows.append(updated)
    csv_path = out_dir / "primitive_sequence_checked.csv"
    _write_csv(csv_path, checked_rows)
    preview_paths = _plot_previews(out_dir, package)
    summary_path = out_dir / "summary.md"
    _write_summary(summary_path, inspection, preview_paths)
    print("Stage 11 physical path preview generated.")
    print(f"selected_path_package={selected}")
    print(f"summary_md={summary_path}")
    print(f"primitive_sequence_checked_csv={csv_path}")
    for name, output in preview_paths.items():
        print(f"{name}={output if output is not None else 'not_generated'}")
    print("gps_course_required=false")
    print("serial_opened=false")
    print("motor_command_generated=false")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
