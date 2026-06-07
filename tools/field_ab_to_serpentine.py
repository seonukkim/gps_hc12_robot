from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import math
from pathlib import Path
from typing import Sequence

try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401

from gps_coverage_core.side_tool_planner import SideToolPlanConfig, generate_tool_serpentine_preview


ALLOWED_PRIMITIVES = {"move_forward", "move_backward", "rotate_left", "rotate_right"}

TOOL_PATH_FIELDS = (
    "tool_segment_index",
    "tool_segment_id",
    "tool_segment_type",
    "tool_start_x_m",
    "tool_start_y_m",
    "tool_end_x_m",
    "tool_end_y_m",
    "tool_active",
    "coverage_contributes",
    "motor_command_generated",
)

PRIMITIVE_FIELDS = (
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
    "motor_command_generated",
)

METERS_PER_DEG_LAT = 111_320.0


def _normalize_deg(angle_deg: float) -> float:
    return ((angle_deg + 180.0) % 360.0) - 180.0


def normalize_field_ab(
    *,
    a_x: float,
    a_y: float,
    b_x: float,
    b_y: float,
) -> dict[str, object]:
    x_min = min(a_x, b_x)
    x_max = max(a_x, b_x)
    y_min = min(a_y, b_y)
    y_max = max(a_y, b_y)
    if math.isclose(x_min, x_max, abs_tol=1e-9):
        raise ValueError("FIELD_AB_LENGTH_ZERO: captured A/B must span nonzero x")
    if math.isclose(y_min, y_max, abs_tol=1e-9):
        raise ValueError("FIELD_AB_WIDTH_ZERO: captured A/B must span nonzero y")
    return {
        "raw_A": {"x_m": a_x, "y_m": a_y},
        "raw_B": {"x_m": b_x, "y_m": b_y},
        "x_min_m": x_min,
        "x_max_m": x_max,
        "y_min_m": y_min,
        "y_max_m": y_max,
        "width_m": x_max - x_min,
        "height_m": y_max - y_min,
        "A_prime_top_left": {"x_m": x_min, "y_m": y_max},
        "B_prime_bottom_right": {"x_m": x_max, "y_m": y_min},
    }


def _load_field_points(path: Path) -> tuple[float, float, float, float]:
    data = json.loads(path.read_text(encoding="utf-8"))
    points = data["points"]
    return (
        float(points["A"]["x_m"]),
        float(points["A"]["y_m"]),
        float(points["B"]["x_m"]),
        float(points["B"]["y_m"]),
    )


def _meters_per_deg_lon(latitude_deg: float) -> float:
    return METERS_PER_DEG_LAT * math.cos(math.radians(latitude_deg))


def _load_georef_points(path: Path) -> tuple[tuple[float, float, float, float], dict[str, object]]:
    data = json.loads(path.read_text(encoding="utf-8"))
    points = data["points"]
    a_lat = float(points["A"]["lat"])
    a_lon = float(points["A"]["lon"])
    b_lat = float(points["B"]["lat"])
    b_lon = float(points["B"]["lon"])
    mean_lat = (a_lat + b_lat) / 2.0
    meters_per_deg_lon = _meters_per_deg_lon(mean_lat)
    raw_a = (0.0, 0.0)
    raw_b = (
        (b_lon - a_lon) * meters_per_deg_lon,
        (b_lat - a_lat) * METERS_PER_DEG_LAT,
    )
    georef = {
        "georeference_available": True,
        "raw_A_lat": a_lat,
        "raw_A_lon": a_lon,
        "raw_B_lat": b_lat,
        "raw_B_lon": b_lon,
        "origin_lat": a_lat,
        "origin_lon": a_lon,
        "local_frame_type": "equirectangular_enu",
        "x_axis_source": "normalized_rectangle",
        "x_axis_bearing_deg": 90.0,
        "meters_per_deg_lat": METERS_PER_DEG_LAT,
        "meters_per_deg_lon": meters_per_deg_lon,
        "meters_per_lat": METERS_PER_DEG_LAT,
        "meters_per_lon": meters_per_deg_lon,
        "raw_A_local": {"x_m": raw_a[0], "y_m": raw_a[1]},
        "raw_B_local": {"x_m": raw_b[0], "y_m": raw_b[1]},
        "motor_command_generated": False,
    }
    return (raw_a[0], raw_a[1], raw_b[0], raw_b[1]), georef


def _rotation_row(
    *,
    index: int,
    start_pose: tuple[float, float, float],
    target_heading_deg: float,
    associated_tool_segment_id: str,
    segment_role: str,
) -> dict[str, object] | None:
    delta = _normalize_deg(target_heading_deg - start_pose[2])
    if abs(delta) <= 1e-9:
        return None
    primitive_type = "rotate_left" if delta > 0.0 else "rotate_right"
    return {
        "primitive_index": index,
        "primitive_type": primitive_type,
        "distance_m": 0.0,
        "angle_deg": abs(delta),
        "segment_role": segment_role,
        "start_x_m": start_pose[0],
        "start_y_m": start_pose[1],
        "start_heading_deg": start_pose[2],
        "end_x_m": start_pose[0],
        "end_y_m": start_pose[1],
        "end_heading_deg": target_heading_deg,
        "associated_tool_segment_id": associated_tool_segment_id,
        "tool_active": False,
        "coverage_contributes": False,
        "motor_command_generated": False,
    }


def build_approach_primitives(
    *,
    current_x: float,
    current_y: float,
    current_heading_deg: float,
    target_x: float,
    target_y: float,
) -> list[dict[str, object]]:
    dx = target_x - current_x
    dy = target_y - current_y
    distance = math.hypot(dx, dy)
    if distance <= 1e-9:
        return []
    bearing = math.degrees(math.atan2(dy, dx))
    rows: list[dict[str, object]] = []
    start_pose = (current_x, current_y, current_heading_deg)
    rotation = _rotation_row(
        index=0,
        start_pose=start_pose,
        target_heading_deg=bearing,
        associated_tool_segment_id="approach_to_A_prime",
        segment_role="approach_rotation",
    )
    if rotation is not None:
        rows.append(rotation)
        start_pose = (current_x, current_y, bearing)
    rows.append(
        {
            "primitive_index": len(rows),
            "primitive_type": "move_forward",
            "distance_m": distance,
            "angle_deg": 0.0,
            "segment_role": "approach_move",
            "start_x_m": current_x,
            "start_y_m": current_y,
            "start_heading_deg": start_pose[2],
            "end_x_m": target_x,
            "end_y_m": target_y,
            "end_heading_deg": start_pose[2],
            "associated_tool_segment_id": "approach_to_A_prime",
            "tool_active": False,
            "coverage_contributes": False,
            "motor_command_generated": False,
        }
    )
    return rows


def _offset_primitives(rows: list[dict[str, object]], offset: int) -> list[dict[str, object]]:
    shifted: list[dict[str, object]] = []
    for row in rows:
        updated = dict(row)
        updated["primitive_index"] = int(updated["primitive_index"]) + offset
        shifted.append(updated)
    return shifted


def _write_csv(path: Path, rows: Sequence[dict[str, object]], fields: Sequence[str]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in fields})


def _write_json(path: Path, data: dict[str, object]) -> None:
    path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")


def _write_summary(path: Path, package: dict[str, object]) -> None:
    summary = package["summary"]  # type: ignore[index]
    lines = [
        "# Field A/B To Serpentine Path Package",
        "",
        "Preview/no-motion only: no rover motor commands are generated.",
        "",
    ]
    for key in (
        "raw_A",
        "raw_B",
        "A_prime_top_left",
        "B_prime_bottom_right",
        "approach_path_generated",
        "serpentine_path_generated",
        "tool_side",
        "step_spacing_m",
        "primitive_sequence_valid",
        "motor_command_generated",
        "physical_output_active",
        "ready_for_outdoor_no_motion_validation",
    ):
        lines.append(f"- {key}: `{summary[key]}`")
    if package.get("georeference"):
        georef = package["georeference"]  # type: ignore[index]
        lines.append(f"- georeference_available: `{georef['georeference_available']}`")
        lines.append(f"- origin_lat: `{georef['origin_lat']}`")
        lines.append(f"- origin_lon: `{georef['origin_lon']}`")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_previews(out_dir: Path, package: dict[str, object]) -> dict[str, Path | None]:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle
    except ImportError:
        return {
            "preview_workspace_ab_aprime_bprime.png": None,
            "preview_tool_path_primary.png": None,
            "preview_primitive_sequence.png": None,
            "preview_approach_then_serpentine.png": None,
        }

    workspace = package["normalized_workspace"]  # type: ignore[index]
    tool_rows = package["tool_path"]  # type: ignore[index]
    primitives = package["primitive_sequence"]  # type: ignore[index]
    x_min = float(workspace["x_min_m"])  # type: ignore[index]
    x_max = float(workspace["x_max_m"])  # type: ignore[index]
    y_min = float(workspace["y_min_m"])  # type: ignore[index]
    y_max = float(workspace["y_max_m"])  # type: ignore[index]
    origin_x = x_min
    origin_y = y_min

    def setup(title: str):
        fig, ax = plt.subplots(figsize=(8, 5))
        ax.add_patch(Rectangle((x_min, y_min), x_max - x_min, y_max - y_min, fill=False, edgecolor="black"))
        ax.set_title(title)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.axis("equal")
        ax.grid(True, linestyle="--", alpha=0.3)
        ax.text(0.01, 0.01, "Preview only: no rover commands, no motor output", transform=ax.transAxes, fontsize=8)
        return fig, ax

    outputs: dict[str, Path | None] = {}
    fig, ax = setup("Workspace Raw A/B And Normalized A'/B'")
    raw_a = workspace["raw_A"]  # type: ignore[index]
    raw_b = workspace["raw_B"]  # type: ignore[index]
    a_prime = workspace["A_prime_top_left"]  # type: ignore[index]
    b_prime = workspace["B_prime_bottom_right"]  # type: ignore[index]
    ax.scatter([raw_a["x_m"], raw_b["x_m"]], [raw_a["y_m"], raw_b["y_m"]], color="tab:blue", label="raw A/B")
    ax.scatter([a_prime["x_m"], b_prime["x_m"]], [a_prime["y_m"], b_prime["y_m"]], color="tab:orange", label="A'/B'")
    ax.annotate("A raw", (raw_a["x_m"], raw_a["y_m"]))
    ax.annotate("B raw", (raw_b["x_m"], raw_b["y_m"]))
    ax.annotate("A' top-left", (a_prime["x_m"], a_prime["y_m"]))
    ax.annotate("B' bottom-right", (b_prime["x_m"], b_prime["y_m"]))
    ax.legend()
    fig.tight_layout()
    path = out_dir / "preview_workspace_ab_aprime_bprime.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    outputs[path.name] = path

    fig, ax = setup("Tool Path Primary: A' To B'")
    for row in tool_rows:
        sx = origin_x + float(row["tool_start_x_m"])
        sy = origin_y + float(row["tool_start_y_m"])
        ex = origin_x + float(row["tool_end_x_m"])
        ey = origin_y + float(row["tool_end_y_m"])
        active = bool(row["tool_active"])
        ax.plot([sx, ex], [sy, ey], color="tab:orange" if active else "0.55", linestyle="-" if active else "--", linewidth=2.2 if active else 1.2)
    fig.tight_layout()
    path = out_dir / "preview_tool_path_primary.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    outputs[path.name] = path

    fig, ax = setup("Primitive Sequence")
    for row in primitives:
        sx = float(row["start_x_m"])
        sy = float(row["start_y_m"])
        ex = float(row["end_x_m"])
        ey = float(row["end_y_m"])
        ptype = str(row["primitive_type"])
        if ptype.startswith("move"):
            ax.annotate("", xy=(ex, ey), xytext=(sx, sy), arrowprops={"arrowstyle": "->", "color": "tab:blue", "linewidth": 1.0})
        else:
            ax.scatter([sx], [sy], color="tab:purple", s=30)
        ax.text(sx, sy, f"P{int(row['primitive_index']):03d}", fontsize=6)
    fig.tight_layout()
    path = out_dir / "preview_primitive_sequence.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    outputs[path.name] = path

    fig, ax = setup("Approach Then Serpentine")
    for row in primitives:
        sx = float(row["start_x_m"])
        sy = float(row["start_y_m"])
        ex = float(row["end_x_m"])
        ey = float(row["end_y_m"])
        role = str(row["segment_role"])
        if str(row["primitive_type"]).startswith("move"):
            ax.plot([sx, ex], [sy, ey], color="tab:green" if role.startswith("approach") else "tab:blue", linewidth=1.2)
    fig.tight_layout()
    path = out_dir / "preview_approach_then_serpentine.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    outputs[path.name] = path
    return outputs


def build_path_package(
    *,
    raw_a: tuple[float, float],
    raw_b: tuple[float, float],
    current_pose: tuple[float, float, float] | None,
    step_spacing_m: float,
    tool_side: str,
    tool_lateral_offset_m: float,
    tool_width_m: float,
    tool_length_m: float,
    robot_width_m: float,
    robot_length_m: float,
    georeference: dict[str, object] | None = None,
) -> dict[str, object]:
    workspace = normalize_field_ab(a_x=raw_a[0], a_y=raw_a[1], b_x=raw_b[0], b_y=raw_b[1])
    a_prime = workspace["A_prime_top_left"]  # type: ignore[index]
    b_prime = workspace["B_prime_bottom_right"]  # type: ignore[index]
    if current_pose is None:
        current_pose = (raw_a[0], raw_a[1], 0.0)
    approach = build_approach_primitives(
        current_x=current_pose[0],
        current_y=current_pose[1],
        current_heading_deg=current_pose[2],
        target_x=float(a_prime["x_m"]),
        target_y=float(a_prime["y_m"]),
    )
    config = SideToolPlanConfig(
        workspace_mode="tool_serpentine_ab",
        a_x_m=float(a_prime["x_m"]),
        a_y_m=float(a_prime["y_m"]),
        b_x_m=float(b_prime["x_m"]),
        b_y_m=float(b_prime["y_m"]),
        step_spacing_m=step_spacing_m,
        tool_side=tool_side,  # type: ignore[arg-type]
        tool_lateral_offset_m=tool_lateral_offset_m,
        tool_width_m=tool_width_m,
        tool_length_m=tool_length_m,
        robot_width_m=robot_width_m,
        robot_length_m=robot_length_m,
        contamination_mode="off",
        tool_active_on_sweep_tracks=True,
        tool_active_on_connectors=False,
    )
    preview = generate_tool_serpentine_preview(config)
    origin_x = float(workspace["x_min_m"])
    origin_y = float(workspace["y_min_m"])
    serpentine_primitives = []
    for row in preview["primitive_rows"]:  # type: ignore[index]
        updated = dict(row)
        for key in ("start_x_m", "end_x_m"):
            updated[key] = float(updated[key]) + origin_x
        for key in ("start_y_m", "end_y_m"):
            updated[key] = float(updated[key]) + origin_y
        serpentine_primitives.append(updated)
    primitives = approach + _offset_primitives(serpentine_primitives, len(approach))
    primitive_sequence_valid = all(
        row["primitive_type"] in ALLOWED_PRIMITIVES and row["motor_command_generated"] is False
        for row in primitives
    )
    tool_path = list(preview["tool_segments"])  # type: ignore[index]
    summary = {
        "raw_A": workspace["raw_A"],
        "raw_B": workspace["raw_B"],
        "A_prime_top_left": workspace["A_prime_top_left"],
        "B_prime_bottom_right": workspace["B_prime_bottom_right"],
        "approach_path_generated": bool(approach),
        "serpentine_path_generated": True,
        "tool_side": tool_side,
        "step_spacing_m": step_spacing_m,
        "primitive_sequence_valid": primitive_sequence_valid,
        "motor_command_generated": False,
        "physical_output_active": False,
        "ready_for_outdoor_no_motion_validation": primitive_sequence_valid,
        "georeference_available": georeference is not None,
    }
    package = {
        "generated_at_utc": dt.datetime.now(tz=dt.UTC).isoformat(),
        "normalized_workspace": workspace,
        "approach_to_A_prime": approach,
        "tool_path": tool_path,
        "chassis_path": preview["chassis_segments"],  # type: ignore[index]
        "primitive_sequence": primitives,
        "simple_serpentine_summary": preview["summary"],  # type: ignore[index]
        "summary": summary,
        "path_preview_only": True,
        "motor_command_generated": False,
        "physical_output_active": False,
    }
    if georeference is not None:
        package["georeference"] = georeference
        package["normalized_workspace"]["georeference_available"] = True  # type: ignore[index]
        package["summary"]["georeference_available"] = True  # type: ignore[index]
        for key in (
            "raw_A_lat",
            "raw_A_lon",
            "raw_B_lat",
            "raw_B_lon",
            "origin_lat",
            "origin_lon",
            "local_frame_type",
            "x_axis_source",
            "x_axis_bearing_deg",
            "meters_per_deg_lat",
            "meters_per_deg_lon",
            "meters_per_lat",
            "meters_per_lon",
        ):
            package["summary"][key] = georeference[key]  # type: ignore[index]
    return package


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Convert captured field A/B points into a no-motion serpentine path package."
    )
    parser.add_argument("--field-points-json")
    parser.add_argument("--field-points-georef-json")
    parser.add_argument("--a-x", type=float)
    parser.add_argument("--a-y", type=float)
    parser.add_argument("--b-x", type=float)
    parser.add_argument("--b-y", type=float)
    parser.add_argument("--current-x", type=float)
    parser.add_argument("--current-y", type=float)
    parser.add_argument("--current-heading-deg", type=float, default=0.0)
    parser.add_argument("--step-spacing-m", type=float, required=True)
    parser.add_argument("--tool-side", choices=("left", "right"), default="left")
    parser.add_argument("--tool-lateral-offset-m", type=float, default=0.24)
    parser.add_argument("--tool-width-m", type=float, default=0.30)
    parser.add_argument("--tool-length-m", type=float, default=0.18)
    parser.add_argument("--robot-width-m", type=float, default=0.18)
    parser.add_argument("--robot-length-m", type=float, default=0.18)
    parser.add_argument("--out-dir", default="outputs/field_ab_serpentine/latest")
    return parser


def _resolve_points(args: argparse.Namespace) -> tuple[tuple[float, float, float, float], dict[str, object] | None]:
    if args.field_points_georef_json:
        return _load_georef_points(Path(args.field_points_georef_json))
    if args.field_points_json:
        return _load_field_points(Path(args.field_points_json)), None
    missing = [
        name
        for name in ("a_x", "a_y", "b_x", "b_y")
        if getattr(args, name) is None
    ]
    if missing:
        raise ValueError("--field-points-json or all manual --a-x/--a-y/--b-x/--b-y values are required")
    return (float(args.a_x), float(args.a_y), float(args.b_x), float(args.b_y)), None


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    raw, georeference = _resolve_points(args)
    current_pose = None
    if args.current_x is not None and args.current_y is not None:
        current_pose = (float(args.current_x), float(args.current_y), float(args.current_heading_deg))
    package = build_path_package(
        raw_a=(raw[0], raw[1]),
        raw_b=(raw[2], raw[3]),
        current_pose=current_pose,
        step_spacing_m=args.step_spacing_m,
        tool_side=args.tool_side,
        tool_lateral_offset_m=args.tool_lateral_offset_m,
        tool_width_m=args.tool_width_m,
        tool_length_m=args.tool_length_m,
        robot_width_m=args.robot_width_m,
        robot_length_m=args.robot_length_m,
        georeference=georeference,
    )
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    workspace_path = out_dir / "normalized_workspace.json"
    tool_csv = out_dir / "tool_path.csv"
    primitive_csv = out_dir / "primitive_sequence.csv"
    package_path = out_dir / "path_package.json"
    summary_path = out_dir / "summary.md"
    _write_json(workspace_path, package["normalized_workspace"])  # type: ignore[arg-type]
    _write_csv(tool_csv, package["tool_path"], TOOL_PATH_FIELDS)  # type: ignore[arg-type]
    _write_csv(primitive_csv, package["primitive_sequence"], PRIMITIVE_FIELDS)  # type: ignore[arg-type]
    _write_json(package_path, package)
    _write_summary(summary_path, package)
    preview_paths = _write_previews(out_dir, package)
    print("Field A/B serpentine path package generated.")
    print("Preview/no-motion only: no rover commands, no motor output, no firmware upload.")
    print(f"normalized_workspace_json: {workspace_path}")
    print(f"tool_path_csv: {tool_csv}")
    print(f"primitive_sequence_csv: {primitive_csv}")
    print(f"path_package_json: {package_path}")
    print(f"summary_md: {summary_path}")
    for name, path in preview_paths.items():
        print(f"{name}: {path if path is not None else 'skipped'}")
    summary = package["summary"]  # type: ignore[index]
    print(f"A_prime_top_left: {summary['A_prime_top_left']}")
    print(f"B_prime_bottom_right: {summary['B_prime_bottom_right']}")
    print(f"primitive_sequence_valid: {summary['primitive_sequence_valid']}")
    print(f"motor_command_generated: {summary['motor_command_generated']}")
    print(f"physical_output_active: {summary['physical_output_active']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
