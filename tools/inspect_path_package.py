from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Sequence

try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401

from tools.path_no_motion_validation import PathPackageResolutionError, resolve_path_package


ALLOWED_PRIMITIVES = {"move_forward", "move_backward", "rotate_left", "rotate_right"}


def _close(a: object, b: object, tolerance: float = 1e-6) -> bool:
    return math.isclose(float(a), float(b), abs_tol=tolerance)


def inspect_package(package: dict[str, object], selected_path: Path) -> dict[str, object]:
    workspace = package["normalized_workspace"]  # type: ignore[index]
    summary = package["summary"]  # type: ignore[index]
    tool_path = list(package["tool_path"])  # type: ignore[index]
    primitives = list(package["primitive_sequence"])  # type: ignore[index]
    approach = list(package.get("approach_to_A_prime", []))  # type: ignore[arg-type]
    a_prime = workspace["A_prime_top_left"]  # type: ignore[index]
    b_prime = workspace["B_prime_bottom_right"]  # type: ignore[index]

    starts_at_a_prime = bool(tool_path) and _close(tool_path[0]["tool_start_x_m"], a_prime["x_m"]) and _close(
        tool_path[0]["tool_start_y_m"], a_prime["y_m"]
    )
    ends_at_b_prime = bool(tool_path) and _close(tool_path[-1]["tool_end_x_m"], b_prime["x_m"]) and _close(
        tool_path[-1]["tool_end_y_m"], b_prime["y_m"]
    )
    continuous = all(
        _close(previous["tool_end_x_m"], current["tool_start_x_m"])
        and _close(previous["tool_end_y_m"], current["tool_start_y_m"])
        for previous, current in zip(tool_path, tool_path[1:])
    )
    connectors_inactive = all(
        row["tool_active"] is False and row["coverage_contributes"] is False
        for row in tool_path
        if row["tool_segment_type"] == "tool_spacing_connector"
    )
    sweep_tracks_active = all(
        row["tool_active"] is True and row["coverage_contributes"] is True
        for row in tool_path
        if row["tool_segment_type"] == "tool_sweep_track"
    )
    primitive_sequence_valid = all(
        row["primitive_type"] in ALLOWED_PRIMITIVES and row["motor_command_generated"] is False
        for row in primitives
    )
    motor_command_generated = bool(package.get("motor_command_generated")) or bool(summary.get("motor_command_generated"))
    motor_command_generated = motor_command_generated or any(bool(row.get("motor_command_generated")) for row in primitives)
    validation = {
        "tool_side_left": summary.get("tool_side") == "left",
        "tool_path_starts_at_A_prime": starts_at_a_prime,
        "tool_path_ends_at_B_prime": ends_at_b_prime,
        "tool_path_continuous": continuous,
        "connectors_inactive": connectors_inactive,
        "sweep_tracks_active": sweep_tracks_active,
        "primitive_sequence_allowed": primitive_sequence_valid,
        "motor_command_generated_false": motor_command_generated is False,
    }
    validation["path_package_valid_for_preview"] = all(bool(value) for value in validation.values())
    return {
        "selected_path_package": str(selected_path),
        "normalized_workspace": workspace,
        "A_prime_top_left": a_prime,
        "B_prime_bottom_right": b_prime,
        "approach_primitive_count": len(approach),
        "tool_path_segment_count": len(tool_path),
        "tool_sweep_track_count": sum(1 for row in tool_path if row["tool_segment_type"] == "tool_sweep_track"),
        "tool_connector_count": sum(1 for row in tool_path if row["tool_segment_type"] == "tool_spacing_connector"),
        "primitive_count": len(primitives),
        "approach_primitives": approach,
        "tool_path_segments": tool_path,
        "primitive_sequence_summary": {
            "allowed_primitives": sorted(ALLOWED_PRIMITIVES),
            "primitive_types_seen": sorted({str(row["primitive_type"]) for row in primitives}),
            "first_primitive": primitives[0] if primitives else None,
            "last_primitive": primitives[-1] if primitives else None,
        },
        "validation": validation,
        "motor_command_generated": False,
        "path_preview_only": True,
    }


def _write_markdown(path: Path, inspection: dict[str, object]) -> None:
    validation = inspection["validation"]  # type: ignore[index]
    lines = [
        "# Stage 11 Path Package Inspection",
        "",
        "Offline/no-motion only. No rover motor commands are generated.",
        "",
        f"- selected_path_package: `{inspection['selected_path_package']}`",
        f"- normalized_workspace: `{inspection['normalized_workspace']}`",
        f"- A_prime_top_left: `{inspection['A_prime_top_left']}`",
        f"- B_prime_bottom_right: `{inspection['B_prime_bottom_right']}`",
        f"- approach_primitive_count: `{inspection['approach_primitive_count']}`",
        f"- tool_path_segment_count: `{inspection['tool_path_segment_count']}`",
        f"- tool_sweep_track_count: `{inspection['tool_sweep_track_count']}`",
        f"- tool_connector_count: `{inspection['tool_connector_count']}`",
        f"- primitive_count: `{inspection['primitive_count']}`",
        "",
        "## Validation",
    ]
    for key, value in validation.items():  # type: ignore[union-attr]
        lines.append(f"- {key}: `{value}`")
    lines.extend(
        [
            "",
            "## Primitive Sequence Summary",
            f"- allowed_primitives: `{inspection['primitive_sequence_summary']['allowed_primitives']}`",  # type: ignore[index]
            f"- primitive_types_seen: `{inspection['primitive_sequence_summary']['primitive_types_seen']}`",  # type: ignore[index]
            "- motor_command_generated: `False`",
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Inspect a generated path package.")
    parser.add_argument("--path-package", default="latest")
    parser.add_argument("--out-dir", default="outputs/path_package_inspection/latest")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    out_dir = Path(args.out_dir)
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
    out_dir.mkdir(parents=True, exist_ok=True)
    json_path = out_dir / "path_package_inspection.json"
    md_path = out_dir / "path_package_inspection.md"
    json_path.write_text(json.dumps(inspection, indent=2) + "\n", encoding="utf-8")
    _write_markdown(md_path, inspection)
    print("Path package inspection complete.")
    print(f"selected_path_package={selected}")
    print(f"path_package_inspection_json={json_path}")
    print(f"path_package_inspection_md={md_path}")
    for key, value in inspection["validation"].items():  # type: ignore[union-attr]
        print(f"{key}={value}")
    print("motor_command_generated=false")
    return 0 if inspection["validation"]["path_package_valid_for_preview"] else 1  # type: ignore[index]


if __name__ == "__main__":
    raise SystemExit(main())
