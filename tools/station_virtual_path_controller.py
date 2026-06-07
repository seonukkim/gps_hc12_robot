from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Sequence

try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401

from tools.path_no_motion_validation import PathPackageResolutionError, resolve_path_package
from tools import station_path_package_tracker


VIRTUAL_FIELDS = (
    "row_index",
    "mode",
    "station_package_target_source",
    "firmware_active_target_source",
    "firmware_still_compile_time",
    "local_pose_available",
    "active_primitive_index",
    "active_tool_segment_id",
    "target_distance_m",
    "target_bearing_deg",
    "cross_track_error_m",
    "current_heading_deg",
    "heading_error_deg",
    "virtual_heading_status",
    "virtual_forward_cmd",
    "virtual_turn_cmd",
    "virtual_left_cmd",
    "virtual_right_cmd",
    "virtual_control_generated",
    "tool_active_expected",
    "coverage_contributes",
    "motor_command_generated",
    "physical_output_active",
    "ready_for_station_virtual_control_preview",
    "ready_for_motor_test",
)


def _optional_float(value: object) -> float | None:
    if value is None:
        return None
    text = str(value).strip()
    if text.upper() in {"", "NA", "NAN", "NONE", "NULL"}:
        return None
    try:
        parsed = float(text)
    except ValueError:
        return None
    return parsed if math.isfinite(parsed) else None


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def compute_virtual_control(
    target_row: dict[str, object],
    *,
    max_virtual_forward_cmd: float = 0.10,
    max_virtual_turn_cmd: float = 0.05,
    lookahead_m: float = 1.0,
) -> dict[str, object]:
    distance = _optional_float(target_row.get("target_distance_m"))
    bearing = _optional_float(target_row.get("target_bearing_deg"))
    cross_track = _optional_float(target_row.get("cross_track_error_m"))
    heading = _optional_float(target_row.get("current_heading_deg"))
    heading_error = _optional_float(target_row.get("heading_error_deg"))
    local_pose_available = target_row.get("local_pose_available") is True
    target_finite = distance is not None and bearing is not None and cross_track is not None
    if not local_pose_available or not target_finite:
        virtual_forward = 0.0
        virtual_turn = 0.0
        virtual_heading_status = "NO_LOCAL_TARGET"
        control_generated = False
    else:
        virtual_forward = min(max_virtual_forward_cmd, max(0.0, float(distance) * 0.2))
        if heading is None or heading_error is None:
            virtual_heading_status = "DIAG_ONLY"
            bearing_component = _clamp(float(bearing) / 90.0, 1.0)
            cross_component = _clamp(float(cross_track) / max(lookahead_m, 1e-6), 1.0)
            virtual_turn = _clamp((0.7 * bearing_component + 0.3 * cross_component) * max_virtual_turn_cmd, max_virtual_turn_cmd)
        else:
            virtual_heading_status = "OK"
            heading_component = _clamp(float(heading_error) / 45.0, 1.0)
            cross_component = _clamp(float(cross_track) / max(lookahead_m, 1e-6), 1.0)
            virtual_turn = _clamp((0.8 * heading_component + 0.2 * cross_component) * max_virtual_turn_cmd, max_virtual_turn_cmd)
        control_generated = True
    virtual_left = virtual_forward - virtual_turn
    virtual_right = virtual_forward + virtual_turn
    ready_preview = bool(local_pose_available and target_finite and control_generated)
    return {
        "row_index": target_row.get("row_index", 0),
        "mode": target_row.get("mode", ""),
        "station_package_target_source": target_row.get("station_package_target_source", "path_package"),
        "firmware_active_target_source": target_row.get("firmware_active_target_source", "unknown"),
        "firmware_still_compile_time": target_row.get("firmware_still_compile_time", False),
        "local_pose_available": local_pose_available,
        "active_primitive_index": target_row.get("active_primitive_index", "NA"),
        "active_tool_segment_id": target_row.get("active_tool_segment_id", "NA"),
        "target_distance_m": "NA" if distance is None else distance,
        "target_bearing_deg": "NA" if bearing is None else bearing,
        "cross_track_error_m": "NA" if cross_track is None else cross_track,
        "current_heading_deg": "NA" if heading is None else heading,
        "heading_error_deg": "NA_DIAG_ONLY" if heading_error is None else heading_error,
        "virtual_heading_status": virtual_heading_status,
        "virtual_forward_cmd": virtual_forward,
        "virtual_turn_cmd": virtual_turn,
        "virtual_left_cmd": virtual_left,
        "virtual_right_cmd": virtual_right,
        "virtual_control_generated": control_generated,
        "tool_active_expected": target_row.get("tool_active_expected", "NA"),
        "coverage_contributes": target_row.get("coverage_contributes", "NA"),
        "motor_command_generated": False,
        "physical_output_active": False,
        "ready_for_station_virtual_control_preview": ready_preview,
        "ready_for_motor_test": False,
    }


def build_virtual_rows(
    target_rows: Sequence[dict[str, object]],
    *,
    max_virtual_forward_cmd: float = 0.10,
    max_virtual_turn_cmd: float = 0.05,
    lookahead_m: float = 1.0,
) -> list[dict[str, object]]:
    return [
        compute_virtual_control(
            row,
            max_virtual_forward_cmd=max_virtual_forward_cmd,
            max_virtual_turn_cmd=max_virtual_turn_cmd,
            lookahead_m=lookahead_m,
        )
        for row in target_rows
    ]


def _summary_from_rows(selected_path: Path, package: dict[str, object], rows: Sequence[dict[str, object]]) -> dict[str, object]:
    first = rows[0] if rows else {}
    primitive_valid = bool(package["summary"]["primitive_sequence_valid"])  # type: ignore[index]
    target_distance = _optional_float(first.get("target_distance_m"))
    target_bearing = _optional_float(first.get("target_bearing_deg"))
    ready_preview = (
        primitive_valid
        and first.get("local_pose_available") is True
        and target_distance is not None
        and target_bearing is not None
        and first.get("motor_command_generated") is False
    )
    return {
        "selected_path_package": str(selected_path),
        "path_package_loaded": True,
        "local_pose_available": first.get("local_pose_available", False),
        "station_package_target_source": first.get("station_package_target_source", "path_package"),
        "firmware_active_target_source": first.get("firmware_active_target_source", "unknown"),
        "firmware_still_compile_time": first.get("firmware_still_compile_time", False),
        "active_primitive_index": first.get("active_primitive_index", "NA"),
        "active_tool_segment_id": first.get("active_tool_segment_id", "NA"),
        "target_distance_m": first.get("target_distance_m", "NA"),
        "target_bearing_deg": first.get("target_bearing_deg", "NA"),
        "cross_track_error_m": first.get("cross_track_error_m", "NA"),
        "current_heading_deg": first.get("current_heading_deg", "NA"),
        "virtual_heading_status": first.get("virtual_heading_status", "NA"),
        "virtual_forward_cmd": first.get("virtual_forward_cmd", 0.0),
        "virtual_turn_cmd": first.get("virtual_turn_cmd", 0.0),
        "virtual_left_cmd": first.get("virtual_left_cmd", 0.0),
        "virtual_right_cmd": first.get("virtual_right_cmd", 0.0),
        "virtual_control_generated": first.get("virtual_control_generated", False),
        "motor_command_generated": False,
        "physical_output_active": False,
        "ready_for_station_virtual_control_preview": ready_preview,
        "ready_for_motor_test": False,
        "next_action": "Virtual controller preview only; do not send these diagnostics to firmware.",
    }


def _write_csv(path: Path, rows: Sequence[dict[str, object]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=VIRTUAL_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in VIRTUAL_FIELDS})


def _write_summary(path: Path, summary: dict[str, object]) -> None:
    lines = [
        "# Stage 14 Station Virtual Path Controller",
        "",
        "Station-side diagnostic preview only. Virtual values are not firmware motor commands.",
        "",
    ]
    for key, value in summary.items():
        lines.append(f"- {key}: `{value}`")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _plot_virtual_control(path: Path, rows: Sequence[dict[str, object]]) -> None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    indices = [int(row.get("row_index", index)) for index, row in enumerate(rows)]
    forward = [float(row.get("virtual_forward_cmd", 0.0)) for row in rows]
    turn = [float(row.get("virtual_turn_cmd", 0.0)) for row in rows]
    left = [float(row.get("virtual_left_cmd", 0.0)) for row in rows]
    right = [float(row.get("virtual_right_cmd", 0.0)) for row in rows]
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(indices, forward, label="virtual_forward_cmd")
    ax.plot(indices, turn, label="virtual_turn_cmd")
    ax.plot(indices, left, label="virtual_left_cmd", linestyle="--")
    ax.plot(indices, right, label="virtual_right_cmd", linestyle="--")
    ax.axhline(0.0, color="0.4", linewidth=0.8)
    ax.set_title("Stage 14 Virtual Control Diagnostics")
    ax.set_xlabel("row_index")
    ax.set_ylabel("diagnostic command")
    ax.grid(True, linestyle="--", alpha=0.3)
    ax.legend()
    ax.text(0.01, 0.01, "No serial writes; no motor commands", transform=ax.transAxes, fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def _target_rows_for_args(args: argparse.Namespace, package: dict[str, object]) -> list[dict[str, object]]:
    if args.mode == "offline_pose":
        if args.current_x is None or args.current_y is None:
            raise SystemExit("--current-x and --current-y are required in offline_pose mode")
        return [
            station_path_package_tracker.compute_target_status(
                package,
                current_x=float(args.current_x),
                current_y=float(args.current_y),
                current_heading_deg=args.current_heading_deg,
                mode=args.mode,
                firmware_active_target_source="not_checked_offline_pose",
                local_pose_source="manual_local",
            )
        ]
    if args.mode == "replay_log":
        if not args.log:
            raise SystemExit("--log is required in replay_log mode")
        return station_path_package_tracker.build_rows_from_replay(
            package,
            Path(args.log).read_text(encoding="utf-8", errors="replace"),
            args.mode,
        )
    if not args.port:
        raise SystemExit("--port is required in live_usbdbg mode")
    usbdbg_rows = station_path_package_tracker._read_live_usbdbg(args.port, args.duration_s)  # noqa: SLF001
    log_text = "\n".join(" ".join(f"{key}={value}" for key, value in row.items()) for row in usbdbg_rows)
    return station_path_package_tracker.build_rows_from_replay(package, log_text, args.mode)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Compute station-side virtual path control diagnostics.")
    parser.add_argument("--path-package", default="latest")
    parser.add_argument("--mode", choices=("offline_pose", "replay_log", "live_usbdbg"), required=True)
    parser.add_argument("--current-x", type=float)
    parser.add_argument("--current-y", type=float)
    parser.add_argument("--current-heading-deg", type=float)
    parser.add_argument("--log")
    parser.add_argument("--port")
    parser.add_argument("--duration-s", type=float, default=60.0)
    parser.add_argument("--max-virtual-forward-cmd", type=float, default=0.10)
    parser.add_argument("--max-virtual-turn-cmd", type=float, default=0.05)
    parser.add_argument("--lookahead-m", type=float, default=1.0)
    parser.add_argument("--out-dir", required=True)
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
        return 2
    package = json.loads(selected.read_text(encoding="utf-8"))
    target_rows = _target_rows_for_args(args, package)
    virtual_rows = build_virtual_rows(
        target_rows,
        max_virtual_forward_cmd=args.max_virtual_forward_cmd,
        max_virtual_turn_cmd=args.max_virtual_turn_cmd,
        lookahead_m=args.lookahead_m,
    )
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / "virtual_control.csv"
    summary_path = out_dir / "summary.md"
    preview_path = out_dir / "preview_virtual_control.png"
    _write_csv(csv_path, virtual_rows)
    summary = _summary_from_rows(selected, package, virtual_rows)
    _write_summary(summary_path, summary)
    _plot_virtual_control(preview_path, virtual_rows)
    print("Stage 14 station virtual path controller preview complete.")
    print(f"selected_path_package={selected}")
    print(f"virtual_control_csv={csv_path}")
    print(f"summary_md={summary_path}")
    print(f"preview_virtual_control_png={preview_path if preview_path.exists() else 'not_generated'}")
    for key in (
        "virtual_control_generated",
        "virtual_heading_status",
        "virtual_forward_cmd",
        "virtual_turn_cmd",
        "ready_for_station_virtual_control_preview",
        "ready_for_motor_test",
        "motor_command_generated",
        "physical_output_active",
    ):
        print(f"{key}={summary[key]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
