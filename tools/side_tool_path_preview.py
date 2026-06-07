from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import sys
from pathlib import Path
from typing import Sequence

try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401

from gps_coverage_core.side_tool_planner import (
    SideToolPlanConfig,
    _cells_for_polygon,
    _coverage_grid,
    _workspace_info,
    boundary_polygon,
    contamination_analysis,
    geometry_samples_for_path,
    generate_tool_serpentine_preview,
    generate_side_tool_path,
    strategy_diagnostics,
    summarize_side_tool_path,
    swept_volume_summary,
)


CSV_FIELDS = (
    "segment_index",
    "index",
    "segment_id",
    "segment_type",
    "primitive_type",
    "primitive_subtype",
    "primitive_index",
    "movement_primitive_type",
    "primitive_sequence_valid",
    "invalid_motion_reason",
    "distance_m",
    "angle_deg",
    "starts_at_x_m",
    "starts_at_y_m",
    "starts_at_heading_deg",
    "ends_at_x_m",
    "ends_at_y_m",
    "ends_at_heading_deg",
    "contamination_free_primitive",
    "selected_transition_primitive",
    "lane_index",
    "direction",
    "motion_direction",
    "travel_direction_deg",
    "kinematic_model",
    "differential_drive_feasible",
    "kinematic_violation_reason",
    "heading_delta_deg",
    "segment_distance_m",
    "segment_rotation_deg",
    "axis_m",
    "lateral_m",
    "x_m",
    "y_m",
    "world_x_m",
    "world_y_m",
    "heading_deg",
    "world_heading_deg",
    "workspace_mode",
    "tool_side",
    "workspace_side",
    "tool_world_side",
    "candidate_mode",
    "tool_x_m",
    "tool_y_m",
    "tool_world_x_m",
    "tool_world_y_m",
    "tool_heading_deg",
    "tool_edge_min_y_m",
    "tool_edge_max_y_m",
    "tool_inner_x_m",
    "tool_inner_y_m",
    "tool_inner_world_x_m",
    "tool_inner_world_y_m",
    "tool_outer_x_m",
    "tool_outer_y_m",
    "tool_outer_world_x_m",
    "tool_outer_world_y_m",
    "tool_lateral_offset_m",
    "tool_width_m",
    "tool_length_m",
    "tool_route_index",
    "tool_segment_id",
    "tool_segment_type",
    "tool_start_x_m",
    "tool_start_y_m",
    "tool_end_x_m",
    "tool_end_y_m",
    "tool_connector_angle_deg",
    "tool_connector_distance_m",
    "tool_waypoint_label",
    "tool_path_primary",
    "chassis_segment_id",
    "associated_tool_segment_id",
    "chassis_start_x_m",
    "chassis_start_y_m",
    "chassis_end_x_m",
    "chassis_end_y_m",
    "chassis_heading_deg",
    "chassis_path_derived_from_tool",
    "primitive_value",
    "chassis_bbox_x_min_m",
    "chassis_bbox_x_max_m",
    "chassis_bbox_y_min_m",
    "chassis_bbox_y_max_m",
    "tool_bbox_x_min_m",
    "tool_bbox_x_max_m",
    "tool_bbox_y_min_m",
    "tool_bbox_y_max_m",
    "combined_bbox_x_min_m",
    "combined_bbox_x_max_m",
    "combined_bbox_y_min_m",
    "combined_bbox_y_max_m",
    "lane_spacing_m",
    "transition_style",
    "transition_envelope_x_min_m",
    "transition_envelope_x_max_m",
    "transition_envelope_y_min_m",
    "transition_envelope_y_max_m",
    "transition_envelope_within_boundary",
    "transition_pocket_side",
    "transition_violation_reason",
    "within_boundary",
    "chassis_within_boundary",
    "tool_within_boundary",
    "tool_swept_within_boundary",
    "rotation_sample_count",
    "translation_sample_count",
    "rotation_swept_chassis_within_boundary",
    "rotation_swept_tool_within_boundary",
    "rotation_swept_combined_within_boundary",
    "rotation_swept_violation_count",
    "rotation_swept_violation_reason",
    "rotation_swept_x_min_m",
    "rotation_swept_x_max_m",
    "rotation_swept_y_min_m",
    "rotation_swept_y_max_m",
    "translation_swept_chassis_within_boundary",
    "translation_swept_tool_within_boundary",
    "translation_swept_combined_within_boundary",
    "translation_swept_violation_count",
    "swept_volume_within_boundary",
    "swept_volume_violation_reason",
    "time_start_index",
    "time_end_index",
    "contamination_checked",
    "contamination_free_segment",
    "contamination_violation_count",
    "contamination_violation_area_m2",
    "cleaned_area_added_m2",
    "tool_reclean_area_m2",
    "chassis_on_prior_cleaned_area_m2",
    "tool_active",
    "first_cleaning_heading_aligned",
    "start_reaches_A",
    "end_reaches_B",
    "start_error_m",
    "end_error_m",
    "route_reaches_B",
    "boundary_violation_reason",
    "coverage_role",
    "coverage_contributes",
    "coverage_cells_added",
    "coverage_area_added_m2",
    "motor_command_generated",
    "notes",
)


GEOMETRY_SAMPLE_FIELDS = (
    "time_step",
    "sample_index",
    "segment_id",
    "segment_type",
    "lane_index",
    "primitive_type",
    "differential_drive_feasible",
    "sample_type",
    "x_m",
    "y_m",
    "heading_deg",
    "chassis_polygon_json",
    "tool_polygon_json",
    "combined_bbox_x_min_m",
    "combined_bbox_x_max_m",
    "combined_bbox_y_min_m",
    "combined_bbox_y_max_m",
    "inside_workspace",
    "chassis_inside_workspace",
    "tool_inside_workspace",
    "violation_reason",
    "chassis_cells_count",
    "tool_cells_count",
    "prior_cleaned_chassis_overlap_cells",
    "prior_cleaned_chassis_overlap_area_m2",
    "current_tool_reclean_cells",
    "current_tool_reclean_area_m2",
    "contamination_violation",
    "contamination_violation_reason",
    "tool_active",
    "frame_path",
    "motor_command_generated",
)

CONTAMINATION_EVENT_FIELDS = (
    "event_index",
    "time_step",
    "segment_id",
    "segment_type",
    "lane_index",
    "x_m",
    "y_m",
    "heading_deg",
    "overlap_area_m2",
    "affected_cell_count",
    "violation_reason",
    "severity",
)

STRATEGY_DIAGNOSTIC_FIELDS = (
    "strategy_name",
    "selected_lane_count",
    "reaches_B",
    "contamination_violation_count",
    "first_violation_lane",
    "first_violation_primitive",
    "coverage_ratio",
    "reason_for_rejection",
    "route_feasible_for_physical_preview",
)

TIMELINE_STATE_FIELDS = (
    "time_step",
    "segment_id",
    "primitive_type",
    "segment_type",
    "lane_index",
    "chassis_x_m",
    "chassis_y_m",
    "chassis_heading_deg",
    "heading_deg",
    "tool_x_m",
    "tool_y_m",
    "tool_heading_deg",
    "chassis_cells_count",
    "tool_cells_count",
    "cleaned_area_m2",
    "coverage_ratio_so_far",
    "chassis_after_cleaned_area_m2",
    "wet_area_m2",
    "contamination_violation_count_so_far",
    "first_cleaning_heading_aligned",
    "tool_active",
    "route_to_B_remaining_m",
    "violation_reason",
    "motor_command_generated",
    "frame_path",
)


def _parse_row_count(value: str) -> int | str:
    if value == "auto":
        return value
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("--row-count must be 'auto' or a positive integer")
    return parsed


def _parse_min_lane_count(value: str) -> int | str:
    if value == "auto":
        return value
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("--min-lane-count must be 'auto' or a positive integer")
    return parsed


def build_simple_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Generate a simple offline tool-centered serpentine preview. "
            "The tool path is primary; no rover motor commands are generated. "
            "Use --advanced for legacy/debug planner options."
        )
    )
    parser.add_argument("--a-x", type=float, required=True)
    parser.add_argument("--a-y", type=float, required=True)
    parser.add_argument("--b-x", type=float, required=True)
    parser.add_argument("--b-y", type=float, required=True)
    parser.add_argument("--step-spacing-m", type=float, required=True)
    parser.add_argument("--tool-side", choices=("left", "right"), default="left")
    parser.add_argument("--tool-lateral-offset-m", type=float, default=0.24)
    parser.add_argument("--tool-width-m", type=float, default=0.30)
    parser.add_argument("--tool-length-m", type=float, default=0.18)
    parser.add_argument("--robot-width-m", type=float, default=0.18)
    parser.add_argument("--robot-length-m", type=float, default=0.18)
    parser.add_argument("--coverage-resolution-m", type=float, default=0.05)
    parser.add_argument(
        "--emit-previews",
        default="true",
        choices=("true", "false"),
        help="Write the four essential PNG previews.",
    )
    parser.add_argument(
        "--out-dir",
        default="outputs/side_tool_path_preview/simple_serpentine",
        help="Output directory for summary, CSV files, and preview images.",
    )
    return parser


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Generate an offline side-mounted cleaning-tool path preview. "
            "This never sends HC-12 frames, rover commands, or motor commands."
        )
    )
    parser.add_argument("--tool-side", choices=("left", "right"), default="left")
    parser.add_argument("--tool-lateral-offset-m", type=float, default=0.24)
    parser.add_argument("--tool-width-m", type=float, default=0.30)
    parser.add_argument("--tool-length-m", type=float, default=0.18)
    parser.add_argument("--lane-spacing-m", type=float, default=0.25)
    parser.add_argument("--step-spacing-m", type=float, default=0.25)
    parser.add_argument("--row-length-m", type=float)
    parser.add_argument("--row-count", type=_parse_row_count, default="auto")
    parser.add_argument("--start-x-m", type=float, default=0.0)
    parser.add_argument("--start-y-m", type=float, default=0.0)
    parser.add_argument("--start-heading-deg", type=float, default=0.0)
    parser.add_argument("--a-x", type=float)
    parser.add_argument("--a-y", type=float)
    parser.add_argument("--b-x", type=float)
    parser.add_argument("--b-y", type=float)
    parser.add_argument(
        "--workspace-mode",
        choices=("tool_serpentine_ab", "ab_diagonal_center", "ab_centerline_width", "diagonal_ab", "axis_width"),
        default="tool_serpentine_ab",
    )
    parser.add_argument("--workspace-side", choices=("left", "right"))
    parser.add_argument("--surface-side", choices=("left", "right", "centered"))
    parser.add_argument("--workspace-width-m", type=float)
    parser.add_argument("--y-min", type=float)
    parser.add_argument("--y-max", type=float)
    parser.add_argument(
        "--first-lane-direction",
        choices=("forward", "reverse"),
        default="forward",
    )
    parser.add_argument(
        "--transition-style",
        choices=("auto_internal", "auto-internal", "side-step-reverse-90", "side_step_reverse_90"),
        default="auto_internal",
    )
    parser.add_argument("--robot-width-m", type=float, default=0.18)
    parser.add_argument("--robot-length-m", type=float, default=0.18)
    parser.add_argument("--robot-radius-m", type=float, default=0.14)
    parser.add_argument("--boundary-mode", choices=("none", "strict"), default="strict")
    parser.add_argument(
        "--chassis-boundary-mode",
        choices=("derived_diagnostic", "centerline_corridor", "clean_surface_strict", "operation_polygon"),
        default="derived_diagnostic",
    )
    parser.add_argument(
        "--auto-orient-tool-inside",
        default="true",
        choices=("true", "false"),
    )
    parser.add_argument(
        "--start-policy",
        default="at_A_or_nearest_feasible",
        choices=("at_A_or_nearest_feasible",),
    )
    parser.add_argument(
        "--end-policy",
        default="near_B_or_last_lane_feasible",
        choices=("near_B_or_last_lane_feasible",),
    )
    parser.add_argument(
        "--auto-inset-endpoints",
        default="true",
        choices=("true", "false"),
    )
    parser.add_argument("--boundary-margin-m", type=float, default=0.03)
    parser.add_argument(
        "--turn-clearance-mode",
        choices=("simple", "bounding_circle"),
        default="simple",
    )
    parser.add_argument(
        "--fail-on-boundary-violation",
        default="false",
        choices=("true", "false"),
    )
    parser.add_argument(
        "--allow-uncovered-margins",
        default="true",
        choices=("true", "false"),
    )
    parser.add_argument("--coverage-resolution-m", type=float, default=0.05)
    parser.add_argument("--rotation-sample-deg", type=float, default=5.0)
    parser.add_argument("--translation-sample-m", type=float, default=0.05)
    parser.add_argument(
        "--swept-volume-validation",
        choices=("strict", "simple", "off"),
        default="strict",
    )
    parser.add_argument(
        "--kinematic-model",
        choices=("differential_drive",),
        default="differential_drive",
    )
    parser.add_argument(
        "--emit-geometry-samples",
        default="true",
        choices=("true", "false"),
    )
    parser.add_argument(
        "--emit-separated-previews",
        default="true",
        choices=("true", "false"),
    )
    parser.add_argument(
        "--contamination-mode",
        choices=("strict", "warn", "off"),
        default="off",
    )
    parser.add_argument(
        "--fail-on-contamination-violation",
        default="true",
        choices=("true", "false"),
    )
    parser.add_argument(
        "--allow-start-end-chassis-outside-clean-surface",
        default="true",
        choices=("true", "false"),
    )
    parser.add_argument(
        "--tool-active-during-alignment",
        default="false",
        choices=("true", "false"),
    )
    parser.add_argument(
        "--tool-active-during-transitions",
        default="false",
        choices=("true", "false"),
    )
    parser.add_argument(
        "--tool-active-during-rotation",
        "--tool-active-during-rotations",
        dest="tool_active_during_rotation",
        default="false",
        choices=("true", "false"),
    )
    parser.add_argument(
        "--visualize-inactive-tool-ghost",
        default="true",
        choices=("true", "false"),
    )
    parser.add_argument(
        "--show-transition-envelope",
        default="false",
        choices=("true", "false"),
    )
    parser.add_argument(
        "--tool-active-on-sweep-tracks",
        default="true",
        choices=("true", "false"),
    )
    parser.add_argument(
        "--tool-active-on-connectors",
        default="false",
        choices=("true", "false"),
    )
    parser.add_argument(
        "--force-end-at-B",
        dest="force_end_at_B",
        default="true",
        choices=("true", "false"),
    )
    parser.add_argument(
        "--adjust-spacing-to-end-at-B",
        dest="adjust_spacing_to_end_at_B",
        default="true",
        choices=("true", "false"),
    )
    parser.add_argument(
        "--sweep-axis",
        default="auto",
        choices=("auto", "x", "y"),
    )
    parser.add_argument(
        "--same-step-tool-before-chassis",
        default="false",
        choices=("true", "false"),
    )
    parser.add_argument(
        "--route-order",
        choices=(
            "auto_temporal_safe",
            "bottom_to_top",
            "top_to_bottom",
            "far_to_near",
            "near_to_far",
            "nearest_to_exit",
            "farthest_to_exit",
            "A_to_B_frontier",
            "B_to_A_retreat",
            "wet_frontier_retreat",
            "wet_frontier_safe_boustrophedon",
            "top_to_bottom_then_exit",
            "bottom_to_top_then_exit",
            "dry_frontier_preserving",
            "dry_corridor_preserving",
            "far_side_to_exit",
            "exit_side_to_far",
            "snake_from_A_to_B",
            "reverse_snake_to_B",
            "tool_lift_transitions",
            "clean_from_far_side_to_exit",
            "fixed_serpentine",
        ),
        default="auto_temporal_safe",
    )
    parser.add_argument(
        "--coverage-pattern",
        choices=("boustrophedon_required", "allow_best_effort_single_lane"),
        default="boustrophedon_required",
    )
    parser.add_argument("--min-lane-count", type=_parse_min_lane_count, default="auto")
    parser.add_argument(
        "--forbid-single-lane-success",
        default="true",
        choices=("true", "false"),
    )
    parser.add_argument(
        "--allow-tool-lift",
        default="true",
        choices=("true", "false"),
    )
    parser.add_argument(
        "--reserve-dry-exit-corridor",
        default="true",
        choices=("true", "false"),
    )
    parser.add_argument("--dry-corridor-strategy", default="auto")
    parser.add_argument(
        "--require-start-at-A",
        dest="require_start_at_A",
        default="auto",
        choices=("auto", "true", "false"),
    )
    parser.add_argument(
        "--require-end-at-B",
        dest="require_end_at_B",
        default="auto",
        choices=("auto", "true", "false"),
    )
    parser.add_argument("--max-start-error-m", type=float, default=0.05)
    parser.add_argument("--max-end-error-m", type=float, default=0.05)
    parser.add_argument("--max-end-offset-m", type=float, default=0.30)
    parser.add_argument(
        "--require-exact-start",
        default="false",
        choices=("true", "false"),
    )
    parser.add_argument(
        "--require-exact-end",
        default="false",
        choices=("true", "false"),
    )
    parser.add_argument(
        "--emit-timeline-frames",
        default="true",
        choices=("true", "false"),
    )
    parser.add_argument(
        "--emit-segment-frames",
        default="true",
        choices=("true", "false"),
    )
    parser.add_argument("--timeline-frame-stride", type=int, default=5)
    parser.add_argument("--max-timeline-frames", type=int)
    parser.add_argument(
        "--emit-contamination-previews",
        default="true",
        choices=("true", "false"),
    )
    parser.add_argument(
        "--allow-contamination-best-effort",
        default="false",
        choices=("true", "false"),
    )
    parser.add_argument(
        "--allow-best-effort",
        default="false",
        choices=("true", "false"),
    )
    parser.add_argument(
        "--allow-unreached-B-best-effort",
        dest="allow_unreached_b_best_effort",
        default="false",
        choices=("true", "false"),
    )
    parser.add_argument(
        "--out-dir",
        default="outputs/side_tool_path_preview",
        help="Output directory for side_tool_path.csv, summary.md, and preview.png",
    )
    return parser


def _write_csv(path: Path, poses: Sequence[dict[str, float | int | str | bool]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for pose in poses:
            row = {field: pose[field] for field in CSV_FIELDS}
            row["primitive_type"] = pose.get("movement_primitive_type", pose["primitive_type"])
            writer.writerow(row)


TOOL_SERPENTINE_CSV_FIELDS = (
    "tool_segment_index",
    "tool_segment_id",
    "tool_segment_type",
    "tool_start_x_m",
    "tool_start_y_m",
    "tool_end_x_m",
    "tool_end_y_m",
    "tool_heading_deg",
    "tool_distance_m",
    "tool_active",
    "coverage_contributes",
    "cleaned_area_added_m2",
    "is_ghost_tool_path",
    "associated_primitive_indices",
    "track_index",
    "connector_index",
    "tool_waypoint_label",
    "chassis_segment_id",
    "associated_tool_segment_id",
    "chassis_start_x_m",
    "chassis_start_y_m",
    "chassis_end_x_m",
    "chassis_end_y_m",
    "chassis_heading_deg",
    "chassis_path_derived_from_tool",
    "chassis_feasible_for_tool_path",
    "chassis_infeasible_reason",
    "motor_command_generated",
)


TOOL_SERPENTINE_PRIMITIVE_FIELDS = (
    "primitive_index",
    "primitive_type",
    "distance_m",
    "angle_deg",
    "segment_role",
    "track_index",
    "connector_index",
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


def _write_tool_serpentine_csv(path: Path, preview: dict[str, object]) -> None:
    tool_segments = list(preview["tool_segments"])  # type: ignore[index]
    chassis_segments = list(preview["chassis_segments"])  # type: ignore[index]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=TOOL_SERPENTINE_CSV_FIELDS)
        writer.writeheader()
        for tool, chassis in zip(tool_segments, chassis_segments, strict=True):
            row = {field: "" for field in TOOL_SERPENTINE_CSV_FIELDS}
            row.update(tool)
            row.update(chassis)
            row["motor_command_generated"] = False
            writer.writerow(row)


def _write_tool_serpentine_primitives_csv(path: Path, preview: dict[str, object]) -> None:
    primitive_rows = list(preview["primitive_rows"])  # type: ignore[index]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=TOOL_SERPENTINE_PRIMITIVE_FIELDS)
        writer.writeheader()
        for row in primitive_rows:
            writer.writerow({field: row.get(field, "") for field in TOOL_SERPENTINE_PRIMITIVE_FIELDS})


def _write_tool_serpentine_summary(path: Path, preview: dict[str, object]) -> None:
    summary = dict(preview["summary"])  # type: ignore[arg-type]
    lines = [
        "# Tool-Centered Serpentine Path Preview",
        "",
        "Preview only: no rover commands, no motor output.",
        "",
    ]
    for key in (
        "planner_mode",
        "workspace_mode",
        "A_corner_role",
        "B_corner_role",
        "planner_primary_path",
        "chassis_path_role",
        "tool_side",
        "robot_width_m",
        "robot_length_m",
        "tool_width_m",
        "tool_length_m",
        "tool_lateral_offset_m",
        "requested_step_spacing_m",
        "actual_spacing_values_m",
        "derived_track_count",
        "final_partial_spacing_m",
        "tool_active_on_sweep_tracks",
        "tool_active_on_connectors",
        "tool_path_continuous",
        "tool_path_starts_at_A",
        "tool_path_ends_at_B",
        "active_paint_path_uses_only_sweep_tracks",
        "connector_painting_disabled",
        "chassis_path_derived_from_tool",
        "primitive_sequence_valid",
        "coverage_ratio",
        "motor_command_generated",
        "path_preview_only",
    ):
        lines.append(f"- {key}: `{summary.get(key)}`")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_tool_serpentine_route_sequence(path: Path, preview: dict[str, object]) -> None:
    tool_segments = list(preview["tool_segments"])  # type: ignore[index]
    chassis_segments = list(preview["chassis_segments"])  # type: ignore[index]
    primitive_rows = list(preview["primitive_rows"])  # type: ignore[index]
    lines = [
        "# Tool-Centered Serpentine Route Sequence",
        "",
        "Preview only: no rover commands, no motor output.",
        "",
        "## Tool-space route",
        "",
    ]
    for segment in tool_segments:
        if segment["tool_segment_type"] == "tool_sweep_track":
            label = f"Track {segment['track_index']}"
        else:
            label = f"Connector {segment['connector_index']}->{int(segment['connector_index']) + 1}"
        lines.append(
            "- {label}: {sid} ({sx:.3f},{sy:.3f}) -> ({ex:.3f},{ey:.3f}), heading={heading:.1f} deg, distance={distance:.3f} m, tool_active={active}, coverage_contributes={coverage}, ghost={ghost}".format(
                sid=segment["tool_segment_id"],
                label=label,
                sx=float(segment["tool_start_x_m"]),
                sy=float(segment["tool_start_y_m"]),
                ex=float(segment["tool_end_x_m"]),
                ey=float(segment["tool_end_y_m"]),
                heading=float(segment["tool_heading_deg"]),
                distance=float(segment["tool_distance_m"]),
                active=segment["tool_active"],
                coverage=segment["coverage_contributes"],
                ghost=segment["is_ghost_tool_path"],
            )
        )
    lines.extend(["", "## Derived chassis route", ""])
    for segment in chassis_segments:
        lines.append(
            "- For {tool}: chassis ({sx:.3f},{sy:.3f}) -> ({ex:.3f},{ey:.3f}), heading={heading:.1f} deg, derived_from_tool={derived}".format(
                tool=segment["associated_tool_segment_id"],
                sx=float(segment["chassis_start_x_m"]),
                sy=float(segment["chassis_start_y_m"]),
                ex=float(segment["chassis_end_x_m"]),
                ey=float(segment["chassis_end_y_m"]),
                heading=float(segment["chassis_heading_deg"]),
                derived=segment["chassis_path_derived_from_tool"],
            )
        )
    lines.extend(["", "## Primitive sequence", ""])
    for primitive in primitive_rows:
        if primitive["primitive_type"] in {"move_forward", "move_backward"}:
            command = f"{primitive['primitive_type']}({float(primitive['distance_m']):.3f} m)"
        else:
            command = f"{primitive['primitive_type']}({float(primitive['angle_deg']):.1f} deg)"
        lines.append(
            "- P{idx:03d} {command}, role={role}, associated_tool_segment_id={tool}, tool_active={active}, coverage_contributes={coverage}, motor_command_generated=False".format(
                idx=int(primitive["primitive_index"]),
                command=command,
                role=primitive["segment_role"],
                tool=primitive["associated_tool_segment_id"],
                active=primitive["tool_active"],
                coverage=primitive["coverage_contributes"],
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_tool_serpentine_previews(out_dir: Path, preview: dict[str, object]) -> dict[str, Path | None]:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle
    except ImportError:
        return {
            "preview_tool_path_primary.png": None,
            "preview_chassis_derived_from_tool.png": None,
            "preview_primitive_sequence.png": None,
            "preview_tool_coverage_only.png": None,
        }

    info = preview["info"]  # type: ignore[index]
    tool_segments = list(preview["tool_segments"])  # type: ignore[index]
    chassis_segments = list(preview["chassis_segments"])  # type: ignore[index]
    primitive_rows = list(preview["primitive_rows"])  # type: ignore[index]
    summary = dict(preview["summary"])  # type: ignore[arg-type]
    x_min = float(info.world_x_min_m)
    x_max = float(info.world_x_max_m)
    y_min = float(info.world_y_min_m)
    y_max = float(info.world_y_max_m)

    def setup(title: str):
        fig, ax = plt.subplots(figsize=(8, 5))
        ax.add_patch(Rectangle((x_min, y_min), x_max - x_min, y_max - y_min, fill=False, edgecolor="black", linewidth=1.4))
        ax.scatter([float(info.a_x_m)], [float(info.a_y_m)], color="black", s=40)
        ax.scatter([float(info.b_x_m)], [float(info.b_y_m)], color="black", s=40)
        ax.annotate("A top-left", (float(info.a_x_m), float(info.a_y_m)), fontsize=9, fontweight="bold")
        ax.annotate("B bottom-right", (float(info.b_x_m), float(info.b_y_m)), fontsize=9, fontweight="bold")
        ax.set_title(title)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.axis("equal")
        ax.grid(True, linestyle="--", alpha=0.3)
        ax.text(
            0.01,
            0.01,
            "Preview only: no rover commands, no motor output",
            transform=ax.transAxes,
            fontsize=8,
            va="bottom",
            bbox={"facecolor": "white", "alpha": 0.78, "edgecolor": "none"},
        )
        return fig, ax

    def world(point_x: float, point_y: float) -> tuple[float, float]:
        return x_min + point_x, y_min + point_y

    def draw_tool_path(ax, *, labels: bool, arrows: bool) -> None:
        points: list[tuple[float, float]] = []
        for index, segment in enumerate(tool_segments):
            start = world(float(segment["tool_start_x_m"]), float(segment["tool_start_y_m"]))
            end = world(float(segment["tool_end_x_m"]), float(segment["tool_end_y_m"]))
            if index == 0:
                points.append(start)
            points.append(end)
            active = bool(segment["tool_active"])
            if segment["tool_segment_type"] == "tool_sweep_track":
                color = "tab:orange" if active else "0.65"
                linestyle = "-" if active else "--"
                linewidth = 2.5 if active else 1.4
            else:
                color = "0.55" if not active else "goldenrod"
                linestyle = "--" if not active else "-"
                linewidth = 1.5 if not active else 2.0
            if arrows:
                ax.annotate(
                    "",
                    xy=end,
                    xytext=start,
                    arrowprops={"arrowstyle": "->", "color": color, "linewidth": linewidth},
                )
            else:
                ax.plot([start[0], end[0]], [start[1], end[1]], color=color, linestyle=linestyle, linewidth=linewidth)
            if labels:
                label = (
                    f"Track {segment['track_index']}"
                    if segment["tool_segment_type"] == "tool_sweep_track"
                    else f"Connector {segment['connector_index']}->{int(segment['connector_index']) + 1}"
                )
                ax.text(
                    (start[0] + end[0]) / 2.0,
                    (start[1] + end[1]) / 2.0,
                    label,
                    fontsize=7,
                    color=color,
                    bbox={"facecolor": "white", "alpha": 0.65, "edgecolor": "none"},
                )
        if points:
            ax.plot([point[0] for point in points], [point[1] for point in points], color="tab:orange", linewidth=1.0, alpha=0.35)

    outputs: dict[str, Path | None] = {}

    fig, ax = setup("Tool Path Primary")
    draw_tool_path(ax, labels=True, arrows=False)
    ax.text(
        0.01,
        0.98,
        (
            "planner_primary_path=tool_center_path\n"
            f"tool_path_continuous={summary['tool_path_continuous']}\n"
            f"tracks={summary['tool_track_count']} connectors={summary['tool_connector_count']}\n"
            "solid=active sweep, dashed=inactive connector"
        ),
        transform=ax.transAxes,
        va="top",
        fontsize=8,
        bbox={"facecolor": "white", "alpha": 0.82, "edgecolor": "none"},
    )
    fig.tight_layout()
    path = out_dir / "preview_tool_path_primary.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    outputs["preview_tool_path_primary.png"] = path

    fig, ax = setup("Chassis Derived From Tool")
    draw_tool_path(ax, labels=False, arrows=False)
    for segment in chassis_segments:
        start = world(float(segment["chassis_start_x_m"]), float(segment["chassis_start_y_m"]))
        end = world(float(segment["chassis_end_x_m"]), float(segment["chassis_end_y_m"]))
        ax.plot([start[0], end[0]], [start[1], end[1]], color="tab:blue", linewidth=1.0, alpha=0.65)
        tool_segment = next(
            item for item in tool_segments if item["tool_segment_id"] == segment["associated_tool_segment_id"]
        )
        tool_start = world(float(tool_segment["tool_start_x_m"]), float(tool_segment["tool_start_y_m"]))
        ax.plot([start[0], tool_start[0]], [start[1], tool_start[1]], color="gray", linewidth=0.5, alpha=0.45)
    fig.tight_layout()
    path = out_dir / "preview_chassis_derived_from_tool.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    outputs["preview_chassis_derived_from_tool.png"] = path

    fig, ax = setup("Primitive Sequence")
    for primitive in primitive_rows:
        start = world(float(primitive["start_x_m"]), float(primitive["start_y_m"]))
        end = world(float(primitive["end_x_m"]), float(primitive["end_y_m"]))
        ptype = str(primitive["primitive_type"])
        if ptype in {"move_forward", "move_backward"}:
            ax.annotate("", xy=end, xytext=start, arrowprops={"arrowstyle": "->", "color": "tab:blue", "linewidth": 1.0})
        else:
            marker = "$↺$" if ptype == "rotate_left" else "$↻$"
            ax.scatter([start[0]], [start[1]], color="tab:purple", marker=marker, s=70)
        ax.text(start[0], start[1], f"P{int(primitive['primitive_index']):03d}", fontsize=6)
    fig.tight_layout()
    path = out_dir / "preview_primitive_sequence.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    outputs["preview_primitive_sequence.png"] = path

    fig, ax = setup("Tool Coverage Only")
    for segment in tool_segments:
        if segment["tool_segment_type"] != "tool_sweep_track":
            continue
        if not bool(segment["coverage_contributes"]):
            continue
        y = y_min + float(segment["tool_start_y_m"])
        ax.fill_between(
            [x_min, x_max],
            max(y_min, y - 0.15),
            min(y_max, y + 0.15),
            color="tab:orange",
            alpha=0.18,
        )
    draw_tool_path(ax, labels=False, arrows=False)
    ax.text(
        0.01,
        0.98,
        "Coverage uses active sweep tracks only\nconnectors are inactive/ghost by default",
        transform=ax.transAxes,
        va="top",
        fontsize=8,
        bbox={"facecolor": "white", "alpha": 0.82, "edgecolor": "none"},
    )
    fig.tight_layout()
    path = out_dir / "preview_tool_coverage_only.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    outputs["preview_tool_coverage_only.png"] = path
    return outputs


def _write_tool_serpentine_timeline_frames(out_dir: Path, preview: dict[str, object]) -> int:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle
    except ImportError:
        return 0
    info = preview["info"]  # type: ignore[index]
    tool_segments = list(preview["tool_segments"])  # type: ignore[index]
    frame_dir = out_dir / "timeline_frames"
    frame_dir.mkdir(parents=True, exist_ok=True)
    x_min = float(info.world_x_min_m)
    x_max = float(info.world_x_max_m)
    y_min = float(info.world_y_min_m)
    y_max = float(info.world_y_max_m)
    for index in range(len(tool_segments) + 1):
        fig, ax = plt.subplots(figsize=(6, 4))
        ax.add_patch(Rectangle((x_min, y_min), x_max - x_min, y_max - y_min, fill=False, edgecolor="black"))
        for segment in tool_segments[:index]:
            sx = x_min + float(segment["tool_start_x_m"])
            sy = y_min + float(segment["tool_start_y_m"])
            ex = x_min + float(segment["tool_end_x_m"])
            ey = y_min + float(segment["tool_end_y_m"])
            active = bool(segment["tool_active"])
            ax.plot(
                [sx, ex],
                [sy, ey],
                color="tab:orange" if active else "0.6",
                linestyle="-" if active else "--",
                linewidth=2.0 if active else 1.2,
            )
        ax.set_title(f"Tool path progress frame {index:04d}")
        ax.axis("equal")
        ax.grid(True, linestyle="--", alpha=0.3)
        ax.text(0.01, 0.01, "motor_command_generated=False", transform=ax.transAxes, fontsize=8)
        fig.tight_layout()
        fig.savefig(frame_dir / f"frame_{index:04d}.png", dpi=130)
        plt.close(fig)
    return len(tool_segments) + 1


def _write_tool_serpentine_segment_frames(out_dir: Path, preview: dict[str, object]) -> int:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle
    except ImportError:
        return 0
    info = preview["info"]  # type: ignore[index]
    tool_segments = list(preview["tool_segments"])  # type: ignore[index]
    segment_dir = out_dir / "segment_frames"
    segment_dir.mkdir(parents=True, exist_ok=True)
    x_min = float(info.world_x_min_m)
    x_max = float(info.world_x_max_m)
    y_min = float(info.world_y_min_m)
    y_max = float(info.world_y_max_m)
    for index, segment in enumerate(tool_segments):
        fig, ax = plt.subplots(figsize=(6, 4))
        ax.add_patch(Rectangle((x_min, y_min), x_max - x_min, y_max - y_min, fill=False, edgecolor="black"))
        sx = x_min + float(segment["tool_start_x_m"])
        sy = y_min + float(segment["tool_start_y_m"])
        ex = x_min + float(segment["tool_end_x_m"])
        ey = y_min + float(segment["tool_end_y_m"])
        active = bool(segment["tool_active"])
        ax.annotate(
            "",
            xy=(ex, ey),
            xytext=(sx, sy),
            arrowprops={
                "arrowstyle": "->",
                "color": "tab:orange" if active else "0.6",
                "linewidth": 2.2 if active else 1.4,
                "linestyle": "-" if active else "--",
            },
        )
        ax.set_title(f"{segment['tool_segment_id']} {segment['tool_segment_type']} tool_active={active}")
        ax.axis("equal")
        ax.grid(True, linestyle="--", alpha=0.3)
        fig.tight_layout()
        fig.savefig(segment_dir / f"segment_{index:03d}_{segment['tool_segment_id']}.png", dpi=130)
        plt.close(fig)
    return len(tool_segments)


def _run_tool_serpentine_preview_output(
    *,
    config: SideToolPlanConfig,
    out_dir: Path,
    tool_csv_name: str,
    emit_previews: bool,
    emit_timeline_frames: bool = False,
    emit_segment_frames: bool = False,
) -> int:
    preview = generate_tool_serpentine_preview(config)
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / tool_csv_name
    primitive_csv_path = out_dir / "primitive_sequence.csv"
    summary_path = out_dir / "summary.md"
    route_sequence_path = out_dir / "preview_route_sequence.md"
    _write_tool_serpentine_csv(csv_path, preview)
    _write_tool_serpentine_primitives_csv(primitive_csv_path, preview)
    _write_tool_serpentine_summary(summary_path, preview)
    _write_tool_serpentine_route_sequence(route_sequence_path, preview)
    preview_paths = _write_tool_serpentine_previews(out_dir, preview) if emit_previews else {}
    timeline_count = (
        _write_tool_serpentine_timeline_frames(out_dir, preview)
        if emit_timeline_frames
        else 0
    )
    segment_count = (
        _write_tool_serpentine_segment_frames(out_dir, preview)
        if emit_segment_frames
        else 0
    )
    summary = preview["summary"]  # type: ignore[index]
    print("Simple tool-centered serpentine path preview generated.")
    print("Preview only: no HC-12 commands, no rover motor commands, no firmware upload.")
    print(f"Tool path CSV: {csv_path}")
    print(f"Primitive sequence CSV: {primitive_csv_path}")
    print(f"Summary: {summary_path}")
    print(f"Route sequence markdown: {route_sequence_path}")
    for label, path in preview_paths.items():
        print(f"{label}: {path if path is not None else 'skipped'}")
    if timeline_count:
        print(f"Timeline frames: {timeline_count} files in {out_dir / 'timeline_frames'}")
    if segment_count:
        print(f"Segment frames: {segment_count} files in {out_dir / 'segment_frames'}")
    print(f"planner_mode: {summary['planner_mode']}")
    print(f"tool_side: {summary['tool_side']}")
    print(f"tool_active_on_connectors: {summary['tool_active_on_connectors']}")
    print(f"connector_painting_disabled: {summary['connector_painting_disabled']}")
    print(f"tool_path_starts_at_A: {summary['tool_path_starts_at_A']}")
    print(f"tool_path_ends_at_B: {summary['tool_path_ends_at_B']}")
    print(f"tool_path_continuous: {summary['tool_path_continuous']}")
    print(f"tool_track_count: {summary['tool_track_count']}")
    print(f"tool_connector_count: {summary['tool_connector_count']}")
    print(f"primitive_sequence_valid: {summary['primitive_sequence_valid']}")
    print(f"primitive_pattern_valid: {summary['primitive_pattern_valid']}")
    print(f"motor_command_generated: {summary['motor_command_generated']}")
    return 0


def _cell_bounds(cell: int, nx: int, cell_width_m: float, cell_height_m: float) -> tuple[float, float, float, float]:
    ix = cell % nx
    iy = cell // nx
    x_min = ix * cell_width_m
    x_max = x_min + cell_width_m
    y_min = iy * cell_height_m
    y_max = y_min + cell_height_m
    return x_min, x_max, y_min, y_max


def _contamination_samples(
    *,
    config: SideToolPlanConfig,
    poses: Sequence[dict[str, float | int | str | bool]],
) -> list[dict[str, object]]:
    info = _workspace_info(config)
    grid = _coverage_grid(info, config.coverage_resolution_m)
    samples = geometry_samples_for_path(config, list(poses))
    pose_lookup = {str(pose["segment_id"]): pose for pose in poses}
    cleaned_at_step: dict[int, int] = {}
    rows: list[dict[str, object]] = []
    for time_step, sample in enumerate(samples):
        pose = pose_lookup[str(sample.segment_id)]
        chassis_cells = _cells_for_polygon(info, grid, sample.chassis_polygon)
        tool_cells = _cells_for_polygon(info, grid, sample.tool_polygon)
        prior_cleaned = {
            cell
            for cell in chassis_cells
            if cell in cleaned_at_step and cleaned_at_step[cell] < time_step
        }
        if str(pose["segment_id"]) == "route_start_A":
            tool_active = bool(config.tool_active_during_alignment)
        elif sample.sample_type == "rotation":
            tool_active = bool(config.tool_active_during_rotation)
        elif pose["primitive_type"] == "transition" or str(pose["segment_id"]) == "route_end_B":
            tool_active = bool(config.tool_active_during_transitions)
        else:
            tool_active = True
        same_step_overlap = (
            set(chassis_cells & tool_cells)
            if tool_active and not config.same_step_tool_before_chassis
            else set()
        )
        current_reclean = {cell for cell in tool_cells if cell in cleaned_at_step}
        if tool_active:
            for cell in tool_cells:
                cleaned_at_step.setdefault(cell, time_step)
        reason = "OK"
        violation_cells = set(prior_cleaned) | same_step_overlap
        if violation_cells:
            reason = (
                "CHASSIS_ON_PRIOR_TOOL_SWEPT_AREA+CHASSIS_ON_WET_AREA_DURING_TRANSITION"
                if prior_cleaned and pose["primitive_type"] == "transition"
                else "CHASSIS_ON_PRIOR_TOOL_SWEPT_AREA+CHASSIS_ON_WET_AREA_DURING_LANE"
                if prior_cleaned
                else "CHASSIS_ON_CURRENT_TOOL_SWEPT_AREA"
            )
        rows.append(
            {
                "time_step": time_step,
                "sample": sample,
                "pose": pose,
                "chassis_cells": chassis_cells,
                "tool_cells": tool_cells,
                "prior_cleaned": frozenset(violation_cells),
                "current_reclean": current_reclean,
                "cleaned_cells": frozenset(cleaned_at_step),
                "tool_active": tool_active,
                "violation_reason": reason,
                "grid_nx": grid.nx,
                "grid_ny": grid.ny,
                "cell_width_m": grid.cell_width_m,
                "cell_height_m": grid.cell_height_m,
                "cell_area_m2": grid.cell_area_m2,
            }
        )
    return rows


def _frame_path_by_step(frame_paths: dict[int, Path] | None) -> dict[int, str]:
    if not frame_paths:
        return {}
    return {step: str(path) for step, path in frame_paths.items()}


def _write_geometry_samples_csv(
    *,
    path: Path,
    config: SideToolPlanConfig,
    poses: Sequence[dict[str, float | int | str | bool]],
    frame_paths: dict[int, Path] | None = None,
) -> None:
    rows = _contamination_samples(config=config, poses=poses)
    frame_lookup = _frame_path_by_step(frame_paths)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=GEOMETRY_SAMPLE_FIELDS)
        writer.writeheader()
        for row in rows:
            sample = row["sample"]
            pose = row["pose"]
            prior_cleaned = row["prior_cleaned"]
            current_reclean = row["current_reclean"]
            writer.writerow(
                {
                    "time_step": row["time_step"],
                    "sample_index": sample.sample_index,
                    "segment_id": sample.segment_id,
                    "segment_type": pose["segment_type"],
                    "lane_index": pose["lane_index"],
                    "primitive_type": pose["primitive_type"],
                    "differential_drive_feasible": pose["differential_drive_feasible"],
                    "sample_type": sample.sample_type,
                    "x_m": f"{sample.x_m:.6f}",
                    "y_m": f"{sample.y_m:.6f}",
                    "heading_deg": f"{sample.heading_deg:.6f}",
                    "chassis_polygon_json": json.dumps(sample.chassis_polygon),
                    "tool_polygon_json": json.dumps(sample.tool_polygon),
                    "combined_bbox_x_min_m": f"{sample.combined_bbox_x_min_m:.6f}",
                    "combined_bbox_x_max_m": f"{sample.combined_bbox_x_max_m:.6f}",
                    "combined_bbox_y_min_m": f"{sample.combined_bbox_y_min_m:.6f}",
                    "combined_bbox_y_max_m": f"{sample.combined_bbox_y_max_m:.6f}",
                    "inside_workspace": sample.inside_workspace,
                    "chassis_inside_workspace": sample.chassis_inside_workspace,
                    "tool_inside_workspace": sample.tool_inside_workspace,
                    "violation_reason": sample.violation_reason,
                    "chassis_cells_count": len(row["chassis_cells"]),
                    "tool_cells_count": len(row["tool_cells"]),
                    "prior_cleaned_chassis_overlap_cells": len(prior_cleaned),
                    "prior_cleaned_chassis_overlap_area_m2": f"{len(prior_cleaned) * float(row['cell_area_m2']):.6f}",
                    "current_tool_reclean_cells": len(current_reclean),
                    "current_tool_reclean_area_m2": f"{len(current_reclean) * float(row['cell_area_m2']):.6f}",
                    "contamination_violation": bool(prior_cleaned),
                    "contamination_violation_reason": row["violation_reason"],
                    "tool_active": row["tool_active"],
                    "frame_path": frame_lookup.get(int(row["time_step"]), ""),
                    "motor_command_generated": False,
                }
            )


def _write_contamination_events_csv(path: Path, analysis: dict[str, object]) -> None:
    events = list(analysis["events"])
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CONTAMINATION_EVENT_FIELDS)
        writer.writeheader()
        for event in events:
            writer.writerow(
                {
                    "event_index": event.event_index,
                    "time_step": event.time_step,
                    "segment_id": event.segment_id,
                    "segment_type": event.segment_type,
                    "lane_index": event.lane_index,
                    "x_m": f"{event.x_m:.6f}",
                    "y_m": f"{event.y_m:.6f}",
                    "heading_deg": f"{event.heading_deg:.6f}",
                    "overlap_area_m2": f"{event.overlap_area_m2:.6f}",
                    "affected_cell_count": len(event.overlap_cells),
                    "violation_reason": event.violation_reason,
                    "severity": event.severity,
                }
            )


def _write_strategy_diagnostics_csv(path: Path, rows: Sequence[dict[str, object]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=STRATEGY_DIAGNOSTIC_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in STRATEGY_DIAGNOSTIC_FIELDS})


def _write_strategy_diagnostics_md(path: Path, rows: Sequence[dict[str, object]]) -> Path:
    lines = [
        "# Strategy Diagnostics",
        "",
        "Offline preview only: no rover commands, no motor output.",
        "",
        "| strategy | lanes | reaches_B | contamination | coverage_ratio | rejection |",
        "|---|---:|---|---:|---:|---|",
    ]
    for row in rows:
        lines.append(
            "| {strategy_name} | {selected_lane_count} | {reaches_B} | "
            "{contamination_violation_count} | {coverage_ratio:.3f} | {reason_for_rejection} |".format(
                strategy_name=row.get("strategy_name", ""),
                selected_lane_count=row.get("selected_lane_count", ""),
                reaches_B=row.get("reaches_B", ""),
                contamination_violation_count=row.get("contamination_violation_count", ""),
                coverage_ratio=float(row.get("coverage_ratio", 0.0)),
                reason_for_rejection=row.get("reason_for_rejection", ""),
            )
        )
    lines.extend(
        [
            "",
            "Potential parameter tradeoffs to investigate:",
            "",
            "- Change `tool_side` or start/end diagonal corners.",
            "- Adjust `lane_spacing_m` or `tool_width_m` assumptions.",
            "- Reserve a larger dry exit corridor or accept an unpainted exit corridor.",
            "- Change cleaning direction or workspace geometry.",
            "- If strict wet-safe full coverage is geometrically impossible, keep `route_feasible_for_physical_preview=false`.",
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def _write_timeline_state_csv(
    *,
    path: Path,
    analysis: dict[str, object],
    frame_paths: dict[int, Path] | None,
    poses: Sequence[dict[str, float | int | str | bool]] = (),
) -> None:
    frame_lookup = _frame_path_by_step(frame_paths)
    pose_lookup = {str(pose["segment_id"]): pose for pose in poses}
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=TIMELINE_STATE_FIELDS)
        writer.writeheader()
        for state in list(analysis["timeline"]):
            writer.writerow(
                {
                    "time_step": state.time_step,
                    "segment_id": state.segment_id,
                    "primitive_type": state.primitive_type,
                    "segment_type": state.segment_type,
                    "lane_index": state.lane_index,
                    "chassis_x_m": f"{state.x_m:.6f}",
                    "chassis_y_m": f"{state.y_m:.6f}",
                    "chassis_heading_deg": f"{state.heading_deg:.6f}",
                    "heading_deg": f"{state.heading_deg:.6f}",
                    "tool_x_m": f"{state.tool_x_m:.6f}",
                    "tool_y_m": f"{state.tool_y_m:.6f}",
                    "tool_heading_deg": f"{state.heading_deg:.6f}",
                    "chassis_cells_count": state.chassis_cells_count,
                    "tool_cells_count": state.tool_cells_count,
                    "cleaned_area_m2": f"{state.cleaned_area_m2:.6f}",
                    "coverage_ratio_so_far": f"{state.coverage_ratio_so_far:.6f}",
                    "chassis_after_cleaned_area_m2": f"{state.prior_cleaned_chassis_overlap_area_m2:.6f}",
                    "wet_area_m2": f"{state.cleaned_area_m2:.6f}",
                    "contamination_violation_count_so_far": state.contamination_violation_count_so_far,
                    "first_cleaning_heading_aligned": pose_lookup.get(state.segment_id, {}).get("first_cleaning_heading_aligned", ""),
                    "tool_active": state.tool_active,
                    "route_to_B_remaining_m": f"{state.route_to_B_remaining_m:.6f}",
                    "violation_reason": state.violation_reason,
                    "motor_command_generated": False,
                    "frame_path": frame_lookup.get(state.time_step, ""),
                }
            )


def _write_swept_volume_json(
    *,
    path: Path,
    config: SideToolPlanConfig,
    poses: Sequence[dict[str, float | int | str | bool]],
) -> None:
    payload = swept_volume_summary(config, list(poses))
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _timeline_frame_steps(total_steps: int, stride: int, max_frames: int | None) -> list[int]:
    if total_steps <= 0:
        return []
    stride = max(1, stride)
    steps = list(range(0, total_steps, stride))
    if steps[-1] != total_steps - 1:
        steps.append(total_steps - 1)
    if max_frames is not None and max_frames > 0 and len(steps) > max_frames:
        if max_frames == 1:
            return [steps[-1]]
        picked = []
        for i in range(max_frames):
            index = round(i * (len(steps) - 1) / (max_frames - 1))
            picked.append(steps[index])
        steps = sorted(set(picked))
    return steps


def _draw_workspace_base(ax, config: SideToolPlanConfig, title: str) -> None:
    polygon = boundary_polygon(config)
    closed = polygon + [polygon[0]]
    ax.plot(
        [point[0] for point in closed],
        [point[1] for point in closed],
        color="black",
        linewidth=1.3,
        label="A/B boundary",
    )
    if config.a_x_m is not None and config.a_y_m is not None:
        ax.scatter([config.a_x_m], [config.a_y_m], color="black", s=28)
        ax.annotate("A", (config.a_x_m, config.a_y_m), fontsize=9, fontweight="bold")
    if config.b_x_m is not None and config.b_y_m is not None:
        ax.scatter([config.b_x_m], [config.b_y_m], color="black", s=28)
        ax.annotate("B", (config.b_x_m, config.b_y_m), fontsize=9, fontweight="bold")
    if (
        config.workspace_mode in {"ab_diagonal_center", "diagonal_ab"}
        and config.a_x_m is not None
        and config.a_y_m is not None
        and config.b_x_m is not None
        and config.b_y_m is not None
    ):
        ax.plot(
            [config.a_x_m, config.b_x_m],
            [config.a_y_m, config.b_y_m],
            color="black",
            linestyle="--",
            linewidth=0.9,
            alpha=0.45,
            label="A-B diagonal",
        )
    ax.set_title(title)
    ax.set_xlabel("workspace x (m)")
    ax.set_ylabel("workspace y (m)")
    ax.grid(True, linestyle="--", alpha=0.25)
    ax.axis("equal")


def _draw_cells(ax, cells: set[int] | frozenset[int], *, nx: int, cell_width_m: float, cell_height_m: float, color: str, alpha: float, label: str | None = None) -> None:
    label_used = False
    from matplotlib.patches import Rectangle

    for cell in cells:
        x_min, x_max, y_min, y_max = _cell_bounds(cell, nx, cell_width_m, cell_height_m)
        ax.add_patch(
            Rectangle(
                (x_min, y_min),
                x_max - x_min,
                y_max - y_min,
                facecolor=color,
                edgecolor="none",
                alpha=alpha,
                label=label if label and not label_used else None,
            )
        )
        label_used = True


def _write_timeline_index_html(path: Path, frame_paths: dict[int, Path]) -> Path:
    lines = [
        "<!doctype html>",
        "<html><head><meta charset=\"utf-8\"><title>Side Tool Timeline</title></head><body>",
        "<h1>Side Tool Contamination Timeline</h1>",
        "<p>Preview only: no rover commands, no motor output.</p>",
    ]
    for step, frame_path in sorted(frame_paths.items()):
        rel = frame_path.relative_to(path.parent)
        lines.append(f"<h2>time_step={step}</h2><img src=\"{rel}\" style=\"max-width: 100%;\">")
    lines.append("</body></html>")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def _write_timeline_frames(
    *,
    out_dir: Path,
    config: SideToolPlanConfig,
    poses: Sequence[dict[str, float | int | str | bool]],
    stride: int,
    max_frames: int | None,
) -> tuple[dict[int, Path], Path | None, Path | None]:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Polygon as PolygonPatch
    except ImportError:
        return {}, None, None

    rows = _contamination_samples(config=config, poses=poses)
    steps = _timeline_frame_steps(len(rows), stride, max_frames)
    step_set = set(steps)
    frame_dir = out_dir / "timeline_frames"
    frame_dir.mkdir(parents=True, exist_ok=True)
    frame_paths: dict[int, Path] = {}
    chassis_x: list[float] = []
    chassis_y: list[float] = []
    tool_x: list[float] = []
    tool_y: list[float] = []

    for row in rows:
        sample = row["sample"]
        pose = row["pose"]
        chassis_x.append(sample.x_m)
        chassis_y.append(sample.y_m)
        tool_center_x = sum(point[0] for point in sample.tool_polygon) / len(sample.tool_polygon)
        tool_center_y = sum(point[1] for point in sample.tool_polygon) / len(sample.tool_polygon)
        tool_x.append(tool_center_x)
        tool_y.append(tool_center_y)
        time_step = int(row["time_step"])
        if time_step not in step_set:
            continue
        fig, ax = plt.subplots(figsize=(7, 4.5))
        _draw_workspace_base(ax, config, f"Temporal Contact: step {time_step}")
        _draw_cells(
            ax,
            row["cleaned_cells"],
            nx=int(row["grid_nx"]),
            cell_width_m=float(row["cell_width_m"]),
            cell_height_m=float(row["cell_height_m"]),
            color="tab:orange",
            alpha=0.22,
            label="cleaned/wet so far",
        )
        if row["prior_cleaned"]:
            _draw_cells(
                ax,
                row["prior_cleaned"],
                nx=int(row["grid_nx"]),
                cell_width_m=float(row["cell_width_m"]),
                cell_height_m=float(row["cell_height_m"]),
                color="red",
                alpha=0.65,
                label="chassis on prior tool-swept",
            )
        ax.plot(chassis_x, chassis_y, color="tab:blue", linewidth=0.9, alpha=0.5, label="chassis path so far")
        ax.plot(tool_x, tool_y, color="tab:orange", linewidth=1.2, alpha=0.85, label="tool path so far")
        ax.add_patch(
            PolygonPatch(
                sample.chassis_polygon,
                closed=True,
                facecolor="tab:blue",
                edgecolor="tab:blue",
                alpha=0.30,
                label="current chassis",
            )
        )
        ax.add_patch(
            PolygonPatch(
                sample.tool_polygon,
                closed=True,
                facecolor="tab:orange",
                edgecolor="tab:orange",
                alpha=0.38,
                label="current tool",
            )
        )
        ax.text(
            0.01,
            0.01,
            (
                f"segment={pose['segment_id']} {pose['segment_type']} lane={pose['lane_index']}\n"
                f"cleaned_area_m2={len(row['cleaned_cells']) * float(row['cell_area_m2']):.3f} "
                f"violations_so_far={'YES' if row['prior_cleaned'] else 'no'}\n"
                "motor_command_generated=False"
            ),
            transform=ax.transAxes,
            fontsize=8,
            va="bottom",
            bbox={"facecolor": "white", "alpha": 0.78, "edgecolor": "none"},
        )
        ax.legend(loc="upper right", fontsize=7)
        fig.tight_layout()
        frame_path = frame_dir / f"frame_{len(frame_paths):04d}.png"
        fig.savefig(frame_path, dpi=135)
        plt.close(fig)
        frame_paths[time_step] = frame_path

    contact_sheet_path: Path | None = None
    if frame_paths:
        preview_steps = list(sorted(frame_paths))[:12]
        cols = min(4, len(preview_steps))
        rows_count = (len(preview_steps) + cols - 1) // cols
        fig, axes = plt.subplots(rows_count, cols, figsize=(cols * 3.2, rows_count * 2.3))
        axes_list = list(axes.flat) if hasattr(axes, "flat") else [axes]
        for ax, step in zip(axes_list, preview_steps):
            row = rows[step]
            sample = row["sample"]
            _draw_workspace_base(ax, config, f"step {step}")
            _draw_cells(
                ax,
                row["cleaned_cells"],
                nx=int(row["grid_nx"]),
                cell_width_m=float(row["cell_width_m"]),
                cell_height_m=float(row["cell_height_m"]),
                color="tab:orange",
                alpha=0.22,
            )
            if row["prior_cleaned"]:
                _draw_cells(
                    ax,
                    row["prior_cleaned"],
                    nx=int(row["grid_nx"]),
                    cell_width_m=float(row["cell_width_m"]),
                    cell_height_m=float(row["cell_height_m"]),
                    color="red",
                    alpha=0.65,
                )
            ax.add_patch(PolygonPatch(sample.chassis_polygon, closed=True, facecolor="tab:blue", alpha=0.32))
            ax.add_patch(PolygonPatch(sample.tool_polygon, closed=True, facecolor="tab:orange", alpha=0.38))
            ax.set_xticks([])
            ax.set_yticks([])
        for ax in axes_list[len(preview_steps):]:
            ax.axis("off")
        fig.suptitle("Timeline Contact Sheet: preview only, no motor output", fontsize=11)
        fig.tight_layout()
        contact_sheet_path = out_dir / "preview_timeline_contact_sheet.png"
        fig.savefig(contact_sheet_path, dpi=140)
        plt.close(fig)

    html_path = _write_timeline_index_html(out_dir / "preview_timeline_index.html", frame_paths)
    return frame_paths, contact_sheet_path, html_path


def _write_contamination_previews(
    *,
    out_dir: Path,
    config: SideToolPlanConfig,
    poses: Sequence[dict[str, float | int | str | bool]],
    analysis: dict[str, object],
) -> dict[str, Path | None]:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return {
            "preview_contamination_timeline.png": None,
            "preview_temporal_layers.png": None,
        }

    rows = _contamination_samples(config=config, poses=poses)
    if not rows:
        return {
            "preview_contamination_timeline.png": None,
            "preview_temporal_layers.png": None,
        }
    nx = int(rows[0]["grid_nx"])
    cell_width = float(rows[0]["cell_width_m"])
    cell_height = float(rows[0]["cell_height_m"])
    first_cleaned: dict[int, int] = {}
    violation_cells: set[int] = set()
    for row in rows:
        for cell in row["cleaned_cells"]:
            first_cleaned.setdefault(cell, int(row["time_step"]))
        violation_cells.update(row["prior_cleaned"])

    outputs: dict[str, Path | None] = {}
    fig, ax = plt.subplots(figsize=(8, 5))
    _draw_workspace_base(ax, config, "Contamination Timeline Summary")
    _draw_cells(ax, set(first_cleaned), nx=nx, cell_width_m=cell_width, cell_height_m=cell_height, color="tab:orange", alpha=0.22, label="tool-swept cleaned cells")
    if violation_cells:
        _draw_cells(ax, violation_cells, nx=nx, cell_width_m=cell_width, cell_height_m=cell_height, color="red", alpha=0.65, label="chassis after cleaned")
    for segment_id in {str(pose["segment_id"]) for pose in poses}:
        segment_rows = [pose for pose in poses if str(pose["segment_id"]) == segment_id]
        if not segment_rows:
            continue
        is_transition = segment_rows[0]["primitive_type"] == "transition"
        ax.plot(
            [float(row["world_x_m"]) for row in segment_rows],
            [float(row["world_y_m"]) for row in segment_rows],
            color="tab:blue",
            linestyle="--" if is_transition else "-",
            linewidth=0.8,
            alpha=0.45,
        )
        ax.plot(
            [float(row["tool_world_x_m"]) for row in segment_rows],
            [float(row["tool_world_y_m"]) for row in segment_rows],
            color="tab:orange",
            linestyle="--" if is_transition else "-",
            linewidth=1.1,
            alpha=0.75,
        )
    ax.text(
        0.01,
        0.01,
        (
            f"contamination_free={analysis['contamination_free']} "
            f"violations={analysis['contamination_violation_count']}\n"
            "Tool-over-tool is allowed; chassis-over-prior-tool-swept is forbidden.\n"
            "Preview only: no rover commands, no motor output"
        ),
        transform=ax.transAxes,
        fontsize=8,
        va="bottom",
        bbox={"facecolor": "white", "alpha": 0.80, "edgecolor": "none"},
    )
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    contamination_path = out_dir / "preview_contamination_timeline.png"
    fig.savefig(contamination_path, dpi=160)
    plt.close(fig)
    outputs["preview_contamination_timeline.png"] = contamination_path

    max_step = max(first_cleaned.values()) if first_cleaned else 1
    early = {cell for cell, step in first_cleaned.items() if step <= max_step / 3.0}
    mid = {cell for cell, step in first_cleaned.items() if max_step / 3.0 < step <= 2.0 * max_step / 3.0}
    late = {cell for cell, step in first_cleaned.items() if step > 2.0 * max_step / 3.0}
    fig, ax = plt.subplots(figsize=(8, 5))
    _draw_workspace_base(ax, config, "Temporal Cleaning Layers")
    _draw_cells(ax, early, nx=nx, cell_width_m=cell_width, cell_height_m=cell_height, color="tab:green", alpha=0.24, label="early cleaned")
    _draw_cells(ax, mid, nx=nx, cell_width_m=cell_width, cell_height_m=cell_height, color="tab:orange", alpha=0.24, label="mid cleaned")
    _draw_cells(ax, late, nx=nx, cell_width_m=cell_width, cell_height_m=cell_height, color="tab:purple", alpha=0.24, label="late cleaned")
    ax.text(
        0.01,
        0.01,
        "Temporal layers show when cells become wet/cleaned. Preview only: no motor output.",
        transform=ax.transAxes,
        fontsize=8,
        va="bottom",
        bbox={"facecolor": "white", "alpha": 0.80, "edgecolor": "none"},
    )
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    layers_path = out_dir / "preview_temporal_layers.png"
    fig.savefig(layers_path, dpi=160)
    plt.close(fig)
    outputs["preview_temporal_layers.png"] = layers_path
    return outputs


def _write_chassis_vs_wet_preview(
    *,
    out_dir: Path,
    config: SideToolPlanConfig,
    poses: Sequence[dict[str, float | int | str | bool]],
) -> Path | None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return None

    rows = _contamination_samples(config=config, poses=poses)
    if not rows:
        return None
    nx = int(rows[0]["grid_nx"])
    cell_width = float(rows[0]["cell_width_m"])
    cell_height = float(rows[0]["cell_height_m"])
    cleaned_cells: set[int] = set()
    chassis_cells: set[int] = set()
    violation_cells: set[int] = set()
    for row in rows:
        cleaned_cells.update(row["cleaned_cells"])
        chassis_cells.update(row["chassis_cells"])
        violation_cells.update(row["prior_cleaned"])

    fig, ax = plt.subplots(figsize=(8, 5))
    _draw_workspace_base(ax, config, "Chassis vs Prior Wet Area")
    _draw_cells(ax, cleaned_cells, nx=nx, cell_width_m=cell_width, cell_height_m=cell_height, color="tab:orange", alpha=0.18, label="tool-swept wet area")
    _draw_cells(ax, chassis_cells, nx=nx, cell_width_m=cell_width, cell_height_m=cell_height, color="tab:blue", alpha=0.18, label="chassis swept area")
    if violation_cells:
        _draw_cells(ax, violation_cells, nx=nx, cell_width_m=cell_width, cell_height_m=cell_height, color="red", alpha=0.65, label="forbidden overlap")
    ax.text(
        0.01,
        0.01,
        f"forbidden_overlap_cells={len(violation_cells)}. Preview only: no motor output.",
        transform=ax.transAxes,
        fontsize=8,
        va="bottom",
        bbox={"facecolor": "white", "alpha": 0.80, "edgecolor": "none"},
    )
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    path = out_dir / "preview_chassis_vs_wet_area.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return path


def _write_segment_frames(
    *,
    out_dir: Path,
    config: SideToolPlanConfig,
    poses: Sequence[dict[str, float | int | str | bool]],
) -> tuple[dict[str, Path], Path | None]:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Polygon as PolygonPatch
    except ImportError:
        return {}, None

    rows = _contamination_samples(config=config, poses=poses)
    if not rows:
        return {}, None
    by_segment: dict[str, list[dict[str, object]]] = {}
    for row in rows:
        by_segment.setdefault(str(row["sample"].segment_id), []).append(row)
    segment_dir = out_dir / "segment_frames"
    segment_dir.mkdir(parents=True, exist_ok=True)
    frame_paths: dict[str, Path] = {}
    for index, (segment_id, segment_rows) in enumerate(by_segment.items()):
        row = segment_rows[-1]
        sample = row["sample"]
        pose = row["pose"]
        fig, ax = plt.subplots(figsize=(7, 4.5))
        _draw_workspace_base(ax, config, f"Segment {index}: {segment_id}")
        _draw_cells(
            ax,
            row["cleaned_cells"],
            nx=int(row["grid_nx"]),
            cell_width_m=float(row["cell_width_m"]),
            cell_height_m=float(row["cell_height_m"]),
            color="tab:orange",
            alpha=0.20,
            label="wet/cleaned by segment end",
        )
        if row["prior_cleaned"]:
            _draw_cells(
                ax,
                row["prior_cleaned"],
                nx=int(row["grid_nx"]),
                cell_width_m=float(row["cell_width_m"]),
                cell_height_m=float(row["cell_height_m"]),
                color="red",
                alpha=0.65,
                label="forbidden overlap",
            )
        ax.plot(
            [float(r["sample"].x_m) for r in segment_rows],
            [float(r["sample"].y_m) for r in segment_rows],
            color="tab:blue",
            linewidth=1.1,
            label="segment chassis samples",
        )
        ax.add_patch(PolygonPatch(sample.chassis_polygon, closed=True, facecolor="tab:blue", alpha=0.30))
        ax.add_patch(PolygonPatch(sample.tool_polygon, closed=True, facecolor="tab:orange", alpha=0.36))
        ax.text(
            0.01,
            0.01,
            (
                f"primitive={pose['primitive_type']} subtype={pose['primitive_subtype']}\n"
                f"lane={pose['lane_index']} tool_active={pose['tool_active']} "
                f"dd_feasible={pose['differential_drive_feasible']}\n"
                "motor_command_generated=False"
            ),
            transform=ax.transAxes,
            fontsize=8,
            va="bottom",
            bbox={"facecolor": "white", "alpha": 0.80, "edgecolor": "none"},
        )
        ax.legend(loc="upper right", fontsize=7)
        fig.tight_layout()
        path = segment_dir / f"segment_{index:03d}_{segment_id}.png"
        fig.savefig(path, dpi=140)
        plt.close(fig)
        frame_paths[segment_id] = path
    return frame_paths, segment_dir


def _write_route_sequence_md(
    *,
    path: Path,
    config: SideToolPlanConfig,
    poses: Sequence[dict[str, float | int | str | bool]],
    segment_frame_paths: dict[str, Path],
) -> Path:
    summary = summarize_side_tool_path(config, list(poses))
    lines = [
        "# Side-Tool Temporal Route Sequence",
        "",
        "Preview only: no rover commands, no motor output.",
        "",
        f"- kinematic_model: `{summary['kinematic_model']}`",
        f"- contamination_free: `{summary['contamination_free']}`",
        f"- route_reaches_B: `{summary['route_reaches_B']}`",
        f"- route_feasible_for_physical_preview: `{summary['route_feasible_for_physical_preview']}`",
        f"- infeasible_reason: `{summary['infeasible_reason']}`",
        f"- primitive_sequence_valid: `{summary['primitive_sequence_valid']}`",
        f"- required_min_lane_count: `{summary['required_min_lane_count']}`",
        f"- selected_lane_count: `{summary['selected_lane_count']}`",
        f"- selected_lane_count_ok: `{summary['selected_lane_count_ok']}`",
        "",
    ]
    lines.append("## Tool-Space Route")
    lines.append("")
    for pose in poses:
        if pose["tool_segment_type"] == "tool_sweep_track" and pose["segment_type"] == "lane_start":
            lines.append(
                "- Tool track {lane}: from ({sx:.3f},{sy:.3f}) to ({ex:.3f},{ey:.3f}); waypoint `{label}`".format(
                    lane=pose["lane_index"],
                    sx=float(pose["tool_start_x_m"]),
                    sy=float(pose["tool_start_y_m"]),
                    ex=float(pose["tool_end_x_m"]),
                    ey=float(pose["tool_end_y_m"]),
                    label=pose["tool_waypoint_label"],
                )
            )
        elif pose["tool_segment_type"] == "tool_zigzag_connector" and str(pose["segment_id"]).startswith("transition_"):
            if float(pose["tool_connector_distance_m"]) <= 1e-9:
                continue
            lines.append(
                "- Tool connector `{segment}`: from ({sx:.3f},{sy:.3f}) to ({ex:.3f},{ey:.3f}); angle={angle:.1f} deg; distance={distance:.3f} m".format(
                    segment=pose["segment_id"],
                    sx=float(pose["tool_start_x_m"]),
                    sy=float(pose["tool_start_y_m"]),
                    ex=float(pose["tool_end_x_m"]),
                    ey=float(pose["tool_end_y_m"]),
                    angle=float(pose["tool_connector_angle_deg"]),
                    distance=float(pose["tool_connector_distance_m"]),
                )
            )
    lines.append("")
    lines.append("## Derived Chassis Route")
    lines.append("")
    for pose in poses:
        if float(pose.get("distance_m", 0.0)) <= 1e-9 and float(pose.get("angle_deg", 0.0)) <= 1e-9:
            continue
        lines.append(
            "- `{segment}`: chassis ({sx:.3f},{sy:.3f}) -> ({ex:.3f},{ey:.3f}), heading={heading:.1f} deg, derived_from_tool={derived}".format(
                segment=pose["segment_id"],
                sx=float(pose["chassis_start_x_m"]),
                sy=float(pose["chassis_start_y_m"]),
                ex=float(pose["chassis_end_x_m"]),
                ey=float(pose["chassis_end_y_m"]),
                heading=float(pose["chassis_heading_deg"]),
                derived=pose["chassis_path_derived_from_tool"],
            )
        )
    lines.append("")
    lines.append("## Primitive Commands")
    lines.append("")
    for index, pose in enumerate(poses):
        primitive = str(pose.get("movement_primitive_type", pose["primitive_type"]))
        distance_m = float(pose.get("distance_m", 0.0))
        angle_deg = float(pose.get("angle_deg", 0.0))
        if primitive in {"move_forward", "move_backward"}:
            command = f"{primitive}({distance_m:.3f} m)"
        elif primitive in {"rotate_left", "rotate_right"}:
            command = f"{primitive}({angle_deg:.1f} deg)"
        else:
            command = f"{primitive}(invalid)"
        lines.extend(
            [
                f"- P{index:03d} `{command}`",
                f"  segment_id: `{pose['segment_id']}`; lane_index: `{pose['lane_index']}`; tool_active: `{pose['tool_active']}`; coverage_contributes: `{pose['coverage_contributes']}`",
                "  start_pose: "
                f"`x={float(pose.get('starts_at_x_m', pose['x_m'])):.3f}, "
                f"y={float(pose.get('starts_at_y_m', pose['y_m'])):.3f}, "
                f"heading={float(pose.get('starts_at_heading_deg', pose['heading_deg'])):.1f}`",
                "  end_pose: "
                f"`x={float(pose.get('ends_at_x_m', pose['x_m'])):.3f}, "
                f"y={float(pose.get('ends_at_y_m', pose['y_m'])):.3f}, "
                f"heading={float(pose.get('ends_at_heading_deg', pose['heading_deg'])):.1f}`",
                f"  primitive_sequence_valid: `{pose.get('primitive_sequence_valid', False)}`; invalid_motion_reason: `{pose.get('invalid_motion_reason', 'OK')}`",
                f"  contamination_free_primitive: `{pose.get('contamination_free_primitive', pose['contamination_free_segment'])}`; "
                f"chassis_on_prior_cleaned_area_m2: `{float(pose['chassis_on_prior_cleaned_area_m2']):.3f}`; "
                "motor_command_generated: `False`",
            ]
        )
    lines.append("")
    segment_ids: list[str] = []
    for pose in poses:
        segment_id = str(pose["segment_id"])
        if segment_id not in segment_ids:
            segment_ids.append(segment_id)
    lines.append("## Segment Summary")
    lines.append("")
    for index, segment_id in enumerate(segment_ids):
        rows = [pose for pose in poses if str(pose["segment_id"]) == segment_id]
        first = rows[0]
        last = rows[-1]
        frame = segment_frame_paths.get(segment_id)
        lines.extend(
            [
                f"### Step {index}: `{segment_id}`",
                "",
                f"- primitive_type: `{first['primitive_type']}`",
                f"- primitive_subtypes: `{', '.join(str(row['primitive_subtype']) for row in rows)}`",
                f"- lane_index: `{first['lane_index']}`",
                f"- tool_active_any: `{any(row['tool_active'] is True for row in rows)}`",
                f"- differential_drive_feasible: `{all(row['differential_drive_feasible'] is True for row in rows)}`",
                f"- contamination_free_segment: `{all(row['contamination_free_segment'] is True for row in rows)}`",
                f"- cleaned_area_added_m2: `{sum(float(row['cleaned_area_added_m2']) for row in rows):.3f}`",
                f"- chassis_on_prior_cleaned_area_m2: `{sum(float(row['chassis_on_prior_cleaned_area_m2']) for row in rows):.3f}`",
                f"- start_pose: `x={float(first['x_m']):.3f}, y={float(first['y_m']):.3f}, heading={float(first['heading_deg']):.1f}`",
                f"- end_pose: `x={float(last['x_m']):.3f}, y={float(last['y_m']):.3f}, heading={float(last['heading_deg']):.1f}`",
                f"- frame: `{frame if frame is not None else 'not generated'}`",
                "",
            ]
        )
    path.write_text("\n".join(lines), encoding="utf-8")
    return path


def _write_summary(
    *,
    path: Path,
    config: SideToolPlanConfig,
    poses: Sequence[dict[str, float | int | str | bool]],
    csv_path: Path,
    png_path: Path | None,
    geometry_csv_path: Path | None,
    swept_json_path: Path | None,
    separated_preview_paths: dict[str, Path | None] | None,
    contamination_events_path: Path | None,
    strategy_diagnostics_csv_path: Path | None,
    strategy_diagnostics_md_path: Path | None,
    timeline_state_path: Path | None,
    timeline_contact_sheet_path: Path | None,
    timeline_index_path: Path | None,
    contamination_preview_paths: dict[str, Path | None] | None,
    chassis_vs_wet_path: Path | None,
    segment_frame_paths: dict[str, Path],
    route_sequence_path: Path | None,
    timeline_frame_count: int,
) -> None:
    path_summary = summarize_side_tool_path(config, list(poses))
    lines = [
        "# Side-Mounted Tool Path Preview",
        "",
        "This artifact is offline preview-only.",
        "",
        "- No HC-12 commands are sent.",
        "- No rover motor commands are generated.",
        "- No firmware is uploaded or modified.",
        "- This is not autonomous execution.",
        "- Physical path following still requires heading/course and steering validation.",
        "",
        "## Inputs",
        "",
        f"- tool_side: `{config.tool_side}`",
        f"- tool_lateral_offset_m: `{config.tool_lateral_offset_m:.3f}`",
        f"- tool_width_m: `{config.tool_width_m:.3f}`",
        f"- tool_length_m: `{config.tool_length_m if config.tool_length_m is not None else 'default_from_robot_length'}`",
        f"- lane_spacing_m: `{config.lane_spacing_m:.3f}`",
        f"- row_length_m: `{config.row_length_m if config.row_length_m is not None else 'inferred_from_A_B'}`",
        f"- row_count: `{config.row_count}`",
        f"- workspace_mode: `{config.workspace_mode}`",
        f"- start_pose: `x={config.start_x_m:.3f}, y={config.start_y_m:.3f}, heading={config.start_heading_deg:.3f} deg`",
        f"- A: `x={config.a_x_m}, y={config.a_y_m}`",
        f"- B: `x={config.b_x_m}, y={config.b_y_m}`",
        f"- surface_side: `{config.surface_side if config.surface_side is not None else config.workspace_side}`",
        f"- workspace_side: `{config.workspace_side if config.workspace_side is not None else 'derived_or_not_used'}`",
        f"- workspace_width_m: `{config.workspace_width_m}`",
        f"- y_min_m/y_max_m: `{config.y_min_m}` / `{config.y_max_m}`",
        f"- robot_width_m: `{config.robot_width_m:.3f}`",
        f"- robot_length_m: `{config.robot_length_m if config.robot_length_m is not None else 'not_set'}`",
        f"- robot_radius_m: `{config.robot_radius_m if config.robot_radius_m is not None else 'derived'}`",
        f"- first_lane_direction: `{config.first_lane_direction}`",
        f"- transition_style: `{config.transition_style}`",
        f"- auto_orient_tool_inside: `{config.auto_orient_tool_inside}`",
        f"- start_policy: `{config.start_policy}`",
        f"- end_policy: `{config.end_policy}`",
        f"- boundary_mode: `{config.boundary_mode}`",
        f"- chassis_boundary_mode: `{config.chassis_boundary_mode}`",
        f"- auto_inset_endpoints: `{config.auto_inset_endpoints}`",
        f"- boundary_margin_m: `{config.boundary_margin_m:.3f}`",
        f"- turn_clearance_mode: `{config.turn_clearance_mode}`",
        f"- allow_uncovered_margins: `{config.allow_uncovered_margins}`",
        f"- coverage_resolution_m: `{config.coverage_resolution_m:.3f}`",
        f"- rotation_sample_deg: `{config.rotation_sample_deg:.3f}`",
        f"- translation_sample_m: `{config.translation_sample_m:.3f}`",
        f"- swept_volume_validation: `{config.swept_volume_validation}`",
        f"- kinematic_model: `{config.kinematic_model}`",
        f"- contamination_mode: `{config.contamination_mode}`",
        f"- fail_on_contamination_violation: `{config.fail_on_contamination_violation}`",
        f"- allow_start_end_chassis_outside_clean_surface: `{config.allow_start_end_chassis_outside_clean_surface}`",
        f"- tool_active_during_alignment: `{config.tool_active_during_alignment}`",
        f"- tool_active_during_transitions: `{config.tool_active_during_transitions}`",
        f"- tool_active_during_rotation: `{config.tool_active_during_rotation}`",
        f"- visualize_inactive_tool_ghost: `{config.visualize_inactive_tool_ghost}`",
        f"- show_transition_envelope: `{config.show_transition_envelope}`",
        f"- same_step_tool_before_chassis: `{config.same_step_tool_before_chassis}`",
        f"- route_order: `{config.route_order}`",
        f"- coverage_pattern: `{config.coverage_pattern}`",
        f"- min_lane_count: `{config.min_lane_count}`",
        f"- forbid_single_lane_success: `{config.forbid_single_lane_success}`",
        f"- allow_tool_lift: `{config.allow_tool_lift}`",
        f"- reserve_dry_exit_corridor: `{config.reserve_dry_exit_corridor}`",
        f"- dry_corridor_strategy: `{config.dry_corridor_strategy}`",
        f"- require_start_at_A: `{config.require_start_at_A}`",
        f"- require_end_at_B: `{config.require_end_at_B}`",
        f"- max_start_error_m: `{config.max_start_error_m:.3f}`",
        f"- max_end_error_m: `{config.max_end_error_m:.3f}`",
        f"- max_end_offset_m: `{config.max_end_offset_m:.3f}`",
        f"- allow_best_effort: `{config.allow_best_effort}`",
        f"- allow_contamination_best_effort: `{config.allow_contamination_best_effort}`",
        f"- allow_unreached_B_best_effort: `{config.allow_unreached_b_best_effort}`",
        "",
        "## Preview Semantics",
        "",
        "- The planner is tool-coverage-first: cleaning coverage is computed only from the swept tool footprint.",
        "- The primary planned route is the tool/paint-tank center path.",
        "- Tool sweep tracks and tool zigzag connectors are connected as one continuous tool-center route.",
        "- The chassis path is a derived support path for positioning the side-mounted tool.",
        "- In default `ab_diagonal_center` mode, A and B are exact rover-center start/end points and opposite rectangle corners.",
        "- The dashed A-B diagonal is a guide only; the route must be a repeated boustrophedon coverage path.",
        "- `ab_centerline_width`, `diagonal_ab`, and `axis_width` are legacy/alternate geometry modes.",
        "- The chassis footprint is a boundary constraint; chassis-only travel does not count as cleaning.",
        "- The contamination model is time-ordered: the chassis may not enter cells already swept by the tool.",
        "- Tool-over-prior-tool-swept area is allowed; chassis-over-prior-tool-swept area is forbidden in strict mode.",
        "- Chassis-over-area-that-will-be-cleaned-later is allowed because it is not wet yet.",
        "- Physical feasibility is validated separately from cleaning coverage using sampled chassis+tool swept volume.",
        "- Rotation and translation samples validate the 18 cm rover body plus side-mounted tool footprint.",
        "- `x_m`, `y_m`, and `heading_deg` are derived chassis centerline poses.",
        "- `world_x_m`, `world_y_m`, and `world_heading_deg` transform local workspace poses back into the A/B frame.",
        "- `tool_x_m`, `tool_y_m`, and tool edge fields describe the cleaning-tool footprint and coverage path.",
        "- `heading_deg` is chassis facing direction; `motion_direction` and `travel_direction_deg` describe actual travel.",
        "- `auto_internal` chooses among feasible internal transition primitives instead of forcing one maneuver.",
        "- `side_step_reverse_90` remains available only as one explicit transition primitive.",
        "- Transition-only rows are separated from cleaning lanes and do not count as cleaned coverage.",
        "- Lane transitions use an inward transition envelope; external turn bays are not allowed.",
        "- A/B bounded previews validate chassis footprint, tool footprint center, and tool footprint edges.",
        "- Transition points and transition envelopes are also validated against the A/B rectangle.",
        "- Unreachable edge strips are reported as uncovered margins instead of allowing geometry to protrude.",
        "- This preview is not a firmware input and must not be connected to rover motor output.",
        "",
        "## Outputs",
        "",
        f"- pose_count: `{len(poses)}`",
        f"- A: `({float(path_summary['a_x_m']):.3f}, {float(path_summary['a_y_m']):.3f})`",
        f"- B: `({float(path_summary['b_x_m']):.3f}, {float(path_summary['b_y_m']):.3f})`",
        f"- planner_objective: `{path_summary['planner_objective']}`",
        f"- planner_primary_path: `{path_summary['planner_primary_path']}`",
        f"- chassis_path_role: `{path_summary['chassis_path_role']}`",
        f"- tool_path_continuous: `{path_summary['tool_path_continuous']}`",
        f"- tool_track_count: `{path_summary['tool_track_count']}`",
        f"- tool_connector_count: `{path_summary['tool_connector_count']}`",
        f"- tool_zigzag_connector_count: `{path_summary['tool_zigzag_connector_count']}`",
        f"- tool_connector_angles_deg: `{path_summary['tool_connector_angles_deg']}`",
        f"- tool_connector_distances_m: `{path_summary['tool_connector_distances_m']}`",
        f"- chassis_path_derived_from_tool: `{path_summary['chassis_path_derived_from_tool']}`",
        f"- kinematic_model: `{path_summary['kinematic_model']}`",
        f"- differential_drive_feasible: `{path_summary['differential_drive_feasible']}`",
        f"- kinematic_violation_count: `{path_summary['kinematic_violation_count']}`",
        f"- primitive_sequence_valid: `{path_summary['primitive_sequence_valid']}`",
        f"- primitive_sequence_violation_count: `{path_summary['primitive_sequence_violation_count']}`",
        f"- invalid_primitive_name_count: `{path_summary['invalid_primitive_name_count']}`",
        f"- contamination_model: `{path_summary['contamination_model']}`",
        f"- contamination_mode: `{path_summary['contamination_mode']}`",
        f"- fail_on_contamination_violation: `{path_summary['fail_on_contamination_violation']}`",
        f"- tool_active_during_transitions: `{path_summary['tool_active_during_transitions']}`",
        f"- tool_active_during_alignment: `{path_summary['tool_active_during_alignment']}`",
        f"- tool_active_during_rotation: `{path_summary['tool_active_during_rotation']}`",
        f"- inactive_tool_ghost_visualized: `{path_summary['inactive_tool_ghost_visualized']}`",
        f"- contamination_free: `{path_summary['contamination_free']}`",
        f"- contamination_violation_count: `{path_summary['contamination_violation_count']}`",
        f"- contamination_violation_area_m2: `{float(path_summary['contamination_violation_area_m2']):.3f}`",
        f"- first_contamination_step: `{path_summary['first_contamination_step']}`",
        f"- first_contamination_segment_id: `{path_summary['first_contamination_segment_id']}`",
        f"- temporal_cleaning_valid: `{path_summary['temporal_cleaning_valid']}`",
        f"- cleaned_area_m2: `{float(path_summary['cleaned_area_m2']):.3f}`",
        f"- tool_reclean_area_m2: `{float(path_summary['tool_reclean_area_m2']):.3f}`",
        f"- chassis_after_cleaned_area_m2: `{float(path_summary['chassis_after_cleaned_area_m2']):.3f}`",
        f"- route_order_strategy: `{path_summary['route_order_strategy']}`",
        f"- route_order_strategy_candidates_evaluated: `{path_summary['route_order_strategy_candidates_evaluated']}`",
        f"- selected_route_order_strategy: `{path_summary['selected_route_order_strategy']}`",
        f"- initial_pose_at_A: `{path_summary['initial_pose_at_A']}`",
        f"- initial_alignment_heading_deg: `{path_summary['initial_alignment_heading_deg']}`",
        f"- lane_axis_heading_deg: `{path_summary['lane_axis_heading_deg']}`",
        f"- first_cleaning_heading_deg: `{path_summary['first_cleaning_heading_deg']}`",
        f"- first_cleaning_heading_aligned: `{path_summary['first_cleaning_heading_aligned']}`",
        f"- preclean_alignment_used: `{path_summary['preclean_alignment_used']}`",
        f"- A_B_are_rover_center_start_end: `{path_summary['A_B_are_rover_center_start_end']}`",
        f"- A_B_are_rectangle_diagonal: `{path_summary['A_B_are_rectangle_diagonal']}`",
        f"- start_reaches_A: `{path_summary['start_reaches_A']}`",
        f"- end_reaches_B: `{path_summary['end_reaches_B']}`",
        f"- route_reaches_B: `{path_summary['route_reaches_B']}`",
        f"- start_error_m: `{float(path_summary['start_error_m']):.3f}`",
        f"- end_error_m: `{float(path_summary['end_error_m']):.3f}`",
        f"- final_distance_to_B_m: `{float(path_summary['final_distance_to_B_m']):.3f}`",
        f"- route_feasible_for_physical_preview: `{path_summary['route_feasible_for_physical_preview']}`",
        f"- infeasible_reason: `{path_summary['infeasible_reason']}`",
        f"- coverage_pattern: `{path_summary['coverage_pattern']}`",
        f"- required_min_lane_count: `{path_summary['required_min_lane_count']}`",
        f"- selected_lane_count_ok: `{path_summary['selected_lane_count_ok']}`",
        f"- single_lane_success_forbidden: `{path_summary['single_lane_success_forbidden']}`",
        f"- boustrophedon_pattern_valid: `{path_summary['boustrophedon_pattern_valid']}`",
        f"- allow_tool_lift: `{path_summary['allow_tool_lift']}`",
        f"- reserve_dry_exit_corridor: `{path_summary['reserve_dry_exit_corridor']}`",
        f"- coverage_model: `{path_summary['coverage_model']}`",
        f"- feasibility_model: `{path_summary['feasibility_model']}`",
        f"- transition_planner: `{path_summary['transition_planner']}`",
        f"- side_step_reverse_90_is_candidate_only: `{path_summary['side_step_reverse_90_is_candidate_only']}`",
        f"- chassis_path_role: `{path_summary['chassis_path_role']}`",
        f"- chassis_area_counts_as_cleaned: `{path_summary['chassis_area_counts_as_cleaned']}`",
        f"- workspace_mode: `{path_summary['workspace_mode']}`",
        f"- A/B semantic: `{path_summary['a_b_semantic']}`",
        f"- ab_length_m: `{float(path_summary['ab_length_m']):.3f}`",
        f"- diagonal_length_m: `{float(path_summary['diagonal_length_m']):.3f}`",
        f"- rectangle_length_m: `{float(path_summary['rectangle_length_m']):.3f}`",
        f"- rectangle_width_m: `{float(path_summary['rectangle_width_m']):.3f}`",
        f"- workspace_side: `{path_summary['workspace_side']}`",
        f"- surface_side: `{path_summary['surface_side']}`",
        f"- chassis_boundary_mode: `{path_summary['chassis_boundary_mode']}`",
        f"- workspace_width_m: `{float(path_summary['workspace_width_m']):.3f}`",
        f"- clean_surface_bounds: `x=[{float(path_summary['clean_surface_x_min_m']):.3f}, {float(path_summary['clean_surface_x_max_m']):.3f}], y=[{float(path_summary['clean_surface_y_min_m']):.3f}, {float(path_summary['clean_surface_y_max_m']):.3f}]`",
        f"- reserved_dry_corridor_area_m2: `{float(path_summary['reserved_dry_corridor_area_m2']):.3f}`",
        f"- unavoidable_uncovered_due_to_exit_corridor_m2: `{float(path_summary['unavoidable_uncovered_due_to_exit_corridor_m2']):.3f}`",
        f"- dry_corridor_strategy: `{path_summary['dry_corridor_strategy']}`",
        f"- exit_corridor_reaches_B: `{path_summary['exit_corridor_reaches_B']}`",
        f"- tool_lift_used: `{path_summary['tool_lift_used']}`",
        f"- dry_corridor_unpainted_area_m2: `{float(path_summary['dry_corridor_unpainted_area_m2']):.3f}`",
        f"- robot_width_m: `{float(path_summary['robot_width_m']):.3f}`",
        f"- robot_length_m: `{path_summary['robot_length_m']}`",
        f"- robot_radius_m: `{float(path_summary['robot_radius_m']):.3f}`",
        f"- tool_side: `{path_summary['tool_side']}`",
        f"- tool_lateral_offset_m: `{float(path_summary['tool_lateral_offset_m']):.3f}`",
        f"- tool_width_m: `{float(path_summary['tool_width_m']):.3f}`",
        f"- tool_length_m: `{float(path_summary['tool_length_m']):.3f}`",
        f"- tool_length_source: `{path_summary['tool_length_source']}`",
        f"- lane_spacing_m: `{float(path_summary['lane_spacing_m']):.3f}`",
        f"- candidate_lane_count: `{path_summary['candidate_lane_count']}`",
        f"- selected_lane_count: `{path_summary['selected_lane_count']}`",
        f"- transition_candidate_count: `{path_summary['transition_candidate_count']}`",
        f"- selected_transition_count: `{path_summary['selected_transition_count']}`",
        f"- requested_row_count: `{path_summary['requested_row_count']}`",
        f"- requested_coverage_area_m2: `{float(path_summary['requested_coverage_area_m2']):.3f}`",
        f"- covered_area_m2: `{float(path_summary['covered_area_m2']):.3f}`",
        f"- uncovered_area_m2: `{float(path_summary['uncovered_area_m2']):.3f}`",
        f"- uncovered_cells_count: `{path_summary['uncovered_cells_count']}`",
        f"- overlap_area_m2: `{float(path_summary['overlap_area_m2']):.3f}`",
        f"- coverage_efficiency: `{float(path_summary['coverage_efficiency']):.3f}`",
        f"- coverage_grid_resolution_m: `{float(path_summary['coverage_grid_resolution_m']):.3f}`",
        f"- uncovered_strips_count: `{path_summary['uncovered_strips_count']}`",
        f"- boundary_violation_count: `{path_summary['boundary_violation_count']}`",
        f"- internal_transition_required: `{path_summary['internal_transition_required']}`",
        f"- external_turn_bay_allowed: `{path_summary['external_turn_bay_allowed']}`",
        f"- transition_clearance_radius_m: `{float(path_summary['transition_clearance_radius_m']):.3f}`",
        f"- transition_envelope_violation_count: `{path_summary['transition_envelope_violation_count']}`",
        f"- plot_artifact_violation_count: `{path_summary['plot_artifact_violation_count']}`",
        f"- physical_swept_volume_validated: `{path_summary['physical_swept_volume_validated']}`",
        f"- rotation_sample_deg: `{float(path_summary['rotation_sample_deg']):.3f}`",
        f"- translation_sample_m: `{float(path_summary['translation_sample_m']):.3f}`",
        f"- total_geometry_sample_count: `{path_summary['total_geometry_sample_count']}`",
        f"- rotation_sample_count: `{path_summary['rotation_sample_count']}`",
        f"- translation_sample_count: `{path_summary['translation_sample_count']}`",
        f"- rotation_swept_violation_count: `{path_summary['rotation_swept_violation_count']}`",
        f"- translation_swept_violation_count: `{path_summary['translation_swept_violation_count']}`",
        f"- combined_swept_violation_count: `{path_summary['combined_swept_violation_count']}`",
        f"- endpoint_inset_m: `{float(path_summary['endpoint_inset_m']):.3f}`",
        f"- start_inset_m: `{float(path_summary['start_inset_m']):.3f}`",
        f"- end_inset_m: `{float(path_summary['end_inset_m']):.3f}`",
        f"- actual_lane_count: `{path_summary['actual_lane_count']}`",
        f"- row_count_adjusted: `{path_summary['row_count_adjusted']}`",
        f"- row_count_warning: `{path_summary['row_count_warning']}`",
        f"- workspace_bounds: `x=[{float(path_summary['workspace_x_min_m']):.3f}, {float(path_summary['workspace_x_max_m']):.3f}], y=[{float(path_summary['workspace_y_min_m']):.3f}, {float(path_summary['workspace_y_max_m']):.3f}]`",
        f"- requested_coverage_bounds: `y=[{float(path_summary['requested_coverage_y_min_m']):.3f}, {float(path_summary['requested_coverage_y_max_m']):.3f}]`",
        f"- actual_tool_coverage_bounds: `y=[{float(path_summary['actual_tool_coverage_y_min_m']):.3f}, {float(path_summary['actual_tool_coverage_y_max_m']):.3f}]`",
        f"- tool_swept_x_bounds: `x=[{float(path_summary['tool_swept_x_min_m']):.3f}, {float(path_summary['tool_swept_x_max_m']):.3f}]`",
        f"- tool_swept_y_bounds: `y=[{float(path_summary['tool_swept_y_min_m']):.3f}, {float(path_summary['tool_swept_y_max_m']):.3f}]`",
        f"- uncovered_margin_low_m: `{float(path_summary['uncovered_margin_low_m']):.3f}`",
        f"- uncovered_margin_high_m: `{float(path_summary['uncovered_margin_high_m']):.3f}`",
        f"- coverage_ratio: `{float(path_summary['coverage_ratio']):.3f}`",
        f"- path_preview_only: `{path_summary['path_preview_only']}`",
        f"- feedback_tracking_ready: `{path_summary['feedback_tracking_ready']}`",
        f"- coverage_grid_model: `tool_footprint_grid_set_cover`",
        f"- motor_command_generated: `{path_summary['motor_command_generated']}`",
        f"- generated_at_utc: `{dt.datetime.now(tz=dt.UTC).isoformat()}`",
        f"- side_tool_path_csv: `{csv_path}`",
        f"- geometry_samples_csv: `{geometry_csv_path if geometry_csv_path is not None else 'not generated'}`",
        f"- swept_volume_summary_json: `{swept_json_path if swept_json_path is not None else 'not generated'}`",
        f"- contamination_events_csv: `{contamination_events_path if contamination_events_path is not None else 'not generated'}`",
        f"- strategy_diagnostics_csv: `{strategy_diagnostics_csv_path if strategy_diagnostics_csv_path is not None else 'not generated'}`",
        f"- strategy_diagnostics_md: `{strategy_diagnostics_md_path if strategy_diagnostics_md_path is not None else 'not generated'}`",
        f"- timeline_state_csv: `{timeline_state_path if timeline_state_path is not None else 'not generated'}`",
        f"- timeline_frame_count: `{timeline_frame_count}`",
        f"- segment_frame_count: `{len(segment_frame_paths)}`",
        f"- timeline_contact_sheet: `{timeline_contact_sheet_path if timeline_contact_sheet_path is not None else 'not generated'}`",
        f"- timeline_html_index: `{timeline_index_path if timeline_index_path is not None else 'not generated'}`",
        f"- route_sequence_md: `{route_sequence_path if route_sequence_path is not None else 'not generated'}`",
        f"- preview_chassis_vs_wet_area: `{chassis_vs_wet_path if chassis_vs_wet_path is not None else 'not generated'}`",
        f"- preview_png: `{png_path if png_path is not None else 'not generated'}`",
    ]
    if separated_preview_paths:
        lines.append("")
        lines.append("## Separated Preview Images")
        lines.append("")
        for label, preview_path in separated_preview_paths.items():
            lines.append(f"- {label}: `{preview_path if preview_path is not None else 'not generated'}`")
    if contamination_preview_paths:
        lines.append("")
        lines.append("## Contamination Preview Images")
        lines.append("")
        for label, preview_path in contamination_preview_paths.items():
            lines.append(f"- {label}: `{preview_path if preview_path is not None else 'not generated'}`")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_png(
    *,
    path: Path,
    config: SideToolPlanConfig,
    poses: Sequence[dict[str, float | int | str | bool]],
) -> Path | None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle
    except ImportError:
        return None

    violation_x = [float(pose["world_x_m"]) for pose in poses if pose["within_boundary"] is not True]
    violation_y = [float(pose["world_y_m"]) for pose in poses if pose["within_boundary"] is not True]
    path_summary = summarize_side_tool_path(config, list(poses))

    fig, ax = plt.subplots(figsize=(8, 6))
    if config.boundary_mode == "strict":
        polygon = boundary_polygon(config)
        polygon_closed = polygon + [polygon[0]]
        ax.plot(
            [point[0] for point in polygon_closed],
            [point[1] for point in polygon_closed],
            color="black",
            linewidth=1.4,
            label="A/B boundary",
        )
        a_point = (config.a_x_m, config.a_y_m) if config.a_x_m is not None and config.a_y_m is not None else polygon[0]
        b_point = (config.b_x_m, config.b_y_m) if config.b_x_m is not None and config.b_y_m is not None else polygon[1]
        ax.scatter([a_point[0], b_point[0]], [a_point[1], b_point[1]], color="black", s=35)
        a_label = "A start/corner" if config.workspace_mode == "ab_diagonal_center" else "A corner"
        b_label = "B end/corner" if config.workspace_mode == "ab_diagonal_center" else "B corner"
        if config.workspace_mode == "ab_centerline_width":
            a_label = "A start"
            b_label = "B end"
        ax.annotate(a_label, a_point, fontsize=10, fontweight="bold")
        ax.annotate(b_label, b_point, fontsize=10, fontweight="bold")
        if config.show_transition_envelope and config.workspace_mode in {"ab_diagonal_center", "diagonal_ab"}:
            ax.plot(
                [a_point[0], b_point[0]],
                [a_point[1], b_point[1]],
                color="black",
                linestyle="--",
                linewidth=1.0,
                alpha=0.5,
                label="A-B diagonal guide",
            )
            low_margin = float(path_summary["uncovered_margin_low_m"])
            high_margin = float(path_summary["uncovered_margin_high_m"])
            if low_margin > 0.0:
                ax.fill_between(
                    [float(path_summary["workspace_x_min_m"]), float(path_summary["workspace_x_max_m"])],
                    float(path_summary["workspace_y_min_m"]),
                    float(path_summary["workspace_y_min_m"]) + low_margin,
                    color="lightgray",
                    alpha=0.25,
                    label="uncovered margin",
                )
            if high_margin > 0.0:
                ax.fill_between(
                    [float(path_summary["workspace_x_min_m"]), float(path_summary["workspace_x_max_m"])],
                    float(path_summary["workspace_y_max_m"]) - high_margin,
                    float(path_summary["workspace_y_max_m"]),
                    color="lightgray",
                    alpha=0.25,
                )
            tool_sweep_label_used = False
            tool_sweep_intervals = sorted(
                {
                    (
                        float(pose["tool_edge_min_y_m"]),
                        float(pose["tool_edge_max_y_m"]),
                    )
                    for pose in poses
                    if pose["coverage_contributes"] is True
                }
            )
            for edge_min_y, edge_max_y in tool_sweep_intervals:
                label = "tool swept footprint" if not tool_sweep_label_used else None
                ax.fill_between(
                    [
                        float(path_summary["tool_swept_x_min_m"]),
                        float(path_summary["tool_swept_x_max_m"]),
                    ],
                    edge_min_y,
                    edge_max_y,
                    color="tab:orange",
                    alpha=0.12,
                    label=label,
                )
                tool_sweep_label_used = True
        lane_starts = [pose for pose in poses if pose["segment_type"] == "lane_start"]
        lane_ends = [pose for pose in poses if pose["segment_type"] == "lane_end"]
        if lane_starts:
            start_pose = lane_starts[0]
            ax.scatter(
                [float(start_pose["world_x_m"])],
                [float(start_pose["world_y_m"])],
                color="black",
                marker="^",
                s=45,
            )
            ax.annotate("start feasible", (float(start_pose["world_x_m"]), float(start_pose["world_y_m"])), fontsize=8)
        if lane_ends:
            end_pose = lane_ends[-1]
            ax.scatter(
                [float(end_pose["world_x_m"])],
                [float(end_pose["world_y_m"])],
                color="black",
                marker="s",
                s=45,
            )
            ax.annotate("end feasible", (float(end_pose["world_x_m"]), float(end_pose["world_y_m"])), fontsize=8)
        if (
            float(path_summary["uncovered_margin_low_m"]) > 0.0
            or float(path_summary["uncovered_margin_high_m"]) > 0.0
        ):
            ax.text(
                0.02,
                0.96,
                (
                    "uncovered margins: "
                    f"low={float(path_summary['uncovered_margin_low_m']):.2f} m, "
                    f"high={float(path_summary['uncovered_margin_high_m']):.2f} m"
                ),
                transform=ax.transAxes,
                fontsize=8,
                va="top",
                bbox={"facecolor": "white", "alpha": 0.75, "edgecolor": "none"},
            )
        if config.workspace_mode in {"ab_diagonal_center", "diagonal_ab"}:
            envelope_label_used = False
            envelope_ids_seen: set[str] = set()
            for pose in poses:
                if pose["transition_pocket_side"] == "none":
                    continue
                segment_id = str(pose["segment_id"])
                if segment_id in envelope_ids_seen:
                    continue
                envelope_ids_seen.add(segment_id)
                envelope_color = (
                    "tab:blue"
                    if pose["transition_envelope_within_boundary"] is True
                    else "red"
                )
                x_min = float(pose["transition_envelope_x_min_m"])
                x_max = float(pose["transition_envelope_x_max_m"])
                y_min = float(pose["transition_envelope_y_min_m"])
                y_max = float(pose["transition_envelope_y_max_m"])
                label = "internal transition envelope" if not envelope_label_used else None
                ax.add_patch(
                    Rectangle(
                        (x_min, y_min),
                        x_max - x_min,
                        y_max - y_min,
                        fill=False,
                        linestyle="--",
                        linewidth=0.9,
                        edgecolor=envelope_color,
                        alpha=0.55,
                        label=label,
                    )
                )
                envelope_label_used = True
    segment_ids = []
    for pose in poses:
        segment_id = str(pose["segment_id"])
        if segment_id not in segment_ids:
            segment_ids.append(segment_id)

    tool_lane_label_used = False
    tool_transition_label_used = False
    chassis_label_used = False
    transition_chassis_label_used = False
    for segment_id in segment_ids:
        rows = [pose for pose in poses if str(pose["segment_id"]) == segment_id]
        if not rows:
            continue
        is_transition = rows[0]["primitive_type"] == "transition"
        xs_tool = [float(pose["tool_world_x_m"]) for pose in rows]
        ys_tool = [float(pose["tool_world_y_m"]) for pose in rows]
        xs_chassis = [float(pose["world_x_m"]) for pose in rows]
        ys_chassis = [float(pose["world_y_m"]) for pose in rows]
        if is_transition:
            label = "transition tool path" if not tool_transition_label_used else None
            ax.plot(
                xs_tool,
                ys_tool,
                marker=".",
                linewidth=1.0,
                linestyle="--",
                color="tab:orange",
                alpha=0.55,
                label=label,
            )
            tool_transition_label_used = True
            chassis_label = "transition chassis support" if not transition_chassis_label_used else None
            ax.plot(
                xs_chassis,
                ys_chassis,
                marker="o",
                linewidth=0.8,
                linestyle="--",
                color="tab:blue",
                alpha=0.35,
                label=chassis_label,
            )
            transition_chassis_label_used = True
        else:
            label = "tool cleaning lane path" if not tool_lane_label_used else None
            ax.plot(
                xs_tool,
                ys_tool,
                marker=".",
                linewidth=2.2,
                color="tab:orange",
                label=label,
            )
            tool_lane_label_used = True
            chassis_label = "derived chassis centerline" if not chassis_label_used else None
            ax.plot(
                xs_chassis,
                ys_chassis,
                marker="o",
                linewidth=1.0,
                color="tab:blue",
                alpha=0.55,
                label=chassis_label,
            )
            chassis_label_used = True
    if violation_x:
        ax.scatter(violation_x, violation_y, color="red", marker="x", s=60, label="boundary violation")
    for pose in poses:
        ax.annotate(str(pose["index"]), (float(pose["world_x_m"]), float(pose["world_y_m"])), fontsize=8)
    for pose in poses:
        if pose["segment_type"] in {"lane_start", "lane_end"}:
            ax.plot(
                [float(pose["tool_inner_world_x_m"]), float(pose["tool_outer_world_x_m"])],
                [float(pose["tool_inner_world_y_m"]), float(pose["tool_outer_world_y_m"])],
                color="red" if pose["within_boundary"] is not True else "tab:orange",
                alpha=0.4,
            )

    readiness = "READY" if path_summary["route_feasible_for_physical_preview"] is True else "INFEASIBLE"
    ax.set_title(f"Side-Mounted Tool Path Preview Only - {readiness}")
    ax.set_xlabel("world x / A-B frame x (m)")
    ax.set_ylabel("world y / A-B frame y (m)")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.axis("equal")
    ax.legend()
    ax.text(
        0.01,
        0.01,
        "Coverage is tool footprint only; chassis path is derived. Preview only: no motor output.",
        transform=ax.transAxes,
        fontsize=8,
        va="bottom",
    )
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return path


def _write_separated_previews(
    *,
    out_dir: Path,
    config: SideToolPlanConfig,
    poses: Sequence[dict[str, float | int | str | bool]],
) -> dict[str, Path | None]:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Polygon as PolygonPatch
        from matplotlib.patches import Rectangle
    except ImportError:
        return {
            "preview_overview.png": None,
            "preview_alignment_and_start.png": None,
            "preview_tool_path_primary.png": None,
            "preview_chassis_derived_from_tool.png": None,
            "preview_sweep_pattern.png": None,
            "preview_primitive_sequence.png": None,
            "preview_tool_coverage_only.png": None,
            "preview_boustrophedon_pattern.png": None,
            "preview_chassis_only.png": None,
            "preview_transitions_only.png": None,
            "preview_rotation_swept_volume.png": None,
            "preview_geometry_samples.png": None,
        }

    path_summary = summarize_side_tool_path(config, list(poses))
    samples = geometry_samples_for_path(config, list(poses))

    def setup_ax(title: str):
        fig, ax = plt.subplots(figsize=(8, 6))
        polygon = boundary_polygon(config)
        closed = polygon + [polygon[0]]
        ax.plot(
            [point[0] for point in closed],
            [point[1] for point in closed],
            color="black",
            linewidth=1.4,
            label="A/B boundary",
        )
        if config.a_x_m is not None and config.a_y_m is not None:
            a_point = (config.a_x_m, config.a_y_m)
            ax.scatter([a_point[0]], [a_point[1]], color="black", s=35)
            ax.annotate("A", a_point, fontsize=10, fontweight="bold")
        if config.b_x_m is not None and config.b_y_m is not None:
            b_point = (config.b_x_m, config.b_y_m)
            ax.scatter([b_point[0]], [b_point[1]], color="black", s=35)
            ax.annotate("B", b_point, fontsize=10, fontweight="bold")
        if (
            config.workspace_mode in {"ab_diagonal_center", "diagonal_ab"}
            and config.a_x_m is not None
            and config.a_y_m is not None
            and config.b_x_m is not None
            and config.b_y_m is not None
        ):
            ax.plot(
                [config.a_x_m, config.b_x_m],
                [config.a_y_m, config.b_y_m],
                color="black",
                linestyle="--",
                linewidth=1.0,
                alpha=0.45,
                label="A-B diagonal guide",
            )
        ax.set_title(title)
        ax.set_xlabel("workspace x (m)")
        ax.set_ylabel("workspace y (m)")
        ax.grid(True, linestyle="--", alpha=0.35)
        ax.axis("equal")
        ax.text(
            0.01,
            0.01,
            "Preview only: no rover commands, no motor output",
            transform=ax.transAxes,
            fontsize=8,
            va="bottom",
            bbox={"facecolor": "white", "alpha": 0.75, "edgecolor": "none"},
        )
        return fig, ax

    def draw_uncovered_and_swept(ax) -> None:
        low_margin = float(path_summary["uncovered_margin_low_m"])
        high_margin = float(path_summary["uncovered_margin_high_m"])
        x_low = float(path_summary["workspace_x_min_m"])
        x_high = float(path_summary["workspace_x_max_m"])
        y_low = float(path_summary["workspace_y_min_m"])
        y_high = float(path_summary["workspace_y_max_m"])
        if low_margin > 0.0:
            ax.fill_between([x_low, x_high], y_low, y_low + low_margin, color="lightgray", alpha=0.28, label="uncovered")
        if high_margin > 0.0:
            ax.fill_between([x_low, x_high], y_high - high_margin, y_high, color="lightgray", alpha=0.28)
        label_used = False
        intervals = sorted(
            {
                (float(pose["tool_edge_min_y_m"]), float(pose["tool_edge_max_y_m"]))
                for pose in poses
                if pose["coverage_contributes"] is True
            }
        )
        for edge_min_y, edge_max_y in intervals:
            label = "cleaned tool swept area" if not label_used else None
            ax.fill_between(
                [float(path_summary["tool_swept_x_min_m"]), float(path_summary["tool_swept_x_max_m"])],
                edge_min_y,
                edge_max_y,
                color="tab:orange",
                alpha=0.14,
                label=label,
            )
            label_used = True

    def draw_segment_lines(ax, *, include_cleaning: bool, include_transitions: bool, tool: bool, chassis: bool) -> None:
        segment_ids: list[str] = []
        for pose in poses:
            segment_id = str(pose["segment_id"])
            if segment_id not in segment_ids:
                segment_ids.append(segment_id)
        tool_label_used = False
        chassis_label_used = False
        transition_label_used = False
        for segment_id in segment_ids:
            rows = [pose for pose in poses if str(pose["segment_id"]) == segment_id]
            if not rows:
                continue
            is_transition = rows[0]["primitive_type"] == "transition"
            if is_transition and not include_transitions:
                continue
            if not is_transition and not include_cleaning:
                continue
            linestyle = "--" if is_transition else "-"
            if tool:
                label = (
                    "transition tool path"
                    if is_transition and not transition_label_used
                    else "tool cleaning lane"
                    if not is_transition and not tool_label_used
                    else None
                )
                ax.plot(
                    [float(row["tool_world_x_m"]) for row in rows],
                    [float(row["tool_world_y_m"]) for row in rows],
                    color="tab:orange",
                    linestyle=linestyle,
                    linewidth=1.2 if is_transition else 2.1,
                    marker=".",
                    alpha=0.65 if is_transition else 1.0,
                    label=label,
                )
                if is_transition:
                    transition_label_used = True
                else:
                    tool_label_used = True
            if chassis:
                label = "derived chassis path" if not chassis_label_used else None
                ax.plot(
                    [float(row["world_x_m"]) for row in rows],
                    [float(row["world_y_m"]) for row in rows],
                    color="tab:blue",
                    linestyle=linestyle,
                    linewidth=0.9,
                    marker="o",
                    alpha=0.45,
                    label=label,
                )
                chassis_label_used = True

    def draw_transition_envelopes(ax) -> None:
        seen: set[str] = set()
        for pose in poses:
            if pose["primitive_type"] != "transition":
                continue
            segment_id = str(pose["segment_id"])
            if segment_id in seen:
                continue
            seen.add(segment_id)
            color = "tab:blue" if pose["transition_envelope_within_boundary"] is True else "red"
            ax.add_patch(
                Rectangle(
                    (float(pose["transition_envelope_x_min_m"]), float(pose["transition_envelope_y_min_m"])),
                    float(pose["transition_envelope_x_max_m"]) - float(pose["transition_envelope_x_min_m"]),
                    float(pose["transition_envelope_y_max_m"]) - float(pose["transition_envelope_y_min_m"]),
                    fill=False,
                    linestyle="--",
                    edgecolor=color,
                    linewidth=0.9,
                    alpha=0.7,
                    label="transition envelope" if len(seen) == 1 else None,
                )
            )

    def add_sample_polygons(ax, sample_type_filter: str | None, max_samples: int, alpha: float) -> None:
        filtered = [
            sample
            for sample in samples
            if sample_type_filter is None or sample.sample_type == sample_type_filter
        ]
        if not filtered:
            return
        stride = max(1, len(filtered) // max_samples)
        for sample in filtered[::stride]:
            color = "tab:green" if sample.inside_workspace else "red"
            ax.add_patch(
                PolygonPatch(
                    sample.chassis_polygon,
                    closed=True,
                    fill=False,
                    edgecolor=color,
                    linewidth=0.55,
                    alpha=alpha,
                )
            )
            ax.add_patch(
                PolygonPatch(
                    sample.tool_polygon,
                    closed=True,
                    fill=False,
                    edgecolor="tab:orange" if sample.inside_workspace else "red",
                    linewidth=0.55,
                    alpha=alpha,
                )
            )

    outputs: dict[str, Path | None] = {}

    overview = out_dir / "preview_overview.png"
    outputs["preview_overview.png"] = _write_png(path=overview, config=config, poses=poses)

    def tool_points() -> list[tuple[float, float]]:
        return [(float(row["tool_world_x_m"]), float(row["tool_world_y_m"])) for row in poses]

    fig, ax = setup_ax("Tool Path Primary")
    draw_uncovered_and_swept(ax)
    points = tool_points()
    if points:
        ax.plot(
            [point[0] for point in points],
            [point[1] for point in points],
            color="tab:orange",
            linewidth=2.8,
            marker="o",
            markersize=3,
            label="primary tool/paint-tank center path",
        )
    draw_segment_lines(ax, include_cleaning=True, include_transitions=True, tool=False, chassis=True)
    for pose in poses:
        label = str(pose.get("tool_waypoint_label", ""))
        if (
            label
            and (
                pose["segment_type"] in {"lane_start", "lane_end"}
                or str(pose["segment_id"]).startswith("transition_")
            )
            and int(pose.get("tool_route_index", pose["segment_index"])) % 2 == 0
        ):
            ax.text(
                float(pose["tool_world_x_m"]),
                float(pose["tool_world_y_m"]),
                label,
                fontsize=6,
                color="tab:orange",
                bbox={"facecolor": "white", "alpha": 0.65, "edgecolor": "none"},
            )
    ax.text(
        0.01,
        0.98,
        (
            "planner_primary_path=tool_center_path\n"
            f"tool_path_continuous={path_summary['tool_path_continuous']}\n"
            f"tool_track_count={path_summary['tool_track_count']} "
            f"tool_connector_count={path_summary['tool_connector_count']}"
        ),
        transform=ax.transAxes,
        fontsize=8,
        va="top",
        bbox={"facecolor": "white", "alpha": 0.82, "edgecolor": "none"},
    )
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    tool_primary_path = out_dir / "preview_tool_path_primary.png"
    fig.savefig(tool_primary_path, dpi=160)
    plt.close(fig)
    outputs["preview_tool_path_primary.png"] = tool_primary_path

    fig, ax = setup_ax("Chassis Derived From Tool Path")
    points = tool_points()
    if points:
        ax.plot(
            [point[0] for point in points],
            [point[1] for point in points],
            color="tab:orange",
            linewidth=2.4,
            marker=".",
            label="intended tool path",
        )
    draw_segment_lines(ax, include_cleaning=True, include_transitions=True, tool=False, chassis=True)
    for pose in poses[:: max(1, len(poses) // 18)]:
        ax.plot(
            [float(pose["world_x_m"]), float(pose["tool_world_x_m"])],
            [float(pose["world_y_m"]), float(pose["tool_world_y_m"])],
            color="gray",
            linewidth=0.6,
            alpha=0.5,
        )
    ax.text(
        0.01,
        0.98,
        (
            f"chassis_path_derived_from_tool={path_summary['chassis_path_derived_from_tool']}\n"
            f"tool_lateral_offset_m={config.tool_lateral_offset_m:.3f}\n"
            "tool = chassis + side * offset * normal(heading)"
        ),
        transform=ax.transAxes,
        fontsize=8,
        va="top",
        bbox={"facecolor": "white", "alpha": 0.82, "edgecolor": "none"},
    )
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    chassis_from_tool_path = out_dir / "preview_chassis_derived_from_tool.png"
    fig.savefig(chassis_from_tool_path, dpi=160)
    plt.close(fig)
    outputs["preview_chassis_derived_from_tool.png"] = chassis_from_tool_path

    fig, ax = setup_ax("Tool Sweep Pattern")
    draw_uncovered_and_swept(ax)
    for pose in poses:
        if pose["tool_segment_type"] != "tool_sweep_track":
            continue
        sx = float(pose["tool_world_x_m"])
        sy = float(pose["tool_world_y_m"])
        ex = sx + (float(pose.get("tool_end_x_m", pose["tool_x_m"])) - float(pose["tool_start_x_m"]))
        ey = sy + (float(pose.get("tool_end_y_m", pose["tool_y_m"])) - float(pose["tool_start_y_m"]))
        if abs(ex - sx) <= 1e-9 and abs(ey - sy) <= 1e-9:
            continue
        ax.annotate(
            "",
            xy=(ex, ey),
            xytext=(sx, sy),
            arrowprops={"arrowstyle": "->", "color": "tab:orange", "linewidth": 2.2},
        )
        ax.text(
            (sx + ex) / 2.0,
            (sy + ey) / 2.0,
            f"T{int(pose['lane_index'])}",
            fontsize=8,
            color="tab:orange",
            bbox={"facecolor": "white", "alpha": 0.70, "edgecolor": "none"},
        )
    for pose in poses:
        if pose["tool_segment_type"] != "tool_zigzag_connector":
            continue
        if not str(pose["segment_id"]).startswith("transition_"):
            continue
        sx = float(pose["tool_world_x_m"])
        sy = float(pose["tool_world_y_m"])
        ex = sx + (float(pose.get("tool_end_x_m", pose["tool_x_m"])) - float(pose["tool_start_x_m"]))
        ey = sy + (float(pose.get("tool_end_y_m", pose["tool_y_m"])) - float(pose["tool_start_y_m"]))
        if abs(ex - sx) <= 1e-9 and abs(ey - sy) <= 1e-9:
            continue
        ax.plot([sx, ex], [sy, ey], color="tab:orange", linestyle="--", linewidth=1.2, alpha=0.75)
    ax.text(
        0.01,
        0.98,
        "Arrows describe tool movement direction, not chassis lane arrows.",
        transform=ax.transAxes,
        fontsize=8,
        va="top",
        bbox={"facecolor": "white", "alpha": 0.82, "edgecolor": "none"},
    )
    fig.tight_layout()
    sweep_pattern_path = out_dir / "preview_sweep_pattern.png"
    fig.savefig(sweep_pattern_path, dpi=160)
    plt.close(fig)
    outputs["preview_sweep_pattern.png"] = sweep_pattern_path

    fig, ax = setup_ax("Alignment and First Cleaning Start")
    route_start_rows = [pose for pose in poses if str(pose["segment_id"]) == "route_start_A"]
    first_lane_rows = [pose for pose in poses if pose["segment_type"] == "lane_start"]
    if route_start_rows:
        ax.plot(
            [float(row["world_x_m"]) for row in route_start_rows],
            [float(row["world_y_m"]) for row in route_start_rows],
            color="tab:blue",
            marker="o",
            linewidth=1.2,
            label="tool-inactive pre-clean alignment",
        )
    if first_lane_rows:
        first = first_lane_rows[0]
        ax.scatter(
            [float(first["world_x_m"])],
            [float(first["world_y_m"])],
            color="tab:orange",
            s=55,
            label="first cleaning lane start",
        )
        heading = float(first["heading_deg"])
        ax.arrow(
            float(first["world_x_m"]),
            float(first["world_y_m"]),
            0.45 if abs(heading) < 1e-6 else -0.45,
            0.0,
            head_width=0.035,
            color="tab:orange",
            length_includes_head=True,
        )
    ax.text(
        0.01,
        0.98,
        (
            f"initial_heading={path_summary['initial_alignment_heading_deg']}\n"
            f"lane_axis_heading={path_summary['lane_axis_heading_deg']}\n"
            f"first_cleaning_heading={path_summary['first_cleaning_heading_deg']}\n"
            f"first_cleaning_heading_aligned={path_summary['first_cleaning_heading_aligned']}\n"
            f"tool_active_during_alignment={path_summary['tool_active_during_alignment']}"
        ),
        transform=ax.transAxes,
        fontsize=8,
        va="top",
        bbox={"facecolor": "white", "alpha": 0.82, "edgecolor": "none"},
    )
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    alignment_path = out_dir / "preview_alignment_and_start.png"
    fig.savefig(alignment_path, dpi=160)
    plt.close(fig)
    outputs["preview_alignment_and_start.png"] = alignment_path

    fig, ax = setup_ax("Primitive Sequence")
    for pose in poses:
        primitive = str(pose.get("movement_primitive_type", pose["primitive_type"]))
        sx = float(pose["world_x_m"])
        sy = float(pose["world_y_m"])
        ex = float(pose.get("ends_at_x_m", pose["x_m"]))
        ey = float(pose.get("ends_at_y_m", pose["y_m"]))
        if config.workspace_mode in {"ab_diagonal_center", "diagonal_ab"}:
            ex += float(path_summary["workspace_x_min_m"])
            ey += float(path_summary["workspace_y_min_m"])
        color = "tab:blue" if primitive in {"move_forward", "move_backward"} else "tab:purple"
        if primitive in {"move_forward", "move_backward"} and (abs(ex - sx) > 1e-9 or abs(ey - sy) > 1e-9):
            ax.annotate(
                "",
                xy=(ex, ey),
                xytext=(sx, sy),
                arrowprops={"arrowstyle": "->", "color": color, "linewidth": 1.0, "alpha": 0.65},
            )
        elif primitive in {"rotate_left", "rotate_right"}:
            marker = "$↺$" if primitive == "rotate_left" else "$↻$"
            ax.scatter([sx], [sy], color=color, marker=marker, s=70, alpha=0.75)
        if int(pose.get("primitive_index", pose["segment_index"])) % 3 == 0:
            ax.text(sx, sy, f"P{int(pose.get('primitive_index', pose['segment_index'])):03d}", fontsize=6)
    ax.text(
        0.01,
        0.98,
        (
            f"primitive_sequence_valid={path_summary['primitive_sequence_valid']}\n"
            "allowed: move_forward, move_backward, rotate_left, rotate_right"
        ),
        transform=ax.transAxes,
        fontsize=8,
        va="top",
        bbox={"facecolor": "white", "alpha": 0.82, "edgecolor": "none"},
    )
    fig.tight_layout()
    primitive_path = out_dir / "preview_primitive_sequence.png"
    fig.savefig(primitive_path, dpi=160)
    plt.close(fig)
    outputs["preview_primitive_sequence.png"] = primitive_path

    fig, ax = setup_ax("Tool Coverage Only")
    draw_uncovered_and_swept(ax)
    draw_segment_lines(ax, include_cleaning=True, include_transitions=False, tool=True, chassis=False)
    ax.legend()
    fig.tight_layout()
    tool_path = out_dir / "preview_tool_coverage_only.png"
    fig.savefig(tool_path, dpi=160)
    plt.close(fig)
    outputs["preview_tool_coverage_only.png"] = tool_path

    fig, ax = setup_ax("Boustrophedon Lane Pattern")
    lane_segments: list[tuple[dict[str, float | int | str | bool], dict[str, float | int | str | bool]]] = []
    lane_indexes = sorted(
        {
            int(pose["lane_index"])
            for pose in poses
            if pose["segment_type"] in {"lane_start", "lane_end"}
            and int(pose["lane_index"]) >= 0
        }
    )
    for lane_index in lane_indexes:
        starts = [
            pose
            for pose in poses
            if pose["segment_type"] == "lane_start" and int(pose["lane_index"]) == lane_index
        ]
        ends = [
            pose
            for pose in poses
            if pose["segment_type"] == "lane_end" and int(pose["lane_index"]) == lane_index
        ]
        if starts and ends:
            lane_segments.append((starts[0], ends[-1]))
    for start, end in lane_segments:
        sx = float(start["tool_world_x_m"])
        sy = float(start["tool_world_y_m"])
        ex = float(end["tool_world_x_m"])
        ey = float(end["tool_world_y_m"])
        lane_index = int(start["lane_index"])
        ax.annotate(
            "",
            xy=(ex, ey),
            xytext=(sx, sy),
            arrowprops={
                "arrowstyle": "->",
                "color": "tab:orange",
                "linewidth": 2.0,
                "shrinkA": 0,
                "shrinkB": 0,
            },
        )
        ax.text(
            (sx + ex) / 2.0,
            (sy + ey) / 2.0,
            f"lane {lane_index}",
            fontsize=8,
            color="tab:orange",
            bbox={"facecolor": "white", "alpha": 0.70, "edgecolor": "none"},
        )
        ax.plot(
            [float(start["world_x_m"]), float(end["world_x_m"])],
            [float(start["world_y_m"]), float(end["world_y_m"])],
            color="tab:blue",
            linewidth=0.8,
            alpha=0.35,
        )
    status = (
        "valid"
        if path_summary["boustrophedon_pattern_valid"] is True
        and path_summary["selected_lane_count_ok"] is True
        else "INFEASIBLE"
    )
    ax.text(
        0.01,
        0.98,
        (
            f"boustrophedon_pattern_valid={path_summary['boustrophedon_pattern_valid']}\n"
            f"selected_lane_count={path_summary['selected_lane_count']} "
            f"required_min_lane_count={path_summary['required_min_lane_count']}\n"
            f"status={status}"
        ),
        transform=ax.transAxes,
        fontsize=8,
        va="top",
        bbox={"facecolor": "white", "alpha": 0.82, "edgecolor": "none"},
    )
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    boustrophedon_path = out_dir / "preview_boustrophedon_pattern.png"
    fig.savefig(boustrophedon_path, dpi=160)
    plt.close(fig)
    outputs["preview_boustrophedon_pattern.png"] = boustrophedon_path

    fig, ax = setup_ax("Chassis Support Path Only")
    draw_segment_lines(ax, include_cleaning=True, include_transitions=True, tool=False, chassis=True)
    add_sample_polygons(ax, None, max_samples=80, alpha=0.24)
    ax.legend()
    fig.tight_layout()
    chassis_path = out_dir / "preview_chassis_only.png"
    fig.savefig(chassis_path, dpi=160)
    plt.close(fig)
    outputs["preview_chassis_only.png"] = chassis_path

    fig, ax = setup_ax("Internal Transitions Only")
    draw_segment_lines(ax, include_cleaning=False, include_transitions=True, tool=True, chassis=True)
    if config.show_transition_envelope:
        draw_transition_envelopes(ax)
    ax.legend()
    fig.tight_layout()
    transitions_path = out_dir / "preview_transitions_only.png"
    fig.savefig(transitions_path, dpi=160)
    plt.close(fig)
    outputs["preview_transitions_only.png"] = transitions_path

    fig, ax = setup_ax("Rotation Swept Volume")
    if config.show_transition_envelope:
        draw_transition_envelopes(ax)
    add_sample_polygons(ax, "rotation", max_samples=120, alpha=0.35)
    ax.legend()
    fig.tight_layout()
    rotation_path = out_dir / "preview_rotation_swept_volume.png"
    fig.savefig(rotation_path, dpi=160)
    plt.close(fig)
    outputs["preview_rotation_swept_volume.png"] = rotation_path

    fig, ax = setup_ax("Geometry Samples")
    add_sample_polygons(ax, None, max_samples=160, alpha=0.22)
    ax.legend()
    fig.tight_layout()
    samples_path = out_dir / "preview_geometry_samples.png"
    fig.savefig(samples_path, dpi=160)
    plt.close(fig)
    outputs["preview_geometry_samples.png"] = samples_path

    return outputs


def main(argv: Sequence[str] | None = None) -> int:
    raw_argv = list(sys.argv[1:] if argv is None else argv)
    advanced_mode = "--advanced" in raw_argv or "--debug-planner-options" in raw_argv
    if not advanced_mode:
        parser = build_simple_parser()
        args, unknown = parser.parse_known_args(raw_argv)
        if unknown:
            parser.error(
                "legacy/debug option(s) are not available in simple serpentine mode: "
                + " ".join(unknown)
                + ". Use --advanced to access legacy/debug parameters."
            )
        config = SideToolPlanConfig(
            workspace_mode="tool_serpentine_ab",
            a_x_m=args.a_x,
            a_y_m=args.a_y,
            b_x_m=args.b_x,
            b_y_m=args.b_y,
            step_spacing_m=args.step_spacing_m,
            tool_side=args.tool_side,
            tool_lateral_offset_m=args.tool_lateral_offset_m,
            tool_width_m=args.tool_width_m,
            tool_length_m=args.tool_length_m,
            robot_width_m=args.robot_width_m,
            robot_length_m=args.robot_length_m,
            robot_radius_m=0.14,
            coverage_resolution_m=args.coverage_resolution_m,
            tool_active_on_sweep_tracks=True,
            tool_active_on_connectors=False,
            tool_active_during_alignment=False,
            tool_active_during_transitions=False,
            tool_active_during_rotation=False,
            contamination_mode="off",
            show_transition_envelope=False,
            chassis_boundary_mode="derived_diagnostic",
        )
        return _run_tool_serpentine_preview_output(
            config=config,
            out_dir=Path(args.out_dir),
            tool_csv_name="tool_path.csv",
            emit_previews=args.emit_previews == "true",
        )

    advanced_argv = [
        arg for arg in raw_argv if arg not in {"--advanced", "--debug-planner-options"}
    ]
    args = build_parser().parse_args(advanced_argv)
    config = SideToolPlanConfig(
        tool_side=args.tool_side,
        tool_lateral_offset_m=args.tool_lateral_offset_m,
        tool_width_m=args.tool_width_m,
        tool_length_m=args.tool_length_m,
        lane_spacing_m=args.lane_spacing_m,
        step_spacing_m=args.step_spacing_m,
        tool_active_on_sweep_tracks=args.tool_active_on_sweep_tracks == "true",
        tool_active_on_connectors=args.tool_active_on_connectors == "true",
        force_end_at_B=args.force_end_at_B == "true",
        adjust_spacing_to_end_at_B=args.adjust_spacing_to_end_at_B == "true",
        sweep_axis=args.sweep_axis,
        row_length_m=args.row_length_m,
        row_count=args.row_count,
        start_x_m=args.start_x_m,
        start_y_m=args.start_y_m,
        start_heading_deg=args.start_heading_deg,
        first_lane_direction=args.first_lane_direction,
        transition_style=args.transition_style,
        workspace_mode=args.workspace_mode,
        a_x_m=args.a_x,
        a_y_m=args.a_y,
        b_x_m=args.b_x,
        b_y_m=args.b_y,
        workspace_side=args.workspace_side,
        surface_side=args.surface_side,
        workspace_width_m=args.workspace_width_m,
        y_min_m=args.y_min,
        y_max_m=args.y_max,
        robot_width_m=args.robot_width_m,
        robot_length_m=args.robot_length_m,
        robot_radius_m=args.robot_radius_m,
        auto_orient_tool_inside=args.auto_orient_tool_inside == "true",
        start_policy=args.start_policy,
        end_policy=args.end_policy,
        boundary_mode=args.boundary_mode,
        chassis_boundary_mode=args.chassis_boundary_mode,
        auto_inset_endpoints=args.auto_inset_endpoints == "true",
        boundary_margin_m=args.boundary_margin_m,
        turn_clearance_mode=args.turn_clearance_mode,
        fail_on_boundary_violation=args.fail_on_boundary_violation == "true",
        allow_uncovered_margins=args.allow_uncovered_margins == "true",
        coverage_resolution_m=args.coverage_resolution_m,
        rotation_sample_deg=args.rotation_sample_deg,
        translation_sample_m=args.translation_sample_m,
        swept_volume_validation=args.swept_volume_validation,
        kinematic_model=args.kinematic_model,
        contamination_mode=args.contamination_mode,
        fail_on_contamination_violation=args.fail_on_contamination_violation == "true",
        allow_start_end_chassis_outside_clean_surface=args.allow_start_end_chassis_outside_clean_surface == "true",
        tool_active_during_alignment=args.tool_active_during_alignment == "true",
        tool_active_during_transitions=args.tool_active_during_transitions == "true",
        tool_active_during_rotation=args.tool_active_during_rotation == "true",
        visualize_inactive_tool_ghost=args.visualize_inactive_tool_ghost == "true",
        show_transition_envelope=args.show_transition_envelope == "true",
        same_step_tool_before_chassis=args.same_step_tool_before_chassis == "true",
        route_order=args.route_order,
        coverage_pattern=args.coverage_pattern,
        min_lane_count=args.min_lane_count,
        forbid_single_lane_success=args.forbid_single_lane_success == "true",
        allow_tool_lift=args.allow_tool_lift == "true",
        reserve_dry_exit_corridor=args.reserve_dry_exit_corridor == "true",
        dry_corridor_strategy=args.dry_corridor_strategy,
        allow_contamination_best_effort=args.allow_contamination_best_effort == "true",
        require_start_at_A=None
        if args.require_start_at_A == "auto"
        else args.require_start_at_A == "true",
        require_end_at_B=None
        if args.require_end_at_B == "auto"
        else args.require_end_at_B == "true",
        require_exact_start=args.require_exact_start == "true",
        require_exact_end=args.require_exact_end == "true",
        max_start_error_m=args.max_start_error_m,
        max_end_error_m=args.max_end_error_m,
        max_end_offset_m=args.max_end_offset_m,
        allow_best_effort=args.allow_best_effort == "true",
        allow_unreached_b_best_effort=args.allow_unreached_b_best_effort == "true",
    )
    if config.workspace_mode == "tool_serpentine_ab":
        return _run_tool_serpentine_preview_output(
            config=config,
            out_dir=Path(args.out_dir),
            tool_csv_name="side_tool_path.csv",
            emit_previews=args.emit_separated_previews == "true",
            emit_timeline_frames=args.emit_timeline_frames == "true",
            emit_segment_frames=args.emit_segment_frames == "true",
        )
    poses = generate_side_tool_path(config)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    csv_path = out_dir / "side_tool_path.csv"
    geometry_csv_path = out_dir / "geometry_samples.csv"
    swept_json_path = out_dir / "swept_volume_summary.json"
    contamination_events_path = out_dir / "contamination_events.csv"
    strategy_diagnostics_csv_path = out_dir / "strategy_diagnostics.csv"
    strategy_diagnostics_md_path = out_dir / "strategy_diagnostics.md"
    timeline_state_path = out_dir / "timeline_state.csv"
    route_sequence_path = out_dir / "preview_route_sequence.md"
    summary_path = out_dir / "summary.md"
    png_candidate = out_dir / "preview.png"
    _write_csv(csv_path, poses)
    contamination = contamination_analysis(config, list(poses))
    diagnostic_rows = strategy_diagnostics(config)
    _write_strategy_diagnostics_csv(strategy_diagnostics_csv_path, diagnostic_rows)
    _write_strategy_diagnostics_md(strategy_diagnostics_md_path, diagnostic_rows)
    frame_paths: dict[int, Path] = {}
    timeline_contact_sheet_path: Path | None = None
    timeline_index_path: Path | None = None
    if args.emit_timeline_frames == "true" and args.contamination_mode != "off":
        frame_paths, timeline_contact_sheet_path, timeline_index_path = _write_timeline_frames(
            out_dir=out_dir,
            config=config,
            poses=poses,
            stride=args.timeline_frame_stride,
            max_frames=args.max_timeline_frames,
        )
    if args.emit_geometry_samples == "true":
        _write_geometry_samples_csv(path=geometry_csv_path, config=config, poses=poses, frame_paths=frame_paths)
        _write_swept_volume_json(path=swept_json_path, config=config, poses=poses)
    else:
        geometry_csv_path = None
        swept_json_path = None
    _write_contamination_events_csv(contamination_events_path, contamination)
    _write_timeline_state_csv(
        path=timeline_state_path,
        analysis=contamination,
        frame_paths=frame_paths,
        poses=poses,
    )
    png_path = _write_png(path=png_candidate, config=config, poses=poses)
    separated_preview_paths = (
        _write_separated_previews(out_dir=out_dir, config=config, poses=poses)
        if args.emit_separated_previews == "true"
        else None
    )
    contamination_preview_paths = (
        _write_contamination_previews(
            out_dir=out_dir,
            config=config,
            poses=poses,
            analysis=contamination,
        )
        if args.emit_contamination_previews == "true" and args.contamination_mode != "off"
        else None
    )
    chassis_vs_wet_path = (
        _write_chassis_vs_wet_preview(out_dir=out_dir, config=config, poses=poses)
        if args.emit_contamination_previews == "true" and args.contamination_mode != "off"
        else None
    )
    segment_frame_paths: dict[str, Path] = {}
    if args.emit_segment_frames == "true":
        segment_frame_paths, _segment_dir = _write_segment_frames(
            out_dir=out_dir,
            config=config,
            poses=poses,
        )
    _write_route_sequence_md(
        path=route_sequence_path,
        config=config,
        poses=poses,
        segment_frame_paths=segment_frame_paths,
    )
    _write_summary(
        path=summary_path,
        config=config,
        poses=poses,
        csv_path=csv_path,
        png_path=png_path,
        geometry_csv_path=geometry_csv_path,
        swept_json_path=swept_json_path,
        separated_preview_paths=separated_preview_paths,
        contamination_events_path=contamination_events_path,
        strategy_diagnostics_csv_path=strategy_diagnostics_csv_path,
        strategy_diagnostics_md_path=strategy_diagnostics_md_path,
        timeline_state_path=timeline_state_path,
        timeline_contact_sheet_path=timeline_contact_sheet_path,
        timeline_index_path=timeline_index_path,
        contamination_preview_paths=contamination_preview_paths,
        chassis_vs_wet_path=chassis_vs_wet_path,
        segment_frame_paths=segment_frame_paths,
        route_sequence_path=route_sequence_path,
        timeline_frame_count=len(frame_paths),
    )

    print("Side-mounted tool path preview generated.")
    print("Preview only: no HC-12 commands, no rover motor commands, no firmware upload.")
    print(f"CSV: {csv_path}")
    if geometry_csv_path is not None:
        print(f"Geometry samples CSV: {geometry_csv_path}")
    if swept_json_path is not None:
        print(f"Swept volume summary JSON: {swept_json_path}")
    print(f"Contamination events CSV: {contamination_events_path}")
    print(f"Strategy diagnostics CSV: {strategy_diagnostics_csv_path}")
    print(f"Strategy diagnostics Markdown: {strategy_diagnostics_md_path}")
    print(f"Timeline state CSV: {timeline_state_path}")
    if timeline_contact_sheet_path is not None:
        print(f"Timeline contact sheet: {timeline_contact_sheet_path}")
    if timeline_index_path is not None:
        print(f"Timeline HTML index: {timeline_index_path}")
    print(f"Route sequence markdown: {route_sequence_path}")
    if chassis_vs_wet_path is not None:
        print(f"Chassis vs wet preview: {chassis_vs_wet_path}")
    if segment_frame_paths:
        print(f"Segment frames: {len(segment_frame_paths)} files in {out_dir / 'segment_frames'}")
    print(f"Summary: {summary_path}")
    final_summary = summarize_side_tool_path(config, poses)
    print(f"Boundary violations: {final_summary['boundary_violation_count']}")
    print(f"Transition envelope violations: {final_summary['transition_envelope_violation_count']}")
    print(f"Rotation swept violations: {final_summary['rotation_swept_violation_count']}")
    print(f"Translation swept violations: {final_summary['translation_swept_violation_count']}")
    print(f"Combined swept violations: {final_summary['combined_swept_violation_count']}")
    print(f"Contamination mode: {final_summary['contamination_mode']}")
    print(f"Contamination free: {final_summary['contamination_free']}")
    print(f"Contamination violations: {final_summary['contamination_violation_count']}")
    print(f"Route order strategy: {final_summary['route_order_strategy']}")
    print(f"Differential-drive feasible: {final_summary['differential_drive_feasible']}")
    print(f"Route reaches B: {final_summary['route_reaches_B']}")
    print(f"Final distance to B m: {float(final_summary['final_distance_to_B_m']):.3f}")
    print(f"Required min lane count: {final_summary['required_min_lane_count']}")
    print(f"Selected lane count: {final_summary['selected_lane_count']}")
    print(f"Selected lane count OK: {final_summary['selected_lane_count_ok']}")
    print(f"Boustrophedon pattern valid: {final_summary['boustrophedon_pattern_valid']}")
    print(f"Physical-preview feasible: {final_summary['route_feasible_for_physical_preview']}")
    print(f"Infeasible reason: {final_summary['infeasible_reason']}")
    print(f"Coverage ratio: {float(final_summary['coverage_ratio']):.3f}")
    if png_path is None:
        print("Preview PNG: skipped because matplotlib is not available")
    else:
        print(f"Preview PNG: {png_path}")
    if separated_preview_paths:
        for label, preview_path in separated_preview_paths.items():
            print(f"{label}: {preview_path if preview_path is not None else 'skipped'}")
    if contamination_preview_paths:
        for label, preview_path in contamination_preview_paths.items():
            print(f"{label}: {preview_path if preview_path is not None else 'skipped'}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except ValueError as exc:
        raise SystemExit(f"ERROR: {exc}") from None
