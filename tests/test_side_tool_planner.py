import csv
import math

import pytest

from gps_coverage_core.side_tool_planner import (
    SideToolPlanConfig,
    chassis_polygon,
    contamination_analysis,
    footprint_sample,
    generate_side_tool_path,
    generate_tool_serpentine_preview,
    geometry_samples_for_path,
    summarize_side_tool_path,
    strategy_diagnostics,
    swept_volume_summary,
    tool_polygon,
    workspace_polygon,
    _workspace_info,
)
from tools import (
    preview_side_tool_path,
    preview_side_tool_waypoints,
    side_tool_path_preview,
    simulate_side_tool_tracking,
)


def _lane_starts(path):
    return [pose for pose in path if pose["segment_type"] == "lane_start"]


def _lane_directions(path):
    return [pose["motion_direction"] for pose in _lane_starts(path)]


def _workspace_config(tool_side="left", **overrides):
    values = dict(
        workspace_mode="diagonal_ab",
        a_x_m=0.0,
        a_y_m=0.0,
        b_x_m=8.0,
        b_y_m=1.2,
        tool_side=tool_side,
        tool_lateral_offset_m=0.24,
        tool_width_m=0.30,
        lane_spacing_m=0.25,
        row_count="auto",
        robot_width_m=0.18,
        robot_length_m=0.18,
        robot_radius_m=0.14,
        first_lane_direction="forward",
        transition_style="side_step_reverse_90",
        boundary_mode="strict",
        chassis_boundary_mode="clean_surface_strict",
        auto_inset_endpoints=True,
        boundary_margin_m=0.03,
        auto_orient_tool_inside=True,
        fail_on_boundary_violation=True,
        contamination_mode="off",
        fail_on_contamination_violation=False,
        require_start_at_A=False,
        require_end_at_B=False,
    )
    values.update(overrides)
    return SideToolPlanConfig(**values)


def _polygon_center(poly):
    return (
        sum(point[0] for point in poly) / len(poly),
        sum(point[1] for point in poly) / len(poly),
    )


def test_rover18_and_assumed_tool_defaults() -> None:
    config = SideToolPlanConfig(a_x_m=0.0, a_y_m=0.0, b_x_m=8.0, b_y_m=1.2)

    assert config.workspace_mode == "tool_serpentine_ab"
    assert config.robot_width_m == pytest.approx(0.18)
    assert config.robot_length_m == pytest.approx(0.18)
    assert config.robot_radius_m == pytest.approx(0.14)
    assert config.tool_width_m == pytest.approx(0.30)
    assert config.tool_lateral_offset_m == pytest.approx(0.24)
    assert config.tool_side == "left"
    assert config.tool_active_on_sweep_tracks is True
    assert config.tool_active_on_connectors is False
    assert config.lane_spacing_m == pytest.approx(0.25)
    assert config.boundary_margin_m == pytest.approx(0.03)
    assert config.tool_length_m == pytest.approx(0.18)
    assert config.rotation_sample_deg == pytest.approx(5.0)
    assert config.translation_sample_m == pytest.approx(0.05)
    assert config.swept_volume_validation == "strict"
    assert config.contamination_mode == "off"


def test_tool_serpentine_ab_preserves_top_left_A_and_bottom_right_B() -> None:
    config = SideToolPlanConfig(
        workspace_mode="tool_serpentine_ab",
        a_x_m=0.0,
        a_y_m=1.2,
        b_x_m=8.0,
        b_y_m=0.0,
        step_spacing_m=0.25,
    )
    preview = generate_tool_serpentine_preview(config)
    summary = preview["summary"]
    tool_segments = preview["tool_segments"]

    assert workspace_polygon(config) == pytest.approx([(0.0, 0.0), (8.0, 0.0), (8.0, 1.2), (0.0, 1.2)])
    assert summary["workspace_mode"] == "tool_serpentine_ab"
    assert summary["A_corner_role"] == "top_left"
    assert summary["B_corner_role"] == "bottom_right"
    assert tool_segments[0]["tool_start_x_m"] == pytest.approx(0.0)
    assert tool_segments[0]["tool_start_y_m"] == pytest.approx(1.2)
    assert tool_segments[-1]["tool_end_x_m"] == pytest.approx(8.0)
    assert tool_segments[-1]["tool_end_y_m"] == pytest.approx(0.0)
    assert summary["tool_path_starts_at_A"] is True
    assert summary["tool_path_ends_at_B"] is True


def test_tool_serpentine_step_spacing_derives_tracks_and_connectors() -> None:
    config = SideToolPlanConfig(
        workspace_mode="tool_serpentine_ab",
        a_x_m=0.0,
        a_y_m=1.2,
        b_x_m=8.0,
        b_y_m=0.0,
        step_spacing_m=0.25,
        adjust_spacing_to_end_at_B=True,
        force_end_at_B=True,
    )
    preview = generate_tool_serpentine_preview(config)
    summary = preview["summary"]
    tool_segments = preview["tool_segments"]
    tracks = [segment for segment in tool_segments if segment["tool_segment_type"] == "tool_sweep_track"]
    connectors = [segment for segment in tool_segments if segment["tool_segment_type"] == "tool_spacing_connector"]

    assert summary["derived_track_count"] == 7
    assert summary["track_count_parity"] == "odd"
    assert summary["tool_track_count"] == 7
    assert summary["tool_connector_count"] == 6
    assert summary["tool_connector_count_equals_track_count_minus_one"] is True
    assert summary["tool_path_continuous"] is True
    assert summary["actual_spacing_values_m"] == "0.200000,0.200000,0.200000,0.200000,0.200000,0.200000"
    assert [track["tool_start_y_m"] for track in tracks] == pytest.approx([1.2, 1.0, 0.8, 0.6, 0.4, 0.2, 0.0])
    assert len(connectors) == len(tracks) - 1
    assert {connector["tool_active"] for connector in connectors} == {False}
    assert {connector["coverage_contributes"] for connector in connectors} == {False}
    for index in range(len(tool_segments) - 1):
        assert tool_segments[index]["tool_end_x_m"] == pytest.approx(tool_segments[index + 1]["tool_start_x_m"])
        assert tool_segments[index]["tool_end_y_m"] == pytest.approx(tool_segments[index + 1]["tool_start_y_m"])


def test_tool_serpentine_chassis_and_primitives_are_derived_from_tool_path() -> None:
    config = SideToolPlanConfig(
        workspace_mode="tool_serpentine_ab",
        a_x_m=0.0,
        a_y_m=1.2,
        b_x_m=8.0,
        b_y_m=0.0,
        step_spacing_m=0.25,
    )
    preview = generate_tool_serpentine_preview(config)
    chassis_segments = preview["chassis_segments"]
    primitive_rows = preview["primitive_rows"]
    summary = preview["summary"]

    assert summary["planner_primary_path"] == "tool_center_path"
    assert summary["chassis_path_role"] == "derived_support_path"
    assert summary["chassis_path_derived_from_tool"] is True
    assert summary["primitive_sequence_valid"] is True
    assert {row["primitive_type"] for row in primitive_rows} <= {
        "move_forward",
        "move_backward",
        "rotate_left",
        "rotate_right",
    }
    assert all(segment["chassis_path_derived_from_tool"] is True for segment in chassis_segments)
    assert {row["motor_command_generated"] for row in primitive_rows} == {False}


def test_ab_centerline_width_treats_ab_as_rover_center_start_end() -> None:
    config = SideToolPlanConfig(
        workspace_mode="ab_centerline_width",
        a_x_m=0.0,
        a_y_m=0.0,
        b_x_m=8.0,
        b_y_m=0.0,
        surface_side="left",
        workspace_width_m=1.2,
        tool_side="left",
        contamination_mode="strict",
        fail_on_contamination_violation=True,
    )

    path = generate_side_tool_path(config)
    summary = summarize_side_tool_path(config, path)

    assert path[0]["segment_type"] == "route_start"
    assert path[0]["x_m"] == pytest.approx(0.0)
    assert path[0]["y_m"] == pytest.approx(0.0)
    assert path[-1]["segment_type"] == "route_end"
    assert path[-1]["x_m"] == pytest.approx(8.0)
    assert path[-1]["y_m"] == pytest.approx(0.0)
    assert summary["workspace_mode"] == "ab_centerline_width"
    assert summary["A_B_are_rover_center_start_end"] is True
    assert summary["start_reaches_A"] is True
    assert summary["end_reaches_B"] is True
    assert summary["route_reaches_B"] is True
    assert summary["route_feasible_for_physical_preview"] is False
    assert "WORKSPACE_MODE_NOT_AB_DIAGONAL_CENTER" in summary["infeasible_reason"]
    assert summary["differential_drive_feasible"] is True
    assert all(pose["motor_command_generated"] is False for pose in path)


def test_ab_diagonal_center_treats_ab_as_start_end_and_rectangle_diagonal() -> None:
    config = SideToolPlanConfig(
        workspace_mode="ab_diagonal_center",
        a_x_m=0.0,
        a_y_m=0.0,
        b_x_m=8.0,
        b_y_m=1.2,
        tool_side="left",
        contamination_mode="off",
        fail_on_contamination_violation=False,
    )

    path = generate_side_tool_path(config)
    summary = summarize_side_tool_path(config, path)

    assert workspace_polygon(config) == pytest.approx(
        [(0.0, 0.0), (8.0, 0.0), (8.0, 1.2), (0.0, 1.2)]
    )
    assert path[0]["segment_type"] == "route_start"
    assert path[0]["x_m"] == pytest.approx(0.0)
    assert path[0]["y_m"] == pytest.approx(0.0)
    assert path[-1]["segment_type"] == "route_end"
    assert path[-1]["x_m"] == pytest.approx(8.0)
    assert path[-1]["y_m"] == pytest.approx(1.2)
    assert summary["workspace_mode"] == "ab_diagonal_center"
    assert summary["A_B_are_rover_center_start_end"] is True
    assert summary["A_B_are_rectangle_diagonal"] is True
    assert summary["start_reaches_A"] is True
    assert summary["end_reaches_B"] is True
    assert summary["route_reaches_B"] is True


def test_single_lane_is_not_physical_preview_feasible_when_multiple_lanes_required() -> None:
    config = SideToolPlanConfig(
        workspace_mode="ab_diagonal_center",
        a_x_m=0.0,
        a_y_m=0.0,
        b_x_m=8.0,
        b_y_m=1.2,
        tool_side="left",
        row_count=1,
        coverage_pattern="boustrophedon_required",
        forbid_single_lane_success=True,
        contamination_mode="off",
        fail_on_contamination_violation=False,
    )

    path = generate_side_tool_path(config)
    summary = summarize_side_tool_path(config, path)

    assert summary["required_min_lane_count"] > 1
    assert summary["selected_lane_count"] == 1
    assert summary["selected_lane_count_ok"] is False
    assert summary["boustrophedon_pattern_valid"] is False
    assert summary["route_feasible_for_physical_preview"] is False
    assert "SELECTED_LANE_COUNT_BELOW_REQUIRED" in summary["infeasible_reason"]


def test_ab_diagonal_center_route_expands_to_four_physical_primitives() -> None:
    config = SideToolPlanConfig(
        workspace_mode="ab_diagonal_center",
        a_x_m=0.0,
        a_y_m=0.0,
        b_x_m=8.0,
        b_y_m=1.2,
        tool_side="right",
        transition_style="auto_internal",
        contamination_mode="off",
        fail_on_contamination_violation=False,
    )

    path = generate_side_tool_path(config)
    summary = summarize_side_tool_path(config, path)
    allowed = {"move_forward", "move_backward", "rotate_left", "rotate_right"}
    first_lane = _lane_starts(path)[0]

    assert summary["primitive_sequence_valid"] is True
    assert {pose["movement_primitive_type"] for pose in path} <= allowed
    assert path[0]["x_m"] == pytest.approx(0.0)
    assert path[0]["y_m"] == pytest.approx(0.0)
    assert path[-1]["x_m"] == pytest.approx(8.0)
    assert path[-1]["y_m"] == pytest.approx(1.2)
    assert float(first_lane["heading_deg"]) in {0.0, 180.0}
    assert first_lane["first_cleaning_heading_aligned"] is True
    assert all(pose["motor_command_generated"] is False for pose in path)


def test_tool_center_path_is_primary_and_continuous() -> None:
    config = SideToolPlanConfig(
        workspace_mode="ab_diagonal_center",
        a_x_m=0.0,
        a_y_m=0.0,
        b_x_m=8.0,
        b_y_m=1.2,
        tool_side="right",
        transition_style="auto_internal",
        contamination_mode="off",
        fail_on_contamination_violation=False,
    )
    path = generate_side_tool_path(config)
    summary = summarize_side_tool_path(config, path)

    assert summary["planner_primary_path"] == "tool_center_path"
    assert summary["chassis_path_role"] == "derived_support_path"
    assert summary["tool_path_continuous"] is True
    assert summary["tool_connector_count"] == summary["tool_track_count"] - 1
    assert summary["chassis_path_derived_from_tool"] is True
    assert {pose["tool_path_primary"] for pose in path} == {True}
    assert all(
        float(path[index]["tool_end_x_m"]) == pytest.approx(float(path[index + 1]["tool_start_x_m"]))
        and float(path[index]["tool_end_y_m"]) == pytest.approx(float(path[index + 1]["tool_start_y_m"]))
        for index in range(len(path) - 1)
    )


def test_tool_connector_angles_and_distances_are_from_tool_waypoints() -> None:
    config = SideToolPlanConfig(
        workspace_mode="ab_diagonal_center",
        a_x_m=0.0,
        a_y_m=0.0,
        b_x_m=8.0,
        b_y_m=1.2,
        tool_side="right",
        transition_style="auto_internal",
        contamination_mode="off",
        fail_on_contamination_violation=False,
    )
    path = generate_side_tool_path(config)
    connectors = [
        pose
        for pose in path
        if pose["tool_segment_type"] == "tool_zigzag_connector"
        and str(pose["segment_id"]).startswith("transition_")
        and float(pose["tool_connector_distance_m"]) > 1e-9
    ]

    assert connectors
    for pose in connectors:
        dx = float(pose["tool_end_x_m"]) - float(pose["tool_start_x_m"])
        dy = float(pose["tool_end_y_m"]) - float(pose["tool_start_y_m"])
        assert float(pose["tool_connector_distance_m"]) == pytest.approx(math.hypot(dx, dy))
        assert float(pose["tool_connector_angle_deg"]) == pytest.approx(math.degrees(math.atan2(dy, dx)))


def test_derived_chassis_pose_places_tool_on_intended_tool_path() -> None:
    config = SideToolPlanConfig(
        workspace_mode="ab_diagonal_center",
        a_x_m=0.0,
        a_y_m=0.0,
        b_x_m=8.0,
        b_y_m=1.2,
        tool_side="right",
        transition_style="auto_internal",
        contamination_mode="off",
        fail_on_contamination_violation=False,
    )
    path = generate_side_tool_path(config)
    side_sign = -1.0
    for pose in path:
        heading = math.radians(float(pose["heading_deg"]) + 90.0)
        normal_x = math.cos(heading)
        normal_y = math.sin(heading)
        derived_x = float(pose["tool_start_x_m"]) - side_sign * config.tool_lateral_offset_m * normal_x
        derived_y = float(pose["tool_start_y_m"]) - side_sign * config.tool_lateral_offset_m * normal_y
        assert derived_x == pytest.approx(float(pose["x_m"]))
        assert derived_y == pytest.approx(float(pose["y_m"]))
        assert pose["chassis_path_derived_from_tool"] is True


def test_ab_centerline_shifted_start_is_not_physical_preview_feasible() -> None:
    config = SideToolPlanConfig(
        workspace_mode="diagonal_ab",
        a_x_m=0.0,
        a_y_m=0.0,
        b_x_m=8.0,
        b_y_m=1.2,
        tool_side="left",
        require_start_at_A=True,
        require_end_at_B=True,
        allow_best_effort=True,
        contamination_mode="off",
        fail_on_contamination_violation=False,
    )

    path = generate_side_tool_path(config)
    summary = summarize_side_tool_path(config, path)

    assert summary["start_reaches_A"] is False
    assert summary["route_reaches_B"] is False
    assert summary["route_feasible_for_physical_preview"] is False


def test_chassis_rectangle_polygon_dimensions_and_orientation() -> None:
    config = _workspace_config("left")
    poly = chassis_polygon(config, x_m=1.0, y_m=0.6, heading_deg=0.0)

    xs = [point[0] for point in poly]
    ys = [point[1] for point in poly]
    assert max(xs) - min(xs) == pytest.approx(0.18)
    assert max(ys) - min(ys) == pytest.approx(0.18)

    rotated = chassis_polygon(config, x_m=1.0, y_m=0.6, heading_deg=90.0)
    rxs = [point[0] for point in rotated]
    rys = [point[1] for point in rotated]
    assert max(rxs) - min(rxs) == pytest.approx(0.18)
    assert max(rys) - min(rys) == pytest.approx(0.18)


def test_side_mounted_tool_polygon_offsets_left_and_right() -> None:
    left_config = _workspace_config("left")
    right_config = _workspace_config("right")
    left_center = _polygon_center(
        tool_polygon(left_config, chassis_x_m=1.0, chassis_y_m=0.6, heading_deg=0.0)
    )
    right_center = _polygon_center(
        tool_polygon(right_config, chassis_x_m=1.0, chassis_y_m=0.6, heading_deg=0.0)
    )

    assert left_center == pytest.approx((1.0, 0.84))
    assert right_center == pytest.approx((1.0, 0.36))


def test_diagonal_ab_workspace_rectangle_uses_opposite_corners() -> None:
    polygon = workspace_polygon(_workspace_config("left"))

    assert polygon == pytest.approx([(0.0, 0.0), (8.0, 0.0), (8.0, 1.2), (0.0, 1.2)])
    assert (0.0, 0.0) in polygon
    assert (8.0, 1.2) in polygon


def test_diagonal_ab_zero_width_fails_clearly() -> None:
    config = SideToolPlanConfig(
        workspace_mode="diagonal_ab",
        a_x_m=0.0,
        a_y_m=0.0,
        b_x_m=8.0,
        b_y_m=0.0,
    )

    with pytest.raises(ValueError, match="DIAGONAL_AB_WIDTH_ZERO"):
        generate_side_tool_path(config)


def test_axis_width_mode_remains_available() -> None:
    config = SideToolPlanConfig(
        workspace_mode="axis_width",
        a_x_m=0.0,
        a_y_m=0.0,
        b_x_m=8.0,
        b_y_m=0.0,
        workspace_side="left",
        workspace_width_m=1.2,
    )

    assert workspace_polygon(config) == pytest.approx([(0.0, 0.0), (8.0, 0.0), (8.0, 1.2), (0.0, 1.2)])


def test_left_tool_workspace_left_stays_inside_ab_rectangle() -> None:
    config = _workspace_config("left")
    path = generate_side_tool_path(config)
    summary = summarize_side_tool_path(config, path)

    assert summary["boundary_violation_count"] == 0
    assert all(pose["within_boundary"] is True for pose in path)
    assert all(pose["chassis_within_boundary"] is True for pose in path)
    assert all(pose["tool_within_boundary"] is True for pose in path)
    assert min(float(pose["tool_edge_min_y_m"]) for pose in path) >= 0.03
    assert max(float(pose["tool_edge_max_y_m"]) for pose in path) <= 1.17
    assert summary["uncovered_margin_low_m"] > 0.0
    assert summary["coverage_ratio"] < 1.0


def test_right_tool_workspace_left_flips_heading_and_uses_reverse_motion() -> None:
    path = generate_side_tool_path(_workspace_config("right"))
    starts = _lane_starts(path)

    assert {float(pose["heading_deg"]) for pose in starts} == {0.0, 180.0}
    assert {pose["tool_world_side"] for pose in starts} == {"above_chassis", "below_chassis"}
    assert starts[0]["motion_direction"] == "forward"
    assert starts[0]["travel_direction_deg"] == pytest.approx(0.0)
    assert all(pose["tool_within_boundary"] is True for pose in path)


def test_left_and_right_tool_cases_stay_inside() -> None:
    for tool_side in ("left", "right"):
        config = _workspace_config(tool_side)
        path = generate_side_tool_path(config)
        summary = summarize_side_tool_path(config, path)

        assert summary["workspace_mode"] == "diagonal_ab"
        assert summary["a_b_semantic"] == "opposite_corners"
        assert summary["boundary_violation_count"] == 0
        assert {pose["workspace_mode"] for pose in path} == {"diagonal_ab"}
        assert all(pose["motor_command_generated"] is False for pose in path)


def test_chassis_radius_and_tool_edges_are_checked() -> None:
    path = generate_side_tool_path(_workspace_config("left"))

    for pose in path:
        assert pose["chassis_within_boundary"] is True
        assert pose["tool_within_boundary"] is True
        assert 0.0 <= float(pose["x_m"]) <= 8.0
        assert 0.0 <= float(pose["y_m"]) <= 1.2
        assert 0.0 <= float(pose["tool_edge_min_y_m"]) <= 1.2
        assert 0.0 <= float(pose["tool_edge_max_y_m"]) <= 1.2


def test_combined_footprint_sample_passes_when_inset() -> None:
    config = _workspace_config("left")
    info = _workspace_info(config)
    sample = footprint_sample(
        config,
        info,
        sample_index=0,
        segment_id="sample",
        sample_type="rotation",
        x_m=1.0,
        y_m=0.6,
        heading_deg=90.0,
    )

    assert sample.inside_workspace is True
    assert sample.chassis_inside_workspace is True
    assert sample.tool_inside_workspace is True
    assert sample.violation_reason == "OK"


def test_rotation_swept_sampling_detects_boundary_violation_near_edge() -> None:
    config = _workspace_config("left")
    info = _workspace_info(config)
    sample = footprint_sample(
        config,
        info,
        sample_index=0,
        segment_id="edge",
        sample_type="rotation",
        x_m=0.1,
        y_m=0.1,
        heading_deg=90.0,
    )

    assert sample.inside_workspace is False
    assert "ROTATION_COMBINED_SWEEP_EXITS_WORKSPACE" in sample.violation_reason


def test_geometry_samples_include_translation_and_rotation_sweeps() -> None:
    config = _workspace_config("left", transition_style="auto_internal")
    path = generate_side_tool_path(config)
    samples = geometry_samples_for_path(config, path)
    summary = swept_volume_summary(config, path)

    assert samples
    assert {"lane_translation", "transition_translation", "rotation"} <= {
        sample.sample_type for sample in samples
    }
    assert summary["physical_swept_volume_validated"] is True
    assert summary["combined_swept_violation_count"] == 0
    assert summary["rotation_swept_violation_count"] == 0
    assert summary["translation_swept_violation_count"] == 0


def test_impossible_geometry_fails_without_protruding() -> None:
    config = _workspace_config(
        b_y_m=0.45,
        tool_lateral_offset_m=0.40,
        tool_width_m=0.40,
        robot_width_m=0.40,
        robot_length_m=0.60,
    )

    with pytest.raises(ValueError, match="NO_FEASIBLE_TOOL_PLACEMENT"):
        generate_side_tool_path(config)


def test_auto_row_count_uses_feasible_coverage_lanes_and_reports_margins() -> None:
    config = _workspace_config("left", row_count="auto")
    path = generate_side_tool_path(config)
    summary = summarize_side_tool_path(config, path)
    lane_starts = _lane_starts(path)

    assert summary["actual_lane_count"] == len(lane_starts)
    assert len(lane_starts) >= 2
    assert summary["requested_coverage_y_min_m"] == pytest.approx(0.0)
    assert summary["requested_coverage_y_max_m"] == pytest.approx(1.2)
    assert summary["uncovered_margin_low_m"] > 0.0
    assert summary["uncovered_margin_high_m"] >= 0.0
    assert all(pose["within_boundary"] is True for pose in path)


def test_coverage_area_is_computed_from_tool_footprint_only() -> None:
    config = _workspace_config("left")
    path = generate_side_tool_path(config)
    summary = summarize_side_tool_path(config, path)
    inset = float(summary["endpoint_inset_m"])

    assert summary["requested_coverage_area_m2"] == pytest.approx(8.0 * 1.2)
    assert summary["tool_swept_x_min_m"] == pytest.approx(inset)
    assert summary["tool_swept_x_max_m"] == pytest.approx(8.0 - inset)
    assert summary["tool_swept_y_min_m"] == pytest.approx(float(summary["uncovered_margin_low_m"]))
    assert summary["tool_swept_y_max_m"] == pytest.approx(1.2 - float(summary["uncovered_margin_high_m"]))
    assert summary["covered_area_m2"] > 7.5
    assert summary["uncovered_area_m2"] == pytest.approx(8.0 * 1.2 - float(summary["covered_area_m2"]))
    assert summary["coverage_ratio"] == pytest.approx(float(summary["covered_area_m2"]) / (8.0 * 1.2))
    assert summary["coverage_ratio"] > 0.8


def test_chassis_only_strip_does_not_count_as_cleaned() -> None:
    config = _workspace_config("left")
    path = generate_side_tool_path(config)
    summary = summarize_side_tool_path(config, path)
    first_lane_start = _lane_starts(path)[0]

    assert 0.0 < float(first_lane_start["y_m"]) < 1.2
    assert not (
        float(first_lane_start["tool_edge_min_y_m"])
        <= float(first_lane_start["y_m"])
        <= float(first_lane_start["tool_edge_max_y_m"])
    )
    assert float(summary["uncovered_margin_low_m"]) == pytest.approx(
        float(summary["endpoint_inset_m"]) - config.tool_lateral_offset_m - config.tool_width_m / 2.0
    )
    assert first_lane_start["coverage_contributes"] is True


def test_transitions_do_not_contribute_to_cleaned_coverage() -> None:
    path = generate_side_tool_path(_workspace_config("left"))
    transition_rows = [pose for pose in path if pose["coverage_role"] == "transition"]

    assert transition_rows
    assert all(pose["coverage_contributes"] is False for pose in transition_rows)
    assert all(pose["motor_command_generated"] is False for pose in transition_rows)


def _manual_contamination_pose(segment_id, segment_type, lane_index, x_m, y_m):
    return {
        "segment_id": segment_id,
        "segment_type": segment_type,
        "lane_index": lane_index,
        "primitive_type": "cleaning_lane",
        "x_m": x_m,
        "y_m": y_m,
        "heading_deg": 0.0,
    }


def test_chassis_over_prior_tool_swept_cell_is_forbidden() -> None:
    config = _workspace_config(
        "left",
        b_x_m=2.0,
        b_y_m=1.5,
        contamination_mode="strict",
        fail_on_contamination_violation=True,
    )
    poses = [
        _manual_contamination_pose("lane_0", "lane_start", 0, 0.4, 0.30),
        _manual_contamination_pose("lane_0", "lane_end", 0, 1.6, 0.30),
        _manual_contamination_pose("lane_1", "lane_start", 1, 0.4, 0.54),
        _manual_contamination_pose("lane_1", "lane_end", 1, 1.6, 0.54),
    ]

    analysis = contamination_analysis(config, poses)

    assert analysis["contamination_checked"] is True
    assert analysis["contamination_free"] is False
    assert analysis["contamination_violation_count"] > 0
    assert "CHASSIS_ON_PRIOR_TOOL_SWEPT_AREA" in analysis["events"][0].violation_reason


def test_chassis_over_future_tool_swept_cell_is_allowed() -> None:
    config = _workspace_config(
        "left",
        b_x_m=2.0,
        b_y_m=1.5,
        contamination_mode="strict",
        fail_on_contamination_violation=True,
    )
    poses = [
        _manual_contamination_pose("lane_0", "lane_start", 0, 0.4, 0.54),
        _manual_contamination_pose("lane_0", "lane_end", 0, 1.6, 0.54),
        _manual_contamination_pose("lane_1", "lane_start", 1, 0.4, 0.30),
        _manual_contamination_pose("lane_1", "lane_end", 1, 1.6, 0.30),
    ]

    analysis = contamination_analysis(config, poses)

    assert analysis["contamination_free"] is True
    assert analysis["contamination_violation_count"] == 0


def test_tool_over_prior_tool_swept_area_is_allowed_and_tracked() -> None:
    config = _workspace_config(
        "left",
        b_x_m=2.0,
        b_y_m=1.5,
        contamination_mode="strict",
        fail_on_contamination_violation=True,
    )
    poses = [
        _manual_contamination_pose("lane_0", "lane_start", 0, 0.4, 0.30),
        _manual_contamination_pose("lane_0", "lane_end", 0, 1.6, 0.30),
        _manual_contamination_pose("lane_1", "lane_start", 1, 0.4, 0.30),
        _manual_contamination_pose("lane_1", "lane_end", 1, 1.6, 0.30),
    ]

    analysis = contamination_analysis(config, poses)

    assert analysis["contamination_free"] is True
    assert analysis["tool_reclean_area_m2"] > 0.0


def test_auto_temporal_safe_does_not_accept_single_lane_to_avoid_contamination() -> None:
    config = _workspace_config(
        "left",
        transition_style="auto_internal",
        contamination_mode="strict",
        fail_on_contamination_violation=True,
    )
    path = generate_side_tool_path(config)
    summary = summarize_side_tool_path(config, path)

    assert summary["contamination_free"] is False
    assert summary["contamination_violation_count"] > 0
    assert summary["route_order_strategy"] in {
        "bottom_to_top",
        "top_to_bottom",
        "A_to_B_frontier",
        "B_to_A_retreat",
        "wet_frontier_retreat",
        "dry_corridor_preserving",
        "far_side_to_exit",
        "exit_side_to_far",
        "snake_from_A_to_B",
        "reverse_snake_to_B",
        "tool_lift_transitions",
        "dry_frontier_preserving",
        "clean_from_far_side_to_exit",
        "fixed_serpentine",
    }
    assert summary["selected_lane_count"] >= summary["required_min_lane_count"]
    assert summary["coverage_ratio"] > 0.5
    assert summary["route_reaches_B"] is False
    assert summary["route_feasible_for_physical_preview"] is False
    assert "CONTAMINATION_VIOLATION" in summary["infeasible_reason"]


def test_best_effort_full_coverage_reports_contamination_when_allowed() -> None:
    config = _workspace_config(
        "left",
        transition_style="auto_internal",
        route_order="bottom_to_top",
        contamination_mode="warn",
        fail_on_contamination_violation=False,
        allow_contamination_best_effort=True,
    )
    path = generate_side_tool_path(config)
    summary = summarize_side_tool_path(config, path)

    assert summary["coverage_ratio"] > 0.8
    assert summary["contamination_free"] is False
    assert summary["contamination_violation_count"] > 0
    assert summary["chassis_after_cleaned_area_m2"] > 0.0


def test_left_and_right_tool_cases_report_same_feasible_tool_coverage() -> None:
    left_config = _workspace_config("left")
    right_config = _workspace_config("right")
    left_summary = summarize_side_tool_path(left_config, generate_side_tool_path(left_config))
    right_summary = summarize_side_tool_path(right_config, generate_side_tool_path(right_config))

    assert right_summary["covered_area_m2"] == pytest.approx(float(left_summary["covered_area_m2"]))
    assert right_summary["uncovered_area_m2"] == pytest.approx(float(left_summary["uncovered_area_m2"]))
    assert right_summary["coverage_ratio"] == pytest.approx(float(left_summary["coverage_ratio"]))


def test_transition_envelopes_fold_inside_at_x_min_and_x_max() -> None:
    config = _workspace_config("left", b_y_m=2.0, row_count=4)
    path = generate_side_tool_path(config)
    transition_rows = [pose for pose in path if pose["coverage_role"] == "transition"]

    assert transition_rows
    assert {pose["transition_pocket_side"] for pose in transition_rows} == {
        "x_min_interior",
        "x_max_interior",
    }
    assert all(pose["transition_envelope_within_boundary"] is True for pose in transition_rows)
    assert all(float(pose["transition_envelope_x_min_m"]) >= 0.03 - 1e-9 for pose in transition_rows)
    assert all(float(pose["transition_envelope_x_max_m"]) <= 7.97 + 1e-9 for pose in transition_rows)
    assert all(float(pose["transition_envelope_y_min_m"]) >= 0.03 - 1e-9 for pose in transition_rows)
    assert all(float(pose["transition_envelope_y_max_m"]) <= 1.97 + 1e-9 for pose in transition_rows)


def test_endpoint_inset_increases_with_transition_clearance() -> None:
    base_config = _workspace_config("left", b_y_m=2.0)
    larger_tool_config = _workspace_config(
        "left",
        b_y_m=2.0,
        tool_lateral_offset_m=0.45,
        tool_width_m=0.40,
    )

    base_summary = summarize_side_tool_path(base_config, generate_side_tool_path(base_config))
    larger_tool_summary = summarize_side_tool_path(
        larger_tool_config,
        generate_side_tool_path(larger_tool_config),
    )

    assert larger_tool_summary["endpoint_inset_m"] > base_summary["endpoint_inset_m"]
    assert larger_tool_summary["transition_clearance_radius_m"] > base_summary["transition_clearance_radius_m"]


def test_disabled_endpoint_inset_exposes_external_transition_envelope() -> None:
    config = _workspace_config(
        "left",
        row_count=2,
        auto_inset_endpoints=False,
        fail_on_boundary_violation=False,
        route_order="bottom_to_top",
    )
    path = generate_side_tool_path(config)
    transition_rows = [pose for pose in path if pose["coverage_role"] == "transition"]

    assert transition_rows
    assert any(pose["transition_envelope_within_boundary"] is False for pose in transition_rows)
    assert any("TRANSITION_ENVELOPE_OUTSIDE_WORKSPACE" in pose["boundary_violation_reason"] for pose in transition_rows)


def test_fixed_row_count_too_high_can_fail_when_margins_not_allowed() -> None:
    config = _workspace_config(
        row_count=20,
        allow_uncovered_margins=False,
    )

    with pytest.raises(ValueError, match="ROW_COUNT_TOO_HIGH"):
        generate_side_tool_path(config)


def test_transition_sequence_and_motion_directions_alternate() -> None:
    path = generate_side_tool_path(_workspace_config("left", row_count=2))

    assert [pose["segment_type"] for pose in path] == [
        "lane_start",
        "lane_end",
        "transition_start",
        "rotate_90",
        "reverse_offset",
        "rotate_back",
        "lane_start",
        "lane_end",
    ]
    assert path[2]["transition_style"] == "side_step_reverse_90"
    assert path[3]["segment_type"] == "rotate_90"
    assert path[4]["segment_type"] == "reverse_offset"
    assert all(pose["differential_drive_feasible"] is True for pose in path)
    assert all(pose["motor_command_generated"] is False for pose in path)


def test_auto_internal_is_default_transition_planner_candidate_search() -> None:
    config = _workspace_config("left", transition_style="auto_internal")
    path = generate_side_tool_path(config)
    summary = summarize_side_tool_path(config, path)
    transition_rows = [pose for pose in path if pose["primitive_type"] == "transition"]

    assert summary["transition_planner"] == "auto_internal"
    assert summary["side_step_reverse_90_is_candidate_only"] is True
    assert summary["selected_transition_count"] > 0
    assert summary["transition_candidate_count"] > summary["selected_transition_count"]
    assert transition_rows
    assert {pose["segment_type"] for pose in transition_rows} == {
        "transition_start",
        "rotate_90",
        "reverse_offset",
        "rotate_back",
    }
    assert "side_step_reverse_90" not in {
        pose["selected_transition_primitive"] for pose in transition_rows
    }
    assert {
        pose["selected_transition_primitive"] for pose in transition_rows
    } <= {
        "rotate_drive_rotate_shift",
        "short_internal_fold",
        "heading_flip_inside",
        "reverse_internal_fold",
    }
    assert "same_heading_reverse_shift" not in {
        pose["selected_transition_primitive"] for pose in transition_rows
    }


def test_auto_internal_transition_segments_are_not_cleaning_coverage() -> None:
    path = generate_side_tool_path(_workspace_config("right", transition_style="auto_internal"))
    transition_rows = [pose for pose in path if pose["primitive_type"] == "transition"]

    assert transition_rows
    assert all(pose["coverage_contributes"] is False for pose in transition_rows)
    assert all(pose["coverage_cells_added"] == 0 for pose in transition_rows)
    assert all(pose["coverage_area_added_m2"] == 0.0 for pose in transition_rows)
    assert all(pose["transition_envelope_within_boundary"] is True for pose in transition_rows)
    assert all(pose["tool_swept_within_boundary"] is True for pose in transition_rows)


def test_plot_segments_have_ids_to_prevent_noncontiguous_tool_connections() -> None:
    path = generate_side_tool_path(_workspace_config("left", transition_style="auto_internal"))
    lane_ids = {pose["segment_id"] for pose in path if pose["primitive_type"] == "cleaning_lane"}
    transition_ids = {pose["segment_id"] for pose in path if pose["primitive_type"] == "transition"}

    assert lane_ids
    assert transition_ids
    assert lane_ids.isdisjoint(transition_ids)
    for segment_id in lane_ids:
        rows = [pose for pose in path if pose["segment_id"] == segment_id]
        assert [pose["segment_type"] for pose in rows] == ["lane_start", "lane_end"]
    for segment_id in transition_ids:
        rows = [pose for pose in path if pose["segment_id"] == segment_id]
        assert [pose["segment_type"] for pose in rows] == [
            "transition_start",
            "rotate_90",
            "reverse_offset",
            "rotate_back",
        ]


def test_auto_internal_keeps_coverage_ratio_from_optimizer() -> None:
    config = _workspace_config("left", transition_style="auto_internal")
    path = generate_side_tool_path(config)
    summary = summarize_side_tool_path(config, path)

    assert summary["coverage_ratio"] > 0.8
    assert summary["boundary_violation_count"] == 0
    assert summary["transition_envelope_violation_count"] == 0
    assert summary["plot_artifact_violation_count"] == 0


def test_heading_and_motion_direction_are_distinct() -> None:
    path = generate_side_tool_path(_workspace_config("right"))
    starts = _lane_starts(path)

    assert {float(pose["heading_deg"]) for pose in starts} == {0.0, 180.0}
    assert {pose["motion_direction"] for pose in starts} == {"forward", "reverse"}
    assert {abs(float(pose["travel_direction_deg"])) for pose in starts} == {0.0, 180.0}
    assert {pose["tool_world_side"] for pose in starts} == {"above_chassis", "below_chassis"}


def test_side_tool_path_never_generates_motor_commands() -> None:
    path = generate_side_tool_path(_workspace_config("left"))

    assert all(pose["motor_command_generated"] is False for pose in path)
    assert not any("cmd" in key for pose in path for key in pose)


def test_side_tool_preview_writes_bounded_outputs(tmp_path) -> None:
    result = side_tool_path_preview.main(
        [
            "--advanced",
            "--a-x",
            "0",
            "--a-y",
            "0",
            "--b-x",
            "8",
            "--b-y",
            "1.2",
            "--workspace-mode",
            "diagonal_ab",
            "--tool-side",
            "left",
            "--tool-lateral-offset-m",
            "0.24",
            "--tool-width-m",
            "0.30",
            "--lane-spacing-m",
            "0.25",
            "--row-count",
            "auto",
            "--robot-width-m",
            "0.18",
            "--robot-length-m",
            "0.18",
            "--robot-radius-m",
            "0.14",
            "--first-lane-direction",
            "forward",
            "--transition-style",
            "side_step_reverse_90",
            "--boundary-mode",
            "strict",
            "--boundary-margin-m",
            "0.03",
            "--fail-on-boundary-violation",
            "true",
            "--contamination-mode",
            "off",
            "--out-dir",
            str(tmp_path),
        ]
    )

    assert result == 0
    csv_path = tmp_path / "side_tool_path.csv"
    summary_path = tmp_path / "summary.md"
    png_path = tmp_path / "preview.png"
    assert csv_path.exists()
    assert summary_path.exists()
    assert png_path.exists()

    summary = summary_path.read_text(encoding="utf-8")
    assert "offline preview-only" in summary
    assert "boundary_violation_count: `0`" in summary
    assert "transition_envelope_violation_count: `0`" in summary
    assert "external_turn_bay_allowed: `False`" in summary
    assert "requested_coverage_area_m2" in summary
    assert "covered_area_m2" in summary
    assert "uncovered_area_m2" in summary
    assert "coverage_model: `tool_footprint_only`" in summary
    assert "coverage_grid_model: `tool_footprint_grid_set_cover`" in summary
    assert "requested_coverage_bounds" in summary
    assert "uncovered_margin_low_m" in summary

    with csv_path.open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert rows[0]["segment_type"] == "lane_start"
    assert rows[0]["motor_command_generated"] == "False"
    assert {row["within_boundary"] for row in rows} == {"True"}
    assert {row["chassis_within_boundary"] for row in rows} == {"True"}
    assert {row["tool_within_boundary"] for row in rows} == {"True"}
    assert {row["transition_envelope_within_boundary"] for row in rows} == {"True"}
    assert {row["coverage_contributes"] for row in rows} == {"False", "True"}
    assert {row["motor_command_generated"] for row in rows} == {"False"}


def test_side_tool_preview_writes_auto_internal_outputs(tmp_path) -> None:
    result = side_tool_path_preview.main(
        [
            "--advanced",
            "--workspace-mode",
            "diagonal_ab",
            "--a-x",
            "0",
            "--a-y",
            "0",
            "--b-x",
            "8",
            "--b-y",
            "1.2",
            "--tool-side",
            "left",
            "--coverage-resolution-m",
            "0.05",
            "--transition-style",
            "auto_internal",
            "--fail-on-boundary-violation",
            "true",
            "--out-dir",
            str(tmp_path),
        ]
    )

    assert result == 0
    summary = (tmp_path / "summary.md").read_text(encoding="utf-8")
    assert "transition_planner: `auto_internal`" in summary
    assert "side_step_reverse_90_is_candidate_only: `True`" in summary
    assert "plot_artifact_violation_count: `0`" in summary
    assert "physical_swept_volume_validated: `True`" in summary
    assert "rotation_swept_violation_count: `0`" in summary
    assert "translation_swept_violation_count: `0`" in summary
    assert "combined_swept_violation_count: `0`" in summary
    assert (tmp_path / "geometry_samples.csv").exists()
    assert (tmp_path / "swept_volume_summary.json").exists()
    assert (tmp_path / "preview_overview.png").exists()
    assert (tmp_path / "preview_tool_coverage_only.png").exists()
    assert (tmp_path / "preview_tool_path_primary.png").exists()
    assert (tmp_path / "preview_chassis_derived_from_tool.png").exists()
    assert (tmp_path / "preview_sweep_pattern.png").exists()
    assert (tmp_path / "preview_boustrophedon_pattern.png").exists()
    assert (tmp_path / "preview_chassis_only.png").exists()
    assert (tmp_path / "preview_transitions_only.png").exists()
    assert (tmp_path / "preview_rotation_swept_volume.png").exists()
    assert (tmp_path / "preview_geometry_samples.png").exists()
    with (tmp_path / "side_tool_path.csv").open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert "route_feasible_for_physical_preview: `False`" in summary
    assert "planner_primary_path: `tool_center_path`" in summary
    assert "chassis_path_role: `derived_support_path`" in summary
    assert "tool_path_continuous: `True`" in summary
    assert "chassis_path_derived_from_tool: `True`" in summary
    assert "ROUTE_INFEASIBLE_REACH_B_WITHOUT_CONTAMINATION" in summary
    assert "contamination_mode: `off`" in summary
    assert {row["swept_volume_within_boundary"] for row in rows} == {"True"}
    assert {row["motor_command_generated"] for row in rows} == {"False"}
    with (tmp_path / "geometry_samples.csv").open(encoding="utf-8", newline="") as handle:
        sample_rows = list(csv.DictReader(handle))
    assert sample_rows
    assert {row["motor_command_generated"] for row in sample_rows} == {"False"}


def test_side_tool_preview_writes_primitive_sequence_and_strategy_diagnostics(tmp_path) -> None:
    result = side_tool_path_preview.main(
        [
            "--advanced",
            "--workspace-mode",
            "ab_diagonal_center",
            "--a-x",
            "0",
            "--a-y",
            "0",
            "--b-x",
            "8",
            "--b-y",
            "1.2",
            "--tool-side",
            "right",
            "--transition-style",
            "auto_internal",
            "--coverage-pattern",
            "boustrophedon_required",
            "--forbid-single-lane-success",
            "true",
            "--contamination-mode",
            "strict",
            "--fail-on-contamination-violation",
            "true",
            "--tool-active-during-transitions",
            "false",
            "--tool-active-during-rotation",
            "false",
            "--emit-timeline-frames",
            "false",
            "--emit-segment-frames",
            "false",
            "--emit-contamination-previews",
            "false",
            "--out-dir",
            str(tmp_path),
        ]
    )

    assert result == 0
    assert (tmp_path / "strategy_diagnostics.csv").exists()
    assert (tmp_path / "strategy_diagnostics.md").exists()
    assert (tmp_path / "preview_primitive_sequence.png").exists()
    route_sequence = (tmp_path / "preview_route_sequence.md").read_text(encoding="utf-8")
    summary = (tmp_path / "summary.md").read_text(encoding="utf-8")
    diagnostics = strategy_diagnostics(
        SideToolPlanConfig(
            workspace_mode="ab_diagonal_center",
            a_x_m=0.0,
            a_y_m=0.0,
            b_x_m=8.0,
            b_y_m=1.2,
            tool_side="right",
            transition_style="auto_internal",
            contamination_mode="strict",
            fail_on_contamination_violation=True,
        )
    )

    assert diagnostics
    assert "## Primitive Commands" in route_sequence
    assert "move_forward(" in route_sequence or "move_backward(" in route_sequence
    assert "rotate_left(" in route_sequence or "rotate_right(" in route_sequence
    assert "strategy_diagnostics_csv" in summary
    assert "route_feasible_for_physical_preview: `False`" in summary

    with (tmp_path / "side_tool_path.csv").open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert rows
    assert {row["primitive_type"] for row in rows} <= {
        "move_forward",
        "move_backward",
        "rotate_left",
        "rotate_right",
    }
    assert {row["primitive_sequence_valid"] for row in rows} == {"True"}
    assert {row["motor_command_generated"] for row in rows} == {"False"}


def test_side_tool_preview_writes_tool_serpentine_reset_outputs(tmp_path) -> None:
    result = side_tool_path_preview.main(
        [
            "--a-x",
            "0",
            "--a-y",
            "1.2",
            "--b-x",
            "8",
            "--b-y",
            "0",
            "--step-spacing-m",
            "0.25",
            "--tool-side",
            "left",
            "--tool-lateral-offset-m",
            "0.24",
            "--tool-width-m",
            "0.30",
            "--tool-length-m",
            "0.18",
            "--robot-width-m",
            "0.18",
            "--robot-length-m",
            "0.18",
            "--out-dir",
            str(tmp_path),
        ]
    )

    assert result == 0
    assert (tmp_path / "tool_path.csv").exists()
    assert (tmp_path / "primitive_sequence.csv").exists()
    assert (tmp_path / "summary.md").exists()
    assert (tmp_path / "preview_route_sequence.md").exists()
    assert (tmp_path / "preview_tool_path_primary.png").exists()
    assert (tmp_path / "preview_chassis_derived_from_tool.png").exists()
    assert (tmp_path / "preview_primitive_sequence.png").exists()
    assert (tmp_path / "preview_tool_coverage_only.png").exists()
    assert not (tmp_path / "timeline_frames").exists()
    assert not (tmp_path / "segment_frames").exists()

    summary = (tmp_path / "summary.md").read_text(encoding="utf-8")
    route_sequence = (tmp_path / "preview_route_sequence.md").read_text(encoding="utf-8")
    assert "planner_mode: `simple_tool_serpentine`" in summary
    assert "tool_side: `left`" in summary
    assert "tool_path_starts_at_A: `True`" in summary
    assert "tool_path_ends_at_B: `True`" in summary
    assert "tool_path_continuous: `True`" in summary
    assert "tool_active_on_connectors: `False`" in summary
    assert "connector_painting_disabled: `True`" in summary
    assert "motor_command_generated: `False`" in summary
    assert "contamination_mode" not in summary
    assert "## Tool-space route" in route_sequence
    assert "## Derived chassis route" in route_sequence
    assert "## Primitive sequence" in route_sequence
    assert "tool_active=False" in route_sequence
    assert "P001 rotate_right(90.0 deg)" in route_sequence
    assert "P002 move_forward(0.200 m)" in route_sequence
    assert "P003 rotate_left(90.0 deg)" in route_sequence
    assert "P004 move_backward(8.000 m)" in route_sequence

    with (tmp_path / "tool_path.csv").open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert rows[0]["tool_start_x_m"] == "0.0"
    assert rows[0]["tool_start_y_m"] == "1.2"
    assert rows[-1]["tool_end_x_m"] == "8.0"
    assert rows[-1]["tool_end_y_m"] == "0.0"
    connectors = [row for row in rows if row["tool_segment_type"] == "tool_spacing_connector"]
    tracks = [row for row in rows if row["tool_segment_type"] == "tool_sweep_track"]
    assert {row["tool_active"] for row in connectors} == {"False"}
    assert {row["coverage_contributes"] for row in connectors} == {"False"}
    assert {row["is_ghost_tool_path"] for row in connectors} == {"True"}
    assert {row["cleaned_area_added_m2"] for row in connectors} == {"0.0"}
    assert {row["tool_active"] for row in tracks} == {"True"}
    assert {row["coverage_contributes"] for row in tracks} == {"True"}
    assert {row["motor_command_generated"] for row in rows} == {"False"}

    with (tmp_path / "primitive_sequence.csv").open(encoding="utf-8", newline="") as handle:
        primitives = list(csv.DictReader(handle))
    assert [row["primitive_type"] for row in primitives[:9]] == [
        "move_forward",
        "rotate_right",
        "move_forward",
        "rotate_left",
        "move_backward",
        "rotate_right",
        "move_forward",
        "rotate_left",
        "move_forward",
    ]
    assert {row["tool_active"] for row in primitives if row["segment_role"].startswith("connector")} == {"False"}
    assert {row["motor_command_generated"] for row in primitives} == {"False"}


def test_side_tool_preview_simple_cli_rejects_legacy_flags(tmp_path) -> None:
    with pytest.raises(SystemExit):
        side_tool_path_preview.main(
            [
                "--a-x",
                "0",
                "--a-y",
                "1.2",
                "--b-x",
                "8",
                "--b-y",
                "0",
                "--step-spacing-m",
                "0.25",
                "--workspace-mode",
                "tool_serpentine_ab",
                "--out-dir",
                str(tmp_path),
            ]
        )


def test_preview_side_tool_path_compatibility_wrapper(tmp_path) -> None:
    result = preview_side_tool_path.main(
        [
            "--a-x",
            "0",
            "--a-y",
            "0",
            "--b-x",
            "8",
            "--b-y",
            "1.2",
            "--workspace-mode",
            "diagonal_ab",
            "--tool-side",
            "right",
            "--tool-lateral-offset-m",
            "0.24",
            "--tool-width-m",
            "0.30",
            "--lane-spacing-m",
            "0.25",
            "--row-count",
            "auto",
            "--out-dir",
            str(tmp_path),
        ]
    )

    assert result == 0
    assert (tmp_path / "side_tool_path.csv").exists()
    assert (tmp_path / "summary.md").exists()


def test_side_tool_waypoint_export_writes_no_motion_targets(tmp_path) -> None:
    result = preview_side_tool_waypoints.main(
        [
            "--tool-side",
            "left",
            "--tool-lateral-offset-m",
            "0.4",
            "--tool-width-m",
            "0.3",
            "--lane-spacing-m",
            "2.0",
            "--row-length-m",
            "10.0",
            "--row-count",
            "2",
            "--out-dir",
            str(tmp_path),
        ]
    )

    assert result == 0
    csv_path = tmp_path / "side_tool_waypoints.csv"
    summary_path = tmp_path / "waypoint_summary.md"
    assert csv_path.exists()
    assert summary_path.exists()

    with csv_path.open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert rows[0]["segment_type"] == "lane_start"
    assert rows[0]["target_bearing_deg"] == "0.000"
    assert rows[2]["segment_type"] == "transition_start"
    assert rows[3]["segment_type"] == "rotate_90"
    assert rows[4]["segment_type"] == "reverse_offset"
    assert rows[4]["reverse_direction_expected"] == "False"
    assert {row["motor_command_generated"] for row in rows} == {"False"}
    assert "No rover motor commands are generated" in summary_path.read_text(encoding="utf-8")


def test_side_tool_preview_diagonal_zero_width_cli_fails(tmp_path) -> None:
    with pytest.raises(ValueError, match="DIAGONAL_AB_WIDTH_ZERO"):
        side_tool_path_preview.main(
            [
                "--advanced",
                "--workspace-mode",
                "diagonal_ab",
                "--a-x",
                "0",
                "--a-y",
                "0",
                "--b-x",
                "8",
                "--b-y",
                "0",
                "--out-dir",
                str(tmp_path),
            ]
        )


def test_side_tool_tracking_simulation_produces_virtual_no_motion_outputs(tmp_path) -> None:
    preview_dir = tmp_path / "preview"
    sim_dir = tmp_path / "sim"
    assert side_tool_path_preview.main(
        [
            "--advanced",
            "--a-x",
            "0",
            "--a-y",
            "0",
            "--b-x",
            "8",
            "--b-y",
            "1.2",
            "--workspace-mode",
            "diagonal_ab",
            "--tool-side",
            "left",
            "--row-count",
            "auto",
            "--out-dir",
            str(preview_dir),
        ]
    ) == 0

    assert simulate_side_tool_tracking.main(
        [
            "--path-csv",
            str(preview_dir / "side_tool_path.csv"),
            "--out-dir",
            str(sim_dir),
        ]
    ) == 0

    csv_path = sim_dir / "tracking_errors.csv"
    summary_path = sim_dir / "summary.md"
    assert csv_path.exists()
    assert summary_path.exists()
    with csv_path.open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert rows
    assert {row["motor_command_generated"] for row in rows} == {"False"}
    assert "virtual_desired_forward_cmd" in rows[0]
    assert "No rover motor commands are generated" in summary_path.read_text(encoding="utf-8")
