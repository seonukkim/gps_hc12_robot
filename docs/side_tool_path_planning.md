# Tool-Centered Serpentine Side-Tool Path Planning Preview

This workflow is offline preview only. It does not upload firmware, open serial
ports, send HC-12 frames, or generate rover motor commands.

## Real Field A/B Semantics

For the current reset workflow, A and B are tool/paint-tank center points and
opposite rectangle corners:

- A is the top-left tool-center start point.
- B is the bottom-right tool-center end point.
- The tool/paint-tank center path is the primary planned route.
- The rover chassis path is derived afterward as support geometry.
- In simple local mode, `A=(0,1.2)` and `B=(8,0)` define a clean surface
  `x=[0,8]`, `y=[0,1.2]`.

The default planner mode is `tool_serpentine_ab`. `ab_diagonal_center`,
`ab_centerline_width`, `diagonal_ab`, and `axis_width` remain available only as
alternate or legacy diagnostic modes; they are not the current default preview
model.

The clean surface and chassis support area are separate in real mode:

- `clean_surface`: the tool coverage target.
- `chassis_boundary_mode=derived_diagnostic`: default; chassis geometry is
  derived from the primary tool route and reported as support diagnostics.
- `chassis_boundary_mode=clean_surface_strict`: optional strict diagnostic where
  the chassis footprint must stay inside the clean surface; this can make exact
  A/B infeasible.

Invalid diagonal example:

```bash
uv run python tools/side_tool_path_preview.py \
  --advanced \
  --workspace-mode diagonal_ab \
  --a-x 0 --a-y 0 \
  --b-x 8 --b-y 0
```

This fails because diagonal mode requires opposite corners with nonzero width.
Use the simple command with `A=(0,1.2)` and `B=(8,0)` for the reset
tool-centered workflow.

## Rover And Tool Assumptions

The rover body is modeled as an 18 cm x 18 cm chassis:

- `robot_width_m=0.18`
- `robot_length_m=0.18`
- `robot_radius_m=0.14` for conservative clearance and rotation checks

The cleaning tool is hypothetical and configurable. Current defaults:

- `tool_width_m=0.30`
- `tool_length_m=0.18`
- `tool_lateral_offset_m=0.24`
- `lane_spacing_m=0.25`
- `boundary_margin_m=0.03`

The `0.24` m tool offset comes from 0.09 m robot half-width plus 0.15 m tool
half-width. The `0.25` m lane spacing gives overlap for a 0.30 m tool.
`tool_width_m` is the lateral cleaning width. `tool_length_m` is the fore-aft
physical length used for swept-volume collision and boundary validation.

## Coverage Model

The planner is coverage-first, not chassis-centerline-first:

1. construct the clean surface from the A/B diagonal corners;
2. generate the tool/paint-tank center path first;
3. connect tool sweep-track endpoints with tool zigzag connector waypoints;
4. derive chassis centerline poses from the tool path and `tool_side`;
5. compute cleaned coverage from swept tool footprint only;
6. validate chassis footprint, tool center, tool edges, and sampled physical
   swept volume;
7. keep A/B rover-center start/end exact, with the tool inactive during
   pre-clean alignment and transitions by default;
8. report unreachable edge strips as uncovered margins instead of protruding
   outside the rectangle.

The primary route is the tool/paint-tank centerline. The chassis path is a
derived support path computed by inverse geometry:

```text
tool_center = chassis_center + side_sign * tool_lateral_offset_m * normal(chassis_heading)
chassis_center = tool_center - side_sign * tool_lateral_offset_m * normal(chassis_heading)
```

The chassis footprint is a boundary/collision constraint and does not count as
cleaned area. A strip under the rover body is still uncovered unless the tool
footprint sweeps it.

## Primitive Boustrophedon Route

`ㄹ-shaped` means repeated physical boustrophedon coverage, not a symbolic
drawing. Every lane change must be decomposed into the four movement primitives
the differential-drive rover can execute in preview:

- `move_forward(distance_m)`
- `move_backward(distance_m)`
- `rotate_left(angle_deg)`
- `rotate_right(angle_deg)`

Sideways movement, same-heading lateral shifts, holonomic strafes, diagonal
translations without first rotating, and teleports are invalid. A transition
from one lane to the next is represented as rotate, straight move, rotate, then
the next cleaning lane. The preview writes this sequence to
`preview_route_sequence.md` and `tool_path.csv` with
`primitive_sequence_valid`.

`preview_tool_path_primary.png` is the main route view: it shows the continuous
tool-center polyline through sweep tracks and track-to-track connectors.
`preview_chassis_derived_from_tool.png` shows the secondary chassis support
path and the lateral offset that places the tool on that primary route.

Reported coverage fields:

- `requested_coverage_area_m2`
- `covered_area_m2`
- `uncovered_area_m2`
- `requested_coverage_y_min_m` / `requested_coverage_y_max_m`
- `actual_tool_coverage_y_min_m` / `actual_tool_coverage_y_max_m`
- `tool_swept_x_min_m` / `tool_swept_x_max_m`
- `tool_swept_y_min_m` / `tool_swept_y_max_m`
- `uncovered_margin_low_m`
- `uncovered_margin_high_m`
- `uncovered_strips_count`
- `coverage_ratio`
- `boundary_violation_count`

The current evaluator is grid-based. It discretizes the workspace using
`--coverage-resolution-m`, marks cells as covered only when the swept cleaning
tool footprint intersects them, and greedily selects candidate tool strips that
add the most new covered cells. Chassis footprint cells are ignored for coverage
and used only for boundary feasibility. The report includes overlap area and
coverage efficiency so redundant tool passes are visible.

Some uncovered margin is acceptable when the robot/tool footprint cannot reach
an edge without leaving the rectangle. `boundary_violation_count` must be `0`
before any later no-motion route validation.

## Swept-Volume Feasibility

Coverage and physical feasibility are separate:

- `coverage_model=tool_footprint_only`
- `feasibility_model=chassis_plus_tool_swept_volume`
- `chassis_area_counts_as_cleaned=false`

Strict swept-volume validation samples the rover body and side-mounted tool as a
combined rigid footprint during lane translations, internal transitions, and
heading changes. It validates:

- the 18 cm x 18 cm chassis rectangle;
- the side-mounted tool rectangle;
- combined chassis+tool bounding volume;
- translation samples at `--translation-sample-m` (default `0.05`);
- rotation samples at `--rotation-sample-deg` (default `5`);
- every sampled polygon vertex against the A/B rectangle and boundary margin.

Simple transition envelopes are useful for early diagnostics, but they are not
enough for final offline preview. Use `--swept-volume-validation strict` before
outdoor physical path planning preview or no-motion target validation.

Swept-volume summary fields include:

- `physical_swept_volume_validated`
- `rotation_sample_deg`
- `translation_sample_m`
- `rotation_swept_violation_count`
- `translation_swept_violation_count`
- `combined_swept_violation_count`
- `total_geometry_sample_count`

## Heading And Motion Direction

`tool_side` is relative to chassis heading. Chassis heading and travel direction
are separate:

- `heading_deg`: chassis facing direction.
- `motion_direction`: forward or reverse relative to chassis heading.
- `travel_direction_deg`: actual movement direction.

With `--auto-orient-tool-inside true`, the planner may flip chassis heading so
the side-mounted tool points into the rectangle. A right-mounted tool can face
B->A and still travel an A->B lane in reverse.

## Differential-Drive Feasibility

The rover is modeled as a differential-drive vehicle. It can rotate in place,
drive straight forward, drive straight backward, and use rotate-drive-rotate
compound maneuvers. It cannot move sideways while keeping the same heading.

Every generated segment is checked with `kinematic_model=differential_drive`.
Invalid primitives include:

- same-heading lateral shifts;
- sideways steps;
- teleporting between lanes;
- travel directions not aligned with `heading_deg` or `heading_deg + 180`.

Any lateral lane change must be represented as a rotation, a straight
forward/reverse move, and another rotation. CSV rows include
`differential_drive_feasible`, `kinematic_violation_reason`,
`segment_distance_m`, and `segment_rotation_deg`. Strict previews require
`differential_drive_feasible=True`.

## Start/End Inset

Exact A or B chassis-center poses may be infeasible because the rover/tool
footprint must stay inside the rectangle. With nearest-feasible policies, lane
endpoints are shortened inward and the summary reports:

- `start_inset_m`
- `end_inset_m`
- `start_offset_from_A_m`
- `end_offset_from_B_m`
- `endpoint_inset_m`
- `start_reaches_A`
- `route_reaches_B`
- `final_distance_to_B_m`

Do not treat A/B as physically reached when these offsets are nonzero. If
`route_reaches_B=False` or `end_offset_from_B_m` exceeds `--max-end-offset-m`,
the route is not ready for physical preview unless best-effort output was
explicitly requested for diagnosis.

## Transition Style

The default transition planner is `auto_internal`. It treats
`side_step_reverse_90` as one candidate primitive, not as the required maneuver.
For each selected tool lane pair, the planner evaluates internal transition
candidates that can be expressed as differential-drive primitives, such as:

- `rotate_drive_rotate_shift`
- `short_internal_fold`
- `reverse_internal_fold`
- `heading_flip_inside`
- `side_step_reverse_90`

The selected primitive is recorded as `selected_transition_primitive`. The
planner prefers internal transitions that stay inside the A/B rectangle, reduce
non-cleaning travel, avoid endpoint zig-zags, and avoid connecting unrelated
tool-lane points.

`same_heading_lateral_shift`, `same_heading_reverse_shift`, or any similar
sideways move is not valid as a physical primitive. If a preview label ever
describes that behavior, it must be decomposed into
`rotate_in_place -> drive_straight_forward/reverse -> rotate_in_place`, or the
route must fail with a kinematic violation.

Transitions are checked against the A/B rectangle with an inward transition
envelope. External turn bays are not allowed. Near the `x_min` end, the
transition pocket must remain inside `x >= x_min + clearance`; near the `x_max`
end, it must remain inside `x <= x_max - clearance`. If the pocket would leave
the rectangle, the planner leaves coverage unresolved or fails in strict mode
instead of drawing outside geometry.

The preview renderer plots every `segment_id` separately. Cleaning lanes,
transition-only paths, and chassis support paths are never joined by an
artificial continuous polyline. That prevents plot artifacts that look like
extra orange/yellow endpoint trajectories.

Strict mode reports or fails with reasons such as
`TRANSITION_POINT_OUTSIDE_WORKSPACE`, `TRANSITION_ENVELOPE_OUTSIDE_WORKSPACE`,
`EXTERNAL_TURN_BAY_NOT_ALLOWED`, `INSUFFICIENT_INTERNAL_TURN_CLEARANCE`,
`NO_FEASIBLE_INWARD_TRANSITION`, `INSUFFICIENT_ENDPOINT_CLEARANCE`, or
`INSUFFICIENT_WORKSPACE_WIDTH`.

## Optional Legacy Contamination Diagnostics

The reset `tool_serpentine_ab` preview defaults to `--contamination-mode off`.
It first verifies that the user-facing tool/paint-tank center path is correct:
continuous serpentine tracks and connectors from A to B. Older wet/paint
diagnostics remain available when explicitly enabled, but they are no longer
part of the default route generator or readiness gate.

When contamination diagnostics are enabled, painting and wet cleaning add a
time-ordered constraint. Once the tool sweeps a cell, that cell is treated as
wet/painted/cleaned:

- tool-over-prior-tool-swept area is allowed;
- chassis-over-never-cleaned area is allowed;
- chassis-over-area-that-will-be-cleaned-later is allowed;
- chassis-over-prior-tool-swept area is forbidden;
- chassis-over-currently-tool-swept area is treated conservatively by sampling
  the time sequence.

This is separate from spatial swept-volume validation. A route can keep every
polygon inside the A/B rectangle and still fail the temporal wet-area rule.

The preview uses a grid with `--coverage-resolution-m` and records when each
cell is first swept by the tool. At every sampled time step, it checks the
chassis footprint before marking the current tool footprint as cleaned. If the
chassis footprint intersects cells already swept by the tool, the route records
`CHASSIS_ON_PRIOR_TOOL_SWEPT_AREA`.

Important CLI controls:

- `--kinematic-model differential_drive`
- `--contamination-mode strict|warn|off`
- `--fail-on-contamination-violation true|false`
- `--tool-active-during-transitions true|false`
- `--same-step-tool-before-chassis true|false`
- `--route-order auto_temporal_safe|bottom_to_top|top_to_bottom|nearest_to_exit|farthest_to_exit|A_to_B_frontier|B_to_A_retreat|wet_frontier_retreat|fixed_serpentine`
- `--max-end-offset-m`
- `--allow-unreached-B-best-effort true|false`
- `--allow-contamination-best-effort true|false`
- `--emit-timeline-frames true|false`
- `--emit-segment-frames true|false`
- `--timeline-frame-stride N`
- `--max-timeline-frames N`
- `--emit-contamination-previews true|false`

The reset preview is considered ready when:

- `workspace_mode=tool_serpentine_ab`
- `planner_primary_path=tool_center_path`
- `tool_path_starts_at_A=True`
- `tool_path_ends_at_B=True`
- `tool_path_continuous=True`
- `tool_connector_count = tool_track_count - 1`
- `chassis_path_derived_from_tool=True`
- `primitive_sequence_valid=True`
- every primitive is one of `move_forward`, `move_backward`, `rotate_left`, or
  `rotate_right`
- `contamination_mode=off`
- `motor_command_generated=False`

For painting/wet-cleaning diagnostics, `--tool-active-during-transitions false`
is the default because a lifted or inactive tool during transitions avoids
painting non-cleaning maneuver paths. If the tool cannot be lifted, set it true
and inspect the stricter wet-area result.

If wet-area behavior is being studied later, run `--contamination-mode warn` or
`strict` and inspect `contamination_events.csv`. Those diagnostics should not be
used to reshape the default tool path unless the planner is explicitly switched
out of reset mode.

## CLI Examples

Current real workflow smoke command:

```bash
uv run python tools/side_tool_path_preview.py \
  --a-x 0 --a-y 1.2 \
  --b-x 8 --b-y 0 \
  --step-spacing-m 0.25 \
  --tool-side left \
  --tool-lateral-offset-m 0.24 \
  --tool-width-m 0.30 \
  --tool-length-m 0.18 \
  --robot-width-m 0.18 \
  --robot-length-m 0.18 \
  --out-dir outputs/side_tool_path_preview/simple_serpentine
```

This reset command writes the primary tool path first. It should report a
continuous route from A to B with `planner_mode=simple_tool_serpentine`,
`tool_path_continuous=true`, `connector_painting_disabled=true`, and
`motor_command_generated=false`.

Legacy centerline-width example:

```bash
uv run python tools/side_tool_path_preview.py \
  --advanced \
  --workspace-mode ab_centerline_width \
  --a-x 0 --a-y 0 \
  --b-x 8 --b-y 0 \
  --surface-side left \
  --workspace-width-m 1.2 \
  --tool-side right \
  --tool-lateral-offset-m 0.24 \
  --tool-width-m 0.30 \
  --tool-length-m 0.18 \
  --lane-spacing-m 0.25 \
  --row-count auto \
  --robot-width-m 0.18 \
  --robot-length-m 0.18 \
  --robot-radius-m 0.14 \
  --boundary-margin-m 0.03 \
  --coverage-resolution-m 0.05 \
  --rotation-sample-deg 5 \
  --translation-sample-m 0.05 \
  --swept-volume-validation strict \
  --kinematic-model differential_drive \
  --contamination-mode strict \
  --fail-on-contamination-violation true \
  --tool-active-during-transitions false \
  --same-step-tool-before-chassis false \
  --route-order auto_temporal_safe \
  --require-start-at-A true \
  --require-end-at-B true \
  --max-start-error-m 0.05 \
  --max-end-error-m 0.05 \
  --allow-best-effort false \
  --emit-geometry-samples true \
  --emit-separated-previews true \
  --emit-timeline-frames true \
  --emit-segment-frames true \
  --timeline-frame-stride 5 \
  --emit-contamination-previews true \
  --auto-orient-tool-inside true \
  --transition-style auto_internal \
  --fail-on-boundary-violation true \
  --out-dir outputs/side_tool_path_preview/ab_centerline_temporal_right
```

Legacy alternate mode:

```bash
uv run python tools/side_tool_path_preview.py \
  --advanced \
  --workspace-mode axis_width \
  --a-x 0 --a-y 0 \
  --b-x 8 --b-y 0 \
  --workspace-side left \
  --workspace-width-m 1.2 \
  --tool-side left
```

## Output Files

The simple preview writes:

- `summary.md`
- `tool_path.csv`
- `primitive_sequence.csv`
- `preview_tool_path_primary.png`
- `preview_tool_coverage_only.png`
- `preview_chassis_derived_from_tool.png`
- `preview_primitive_sequence.png`
- `preview_route_sequence.md`

Advanced diagnostic mode can additionally write geometry, swept-volume,
contamination, strategy, timeline, and segment-frame files.

CSV rows include:

- `workspace_mode`
- `segment_index`
- `segment_id`
- `segment_type`
- `primitive_type`
- `primitive_subtype`
- `selected_transition_primitive`
- `lane_index`
- `x_m`, `y_m`
- `heading_deg`
- `motion_direction`
- `travel_direction_deg`
- `kinematic_model`
- `differential_drive_feasible`
- `kinematic_violation_reason`
- `heading_delta_deg`
- `segment_distance_m`
- `segment_rotation_deg`
- `tool_x_m`, `tool_y_m`
- `tool_edge_min_y_m`, `tool_edge_max_y_m`
- chassis/tool/combined bbox fields
- `chassis_within_boundary`
- `tool_within_boundary`
- `tool_swept_within_boundary`
- `rotation_sample_count`
- `translation_sample_count`
- `rotation_swept_combined_within_boundary`
- `translation_swept_combined_within_boundary`
- `swept_volume_within_boundary`
- `swept_volume_violation_reason`
- `transition_envelope_x_min_m`, `transition_envelope_x_max_m`
- `transition_envelope_y_min_m`, `transition_envelope_y_max_m`
- `transition_envelope_within_boundary`
- `transition_pocket_side`
- `transition_violation_reason`
- `within_boundary`
- `boundary_violation_reason`
- `coverage_role`
- `coverage_contributes`
- `coverage_cells_added`
- `coverage_area_added_m2`
- `time_start_index`
- `time_end_index`
- `contamination_checked`
- `contamination_free_segment`
- `contamination_violation_count`
- `contamination_violation_area_m2`
- `cleaned_area_added_m2`
- `tool_reclean_area_m2`
- `chassis_on_prior_cleaned_area_m2`
- `tool_active`
- `start_reaches_A`
- `end_reaches_B`
- `start_error_m`
- `end_error_m`
- `route_reaches_B`
- `motor_command_generated=False`

Reset summary fields include:

- `workspace_mode=tool_serpentine_ab`
- `A_corner_role=top_left`
- `B_corner_role=bottom_right`
- `planner_primary_path=tool_center_path`
- `chassis_path_role=derived_support_path`
- `tool_side`
- `requested_step_spacing_m`
- `actual_spacing_values_m`
- `derived_track_count`
- `track_count_parity`
- `final_partial_spacing_m`
- `force_end_at_B`
- `adjust_spacing_to_end_at_B`
- `tool_path_continuous`
- `tool_track_count`
- `tool_connector_count`
- `tool_connector_count_equals_track_count_minus_one`
- `tool_path_starts_at_A`
- `tool_path_ends_at_B`
- `chassis_path_derived_from_tool`
- `primitive_sequence_valid`
- `chassis_feasible_for_tool_path`
- `chassis_infeasible_reason`
- `coverage_ratio`
- `uncovered_area_m2`
- `contamination_mode=off`
- `path_preview_only=True`
- `motor_command_generated=False`

`preview_tool_path_primary.png` is the primary visual. It labels A as the
top-left tool-center start and B as the bottom-right tool-center end, then draws
the continuous serpentine tool path through every track and connector. Chassis
visualization is separated into `preview_chassis_derived_from_tool.png` so it
cannot be mistaken for the planned route.

The reset separated preview images isolate the major views:

- `preview_tool_path_primary.png`: prominent continuous tool-center path.
- `preview_chassis_derived_from_tool.png`: secondary chassis support path.
- `preview_primitive_sequence.png`: rotate/move primitive sequence.
- `preview_tool_coverage_only.png`: swept tool coverage and uncovered area.
- `preview_route_sequence.md`: separate tool-space route, derived chassis route,
  and primitive movement sequence.

Transition-envelope boxes are hidden by default. If
`--show-transition-envelope true` is used for diagnostics, the envelope is drawn
only in diagnostic views and never replaces the actual tool-center path.

For the reset preview, readiness means the tool path is continuous from A to B,
the chassis path is derived from that tool path, the primitive sequence contains
only the four allowed preview primitives, and every row reports
`motor_command_generated=False`. Contamination and wet-area overlays are
separate optional diagnostics and are not enabled by default.

## Staged Progression

- Stage 0: offline diagonal A/B bounded path preview.
- Stage 1: offline tracking simulation from the preview CSV.
- Stage 2: integrated no-motion target logging.
- Stage 3: actuator-free or wheels-off checks only if a future safety review
  requires them.
- Stage 4: guarded crawl discussion only after separate explicit approval.

Hand-carry route validation is not required in the immediate checklist when the
offline geometry and tracking simulation pass. It can still be used later as an
extra sensor-validation tool, but it is not a substitute for motor safety gates.

Physical motor enable remains prohibited in this workflow.
