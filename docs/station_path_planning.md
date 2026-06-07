# Station Path Planning Dry-Run

This workflow generates a station-side coverage mission from two GPS corner
points. It is file generation only. It does not open a serial port, does not
send HC-12 frames, and does not command the rover.

## Inputs

- Point A latitude/longitude: start corner of the coverage rectangle
- Point B latitude/longitude: opposite/end corner of the coverage rectangle
- Lane spacing in meters: sweep interval between lanes
- Optional speed in meters per second

Default planner mode is `corner-rectangle`. Point A and Point B define an
axis-aligned rectangle in the local East/North metric frame. Point A is local
`(0, 0)`, and Point B is the intended final corner. `--sweep-width-m` is not
used in this default mode.

## Command

Exact tested command:

```bash
uv run python scripts/station/plan_coverage_path.py \
  --point-a 35.571070,129.186000 \
  --point-b 35.571250,129.186300 \
  --lane-spacing-m 5.0 \
  --speed-mps 0.4 \
  --mission-name codex_corner_rectangle_smoke
```

Default output directory:

```text
outputs/missions/
```

Generated files:

```text
outputs/missions/codex_corner_rectangle_smoke/mission.json
outputs/missions/codex_corner_rectangle_smoke/mission.csv
outputs/missions/codex_corner_rectangle_smoke/preview.png
```

Observed dry-run result:

- `mission.json`, `mission.csv`, and `preview.png` were generated.
- `lane_count=6`.
- `waypoint_count=13`.
- The preview showed corner-to-corner lawnmower/boustrophedon lanes.
- Path preview is complete for the current station-side milestone.
- No rover firmware was modified.
- No commands were sent to the rover.

## Geometry

The default tool converts latitude/longitude into a local East/North meter
frame using Point A as the origin. It then builds an axis-aligned local
rectangle from Point A and Point B:

- local Point A is the start corner;
- local Point B is the opposite/end corner;
- lanes run parallel to the local east/west extent;
- lane offsets advance along the local north/south extent;
- nominal offsets advance as `0, lane_spacing, 2*lane_spacing, ...` while
  staying inside the rectangle;
- when the alternating lane order would not naturally end at Point B, the
  planner adds a final connector waypoint with a `notes` field.

The generated local waypoints are converted back to latitude/longitude before
being written to JSON and CSV.

## Edge And Remainder Policy

The planner is boundary-safe. It must not add an extra lane outside the
rectangle just to remove a small remaining margin at the far edge.

When the rectangle extent is not exactly divisible by `lane_spacing_m`, a small
remaining strip at the boundary is acceptable for this preview dry-run stage.
Generated coverage lanes must stay inside or on the rectangle boundary. A final
Point B connector may be added to make the mission endpoint explicit, but that
connector is not a command to drive outside the boundary and is not real rover
motion.

The previous baseline plus width behavior is retained only as an explicit
legacy mode:

```bash
uv run python scripts/station/plan_coverage_path.py \
  --planner-mode baseline-width \
  --point-a 35.571070,129.186000 \
  --point-b 35.571070,129.186300 \
  --sweep-width-m 20.0 \
  --lane-spacing-m 5.0 \
  --mission-name codex_baseline_width_legacy
```

## Mission Schema

Generated missions use schema `station_coverage_path.v1`. The file is a
station-side dry-run artifact, not a rover command file. It can be inspected by
future onboard dry-run tooling, but it must not be streamed to the rover as live
motor commands.

Top-level fields:

- `schema`: mission file format identifier.
- `metadata`: mission name, generation time, dry-run flag, rover-command flag,
  and planner mode.
- `inputs`: original GPS points and planning parameters.
- `local_origin`: latitude/longitude used as the local metric-frame origin.
- `input_points_local`: Point A and Point B positions in local meters plus
  their roles.
- `local_frame`: axis naming and planner-mode rule.
- `summary`: lane count, waypoint count, lane length, and rectangle extents.
- `coverage_boundary`: rectangle vertices in local meters.
- `safety`: explicit dry-run constraints.
- `waypoints`: ordered mission preview waypoints.

## Mission JSON Fields

The JSON file contains:

- metadata;
- input points and planning parameters;
- `dry_run=true`;
- `sends_rover_commands=false`;
- local origin;
- local Point A and Point B roles;
- local frame description;
- coverage boundary;
- lane/waypoint summary, including rectangle extents in default
  `corner-rectangle` mode;
- explicit safety notes;
- generated waypoints with index, lat/lon, local `x_m` / `y_m`, lane index,
  `segment_type`, `notes`, and optional `speed_mps`.

Waypoint fields:

- `index`: zero-based waypoint order in the preview path.
- `lat` / `lon`: waypoint converted back to GPS coordinates.
- `x_m` / `y_m`: local East/North position in meters from Point A.
- `segment_type`: `lane_start`, `lane_end`, or `final_connector`.
- `lane`: zero-based lane index.
- `offset_m`: lane offset from the Point A side.
- `speed_mps`: optional speed metadata only; it is not sent to the rover.
- `notes`: human-readable explanation, such as a final connector to Point B.

## Mission CSV

The CSV file is the same waypoint list in table form. It is intended for quick
inspection, spreadsheet review, and later dry-run import tests. It includes:

- `index`
- `lat`
- `lon`
- `x_m`
- `y_m`
- `segment_type`
- `notes`

The extra columns `lane`, `offset_m`, and `speed_mps` are included for dry-run
inspection. CSV rows are ordered exactly as the preview path order.

## Preview PNG

The preview image shows:

- Point A as the start corner;
- Point B as the intended final/end corner;
- generated lanes;
- waypoint order labels;
- lane spacing and final residual strip note when present;
- the coverage boundary rectangle.

Interpretation rules:

- The dashed rectangle is the intended local coverage boundary.
- Numbered markers show waypoint order.
- The path is a preview of a future mission shape, not an executed route.
- Any final residual note means the planner stayed boundary-safe instead of
  adding an out-of-bounds lane.
- A `final_connector` marker means the preview ends exactly at Point B.

## Safety Rules

- This is station-side path generation only.
- Do not connect this tool to HC-12.
- Do not send the generated waypoints to the rover yet.
- This mission output is not yet executed by the rover.
- The generated missions are not yet executed by the rover.
- The next rover-side autonomy step is single-waypoint controlled motion
  preparation, not full coverage/lawnmower driving.
- Keep the fixed-wiring RC + GPS firmware in dry-run mode for readiness checks
  only.
- Real waypoint following requires a separate safety design, heading source,
  GPS validity policy, STOP behavior, and wheel-off-ground validation.

## Side-Mounted Cleaning-Tool Preview

For the side-mounted cleaning-tool rover, use the separate offline preview tool:

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

The compatibility alias below is also supported and delegates to the same tool:

```bash
uv run python tools/preview_side_tool_path.py \
  --workspace-mode ab_diagonal_center \
  --a-x 0 --a-y 0 \
  --b-x 8 --b-y 1.2 \
  --tool-side left \
  --tool-lateral-offset-m 0.24 \
  --tool-width-m 0.30 \
  --lane-spacing-m 0.25 \
  --row-count auto \
  --transition-style auto_internal \
  --out-dir outputs/side_tool_path_preview/ab_diagonal_temporal_left
```

This planner is for preview only. It is tool-coverage-first: the cleaning-tool
footprint is the coverage path, and the chassis centerline is only the derived
support path. Chassis-only travel does not count as cleaning. It outputs:

- chassis centerline poses;
- cleaning-tool footprint center and edge poses;
- lane index;
- chassis heading and motion direction as separate fields;
- lane travel direction (`forward` / `reverse` relative to chassis heading);
- internal transition-only segments with `selected_transition_primitive` recorded
  for each transition.

The side-mounted planner supports `--tool-side left` and `--tool-side right`.
The default CLI treats A as the top-left tool-center start and B as the
bottom-right tool-center end. The tool path is generated first from
`--step-spacing-m`; the chassis path is derived afterward. Legacy diagnostic
modes such as `ab_diagonal_center`, `ab_centerline_width`, and `diagonal_ab` are
available only through `--advanced` or the compatibility wrapper. Every emitted
primitive row must be one of `move_forward`, `move_backward`, `rotate_left`, or
`rotate_right`.
It does not generate motor commands, does not open serial ports, does not send
HC-12 frames, and does not upload firmware.

For A/B tool-centered planning, use
[docs/side_tool_path_planning.md](side_tool_path_planning.md). The reset planner
draws the fixed rectangle from the top-left A and bottom-right B tool-center
points, generates a continuous serpentine tool route from `--step-spacing-m`,
derives the chassis support path afterward, and writes
`preview_tool_path_primary.png`, `preview_chassis_derived_from_tool.png`,
`preview_primitive_sequence.png`, and `preview_tool_coverage_only.png`. Reset
readiness requires
`tool_path_starts_at_A=true`, `tool_path_ends_at_B=true`,
`tool_path_continuous=true`, `tool_connector_count = tool_track_count - 1`,
`primitive_sequence_valid=true`, and `motor_command_generated=false`.
Contamination and transition-envelope checks are optional legacy diagnostics,
not the default planner gate.
Use `--workspace-mode axis_width` only for the older A->B axis plus width
interpretation.

To export preview poses as offline target waypoint diagnostics:

```bash
uv run python tools/preview_side_tool_waypoints.py \
  --tool-side left \
  --tool-lateral-offset-m 0.35 \
  --tool-width-m 0.30 \
  --lane-spacing-m 0.30 \
  --row-length-m 8.0 \
  --row-count 4 \
  --start-heading-deg 0 \
  --first-lane-direction forward \
  --out-dir outputs/side_tool_waypoint_preview/left_tool_example
```

The waypoint export writes `side_tool_waypoints.csv` and
`waypoint_summary.md`. It includes target bearings, segment labels, expected
rover heading, reverse-direction flags, and `motor_command_generated=False` for
every row. It is not a firmware input and must not be streamed to the rover.
