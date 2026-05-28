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
- offsets are `0, lane_spacing, 2*lane_spacing, ..., final_extent`;
- the final rectangle boundary is always included, even when the residual strip
  is smaller than `lane_spacing_m`;
- when the alternating lane order would not naturally end at Point B, the
  planner adds a final connector waypoint with a `notes` field.

The generated local waypoints are converted back to latitude/longitude before
being written to JSON and CSV.

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

## Mission JSON

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

## Mission CSV

The CSV file includes at least:

- `index`
- `lat`
- `lon`
- `x_m`
- `y_m`
- `segment_type`
- `notes`

The extra columns `lane`, `offset_m`, and `speed_mps` are included for dry-run
inspection.

## Preview PNG

The preview image shows:

- Point A as the start corner;
- Point B as the intended final/end corner;
- generated lanes;
- waypoint order labels;
- lane spacing and final residual strip note when present;
- the coverage boundary rectangle.

## Safety Rules

- This is station-side path generation only.
- Do not connect this tool to HC-12.
- Do not send the generated waypoints to the rover yet.
- This mission output is not yet executed by the rover.
- Next step is onboard mission dry-run, not real motion.
- Keep the fixed-wiring RC + GPS firmware in dry-run mode for readiness checks
  only.
- Real waypoint following requires a separate safety design, heading source,
  GPS validity policy, STOP behavior, and wheel-off-ground validation.
