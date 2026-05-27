# Station Path Planning Dry-Run

This workflow generates a station-side coverage mission from two GPS points and
a requested sweep width. It is file generation only. It does not open a serial
port, does not send HC-12 frames, and does not command the rover.

## Inputs

- Point A latitude/longitude
- Point B latitude/longitude
- Sweep width in meters
- Lane spacing in meters
- Optional speed in meters per second

Point A and Point B define the baseline of the coverage region. The sweep width
extends to the left side of the A-to-B baseline in the local metric frame.

## Command

Exact tested command:

```bash
uv run python scripts/station/plan_coverage_path.py \
  --point-a 35.571070,129.186000 \
  --point-b 35.571070,129.186300 \
  --sweep-width-m 20.0 \
  --lane-spacing-m 5.0 \
  --speed-mps 0.4 \
  --mission-name codex_station_path_smoke
```

Default output directory:

```text
outputs/missions/
```

Generated files:

```text
outputs/missions/codex_station_path_smoke/mission.json
outputs/missions/codex_station_path_smoke/mission.csv
outputs/missions/codex_station_path_smoke/preview.png
```

Observed dry-run result:

- `mission.json`, `mission.csv`, and `preview.png` were generated.
- The preview showed lawnmower/boustrophedon lanes.
- No rover firmware was modified.
- No commands were sent to the rover.

## Geometry

The tool converts latitude/longitude into a local East/North meter frame using
Point A as the origin. It then generates a boustrophedon path:

- even-numbered lanes run from the A side toward the B side;
- odd-numbered lanes run back from the B side toward the A side;
- lane offsets include both the near edge and far edge of the requested sweep
  width;
- if the sweep width is not divisible by the lane spacing, the final lane is
  placed on the far sweep edge.

The generated local waypoints are converted back to latitude/longitude before
being written to JSON and CSV.

## Mission JSON

The JSON file contains:

- metadata;
- input points and planning parameters;
- `dry_run=true`;
- `sends_rover_commands=false`;
- local origin;
- local frame description;
- coverage boundary;
- lane/waypoint summary;
- explicit safety notes;
- generated waypoints with lat/lon, local `x_m` / `y_m`, lane index, order, and
  optional `speed_mps`.

## Mission CSV

The CSV file includes at least:

- `index`
- `lat`
- `lon`
- `x_m`
- `y_m`
- `segment_type`

The extra columns `lane`, `offset_m`, and `speed_mps` are included for dry-run
inspection.

## Preview PNG

The preview image shows:

- point A;
- point B;
- generated lanes;
- waypoint order labels;
- the sweep boundary rectangle.

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
