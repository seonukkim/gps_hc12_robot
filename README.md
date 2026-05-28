# Ship Hull Coverage Robot

Pre-ROS2 Python prototype for an Industrial Engineering project: a ship exterior
cleaning and painting robot intended to operate on the outer hull of a ship. The
current planning assumption is a locally planar surface. The adhesion concept is
magnetic wheels, but magnetic adhesion and full cleaning/painting operation have
not yet been validated.

The repository currently focuses on safe rover bring-up, GPS/log handling,
protocol utilities, and mock planar coverage planning. ROS2 Jazzy integration is
planned, but the current ROS2 packages are skeletons only.

## Safety Defaults

Station-side defaults are intentionally conservative:

- Default USB serial device: `/dev/ttyACM0`.
- Default baudrate: `9600`.
- Serial tools expose `--port` so the device can be changed explicitly.
- Station loops default to heartbeat and `STOP` only.
- Station startup must not send live motor-driving `AUTO` commands.
- Rover motor testing is wheel-off-ground only.
- RC manual override and rover-side failsafe logic remain authoritative.

## Industrial Engineering Motivation

Ship hull exterior work is labor-intensive, repetitive, and difficult to keep
consistent over large surfaces. This project frames that problem as an
Industrial Engineering workflow:

- reduce manual burden in hull cleaning and painting tasks;
- define repeatable coverage regions from operator-selected points;
- generate systematic lawnmower-style paths for surface coverage;
- automate the workflow from manual setup to planned execution;
- validate safety behavior before expanding toward autonomous field operation.

## System Concept

The intended operation concept is:

1. Drive the rover manually on or near the target hull surface.
2. Record operator-selected A/B reference points.
3. Define a rectangular planar work region from those references.
4. Generate a lawnmower-style coverage path with configurable lane spacing.
5. Preview or export the mock mission offline.
6. Later, send mission commands through the station, HC-12 radio, and ROS2 stack.

Current code supports the Python-side protocol and mock planning pieces. It does
not yet implement completed autonomous ROS2 execution or confirmed end-to-end
station HC-12 operation.

## Hardware Overview

- Target surface: outer hull of a ship, currently approximated as planar.
- Adhesion concept: magnetic wheels, pending design and validation.
- Rover controller: OpenRB-150.
- Manual control: RC receiver with PPM input; RC manual mode has been verified.
- GPS: fixed on the central OpenRB connector, confirmed as `Serial2` at
  `9600` baud; GPS FIX has been verified.
- Radio link: HC-12 UART is the intended station-to-rover link. Station-side
  HC-12 USB confirmation and current rover-side wiring audit are still pending.
- Actuation: ESC/motor outputs are managed by rover firmware. Bench motor tests
  must remain wheel-off-ground.
- Station/development OS: Ubuntu 24.04. WSL2 Ubuntu 24.04 and Jetson are target
  station environments.

See [docs/current_hardware_status.md](docs/current_hardware_status.md),
[docs/wiring.md](docs/wiring.md), and [firmware/README.md](firmware/README.md).

## Legacy HC-12 References

Legacy HC-12 scripts and notes from `~/Desktop/project-lab/hc12` have been
audited under [references/legacy_hc12](references/legacy_hc12). They are
reference material only, not production station or rover code.

The useful legacy patterns are mostly `9600` baud PC `readline()` loops,
Arduino/Nano `SoftwareSerial` bridges, RP2040 UART bridge notes, and old
OpenRB/Mega-style `Serial3` transmit experiments. Known problems include
hardcoded `COM4` or `/dev/cu.usbserial-*` ports, blocking loops, inconsistent
variable names, old unverified UART assumptions, and examples that directly
drive motors without this rover's STOP/failsafe model.

Do not copy those examples into active firmware or station tools blindly. Use
them only to inform new receive-only HC-12 diagnostics after the current fixed
wiring and safety constraints are rechecked.

## Firmware Modes

The OpenRB firmware modes are intentionally separated. Do not infer GPS,
HC-12, or motor behavior from the wrong mode.

| Mode | Sketch / build | Upload target | Intended use | GPS behavior | HC-12 behavior | Motor behavior |
|---|---|---|---|---|---|---|
| Default rover controller | `firmware/openrb_robot_controller` | OpenRB-150 | RC manual and HC-12 protocol baseline | Default firmware reads GPS from `Serial3`; under current fixed wiring, GPS `Serial2` is not available here and `gps_chars=0` is expected | enabled | normal safety-gated rover behavior |
| Fixed-wiring GPS Serial2 diagnostic | `firmware/openrb_robot_controller` with `FIXED_WIRING_GPS_SERIAL2_DIAG=1` | OpenRB-150 | Integrated GPS-on-`Serial2` USB debug | reads fixed GPS wiring on `Serial2` at `9600` | disabled/ignored to avoid possible `Serial2` conflict | forced neutral; manual driving does not work by design |
| Fixed-wiring RC + GPS autonomy dry-run | `firmware/openrb_robot_controller` with `FIXED_WIRING_GPS_SERIAL2_RC_AUTONOMY_DRYRUN=1` | OpenRB-150 | RC manual plus GPS readiness/distance/bearing dry-run | reads fixed GPS wiring on `Serial2` at `9600` | disabled/ignored to avoid possible `Serial2` conflict | RC MANUAL drives normally; AUTO forces neutral and computes readiness only |
| Standalone GPS probe | `firmware/gps_uart_probe` | OpenRB-150 | GPS UART/baud validation | selectable; current fixed GPS path is `Serial2` at `9600` | not used | no motor outputs |
| Serial3 loopback test | `firmware/serial3_loopback_test` | OpenRB-150 | Historical UART pin test | not a GPS test | not used | no motor outputs |
| Pin finder test | `firmware/pin_finder_test` | OpenRB-150 | Historical physical pin finder | not a GPS test | not used | no motor outputs |

### Default Rover Controller

Compile:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-default firmware/openrb_robot_controller
```

Upload:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-default firmware/openrb_robot_controller
```

Monitor:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

Expected under current fixed wiring:

- `fixed_wiring_gps_serial2_diag=false`
- `hc12_enabled=true`
- `gps_chars=0` because default firmware still reads GPS from `Serial3`
- Manual driving requires RC mode switch out of AUTO; if USBDBG shows
  `mode=AUTO_READY`, `auto_sw=true`, and `control_source=STOP`, switch RC mode
  back to manual before validating `control_source=RC_MANUAL`

### Fixed-Wiring GPS Serial2 Diagnostic

Compile:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-gps-s2-diag --build-property 'compiler.cpp.extra_flags=-DFIXED_WIRING_GPS_SERIAL2_DIAG=1' firmware/openrb_robot_controller
```

Upload:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-gps-s2-diag firmware/openrb_robot_controller
```

Monitor:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

Expected:

- GPS uses fixed `Serial2` wiring at `9600`
- HC-12 is disabled/ignored
- motors are forced neutral
- manual driving does not work in this diagnostic build by design

### Fixed-Wiring RC + GPS Autonomy Dry-Run

Compile:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-gps-s2-rc-dryrun --build-property 'compiler.cpp.extra_flags=-DFIXED_WIRING_GPS_SERIAL2_RC_AUTONOMY_DRYRUN=1' firmware/openrb_robot_controller
```

Upload:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-controller-gps-s2-rc-dryrun firmware/openrb_robot_controller
```

Monitor:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

Expected:

- GPS uses fixed `Serial2` wiring at `9600`
- HC-12 is disabled/ignored
- RC MANUAL mode preserves current manual driving behavior and reports
  `control_source=RC_MANUAL`
- RC AUTO switch position does not move the rover; it forces `left_cmd=0` and
  `right_cmd=0`, prints `autonomy_dryrun=true`, target placeholder fields,
  distance/bearing to target, and `gps_ready`, `target_ready`,
  `autonomy_ready`
- placeholder target is dry-run only:
  `35.571120,129.186050`
- onboard geodesy computes placeholder target distance and initial bearing over
  USB debug only
- no real waypoint following is implemented
- Arduino-side distance/bearing helpers are validated manually from USBDBG:
  `target_distance_m` finite with `gps_fix=true`, `target_bearing_deg` in
  `0..360`, and AUTO still `left_cmd=0` / `right_cmd=0`

Validated:

- This is the first firmware mode where RC MANUAL driving and fixed-wiring GPS
  dry-run coexist in one build.
- MANUAL mode was tested with `control_source=RC_MANUAL`, and stick input
  changed `left_cmd` / `right_cmd`.
- AUTO mode was tested with `autonomy_dryrun=true`, GPS fields,
  target distance/bearing fields, and `left_cmd=0` / `right_cmd=0`.
- With the antenna outside/open sky, `gps_fix=true` was observed.
- AUTO is still computation-only. Real motion is not enabled yet.

### Standalone GPS Probe

Compile for confirmed fixed GPS wiring:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/gps-probe-s2-9600 --build-property 'compiler.cpp.extra_flags=-DGPS_PROBE_MODE=2 -DGPS_PROBE_BAUD=9600' firmware/gps_uart_probe
```

Upload:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/gps-probe-s2-9600 firmware/gps_uart_probe
```

Monitor:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

### Serial3 Loopback Test

Historical UART pin test. Under the Fixed Wiring Plan, do not move GPS or
HC-12 unless there is an explicit hardware bench-test reason.

Compile:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-serial3-loopback-9600 --build-property 'compiler.cpp.extra_flags=-DSERIAL3_LOOPBACK_BAUD=9600' firmware/serial3_loopback_test
```

Upload:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-serial3-loopback-9600 firmware/serial3_loopback_test
```

Monitor:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

### Pin Finder Test

Historical physical pin finder. Under the Fixed Wiring Plan, do not move GPS or
HC-12 unless there is an explicit hardware bench-test reason.

Compile:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' compile --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-pin-finder-d13-d14 firmware/pin_finder_test
```

Upload:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' upload -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --build-path /private/tmp/openrb-pin-finder-d13-d14 firmware/pin_finder_test
```

Monitor:

```bash
'/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli' monitor -p /dev/cu.usbmodem12101 --fqbn OpenRB-150:samd:OpenRB-150 --config baudrate=115200
```

### Do Not Repeat

- Do not expect GPS in the default controller build under current fixed wiring;
  `gps_chars=0` is expected there because default firmware still reads GPS from
  `Serial3`.
- Do not expect manual driving in `FIXED_WIRING_GPS_SERIAL2_DIAG`; motors are
  neutral and HC-12 is disabled by design.
- Do not connect both OpenRB USB and station USB-serial during OpenRB upload if
  `arduino-cli` selects the wrong upload port.
- If upload fails because it selected `/dev/cu.usbserial-02444963`, unplug the
  station USB-serial and upload with only OpenRB connected.

### Next Architecture

For current fixed wiring, the future architecture should be a GPS `Serial2` +
RC-controlled onboard mode:

- Auto OFF: RC manual drive.
- Auto ON: onboard GPS mission/autonomy, after explicit safety design and tests.
- HC-12 is not used in this mode until hardware can be revised or proven
  independent from GPS `Serial2`.
- Station-side path planning remains dry-run until autonomy is explicitly
  implemented and safety-gated.

## Troubleshooting

### GPS In Default Rover Controller

Under current fixed wiring, do not treat default-build `gps_chars=0` as GPS
failure. The default rover controller still reads GPS from `Serial3`, while the
actual fixed GPS wiring is on `Serial2`. This build is kept as the HC-12/manual
baseline.

### GPS Serial2 Diagnostic Sky Test

The `FIXED_WIRING_GPS_SERIAL2_DIAG` build has now been sky-tested:

- `fixed_wiring_gps_serial2_diag=true`
- `hc12_enabled=false`
- `gps_chars` increased continuously
- `gps_fix=true`
- `gps_lat` / `gps_lon`, `gps_sats`, and `gps_hdop` became valid
- motors remained disarmed/neutral

This confirms GPS works through the fixed `Serial2` wiring inside integrated
firmware when HC-12 is disabled for the diagnostic mode. The successful fix
occurred after moving the external GPS antenna farther outside into clearer sky
view.

### GPS Data But No Fix

If `gps_chars>0` but `gps_fix=false`, GPS UART data is arriving but satellite
fix quality is not sufficient yet. Indoor or window-side tests may receive NMEA
bytes without acquiring a valid fix. For first fix, place the GPS antenna
outdoors with open sky view and wait before changing firmware.

If `gps_chars=0`, debug wiring, selected UART, baudrate, power, or GPS output
configuration first.

GPS sky-fix checklist:

- `gps_chars` increasing means the selected UART and baudrate are working.
- `gps_sats=0` and `gps_hdop=99.99` mean no satellite acquisition yet, not a
  UART or firmware failure by themselves.
- Move the antenna outside or into open sky before suspecting code.
- Rain did not prevent fix during the observed test once the antenna had clear
  sky exposure, but electronics, USB adapters, and antenna connectors must be
  protected from water.

## Software Architecture

Current implementation is Python-based and pre-ROS2:

- `gps_coverage_core.protocol`: serial frame encoding/decoding and checksums.
- `gps_coverage_core.geo`: WGS84/local coordinate conversion helpers.
- `gps_coverage_core.planner`: simple planar A/B lawnmower path generation.
- `gps_coverage_core.nmea`: supported NMEA parsing helpers.
- `gps_coverage_core.telemetry`: GPS telemetry model.
- `tools/`: station-side utilities for safe serial loops, GPS logging, log
  analysis, NMEA replay, path previews, and mock mission generation.
- `firmware/`: OpenRB sketches for integrated rover control and focused bring-up
  tests.
- `ros2_ws/src/`: ROS2 Jazzy package skeletons for future bridge, mission,
  planner, and waypoint follower nodes.

Core protocol and planning modules are intentionally independent from ROS2 so
they can be used by plain Python tools now and reused during the ROS2 migration.

Planned ROS2 migration:

- `hc12_bridge_node`: bridge HC-12 serial frames into ROS2 topics/services.
- `station_mission_node`: manage operator workflow and mission state.
- `coverage_planner_node`: expose coverage path generation through ROS2.
- `waypoint_follower_node`: consume planned paths and produce rover commands.

These ROS2 nodes are not complete runtime behavior yet.

## Verified Progress

Verified or implemented so far:

- RC manual control on the rover.
- GPS module communication and GPS FIX.
- Failsafe STOP behavior.
- Safe station defaults that send heartbeat and `STOP`, not live `AUTO`.
- Mock lawnmower path planning and preview generation.
- Unit tests for protocol, geodesy, and planner behavior.

Pending:

- Station-side HC-12 USB device confirmation and end-to-end link test.
- Reconciliation of GPS telemetry payload schema across firmware and Python
  tools.
- ROS2 node integration beyond skeleton packages.
- Magnetic wheel adhesion validation.
- Full autonomous field test on the target surface.

## Directory Structure

```text
gps_coverage_core/        ROS-independent Python protocol, geo, telemetry, NMEA, planner
tools/                    Station utilities, log analyzers, mock mission generators
firmware/                 OpenRB/Arduino sketches and bring-up tests
ros2_ws/src/              ROS2 Jazzy package skeletons, not completed runtime nodes
tests/                    Python tests for protocol, geo, and planner modules
scripts/                  Environment, requirements, and ROS2 workspace helper scripts
docs/                     Architecture, protocol, safety, wiring, reports, figures
docs/project_notes/       Repository audit and cleanup notes
docs/figures/             Shared generated/raw/external figure library
docs/reports/interim/     Interim report workspace
docs/reports/final/       Final report workspace
data/                     Local logs and generated mock mission outputs
outputs/                  Local generated outputs
```

## Setup

Use `uv` for dependency and task execution:

```bash
uv sync --extra dev --extra web
```

Useful checks:

```bash
uv run pytest -q
uv run python tools/verify_env.py --port /dev/ttyACM0
./scripts/export_requirements.sh
```

## Running Current Components

Analyze GPS USB debug logs:

```bash
uv run python tools/analyze_gps_log.py data/gps_logs/*.log
```

Analyze safety USB debug logs:

```bash
uv run python tools/analyze_safety_log.py data/safety_logs/*.log
```

Generate a mock station-side coverage mission without HC-12 or ROS2:

```bash
uv run python tools/station_mock_mission.py \
  --a-lat 35.123456 --a-lon 129.123456 \
  --b-lat 35.123456 --b-lon 129.124556 \
  --spacing-m 5.0 \
  --num-lanes 4 \
  --out-dir data/mock_runs/example
```

### Path Planning Preview Dry-run

Generate a station-side coverage mission dry-run from GPS corner points. Point
A is the start corner, Point B is the opposite/end corner, and
`lane_spacing_m` is the sweep interval. The default `corner-rectangle` mode
does not use `sweep_width_m`. This writes JSON, CSV, and PNG files under
`outputs/missions/` and sends no rover commands:

```bash
uv run python scripts/station/plan_coverage_path.py \
  --point-a 35.571070,129.186000 \
  --point-b 35.571250,129.186300 \
  --lane-spacing-m 5.0 \
  --speed-mps 0.4 \
  --mission-name codex_corner_rectangle_smoke
```

Outputs:

```text
outputs/missions/codex_corner_rectangle_smoke/mission.json
outputs/missions/codex_corner_rectangle_smoke/mission.csv
outputs/missions/codex_corner_rectangle_smoke/preview.png
```

Inspect the generated files:

```bash
uv run python -m json.tool outputs/missions/codex_corner_rectangle_smoke/mission.json
head -n 12 outputs/missions/codex_corner_rectangle_smoke/mission.csv
tail -n 3 outputs/missions/codex_corner_rectangle_smoke/mission.csv
ls -lh outputs/missions/codex_corner_rectangle_smoke/preview.png
```

See [docs/station_path_planning.md](docs/station_path_planning.md). Path
generation remains dry-run only and must not be sent to the rover yet. The
tested mission output is not yet executed by the rover; the next step is onboard
mission dry-run, not real motion.

Edge/remainder policy: if the rectangle extent is not exactly divisible by
`lane_spacing_m`, a small remaining margin at the edge is acceptable. Do not add
an extra lane outside the boundary just to remove that margin.

The previous A/B baseline plus sweep-width interpretation is retained only
behind `--planner-mode baseline-width --sweep-width-m ...` for comparison.

Generate a standalone path preview figure:

```bash
uv run python tools/path_preview.py \
  --lat-a 35.123456 --lon-a 129.123456 \
  --lat-b 35.123456 --lon-b 129.124556 \
  --spacing 5.0 \
  --output docs/figures/generated/path_preview.png
```

Generate report-ready mock mission artifacts in the shared figure area:

```bash
uv run python tools/station_mock_mission.py \
  --a-lat 35.123456 --a-lon 129.123456 \
  --b-lat 35.123456 --b-lon 129.124556 \
  --spacing-m 5.0 \
  --num-lanes 4 \
  --out-dir docs/figures/generated/mock_mission_example
```

When the station HC-12 USB device is connected, run only safe heartbeat/receive
testing first:

```bash
uv run python tools/station_hc12_test.py --port /dev/ttyACM0
```

The safe controller loop also defaults to heartbeat and periodic `STOP`:

```bash
uv run python tools/station_controller.py --port /dev/ttyACM0
```

Manual keyboard station testing exists, but it sends manual command frames and
must be treated as wheel-off-ground motor testing only:

```bash
uv run python tools/station_keyboard_manual.py --port /dev/ttyACM0 --max-speed 0.25
```

The keyboard tool starts in heartbeat-plus-`STOP` mode. Press `e` to arm station
manual control, press space to enable the deadman, then use `WASD` or arrow keys
for short manual pulses. Press `x` for local E-stop/disarm and `q` to exit; exit
sends repeated `STOP` frames.

See [docs/manual_control.md](docs/manual_control.md) for the current rover
firmware upload steps, RC direction mapping, USB debug checks, and station
manual-control procedure.

## Figure Gallery

Shared generated figures belong in
[docs/figures/generated/](docs/figures/generated/). This directory may be empty
until the figure-generation commands above are run.

Figure source classes are documented in [docs/figures/README.md](docs/figures/README.md):

- `generated/`: reproducible project-generated figures from scripts, logs, mock
  missions, diagrams, or processed data.
- `raw/`: original captures such as photos, screenshots, serial captures, or
  unedited exported plots.
- `external/`: third-party or reference figures with source and license notes.
- `thumbnails/`: small convenience previews derived from another source.

Report-specific figures can also be placed under:

- [docs/reports/interim/figures/generated/](docs/reports/interim/figures/generated/)
- [docs/reports/final/figures/generated/](docs/reports/final/figures/generated/)

## Report Material Map

- [docs/reports/interim/](docs/reports/interim/): interim report workspace,
  including generated/raw/external figure folders and tables.
- [docs/reports/final/](docs/reports/final/): final report workspace for
  verified claims and final evidence.
- [docs/project_notes/repo_audit.md](docs/project_notes/repo_audit.md):
  current repository audit and risk notes.
- [docs/project_notes/directory_structure_summary.md](docs/project_notes/directory_structure_summary.md):
  documentation and report folder map.
- [docs/project_notes/cleanup_summary.md](docs/project_notes/cleanup_summary.md):
  cleanup status and remaining artifact-policy decisions.

Final report claims should stay tied to verified evidence. Do not describe HC-12
station operation, ROS2 autonomy, magnetic adhesion, or full cleaning/painting
operation as complete until those items are implemented and tested.

## Limitations And Next Steps

Current limitations:

- Planner assumes a locally planar rectangular work region.
- No hull curvature handling, obstacle handling, edge exclusion zones, coating
  process constraints, or localization uncertainty margins yet.
- Magnetic wheel adhesion is a concept, not a validated subsystem.
- Station HC-12 USB and end-to-end radio communication remain pending.
- ROS2 packages are skeletons only.
- GPS payload schema needs alignment between firmware and Python telemetry
  parsing before relying on all station-side GPS tools.

Next steps:

- Confirm station-side HC-12 USB attachment and safe heartbeat/telemetry link.
- Align and document one GPS telemetry payload schema.
- Keep expanding mock mission evidence without claiming autonomous field
  completion.
- Implement ROS2 bridge, mission, planner, and follower nodes around the
  existing ROS-independent core modules.
- Validate magnetic wheel adhesion and safety behavior before any live hull test.
