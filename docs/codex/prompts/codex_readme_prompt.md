# Codex README Prompt

Use this reusable prompt when updating README or top-level project narrative.

## Prompt

You are working in the `gps_hc12_robot` repository on README-quality project
documentation.

Project context:

- Current implementation is Python-based and pre-ROS2 unless current files prove
  otherwise.
- Implemented areas include ROS-independent protocol/planner utilities,
  station-side safety tooling, GPS/log handling, mock missions, tests, and
  generated report figures.
- ROS2 Jazzy integration is planned and package skeletons may exist, but runtime
  bridge, mission, planner, and follower behavior must not be claimed complete
  unless implemented and tested.
- HC-12 is the intended UART radio link. Station-side HC-12 USB confirmation and
  end-to-end operation remain pending unless verified by current evidence.
- Hardware mappings, serial defaults, safety constraints, and wheel-off-ground
  motor-test language must be preserved.

Before editing:

1. Inspect `README.md`, `docs/current_hardware_status.md`,
   `docs/architecture/`, `docs/protocol.md`, and relevant tools/tests.
2. Check `git status` and preserve unrelated changes.
3. Search for existing ROS2 and HC-12 wording before adding new claims.

Recommended checks:

```bash
rg -n "ROS2|ROS 2|HC-12|HC12" README.md docs
uv run pytest -q
```

Working rules:

- Distinguish implemented behavior from planned behavior.
- Do not turn intended workflow into completed field results.
- Keep HC-12 pending unless verified.
- Keep ROS2 planned unless implemented.
- Preserve `/dev/ttyACM0` as the default USB serial device and expose `--port`
  in station serial tooling language.
- Preserve safe station defaults: heartbeat and `STOP` only unless explicitly
  changed.
- Do not create `codex_task*.md` files or temporary task folders in the repo.
- Do not commit or push unless explicitly asked.

Task:

[Describe the README or narrative update.]

Expected validation:

[Describe checks to run or docs to inspect.]
