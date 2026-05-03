# Codex Master Prompt

Use this reusable prompt to start future Codex sessions for this repository.
Replace bracketed values before use.

## Prompt

You are working in the `gps_hc12_robot` repository.

Project context:

- This is a pre-ROS2 Python/OpenRB prototype for an Industrial Engineering ship
  exterior surface robot project.
- The current implemented work focuses on safe station tooling, protocol
  utilities, GPS/log handling, ROS-independent coverage planning, mock mission
  outputs, documentation, and report figures.
- The intended workflow is manual operation, A/B point recording, rectangular
  work-region definition, and lawnmower-style coverage path generation.
- ROS2 Jazzy integration is planned, but current ROS2 packages are skeletons
  unless current files prove otherwise.
- HC-12 is the intended station-to-rover UART radio link. Keep station-side
  HC-12 operation pending unless current hardware logs or docs verify it.
- Core protocol and planning modules must remain independent from ROS2.
- Safe station defaults are heartbeat and `STOP` only unless explicitly changed.
- Default station USB serial device is `/dev/ttyACM0`; station serial tools
  should expose `--port`.
- Rover motor testing is wheel-off-ground only.

Before making changes:

1. Inspect the current files first. Do not rely on older memory of the repo.
2. Check `git status` and preserve any existing user changes.
3. Read the relevant implementation, docs, tests, and scripts before editing.

Working rules:

- Do not commit or push automatically. Only commit or push when explicitly asked.
- Do not create or recreate `codex_task*.md` files.
- Do not create temporary task folders such as `docs/codex/tasks/`.
- Keep one-off prompts in `/tmp` or paste them directly into the session.
- Preserve documented hardware mappings, pin mappings, serial defaults, safety
  limits, and wheel-off-ground motor-test constraints.
- Distinguish implemented behavior from planned behavior in README, reports,
  architecture docs, and captions.
- Keep HC-12 station operation pending unless verified by current evidence.
- Keep ROS2 runtime behavior planned unless implemented and tested.
- Do not introduce station-side startup behavior that sends live motor-driving
  `AUTO` commands.
- Prefer reproducible scripts and report-friendly outputs over hand-edited
  screenshots or ad hoc generated files.

Typical validation:

```bash
git status
find . -name 'codex_task*.md' -not -path './.git/*'
uv run python scripts/analysis/generate_all_figures.py
uv run pytest -q
rg -n "ROS2|ROS 2|HC-12|HC12" README.md docs
find docs/reports -type f -name '*.md' -print
```

Current task:

[Describe the specific task here.]

Expected output:

[Describe the files, docs, tests, figures, or report sections expected.]
