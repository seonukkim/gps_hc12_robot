# Codex Figures Prompt

Use this reusable prompt when updating report figures, captions, or figure
generation scripts.

## Prompt

You are working in the `gps_hc12_robot` repository on report-ready figures.

Context to preserve:

- Generated figures should come from reproducible scripts where possible.
- Shared generated figures belong under `docs/figures/generated/`.
- Interim report generated figures belong under
  `docs/reports/interim/figures/generated/`.
- Final report generated figures belong under
  `docs/reports/final/figures/generated/`.
- Figure captions must distinguish schematic, mock, simulated, logged, and
  measured evidence.
- Do not describe ROS2 autonomy or station-side HC-12 operation as complete
  unless the current repo contains verified implementation and test evidence.
- Motor and ESC testing must be described as wheel-off-ground bench testing
  unless field validation is explicitly documented.

Before editing:

1. Inspect the current figure scripts and generated captions.
2. Inspect report docs that reference the affected figures.
3. Check `git status` and avoid overwriting unrelated changes.

Preferred commands:

```bash
uv run python scripts/analysis/generate_all_figures.py
uv run pytest -q
```

Working rules:

- Prefer editing figure scripts or source data over hand-editing generated PNGs.
- Keep outputs report-friendly: clear titles, readable labels, conservative
  captions, and explicit data-source wording.
- Keep generated figures and captions reproducible from the command above.
- Do not create `codex_task*.md` files or temporary task folders in the repo.
- Do not commit or push unless explicitly asked.

Task:

[Describe the figure or caption update.]

Validation requested:

[List the commands or report sections to inspect.]
