# Figure Asset Library

This directory is the shared figure library for documentation and reports. Report-specific copies can also live under `docs/reports/interim/figures/` or `docs/reports/final/figures/` when they are tied to a specific submission.

## Figure Classes

- `generated/`: figures created by this repository from scripts, logs, simulations, diagrams, or processed project data. These should be reproducible or explain how they were generated.
- `raw/`: original, unedited evidence captures such as hardware photos, screenshots, serial monitor captures, exported plots, or lab images. Keep raw files as source evidence.
- `external/`: figures from outside the project, including vendor images, standards diagrams, course slides, papers, or web references. Store source, license, and attribution notes with the asset.
- `thumbnails/`: small previews derived from generated, raw, or external figures. Thumbnails are convenience files, not source evidence.

Do not overwrite raw evidence with edited versions. Put annotated, cropped, converted, or cleaned-up outputs in `generated/` unless they are report-specific.

## Generated Report Figures

Run all figure generators from the repository root:

```bash
python3 scripts/analysis/generate_all_figures.py
```

The command writes the same generated figure set to:

- `docs/figures/generated/`
- `docs/reports/interim/figures/generated/`
- `docs/reports/final/figures/generated/`

No required figure uses an external data source. Schematic and path-planning figures are mock/project-derived and should not be described as measured validation results.

| Filename | Script | Data source | Recommended README/report use |
| --- | --- | --- | --- |
| `fig_system_overview.png` | `scripts/analysis/generate_system_figures.py` | mock schematic derived from repository docs/code | System architecture overview and boundary explanation |
| `fig_control_flow.png` | `scripts/analysis/generate_system_figures.py` | mock schematic derived from repository docs/code | Control architecture and safe station-default explanation |
| `fig_state_machine.png` | `scripts/analysis/generate_system_figures.py` | mock schematic derived from repository docs/code | Rover safety-state summary; design intent, not formal verification |
| `fig_ab_region_definition.png` | `scripts/analysis/generate_path_figures.py` | mock mission waypoints, with deterministic fallback | Planning method: manual A/B point selection and planar region definition |
| `fig_lawnmower_path_preview.png` | `scripts/analysis/generate_path_figures.py` | mock mission waypoints, with deterministic fallback | Planning preview: lawnmower-style coverage geometry |
| `fig_waypoint_sequence.png` | `scripts/analysis/generate_path_figures.py` | mock mission waypoints, with deterministic fallback | Planner implementation: waypoint order and alternating lane direction |
| `fig_gps_fix_timeline.png` | `scripts/analysis/generate_gps_figures.py` | real GPS log when available; mock/demo fallback | GPS validation: fix availability over the selected capture |
| `fig_gps_satellites_vs_time.png` | `scripts/analysis/generate_gps_figures.py` | real GPS log when available; mock/demo fallback | GPS validation: satellite count during valid-fix records |
| `fig_gps_hdop_vs_time.png` | `scripts/analysis/generate_gps_figures.py` | real GPS log when available; mock/demo fallback | GPS validation: HDOP trend without overclaiming navigation accuracy |
| `fig_gps_position_scatter.png` | `scripts/analysis/generate_gps_figures.py` | real GPS log when available; mock/demo fallback | GPS validation: local position spread for valid fixes |
| `fig_manual_control_timeline.png` | `scripts/analysis/generate_manual_control_figures.py` | real safety log when available; mock/demo fallback | Safety validation: RC/manual input and normalized motor command timeline |
| `fig_failsafe_event_timeline.png` | `scripts/analysis/generate_manual_control_figures.py` | real USB debug log with failsafe events when available; mock/demo fallback | Failsafe validation: logged invalid-RC, failsafe, STOP, and zero-command intervals |
| `fig_control_source_transition.png` | `scripts/analysis/generate_manual_control_figures.py` | real safety log when available; mock/demo fallback | Control validation: transitions among logged control sources |

Captions for report insertion are generated at `docs/figures/generated/figure_captions.md`.
