# Validation Before Commit

Use this checklist before committing repository changes. Keep one-off Codex
task prompts outside the repository, such as in `/tmp`, or paste them directly
into the session.

## 1. Check Working Tree

```bash
git status
```

Confirm the changed files are intentional. Do not revert unrelated user changes.

## 2. Find Legacy Codex Task Files

```bash
find . -name 'codex_task*.md' -not -path './.git/*'
```

Expected result: no files. Do not commit `codex_task*.md` files or
`docs/codex/tasks/` temporary task files.

## 3. Generate Figures

```bash
uv run python scripts/analysis/generate_all_figures.py
```

Review generated figures and `docs/figures/generated/figure_captions.md` for
clear data-source wording.

## 4. Run Tests If Available

```bash
uv run pytest -q
```

If tests are unavailable in a future environment, note why they could not be
run before committing.

## 5. Check README For False ROS2/HC-12 Claims

```bash
rg -n "ROS2|ROS 2|HC-12|HC12" README.md docs
```

README and docs should say ROS2 runtime behavior is planned unless implemented
and tested. HC-12 station-side operation should remain pending unless verified
by current evidence.

## 6. Inspect Report Docs

```bash
find docs/reports -type f -name '*.md' -print
```

Open the affected report docs and confirm they distinguish implemented,
planned, mock, schematic, logged, and measured results.

## 7. Commit

```bash
git status
git add README.md docs scripts tools tests gps_coverage_core firmware
git commit -m "docs: update project validation notes"
```

Adjust the `git add` paths to include only intended changes.

Suggested commit messages:

```text
docs: add reusable Codex workflow prompts
docs: update validation checklist before commit
docs: refresh report figures and captions
docs: clarify ROS2 and HC-12 project status
test: cover protocol and planner behavior
```

## 8. Push

```bash
git push
```

Push only after the commit contents and validation results are reviewed.
