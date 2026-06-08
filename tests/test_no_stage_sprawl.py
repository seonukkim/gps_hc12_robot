from __future__ import annotations

import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_active_scripts_do_not_expose_numbered_launchers() -> None:
    assert not list((ROOT / "scripts").glob("run_" + "stage*.sh"))


def test_active_tools_do_not_expose_numbered_modules() -> None:
    active_tools = ROOT / "tools"
    assert not list(active_tools.glob("stage*.py"))
    assert not list(active_tools.glob("check_" + "stage*.py"))
    assert not list(active_tools.glob("build_" + "stage*.py"))


def test_active_tests_do_not_collect_numbered_stage_tests() -> None:
    assert not list((ROOT / "tests").glob("test_" + "stage*.py"))


def test_active_configs_do_not_expose_numbered_stage_configs() -> None:
    assert not list((ROOT / "configs").glob("*" + "stage*"))


def test_primary_docs_do_not_recommend_numbered_stage_workflow() -> None:
    primary_docs = [
        ROOT / "README.md",
        ROOT / "docs" / "README_physical_path_planning.md",
        ROOT / "docs" / "field_test_manual.md",
        ROOT / "docs" / "physical_path_planning_architecture.md",
    ]
    forbidden = [
        "run_" + "stage",
        "tools/" + "stage",
        "check_" + "stage",
        "test_" + "stage",
    ]
    for path in primary_docs:
        text = path.read_text(encoding="utf-8")
        assert "scripts/run_physical_path_planner.sh" in text
        for token in forbidden:
            assert token not in text


def test_unified_cli_help_does_not_expose_numbered_terms() -> None:
    completed = subprocess.run(
        ["bash", "scripts/run_physical_path_planner.sh", "--help"],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert completed.returncode == 0
    help_text = completed.stdout
    assert "run_" + "stage" not in help_text
    for old_term in ("Stage" + "20", "Stage" + "16", "Stage" + "35", "Stage" + "36"):
        assert old_term not in help_text
