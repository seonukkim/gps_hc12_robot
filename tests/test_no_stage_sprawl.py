"""단계(stage) 번호 스프롤 방지 가드 테스트 / Guard test against numbered "stage" sprawl in the active tree.

목적/역할:
    이 저장소는 예전 "stageNN" 번호식 워크플로(런처 스크립트/도구/테스트/설정/문서)를 단일
    통합 CLI(`scripts/run_physical_path_planner.sh`)로 대체했다. 이 테스트는 활성 트리에
    번호식 잔재가 **다시 새어 들어오지 못하도록** 잠근다(회귀 방지 가드).

시스템 내 위치:
    소스 계약이 아니라 저장소 위생(hygiene) 계약이다. scripts/·tools/·tests/·configs/·docs/
    를 훑어 금지 패턴을 확인하고, 통합 런처의 --help 출력까지 검사한다. 레거시는 `legacy/`로
    격리되어 있으므로 이 가드의 대상이 아니다.

핵심 개념·불변식:
    - 금지 토큰(예 run_stage, tools/stage, check_stage, test_stage, Stage20/16/35/36)이
      활성 경로·주요 문서·CLI 도움말에 존재하면 실패한다.
    - 토큰 문자열은 "run_" + "stage"처럼 **분할 리터럴**로 적혀 있다 — 이 파일 자체가 자신의
      grep에 걸리지 않게 하려는 의도적 표현이므로 합치지 말 것(합치면 자기참조로 오탐).

리팩토링 노트:
    통합 CLI 경로(scripts/run_physical_path_planner.sh)를 바꾸면 문서 검사 문자열도 갱신할 것.

Guard test: this repo replaced the old numbered "stageNN" workflow (launcher scripts, tools, tests,
configs, docs) with one unified CLI (``scripts/run_physical_path_planner.sh``). These tests lock the
active tree so numbered remnants cannot creep back in. This is a repo-hygiene contract, not a source
contract; legacy code lives isolated under ``legacy/``. NOTE: forbidden tokens are written as split
literals (e.g. ``"run_" + "stage"``) on purpose so this file does not match its own greps — do not
join them.
"""

from __future__ import annotations

import subprocess
from pathlib import Path


# ── 저장소 루트 / Repository root (tests/ 의 부모) ──
ROOT = Path(__file__).resolve().parents[1]


def test_active_scripts_do_not_expose_numbered_launchers() -> None:
    """scripts/에 run_stage*.sh 번호식 런처가 없음을 검증. / No numbered run_stage*.sh launchers remain under scripts/."""
    assert not list((ROOT / "scripts").glob("run_" + "stage*.sh"))


def test_active_tools_do_not_expose_numbered_modules() -> None:
    """tools/에 stage*/check_stage*/build_stage* 모듈이 없음을 검증. / No numbered stage modules remain under tools/."""
    active_tools = ROOT / "tools"
    assert not list(active_tools.glob("stage*.py"))
    assert not list(active_tools.glob("check_" + "stage*.py"))
    assert not list(active_tools.glob("build_" + "stage*.py"))


def test_active_tests_do_not_collect_numbered_stage_tests() -> None:
    """tests/에 test_stage*.py 번호식 테스트가 없음을 검증. / No numbered test_stage*.py test files remain under tests/."""
    assert not list((ROOT / "tests").glob("test_" + "stage*.py"))


def test_active_configs_do_not_expose_numbered_stage_configs() -> None:
    """configs/에 stage 이름이 든 설정이 없음을 검증. / No config files containing "stage" remain under configs/."""
    assert not list((ROOT / "configs").glob("*" + "stage*"))


def test_primary_docs_do_not_recommend_numbered_stage_workflow() -> None:
    """주요 문서가 통합 런처를 안내하고 번호식 토큰을 언급하지 않음을 검증. / Primary docs point to the unified launcher and never mention numbered-stage tokens."""
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
    """통합 CLI --help가 성공하고 번호식 용어를 노출하지 않음을 검증. / The unified CLI --help exits 0 and exposes no numbered-stage terms."""
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
