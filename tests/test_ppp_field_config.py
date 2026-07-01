"""preview 와 execute-plan 이 "해석된 필드 설정(resolved field config)"을 일관되게
산출·공유함을 고정하는 통합 테스트.

목적/역할 (KO):
    ``preview`` 로 계획을 세우면 A(원점)와 B(오프셋)가 로컬 좌표로 해석되어
    ``field_config_resolved.json`` / ``summary.json`` 에 기록되고, 그 계획 폴더를
    ``execute-plan`` 이 다시 읽었을 때 *동일한* 해석 결과를 재현함을 CLI end-to-end
    로 못 박는다. preview 와 run 이 서로 다른 필드 설정을 쓰면 "미리 본 것과 다른
    경로를 실행"하는 사고가 나므로, 이 일치가 핵심 안전 계약이다.

핵심 계약·불변식 (KO):
    - ``relative_enu`` 모드: 시작점은 로컬 원점 (0, 0), 목표는 동/북 오프셋
      그대로 ``resolved_goal_x_m`` / ``resolved_goal_y_m`` 로 해석된다.
    - preview 는 ``field_config_resolved.json`` + ``planned_segments.csv`` +
      ``planned_primitives.csv`` 를 남기고, ``summary.json`` 에도 필드 설정을 포함.
    - ``execute-plan --plan-dir <preview>`` 는 계획 폴더에서 필드 설정을 로드하며
      (``start_source="plan_dir"``) 동일한 해석 좌표를 재현한다.
    - ``--print-field-config true`` 로 출력한 사람이 읽는(human-readable) 필드
      설정 블록이 preview 와 run 양쪽에서 동일한 줄들을 포함한다.

Purpose (EN):
    CLI end-to-end tests that ``preview`` resolves the A-origin / B-offset field
    config, persists it (``field_config_resolved.json``, ``summary.json``,
    planned CSVs), and that ``execute-plan --plan-dir`` reloads and reproduces the
    *same* resolved coordinates (``start_source=plan_dir``). Also asserts preview
    and run print an identical human-readable field-config block -- so you never
    execute a path different from the one you previewed.
"""
import json
from pathlib import Path

import pytest

from tools.physical_path_planning import cli


# ── 공용 preview 인자(상대 ENU 필드) / Shared preview args (relative-ENU field) ──
_RELATIVE_PREVIEW_ARGS = [
    "preview",
    "--start-lat",
    "35.5709000",
    "--start-lon",
    "129.1871000",
    "--goal-mode",
    "relative_enu",
    "--goal-east-m",
    "4.0",
    "--goal-north-m",
    "-1.2",
    "--workspace-width-m",
    "1.2",
    "--step-spacing-m",
    "0.25",
    "--no-png",
]


def test_relative_enu_field_config_resolves_a_origin_and_b_offset(tmp_path: Path) -> None:
    """relative_enu preview 가 A=원점·B=오프셋을 해석해 파일로 남긴다(계획 CSV 포함)
    / relative_enu preview resolves A=origin, B=offset and writes them to the
    resolved-config JSON plus the planned CSVs."""
    rc = cli.main([*_RELATIVE_PREVIEW_ARGS, "--out-dir", str(tmp_path)])
    assert rc == 0
    field = json.loads((tmp_path / "field_config_resolved.json").read_text())
    assert field["start_x_m"] == 0.0
    assert field["start_y_m"] == 0.0
    assert field["goal_mode"] == "relative_enu"
    assert field["goal_east_m"] == 4.0
    assert field["goal_north_m"] == -1.2
    assert field["resolved_goal_x_m"] == 4.0
    assert field["resolved_goal_y_m"] == -1.2
    assert field["workspace_width_m"] == 1.2
    assert field["step_spacing_m"] == 0.25
    assert field["expected_lane_count"] > 1
    assert (tmp_path / "planned_segments.csv").exists()
    assert (tmp_path / "planned_primitives.csv").exists()


def test_preview_5m_relative_enu_resolves_a_origin_and_5m_east_goal(tmp_path: Path) -> None:
    """문서화된 5m 현장 실험(A=(0,0), B=(5,0), 폭 1.5m)이 그대로 해석된다 / the
    documented 5m field experiment resolves A=(0,0), B=(5,0) with a 1.5m width."""
    # The documented 5m field experiment: A=(0,0), B=(5,0), width 1.5m.
    rc = cli.main(
        [
            "preview",
            "--start-lat", "35.5709000",
            "--start-lon", "129.1871000",
            "--goal-mode", "relative_enu",
            "--goal-east-m", "5.0",
            "--goal-north-m", "0.0",
            "--workspace-width-m", "1.5",
            "--step-spacing-m", "0.30",
            "--path-shape", "diagonal_rectangle_serpentine",
            "--no-png",
            "--out-dir", str(tmp_path),
        ]
    )
    assert rc == 0
    field = json.loads((tmp_path / "field_config_resolved.json").read_text())
    assert field["start_x_m"] == 0.0
    assert field["start_y_m"] == 0.0
    assert field["resolved_goal_x_m"] == pytest.approx(5.0)
    assert field["resolved_goal_y_m"] == pytest.approx(0.0)
    assert field["workspace_width_m"] == 1.5
    assert field["step_spacing_m"] == 0.30
    assert field["lane_count"] > 1
    assert field["segment_count"] > 1


def test_preview_summary_includes_resolved_field_config(tmp_path: Path) -> None:
    """summary.json 이 해석된 필드 설정 블록(시작 위경도·폭·경로 형상)을 포함한다 /
    ``summary.json`` embeds the resolved field-config block (start lat/lon, width,
    path shape)."""
    rc = cli.main([*_RELATIVE_PREVIEW_ARGS, "--out-dir", str(tmp_path)])
    assert rc == 0
    summary = json.loads((tmp_path / "summary.json").read_text())
    assert summary["field_config"]["start_lat"] == 35.5709
    assert summary["field_config"]["workspace_width_m"] == 1.2
    # No --path-shape given, so it defaults to coverage_lawnmower (not diagonal).
    assert summary["field_config"]["path_shape"] == "coverage_lawnmower"


def test_execute_plan_loads_field_config_from_plan_dir(tmp_path: Path) -> None:
    """execute-plan 이 계획 폴더에서 필드 설정을 로드해(start_source=plan_dir) 동일
    해석 좌표를 재현한다 / ``execute-plan`` reloads the field config from the plan
    dir (``start_source=plan_dir``) and reproduces the same resolved coords."""
    plan_dir = tmp_path / "preview"
    out_dir = tmp_path / "execute"
    rc = cli.main([*_RELATIVE_PREVIEW_ARGS, "--out-dir", str(plan_dir)])
    assert rc == 0
    rc = cli.main(["execute-plan", "--plan-dir", str(plan_dir), "--print-plan", "--out-dir", str(out_dir)])
    assert rc == 0
    field = json.loads((out_dir / "field_config_resolved.json").read_text())
    assert field["start_source"] == "plan_dir"
    assert field["resolved_goal_x_m"] == 4.0
    assert field["resolved_goal_y_m"] == -1.2


def test_preview_and_run_print_same_resolved_field_config(tmp_path: Path, capsys) -> None:
    """--print-field-config 출력이 preview 와 run 양쪽에서 동일한 줄들을 담는다(미리
    본 것과 다른 경로 실행 방지) / ``--print-field-config`` prints identical lines
    in both preview and run, so you never execute a path you didn't preview."""
    preview_dir = tmp_path / "preview"
    run_dir = tmp_path / "run"
    rc = cli.main([*_RELATIVE_PREVIEW_ARGS, "--print-field-config", "true", "--out-dir", str(preview_dir)])
    assert rc == 0
    preview_out = capsys.readouterr().out
    rc = cli.main(
        [
            "execute-plan",
            "--plan-dir",
            str(preview_dir),
            "--print-plan",
            "--print-field-config",
            "true",
            "--out-dir",
            str(run_dir),
        ]
    )
    assert rc == 0
    run_out = capsys.readouterr().out
    for expected in (
        "Field configuration:",
        "goal_mode=relative_enu",
        "resolved_goal_x_m=4.0",
        "resolved_goal_y_m=-1.2",
        "workspace_width_m=1.2",
        "step_spacing_m=0.25",
    ):
        assert expected in preview_out
        assert expected in run_out
