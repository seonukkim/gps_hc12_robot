"""커버리지 잔디깎기(ㄹ자) 계획 생성과 preview/inspect/execute 아티팩트 계약.

목적/역할: relative_enu 목표에 대해 ``preview.build_preview`` 가 만드는 잔디깎기(lawnmower,
ㄹ자) 경로의 기하/세그먼트 구조를 잠근다. 교대(전진/후진) 레인, 레인-간 커넥터, 그리고
기본 코너 스타일(turn_step_turn: 피벗 -> 스텝오버 직진 -> 피벗)의 방향/각도/헤딩을 검증한다.

시스템 내 위치: ``preview`` 는 계획 단계(모션 없음)이고, ``cli`` 의 preview/inspect-plan/
execute-plan 모드가 이를 파일로 내보낸다. 이 테스트는 그 계획 구조와 산출 파일 집합을 고정한다.

핵심 개념·불변식:
  - 기본 경로 shape 는 ``coverage_lawnmower``, sweep 스타일은 ``lawnmower_ㄹ``.
  - full 레인은 전진/후진이 교대해도 body_heading 은 항상 레인 축(동쪽, 0°)을 향한다
    — 후진 레인을 주행 가능하게 만드는 핵심(스텝오버만 북향 이동).
  - 프리미티브는 세그먼트와 1:1 동기화, 회전(turn_*) 프리미티브는 b-only(a_cmd≈0).
  - ``diagonal_rectangle_serpentine`` 는 선택 가능하지만 기본이 아니다.

Coverage lawnmower (ㄹ-shape) planning + preview/inspect/execute artifact
contracts. Locks in the lawnmower geometry/segment structure ``build_preview``
emits for a relative_enu goal: alternating forward/backward lanes, lane-to-lane
connectors, and the default turn_step_turn corner (pivot -> straight step-over ->
pivot). Full lanes keep body_heading on the lane axis (east/0°) even as travel
alternates, which is what makes reverse lanes drivable; primitives stay 1:1 with
segments and turn primitives are b-only. Also asserts the CLI preview/inspect/
execute artifact set and that serpentine is available but not the default.
"""
from __future__ import annotations

import json
from pathlib import Path

import pytest

from tools.physical_path_planning import cli, preview


# ── 픽스처·헬퍼 / Fixtures & helpers ──────────────────────────────────────────


def _coverage_preview_args(out_dir: Path) -> list[str]:
    """커버리지 preview CLI 인자 한 벌(고정 시작점/1.2m 사각/0.4m 간격). / Canned preview CLI args."""
    return [
        "preview",
        "--start-lat",
        "35.5709000",
        "--start-lon",
        "129.1871000",
        "--goal-mode",
        "relative_enu",
        "--goal-east-m",
        "1.2",
        "--goal-north-m",
        "1.2",
        "--workspace-width-m",
        "1.2",
        "--step-spacing-m",
        "0.4",
        "--out-dir",
        str(out_dir),
    ]


def test_coverage_lawnmower_relative_enu_creates_alternating_lanes() -> None:
    """1.2m×1.2m/0.4m 간격 -> 교대 방향의 레인 4개 + 커넥터 3개, ㄹ자 sweep.

    각 레인은 한 축이 고정(수평/수직)이고 시작/끝 x 가 0↔1.2 로 교대함을 확인.
    A 1.2m square at 0.4m spacing yields 4 alternating lanes + 3 connectors in a
    ㄹ (lawnmower) sweep; each lane holds one axis fixed and x alternates 0<->1.2."""
    plan = preview.build_preview(
        start_lat=35.5709,
        start_lon=129.1871,
        goal_mode="relative_enu",
        goal_east_m=1.2,
        goal_north_m=1.2,
        workspace_width_m=1.2,
        step_spacing_m=0.4,
    )

    assert plan["path_shape"] == "coverage_lawnmower"
    lanes = [
        segment
        for segment in plan["segments"]
        if str(segment["segment_type"]) in {"forward_lane", "backward_lane"}
    ]
    assert len(lanes) == 4
    assert plan["lane_count"] == 4
    assert plan["connector_count"] == 3  # lane-to-lane transitions
    assert lanes[0]["start_x_m"] == pytest.approx(0.0)
    assert lanes[0]["start_y_m"] == pytest.approx(0.0)
    assert lanes[0]["end_x_m"] == pytest.approx(1.2)
    assert lanes[0]["end_y_m"] == pytest.approx(0.0)
    assert lanes[1]["start_x_m"] == pytest.approx(1.2)
    assert lanes[1]["end_x_m"] == pytest.approx(0.0)
    assert lanes[2]["start_x_m"] == pytest.approx(0.0)
    assert lanes[2]["end_x_m"] == pytest.approx(1.2)
    assert lanes[3]["start_x_m"] == pytest.approx(1.2)
    assert lanes[3]["end_x_m"] == pytest.approx(0.0)
    for lane in lanes:
        assert lane["start_x_m"] == pytest.approx(lane["end_x_m"]) or lane["start_y_m"] == pytest.approx(lane["end_y_m"])
    assert plan["expected_sweep_style"] == "lawnmower_ㄹ"


def test_coverage_lawnmower_default_corner_is_turn_step_turn() -> None:
    """기본 코너 = turn_step_turn: 피벗~90° -> 스텝오버 직진 -> 피벗 복귀.

    전진 레인 뒤 스텝은 forward, 후진 레인 뒤 스텝은 reverse 지만 둘 다 북향 이동.
    full 레인 body_heading 은 항상 0°(동), 회전 프리미티브는 6개·b-only 임을 검증.
    Default corner is turn_step_turn (pivot ~90 -> straight step-over -> pivot);
    step is forward after forward lanes, reverse after backward lanes, both travel
    north; full lanes face east (0deg) and the 6 turn primitives are b-only."""
    # Default ㄹ corner is drivable: pivot ~90, drive the step-over straight
    # (forward after forward lanes, reverse after backward lanes), pivot back.
    plan = preview.build_preview(
        start_lat=35.5709,
        start_lon=129.1871,
        goal_mode="relative_enu",
        goal_east_m=1.2,
        goal_north_m=1.2,
        workspace_width_m=1.2,
        step_spacing_m=0.4,
    )
    assert plan["connector_style"] == "turn_step_turn"
    assert plan["step_lane_count"] == 3
    assert plan["connector_turn_count"] == 6
    segments = plan["segments"]
    types = [str(seg["segment_type"]) for seg in segments]
    assert types == [
        "forward_lane",
        "connector_turn", "step_lane", "connector_turn",
        "backward_lane",
        "connector_turn", "step_lane", "connector_turn",
        "forward_lane",
        "connector_turn", "step_lane", "connector_turn",
        "backward_lane",
    ]

    # Transition after the FORWARD lane: left pivot, forward step north, right pivot.
    turn_in, step, turn_out = segments[1], segments[2], segments[3]
    assert turn_in["expected_motion_direction"] == "turn_left"
    assert turn_in["turn_angle_deg"] == pytest.approx(90.0)
    assert turn_in["length_m"] == pytest.approx(0.0)
    assert step["expected_motion_direction"] == "forward"
    assert step["length_m"] == pytest.approx(0.4)
    assert step["start_x_m"] == pytest.approx(1.2)
    assert step["start_y_m"] == pytest.approx(0.0)
    assert step["end_y_m"] == pytest.approx(0.4)
    assert step["target_heading_deg"] == pytest.approx(90.0)  # travel north
    assert step["body_heading_deg"] == pytest.approx(90.0)  # body faces travel
    assert turn_out["expected_motion_direction"] == "turn_right"
    assert turn_out["turn_angle_deg"] == pytest.approx(-90.0)

    # Transition after the BACKWARD lane: right pivot, REVERSE step north, left pivot.
    turn_in2, step2, turn_out2 = segments[5], segments[6], segments[7]
    assert turn_in2["expected_motion_direction"] == "turn_right"
    assert turn_in2["turn_angle_deg"] == pytest.approx(-90.0)
    assert step2["expected_motion_direction"] == "backward"
    assert step2["target_heading_deg"] == pytest.approx(90.0)  # still travels north
    assert step2["body_heading_deg"] == pytest.approx(-90.0)  # body faces south while reversing
    assert turn_out2["expected_motion_direction"] == "turn_left"
    assert turn_out2["turn_angle_deg"] == pytest.approx(90.0)

    # Body heading: full lanes alternate forward/backward but the body always
    # faces east (the lane axis), which is what makes reverse lanes drivable.
    lanes = [seg for seg in segments if str(seg["segment_type"]).endswith("_lane") and seg["segment_type"] != "step_lane"]
    assert all(float(seg["body_heading_deg"]) == pytest.approx(0.0) for seg in lanes)

    # Primitives stay in sync with segments (one per segment, turns are B-only).
    primitives = plan["primitives"]
    assert len(primitives) == len(segments)
    turn_primitives = [p for p in primitives if str(p["primitive_type"]).startswith("turn_")]
    assert len(turn_primitives) == 6
    assert all(float(p["a_cmd"]) == pytest.approx(0.0) for p in turn_primitives)


def test_coverage_lawnmower_single_turn_style_keeps_legacy_structure() -> None:
    """connector_style=single_turn 은 레거시 구조 유지: 커넥터 3개, 세그먼트 7개, 스텝 레인 0.

    커넥터 방향은 left/right/left 로 교대.
    connector_style=single_turn keeps the legacy shape: 3 connectors, 7 segments,
    0 step lanes, with connector turns alternating left/right/left."""
    plan = preview.build_preview(
        start_lat=35.5709,
        start_lon=129.1871,
        goal_mode="relative_enu",
        goal_east_m=1.2,
        goal_north_m=1.2,
        workspace_width_m=1.2,
        step_spacing_m=0.4,
        connector_style="single_turn",
    )
    connectors = [seg for seg in plan["segments"] if seg["segment_type"] == "path_connector"]
    assert len(connectors) == 3
    assert plan["segment_count"] == 7
    assert plan["connector_style"] == "single_turn"
    assert plan["step_lane_count"] == 0
    assert [c["expected_motion_direction"] for c in connectors] == [
        "turn_left", "turn_right", "turn_left",
    ]


def test_coverage_lawnmower_preview_writes_required_artifacts(tmp_path: Path) -> None:
    """preview CLI(rc=0)가 요구 산출물(PNG 2 + JSON/CSV/MD)을 모두 쓰고 field 를 해석.

    field_config_resolved.json 의 shape/목표좌표/커넥터 수/sweep/이미지 경로를 확인.
    The preview CLI (rc=0) writes the full artifact set (2 PNGs + JSON/CSV/MD) and
    a resolved field config with the expected shape, goal coords, connectors, sweep."""
    rc = cli.main(_coverage_preview_args(tmp_path))
    assert rc == 0

    assert (tmp_path / "preview_current_goal_rectangle_path.png").exists()
    assert (tmp_path / "preview_overview.png").exists()
    assert (tmp_path / "field_config_resolved.json").exists()
    assert (tmp_path / "planned_segments.csv").exists()
    assert (tmp_path / "planned_segments.json").exists()
    assert (tmp_path / "summary.md").exists()
    assert (tmp_path / "summary.json").exists()
    field = json.loads((tmp_path / "field_config_resolved.json").read_text())
    assert field["path_shape"] == "coverage_lawnmower"
    assert field["goal_x_m"] == pytest.approx(1.2)
    assert field["goal_y_m"] == pytest.approx(1.2)
    assert field["connector_count"] == 3
    assert field["expected_sweep_style"] == "lawnmower_ㄹ"
    assert "preview_current_goal_rectangle_path" in field["image_paths"]


def test_inspect_plan_detects_missing_preview_images(tmp_path: Path) -> None:
    """inspect-plan 은 삭제된 preview 이미지를 감지해 rc=1 + PREVIEW_IMAGE_MISSING 보고.

    inspect-plan detects a deleted preview image (rc=1, PREVIEW_IMAGE_MISSING)."""
    plan_dir = tmp_path / "preview"
    rc = cli.main(_coverage_preview_args(plan_dir))
    assert rc == 0
    (plan_dir / "preview_overview.png").unlink()

    rc = cli.main(["inspect-plan", "--plan-dir", str(plan_dir)])

    assert rc == 1
    summary = json.loads((plan_dir / "summary.json").read_text())
    assert summary["mode"] == "inspect-plan"
    assert summary["reason"] == "PREVIEW_IMAGE_MISSING"
    assert any("preview_overview.png" in path for path in summary["missing_preview_images"])


def test_execute_plan_print_plan_uses_saved_planned_segments(tmp_path: Path) -> None:
    """execute-plan --print-plan 은 재계산이 아니라 저장된 planned_segments.json 을 그대로 사용.

    저장 파일을 세그먼트 1개로 잘라두면 요약 segment_count 도 1, start_source=plan_dir.
    execute-plan --print-plan reuses the saved planned_segments.json (not a re-plan):
    trimming it to one segment yields segment_count==1 and start_source==plan_dir."""
    plan_dir = tmp_path / "preview"
    out_dir = tmp_path / "execute"
    rc = cli.main(_coverage_preview_args(plan_dir))
    assert rc == 0
    saved_segments = json.loads((plan_dir / "planned_segments.json").read_text())
    first_only = [saved_segments[0]]
    (plan_dir / "planned_segments.json").write_text(json.dumps(first_only, indent=2) + "\n")

    rc = cli.main(["execute-plan", "--plan-dir", str(plan_dir), "--print-plan", "--out-dir", str(out_dir)])

    assert rc == 0
    summary = json.loads((out_dir / "summary.json").read_text())
    assert summary["start_source"] == "plan_dir"
    assert summary["segment_count"] == 1
    assert summary["field_config"]["path_shape"] == "coverage_lawnmower"


def test_diagonal_rectangle_serpentine_available_but_not_default(capsys: pytest.CaptureFixture[str]) -> None:
    """기본은 coverage_lawnmower 이고, diagonal_rectangle_serpentine 은 옵트인시에만 선택됨.

    명시 요청 시 resolve_plan 이 A-B 대각 프레임 안내 문구를 출력하는지 확인.
    Default stays coverage_lawnmower; diagonal_rectangle_serpentine is opt-in and,
    when requested, resolve_plan prints its A-B diagonal-frame notice."""
    default_plan = preview.build_preview(
        start_lat=35.5709,
        start_lon=129.1871,
        goal_mode="relative_enu",
        goal_east_m=1.2,
        goal_north_m=1.2,
        workspace_width_m=1.2,
        step_spacing_m=0.4,
    )
    assert default_plan["path_shape"] == "coverage_lawnmower"

    parser = cli.build_parser()
    args = parser.parse_args(
        [
            "preview",
            "--start-lat",
            "35.5709000",
            "--start-lon",
            "129.1871000",
            "--goal-mode",
            "relative_enu",
            "--goal-east-m",
            "1.2",
            "--goal-north-m",
            "1.2",
            "--workspace-width-m",
            "1.2",
            "--step-spacing-m",
            "0.4",
            "--path-shape",
            "diagonal_rectangle_serpentine",
        ]
    )
    cli.resolve_plan(args, cli.resolve_calibration(args))
    out = capsys.readouterr().out
    assert "diagonal_rectangle_serpentine follows the A-B diagonal frame" in out
