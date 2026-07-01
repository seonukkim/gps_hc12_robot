"""사이드 툴(측면 장착 도구) 커버리지 플래너 계약 회귀 테스트.

목적/역할:
    `gps_coverage_core.side_tool_planner` 와 그 위에 얹힌 `tools/*` 미리보기
    CLI(`side_tool_path_preview`, `preview_side_tool_path`,
    `preview_side_tool_waypoints`, `simulate_side_tool_tracking`)가 지키기로 한
    **동작 계약(contract)** 을 잠근다(lock in). 여기서 검증하는 핵심 계약은:
      1) 기본값(rover18 섀시 + 가정 툴 치수)이 회귀 없이 유지된다.
      2) A/B 의 의미(semantic)가 `workspace_mode` 마다 정확하다 — 코너 대각선,
         중심선+폭, 축+폭, 그리고 툴 우선(tool-first) serpentine.
      3) "툴 우선" 원칙: 툴 중심 경로가 1차(primary), 섀시 경로는 그로부터 역산된
         지지(derived support) 경로다.
      4) 경계(boundary)·스윕 볼륨(swept-volume)·오염(contamination) 검사가
         위반을 정확히 잡아내고, 실현 불가한 기하는 명확한 ValueError 로 실패한다.
      5) **무모터 불변식**: 이 서브시스템의 어떤 경로/CLI도 모터 명령을 만들지
         않는다(모든 pose·CSV 행에서 `motor_command_generated` 는 False).

    Locks in the behavioral contract of the side-mounted-tool coverage planner
    (`gps_coverage_core.side_tool_planner`) and its preview CLIs. Covers default
    dimensions, per-mode A/B semantics, the "tool-first" primary/derived-path
    rule, boundary/swept-volume/contamination checks, clear failures for
    infeasible geometry, and the invariant that no motor command is ever emitted.

시스템 내 위치:
    - 테스트 대상(코어): `gps_coverage_core.side_tool_planner` 의 공개 함수와
      준(準)공개 밑줄 헬퍼(`_workspace_info`)를 직접 호출한다. 즉 이 파일은 그
      준공개 심볼들도 "계약"으로 취급한다(시그니처를 바꾸면 여기서 깨진다).
    - 테스트 대상(CLI): `tools` 패키지의 미리보기 진입점들을 `main([...])` 인자
      배열로 구동하고, 산출물(CSV/summary.md/PNG)을 파일로 검증한다.

핵심 개념·불변식(invariant):
    - **툴 우선(tool-first)**: `planner_primary_path == "tool_center_path"`,
      `chassis_path_role == "derived_support_path"`. 섀시 포즈는 툴 오프셋을 역산해
      얻는다(test_derived_chassis_pose_* 가 그 공식을 직접 재현해 검증).
    - **로컬↔월드**: 계획은 작업 사각형 로컬 프레임(x=A→B, y=폭)에서 이뤄진다.
    - **무모터**: `motor_command_generated` 는 어디서도 True 가 되지 않으며, pose
      키에 "cmd" 문자열조차 없어야 한다(test_side_tool_path_never_generates_*).

리팩토링 노트:
    - 다수 테스트가 `_workspace_config()` 팩토리(아래)의 정확한 기본값에 의존한다.
      기본값을 바꾸면 커버리지/여백/경계 관련 단언이 연쇄적으로 흔들리므로 주의.
    - CLI 테스트는 산출물 **파일명과 summary.md 안의 정확한 문자열**을 검사한다.
      이는 하류(사람이 읽는 리포트)와의 계약이므로 렌더러 출력 형식 변경 시 동기화 필요.

Pytest module: contract/regression tests. Treats even the planner's underscore
helpers as part of the locked contract, and drives the preview CLIs end-to-end
by asserting on their emitted files (CSV / summary.md / PNG) and exact strings.
"""

import csv
import math

import pytest

from gps_coverage_core.side_tool_planner import (
    SideToolPlanConfig,
    chassis_polygon,
    contamination_analysis,
    footprint_sample,
    generate_side_tool_path,
    generate_tool_serpentine_preview,
    geometry_samples_for_path,
    summarize_side_tool_path,
    strategy_diagnostics,
    swept_volume_summary,
    tool_length_m,
    tool_polygon,
    workspace_polygon,
    _workspace_info,
)
from tools import (
    preview_side_tool_path,
    preview_side_tool_waypoints,
    side_tool_path_preview,
    simulate_side_tool_tracking,
)


# ── 테스트 헬퍼 & 픽스처 / Test helpers & fixtures ──


def _lane_starts(path):
    """경로에서 `lane_start` 세그먼트 pose 만 추출 / Extract lane-start poses."""
    return [pose for pose in path if pose["segment_type"] == "lane_start"]


def _lane_directions(path):
    """각 레인 시작의 진행 방향(motion_direction) 리스트 / Per-lane motion directions."""
    return [pose["motion_direction"] for pose in _lane_starts(path)]


def _workspace_config(tool_side="left", **overrides):
    """다수 테스트가 공유하는 표준 `diagonal_ab` 설정 팩토리.

    8.0 x 1.2 m 대각선 A/B 사각형에 rover18 기본 치수를 채운 `SideToolPlanConfig`
    를 만든다. `**overrides` 로 개별 필드만 덮어써 테스트별 변형을 만든다. 주의:
    아래 기본값(오프셋/폭/여백 등)은 커버리지·경계 단언들이 의존하는 계약값이다.

    Standard `diagonal_ab` config factory shared by many tests; override any field
    via kwargs. The baseline dimensions are load-bearing for coverage/boundary
    assertions.
    """
    values = dict(
        workspace_mode="diagonal_ab",
        a_x_m=0.0,
        a_y_m=0.0,
        b_x_m=8.0,
        b_y_m=1.2,
        tool_side=tool_side,
        tool_lateral_offset_m=0.24,
        tool_width_m=0.30,
        lane_spacing_m=0.25,
        row_count="auto",
        robot_width_m=0.18,
        robot_length_m=0.18,
        robot_radius_m=0.14,
        first_lane_direction="forward",
        transition_style="side_step_reverse_90",
        boundary_mode="strict",
        chassis_boundary_mode="clean_surface_strict",
        auto_inset_endpoints=True,
        boundary_margin_m=0.03,
        auto_orient_tool_inside=True,
        fail_on_boundary_violation=True,
        contamination_mode="off",
        fail_on_contamination_violation=False,
        require_start_at_A=False,
        require_end_at_B=False,
    )
    values.update(overrides)
    return SideToolPlanConfig(**values)


def _polygon_center(poly):
    """다각형 꼭짓점 평균(무게중심 근사) / Centroid-ish mean of polygon vertices."""
    return (
        sum(point[0] for point in poly) / len(poly),
        sum(point[1] for point in poly) / len(poly),
    )


# ── 기본값 & 워크스페이스 모드 의미 / Defaults & workspace-mode semantics ──


def test_rover18_and_assumed_tool_defaults() -> None:
    """생성자 기본값이 rover18 섀시 + 가정 툴 치수로 회귀 없이 유지되는지 확인.

    Guards the default `SideToolPlanConfig` dimensions (rover18 chassis, assumed
    tool geometry, sampling/validation modes) against silent regression.
    """
    config = SideToolPlanConfig(a_x_m=0.0, a_y_m=0.0, b_x_m=8.0, b_y_m=1.2)

    assert config.workspace_mode == "tool_serpentine_ab"
    assert config.robot_width_m == pytest.approx(0.18)
    assert config.robot_length_m == pytest.approx(0.18)
    assert config.robot_radius_m == pytest.approx(0.14)
    assert config.tool_width_m == pytest.approx(0.30)
    assert config.tool_lateral_offset_m == pytest.approx(0.24)
    assert config.tool_side == "left"
    assert config.tool_active_on_sweep_tracks is True
    assert config.tool_active_on_connectors is False
    assert config.lane_spacing_m == pytest.approx(0.25)
    assert config.boundary_margin_m == pytest.approx(0.03)
    assert config.tool_length_m == pytest.approx(0.18)
    assert config.rotation_sample_deg == pytest.approx(5.0)
    assert config.translation_sample_m == pytest.approx(0.05)
    assert config.swept_volume_validation == "strict"
    assert config.contamination_mode == "off"


def test_tool_serpentine_ab_preserves_top_left_A_and_bottom_right_B() -> None:
    """`tool_serpentine_ab` 에서 A=좌상단·B=우하단 규약이 툴 경로 끝점에 그대로 보존됨.

    Verifies the A=top-left / B=bottom-right convention is preserved: the tool
    path starts exactly at A and ends exactly at B for this mode.
    """
    config = SideToolPlanConfig(
        workspace_mode="tool_serpentine_ab",
        a_x_m=0.0,
        a_y_m=1.2,
        b_x_m=8.0,
        b_y_m=0.0,
        step_spacing_m=0.25,
    )
    preview = generate_tool_serpentine_preview(config)
    summary = preview["summary"]
    tool_segments = preview["tool_segments"]

    assert workspace_polygon(config) == pytest.approx([(0.0, 0.0), (8.0, 0.0), (8.0, 1.2), (0.0, 1.2)])
    assert summary["workspace_mode"] == "tool_serpentine_ab"
    assert summary["A_corner_role"] == "top_left"
    assert summary["B_corner_role"] == "bottom_right"
    assert tool_segments[0]["tool_start_x_m"] == pytest.approx(0.0)
    assert tool_segments[0]["tool_start_y_m"] == pytest.approx(1.2)
    assert tool_segments[-1]["tool_end_x_m"] == pytest.approx(8.0)
    assert tool_segments[-1]["tool_end_y_m"] == pytest.approx(0.0)
    assert summary["tool_path_starts_at_A"] is True
    assert summary["tool_path_ends_at_B"] is True


def test_tool_serpentine_summary_tool_length_falls_back_when_unset() -> None:
    """tool_length_m 미설정 시 summary가 tool→robot→0.18m 폴백을 쓰는지 검증(회귀).

    과거 이 경로는 정의되지 않은 `_robot_length()`를 호출해 NameError를 냈다.
    이제 기존 `tool_length_m()` 헬퍼를 재사용하므로 tool_length_m=None 이면
    robot_length_m(기본 0.18m)로 대체되어야 한다. / Regression: with
    tool_length_m unset, the preview summary must fall back to the robot length
    (0.18 m default) instead of raising the old undefined-name error.
    """
    config = SideToolPlanConfig(
        workspace_mode="tool_serpentine_ab",
        a_x_m=0.0,
        a_y_m=1.2,
        b_x_m=8.0,
        b_y_m=0.0,
        step_spacing_m=0.25,
        tool_length_m=None,
    )
    preview = generate_tool_serpentine_preview(config)
    assert preview["summary"]["tool_length_m"] == pytest.approx(0.18)
    assert preview["summary"]["tool_length_m"] == pytest.approx(tool_length_m(config))


def test_tool_serpentine_step_spacing_derives_tracks_and_connectors() -> None:
    """step_spacing 으로부터 트랙 수(홀수)·커넥터 수(트랙-1)·연속성이 파생되는지 검증.

    간격 조정으로 실제 spacing 이 0.2 m 로 재계산되고, 커넥터는 비활성(무커버리지),
    각 세그먼트 끝점이 다음 시작점과 연속됨을 확인한다.

    Checks that step spacing derives an odd track count, connector_count ==
    track_count-1, non-painting connectors, and a continuous (end==next-start)
    tool path.
    """
    config = SideToolPlanConfig(
        workspace_mode="tool_serpentine_ab",
        a_x_m=0.0,
        a_y_m=1.2,
        b_x_m=8.0,
        b_y_m=0.0,
        step_spacing_m=0.25,
        adjust_spacing_to_end_at_B=True,
        force_end_at_B=True,
    )
    preview = generate_tool_serpentine_preview(config)
    summary = preview["summary"]
    tool_segments = preview["tool_segments"]
    tracks = [segment for segment in tool_segments if segment["tool_segment_type"] == "tool_sweep_track"]
    connectors = [segment for segment in tool_segments if segment["tool_segment_type"] == "tool_spacing_connector"]

    assert summary["derived_track_count"] == 7
    assert summary["track_count_parity"] == "odd"
    assert summary["tool_track_count"] == 7
    assert summary["tool_connector_count"] == 6
    assert summary["tool_connector_count_equals_track_count_minus_one"] is True
    assert summary["tool_path_continuous"] is True
    assert summary["actual_spacing_values_m"] == "0.200000,0.200000,0.200000,0.200000,0.200000,0.200000"
    assert [track["tool_start_y_m"] for track in tracks] == pytest.approx([1.2, 1.0, 0.8, 0.6, 0.4, 0.2, 0.0])
    assert len(connectors) == len(tracks) - 1
    assert {connector["tool_active"] for connector in connectors} == {False}
    assert {connector["coverage_contributes"] for connector in connectors} == {False}
    for index in range(len(tool_segments) - 1):
        assert tool_segments[index]["tool_end_x_m"] == pytest.approx(tool_segments[index + 1]["tool_start_x_m"])
        assert tool_segments[index]["tool_end_y_m"] == pytest.approx(tool_segments[index + 1]["tool_start_y_m"])


def test_tool_serpentine_chassis_and_primitives_are_derived_from_tool_path() -> None:
    """툴 우선 원칙 검증: 섀시 경로·프리미티브가 모두 툴 경로에서 파생됨.

    primary=tool_center_path, chassis=derived_support_path 이고, 프리미티브는
    허용 집합(move/rotate) 안에서만 나오며 모터 명령을 만들지 않음을 확인한다.

    Asserts the tool-first rule for serpentine mode: chassis segments and the
    primitive sequence are all derived from the tool path, primitives stay within
    the allowed move/rotate set, and no motor command is generated.
    """
    config = SideToolPlanConfig(
        workspace_mode="tool_serpentine_ab",
        a_x_m=0.0,
        a_y_m=1.2,
        b_x_m=8.0,
        b_y_m=0.0,
        step_spacing_m=0.25,
    )
    preview = generate_tool_serpentine_preview(config)
    chassis_segments = preview["chassis_segments"]
    primitive_rows = preview["primitive_rows"]
    summary = preview["summary"]

    assert summary["planner_primary_path"] == "tool_center_path"
    assert summary["chassis_path_role"] == "derived_support_path"
    assert summary["chassis_path_derived_from_tool"] is True
    assert summary["primitive_sequence_valid"] is True
    assert {row["primitive_type"] for row in primitive_rows} <= {
        "move_forward",
        "move_backward",
        "rotate_left",
        "rotate_right",
    }
    assert all(segment["chassis_path_derived_from_tool"] is True for segment in chassis_segments)
    assert {row["motor_command_generated"] for row in primitive_rows} == {False}


def test_ab_centerline_width_treats_ab_as_rover_center_start_end() -> None:
    """`ab_centerline_width` 에서 A/B 는 로버 "중심선" 시작/끝이며 물리 프리뷰는 불가.

    경로가 A→B 중심선을 따르되, 이 모드는 물리 프리뷰 대상이 아니어서
    `route_feasible_for_physical_preview` 는 False, 사유에
    WORKSPACE_MODE_NOT_AB_DIAGONAL_CENTER 가 담긴다.

    In centerline mode A/B are the rover-center start/end; the route reaches B but
    is flagged not physical-preview-feasible with the documented reason string.
    """
    config = SideToolPlanConfig(
        workspace_mode="ab_centerline_width",
        a_x_m=0.0,
        a_y_m=0.0,
        b_x_m=8.0,
        b_y_m=0.0,
        surface_side="left",
        workspace_width_m=1.2,
        tool_side="left",
        contamination_mode="strict",
        fail_on_contamination_violation=True,
    )

    path = generate_side_tool_path(config)
    summary = summarize_side_tool_path(config, path)

    assert path[0]["segment_type"] == "route_start"
    assert path[0]["x_m"] == pytest.approx(0.0)
    assert path[0]["y_m"] == pytest.approx(0.0)
    assert path[-1]["segment_type"] == "route_end"
    assert path[-1]["x_m"] == pytest.approx(8.0)
    assert path[-1]["y_m"] == pytest.approx(0.0)
    assert summary["workspace_mode"] == "ab_centerline_width"
    assert summary["A_B_are_rover_center_start_end"] is True
    assert summary["start_reaches_A"] is True
    assert summary["end_reaches_B"] is True
    assert summary["route_reaches_B"] is True
    assert summary["route_feasible_for_physical_preview"] is False
    assert "WORKSPACE_MODE_NOT_AB_DIAGONAL_CENTER" in summary["infeasible_reason"]
    assert summary["differential_drive_feasible"] is True
    assert all(pose["motor_command_generated"] is False for pose in path)


def test_ab_diagonal_center_treats_ab_as_start_end_and_rectangle_diagonal() -> None:
    """`ab_diagonal_center`: A/B 가 로버 중심 시작/끝이자 사각형 대각선을 동시에 정의.

    작업 사각형이 A,B 대각선으로 구성되고 경로가 A→B 를 실제로 도달함을 확인.

    In diagonal-center mode A/B are simultaneously the rover-center start/end and
    the rectangle diagonal; the workspace polygon and the reached endpoints agree.
    """
    config = SideToolPlanConfig(
        workspace_mode="ab_diagonal_center",
        a_x_m=0.0,
        a_y_m=0.0,
        b_x_m=8.0,
        b_y_m=1.2,
        tool_side="left",
        contamination_mode="off",
        fail_on_contamination_violation=False,
    )

    path = generate_side_tool_path(config)
    summary = summarize_side_tool_path(config, path)

    assert workspace_polygon(config) == pytest.approx(
        [(0.0, 0.0), (8.0, 0.0), (8.0, 1.2), (0.0, 1.2)]
    )
    assert path[0]["segment_type"] == "route_start"
    assert path[0]["x_m"] == pytest.approx(0.0)
    assert path[0]["y_m"] == pytest.approx(0.0)
    assert path[-1]["segment_type"] == "route_end"
    assert path[-1]["x_m"] == pytest.approx(8.0)
    assert path[-1]["y_m"] == pytest.approx(1.2)
    assert summary["workspace_mode"] == "ab_diagonal_center"
    assert summary["A_B_are_rover_center_start_end"] is True
    assert summary["A_B_are_rectangle_diagonal"] is True
    assert summary["start_reaches_A"] is True
    assert summary["end_reaches_B"] is True
    assert summary["route_reaches_B"] is True


def test_single_lane_is_not_physical_preview_feasible_when_multiple_lanes_required() -> None:
    """다중 레인이 필요한데 1레인만 선택되면 물리 프리뷰 불가로 보고되는지 검증.

    `boustrophedon_required` + `forbid_single_lane_success` 하에서 선택 레인 수가
    필요 최소치 미만이면 boustrophedon 무효·프리뷰 불가·사유
    SELECTED_LANE_COUNT_BELOW_REQUIRED 를 확인한다.

    When coverage needs >1 lane but only 1 is selected, the plan is reported as
    not physical-preview-feasible with the below-required-lane-count reason.
    """
    config = SideToolPlanConfig(
        workspace_mode="ab_diagonal_center",
        a_x_m=0.0,
        a_y_m=0.0,
        b_x_m=8.0,
        b_y_m=1.2,
        tool_side="left",
        row_count=1,
        coverage_pattern="boustrophedon_required",
        forbid_single_lane_success=True,
        contamination_mode="off",
        fail_on_contamination_violation=False,
    )

    path = generate_side_tool_path(config)
    summary = summarize_side_tool_path(config, path)

    assert summary["required_min_lane_count"] > 1
    assert summary["selected_lane_count"] == 1
    assert summary["selected_lane_count_ok"] is False
    assert summary["boustrophedon_pattern_valid"] is False
    assert summary["route_feasible_for_physical_preview"] is False
    assert "SELECTED_LANE_COUNT_BELOW_REQUIRED" in summary["infeasible_reason"]


# ── 툴 우선 경로 & 파생 섀시 / Tool-first path & derived chassis ──


def test_ab_diagonal_center_route_expands_to_four_physical_primitives() -> None:
    """대각선 경로가 4개 물리 프리미티브(move/rotate)로 전개되고 첫 청소 헤딩이 정렬됨.

    프리미티브 시퀀스가 유효하고 허용 집합에 속하며, 끝점이 A/B 와 일치하고 첫
    레인 헤딩이 0/180도로 정렬(정면 청소)됨을 확인. 모터 명령은 없음.

    The diagonal route expands into four valid move/rotate primitives, endpoints
    match A/B, and the first cleaning lane heading is axis-aligned (0/180 deg).
    """
    config = SideToolPlanConfig(
        workspace_mode="ab_diagonal_center",
        a_x_m=0.0,
        a_y_m=0.0,
        b_x_m=8.0,
        b_y_m=1.2,
        tool_side="right",
        transition_style="auto_internal",
        contamination_mode="off",
        fail_on_contamination_violation=False,
    )

    path = generate_side_tool_path(config)
    summary = summarize_side_tool_path(config, path)
    allowed = {"move_forward", "move_backward", "rotate_left", "rotate_right"}
    first_lane = _lane_starts(path)[0]

    assert summary["primitive_sequence_valid"] is True
    assert {pose["movement_primitive_type"] for pose in path} <= allowed
    assert path[0]["x_m"] == pytest.approx(0.0)
    assert path[0]["y_m"] == pytest.approx(0.0)
    assert path[-1]["x_m"] == pytest.approx(8.0)
    assert path[-1]["y_m"] == pytest.approx(1.2)
    assert float(first_lane["heading_deg"]) in {0.0, 180.0}
    assert first_lane["first_cleaning_heading_aligned"] is True
    assert all(pose["motor_command_generated"] is False for pose in path)


def test_tool_center_path_is_primary_and_continuous() -> None:
    """툴 중심 경로가 1차이며 연속(각 세그먼트 끝=다음 시작)이고 섀시는 파생됨.

    primary=tool_center_path, 커넥터 수=트랙-1, 모든 pose 가 tool_path_primary=True,
    그리고 툴 끝점↔다음 시작점이 연속임을 검증.

    Confirms the tool-center path is primary and continuous (end==next-start for
    every segment) with connector_count == track_count-1 and chassis derived.
    """
    config = SideToolPlanConfig(
        workspace_mode="ab_diagonal_center",
        a_x_m=0.0,
        a_y_m=0.0,
        b_x_m=8.0,
        b_y_m=1.2,
        tool_side="right",
        transition_style="auto_internal",
        contamination_mode="off",
        fail_on_contamination_violation=False,
    )
    path = generate_side_tool_path(config)
    summary = summarize_side_tool_path(config, path)

    assert summary["planner_primary_path"] == "tool_center_path"
    assert summary["chassis_path_role"] == "derived_support_path"
    assert summary["tool_path_continuous"] is True
    assert summary["tool_connector_count"] == summary["tool_track_count"] - 1
    assert summary["chassis_path_derived_from_tool"] is True
    assert {pose["tool_path_primary"] for pose in path} == {True}
    assert all(
        float(path[index]["tool_end_x_m"]) == pytest.approx(float(path[index + 1]["tool_start_x_m"]))
        and float(path[index]["tool_end_y_m"]) == pytest.approx(float(path[index + 1]["tool_start_y_m"]))
        for index in range(len(path) - 1)
    )


def test_tool_connector_angles_and_distances_are_from_tool_waypoints() -> None:
    """지그재그 커넥터의 거리/각도가 실제 툴 웨이포인트 좌표에서 유도됨을 검증.

    각 전이 커넥터에 대해 hypot(dx,dy)==보고 거리, atan2(dy,dx)==보고 각도임을 확인
    (기하 일관성 계약).

    The reported connector distance/angle exactly equal hypot/atan2 of the tool
    start→end delta — geometry-consistency contract.
    """
    config = SideToolPlanConfig(
        workspace_mode="ab_diagonal_center",
        a_x_m=0.0,
        a_y_m=0.0,
        b_x_m=8.0,
        b_y_m=1.2,
        tool_side="right",
        transition_style="auto_internal",
        contamination_mode="off",
        fail_on_contamination_violation=False,
    )
    path = generate_side_tool_path(config)
    connectors = [
        pose
        for pose in path
        if pose["tool_segment_type"] == "tool_zigzag_connector"
        and str(pose["segment_id"]).startswith("transition_")
        and float(pose["tool_connector_distance_m"]) > 1e-9
    ]

    assert connectors
    for pose in connectors:
        dx = float(pose["tool_end_x_m"]) - float(pose["tool_start_x_m"])
        dy = float(pose["tool_end_y_m"]) - float(pose["tool_start_y_m"])
        assert float(pose["tool_connector_distance_m"]) == pytest.approx(math.hypot(dx, dy))
        assert float(pose["tool_connector_angle_deg"]) == pytest.approx(math.degrees(math.atan2(dy, dx)))


def test_derived_chassis_pose_places_tool_on_intended_tool_path() -> None:
    """파생 섀시 포즈가 툴을 의도한 툴 경로 위에 정확히 놓는지, 역산 공식으로 재현 검증.

    각 pose 에서 (heading+90도) 법선 방향으로 lateral_offset 만큼 이동한 섀시 좌표가
    보고된 x/y 와 일치함을 확인 — 툴 오프셋 역산의 정확성 계약.

    Reproduces the chassis-from-tool offset formula and asserts the derived
    chassis (x,y) matches the reported pose, i.e. the tool lands on its intended
    path.
    """
    config = SideToolPlanConfig(
        workspace_mode="ab_diagonal_center",
        a_x_m=0.0,
        a_y_m=0.0,
        b_x_m=8.0,
        b_y_m=1.2,
        tool_side="right",
        transition_style="auto_internal",
        contamination_mode="off",
        fail_on_contamination_violation=False,
    )
    path = generate_side_tool_path(config)
    side_sign = -1.0
    for pose in path:
        heading = math.radians(float(pose["heading_deg"]) + 90.0)
        normal_x = math.cos(heading)
        normal_y = math.sin(heading)
        derived_x = float(pose["tool_start_x_m"]) - side_sign * config.tool_lateral_offset_m * normal_x
        derived_y = float(pose["tool_start_y_m"]) - side_sign * config.tool_lateral_offset_m * normal_y
        assert derived_x == pytest.approx(float(pose["x_m"]))
        assert derived_y == pytest.approx(float(pose["y_m"]))
        assert pose["chassis_path_derived_from_tool"] is True


def test_ab_centerline_shifted_start_is_not_physical_preview_feasible() -> None:
    """A 에서 시작·B 에서 끝을 강제해도 diagonal_ab 로는 못 맞추면 프리뷰 불가로 처리.

    `require_start_at_A`/`require_end_at_B` + best-effort 상황에서 시작이 A 에
    닿지 못하고 B 에 도달 못하면 물리 프리뷰 불가가 됨을 확인.

    With required start-at-A/end-at-B under best-effort, diagonal_ab cannot honor
    both, so start_reaches_A / route_reaches_B are False and the route is not
    physical-preview-feasible.
    """
    config = SideToolPlanConfig(
        workspace_mode="diagonal_ab",
        a_x_m=0.0,
        a_y_m=0.0,
        b_x_m=8.0,
        b_y_m=1.2,
        tool_side="left",
        require_start_at_A=True,
        require_end_at_B=True,
        allow_best_effort=True,
        contamination_mode="off",
        fail_on_contamination_violation=False,
    )

    path = generate_side_tool_path(config)
    summary = summarize_side_tool_path(config, path)

    assert summary["start_reaches_A"] is False
    assert summary["route_reaches_B"] is False
    assert summary["route_feasible_for_physical_preview"] is False


# ── 기하 프리미티브(다각형·오프셋) / Geometry primitives (polygons, offsets) ──


def test_chassis_rectangle_polygon_dimensions_and_orientation() -> None:
    """섀시 사각형 다각형의 치수와 회전(0도/90도) 후 바운딩 박스가 정사각 유지되는지.

    18x18 cm 정사각 섀시라 헤딩 0도든 90도든 x/y 폭이 0.18 m 로 동일함을 확인.

    Chassis rectangle keeps a 0.18 m bounding box in both x and y at heading 0 and
    90 deg (square chassis).
    """
    config = _workspace_config("left")
    poly = chassis_polygon(config, x_m=1.0, y_m=0.6, heading_deg=0.0)

    xs = [point[0] for point in poly]
    ys = [point[1] for point in poly]
    assert max(xs) - min(xs) == pytest.approx(0.18)
    assert max(ys) - min(ys) == pytest.approx(0.18)

    rotated = chassis_polygon(config, x_m=1.0, y_m=0.6, heading_deg=90.0)
    rxs = [point[0] for point in rotated]
    rys = [point[1] for point in rotated]
    assert max(rxs) - min(rxs) == pytest.approx(0.18)
    assert max(rys) - min(rys) == pytest.approx(0.18)


def test_side_mounted_tool_polygon_offsets_left_and_right() -> None:
    """툴 다각형이 tool_side 에 따라 섀시 중심에서 좌/우로 정확히 오프셋됨.

    heading 0도에서 좌측 툴 중심은 +y(0.84), 우측 툴 중심은 -y(0.36)로,
    lateral_offset 0.24 m 만큼 대칭 오프셋됨을 확인.

    Left tool sits at +y and right tool at -y of the chassis center by the lateral
    offset (0.84 vs 0.36 at heading 0).
    """
    left_config = _workspace_config("left")
    right_config = _workspace_config("right")
    left_center = _polygon_center(
        tool_polygon(left_config, chassis_x_m=1.0, chassis_y_m=0.6, heading_deg=0.0)
    )
    right_center = _polygon_center(
        tool_polygon(right_config, chassis_x_m=1.0, chassis_y_m=0.6, heading_deg=0.0)
    )

    assert left_center == pytest.approx((1.0, 0.84))
    assert right_center == pytest.approx((1.0, 0.36))


def test_diagonal_ab_workspace_rectangle_uses_opposite_corners() -> None:
    """`diagonal_ab` 작업 사각형이 A,B 를 대각(opposite) 코너로 삼는지 확인.

    A=(0,0), B=(8,1.2)가 사각형의 마주보는 두 꼭짓점이 됨을 검증.

    In diagonal_ab, A and B are opposite corners of the axis-aligned rectangle.
    """
    polygon = workspace_polygon(_workspace_config("left"))

    assert polygon == pytest.approx([(0.0, 0.0), (8.0, 0.0), (8.0, 1.2), (0.0, 1.2)])
    assert (0.0, 0.0) in polygon
    assert (8.0, 1.2) in polygon


def test_diagonal_ab_zero_width_fails_clearly() -> None:
    """A,B 가 같은 y 라 폭 0 인 대각선 사각형은 명확한 ValueError 로 실패해야 함.

    Zero-width diagonal (A.y==B.y) raises DIAGONAL_AB_WIDTH_ZERO rather than
    producing a degenerate path.
    """
    config = SideToolPlanConfig(
        workspace_mode="diagonal_ab",
        a_x_m=0.0,
        a_y_m=0.0,
        b_x_m=8.0,
        b_y_m=0.0,
    )

    with pytest.raises(ValueError, match="DIAGONAL_AB_WIDTH_ZERO"):
        generate_side_tool_path(config)


def test_axis_width_mode_remains_available() -> None:
    """레거시 `axis_width` 모드(축 + 폭)가 계속 지원되어 같은 사각형을 만드는지 확인.

    The legacy axis_width mode (A→B axis plus a side width) still yields the
    expected rectangle.
    """
    config = SideToolPlanConfig(
        workspace_mode="axis_width",
        a_x_m=0.0,
        a_y_m=0.0,
        b_x_m=8.0,
        b_y_m=0.0,
        workspace_side="left",
        workspace_width_m=1.2,
    )

    assert workspace_polygon(config) == pytest.approx([(0.0, 0.0), (8.0, 0.0), (8.0, 1.2), (0.0, 1.2)])


# ── 경계 준수 & 헤딩/방향 / Boundary containment & heading vs motion ──


def test_left_tool_workspace_left_stays_inside_ab_rectangle() -> None:
    """좌측 툴이 A/B 사각형 안에 머무르고, 완전 커버리지가 아닌(여백 존재) 상태 확인.

    경계 위반 0, 모든 pose 의 섀시/툴이 내부, 툴 가장자리 y 가 여백 범위 안,
    그리고 커버리지 비율<1(측면 여백 때문)임을 검증.

    Left-mounted tool stays fully inside the rectangle (zero violations) while
    leaving an uncovered side margin, so coverage_ratio < 1.
    """
    config = _workspace_config("left")
    path = generate_side_tool_path(config)
    summary = summarize_side_tool_path(config, path)

    assert summary["boundary_violation_count"] == 0
    assert all(pose["within_boundary"] is True for pose in path)
    assert all(pose["chassis_within_boundary"] is True for pose in path)
    assert all(pose["tool_within_boundary"] is True for pose in path)
    assert min(float(pose["tool_edge_min_y_m"]) for pose in path) >= 0.03
    assert max(float(pose["tool_edge_max_y_m"]) for pose in path) <= 1.17
    assert summary["uncovered_margin_low_m"] > 0.0
    assert summary["coverage_ratio"] < 1.0


def test_right_tool_workspace_left_flips_heading_and_uses_reverse_motion() -> None:
    """우측 툴로 좌측을 청소하려면 헤딩을 뒤집고 후진 주행을 섞어 쓰는지 확인.

    레인 헤딩이 0/180도로 뒤집히고 툴이 섀시 위/아래로 번갈아 배치되며, 첫 레인은
    전진(travel 0도)으로 시작함을 검증. 여전히 경계 내부.

    A right tool covering the left workspace flips lane headings (0/180) and uses
    reverse motion; the tool alternates above/below the chassis and stays inside.
    """
    path = generate_side_tool_path(_workspace_config("right"))
    starts = _lane_starts(path)

    assert {float(pose["heading_deg"]) for pose in starts} == {0.0, 180.0}
    assert {pose["tool_world_side"] for pose in starts} == {"above_chassis", "below_chassis"}
    assert starts[0]["motion_direction"] == "forward"
    assert starts[0]["travel_direction_deg"] == pytest.approx(0.0)
    assert all(pose["tool_within_boundary"] is True for pose in path)


def test_left_and_right_tool_cases_stay_inside() -> None:
    """좌·우 툴 양쪽 모두 diagonal_ab 사각형 안에 머물고 A/B 가 대각 코너 의미를 유지.

    두 tool_side 에 대해 경계 위반 0, a_b_semantic=opposite_corners, 모터 미생성 확인.

    Both left and right tool sides stay inside the diagonal_ab rectangle with
    zero boundary violations and opposite-corners semantics.
    """
    for tool_side in ("left", "right"):
        config = _workspace_config(tool_side)
        path = generate_side_tool_path(config)
        summary = summarize_side_tool_path(config, path)

        assert summary["workspace_mode"] == "diagonal_ab"
        assert summary["a_b_semantic"] == "opposite_corners"
        assert summary["boundary_violation_count"] == 0
        assert {pose["workspace_mode"] for pose in path} == {"diagonal_ab"}
        assert all(pose["motor_command_generated"] is False for pose in path)


def test_chassis_radius_and_tool_edges_are_checked() -> None:
    """섀시 반경과 툴 가장자리가 모두 사각형(0..8, 0..1.2) 내부로 검사되는지 확인.

    모든 pose 에서 섀시/툴 내부 플래그가 True 이고 중심·툴 가장자리 좌표가 경계
    안에 있음을 검증.

    Every pose reports chassis and tool edges inside the [0,8]x[0,1.2] rectangle.
    """
    path = generate_side_tool_path(_workspace_config("left"))

    for pose in path:
        assert pose["chassis_within_boundary"] is True
        assert pose["tool_within_boundary"] is True
        assert 0.0 <= float(pose["x_m"]) <= 8.0
        assert 0.0 <= float(pose["y_m"]) <= 1.2
        assert 0.0 <= float(pose["tool_edge_min_y_m"]) <= 1.2
        assert 0.0 <= float(pose["tool_edge_max_y_m"]) <= 1.2


# ── 발자국 표본 & 스윕 볼륨 검증 / Footprint samples & swept-volume checks ──


def test_combined_footprint_sample_passes_when_inset() -> None:
    """사각형 내부 충분히 안쪽 지점에서 결합 발자국(섀시+툴) 표본이 통과(OK)하는지.

    안쪽(1.0, 0.6)·헤딩 90도 표본은 workspace/chassis/tool 모두 내부이고
    violation_reason=="OK" 임을 확인.

    A footprint sample well inside the rectangle passes: chassis and tool both
    inside, reason "OK".
    """
    config = _workspace_config("left")
    info = _workspace_info(config)
    sample = footprint_sample(
        config,
        info,
        sample_index=0,
        segment_id="sample",
        sample_type="rotation",
        x_m=1.0,
        y_m=0.6,
        heading_deg=90.0,
    )

    assert sample.inside_workspace is True
    assert sample.chassis_inside_workspace is True
    assert sample.tool_inside_workspace is True
    assert sample.violation_reason == "OK"


def test_rotation_swept_sampling_detects_boundary_violation_near_edge() -> None:
    """가장자리 근처 회전 표본이 결합 스윕 볼륨의 경계 이탈을 잡아내는지 확인.

    코너 근처(0.1,0.1) 회전 표본은 내부가 아니고 사유에
    ROTATION_COMBINED_SWEEP_EXITS_WORKSPACE 를 담아야 함(회전 중 쓸림 검출).

    A rotation sample near the corner is detected as exiting the workspace
    (ROTATION_COMBINED_SWEEP_EXITS_WORKSPACE) — the swept rotation is checked, not
    just the static pose.
    """
    config = _workspace_config("left")
    info = _workspace_info(config)
    sample = footprint_sample(
        config,
        info,
        sample_index=0,
        segment_id="edge",
        sample_type="rotation",
        x_m=0.1,
        y_m=0.1,
        heading_deg=90.0,
    )

    assert sample.inside_workspace is False
    assert "ROTATION_COMBINED_SWEEP_EXITS_WORKSPACE" in sample.violation_reason


def test_geometry_samples_include_translation_and_rotation_sweeps() -> None:
    """기하 표본이 병진(레인/전이)·회전 스윕을 모두 포함하고 위반 0으로 검증됨.

    표본 타입 집합이 {lane_translation, transition_translation, rotation}을 포함하고
    스윕 볼륨 요약의 각 위반 카운트가 0 임을 확인.

    Geometry samples span lane/transition translation and rotation sweeps, and the
    swept-volume summary validates with zero violations of each kind.
    """
    config = _workspace_config("left", transition_style="auto_internal")
    path = generate_side_tool_path(config)
    samples = geometry_samples_for_path(config, path)
    summary = swept_volume_summary(config, path)

    assert samples
    assert {"lane_translation", "transition_translation", "rotation"} <= {
        sample.sample_type for sample in samples
    }
    assert summary["physical_swept_volume_validated"] is True
    assert summary["combined_swept_violation_count"] == 0
    assert summary["rotation_swept_violation_count"] == 0
    assert summary["translation_swept_violation_count"] == 0


def test_impossible_geometry_fails_without_protruding() -> None:
    """폭이 너무 좁고 툴/섀시가 너무 커 배치 불가하면 삐져나오지 말고 명확히 실패.

    실현 불가한 치수 조합은 경계를 넘는 경로를 만드는 대신
    NO_FEASIBLE_TOOL_PLACEMENT ValueError 로 실패해야 함(안전).

    Infeasible dimensions raise NO_FEASIBLE_TOOL_PLACEMENT instead of protruding
    outside the workspace.
    """
    config = _workspace_config(
        b_y_m=0.45,
        tool_lateral_offset_m=0.40,
        tool_width_m=0.40,
        robot_width_m=0.40,
        robot_length_m=0.60,
    )

    with pytest.raises(ValueError, match="NO_FEASIBLE_TOOL_PLACEMENT"):
        generate_side_tool_path(config)


# ── 커버리지 회계(툴 발자국만) / Coverage accounting (tool footprint only) ──


def test_auto_row_count_uses_feasible_coverage_lanes_and_reports_margins() -> None:
    """`row_count="auto"` 가 실현 가능한 커버리지 레인 수를 고르고 미커버 여백을 보고.

    자동 레인 수가 실제 lane_start 수와 일치(>=2), 요청 커버리지 범위(0..1.2)와
    상/하단 미커버 여백이 보고되고 모든 pose 가 경계 내부임을 확인.

    Auto row count selects a feasible number of coverage lanes (>=2), matches the
    emitted lane starts, and reports the uncovered low/high margins.
    """
    config = _workspace_config("left", row_count="auto")
    path = generate_side_tool_path(config)
    summary = summarize_side_tool_path(config, path)
    lane_starts = _lane_starts(path)

    assert summary["actual_lane_count"] == len(lane_starts)
    assert len(lane_starts) >= 2
    assert summary["requested_coverage_y_min_m"] == pytest.approx(0.0)
    assert summary["requested_coverage_y_max_m"] == pytest.approx(1.2)
    assert summary["uncovered_margin_low_m"] > 0.0
    assert summary["uncovered_margin_high_m"] >= 0.0
    assert all(pose["within_boundary"] is True for pose in path)


def test_coverage_area_is_computed_from_tool_footprint_only() -> None:
    """청소 면적이 **툴 발자국만**으로 계산되고 면적/여백 산식이 서로 일치하는지 검증.

    툴 스윕 x/y 경계가 endpoint_inset·여백과 정합하고, uncovered=요청-covered,
    coverage_ratio=covered/요청 이 성립함을 확인(회계 일관성 계약).

    Covered area is derived from the tool footprint alone; the swept x/y bounds,
    uncovered area, and coverage ratio are internally consistent.
    """
    config = _workspace_config("left")
    path = generate_side_tool_path(config)
    summary = summarize_side_tool_path(config, path)
    inset = float(summary["endpoint_inset_m"])

    assert summary["requested_coverage_area_m2"] == pytest.approx(8.0 * 1.2)
    assert summary["tool_swept_x_min_m"] == pytest.approx(inset)
    assert summary["tool_swept_x_max_m"] == pytest.approx(8.0 - inset)
    assert summary["tool_swept_y_min_m"] == pytest.approx(float(summary["uncovered_margin_low_m"]))
    assert summary["tool_swept_y_max_m"] == pytest.approx(1.2 - float(summary["uncovered_margin_high_m"]))
    assert summary["covered_area_m2"] > 7.5
    assert summary["uncovered_area_m2"] == pytest.approx(8.0 * 1.2 - float(summary["covered_area_m2"]))
    assert summary["coverage_ratio"] == pytest.approx(float(summary["covered_area_m2"]) / (8.0 * 1.2))
    assert summary["coverage_ratio"] > 0.8


def test_chassis_only_strip_does_not_count_as_cleaned() -> None:
    """섀시만 지나간(툴이 덮지 않은) 띠는 청소로 집계되지 않음을 검증.

    첫 레인의 섀시 중심 y 가 툴 가장자리 범위 밖(=툴이 덮지 않음)이고, 하단 미커버
    여백이 오프셋+반폭 공식과 일치함을 확인. 레인 자체는 커버리지에 기여.

    A strip covered only by the chassis (not the offset tool) is not counted as
    cleaned; the uncovered low margin equals inset - offset - half-tool-width.
    """
    config = _workspace_config("left")
    path = generate_side_tool_path(config)
    summary = summarize_side_tool_path(config, path)
    first_lane_start = _lane_starts(path)[0]

    assert 0.0 < float(first_lane_start["y_m"]) < 1.2
    assert not (
        float(first_lane_start["tool_edge_min_y_m"])
        <= float(first_lane_start["y_m"])
        <= float(first_lane_start["tool_edge_max_y_m"])
    )
    assert float(summary["uncovered_margin_low_m"]) == pytest.approx(
        float(summary["endpoint_inset_m"]) - config.tool_lateral_offset_m - config.tool_width_m / 2.0
    )
    assert first_lane_start["coverage_contributes"] is True


def test_transitions_do_not_contribute_to_cleaned_coverage() -> None:
    """레인 사이 전이(transition) 구간은 청소 커버리지에 기여하지 않음을 검증.

    coverage_role=="transition" 인 모든 행이 coverage_contributes=False 이고
    모터 명령을 만들지 않음을 확인.

    Transition segments never contribute to cleaned coverage and emit no motor
    command.
    """
    path = generate_side_tool_path(_workspace_config("left"))
    transition_rows = [pose for pose in path if pose["coverage_role"] == "transition"]

    assert transition_rows
    assert all(pose["coverage_contributes"] is False for pose in transition_rows)
    assert all(pose["motor_command_generated"] is False for pose in transition_rows)


# ── 오염(젖은 셀 재밟기) 검사 / Contamination (re-driving over wet cells) ──


def _manual_contamination_pose(segment_id, segment_type, lane_index, x_m, y_m):
    """오염 분석 입력용 최소 pose dict 를 손으로 구성 / Hand-built pose for
    `contamination_analysis` (fixed heading, minimal fields)."""
    return {
        "segment_id": segment_id,
        "segment_type": segment_type,
        "lane_index": lane_index,
        "primitive_type": "cleaning_lane",
        "x_m": x_m,
        "y_m": y_m,
        "heading_deg": 0.0,
    }


def test_chassis_over_prior_tool_swept_cell_is_forbidden() -> None:
    """섀시가 **먼저** 툴이 지나간(젖은) 셀 위를 나중에 밟으면 오염 위반으로 잡힘.

    시간 순서상 앞선 레인이 청소한 셀을 뒤 레인의 섀시가 밟으면 위반 카운트>0,
    사유 CHASSIS_ON_PRIOR_TOOL_SWEPT_AREA 를 확인.

    Chassis driving over a cell the tool already cleaned earlier is a violation
    (CHASSIS_ON_PRIOR_TOOL_SWEPT_AREA) — temporal ordering matters.
    """
    config = _workspace_config(
        "left",
        b_x_m=2.0,
        b_y_m=1.5,
        contamination_mode="strict",
        fail_on_contamination_violation=True,
    )
    poses = [
        _manual_contamination_pose("lane_0", "lane_start", 0, 0.4, 0.30),
        _manual_contamination_pose("lane_0", "lane_end", 0, 1.6, 0.30),
        _manual_contamination_pose("lane_1", "lane_start", 1, 0.4, 0.54),
        _manual_contamination_pose("lane_1", "lane_end", 1, 1.6, 0.54),
    ]

    analysis = contamination_analysis(config, poses)

    assert analysis["contamination_checked"] is True
    assert analysis["contamination_free"] is False
    assert analysis["contamination_violation_count"] > 0
    assert "CHASSIS_ON_PRIOR_TOOL_SWEPT_AREA" in analysis["events"][0].violation_reason


def test_chassis_over_future_tool_swept_cell_is_allowed() -> None:
    """섀시가 **아직** 청소되지 않은(미래에 청소될) 셀 위를 밟는 것은 허용됨.

    시간 순서상 나중에 청소될 셀을 앞 레인 섀시가 미리 밟는 것은 오염이 아님
    (contamination_free=True, 위반 0). 앞 테스트의 대칭 케이스.

    Chassis over a cell that will only be cleaned later is allowed (no
    contamination) — the temporal mirror of the forbidden case.
    """
    config = _workspace_config(
        "left",
        b_x_m=2.0,
        b_y_m=1.5,
        contamination_mode="strict",
        fail_on_contamination_violation=True,
    )
    poses = [
        _manual_contamination_pose("lane_0", "lane_start", 0, 0.4, 0.54),
        _manual_contamination_pose("lane_0", "lane_end", 0, 1.6, 0.54),
        _manual_contamination_pose("lane_1", "lane_start", 1, 0.4, 0.30),
        _manual_contamination_pose("lane_1", "lane_end", 1, 1.6, 0.30),
    ]

    analysis = contamination_analysis(config, poses)

    assert analysis["contamination_free"] is True
    assert analysis["contamination_violation_count"] == 0


def test_tool_over_prior_tool_swept_area_is_allowed_and_tracked() -> None:
    """툴이 이미 청소한 영역을 툴이 다시 지나는 것(재청소)은 허용되고 별도로 집계됨.

    같은 y 를 두 레인이 청소하면 오염은 아니지만(툴↔툴), 재청소 면적
    (tool_reclean_area_m2>0)으로 추적됨을 확인.

    Tool re-cleaning its own prior area is allowed (not contamination) and tracked
    as tool_reclean_area_m2 > 0.
    """
    config = _workspace_config(
        "left",
        b_x_m=2.0,
        b_y_m=1.5,
        contamination_mode="strict",
        fail_on_contamination_violation=True,
    )
    poses = [
        _manual_contamination_pose("lane_0", "lane_start", 0, 0.4, 0.30),
        _manual_contamination_pose("lane_0", "lane_end", 0, 1.6, 0.30),
        _manual_contamination_pose("lane_1", "lane_start", 1, 0.4, 0.30),
        _manual_contamination_pose("lane_1", "lane_end", 1, 1.6, 0.30),
    ]

    analysis = contamination_analysis(config, poses)

    assert analysis["contamination_free"] is True
    assert analysis["tool_reclean_area_m2"] > 0.0


def test_auto_temporal_safe_does_not_accept_single_lane_to_avoid_contamination() -> None:
    """strict 오염 모드에서 커버리지 유지 우선 → 오염을 감수해도 1레인 편법은 안 씀.

    선택 레인 수가 필요 최소치 이상이고 커버리지>0.5 를 유지하되, 이 좁은 사각형
    에서는 오염 없이는 B 도달 불가 → 프리뷰 불가·사유 CONTAMINATION_VIOLATION.
    또한 route_order_strategy 가 알려진 전략 집합 중 하나임을 확인.

    Under strict contamination, the planner keeps required coverage rather than
    degenerating to a single lane; it still cannot reach B contamination-free, so
    it reports not-feasible with CONTAMINATION_VIOLATION and a known strategy.
    """
    config = _workspace_config(
        "left",
        transition_style="auto_internal",
        contamination_mode="strict",
        fail_on_contamination_violation=True,
    )
    path = generate_side_tool_path(config)
    summary = summarize_side_tool_path(config, path)

    assert summary["contamination_free"] is False
    assert summary["contamination_violation_count"] > 0
    assert summary["route_order_strategy"] in {
        "bottom_to_top",
        "top_to_bottom",
        "A_to_B_frontier",
        "B_to_A_retreat",
        "wet_frontier_retreat",
        "dry_corridor_preserving",
        "far_side_to_exit",
        "exit_side_to_far",
        "snake_from_A_to_B",
        "reverse_snake_to_B",
        "tool_lift_transitions",
        "dry_frontier_preserving",
        "clean_from_far_side_to_exit",
        "fixed_serpentine",
    }
    assert summary["selected_lane_count"] >= summary["required_min_lane_count"]
    assert summary["coverage_ratio"] > 0.5
    assert summary["route_reaches_B"] is False
    assert summary["route_feasible_for_physical_preview"] is False
    assert "CONTAMINATION_VIOLATION" in summary["infeasible_reason"]


def test_best_effort_full_coverage_reports_contamination_when_allowed() -> None:
    """warn 모드 + best-effort 허용 시, 완전 커버리지를 얻되 오염을 정직하게 보고.

    coverage_ratio>0.8 을 달성하지만 contamination_free=False, 위반>0, 그리고
    청소된 영역 위 섀시 통과 면적(chassis_after_cleaned_area_m2>0)을 보고함을 확인.

    In warn/best-effort mode the planner achieves high coverage but honestly
    reports contamination (violations > 0 and chassis-after-cleaned area > 0).
    """
    config = _workspace_config(
        "left",
        transition_style="auto_internal",
        route_order="bottom_to_top",
        contamination_mode="warn",
        fail_on_contamination_violation=False,
        allow_contamination_best_effort=True,
    )
    path = generate_side_tool_path(config)
    summary = summarize_side_tool_path(config, path)

    assert summary["coverage_ratio"] > 0.8
    assert summary["contamination_free"] is False
    assert summary["contamination_violation_count"] > 0
    assert summary["chassis_after_cleaned_area_m2"] > 0.0


def test_left_and_right_tool_cases_report_same_feasible_tool_coverage() -> None:
    """좌·우 툴이 대칭이라 커버 면적/미커버 면적/커버리지 비율이 동일하게 보고됨.

    Left and right tool mounts are symmetric, so covered/uncovered area and
    coverage ratio come out equal.
    """
    left_config = _workspace_config("left")
    right_config = _workspace_config("right")
    left_summary = summarize_side_tool_path(left_config, generate_side_tool_path(left_config))
    right_summary = summarize_side_tool_path(right_config, generate_side_tool_path(right_config))

    assert right_summary["covered_area_m2"] == pytest.approx(float(left_summary["covered_area_m2"]))
    assert right_summary["uncovered_area_m2"] == pytest.approx(float(left_summary["uncovered_area_m2"]))
    assert right_summary["coverage_ratio"] == pytest.approx(float(left_summary["coverage_ratio"]))


# ── 전이 엔벨로프 & 끝단 인셋 / Transition envelopes & endpoint inset ──


def test_transition_envelopes_fold_inside_at_x_min_and_x_max() -> None:
    """레인 전환이 x_min/x_max 양쪽 "안쪽 포켓"으로 접혀 엔벨로프가 경계 안에 머무름.

    전이 포켓이 x_min_interior/x_max_interior 양쪽에서 발생하고, 전이 엔벨로프의
    x/y 범위가 여백(0.03) 안에 드는지 확인(내부 접힘으로 경계 이탈 방지).

    Lane transitions fold into interior pockets at both x_min and x_max; their
    envelopes stay within the margin-inset boundary.
    """
    config = _workspace_config("left", b_y_m=2.0, row_count=4)
    path = generate_side_tool_path(config)
    transition_rows = [pose for pose in path if pose["coverage_role"] == "transition"]

    assert transition_rows
    assert {pose["transition_pocket_side"] for pose in transition_rows} == {
        "x_min_interior",
        "x_max_interior",
    }
    assert all(pose["transition_envelope_within_boundary"] is True for pose in transition_rows)
    assert all(float(pose["transition_envelope_x_min_m"]) >= 0.03 - 1e-9 for pose in transition_rows)
    assert all(float(pose["transition_envelope_x_max_m"]) <= 7.97 + 1e-9 for pose in transition_rows)
    assert all(float(pose["transition_envelope_y_min_m"]) >= 0.03 - 1e-9 for pose in transition_rows)
    assert all(float(pose["transition_envelope_y_max_m"]) <= 1.97 + 1e-9 for pose in transition_rows)


def test_endpoint_inset_increases_with_transition_clearance() -> None:
    """툴/오프셋이 커지면 필요한 전이 여유가 커지고 끝단 인셋도 함께 증가함을 검증.

    더 큰 툴 설정이 base 대비 endpoint_inset 과 transition_clearance_radius 를 모두
    키우는지 확인(인셋이 전이 여유에 의해 구동됨).

    Larger tool/offset increases both the endpoint inset and the transition
    clearance radius — inset is driven by turn clearance.
    """
    base_config = _workspace_config("left", b_y_m=2.0)
    larger_tool_config = _workspace_config(
        "left",
        b_y_m=2.0,
        tool_lateral_offset_m=0.45,
        tool_width_m=0.40,
    )

    base_summary = summarize_side_tool_path(base_config, generate_side_tool_path(base_config))
    larger_tool_summary = summarize_side_tool_path(
        larger_tool_config,
        generate_side_tool_path(larger_tool_config),
    )

    assert larger_tool_summary["endpoint_inset_m"] > base_summary["endpoint_inset_m"]
    assert larger_tool_summary["transition_clearance_radius_m"] > base_summary["transition_clearance_radius_m"]


def test_disabled_endpoint_inset_exposes_external_transition_envelope() -> None:
    """끝단 인셋을 끄면 전이 엔벨로프가 사각형 밖으로 삐져나옴을 감지하는지 확인.

    `auto_inset_endpoints=False` 일 때 일부 전이 엔벨로프가 경계 밖(within=False)이고
    사유에 TRANSITION_ENVELOPE_OUTSIDE_WORKSPACE 가 담김을 검증(인셋의 존재 이유 입증).

    Disabling endpoint inset exposes at least one transition envelope outside the
    workspace (TRANSITION_ENVELOPE_OUTSIDE_WORKSPACE) — showing why inset exists.
    """
    config = _workspace_config(
        "left",
        row_count=2,
        auto_inset_endpoints=False,
        fail_on_boundary_violation=False,
        route_order="bottom_to_top",
    )
    path = generate_side_tool_path(config)
    transition_rows = [pose for pose in path if pose["coverage_role"] == "transition"]

    assert transition_rows
    assert any(pose["transition_envelope_within_boundary"] is False for pose in transition_rows)
    assert any("TRANSITION_ENVELOPE_OUTSIDE_WORKSPACE" in pose["boundary_violation_reason"] for pose in transition_rows)


def test_fixed_row_count_too_high_can_fail_when_margins_not_allowed() -> None:
    """여백 불허 상태에서 고정 레인 수가 너무 높으면 명확한 ValueError 로 실패.

    Too-high fixed row count with margins disallowed raises ROW_COUNT_TOO_HIGH.
    """
    config = _workspace_config(
        row_count=20,
        allow_uncovered_margins=False,
    )

    with pytest.raises(ValueError, match="ROW_COUNT_TOO_HIGH"):
        generate_side_tool_path(config)


# ── 전이 시퀀스 & auto_internal 플래너 / Transition sequence & auto_internal ──


def test_transition_sequence_and_motion_directions_alternate() -> None:
    """기본 side_step_reverse_90 전이의 정확한 세그먼트 순서와 교대 방향을 잠금.

    2레인 경로의 세그먼트 순서가 lane→transition(rotate_90/reverse_offset/rotate_back)
    →lane 로 나오고, 모두 차동구동 가능·모터 미생성임을 확인.

    Locks the exact segment order of the default side_step_reverse_90 transition
    (rotate_90 / reverse_offset / rotate_back) between two lanes.
    """
    path = generate_side_tool_path(_workspace_config("left", row_count=2))

    assert [pose["segment_type"] for pose in path] == [
        "lane_start",
        "lane_end",
        "transition_start",
        "rotate_90",
        "reverse_offset",
        "rotate_back",
        "lane_start",
        "lane_end",
    ]
    assert path[2]["transition_style"] == "side_step_reverse_90"
    assert path[3]["segment_type"] == "rotate_90"
    assert path[4]["segment_type"] == "reverse_offset"
    assert all(pose["differential_drive_feasible"] is True for pose in path)
    assert all(pose["motor_command_generated"] is False for pose in path)


def test_auto_internal_is_default_transition_planner_candidate_search() -> None:
    """`auto_internal` 이 후보 탐색으로 동작: side_step_reverse_90 은 후보일 뿐 최종 선택 아님.

    선택 전이 수>0, 후보 수>선택 수, 선택 프리미티브가 내부 접힘 계열 집합에 속하고
    side_step_reverse_90/same_heading_reverse_shift 는 선택되지 않음을 확인.

    In auto_internal the planner searches candidates: it selects internal-fold
    primitives, and side_step_reverse_90 remains a candidate only (not selected).
    """
    config = _workspace_config("left", transition_style="auto_internal")
    path = generate_side_tool_path(config)
    summary = summarize_side_tool_path(config, path)
    transition_rows = [pose for pose in path if pose["primitive_type"] == "transition"]

    assert summary["transition_planner"] == "auto_internal"
    assert summary["side_step_reverse_90_is_candidate_only"] is True
    assert summary["selected_transition_count"] > 0
    assert summary["transition_candidate_count"] > summary["selected_transition_count"]
    assert transition_rows
    assert {pose["segment_type"] for pose in transition_rows} == {
        "transition_start",
        "rotate_90",
        "reverse_offset",
        "rotate_back",
    }
    assert "side_step_reverse_90" not in {
        pose["selected_transition_primitive"] for pose in transition_rows
    }
    assert {
        pose["selected_transition_primitive"] for pose in transition_rows
    } <= {
        "rotate_drive_rotate_shift",
        "short_internal_fold",
        "heading_flip_inside",
        "reverse_internal_fold",
    }
    assert "same_heading_reverse_shift" not in {
        pose["selected_transition_primitive"] for pose in transition_rows
    }


def test_auto_internal_transition_segments_are_not_cleaning_coverage() -> None:
    """auto_internal 전이 세그먼트가 커버리지에 기여하지 않고 셀/면적도 0임을 검증.

    전이 행들은 coverage_contributes=False, 추가 셀/면적 0, 엔벨로프·툴 스윕 모두
    경계 내부임을 확인.

    auto_internal transitions add no coverage cells/area and keep their envelope
    and tool sweep inside the boundary.
    """
    path = generate_side_tool_path(_workspace_config("right", transition_style="auto_internal"))
    transition_rows = [pose for pose in path if pose["primitive_type"] == "transition"]

    assert transition_rows
    assert all(pose["coverage_contributes"] is False for pose in transition_rows)
    assert all(pose["coverage_cells_added"] == 0 for pose in transition_rows)
    assert all(pose["coverage_area_added_m2"] == 0.0 for pose in transition_rows)
    assert all(pose["transition_envelope_within_boundary"] is True for pose in transition_rows)
    assert all(pose["tool_swept_within_boundary"] is True for pose in transition_rows)


def test_plot_segments_have_ids_to_prevent_noncontiguous_tool_connections() -> None:
    """세그먼트 ID 로 레인/전이가 분리되어 플롯에서 비연속 툴 연결이 생기지 않음.

    레인 ID 집합과 전이 ID 집합이 서로소이고, 각 ID 그룹의 세그먼트 타입 순서가
    레인=(lane_start,lane_end) / 전이=(transition_start,rotate_90,reverse_offset,
    rotate_back)로 고정됨을 확인(플롯 아티팩트 방지 계약).

    Segment IDs keep lanes and transitions disjoint with fixed per-id type
    sequences, preventing non-contiguous tool connections in plots.
    """
    path = generate_side_tool_path(_workspace_config("left", transition_style="auto_internal"))
    lane_ids = {pose["segment_id"] for pose in path if pose["primitive_type"] == "cleaning_lane"}
    transition_ids = {pose["segment_id"] for pose in path if pose["primitive_type"] == "transition"}

    assert lane_ids
    assert transition_ids
    assert lane_ids.isdisjoint(transition_ids)
    for segment_id in lane_ids:
        rows = [pose for pose in path if pose["segment_id"] == segment_id]
        assert [pose["segment_type"] for pose in rows] == ["lane_start", "lane_end"]
    for segment_id in transition_ids:
        rows = [pose for pose in path if pose["segment_id"] == segment_id]
        assert [pose["segment_type"] for pose in rows] == [
            "transition_start",
            "rotate_90",
            "reverse_offset",
            "rotate_back",
        ]


def test_auto_internal_keeps_coverage_ratio_from_optimizer() -> None:
    """auto_internal 전이를 써도 옵티마이저의 높은 커버리지가 유지되고 위반 0.

    coverage_ratio>0.8, 경계/전이 엔벨로프/플롯 아티팩트 위반이 모두 0 임을 확인.

    Using auto_internal transitions preserves the optimizer's high coverage with
    zero boundary/envelope/plot-artifact violations.
    """
    config = _workspace_config("left", transition_style="auto_internal")
    path = generate_side_tool_path(config)
    summary = summarize_side_tool_path(config, path)

    assert summary["coverage_ratio"] > 0.8
    assert summary["boundary_violation_count"] == 0
    assert summary["transition_envelope_violation_count"] == 0
    assert summary["plot_artifact_violation_count"] == 0


def test_heading_and_motion_direction_are_distinct() -> None:
    """헤딩(로봇이 향한 방향)과 진행 방향(전/후진)이 별개 개념으로 올바로 구분됨.

    우측 툴 케이스에서 헤딩은 0/180도, motion_direction 은 forward/reverse,
    travel_direction 절댓값은 0/180도, 툴 세계 측면은 above/below 로 갈림을 확인.

    Heading (facing) and motion direction (forward/reverse) are tracked as
    distinct fields; a right tool uses both facings and both drive directions.
    """
    path = generate_side_tool_path(_workspace_config("right"))
    starts = _lane_starts(path)

    assert {float(pose["heading_deg"]) for pose in starts} == {0.0, 180.0}
    assert {pose["motion_direction"] for pose in starts} == {"forward", "reverse"}
    assert {abs(float(pose["travel_direction_deg"])) for pose in starts} == {0.0, 180.0}
    assert {pose["tool_world_side"] for pose in starts} == {"above_chassis", "below_chassis"}


def test_side_tool_path_never_generates_motor_commands() -> None:
    """무모터 불변식: 어떤 pose 도 모터 명령을 만들지 않고 키에 "cmd" 조차 없어야 함.

    No-motor invariant: no pose sets motor_command_generated True, and no pose key
    even contains the substring "cmd".
    """
    path = generate_side_tool_path(_workspace_config("left"))

    assert all(pose["motor_command_generated"] is False for pose in path)
    assert not any("cmd" in key for pose in path for key in pose)


# ── CLI 산출물 계약(파일·요약 문자열) / CLI artifact contracts (files & strings) ──


def test_side_tool_preview_writes_bounded_outputs(tmp_path) -> None:
    """기본 diagonal_ab 미리보기 CLI 가 CSV/summary/PNG 를 쓰고 핵심 문자열/열값을 담음.

    exit 0, 3개 산출물 존재, summary 에 "offline preview-only"·위반 0·커버리지
    모델 문구 등이 있고, CSV 첫 행이 lane_start·모든 행의 경계/모터 플래그가 계약
    대로임을 확인.

    The default diagonal_ab preview CLI writes CSV/summary/PNG with the expected
    offline-only strings, zero-violation lines, and per-row boundary/motor flags.
    """
    result = side_tool_path_preview.main(
        [
            "--advanced",
            "--a-x",
            "0",
            "--a-y",
            "0",
            "--b-x",
            "8",
            "--b-y",
            "1.2",
            "--workspace-mode",
            "diagonal_ab",
            "--tool-side",
            "left",
            "--tool-lateral-offset-m",
            "0.24",
            "--tool-width-m",
            "0.30",
            "--lane-spacing-m",
            "0.25",
            "--row-count",
            "auto",
            "--robot-width-m",
            "0.18",
            "--robot-length-m",
            "0.18",
            "--robot-radius-m",
            "0.14",
            "--first-lane-direction",
            "forward",
            "--transition-style",
            "side_step_reverse_90",
            "--boundary-mode",
            "strict",
            "--boundary-margin-m",
            "0.03",
            "--fail-on-boundary-violation",
            "true",
            "--contamination-mode",
            "off",
            "--out-dir",
            str(tmp_path),
        ]
    )

    assert result == 0
    csv_path = tmp_path / "side_tool_path.csv"
    summary_path = tmp_path / "summary.md"
    png_path = tmp_path / "preview.png"
    assert csv_path.exists()
    assert summary_path.exists()
    assert png_path.exists()

    summary = summary_path.read_text(encoding="utf-8")
    assert "offline preview-only" in summary
    assert "boundary_violation_count: `0`" in summary
    assert "transition_envelope_violation_count: `0`" in summary
    assert "external_turn_bay_allowed: `False`" in summary
    assert "requested_coverage_area_m2" in summary
    assert "covered_area_m2" in summary
    assert "uncovered_area_m2" in summary
    assert "coverage_model: `tool_footprint_only`" in summary
    assert "coverage_grid_model: `tool_footprint_grid_set_cover`" in summary
    assert "requested_coverage_bounds" in summary
    assert "uncovered_margin_low_m" in summary

    with csv_path.open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert rows[0]["segment_type"] == "lane_start"
    assert rows[0]["motor_command_generated"] == "False"
    assert {row["within_boundary"] for row in rows} == {"True"}
    assert {row["chassis_within_boundary"] for row in rows} == {"True"}
    assert {row["tool_within_boundary"] for row in rows} == {"True"}
    assert {row["transition_envelope_within_boundary"] for row in rows} == {"True"}
    assert {row["coverage_contributes"] for row in rows} == {"False", "True"}
    assert {row["motor_command_generated"] for row in rows} == {"False"}


def test_side_tool_preview_writes_auto_internal_outputs(tmp_path) -> None:
    """--advanced + auto_internal 모드가 확장 산출물 집합(스윕/기하 표본·다수 PNG)을 생성.

    summary 에 transition_planner/스윕 검증/툴-우선 문구가 있고, geometry_samples·
    swept_volume_summary·여러 preview PNG 가 존재하며, CSV 각 행의 스윕/모터 플래그가
    계약대로임을 확인. 이 좁은 사각형에서는 오염 없이 B 도달 불가로 표기됨.

    The advanced auto_internal run emits the extended artifact set (swept-volume +
    geometry samples + many PNGs) and the tool-first / not-feasible summary
    strings, with correct per-row swept/motor flags.
    """
    result = side_tool_path_preview.main(
        [
            "--advanced",
            "--workspace-mode",
            "diagonal_ab",
            "--a-x",
            "0",
            "--a-y",
            "0",
            "--b-x",
            "8",
            "--b-y",
            "1.2",
            "--tool-side",
            "left",
            "--coverage-resolution-m",
            "0.05",
            "--transition-style",
            "auto_internal",
            "--fail-on-boundary-violation",
            "true",
            "--out-dir",
            str(tmp_path),
        ]
    )

    assert result == 0
    summary = (tmp_path / "summary.md").read_text(encoding="utf-8")
    assert "transition_planner: `auto_internal`" in summary
    assert "side_step_reverse_90_is_candidate_only: `True`" in summary
    assert "plot_artifact_violation_count: `0`" in summary
    assert "physical_swept_volume_validated: `True`" in summary
    assert "rotation_swept_violation_count: `0`" in summary
    assert "translation_swept_violation_count: `0`" in summary
    assert "combined_swept_violation_count: `0`" in summary
    assert (tmp_path / "geometry_samples.csv").exists()
    assert (tmp_path / "swept_volume_summary.json").exists()
    assert (tmp_path / "preview_overview.png").exists()
    assert (tmp_path / "preview_tool_coverage_only.png").exists()
    assert (tmp_path / "preview_tool_path_primary.png").exists()
    assert (tmp_path / "preview_chassis_derived_from_tool.png").exists()
    assert (tmp_path / "preview_sweep_pattern.png").exists()
    assert (tmp_path / "preview_boustrophedon_pattern.png").exists()
    assert (tmp_path / "preview_chassis_only.png").exists()
    assert (tmp_path / "preview_transitions_only.png").exists()
    assert (tmp_path / "preview_rotation_swept_volume.png").exists()
    assert (tmp_path / "preview_geometry_samples.png").exists()
    with (tmp_path / "side_tool_path.csv").open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert "route_feasible_for_physical_preview: `False`" in summary
    assert "planner_primary_path: `tool_center_path`" in summary
    assert "chassis_path_role: `derived_support_path`" in summary
    assert "tool_path_continuous: `True`" in summary
    assert "chassis_path_derived_from_tool: `True`" in summary
    assert "ROUTE_INFEASIBLE_REACH_B_WITHOUT_CONTAMINATION" in summary
    assert "contamination_mode: `off`" in summary
    assert {row["swept_volume_within_boundary"] for row in rows} == {"True"}
    assert {row["motor_command_generated"] for row in rows} == {"False"}
    with (tmp_path / "geometry_samples.csv").open(encoding="utf-8", newline="") as handle:
        sample_rows = list(csv.DictReader(handle))
    assert sample_rows
    assert {row["motor_command_generated"] for row in sample_rows} == {"False"}


def test_side_tool_preview_writes_primitive_sequence_and_strategy_diagnostics(tmp_path) -> None:
    """ab_diagonal_center 미리보기가 프리미티브 시퀀스 + 전략 진단 산출물을 함께 냄.

    strategy_diagnostics CSV/MD 와 프리미티브 시퀀스 PNG 가 존재하고, route_sequence
    에 실제 move/rotate 명령 텍스트가 있으며, side_tool_path.csv 의 primitive_type 이
    허용 집합·유효·모터 미생성임을 확인. `strategy_diagnostics()` 직접 호출도 검증.

    The ab_diagonal_center preview emits primitive-sequence and strategy-diagnostics
    artifacts, human-readable move/rotate commands, and a valid, motor-free
    primitive CSV.
    """
    result = side_tool_path_preview.main(
        [
            "--advanced",
            "--workspace-mode",
            "ab_diagonal_center",
            "--a-x",
            "0",
            "--a-y",
            "0",
            "--b-x",
            "8",
            "--b-y",
            "1.2",
            "--tool-side",
            "right",
            "--transition-style",
            "auto_internal",
            "--coverage-pattern",
            "boustrophedon_required",
            "--forbid-single-lane-success",
            "true",
            "--contamination-mode",
            "strict",
            "--fail-on-contamination-violation",
            "true",
            "--tool-active-during-transitions",
            "false",
            "--tool-active-during-rotation",
            "false",
            "--emit-timeline-frames",
            "false",
            "--emit-segment-frames",
            "false",
            "--emit-contamination-previews",
            "false",
            "--out-dir",
            str(tmp_path),
        ]
    )

    assert result == 0
    assert (tmp_path / "strategy_diagnostics.csv").exists()
    assert (tmp_path / "strategy_diagnostics.md").exists()
    assert (tmp_path / "preview_primitive_sequence.png").exists()
    route_sequence = (tmp_path / "preview_route_sequence.md").read_text(encoding="utf-8")
    summary = (tmp_path / "summary.md").read_text(encoding="utf-8")
    diagnostics = strategy_diagnostics(
        SideToolPlanConfig(
            workspace_mode="ab_diagonal_center",
            a_x_m=0.0,
            a_y_m=0.0,
            b_x_m=8.0,
            b_y_m=1.2,
            tool_side="right",
            transition_style="auto_internal",
            contamination_mode="strict",
            fail_on_contamination_violation=True,
        )
    )

    assert diagnostics
    assert "## Primitive Commands" in route_sequence
    assert "move_forward(" in route_sequence or "move_backward(" in route_sequence
    assert "rotate_left(" in route_sequence or "rotate_right(" in route_sequence
    assert "strategy_diagnostics_csv" in summary
    assert "route_feasible_for_physical_preview: `False`" in summary

    with (tmp_path / "side_tool_path.csv").open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert rows
    assert {row["primitive_type"] for row in rows} <= {
        "move_forward",
        "move_backward",
        "rotate_left",
        "rotate_right",
    }
    assert {row["primitive_sequence_valid"] for row in rows} == {"True"}
    assert {row["motor_command_generated"] for row in rows} == {"False"}


def test_side_tool_preview_writes_tool_serpentine_reset_outputs(tmp_path) -> None:
    """단순(비-advanced) serpentine CLI 산출물과 정확한 프리미티브 시퀀스를 잠금.

    planner_mode=simple_tool_serpentine, 툴 경로가 A(0,1.2)→B(8,0)로 시작/끝·연속,
    커넥터 비활성(고스트·무커버리지)·트랙 활성, route_sequence 의 P001..P004 명령
    텍스트와 primitive_sequence.csv 첫 9개 프리미티브 순서를 정확히 확인. 오염 문구 없음.

    Locks the simple serpentine CLI outputs: tool path A→B, ghost/inactive
    connectors vs active tracks, and the exact primitive sequence
    (rotate/move ...), all motor-free.
    """
    result = side_tool_path_preview.main(
        [
            "--a-x",
            "0",
            "--a-y",
            "1.2",
            "--b-x",
            "8",
            "--b-y",
            "0",
            "--step-spacing-m",
            "0.25",
            "--tool-side",
            "left",
            "--tool-lateral-offset-m",
            "0.24",
            "--tool-width-m",
            "0.30",
            "--tool-length-m",
            "0.18",
            "--robot-width-m",
            "0.18",
            "--robot-length-m",
            "0.18",
            "--out-dir",
            str(tmp_path),
        ]
    )

    assert result == 0
    assert (tmp_path / "tool_path.csv").exists()
    assert (tmp_path / "primitive_sequence.csv").exists()
    assert (tmp_path / "summary.md").exists()
    assert (tmp_path / "preview_route_sequence.md").exists()
    assert (tmp_path / "preview_tool_path_primary.png").exists()
    assert (tmp_path / "preview_chassis_derived_from_tool.png").exists()
    assert (tmp_path / "preview_primitive_sequence.png").exists()
    assert (tmp_path / "preview_tool_coverage_only.png").exists()
    assert not (tmp_path / "timeline_frames").exists()
    assert not (tmp_path / "segment_frames").exists()

    summary = (tmp_path / "summary.md").read_text(encoding="utf-8")
    route_sequence = (tmp_path / "preview_route_sequence.md").read_text(encoding="utf-8")
    assert "planner_mode: `simple_tool_serpentine`" in summary
    assert "tool_side: `left`" in summary
    assert "tool_path_starts_at_A: `True`" in summary
    assert "tool_path_ends_at_B: `True`" in summary
    assert "tool_path_continuous: `True`" in summary
    assert "tool_active_on_connectors: `False`" in summary
    assert "connector_painting_disabled: `True`" in summary
    assert "motor_command_generated: `False`" in summary
    assert "contamination_mode" not in summary
    assert "## Tool-space route" in route_sequence
    assert "## Derived chassis route" in route_sequence
    assert "## Primitive sequence" in route_sequence
    assert "tool_active=False" in route_sequence
    assert "P001 rotate_right(90.0 deg)" in route_sequence
    assert "P002 move_forward(0.200 m)" in route_sequence
    assert "P003 rotate_left(90.0 deg)" in route_sequence
    assert "P004 move_backward(8.000 m)" in route_sequence

    with (tmp_path / "tool_path.csv").open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert rows[0]["tool_start_x_m"] == "0.0"
    assert rows[0]["tool_start_y_m"] == "1.2"
    assert rows[-1]["tool_end_x_m"] == "8.0"
    assert rows[-1]["tool_end_y_m"] == "0.0"
    connectors = [row for row in rows if row["tool_segment_type"] == "tool_spacing_connector"]
    tracks = [row for row in rows if row["tool_segment_type"] == "tool_sweep_track"]
    assert {row["tool_active"] for row in connectors} == {"False"}
    assert {row["coverage_contributes"] for row in connectors} == {"False"}
    assert {row["is_ghost_tool_path"] for row in connectors} == {"True"}
    assert {row["cleaned_area_added_m2"] for row in connectors} == {"0.0"}
    assert {row["tool_active"] for row in tracks} == {"True"}
    assert {row["coverage_contributes"] for row in tracks} == {"True"}
    assert {row["motor_command_generated"] for row in rows} == {"False"}

    with (tmp_path / "primitive_sequence.csv").open(encoding="utf-8", newline="") as handle:
        primitives = list(csv.DictReader(handle))
    assert [row["primitive_type"] for row in primitives[:9]] == [
        "move_forward",
        "rotate_right",
        "move_forward",
        "rotate_left",
        "move_backward",
        "rotate_right",
        "move_forward",
        "rotate_left",
        "move_forward",
    ]
    assert {row["tool_active"] for row in primitives if row["segment_role"].startswith("connector")} == {"False"}
    assert {row["motor_command_generated"] for row in primitives} == {"False"}


def test_side_tool_preview_simple_cli_rejects_legacy_flags(tmp_path) -> None:
    """단순 CLI(비-advanced)는 레거시 플래그(--workspace-mode)를 거부(SystemExit)해야 함.

    The simple CLI rejects legacy/advanced-only flags like --workspace-mode with a
    SystemExit (argparse error).
    """
    with pytest.raises(SystemExit):
        side_tool_path_preview.main(
            [
                "--a-x",
                "0",
                "--a-y",
                "1.2",
                "--b-x",
                "8",
                "--b-y",
                "0",
                "--step-spacing-m",
                "0.25",
                "--workspace-mode",
                "tool_serpentine_ab",
                "--out-dir",
                str(tmp_path),
            ]
        )


def test_preview_side_tool_path_compatibility_wrapper(tmp_path) -> None:
    """레거시 진입점 `preview_side_tool_path` 호환 래퍼가 여전히 CSV/summary 를 냄.

    The legacy `preview_side_tool_path` wrapper still runs and writes the CSV +
    summary (back-compat).
    """
    result = preview_side_tool_path.main(
        [
            "--a-x",
            "0",
            "--a-y",
            "0",
            "--b-x",
            "8",
            "--b-y",
            "1.2",
            "--workspace-mode",
            "diagonal_ab",
            "--tool-side",
            "right",
            "--tool-lateral-offset-m",
            "0.24",
            "--tool-width-m",
            "0.30",
            "--lane-spacing-m",
            "0.25",
            "--row-count",
            "auto",
            "--out-dir",
            str(tmp_path),
        ]
    )

    assert result == 0
    assert (tmp_path / "side_tool_path.csv").exists()
    assert (tmp_path / "summary.md").exists()


def test_side_tool_waypoint_export_writes_no_motion_targets(tmp_path) -> None:
    """웨이포인트 내보내기 CLI 가 목표 방위 등을 담되 모터 명령은 만들지 않음을 확인.

    CSV 첫 행이 lane_start·target_bearing 0도, 전이 세그먼트 순서가 맞고 reverse
    기대 플래그가 False, 모든 행 모터 미생성·요약에 무모터 문구가 있음을 확인.

    The waypoint-export CLI writes target bearings and the transition sequence
    while keeping every row motor-free (documented no-motion export).
    """
    result = preview_side_tool_waypoints.main(
        [
            "--tool-side",
            "left",
            "--tool-lateral-offset-m",
            "0.4",
            "--tool-width-m",
            "0.3",
            "--lane-spacing-m",
            "2.0",
            "--row-length-m",
            "10.0",
            "--row-count",
            "2",
            "--out-dir",
            str(tmp_path),
        ]
    )

    assert result == 0
    csv_path = tmp_path / "side_tool_waypoints.csv"
    summary_path = tmp_path / "waypoint_summary.md"
    assert csv_path.exists()
    assert summary_path.exists()

    with csv_path.open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert rows[0]["segment_type"] == "lane_start"
    assert rows[0]["target_bearing_deg"] == "0.000"
    assert rows[2]["segment_type"] == "transition_start"
    assert rows[3]["segment_type"] == "rotate_90"
    assert rows[4]["segment_type"] == "reverse_offset"
    assert rows[4]["reverse_direction_expected"] == "False"
    assert {row["motor_command_generated"] for row in rows} == {"False"}
    assert "No rover motor commands are generated" in summary_path.read_text(encoding="utf-8")


def test_side_tool_preview_diagonal_zero_width_cli_fails(tmp_path) -> None:
    """폭 0 diagonal_ab 를 CLI 로 넘겨도 코어와 동일하게 명확한 ValueError 로 실패.

    The CLI propagates the same DIAGONAL_AB_WIDTH_ZERO failure as the core planner
    for a zero-width diagonal.
    """
    with pytest.raises(ValueError, match="DIAGONAL_AB_WIDTH_ZERO"):
        side_tool_path_preview.main(
            [
                "--advanced",
                "--workspace-mode",
                "diagonal_ab",
                "--a-x",
                "0",
                "--a-y",
                "0",
                "--b-x",
                "8",
                "--b-y",
                "0",
                "--out-dir",
                str(tmp_path),
            ]
        )


def test_side_tool_tracking_simulation_produces_virtual_no_motion_outputs(tmp_path) -> None:
    """미리보기 CSV 를 입력으로 추적 시뮬레이터가 "가상"(무모터) 산출물을 만드는지 확인.

    미리보기로 생성한 side_tool_path.csv 를 simulate_side_tool_tracking 에 먹여
    tracking_errors.csv/summary 를 얻고, 모든 행 모터 미생성·virtual_desired_forward_cmd
    열 존재·요약에 무모터 문구가 있음을 검증(2단계 파이프라인).

    Feeding the preview CSV into the tracking simulator yields virtual (motor-free)
    tracking outputs — a two-stage preview→simulate pipeline check.
    """
    preview_dir = tmp_path / "preview"
    sim_dir = tmp_path / "sim"
    assert side_tool_path_preview.main(
        [
            "--advanced",
            "--a-x",
            "0",
            "--a-y",
            "0",
            "--b-x",
            "8",
            "--b-y",
            "1.2",
            "--workspace-mode",
            "diagonal_ab",
            "--tool-side",
            "left",
            "--row-count",
            "auto",
            "--out-dir",
            str(preview_dir),
        ]
    ) == 0

    assert simulate_side_tool_tracking.main(
        [
            "--path-csv",
            str(preview_dir / "side_tool_path.csv"),
            "--out-dir",
            str(sim_dir),
        ]
    ) == 0

    csv_path = sim_dir / "tracking_errors.csv"
    summary_path = sim_dir / "summary.md"
    assert csv_path.exists()
    assert summary_path.exists()
    with csv_path.open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert rows
    assert {row["motor_command_generated"] for row in rows} == {"False"}
    assert "virtual_desired_forward_cmd" in rows[0]
    assert "No rover motor commands are generated" in summary_path.read_text(encoding="utf-8")
