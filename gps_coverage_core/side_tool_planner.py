"""사이드 툴(측면 장착 청소/도장 도구) 커버리지 경로 계획 서브시스템.

목적/역할:
    로버 섀시의 옆(좌/우)에 오프셋되어 달린 "사이드 툴"이 직사각형 작업 영역을
    빈틈없이 쓸고 지나가도록(coverage) 하는 경로를 **기하학적으로만** 계획한다.
    핵심 아이디어는 "툴 우선(tool-first)": 먼저 툴 중심선이 지나갈 스트립(strip)을
    그리드 집합피복(set-cover)으로 고르고, 그 다음 그 툴 배치를 지지(support)하도록
    섀시 포즈를 역산한다. 결과 행(row)들은 미리보기용 기하 정보일 뿐이며,
    모터 명령이 아니다(`motor_command_generated`는 항상 False).

    Plans a coverage path for a *side-mounted* tool (cleaning/painting head)
    offset laterally from the rover chassis. Strategy is "tool-first": tool
    swept strips are chosen by a grid set-cover pass, then chassis poses are
    derived to support them. Output rows are geometry-only previews, never
    motor commands.

시스템 내 위치 (pipeline):
    - 이 모듈은 `gps_coverage_core` 안의 **보조(secondary)** 플래너다. 메인 커버리지
      플래너는 `planner.py`(위경도↔로컬 변환 + 왕복 웨이포인트 생성)이며, 이 파일은
      그와 독립적인 사이드 툴 전용 경로 모델을 제공한다.
    - Import 하는 쪽: `tools/side_tool_path_preview.py`,
      `tools/preview_side_tool_waypoints.py`, `tools/field_ab_to_serpentine.py`
      (CLI 미리보기/시각화 도구). 이들은 공개 함수뿐 아니라 `_workspace_info`,
      `_coverage_grid`, `_cells_for_polygon` 같은 밑줄 헬퍼도 직접 import 하므로,
      리팩토링 시 이 "준(準)공개" 심볼들의 시그니처를 함부로 바꾸면 CLI가 깨진다.
    - 이 파일이 import 하는 것: 표준 라이브러리 `math`, `dataclasses`, `typing`뿐.
      NumPy/외부 의존성 없음(순수 파이썬 기하 계산).

    Secondary planner in `gps_coverage_core`; the main path routing lives in
    `planner.py`. Consumed by the `tools/*` preview CLIs, which import even the
    underscore helpers — treat those as quasi-public when refactoring.

핵심 개념·불변식 (concepts / invariants):
    - **로컬 좌표계**: 계획은 작업 사각형의 로컬 프레임 (x = A→B 축 방향 길이,
      y = 폭 방향, 0..workspace_width_m)에서 이뤄지고, 최종적으로만
      `_workspace_to_world`로 월드 좌표로 사상된다.
    - **A/B 의미(semantic)**는 `workspace_mode`마다 다르다(코너 대각선 vs 중심선+폭
      vs 축+폭). `WorkspaceInfo.a_b_semantic` 참고.
    - **오염(contamination) 모델**: 섀시가 이미 툴이 지나간(젖은) 셀 위를 다시 밟으면
      위반. 시간 순서(temporal)로 셀별 최초 청소 시각을 기록해 검사한다.
    - **미분 구동(differential-drive) 제약**: 옆으로 미끄러지는 이동 불가. 전이
      (transition)는 내부 회전 포켓 안에서만 허용(외부 turn bay 금지).
    - **차원 단위**: 모든 거리 필드 접미사 `_m`(미터), 각도 `_deg`(도). 각도는
      `_normalize_angle_deg`로 (-180, 180] 범위 정규화.

사용법/진입점 (entry points):
    - `generate_side_tool_path(config)` — A/B 경계 기반 다중 레인 커버리지(주 진입점,
      `ab_diagonal_center`/`ab_centerline_width` 등).
    - `generate_tool_serpentine_preview(config)` — 단순 왕복(serpentine) 미리보기
      (`tool_serpentine_ab` 전용, 레거시 레인 탐색을 쓰지 않는 신형 모델).
    - `strategy_diagnostics(config)` — 여러 route-order 전략의 실패 원인 오프라인 진단.
    - `summarize_side_tool_path`, `swept_volume_summary`, `contamination_analysis`
      — 생성된 포즈열에 대한 지표/검증 요약.

리팩토링 노트 (refactor cautions):
    - 포즈 행(dict)은 CLI가 CSV로 그대로 내보내는 **넓은 계약(wide contract)**이다
      (`_pose_dict` 참고). 키 이름을 바꾸면 CSV 스키마·시각화가 깨진다.
    - `route_order`/`workspace_mode`/`chassis_boundary_mode` 등 문자열 리터럴은
      `_validate_config`의 허용 집합과 반드시 동기화할 것.
    - `SideToolPlanConfig`는 frozen dataclass다. 변형이 필요하면 `dataclasses.replace`
      사용(예: `strategy_diagnostics`).
    - `generate_tool_serpentine_preview`의 summary는 `tool_length_m()`를 재사용해
      유효 툴 길이를 tool → robot → 0.18m 순으로 대체한다. (과거 이 자리에서
      미정의 `_robot_length()`를 호출하던 잠재적 NameError 버그는 수정됨.)

    Entry points: generate_side_tool_path / generate_tool_serpentine_preview /
    strategy_diagnostics. Pose rows are a wide CSV-facing contract; do not rename
    keys. Config is a frozen dataclass — mutate via dataclasses.replace.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Literal

# ── 타입 별칭 / Type aliases ──
# 문자열 리터럴 유니온으로 계획 옵션의 유효값을 문서화·강제한다. 이 집합은
# _validate_config 의 검사와 반드시 일치해야 한다.
# String-literal unions documenting valid option values; keep in sync with
# _validate_config.
ToolSide = Literal["left", "right"]
WorkspaceSide = Literal["left", "right"]
SurfaceSide = Literal["left", "right", "centered"]
WorkspaceMode = Literal[
    "tool_serpentine_ab",
    "ab_diagonal_center",
    "ab_centerline_width",
    "diagonal_ab",
    "axis_width",
]
LaneDirection = Literal["forward", "reverse"]
SegmentType = Literal[
    "route_start",
    "route_end",
    "lane_start",
    "lane_end",
    "rotate_90",
    "reverse_offset",
    "rotate_back",
    "transition_start",
    "transition_end",
]
BoundaryMode = Literal["none", "strict"]
ChassisBoundaryMode = Literal[
    "derived_diagnostic",
    "centerline_corridor",
    "clean_surface_strict",
    "operation_polygon",
]
TurnClearanceMode = Literal["simple", "bounding_circle"]
RowCount = int | Literal["auto"]


# ── 데이터 구조: 설정과 결과 레코드 / Data structures: config & result records ──


@dataclass(frozen=True)
class SideToolPlanConfig:
    """사이드 툴 플래너의 전체 입력 설정(불변).

    작업 영역(A/B, 폭, 모드), 로봇/툴 치수, 경계·오염·운동학 제약, 경로 순서
    전략 등 계획에 필요한 모든 파라미터를 담는다. frozen=True 이므로 변경은
    `dataclasses.replace`로만 한다. 대부분의 문자열 필드는 위 Literal 타입의
    유효값만 허용되며 `_validate_config`가 이를 검증한다.

    Immutable full input config for the side-tool planner (workspace A/B,
    robot/tool dimensions, boundary/contamination/kinematic constraints,
    route-order strategy). Frozen — mutate via dataclasses.replace; values are
    validated by _validate_config.
    """

    workspace_mode: WorkspaceMode = "tool_serpentine_ab"
    tool_side: ToolSide = "left"
    tool_lateral_offset_m: float = 0.24
    tool_width_m: float = 0.30
    tool_length_m: float | None = 0.18
    lane_spacing_m: float = 0.25
    step_spacing_m: float = 0.25
    tool_active_on_sweep_tracks: bool = True
    tool_active_on_connectors: bool = False
    force_end_at_B: bool = True
    adjust_spacing_to_end_at_B: bool = True
    sweep_axis: str = "auto"
    row_count: RowCount = "auto"
    a_x_m: float | None = None
    a_y_m: float | None = None
    b_x_m: float | None = None
    b_y_m: float | None = None
    workspace_side: WorkspaceSide | None = None
    surface_side: SurfaceSide | None = None
    workspace_width_m: float | None = None
    row_length_m: float | None = None
    start_x_m: float = 0.0
    start_y_m: float = 0.0
    start_heading_deg: float = 0.0
    first_lane_direction: LaneDirection = "forward"
    transition_style: str = "auto_internal"
    robot_width_m: float = 0.18
    robot_length_m: float | None = 0.18
    robot_radius_m: float | None = 0.14
    boundary_margin_m: float = 0.03
    auto_orient_tool_inside: bool = True
    start_policy: str = "at_A_or_nearest_feasible"
    end_policy: str = "near_B_or_last_lane_feasible"
    boundary_mode: BoundaryMode = "strict"
    chassis_boundary_mode: ChassisBoundaryMode = "derived_diagnostic"
    auto_inset_endpoints: bool = True
    turn_clearance_mode: TurnClearanceMode = "simple"
    fail_on_boundary_violation: bool = False
    allow_uncovered_margins: bool = True
    coverage_resolution_m: float = 0.05
    rotation_sample_deg: float = 5.0
    translation_sample_m: float = 0.05
    swept_volume_validation: str = "strict"
    kinematic_model: str = "differential_drive"
    contamination_mode: str = "off"
    fail_on_contamination_violation: bool = True
    allow_start_end_chassis_outside_clean_surface: bool = True
    tool_active_during_alignment: bool = False
    tool_active_during_transitions: bool = False
    tool_active_during_rotation: bool = False
    visualize_inactive_tool_ghost: bool = True
    show_transition_envelope: bool = False
    same_step_tool_before_chassis: bool = False
    route_order: str = "auto_temporal_safe"
    coverage_pattern: str = "boustrophedon_required"
    min_lane_count: int | Literal["auto"] = "auto"
    forbid_single_lane_success: bool = True
    allow_tool_lift: bool = True
    reserve_dry_exit_corridor: bool = True
    dry_corridor_strategy: str = "auto"
    allow_contamination_best_effort: bool = False
    require_exact_start: bool = False
    require_exact_end: bool = False
    require_start_at_A: bool | None = None
    require_end_at_B: bool | None = None
    max_start_error_m: float = 0.05
    max_end_error_m: float = 0.05
    max_end_offset_m: float = 0.30
    allow_best_effort: bool = False
    allow_unreached_b_best_effort: bool = False
    y_min_m: float | None = None
    y_max_m: float | None = None


@dataclass(frozen=True)
class WorkspaceInfo:
    """설정에서 유도한 작업 영역 기하 정보(불변, 계획 전반의 공유 컨텍스트).

    A/B 좌표, 사각형 길이·폭, 로컬 원점, 월드 경계 상자, 로봇/전이 반경, 헤딩
    부호 등 계획 루틴들이 반복 사용하는 파생값을 한 번에 계산해 담는다.
    `_workspace_info`가 생성한다.

    Immutable derived workspace geometry (A/B, rectangle size, local origin,
    world bbox, radii, heading signs) shared across planning routines; built by
    _workspace_info.
    """

    workspace_mode: WorkspaceMode
    a_b_semantic: str
    bounded: bool
    a_x_m: float
    a_y_m: float
    b_x_m: float
    b_y_m: float
    ab_length_m: float
    diagonal_length_m: float
    rectangle_length_m: float
    rectangle_width_m: float
    ab_heading_deg: float
    workspace_side: WorkspaceSide
    surface_side: SurfaceSide
    workspace_width_m: float
    centerline_y_m: float
    world_x_min_m: float
    world_x_max_m: float
    world_y_min_m: float
    world_y_max_m: float
    world_origin_x_m: float
    world_origin_y_m: float
    endpoint_inset_m: float
    robot_radius_m: float
    transition_clearance_radius_m: float
    first_travel_sign: int
    heading_axis_sign: int
    chassis_heading_deg: float
    world_chassis_heading_deg: float


@dataclass(frozen=True)
class CoverageGridInfo:
    """커버리지 판정을 위한 균일 셀 격자 정보. / Uniform coverage grid over the workspace.

    셀 인덱스는 `iy * nx + ix` 단일 정수로 인코딩된다(집합 연산용).
    Cell index is encoded as a single int `iy * nx + ix` for set operations.
    """

    nx: int
    ny: int
    cell_width_m: float
    cell_height_m: float
    cell_area_m2: float
    total_cells: int


@dataclass(frozen=True)
class ToolStripCandidate:
    """후보 툴 스트립 하나: 특정 y 오프셋·헤딩에서 툴이 덮는 셀 집합.

    집합피복 선택(`_select_tool_strip_candidates`)의 원소이며, 파생 섀시 y와
    툴 가장자리 y, 덮는 셀(frozenset)을 함께 보관한다.

    One candidate tool strip (tool center y + heading) with its covered cell
    set; an element of the set-cover selection.
    """

    candidate_index: int
    tool_center_y_m: float
    tool_world_sign: int
    tool_world_side: str
    heading_axis_sign: int
    heading_deg: float
    candidate_mode: str
    chassis_y_m: float
    tool_edge_min_y_m: float
    tool_edge_max_y_m: float
    covered_cells: frozenset[int]


@dataclass(frozen=True)
class TransitionChoice:
    """레인 간 전이(방향 전환) 프리미티브 선택 결과와 그 포장(envelope) 검사.

    선택된 전이 기법, 비용, 회전 포장 상자, 경계 내 여부/위반 사유를 담는다.
    Chosen inter-lane transition primitive plus its swept envelope box and
    boundary check (used to score/validate turns).
    """

    primitive: str
    candidate_count: int
    cost: float
    envelope_x_min_m: float
    envelope_x_max_m: float
    envelope_y_min_m: float
    envelope_y_max_m: float
    envelope_within_boundary: bool
    violation_reason: str


@dataclass(frozen=True)
class FootprintSample:
    """경로를 따라 촘촘히 샘플링한 한 순간의 섀시+툴 자취(footprint) 스냅샷.

    스위프 볼륨(swept-volume) 검증의 기본 단위. 각 샘플에서 섀시/툴 폴리곤,
    결합 경계 상자, 작업 영역 내 포함 여부를 계산한다.

    One sampled instant of chassis+tool footprint along the path (polygons,
    bbox, inside-workspace flags); the unit of swept-volume validation.
    """

    sample_index: int
    segment_id: str
    sample_type: str
    x_m: float
    y_m: float
    heading_deg: float
    chassis_polygon: tuple[tuple[float, float], ...]
    tool_polygon: tuple[tuple[float, float], ...]
    combined_bbox_x_min_m: float
    combined_bbox_x_max_m: float
    combined_bbox_y_min_m: float
    combined_bbox_y_max_m: float
    inside_workspace: bool
    chassis_inside_workspace: bool
    tool_inside_workspace: bool
    violation_reason: str


@dataclass(frozen=True)
class ContaminationEvent:
    """오염 위반 사건 하나: 섀시가 이미 청소된(젖은) 셀을 밟은 시점 기록.

    발생 시각(time_step)·세그먼트·겹친 셀/면적·사유·심각도를 담는다.
    One contamination violation: chassis overlapping already-cleaned cells at a
    given time step (segment, overlap cells/area, reason, severity).
    """

    event_index: int
    time_step: int
    segment_id: str
    segment_type: str
    lane_index: int
    x_m: float
    y_m: float
    heading_deg: float
    overlap_cells: frozenset[int]
    overlap_area_m2: float
    violation_reason: str
    severity: str


@dataclass(frozen=True)
class TimelineState:
    """시간 축 한 스텝의 누적 상태(청소 면적·커버리지·오염 카운트 등) 스냅샷.

    시각화·검증용 타임라인의 한 행. 지금까지 청소된 면적, 커버리지 비율, B까지
    남은 거리, 재청소/오염 셀 수 등 누적 지표를 담는다.

    Per-time-step cumulative state snapshot (cleaned area, coverage ratio,
    contamination counts, distance-to-B) — one row of the timeline.
    """

    time_step: int
    segment_id: str
    segment_type: str
    lane_index: int
    primitive_type: str
    sample_type: str
    x_m: float
    y_m: float
    heading_deg: float
    tool_x_m: float
    tool_y_m: float
    cleaned_area_m2: float
    coverage_ratio_so_far: float
    contamination_violation_count_so_far: int
    route_to_B_remaining_m: float
    violation_reason: str
    chassis_cells_count: int
    tool_cells_count: int
    prior_cleaned_chassis_overlap_cells: int
    prior_cleaned_chassis_overlap_area_m2: float
    current_tool_reclean_cells: int
    current_tool_reclean_area_m2: float
    tool_active: bool


# ── 기하 원시 함수 / Geometry primitives (angles, unit vectors, side signs) ──


def _normalize_angle_deg(angle_deg: float) -> float:
    """각도를 (-180, 180] 로 정규화. / Wrap an angle into (-180, 180] degrees."""
    return ((angle_deg + 180.0) % 360.0) - 180.0


def _heading_unit(heading_deg: float) -> tuple[float, float]:
    """헤딩 각의 전방 단위 벡터 (cos, sin). / Forward unit vector for a heading."""
    heading_rad = math.radians(heading_deg)
    return math.cos(heading_rad), math.sin(heading_rad)


def _left_normal(heading_deg: float) -> tuple[float, float]:
    """헤딩 기준 좌측 법선 단위 벡터. / Left-hand normal unit vector of a heading.

    전방 벡터를 +90° 회전(=(-uy, ux))한 것. 툴 측면 오프셋 방향 계산의 기준.
    Forward vector rotated +90°; basis for the tool's lateral offset direction.
    """
    ux, uy = _heading_unit(heading_deg)
    return -uy, ux


def _side_sign(side: str) -> int:
    """'left'→+1, 'right'→-1 부호. / Sign for a side: left=+1, right=-1."""
    if side == "left":
        return 1
    if side == "right":
        return -1
    raise ValueError("side must be 'left' or 'right'")


def _surface_side_sign(side: SurfaceSide) -> int:
    """청소 표면 측 부호. 'centered'는 +1로 취급. / Surface-side sign; 'centered'→+1."""
    if side == "left":
        return 1
    if side == "right":
        return -1
    if side == "centered":
        return 1
    raise ValueError("surface_side must be 'left', 'right', or 'centered'")


def _normalize_transition_style(style: str) -> str:
    """전이 스타일 문자열을 표준형으로 정규화(하이픈/언더스코어 허용).

    Normalize transition-style aliases (hyphen vs underscore) to canonical form.
    """
    if style in {"auto-internal", "auto_internal"}:
        return "auto_internal"
    if style in {"side-step-reverse-90", "side_step_reverse_90"}:
        return "side_step_reverse_90"
    raise ValueError("transition_style must be 'auto_internal' or 'side_step_reverse_90'")


def _direction_for_lane(first_lane_direction: LaneDirection, lane_index: int) -> int:
    """레인 인덱스별 진행 부호(±1). 왕복(boustrophedon)이라 홀짝마다 뒤집힌다.

    Travel sign (±1) per lane; alternates every lane for the boustrophedon
    back-and-forth pattern.
    """
    if first_lane_direction not in {"forward", "reverse"}:
        raise ValueError("first_lane_direction must be 'forward' or 'reverse'")
    first = 1 if first_lane_direction == "forward" else -1
    return first if lane_index % 2 == 0 else -first


def _motion_direction(travel_sign: int, heading_axis_sign: int) -> LaneDirection:
    """진행 부호와 헤딩 축 부호가 같으면 전진, 다르면 후진.

    forward if travel and heading-axis signs agree, else reverse.
    """
    return "forward" if travel_sign == heading_axis_sign else "reverse"


# ── 반경·간극·좌표 사상 헬퍼 / Radius, clearance & frame-mapping helpers ──


def _robot_radius(config: SideToolPlanConfig) -> float:
    """섀시의 대표 반경(회전 간극용). / Effective chassis radius for clearance.

    명시 반경이 있으면 그 값, 없으면 폭/길이의 반대각선(외접원 반경)을 쓴다.
    Uses explicit radius if given, else the half-diagonal (circumscribed radius)
    of the width/length box.
    """
    if config.robot_radius_m is not None:
        if config.robot_radius_m <= 0.0:
            raise ValueError("robot_radius_m must be positive")
        return config.robot_radius_m
    if config.robot_width_m <= 0.0:
        raise ValueError("robot_width_m must be positive")
    if config.robot_length_m is not None:
        if config.robot_length_m <= 0.0:
            raise ValueError("robot_length_m must be positive")
        return math.hypot(config.robot_width_m / 2.0, config.robot_length_m / 2.0)
    return config.robot_width_m / 2.0


def _transition_clearance_radius(config: SideToolPlanConfig, robot_radius: float) -> float:
    """제자리 전이 회전에 필요한 최대 간극 반경. / Clearance radius for in-place turns.

    섀시 반경과 "툴의 가장 먼 모서리까지 거리" 중 큰 값. 툴이 옆으로 오프셋+폭을
    가지므로 회전 시 툴 코너가 섀시보다 더 바깥을 쓸 수 있어 이를 포함한다.

    max(chassis radius, tool far-corner radius); the offset side tool can sweep
    wider than the chassis during a turn, so its corner reach is included.
    """
    tool_corner_radius = math.hypot(
        tool_length_m(config) / 2.0,
        config.tool_lateral_offset_m + config.tool_width_m / 2.0,
    )
    return max(robot_radius, tool_corner_radius)


def _has_ab(config: SideToolPlanConfig) -> bool:
    """A/B 네 좌표가 모두 주어졌는지. / True iff all four A/B coords are provided."""
    return (
        config.a_x_m is not None
        and config.a_y_m is not None
        and config.b_x_m is not None
        and config.b_y_m is not None
    )


def _axis_width_to_world(
    origin_x_m: float,
    origin_y_m: float,
    heading_deg: float,
    workspace_side: WorkspaceSide,
    x_m: float,
    y_m: float,
) -> tuple[float, float]:
    """(축 x, 폭 y) 로컬 좌표를 월드로 변환. / Map local (axis x, width y) to world.

    axis_width/centerline 계열 모드에서 로컬 원점·A→B 헤딩·작업측 부호를 써서
    로컬 좌표를 월드로 사상한다. y는 작업측(왼/오른쪽)에 따라 법선 방향이 뒤집힌다.

    For axis_width/centerline modes: places local coords into world using origin,
    A→B heading, and side sign (the width axis flips with workspace side).
    """
    ux, uy = _heading_unit(heading_deg)
    left_x, left_y = _left_normal(heading_deg)
    side = _side_sign(workspace_side)
    nx, ny = left_x * side, left_y * side
    return origin_x_m + ux * x_m + nx * y_m, origin_y_m + uy * x_m + ny * y_m


# ── 설정 검증 / Config validation ──


def _validate_config(config: SideToolPlanConfig) -> None:
    """설정 값의 유효 범위·허용 리터럴을 검사(위반 시 ValueError).

    부수효과 없음. 각 Literal 필드의 허용 집합이 파일 상단 타입 별칭과 동일해야
    한다(둘을 함께 갱신할 것). `_workspace_info` 진입 시 가장 먼저 호출된다.

    Validate config ranges and allowed string literals (raises ValueError). No
    side effects; the allowed sets must match the type aliases above.
    """
    _side_sign(config.tool_side)
    if config.workspace_mode not in {
        "tool_serpentine_ab",
        "ab_diagonal_center",
        "ab_centerline_width",
        "diagonal_ab",
        "axis_width",
    }:
        raise ValueError(
            "workspace_mode must be 'tool_serpentine_ab', 'ab_diagonal_center', "
            "'ab_centerline_width', 'diagonal_ab', or 'axis_width'"
        )
    if config.workspace_side is not None:
        _side_sign(config.workspace_side)
    if config.surface_side is not None and config.surface_side not in {"left", "right", "centered"}:
        raise ValueError("surface_side must be 'left', 'right', or 'centered'")
    _normalize_transition_style(config.transition_style)
    if config.tool_lateral_offset_m < 0.0:
        raise ValueError("tool_lateral_offset_m must be non-negative")
    if config.tool_width_m <= 0.0:
        raise ValueError("tool_width_m must be positive")
    if config.tool_length_m is not None and config.tool_length_m <= 0.0:
        raise ValueError("tool_length_m must be positive")
    if config.lane_spacing_m <= 0.0:
        raise ValueError("lane_spacing_m must be positive")
    if config.step_spacing_m <= 0.0:
        raise ValueError("step_spacing_m must be positive")
    if config.sweep_axis not in {"auto", "x", "y"}:
        raise ValueError("sweep_axis must be 'auto', 'x', or 'y'")
    if config.boundary_margin_m < 0.0:
        raise ValueError("boundary_margin_m must be non-negative")
    if config.row_count != "auto" and config.row_count <= 0:
        raise ValueError("row_count must be 'auto' or positive")
    if config.min_lane_count != "auto" and config.min_lane_count <= 0:
        raise ValueError("min_lane_count must be 'auto' or positive")
    if config.coverage_pattern not in {
        "boustrophedon_required",
        "allow_best_effort_single_lane",
    }:
        raise ValueError(
            "coverage_pattern must be 'boustrophedon_required' or 'allow_best_effort_single_lane'"
        )
    if config.workspace_width_m is not None and config.workspace_width_m <= 0.0:
        raise ValueError("workspace_width_m must be positive")
    if config.row_length_m is not None and config.row_length_m <= 0.0:
        raise ValueError("row_length_m must be positive")
    if config.coverage_resolution_m <= 0.0:
        raise ValueError("coverage_resolution_m must be positive")
    if config.rotation_sample_deg <= 0.0:
        raise ValueError("rotation_sample_deg must be positive")
    if config.translation_sample_m <= 0.0:
        raise ValueError("translation_sample_m must be positive")
    if config.swept_volume_validation not in {"strict", "simple", "off"}:
        raise ValueError("swept_volume_validation must be 'strict', 'simple', or 'off'")
    if config.kinematic_model != "differential_drive":
        raise ValueError("kinematic_model must be 'differential_drive'")
    if config.contamination_mode not in {"strict", "warn", "off"}:
        raise ValueError("contamination_mode must be 'strict', 'warn', or 'off'")
    if config.max_end_offset_m < 0.0:
        raise ValueError("max_end_offset_m must be non-negative")
    if config.max_start_error_m < 0.0:
        raise ValueError("max_start_error_m must be non-negative")
    if config.max_end_error_m < 0.0:
        raise ValueError("max_end_error_m must be non-negative")
    if config.route_order not in {
        "auto_temporal_safe",
        "bottom_to_top",
        "top_to_bottom",
        "far_to_near",
        "near_to_far",
        "nearest_to_exit",
        "farthest_to_exit",
        "A_to_B_frontier",
        "B_to_A_retreat",
    "wet_frontier_retreat",
    "wet_frontier_safe_boustrophedon",
    "top_to_bottom_then_exit",
    "bottom_to_top_then_exit",
        "wet_frontier_safe_boustrophedon",
        "top_to_bottom_then_exit",
        "bottom_to_top_then_exit",
        "dry_frontier_preserving",
        "dry_corridor_preserving",
        "far_side_to_exit",
        "exit_side_to_far",
        "snake_from_A_to_B",
        "reverse_snake_to_B",
        "tool_lift_transitions",
        "clean_from_far_side_to_exit",
        "fixed_serpentine",
    }:
        raise ValueError(
            "route_order must be auto_temporal_safe, bottom_to_top, top_to_bottom, "
            "far_to_near, near_to_far, nearest_to_exit, farthest_to_exit, "
            "A_to_B_frontier, B_to_A_retreat, wet_frontier_retreat, "
            "wet_frontier_safe_boustrophedon, top_to_bottom_then_exit, "
            "bottom_to_top_then_exit, dry_frontier_preserving, "
            "dry_corridor_preserving, far_side_to_exit, "
            "exit_side_to_far, snake_from_A_to_B, reverse_snake_to_B, "
            "tool_lift_transitions, clean_from_far_side_to_exit, or fixed_serpentine"
        )
    if config.boundary_mode not in {"none", "strict"}:
        raise ValueError("boundary_mode must be 'none' or 'strict'")
    if config.chassis_boundary_mode not in {
        "derived_diagnostic",
        "centerline_corridor",
        "clean_surface_strict",
        "operation_polygon",
    }:
        raise ValueError(
            "chassis_boundary_mode must be derived_diagnostic, centerline_corridor, "
            "clean_surface_strict, or operation_polygon"
        )
    if config.turn_clearance_mode not in {"simple", "bounding_circle"}:
        raise ValueError("turn_clearance_mode must be 'simple' or 'bounding_circle'")
    if config.start_policy != "at_A_or_nearest_feasible":
        raise ValueError("only start_policy='at_A_or_nearest_feasible' is supported")
    if config.end_policy != "near_B_or_last_lane_feasible":
        raise ValueError("only end_policy='near_B_or_last_lane_feasible' is supported")


# ── 작업 영역 유도 / Workspace derivation (mode-specific A/B → geometry) ──


def _workspace_info(config: SideToolPlanConfig) -> WorkspaceInfo:
    """설정을 검증하고 `workspace_mode`별로 작업 영역 기하를 계산한다.

    각 모드는 A/B의 의미가 다르다:
      - tool_serpentine_ab: A=좌상단, B=우하단 코너(축 정렬 사각형).
      - ab_diagonal_center / diagonal_ab: A/B는 사각형의 대각선 반대 코너.
      - ab_centerline_width / axis_width: A/B는 중심선 시작·끝, 폭은 별도 지정.
      - (A/B 없음): 레거시 row_length 모드.
    공통적으로 로컬 사각형(길이×폭), 로컬 원점, 월드 경계 상자, 로봇/전이 반경,
    헤딩 축 부호, 엔드포인트 인셋을 계산해 `WorkspaceInfo`로 반환한다.

    Validate config and derive workspace geometry per workspace_mode (A/B carry
    different meanings per mode). Returns a WorkspaceInfo with local rectangle,
    origin, world bbox, radii, heading signs, and endpoint inset. Raises
    ValueError on degenerate/inconsistent inputs.
    """
    _validate_config(config)
    bounded = _has_ab(config)
    robot_radius = _robot_radius(config)

    if bounded and config.workspace_mode == "tool_serpentine_ab":
        # A는 좌상단, B는 우하단이어야 함(축 정렬 직사각형 규약).
        # A must be top-left and B bottom-right (axis-aligned rectangle contract).
        assert config.a_x_m is not None
        assert config.a_y_m is not None
        assert config.b_x_m is not None
        assert config.b_y_m is not None
        a_x = config.a_x_m
        a_y = config.a_y_m
        b_x = config.b_x_m
        b_y = config.b_y_m
        x_min = min(a_x, b_x)
        x_max = max(a_x, b_x)
        y_min = min(a_y, b_y)
        y_max = max(a_y, b_y)
        rectangle_length = x_max - x_min
        workspace_width = y_max - y_min
        if rectangle_length <= 0.0:
            raise ValueError("TOOL_SERPENTINE_AB_LENGTH_ZERO")
        if workspace_width <= 0.0:
            raise ValueError("TOOL_SERPENTINE_AB_WIDTH_ZERO")
        if abs(a_x - x_min) > 1e-9 or abs(a_y - y_max) > 1e-9:
            raise ValueError(
                "TOOL_SERPENTINE_A_NOT_TOP_LEFT: tool_serpentine_ab expects A at the top-left corner"
            )
        if abs(b_x - x_max) > 1e-9 or abs(b_y - y_min) > 1e-9:
            raise ValueError(
                "TOOL_SERPENTINE_B_NOT_BOTTOM_RIGHT: tool_serpentine_ab expects B at the bottom-right corner"
            )
        diagonal_length = math.hypot(b_x - a_x, b_y - a_y)
        ab_length = rectangle_length
        ab_heading = 0.0
        workspace_side = "left"
        surface_side = "left"
        centerline_y = 0.0
        world_origin_x = x_min
        world_origin_y = y_min
        a_b_semantic = "tool_center_start_end_top_left_bottom_right"
    elif bounded and config.workspace_mode == "ab_diagonal_center":
        assert config.a_x_m is not None
        assert config.a_y_m is not None
        assert config.b_x_m is not None
        assert config.b_y_m is not None
        a_x = config.a_x_m
        a_y = config.a_y_m
        b_x = config.b_x_m
        b_y = config.b_y_m
        if config.workspace_width_m is not None or config.workspace_side is not None or config.surface_side is not None:
            raise ValueError(
                "In ab_diagonal_center mode, A and B are rover-center start/end points and opposite rectangle corners. "
                "Do not pass surface/workspace width; use ab_centerline_width for A->B plus width."
            )
        x_min = min(a_x, b_x)
        x_max = max(a_x, b_x)
        y_min = min(a_y, b_y)
        y_max = max(a_y, b_y)
        rectangle_length = x_max - x_min
        workspace_width = y_max - y_min
        if rectangle_length <= 0.0:
            raise ValueError("AB_DIAGONAL_CENTER_LENGTH_ZERO")
        if workspace_width <= 0.0:
            raise ValueError(
                "AB_DIAGONAL_CENTER_WIDTH_ZERO: In ab_diagonal_center mode, A and B must be opposite rectangle corners. "
                "Use B y != A y, or use workspace-mode ab_centerline_width."
            )
        diagonal_length = math.hypot(b_x - a_x, b_y - a_y)
        ab_length = rectangle_length
        ab_heading = 0.0
        workspace_side = "left"
        surface_side = "left"
        centerline_y = 0.0
        world_origin_x = x_min
        world_origin_y = y_min
        a_b_semantic = "rover_center_start_end_diagonal_corners"
    elif bounded and config.workspace_mode == "ab_centerline_width":
        assert config.a_x_m is not None
        assert config.a_y_m is not None
        assert config.b_x_m is not None
        assert config.b_y_m is not None
        a_x = config.a_x_m
        a_y = config.a_y_m
        b_x = config.b_x_m
        b_y = config.b_y_m
        ab_length = math.hypot(b_x - a_x, b_y - a_y)
        if ab_length <= 0.0:
            raise ValueError("A/B length must be positive")
        ab_heading = math.degrees(math.atan2(b_y - a_y, b_x - a_x))
        if config.workspace_width_m is None:
            raise ValueError("workspace_width_m is required for ab_centerline_width planning")
        workspace_width = config.workspace_width_m
        rectangle_length = ab_length
        diagonal_length = math.hypot(rectangle_length, workspace_width)
        surface_side: SurfaceSide = config.surface_side or (
            config.workspace_side if config.workspace_side is not None else "left"
        )
        workspace_side = "right" if surface_side == "right" else "left"
        centerline_y = workspace_width / 2.0 if surface_side == "centered" else 0.0
        # A/B는 청소 표면의 중심선을 정의한다. 로컬 원점(폭 y=0 모서리)은 A에서
        # centerline만큼 되돌린 지점 → 이후 폭 방향 좌표를 0..width로 다룰 수 있다.
        # A/B define the surface centerline; back off by `centerline` so the local
        # width axis runs 0..width from a rectangle edge, not from the centerline.
        world_origin_x, world_origin_y = _axis_width_to_world(
            a_x,
            a_y,
            ab_heading,
            workspace_side,
            0.0,
            -centerline_y,
        )
        corners = [
            _axis_width_to_world(world_origin_x, world_origin_y, ab_heading, workspace_side, 0.0, 0.0),
            _axis_width_to_world(world_origin_x, world_origin_y, ab_heading, workspace_side, ab_length, 0.0),
            _axis_width_to_world(world_origin_x, world_origin_y, ab_heading, workspace_side, ab_length, workspace_width),
            _axis_width_to_world(world_origin_x, world_origin_y, ab_heading, workspace_side, 0.0, workspace_width),
        ]
        x_min = min(point[0] for point in corners)
        x_max = max(point[0] for point in corners)
        y_min = min(point[1] for point in corners)
        y_max = max(point[1] for point in corners)
        a_b_semantic = "rover_center_start_end"
    elif bounded and config.workspace_mode == "diagonal_ab":
        assert config.a_x_m is not None
        assert config.a_y_m is not None
        assert config.b_x_m is not None
        assert config.b_y_m is not None
        a_x = config.a_x_m
        a_y = config.a_y_m
        b_x = config.b_x_m
        b_y = config.b_y_m
        if config.workspace_width_m is not None or config.workspace_side is not None:
            raise ValueError(
                "In diagonal_ab mode, A and B must be opposite rectangle corners. "
                "Do not pass workspace_side/workspace_width_m; use workspace-mode axis_width for A->B plus width."
            )
        x_min = min(a_x, b_x)
        x_max = max(a_x, b_x)
        y_min = min(a_y, b_y)
        y_max = max(a_y, b_y)
        rectangle_length = x_max - x_min
        workspace_width = y_max - y_min
        if rectangle_length <= 0.0:
            raise ValueError("DIAGONAL_AB_LENGTH_ZERO")
        if workspace_width <= 0.0:
            raise ValueError(
                "DIAGONAL_AB_WIDTH_ZERO: In diagonal_ab mode, A and B must be opposite rectangle corners. "
                "Use B y != A y, or use workspace-mode axis_width."
            )
        diagonal_length = math.hypot(b_x - a_x, b_y - a_y)
        ab_length = rectangle_length
        ab_heading = 0.0
        workspace_side: WorkspaceSide = "left"
        surface_side: SurfaceSide = "left"
        centerline_y = 0.0
        world_origin_x = x_min
        world_origin_y = y_min
        a_b_semantic = "opposite_corners"
    elif bounded:
        assert config.a_x_m is not None
        assert config.a_y_m is not None
        assert config.b_x_m is not None
        assert config.b_y_m is not None
        a_x = config.a_x_m
        a_y = config.a_y_m
        b_x = config.b_x_m
        b_y = config.b_y_m
        ab_length = math.hypot(b_x - a_x, b_y - a_y)
        if ab_length <= 0.0:
            raise ValueError("A/B length must be positive")
        ab_heading = math.degrees(math.atan2(b_y - a_y, b_x - a_x))
        if config.workspace_width_m is None:
            raise ValueError("workspace_width_m is required for axis_width planning")
        workspace_width = config.workspace_width_m
        rectangle_length = ab_length
        diagonal_length = math.hypot(rectangle_length, workspace_width)
        workspace_side = config.workspace_side or "left"
        surface_side = workspace_side
        centerline_y = 0.0
        world_origin_x = a_x
        world_origin_y = a_y
        x_min = min(point[0] for point in [
            (a_x, a_y),
            _axis_width_to_world(a_x, a_y, ab_heading, workspace_side, ab_length, 0.0),
            _axis_width_to_world(a_x, a_y, ab_heading, workspace_side, ab_length, workspace_width),
            _axis_width_to_world(a_x, a_y, ab_heading, workspace_side, 0.0, workspace_width),
        ])
        x_max = max(point[0] for point in [
            (a_x, a_y),
            _axis_width_to_world(a_x, a_y, ab_heading, workspace_side, ab_length, 0.0),
            _axis_width_to_world(a_x, a_y, ab_heading, workspace_side, ab_length, workspace_width),
            _axis_width_to_world(a_x, a_y, ab_heading, workspace_side, 0.0, workspace_width),
        ])
        y_min = min(point[1] for point in [
            (a_x, a_y),
            _axis_width_to_world(a_x, a_y, ab_heading, workspace_side, ab_length, 0.0),
            _axis_width_to_world(a_x, a_y, ab_heading, workspace_side, ab_length, workspace_width),
            _axis_width_to_world(a_x, a_y, ab_heading, workspace_side, 0.0, workspace_width),
        ])
        y_max = max(point[1] for point in [
            (a_x, a_y),
            _axis_width_to_world(a_x, a_y, ab_heading, workspace_side, ab_length, 0.0),
            _axis_width_to_world(a_x, a_y, ab_heading, workspace_side, ab_length, workspace_width),
            _axis_width_to_world(a_x, a_y, ab_heading, workspace_side, 0.0, workspace_width),
        ])
        a_b_semantic = "axis_plus_width"
    else:
        if config.row_length_m is None:
            raise ValueError("row_length_m is required when A/B is not provided")
        a_x = config.start_x_m
        a_y = config.start_y_m
        ab_length = config.row_length_m
        ab_heading = config.start_heading_deg
        ux, uy = _heading_unit(ab_heading)
        b_x = a_x + ux * ab_length
        b_y = a_y + uy * ab_length
        rectangle_length = ab_length
        workspace_width = config.workspace_width_m
        if workspace_width is None:
            # 폭 미지정 시: 첫/마지막 레인 중심이 경계·반경·전이 간극을 모두 만족하도록
            # 최소 작업 폭을 역산한다(레인 수와 간격 기반).
            # No width given: back-solve a minimum width so first/last lane centers
            # clear boundary margin, robot radius, and turn clearance.
            lane_span = (
                (int(config.row_count) - 1) * config.lane_spacing_m
                if config.row_count != "auto"
                else 0.0
            )
            transition_radius = _transition_clearance_radius(config, robot_radius)
            first_center = max(
                config.boundary_margin_m + config.tool_width_m / 2.0,
                config.boundary_margin_m + robot_radius + config.tool_lateral_offset_m,
                config.boundary_margin_m + transition_radius + config.tool_lateral_offset_m,
            )
            last_center = first_center + lane_span
            workspace_width = max(
                last_center + config.boundary_margin_m + config.tool_width_m / 2.0,
                last_center + config.boundary_margin_m + robot_radius - config.tool_lateral_offset_m,
                last_center + config.boundary_margin_m + transition_radius - config.tool_lateral_offset_m,
            )
        diagonal_length = math.hypot(rectangle_length, workspace_width)
        workspace_side = config.workspace_side or "left"
        surface_side = workspace_side
        centerline_y = 0.0
        world_origin_x = a_x
        world_origin_y = a_y
        x_min = min(point[0] for point in [
            (a_x, a_y),
            _axis_width_to_world(a_x, a_y, ab_heading, workspace_side, ab_length, 0.0),
            _axis_width_to_world(a_x, a_y, ab_heading, workspace_side, ab_length, workspace_width),
            _axis_width_to_world(a_x, a_y, ab_heading, workspace_side, 0.0, workspace_width),
        ])
        x_max = max(point[0] for point in [
            (a_x, a_y),
            _axis_width_to_world(a_x, a_y, ab_heading, workspace_side, ab_length, 0.0),
            _axis_width_to_world(a_x, a_y, ab_heading, workspace_side, ab_length, workspace_width),
            _axis_width_to_world(a_x, a_y, ab_heading, workspace_side, 0.0, workspace_width),
        ])
        y_min = min(point[1] for point in [
            (a_x, a_y),
            _axis_width_to_world(a_x, a_y, ab_heading, workspace_side, ab_length, 0.0),
            _axis_width_to_world(a_x, a_y, ab_heading, workspace_side, ab_length, workspace_width),
            _axis_width_to_world(a_x, a_y, ab_heading, workspace_side, 0.0, workspace_width),
        ])
        y_max = max(point[1] for point in [
            (a_x, a_y),
            _axis_width_to_world(a_x, a_y, ab_heading, workspace_side, ab_length, 0.0),
            _axis_width_to_world(a_x, a_y, ab_heading, workspace_side, ab_length, workspace_width),
            _axis_width_to_world(a_x, a_y, ab_heading, workspace_side, 0.0, workspace_width),
        ])
        a_b_semantic = "legacy_row_length"

    # 헤딩 축 부호: 툴을 사각형 안쪽으로 향하게 하려면 툴 측 부호를 그대로 쓴다.
    # Heading-axis sign: to keep the tool pointing inward, follow the tool-side sign.
    if config.auto_orient_tool_inside:
        heading_axis_sign = _side_sign(config.tool_side)
    else:
        heading_axis_sign = 1 if abs(_normalize_angle_deg(config.start_heading_deg)) < 90.0 else -1

    first_travel_sign = 1 if config.first_lane_direction == "forward" else -1
    chassis_heading = 0.0 if heading_axis_sign > 0 else 180.0
    transition_clearance_radius = _transition_clearance_radius(config, robot_radius)
    endpoint_inset = 0.0
    # centerline_width 모드는 별도 X 인셋 로직을 쓰므로 여기서 자동 인셋 제외.
    # centerline_width mode handles its own X inset elsewhere, so skip auto-inset.
    if config.auto_inset_endpoints and config.workspace_mode != "ab_centerline_width":
        endpoint_inset = transition_clearance_radius + config.boundary_margin_m
    if endpoint_inset * 2.0 >= ab_length:
        raise ValueError("INSUFFICIENT_ENDPOINT_CLEARANCE")

    return WorkspaceInfo(
        workspace_mode=config.workspace_mode,
        a_b_semantic=a_b_semantic,
        bounded=bounded,
        a_x_m=a_x,
        a_y_m=a_y,
        b_x_m=b_x,
        b_y_m=b_y,
        ab_length_m=ab_length,
        diagonal_length_m=diagonal_length,
        rectangle_length_m=rectangle_length,
        rectangle_width_m=workspace_width,
        ab_heading_deg=ab_heading,
        workspace_side=workspace_side,
        surface_side=surface_side,
        workspace_width_m=workspace_width,
        centerline_y_m=centerline_y,
        world_x_min_m=x_min,
        world_x_max_m=x_max,
        world_y_min_m=y_min,
        world_y_max_m=y_max,
        world_origin_x_m=world_origin_x,
        world_origin_y_m=world_origin_y,
        endpoint_inset_m=endpoint_inset,
        robot_radius_m=robot_radius,
        transition_clearance_radius_m=transition_clearance_radius,
        first_travel_sign=first_travel_sign,
        heading_axis_sign=heading_axis_sign,
        chassis_heading_deg=chassis_heading,
        world_chassis_heading_deg=_normalize_angle_deg(ab_heading + chassis_heading),
    )


def _workspace_to_world(info: WorkspaceInfo, x_m: float, y_m: float) -> tuple[float, float]:
    """로컬 (x, y)를 월드로 변환. / Map local (x, y) to world coordinates.

    코너 정렬 모드(serpentine/diagonal)는 단순 평행이동, 축+폭 모드는 헤딩 회전이
    포함된 `_axis_width_to_world`를 쓴다.

    Corner-aligned modes use a pure translation; axis+width modes rotate via
    _axis_width_to_world.
    """
    if info.workspace_mode in {"tool_serpentine_ab", "ab_diagonal_center", "diagonal_ab"}:
        return info.world_origin_x_m + x_m, info.world_origin_y_m + y_m
    return _axis_width_to_world(
        info.world_origin_x_m,
        info.world_origin_y_m,
        info.ab_heading_deg,
        info.workspace_side,
        x_m,
        y_m,
    )


# ── 폴리곤/자취 기하 / Polygon & footprint geometry ──


def tool_length_m(config: SideToolPlanConfig) -> float:
    """유효 툴 길이(m)를 결정. / Resolve the effective tool length in meters.

    툴 길이 → (없으면) 로봇 길이 → (없으면) 기본 0.18m 순으로 대체(fallback).
    Falls back tool_length → robot_length → 0.18 m.
    """
    if config.tool_length_m is not None:
        return config.tool_length_m
    if config.robot_length_m is not None:
        return config.robot_length_m
    return 0.18


def rectangle_polygon(
    *,
    center_x_m: float,
    center_y_m: float,
    heading_deg: float,
    length_m: float,
    width_m: float,
) -> tuple[tuple[float, float], ...]:
    """중심·헤딩·길이·폭으로 정의된 회전 직사각형의 4코너를 반환.

    코너는 (앞-좌, 앞-우, 뒤-우, 뒤-좌) 순서(CW/CCW 일관). 전방=length, 측방=width.
    Corners of a rotated rectangle (front-left, front-right, back-right,
    back-left) from center/heading/length/width.
    """
    ux, uy = _heading_unit(heading_deg)
    nx, ny = _left_normal(heading_deg)
    half_length = length_m / 2.0
    half_width = width_m / 2.0
    corners = []
    for forward, lateral in (
        (half_length, half_width),
        (half_length, -half_width),
        (-half_length, -half_width),
        (-half_length, half_width),
    ):
        corners.append(
            (
                center_x_m + ux * forward + nx * lateral,
                center_y_m + uy * forward + ny * lateral,
            )
        )
    return tuple(corners)


def chassis_polygon(
    config: SideToolPlanConfig,
    *,
    x_m: float,
    y_m: float,
    heading_deg: float,
) -> tuple[tuple[float, float], ...]:
    """섀시 사각형 폴리곤(중심 = 섀시 위치). / Chassis rectangle polygon at (x, y).

    길이 미지정 시 폭을 대신 사용(정사각형 근사). / Falls back to width if no length.
    """
    robot_length = config.robot_length_m if config.robot_length_m is not None else config.robot_width_m
    return rectangle_polygon(
        center_x_m=x_m,
        center_y_m=y_m,
        heading_deg=heading_deg,
        length_m=robot_length,
        width_m=config.robot_width_m,
    )


def tool_polygon(
    config: SideToolPlanConfig,
    *,
    chassis_x_m: float,
    chassis_y_m: float,
    heading_deg: float,
) -> tuple[tuple[float, float], ...]:
    """섀시 위치에서 오프셋된 툴 사각형 폴리곤. / Tool rectangle polygon.

    섀시 중심에서 측면 오프셋만큼 이동한 툴 중심을 `_tool_pose`로 구해 사각형화한다.
    Tool center is the chassis center shifted by the lateral offset (_tool_pose).
    """
    tool_x_m, tool_y_m, _, _ = _tool_pose(config, heading_deg, chassis_x_m, chassis_y_m)
    return rectangle_polygon(
        center_x_m=tool_x_m,
        center_y_m=tool_y_m,
        heading_deg=heading_deg,
        length_m=tool_length_m(config),
        width_m=config.tool_width_m,
    )


def polygon_bbox(
    polygon: tuple[tuple[float, float], ...],
) -> tuple[float, float, float, float]:
    """폴리곤의 축 정렬 경계 상자 (x_min, x_max, y_min, y_max).

    Axis-aligned bounding box of a polygon.
    """
    xs = [point[0] for point in polygon]
    ys = [point[1] for point in polygon]
    return min(xs), max(xs), min(ys), max(ys)


def _combined_bbox(
    polygons: list[tuple[tuple[float, float], ...]],
) -> tuple[float, float, float, float]:
    """여러 폴리곤을 합친 경계 상자. / Combined bbox over multiple polygons."""
    xs = [point[0] for polygon in polygons for point in polygon]
    ys = [point[1] for polygon in polygons for point in polygon]
    return min(xs), max(xs), min(ys), max(ys)


# ── 경계 포함 검사 / Boundary containment checks ──


def _polygon_inside_workspace(
    info: WorkspaceInfo,
    polygon: tuple[tuple[float, float], ...],
    margin_m: float,
) -> bool:
    """폴리곤 전체가 마진 안쪽의 작업 사각형에 들어오는지. / All corners inside (with margin)."""
    return all(
        _in_interval(x, margin_m, info.ab_length_m - margin_m)
        and _in_interval(y, margin_m, info.workspace_width_m - margin_m)
        for x, y in polygon
    )


def _chassis_allowed_in_workspace(
    config: SideToolPlanConfig,
    info: WorkspaceInfo,
    *,
    x_m: float,
    y_m: float,
) -> bool:
    """섀시 중심이 현재 경계 모드에서 허용되는 위치인지 판정.

    clean_surface_strict: 섀시 원반이 청소면 안(마진+반경)에 완전히 들어와야 함.
    그 외(centerline_corridor 등): 섀시 중심이 A→B 기준선을 따라가고 폭 방향으로만
    반경+마진만큼 삐져나가는 것을 허용(툴은 청소면으로 뻗을 수 있으므로).

    Whether the chassis center is allowed under the active boundary mode. Strict
    mode keeps the whole chassis disk on the clean surface; corridor mode lets
    the center ride the A→B line and bulge out laterally by radius+margin.
    """
    if config.chassis_boundary_mode == "clean_surface_strict":
        r = info.robot_radius_m
        margin = config.boundary_margin_m
        return _in_interval(x_m, margin + r, info.ab_length_m - margin - r) and _in_interval(
            y_m,
            margin + r,
            info.workspace_width_m - margin - r,
        )
    # In the real A/B centerline workflow, the clean surface is not automatically
    # the chassis hard boundary. The chassis center may run on the A->B
    # reference line while the side tool reaches into the clean surface.
    r = info.robot_radius_m
    return _in_interval(x_m, 0.0, info.ab_length_m) and _in_interval(
        y_m,
        -r - config.boundary_margin_m,
        info.workspace_width_m + r + config.boundary_margin_m,
    )


def _tool_clean_surface_ok(
    config: SideToolPlanConfig,
    info: WorkspaceInfo,
    *,
    segment_type: SegmentType,
    primitive_type: str,
    tool_x_m: float,
    tool_y_m: float,
    tool_edge_min_y_m: float,
    tool_edge_max_y_m: float,
) -> bool:
    """활성 청소 레인에서 툴이 청소면(마진 안) 안에 완전히 들어오는지.

    청소 레인이 아니면 항상 True(전이/정렬 중 툴은 청소면 밖도 허용). 청소 중에는
    툴 중심(길이 방향)과 양쪽 폭 가장자리가 모두 경계 안이어야 한다.

    Whether the tool stays on the clean surface during an active cleaning lane.
    Non-cleaning primitives pass unconditionally; while cleaning, both the
    length-wise center and both width edges must be within margins.
    """
    if primitive_type != "cleaning_lane":
        return True
    margin = config.boundary_margin_m
    tool_half_length = tool_length_m(config) / 2.0
    return (
        _in_interval(tool_x_m, margin + tool_half_length, info.ab_length_m - margin - tool_half_length)
        and _in_interval(tool_y_m, margin, info.workspace_width_m - margin)
        and _in_interval(tool_edge_min_y_m, margin, info.workspace_width_m - margin)
        and _in_interval(tool_edge_max_y_m, margin, info.workspace_width_m - margin)
    )


# ── 셀 커버리지 래스터화 / Cell coverage rasterization ──


def _point_in_polygon(x_m: float, y_m: float, polygon: tuple[tuple[float, float], ...]) -> bool:
    """점이 폴리곤 내부인지(레이 캐스팅). / Ray-casting point-in-polygon test.

    수평 반직선이 변과 홀수 번 교차하면 내부. `(yj - yi) or 1e-12`는 수평 변에서의
    0 나눗셈을 피하기 위한 방어값.

    Odd crossings of a horizontal ray → inside. The `or 1e-12` guards against
    division by zero on horizontal edges.
    """
    inside = False
    j = len(polygon) - 1
    for i, point in enumerate(polygon):
        xi, yi = point
        xj, yj = polygon[j]
        intersects = (yi > y_m) != (yj > y_m) and (
            x_m < (xj - xi) * (y_m - yi) / ((yj - yi) or 1e-12) + xi
        )
        if intersects:
            inside = not inside
        j = i
    return inside


def _cells_for_polygon(
    info: WorkspaceInfo,
    grid: CoverageGridInfo,
    polygon: tuple[tuple[float, float], ...],
) -> frozenset[int]:
    """폴리곤이 덮는 격자 셀 인덱스 집합을 반환. / Grid cells covered by a polygon.

    폴리곤 bbox로 후보 셀 범위를 좁힌 뒤, 각 셀 중심이 작업 영역 내이면서
    폴리곤 내부인지 검사(중심점 샘플링). 셀 인덱스는 `iy*nx+ix`.

    Narrows to the polygon bbox, then keeps cells whose center is inside both the
    workspace and the polygon (center-sampling). Index = iy*nx+ix.
    """
    x_min, x_max, y_min, y_max = polygon_bbox(polygon)
    ix_min = max(0, int(math.floor(x_min / grid.cell_width_m)))
    ix_max = min(grid.nx - 1, int(math.ceil(x_max / grid.cell_width_m)))
    iy_min = max(0, int(math.floor(y_min / grid.cell_height_m)))
    iy_max = min(grid.ny - 1, int(math.ceil(y_max / grid.cell_height_m)))
    cells: set[int] = set()
    for ix in range(ix_min, ix_max + 1):
        cx = (ix + 0.5) * grid.cell_width_m
        if not _in_interval(cx, 0.0, info.ab_length_m):
            continue
        for iy in range(iy_min, iy_max + 1):
            cy = (iy + 0.5) * grid.cell_height_m
            if _in_interval(cy, 0.0, info.workspace_width_m) and _point_in_polygon(
                cx,
                cy,
                polygon,
            ):
                cells.add(iy * grid.nx + ix)
    return frozenset(cells)


def footprint_sample(
    config: SideToolPlanConfig,
    info: WorkspaceInfo,
    *,
    sample_index: int,
    segment_id: str,
    sample_type: str,
    x_m: float,
    y_m: float,
    heading_deg: float,
) -> FootprintSample:
    """한 포즈에서 섀시+툴 자취 샘플을 만들고 경계 위반 사유를 채운다.

    경계 모드에 따라 섀시/툴 포함 검사 방식이 다르다(strict=폴리곤 전체 포함,
    corridor=섀시 중심 허용 + 활성 툴은 레인별 검사이므로 여기선 tool_inside=True).

    Build a chassis+tool footprint sample at one pose and record boundary
    violations. Containment differs by boundary mode (strict = full polygon;
    corridor = chassis-center rule, tool checked per-lane so tool_inside=True).
    """
    chassis = chassis_polygon(config, x_m=x_m, y_m=y_m, heading_deg=heading_deg)
    tool = tool_polygon(config, chassis_x_m=x_m, chassis_y_m=y_m, heading_deg=heading_deg)
    bbox = _combined_bbox([chassis, tool])
    margin = config.boundary_margin_m
    if config.chassis_boundary_mode == "clean_surface_strict":
        chassis_inside = _polygon_inside_workspace(info, chassis, margin)
        tool_inside = _polygon_inside_workspace(info, tool, margin)
    else:
        chassis_inside = _chassis_allowed_in_workspace(config, info, x_m=x_m, y_m=y_m)
        # In centerline-corridor mode the clean surface is not the hard physical
        # boundary for an inactive tool/chassis support maneuver. Active tool
        # clean-surface containment is checked per planned row.
        tool_inside = True
    inside = chassis_inside and tool_inside
    reasons: list[str] = []
    if not chassis_inside:
        reasons.append("ROTATION_CHASSIS_SWEEP_EXITS_WORKSPACE")
    if not tool_inside:
        reasons.append("ROTATION_TOOL_SWEEP_EXITS_WORKSPACE")
    if not inside:
        reasons.append("ROTATION_COMBINED_SWEEP_EXITS_WORKSPACE")
        reasons.append("ROTATION_SAMPLE_OUTSIDE_WORKSPACE")
    return FootprintSample(
        sample_index=sample_index,
        segment_id=segment_id,
        sample_type=sample_type,
        x_m=x_m,
        y_m=y_m,
        heading_deg=heading_deg,
        chassis_polygon=chassis,
        tool_polygon=tool,
        combined_bbox_x_min_m=bbox[0],
        combined_bbox_x_max_m=bbox[1],
        combined_bbox_y_min_m=bbox[2],
        combined_bbox_y_max_m=bbox[3],
        inside_workspace=inside,
        chassis_inside_workspace=chassis_inside,
        tool_inside_workspace=tool_inside,
        violation_reason="+".join(reasons) if reasons else "OK",
    )


# ── 스위프 볼륨 샘플링 / Swept-volume sampling (translation & rotation) ──


def _interpolate_angle(start_deg: float, end_deg: float, t: float) -> float:
    """두 각도를 최단 방향으로 t∈[0,1] 보간. / Shortest-arc angle interp at t∈[0,1]."""
    delta = _normalize_angle_deg(end_deg - start_deg)
    return _normalize_angle_deg(start_deg + delta * t)


def _sample_count_for_distance(distance_m: float, step_m: float) -> int:
    """이동 거리를 step_m 간격으로 샘플링할 개수(최소 2). / Sample count for a translation."""
    return max(2, math.ceil(distance_m / step_m) + 1)


def _sample_count_for_rotation(start_deg: float, end_deg: float, step_deg: float) -> int:
    """회전량을 step_deg 간격으로 샘플링할 개수(무회전이면 0). / Sample count for a rotation."""
    delta = abs(_normalize_angle_deg(end_deg - start_deg))
    if delta < 1e-9:
        return 0
    return max(2, math.ceil(delta / step_deg) + 1)


def _sample_segment_translation(
    config: SideToolPlanConfig,
    info: WorkspaceInfo,
    start: dict[str, float | int | str | bool],
    end: dict[str, float | int | str | bool],
    sample_index_start: int,
    sample_type: str,
) -> list[FootprintSample]:
    """두 포즈 사이 직선 이동 구간을 자취 샘플 리스트로 이산화.

    Discretize a straight-line move between two poses into footprint samples.
    """
    distance = math.hypot(
        float(end["x_m"]) - float(start["x_m"]),
        float(end["y_m"]) - float(start["y_m"]),
    )
    count = _sample_count_for_distance(distance, config.translation_sample_m)
    samples: list[FootprintSample] = []
    for i in range(count):
        t = i / (count - 1) if count > 1 else 0.0
        x_m = float(start["x_m"]) + (float(end["x_m"]) - float(start["x_m"])) * t
        y_m = float(start["y_m"]) + (float(end["y_m"]) - float(start["y_m"])) * t
        heading_deg = _interpolate_angle(float(start["heading_deg"]), float(end["heading_deg"]), t)
        samples.append(
            footprint_sample(
                config,
                info,
                sample_index=sample_index_start + len(samples),
                segment_id=str(start["segment_id"]),
                sample_type=sample_type,
                x_m=x_m,
                y_m=y_m,
                heading_deg=heading_deg,
            )
        )
    return samples


def _sample_segment_rotation(
    config: SideToolPlanConfig,
    info: WorkspaceInfo,
    start: dict[str, float | int | str | bool],
    end: dict[str, float | int | str | bool],
    sample_index_start: int,
) -> list[FootprintSample]:
    """제자리 회전 구간을 자취 샘플 리스트로 이산화(회전 중심=끝 포즈).

    Discretize an in-place rotation into footprint samples (center = end pose).
    """
    count = _sample_count_for_rotation(
        float(start["heading_deg"]),
        float(end["heading_deg"]),
        config.rotation_sample_deg,
    )
    if count == 0:
        return []
    samples: list[FootprintSample] = []
    # Use the end pose as the rotation center for an internal transition. This
    # matches the preview semantics: the route translates into an internal
    # pocket, then changes heading there.
    x_m = float(end["x_m"])
    y_m = float(end["y_m"])
    for i in range(count):
        t = i / (count - 1) if count > 1 else 0.0
        samples.append(
            footprint_sample(
                config,
                info,
                sample_index=sample_index_start + len(samples),
                segment_id=str(start["segment_id"]),
                sample_type="rotation",
                x_m=x_m,
                y_m=y_m,
                heading_deg=_interpolate_angle(float(start["heading_deg"]), float(end["heading_deg"]), t),
            )
        )
    return samples


def geometry_samples_for_path(
    config: SideToolPlanConfig,
    poses: list[dict[str, float | int | str | bool]],
) -> list[FootprintSample]:
    """포즈열 전체를 촘촘한 자취 샘플 리스트로 전개(공개).

    인접 포즈 쌍마다 이동+회전 구간을 나눠 샘플링한다. `swept_volume_validation`이
    'off'면 빈 리스트. CLI 도구가 직접 호출하는 준공개 함수.

    Expand a full pose list into dense footprint samples (translation +
    rotation per adjacent pair). Empty if validation is 'off'. Called directly
    by the preview CLIs.
    """
    if config.swept_volume_validation == "off":
        return []
    info = _workspace_info(config)
    samples: list[FootprintSample] = []
    for start, end in zip(poses, poses[1:]):
        distance = math.hypot(
            float(end["x_m"]) - float(start["x_m"]),
            float(end["y_m"]) - float(start["y_m"]),
        )
        heading_delta = abs(
            _normalize_angle_deg(float(end["heading_deg"]) - float(start["heading_deg"]))
        )
        sample_type = (
            "lane_translation"
            if start["primitive_type"] == "cleaning_lane"
            else "transition_translation"
        )
        if distance > 1e-9:
            samples.extend(
                _sample_segment_translation(
                    config,
                    info,
                    start,
                    end,
                    len(samples),
                    sample_type,
                )
            )
        if heading_delta > 1e-9:
            samples.extend(
                _sample_segment_rotation(
                    config,
                    info,
                    start,
                    end,
                    len(samples),
                )
            )
    return samples


def swept_volume_summary(
    config: SideToolPlanConfig,
    poses: list[dict[str, float | int | str | bool]],
) -> dict[str, float | int | str | bool]:
    """경로의 스위프 볼륨 검증 결과 요약 딕셔너리(공개).

    회전/이동/결합별 위반 샘플 수, 샘플 총수, 로봇·툴 치수, 검증 모드 등을 담는다.
    `motor_command_generated`는 항상 False(미리보기 전용임을 명시).

    Summary of swept-volume validation (per-type violation counts, sample totals,
    robot/tool dims). motor_command_generated is always False (preview only).
    """
    info = _workspace_info(config)
    samples = geometry_samples_for_path(config, poses)
    rotation_samples = [sample for sample in samples if sample.sample_type == "rotation"]
    translation_samples = [sample for sample in samples if sample.sample_type != "rotation"]
    rotation_violations = [sample for sample in rotation_samples if not sample.inside_workspace]
    translation_violations = [sample for sample in translation_samples if not sample.inside_workspace]
    combined_violations = [sample for sample in samples if not sample.inside_workspace]
    return {
        "workspace_x_min_m": 0.0,
        "workspace_x_max_m": info.ab_length_m,
        "workspace_y_min_m": 0.0,
        "workspace_y_max_m": info.workspace_width_m,
        "robot_width_m": config.robot_width_m,
        "robot_length_m": config.robot_length_m if config.robot_length_m is not None else config.robot_width_m,
        "robot_radius_m": info.robot_radius_m,
        "tool_width_m": config.tool_width_m,
        "tool_length_m": tool_length_m(config),
        "tool_lateral_offset_m": config.tool_lateral_offset_m,
        "tool_length_source": "TOOL_LENGTH_M" if config.tool_length_m is not None else "ROBOT_LENGTH_DEFAULT",
        "rotation_sample_deg": config.rotation_sample_deg,
        "translation_sample_m": config.translation_sample_m,
        "swept_volume_validation": config.swept_volume_validation,
        "total_geometry_sample_count": len(samples),
        "rotation_sample_count": len(rotation_samples),
        "translation_sample_count": len(translation_samples),
        "rotation_swept_violation_count": len(rotation_violations),
        "translation_swept_violation_count": len(translation_violations),
        "combined_swept_violation_count": len(combined_violations),
        "physical_swept_volume_validated": config.swept_volume_validation != "off",
        "motor_command_generated": False,
    }


# ── 오염 분석 / Contamination analysis (chassis over wet cells) ──


def _pose_by_segment(
    poses: list[dict[str, float | int | str | bool]],
) -> dict[str, dict[str, float | int | str | bool]]:
    """세그먼트 ID → 첫 포즈 매핑. / Map each segment_id to its first pose."""
    mapping: dict[str, dict[str, float | int | str | bool]] = {}
    for pose in poses:
        mapping.setdefault(str(pose["segment_id"]), pose)
    return mapping


def _tool_active_for_sample(
    config: SideToolPlanConfig,
    pose: dict[str, float | int | str | bool],
    sample: FootprintSample,
) -> bool:
    """이 샘플 순간에 툴이 활성(청소 중)인지 판정.

    회전/시작 정렬/전이 구간의 활성 여부는 각각의 config 플래그로 결정되고,
    그 외 일반 청소 구간은 활성으로 본다.

    Whether the tool is active (cleaning) at this sample; rotation/start-align/
    transition follow their config flags, otherwise active.
    """
    if sample.sample_type == "rotation":
        return config.tool_active_during_rotation
    if str(pose["segment_id"]).startswith("route_start_A"):
        return config.tool_active_during_alignment
    if pose["primitive_type"] == "transition":
        return config.tool_active_during_transitions
    return True


def contamination_analysis(
    config: SideToolPlanConfig,
    poses: list[dict[str, float | int | str | bool]],
) -> dict[str, object]:
    """섀시가 이미 청소된(젖은) 셀을 다시 밟는 시간적 오염을 분석(공개).

    각 시간 스텝에서 섀시/툴 셀을 래스터화하고, (1) 이전 스텝에 이미 청소된 셀을
    섀시가 밟았는지, (2) 같은 스텝에 툴이 방금 청소한 셀을 섀시가 밟았는지
    검사한다. 위반 사건 목록과 스텝별 타임라인, 청소된 셀 집합을 반환한다.
    `contamination_mode == "off"`면 검사를 건너뛰고 기본(무위반) 요약을 준다.

    Analyze temporal contamination — chassis stepping onto already-cleaned (wet)
    cells. Per time step it checks overlap with (1) prior-cleaned cells and
    (2) the same-step tool sweep, returning events, a timeline, and cleaned
    cells. Skips the check when contamination_mode == "off".
    """
    info = _workspace_info(config)
    grid = _coverage_grid(info, config.coverage_resolution_m)
    selected_strategy = (
        str(poses[0].get("route_order_strategy", config.route_order))
        if poses
        else config.route_order
    )
    strategies_evaluated = (
        int(poses[0].get("route_order_strategy_candidates_evaluated", 1))
        if poses
        else 0
    )
    if config.contamination_mode == "off":
        return {
            "contamination_model": "no_chassis_on_prior_tool_swept",
            "contamination_mode": "off",
            "fail_on_contamination_violation": config.fail_on_contamination_violation,
            "tool_active_during_alignment": config.tool_active_during_alignment,
            "tool_active_during_transitions": config.tool_active_during_transitions,
            "tool_active_during_rotation": config.tool_active_during_rotation,
            "same_step_tool_before_chassis": config.same_step_tool_before_chassis,
            "contamination_checked": False,
            "contamination_free": True,
            "temporal_cleaning_valid": False,
            "contamination_violation_count": 0,
            "contamination_violation_area_m2": 0.0,
            "first_contamination_step": "NA",
            "first_contamination_segment_id": "NA",
            "cleaned_area_m2": 0.0,
            "chassis_after_cleaned_area_m2": 0.0,
            "tool_reclean_area_m2": 0.0,
            "route_order_strategy": selected_strategy,
            "route_order_strategy_candidates_evaluated": strategies_evaluated,
            "events": [],
            "timeline": [],
            "cleaned_cells": set(),
        }

    samples = geometry_samples_for_path(config, poses)
    pose_lookup = _pose_by_segment(poses)
    cleaned_at_step: dict[int, int] = {}
    tool_reclean_cells_seen: set[int] = set()
    chassis_after_cleaned_cells_seen: set[int] = set()
    events: list[ContaminationEvent] = []
    timeline: list[TimelineState] = []

    for time_step, sample in enumerate(samples):
        pose = pose_lookup[str(sample.segment_id)]
        chassis_cells = _cells_for_polygon(info, grid, sample.chassis_polygon)
        tool_cells = _cells_for_polygon(info, grid, sample.tool_polygon)
        # 이전 스텝에서 이미 청소된 셀을 섀시가 밟으면 오염(엄격한 시간 순서 검사).
        # Chassis stepping on cells cleaned at an *earlier* step = contamination.
        prior_cleaned = {
            cell
            for cell in chassis_cells
            if cell in cleaned_at_step and cleaned_at_step[cell] < time_step
        }
        tool_active = _tool_active_for_sample(config, pose, sample)
        # 같은 스텝에 툴이 방금 청소한 셀을 섀시가 밟는 경우도 위반으로 본다. 단,
        # same_step_tool_before_chassis 이면 툴이 섀시보다 앞서므로 허용.
        # Same-step chassis/tool overlap is a violation unless the tool is
        # declared to lead the chassis within a step.
        same_step_overlap = (
            set(chassis_cells & tool_cells)
            if tool_active and not config.same_step_tool_before_chassis
            else set()
        )
        violation_reason = "OK"
        violation_cells = set(prior_cleaned) | set(same_step_overlap)
        if violation_cells:
            chassis_after_cleaned_cells_seen.update(violation_cells)
            violation_reason = (
                "CHASSIS_ON_WET_AREA_DURING_TRANSITION"
                if pose["primitive_type"] == "transition"
                else "CHASSIS_ON_WET_AREA_DURING_LANE"
            )
            if same_step_overlap:
                violation_reason = f"{violation_reason}+CHASSIS_ON_CURRENT_TOOL_SWEPT_AREA"
            reason_prefix = (
                "CHASSIS_ON_PRIOR_TOOL_SWEPT_AREA"
                if prior_cleaned
                else "CHASSIS_ON_CURRENT_TOOL_SWEPT_AREA"
            )
            events.append(
                ContaminationEvent(
                    event_index=len(events),
                    time_step=time_step,
                    segment_id=str(sample.segment_id),
                    segment_type=str(pose["segment_type"]),
                    lane_index=int(pose["lane_index"]),
                    x_m=sample.x_m,
                    y_m=sample.y_m,
                    heading_deg=sample.heading_deg,
                    overlap_cells=frozenset(violation_cells),
                    overlap_area_m2=len(violation_cells) * grid.cell_area_m2,
                    violation_reason=f"{reason_prefix}+{violation_reason}",
                    severity="ERROR" if config.contamination_mode == "strict" else "WARN",
                )
            )

        current_reclean = {cell for cell in tool_cells if cell in cleaned_at_step}
        if tool_active:
            tool_reclean_cells_seen.update(current_reclean)
            for cell in tool_cells:
                cleaned_at_step.setdefault(cell, time_step)

        cleaned_area = len(cleaned_at_step) * grid.cell_area_m2
        timeline.append(
            TimelineState(
                time_step=time_step,
                segment_id=str(sample.segment_id),
                segment_type=str(pose["segment_type"]),
                lane_index=int(pose["lane_index"]),
                primitive_type=str(pose["primitive_type"]),
                sample_type=sample.sample_type,
                x_m=sample.x_m,
                y_m=sample.y_m,
                heading_deg=sample.heading_deg,
                tool_x_m=sum(point[0] for point in sample.tool_polygon) / len(sample.tool_polygon),
                tool_y_m=sum(point[1] for point in sample.tool_polygon) / len(sample.tool_polygon),
                cleaned_area_m2=cleaned_area,
                coverage_ratio_so_far=cleaned_area / (info.ab_length_m * info.workspace_width_m),
                contamination_violation_count_so_far=len(events),
                route_to_B_remaining_m=math.hypot(
                    info.ab_length_m - sample.x_m,
                    info.centerline_y_m - sample.y_m,
                ),
                violation_reason=events[-1].violation_reason if violation_cells else "OK",
                chassis_cells_count=len(chassis_cells),
                tool_cells_count=len(tool_cells),
                prior_cleaned_chassis_overlap_cells=len(prior_cleaned),
                prior_cleaned_chassis_overlap_area_m2=len(violation_cells) * grid.cell_area_m2,
                current_tool_reclean_cells=len(current_reclean),
                current_tool_reclean_area_m2=len(current_reclean) * grid.cell_area_m2,
                tool_active=tool_active,
            )
        )

    first = events[0] if events else None
    return {
        "contamination_model": "no_chassis_on_prior_tool_swept",
        "contamination_mode": config.contamination_mode,
        "fail_on_contamination_violation": config.fail_on_contamination_violation,
        "tool_active_during_alignment": config.tool_active_during_alignment,
        "tool_active_during_transitions": config.tool_active_during_transitions,
        "tool_active_during_rotation": config.tool_active_during_rotation,
        "same_step_tool_before_chassis": config.same_step_tool_before_chassis,
        "contamination_checked": True,
        "contamination_free": len(events) == 0,
        "temporal_cleaning_valid": len(events) == 0,
        "contamination_violation_count": len(events),
        "contamination_violation_area_m2": sum(event.overlap_area_m2 for event in events),
        "first_contamination_step": first.time_step if first is not None else "NA",
        "first_contamination_segment_id": first.segment_id if first is not None else "NA",
        "cleaned_area_m2": len(cleaned_at_step) * grid.cell_area_m2,
        "chassis_after_cleaned_area_m2": len(chassis_after_cleaned_cells_seen) * grid.cell_area_m2,
        "tool_reclean_area_m2": len(tool_reclean_cells_seen) * grid.cell_area_m2,
        "route_order_strategy": selected_strategy,
        "route_order_strategy_candidates_evaluated": strategies_evaluated,
        "events": events,
        "timeline": timeline,
        "cleaned_cells": set(cleaned_at_step),
    }


# ── 포즈 행 주석기(annotators) / Pose-row annotators (fill CSV-facing fields) ──


def _sample_bbox(samples: list[FootprintSample]) -> tuple[float, float, float, float]:
    """샘플들의 결합 경계 상자. / Combined bbox over a list of samples."""
    return (
        min(sample.combined_bbox_x_min_m for sample in samples),
        max(sample.combined_bbox_x_max_m for sample in samples),
        min(sample.combined_bbox_y_min_m for sample in samples),
        max(sample.combined_bbox_y_max_m for sample in samples),
    )


def _sample_violation_reason(samples: list[FootprintSample], default: str) -> str:
    """샘플들의 고유 위반 사유를 '+'로 합치고, 없으면 default 반환.

    Join distinct non-OK violation reasons with '+', else return default.
    """
    reasons = sorted(
        {
            sample.violation_reason
            for sample in samples
            if sample.violation_reason != "OK"
        }
    )
    return "+".join(reasons) if reasons else default


def annotate_swept_volume_fields(
    config: SideToolPlanConfig,
    poses: list[dict[str, float | int | str | bool]],
) -> None:
    """각 포즈 dict에 스위프 볼륨 검증 필드를 in-place로 채운다(부수효과).

    세그먼트별로 회전/이동 샘플의 경계 상자와 위반 여부를 집계해 pose 키에 쓴다.
    검증 'off'면 통과 플래그만 설정. **poses를 직접 변형**한다.

    Fill swept-volume validation fields into each pose dict in place (bbox,
    within-boundary flags, violation counts). Mutates `poses`.
    """
    if config.swept_volume_validation == "off":
        for pose in poses:
            pose["swept_volume_within_boundary"] = True
            pose["swept_volume_violation_reason"] = "SWEEP_VALIDATION_OFF"
        return
    samples = geometry_samples_for_path(config, poses)
    samples_by_segment: dict[str, list[FootprintSample]] = {}
    for sample in samples:
        samples_by_segment.setdefault(sample.segment_id, []).append(sample)

    for pose in poses:
        segment_samples = samples_by_segment.get(str(pose["segment_id"]), [])
        rotation_samples = [sample for sample in segment_samples if sample.sample_type == "rotation"]
        translation_samples = [sample for sample in segment_samples if sample.sample_type != "rotation"]
        if segment_samples:
            bbox = _sample_bbox(segment_samples)
            pose["combined_bbox_x_min_m"] = bbox[0]
            pose["combined_bbox_x_max_m"] = bbox[1]
            pose["combined_bbox_y_min_m"] = bbox[2]
            pose["combined_bbox_y_max_m"] = bbox[3]
        if rotation_samples:
            rbbox = _sample_bbox(rotation_samples)
            pose["rotation_sample_count"] = len(rotation_samples)
            pose["rotation_swept_x_min_m"] = rbbox[0]
            pose["rotation_swept_x_max_m"] = rbbox[1]
            pose["rotation_swept_y_min_m"] = rbbox[2]
            pose["rotation_swept_y_max_m"] = rbbox[3]
            pose["rotation_swept_chassis_within_boundary"] = all(
                sample.chassis_inside_workspace for sample in rotation_samples
            )
            pose["rotation_swept_tool_within_boundary"] = all(
                sample.tool_inside_workspace for sample in rotation_samples
            )
            pose["rotation_swept_combined_within_boundary"] = all(
                sample.inside_workspace for sample in rotation_samples
            )
            pose["rotation_swept_violation_count"] = sum(
                1 for sample in rotation_samples if not sample.inside_workspace
            )
            pose["rotation_swept_violation_reason"] = _sample_violation_reason(
                rotation_samples,
                "OK",
            )
        if translation_samples:
            pose["translation_sample_count"] = len(translation_samples)
            pose["translation_swept_chassis_within_boundary"] = all(
                sample.chassis_inside_workspace for sample in translation_samples
            )
            pose["translation_swept_tool_within_boundary"] = all(
                sample.tool_inside_workspace for sample in translation_samples
            )
            pose["translation_swept_combined_within_boundary"] = all(
                sample.inside_workspace for sample in translation_samples
            )
            pose["translation_swept_violation_count"] = sum(
                1 for sample in translation_samples if not sample.inside_workspace
            )
        swept_ok = (
            bool(pose["rotation_swept_combined_within_boundary"])
            and bool(pose["translation_swept_combined_within_boundary"])
        )
        pose["swept_volume_within_boundary"] = swept_ok
        if not swept_ok:
            pose["swept_volume_violation_reason"] = _sample_violation_reason(
                segment_samples,
                "ROTATION_SWEEP_NOT_VALIDATED",
            )


def annotate_contamination_fields(
    config: SideToolPlanConfig,
    poses: list[dict[str, float | int | str | bool]],
) -> None:
    """각 포즈 dict에 오염 분석 결과(세그먼트 단위 집계)를 in-place로 채운다.

    타임라인/사건을 세그먼트별로 묶어 청소 추가 면적, 재청소 면적, 위반 수 등을
    pose 키에 쓴다. **poses를 직접 변형**한다.

    Fill per-segment contamination results into each pose dict in place
    (cleaned-area added, reclean area, violation counts). Mutates `poses`.
    """
    analysis = contamination_analysis(config, poses)
    timeline = list(analysis["timeline"])
    events = list(analysis["events"])
    timeline_by_segment: dict[str, list[TimelineState]] = {}
    for state in timeline:
        timeline_by_segment.setdefault(state.segment_id, []).append(state)
    events_by_segment: dict[str, list[ContaminationEvent]] = {}
    for event in events:
        events_by_segment.setdefault(event.segment_id, []).append(event)

    for pose in poses:
        segment_id = str(pose["segment_id"])
        states = timeline_by_segment.get(segment_id, [])
        segment_events = events_by_segment.get(segment_id, [])
        if states:
            time_start = states[0].time_step
            time_end = states[-1].time_step
            cleaned_start = states[0].cleaned_area_m2
            cleaned_end = states[-1].cleaned_area_m2
            tool_reclean_area = sum(state.current_tool_reclean_area_m2 for state in states)
            chassis_after_cleaned = sum(state.prior_cleaned_chassis_overlap_area_m2 for state in states)
            tool_active = any(state.tool_active for state in states)
        else:
            time_start = "NA"
            time_end = "NA"
            cleaned_start = 0.0
            cleaned_end = 0.0
            tool_reclean_area = 0.0
            chassis_after_cleaned = 0.0
            tool_active = pose["primitive_type"] != "transition" or config.tool_active_during_transitions
        violation_area = sum(event.overlap_area_m2 for event in segment_events)
        pose["time_start_index"] = time_start
        pose["time_end_index"] = time_end
        pose["contamination_checked"] = bool(analysis["contamination_checked"])
        pose["contamination_free_segment"] = not segment_events
        pose["contamination_violation_count"] = len(segment_events)
        pose["contamination_violation_area_m2"] = violation_area
        pose["contamination_free_primitive"] = not segment_events
        pose["cleaned_area_added_m2"] = max(0.0, cleaned_end - cleaned_start)
        pose["tool_reclean_area_m2"] = tool_reclean_area
        pose["chassis_on_prior_cleaned_area_m2"] = chassis_after_cleaned
        pose["tool_active"] = tool_active


# ── 운동학 주석 / Kinematics annotation (differential-drive feasibility) ──


def _movement_kinematics(
    current: dict[str, float | int | str | bool],
    nxt: dict[str, float | int | str | bool] | None,
) -> tuple[float, float, float, str, bool, str, str]:
    """두 연속 포즈로부터 이동 프리미티브와 미분구동 실현가능성을 판정.

    반환: (거리, 헤딩변화, 회전량, 진행방향, 실현가능, 사유, 서브타입).
    핵심 규약: 이동과 회전은 동시에 불가(비홀로노믹) — 거리가 있으면서 헤딩이
    바뀌면 위반. 옆으로 미끄러지는 이동도 금지. 진행방향이 헤딩과(또는 +180°와)
    정렬돼야 전진/후진으로 분류된다.

    Classify the movement primitive and differential-drive feasibility from two
    consecutive poses. Returns (distance, heading_delta, rotation, travel_dir,
    feasible, reason, subtype). Simultaneous move+turn and sideways motion are
    infeasible; travel must align with heading (or heading+180°).
    """
    if nxt is None:
        return 0.0, 0.0, 0.0, float(current["heading_deg"]), True, "OK", "move_forward"
    dx = float(nxt["x_m"]) - float(current["x_m"])
    dy = float(nxt["y_m"]) - float(current["y_m"])
    distance = math.hypot(dx, dy)
    heading = float(current["heading_deg"])
    heading_delta = _normalize_angle_deg(float(nxt["heading_deg"]) - heading)
    rotation = abs(heading_delta)
    if distance <= 1e-9:
        if rotation <= 1e-9:
            return 0.0, heading_delta, 0.0, heading, True, "OK", "move_forward"
        primitive = "rotate_left" if heading_delta > 0.0 else "rotate_right"
        return 0.0, heading_delta, rotation, heading, True, "OK", primitive

    travel_direction = _normalize_angle_deg(math.degrees(math.atan2(dy, dx)))
    if rotation > 1e-6:
        return (
            distance,
            heading_delta,
            rotation,
            travel_direction,
            False,
            "NONHOLONOMIC_CONSTRAINT_VIOLATION",
            "invalid_motion",
        )

    forward_error = abs(_normalize_angle_deg(travel_direction - heading))
    reverse_error = abs(_normalize_angle_deg(travel_direction - heading - 180.0))
    if min(forward_error, reverse_error) > 1e-5:
        reason = "TRAVEL_DIRECTION_NOT_ALIGNED_WITH_HEADING"
        if abs(dx) <= 1e-9 and abs(_normalize_angle_deg(heading)) in {0.0, 180.0}:
            reason = "SIDEWAYS_MOTION_NOT_ALLOWED"
        return distance, heading_delta, rotation, travel_direction, False, reason, "invalid_motion"

    subtype = "move_forward" if forward_error <= reverse_error else "move_backward"
    return distance, heading_delta, rotation, travel_direction, True, "OK", subtype


def annotate_kinematic_fields(
    config: SideToolPlanConfig,
    poses: list[dict[str, float | int | str | bool]],
) -> None:
    """각 포즈 dict에 운동학 필드(거리·회전·프리미티브·실현가능성)를 in-place로 채운다.

    포즈 i→i+1 전이를 `_movement_kinematics`로 분석해 이동 프리미티브, 시작/끝 좌표,
    실현가능 플래그 등을 pose 키에 쓴다. **poses를 직접 변형**한다.

    Fill kinematic fields (distance, rotation, primitive type, feasibility,
    start/end coords) into each pose dict in place. Mutates `poses`.
    """
    for index, pose in enumerate(poses):
        nxt = poses[index + 1] if index + 1 < len(poses) else None
        (
            distance,
            heading_delta,
            rotation,
            travel_direction,
            feasible,
            reason,
            subtype,
        ) = _movement_kinematics(pose, nxt)
        pose["kinematic_model"] = config.kinematic_model
        pose["segment_distance_m"] = distance
        pose["heading_delta_deg"] = heading_delta
        pose["segment_rotation_deg"] = rotation
        pose["travel_direction_deg"] = travel_direction
        pose["differential_drive_feasible"] = feasible
        pose["kinematic_violation_reason"] = reason
        pose["movement_primitive_type"] = subtype
        pose["primitive_sequence_valid"] = feasible
        pose["invalid_motion_reason"] = reason
        pose["primitive_subtype"] = subtype
        pose["primitive_index"] = index
        pose["distance_m"] = distance
        pose["angle_deg"] = rotation
        pose["starts_at_x_m"] = float(pose["x_m"])
        pose["starts_at_y_m"] = float(pose["y_m"])
        pose["starts_at_heading_deg"] = float(pose["heading_deg"])
        pose["ends_at_x_m"] = float(nxt["x_m"]) if nxt is not None else float(pose["x_m"])
        pose["ends_at_y_m"] = float(nxt["y_m"]) if nxt is not None else float(pose["y_m"])
        pose["ends_at_heading_deg"] = float(nxt["heading_deg"]) if nxt is not None else float(pose["heading_deg"])
        pose["contamination_free_primitive"] = True
        if distance > 1e-9 and feasible:
            heading = float(pose["heading_deg"])
            pose["motion_direction"] = (
                "forward"
                if abs(_normalize_angle_deg(float(travel_direction) - heading))
                <= abs(_normalize_angle_deg(float(travel_direction) - heading - 180.0))
                else "reverse"
            )
            pose["direction"] = pose["motion_direction"]


def _chassis_center_from_tool(
    config: SideToolPlanConfig,
    *,
    tool_x_m: float,
    tool_y_m: float,
    heading_deg: float,
) -> tuple[float, float]:
    """툴 중심에서 섀시 중심을 역산(측면 오프셋만큼 반대로 이동).

    `_tool_pose`의 역연산. 툴 우선 계획에서 섀시 지지 경로를 유도할 때 쓴다.
    Inverse of _tool_pose: shift back by the lateral offset to get the chassis
    center from a tool center (used to derive the support path in tool-first).
    """
    side = _side_sign(config.tool_side)
    normal_x, normal_y = _left_normal(heading_deg)
    return (
        tool_x_m - normal_x * side * config.tool_lateral_offset_m,
        tool_y_m - normal_y * side * config.tool_lateral_offset_m,
    )


def annotate_tool_primary_fields(
    config: SideToolPlanConfig,
    poses: list[dict[str, float | int | str | bool]],
) -> None:
    """툴을 "주(primary) 경로"로 삼는 필드를 각 포즈 dict에 in-place로 채운다.

    툴 세그먼트 라벨/타입, 툴 시작·끝 좌표, 연결자 각도·거리, 그리고 섀시가 툴에서
    올바르게 역산되는지(`chassis_path_derived_from_tool`)를 검증해 기록한다.
    **poses를 직접 변형**한다.

    Fill "tool-as-primary-path" fields (tool labels/types, tool start/end,
    connector angle/distance, chassis-derived-from-tool check) into each pose
    dict in place. Mutates `poses`.
    """
    for index, pose in enumerate(poses):
        nxt = poses[index + 1] if index + 1 < len(poses) else None
        tool_start_x = float(pose["tool_x_m"])
        tool_start_y = float(pose["tool_y_m"])
        tool_end_x = float(nxt["tool_x_m"]) if nxt is not None else tool_start_x
        tool_end_y = float(nxt["tool_y_m"]) if nxt is not None else tool_start_y
        dx = tool_end_x - tool_start_x
        dy = tool_end_y - tool_start_y
        connector_distance = math.hypot(dx, dy)
        connector_angle = math.degrees(math.atan2(dy, dx)) if connector_distance > 1e-9 else 0.0
        if pose["segment_type"] == "lane_start":
            tool_segment_type = "tool_sweep_track"
        else:
            tool_segment_type = "tool_zigzag_connector"
        lane_index = int(pose["lane_index"])
        if pose["segment_type"] == "lane_start" and lane_index >= 0:
            label = f"T{lane_index}_start"
        elif pose["segment_type"] == "lane_end" and lane_index >= 0:
            label = f"T{lane_index}_end"
        elif str(pose["segment_id"]).startswith("transition_"):
            label = f"C{lane_index}_to_{int(nxt['lane_index']) if nxt is not None else lane_index}"
        elif str(pose["segment_id"]) == "route_start_A":
            label = "A_tool_staging"
        elif str(pose["segment_id"]) == "route_end_B":
            label = "B_tool_staging"
        else:
            label = str(pose["segment_id"])

        derived_x, derived_y = _chassis_center_from_tool(
            config,
            tool_x_m=tool_start_x,
            tool_y_m=tool_start_y,
            heading_deg=float(pose["heading_deg"]),
        )
        if nxt is not None:
            end_heading = float(nxt["heading_deg"])
            derived_end_x, derived_end_y = _chassis_center_from_tool(
                config,
                tool_x_m=tool_end_x,
                tool_y_m=tool_end_y,
                heading_deg=end_heading,
            )
            expected_end_x = float(nxt["x_m"])
            expected_end_y = float(nxt["y_m"])
        else:
            derived_end_x = derived_x
            derived_end_y = derived_y
            expected_end_x = float(pose["x_m"])
            expected_end_y = float(pose["y_m"])
        derived_ok = (
            abs(derived_x - float(pose["x_m"])) <= 1e-6
            and abs(derived_y - float(pose["y_m"])) <= 1e-6
            and abs(derived_end_x - expected_end_x) <= 1e-6
            and abs(derived_end_y - expected_end_y) <= 1e-6
        )

        pose["tool_route_index"] = index
        pose["tool_segment_id"] = str(pose["segment_id"])
        pose["tool_segment_type"] = tool_segment_type
        pose["tool_start_x_m"] = tool_start_x
        pose["tool_start_y_m"] = tool_start_y
        pose["tool_end_x_m"] = tool_end_x
        pose["tool_end_y_m"] = tool_end_y
        pose["tool_connector_angle_deg"] = connector_angle
        pose["tool_connector_distance_m"] = connector_distance
        pose["tool_waypoint_label"] = label
        pose["tool_path_primary"] = True
        pose["chassis_segment_id"] = str(pose["segment_id"])
        pose["chassis_start_x_m"] = float(pose["x_m"])
        pose["chassis_start_y_m"] = float(pose["y_m"])
        pose["chassis_end_x_m"] = float(nxt["x_m"]) if nxt is not None else float(pose["x_m"])
        pose["chassis_end_y_m"] = float(nxt["y_m"]) if nxt is not None else float(pose["y_m"])
        pose["chassis_heading_deg"] = float(pose["heading_deg"])
        pose["chassis_path_derived_from_tool"] = derived_ok
        pose["primitive_value"] = float(pose["distance_m"]) if str(pose["movement_primitive_type"]).startswith("move_") else float(pose["angle_deg"])


# ── A/B 엔드포인트 판정 / A/B endpoint targets & reachability ──


def _ab_targets_local(info: WorkspaceInfo) -> tuple[tuple[float, float], tuple[float, float]]:
    """로컬 좌표계에서의 시작(A)·끝(B) 목표점을 반환.

    코너/대각선 모드는 A/B를 원점 기준 로컬로 환산하고, 축+폭 모드는 중심선의
    양 끝 (0, centerline)·(길이, centerline)을 목표로 한다.

    Start(A)/end(B) targets in local coords: corner/diagonal modes use A/B
    relative to origin; axis+width modes use the centerline endpoints.
    """
    if info.workspace_mode in {"tool_serpentine_ab", "ab_diagonal_center"}:
        return (
            (info.a_x_m - info.world_origin_x_m, info.a_y_m - info.world_origin_y_m),
            (info.b_x_m - info.world_origin_x_m, info.b_y_m - info.world_origin_y_m),
        )
    if info.workspace_mode == "diagonal_ab":
        return (
            (info.a_x_m - info.world_origin_x_m, info.a_y_m - info.world_origin_y_m),
            (info.b_x_m - info.world_origin_x_m, info.b_y_m - info.world_origin_y_m),
        )
    return (0.0, info.centerline_y_m), (info.ab_length_m, info.centerline_y_m)


def _require_start_at_a(config: SideToolPlanConfig) -> bool:
    """시작점이 정확히 A여야 하는지 결정. / Whether the route must start exactly at A.

    명시 플래그 우선, 없으면 rover-center 모드(diagonal_center/centerline_width)는
    기본 True, 그 외는 require_exact_start.

    Explicit flag wins; else rover-center modes default True, otherwise
    require_exact_start.
    """
    if config.require_start_at_A is not None:
        return config.require_start_at_A
    if config.workspace_mode in {"ab_diagonal_center", "ab_centerline_width"}:
        return True
    return config.require_exact_start


def _require_end_at_b(config: SideToolPlanConfig) -> bool:
    """끝점이 정확히 B여야 하는지 결정(_require_start_at_a와 대칭).

    Whether the route must end exactly at B (mirror of _require_start_at_a).
    """
    if config.require_end_at_B is not None:
        return config.require_end_at_B
    if config.workspace_mode in {"ab_diagonal_center", "ab_centerline_width"}:
        return True
    return config.require_exact_end


def _route_endpoint_status(
    config: SideToolPlanConfig,
    info: WorkspaceInfo,
    poses: list[dict[str, float | int | str | bool]],
) -> dict[str, float | bool | str]:
    """경로의 시작/끝이 A/B에 도달했는지와 오차·실현가능 사유를 계산.

    시작·끝 포즈와 A/B 목표점의 거리를 재고 허용 오차와 비교한다. 요구 사항 위반
    시 사유 문자열을 누적한다. 포즈가 없으면 무한대 오차의 실패 상태를 반환.

    Compute whether the route reaches A/B, the errors, and infeasibility
    reasons. Empty poses → an all-infeasible status with infinite errors.
    """
    start_pose = poses[0] if poses else None
    final_pose = poses[-1] if poses else None
    start_target, end_target = _ab_targets_local(info)
    if start_pose is None or final_pose is None:
        return {
            "start_reaches_A": False,
            "end_reaches_B": False,
            "route_reaches_B": False,
            "start_error_m": float("inf"),
            "end_error_m": float("inf"),
            "start_offset_from_A_m": float("inf"),
            "end_offset_from_B_m": float("inf"),
            "final_distance_to_B_m": float("inf"),
            "route_feasible_for_physical_preview": False,
            "infeasible_reason": "NO_ROUTE_POSES",
        }
    start_offset = math.hypot(
        float(start_pose["x_m"]) - start_target[0],
        float(start_pose["y_m"]) - start_target[1],
    )
    end_offset = math.hypot(
        float(final_pose["x_m"]) - end_target[0],
        float(final_pose["y_m"]) - end_target[1],
    )
    start_reaches = start_offset <= config.max_start_error_m
    end_reaches = end_offset <= config.max_end_error_m
    route_reaches = end_reaches
    reasons: list[str] = []
    if _require_start_at_a(config) and not start_reaches:
        reasons.append("EXACT_START_A_INFEASIBLE")
    if _require_end_at_b(config) and not end_reaches:
        reasons.append("EXACT_END_B_INFEASIBLE")
    if not route_reaches:
        reasons.append("ROUTE_INFEASIBLE_REACH_B_WITHOUT_CONTAMINATION")
    return {
        "start_reaches_A": start_reaches,
        "end_reaches_B": end_reaches,
        "route_reaches_B": route_reaches,
        "start_error_m": start_offset,
        "end_error_m": end_offset,
        "start_offset_from_A_m": start_offset,
        "end_offset_from_B_m": end_offset,
        "final_distance_to_B_m": end_offset,
        "route_feasible_for_physical_preview": not reasons,
        "infeasible_reason": "+".join(reasons) if reasons else "OK",
    }


def annotate_endpoint_fields(
    config: SideToolPlanConfig,
    poses: list[dict[str, float | int | str | bool]],
) -> None:
    """각 포즈 dict에 A/B 도달 여부·오차 필드를 in-place로 채운다.

    Fill A/B reachability and error fields into each pose dict in place.
    Mutates `poses`.
    """
    info = _workspace_info(config)
    status = _route_endpoint_status(config, info, poses)
    for pose in poses:
        pose["start_reaches_A"] = bool(status["start_reaches_A"])
        pose["end_reaches_B"] = bool(status["end_reaches_B"])
        pose["start_error_m"] = float(status["start_error_m"])
        pose["end_error_m"] = float(status["end_error_m"])
        pose["route_reaches_B"] = bool(status["route_reaches_B"])


def workspace_polygon(config: SideToolPlanConfig) -> list[tuple[float, float]]:
    """작업 사각형의 4코너를 월드 좌표로 반환(공개, 시각화용).

    The workspace rectangle's four corners in world coords (public; for plots).
    """
    info = _workspace_info(config)
    return [
        _workspace_to_world(info, 0.0, 0.0),
        _workspace_to_world(info, info.ab_length_m, 0.0),
        _workspace_to_world(info, info.ab_length_m, info.workspace_width_m),
        _workspace_to_world(info, 0.0, info.workspace_width_m),
    ]


def boundary_polygon(config: SideToolPlanConfig) -> list[tuple[float, float]]:
    """경계 폴리곤(= 작업 폴리곤)의 별칭. / Alias of workspace_polygon (boundary)."""
    return workspace_polygon(config)


# ── 툴 스트립 후보: 생성·집합피복 선택 / Tool strips: generation & set-cover ──


def _feasible_tool_center_bounds_for_sign(
    config: SideToolPlanConfig,
    info: WorkspaceInfo,
    tool_world_sign: int,
) -> tuple[float, float]:
    """주어진 툴 배치 부호(위/아래)에 대해 툴 중심 y의 허용 [low, high] 범위.

    툴 폭·섀시 반경·전이 간극·마진을 모두 만족해야 한다. tool_world_sign은 툴이
    섀시의 어느 쪽(폭 +/-)에 놓이는지이며, 이에 따라 오프셋이 범위를 밀고 당긴다.
    범위가 비면(high<low) ValueError.

    Feasible [low, high] for the tool center y for a given placement sign
    (tool above/below chassis), respecting tool width, robot radius, turn
    clearance, and margins. Raises ValueError if empty.
    """
    margin = config.boundary_margin_m
    robot_radius = info.robot_radius_m
    transition_radius = info.transition_clearance_radius_m
    offset_inside = config.tool_lateral_offset_m
    if config.workspace_mode in {"ab_diagonal_center", "ab_centerline_width"} and config.chassis_boundary_mode == "centerline_corridor":
        low = margin + config.tool_width_m / 2.0
        high = info.workspace_width_m - margin - config.tool_width_m / 2.0
        if high < low - 1e-9:
            raise ValueError("NO_FEASIBLE_TOOL_LANE")
        return low, high
    low = max(
        margin + config.tool_width_m / 2.0,
        margin + robot_radius + tool_world_sign * offset_inside,
        margin + transition_radius + tool_world_sign * offset_inside,
    )
    high = min(
        info.workspace_width_m - margin - config.tool_width_m / 2.0,
        info.workspace_width_m - margin - robot_radius + tool_world_sign * offset_inside,
        info.workspace_width_m - margin - transition_radius + tool_world_sign * offset_inside,
    )
    if high < low - 1e-9:
        raise ValueError("NO_FEASIBLE_INWARD_TRANSITION+INSUFFICIENT_INTERNAL_TURN_CLEARANCE")
    return low, high


def _feasible_tool_center_bounds(
    config: SideToolPlanConfig,
    info: WorkspaceInfo,
) -> tuple[float, float]:
    """양쪽 배치 부호를 합친 툴 중심 y의 전체 허용 범위.

    Union of feasible tool-center bounds over both placement signs.
    """
    bounds: list[tuple[float, float]] = []
    for tool_world_sign in (1, -1):
        try:
            bounds.append(_feasible_tool_center_bounds_for_sign(config, info, tool_world_sign))
        except ValueError:
            continue
    if not bounds:
        raise ValueError("NO_FEASIBLE_TOOL_PLACEMENT")
    return min(bound[0] for bound in bounds), max(bound[1] for bound in bounds)


def _feasible_tool_centers(config: SideToolPlanConfig, info: WorkspaceInfo) -> list[float]:
    """레인 간격으로 배치한 실현가능 툴 중심 y 목록.

    row_count가 'auto'면 허용 범위를 lane_spacing으로 채우고 마지막에 high를 보정,
    고정 개수면 그만큼 배치한다.

    Feasible tool-center ys spaced by lane_spacing across the feasible range
    ('auto' fills and snaps the last to high; fixed count places that many).
    """
    low, high = _feasible_tool_center_bounds(config, info)

    centers: list[float] = []
    if config.row_count == "auto":
        y = low
        while y <= high + 1e-9:
            centers.append(min(y, high))
            y += config.lane_spacing_m
        if centers and high - centers[-1] > 1e-6:
            centers.append(high)
        if not centers:
            centers.append(low)
    else:
        y = low
        for _ in range(config.row_count):
            if y <= high + 1e-9:
                centers.append(y)
            y += config.lane_spacing_m
        if len(centers) < config.row_count and not config.allow_uncovered_margins:
            raise ValueError("ROW_COUNT_TOO_HIGH")
    if not centers:
        raise ValueError("NO_FEASIBLE_TOOL_LANE")
    return centers


def _tool_offset_sign(config: SideToolPlanConfig, info: WorkspaceInfo) -> int:
    """월드 기준 툴 오프셋 부호(툴 측 부호 × 헤딩 축 부호).

    World-frame tool-offset sign (tool-side sign × heading-axis sign).
    """
    return _side_sign(config.tool_side) * info.heading_axis_sign


def _heading_axis_sign_for_tool_world_sign(config: SideToolPlanConfig, tool_world_sign: int) -> int:
    """툴 배치 부호로부터 헤딩 축 부호를 역산. / Heading-axis sign from tool placement sign."""
    return tool_world_sign * _side_sign(config.tool_side)


def _heading_deg_for_axis_sign(heading_axis_sign: int) -> float:
    """헤딩 축 부호 → 헤딩 각(0° 또는 180°). / Heading angle (0/180) from axis sign."""
    return 0.0 if heading_axis_sign > 0 else 180.0


def _tool_world_side(tool_world_sign: int) -> str:
    """툴이 섀시의 위/아래 어느 쪽인지 라벨. / Label: tool above/below chassis."""
    return "above_chassis" if tool_world_sign > 0 else "below_chassis"


def _candidate_mode(heading_axis_sign: int, tool_world_sign: int) -> str:
    """후보의 (헤딩, 툴 배치) 조합을 사람이 읽을 라벨로. / Human-readable candidate-mode label."""
    heading_label = "heading_0" if heading_axis_sign > 0 else "heading_180"
    side_label = "tool_above_chassis" if tool_world_sign > 0 else "tool_below_chassis"
    return f"{heading_label}_{side_label}"


def _coverage_grid(info: WorkspaceInfo, resolution_m: float) -> CoverageGridInfo:
    """작업 영역을 균일 셀로 나눈 커버리지 격자를 만든다(준공개, CLI가 호출).

    셀 크기는 요청 해상도를 넘지 않도록 ceil로 셀 수를 정하고, 폭·길이를 정확히
    나눠 실제 셀 크기를 맞춘다.

    Build a uniform coverage grid (quasi-public; used by CLIs). Cell counts use
    ceil so cells never exceed the requested resolution, then sizes divide the
    span exactly.
    """
    nx = max(1, math.ceil(info.ab_length_m / resolution_m))
    ny = max(1, math.ceil(info.workspace_width_m / resolution_m))
    cell_width = info.ab_length_m / nx
    cell_height = info.workspace_width_m / ny
    return CoverageGridInfo(
        nx=nx,
        ny=ny,
        cell_width_m=cell_width,
        cell_height_m=cell_height,
        cell_area_m2=cell_width * cell_height,
        total_cells=nx * ny,
    )


def _required_min_lane_count(config: SideToolPlanConfig, info: WorkspaceInfo) -> int:
    """폭을 툴 폭+간격으로 덮는 데 필요한 최소 레인 수.

    명시값이 있으면 그 값. 폭이 툴 폭 이하면 1레인이면 충분. 커버리지 완전성
    검사(`selected_lane_count_ok`)의 기준.

    Minimum lanes needed to cover the width by tool width + spacing (explicit
    value wins; ≤ tool width → 1). Basis for the coverage-completeness check.
    """
    if config.min_lane_count != "auto":
        return int(config.min_lane_count)
    if info.workspace_width_m <= config.tool_width_m:
        return 1
    return max(1, math.ceil((info.workspace_width_m - config.tool_width_m) / config.lane_spacing_m) + 1)


def _covered_cells_for_tool_strip(
    *,
    info: WorkspaceInfo,
    grid: CoverageGridInfo,
    x_min_m: float,
    x_max_m: float,
    y_min_m: float,
    y_max_m: float,
) -> frozenset[int]:
    """축 정렬 사각 스트립 [x_min,x_max]×[y_min,y_max]이 덮는 셀 집합.

    폴리곤 대신 직사각형 툴 스트립 전용의 빠른 셀 집계(중심점 샘플링).
    Cells covered by an axis-aligned rectangular tool strip (fast path vs the
    polygon rasterizer; center-sampling).
    """
    cells: set[int] = set()
    for ix in range(grid.nx):
        cx = (ix + 0.5) * grid.cell_width_m
        if not _in_interval(cx, x_min_m, x_max_m):
            continue
        for iy in range(grid.ny):
            cy = (iy + 0.5) * grid.cell_height_m
            if _in_interval(cy, y_min_m, y_max_m):
                cells.add(iy * grid.nx + ix)
    return frozenset(cells)


def _cleaning_x_bounds(config: SideToolPlanConfig, info: WorkspaceInfo) -> tuple[float, float]:
    """청소 레인이 커버할 x(길이 방향) [start, end] 범위.

    centerline_width는 마진+툴 반길이만큼 양끝을 들여쓰고, 그 외는 미리 계산된
    endpoint_inset을 쓴다.

    X range for cleaning lanes; centerline_width insets by margin+half tool
    length, others use the precomputed endpoint_inset.
    """
    if config.workspace_mode == "ab_centerline_width":
        inset = config.boundary_margin_m + tool_length_m(config) / 2.0
        if inset * 2.0 >= info.ab_length_m:
            raise ValueError("INSUFFICIENT_ENDPOINT_CLEARANCE")
        return inset, info.ab_length_m - inset
    return info.endpoint_inset_m, info.ab_length_m - info.endpoint_inset_m


def _candidate_tool_centers_for_sign(
    config: SideToolPlanConfig,
    info: WorkspaceInfo,
    tool_world_sign: int,
) -> list[float]:
    """주어진 배치 부호에 대한 후보 툴 중심 y 목록(모드별 특수점 포함).

    기본은 lane_spacing 격자. centerline_width면 중심선 정렬 후보를, diagonal_center면
    양 끝 섀시 y(0, width)에 대응하는 후보를 추가해 A/B 정확 도달을 돕는다.
    중복은 반올림 후 제거·정렬한다.

    Candidate tool-center ys for a placement sign, seeded with mode-specific
    points (centerline for centerline_width; endpoint chassis ys for
    diagonal_center) plus a lane_spacing grid; deduped and sorted.
    """
    low, high = _feasible_tool_center_bounds_for_sign(config, info, tool_world_sign)
    centers: list[float] = []
    if config.workspace_mode == "ab_centerline_width":
        centerline_tool_center = info.centerline_y_m + tool_world_sign * config.tool_lateral_offset_m
        if _in_interval(centerline_tool_center, low, high):
            centers.append(centerline_tool_center)
    if config.workspace_mode == "ab_diagonal_center":
        for chassis_y in (0.0, info.workspace_width_m):
            endpoint_tool_center = chassis_y + tool_world_sign * config.tool_lateral_offset_m
            if _in_interval(endpoint_tool_center, low, high):
                centers.append(endpoint_tool_center)
    y = low
    while y <= high + 1e-9:
        centers.append(min(y, high))
        y += config.lane_spacing_m
    if centers and high - centers[-1] > 1e-6:
        centers.append(high)
    if not centers:
        centers.append(low)
    return sorted({round(center, 9) for center in centers})


def _generate_tool_strip_candidates(
    config: SideToolPlanConfig,
    info: WorkspaceInfo,
    grid: CoverageGridInfo,
) -> list[ToolStripCandidate]:
    """가능한 모든 툴 스트립 후보를 생성(각 후보 = 배치·헤딩·덮는 셀 집합).

    양쪽 배치 부호 × 각 후보 중심 y에 대해 섀시 y와 툴 가장자리, 덮는 셀을 계산해
    `ToolStripCandidate`로 모은다. 후보가 하나도 없으면 ValueError.

    Generate all tool-strip candidates (each = placement, heading, covered
    cells) over both signs and candidate ys. Raises ValueError if none exist.
    """
    candidates: list[ToolStripCandidate] = []
    x_min, x_max = _cleaning_x_bounds(config, info)
    for tool_world_sign in (-1, 1):
        try:
            centers = _candidate_tool_centers_for_sign(config, info, tool_world_sign)
        except ValueError:
            continue
        heading_axis_sign = _heading_axis_sign_for_tool_world_sign(config, tool_world_sign)
        heading_deg = _heading_deg_for_axis_sign(heading_axis_sign)
        for tool_center_y in centers:
            chassis_y = tool_center_y - tool_world_sign * config.tool_lateral_offset_m
            tool_edge_min_y = tool_center_y - config.tool_width_m / 2.0
            tool_edge_max_y = tool_center_y + config.tool_width_m / 2.0
            cells = _covered_cells_for_tool_strip(
                info=info,
                grid=grid,
                x_min_m=x_min,
                x_max_m=x_max,
                y_min_m=tool_edge_min_y,
                y_max_m=tool_edge_max_y,
            )
            if not cells:
                continue
            candidates.append(
                ToolStripCandidate(
                    candidate_index=len(candidates),
                    tool_center_y_m=tool_center_y,
                    tool_world_sign=tool_world_sign,
                    tool_world_side=_tool_world_side(tool_world_sign),
                    heading_axis_sign=heading_axis_sign,
                    heading_deg=heading_deg,
                    candidate_mode=_candidate_mode(heading_axis_sign, tool_world_sign),
                    chassis_y_m=chassis_y,
                    tool_edge_min_y_m=tool_edge_min_y,
                    tool_edge_max_y_m=tool_edge_max_y,
                    covered_cells=cells,
                )
            )
    if not candidates:
        raise ValueError("NO_FEASIBLE_TOOL_PLACEMENT")
    return candidates


def _select_tool_strip_candidates(
    config: SideToolPlanConfig,
    candidates: list[ToolStripCandidate],
) -> list[tuple[ToolStripCandidate, frozenset[int]]]:
    """탐욕적 집합피복으로 커버리지에 쓸 툴 스트립 부분집합을 고른다.

    각 스트립이 "새로" 덮는 셀 수가 최대인 것을 반복 선택한다(동률이면 섀시가
    중심선에 가깝고 인덱스가 작은 쪽 우선 — 결정적/재현가능). 특수 처리:
    - centerline_width + 엄격 오염 + A/B 정확 요구: 중심선 정렬 후보 하나만 선택.
    - diagonal_center + boustrophedon: 내부 회전 포켓을 확보 못 하는(가장자리에
      너무 가까운) 스트립은 후보에서 제외.
    반환은 (스트립, 그 스트립이 새로 덮는 셀) 목록을 툴 중심 y로 정렬한 것.

    Greedy set-cover selection of tool strips (pick max new coverage; ties →
    closer-to-centerline, lower-index for determinism). Special cases handle
    centerline exact-A/B and diagonal-center internal-pocket feasibility.
    """
    if (
        config.workspace_mode == "ab_centerline_width"
        and config.contamination_mode == "strict"
        and (config.require_start_at_A is not False)
        and (config.require_end_at_B is not False)
    ):
        centerline_candidates = [
            candidate
            for candidate in candidates
            if abs(candidate.chassis_y_m) <= max(1e-6, config.max_start_error_m)
        ]
        if centerline_candidates:
            best = max(
                centerline_candidates,
                key=lambda candidate: (len(candidate.covered_cells), -abs(candidate.tool_center_y_m)),
            )
            return [(best, best.covered_cells)]

    if config.workspace_mode == "ab_diagonal_center" and config.coverage_pattern == "boustrophedon_required":
        # In the real diagonal-center workflow, a selected cleaning strip must
        # also admit an internal lane-change pocket. Strips whose derived
        # chassis center is too close to the rectangle edge can be spatially
        # useful but cannot participate in a differential-drive ㄹ route without
        # an external turn bay.
        info = _workspace_info(config)
        y_low = config.boundary_margin_m + info.transition_clearance_radius_m
        y_high = info.workspace_width_m - config.boundary_margin_m - info.transition_clearance_radius_m
        internal = [
            candidate
            for candidate in candidates
            if _in_interval(candidate.chassis_y_m, y_low, y_high)
        ]
        if internal:
            candidates = internal

    selected: list[tuple[ToolStripCandidate, frozenset[int]]] = []
    covered: set[int] = set()
    max_lanes = None if config.row_count == "auto" else int(config.row_count)
    remaining = list(candidates)
    while remaining and (max_lanes is None or len(selected) < max_lanes):
        best: ToolStripCandidate | None = None
        best_new: frozenset[int] = frozenset()
        for candidate in remaining:
            new_cells = frozenset(candidate.covered_cells - covered)
            if len(new_cells) > len(best_new):
                best = candidate
                best_new = new_cells
            elif len(new_cells) == len(best_new) and best is not None:
                candidate_key = (
                    abs(candidate.chassis_y_m),
                    candidate.tool_center_y_m,
                    candidate.candidate_index,
                )
                best_key = (
                    abs(best.chassis_y_m),
                    best.tool_center_y_m,
                    best.candidate_index,
                )
                if candidate_key < best_key:
                    best = candidate
                    best_new = new_cells
        if best is None or not best_new:
            break
        selected.append((best, best_new))
        covered.update(best.covered_cells)
        remaining = [candidate for candidate in remaining if candidate != best]
    if not selected:
        raise ValueError("NO_FEASIBLE_TOOL_PLACEMENT")
    if max_lanes is not None and len(selected) < max_lanes and not config.allow_uncovered_margins:
        raise ValueError("ROW_COUNT_TOO_HIGH")
    return sorted(selected, key=lambda item: (item[0].tool_center_y_m, item[0].candidate_index))


def _ordered_candidate_sequences(
    config: SideToolPlanConfig,
    selected: list[tuple[ToolStripCandidate, frozenset[int]]],
) -> list[tuple[str, list[tuple[ToolStripCandidate, frozenset[int]]]]]:
    """선택된 스트립들을 route-order 전략에 따라 방문 순서로 배열한다.

    대부분의 명명 전략은 결국 "아래→위" 또는 "위→아래" 정렬 중 하나로 귀결된다
    (전략 이름은 의도 문서화 목적). `route_order == "auto_temporal_safe"`면 여러
    전략을 순서대로 시도할 후보열로 펼치되, boustrophedon이 아니면 각 전략의
    접두(prefix) 길이별 변형까지 생성한다(오염 회피용 부분 커버). 동일 방문열
    (candidate_index 시그니처)은 중복 제거.

    Order selected strips into visitation sequences per route_order. Most named
    strategies reduce to bottom→top or top→bottom; "auto_temporal_safe" expands
    into many candidate sequences (with prefix variants for best-effort),
    deduped by visitation signature.
    """
    bottom_to_top = sorted(selected, key=lambda item: (item[0].tool_center_y_m, item[0].candidate_index))
    top_to_bottom = list(reversed(bottom_to_top))
    fixed = list(selected)
    strategies = {
        "bottom_to_top": bottom_to_top,
        "near_to_far": bottom_to_top,
        "nearest_to_exit": bottom_to_top,
        "A_to_B_frontier": bottom_to_top,
        "wet_frontier_retreat": bottom_to_top,
        "wet_frontier_safe_boustrophedon": top_to_bottom,
        "top_to_bottom_then_exit": top_to_bottom,
        "bottom_to_top_then_exit": bottom_to_top,
        "dry_frontier_preserving": bottom_to_top,
        "dry_corridor_preserving": bottom_to_top,
        "exit_side_to_far": bottom_to_top,
        "snake_from_A_to_B": bottom_to_top,
        "top_to_bottom": top_to_bottom,
        "far_to_near": top_to_bottom,
        "far_side_to_exit": top_to_bottom,
        "farthest_to_exit": top_to_bottom,
        "B_to_A_retreat": top_to_bottom,
        "reverse_snake_to_B": top_to_bottom,
        "tool_lift_transitions": top_to_bottom,
        "clean_from_far_side_to_exit": top_to_bottom,
        "fixed_serpentine": fixed,
    }
    if config.route_order == "auto_temporal_safe":
        ordered: list[tuple[str, list[tuple[ToolStripCandidate, frozenset[int]]]]] = []
        seen: set[tuple[int, ...]] = set()
        for name in (
            "bottom_to_top",
            "top_to_bottom",
            "A_to_B_frontier",
            "B_to_A_retreat",
            "wet_frontier_safe_boustrophedon",
            "top_to_bottom_then_exit",
            "bottom_to_top_then_exit",
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
        ):
            sequence = strategies[name]
            prefix_lengths = (
                [len(sequence)]
                if config.coverage_pattern == "boustrophedon_required"
                else list(range(len(sequence), 0, -1))
            )
            for prefix_len in prefix_lengths:
                candidate_sequence = sequence[:prefix_len]
                signature = tuple(item[0].candidate_index for item in candidate_sequence)
                if signature not in seen:
                    seen.add(signature)
                    suffix = "" if prefix_len == len(sequence) else f"_prefix_{prefix_len}"
                    ordered.append((f"{name}{suffix}", candidate_sequence))
        return ordered
    return [(config.route_order, strategies[config.route_order])]


# ── 툴 포즈·전이 포장·경계 상태 / Tool pose, transition envelopes, boundary status ──


def _tool_pose(
    config: SideToolPlanConfig,
    heading_deg: float,
    chassis_x_m: float,
    chassis_y_m: float,
) -> tuple[float, float, float, float]:
    """섀시 포즈로부터 툴 중심과 툴의 폭 방향 min/max y를 계산.

    반환: (tool_x, tool_y, tool_edge_min_y, tool_edge_max_y). 툴은 좌측 법선 ×
    측 부호 방향으로 lateral_offset만큼 이동한다(`_chassis_center_from_tool`의 역).

    Tool center and tool width-edge min/max y from a chassis pose. Returns
    (tool_x, tool_y, edge_min_y, edge_max_y). Inverse of _chassis_center_from_tool.
    """
    side = _side_sign(config.tool_side)
    normal_x, normal_y = _left_normal(heading_deg)
    tool_x = chassis_x_m + normal_x * side * config.tool_lateral_offset_m
    tool_y = chassis_y_m + normal_y * side * config.tool_lateral_offset_m
    edge_1_y = tool_y - normal_y * side * config.tool_width_m / 2.0
    edge_2_y = tool_y + normal_y * side * config.tool_width_m / 2.0
    return tool_x, tool_y, min(edge_1_y, edge_2_y), max(edge_1_y, edge_2_y)


def _in_interval(value: float, low: float, high: float, eps: float = 1e-9) -> bool:
    """value가 [low, high]에 (부동소수 eps 여유로) 드는지. / Closed-interval test with eps."""
    return low - eps <= value <= high + eps


def _is_transition_segment(segment_type: SegmentType) -> bool:
    """세그먼트가 전이(회전/오프셋 이동 등)인지. / Whether a segment is a transition."""
    return segment_type in {
        "rotate_90",
        "reverse_offset",
        "rotate_back",
        "transition_start",
        "transition_end",
    }


def _transition_pocket_side(info: WorkspaceInfo, segment_type: SegmentType, x_m: float) -> str:
    """전이가 x축 어느 쪽 내부 포켓에서 일어나는지 라벨(중점 기준).

    Which interior pocket side (x_min/x_max) the transition occupies, by midpoint.
    """
    if not _is_transition_segment(segment_type):
        return "none"
    midpoint = info.ab_length_m / 2.0
    return "x_max_interior" if x_m >= midpoint else "x_min_interior"


def _transition_envelope(
    config: SideToolPlanConfig,
    info: WorkspaceInfo,
    segment_type: SegmentType,
    x_m: float,
    y_m: float,
) -> tuple[float, float, float, float, bool, str, str]:
    """한 점 전이의 회전 포장(원반 근사) 상자와 경계 내 여부를 계산.

    전이 세그먼트는 (x,y)를 중심으로 transition_clearance_radius 반경의 원반이
    필요하다고 보고 그 외접 상자가 마진 안쪽 작업 영역에 드는지 검사한다. 외부
    turn bay는 허용하지 않으므로 벗어나면 위반. 비전이는 점 자체 포함만 검사.

    Envelope box (disk approximation) and boundary check for a point transition.
    Transitions need a clearance-radius disk inside the margined workspace
    (external turn bays are disallowed); non-transitions just check the point.
    """
    if not _is_transition_segment(segment_type):
        center_ok = (
            _in_interval(x_m, 0.0, info.ab_length_m)
            and _in_interval(y_m, 0.0, info.workspace_width_m)
        )
        return x_m, x_m, y_m, y_m, center_ok, "none", "OK" if center_ok else "TRANSITION_POINT_OUTSIDE_WORKSPACE"

    radius = info.transition_clearance_radius_m
    x_min = x_m - radius
    x_max = x_m + radius
    y_min = y_m - radius
    y_max = y_m + radius
    margin = config.boundary_margin_m
    pocket_side = _transition_pocket_side(info, segment_type, x_m)
    within = (
        _in_interval(x_min, margin, info.ab_length_m - margin)
        and _in_interval(x_max, margin, info.ab_length_m - margin)
        and _in_interval(y_min, margin, info.workspace_width_m - margin)
        and _in_interval(y_max, margin, info.workspace_width_m - margin)
    )
    reason = (
        "OK"
        if within
        else "TRANSITION_ENVELOPE_OUTSIDE_WORKSPACE+EXTERNAL_TURN_BAY_NOT_ALLOWED"
    )
    return x_min, x_max, y_min, y_max, within, pocket_side, reason


def _transition_envelope_for_shift(
    config: SideToolPlanConfig,
    info: WorkspaceInfo,
    x_m: float,
    y1_m: float,
    y2_m: float,
) -> tuple[float, float, float, float, bool, str]:
    """레인 y1→y2 측면 이동 전이의 포장 상자와 경계 내 여부.

    이동 구간 [y1,y2] 양끝에 전이 반경을 더한 상자가 마진 안 작업 영역에 드는지
    검사(외부 turn bay 금지).

    Envelope box and boundary check for a lateral y1→y2 shift transition (band
    plus clearance radius must stay inside the margined workspace).
    """
    radius = info.transition_clearance_radius_m
    margin = config.boundary_margin_m
    x_min = x_m - radius
    x_max = x_m + radius
    y_min = min(y1_m, y2_m) - radius
    y_max = max(y1_m, y2_m) + radius
    within = (
        _in_interval(x_min, margin, info.ab_length_m - margin)
        and _in_interval(x_max, margin, info.ab_length_m - margin)
        and _in_interval(y_min, margin, info.workspace_width_m - margin)
        and _in_interval(y_max, margin, info.workspace_width_m - margin)
    )
    reason = "OK" if within else "TRANSITION_ENVELOPE_OUTSIDE_WORKSPACE+EXTERNAL_TURN_BAY_NOT_ALLOWED"
    return x_min, x_max, y_min, y_max, within, reason


def _transition_primitive_candidates(
    current: ToolStripCandidate,
    nxt: ToolStripCandidate,
) -> list[str]:
    """두 레인 사이에서 시도할 전이 프리미티브 후보 목록.

    두 레인의 헤딩 축 부호가 같으면 회전-이동-회전/짧은 접기 계열, 다르면 헤딩
    뒤집기/역방향 접기 계열을 제시하고, 최후 수단으로 side_step_reverse_90 추가.

    Candidate transition primitives between two lanes (same heading axis →
    rotate-drive-rotate/fold; different → heading-flip/reverse-fold), always
    plus side_step_reverse_90 as fallback.
    """
    primitives: list[str] = []
    if current.heading_axis_sign == nxt.heading_axis_sign:
        primitives.append("rotate_drive_rotate_shift")
        primitives.append("short_internal_fold")
    else:
        primitives.append("heading_flip_inside")
        primitives.append("reverse_internal_fold")
    primitives.append("side_step_reverse_90")
    return primitives


def _transition_cost(
    primitive: str,
    current: ToolStripCandidate,
    nxt: ToolStripCandidate,
    x_m: float,
) -> float:
    """전이 프리미티브의 비용(작을수록 선호). / Cost of a transition primitive (lower=better).

    측면 이동량 + 헤딩 변화(가중) + 프리미티브별 페널티 + 끝단 근접 페널티의 합.
    페널티 표는 단순/자연스러운 회전을 복잡한 후진 기동보다 선호하도록 설정됨.

    Sum of lateral shift + weighted heading change + per-primitive penalty +
    endpoint-proximity penalty; the table favors simple turns over complex
    reversing maneuvers.
    """
    lateral = abs(nxt.chassis_y_m - current.chassis_y_m)
    heading_change = abs(_normalize_angle_deg(nxt.heading_deg - current.heading_deg))
    primitive_penalty = {
        "rotate_drive_rotate_shift": 0.0,
        "short_internal_fold": 0.12,
        "heading_flip_inside": 0.20,
        "reverse_internal_fold": 0.30,
        "side_step_reverse_90": 0.65,
    }[primitive]
    endpoint_penalty = 0.02 * min(x_m, abs(x_m))
    return lateral + 0.002 * heading_change + primitive_penalty + endpoint_penalty


def _select_transition_choice(
    config: SideToolPlanConfig,
    info: WorkspaceInfo,
    current: ToolStripCandidate,
    nxt: ToolStripCandidate,
    x_m: float,
) -> TransitionChoice:
    """후보 전이들 중 최소 비용의 실현가능 전이를 선택.

    스타일이 side_step_reverse_90이면 그 하나만 고려. 경계 밖 포장은 거대한 비용을
    더해 사실상 배제한다(그래도 전부 밖이면 최소값이 선택되지만, fail 플래그가
    켜져 있으면 ValueError). 부수효과 없음.

    Pick the min-cost feasible transition. Out-of-bounds envelopes get a huge
    cost so they lose; if all fail and fail_on_boundary_violation is set, raise.
    """
    style = _normalize_transition_style(config.transition_style)
    primitives = (
        ["side_step_reverse_90"]
        if style == "side_step_reverse_90"
        else _transition_primitive_candidates(current, nxt)
    )
    best: TransitionChoice | None = None
    for primitive in primitives:
        x_min, x_max, y_min, y_max, within, reason = _transition_envelope_for_shift(
            config,
            info,
            x_m,
            current.chassis_y_m,
            nxt.chassis_y_m,
        )
        cost = _transition_cost(primitive, current, nxt, x_m)
        if not within:
            cost += 1_000_000.0
        choice = TransitionChoice(
            primitive=primitive,
            candidate_count=len(primitives),
            cost=cost,
            envelope_x_min_m=x_min,
            envelope_x_max_m=x_max,
            envelope_y_min_m=y_min,
            envelope_y_max_m=y_max,
            envelope_within_boundary=within,
            violation_reason=reason,
        )
        if best is None or choice.cost < best.cost:
            best = choice
    if best is None:
        raise ValueError("NO_FEASIBLE_INTERNAL_TRANSITION")
    if not best.envelope_within_boundary and config.fail_on_boundary_violation:
        raise ValueError(f"NO_FEASIBLE_INTERNAL_TRANSITION:{best.violation_reason}")
    return best


def _merge_intervals(intervals: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """겹치는 1D 구간들을 병합. / Merge overlapping 1D intervals (커버리지 길이 계산용)."""
    if not intervals:
        return []
    sorted_intervals = sorted((min(a, b), max(a, b)) for a, b in intervals)
    merged = [sorted_intervals[0]]
    for low, high in sorted_intervals[1:]:
        current_low, current_high = merged[-1]
        if low <= current_high + 1e-9:
            merged[-1] = (current_low, max(current_high, high))
        else:
            merged.append((low, high))
    return merged


def _interval_total_length(intervals: list[tuple[float, float]]) -> float:
    """구간들의 총 길이 합. / Total covered length of intervals."""
    return sum(max(0.0, high - low) for low, high in intervals)


# ── 경계 상태 종합 / Combined boundary status per pose ──


def _boundary_status(
    config: SideToolPlanConfig,
    info: WorkspaceInfo,
    segment_type: SegmentType,
    primitive_type: str,
    x_m: float,
    y_m: float,
    tool_x_m: float,
    tool_y_m: float,
    tool_edge_min_y_m: float,
    tool_edge_max_y_m: float,
) -> tuple[bool, bool, bool, str]:
    """한 포즈의 섀시·툴 경계 준수 상태를 종합 판정.

    반환: (전체 OK, 섀시 OK, 툴 OK, 사유문자열). boundary_mode='none'이면 무조건
    통과. 섀시 중심 허용 여부와 활성 툴 청소면 포함을 검사해 위반 사유를 모은다.

    Combined boundary status for one pose: (all_ok, chassis_ok, tool_ok, reason).
    'none' mode passes; otherwise checks chassis-center allowance and active-tool
    clean-surface containment.
    """
    if config.boundary_mode == "none":
        return True, True, True, "OK"

    margin = config.boundary_margin_m
    r = info.robot_radius_m
    reasons: list[str] = []
    chassis_center_ok = _chassis_allowed_in_workspace(config, info, x_m=x_m, y_m=y_m)
    if not chassis_center_ok:
        reasons.append(
            "TRANSITION_POINT_OUTSIDE_WORKSPACE"
            if _is_transition_segment(segment_type)
            else "CHASSIS_CENTER_OUTSIDE_WORKSPACE"
        )
    chassis_footprint_ok = _chassis_allowed_in_workspace(config, info, x_m=x_m, y_m=y_m)
    if not chassis_footprint_ok:
        reasons.append("CHASSIS_FOOTPRINT_OUTSIDE_WORKSPACE")

    tool_center_ok = _tool_clean_surface_ok(
        config,
        info,
        segment_type=segment_type,
        primitive_type=primitive_type,
        tool_x_m=tool_x_m,
        tool_y_m=tool_y_m,
        tool_edge_min_y_m=tool_edge_min_y_m,
        tool_edge_max_y_m=tool_edge_max_y_m,
    )
    if not tool_center_ok:
        reasons.append("TOOL_CENTER_OUTSIDE_WORKSPACE")
    tool_edge_ok = _tool_clean_surface_ok(
        config,
        info,
        segment_type=segment_type,
        primitive_type=primitive_type,
        tool_x_m=tool_x_m,
        tool_y_m=tool_y_m,
        tool_edge_min_y_m=tool_edge_min_y_m,
        tool_edge_max_y_m=tool_edge_max_y_m,
    )
    if not tool_edge_ok:
        reasons.append("TOOL_EDGE_OUTSIDE_WORKSPACE")
    chassis_ok = chassis_center_ok and chassis_footprint_ok
    tool_ok = tool_center_ok and tool_edge_ok

    within = not reasons
    return within, chassis_ok, tool_ok, "+".join(reasons) if reasons else "OK"


def _pose_dict(
    *,
    config: SideToolPlanConfig,
    info: WorkspaceInfo,
    segment_index: int,
    segment_id: str,
    segment_type: SegmentType,
    lane_index: int,
    travel_sign: int,
    x_m: float,
    chassis_y_m: float,
    heading_deg: float,
    heading_axis_sign: int,
    tool_world_side: str,
    candidate_mode: str,
    primitive_type: str,
    selected_transition_primitive: str,
    coverage_role: str,
    coverage_cells_added: int,
    coverage_area_added_m2: float,
    transition_choice: TransitionChoice | None,
    notes: str,
) -> dict[str, float | int | str | bool]:
    """하나의 경로 세그먼트를 나타내는 넓은 포즈 dict(포즈 행)를 만든다.

    이 dict의 키 집합은 CLI가 CSV로 그대로 내보내는 **핵심 계약**이다. 섀시/툴
    좌표(로컬·월드), 폴리곤 경계 상자, 경계·전이 검사 결과, 커버리지 역할, 각종
    운동학/오염 자리표시자 필드를 채운다. `motor_command_generated`=False.

    리팩토링 주의: 여기 키를 추가/이름변경/삭제하면 CSV 스키마와 시각화, 그리고
    이후 annotate_* 단계가 덮어쓰는 필드들과의 정합이 깨질 수 있다.

    Build one wide "pose row" dict for a path segment — the core CSV-facing
    contract emitted by the CLIs (chassis/tool coords local+world, bboxes,
    boundary/transition checks, coverage role, kinematic/contamination
    placeholders). Do not casually rename/remove keys.
    """
    motion_direction = _motion_direction(travel_sign, heading_axis_sign)
    travel_direction_deg = 0.0 if travel_sign > 0 else 180.0
    tool_x_m, tool_y_m, tool_edge_min_y_m, tool_edge_max_y_m = _tool_pose(
        config, heading_deg, x_m, chassis_y_m
    )
    within, chassis_ok, tool_ok, reason = _boundary_status(
        config,
        info,
        segment_type,
        primitive_type,
        x_m,
        chassis_y_m,
        tool_x_m,
        tool_y_m,
        tool_edge_min_y_m,
        tool_edge_max_y_m,
    )
    (
        transition_envelope_x_min_m,
        transition_envelope_x_max_m,
        transition_envelope_y_min_m,
        transition_envelope_y_max_m,
        transition_envelope_ok,
        transition_pocket_side,
        transition_violation_reason,
    ) = _transition_envelope(config, info, segment_type, x_m, chassis_y_m)
    if transition_choice is not None:
        transition_envelope_x_min_m = transition_choice.envelope_x_min_m
        transition_envelope_x_max_m = transition_choice.envelope_x_max_m
        transition_envelope_y_min_m = transition_choice.envelope_y_min_m
        transition_envelope_y_max_m = transition_choice.envelope_y_max_m
        transition_envelope_ok = transition_choice.envelope_within_boundary
        transition_violation_reason = transition_choice.violation_reason
    if _is_transition_segment(segment_type) and not transition_envelope_ok:
        within = False
        reason = (
            transition_violation_reason
            if reason == "OK"
            else f"{reason}+{transition_violation_reason}"
        )
    world_x, world_y = _workspace_to_world(info, x_m, chassis_y_m)
    tool_world_x, tool_world_y = _workspace_to_world(info, tool_x_m, tool_y_m)
    tool_inner_world_x, tool_inner_world_y = _workspace_to_world(
        info, tool_x_m, tool_edge_min_y_m
    )
    tool_outer_world_x, tool_outer_world_y = _workspace_to_world(
        info, tool_x_m, tool_edge_max_y_m
    )
    chassis_poly = chassis_polygon(config, x_m=x_m, y_m=chassis_y_m, heading_deg=heading_deg)
    tool_poly = tool_polygon(config, chassis_x_m=x_m, chassis_y_m=chassis_y_m, heading_deg=heading_deg)
    chassis_bbox = polygon_bbox(chassis_poly)
    tool_bbox = polygon_bbox(tool_poly)
    combined_bbox = _combined_bbox([chassis_poly, tool_poly])
    return {
        "segment_index": segment_index,
        "index": segment_index,
        "segment_id": segment_id,
        "segment_type": segment_type,
        "primitive_type": primitive_type,
        "primitive_subtype": selected_transition_primitive if primitive_type == "transition" else "drive_straight",
        "primitive_index": segment_index,
        "movement_primitive_type": "move_forward",
        "primitive_sequence_valid": True,
        "invalid_motion_reason": "OK",
        "distance_m": 0.0,
        "angle_deg": 0.0,
        "starts_at_x_m": x_m,
        "starts_at_y_m": chassis_y_m,
        "starts_at_heading_deg": heading_deg,
        "ends_at_x_m": x_m,
        "ends_at_y_m": chassis_y_m,
        "ends_at_heading_deg": heading_deg,
        "contamination_free_primitive": True,
        "selected_transition_primitive": selected_transition_primitive,
        "lane_index": lane_index,
        "direction": motion_direction,
        "motion_direction": motion_direction,
        "travel_direction_deg": travel_direction_deg,
        "kinematic_model": config.kinematic_model,
        "differential_drive_feasible": True,
        "kinematic_violation_reason": "OK",
        "heading_delta_deg": 0.0,
        "segment_distance_m": 0.0,
        "segment_rotation_deg": 0.0,
        "x_m": x_m,
        "y_m": chassis_y_m,
        "axis_m": x_m,
        "lateral_m": chassis_y_m,
        "world_x_m": world_x,
        "world_y_m": world_y,
        "heading_deg": heading_deg,
        "world_heading_deg": _normalize_angle_deg(info.ab_heading_deg + heading_deg),
        "workspace_mode": config.workspace_mode,
        "tool_side": config.tool_side,
        "workspace_side": config.workspace_side,
        "tool_world_side": tool_world_side,
        "candidate_mode": candidate_mode,
        "tool_x_m": tool_x_m,
        "tool_y_m": tool_y_m,
        "tool_world_x_m": tool_world_x,
        "tool_world_y_m": tool_world_y,
        "tool_heading_deg": heading_deg,
        "tool_edge_min_y_m": tool_edge_min_y_m,
        "tool_edge_max_y_m": tool_edge_max_y_m,
        "tool_inner_x_m": tool_x_m,
        "tool_inner_y_m": tool_edge_min_y_m,
        "tool_inner_world_x_m": tool_inner_world_x,
        "tool_inner_world_y_m": tool_inner_world_y,
        "tool_outer_x_m": tool_x_m,
        "tool_outer_y_m": tool_edge_max_y_m,
        "tool_outer_world_x_m": tool_outer_world_x,
        "tool_outer_world_y_m": tool_outer_world_y,
        "tool_lateral_offset_m": config.tool_lateral_offset_m,
        "tool_width_m": config.tool_width_m,
        "tool_length_m": tool_length_m(config),
        "tool_route_index": segment_index,
        "tool_segment_id": segment_id,
        "tool_segment_type": "tool_sweep_track" if coverage_role == "tool_sweep" else "tool_zigzag_connector",
        "tool_start_x_m": tool_x_m,
        "tool_start_y_m": tool_y_m,
        "tool_end_x_m": tool_x_m,
        "tool_end_y_m": tool_y_m,
        "tool_connector_angle_deg": 0.0,
        "tool_connector_distance_m": 0.0,
        "tool_waypoint_label": "",
        "tool_path_primary": True,
        "chassis_segment_id": segment_id,
        "chassis_start_x_m": x_m,
        "chassis_start_y_m": chassis_y_m,
        "chassis_end_x_m": x_m,
        "chassis_end_y_m": chassis_y_m,
        "chassis_heading_deg": heading_deg,
        "chassis_path_derived_from_tool": True,
        "associated_tool_segment_id": segment_id,
        "primitive_value": 0.0,
        "chassis_bbox_x_min_m": chassis_bbox[0],
        "chassis_bbox_x_max_m": chassis_bbox[1],
        "chassis_bbox_y_min_m": chassis_bbox[2],
        "chassis_bbox_y_max_m": chassis_bbox[3],
        "tool_bbox_x_min_m": tool_bbox[0],
        "tool_bbox_x_max_m": tool_bbox[1],
        "tool_bbox_y_min_m": tool_bbox[2],
        "tool_bbox_y_max_m": tool_bbox[3],
        "combined_bbox_x_min_m": combined_bbox[0],
        "combined_bbox_x_max_m": combined_bbox[1],
        "combined_bbox_y_min_m": combined_bbox[2],
        "combined_bbox_y_max_m": combined_bbox[3],
        "lane_spacing_m": config.lane_spacing_m,
        "transition_style": _normalize_transition_style(config.transition_style),
        "transition_envelope_x_min_m": transition_envelope_x_min_m,
        "transition_envelope_x_max_m": transition_envelope_x_max_m,
        "transition_envelope_y_min_m": transition_envelope_y_min_m,
        "transition_envelope_y_max_m": transition_envelope_y_max_m,
        "transition_envelope_within_boundary": transition_envelope_ok,
        "transition_pocket_side": transition_pocket_side,
        "transition_violation_reason": transition_violation_reason,
        "within_boundary": within,
        "chassis_within_boundary": chassis_ok,
        "tool_within_boundary": tool_ok,
        "tool_swept_within_boundary": tool_ok and (
            transition_envelope_ok if _is_transition_segment(segment_type) else True
        ),
        "rotation_sample_count": 0,
        "translation_sample_count": 0,
        "rotation_swept_chassis_within_boundary": True,
        "rotation_swept_tool_within_boundary": True,
        "rotation_swept_combined_within_boundary": True,
        "rotation_swept_violation_count": 0,
        "rotation_swept_violation_reason": "OK",
        "rotation_swept_x_min_m": combined_bbox[0],
        "rotation_swept_x_max_m": combined_bbox[1],
        "rotation_swept_y_min_m": combined_bbox[2],
        "rotation_swept_y_max_m": combined_bbox[3],
        "translation_swept_chassis_within_boundary": True,
        "translation_swept_tool_within_boundary": True,
        "translation_swept_combined_within_boundary": True,
        "translation_swept_violation_count": 0,
        "swept_volume_within_boundary": True,
        "swept_volume_violation_reason": "OK",
        "boundary_violation_reason": reason,
        "coverage_role": coverage_role,
        "coverage_contributes": coverage_role == "tool_sweep",
        "coverage_cells_added": coverage_cells_added,
        "coverage_area_added_m2": coverage_area_added_m2,
        "first_cleaning_heading_aligned": True,
        "start_reaches_A": False,
        "end_reaches_B": False,
        "start_error_m": 0.0,
        "end_error_m": 0.0,
        "route_reaches_B": False,
        "motor_command_generated": False,
        "notes": notes,
    }


# ── 경로 요약·검증 / Path summary & feasibility roll-up ──


def summarize_side_tool_path(
    config: SideToolPlanConfig,
    poses: list[dict[str, float | int | str | bool]],
) -> dict[str, float | int | str | bool]:
    """생성된 포즈열에 대한 커버리지·검증 지표를 하나의 요약 dict로 집계(공개).

    커버리지 비율/면적/겹침, 스위프 볼륨·오염·운동학·경계 위반 수, 레인/전이/연결자
    통계, boustrophedon 유효성, A/B 도달, 그리고 종합 실현가능성
    (`route_feasible_for_physical_preview`)과 그 실패 사유를 계산한다. 미리보기 전용
    (`motor_command_generated`=False). 부수효과 없음(poses를 변형하지 않음).

    Roll up coverage and validation metrics for a pose list into one summary
    dict (coverage ratio/area/overlap, violation counts, lane/transition stats,
    boustrophedon validity, A/B reach, overall feasibility + reason). Preview
    only; does not mutate poses.

    주의(gotcha): `route_feasible_for_physical_preview`는 `workspace_mode ==
    "ab_diagonal_center"`를 강제 조건에 포함한다 — 다른 모드에서는 이 요약 기준상
    실현가능으로 표시되지 않는다.
    Note: overall feasibility here requires workspace_mode == "ab_diagonal_center".
    """
    info = _workspace_info(config)
    grid = _coverage_grid(info, config.coverage_resolution_m)
    lane_rows = [pose for pose in poses if pose["coverage_role"] == "tool_sweep"]
    lane_start_rows = [pose for pose in poses if pose["segment_type"] == "lane_start"]
    tool_edges_low = [float(pose["tool_edge_min_y_m"]) for pose in lane_rows]
    tool_edges_high = [float(pose["tool_edge_max_y_m"]) for pose in lane_rows]
    tool_x_values = [float(pose["tool_x_m"]) for pose in lane_rows]
    actual_low = min(tool_edges_low) if tool_edges_low else 0.0
    actual_high = max(tool_edges_high) if tool_edges_high else 0.0
    swept_x_min = min(tool_x_values) if tool_x_values else 0.0
    swept_x_max = max(tool_x_values) if tool_x_values else 0.0
    requested_low = 0.0
    requested_high = info.workspace_width_m
    uncovered_low = max(0.0, actual_low - requested_low)
    uncovered_high = max(0.0, requested_high - actual_high)
    tool_y_intervals = _merge_intervals(
        [
            (
                max(requested_low, float(pose["tool_edge_min_y_m"])),
                min(requested_high, float(pose["tool_edge_max_y_m"])),
            )
            for pose in lane_rows
        ]
    )
    covered_y_length = _interval_total_length(tool_y_intervals)
    swept_x_span = max(0.0, swept_x_max - swept_x_min)
    requested_area = info.ab_length_m * info.workspace_width_m
    selected_cell_sets = [
        _covered_cells_for_tool_strip(
            info=info,
            grid=grid,
            x_min_m=swept_x_min,
            x_max_m=swept_x_max,
            y_min_m=float(pose["tool_edge_min_y_m"]),
            y_max_m=float(pose["tool_edge_max_y_m"]),
        )
        for pose in lane_start_rows
    ]
    covered_cells = set().union(*selected_cell_sets) if selected_cell_sets else set()
    selected_swept_cell_count = sum(len(cells) for cells in selected_cell_sets)
    overlap_cells_count = max(0, selected_swept_cell_count - len(covered_cells))
    covered_area = len(covered_cells) * grid.cell_area_m2
    uncovered_area = max(0.0, requested_area - covered_area)
    coverage_ratio = covered_area / requested_area if requested_area > 0.0 else 0.0
    overlap_area = overlap_cells_count * grid.cell_area_m2
    coverage_efficiency = (
        len(covered_cells) / selected_swept_cell_count
        if selected_swept_cell_count > 0
        else 0.0
    )
    uncovered_y_gaps: list[tuple[float, float]] = []
    cursor = requested_low
    for low, high in tool_y_intervals:
        if low > cursor + 1e-9:
            uncovered_y_gaps.append((cursor, low))
        cursor = max(cursor, high)
    if cursor < requested_high - 1e-9:
        uncovered_y_gaps.append((cursor, requested_high))
    uncovered_strips_count = len(uncovered_y_gaps)
    if swept_x_min > 0.0:
        uncovered_strips_count += 1
    if swept_x_max < info.ab_length_m:
        uncovered_strips_count += 1
    violation_count = sum(1 for pose in poses if pose["within_boundary"] is not True)
    transition_envelope_violation_count = sum(
        1 for pose in poses if pose["transition_envelope_within_boundary"] is not True
    )
    swept_summary = swept_volume_summary(config, poses)
    contamination = contamination_analysis(config, poses)
    endpoint_status = _route_endpoint_status(config, info, poses)
    kinematic_violation_count = sum(
        1 for pose in poses if pose.get("differential_drive_feasible") is not True
    )
    primitive_sequence_violation_count = sum(
        1 for pose in poses if pose.get("primitive_sequence_valid") is not True
    )
    allowed_movement_primitives = {"move_forward", "move_backward", "rotate_left", "rotate_right"}
    invalid_primitive_name_count = sum(
        1 for pose in poses if str(pose.get("movement_primitive_type")) not in allowed_movement_primitives
    )
    transition_segment_ids = {
        str(pose["segment_id"])
        for pose in poses
        if pose.get("primitive_type") == "transition"
    }
    selected_transition_count = len(transition_segment_ids)
    transition_style = _normalize_transition_style(config.transition_style)
    transition_candidate_count = selected_transition_count * (
        1 if transition_style == "side_step_reverse_90" else 3
    )
    lane_count = len({int(pose["lane_index"]) for pose in lane_rows})
    tool_track_count = len(lane_start_rows)
    tool_connector_ids: list[str] = []
    for pose in poses:
        segment_id = str(pose["segment_id"])
        if segment_id.startswith("transition_") and segment_id not in tool_connector_ids:
            tool_connector_ids.append(segment_id)
    tool_connector_count = len(tool_connector_ids)
    tool_connector_count_ok = tool_connector_count == max(0, tool_track_count - 1)
    tool_path_continuous = all(
        abs(float(poses[index].get("tool_end_x_m", poses[index]["tool_x_m"])) - float(poses[index + 1].get("tool_start_x_m", poses[index + 1]["tool_x_m"]))) <= 1e-6
        and abs(float(poses[index].get("tool_end_y_m", poses[index]["tool_y_m"])) - float(poses[index + 1].get("tool_start_y_m", poses[index + 1]["tool_y_m"]))) <= 1e-6
        for index in range(len(poses) - 1)
    )
    chassis_path_derived_from_tool = all(
        pose.get("chassis_path_derived_from_tool") is True for pose in poses
    )
    connector_angles: list[str] = []
    connector_distances: list[str] = []
    for segment_id in tool_connector_ids:
        rows = [pose for pose in poses if str(pose["segment_id"]) == segment_id]
        if not rows:
            continue
        start_x = float(rows[0].get("tool_start_x_m", rows[0]["tool_x_m"]))
        start_y = float(rows[0].get("tool_start_y_m", rows[0]["tool_y_m"]))
        end_x = float(rows[-1].get("tool_end_x_m", rows[-1]["tool_x_m"]))
        end_y = float(rows[-1].get("tool_end_y_m", rows[-1]["tool_y_m"]))
        dx = end_x - start_x
        dy = end_y - start_y
        distance = math.hypot(dx, dy)
        angle = math.degrees(math.atan2(dy, dx)) if distance > 1e-9 else 0.0
        connector_angles.append(f"{segment_id}:{angle:.3f}")
        connector_distances.append(f"{segment_id}:{distance:.3f}")
    required_min_lane_count = _required_min_lane_count(config, info)
    selected_lane_count_ok = lane_count >= required_min_lane_count and (
        not config.forbid_single_lane_success or required_min_lane_count <= 1 or lane_count > 1
    )
    lane_start_pose_rows = [pose for pose in poses if pose["segment_type"] == "lane_start"]
    lane_order = [int(pose["lane_index"]) for pose in lane_start_pose_rows]
    lane_start_xs = [float(pose["x_m"]) for pose in lane_start_pose_rows]
    # 왕복(boustrophedon)이면 이웃 레인의 시작 x가 서로 반대편이어야 한다(길이의
    # 절반 이상 차이). 이를 통해 지그재그 방향 교대를 확인한다.
    # In a boustrophedon, adjacent lanes start on opposite ends (>half-length
    # apart) — this verifies the back-and-forth alternation.
    lane_start_alternates = all(
        abs(lane_start_xs[index] - lane_start_xs[index - 1]) > info.ab_length_m / 2.0
        for index in range(1, len(lane_start_xs))
    )
    boustrophedon_pattern_valid = (
        lane_count >= 2
        and lane_order == sorted(lane_order)
        and lane_start_alternates
    )
    if required_min_lane_count <= 1 and lane_count == 1:
        boustrophedon_pattern_valid = True
    try:
        candidate_lane_count = len(_generate_tool_strip_candidates(config, info, grid))
    except ValueError:
        candidate_lane_count = 0
    requested_row_count = config.row_count
    row_count_adjusted = (
        requested_row_count != "auto" and lane_count < int(requested_row_count)
    )
    first_lane = next((pose for pose in poses if pose["segment_type"] == "lane_start"), None)
    lane_axis_heading = 0.0
    first_cleaning_heading = float(first_lane["heading_deg"]) if first_lane is not None else float("nan")
    first_cleaning_heading_aligned = (
        first_lane is not None
        and (
            abs(_normalize_angle_deg(first_cleaning_heading - lane_axis_heading)) <= 1e-6
            or abs(_normalize_angle_deg(first_cleaning_heading - lane_axis_heading - 180.0)) <= 1e-6
        )
    )
    initial_pose_at_a = bool(endpoint_status["start_reaches_A"])
    preclean_alignment_used = any(
        str(pose["segment_id"]) == "route_start_A"
        and pose["segment_type"] == "route_start"
        for pose in poses
    )
    start_offset = float(endpoint_status["start_offset_from_A_m"])
    end_offset = float(endpoint_status["end_offset_from_B_m"])
    route_feasible = (
        bool(endpoint_status["route_feasible_for_physical_preview"])
        and int(contamination["contamination_violation_count"]) == 0
        and kinematic_violation_count == 0
        and primitive_sequence_violation_count == 0
        and invalid_primitive_name_count == 0
        and violation_count == 0
        and int(swept_summary["combined_swept_violation_count"]) == 0
        and selected_lane_count_ok
        and boustrophedon_pattern_valid
        and info.workspace_mode == "ab_diagonal_center"
        and first_cleaning_heading_aligned
        and tool_path_continuous
        and tool_connector_count_ok
        and chassis_path_derived_from_tool
    )
    infeasible_reasons: list[str] = []
    if endpoint_status["infeasible_reason"] != "OK":
        infeasible_reasons.append(str(endpoint_status["infeasible_reason"]))
    if int(contamination["contamination_violation_count"]) > 0:
        infeasible_reasons.append("CONTAMINATION_VIOLATION")
    if kinematic_violation_count > 0:
        infeasible_reasons.append("KINEMATIC_VIOLATION")
    if primitive_sequence_violation_count > 0 or invalid_primitive_name_count > 0:
        infeasible_reasons.append("PRIMITIVE_SEQUENCE_INVALID")
    if violation_count > 0 or int(swept_summary["combined_swept_violation_count"]) > 0:
        infeasible_reasons.append("BOUNDARY_OR_SWEPT_VOLUME_VIOLATION")
    if not selected_lane_count_ok:
        infeasible_reasons.append("SELECTED_LANE_COUNT_BELOW_REQUIRED")
    if not boustrophedon_pattern_valid:
        infeasible_reasons.append("BOUSTROPHEDON_PATTERN_INVALID")
    if info.workspace_mode != "ab_diagonal_center":
        infeasible_reasons.append("WORKSPACE_MODE_NOT_AB_DIAGONAL_CENTER")
    if not first_cleaning_heading_aligned:
        infeasible_reasons.append("DIAGONAL_CLEANING_HEADING_NOT_ALLOWED")
    if not tool_path_continuous:
        infeasible_reasons.append("TOOL_PATH_NOT_CONTINUOUS")
    if not tool_connector_count_ok:
        infeasible_reasons.append("TOOL_CONNECTOR_COUNT_MISMATCH")
    if not chassis_path_derived_from_tool:
        infeasible_reasons.append("CHASSIS_PATH_NOT_DERIVED_FROM_TOOL")
    if (
        config.coverage_pattern == "boustrophedon_required"
        and (not selected_lane_count_ok or not boustrophedon_pattern_valid)
    ):
        infeasible_reasons.append("ROUTE_INFEASIBLE_BOUSTROPHEDON_WITH_CONTAMINATION")
    return {
        "a_x_m": info.a_x_m,
        "a_y_m": info.a_y_m,
        "b_x_m": info.b_x_m,
        "b_y_m": info.b_y_m,
        "planner_objective": "tool_coverage_first",
        "planner_primary_path": "tool_center_path",
        "kinematic_model": config.kinematic_model,
        "differential_drive_feasible": kinematic_violation_count == 0,
        "kinematic_violation_count": kinematic_violation_count,
        "primitive_sequence_valid": primitive_sequence_violation_count == 0 and invalid_primitive_name_count == 0,
        "primitive_sequence_violation_count": primitive_sequence_violation_count,
        "invalid_primitive_name_count": invalid_primitive_name_count,
        "contamination_model": contamination["contamination_model"],
        "contamination_mode": contamination["contamination_mode"],
        "fail_on_contamination_violation": contamination["fail_on_contamination_violation"],
        "allow_start_end_chassis_outside_clean_surface": config.allow_start_end_chassis_outside_clean_surface,
        "staging_allowed": config.allow_start_end_chassis_outside_clean_surface,
        "start_end_chassis_outside_clean_surface_allowed": config.allow_start_end_chassis_outside_clean_surface,
        "tool_active_during_alignment": contamination["tool_active_during_alignment"],
        "tool_active_during_transitions": contamination["tool_active_during_transitions"],
        "tool_active_during_rotation": contamination["tool_active_during_rotation"],
        "visualize_inactive_tool_ghost": config.visualize_inactive_tool_ghost,
        "inactive_tool_ghost_visualized": config.visualize_inactive_tool_ghost,
        "show_transition_envelope": config.show_transition_envelope,
        "contamination_free": contamination["contamination_free"],
        "contamination_violation_count": contamination["contamination_violation_count"],
        "contamination_violation_area_m2": contamination["contamination_violation_area_m2"],
        "first_contamination_step": contamination["first_contamination_step"],
        "first_contamination_segment_id": contamination["first_contamination_segment_id"],
        "temporal_cleaning_valid": contamination["temporal_cleaning_valid"],
        "cleaned_area_m2": contamination["cleaned_area_m2"],
        "chassis_after_cleaned_area_m2": contamination["chassis_after_cleaned_area_m2"],
        "tool_reclean_area_m2": contamination["tool_reclean_area_m2"],
        "rotation_tool_swept_area_m2": 0.0,
        "rotation_tool_active_area_m2": 0.0,
        "rotation_cleaning_cells_count": 0,
        "route_order_strategy": contamination["route_order_strategy"],
        "route_order_strategy_candidates_evaluated": contamination["route_order_strategy_candidates_evaluated"],
        "selected_route_order_strategy": contamination["route_order_strategy"],
        "initial_pose_at_A": initial_pose_at_a,
        "initial_alignment_heading_deg": config.start_heading_deg,
        "lane_axis_heading_deg": lane_axis_heading,
        "first_cleaning_heading_deg": first_cleaning_heading if first_lane is not None else "NA",
        "first_cleaning_heading_aligned": first_cleaning_heading_aligned,
        "preclean_alignment_used": preclean_alignment_used,
        "coverage_pattern": config.coverage_pattern,
        "required_min_lane_count": required_min_lane_count,
        "selected_lane_count_ok": selected_lane_count_ok,
        "single_lane_success_forbidden": config.forbid_single_lane_success,
        "boustrophedon_pattern_valid": boustrophedon_pattern_valid,
        "allow_tool_lift": config.allow_tool_lift,
        "reserve_dry_exit_corridor": config.reserve_dry_exit_corridor,
        "start_reaches_A": endpoint_status["start_reaches_A"],
        "end_reaches_B": endpoint_status["end_reaches_B"],
        "route_reaches_B": endpoint_status["route_reaches_B"],
        "start_error_m": endpoint_status["start_error_m"],
        "end_error_m": endpoint_status["end_error_m"],
        "final_distance_to_B_m": endpoint_status["final_distance_to_B_m"],
        "route_feasible_for_physical_preview": route_feasible,
        "infeasible_reason": "+".join(infeasible_reasons) if infeasible_reasons else "OK",
        "A_B_are_rover_center_start_end": info.workspace_mode in {
            "ab_diagonal_center",
            "ab_centerline_width",
        },
        "A_B_are_rectangle_diagonal": info.workspace_mode == "ab_diagonal_center",
        "chassis_boundary_mode": config.chassis_boundary_mode,
        "surface_side": info.surface_side,
        "clean_surface_x_min_m": 0.0,
        "clean_surface_x_max_m": info.ab_length_m,
        "clean_surface_y_min_m": 0.0,
        "clean_surface_y_max_m": info.workspace_width_m,
        "reserved_dry_corridor_area_m2": (
            info.ab_length_m * config.robot_width_m
            if config.reserve_dry_exit_corridor
            else 0.0
        ),
        "unavoidable_uncovered_due_to_exit_corridor_m2": (
            uncovered_area if config.reserve_dry_exit_corridor and not route_feasible else 0.0
        ),
        "dry_corridor_strategy": config.dry_corridor_strategy
        if config.reserve_dry_exit_corridor
        else "disabled",
        "exit_corridor_reaches_B": endpoint_status["route_reaches_B"],
        "tool_lift_used": config.allow_tool_lift and not config.tool_active_during_transitions,
        "dry_corridor_unpainted_area_m2": (
            uncovered_area if config.reserve_dry_exit_corridor and not route_feasible else 0.0
        ),
        "coverage_model": "tool_footprint_only",
        "feasibility_model": "chassis_plus_tool_swept_volume",
        "transition_planner": transition_style,
        "side_step_reverse_90_is_candidate_only": transition_style == "auto_internal",
        "chassis_path_role": "derived_support_path",
        "tool_path_continuous": tool_path_continuous,
        "tool_track_count": tool_track_count,
        "tool_connector_count": tool_connector_count,
        "tool_zigzag_connector_count": tool_connector_count,
        "tool_connector_angles_deg": ",".join(connector_angles) if connector_angles else "none",
        "tool_connector_distances_m": ",".join(connector_distances) if connector_distances else "none",
        "chassis_path_derived_from_tool": chassis_path_derived_from_tool,
        "chassis_area_counts_as_cleaned": False,
        "workspace_mode": info.workspace_mode,
        "a_b_semantic": info.a_b_semantic,
        "ab_length_m": info.ab_length_m,
        "diagonal_length_m": info.diagonal_length_m,
        "rectangle_length_m": info.rectangle_length_m,
        "rectangle_width_m": info.rectangle_width_m,
        "workspace_side": info.workspace_side,
        "workspace_width_m": info.workspace_width_m,
        "robot_width_m": config.robot_width_m,
        "robot_length_m": config.robot_length_m if config.robot_length_m is not None else "NA",
        "robot_radius_m": info.robot_radius_m,
        "tool_side": config.tool_side,
        "tool_lateral_offset_m": config.tool_lateral_offset_m,
        "tool_width_m": config.tool_width_m,
        "lane_spacing_m": config.lane_spacing_m,
        "boundary_mode": config.boundary_mode,
        "boundary_violation_count": violation_count,
        "internal_transition_required": True,
        "external_turn_bay_allowed": False,
        "transition_clearance_radius_m": info.transition_clearance_radius_m,
        "transition_candidate_count": transition_candidate_count,
        "selected_transition_count": selected_transition_count,
        "transition_envelope_violation_count": transition_envelope_violation_count,
        "plot_artifact_violation_count": 0,
        "physical_swept_volume_validated": swept_summary["physical_swept_volume_validated"],
        "rotation_sample_deg": config.rotation_sample_deg,
        "translation_sample_m": config.translation_sample_m,
        "tool_length_m": tool_length_m(config),
        "tool_length_source": swept_summary["tool_length_source"],
        "rotation_sample_count": swept_summary["rotation_sample_count"],
        "translation_sample_count": swept_summary["translation_sample_count"],
        "total_geometry_sample_count": swept_summary["total_geometry_sample_count"],
        "rotation_swept_violation_count": swept_summary["rotation_swept_violation_count"],
        "translation_swept_violation_count": swept_summary["translation_swept_violation_count"],
        "combined_swept_violation_count": swept_summary["combined_swept_violation_count"],
        "endpoint_inset_m": info.endpoint_inset_m,
        "start_inset_m": start_offset,
        "end_inset_m": end_offset,
        "start_offset_from_A_m": start_offset,
        "end_offset_from_B_m": end_offset,
        "workspace_x_min_m": 0.0,
        "workspace_x_max_m": info.ab_length_m,
        "workspace_y_min_m": 0.0,
        "workspace_y_max_m": info.workspace_width_m,
        "requested_coverage_y_min_m": requested_low,
        "requested_coverage_y_max_m": requested_high,
        "actual_tool_coverage_y_min_m": actual_low,
        "actual_tool_coverage_y_max_m": actual_high,
        "tool_swept_y_min_m": actual_low,
        "tool_swept_y_max_m": actual_high,
        "tool_swept_x_min_m": swept_x_min,
        "tool_swept_x_max_m": swept_x_max,
        "requested_coverage_area_m2": requested_area,
        "covered_area_m2": covered_area,
        "uncovered_area_m2": uncovered_area,
        "uncovered_cells_count": max(0, grid.total_cells - len(covered_cells)),
        "overlap_area_m2": overlap_area,
        "coverage_efficiency": coverage_efficiency,
        "coverage_grid_resolution_m": config.coverage_resolution_m,
        "uncovered_strips_count": uncovered_strips_count,
        "coverage_resolution_m": config.coverage_resolution_m,
        "tool_coverage_y_min_m": actual_low,
        "tool_coverage_y_max_m": actual_high,
        "declared_tool_coverage_y_min_m": requested_low,
        "declared_tool_coverage_y_max_m": requested_high,
        "uncovered_margin_low_m": uncovered_low,
        "uncovered_margin_high_m": uncovered_high,
        "coverage_ratio": coverage_ratio,
        "row_count": config.row_count,
        "requested_row_count": requested_row_count,
        "actual_lane_count": lane_count,
        "candidate_lane_count": candidate_lane_count,
        "selected_lane_count": lane_count,
        "row_count_adjusted": row_count_adjusted,
        "row_count_warning": "ROW_COUNT_TOO_HIGH" if row_count_adjusted else "OK",
        "allow_uncovered_margins": config.allow_uncovered_margins,
        "path_preview_only": True,
        "preview_only": True,
        "feedback_tracking_ready": False,
        "motor_command_generated": False,
    }


# ── 경로 조립 / Route assembly (poses: start → lanes+transitions → end) ──


def _build_side_tool_path_for_candidates(
    config: SideToolPlanConfig,
    info: WorkspaceInfo,
    grid: CoverageGridInfo,
    selected_candidates: list[tuple[ToolStripCandidate, frozenset[int]]],
    route_order_strategy: str,
    route_order_candidates_evaluated: int,
) -> list[dict[str, float | int | str | bool]]:
    """선택·정렬된 스트립열로부터 전체 포즈열(경로)을 조립한다.

    구조: (rover-center 모드면) A에서의 툴-비활성 정렬 시작 연결자 → 각 스트립의
    청소 레인(왕복 방향 교대) + 레인 간 내부 전이(회전-이동-회전) → (마지막에)
    B로의 툴-비활성 종료 연결자. 이후 annotate_* 로 운동학/툴/엔드포인트 필드를
    채우고 route-order 메타와 첫 레인 헤딩 정렬 플래그를 기록한다.

    Assemble the full pose list from ordered strips: optional tool-inactive
    start connector at A, per-strip cleaning lanes (alternating direction) with
    internal transitions between them, optional tool-inactive end connector to
    B; then run the annotators.
    """
    x_min, x_max = _cleaning_x_bounds(config, info)

    poses: list[dict[str, float | int | str | bool]] = []
    start_target, end_target = _ab_targets_local(info)

    # 내부 헬퍼: 세그먼트 인덱스를 자동 부여하며 포즈 행을 리스트에 추가.
    # Local helper: append a pose row, auto-assigning the running segment index.
    def add_pose(
        *,
        segment_id: str,
        segment_type: SegmentType,
        lane_index: int,
        travel_sign: int,
        x_m: float,
        candidate: ToolStripCandidate,
        heading_deg: float,
        heading_axis_sign: int,
        primitive_type: str,
        selected_transition_primitive: str,
        coverage_role: str,
        coverage_cells_added: int,
        coverage_area_added_m2: float,
        transition_choice: TransitionChoice | None = None,
        notes: str,
    ) -> None:
        poses.append(
            _pose_dict(
                config=config,
                info=info,
                segment_index=len(poses),
                segment_id=segment_id,
                segment_type=segment_type,
                lane_index=lane_index,
                travel_sign=travel_sign,
                x_m=x_m,
                chassis_y_m=candidate.chassis_y_m,
                heading_deg=heading_deg,
                heading_axis_sign=heading_axis_sign,
                tool_world_side=candidate.tool_world_side,
                candidate_mode=candidate.candidate_mode,
                primitive_type=primitive_type,
                selected_transition_primitive=selected_transition_primitive,
                coverage_role=coverage_role,
                coverage_cells_added=coverage_cells_added,
                coverage_area_added_m2=coverage_area_added_m2,
                transition_choice=transition_choice,
                notes=notes,
            )
        )

    for lane_index, (candidate, new_cells) in enumerate(selected_candidates):
        # 시작 연결자(rover-center 모드 전용): A의 정확한 로버 중심에서 출발해 툴을
        # 켜지 않은 채(dry) 첫 청소 레인 진입 자세까지 회전·이동으로 도달한다.
        # Start connector (rover-center modes): from the exact A center, drive dry
        # (tool off) via rotate/translate to reach the first cleaning lane pose.
        if lane_index == 0 and config.workspace_mode in {"ab_diagonal_center", "ab_centerline_width"}:
            start_x = x_min if _direction_for_lane(config.first_lane_direction, lane_index) > 0 else x_max
            start_dx = start_x - start_target[0]
            start_dy = candidate.chassis_y_m - start_target[1]
            initial_heading = _normalize_angle_deg(config.start_heading_deg)
            lane_axis_heading = 0.0 if start_dx >= 0.0 else 180.0
            lateral_heading = 90.0 if start_dy >= 0.0 else -90.0
            route_start_candidate = ToolStripCandidate(
                candidate_index=-1,
                tool_center_y_m=start_target[1]
                + candidate.tool_world_sign * config.tool_lateral_offset_m,
                tool_world_sign=candidate.tool_world_sign,
                tool_world_side=candidate.tool_world_side,
                heading_axis_sign=candidate.heading_axis_sign,
                heading_deg=initial_heading,
                candidate_mode="exact_A_centerline_start",
                chassis_y_m=start_target[1],
                tool_edge_min_y_m=0.0,
                tool_edge_max_y_m=0.0,
                covered_cells=frozenset(),
            )
            add_pose(
                segment_id="route_start_A",
                segment_type="route_start",
                lane_index=-1,
                travel_sign=1,
                x_m=start_target[0],
                candidate=route_start_candidate,
                heading_deg=initial_heading,
                heading_axis_sign=candidate.heading_axis_sign,
                primitive_type="transition",
                selected_transition_primitive="tool_inactive_start_connector",
                coverage_role="transition",
                coverage_cells_added=0,
                coverage_area_added_m2=0.0,
                notes="exact rover-center start at A; tool inactive pre-clean alignment",
            )
            if math.hypot(start_dx, start_dy) > 1e-9:
                if abs(_normalize_angle_deg(initial_heading - lane_axis_heading)) > 1e-9:
                    add_pose(
                        segment_id="route_start_A",
                        segment_type="route_start",
                        lane_index=-1,
                        travel_sign=1,
                        x_m=start_target[0],
                        candidate=route_start_candidate,
                        heading_deg=lane_axis_heading,
                        heading_axis_sign=candidate.heading_axis_sign,
                        primitive_type="transition",
                        selected_transition_primitive="tool_inactive_start_connector",
                        coverage_role="transition",
                        coverage_cells_added=0,
                        coverage_area_added_m2=0.0,
                        notes="tool-inactive rotate at A onto rectangle lane axis",
                    )
                if abs(start_dx) > 1e-9:
                    add_pose(
                        segment_id="route_start_A",
                        segment_type="route_start",
                        lane_index=-1,
                        travel_sign=1 if start_dx >= 0.0 else -1,
                        x_m=start_x,
                        candidate=route_start_candidate,
                        heading_deg=lane_axis_heading,
                        heading_axis_sign=candidate.heading_axis_sign,
                        primitive_type="transition",
                        selected_transition_primitive="tool_inactive_start_connector",
                        coverage_role="transition",
                        coverage_cells_added=0,
                        coverage_area_added_m2=0.0,
                        notes="tool-inactive dry staging drive parallel to rectangle lane axis",
                    )
                if abs(start_dy) > 1e-9:
                    add_pose(
                        segment_id="route_start_A",
                        segment_type="route_start",
                        lane_index=-1,
                        travel_sign=1,
                        x_m=start_x,
                        candidate=route_start_candidate,
                        heading_deg=lateral_heading,
                        heading_axis_sign=candidate.heading_axis_sign,
                        primitive_type="transition",
                        selected_transition_primitive="tool_inactive_start_connector",
                        coverage_role="transition",
                        coverage_cells_added=0,
                        coverage_area_added_m2=0.0,
                        notes="tool-inactive rotate before lateral dry staging drive",
                    )
                    add_pose(
                        segment_id="route_start_A",
                        segment_type="route_start",
                        lane_index=-1,
                        travel_sign=1 if start_dy >= 0.0 else -1,
                        x_m=start_x,
                        candidate=candidate,
                        heading_deg=lateral_heading,
                        heading_axis_sign=candidate.heading_axis_sign,
                        primitive_type="transition",
                        selected_transition_primitive="tool_inactive_start_connector",
                        coverage_role="transition",
                        coverage_cells_added=0,
                        coverage_area_added_m2=0.0,
                        notes="tool-inactive dry staging drive to first chassis support lane",
                    )
                add_pose(
                    segment_id="route_start_A",
                    segment_type="route_start",
                    lane_index=-1,
                    travel_sign=1,
                    x_m=start_x,
                    candidate=candidate,
                    heading_deg=candidate.heading_deg,
                    heading_axis_sign=candidate.heading_axis_sign,
                    primitive_type="transition",
                    selected_transition_primitive="tool_inactive_start_connector",
                    coverage_role="transition",
                    coverage_cells_added=0,
                    coverage_area_added_m2=0.0,
                    notes="differential-drive start connector: rotate onto first lane heading",
                )

        travel_sign = _direction_for_lane(config.first_lane_direction, lane_index)
        start_x = x_min if travel_sign > 0 else x_max
        end_x = x_max if travel_sign > 0 else x_min
        coverage_area_added = len(new_cells) * grid.cell_area_m2
        lane_segment_id = f"lane_{lane_index:03d}"
        add_pose(
            segment_id=lane_segment_id,
            segment_type="lane_start",
            lane_index=lane_index,
            travel_sign=travel_sign,
            x_m=start_x,
            candidate=candidate,
            heading_deg=candidate.heading_deg,
            heading_axis_sign=candidate.heading_axis_sign,
            primitive_type="cleaning_lane",
            selected_transition_primitive="none",
            coverage_role="tool_sweep",
            coverage_cells_added=len(new_cells),
            coverage_area_added_m2=coverage_area_added,
            notes="start of selected tool-coverage strip",
        )
        add_pose(
            segment_id=lane_segment_id,
            segment_type="lane_end",
            lane_index=lane_index,
            travel_sign=travel_sign,
            x_m=end_x,
            candidate=candidate,
            heading_deg=candidate.heading_deg,
            heading_axis_sign=candidate.heading_axis_sign,
            primitive_type="cleaning_lane",
            selected_transition_primitive="none",
            coverage_role="tool_sweep",
            coverage_cells_added=0,
            coverage_area_added_m2=0.0,
            notes="end of selected tool-coverage strip",
        )

        # 마지막 레인 뒤에는 전이가 없다. / No transition after the final lane.
        if lane_index == len(selected_candidates) - 1:
            continue
        # 레인 간 내부 전이: 다음 레인 쪽(±90°)으로 제자리 회전 → 레인 중심 간 직선
        # 이동 → 다시 청소 헤딩으로 회전(4개 포즈로 표현).
        # Inter-lane transition: rotate toward the next lane (±90°), drive between
        # lane centers, rotate back to cleaning heading (four poses).
        next_candidate = selected_candidates[lane_index + 1][0]
        transition_heading = 90.0 if next_candidate.chassis_y_m >= candidate.chassis_y_m else -90.0
        transition_choice = _select_transition_choice(
            config,
            info,
            candidate,
            next_candidate,
            end_x,
        )
        transition_segment_id = f"transition_{lane_index:03d}_to_{lane_index + 1:03d}"
        add_pose(
            segment_id=transition_segment_id,
            segment_type="transition_start",
            lane_index=lane_index,
            travel_sign=travel_sign,
            x_m=end_x,
            candidate=candidate,
            heading_deg=candidate.heading_deg,
            heading_axis_sign=candidate.heading_axis_sign,
            primitive_type="transition",
            selected_transition_primitive=transition_choice.primitive,
            coverage_role="transition",
            coverage_cells_added=0,
            coverage_area_added_m2=0.0,
            transition_choice=transition_choice,
            notes=f"differential-drive transition start using {transition_choice.primitive}",
        )
        add_pose(
            segment_id=transition_segment_id,
            segment_type="rotate_90",
            lane_index=lane_index,
            travel_sign=travel_sign,
            x_m=end_x,
            candidate=candidate,
            heading_deg=transition_heading,
            heading_axis_sign=candidate.heading_axis_sign,
            primitive_type="transition",
            selected_transition_primitive=transition_choice.primitive,
            coverage_role="transition",
            coverage_cells_added=0,
            coverage_area_added_m2=0.0,
            transition_choice=transition_choice,
            notes="differential-drive transition: rotate in place toward next lane",
        )
        add_pose(
            segment_id=transition_segment_id,
            segment_type="reverse_offset",
            lane_index=lane_index + 1,
            travel_sign=1 if next_candidate.chassis_y_m >= candidate.chassis_y_m else -1,
            x_m=end_x,
            candidate=next_candidate,
            heading_deg=transition_heading,
            heading_axis_sign=1 if transition_heading >= 0.0 else -1,
            primitive_type="transition",
            selected_transition_primitive=transition_choice.primitive,
            coverage_role="transition",
            coverage_cells_added=0,
            coverage_area_added_m2=0.0,
            transition_choice=transition_choice,
            notes="differential-drive transition: drive straight between lane centers",
        )
        add_pose(
            segment_id=transition_segment_id,
            segment_type="rotate_back",
            lane_index=lane_index + 1,
            travel_sign=-travel_sign,
            x_m=end_x,
            candidate=next_candidate,
            heading_deg=next_candidate.heading_deg,
            heading_axis_sign=next_candidate.heading_axis_sign,
            primitive_type="transition",
            selected_transition_primitive=transition_choice.primitive,
            coverage_role="transition",
            coverage_cells_added=0,
            coverage_area_added_m2=0.0,
            transition_choice=transition_choice,
            notes="differential-drive transition: rotate in place to selected tool-strip heading",
        )

    # 종료 연결자(rover-center 모드 전용): 마지막 레인 끝에서 툴을 끈 채 B의 정확한
    # 로버 중심까지 회전·이동으로 마무리한다.
    # End connector (rover-center modes): from the last lane end, drive dry to the
    # exact B center via rotate/translate.
    if poses and config.workspace_mode in {"ab_diagonal_center", "ab_centerline_width"}:
        last_lane = selected_candidates[-1][0]
        last_pose = poses[-1]
        end_dx = end_target[0] - float(last_pose["x_m"])
        end_dy = end_target[1] - last_lane.chassis_y_m
        end_axis_heading = 0.0 if end_dx >= 0.0 else 180.0
        end_lateral_heading = 90.0 if end_dy >= 0.0 else -90.0
        route_candidate = ToolStripCandidate(
            candidate_index=-2,
            tool_center_y_m=end_target[1] + last_lane.tool_world_sign * config.tool_lateral_offset_m,
            tool_world_sign=last_lane.tool_world_sign,
            tool_world_side=last_lane.tool_world_side,
            heading_axis_sign=last_lane.heading_axis_sign,
            heading_deg=end_lateral_heading,
            candidate_mode="exact_B_centerline_end",
            chassis_y_m=end_target[1],
            tool_edge_min_y_m=0.0,
            tool_edge_max_y_m=0.0,
            covered_cells=frozenset(),
        )
        if math.hypot(end_dx, end_dy) > 1e-9:
            if abs(end_dx) > 1e-9:
                if abs(_normalize_angle_deg(float(last_pose["heading_deg"]) - end_axis_heading)) > 1e-9:
                    add_pose(
                        segment_id="route_end_B",
                        segment_type="route_end",
                        lane_index=-1,
                        travel_sign=1,
                        x_m=float(last_pose["x_m"]),
                        candidate=last_lane,
                        heading_deg=end_axis_heading,
                        heading_axis_sign=last_lane.heading_axis_sign,
                        primitive_type="transition",
                        selected_transition_primitive="tool_inactive_end_connector",
                        coverage_role="transition",
                        coverage_cells_added=0,
                        coverage_area_added_m2=0.0,
                        notes="tool-inactive end connector: rotate parallel to rectangle lane axis",
                    )
                add_pose(
                    segment_id="route_end_B",
                    segment_type="route_end",
                    lane_index=-1,
                    travel_sign=1 if end_dx >= 0.0 else -1,
                    x_m=end_target[0],
                    candidate=last_lane,
                    heading_deg=end_axis_heading,
                    heading_axis_sign=last_lane.heading_axis_sign,
                    primitive_type="transition",
                    selected_transition_primitive="tool_inactive_end_connector",
                    coverage_role="transition",
                    coverage_cells_added=0,
                    coverage_area_added_m2=0.0,
                    notes="tool-inactive end connector: drive dry cells along rectangle lane axis",
                )
            add_pose(
                segment_id="route_end_B",
                segment_type="route_end",
                lane_index=-1,
                travel_sign=1,
                x_m=end_target[0],
                candidate=last_lane,
                heading_deg=end_lateral_heading,
                heading_axis_sign=last_lane.heading_axis_sign,
                primitive_type="transition",
                selected_transition_primitive="tool_inactive_end_connector",
                coverage_role="transition",
                coverage_cells_added=0,
                coverage_area_added_m2=0.0,
                notes="tool-inactive end connector: rotate toward final B lateral offset",
            )
            if abs(end_dy) > 1e-9:
                add_pose(
                    segment_id="route_end_B",
                    segment_type="route_end",
                    lane_index=-1,
                    travel_sign=1 if end_dy >= 0.0 else -1,
                    x_m=end_target[0],
                    candidate=route_candidate,
                    heading_deg=end_lateral_heading,
                    heading_axis_sign=last_lane.heading_axis_sign,
                    primitive_type="transition",
                    selected_transition_primitive="tool_inactive_end_connector",
                    coverage_role="transition",
                    coverage_cells_added=0,
                    coverage_area_added_m2=0.0,
                    notes="tool-inactive end connector: drive dry cells to B",
                )
        add_pose(
            segment_id="route_end_B",
            segment_type="route_end",
            lane_index=-1,
            travel_sign=1,
            x_m=end_target[0],
            candidate=route_candidate,
            heading_deg=end_lateral_heading,
            heading_axis_sign=last_lane.heading_axis_sign,
            primitive_type="transition",
            selected_transition_primitive="tool_inactive_end_connector",
            coverage_role="transition",
            coverage_cells_added=0,
            coverage_area_added_m2=0.0,
            notes="exact rover-center end at B; tool inactive",
        )

    annotate_swept_volume_fields(config, poses)
    annotate_kinematic_fields(config, poses)
    annotate_tool_primary_fields(config, poses)
    annotate_endpoint_fields(config, poses)
    for pose in poses:
        pose["route_order_strategy"] = route_order_strategy
        pose["route_order_strategy_candidates_evaluated"] = route_order_candidates_evaluated
    first_lane_pose = next((pose for pose in poses if pose["segment_type"] == "lane_start"), None)
    first_heading_aligned = (
        first_lane_pose is not None
        and (
            abs(_normalize_angle_deg(float(first_lane_pose["heading_deg"]))) <= 1e-6
            or abs(_normalize_angle_deg(float(first_lane_pose["heading_deg"]) - 180.0)) <= 1e-6
        )
    )
    for pose in poses:
        pose["first_cleaning_heading_aligned"] = first_heading_aligned

    return poses


def generate_side_tool_path(
    config: SideToolPlanConfig,
) -> list[dict[str, float | int | str | bool]]:
    """A/B 경계 기반 사이드 툴 커버리지 경로 미리보기를 생성한다(주 진입점, 공개).

    파이프라인: 툴 스트립 후보 생성 → 집합피복 선택 → route-order 전략별 방문열 →
    각 방문열로 포즈열 조립·채점 → 최적 후보 선택. 채점은 오염·운동학·스위프·경계
    위반 수, B 도달 여부, 커버리지 비율, 최종 B까지 거리를 사전식으로 비교한다.
    결과 행은 기하 정보일 뿐이며 로버 명령이 아니다(모터 명령 없음).

    Generate an A/B-bounded side-mounted cleaning-tool coverage preview
    (public main entry). Pipeline: candidate strips → set-cover selection →
    route-order sequences → build+score poses per sequence → pick the best by a
    lexicographic score (violations, reaches-B, coverage, distance-to-B).
    Output rows are geometry only, never rover/motor commands.

    부수효과: 선택된 포즈열에 대해 annotate_contamination/endpoint를 in-place로
    적용해 반환한다. fail_on_boundary_violation이면 위반 시 ValueError.
    Side effect: annotates the chosen poses in place before returning; may raise
    ValueError on boundary violations when configured to fail.
    """
    info = _workspace_info(config)
    grid = _coverage_grid(info, config.coverage_resolution_m)
    candidates = _generate_tool_strip_candidates(config, info, grid)
    selected_candidates = _select_tool_strip_candidates(config, candidates)
    sequences = _ordered_candidate_sequences(config, selected_candidates)

    scored: list[tuple[tuple[float, float, float, float], list[dict[str, float | int | str | bool]]]] = []
    for route_name, sequence in sequences:
        poses = _build_side_tool_path_for_candidates(
            config,
            info,
            grid,
            sequence,
            route_name,
            len(sequences),
        )
        contam = contamination_analysis(config, poses)
        summary = summarize_side_tool_path(config, poses)
        # 사전식(lexicographic) 점수: 앞쪽 항목일수록 우선순위가 높다. 위반이 적고,
        # B에 도달하며, 커버리지가 높고(음수라 작을수록 좋음), B에 가까운 경로를 선호.
        # Lexicographic score (earlier keys dominate): fewer violations, reaches B,
        # higher coverage (negated), then closer to B.
        score = (
            float(contam["contamination_violation_count"]),
            float(summary["kinematic_violation_count"]),
            float(summary["combined_swept_violation_count"]),
            float(summary["boundary_violation_count"]),
            0.0 if summary["route_reaches_B"] is True else 1.0,
            -float(summary["coverage_ratio"]),
            float(summary["final_distance_to_B_m"]),
        )
        scored.append((score, poses))

    if not scored:
        raise ValueError("NO_FEASIBLE_TOOL_PLACEMENT")
    scored.sort(key=lambda item: item[0])
    poses = scored[0][1]

    violations = [
        pose
        for pose in poses
        if pose["within_boundary"] is not True
        or pose["swept_volume_within_boundary"] is not True
    ]
    if config.fail_on_boundary_violation and violations:
        first = violations[0]
        reason = (
            first["swept_volume_violation_reason"]
            if first["swept_volume_within_boundary"] is not True
            else first["boundary_violation_reason"]
        )
        raise ValueError(
            "boundary violation in side-tool preview: "
            f"index={first['segment_index']} reason={reason}"
        )

    contam = contamination_analysis(config, poses)
    annotate_contamination_fields(config, poses)
    annotate_endpoint_fields(config, poses)
    final_summary = summarize_side_tool_path(config, poses)
    _ = contam
    _ = final_summary

    return poses


# ── 진단 / Diagnostics (offline route-order comparison) ──


def strategy_diagnostics(
    config: SideToolPlanConfig,
) -> list[dict[str, float | int | str | bool]]:
    """여러 route-order 전략을 평가해 실현 불가 원인을 오프라인 진단(공개).

    각 전략별로 포즈열을 만들어 레인 수, B 도달, 오염 위반, 커버리지, 거절 사유 등을
    한 행으로 요약해 리스트로 반환한다. 후보 생성 자체가 실패하면 단일 진단 행 반환.
    `dataclasses.replace`로 route_order만 바꿔 시도하므로 원본 config는 불변.

    Evaluate route-order strategies for offline infeasibility diagnosis; returns
    one summary row per strategy (lane count, reaches-B, contamination, coverage,
    rejection reason). Uses dataclasses.replace to vary only route_order.
    """
    info = _workspace_info(config)
    grid = _coverage_grid(info, config.coverage_resolution_m)
    diagnostics: list[dict[str, float | int | str | bool]] = []
    strategy_names = [
        "wet_frontier_safe_boustrophedon",
        "bottom_to_top",
        "top_to_bottom",
        "far_side_to_exit",
        "exit_side_to_far",
        "dry_corridor_preserving",
        "snake_from_A_to_B",
        "reverse_snake_to_B",
        "tool_lift_transitions",
        "fixed_serpentine",
    ]
    try:
        candidates = _generate_tool_strip_candidates(config, info, grid)
        selected = _select_tool_strip_candidates(config, candidates)
    except ValueError as exc:
        return [
            {
                "strategy_name": "candidate_generation",
                "selected_lane_count": 0,
                "reaches_B": False,
                "contamination_violation_count": 0,
                "first_violation_lane": "NA",
                "first_violation_primitive": "NA",
                "coverage_ratio": 0.0,
                "reason_for_rejection": str(exc),
                "route_feasible_for_physical_preview": False,
            }
        ]

    for name in strategy_names:
        cfg = replace(
            config,
            route_order=name,
            fail_on_boundary_violation=False,
        )
        try:
            sequence = _ordered_candidate_sequences(cfg, selected)[0][1]
            poses = _build_side_tool_path_for_candidates(
                cfg,
                info,
                grid,
                sequence,
                name,
                len(strategy_names),
            )
            annotate_contamination_fields(cfg, poses)
            summary = summarize_side_tool_path(cfg, poses)
            events = list(contamination_analysis(cfg, poses)["events"])
            first_event = events[0] if events else None
            diagnostics.append(
                {
                    "strategy_name": name,
                    "selected_lane_count": summary["selected_lane_count"],
                    "reaches_B": summary["route_reaches_B"],
                    "contamination_violation_count": summary["contamination_violation_count"],
                    "first_violation_lane": first_event.lane_index if first_event is not None else "NA",
                    "first_violation_primitive": first_event.segment_id if first_event is not None else "NA",
                    "coverage_ratio": summary["coverage_ratio"],
                    "reason_for_rejection": summary["infeasible_reason"],
                    "route_feasible_for_physical_preview": summary["route_feasible_for_physical_preview"],
                }
            )
        except ValueError as exc:
            diagnostics.append(
                {
                    "strategy_name": name,
                    "selected_lane_count": 0,
                    "reaches_B": False,
                    "contamination_violation_count": 0,
                    "first_violation_lane": "NA",
                    "first_violation_primitive": "NA",
                    "coverage_ratio": 0.0,
                    "reason_for_rejection": str(exc),
                    "route_feasible_for_physical_preview": False,
                }
            )
    return diagnostics


# ══════════════════════════════════════════════════════════════════════════
# 단순 툴 왕복(serpentine) 미리보기 모델 / Simple tool-serpentine preview model
#   tool_serpentine_ab 전용. 위의 레인 탐색/집합피복과 독립된 별도 경로 모델로,
#   툴 중심선(왕복 트랙)을 먼저 만들고 섀시/프리미티브를 나중에 유도한다.
#   Standalone from the lane-search planner above; tool centerline first, then
#   chassis/primitives derived.
# ══════════════════════════════════════════════════════════════════════════


def _tool_serpentine_track_ys(config: SideToolPlanConfig, height_m: float) -> tuple[list[float], list[float], float]:
    """왕복 트랙들의 y 위치, 실제 간격, 마지막 부분 간격을 계산.

    B(하단, y=0)에서 정확히 끝나도록 간격을 조정할 수 있다(`adjust_spacing_to_end_at_B`):
    이 경우 간격 수를 올림하고, force_end_at_B면 홀짝(parity)을 맞춰 마지막 트랙이
    y=0에 오게 한다. 조정하지 않으면 요청 간격을 쓰되 parity가 안 맞으면 ValueError.

    Track y positions (top→bottom), actual spacings, and final partial spacing.
    Optionally adjusts spacing so the last track lands exactly at B (y=0), fixing
    parity when force_end_at_B; otherwise uses the requested spacing and raises on
    a parity mismatch.
    """
    if height_m <= 0.0:
        raise ValueError("TOOL_SERPENTINE_AB_WIDTH_ZERO")
    requested = config.step_spacing_m
    if config.adjust_spacing_to_end_at_B:
        interval_count = max(1, math.ceil(height_m / requested))
        # 짝수 간격이어야 트랙 수가 홀수가 되어 마지막 트랙이 시작과 같은 변(y=0=B)에서
        # 끝난다. 홀수면 하나 늘려 B 종료 보장.
        # Even interval count → odd track count → last track ends on the start side
        # (y=0=B). Bump by one if odd to guarantee ending at B.
        if config.force_end_at_B and interval_count % 2 == 1:
            interval_count += 1
        actual_spacing = height_m / interval_count
        ys = [height_m - index * actual_spacing for index in range(interval_count + 1)]
        ys[-1] = 0.0
    else:
        ys = [height_m]
        cursor = height_m
        while cursor - requested > 0.0:
            cursor -= requested
            ys.append(cursor)
        if abs(ys[-1]) > 1e-9:
            ys.append(0.0)
        if config.force_end_at_B and len(ys) % 2 == 0:
            raise ValueError(
                "TOOL_SERPENTINE_PARITY_CANNOT_END_AT_B_WITH_REQUESTED_SPACING: "
                "use --adjust-spacing-to-end-at-B true"
            )
    actual_spacings = [ys[index] - ys[index + 1] for index in range(len(ys) - 1)]
    full_intervals = math.floor(height_m / requested)
    final_partial = height_m - full_intervals * requested
    if final_partial <= 1e-9 and actual_spacings:
        final_partial = actual_spacings[-1]
    return ys, actual_spacings, final_partial


def _tool_serpentine_chassis_pose(
    config: SideToolPlanConfig,
    tool_x_m: float,
    tool_y_m: float,
    heading_deg: float,
) -> tuple[float, float, float]:
    """툴 (x, y, heading)에서 섀시 포즈(x, y, heading)를 유도. / Chassis pose from tool pose."""
    chassis_x, chassis_y = _chassis_center_from_tool(
        config,
        tool_x_m=tool_x_m,
        tool_y_m=tool_y_m,
        heading_deg=heading_deg,
    )
    return chassis_x, chassis_y, heading_deg


def generate_tool_serpentine_preview(
    config: SideToolPlanConfig,
) -> dict[str, object]:
    """툴 중심 왕복(serpentine) 미리보기 모델을 생성한다(공개, tool_serpentine_ab 전용).

    위쪽의 레인 탐색/집합피복 플래너와 **독립**된 신형 모델이다. 먼저 툴/도료탱크
    중심선(왕복 트랙 + 간격 연결자)을 만들고, 그로부터 섀시 세그먼트와 미분구동
    프리미티브(전진/후진/좌·우회전)를 유도한다. 반환 dict에는 info, tool_segments,
    chassis_segments, primitive_rows, summary가 들어 있으며 모두 미리보기 기하다
    (모터 명령 없음).

    검증 항목: 프리미티브 시퀀스 유효성, 전진/후진 교대, 연결자 패턴(우회전-전진-
    좌회전, 도색 비활성), 툴 경로 연속성, A/B 시작·종료 일치, 커버리지 비율.

    Fresh tool-centered serpentine preview (tool_serpentine_ab only); independent
    of the lane-search planner. Builds the tool centerline first, then derives
    chassis segments and differential-drive primitives. Returns a dict of info /
    tool_segments / chassis_segments / primitive_rows / summary (preview geometry,
    no motor commands).

    리팩토링 주의: 이 모드는 tool_serpentine_ab 전용이라 A=좌상단, B=우하단을 가정한다.
    summary의 tool_length_m는 `tool_length_m()`로 결정한다(tool → robot → 0.18m).
    """
    if config.workspace_mode != "tool_serpentine_ab":
        raise ValueError("generate_tool_serpentine_preview requires workspace_mode='tool_serpentine_ab'")
    info = _workspace_info(config)
    width = info.rectangle_length_m
    height = info.rectangle_width_m
    y_positions, spacing_values, final_partial = _tool_serpentine_track_ys(config, height)
    x_min = 0.0
    x_max = width
    # 1) 툴 중심선 세그먼트 생성: 왕복 트랙(짝수=좌→우, 홀수=우→좌)과 그 사이 간격
    #    연결자를 만든다. / Build tool centerline: alternating sweep tracks + connectors.
    tool_segments: list[dict[str, object]] = []
    segment_index = 0
    for track_index, y_m in enumerate(y_positions):
        if track_index % 2 == 0:
            start = (x_min, y_m)
            end = (x_max, y_m)
        else:
            start = (x_max, y_m)
            end = (x_min, y_m)
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        distance = math.hypot(dx, dy)
        heading = math.degrees(math.atan2(dy, dx)) if distance > 1e-9 else 0.0
        tool_active = bool(config.tool_active_on_sweep_tracks)
        tool_segments.append(
            {
                "tool_segment_index": segment_index,
                "tool_segment_id": f"T{track_index:03d}",
                "tool_segment_type": "tool_sweep_track",
                "track_index": track_index,
                "connector_index": "",
                "tool_start_x_m": start[0],
                "tool_start_y_m": start[1],
                "tool_end_x_m": end[0],
                "tool_end_y_m": end[1],
                "tool_heading_deg": heading,
                "tool_distance_m": distance,
                "tool_active": tool_active,
                "coverage_contributes": tool_active,
                "cleaned_area_added_m2": distance * config.tool_width_m if tool_active else 0.0,
                "is_ghost_tool_path": not tool_active,
                "associated_primitive_indices": "",
                "tool_waypoint_label": f"T{track_index}_start",
            }
        )
        segment_index += 1
        if track_index == len(y_positions) - 1:
            continue
        next_y = y_positions[track_index + 1]
        connector_start = end
        connector_end = (end[0], next_y)
        dx = connector_end[0] - connector_start[0]
        dy = connector_end[1] - connector_start[1]
        distance = math.hypot(dx, dy)
        heading = math.degrees(math.atan2(dy, dx)) if distance > 1e-9 else 0.0
        tool_segments.append(
            {
                "tool_segment_index": segment_index,
                "tool_segment_id": f"C{track_index:03d}_to_{track_index + 1:03d}",
                "tool_segment_type": "tool_spacing_connector",
                "track_index": "",
                "connector_index": track_index,
                "tool_start_x_m": connector_start[0],
                "tool_start_y_m": connector_start[1],
                "tool_end_x_m": connector_end[0],
                "tool_end_y_m": connector_end[1],
                "tool_heading_deg": heading,
                "tool_distance_m": distance,
                "tool_active": config.tool_active_on_connectors,
                "coverage_contributes": config.tool_active_on_connectors,
                "cleaned_area_added_m2": distance * config.tool_width_m if config.tool_active_on_connectors else 0.0,
                "is_ghost_tool_path": not config.tool_active_on_connectors,
                "associated_primitive_indices": "",
                "tool_waypoint_label": f"C{track_index}_to_{track_index + 1}",
            }
        )
        segment_index += 1

    # 2) 각 툴 세그먼트에서 섀시 세그먼트를 유도(측면 오프셋 역산). 스위프 트랙은
    #    레인 헤딩, 연결자는 -90° 헤딩. / Derive chassis segments from tool segments.
    lane_heading_deg = 0.0
    connector_heading_deg = -90.0
    chassis_segments: list[dict[str, object]] = []
    for segment in tool_segments:
        start_x, start_y, _ = _tool_serpentine_chassis_pose(
            config,
            float(segment["tool_start_x_m"]),
            float(segment["tool_start_y_m"]),
            lane_heading_deg,
        )
        end_x, end_y, _ = _tool_serpentine_chassis_pose(
            config,
            float(segment["tool_end_x_m"]),
            float(segment["tool_end_y_m"]),
            lane_heading_deg,
        )
        chassis_heading = (
            lane_heading_deg
            if segment["tool_segment_type"] == "tool_sweep_track"
            else connector_heading_deg
        )
        chassis_segments.append(
            {
                "chassis_segment_id": f"CH_{segment['tool_segment_id']}",
                "associated_tool_segment_id": segment["tool_segment_id"],
                "chassis_start_x_m": start_x,
                "chassis_start_y_m": start_y,
                "chassis_end_x_m": end_x,
                "chassis_end_y_m": end_y,
                "chassis_heading_deg": chassis_heading,
                "chassis_path_derived_from_tool": True,
                "chassis_feasible_for_tool_path": True,
                "chassis_infeasible_reason": "OK",
            }
        )

    # 3) 섀시 세그먼트로부터 미분구동 프리미티브(회전/전진/후진) 행을 생성.
    #    아래 중첩 헬퍼들이 current_pose를 이어가며 rotate/move 행을 추가한다.
    #    Generate differential-drive primitive rows from chassis segments; the
    #    nested helpers thread current_pose and emit rotate/move rows.
    primitive_rows: list[dict[str, object]] = []
    current_pose: tuple[float, float, float] | None = None

    def _index_value(value: object) -> int | str:
        """빈 문자열은 그대로, 아니면 int로. / Keep '' as-is, else coerce to int."""
        return "" if value == "" else int(value)

    def add_primitive(
        *,
        primitive_type: str,
        distance_m: float,
        angle_deg: float,
        start_pose: tuple[float, float, float],
        end_pose: tuple[float, float, float],
        segment: dict[str, object],
        segment_role: str,
        tool_active: bool,
        coverage_contributes: bool,
    ) -> None:
        """프리미티브 행 하나를 목록에 추가. / Append one primitive row."""
        primitive_rows.append(
            {
                "primitive_index": len(primitive_rows),
                "primitive_type": primitive_type,
                "distance_m": distance_m,
                "angle_deg": angle_deg,
                "segment_role": segment_role,
                "track_index": _index_value(segment["track_index"]),
                "connector_index": _index_value(segment["connector_index"]),
                "start_x_m": start_pose[0],
                "start_y_m": start_pose[1],
                "start_heading_deg": start_pose[2],
                "end_x_m": end_pose[0],
                "end_y_m": end_pose[1],
                "end_heading_deg": end_pose[2],
                "associated_tool_segment_id": segment["tool_segment_id"],
                "tool_active": tool_active,
                "coverage_contributes": coverage_contributes,
                "motor_command_generated": False,
            }
        )

    def add_rotate(
        target_heading: float,
        segment: dict[str, object],
        segment_role: str,
    ) -> None:
        """현재 헤딩에서 target_heading으로 제자리 회전 프리미티브 추가(무회전이면 생략).

        Emit an in-place rotate to target_heading (skips if already aligned).
        """
        nonlocal current_pose
        if current_pose is None:
            return
        sx, sy, start_heading = current_pose
        delta = _normalize_angle_deg(target_heading - start_heading)
        if abs(delta) <= 1e-9:
            return
        primitive_type = "rotate_left" if delta > 0.0 else "rotate_right"
        end_pose = (sx, sy, target_heading)
        add_primitive(
            primitive_type=primitive_type,
            distance_m=0.0,
            angle_deg=abs(delta),
            start_pose=current_pose,
            end_pose=end_pose,
            segment=segment,
            segment_role=segment_role,
            tool_active=False,
            coverage_contributes=False,
        )
        current_pose = end_pose

    def add_track_move(chassis: dict[str, object], segment: dict[str, object]) -> None:
        """스위프 트랙: 레인 헤딩 정렬 후 전진/후진 이동 프리미티브 추가.

        Sweep track: align to lane heading, then a forward/backward move.
        """
        nonlocal current_pose
        sx = float(chassis["chassis_start_x_m"])
        sy = float(chassis["chassis_start_y_m"])
        ex = float(chassis["chassis_end_x_m"])
        ey = float(chassis["chassis_end_y_m"])
        heading = lane_heading_deg
        if current_pose is None:
            current_pose = (sx, sy, heading)
        add_rotate(heading, segment, "alignment")
        current_pose = (sx, sy, heading)
        distance = math.hypot(ex - sx, ey - sy)
        primitive_type = "move_forward" if ex >= sx else "move_backward"
        end_pose = (ex, ey, heading)
        add_primitive(
            primitive_type=primitive_type,
            distance_m=distance,
            angle_deg=0.0,
            start_pose=current_pose,
            end_pose=end_pose,
            segment=segment,
            segment_role="sweep_track",
            tool_active=bool(segment["tool_active"]),
            coverage_contributes=bool(segment["coverage_contributes"]),
        )
        current_pose = end_pose

    def add_connector_primitives(chassis: dict[str, object], segment: dict[str, object]) -> None:
        """간격 연결자: 회전(→-90°) → 전진 → 회전(→레인 헤딩)의 3 프리미티브 추가.

        연결자 구간은 항상 툴 비활성(도색 안 함)이라 커버리지에 기여하지 않는다.
        Spacing connector: rotate → forward → rotate (tool inactive, no coverage).
        """
        nonlocal current_pose
        sx = float(chassis["chassis_start_x_m"])
        sy = float(chassis["chassis_start_y_m"])
        ex = float(chassis["chassis_end_x_m"])
        ey = float(chassis["chassis_end_y_m"])
        if current_pose is None:
            current_pose = (sx, sy, lane_heading_deg)
        current_pose = (sx, sy, current_pose[2])
        add_rotate(connector_heading_deg, segment, "connector_rotation")
        start_pose = current_pose
        end_pose = (ex, ey, connector_heading_deg)
        add_primitive(
            primitive_type="move_forward",
            distance_m=math.hypot(ex - sx, ey - sy),
            angle_deg=0.0,
            start_pose=start_pose,
            end_pose=end_pose,
            segment=segment,
            segment_role="connector_spacing_move",
            tool_active=False,
            coverage_contributes=False,
        )
        current_pose = end_pose
        add_rotate(lane_heading_deg, segment, "connector_rotation")

    segment_to_primitive_indices: dict[str, list[int]] = {}
    for segment, chassis in zip(tool_segments, chassis_segments, strict=True):
        start_index = len(primitive_rows)
        if segment["tool_segment_type"] == "tool_sweep_track":
            add_track_move(chassis, segment)
        else:
            add_connector_primitives(chassis, segment)
        segment_to_primitive_indices[str(segment["tool_segment_id"])] = list(
            range(start_index, len(primitive_rows))
        )
    for segment in tool_segments:
        indices = segment_to_primitive_indices[str(segment["tool_segment_id"])]
        segment["associated_primitive_indices"] = ",".join(str(index) for index in indices)

    # 4) 검증·요약: 프리미티브 시퀀스/패턴, 커버리지, A·B 정합을 확인하고 요약 dict 작성.
    #    Validate & summarize: primitive/connector patterns, coverage, A/B match.
    allowed_primitives = {"move_forward", "move_backward", "rotate_left", "rotate_right"}
    primitive_sequence_valid = all(
        str(row["primitive_type"]) in allowed_primitives for row in primitive_rows
    )
    track_count = sum(1 for segment in tool_segments if segment["tool_segment_type"] == "tool_sweep_track")
    connector_count = sum(1 for segment in tool_segments if segment["tool_segment_type"] == "tool_spacing_connector")
    track_move_rows = [
        row for row in primitive_rows if row["segment_role"] == "sweep_track"
    ]
    expected_track_moves = [
        "move_forward" if index % 2 == 0 else "move_backward"
        for index in range(len(track_move_rows))
    ]
    forward_backward_alternation_valid = [
        row["primitive_type"] for row in track_move_rows
    ] == expected_track_moves
    connector_pattern_valid = True
    for segment in tool_segments:
        if segment["tool_segment_type"] != "tool_spacing_connector":
            continue
        indices = [
            int(index)
            for index in str(segment["associated_primitive_indices"]).split(",")
            if index != ""
        ]
        rows = [primitive_rows[index] for index in indices]
        if [row["primitive_type"] for row in rows] != ["rotate_right", "move_forward", "rotate_left"]:
            connector_pattern_valid = False
        if any(bool(row["tool_active"]) or bool(row["coverage_contributes"]) for row in rows):
            connector_pattern_valid = False
        if any(float(row["angle_deg"]) not in {0.0, 90.0} for row in rows):
            connector_pattern_valid = False
    active_paint_path_uses_only_sweep_tracks = all(
        segment["tool_segment_type"] == "tool_sweep_track"
        for segment in tool_segments
        if bool(segment["tool_active"]) or bool(segment["coverage_contributes"])
    )
    connector_painting_disabled = all(
        not bool(segment["tool_active"]) and not bool(segment["coverage_contributes"])
        for segment in tool_segments
        if segment["tool_segment_type"] == "tool_spacing_connector"
    )
    tool_path_continuous = all(
        abs(float(tool_segments[index]["tool_end_x_m"]) - float(tool_segments[index + 1]["tool_start_x_m"])) <= 1e-9
        and abs(float(tool_segments[index]["tool_end_y_m"]) - float(tool_segments[index + 1]["tool_start_y_m"])) <= 1e-9
        for index in range(len(tool_segments) - 1)
    )
    start_segment = tool_segments[0]
    end_segment = tool_segments[-1]
    tool_path_starts_at_A = (
        abs(float(start_segment["tool_start_x_m"]) - (info.a_x_m - info.world_origin_x_m)) <= 1e-9
        and abs(float(start_segment["tool_start_y_m"]) - (info.a_y_m - info.world_origin_y_m)) <= 1e-9
    )
    tool_path_ends_at_B = (
        abs(float(end_segment["tool_end_x_m"]) - (info.b_x_m - info.world_origin_x_m)) <= 1e-9
        and abs(float(end_segment["tool_end_y_m"]) - (info.b_y_m - info.world_origin_y_m)) <= 1e-9
    )
    requested_area = width * height
    intervals = _merge_intervals(
        [
            (
                max(0.0, float(segment["tool_start_y_m"]) - config.tool_width_m / 2.0),
                min(height, float(segment["tool_start_y_m"]) + config.tool_width_m / 2.0),
            )
            for segment in tool_segments
            if segment["tool_segment_type"] == "tool_sweep_track"
            and bool(segment["coverage_contributes"])
        ]
    )
    covered_area = width * _interval_total_length(intervals)
    uncovered_area = max(0.0, requested_area - covered_area)
    summary = {
        "planner_mode": "simple_tool_serpentine",
        "workspace_mode": "tool_serpentine_ab",
        "A_corner_role": "top_left",
        "B_corner_role": "bottom_right",
        "planner_primary_path": "tool_center_path",
        "chassis_path_role": "derived_support_path",
        "tool_side": config.tool_side,
        "robot_width_m": config.robot_width_m,
        "robot_length_m": config.robot_length_m,
        "tool_width_m": config.tool_width_m,
        "tool_length_m": tool_length_m(config),
        "tool_lateral_offset_m": config.tool_lateral_offset_m,
        "requested_step_spacing_m": config.step_spacing_m,
        "actual_spacing_values_m": ",".join(f"{value:.6f}" for value in spacing_values),
        "derived_track_count": track_count,
        "track_count_parity": "odd" if track_count % 2 == 1 else "even",
        "final_partial_spacing_m": final_partial,
        "force_end_at_B": config.force_end_at_B,
        "adjust_spacing_to_end_at_B": config.adjust_spacing_to_end_at_B,
        "tool_active_on_sweep_tracks": config.tool_active_on_sweep_tracks,
        "tool_active_on_connectors": config.tool_active_on_connectors,
        "tool_active_during_rotations": config.tool_active_during_rotation,
        "tool_path_continuous": tool_path_continuous,
        "tool_track_count": track_count,
        "tool_connector_count": connector_count,
        "tool_connector_count_equals_track_count_minus_one": connector_count == track_count - 1,
        "tool_path_starts_at_A": tool_path_starts_at_A,
        "tool_path_ends_at_B": tool_path_ends_at_B,
        "active_paint_path_uses_only_sweep_tracks": active_paint_path_uses_only_sweep_tracks,
        "connector_painting_disabled": connector_painting_disabled,
        "chassis_path_derived_from_tool": True,
        "primitive_sequence_valid": primitive_sequence_valid,
        "primitive_pattern_valid": (
            primitive_sequence_valid
            and forward_backward_alternation_valid
            and connector_pattern_valid
        ),
        "forward_backward_alternation_valid": forward_backward_alternation_valid,
        "connector_pattern_valid": connector_pattern_valid,
        "chassis_feasible_for_tool_path": primitive_sequence_valid,
        "chassis_infeasible_reason": "OK" if primitive_sequence_valid else "INVALID_PRIMITIVE_SEQUENCE",
        "coverage_ratio": covered_area / requested_area if requested_area > 0.0 else 0.0,
        "uncovered_area_m2": uncovered_area,
        "contamination_mode": config.contamination_mode,
        "motor_command_generated": False,
        "path_preview_only": True,
    }
    return {
        "info": info,
        "tool_segments": tool_segments,
        "chassis_segments": chassis_segments,
        "primitive_rows": primitive_rows,
        "summary": summary,
    }
