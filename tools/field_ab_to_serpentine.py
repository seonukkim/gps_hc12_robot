"""필드 A/B 캡처점 → 무동작(no-motion) 지그재그(serpentine) 경로 패키지 변환기.

목적/역할:
    현장에서 캡처한 두 점 A, B(로컬 미터 좌표 또는 위경도)를 받아, 정규화된 직사각형
    작업 영역을 만들고 그 위에 도구(side-tool) 지그재그 커버리지 경로와 이를 실행하기 위한
    프리미티브(rotate/move) 시퀀스를 생성한다. 결과는 CSV/JSON/Markdown/PNG로 저장되는
    "경로 패키지(path package)"이며, **미리보기 전용**이라 모터 명령이나 물리 출력은 전혀
    만들지 않는다(모든 `motor_command_generated` 플래그는 False).

시스템 내 위치:
    - 파이프라인 상류(upstream): 코어 플래너 `gps_coverage_core.side_tool_planner`의
      `generate_tool_serpentine_preview` / `SideToolPlanConfig`를 호출해 실제 지그재그를 만든다.
    - 파이프라인 하류(downstream): 여기서 쓴 `path_package.json`을
      `tools/inspect_path_package.py`, `tools/path_no_motion_validation.py`,
      `tools/physical_path_preview_from_package.py`가 다시 읽어 검증/미리보기한다.
    - 즉 이 파일이 "A/B → 경로 패키지" 파이프라인의 **진입점(entry point)**이다.

핵심 개념·불변식(invariant):
    - A'/B' 정규화: raw A/B의 min/max로 축 정렬 직사각형을 만든 뒤 A'=좌상단(top-left),
      B'=우하단(bottom-right)으로 고정한다. 코어 플래너의 `tool_serpentine_ab` 모드가
      "A=좌상단, B=우하단"을 요구하므로 이 규약을 반드시 지켜야 한다.
    - x는 동쪽(East), y는 북쪽(North). 위경도 입력은 등장방형(equirectangular ENU) 근사로
      로컬 미터로 환산하며, 원점은 A로 잡는다.
    - 좌표 프레임 오프셋: 코어 플래너는 (0,0) 기준의 로컬 좌표를 돌려주므로, 여기서
      `origin_x/origin_y`(= x_min/y_min)를 더해 raw 프레임으로 되돌린다.
    - 안전 불변식: 프리미티브는 ALLOWED_PRIMITIVES 안에서만, 모든 행의
      `motor_command_generated`는 False여야 `primitive_sequence_valid`가 True가 된다.

사용법/진입점:
    CLI. `main()`이 진입점이며 `python tools/field_ab_to_serpentine.py ...`로 실행한다.
    입력은 (1) `--field-points-json`, (2) `--field-points-georef-json`(위경도),
    (3) 수동 `--a-x/--a-y/--b-x/--b-y` 중 하나. `--step-spacing-m`은 필수.

리팩토링 노트:
    - 코어 플래너와의 결합점은 `build_path_package` 안의 `SideToolPlanConfig` 구성과
      `generate_tool_serpentine_preview` 반환 키(`primitive_rows`, `tool_segments`,
      `chassis_segments`, `summary`)뿐이다. 반환 키가 바뀌면 여기도 함께 고쳐야 한다.
    - CSV 필드 순서(TOOL_PATH_FIELDS/PRIMITIVE_FIELDS)는 하류 소비자와의 계약이므로
      함부로 바꾸지 말 것.

Convert captured field A/B points into a no-motion serpentine path package.
Takes two points (local metres or lat/lon), builds a normalised rectangle
(A' top-left, B' bottom-right), delegates the actual serpentine to the core
`side_tool_planner`, and emits CSV/JSON/Markdown/PNG artifacts. Preview-only:
no motor commands, no physical output. This is the entry point of the
"A/B -> path package" pipeline consumed by the other tools/ path scripts.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import math
from pathlib import Path
from typing import Sequence

try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401

from gps_coverage_core.side_tool_planner import SideToolPlanConfig, generate_tool_serpentine_preview


# ── 상수·필드 정의 / Constants & CSV field contracts ──
# 무동작 미리보기에서 허용되는 프리미티브 집합. 이 밖의 타입이 나오면 시퀀스는 무효.
# Set of primitives allowed in a no-motion preview; anything else invalidates the sequence.
ALLOWED_PRIMITIVES = {"move_forward", "move_backward", "rotate_left", "rotate_right"}

# CSV 헤더 순서 = 하류 도구와의 계약. 순서/이름 변경은 소비자를 깨뜨릴 수 있음.
# CSV header order is a contract with downstream tools; reordering/renaming can break them.
TOOL_PATH_FIELDS = (
    "tool_segment_index",
    "tool_segment_id",
    "tool_segment_type",
    "tool_start_x_m",
    "tool_start_y_m",
    "tool_end_x_m",
    "tool_end_y_m",
    "tool_active",
    "coverage_contributes",
    "motor_command_generated",
)

PRIMITIVE_FIELDS = (
    "primitive_index",
    "primitive_type",
    "distance_m",
    "angle_deg",
    "segment_role",
    "start_x_m",
    "start_y_m",
    "start_heading_deg",
    "end_x_m",
    "end_y_m",
    "end_heading_deg",
    "associated_tool_segment_id",
    "tool_active",
    "coverage_contributes",
    "motor_command_generated",
)

# 위도 1도당 미터(구면 근사 상수). 경도는 위도에 따라 cos 보정이 필요.
# Metres per degree of latitude (spherical approx); longitude needs cos(lat) scaling.
METERS_PER_DEG_LAT = 111_320.0


# ── 좌표·정규화 유틸 / Geometry & normalisation helpers ──
def _normalize_deg(angle_deg: float) -> float:
    """각도를 (-180, 180] 범위로 접어 반환 / Wrap an angle into (-180, 180] degrees."""
    return ((angle_deg + 180.0) % 360.0) - 180.0


def normalize_field_ab(
    *,
    a_x: float,
    a_y: float,
    b_x: float,
    b_y: float,
) -> dict[str, object]:
    """raw A/B로 축 정렬 직사각형을 만들고 A'(좌상)·B'(우하)를 확정한다.

    무엇을/왜: 캡처된 두 점의 순서에 관계없이 min/max로 작업 영역을 정규화해,
    코어 플래너 `tool_serpentine_ab`가 요구하는 "A=좌상단, B=우하단" 규약을 보장한다.
    반환: x/y min·max, 폭/높이, raw A/B, A'/B' 좌표를 담은 dict.
    부수효과: 없음(순수 함수). x 또는 y 범위가 0이면 ValueError를 던진다(경로 생성 불가).
    리팩토링 주의: 여기서 정하는 A'/B' 규약이 파이프라인 전체의 좌표 불변식이다.

    Normalise raw A/B into an axis-aligned rectangle and pin A'=top-left,
    B'=bottom-right (the convention required by the core serpentine planner).
    Raises ValueError on a zero-width/height span. Pure function.
    """
    x_min = min(a_x, b_x)
    x_max = max(a_x, b_x)
    y_min = min(a_y, b_y)
    y_max = max(a_y, b_y)
    if math.isclose(x_min, x_max, abs_tol=1e-9):
        raise ValueError("FIELD_AB_LENGTH_ZERO: captured A/B must span nonzero x")
    if math.isclose(y_min, y_max, abs_tol=1e-9):
        raise ValueError("FIELD_AB_WIDTH_ZERO: captured A/B must span nonzero y")
    return {
        "raw_A": {"x_m": a_x, "y_m": a_y},
        "raw_B": {"x_m": b_x, "y_m": b_y},
        "x_min_m": x_min,
        "x_max_m": x_max,
        "y_min_m": y_min,
        "y_max_m": y_max,
        "width_m": x_max - x_min,
        "height_m": y_max - y_min,
        "A_prime_top_left": {"x_m": x_min, "y_m": y_max},
        "B_prime_bottom_right": {"x_m": x_max, "y_m": y_min},
    }


# ── 입력 로더 / Input loaders (local-metre JSON, geo-referenced JSON) ──
def _load_field_points(path: Path) -> tuple[float, float, float, float]:
    """로컬 미터 JSON에서 A/B 좌표를 읽어 (a_x, a_y, b_x, b_y)로 반환.

    Load A/B local-metre coordinates from a points JSON file.
    """
    data = json.loads(path.read_text(encoding="utf-8"))
    points = data["points"]
    return (
        float(points["A"]["x_m"]),
        float(points["A"]["y_m"]),
        float(points["B"]["x_m"]),
        float(points["B"]["y_m"]),
    )


def _meters_per_deg_lon(latitude_deg: float) -> float:
    """해당 위도에서 경도 1도당 미터(cos 보정) / Metres per degree of longitude at a latitude."""
    return METERS_PER_DEG_LAT * math.cos(math.radians(latitude_deg))


def _load_georef_points(path: Path) -> tuple[tuple[float, float, float, float], dict[str, object]]:
    """위경도 A/B를 로컬 ENU 미터로 환산하고 지오참조 메타데이터를 함께 반환.

    무엇을/왜: 위경도 입력을 A 원점 기준 등장방형(equirectangular ENU) 근사로 로컬
    미터 좌표로 바꾼다. A는 (0,0), B는 (동쪽 m, 북쪽 m)이 된다.
    반환: ((a_x, a_y, b_x, b_y), georef_dict). georef_dict는 원점 위경도, 축척,
    로컬 좌표 등을 담아 나중에 패키지 summary에 병합된다.
    함정(gotcha): 평면 근사이므로 넓은 영역/고위도에서는 왜곡이 커진다. x축 방위는
    90도(정동)로 가정한다.

    Convert lat/lon A/B into local ENU metres (A at origin) and return the pair
    plus geo-reference metadata for later merge into the package summary.
    """
    data = json.loads(path.read_text(encoding="utf-8"))
    points = data["points"]
    a_lat = float(points["A"]["lat"])
    a_lon = float(points["A"]["lon"])
    b_lat = float(points["B"]["lat"])
    b_lon = float(points["B"]["lon"])
    mean_lat = (a_lat + b_lat) / 2.0
    meters_per_deg_lon = _meters_per_deg_lon(mean_lat)
    raw_a = (0.0, 0.0)
    raw_b = (
        (b_lon - a_lon) * meters_per_deg_lon,
        (b_lat - a_lat) * METERS_PER_DEG_LAT,
    )
    georef = {
        "georeference_available": True,
        "raw_A_lat": a_lat,
        "raw_A_lon": a_lon,
        "raw_B_lat": b_lat,
        "raw_B_lon": b_lon,
        "origin_lat": a_lat,
        "origin_lon": a_lon,
        "local_frame_type": "equirectangular_enu",
        "x_axis_source": "normalized_rectangle",
        "x_axis_bearing_deg": 90.0,
        "meters_per_deg_lat": METERS_PER_DEG_LAT,
        "meters_per_deg_lon": meters_per_deg_lon,
        "meters_per_lat": METERS_PER_DEG_LAT,
        "meters_per_lon": meters_per_deg_lon,
        "raw_A_local": {"x_m": raw_a[0], "y_m": raw_a[1]},
        "raw_B_local": {"x_m": raw_b[0], "y_m": raw_b[1]},
        "motor_command_generated": False,
    }
    return (raw_a[0], raw_a[1], raw_b[0], raw_b[1]), georef


# ── 접근(approach) 프리미티브 생성 / Approach primitive builders ──
def _rotation_row(
    *,
    index: int,
    start_pose: tuple[float, float, float],
    target_heading_deg: float,
    associated_tool_segment_id: str,
    segment_role: str,
) -> dict[str, object] | None:
    """목표 방위로의 제자리 회전 프리미티브 한 행을 만든다(회전 불필요 시 None).

    무엇을/왜: 현재 헤딩과 목표 헤딩 차(delta)를 (-180,180]로 정규화해, 부호에 따라
    rotate_left(+)/rotate_right(-)를 선택한다. delta가 거의 0이면 회전이 필요 없어 None.
    반환: 프리미티브 dict 또는 None. 위치는 그대로 두고 헤딩만 바뀐다(거리 0).

    Build one in-place rotation primitive toward a target heading, or None if
    already aligned. Left/right chosen by the sign of the normalised delta.
    """
    delta = _normalize_deg(target_heading_deg - start_pose[2])
    if abs(delta) <= 1e-9:
        return None
    primitive_type = "rotate_left" if delta > 0.0 else "rotate_right"
    return {
        "primitive_index": index,
        "primitive_type": primitive_type,
        "distance_m": 0.0,
        "angle_deg": abs(delta),
        "segment_role": segment_role,
        "start_x_m": start_pose[0],
        "start_y_m": start_pose[1],
        "start_heading_deg": start_pose[2],
        "end_x_m": start_pose[0],
        "end_y_m": start_pose[1],
        "end_heading_deg": target_heading_deg,
        "associated_tool_segment_id": associated_tool_segment_id,
        "tool_active": False,
        "coverage_contributes": False,
        "motor_command_generated": False,
    }


def build_approach_primitives(
    *,
    current_x: float,
    current_y: float,
    current_heading_deg: float,
    target_x: float,
    target_y: float,
) -> list[dict[str, object]]:
    """현재 포즈에서 A'까지 이동하는 접근 프리미티브(회전 후 전진)를 만든다.

    무엇을/왜: 지그재그 본선을 시작하기 전, 로봇을 A'(작업 영역 좌상단)로 데려가는
    "접근(approach)" 구간이다. 목표까지 방위를 계산해 필요 시 회전 1개 + 전진 1개를 넣는다.
    반환: 프리미티브 dict 리스트. 이미 목표에 있으면 빈 리스트.
    부수효과: 없음. 모든 행은 tool_active/coverage/motor 모두 False(비작업·무동작).

    Build the approach primitives (rotate-then-move) that drive the rover from
    its current pose to A'. Returns an empty list if already at the target.
    """
    dx = target_x - current_x
    dy = target_y - current_y
    distance = math.hypot(dx, dy)
    if distance <= 1e-9:
        return []
    bearing = math.degrees(math.atan2(dy, dx))
    rows: list[dict[str, object]] = []
    start_pose = (current_x, current_y, current_heading_deg)
    rotation = _rotation_row(
        index=0,
        start_pose=start_pose,
        target_heading_deg=bearing,
        associated_tool_segment_id="approach_to_A_prime",
        segment_role="approach_rotation",
    )
    if rotation is not None:
        rows.append(rotation)
        start_pose = (current_x, current_y, bearing)
    rows.append(
        {
            "primitive_index": len(rows),
            "primitive_type": "move_forward",
            "distance_m": distance,
            "angle_deg": 0.0,
            "segment_role": "approach_move",
            "start_x_m": current_x,
            "start_y_m": current_y,
            "start_heading_deg": start_pose[2],
            "end_x_m": target_x,
            "end_y_m": target_y,
            "end_heading_deg": start_pose[2],
            "associated_tool_segment_id": "approach_to_A_prime",
            "tool_active": False,
            "coverage_contributes": False,
            "motor_command_generated": False,
        }
    )
    return rows


def _offset_primitives(rows: list[dict[str, object]], offset: int) -> list[dict[str, object]]:
    """각 행의 primitive_index에 offset을 더해 복사본을 반환(전역 인덱스 재부여).

    무엇을/왜: 접근 프리미티브 뒤에 지그재그 프리미티브를 이어 붙일 때, 지그재그의
    인덱스를 접근 개수만큼 밀어 전체 시퀀스에서 연속되게 만든다. 원본은 변경하지 않는다.

    Return copies of rows with primitive_index shifted by offset, so serpentine
    primitives get contiguous global indices after the approach ones.
    """
    shifted: list[dict[str, object]] = []
    for row in rows:
        updated = dict(row)
        updated["primitive_index"] = int(updated["primitive_index"]) + offset
        shifted.append(updated)
    return shifted


# ── 산출물 writer / Artifact writers (CSV, JSON, Markdown, PNG previews) ──
def _write_csv(path: Path, rows: Sequence[dict[str, object]], fields: Sequence[str]) -> None:
    """지정한 필드 순서로 행들을 CSV로 기록(누락 필드는 빈 문자열).

    Write rows to CSV using the given field order; missing fields become "".
    """
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in fields})


def _write_json(path: Path, data: dict[str, object]) -> None:
    """dict를 들여쓰기 2칸 JSON(+개행)으로 기록 / Write a dict as indented JSON with a trailing newline."""
    path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")


def _write_summary(path: Path, package: dict[str, object]) -> None:
    """사람이 읽는 Markdown 요약을 기록(무동작임을 명시하는 배너 포함).

    무엇을/왜: 패키지 summary의 주요 키와(있으면) 지오참조 정보를 표 형태로 남긴다.
    상단에 "no motor commands"임을 못박아 오해를 막는다. 순수 출력용.

    Write the human-readable Markdown summary, headed by a no-motion banner.
    """
    summary = package["summary"]  # type: ignore[index]
    lines = [
        "# Field A/B To Serpentine Path Package",
        "",
        "Preview/no-motion only: no rover motor commands are generated.",
        "",
    ]
    for key in (
        "raw_A",
        "raw_B",
        "A_prime_top_left",
        "B_prime_bottom_right",
        "approach_path_generated",
        "serpentine_path_generated",
        "tool_side",
        "step_spacing_m",
        "primitive_sequence_valid",
        "motor_command_generated",
        "physical_output_active",
        "ready_for_outdoor_no_motion_validation",
    ):
        lines.append(f"- {key}: `{summary[key]}`")
    if package.get("georeference"):
        georef = package["georeference"]  # type: ignore[index]
        lines.append(f"- georeference_available: `{georef['georeference_available']}`")
        lines.append(f"- origin_lat: `{georef['origin_lat']}`")
        lines.append(f"- origin_lon: `{georef['origin_lon']}`")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_previews(out_dir: Path, package: dict[str, object]) -> dict[str, Path | None]:
    """4종 PNG 미리보기(작업영역/도구경로/프리미티브/접근+지그재그)를 저장한다.

    무엇을/왜: 시각 검수용 그림을 만든다. matplotlib이 없으면 각 항목을 None으로 채운
    dict를 돌려주어 파이프라인이 계속 진행되게 한다(선택적 의존성).
    반환: {파일명: Path 또는 None} 매핑.
    좌표 주의: tool_path 행은 (0,0) 기준 로컬 좌표라서 origin_x/origin_y를 더해 raw
    프레임으로 그린다. primitive_sequence는 이미 raw 프레임이라 그대로 사용한다.

    Save four preview PNGs (workspace / tool path / primitive sequence /
    approach+serpentine). If matplotlib is missing, return None for each so the
    pipeline can continue (optional dependency).
    """
    try:
        import matplotlib

        matplotlib.use("Agg")  # 헤드리스 백엔드: 디스플레이 없이 PNG만 저장 / headless PNG-only backend
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle
    except ImportError:
        return {
            "preview_workspace_ab_aprime_bprime.png": None,
            "preview_tool_path_primary.png": None,
            "preview_primitive_sequence.png": None,
            "preview_approach_then_serpentine.png": None,
        }

    workspace = package["normalized_workspace"]  # type: ignore[index]
    tool_rows = package["tool_path"]  # type: ignore[index]
    primitives = package["primitive_sequence"]  # type: ignore[index]
    x_min = float(workspace["x_min_m"])  # type: ignore[index]
    x_max = float(workspace["x_max_m"])  # type: ignore[index]
    y_min = float(workspace["y_min_m"])  # type: ignore[index]
    y_max = float(workspace["y_max_m"])  # type: ignore[index]
    origin_x = x_min  # 로컬(0,0)→raw 프레임 오프셋 / offset to map local (0,0) back to raw frame
    origin_y = y_min

    def setup(title: str):
        """공통 축 세팅(작업영역 사각형·격자·미리보기 배너)을 적용한 fig/ax 반환.

        Return a fig/ax with the shared axis setup (workspace rectangle, grid,
        preview-only banner).
        """
        fig, ax = plt.subplots(figsize=(8, 5))
        ax.add_patch(Rectangle((x_min, y_min), x_max - x_min, y_max - y_min, fill=False, edgecolor="black"))
        ax.set_title(title)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.axis("equal")
        ax.grid(True, linestyle="--", alpha=0.3)
        ax.text(0.01, 0.01, "Preview only: no rover commands, no motor output", transform=ax.transAxes, fontsize=8)
        return fig, ax

    outputs: dict[str, Path | None] = {}
    fig, ax = setup("Workspace Raw A/B And Normalized A'/B'")
    raw_a = workspace["raw_A"]  # type: ignore[index]
    raw_b = workspace["raw_B"]  # type: ignore[index]
    a_prime = workspace["A_prime_top_left"]  # type: ignore[index]
    b_prime = workspace["B_prime_bottom_right"]  # type: ignore[index]
    ax.scatter([raw_a["x_m"], raw_b["x_m"]], [raw_a["y_m"], raw_b["y_m"]], color="tab:blue", label="raw A/B")
    ax.scatter([a_prime["x_m"], b_prime["x_m"]], [a_prime["y_m"], b_prime["y_m"]], color="tab:orange", label="A'/B'")
    ax.annotate("A raw", (raw_a["x_m"], raw_a["y_m"]))
    ax.annotate("B raw", (raw_b["x_m"], raw_b["y_m"]))
    ax.annotate("A' top-left", (a_prime["x_m"], a_prime["y_m"]))
    ax.annotate("B' bottom-right", (b_prime["x_m"], b_prime["y_m"]))
    ax.legend()
    fig.tight_layout()
    path = out_dir / "preview_workspace_ab_aprime_bprime.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    outputs[path.name] = path

    fig, ax = setup("Tool Path Primary: A' To B'")
    for row in tool_rows:
        sx = origin_x + float(row["tool_start_x_m"])
        sy = origin_y + float(row["tool_start_y_m"])
        ex = origin_x + float(row["tool_end_x_m"])
        ey = origin_y + float(row["tool_end_y_m"])
        active = bool(row["tool_active"])
        ax.plot([sx, ex], [sy, ey], color="tab:orange" if active else "0.55", linestyle="-" if active else "--", linewidth=2.2 if active else 1.2)
    fig.tight_layout()
    path = out_dir / "preview_tool_path_primary.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    outputs[path.name] = path

    fig, ax = setup("Primitive Sequence")
    for row in primitives:
        sx = float(row["start_x_m"])
        sy = float(row["start_y_m"])
        ex = float(row["end_x_m"])
        ey = float(row["end_y_m"])
        ptype = str(row["primitive_type"])
        if ptype.startswith("move"):
            ax.annotate("", xy=(ex, ey), xytext=(sx, sy), arrowprops={"arrowstyle": "->", "color": "tab:blue", "linewidth": 1.0})
        else:
            ax.scatter([sx], [sy], color="tab:purple", s=30)
        ax.text(sx, sy, f"P{int(row['primitive_index']):03d}", fontsize=6)
    fig.tight_layout()
    path = out_dir / "preview_primitive_sequence.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    outputs[path.name] = path

    fig, ax = setup("Approach Then Serpentine")
    for row in primitives:
        sx = float(row["start_x_m"])
        sy = float(row["start_y_m"])
        ex = float(row["end_x_m"])
        ey = float(row["end_y_m"])
        role = str(row["segment_role"])
        if str(row["primitive_type"]).startswith("move"):
            ax.plot([sx, ex], [sy, ey], color="tab:green" if role.startswith("approach") else "tab:blue", linewidth=1.2)
    fig.tight_layout()
    path = out_dir / "preview_approach_then_serpentine.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    outputs[path.name] = path
    return outputs


# ── 패키지 조립(핵심) / Package assembly (core orchestration) ──
def build_path_package(
    *,
    raw_a: tuple[float, float],
    raw_b: tuple[float, float],
    current_pose: tuple[float, float, float] | None,
    step_spacing_m: float,
    tool_side: str,
    tool_lateral_offset_m: float,
    tool_width_m: float,
    tool_length_m: float,
    robot_width_m: float,
    robot_length_m: float,
    georeference: dict[str, object] | None = None,
) -> dict[str, object]:
    """이 모듈의 핵심: raw A/B로부터 완전한 무동작 경로 패키지 dict를 조립한다.

    무엇을/왜: (1) A/B 정규화, (2) 현재 포즈→A' 접근 프리미티브, (3) 코어 플래너로
    지그재그 도구경로/섀시경로/프리미티브 생성, (4) 좌표를 raw 프레임으로 되돌리고
    접근+지그재그를 하나의 시퀀스로 결합, (5) summary/검증 플래그 채우기, (6) (옵션)
    지오참조 병합. 반환은 하류 도구가 소비하는 path_package dict.
    핵심 인자: step_spacing_m(트랙 간격), tool_side/offset/size, robot size,
    current_pose(None이면 raw_a에서 헤딩 0으로 시작), georeference(위경도 입력 시).
    부수효과: 없음(파일 쓰기는 main이 담당). 순수 조립 함수.
    불변식: primitive_sequence_valid = 모든 프리미티브가 허용 타입 & motor False.
    리팩토링 주의: 코어 반환 키(primitive_rows/tool_segments/chassis_segments/summary)와
    origin 오프셋 처리에 강하게 결합되어 있음.

    Core assembly: turn raw A/B into a full no-motion path package dict —
    normalise A/B, build the approach, delegate the serpentine to the core
    planner, shift coordinates back to the raw frame, join approach+serpentine,
    fill summary/validation flags, and optionally merge geo-reference metadata.
    Pure function (file writing happens in main).
    """
    workspace = normalize_field_ab(a_x=raw_a[0], a_y=raw_a[1], b_x=raw_b[0], b_y=raw_b[1])
    a_prime = workspace["A_prime_top_left"]  # type: ignore[index]
    b_prime = workspace["B_prime_bottom_right"]  # type: ignore[index]
    if current_pose is None:
        current_pose = (raw_a[0], raw_a[1], 0.0)
    approach = build_approach_primitives(
        current_x=current_pose[0],
        current_y=current_pose[1],
        current_heading_deg=current_pose[2],
        target_x=float(a_prime["x_m"]),
        target_y=float(a_prime["y_m"]),
    )
    config = SideToolPlanConfig(
        workspace_mode="tool_serpentine_ab",
        a_x_m=float(a_prime["x_m"]),
        a_y_m=float(a_prime["y_m"]),
        b_x_m=float(b_prime["x_m"]),
        b_y_m=float(b_prime["y_m"]),
        step_spacing_m=step_spacing_m,
        tool_side=tool_side,  # type: ignore[arg-type]
        tool_lateral_offset_m=tool_lateral_offset_m,
        tool_width_m=tool_width_m,
        tool_length_m=tool_length_m,
        robot_width_m=robot_width_m,
        robot_length_m=robot_length_m,
        contamination_mode="off",
        tool_active_on_sweep_tracks=True,
        tool_active_on_connectors=False,
    )
    preview = generate_tool_serpentine_preview(config)
    # 코어는 (0,0) 기준 로컬 좌표를 돌려줌 → origin을 더해 raw 프레임으로 복원.
    # Core returns local (0,0)-based coords; add origin to restore the raw frame.
    origin_x = float(workspace["x_min_m"])
    origin_y = float(workspace["y_min_m"])
    serpentine_primitives = []
    for row in preview["primitive_rows"]:  # type: ignore[index]
        updated = dict(row)
        for key in ("start_x_m", "end_x_m"):
            updated[key] = float(updated[key]) + origin_x
        for key in ("start_y_m", "end_y_m"):
            updated[key] = float(updated[key]) + origin_y
        serpentine_primitives.append(updated)
    # 접근 프리미티브 뒤에 인덱스를 밀어 지그재그를 이어 붙임(전역 연속 인덱스).
    # Append serpentine after approach with shifted indices (contiguous globally).
    primitives = approach + _offset_primitives(serpentine_primitives, len(approach))
    # 안전 게이트: 허용 타입만 & 모든 행 motor False 여야 유효.
    # Safety gate: only allowed types AND every row motor-False counts as valid.
    primitive_sequence_valid = all(
        row["primitive_type"] in ALLOWED_PRIMITIVES and row["motor_command_generated"] is False
        for row in primitives
    )
    tool_path = list(preview["tool_segments"])  # type: ignore[index]
    summary = {
        "raw_A": workspace["raw_A"],
        "raw_B": workspace["raw_B"],
        "A_prime_top_left": workspace["A_prime_top_left"],
        "B_prime_bottom_right": workspace["B_prime_bottom_right"],
        "approach_path_generated": bool(approach),
        "serpentine_path_generated": True,
        "tool_side": tool_side,
        "step_spacing_m": step_spacing_m,
        "primitive_sequence_valid": primitive_sequence_valid,
        "motor_command_generated": False,
        "physical_output_active": False,
        "ready_for_outdoor_no_motion_validation": primitive_sequence_valid,
        "georeference_available": georeference is not None,
    }
    package = {
        "generated_at_utc": dt.datetime.now(tz=dt.UTC).isoformat(),
        "normalized_workspace": workspace,
        "approach_to_A_prime": approach,
        "tool_path": tool_path,
        "chassis_path": preview["chassis_segments"],  # type: ignore[index]
        "primitive_sequence": primitives,
        "simple_serpentine_summary": preview["summary"],  # type: ignore[index]
        "summary": summary,
        "path_preview_only": True,
        "motor_command_generated": False,
        "physical_output_active": False,
    }
    if georeference is not None:
        package["georeference"] = georeference
        package["normalized_workspace"]["georeference_available"] = True  # type: ignore[index]
        package["summary"]["georeference_available"] = True  # type: ignore[index]
        for key in (
            "raw_A_lat",
            "raw_A_lon",
            "raw_B_lat",
            "raw_B_lon",
            "origin_lat",
            "origin_lon",
            "local_frame_type",
            "x_axis_source",
            "x_axis_bearing_deg",
            "meters_per_deg_lat",
            "meters_per_deg_lon",
            "meters_per_lat",
            "meters_per_lon",
        ):
            package["summary"][key] = georeference[key]  # type: ignore[index]
    return package


# ── CLI 진입점 / CLI entry point (argument parsing, main) ──
def build_parser() -> argparse.ArgumentParser:
    """CLI 인자 파서를 구성해 반환 / Build and return the CLI argument parser."""
    parser = argparse.ArgumentParser(
        description="Convert captured field A/B points into a no-motion serpentine path package."
    )
    parser.add_argument("--field-points-json")
    parser.add_argument("--field-points-georef-json")
    parser.add_argument("--a-x", type=float)
    parser.add_argument("--a-y", type=float)
    parser.add_argument("--b-x", type=float)
    parser.add_argument("--b-y", type=float)
    parser.add_argument("--current-x", type=float)
    parser.add_argument("--current-y", type=float)
    parser.add_argument("--current-heading-deg", type=float, default=0.0)
    parser.add_argument("--step-spacing-m", type=float, required=True)
    parser.add_argument("--tool-side", choices=("left", "right"), default="left")
    parser.add_argument("--tool-lateral-offset-m", type=float, default=0.24)
    parser.add_argument("--tool-width-m", type=float, default=0.30)
    parser.add_argument("--tool-length-m", type=float, default=0.18)
    parser.add_argument("--robot-width-m", type=float, default=0.18)
    parser.add_argument("--robot-length-m", type=float, default=0.18)
    parser.add_argument("--out-dir", default="outputs/field_ab_serpentine/latest")
    return parser


def _resolve_points(args: argparse.Namespace) -> tuple[tuple[float, float, float, float], dict[str, object] | None]:
    """세 입력 방식(위경도 JSON / 로컬 JSON / 수동 값) 중 우선순위대로 A/B를 확정.

    무엇을/왜: 위경도 JSON이 있으면 그것을 우선하고(지오참조 dict 동반), 다음 로컬 JSON,
    마지막으로 수동 --a-x/--a-y/--b-x/--b-y. 아무것도 없으면 ValueError.
    반환: ((a_x, a_y, b_x, b_y), georeference|None).

    Resolve A/B from one of three input modes in priority order (geo-ref JSON,
    local JSON, manual values); raises ValueError if none is provided.
    """
    if args.field_points_georef_json:
        return _load_georef_points(Path(args.field_points_georef_json))
    if args.field_points_json:
        return _load_field_points(Path(args.field_points_json)), None
    missing = [
        name
        for name in ("a_x", "a_y", "b_x", "b_y")
        if getattr(args, name) is None
    ]
    if missing:
        raise ValueError("--field-points-json or all manual --a-x/--a-y/--b-x/--b-y values are required")
    return (float(args.a_x), float(args.a_y), float(args.b_x), float(args.b_y)), None


def main(argv: Sequence[str] | None = None) -> int:
    """CLI 진입점: 인자 파싱→패키지 조립→산출물 기록→요약 출력, 종료코드 0 반환.

    부수효과: out_dir에 JSON/CSV/Markdown/PNG를 쓰고 표준출력에 경로/플래그를 찍는다.
    모터 명령·물리 출력은 없음.

    CLI entry point: parse args, build the package, write artifacts, print a
    summary. Side effects: writes files under out_dir and prints paths/flags.
    """
    args = build_parser().parse_args(argv)
    raw, georeference = _resolve_points(args)
    current_pose = None
    if args.current_x is not None and args.current_y is not None:
        current_pose = (float(args.current_x), float(args.current_y), float(args.current_heading_deg))
    package = build_path_package(
        raw_a=(raw[0], raw[1]),
        raw_b=(raw[2], raw[3]),
        current_pose=current_pose,
        step_spacing_m=args.step_spacing_m,
        tool_side=args.tool_side,
        tool_lateral_offset_m=args.tool_lateral_offset_m,
        tool_width_m=args.tool_width_m,
        tool_length_m=args.tool_length_m,
        robot_width_m=args.robot_width_m,
        robot_length_m=args.robot_length_m,
        georeference=georeference,
    )
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    workspace_path = out_dir / "normalized_workspace.json"
    tool_csv = out_dir / "tool_path.csv"
    primitive_csv = out_dir / "primitive_sequence.csv"
    package_path = out_dir / "path_package.json"
    summary_path = out_dir / "summary.md"
    _write_json(workspace_path, package["normalized_workspace"])  # type: ignore[arg-type]
    _write_csv(tool_csv, package["tool_path"], TOOL_PATH_FIELDS)  # type: ignore[arg-type]
    _write_csv(primitive_csv, package["primitive_sequence"], PRIMITIVE_FIELDS)  # type: ignore[arg-type]
    _write_json(package_path, package)
    _write_summary(summary_path, package)
    preview_paths = _write_previews(out_dir, package)
    print("Field A/B serpentine path package generated.")
    print("Preview/no-motion only: no rover commands, no motor output, no firmware upload.")
    print(f"normalized_workspace_json: {workspace_path}")
    print(f"tool_path_csv: {tool_csv}")
    print(f"primitive_sequence_csv: {primitive_csv}")
    print(f"path_package_json: {package_path}")
    print(f"summary_md: {summary_path}")
    for name, path in preview_paths.items():
        print(f"{name}: {path if path is not None else 'skipped'}")
    summary = package["summary"]  # type: ignore[index]
    print(f"A_prime_top_left: {summary['A_prime_top_left']}")
    print(f"B_prime_bottom_right: {summary['B_prime_bottom_right']}")
    print(f"primitive_sequence_valid: {summary['primitive_sequence_valid']}")
    print(f"motor_command_generated: {summary['motor_command_generated']}")
    print(f"physical_output_active: {summary['physical_output_active']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
