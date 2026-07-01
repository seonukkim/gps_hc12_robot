"""사이드툴 웨이포인트 미리보기 내보내기 / Export side-tool preview poses as target waypoints.

목적/역할:
    사이드툴 경로 플래너가 만든 오프라인 포즈 시퀀스를 '타깃 웨이포인트 진단' CSV와 마크다운
    요약으로 내보낸다. 각 포즈에 대해 다음 포즈로의 플래너 프레임 방위(target_bearing_deg),
    기대 로버 헤딩, 후진 여부, 툴 위치 등을 계산해 기록한다. 펌웨어 업로드·HC-12 전송·모터 명령을
    절대 하지 않는(무동작 진단) 도구다.

    Exports the offline pose sequence from the side-tool path planner as a "target waypoint
    diagnostics" CSV plus a markdown summary. For each pose it records the planner-frame
    bearing to the next pose, the expected rover heading, a reverse flag, and tool position.
    It never uploads firmware, sends HC-12 frames, or generates motor commands.

시스템 내 위치:
    `tools/` CLI 진입점. 경로 생성은 `gps_coverage_core.side_tool_planner`의
    SideToolPlanConfig / generate_side_tool_path에 의존한다. 전체 경로 CSV를 원하면
    `tools/side_tool_path_preview.py`(및 래퍼 `preview_side_tool_path.py`)를, 추종 시뮬레이션은
    `tools/simulate_side_tool_tracking.py`를 참고. _bootstrap은 경로 설정용.

    Entry-point script in `tools/`. Path generation depends on SideToolPlanConfig /
    generate_side_tool_path from `gps_coverage_core.side_tool_planner`. See
    `side_tool_path_preview.py` for the full path CSV and `simulate_side_tool_tracking.py`
    for tracking simulation. `_bootstrap` sets up import paths.

핵심 개념·불변식:
    - target_bearing_deg는 '플래너 프레임' 방위다: 0°=+x, 90°=+y. 나침반 방위가 아님에 주의.
    - 각도는 항상 (-180, 180]로 정규화한다(_normalize_deg).
    - 마지막 포즈는 다음 포즈가 없어 target_* 필드가 "NA"가 된다.
    - CSV의 `motor_command_generated`는 항상 False로 기록된다(무동작 계약).
    - workspace_mode="axis_width", contamination_mode="off"로 고정 설정된다.

    - target_bearing_deg is a *planner-frame* bearing (0°=+x, 90°=+y), not a compass bearing.
    - Angles are normalized to (-180, 180] via _normalize_deg.
    - The last pose has no successor, so its target_* fields become "NA".
    - CSV `motor_command_generated` is always False (no-motion contract); the config is
      pinned to workspace_mode="axis_width", contamination_mode="off".

사용법/진입점:
    CLI 진입점은 main(). 툴/레인 기하 인자는 필수다. 예:
    `python -m tools.preview_side_tool_waypoints --tool-side left --tool-lateral-offset-m 0.5
     --tool-width-m 0.3 --lane-spacing-m 0.75 --row-length-m 10 --row-count 4`.
    출력은 side_tool_waypoints.csv와 waypoint_summary.md.

    CLI entry point is main(); tool/lane geometry args are required. Emits
    side_tool_waypoints.csv and waypoint_summary.md.

리팩토링 노트:
    CSV 컬럼 순서·이름은 CSV_FIELDS로 고정되어 있고 요약과 함께 파싱될 수 있으니 주의. 방위
    관례(플래너 프레임)를 바꾸면 추종 시뮬레이터·미리보기와 어긋난다. 안전 플래그 유지.

    CSV column order/names are pinned by CSV_FIELDS; changing the (planner-frame) bearing
    convention would desync the tracking simulator and preview. Keep the safety flags.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import math
from pathlib import Path
from typing import Sequence

try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401

from gps_coverage_core.side_tool_planner import SideToolPlanConfig, generate_side_tool_path


CSV_FIELDS = (
    "index",
    "segment_type",
    "lane_index",
    "direction",
    "x_m",
    "y_m",
    "target_x_m",
    "target_y_m",
    "target_bearing_deg",
    "expected_rover_heading_deg",
    "reverse_direction_expected",
    "transition_style",
    "tool_side",
    "tool_x_m",
    "tool_y_m",
    "motor_command_generated",
    "notes",
)


def _normalize_deg(angle_deg: float) -> float:
    """각도를 (-180, 180] 범위로 정규화한다 / wrap an angle into (-180, 180]."""
    return ((angle_deg + 180.0) % 360.0) - 180.0


def _target_bearing_deg(
    current_pose: dict[str, float | int | str | bool],
    next_pose: dict[str, float | int | str | bool] | None,
) -> float | None:
    """현재 포즈에서 다음 포즈로의 플래너 프레임 방위(도)를 반환한다.

    Return the planner-frame bearing (deg) from current pose to next pose.
    다음 포즈가 없거나 두 점이 겹치면 None / None when no next pose or zero displacement.
    """
    if next_pose is None:
        return None
    dx = float(next_pose["x_m"]) - float(current_pose["x_m"])
    dy = float(next_pose["y_m"]) - float(current_pose["y_m"])
    if math.hypot(dx, dy) == 0.0:
        return None
    # Planner-frame bearing: 0 deg points along +x, 90 deg points along +y.
    # 플래너 프레임 방위: 0°=+x축, 90°=+y축 (나침반 방위 아님) / not a compass bearing
    return _normalize_deg(math.degrees(math.atan2(dy, dx)))


def build_waypoint_rows(
    poses: Sequence[dict[str, float | int | str | bool]],
) -> list[dict[str, float | int | str | bool]]:
    """플래너 포즈 시퀀스를 웨이포인트 진단 행(dict) 리스트로 변환한다.

    Convert the planner pose sequence into waypoint-diagnostic rows.
    각 행에 다음 포즈로의 방위·기대 헤딩·후진 여부 등을 채운다 / fills bearing, heading, reverse.
    부수효과 없음(순수 함수) / no side effects (pure).
    """
    rows: list[dict[str, float | int | str | bool]] = []
    for index, pose in enumerate(poses):
        # 마지막 포즈는 다음 포즈가 없어 타깃 필드가 NA가 된다 / last pose -> target fields NA
        next_pose = poses[index + 1] if index + 1 < len(poses) else None
        target_bearing = _target_bearing_deg(pose, next_pose)
        rows.append(
            {
                "index": pose["index"],
                "segment_type": pose["segment_type"],
                "lane_index": pose["lane_index"],
                "direction": pose["direction"],
                "x_m": pose["x_m"],
                "y_m": pose["y_m"],
                "target_x_m": next_pose["x_m"] if next_pose is not None else "NA",
                "target_y_m": next_pose["y_m"] if next_pose is not None else "NA",
                "target_bearing_deg": f"{target_bearing:.3f}" if target_bearing is not None else "NA",
                "expected_rover_heading_deg": pose["heading_deg"],
                "reverse_direction_expected": pose["direction"] == "reverse",
                "transition_style": pose["transition_style"],
                "tool_side": pose["tool_side"],
                "tool_x_m": pose["tool_x_m"],
                "tool_y_m": pose["tool_y_m"],
                "motor_command_generated": False,
                "notes": pose["notes"],
            }
        )
    return rows


def build_parser() -> argparse.ArgumentParser:
    """이 도구의 argparse 파서를 구성한다 / build the argparse parser for this tool."""
    parser = argparse.ArgumentParser(
        description=(
            "Export offline side-tool preview poses as target waypoint diagnostics. "
            "This never uploads firmware, sends HC-12 frames, or generates motor commands."
        )
    )
    parser.add_argument("--tool-side", choices=("left", "right"), required=True)
    parser.add_argument("--tool-lateral-offset-m", type=float, required=True)
    parser.add_argument("--tool-width-m", type=float, required=True)
    parser.add_argument("--lane-spacing-m", type=float, required=True)
    parser.add_argument("--row-length-m", type=float, required=True)
    parser.add_argument("--row-count", type=int, required=True)
    parser.add_argument("--start-x-m", type=float, default=0.0)
    parser.add_argument("--start-y-m", type=float, default=0.0)
    parser.add_argument("--start-heading-deg", type=float, default=0.0)
    parser.add_argument("--first-lane-direction", choices=("forward", "reverse"), default="forward")
    parser.add_argument("--transition-style", choices=("side-step-reverse-90",), default="side-step-reverse-90")
    parser.add_argument(
        "--out-dir",
        default="outputs/side_tool_waypoint_preview",
        help="Output directory for side_tool_waypoints.csv and waypoint_summary.md",
    )
    return parser


def _write_csv(path: Path, rows: Sequence[dict[str, float | int | str | bool]]) -> None:
    """웨이포인트 행들을 CSV_FIELDS 순서로 CSV 파일에 기록한다.

    Write waypoint rows to a CSV file in CSV_FIELDS order.
    부수효과: 파일 기록 / side effect: writes the file.
    """
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row[field] for field in CSV_FIELDS})


def _write_summary(
    *,
    path: Path,
    config: SideToolPlanConfig,
    rows: Sequence[dict[str, float | int | str | bool]],
    csv_path: Path,
) -> None:
    """입력·출력 의미론·안전 고지를 담은 마크다운 요약을 기록한다.

    Write a markdown summary covering inputs, output semantics, and safety notices.
    부수효과: 파일 기록 / side effect: writes the file.
    """
    reverse_count = sum(1 for row in rows if row["reverse_direction_expected"] is True)
    lines = [
        "# Side-Tool Waypoint Preview",
        "",
        "This is an offline no-motion diagnostic artifact.",
        "",
        "- No HC-12 frames are sent.",
        "- No firmware is uploaded or modified.",
        "- No rover motor commands are generated.",
        "- These waypoints are not approved for physical path following.",
        "",
        "## Inputs",
        "",
        f"- tool_side: `{config.tool_side}`",
        f"- tool_lateral_offset_m: `{config.tool_lateral_offset_m:.3f}`",
        f"- tool_width_m: `{config.tool_width_m:.3f}`",
        f"- lane_spacing_m: `{config.lane_spacing_m:.3f}`",
        f"- row_length_m: `{config.row_length_m:.3f}`",
        f"- row_count: `{config.row_count}`",
        f"- first_lane_direction: `{config.first_lane_direction}`",
        f"- transition_style: `{config.transition_style}`",
        "",
        "## Output Semantics",
        "",
        "- `target_bearing_deg` is a planner-frame bearing to the next preview pose.",
        "- `expected_rover_heading_deg` is the geometric pose heading for diagnostics.",
        "- `reverse_direction_expected=true` marks reverse lanes and reverse offset transitions.",
        "- `motor_command_generated` is always `False`.",
        "",
        "## Outputs",
        "",
        f"- waypoint_rows: `{len(rows)}`",
        f"- reverse_expected_rows: `{reverse_count}`",
        f"- generated_at_utc: `{dt.datetime.now(tz=dt.UTC).isoformat()}`",
        f"- csv: `{csv_path}`",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main(argv: Sequence[str] | None = None) -> int:
    """CLI 진입점: 설정 구성→경로 생성→웨이포인트 행 산출→CSV/요약 기록. 항상 0 반환.

    CLI entry point: build config, generate path, derive rows, write CSV/summary; returns 0.
    """
    args = build_parser().parse_args(argv)
    config = SideToolPlanConfig(
        workspace_mode="axis_width",
        tool_side=args.tool_side,
        tool_lateral_offset_m=args.tool_lateral_offset_m,
        tool_width_m=args.tool_width_m,
        lane_spacing_m=args.lane_spacing_m,
        row_length_m=args.row_length_m,
        row_count=args.row_count,
        start_x_m=args.start_x_m,
        start_y_m=args.start_y_m,
        start_heading_deg=args.start_heading_deg,
        first_lane_direction=args.first_lane_direction,
        transition_style=args.transition_style,
        contamination_mode="off",
        fail_on_contamination_violation=False,
    )
    rows = build_waypoint_rows(generate_side_tool_path(config))
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / "side_tool_waypoints.csv"
    summary_path = out_dir / "waypoint_summary.md"
    _write_csv(csv_path, rows)
    _write_summary(path=summary_path, config=config, rows=rows, csv_path=csv_path)

    print("Side-tool waypoint preview generated.")
    print("Preview only: no HC-12 commands, no rover motor commands, no firmware upload.")
    print(f"CSV: {csv_path}")
    print(f"Summary: {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
