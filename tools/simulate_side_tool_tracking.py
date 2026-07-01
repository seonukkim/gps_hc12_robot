"""사이드툴 경로 추종 시뮬레이션 / Offline side-tool path-tracking simulation.

목적/역할:
    사이드툴 경로 미리보기 CSV(side_tool_path.csv)를 입력받아, 각 구간에서의 목표 방위·헤딩
    오차·누적 진행거리 등 '가상 추종 진단'을 계산해 CSV와 마크다운 요약으로 내보낸다. 실제 제어
    루프가 아니라 기하학 기반의 오프라인 시뮬레이션이며, 모터 명령을 절대 만들지 않는다.

    Reads a side-tool path preview CSV and computes *virtual* tracking diagnostics per
    segment (target bearing, heading error, cumulative progress) into a CSV plus a markdown
    summary. It is a geometry-based offline simulation, not a real control loop, and never
    generates motor commands.

시스템 내 위치:
    `tools/` CLI 진입점. 입력 CSV는 `tools/side_tool_path_preview.py`(래퍼
    `preview_side_tool_path.py`)가 만든다. 웨이포인트 진단만 필요하면
    `tools/preview_side_tool_waypoints.py`를 참고. _bootstrap은 경로 설정용이며, 그 외 프로젝트
    모듈 의존성은 없다(표준 라이브러리만 사용).

    Entry-point script in `tools/`. Its input CSV is produced by
    `tools/side_tool_path_preview.py` (wrapper `preview_side_tool_path.py`); see
    `preview_side_tool_waypoints.py` for waypoint-only diagnostics. `_bootstrap` sets up
    import paths; otherwise it uses only the standard library.

핵심 개념·불변식:
    - target_bearing_deg는 다음 행으로의 방위(도)이고, heading_error_deg는 그 방위와 현재
      heading_deg의 차이를 (-180, 180]로 정규화한 값이다.
    - 마지막 행은 다음 행이 없어 target_*/heading_error가 "NA"가 된다.
    - virtual_desired_forward_cmd는 motion_direction에 따라 ±0.10의 고정 진단값이고,
      cross_track_error_m·turn_cmd는 항상 0으로 둔다(순수 기하 추종 가정).
    - `motor_command_generated`는 항상 False, feedback_source는 "offline_preview_geometry".

    - target_bearing_deg is the bearing to the next row; heading_error_deg is that bearing
      minus the current heading, normalized to (-180, 180].
    - The last row has no successor, so its target_*/heading_error become "NA".
    - virtual_desired_forward_cmd is a fixed ±0.10 diagnostic per motion_direction;
      cross-track error and turn command are always 0. `motor_command_generated` is always
      False; feedback_source is "offline_preview_geometry".

사용법/진입점:
    CLI 진입점은 main(). 예:
    `python -m tools.simulate_side_tool_tracking --path-csv outputs/.../side_tool_path.csv`.
    출력은 tracking_errors.csv와 summary.md.

    CLI entry point is main(); pass --path-csv. Emits tracking_errors.csv and summary.md.

리팩토링 노트:
    출력 CSV 컬럼 순서·이름은 CSV_FIELDS로 고정. 입력 컬럼(x_m, y_m, heading_deg,
    motion_direction 등)은 미리보기 CSV 스키마와 결합되어 있으니 함께 확인할 것. 방위 관례는
    preview_side_tool_waypoints의 플래너 프레임과 일치해야 한다. 안전 플래그 유지.

    Output CSV columns are pinned by CSV_FIELDS; input columns are coupled to the preview CSV
    schema. The bearing convention must match the planner frame used elsewhere. Keep the
    safety flags intact.
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


CSV_FIELDS = (
    "segment_index",
    "segment_type",
    "lane_index",
    "x_m",
    "y_m",
    "heading_deg",
    "motion_direction",
    "travel_direction_deg",
    "target_x_m",
    "target_y_m",
    "target_bearing_deg",
    "heading_error_deg",
    "cross_track_error_m",
    "along_track_progress_m",
    "virtual_desired_forward_cmd",
    "virtual_desired_turn_cmd",
    "feedback_source",
    "motor_command_generated",
)


def _normalize_deg(angle_deg: float) -> float:
    """각도를 (-180, 180] 범위로 정규화한다 / wrap an angle into (-180, 180]."""
    return ((angle_deg + 180.0) % 360.0) - 180.0


def _float(row: dict[str, str], key: str, default: float = 0.0) -> float:
    """CSV 행에서 키를 float로 읽되 빈값/NA/None은 기본값으로 대체한다.

    Read a key from a CSV row as float; empty/NA/None fall back to default.
    """
    value = row.get(key, "")
    if value in {"", "NA", "None"}:
        return default
    return float(value)


def build_tracking_rows(
    preview_rows: Sequence[dict[str, str]],
) -> list[dict[str, str | int | float | bool]]:
    """미리보기 행 시퀀스로부터 가상 추종 진단 행들을 계산한다.

    Compute virtual tracking-diagnostic rows from the preview row sequence.
    구간별 방위·헤딩 오차·누적 진행거리를 채운다 / fills bearing, heading error, progress.
    부수효과 없음(순수 함수) / no side effects (pure).
    """
    rows: list[dict[str, str | int | float | bool]] = []
    cumulative = 0.0
    for index, row in enumerate(preview_rows):
        next_row = preview_rows[index + 1] if index + 1 < len(preview_rows) else None
        x = _float(row, "x_m")
        y = _float(row, "y_m")
        heading = _float(row, "heading_deg")
        # 마지막 행은 다음 목표가 없으므로 타깃/오차를 NA로 둔다 / last row -> NA targets
        if next_row is None:
            target_x = "NA"
            target_y = "NA"
            target_bearing = "NA"
            heading_error = "NA"
            step_distance = 0.0
        else:
            nx = _float(next_row, "x_m")
            ny = _float(next_row, "y_m")
            dx = nx - x
            dy = ny - y
            step_distance = math.hypot(dx, dy)
            target_x = f"{nx:.3f}"
            target_y = f"{ny:.3f}"
            if step_distance > 0.0:
                bearing = math.degrees(math.atan2(dy, dx))
                target_bearing = f"{bearing:.3f}"
                heading_error = f"{_normalize_deg(bearing - heading):.3f}"
            else:
                target_bearing = "NA"
                heading_error = "NA"
        cumulative += step_distance
        motion = row.get("motion_direction", "forward")
        # 실제 속도가 아니라 방향만 나타내는 고정 진단값(±0.10) / fixed sign-only diagnostic
        virtual_forward = 0.10 if motion == "forward" else -0.10
        rows.append(
            {
                "segment_index": row.get("segment_index", row.get("index", index)),
                "segment_type": row.get("segment_type", "unknown"),
                "lane_index": row.get("lane_index", -1),
                "x_m": f"{x:.3f}",
                "y_m": f"{y:.3f}",
                "heading_deg": f"{heading:.3f}",
                "motion_direction": motion,
                "travel_direction_deg": row.get("travel_direction_deg", "NA"),
                "target_x_m": target_x,
                "target_y_m": target_y,
                "target_bearing_deg": target_bearing,
                "heading_error_deg": heading_error,
                "cross_track_error_m": "0.000",
                "along_track_progress_m": f"{cumulative:.3f}",
                "virtual_desired_forward_cmd": f"{virtual_forward:.3f}",
                "virtual_desired_turn_cmd": "0.000",
                "feedback_source": "offline_preview_geometry",
                "motor_command_generated": False,
            }
        )
    return rows


def build_parser() -> argparse.ArgumentParser:
    """이 도구의 argparse 파서를 구성한다 / build the argparse parser for this tool."""
    parser = argparse.ArgumentParser(
        description=(
            "Simulate side-tool path tracking from an offline preview CSV. "
            "This produces virtual diagnostics only and never generates motor commands."
        )
    )
    parser.add_argument("--path-csv", required=True, help="side_tool_path.csv from side_tool_path_preview.py")
    parser.add_argument("--out-dir", default="outputs/side_tool_tracking_sim")
    return parser


def _read_preview(path: Path) -> list[dict[str, str]]:
    """미리보기 CSV를 행 dict 리스트로 읽는다 / read the preview CSV into row dicts."""
    with path.open(encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def _write_csv(path: Path, rows: Sequence[dict[str, str | int | float | bool]]) -> None:
    """추종 진단 행들을 CSV_FIELDS 순서로 CSV 파일에 기록한다.

    Write tracking-diagnostic rows to a CSV file in CSV_FIELDS order.
    부수효과: 파일 기록 / side effect: writes the file.
    """
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row[field] for field in CSV_FIELDS})


def _write_summary(path: Path, rows: Sequence[dict[str, str | int | float | bool]], csv_path: Path) -> None:
    """시뮬레이션 결과·안전 고지를 담은 마크다운 요약을 기록한다.

    Write a markdown summary of the simulation results and safety notices.
    부수효과: 파일 기록 / side effect: writes the file.
    """
    lines = [
        "# Side-Tool Tracking Simulation",
        "",
        "This is an offline no-motion simulation artifact.",
        "",
        "- No firmware is uploaded.",
        "- No serial ports are opened.",
        "- No HC-12 frames are sent.",
        "- No rover motor commands are generated.",
        "- `virtual_desired_forward_cmd` and `virtual_desired_turn_cmd` are diagnostic values only.",
        "",
        "## Output",
        "",
        f"- tracking_rows: `{len(rows)}`",
        "- motor_command_generated: `False`",
        "- feedback_tracking_ready: `simulation_only`",
        f"- generated_at_utc: `{dt.datetime.now(tz=dt.UTC).isoformat()}`",
        f"- tracking_errors_csv: `{csv_path}`",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main(argv: Sequence[str] | None = None) -> int:
    """CLI 진입점: 미리보기 CSV 읽기→추종 행 산출→CSV/요약 기록→출력. 항상 0 반환.

    CLI entry point: read preview CSV, derive tracking rows, write CSV/summary; returns 0.
    """
    args = build_parser().parse_args(argv)
    preview_path = Path(args.path_csv)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    rows = build_tracking_rows(_read_preview(preview_path))
    csv_path = out_dir / "tracking_errors.csv"
    summary_path = out_dir / "summary.md"
    _write_csv(csv_path, rows)
    _write_summary(summary_path, rows, csv_path)
    print("Side-tool tracking simulation generated.")
    print("Simulation only: no rover commands, no motor output, no firmware upload.")
    print(f"CSV: {csv_path}")
    print(f"Summary: {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
