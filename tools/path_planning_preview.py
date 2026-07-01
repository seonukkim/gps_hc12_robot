"""스테이션 측 시작→목표(start→goal) 경로 미리보기 생성기(위경도 입력).

목적/역할:
    시작/목표 위경도 두 점을 받아, 일정 간격(spacing)으로 나눈 직선 웨이포인트 목록을
    만든다. 각 웨이포인트는 위경도와 로컬 미터(ENU) 좌표, 시작점으로부터의 거리,
    구간 유형(start/intermediate/goal)을 갖는다. 산출물은 CSV/Markdown/(선택)PNG.
    **미리보기 전용**: HC-12나 로버 명령을 보내지 않고 펌웨어도 건드리지 않는다.

시스템 내 위치:
    - 코어 지오 유틸 `gps_coverage_core.geo`(GeoPoint/LocalPoint/latlon_to_local/
      local_to_latlon)를 사용해 좌표를 변환한다.
    - 필드 A/B 지그재그 파이프라인(field_ab_to_serpentine 계열)과는 별개의, 단순
      두 점 사이 직선 경로 미리보기 도구다.

핵심 개념·불변식(invariant):
    - 좌표계: 시작점을 원점으로 하는 로컬 ENU(x=동, y=북). 변환은 코어 geo에 위임.
    - 간격 규칙: 0..total_distance를 spacing으로 채우고, 마지막에 정확히 목표점을
      한 번 추가한다(부동소수 근접은 isclose로 처리해 중복/누락 방지).
    - 검증: 위도[-90,90]/경도[-180,180] 범위, spacing>0, 시작≠목표. 위반 시 ValueError.

사용법/진입점:
    CLI. `main()`이 진입점. 예:
    `python tools/path_planning_preview.py --start-lat .. --start-lon .. --goal-lat .. --goal-lon .. --spacing-m 2`.

리팩토링 노트:
    - CSV_FIELDS 순서는 산출물 계약. PNG는 matplotlib 없으면 조용히 생략(선택 의존성).
    - 좌표 변환 규약이 바뀌면 코어 geo와 이 파일의 웨이포인트 의미가 함께 흔들린다.

Station-side start->goal path preview generator (lat/lon input). Splits the
straight line between two points into evenly spaced waypoints, each with
lat/lon, local ENU metres, distance-from-start, and a start/intermediate/goal
label. Emits CSV/Markdown and an optional PNG. Preview-only: no HC-12, no rover
commands, no firmware changes. Coordinate conversion is delegated to
`gps_coverage_core.geo`.
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

from gps_coverage_core.geo import GeoPoint, LocalPoint, latlon_to_local, local_to_latlon


# waypoints.csv의 컬럼 순서 = 산출물 계약 / CSV column order is an output contract.
CSV_FIELDS = (
    "index",
    "lat",
    "lon",
    "x_m",
    "y_m",
    "distance_from_start_m",
    "segment_type",
)


def build_parser() -> argparse.ArgumentParser:
    """CLI 인자 파서를 구성해 반환 / Build and return the CLI argument parser."""
    parser = argparse.ArgumentParser(
        description=(
            "Generate a station-side start/goal path preview. "
            "This is preview-only and sends no HC-12 or rover commands."
        )
    )
    parser.add_argument("--start-lat", type=float, required=True, help="Start latitude")
    parser.add_argument("--start-lon", type=float, required=True, help="Start longitude")
    parser.add_argument("--goal-lat", type=float, required=True, help="Goal latitude")
    parser.add_argument("--goal-lon", type=float, required=True, help="Goal longitude")
    parser.add_argument("--spacing-m", type=float, default=2.0, help="Waypoint spacing in meters")
    parser.add_argument(
        "--out-dir",
        default="outputs/path_preview",
        help="Output directory for waypoints.csv, summary.md, and optional preview.png",
    )
    return parser


def _validate_lat_lon(lat: float, lon: float, label: str) -> None:
    """위경도 범위를 검증(위반 시 ValueError) / Validate lat/lon ranges, raising ValueError."""
    if not -90.0 <= lat <= 90.0:
        raise ValueError(f"{label} latitude must be between -90 and 90")
    if not -180.0 <= lon <= 180.0:
        raise ValueError(f"{label} longitude must be between -180 and 180")


def build_waypoints(
    *,
    start: GeoPoint,
    goal: GeoPoint,
    spacing_m: float,
) -> list[dict[str, float | int | str]]:
    """시작→목표 직선을 spacing 간격 웨이포인트 목록으로 변환한다.

    무엇을/왜: 목표를 로컬 ENU로 변환해 총거리와 단위벡터를 구한 뒤, 0부터 spacing씩
    쌓아 거리 목록을 만들고 마지막에 정확히 목표점을 넣는다. 각 거리마다 로컬→위경도로
    되돌려 웨이포인트 dict(index/lat/lon/x_m/y_m/거리/구간유형)를 만든다.
    반환: 웨이포인트 dict 리스트(첫=start, 끝=goal).
    부수효과: 없음. spacing<=0, 위경도 범위 위반, 시작==목표면 ValueError.
    함정: 부동소수 근접은 math.isclose로 처리해 마지막 간격 중복/누락을 막는다.

    Split the start->goal straight line into evenly spaced waypoints (local ENU
    plus lat/lon), always ending exactly at the goal. Pure function; raises
    ValueError on bad spacing/ranges or identical endpoints.
    """
    if spacing_m <= 0.0:
        raise ValueError("spacing_m must be positive")
    _validate_lat_lon(start.lat, start.lon, "start")
    _validate_lat_lon(goal.lat, goal.lon, "goal")

    goal_local = latlon_to_local(start, goal)
    total_distance_m = math.hypot(goal_local.x_m, goal_local.y_m)
    if math.isclose(total_distance_m, 0.0, abs_tol=1e-9):
        raise ValueError("start and goal must not be identical")

    unit_x = goal_local.x_m / total_distance_m
    unit_y = goal_local.y_m / total_distance_m
    # 0부터 spacing씩 채우되, 목표에 근접한 지점은 건너뛴다(마지막에 정확히 목표를 넣기 위함).
    # Step from 0 by spacing, skipping points near the goal so we end exactly on it.
    distances = [0.0]
    next_distance = spacing_m
    while next_distance < total_distance_m and not math.isclose(
        next_distance, total_distance_m, abs_tol=1e-9
    ):
        distances.append(next_distance)
        next_distance += spacing_m
    # 마지막 점이 이미 목표가 아니면 목표를 정확히 한 번 추가 / append the exact goal if not already there.
    if not math.isclose(distances[-1], total_distance_m, abs_tol=1e-9):
        distances.append(total_distance_m)

    waypoints: list[dict[str, float | int | str]] = []
    for index, distance_m in enumerate(distances):
        local = LocalPoint(x_m=unit_x * distance_m, y_m=unit_y * distance_m)
        point = local_to_latlon(start, local)
        if index == 0:
            segment_type = "start"
        elif index == len(distances) - 1:
            segment_type = "goal"
        else:
            segment_type = "intermediate"
        waypoints.append(
            {
                "index": index,
                "lat": point.lat,
                "lon": point.lon,
                "x_m": local.x_m,
                "y_m": local.y_m,
                "distance_from_start_m": distance_m,
                "segment_type": segment_type,
            }
        )
    return waypoints


# ── 산출물 writer / Artifact writers (CSV, Markdown, optional PNG) ──
def _write_csv(path: Path, waypoints: Sequence[dict[str, float | int | str]]) -> None:
    """웨이포인트를 CSV_FIELDS 순서로 CSV에 기록 / Write waypoints to CSV in CSV_FIELDS order."""
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for waypoint in waypoints:
            writer.writerow({field: waypoint[field] for field in CSV_FIELDS})


def _format_optional_png(path: Path | None) -> str:
    """PNG 경로 또는 미생성 사유 문자열 / PNG path, or a not-generated reason string."""
    if path is None:
        return "not generated; matplotlib is not available"
    return str(path)


def _write_summary(
    *,
    path: Path,
    start: GeoPoint,
    goal: GeoPoint,
    spacing_m: float,
    waypoints: Sequence[dict[str, float | int | str]],
    csv_path: Path,
    png_path: Path | None,
) -> None:
    """입력·요약·다음 검증단계를 담은 Markdown 요약을 기록한다.

    무엇을/왜: 미리보기 전용임을 명시하는 배너와 함께 입력(시작/목표/간격), 총거리·
    웨이포인트 수, 산출물 경로, 그리고 실물 추종 전 필요한 후속 검증 단계를 남긴다.

    Write the Markdown summary (preview-only banner, inputs, totals, output
    paths, and the follow-up validation steps required before physical following).
    """
    total_distance_m = float(waypoints[-1]["distance_from_start_m"])
    lines = [
        "# Station Path Planning Preview",
        "",
        "This artifact is preview-only.",
        "",
        "- No HC-12 commands are sent.",
        "- No rover motor commands are generated.",
        "- No firmware is uploaded or modified.",
        "- This is not autonomous execution.",
        "- Physical path following still requires heading/course estimation and steering validation.",
        "",
        "## Inputs",
        "",
        f"- start: `{start.lat:.7f}, {start.lon:.7f}`",
        f"- goal: `{goal.lat:.7f}, {goal.lon:.7f}`",
        f"- spacing_m: `{spacing_m:.3f}`",
        "",
        "## Summary",
        "",
        f"- total_distance_m: `{total_distance_m:.3f}`",
        f"- waypoint_count: `{len(waypoints)}`",
        f"- generated_at_utc: `{dt.datetime.now(tz=dt.UTC).isoformat()}`",
        f"- waypoints_csv: `{csv_path}`",
        f"- preview_png: `{_format_optional_png(png_path)}`",
        "",
        "## Next Required Validation",
        "",
        "1. Use this output only for station-side visual inspection.",
        "2. Add single-waypoint steering dry-run before physical waypoint following.",
        "3. Add heading/course estimation before any rover follows this path.",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_png(
    *,
    path: Path,
    waypoints: Sequence[dict[str, float | int | str]],
    start: GeoPoint,
    goal: GeoPoint,
) -> Path | None:
    """로컬 좌표로 경로 미리보기 PNG를 저장(시작/목표 강조). matplotlib 없으면 None.

    무엇을/왜: 스테이션 측 시각 검수용 그림. 선택 의존성이므로 matplotlib이 없으면
    조용히 None을 반환해 파이프라인을 막지 않는다.
    반환: 저장한 Path 또는 None.

    Save a preview PNG in local coordinates (start/goal highlighted), or None if
    matplotlib is unavailable (optional dependency).
    """
    try:
        import matplotlib

        matplotlib.use("Agg")  # 헤드리스 백엔드 / headless backend
        import matplotlib.pyplot as plt
    except ImportError:
        return None

    xs = [float(waypoint["x_m"]) for waypoint in waypoints]
    ys = [float(waypoint["y_m"]) for waypoint in waypoints]
    fig, ax = plt.subplots(figsize=(7, 5))
    ax.plot(xs, ys, marker="o", linewidth=1.5, label="preview path")
    for waypoint in waypoints:
        ax.annotate(str(waypoint["index"]), (float(waypoint["x_m"]), float(waypoint["y_m"])))
    ax.scatter([xs[0]], [ys[0]], c="green", s=70, label="start")
    ax.scatter([xs[-1]], [ys[-1]], c="red", s=70, label="goal")
    ax.set_title("Station-Side Path Preview Only")
    ax.set_xlabel("East from start (m)")
    ax.set_ylabel("North from start (m)")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.axis("equal")
    ax.legend()
    ax.text(
        0.01,
        0.01,
        "No HC-12, no rover commands, no firmware upload",
        transform=ax.transAxes,
        fontsize=8,
        va="bottom",
    )
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    _ = (start, goal)  # 시그니처 유지용(현재 미사용) / kept for signature; currently unused
    return path


def main(argv: Sequence[str] | None = None) -> int:
    """CLI 진입점: 인자 파싱→웨이포인트 생성→CSV/PNG/요약 기록·출력, 종료코드 0.

    부수효과: out_dir에 waypoints.csv/summary.md/(선택)preview.png를 쓰고 경로를 출력.
    HC-12/로버 명령·펌웨어 업로드 없음.

    CLI entry point: parse args, build waypoints, write CSV/PNG/summary and print
    paths. No HC-12/rover commands, no firmware upload.
    """
    args = build_parser().parse_args(argv)
    start = GeoPoint(lat=args.start_lat, lon=args.start_lon)
    goal = GeoPoint(lat=args.goal_lat, lon=args.goal_lon)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    waypoints = build_waypoints(start=start, goal=goal, spacing_m=args.spacing_m)
    csv_path = out_dir / "waypoints.csv"
    summary_path = out_dir / "summary.md"
    png_candidate = out_dir / "preview.png"

    _write_csv(csv_path, waypoints)
    png_path = _write_png(path=png_candidate, waypoints=waypoints, start=start, goal=goal)
    _write_summary(
        path=summary_path,
        start=start,
        goal=goal,
        spacing_m=args.spacing_m,
        waypoints=waypoints,
        csv_path=csv_path,
        png_path=png_path,
    )

    print("Station path planning preview generated.")
    print("Preview only: no HC-12 commands, no rover motor commands, no firmware upload.")
    print(f"Waypoints CSV: {csv_path}")
    print(f"Summary: {summary_path}")
    if png_path is None:
        print("Preview PNG: skipped because matplotlib is not available")
    else:
        print(f"Preview PNG: {png_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
