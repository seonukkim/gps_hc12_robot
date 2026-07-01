"""A/B 구역·잔디깎기 경로 리포트 그림 생성 / Report figures for A/B region and lawnmower path.

목적/역할:
    목(mock) 미션 웨이포인트로부터 리포트/논문용 그림 3종을 만든다 — (1) 운용자가 고른 A/B
    두 점으로 정의한 평면 작업 구역, (2) 생성된 잔디깎기(사행) 커버리지 경로 미리보기,
    (3) 웨이포인트 방문 순서. 모두 "계획 기하"를 보여줄 뿐 실제 필드 주행 결과가 아니다
    (그림 캡션·패널 라벨에 MOCK 표기가 강제된다).

시스템 내 위치:
    - `scripts/analysis/generate_all_figures.py`가 이 모듈의 generate()를 호출하는 상위 오케스트레이터.
    - 공용 유틸(색상, 축 정리, 저장, 데이터 로딩)은 `_figure_common.py`에서 import한다.
    - 웨이포인트는 `_figure_common.load_waypoint_dataset()`가 data/mock_runs/**/waypoints.* 를
      읽거나, 없으면 `gps_coverage_core.planner`로 결정론적 목 미션을 만들어 공급한다.

핵심 개념·불변식:
    - 웨이포인트 dict는 order/lane/x/y/lat/lon 키를 가진다(_figure_common의 로더 계약).
    - x=East(m), y=North(m) 로컬 평면. 그림 축 라벨도 이 규약을 따른다.
    - 각 _generate_* 함수는 그림을 저장하고 캡션·용도를 담은 FigureResult를 반환한다(부수효과:
      save_figure_all이 여러 출력 디렉터리에 PNG를 쓴다).

사용법/진입점:
    CLI: ``python scripts/analysis/generate_path_figures.py [--waypoints ...] [--output-dirs ...]``.
    프로그램적으로는 generate(output_dirs=..., waypoint_paths=...)를 호출한다.

리팩토링 노트:
    _figure_common의 헬퍼 시그니처와 강하게 결합되어 있다. 웨이포인트 키를 바꾸면 planner와
    로더도 함께 수정해야 한다. 그림 파일명 문자열은 리포트 문서에서 참조되므로 변경 시 파급 확인.

Purpose: build three report-quality figures from mock mission waypoints — the operator-defined A/B
planar work region, a lawnmower (serpentine) coverage-path preview, and the waypoint visit order.
These illustrate planning geometry only, never a validated field run (MOCK labels are enforced).
System: called by ``generate_all_figures.py``; shared plotting/loading helpers come from
``_figure_common``; waypoints are supplied by ``load_waypoint_dataset`` (reads data/mock_runs or
falls back to a deterministic planner mission). Local frame is x=East, y=North. Each ``_generate_*``
saves a PNG (side effect) and returns a ``FigureResult`` with caption/use metadata. Coupled to the
``_figure_common`` helper signatures and waypoint key set.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Sequence

from _figure_common import (
    COLOR_BLUE,
    COLOR_GRAY,
    COLOR_GREEN,
    COLOR_LIGHT,
    COLOR_NAVY,
    COLOR_ORANGE,
    FigureResult,
    add_output_args,
    add_panel_label,
    add_source_note,
    equalize_xy,
    finalize_axes,
    load_waypoint_dataset,
    save_figure_all,
)

import matplotlib.pyplot as plt

SCRIPT_NAME = "scripts/analysis/generate_path_figures.py"


def build_parser() -> argparse.ArgumentParser:
    """CLI 인자 파서 구성(--waypoints, 출력 디렉터리). / Build the CLI argument parser (waypoints + output dirs)."""
    parser = argparse.ArgumentParser(
        description="Generate report-quality A/B region and lawnmower path figures."
    )
    parser.add_argument(
        "--waypoints",
        nargs="*",
        type=Path,
        default=None,
        help="Optional waypoint CSV/JSON files. Defaults to data/mock_runs/**/waypoints.*.",
    )
    add_output_args(parser)
    return parser


def _xy_points(waypoints: Sequence[dict[str, float | int]]) -> tuple[list[float], list[float]]:
    """웨이포인트 목록에서 (xs, ys) 두 리스트를 뽑아낸다. / Split a waypoint list into parallel x and y lists."""
    return [float(point["x"]) for point in waypoints], [float(point["y"]) for point in waypoints]


def _lane_endpoints(
    waypoints: Sequence[dict[str, float | int]],
) -> dict[int, list[tuple[float, float]]]:
    """레인 인덱스별로 (x,y) 끝점을 모은 dict 반환. / Group (x,y) endpoints by lane index.

    구역 사각형을 그릴 때 첫/마지막 레인의 양 끝을 코너로 쓴다. / Used to pick region corners.
    """
    lanes: dict[int, list[tuple[float, float]]] = {}
    for waypoint in waypoints:
        lanes.setdefault(int(waypoint["lane"]), []).append(
            (float(waypoint["x"]), float(waypoint["y"]))
        )
    return lanes


def _plot_region(ax, waypoints: Sequence[dict[str, float | int]]) -> None:
    """작업 구역 사각형(음영+테두리)을 축에 그린다. / Draw the shaded work-region rectangle onto the axes.

    첫 레인과 마지막 레인의 끝점을 코너로 삼아 평면 사각형을 채운다(부수효과: ax에 직접 그림).
    Uses first/last lane endpoints as corners; draws in place onto ``ax``.
    """
    lanes = _lane_endpoints(waypoints)
    first_lane = lanes[min(lanes)]
    last_lane = lanes[max(lanes)]
    corners = [first_lane[0], first_lane[1], last_lane[0], last_lane[1], first_lane[0]]
    xs = [point[0] for point in corners]
    ys = [point[1] for point in corners]
    ax.fill(xs, ys, color=COLOR_LIGHT, alpha=0.95, label="Planar work region")
    ax.plot(xs, ys, color=COLOR_NAVY, linewidth=1.4)


def _generate_ab_region(dataset, output_dirs: Sequence[str | Path] | None) -> FigureResult:
    """A/B 구역 정의 그림 생성·저장. / Render and save the A/B region-definition figure.

    운용자가 고른 A(시작)·B(끝) 두 점과 그로부터 정의되는 평면 사각형을 시각화한다.
    반환: 캡션·용도를 담은 FigureResult. 부수효과: PNG 저장. / Returns a FigureResult; saves a PNG.
    """
    waypoints = dataset.waypoints
    xs, ys = _xy_points(waypoints)
    fig, ax = plt.subplots(figsize=(7.2, 5.4))
    _plot_region(ax, waypoints)

    ax.scatter([xs[0], xs[1]], [ys[0], ys[1]], s=70, color=[COLOR_GREEN, COLOR_ORANGE], zorder=4)
    ax.annotate("A", (xs[0], ys[0]), xytext=(-15, -16), textcoords="offset points", weight="bold")
    ax.annotate("B", (xs[1], ys[1]), xytext=(9, 8), textcoords="offset points", weight="bold")
    ax.annotate(
        "Operator-selected A/B edge",
        xy=((xs[0] + xs[1]) / 2.0, (ys[0] + ys[1]) / 2.0),
        xytext=(10, -26),
        textcoords="offset points",
        arrowprops={"arrowstyle": "->", "color": COLOR_GRAY, "lw": 1.0},
        color=COLOR_GRAY,
        fontsize=9,
    )
    ax.plot([xs[0], xs[1]], [ys[0], ys[1]], color=COLOR_ORANGE, linewidth=2.0)
    ax.set_title("A/B Region Definition")
    ax.set_xlabel("Local East (m)")
    ax.set_ylabel("Local North (m)")
    add_panel_label(ax, "MOCK MISSION", color=COLOR_GRAY)
    add_source_note(ax, dataset.source_label, mock=dataset.is_mock)
    equalize_xy(ax, xs, ys)
    finalize_axes(ax)
    fig.tight_layout()
    save_figure_all(fig, "fig_ab_region_definition.png", output_dirs)
    return FigureResult(
        filename="fig_ab_region_definition.png",
        script=SCRIPT_NAME,
        data_source=dataset.data_source,
        recommended_use="Planning method section: explain manual A/B point selection and planar rectangular region definition.",
        caption=(
            "Mock planar work-region definition from operator-selected A/B points. The shaded "
            "rectangle and offsets illustrate the current planning assumption, not a validated hull survey."
        ),
    )


def _generate_lawnmower_preview(dataset, output_dirs: Sequence[str | Path] | None) -> FigureResult:
    """잔디깎기(사행) 경로 미리보기 그림 생성·저장. / Render and save the lawnmower-path preview figure.

    구역 위에 방문 순서대로 웨이포인트를 잇고 각 구간에 진행 방향 화살표를 얹는다.
    반환: FigureResult. 부수효과: PNG 저장. / Returns a FigureResult; saves a PNG.
    """
    waypoints = dataset.waypoints
    xs, ys = _xy_points(waypoints)
    fig, ax = plt.subplots(figsize=(7.2, 5.4))
    _plot_region(ax, waypoints)
    ax.plot(xs, ys, marker="o", color=COLOR_BLUE, linewidth=2.0, markersize=5)
    for left, right in zip(waypoints, waypoints[1:]):
        ax.annotate(
            "",
            xy=(float(right["x"]), float(right["y"])),
            xytext=(float(left["x"]), float(left["y"])),
            arrowprops={"arrowstyle": "->", "color": COLOR_BLUE, "lw": 1.1, "shrinkA": 6, "shrinkB": 6},
        )
    ax.set_title("Lawnmower Path Preview")
    ax.set_xlabel("Local East (m)")
    ax.set_ylabel("Local North (m)")
    add_panel_label(ax, "MOCK MISSION", color=COLOR_GRAY)
    add_source_note(ax, dataset.source_label, mock=dataset.is_mock)
    equalize_xy(ax, xs, ys)
    finalize_axes(ax)
    fig.tight_layout()
    save_figure_all(fig, "fig_lawnmower_path_preview.png", output_dirs)
    return FigureResult(
        filename="fig_lawnmower_path_preview.png",
        script=SCRIPT_NAME,
        data_source=dataset.data_source,
        recommended_use="Planning/results section: show the generated coverage pattern before hardware execution.",
        caption=(
            "Mock lawnmower coverage preview generated by the ROS-independent Python planner. "
            "The sequence shows the intended coverage geometry only; it is not an autonomous field result."
        ),
    )


def _generate_waypoint_sequence(dataset, output_dirs: Sequence[str | Path] | None) -> FigureResult:
    """웨이포인트 방문 순서 그림 생성·저장. / Render and save the waypoint-sequence figure.

    order 값으로 점을 색칠(컬러바)하고 각 점에 순번을 라벨링해 교대 레인 진행을 보여준다.
    반환: FigureResult. 부수효과: PNG 저장. / Returns a FigureResult; saves a PNG.
    """
    waypoints = dataset.waypoints
    xs, ys = _xy_points(waypoints)
    orders = [int(point["order"]) for point in waypoints]
    fig, ax = plt.subplots(figsize=(7.2, 5.4))
    _plot_region(ax, waypoints)
    scatter = ax.scatter(xs, ys, c=orders, cmap="viridis", s=80, edgecolor="white", linewidth=0.8, zorder=4)
    ax.plot(xs, ys, color=COLOR_NAVY, alpha=0.55, linewidth=1.2)
    for waypoint in waypoints:
        ax.annotate(
            str(int(waypoint["order"])),
            (float(waypoint["x"]), float(waypoint["y"])),
            xytext=(6, 6),
            textcoords="offset points",
            fontsize=8,
            color=COLOR_NAVY,
        )
    colorbar = fig.colorbar(scatter, ax=ax, fraction=0.045, pad=0.03)
    colorbar.set_label("Waypoint order")
    ax.set_title("Waypoint Sequence")
    ax.set_xlabel("Local East (m)")
    ax.set_ylabel("Local North (m)")
    add_panel_label(ax, "MOCK MISSION", color=COLOR_GRAY)
    add_source_note(ax, dataset.source_label, mock=dataset.is_mock)
    equalize_xy(ax, xs, ys)
    finalize_axes(ax)
    fig.tight_layout()
    save_figure_all(fig, "fig_waypoint_sequence.png", output_dirs)
    return FigureResult(
        filename="fig_waypoint_sequence.png",
        script=SCRIPT_NAME,
        data_source=dataset.data_source,
        recommended_use="Planner implementation section: document waypoint ordering and alternating lane traversal.",
        caption=(
            "Mock waypoint order for the generated lawnmower path. Labels identify the planner's "
            "sequential output and alternating lane direction."
        ),
    )


def generate(
    *,
    output_dirs: Sequence[str | Path] | None = None,
    waypoint_paths: Sequence[Path] | None = None,
) -> list[FigureResult]:
    """웨이포인트 데이터셋을 로드하고 3종 그림을 모두 생성. / Load the waypoint dataset and produce all three figures.

    상위 오케스트레이터가 호출하는 공개 진입점. 반환: FigureResult 리스트.
    Public entry point used by the figure orchestrator; returns the list of FigureResults.
    """
    dataset = load_waypoint_dataset(waypoint_paths)
    return [
        _generate_ab_region(dataset, output_dirs),
        _generate_lawnmower_preview(dataset, output_dirs),
        _generate_waypoint_sequence(dataset, output_dirs),
    ]


def main() -> int:
    """CLI 진입점: 인자 파싱→그림 생성→요약 출력. / CLI entry point: parse args, generate figures, print a summary."""
    args = build_parser().parse_args()
    results = generate(output_dirs=args.output_dirs, waypoint_paths=args.waypoints)
    for result in results:
        print(f"generated {result.filename} from {result.data_source}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
