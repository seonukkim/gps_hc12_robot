"""GPS 검증 리포트 그림 생성 / Report-quality GPS validation figures.

목적/역할:
    실제 GPS 로그(또는 데모 데이터)로부터 리포트용 그림 4종을 만든다 — (1) fix 획득 타임라인,
    (2) 위성 수 vs 시간, (3) HDOP vs 시간, (4) 로컬 좌표 위치 산포. 로그 "품질 요약"이 목적이며
    폐루프 경로 추종 정확도를 주장하지 않는다(캡션에 그 한계를 명시).

시스템 내 위치:
    - `scripts/analysis/generate_all_figures.py`가 generate()를 호출한다.
    - 데이터·플로팅 헬퍼는 `_figure_common.py`에서 import(GPS 로그 파싱은 tools/analyze_gps_log.py에 위임).
    - 로그가 없으면 mock_gps_dataset()이 결정론적 데모 시계열을 공급한다.

핵심 개념·불변식:
    - 여러 로그 중 가장 "정보량 많은" 것을 _choose_dataset()이 점수로 고른다(fix 전이 유무 →
      유효 점 수 → 전체 레코드 수 → 실로그 우선). 목/실로그 여부는 패널 라벨·캡션에 반영된다.
    - t_s는 로그에 타임스탬프가 없으면 샘플 순서로 재구성된 '명목' 시간이다(캡션에 경고).
    - 유효 지표만 그린다: fix 없는 레코드·None 값·비현실적 HDOP(>=20)는 _valid_metric_records가 배제.

사용법/진입점:
    CLI: ``python scripts/analysis/generate_gps_figures.py [--gps-log ...] [--output-dirs ...]``.
    프로그램적으로는 generate(output_dirs=..., gps_paths=...).

리팩토링 노트:
    GPSRecord 필드명(fix_valid/satellites/hdop/t_s 등)과 _figure_common 헬퍼에 결합. HDOP 컷오프 20,
    점수 튜플 순서를 바꾸면 어떤 로그가 선택되는지가 달라진다 — 회귀에 주의.

Purpose: build four report figures from a real GPS log (or demo data) — fix timeline, satellites vs
time, HDOP vs time, and a local-frame position scatter. These summarize log quality only; they do not
claim closed-loop tracking accuracy. System: called by ``generate_all_figures.py``; parsing/plotting
helpers come from ``_figure_common`` (which delegates NMEA/USBDBG parsing to tools/analyze_gps_log.py);
falls back to ``mock_gps_dataset`` when no log exists. ``_choose_dataset`` ranks candidate logs by
information content (fix transition, valid points, record count, real-over-mock). ``t_s`` may be a
nominal, sample-order time when the log has no timestamps. Coupled to GPSRecord field names and the
HDOP<20 cutoff.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Sequence

from _figure_common import (
    COLOR_BLUE,
    COLOR_GRAY,
    COLOR_GREEN,
    COLOR_NAVY,
    COLOR_ORANGE,
    FigureResult,
    add_output_args,
    add_panel_label,
    add_source_note,
    equalize_xy,
    finalize_axes,
    fixed_xy_records,
    gps_local_xy,
    load_gps_datasets,
    mock_gps_dataset,
    save_figure_all,
)

import matplotlib.pyplot as plt

SCRIPT_NAME = "scripts/analysis/generate_gps_figures.py"


def build_parser() -> argparse.ArgumentParser:
    """CLI 인자 파서 구성(--gps-log, 출력 디렉터리). / Build the CLI argument parser (gps log + output dirs)."""
    parser = argparse.ArgumentParser(description="Generate report-quality GPS validation figures.")
    parser.add_argument(
        "--gps-log",
        nargs="*",
        type=Path,
        default=None,
        help="Optional GPS CSV/log files. Defaults to data/nmea_logs/*.csv and data/gps_logs/*.log.",
    )
    add_output_args(parser)
    return parser


def _choose_dataset(paths: Sequence[Path] | None) -> object:
    """가장 정보량 많은 GPS 로그 하나를 선택(없으면 목 데이터). / Pick the most informative GPS log (or mock data if none).

    점수 우선순위: fix 상태 전이 유무 → 유효 좌표 점 수 → 전체 레코드 수 → 실로그(비목) 우선.
    이렇게 하면 '항상 fix' 또는 '항상 no-fix'인 단조 로그보다 전이가 담긴 로그를 선호한다.
    Ranks by: has a fix transition, then valid-point count, record count, and prefers real over mock.
    """
    datasets = load_gps_datasets(paths)
    if not datasets:
        return mock_gps_dataset()

    def score(dataset) -> tuple[int, int, int, int]:
        records = dataset.records
        fix_values = {record.fix_valid for record in records}
        valid_points = len(fixed_xy_records(records))
        has_fix_transition = int(len(fix_values) > 1)
        return (has_fix_transition, valid_points, len(records), -int(dataset.is_mock))

    return max(datasets, key=score)


def _valid_metric_records(dataset, metric: str):
    """지정 지표가 유효한 레코드만 필터링. / Keep only records whose ``metric`` is valid for plotting.

    fix가 없거나 값이 None인 레코드는 제외. HDOP는 20 이상이면 수신기 placeholder로 보고 버린다.
    Excludes no-fix and None records; HDOP>=20 is treated as a receiver placeholder and dropped.
    """
    valid = []
    for record in dataset.records:
        value = getattr(record, metric)
        if not record.fix_valid or value is None:
            continue
        # HDOP 20+는 실제 정밀도가 아니라 '값 없음' 표식인 경우가 많아 축 스케일을 망친다 → 배제.
        # / HDOP >= 20 is usually a "no value" sentinel, not real precision; it would wreck the y-scale.
        if metric == "hdop" and value >= 20:
            continue
        valid.append(record)
    return valid


def _generate_fix_timeline(dataset, output_dirs: Sequence[str | Path] | None) -> FigureResult:
    """GPS fix 획득 타임라인 그림 생성·저장. / Render and save the GPS fix-status timeline figure.

    시간에 따른 fix 유무(0/1)를 계단 그래프로 표시. 반환: FigureResult. 부수효과: PNG 저장.
    Step plot of fix present/absent over time. Returns a FigureResult; saves a PNG.
    """
    t_s = [record.t_s for record in dataset.records]
    fix = [1 if record.fix_valid else 0 for record in dataset.records]
    fig, ax = plt.subplots(figsize=(8, 3.8))
    ax.step(t_s, fix, where="post", color=COLOR_GREEN, linewidth=2.0)
    ax.fill_between(t_s, fix, step="post", color=COLOR_GREEN, alpha=0.16)
    ax.set_ylim(-0.15, 1.15)
    ax.set_yticks([0, 1])
    ax.set_yticklabels(["No fix", "Fix"])
    ax.set_title("GPS Fix Timeline")
    ax.set_xlabel("Elapsed sample time (s)")
    ax.set_ylabel("Fix status")
    add_panel_label(ax, "MOCK/DEMO" if dataset.is_mock else "REAL LOG", color=COLOR_GRAY)
    add_source_note(ax, dataset.source_label, mock=dataset.is_mock)
    finalize_axes(ax)
    fig.tight_layout()
    save_figure_all(fig, "fig_gps_fix_timeline.png", output_dirs)
    return FigureResult(
        filename="fig_gps_fix_timeline.png",
        script=SCRIPT_NAME,
        data_source=dataset.data_source,
        recommended_use="GPS validation section: show whether the selected log reported fix availability over the capture.",
        caption=(
            "GPS fix status over the selected log. For USB debug captures without per-line timestamps, "
            "elapsed time is reconstructed from sample order and should be treated as nominal."
        ),
    )


def _generate_satellites(dataset, output_dirs: Sequence[str | Path] | None) -> FigureResult:
    """위성 수 vs 시간 그림 생성·저장. / Render and save the satellites-vs-time figure.

    유효 fix 레코드의 위성 개수만 그린다. 반환: FigureResult. 부수효과: PNG 저장.
    Plots satellite count for valid-fix records. Returns a FigureResult; saves a PNG.
    """
    records = _valid_metric_records(dataset, "satellites")
    fig, ax = plt.subplots(figsize=(8, 3.8))
    ax.plot(
        [record.t_s for record in records],
        [record.satellites for record in records],
        marker="o",
        markersize=3,
        color=COLOR_BLUE,
    )
    ax.set_title("GPS Satellites vs Time")
    ax.set_xlabel("Elapsed sample time (s)")
    ax.set_ylabel("Satellites")
    ax.set_ylim(bottom=0)
    add_panel_label(ax, "MOCK/DEMO" if dataset.is_mock else "REAL LOG", color=COLOR_GRAY)
    add_source_note(ax, dataset.source_label, mock=dataset.is_mock)
    finalize_axes(ax)
    fig.tight_layout()
    save_figure_all(fig, "fig_gps_satellites_vs_time.png", output_dirs)
    return FigureResult(
        filename="fig_gps_satellites_vs_time.png",
        script=SCRIPT_NAME,
        data_source=dataset.data_source,
        recommended_use="GPS validation section: summarize satellite count stability during valid-fix records.",
        caption=(
            "Satellite count for records with valid GPS fix in the selected log. No-fix placeholder "
            "values are omitted from the plotted series."
        ),
    )


def _generate_hdop(dataset, output_dirs: Sequence[str | Path] | None) -> FigureResult:
    """HDOP vs 시간 그림 생성·저장. / Render and save the HDOP-vs-time figure.

    유효 fix의 HDOP 추이와 참고선(2.5)을 그린다. 항법 정확도 주장이 아니라 로그 품질 요약.
    반환: FigureResult. 부수효과: PNG 저장. / Log-quality summary, not an accuracy claim.
    """
    records = _valid_metric_records(dataset, "hdop")
    fig, ax = plt.subplots(figsize=(8, 3.8))
    ax.plot(
        [record.t_s for record in records],
        [record.hdop for record in records],
        marker="o",
        markersize=3,
        color=COLOR_ORANGE,
    )
    ax.axhline(2.5, color=COLOR_GRAY, linewidth=1.0, linestyle=":", label="Reference: HDOP 2.5")
    ax.set_title("GPS HDOP vs Time")
    ax.set_xlabel("Elapsed sample time (s)")
    ax.set_ylabel("HDOP")
    ax.set_ylim(bottom=0)
    ax.legend(loc="upper right")
    add_panel_label(ax, "MOCK/DEMO" if dataset.is_mock else "REAL LOG", color=COLOR_GRAY)
    add_source_note(ax, dataset.source_label, mock=dataset.is_mock)
    finalize_axes(ax)
    fig.tight_layout()
    save_figure_all(fig, "fig_gps_hdop_vs_time.png", output_dirs)
    return FigureResult(
        filename="fig_gps_hdop_vs_time.png",
        script=SCRIPT_NAME,
        data_source=dataset.data_source,
        recommended_use="GPS validation section: report HDOP trend for valid-fix samples without claiming navigation accuracy.",
        caption=(
            "HDOP values for records with valid GPS fix in the selected log. The plot is a log "
            "quality summary and does not by itself validate closed-loop path tracking."
        ),
    )


def _generate_position_scatter(dataset, output_dirs: Sequence[str | Path] | None) -> FigureResult:
    """로컬 좌표 위치 산포 그림 생성·저장. / Render and save the local-frame position scatter figure.

    유효 fix 좌표를 첫 fix 기준 로컬 미터로 변환해 산포도로 그리고 시간을 색으로 인코딩한다.
    관측된 산포 요약일 뿐 측위 성능 검증은 아니다. 반환: FigureResult. 부수효과: PNG 저장.
    Fixed positions relative to the first fix, colored by time. Returns a FigureResult; saves a PNG.
    """
    fixed = fixed_xy_records(dataset.records)
    xs, ys = gps_local_xy(dataset.records)
    times = [record.t_s for record in fixed]
    fig, ax = plt.subplots(figsize=(6.6, 5.4))
    scatter = ax.scatter(xs, ys, c=times, cmap="plasma", s=36, edgecolor="white", linewidth=0.4)
    if xs and ys:
        ax.scatter([xs[0]], [ys[0]], marker="s", s=70, color=COLOR_GREEN, edgecolor="white", label="First fix")
        ax.scatter([xs[-1]], [ys[-1]], marker="^", s=80, color=COLOR_NAVY, edgecolor="white", label="Last fix")
    colorbar = fig.colorbar(scatter, ax=ax, fraction=0.045, pad=0.03)
    colorbar.set_label("Elapsed sample time (s)")
    ax.set_title("GPS Position Scatter")
    ax.set_xlabel("Local East from first fix (m)")
    ax.set_ylabel("Local North from first fix (m)")
    ax.legend(loc="best")
    add_panel_label(ax, "MOCK/DEMO" if dataset.is_mock else "REAL LOG", color=COLOR_GRAY)
    add_source_note(ax, dataset.source_label, mock=dataset.is_mock)
    equalize_xy(ax, xs, ys)
    finalize_axes(ax)
    fig.tight_layout()
    save_figure_all(fig, "fig_gps_position_scatter.png", output_dirs)
    return FigureResult(
        filename="fig_gps_position_scatter.png",
        script=SCRIPT_NAME,
        data_source=dataset.data_source,
        recommended_use="GPS validation section: visualize fixed-position spread in local meters.",
        caption=(
            "Local-position scatter for valid GPS fixes, converted relative to the first fixed point. "
            "This summarizes observed spread in the selected log, not hull-surface localization performance."
        ),
    )


def generate(
    *,
    output_dirs: Sequence[str | Path] | None = None,
    gps_paths: Sequence[Path] | None = None,
) -> list[FigureResult]:
    """GPS 로그를 선택하고 4종 그림을 모두 생성. / Choose a GPS log and produce all four figures.

    상위 오케스트레이터의 공개 진입점. 반환: FigureResult 리스트.
    Public entry point for the figure orchestrator; returns the list of FigureResults.
    """
    dataset = _choose_dataset(gps_paths)
    return [
        _generate_fix_timeline(dataset, output_dirs),
        _generate_satellites(dataset, output_dirs),
        _generate_hdop(dataset, output_dirs),
        _generate_position_scatter(dataset, output_dirs),
    ]


def main() -> int:
    """CLI 진입점: 인자 파싱→그림 생성→요약 출력. / CLI entry point: parse args, generate figures, print a summary."""
    args = build_parser().parse_args()
    results = generate(output_dirs=args.output_dirs, gps_paths=args.gps_log)
    for result in results:
        print(f"generated {result.filename} from {result.data_source}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
