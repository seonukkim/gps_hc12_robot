"""수동 제어·페일세이프 리포트 그림 생성 / Manual-control and failsafe report figures.

목적/역할:
    USBDBG 안전 로그(또는 데모 데이터)로부터 리포트용 그림 3종을 만든다 — (1) 수동 제어 타임라인
    (RC 입력 vs 좌/우 모터 명령), (2) 페일세이프 이벤트 타임라인(rc_ok=false / FAILSAFE / STOP /
    모터 정지 구간), (3) 제어 소스 전이(STOP↔RC_MANUAL 등). 로그에 기록된 상태만 보여줄 뿐
    완결된 자율 안전 검증을 주장하지 않는다(캡션·패널 라벨에 그 한계를 명시).

시스템 내 위치:
    - `scripts/analysis/generate_all_figures.py`가 generate()를 호출.
    - 데이터·플로팅 헬퍼는 `_figure_common.py`(안전 로그 파싱은 tools/analyze_safety_log.py에 위임).
    - 로그가 없으면 mock_safety_dataset()이 FAILSAFE→MANUAL→AUTO_READY→MANUAL 시나리오를 공급.

핵심 개념·불변식:
    - 그림마다 요구 특성이 달라 **데이터셋을 따로 고른다**: 수동은 비영(非零) 모터 명령이 많은 로그,
      페일세이프는 FAILSAFE/rc_bad가 있는 로그(없으면 목으로 대체), 전이는 제어 소스 종류가 많은 로그.
    - 모터 '정지' 판정 임계값은 |cmd|<=0.001 (부동소수 0 근사). 여러 곳에서 동일하게 쓰인다.
    - 목/실로그 여부는 패널 라벨·캡션에 반영된다.

사용법/진입점:
    CLI: ``python scripts/analysis/generate_manual_control_figures.py [--safety-log ...] [--output-dirs ...]``.
    프로그램적으로는 generate(output_dirs=..., safety_paths=...).

리팩토링 노트:
    SafetyRecord 필드명(mode/rc_ok/control_source/left_cmd/right_cmd 등)과 문자열 상수
    ("FAILSAFE","STOP","RC_MANUAL")에 결합. 점수 함수 튜플 순서를 바꾸면 선택 로그가 달라진다.

Purpose: build three report figures from a USBDBG safety log (or demo data) — a manual-control
timeline (RC input vs motor commands), a failsafe event timeline (rc_ok=false / FAILSAFE / STOP /
zero-motor spans), and a control-source transition plot. They document logged state only, not a
completed autonomy safety validation. System: called by ``generate_all_figures.py``; parsing/plotting
helpers come from ``_figure_common`` (delegating to tools/analyze_safety_log.py); falls back to
``mock_safety_dataset``. Each figure chooses its own best-fitting dataset via a scoring function. The
motor-stopped threshold is |cmd|<=0.001. Coupled to SafetyRecord field names and the mode/source
string constants.
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
    COLOR_RED,
    FigureResult,
    add_output_args,
    add_panel_label,
    add_source_note,
    finalize_axes,
    load_safety_datasets,
    mock_safety_dataset,
    save_figure_all,
)

import matplotlib.pyplot as plt

SCRIPT_NAME = "scripts/analysis/generate_manual_control_figures.py"


def build_parser() -> argparse.ArgumentParser:
    """CLI 인자 파서 구성(--safety-log, 출력 디렉터리). / Build the CLI argument parser (safety log + output dirs)."""
    parser = argparse.ArgumentParser(
        description="Generate report-quality manual-control and failsafe figures."
    )
    parser.add_argument(
        "--safety-log",
        nargs="*",
        type=Path,
        default=None,
        help=(
            "Optional USBDBG safety log files. Defaults to data/safety_logs/*.log "
            "and data/gps_logs/*.log."
        ),
    )
    add_output_args(parser)
    return parser


# ── 데이터셋 선택 / Dataset selection ──
def _datasets_or_mock(paths: Sequence[Path] | None):
    """로그를 로드하되 비면 목 데이터 한 개를 반환. / Load safety logs, or return a single mock dataset if none."""
    datasets = load_safety_datasets(paths)
    return datasets if datasets else [mock_safety_dataset()]


def _choose_manual_dataset(paths: Sequence[Path] | None):
    """수동 제어 타임라인에 가장 알맞은 로그 선택. / Pick the log best suited to the manual-control timeline.

    비영(非零) 모터 명령이 많고 총 명령량이 큰(=실제 수동 조작이 담긴) 로그를 선호하며,
    safety_logs 경로와 RC_MANUAL 레코드 수로 동점을 가린다.
    Prefers logs with the most nonzero motor commands / largest total command; safety-log path and
    RC_MANUAL count break ties.
    """
    datasets = _datasets_or_mock(paths)

    def score(dataset) -> tuple[int, int, float, int, int]:
        nonzero = sum(
            1
            for record in dataset.records
            if abs(record.left_cmd) > 0.001 or abs(record.right_cmd) > 0.001
        )
        rc_manual = sum(1 for record in dataset.records if record.control_source == "RC_MANUAL")
        total_command = sum(abs(record.left_cmd) + abs(record.right_cmd) for record in dataset.records)
        safety_log = int("data/safety_logs/" in dataset.source_label)
        return (int(nonzero > 0), nonzero, total_command, safety_log, rc_manual)

    return max(datasets, key=score)


def _choose_failsafe_dataset(paths: Sequence[Path] | None):
    """페일세이프 이벤트가 담긴 로그 선택(없으면 목). / Pick a log that actually contains failsafe events (else mock).

    FAILSAFE/rc_bad 레코드 수로 고르고, 선택된 로그에 그런 이벤트가 전혀 없으면 목 데이터로 대체한다
    (빈 페일세이프 그림을 피하려는 의도). / Falls back to mock when the winner has no failsafe/rc-bad
    records, avoiding an empty failsafe figure.
    """
    datasets = _datasets_or_mock(paths)

    def score(dataset) -> tuple[int, int, int]:
        failsafe = sum(1 for record in dataset.records if record.mode == "FAILSAFE")
        rc_bad = sum(1 for record in dataset.records if not record.rc_ok)
        stop = sum(1 for record in dataset.records if record.control_source == "STOP")
        return (failsafe + rc_bad, stop, len(dataset.records))

    selected = max(datasets, key=score)
    # 우승 로그에 페일세이프/RC 불량이 하나도 없으면 그림이 빈다 → 데모 데이터로 대체.
    # / If the winner shows no failsafe/RC-bad at all, the figure would be blank; use the demo instead.
    if not any(record.mode == "FAILSAFE" or not record.rc_ok for record in selected.records):
        return mock_safety_dataset()
    return selected


def _choose_transition_dataset(paths: Sequence[Path] | None):
    """제어 소스 전이가 가장 다양한 로그 선택. / Pick the log with the most control-source variety.

    서로 다른 제어 소스 종류 수를 최우선으로, safety_logs 경로·모드 종류·레코드 수로 동점 처리.
    Ranks by count of distinct control sources; safety-log path, mode variety and length break ties.
    """
    datasets = _datasets_or_mock(paths)

    def score(dataset) -> tuple[int, int, int, int]:
        unique_sources = len({record.control_source for record in dataset.records})
        unique_modes = len({record.mode for record in dataset.records})
        safety_log = int("data/safety_logs/" in dataset.source_label)
        return (unique_sources, safety_log, unique_modes, len(dataset.records))

    return max(datasets, key=score)


# ── 헬퍼: 불리언 구간 추출 / Helper: boolean-run extraction ──
def _boolean_segments(times: Sequence[float], flags: Sequence[bool]) -> list[tuple[float, float]]:
    """연속으로 True인 구간을 (시작, 폭) 목록으로 변환. / Convert runs of True into (start, width) spans.

    matplotlib broken_barh 입력용. flags가 True인 연속 구간마다 시작 시각과 폭(다음 샘플까지의
    시간)을 계산한다. 마지막 샘플까지 True이면 default_step 만큼 폭을 준다(마지막 막대가 사라지지
    않게). 폭은 최소 default_step으로 하한 처리(0폭 막대 방지).
    For broken_barh: each maximal True run yields its start time and a width up to the next sample.
    A run reaching the last sample gets one ``default_step`` of width; widths are floored to
    ``default_step`` so no zero-width bar disappears.
    """
    segments: list[tuple[float, float]] = []
    if not times:
        return segments
    # 샘플 간격 추정치. 등간격 로그를 가정하며 마지막 구간의 폭 하한으로도 쓰인다.
    # / Estimated sample step; assumes uniform sampling and also floors the final span's width.
    default_step = times[1] - times[0] if len(times) > 1 else 1.0
    start: float | None = None
    for index, flag in enumerate(flags):
        if flag and start is None:
            start = times[index]
        # 구간 종료 조건: flag가 False로 떨어지거나, 마지막 샘플에 도달했는데 아직 열려 있을 때.
        # / Close the run when the flag drops, or when we hit the last sample with a run still open.
        if start is not None and (not flag or index == len(flags) - 1):
            end_index = index if not flag else index + 1
            if end_index < len(times):
                end = times[end_index]
            else:
                end = times[-1] + default_step
            segments.append((start, max(end - start, default_step)))
            start = None
    return segments


# ── 그림 생성기 / Figure generators ──
def _generate_manual_timeline(dataset, output_dirs: Sequence[str | Path] | None) -> FigureResult:
    """수동 제어 타임라인 그림 생성·저장. / Render and save the manual-control timeline figure.

    RC 스로틀·조향 입력과 그 결과인 좌/우 모터 정규화 명령을 한 축에 겹쳐 그린다.
    반환: FigureResult. 부수효과: PNG 저장. / RC input vs resulting motor commands; saves a PNG.
    """
    records = dataset.records
    t_s = [record.t_s for record in records]
    fig, ax = plt.subplots(figsize=(8.4, 4.2))
    ax.plot(t_s, [record.throttle_norm for record in records], color=COLOR_BLUE, label="RC throttle")
    ax.plot(t_s, [record.steer_norm for record in records], color=COLOR_ORANGE, label="RC steer")
    ax.plot(t_s, [record.left_cmd for record in records], color=COLOR_GREEN, label="Left motor cmd")
    ax.plot(t_s, [record.right_cmd for record in records], color=COLOR_NAVY, label="Right motor cmd")
    ax.axhline(0.0, color=COLOR_GRAY, linewidth=0.8)
    ax.set_ylim(-1.08, 1.08)
    ax.set_title("Manual Control Timeline")
    ax.set_xlabel("Elapsed sample time (s)")
    ax.set_ylabel("Normalized command")
    ax.legend(loc="upper right", ncols=2)
    add_panel_label(ax, "MOCK/DEMO" if dataset.is_mock else "REAL LOG", color=COLOR_GRAY)
    add_source_note(ax, dataset.source_label, mock=dataset.is_mock)
    finalize_axes(ax)
    fig.tight_layout()
    save_figure_all(fig, "fig_manual_control_timeline.png", output_dirs)
    return FigureResult(
        filename="fig_manual_control_timeline.png",
        script=SCRIPT_NAME,
        data_source=dataset.data_source,
        recommended_use="Safety validation section: show RC/manual input and resulting normalized motor commands.",
        caption=(
            "Manual-control timeline from the selected USB debug log. The figure summarizes logged "
            "commands only and should be discussed with the wheel-off-ground bench-test constraint."
        ),
    )


def _generate_failsafe_timeline(dataset, output_dirs: Sequence[str | Path] | None) -> FigureResult:
    """페일세이프 이벤트 타임라인 그림 생성·저장. / Render and save the failsafe-event timeline figure.

    네 가지 조건(rc_ok=false / mode=FAILSAFE / control_source=STOP / 모터 정지)을 각각 한 레인의
    구간 막대로 그린다. 반환: FigureResult. 부수효과: PNG 저장. / One lane of bars per condition.
    """
    records = dataset.records
    t_s = [record.t_s for record in records]
    # 각 튜플=(레인 라벨, 샘플별 불리언 플래그, 막대 색). broken_barh용으로 아래에서 순회한다.
    # / Each tuple is (lane label, per-sample boolean flags, bar color); iterated below for broken_barh.
    lanes = [
        ("rc_ok=false", [not record.rc_ok for record in records], COLOR_RED),
        ("mode=FAILSAFE", [record.mode == "FAILSAFE" for record in records], COLOR_ORANGE),
        ("control_source=STOP", [record.control_source == "STOP" for record in records], COLOR_BLUE),
        (
            "motor_cmd_zero",
            [abs(record.left_cmd) <= 0.001 and abs(record.right_cmd) <= 0.001 for record in records],
            COLOR_GREEN,
        ),
    ]
    fig, ax = plt.subplots(figsize=(8.4, 4.1))
    for y_index, (_, flags, color) in enumerate(lanes):
        ax.broken_barh(
            _boolean_segments(t_s, flags),
            (y_index - 0.34, 0.68),
            facecolors=color,
            edgecolors="none",
            alpha=0.86,
        )
    ax.set_yticks(range(len(lanes)))
    ax.set_yticklabels([name for name, _, _ in lanes])
    ax.set_ylim(-0.65, len(lanes) - 0.35)
    ax.set_title("Failsafe Event Timeline")
    ax.set_xlabel("Elapsed sample time (s)")
    ax.set_ylabel("Logged condition")
    add_panel_label(ax, "MOCK/DEMO" if dataset.is_mock else "REAL LOG", color=COLOR_GRAY)
    add_source_note(ax, dataset.source_label, mock=dataset.is_mock)
    finalize_axes(ax)
    fig.tight_layout()
    save_figure_all(fig, "fig_failsafe_event_timeline.png", output_dirs)
    return FigureResult(
        filename="fig_failsafe_event_timeline.png",
        script=SCRIPT_NAME,
        data_source=dataset.data_source,
        recommended_use="Failsafe validation section: show when invalid RC/failsafe/STOP conditions appear in the log.",
        caption=(
            "Failsafe-related conditions from the selected USB debug log. The bars show logged "
            "state conditions and zero-command intervals; they do not claim full autonomous safety validation."
        ),
    )


def _generate_control_source_transition(dataset, output_dirs: Sequence[str | Path] | None) -> FigureResult:
    """제어 소스 전이 그림 생성·저장. / Render and save the control-source transition figure.

    STOP·RC_MANUAL 등 제어 소스를 등장 순서대로 y축 카테고리로 놓고 시간에 따른 전환을 계단으로 표시.
    반환: FigureResult. 부수효과: PNG 저장. / Step plot of control-source switches over time.
    """
    records = dataset.records
    t_s = [record.t_s for record in records]
    # 소스를 '처음 등장한 순서'대로 y축에 배치(알파벳순이 아님) → 시간 흐름과 라벨 순서를 맞춘다.
    # / Order sources by first appearance (not alphabetically) so the y-axis follows the timeline.
    ordered_sources: list[str] = []
    for record in records:
        if record.control_source not in ordered_sources:
            ordered_sources.append(record.control_source)
    source_index = {source: index for index, source in enumerate(ordered_sources)}
    y_values = [source_index[record.control_source] for record in records]

    fig, ax = plt.subplots(figsize=(8.4, 3.9))
    ax.step(t_s, y_values, where="post", color=COLOR_NAVY, linewidth=2.1)
    ax.scatter(t_s, y_values, color=COLOR_NAVY, s=12, alpha=0.65)
    ax.set_yticks(range(len(ordered_sources)))
    ax.set_yticklabels(ordered_sources)
    ax.set_title("Control Source Transition")
    ax.set_xlabel("Elapsed sample time (s)")
    ax.set_ylabel("Control source")
    add_panel_label(ax, "MOCK/DEMO" if dataset.is_mock else "REAL LOG", color=COLOR_GRAY)
    add_source_note(ax, dataset.source_label, mock=dataset.is_mock)
    finalize_axes(ax)
    fig.tight_layout()
    save_figure_all(fig, "fig_control_source_transition.png", output_dirs)
    return FigureResult(
        filename="fig_control_source_transition.png",
        script=SCRIPT_NAME,
        data_source=dataset.data_source,
        recommended_use="Control and safety section: document transitions among STOP, RC manual, and other logged sources.",
        caption=(
            "Logged control-source transitions over the selected USB debug capture. This figure "
            "documents source selection in the log and should not be described as completed ROS2 autonomy."
        ),
    )


def generate(
    *,
    output_dirs: Sequence[str | Path] | None = None,
    safety_paths: Sequence[Path] | None = None,
) -> list[FigureResult]:
    """그림별 최적 로그를 골라 3종 그림을 모두 생성. / Choose the per-figure best log and produce all three figures.

    상위 오케스트레이터의 공개 진입점. 세 그림은 서로 다른 데이터셋을 쓸 수 있다. 반환: FigureResult 리스트.
    Public entry point; the three figures may each use a different dataset. Returns the FigureResults.
    """
    manual_dataset = _choose_manual_dataset(safety_paths)
    failsafe_dataset = _choose_failsafe_dataset(safety_paths)
    transition_dataset = _choose_transition_dataset(safety_paths)
    return [
        _generate_manual_timeline(manual_dataset, output_dirs),
        _generate_failsafe_timeline(failsafe_dataset, output_dirs),
        _generate_control_source_transition(transition_dataset, output_dirs),
    ]


def main() -> int:
    """CLI 진입점: 인자 파싱→그림 생성→요약 출력. / CLI entry point: parse args, generate figures, print a summary."""
    args = build_parser().parse_args()
    results = generate(output_dirs=args.output_dirs, safety_paths=args.safety_log)
    for result in results:
        print(f"generated {result.filename} from {result.data_source}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
