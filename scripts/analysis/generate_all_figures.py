"""모든 보고서 그림 일괄 생성 오케스트레이터 / Orchestrator for all report figures.

목적/역할:
    시스템·경로·GPS·수동제어 네 갈래의 그림 생성기를 한 번에 실행하고, 산출 결과
    (FigureResult 목록)로 캡션 마크다운을 갱신하는 상위 진입점(CLI)이다.
    Top-level CLI that runs the system/path/GPS/manual-control figure generators and
    refreshes the caption markdown from the collected ``FigureResult``s.

시스템 내 위치 / Where this sits:
    - import 하는 대상 / imports: ``_figure_common`` (인자 헬퍼·캡션 작성) 및 각
      ``generate_*_figures`` 모듈의 ``generate()``.
    - 파이프라인 위치 / stage: 파이프라인의 최상위 실행기. 개별 스크립트를 직접 돌릴 수도
      있지만 이 파일이 "전부 생성 + 캡션 갱신"을 담당한다.

사용법/진입점 / Usage:
    ``python3 scripts/analysis/generate_all_figures.py [--waypoints ...] [--gps-log ...]
    [--safety-log ...] [--no-captions] [--output-dirs ...]``. main()이 진입점이며 종료코드를
    반환한다. Optional inputs override discovery; ``--no-captions`` skips the markdown rewrite.

리팩토링 노트 / Refactoring notes:
    새 그림 카테고리를 추가하려면 해당 ``generate_*`` 모듈을 import 하고 main()에서
    ``results.extend(...)`` 한 줄을 더하면 된다. 생성 순서가 콘솔 출력·캡션 순서를 정한다.
    To add a category: import its module and extend ``results`` in main(); order matters.
"""

from __future__ import annotations

import argparse
from pathlib import Path

from _figure_common import add_output_args, write_figure_captions
from generate_gps_figures import generate as generate_gps_figures
from generate_manual_control_figures import generate as generate_manual_control_figures
from generate_path_figures import generate as generate_path_figures
from generate_system_figures import generate as generate_system_figures


def build_parser() -> argparse.ArgumentParser:
    """CLI 인자 파서를 구성한다(웨이포인트·GPS·안전 로그 입력, 캡션 생략, 출력 경로).

    Build the argparse parser for the all-figures CLI.
    """
    parser = argparse.ArgumentParser(description="Generate all report-quality project figures.")
    parser.add_argument(
        "--waypoints",
        nargs="*",
        type=Path,
        default=None,
        help="Optional waypoint CSV/JSON files for path figures.",
    )
    parser.add_argument(
        "--gps-log",
        nargs="*",
        type=Path,
        default=None,
        help="Optional GPS CSV/log files for GPS figures.",
    )
    parser.add_argument(
        "--safety-log",
        nargs="*",
        type=Path,
        default=None,
        help="Optional USBDBG safety log files for manual/failsafe figures.",
    )
    parser.add_argument(
        "--no-captions",
        action="store_true",
        help="Generate figures without rewriting docs/figures/generated/figure_captions.md.",
    )
    add_output_args(parser)
    return parser


def main() -> int:
    """모든 그림 생성기를 순서대로 실행하고, 필요시 캡션을 갱신한 뒤 요약을 출력한다.

    Run every figure generator in order, optionally rewrite captions, print a summary,
    and return the process exit code (0). 네 카테고리의 FigureResult를 하나의 리스트로 모은다.
    Collects FigureResults from all four categories into a single list.
    """
    args = build_parser().parse_args()
    output_dirs = args.output_dirs

    # 각 생성기를 호출해 FigureResult를 누적 / call each generator, accumulating results
    results = []
    results.extend(generate_system_figures(output_dirs=output_dirs))
    results.extend(generate_path_figures(output_dirs=output_dirs, waypoint_paths=args.waypoints))
    results.extend(generate_gps_figures(output_dirs=output_dirs, gps_paths=args.gps_log))
    results.extend(
        generate_manual_control_figures(output_dirs=output_dirs, safety_paths=args.safety_log)
    )

    if not args.no_captions:
        caption_path = write_figure_captions(results)
        # parents[3] == repo 루트 → repo 상대경로로 출력 / print repo-relative path
        print(f"updated {caption_path.relative_to(caption_path.parents[3])}")

    for result in results:
        print(f"generated {result.filename} from {result.data_source}")
    return 0


# 스크립트로 직접 실행될 때만 main()을 돌리고 그 반환값을 종료코드로 사용.
# Run main() and use its return value as the exit code only when executed directly.
if __name__ == "__main__":
    raise SystemExit(main())
