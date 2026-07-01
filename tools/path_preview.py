"""레거시 잔디깎기(lawnmower) 커버리지 경로 미리보기 — `make path-preview` 전용.

목적/역할:
    직사각형의 두 대각 코너(A/B) 위경도와 행 간격(spacing)을 받아, 코어 플래너
    `gps_coverage_core.planner.generate_lawnmower_path`로 잔디깎기(보스트로페돈) 경로를
    만들고 그 궤적을 PNG 한 장으로 저장한다. 시각 검수용 **미리보기 전용** 도구다.

시스템 내 위치:
    - `Makefile`의 `path-preview` 타깃이 이 파일을 고정 좌표/간격으로 실행한다
      (프로젝트에 배선이 잘 되어 있는지 확인하는 스모크(smoke) 용도).
    - 코어의 레거시 API `generate_lawnmower_path(point_a, point_b, spacing)`를 사용하며,
      이는 신형 `side_tool_planner` 계열(field_ab_to_serpentine)과는 별개의 옛 경로다.

핵심 개념·불변식(invariant):
    - 좌표: 코어가 돌려주는 각 점의 x(동)/y(북) 미터를 그대로 그린다.
    - `--port`는 CLI 형식 일관성을 위한 자리표시자일 뿐, 시리얼을 열지 않는다.
    - 출력 파일명 기본값은 import(모듈 로드) 시각으로 한 번 결정된다(호출 시각 아님).

사용법/진입점:
    CLI. `main()`이 진입점(인자 없이 `parse_args()` 사용). 대표 실행은
    `make path-preview`이며 내부적으로 `python tools/path_preview.py --lat-a .. --spacing 5.0`.

리팩토링 노트:
    - 이 파일은 matplotlib을 **모듈 상단에서 즉시** import 한다(다른 tools의 지연 import와
      다름). 헤드리스에서 안전하도록 import 직후 `matplotlib.use("Agg")`를 호출한다 —
      pyplot import보다 먼저여야 하므로 이 import 순서를 유지할 것.
    - `_bootstrap`은 `sys.path` 설정용 부작용 import(직접 사용 안 함).
    - 신형 파이프라인으로 대체 예정인 레거시. 새 기능은 여기 말고 신형 도구에 추가하라.

Legacy lawnmower coverage-path preview, run by `make path-preview`. Takes two
diagonal corners (lat/lon) plus a row spacing, delegates to the core
`generate_lawnmower_path`, and saves the trajectory as a single PNG. Preview-only
(visual inspection). `--port` is a placeholder for CLI parity and opens nothing.
matplotlib is imported eagerly at module top with the Agg backend selected
before pyplot — keep that import order for headless safety.
"""

from __future__ import annotations

import argparse
import datetime as dt
from pathlib import Path

import _bootstrap  # noqa: F401
import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt

from gps_coverage_core.planner import generate_lawnmower_path


def build_parser() -> argparse.ArgumentParser:
    """CLI 인자 파서를 구성해 반환(--port는 자리표시자) / Build the CLI parser (--port is a placeholder)."""
    parser = argparse.ArgumentParser(description="Preview a lawnmower coverage path.")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Unused placeholder for CLI consistency")
    parser.add_argument("--lat-a", type=float, required=True, help="First corner latitude")
    parser.add_argument("--lon-a", type=float, required=True, help="First corner longitude")
    parser.add_argument("--lat-b", type=float, required=True, help="Opposite corner latitude")
    parser.add_argument("--lon-b", type=float, required=True, help="Opposite corner longitude")
    parser.add_argument("--spacing", type=float, required=True, help="Row spacing in meters")
    parser.add_argument(
        "--output",
        default=f"outputs/path_preview_{dt.datetime.now():%Y%m%d_%H%M%S}.png",
        help="Figure output path",
    )
    return parser


def main() -> int:
    """CLI 진입점: 코너 A/B·간격으로 잔디깎기 경로를 만들어 PNG로 저장, 종료코드 0.

    무엇을/왜: 코어 `generate_lawnmower_path`로 경로를 생성하고 x/y를 뽑아 한 장의
    미리보기 그림을 저장한다. 모터/시리얼/펌웨어와 무관한 순수 시각화.
    부수효과: --output 경로에 PNG를 쓰고 웨이포인트 수를 출력.

    CLI entry point: build a lawnmower path from corners A/B and spacing, then
    save one preview PNG. Pure visualisation; no motor/serial/firmware.
    """
    args = build_parser().parse_args()
    point_a = {"lat": args.lat_a, "lon": args.lon_a}
    point_b = {"lat": args.lat_b, "lon": args.lon_b}
    path = generate_lawnmower_path(point_a, point_b, args.spacing)

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    xs = [float(point["x"]) for point in path]
    ys = [float(point["y"]) for point in path]

    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot(xs, ys, marker="o", linewidth=1.5)
    ax.set_title("Lawnmower Path Preview")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.grid(True, linestyle="--", alpha=0.4)
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)

    print(f"Saved {len(path)} waypoints to preview figure: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
