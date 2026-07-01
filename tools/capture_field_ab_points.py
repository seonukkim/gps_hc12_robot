"""필드 A/B 지점 캡처 도구 (로컬 미터 좌표) / Capture field A/B points in local meters.

목적/역할:
    오프라인 경로 계획을 위해 필드의 두 기준점 A·B를 로컬 미터(x, y) 좌표로 기록한다.
    측량한 두 지점만 있으면 이후 단계에서 경로를 생성할 수 있도록 `field_points.json`과
    `field_points.csv`를 남기는 것이 전부다. 시리얼 포트를 열지 않고, 로버에 명령을 보내지
    않으며, 모터 출력을 생성하지 않는다(안전상 preview 전용).

    Records two field reference points A and B as local-meter (x, y) coordinates for
    offline path planning. It only writes `field_points.json` / `field_points.csv`; it
    never opens a serial port, never commands the rover, and never generates motor output.

시스템 내 위치:
    `tools/` CLI 유틸리티 모음의 진입점 스크립트. `_bootstrap`(경로 부트스트랩)만 import 하고
    다른 프로젝트 모듈에 의존하지 않는 독립 캡처 단계다. 위경도 기반 캡처가 필요하면 짝이 되는
    `tools/capture_georef_ab_points.py`를 사용한다. 여기서 만든 A/B 지점은 하류 경로 생성
    도구가 소비한다.

    Entry-point script in the `tools/` CLI collection. Standalone capture stage that only
    imports `_bootstrap`; use the sibling `tools/capture_georef_ab_points.py` for
    lat/lon-based capture. Downstream path-generation tools consume these A/B points.

핵심 개념·불변식:
    - 좌표는 로컬 미터 평면(x, y)이며, 위경도가 아니다. capture_mode는 항상
      "manual_local_meters"로 고정된다.
    - 출력 JSON에는 안전 플래그 `path_preview_only=True`, `motor_command_generated=False`가
      항상 포함된다 — 이 스크립트는 결코 물리 출력을 만들지 않는다는 계약.
    - 각 지점의 sample_count는 수동 입력이므로 1이다(평균 없음).

    - Coordinates are a local-meter (x, y) plane, not lat/lon. capture_mode is fixed to
      "manual_local_meters". The output always carries the safety flags
      `path_preview_only=True` and `motor_command_generated=False`.

사용법/진입점:
    CLI: `python -m tools.capture_field_ab_points --a-x 0 --a-y 0 --b-x 10 --b-y 0`.
    A/B 좌표를 인자로 주지 않으면 대화형으로 'x,y' 입력을 요청한다. 진입점은 main()이다.

    CLI entry point is main(); pass --a-x/--a-y/--b-x/--b-y or answer the interactive
    'x,y' prompts.

리팩토링 노트:
    CSV 컬럼 순서는 CSV_FIELDS로 고정되어 있다. 위경도 캡처와 출력 스키마를 맞출 때는 짝
    스크립트와 필드 이름을 함께 확인할 것. 안전 플래그를 제거·변경하지 말 것.

    CSV column order is pinned by CSV_FIELDS. Keep the safety flags intact when refactoring.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
from pathlib import Path
from typing import Sequence

try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401


CSV_FIELDS = ("point_label", "x_m", "y_m", "sample_count")


def _point_from_args_or_prompt(
    *,
    label: str,
    x_value: float | None,
    y_value: float | None,
) -> tuple[float, float]:
    """CLI 인자 우선, 없으면 'x,y' 대화형 입력으로 한 지점을 얻는다.

    Return one (x, y) point: use CLI args if both given, else prompt for 'x,y'.
    ValueError는 입력 형식이 x,y가 아닐 때 발생 / raises ValueError on malformed input.
    """
    # 두 좌표가 모두 인자로 주어졌을 때만 프롬프트를 건너뛴다 / skip prompt only if both provided
    if x_value is not None and y_value is not None:
        return x_value, y_value
    raw = input(f"Enter point {label} as 'x,y' in local meters: ").strip()
    parts = [part.strip() for part in raw.split(",")]
    if len(parts) != 2:
        raise ValueError(f"point {label} must be entered as x,y")
    return float(parts[0]), float(parts[1])


def build_field_points(
    *,
    a_x: float,
    a_y: float,
    b_x: float,
    b_y: float,
) -> dict[str, object]:
    """A/B 로컬 좌표를 안전 플래그가 포함된 캡처 페이로드 dict로 조립한다.

    Assemble the capture payload dict from A/B local coords, including safety flags.
    부수효과 없음(순수 함수) / no side effects (pure).
    """
    return {
        "capture_mode": "manual_local_meters",
        "generated_at_utc": dt.datetime.now(tz=dt.UTC).isoformat(),
        "points": {
            "A": {"x_m": a_x, "y_m": a_y, "sample_count": 1},
            "B": {"x_m": b_x, "y_m": b_y, "sample_count": 1},
        },
        "path_preview_only": True,
        "motor_command_generated": False,
    }


def write_field_points(out_dir: Path, data: dict[str, object]) -> tuple[Path, Path]:
    """캡처 페이로드를 field_points.json/csv 두 파일로 기록하고 경로를 반환한다.

    Write the payload to field_points.json and field_points.csv; return (json, csv) paths.
    부수효과: out_dir 생성 및 두 파일 기록 / side effect: creates out_dir and writes two files.
    """
    out_dir.mkdir(parents=True, exist_ok=True)
    json_path = out_dir / "field_points.json"
    csv_path = out_dir / "field_points.csv"
    json_path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    points = data["points"]  # type: ignore[index]
    with csv_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for label in ("A", "B"):
            point = points[label]  # type: ignore[index]
            writer.writerow(
                {
                    "point_label": label,
                    "x_m": point["x_m"],
                    "y_m": point["y_m"],
                    "sample_count": point["sample_count"],
                }
            )
    return json_path, csv_path


def build_parser() -> argparse.ArgumentParser:
    """이 도구의 argparse 파서를 구성한다 / build the argparse parser for this tool."""
    parser = argparse.ArgumentParser(
        description=(
            "Capture field A/B points for offline path planning. Manual local-meter "
            "input is supported now; no serial ports are opened and no motor commands "
            "are generated."
        )
    )
    parser.add_argument("--a-x", type=float)
    parser.add_argument("--a-y", type=float)
    parser.add_argument("--b-x", type=float)
    parser.add_argument("--b-y", type=float)
    parser.add_argument(
        "--out-dir",
        default="outputs/field_ab_capture/latest",
        help="Directory for field_points.json and field_points.csv",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """CLI 진입점: A/B 지점을 확보→페이로드 조립→파일 기록→요약 출력. 항상 0 반환.

    CLI entry point: capture A/B, build payload, write files, print summary; always returns 0.
    """
    args = build_parser().parse_args(argv)
    a_x, a_y = _point_from_args_or_prompt(label="A", x_value=args.a_x, y_value=args.a_y)
    b_x, b_y = _point_from_args_or_prompt(label="B", x_value=args.b_x, y_value=args.b_y)
    data = build_field_points(a_x=a_x, a_y=a_y, b_x=b_x, b_y=b_y)
    json_path, csv_path = write_field_points(Path(args.out_dir), data)
    print("Field A/B points captured.")
    print("Preview only: no serial monitor, no rover commands, no motor output.")
    print(f"field_points_json: {json_path}")
    print(f"field_points_csv: {csv_path}")
    print("motor_command_generated: False")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
