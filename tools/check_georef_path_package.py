"""경로 패키지의 지오레퍼런스 메타데이터 점검 도구 / Inspect georeference in a path package.

목적/역할:
    `path_package.json` 안의 georeference 블록을 읽어, 위경도 원점·raw A/B 좌표를 로컬 좌표로
    변환해 보고, 예상 사각형 폭/높이와 함께 변환이 정상인지(sanity) 진단한다. 사람이 읽는
    `key=value` 요약을 출력하고, --out-dir가 주어지면 `georef_path_package_check.json`도 남긴다.
    검증 전용이며 어떤 모터 명령도 만들지 않는다.

    Reads the georeference block of a `path_package.json`, converts raw A/B lat/lon to local
    coordinates, and reports a conversion sanity check alongside the expected rectangle
    width/height. Prints a key=value summary and optionally writes
    `georef_path_package_check.json`. Inspection only; never generates motor commands.

시스템 내 위치:
    `tools/` CLI 진입점. 경로 패키지 선택은 `tools.path_no_motion_validation`의
    resolve_path_package(및 PathPackageResolutionError)에, 좌표 변환은
    `tools.station_path_package_tracker.lat_lon_to_local`에 의존한다. 위경도 A/B는 보통
    `tools/capture_georef_ab_points.py`에서 캡처되어 경로 패키지에 들어간다.

    Entry-point script in `tools/`. Depends on resolve_path_package /
    PathPackageResolutionError from `tools.path_no_motion_validation` for package selection
    and on `tools.station_path_package_tracker.lat_lon_to_local` for the coordinate
    conversion. The lat/lon A/B usually originate from `capture_georef_ab_points.py`.

핵심 개념·불변식:
    - georeference 블록이 없으면 실패가 아니라 `georeference_available=False`를 보고하고
      main()은 종료코드 1을 반환한다(=지오레퍼런스 없음).
    - conversion_sanity_check는 A·B 로컬 변환이 모두 성공하고 폭·높이가 모두 > 0일 때만 True다.
    - 반환 dict에는 항상 `motor_command_generated=False`가 포함된다.
    - 종료 코드: 패키지 자체를 못 찾으면 2, 지오레퍼런스 없음이면 1, 정상이면 0.

    - A missing georeference block is not an error: it reports
      `georeference_available=False` and main() returns exit code 1.
    - conversion_sanity_check is True only when both A/B convert and width/height are > 0.
    - The result dict always includes `motor_command_generated=False`.
    - Exit codes: 2 if the package cannot be resolved, 1 if no georeference, else 0.

사용법/진입점:
    CLI 진입점은 main(). 예: `python -m tools.check_georef_path_package --path-package latest`.
    --path-package 기본값은 "latest"(가장 최근 패키지). --out-dir로 JSON 리포트 저장 가능.

    CLI entry point is main(); --path-package defaults to "latest", --out-dir writes a report.

리팩토링 노트:
    출력은 순서 있는 dict를 `key=value`로 그대로 찍는다 — 키 이름·순서를 바꾸면 이 요약을
    파싱하는 도구에 영향을 준다. 종료 코드 계약(0/1/2)을 유지할 것.

    Output prints an ordered dict as key=value; changing key names/order affects any parser
    of this summary. Preserve the 0/1/2 exit-code contract.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Sequence

try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401

from tools.path_no_motion_validation import PathPackageResolutionError, resolve_path_package
from tools.station_path_package_tracker import lat_lon_to_local


def inspect_georef(package: dict[str, object], selected_path: Path) -> dict[str, object]:
    """경로 패키지의 georeference를 검사해 진단 결과 dict를 만든다.

    Inspect the package's georeference and return a diagnostic result dict.
    georeference가 없으면 available=False 결과를 반환 / returns available=False if absent.
    raw A/B를 로컬로 변환하고 폭·높이로 sanity를 판정 / converts A/B and checks sanity.
    부수효과 없음(순수 함수) / no side effects (pure).
    """
    georef = package.get("georeference")
    workspace = package.get("normalized_workspace", {})
    # georeference 블록 부재는 오류가 아니라 '사용 불가'로 보고 / absence is reported, not raised
    if not isinstance(georef, dict):
        return {
            "selected_path_package": str(selected_path),
            "georeference_available": False,
            "reason": "NO_GEOREFERENCE_IN_PATH_PACKAGE",
            "motor_command_generated": False,
        }
    raw_a_lat = float(georef["raw_A_lat"])
    raw_a_lon = float(georef["raw_A_lon"])
    raw_b_lat = float(georef["raw_B_lat"])
    raw_b_lon = float(georef["raw_B_lon"])
    a_local = lat_lon_to_local(package, raw_a_lat, raw_a_lon)
    b_local = lat_lon_to_local(package, raw_b_lat, raw_b_lon)
    width = float(workspace.get("width_m", 0.0)) if isinstance(workspace, dict) else 0.0
    height = float(workspace.get("height_m", 0.0)) if isinstance(workspace, dict) else 0.0
    # 변환 성공 + 양의 작업공간 크기일 때만 정상으로 판정 / sane only if both convert and area>0
    sanity_ok = a_local is not None and b_local is not None and width > 0 and height > 0
    return {
        "selected_path_package": str(selected_path),
        "georeference_available": True,
        "origin_lat": georef["origin_lat"],
        "origin_lon": georef["origin_lon"],
        "raw_A_lat": raw_a_lat,
        "raw_A_lon": raw_a_lon,
        "raw_B_lat": raw_b_lat,
        "raw_B_lon": raw_b_lon,
        "A_local_coordinates": a_local,
        "B_local_coordinates": b_local,
        "local_frame_type": georef["local_frame_type"],
        "x_axis_source": georef["x_axis_source"],
        "meters_per_deg_lat": georef["meters_per_deg_lat"],
        "meters_per_deg_lon": georef["meters_per_deg_lon"],
        "expected_rectangle_width_m": width,
        "expected_rectangle_height_m": height,
        "conversion_sanity_check": sanity_ok,
        "motor_command_generated": False,
    }


def build_parser() -> argparse.ArgumentParser:
    """이 도구의 argparse 파서를 구성한다 / build the argparse parser for this tool."""
    parser = argparse.ArgumentParser(description="Check georeference metadata in a path_package.json.")
    parser.add_argument("--path-package", default="latest")
    parser.add_argument("--out-dir")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """CLI 진입점: 패키지 해석→지오레퍼런스 검사→요약 출력(선택적 JSON 저장).

    CLI entry point: resolve package, inspect georeference, print summary, optionally save JSON.
    반환 코드 2=패키지 미해석, 1=지오레퍼런스 없음, 0=정상 / 2/1/0 exit codes as documented.
    """
    args = build_parser().parse_args(argv)
    try:
        selected = resolve_path_package(args.path_package)
    except PathPackageResolutionError as exc:
        print(f"provided_path_package={exc.provided}")
        print("file_exists=false")
        print("nearest_candidates:")
        for candidate in exc.candidates:
            print(f"- {candidate}")
        return 2
    package = json.loads(selected.read_text(encoding="utf-8"))
    result = inspect_georef(package, selected)
    for key, value in result.items():
        print(f"{key}={value}")
    if args.out_dir:
        out_dir = Path(args.out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        (out_dir / "georef_path_package_check.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    return 0 if result.get("georeference_available") else 1


if __name__ == "__main__":
    raise SystemExit(main())
