"""Stage 11: 생성된 경로 패키지의 오프라인 무결성 검사기(무동작).

목적/역할:
    `field_ab_to_serpentine.py`가 만든 `path_package.json`을 읽어, 경로가 규격을
    만족하는지 순수 오프라인으로 검증한다: 도구경로가 A'에서 시작해 B'에서 끝나는지,
    구간이 연속인지, 연결자(connector)는 비활성이고 스윕 트랙은 활성인지, 프리미티브가
    모두 허용 타입인지, 어디에도 모터 명령이 없는지. 결과를 JSON/Markdown으로 남기고
    전체 유효성에 따라 종료코드(0/1)를 돌려준다.

시스템 내 위치:
    - 상류: `field_ab_to_serpentine`의 경로 패키지.
    - 이 파일의 `ALLOWED_PRIMITIVES`와 `inspect_package`는
      `tools.physical_path_preview_from_package`가 import 해 재사용한다(검증 단일 출처).
    - 패키지 해석은 `tools.path_no_motion_validation`의 `resolve_path_package`/
      `PathPackageResolutionError`를 공유한다.

핵심 개념·불변식(invariant):
    - A'(좌상)/B'(우하) 규약은 field_ab_to_serpentine의 정규화와 일치해야 한다.
    - 부동소수 비교는 `_close`(math.isclose)로 처리(좌표가 정확히 같지 않아도 통과).
    - `path_package_valid_for_preview`는 모든 개별 검증의 AND. 하나라도 False면 종료코드 1.
    - 무동작: 산출물의 motor_command_generated는 False, path_preview_only는 True.

사용법/진입점:
    CLI. `main()`이 진입점. 예: `python tools/inspect_path_package.py --path-package latest`.
    종료코드 0=유효, 1=검증 실패, 2=패키지 해석 실패.

리팩토링 노트:
    - `inspect_package`가 돌려주는 `validation` 딕셔너리의 키가 곧 마크다운/표준출력 및
      physical_path_preview 요약과의 계약이다. 항목 추가/삭제 시 소비자도 함께 확인하라.
    - 세그먼트 유형 문자열("tool_sweep_track"/"tool_spacing_connector")은 코어 플래너와의
      결합점. 코어가 이름을 바꾸면 여기 필터도 깨진다.

Stage 11 offline integrity checker for a generated path package (no motion).
Reads the `path_package.json` and verifies it purely offline: tool path starts
at A' and ends at B', segments are continuous, connectors are inactive while
sweep tracks are active, all primitives are allowed, and no motor command
appears anywhere. Writes JSON/Markdown and returns exit code 0 (valid) / 1
(invalid) / 2 (unresolved package). `ALLOWED_PRIMITIVES` and `inspect_package`
here are reused by the physical-path-preview tool; discovery is shared with
`path_no_motion_validation`.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Sequence

try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401

from tools.path_no_motion_validation import PathPackageResolutionError, resolve_path_package


# 무동작 미리보기에서 허용되는 프리미티브 집합(field_ab_to_serpentine와 동일 정의).
# Set of primitives allowed in a no-motion preview (same definition as field_ab_to_serpentine).
ALLOWED_PRIMITIVES = {"move_forward", "move_backward", "rotate_left", "rotate_right"}


def _close(a: object, b: object, tolerance: float = 1e-6) -> bool:
    """두 값이 허용오차 내로 같은지(부동소수 안전) / Float-safe near-equality within tolerance."""
    return math.isclose(float(a), float(b), abs_tol=tolerance)


# ── 검증 핵심 / Core inspection (shared with physical_path_preview) ──
def inspect_package(package: dict[str, object], selected_path: Path) -> dict[str, object]:
    """경로 패키지를 검사해 검증 플래그와 요약 통계를 담은 dict를 만든다(핵심 로직).

    무엇을/왜: 도구경로/프리미티브/워크스페이스를 읽어 다음을 판정한다 — A'에서 시작·
    B'에서 종료, 세그먼트 연속성, 연결자 비활성·스윕 트랙 활성, 프리미티브 허용 여부,
    모터 명령 부재. 개별 결과의 AND로 `path_package_valid_for_preview`를 정한다.
    반환: selected_path, 워크스페이스, 각종 카운트, primitive 요약, `validation` dict 등.
    부수효과: 없음(순수). physical_path_preview_from_package가 이 함수를 재사용한다.
    함정: 세그먼트 유형 문자열이 코어 플래너와 정확히 일치해야 필터가 동작한다.

    Core inspection: read the tool path/primitives/workspace and compute the
    validation flags (starts at A', ends at B', continuous, connectors inactive,
    sweep tracks active, primitives allowed, no motor command), AND-combined into
    path_package_valid_for_preview. Pure function reused by the physical preview.
    """
    workspace = package["normalized_workspace"]  # type: ignore[index]
    summary = package["summary"]  # type: ignore[index]
    tool_path = list(package["tool_path"])  # type: ignore[index]
    primitives = list(package["primitive_sequence"])  # type: ignore[index]
    approach = list(package.get("approach_to_A_prime", []))  # type: ignore[arg-type]
    a_prime = workspace["A_prime_top_left"]  # type: ignore[index]
    b_prime = workspace["B_prime_bottom_right"]  # type: ignore[index]

    starts_at_a_prime = bool(tool_path) and _close(tool_path[0]["tool_start_x_m"], a_prime["x_m"]) and _close(
        tool_path[0]["tool_start_y_m"], a_prime["y_m"]
    )
    ends_at_b_prime = bool(tool_path) and _close(tool_path[-1]["tool_end_x_m"], b_prime["x_m"]) and _close(
        tool_path[-1]["tool_end_y_m"], b_prime["y_m"]
    )
    continuous = all(
        _close(previous["tool_end_x_m"], current["tool_start_x_m"])
        and _close(previous["tool_end_y_m"], current["tool_start_y_m"])
        for previous, current in zip(tool_path, tool_path[1:])
    )
    # 연결자(트랙 사이 이동 구간)는 반드시 비활성·커버리지 미기여여야 함.
    # Connectors (inter-track moves) must be inactive and contribute no coverage.
    connectors_inactive = all(
        row["tool_active"] is False and row["coverage_contributes"] is False
        for row in tool_path
        if row["tool_segment_type"] == "tool_spacing_connector"
    )
    # 스윕 트랙(실제 작업 구간)은 반드시 활성·커버리지 기여여야 함.
    # Sweep tracks (the working passes) must be active and contribute coverage.
    sweep_tracks_active = all(
        row["tool_active"] is True and row["coverage_contributes"] is True
        for row in tool_path
        if row["tool_segment_type"] == "tool_sweep_track"
    )
    primitive_sequence_valid = all(
        row["primitive_type"] in ALLOWED_PRIMITIVES and row["motor_command_generated"] is False
        for row in primitives
    )
    # 패키지/요약/개별 프리미티브 어디에든 모터 플래그가 있으면 True로 승격(안전측).
    # Any motor flag anywhere (package/summary/primitive) promotes this to True (fail-safe).
    motor_command_generated = bool(package.get("motor_command_generated")) or bool(summary.get("motor_command_generated"))
    motor_command_generated = motor_command_generated or any(bool(row.get("motor_command_generated")) for row in primitives)
    validation = {
        "tool_side_left": summary.get("tool_side") == "left",
        "tool_path_starts_at_A_prime": starts_at_a_prime,
        "tool_path_ends_at_B_prime": ends_at_b_prime,
        "tool_path_continuous": continuous,
        "connectors_inactive": connectors_inactive,
        "sweep_tracks_active": sweep_tracks_active,
        "primitive_sequence_allowed": primitive_sequence_valid,
        "motor_command_generated_false": motor_command_generated is False,
    }
    # 종합 판정 = 위 모든 개별 검증의 AND. 하나라도 거짓이면 미리보기 부적합.
    # Overall verdict = AND of all checks above; any false means not valid for preview.
    validation["path_package_valid_for_preview"] = all(bool(value) for value in validation.values())
    return {
        "selected_path_package": str(selected_path),
        "normalized_workspace": workspace,
        "A_prime_top_left": a_prime,
        "B_prime_bottom_right": b_prime,
        "approach_primitive_count": len(approach),
        "tool_path_segment_count": len(tool_path),
        "tool_sweep_track_count": sum(1 for row in tool_path if row["tool_segment_type"] == "tool_sweep_track"),
        "tool_connector_count": sum(1 for row in tool_path if row["tool_segment_type"] == "tool_spacing_connector"),
        "primitive_count": len(primitives),
        "approach_primitives": approach,
        "tool_path_segments": tool_path,
        "primitive_sequence_summary": {
            "allowed_primitives": sorted(ALLOWED_PRIMITIVES),
            "primitive_types_seen": sorted({str(row["primitive_type"]) for row in primitives}),
            "first_primitive": primitives[0] if primitives else None,
            "last_primitive": primitives[-1] if primitives else None,
        },
        "validation": validation,
        "motor_command_generated": False,
        "path_preview_only": True,
    }


# ── 산출물 writer / Artifact writer (Markdown) ──
def _write_markdown(path: Path, inspection: dict[str, object]) -> None:
    """검사 결과를 사람이 읽는 Markdown으로 기록(검증 항목·프리미티브 요약 포함).

    Write the inspection result as human-readable Markdown (validation items and
    primitive summary), headed by a no-motion banner.
    """
    validation = inspection["validation"]  # type: ignore[index]
    lines = [
        "# Stage 11 Path Package Inspection",
        "",
        "Offline/no-motion only. No rover motor commands are generated.",
        "",
        f"- selected_path_package: `{inspection['selected_path_package']}`",
        f"- normalized_workspace: `{inspection['normalized_workspace']}`",
        f"- A_prime_top_left: `{inspection['A_prime_top_left']}`",
        f"- B_prime_bottom_right: `{inspection['B_prime_bottom_right']}`",
        f"- approach_primitive_count: `{inspection['approach_primitive_count']}`",
        f"- tool_path_segment_count: `{inspection['tool_path_segment_count']}`",
        f"- tool_sweep_track_count: `{inspection['tool_sweep_track_count']}`",
        f"- tool_connector_count: `{inspection['tool_connector_count']}`",
        f"- primitive_count: `{inspection['primitive_count']}`",
        "",
        "## Validation",
    ]
    for key, value in validation.items():  # type: ignore[union-attr]
        lines.append(f"- {key}: `{value}`")
    lines.extend(
        [
            "",
            "## Primitive Sequence Summary",
            f"- allowed_primitives: `{inspection['primitive_sequence_summary']['allowed_primitives']}`",  # type: ignore[index]
            f"- primitive_types_seen: `{inspection['primitive_sequence_summary']['primitive_types_seen']}`",  # type: ignore[index]
            "- motor_command_generated: `False`",
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


# ── CLI 진입점 / CLI entry point (argument parsing, main) ──
def build_parser() -> argparse.ArgumentParser:
    """CLI 인자 파서를 구성해 반환 / Build and return the CLI argument parser."""
    parser = argparse.ArgumentParser(description="Inspect a generated path package.")
    parser.add_argument("--path-package", default="latest")
    parser.add_argument("--out-dir", default="outputs/path_package_inspection/latest")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """CLI 진입점: 패키지 해석→검사→JSON/Markdown 기록·출력, 검증 결과로 종료코드 결정.

    무엇을/왜: 패키지를 찾아 inspect_package로 검사하고 결과를 저장·출력한다. 종료코드는
    유효하면 0, 검증 실패면 1, 패키지 해석 실패면 2 — 자동화가 결과를 판정에 쓸 수 있다.
    부수효과: out_dir에 JSON/Markdown을 쓰고 표준출력에 검증 항목을 찍는다. 모터 출력 없음.

    CLI entry point: resolve and inspect the package, write JSON/Markdown, and
    print validation items. Exit code: 0 valid, 1 invalid, 2 unresolved package.
    """
    args = build_parser().parse_args(argv)
    out_dir = Path(args.out_dir)
    try:
        selected = resolve_path_package(args.path_package)
    except PathPackageResolutionError as exc:
        print(f"provided_path_package={exc.provided}")
        print("file_exists=false")
        print("nearest_candidates:")
        for candidate in exc.candidates:
            print(f"- {candidate}")
        print("next_action=Run tools/field_ab_to_serpentine.py first.")
        return 2
    package = json.loads(selected.read_text(encoding="utf-8"))
    inspection = inspect_package(package, selected)
    out_dir.mkdir(parents=True, exist_ok=True)
    json_path = out_dir / "path_package_inspection.json"
    md_path = out_dir / "path_package_inspection.md"
    json_path.write_text(json.dumps(inspection, indent=2) + "\n", encoding="utf-8")
    _write_markdown(md_path, inspection)
    print("Path package inspection complete.")
    print(f"selected_path_package={selected}")
    print(f"path_package_inspection_json={json_path}")
    print(f"path_package_inspection_md={md_path}")
    for key, value in inspection["validation"].items():  # type: ignore[union-attr]
        print(f"{key}={value}")
    print("motor_command_generated=false")
    # 종합 유효성으로 종료코드 결정(자동화 게이트용): 유효=0, 실패=1.
    # Exit code from the overall verdict (for automation gating): valid=0, invalid=1.
    return 0 if inspection["validation"]["path_package_valid_for_preview"] else 1  # type: ignore[index]


if __name__ == "__main__":
    raise SystemExit(main())
