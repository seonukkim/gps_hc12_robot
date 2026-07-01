"""Stage 11: 경로 패키지로부터 "물리 경로" 미리보기 생성(소프트웨어 측, 무동작).

목적/역할:
    `field_ab_to_serpentine.py`가 만든 `path_package.json`을 읽어, 도구경로·(파생)섀시경로·
    프리미티브 시퀀스를 그림(PNG)과 검사된 CSV로 시각화한다. 각 프리미티브가 허용 타입인지
    표시하고, 패키지가 미리보기용으로 유효한지 요약한다. 이름은 "physical"이지만 **모터·
    시리얼·HC-12·GPS 코스를 전혀 쓰지 않는** 소프트웨어 미리보기다.

시스템 내 위치:
    - 상류: `field_ab_to_serpentine`의 경로 패키지.
    - 재사용: 검증 로직은 `tools.inspect_path_package.inspect_package`와 상수
      `ALLOWED_PRIMITIVES`를 그대로 가져오고, 패키지 해석은
      `tools.path_no_motion_validation.resolve_path_package`/`PathPackageResolutionError`를
      공유한다(로직 중복을 피하기 위한 의도적 결합).

핵심 개념·불변식(invariant):
    - 무동작: 산출물의 `motor_command_generated`는 항상 False, `path_preview_only`는 True.
    - 그림은 도구경로의 활성/비활성(연결자)을 색·선종류로 구분한다(active=주황 실선).
    - CSV의 `primitive_allowed`는 각 프리미티브 타입이 ALLOWED_PRIMITIVES에 드는지 여부.

사용법/진입점:
    CLI. `main()`이 진입점. 예:
    `python tools/physical_path_preview_from_package.py --path-package latest`.

리팩토링 노트:
    - `inspect_package` 반환 구조(특히 `validation`, `selected_path_package`)에 요약 출력이
      결합되어 있다. 검증 항목을 바꾸면 `_write_summary`도 함께 갱신하라.
    - matplotlib은 지연 import(선택 의존성). 없으면 각 PNG를 None으로 두고 진행한다.

Stage 11 software-side "physical path" preview from a path package. Reads the
`path_package.json`, and renders the tool path, derived chassis path, and
primitive sequence as PNGs plus a checked CSV, then summarises whether the
package is valid for preview. Despite the name it uses no motor/serial/HC-12/GPS
course. Reuses `inspect_package`/`ALLOWED_PRIMITIVES` and the shared
`resolve_path_package` for discovery. Preview-only: motor flag always False.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Sequence

try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401

from tools.inspect_path_package import ALLOWED_PRIMITIVES, inspect_package
from tools.path_no_motion_validation import PathPackageResolutionError, resolve_path_package


# 검사된 프리미티브 CSV의 컬럼 순서(=계약). primitive_allowed 열이 추가됨.
# Column order for the checked-primitive CSV (a contract); adds primitive_allowed.
CHECKED_PRIMITIVE_FIELDS = (
    "primitive_index",
    "primitive_type",
    "distance_m",
    "angle_deg",
    "segment_role",
    "start_x_m",
    "start_y_m",
    "start_heading_deg",
    "end_x_m",
    "end_y_m",
    "end_heading_deg",
    "associated_tool_segment_id",
    "tool_active",
    "coverage_contributes",
    "primitive_allowed",
    "motor_command_generated",
)


# ── 산출물 writer / Artifact writers (CSV, PNG previews, Markdown) ──
def _write_csv(path: Path, rows: Sequence[dict[str, object]]) -> None:
    """검사된 프리미티브 행을 고정 필드 순서로 CSV에 기록 / Write checked primitive rows to CSV."""
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CHECKED_PRIMITIVE_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in CHECKED_PRIMITIVE_FIELDS})


def _plot_previews(out_dir: Path, package: dict[str, object]) -> dict[str, Path | None]:
    """4종 PNG(도구경로/섀시경로/프리미티브/접근+지그재그)를 저장한다.

    무엇을/왜: 패키지의 tool_path/chassis_path/primitive_sequence를 시각화한다. 활성
    도구경로는 주황 실선, 비활성(연결자)은 회색 점선으로 구분한다. matplotlib이 없으면
    각 항목 None인 dict를 반환(선택 의존성).
    반환: {파일명: Path 또는 None}.
    좌표 주의: 이 그림들은 패키지에 이미 들어있는 좌표를 그대로 사용한다(추가 오프셋 없음).

    Save four preview PNGs (tool path / chassis path / primitive sequence /
    approach+serpentine). Active tool path is orange-solid, inactive connectors
    grey-dashed. Returns None per item if matplotlib is missing.
    """
    try:
        import matplotlib

        matplotlib.use("Agg")  # 헤드리스 백엔드 / headless backend
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle
    except ImportError:
        return {
            "preview_tool_path.png": None,
            "preview_chassis_path.png": None,
            "preview_primitive_sequence.png": None,
            "preview_approach_then_serpentine.png": None,
        }

    workspace = package["normalized_workspace"]  # type: ignore[index]
    tool_path = package["tool_path"]  # type: ignore[index]
    chassis_path = package.get("chassis_path", [])
    primitives = package["primitive_sequence"]  # type: ignore[index]
    x_min = float(workspace["x_min_m"])  # type: ignore[index]
    x_max = float(workspace["x_max_m"])  # type: ignore[index]
    y_min = float(workspace["y_min_m"])  # type: ignore[index]
    y_max = float(workspace["y_max_m"])  # type: ignore[index]

    def setup(title: str):
        """공통 축 세팅(작업영역 사각형·격자·미리보기 배너)을 적용한 fig/ax 반환.

        Return a fig/ax with the shared axis setup (workspace rectangle, grid,
        preview-only banner).
        """
        fig, ax = plt.subplots(figsize=(9, 5))
        ax.add_patch(Rectangle((x_min, y_min), x_max - x_min, y_max - y_min, fill=False, edgecolor="black"))
        ax.set_title(title)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.axis("equal")
        ax.grid(True, linestyle="--", alpha=0.3)
        ax.text(0.01, 0.01, "Stage 11 preview only: no motor commands", transform=ax.transAxes, fontsize=8)
        return fig, ax

    outputs: dict[str, Path | None] = {}
    fig, ax = setup("Tool Path From Path Package")
    for row in tool_path:
        active = bool(row["tool_active"])
        ax.plot(
            [float(row["tool_start_x_m"]), float(row["tool_end_x_m"])],
            [float(row["tool_start_y_m"]), float(row["tool_end_y_m"])],
            color="tab:orange" if active else "0.55",
            linestyle="-" if active else "--",
            linewidth=2.4 if active else 1.2,
        )
    path = out_dir / "preview_tool_path.png"
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    outputs[path.name] = path

    fig, ax = setup("Derived Chassis Path From Package")
    for row in chassis_path:  # type: ignore[assignment]
        ax.plot(
            [float(row["chassis_start_x_m"]), float(row["chassis_end_x_m"])],
            [float(row["chassis_start_y_m"]), float(row["chassis_end_y_m"])],
            color="tab:blue",
            linewidth=1.5,
        )
    for row in tool_path:
        ax.plot(
            [float(row["tool_start_x_m"]), float(row["tool_end_x_m"])],
            [float(row["tool_start_y_m"]), float(row["tool_end_y_m"])],
            color="tab:orange",
            alpha=0.25,
            linewidth=1.0,
        )
    path = out_dir / "preview_chassis_path.png"
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    outputs[path.name] = path

    fig, ax = setup("Primitive Sequence From Package")
    for row in primitives:
        sx = float(row["start_x_m"])
        sy = float(row["start_y_m"])
        ex = float(row["end_x_m"])
        ey = float(row["end_y_m"])
        ptype = str(row["primitive_type"])
        if ptype.startswith("move"):
            ax.annotate("", xy=(ex, ey), xytext=(sx, sy), arrowprops={"arrowstyle": "->", "color": "tab:green"})
        else:
            ax.scatter([sx], [sy], color="tab:purple", s=22)
        ax.text(sx, sy, f"P{int(row['primitive_index']):03d}", fontsize=6)
    path = out_dir / "preview_primitive_sequence.png"
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    outputs[path.name] = path

    fig, ax = setup("Approach Then Serpentine From Package")
    for row in primitives:
        if not str(row["primitive_type"]).startswith("move"):
            continue
        role = str(row["segment_role"])
        color = "tab:green" if role.startswith("approach") else "tab:blue"
        ax.plot([float(row["start_x_m"]), float(row["end_x_m"])], [float(row["start_y_m"]), float(row["end_y_m"])], color=color)
    path = out_dir / "preview_approach_then_serpentine.png"
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    outputs[path.name] = path
    return outputs


def _write_summary(path: Path, inspection: dict[str, object], preview_paths: dict[str, Path | None]) -> None:
    """검증 결과와 생성된 미리보기 목록을 Markdown 요약으로 기록.

    무엇을/왜: inspect_package의 validation 플래그(시작/종료/연속성/연결자·트랙 활성/
    프리미티브 허용 등)와 산출 PNG 경로를 남긴다. 무동작임을 배너로 명시.

    Write a Markdown summary of the validation flags and generated previews,
    headed by a no-motion banner.
    """
    validation = inspection["validation"]  # type: ignore[index]
    lines = [
        "# Stage 11 Physical Path Preview From Package",
        "",
        "Software-side no-motion preview only. This does not use GPS course, serial, HC-12, or motor output.",
        "",
        f"- selected_path_package: `{inspection['selected_path_package']}`",
        f"- tool_path_starts_at_A_prime: `{validation['tool_path_starts_at_A_prime']}`",
        f"- tool_path_ends_at_B_prime: `{validation['tool_path_ends_at_B_prime']}`",
        f"- tool_path_continuous: `{validation['tool_path_continuous']}`",
        f"- connectors_inactive: `{validation['connectors_inactive']}`",
        f"- sweep_tracks_active: `{validation['sweep_tracks_active']}`",
        f"- primitive_sequence_allowed: `{validation['primitive_sequence_allowed']}`",
        f"- motor_command_generated: `False`",
        f"- path_preview_only: `True`",
        "",
        "## Outputs",
    ]
    for name, output in preview_paths.items():
        lines.append(f"- {name}: `{output if output is not None else 'not generated'}`")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


# ── CLI 진입점 / CLI entry point (argument parsing, main) ──
def build_parser() -> argparse.ArgumentParser:
    """CLI 인자 파서를 구성해 반환 / Build and return the CLI argument parser."""
    parser = argparse.ArgumentParser(description="Generate a physical path preview from path_package.json.")
    parser.add_argument("--path-package", default="latest")
    parser.add_argument("--out-dir", default="outputs/physical_path_preview/latest")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """CLI 진입점: 패키지 해석→검사→CSV/PNG/요약 기록·출력, 종료코드 0(해석 실패 시 2).

    무엇을/왜: 패키지를 찾아 inspect_package로 검사하고, 각 프리미티브에
    primitive_allowed/motor_command_generated(False)를 덧붙인 CSV와 미리보기 PNG,
    요약을 만든다. 모터/시리얼/GPS 코스 없음.
    부수효과: out_dir에 산출물을 쓰고 표준출력에 경로/플래그를 찍는다.

    CLI entry point: resolve the package, inspect it, and write a checked CSV,
    preview PNGs, and a summary. No motor/serial/GPS course. Returns 0, or 2 if
    the package cannot be resolved.
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
        print("next_action=Run tools/field_ab_to_serpentine.py first.")
        return 2
    package = json.loads(selected.read_text(encoding="utf-8"))
    inspection = inspect_package(package, selected)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    # 각 프리미티브에 허용 여부 열을 덧붙이고 모터 플래그를 명시적으로 False로 고정.
    # Annotate each primitive with an allowed flag and pin the motor flag to False.
    checked_rows = []
    for row in package["primitive_sequence"]:  # type: ignore[index]
        updated = dict(row)
        updated["primitive_allowed"] = row["primitive_type"] in ALLOWED_PRIMITIVES
        updated["motor_command_generated"] = False
        checked_rows.append(updated)
    csv_path = out_dir / "primitive_sequence_checked.csv"
    _write_csv(csv_path, checked_rows)
    preview_paths = _plot_previews(out_dir, package)
    summary_path = out_dir / "summary.md"
    _write_summary(summary_path, inspection, preview_paths)
    print("Stage 11 physical path preview generated.")
    print(f"selected_path_package={selected}")
    print(f"summary_md={summary_path}")
    print(f"primitive_sequence_checked_csv={csv_path}")
    for name, output in preview_paths.items():
        print(f"{name}={output if output is not None else 'not_generated'}")
    print("gps_course_required=false")
    print("serial_opened=false")
    print("motor_command_generated=false")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
