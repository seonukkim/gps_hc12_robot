"""Stage 14 스테이션 가상 경로 제어기 — 진단 전용 가상 명령 프리뷰.
Stage 14 station virtual path controller (diagnostic-only virtual commands).

목적/역할:
    Stage 12 타깃 상태 행(거리/방위/cross-track/헤딩 오차) 위에, 로버가 이 타깃을 따라가려면
    어떤 전진/회전량이 필요한지 **가상으로** 계산해 좌/우 바퀴 명령까지 프리뷰한다. 이 값들은
    firmware 모터 명령이 아니다 — 순수 진단 수치다. CSV/요약/그래프 PNG 로 출력한다.
    On top of Stage 12 target rows, it computes *virtual* forward/turn (and L/R
    wheel) commands as a preview. These are NOT firmware motor commands — pure
    diagnostics. Outputs CSV + summary + a plot PNG.

시스템 내 위치:
    ``tools.station_path_package_tracker`` 를 재사용한다:
    ``compute_target_status`` / ``build_rows_from_replay`` / ``_read_live_usbdbg`` 로 먼저
    타깃 행을 만든 뒤 그 위에 가상 제어를 얹는다. 경로 패키지 해석은
    ``path_no_motion_validation.resolve_path_package`` 에 위임한다. 상위 파이프라인에 속하지
    않는 독립 진단 CLI 다.
    Reuses ``station_path_package_tracker`` for target rows, then layers virtual
    control; delegates package resolution to ``path_no_motion_validation``.

핵심 개념·불변식:
    - **무모터/무전송 불변식**: ``virtual_control_generated`` 가 True 여도 모터/시리얼로는
      아무것도 보내지 않는다. ``motor_command_generated`` / ``physical_output_active`` /
      ``ready_for_motor_test`` 는 항상 False. 이 진단값을 firmware 로 전달하지 말 것 (함정).
    - 전진/회전 명령은 ``max_virtual_forward_cmd`` / ``max_virtual_turn_cmd`` 로 클램프된다.
    - 헤딩 유무에 따라 회전 산식이 갈린다: 헤딩 있으면 heading_error 기반(OK), 없으면 target
      bearing 기반(DIAG_ONLY). 로컬 타깃이 없으면 NO_LOCAL_TARGET 으로 0 명령.
    - Invariants: nothing is ever sent (motor/serial); virtual commands are clamped
      to the max-* limits; turn law differs by heading availability (OK vs DIAG_ONLY).

사용법/진입점:
    ``python tools/station_virtual_path_controller.py --mode {...} --out-dir OUT [...]`` —
    ``main()`` 이 진입점. 모드는 Stage 12 와 동일(offline_pose/replay_log/live_usbdbg).
    Entry point ``main()``; same three modes as Stage 12.

리팩토링 노트:
    ``VIRTUAL_FIELDS`` 순서는 CSV 계약이다. 튜닝 계수(0.7/0.3, 0.8/0.2, /90, /45 등)는
    ``compute_virtual_control`` 안에 하드코딩돼 있으니 제어 특성을 바꿀 때 이 함수만 손대면 된다.
    ``VIRTUAL_FIELDS`` order is the CSV contract; tuning constants live inside
    ``compute_virtual_control``.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Sequence

try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401

from tools.path_no_motion_validation import PathPackageResolutionError, resolve_path_package
from tools import station_path_package_tracker


# ── CSV 스키마 / CSV schema ──
# virtual_control.csv 의 열 순서·이름 계약. 맨 끝 motor/ready_for_motor_test 는 항상 False.
# Column order/names for virtual_control.csv; motor fields are always False.
VIRTUAL_FIELDS = (
    "row_index",
    "mode",
    "station_package_target_source",
    "firmware_active_target_source",
    "firmware_still_compile_time",
    "local_pose_available",
    "active_primitive_index",
    "active_tool_segment_id",
    "target_distance_m",
    "target_bearing_deg",
    "cross_track_error_m",
    "current_heading_deg",
    "heading_error_deg",
    "virtual_heading_status",
    "virtual_forward_cmd",
    "virtual_turn_cmd",
    "virtual_left_cmd",
    "virtual_right_cmd",
    "virtual_control_generated",
    "tool_active_expected",
    "coverage_contributes",
    "motor_command_generated",
    "physical_output_active",
    "ready_for_station_virtual_control_preview",
    "ready_for_motor_test",
)


# ── 스칼라 헬퍼 / Scalar helpers ──


def _optional_float(value: object) -> float | None:
    """유한 float 로 변환하되 빈 값/NA/NaN/변환 실패는 None. / Parse to finite float or None."""
    if value is None:
        return None
    text = str(value).strip()
    if text.upper() in {"", "NA", "NAN", "NONE", "NULL"}:
        return None
    try:
        parsed = float(text)
    except ValueError:
        return None
    return parsed if math.isfinite(parsed) else None


def _clamp(value: float, limit: float) -> float:
    """대칭 클램프: 값을 [-limit, +limit] 로 제한한다. / Symmetric clamp to [-limit, +limit]."""
    return max(-limit, min(limit, value))


# ── 가상 제어 계산 / Virtual control computation ──


def compute_virtual_control(
    target_row: dict[str, object],
    *,
    max_virtual_forward_cmd: float = 0.10,
    max_virtual_turn_cmd: float = 0.05,
    lookahead_m: float = 1.0,
) -> dict[str, object]:
    """한 타깃 행에서 가상 전진/회전/좌우 명령을 계산한 진단 행을 만든다(전송 없음).
    Compute virtual forward/turn/L-R commands for one target row (never sent).

    로컬 포즈+유한 타깃이 없으면 0 명령 + NO_LOCAL_TARGET. 있으면 전진은 거리 비례(상한
    클램프), 회전은 헤딩 유무로 분기: 헤딩 있으면 heading_error 위주(OK), 없으면 target
    bearing 위주(DIAG_ONLY). 좌/우 = 전진 ∓ 회전.
    부수효과 없음. 모터/전송 관련 필드는 항상 False.
    No local pose/finite target → zero commands. Otherwise forward ∝ distance
    (clamped); turn law branches on heading availability. Pure; motor fields False."""
    distance = _optional_float(target_row.get("target_distance_m"))
    bearing = _optional_float(target_row.get("target_bearing_deg"))
    cross_track = _optional_float(target_row.get("cross_track_error_m"))
    heading = _optional_float(target_row.get("current_heading_deg"))
    heading_error = _optional_float(target_row.get("heading_error_deg"))
    local_pose_available = target_row.get("local_pose_available") is True
    target_finite = distance is not None and bearing is not None and cross_track is not None
    if not local_pose_available or not target_finite:
        virtual_forward = 0.0
        virtual_turn = 0.0
        virtual_heading_status = "NO_LOCAL_TARGET"
        control_generated = False
    else:
        # 전진은 거리 비례(계수 0.2)이되 상한으로 클램프 / forward ∝ distance, capped.
        virtual_forward = min(max_virtual_forward_cmd, max(0.0, float(distance) * 0.2))
        if heading is None or heading_error is None:
            # 헤딩 미확보: 실제 헤딩 오차를 모르므로 target bearing 을 대리로 사용(진단 전용).
            # No heading: fall back to target bearing as a proxy (diagnostic only).
            virtual_heading_status = "DIAG_ONLY"
            bearing_component = _clamp(float(bearing) / 90.0, 1.0)
            cross_component = _clamp(float(cross_track) / max(lookahead_m, 1e-6), 1.0)
            virtual_turn = _clamp((0.7 * bearing_component + 0.3 * cross_component) * max_virtual_turn_cmd, max_virtual_turn_cmd)
        else:
            # 헤딩 확보: heading_error 를 주(0.8)로, cross-track 을 보조(0.2)로 섞는다.
            # Heading known: weight heading_error (0.8) over cross-track (0.2).
            virtual_heading_status = "OK"
            heading_component = _clamp(float(heading_error) / 45.0, 1.0)
            cross_component = _clamp(float(cross_track) / max(lookahead_m, 1e-6), 1.0)
            virtual_turn = _clamp((0.8 * heading_component + 0.2 * cross_component) * max_virtual_turn_cmd, max_virtual_turn_cmd)
        control_generated = True
    # 차동 구동 프리뷰: 좌/우 바퀴 = 전진 ∓ 회전 / differential drive: L/R = forward ∓ turn.
    virtual_left = virtual_forward - virtual_turn
    virtual_right = virtual_forward + virtual_turn
    ready_preview = bool(local_pose_available and target_finite and control_generated)
    return {
        "row_index": target_row.get("row_index", 0),
        "mode": target_row.get("mode", ""),
        "station_package_target_source": target_row.get("station_package_target_source", "path_package"),
        "firmware_active_target_source": target_row.get("firmware_active_target_source", "unknown"),
        "firmware_still_compile_time": target_row.get("firmware_still_compile_time", False),
        "local_pose_available": local_pose_available,
        "active_primitive_index": target_row.get("active_primitive_index", "NA"),
        "active_tool_segment_id": target_row.get("active_tool_segment_id", "NA"),
        "target_distance_m": "NA" if distance is None else distance,
        "target_bearing_deg": "NA" if bearing is None else bearing,
        "cross_track_error_m": "NA" if cross_track is None else cross_track,
        "current_heading_deg": "NA" if heading is None else heading,
        "heading_error_deg": "NA_DIAG_ONLY" if heading_error is None else heading_error,
        "virtual_heading_status": virtual_heading_status,
        "virtual_forward_cmd": virtual_forward,
        "virtual_turn_cmd": virtual_turn,
        "virtual_left_cmd": virtual_left,
        "virtual_right_cmd": virtual_right,
        "virtual_control_generated": control_generated,
        "tool_active_expected": target_row.get("tool_active_expected", "NA"),
        "coverage_contributes": target_row.get("coverage_contributes", "NA"),
        "motor_command_generated": False,
        "physical_output_active": False,
        "ready_for_station_virtual_control_preview": ready_preview,
        "ready_for_motor_test": False,
    }


def build_virtual_rows(
    target_rows: Sequence[dict[str, object]],
    *,
    max_virtual_forward_cmd: float = 0.10,
    max_virtual_turn_cmd: float = 0.05,
    lookahead_m: float = 1.0,
) -> list[dict[str, object]]:
    """모든 타깃 행에 ``compute_virtual_control`` 을 적용해 가상 제어 행 목록을 만든다.
    Map ``compute_virtual_control`` over all target rows."""
    return [
        compute_virtual_control(
            row,
            max_virtual_forward_cmd=max_virtual_forward_cmd,
            max_virtual_turn_cmd=max_virtual_turn_cmd,
            lookahead_m=lookahead_m,
        )
        for row in target_rows
    ]


# ── 요약/출력 렌더링 / Summary & output rendering ──


def _summary_from_rows(selected_path: Path, package: dict[str, object], rows: Sequence[dict[str, object]]) -> dict[str, object]:
    """첫 행 기준 상위 요약(프리뷰 준비 여부 포함)을 만든다. ready_for_motor_test 는 항상 False.
    Build the top-level summary from the first row; ready_for_motor_test stays False."""
    first = rows[0] if rows else {}
    primitive_valid = bool(package["summary"]["primitive_sequence_valid"])  # type: ignore[index]
    target_distance = _optional_float(first.get("target_distance_m"))
    target_bearing = _optional_float(first.get("target_bearing_deg"))
    ready_preview = (
        primitive_valid
        and first.get("local_pose_available") is True
        and target_distance is not None
        and target_bearing is not None
        and first.get("motor_command_generated") is False
    )
    return {
        "selected_path_package": str(selected_path),
        "path_package_loaded": True,
        "local_pose_available": first.get("local_pose_available", False),
        "station_package_target_source": first.get("station_package_target_source", "path_package"),
        "firmware_active_target_source": first.get("firmware_active_target_source", "unknown"),
        "firmware_still_compile_time": first.get("firmware_still_compile_time", False),
        "active_primitive_index": first.get("active_primitive_index", "NA"),
        "active_tool_segment_id": first.get("active_tool_segment_id", "NA"),
        "target_distance_m": first.get("target_distance_m", "NA"),
        "target_bearing_deg": first.get("target_bearing_deg", "NA"),
        "cross_track_error_m": first.get("cross_track_error_m", "NA"),
        "current_heading_deg": first.get("current_heading_deg", "NA"),
        "virtual_heading_status": first.get("virtual_heading_status", "NA"),
        "virtual_forward_cmd": first.get("virtual_forward_cmd", 0.0),
        "virtual_turn_cmd": first.get("virtual_turn_cmd", 0.0),
        "virtual_left_cmd": first.get("virtual_left_cmd", 0.0),
        "virtual_right_cmd": first.get("virtual_right_cmd", 0.0),
        "virtual_control_generated": first.get("virtual_control_generated", False),
        "motor_command_generated": False,
        "physical_output_active": False,
        "ready_for_station_virtual_control_preview": ready_preview,
        "ready_for_motor_test": False,
        "next_action": "Virtual controller preview only; do not send these diagnostics to firmware.",
    }


def _write_csv(path: Path, rows: Sequence[dict[str, object]]) -> None:
    """가상 제어 행들을 ``VIRTUAL_FIELDS`` 스키마로 CSV 에 쓴다. 부수효과: 파일 쓰기.
    Write virtual-control rows to CSV using ``VIRTUAL_FIELDS``. Side effect: file write."""
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=VIRTUAL_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in VIRTUAL_FIELDS})


def _write_summary(path: Path, summary: dict[str, object]) -> None:
    """요약 딕셔너리를 Markdown 파일로 쓴다(진단 전용 고지 포함). 부수효과: 파일 쓰기.
    Write the summary dict as Markdown (with diagnostic-only notice). Side effect: file write."""
    lines = [
        "# Stage 14 Station Virtual Path Controller",
        "",
        "Station-side diagnostic preview only. Virtual values are not firmware motor commands.",
        "",
    ]
    for key, value in summary.items():
        lines.append(f"- {key}: `{value}`")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _plot_virtual_control(path: Path, rows: Sequence[dict[str, object]]) -> None:
    """행 인덱스에 따른 가상 forward/turn/left/right 명령을 PNG 그래프로 그린다. matplotlib 없으면 skip.
    Plot virtual forward/turn/left/right vs row index to a PNG. No-op sans matplotlib.

    부수효과: PNG 생성. Agg 백엔드 강제로 헤드리스 안전. / Side effect: writes PNG (Agg backend)."""
    try:
        import matplotlib

        matplotlib.use("Agg")  # 헤드리스 저장 전용 / headless, file-only
        import matplotlib.pyplot as plt
    except ImportError:
        return
    indices = [int(row.get("row_index", index)) for index, row in enumerate(rows)]
    forward = [float(row.get("virtual_forward_cmd", 0.0)) for row in rows]
    turn = [float(row.get("virtual_turn_cmd", 0.0)) for row in rows]
    left = [float(row.get("virtual_left_cmd", 0.0)) for row in rows]
    right = [float(row.get("virtual_right_cmd", 0.0)) for row in rows]
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(indices, forward, label="virtual_forward_cmd")
    ax.plot(indices, turn, label="virtual_turn_cmd")
    ax.plot(indices, left, label="virtual_left_cmd", linestyle="--")
    ax.plot(indices, right, label="virtual_right_cmd", linestyle="--")
    ax.axhline(0.0, color="0.4", linewidth=0.8)
    ax.set_title("Stage 14 Virtual Control Diagnostics")
    ax.set_xlabel("row_index")
    ax.set_ylabel("diagnostic command")
    ax.grid(True, linestyle="--", alpha=0.3)
    ax.legend()
    ax.text(0.01, 0.01, "No serial writes; no motor commands", transform=ax.transAxes, fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


# ── CLI 진입점 / CLI entry point ──


def _target_rows_for_args(args: argparse.Namespace, package: dict[str, object]) -> list[dict[str, object]]:
    """CLI 모드에 따라 Stage 12 타깃 행을 만들어 반환한다(가상 제어 계산의 입력).
    Produce Stage 12 target rows per CLI mode (input to virtual control).

    각 모드는 station_path_package_tracker 의 함수를 위임 호출한다: offline_pose 는
    수동 좌표, replay_log 는 로그 파일, live_usbdbg 는 실시간 포트. 필수 인자 누락 시
    ``SystemExit``. / Delegates to the tracker; ``SystemExit`` on missing required args."""
    if args.mode == "offline_pose":
        if args.current_x is None or args.current_y is None:
            raise SystemExit("--current-x and --current-y are required in offline_pose mode")
        return [
            station_path_package_tracker.compute_target_status(
                package,
                current_x=float(args.current_x),
                current_y=float(args.current_y),
                current_heading_deg=args.current_heading_deg,
                mode=args.mode,
                firmware_active_target_source="not_checked_offline_pose",
                local_pose_source="manual_local",
            )
        ]
    if args.mode == "replay_log":
        if not args.log:
            raise SystemExit("--log is required in replay_log mode")
        return station_path_package_tracker.build_rows_from_replay(
            package,
            Path(args.log).read_text(encoding="utf-8", errors="replace"),
            args.mode,
        )
    if not args.port:
        raise SystemExit("--port is required in live_usbdbg mode")
    # Stage 12 의 라이브 리더를 재사용(_ 접두 private). 시리얼 로직을 중복 구현하지 않기 위함.
    # Reuse Stage 12's live reader (private) to avoid duplicating serial logic.
    usbdbg_rows = station_path_package_tracker._read_live_usbdbg(args.port, args.duration_s)  # noqa: SLF001
    log_text = "\n".join(" ".join(f"{key}={value}" for key, value in row.items()) for row in usbdbg_rows)
    return station_path_package_tracker.build_rows_from_replay(package, log_text, args.mode)


def build_parser() -> argparse.ArgumentParser:
    """CLI 인자 파서를 만든다(모드/포즈/로그/포트 + 가상 명령 상한·lookahead + 출력 디렉터리).
    Build the argparse parser (mode, pose/log/port, virtual limits/lookahead, out dir)."""
    parser = argparse.ArgumentParser(description="Compute station-side virtual path control diagnostics.")
    parser.add_argument("--path-package", default="latest")
    parser.add_argument("--mode", choices=("offline_pose", "replay_log", "live_usbdbg"), required=True)
    parser.add_argument("--current-x", type=float)
    parser.add_argument("--current-y", type=float)
    parser.add_argument("--current-heading-deg", type=float)
    parser.add_argument("--log")
    parser.add_argument("--port")
    parser.add_argument("--duration-s", type=float, default=60.0)
    parser.add_argument("--max-virtual-forward-cmd", type=float, default=0.10)
    parser.add_argument("--max-virtual-turn-cmd", type=float, default=0.05)
    parser.add_argument("--lookahead-m", type=float, default=1.0)
    parser.add_argument("--out-dir", required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """타깃 행 → 가상 제어 행을 만들고 CSV/요약/그래프를 써서 프리뷰를 완성한다.
    Run the controller: build target→virtual rows, write CSV/summary/plot, print status.

    반환값: 정상 0, 경로 패키지 해석 실패 2. 부수효과: out-dir 생성 및 파일 3종 쓰기,
    stdout 출력. 모터로는 아무것도 전송하지 않는다(무모터/무전송 불변식).
    Returns 0 on success, 2 if the package can't be resolved. Side effects: creates
    out-dir, writes 3 files, prints. Nothing is ever sent to motors."""
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
    target_rows = _target_rows_for_args(args, package)
    virtual_rows = build_virtual_rows(
        target_rows,
        max_virtual_forward_cmd=args.max_virtual_forward_cmd,
        max_virtual_turn_cmd=args.max_virtual_turn_cmd,
        lookahead_m=args.lookahead_m,
    )
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / "virtual_control.csv"
    summary_path = out_dir / "summary.md"
    preview_path = out_dir / "preview_virtual_control.png"
    _write_csv(csv_path, virtual_rows)
    summary = _summary_from_rows(selected, package, virtual_rows)
    _write_summary(summary_path, summary)
    _plot_virtual_control(preview_path, virtual_rows)
    print("Stage 14 station virtual path controller preview complete.")
    print(f"selected_path_package={selected}")
    print(f"virtual_control_csv={csv_path}")
    print(f"summary_md={summary_path}")
    print(f"preview_virtual_control_png={preview_path if preview_path.exists() else 'not_generated'}")
    for key in (
        "virtual_control_generated",
        "virtual_heading_status",
        "virtual_forward_cmd",
        "virtual_turn_cmd",
        "ready_for_station_virtual_control_preview",
        "ready_for_motor_test",
        "motor_command_generated",
        "physical_output_active",
    ):
        print(f"{key}={summary[key]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
