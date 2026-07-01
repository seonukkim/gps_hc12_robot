"""Stage 12 스테이션 측 경로 패키지 추적기 — 무(無)모터 타깃 프리뷰.
Stage 12 station-side path-package target tracker (no-motion preview).

목적/역할:
    ``path_package.json`` 과 로버의 로컬 포즈(직접 입력/리플레이 로그/라이브 USBDBG)를 받아,
    현재 위치에서 어떤 프리미티브의 어느 지점이 "다음 타깃"인지 계산해 CSV/요약/프리뷰 PNG 로
    낸다. **모터 명령을 절대 만들지 않는다** — 오직 스테이션 측 프리뷰/진단이다.
    Takes a path package + rover local pose and computes the next target
    (distance/bearing/cross-track/heading error) as CSV + summary + preview PNG.
    It never generates motor commands — station-side preview/diagnostics only.

시스템 내 위치:
    - 이 모듈이 ``parse_usbdbg_rows`` 의 **정본(owner)** 이다.
      ``tools/physical_path_planning/telemetry.py`` 가 이를 *재-export* 만 하고(복사 아님),
      controller/cli 계층은 ``telemetry.parse_usbdbg_rows`` 를 통해 이 함수를 쓴다. 큰 테스트
      표면(``tests/test_ppp_*``)이 이 함수에 의존하므로 시그니처/동작을 함부로 바꾸지 말 것.
    - ``tools/station_virtual_path_controller.py`` (Stage 14) 가 여기의
      ``compute_target_status`` / ``build_rows_from_replay`` / ``_read_live_usbdbg`` 를
      재사용해 그 위에 가상 제어 진단을 얹는다.
    - 경로 패키지 해석은 ``path_no_motion_validation.resolve_path_package`` 에 위임한다.
    - This module OWNS ``parse_usbdbg_rows``; ``physical_path_planning/telemetry``
      re-exports it (not a copy), and Stage 14's virtual controller reuses several
      functions here. Package resolution is delegated to ``path_no_motion_validation``.

핵심 개념·불변식:
    - **무모터 불변식**: 모든 출력 행에서 ``motor_command_generated`` / ``physical_output_active``
      는 언제나 False 다. 이 파일의 어떤 경로도 시리얼/HC-12/모터 프레임을 만들지 않는다.
    - "target" 은 물리 이동 목표가 아니라 프리뷰용 계산값이다 (함정: 실제 주행 명령 아님).
    - 로컬 포즈 소스 우선순위: 행에 local x/y 가 있으면 그대로, 없으면 lat/lon 을 패키지의
      georeference 로 로컬 좌표로 변환한다. georeference 가 없으면 진단 행으로 강등된다.
    - 헤딩이 없을 때는 ``heading_error_deg`` = ``"NA_DIAG_ONLY"`` 로 표기(진단 전용).
    - Invariants: motor fields are always False; "target" is a preview value, not a
      drive command; heading-less rows report ``NA_DIAG_ONLY``.

사용법/진입점:
    ``python tools/station_path_package_tracker.py --mode {offline_pose|replay_log|live_usbdbg} \\
        --out-dir OUT [...]`` — ``main()`` 이 진입점. 세 모드는 각각 수동 좌표, 로그 파일,
    실시간 USBDBG 스트림에서 행을 만든다.
    Entry point ``main()`` with three modes (offline_pose / replay_log / live_usbdbg).

리팩토링 노트:
    스칼라 변환 헬퍼(``_optional_float`` 등)는 여기와 ``telemetry.py`` 에 각각 존재한다;
    ``parse_usbdbg_rows`` 만은 반드시 단일 정본을 유지할 것(테스트가 identity 를 못박음).
    ``STATUS_FIELDS`` 순서는 CSV 스키마이자 하위 도구 계약이므로 재배열에 주의.
    Keep ``parse_usbdbg_rows`` single-sourced (a test pins the identity);
    ``STATUS_FIELDS`` order is the CSV schema/contract — reorder with care.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import re
import time
from pathlib import Path
from typing import Sequence

try:
    from tools import _bootstrap  # type: ignore  # noqa: F401
except ImportError:
    import _bootstrap  # type: ignore  # noqa: F401

from tools.path_no_motion_validation import PathPackageResolutionError, resolve_path_package


# ── CSV 스키마 / CSV schema ──
# station_target_status.csv 의 열 순서·이름을 정의하는 계약. 하위 도구가 이 순서에 의존한다.
# Column order/names for station_target_status.csv; downstream tools depend on it.
# NOTE: 맨 끝의 motor_command_generated / physical_output_active 는 항상 False(무모터 불변식).
STATUS_FIELDS = (
    "row_index",
    "mode",
    "station_package_target_source",
    "firmware_active_target_source",
    "firmware_still_compile_time",
    "local_pose_available",
    "local_pose_source",
    "reason",
    "current_x_m",
    "current_y_m",
    "current_heading_deg",
    "current_lat",
    "current_lon",
    "active_primitive_index",
    "active_tool_segment_id",
    "target_x_m",
    "target_y_m",
    "target_distance_m",
    "target_bearing_deg",
    "cross_track_error_m",
    "along_track_progress_m",
    "heading_error_deg",
    "tool_active_expected",
    "coverage_contributes",
    "motor_command_generated",
    "physical_output_active",
)


# ── 스칼라 변환 헬퍼 / Scalar coercion helpers ──


def _parse_bool(value: object) -> bool:
    """느슨한 불리언 파싱: 1/true/ok/yes/ready → True, 그 외 → False.
    Lenient boolean parse (1/true/ok/yes/ready are truthy)."""
    return str(value).strip().lower() in {"1", "true", "ok", "yes", "ready"}


def _optional_float(value: object) -> float | None:
    """유한 float 로 변환하되 빈 값/NA/NaN/변환 실패는 None 으로. / Parse to finite float or None."""
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


def _normalize_deg(angle_deg: float) -> float:
    """각도를 (-180, 180] 범위로 감싼다. / Wrap an angle into (-180, 180]."""
    return ((angle_deg + 180.0) % 360.0) - 180.0


# ── USBDBG 원시 파서 (정본) / Raw USBDBG parser (canonical owner) ──


def parse_usbdbg_rows(text: str) -> list[dict[str, str]]:
    """USBDBG 로그 텍스트를 줄 단위 ``key=value`` 딕셔너리 목록으로 파싱한다(키는 소문자화).
    Parse USBDBG log text into per-line ``key=value`` dicts (keys lower-cased).

    이 함수가 프로젝트 전체의 **단일 정본**이다: ``physical_path_planning/telemetry`` 는
    이 객체를 그대로 재-export 하며 복사하지 않는다. 값은 콤마/공백을 포함하지 않는 토큰만
    잡으므로(정규식), 자유 텍스트가 섞인 줄에서도 필드만 안전하게 추출한다. 빈 줄/매칭 없는
    줄은 건너뛴다.
    Single source of truth: ``telemetry`` re-exports this exact object (no copy).
    Only tokens without commas/whitespace are captured; empty/non-matching lines
    are skipped.
    """
    rows: list[dict[str, str]] = []
    for line in text.splitlines():
        values = {key.lower(): value for key, value in re.findall(r"([A-Za-z0-9_]+)=([^,\s]+)", line)}
        if values:
            rows.append(values)
    return rows


# ── 지오레퍼런싱 & 로컬 포즈 / Georeferencing & local pose ──


def _latest_firmware_target_source(rows: Sequence[dict[str, str]]) -> str:
    """가장 최근 행의 ``active_target_source`` 를 반환(없으면 "unknown"). 최신값 우선.
    Return the newest ``active_target_source`` (else "unknown")."""
    for row in reversed(rows):
        if "active_target_source" in row:
            return row["active_target_source"]
    return "unknown"


def _georeference(package: dict[str, object]) -> dict[str, float] | None:
    """패키지에서 georeference 파라미터를 여러 후보 컨테이너에서 관용적으로 추출한다.
    Extract georeference params from the package, tolerating alt key names/locations.

    origin_lat/lon, x축 방위, 위/경도당 미터가 모두 유한하게 모이면 그 딕셔너리를, 아니면
    None 을 반환한다. 키 이름 변형(예: origin_lat vs origin_lat_deg)을 흡수한다.
    Returns the dict only if all five fields are finite, else None."""
    containers = [
        package,
        package.get("georeference", {}),
        package.get("normalized_workspace", {}),
        package.get("summary", {}),
    ]
    for container in containers:
        if not isinstance(container, dict):
            continue
        origin_lat = _optional_float(container.get("origin_lat", container.get("origin_lat_deg")))
        origin_lon = _optional_float(container.get("origin_lon", container.get("origin_lon_deg")))
        bearing = _optional_float(container.get("x_axis_bearing_deg"))
        meters_per_lat = _optional_float(container.get("meters_per_lat", container.get("meters_per_deg_lat")))
        meters_per_lon = _optional_float(container.get("meters_per_lon", container.get("meters_per_deg_lon")))
        if None not in (origin_lat, origin_lon, bearing, meters_per_lat, meters_per_lon):
            return {
                "origin_lat": float(origin_lat),
                "origin_lon": float(origin_lon),
                "x_axis_bearing_deg": float(bearing),
                "meters_per_lat": float(meters_per_lat),
                "meters_per_lon": float(meters_per_lon),
            }
    return None


def lat_lon_to_local(package: dict[str, object], lat: float, lon: float) -> tuple[float, float] | None:
    """(lat, lon) 을 패키지 georeference 기준 로컬 (x, y) 미터로 변환. georef 없으면 None.
    Convert (lat, lon) to local (x, y) meters via the package georeference, else None.

    동/북 오프셋을 미터로 만든 뒤 x축 방위 ``theta`` 만큼 회전해 로버 로컬 프레임에 맞춘다.
    Builds east/north meters, then rotates by the x-axis bearing into the local frame."""
    georef = _georeference(package)
    if georef is None:
        return None
    east_m = (lon - georef["origin_lon"]) * georef["meters_per_lon"]
    north_m = (lat - georef["origin_lat"]) * georef["meters_per_lat"]
    theta = math.radians(georef["x_axis_bearing_deg"])
    local_x = east_m * math.sin(theta) + north_m * math.cos(theta)
    local_y = -east_m * math.cos(theta) + north_m * math.sin(theta)
    return local_x, local_y


def local_pose_from_row(package: dict[str, object], row: dict[str, str]) -> tuple[bool, str, str, float, float, float | None]:
    """한 USBDBG 행에서 로컬 포즈를 뽑는다: (사용가능?, 사유, 소스, x, y, heading).
    Derive local pose from one row: (available, reason, source, x, y, heading).

    우선순위: (1) 행에 로컬 x/y 가 있으면 그대로 사용, (2) 없으면 lat/lon 을 georeference
    로 변환, (3) 둘 다 불가하면 available=False 와 사유 코드를 돌려준다. heading 은 없을 수
    있어 Optional 이다 (함정: 헤딩 부재는 진단 전용 흐름으로 이어짐).
    Priority: local x/y in row → lat/lon via georeference → unavailable with a
    reason code. Heading may be None."""
    for x_key, y_key in (("current_x_m", "current_y_m"), ("x_m", "y_m")):
        if x_key in row and y_key in row:
            heading = _optional_float(row.get("current_heading_deg", row.get("heading_deg")))
            return True, "OK", "local_xy", float(row[x_key]), float(row[y_key]), heading
    lat = _optional_float(row.get("current_lat", row.get("lat")))
    lon = _optional_float(row.get("current_lon", row.get("lon")))
    if lat is not None and lon is not None:
        local = lat_lon_to_local(package, lat, lon)
        if local is None:
            return False, "NO_GEOREFERENCE_FOR_LAT_LON_TO_LOCAL", "none", 0.0, 0.0, None
        heading = _optional_float(row.get("current_heading_deg", row.get("heading_deg")))
        return True, "OK", "gps_georeference", local[0], local[1], heading
    return False, "NO_LOCAL_POSE_IN_ROW", "none", 0.0, 0.0, None


# ── 타깃 계산 / Target computation ──


def _project_to_move(row: dict[str, object], x: float, y: float) -> tuple[float, float, float]:
    """점 (x, y) 를 move 프리미티브 선분에 투영해 (t, cross_track, along) 을 반환한다.
    Project (x, y) onto a move segment; return (t, cross_track_m, along_track_m).

    ``t`` 는 [0,1] 로 클램프한 정규화 진행도, cross 는 수직 거리, along 은 시작점부터의
    진행 거리다. 길이가 0에 가까운 선분은 시작점까지의 거리로 안전하게 처리한다 (0-division 회피).
    ``t`` is clamped to [0,1]; degenerate (zero-length) segments fall back safely."""
    sx = float(row["start_x_m"])
    sy = float(row["start_y_m"])
    ex = float(row["end_x_m"])
    ey = float(row["end_y_m"])
    dx = ex - sx
    dy = ey - sy
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-12:
        return 0.0, math.hypot(x - sx, y - sy), 0.0
    raw_t = ((x - sx) * dx + (y - sy) * dy) / length_sq
    t = max(0.0, min(1.0, raw_t))
    proj_x = sx + t * dx
    proj_y = sy + t * dy
    return t, math.hypot(x - proj_x, y - proj_y), math.sqrt(length_sq) * t


def compute_target_status(
    package: dict[str, object],
    *,
    current_x: float,
    current_y: float,
    current_heading_deg: float | None,
    mode: str,
    row_index: int = 0,
    firmware_active_target_source: str = "unknown",
    current_lat: float | None = None,
    current_lon: float | None = None,
    local_pose_source: str = "manual_local",
) -> dict[str, object]:
    """현재 포즈에 대해 "다음 타깃" 프리미티브를 골라 프리뷰용 타깃 상태 행을 만든다.
    Pick the "next target" primitive for the current pose and build a status row.

    프리미티브 시퀀스에서 현재 위치와 가장 잘 맞는(cross-track 최소, 회전형은 소폭 패널티)
    프리미티브를 고르고, 그 프리미티브가 거의 끝났으면 다음으로 진행시킨 뒤, 타깃까지의
    거리/방위/cross-track/헤딩 오차를 계산한다. 헤딩이 None 이면 heading_error 는
    "NA_DIAG_ONLY".
    부수효과 없음. **무모터 불변식**: 반환 행의 motor/physical 필드는 항상 False.
    Chooses the best-matching primitive, advances if nearly complete, and computes
    target distance/bearing/cross-track/heading error. Pure; motor fields stay False.
    """
    primitives = list(package["primitive_sequence"])  # type: ignore[index]
    best_index = 0
    best_score = float("inf")
    best_t = 0.0
    best_cross_track = 0.0
    best_along = 0.0
    # 각 프리미티브에 대한 "적합도" 점수를 매겨 가장 낮은(가까운) 것을 현재 활성으로 본다.
    # Score each primitive; the lowest (closest) is treated as the active one.
    for index, primitive in enumerate(primitives):
        ptype = str(primitive["primitive_type"])
        if ptype.startswith("move"):
            t, cross, along = _project_to_move(primitive, current_x, current_y)
            score = cross
        else:
            # 회전 등 비-move 프리미티브: 시작점 거리 + 소폭 패널티(0.05)로 move 를 선호하게 함.
            # Non-move primitive: start-point distance + small bias so moves win ties.
            cross = math.hypot(current_x - float(primitive["start_x_m"]), current_y - float(primitive["start_y_m"]))
            along = 0.0
            t = 0.0
            score = cross + 0.05
        if score < best_score:
            best_index = index
            best_score = score
            best_t = t
            best_cross_track = cross
            best_along = along
    # 현재 프리미티브를 거의 다 지났으면(t≈1) 다음 프리미티브를 타깃으로 넘긴다.
    # If we are ~at the end of the best primitive, advance the target to the next one.
    if best_t >= 0.995 and best_index + 1 < len(primitives):
        best_index += 1
        best = primitives[best_index]
        if str(best["primitive_type"]).startswith("move"):
            best_t, best_cross_track, best_along = _project_to_move(best, current_x, current_y)
        else:
            best_t = 0.0
            best_cross_track = math.hypot(current_x - float(best["start_x_m"]), current_y - float(best["start_y_m"]))
            best_along = 0.0
    primitive = primitives[best_index]
    target_x = float(primitive["end_x_m"])
    target_y = float(primitive["end_y_m"])
    dx = target_x - current_x
    dy = target_y - current_y
    target_distance = math.hypot(dx, dy)
    target_bearing = math.degrees(math.atan2(dy, dx)) if target_distance > 1e-9 else (
        float(current_heading_deg) if current_heading_deg is not None else 0.0
    )
    heading_error: str | float
    if current_heading_deg is None:
        heading_error = "NA_DIAG_ONLY"
    else:
        heading_error = _normalize_deg(target_bearing - current_heading_deg)
    return {
        "row_index": row_index,
        "mode": mode,
        "station_package_target_source": "path_package",
        "firmware_active_target_source": firmware_active_target_source,
        "firmware_still_compile_time": firmware_active_target_source == "compile_time",
        "local_pose_available": True,
        "local_pose_source": local_pose_source,
        "reason": "OK",
        "current_x_m": current_x,
        "current_y_m": current_y,
        "current_heading_deg": "" if current_heading_deg is None else current_heading_deg,
        "current_lat": "" if current_lat is None else current_lat,
        "current_lon": "" if current_lon is None else current_lon,
        "active_primitive_index": primitive["primitive_index"],
        "active_tool_segment_id": primitive.get("associated_tool_segment_id", ""),
        "target_x_m": target_x,
        "target_y_m": target_y,
        "target_distance_m": target_distance,
        "target_bearing_deg": target_bearing,
        "cross_track_error_m": best_cross_track,
        "along_track_progress_m": best_along,
        "heading_error_deg": heading_error,
        "tool_active_expected": primitive["tool_active"],
        "coverage_contributes": primitive["coverage_contributes"],
        "motor_command_generated": False,
        "physical_output_active": False,
    }


def diagnostic_status(
    *,
    mode: str,
    row_index: int,
    reason: str,
    firmware_active_target_source: str = "unknown",
    current_lat: float | None = None,
    current_lon: float | None = None,
) -> dict[str, object]:
    """로컬 포즈가 없어 타깃을 계산할 수 없을 때의 진단 전용 상태 행을 만든다.
    Build a diagnostic-only status row when local pose is missing (target = NA).

    타깃 관련 필드는 모두 "NA", heading_error 는 "NA_DIAG_ONLY". ``reason`` 에 왜 진단
    상태인지(사유 코드)를 담는다. 무모터 불변식 유지(motor/physical 필드 False).
    All target fields are "NA"; ``reason`` carries the cause code. Motor fields False."""
    return {
        "row_index": row_index,
        "mode": mode,
        "station_package_target_source": "path_package",
        "firmware_active_target_source": firmware_active_target_source,
        "firmware_still_compile_time": firmware_active_target_source == "compile_time",
        "local_pose_available": False,
        "local_pose_source": "none",
        "reason": reason,
        "current_x_m": "",
        "current_y_m": "",
        "current_heading_deg": "",
        "current_lat": "" if current_lat is None else current_lat,
        "current_lon": "" if current_lon is None else current_lon,
        "active_primitive_index": "NA",
        "active_tool_segment_id": "NA",
        "target_x_m": "NA",
        "target_y_m": "NA",
        "target_distance_m": "NA",
        "target_bearing_deg": "NA",
        "cross_track_error_m": "NA",
        "along_track_progress_m": "NA",
        "heading_error_deg": "NA_DIAG_ONLY",
        "tool_active_expected": "NA",
        "coverage_contributes": "NA",
        "motor_command_generated": False,
        "physical_output_active": False,
    }


# ── 요약/출력 렌더링 / Summary & output rendering ──


def _summary_from_rows(package: dict[str, object], selected_path: Path, rows: Sequence[dict[str, object]]) -> dict[str, object]:
    """첫 행을 기준으로 상위 요약(readiness 포함)을 만든다. 프리뷰 준비 여부 판정 포함.
    Build the top-level summary from the first row, including readiness flags.

    ``ready_for_station_side_target_preview`` 는 프리미티브 유효 + 로컬 포즈 존재 +
    타깃 거리/방위 유한 + 모터명령 없음일 때만 True. ``ready_for_motor_test`` 는 항상 False
    (무모터 정책). / ``ready_for_motor_test`` is always False by policy."""
    first = rows[0] if rows else diagnostic_status(mode="unknown", row_index=0, reason="NO_ROWS")
    primitive_sequence_valid = bool(package["summary"]["primitive_sequence_valid"])  # type: ignore[index]
    target_distance = _optional_float(first.get("target_distance_m"))
    target_bearing = _optional_float(first.get("target_bearing_deg"))
    ready = (
        primitive_sequence_valid
        and bool(first["local_pose_available"])
        and target_distance is not None
        and target_bearing is not None
        and first["motor_command_generated"] is False
    )
    return {
        "selected_path_package": str(selected_path),
        "path_package_loaded": True,
        "station_package_target_source": "path_package",
        "firmware_active_target_source": first["firmware_active_target_source"],
        "firmware_still_compile_time": first["firmware_still_compile_time"],
        "primitive_sequence_valid": primitive_sequence_valid,
        "local_pose_available": first["local_pose_available"],
        "local_pose_source": first["local_pose_source"],
        "reason": first["reason"],
        "active_primitive_index": first["active_primitive_index"],
        "active_tool_segment_id": first["active_tool_segment_id"],
        "target_distance_m": first["target_distance_m"],
        "target_bearing_deg": first["target_bearing_deg"],
        "cross_track_error_m": first["cross_track_error_m"],
        "tool_active_expected": first["tool_active_expected"],
        "motor_command_generated": False,
        "physical_output_active": False,
        "ready_for_station_side_target_preview": ready,
        "ready_for_motor_test": False,
        "next_action": "Station-side target preview ready; motor tests remain prohibited."
        if ready
        else "Provide local pose or georeferenced A/B package before station-side target preview.",
    }


def _write_csv(path: Path, rows: Sequence[dict[str, object]]) -> None:
    """상태 행들을 ``STATUS_FIELDS`` 스키마로 CSV 파일에 쓴다. 부수효과: 파일 쓰기.
    Write status rows to CSV using the ``STATUS_FIELDS`` schema. Side effect: file write."""
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=STATUS_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in STATUS_FIELDS})


def _write_summary(path: Path, summary: dict[str, object]) -> None:
    """요약 딕셔너리를 사람이 읽는 Markdown 파일로 쓴다(무모터 고지 포함). 부수효과: 파일 쓰기.
    Write the summary dict as a human-readable Markdown file. Side effect: file write."""
    lines = [
        "# Stage 12 Station-Side Path Package Tracker",
        "",
        "Station-side preview/no-motion only. No serial writes, HC-12 frames, or motor commands are generated.",
        "",
    ]
    for key, value in summary.items():
        lines.append(f"- {key}: `{value}`")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _plot_current_target(path: Path, package: dict[str, object], status: dict[str, object]) -> None:
    """작업영역·툴 경로·현재/타깃 지점을 PNG 프리뷰로 렌더링한다. matplotlib 없으면 조용히 skip.
    Render a PNG preview (workspace + tool path + current/target). No-op sans matplotlib.

    부수효과: PNG 파일 생성. Agg 백엔드를 강제해 헤드리스 환경에서도 안전하게 그린다.
    Side effect: writes PNG; forces the Agg backend for headless safety."""
    try:
        import matplotlib

        matplotlib.use("Agg")  # 헤드리스: 디스플레이 없이 파일로만 저장 / headless: file only
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle
    except ImportError:
        return
    workspace = package["normalized_workspace"]  # type: ignore[index]
    tool_path = package["tool_path"]  # type: ignore[index]
    x_min = float(workspace["x_min_m"])  # type: ignore[index]
    x_max = float(workspace["x_max_m"])  # type: ignore[index]
    y_min = float(workspace["y_min_m"])  # type: ignore[index]
    y_max = float(workspace["y_max_m"])  # type: ignore[index]
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.add_patch(Rectangle((x_min, y_min), x_max - x_min, y_max - y_min, fill=False, edgecolor="black"))
    for row in tool_path:
        ax.plot(
            [float(row["tool_start_x_m"]), float(row["tool_end_x_m"])],
            [float(row["tool_start_y_m"]), float(row["tool_end_y_m"])],
            color="tab:orange" if row["tool_active"] else "0.55",
            linestyle="-" if row["tool_active"] else "--",
            linewidth=2.0 if row["tool_active"] else 1.0,
        )
    if status["local_pose_available"] is True:
        ax.scatter([float(status["current_x_m"])], [float(status["current_y_m"])], color="tab:blue", label="current")
        ax.scatter([float(status["target_x_m"])], [float(status["target_y_m"])], color="tab:red", label="target")
        ax.plot(
            [float(status["current_x_m"]), float(status["target_x_m"])],
            [float(status["current_y_m"]), float(status["target_y_m"])],
            color="tab:red",
            linestyle=":",
        )
    ax.set_title("Stage 12 Station Package Target Preview")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.axis("equal")
    ax.grid(True, linestyle="--", alpha=0.3)
    handles, labels = ax.get_legend_handles_labels()
    if labels:
        ax.legend(handles, labels, loc="best")
    ax.text(0.01, 0.01, "No motor commands generated", transform=ax.transAxes, fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def _read_live_usbdbg(port: str, duration_s: float) -> list[dict[str, str]]:
    """라이브 USBDBG 포트를 지정 시간 동안 읽어 파싱된 행 목록을 반환한다(읽기 전용).
    Read a live USBDBG port for ``duration_s`` and return parsed rows (read-only).

    부수효과: 시리얼 포트 오픈/읽기. pyserial 미설치 시 ``RuntimeError`` 로 승격.
    Side effect: opens/reads the serial port; raises ``RuntimeError`` without pyserial."""
    try:
        import serial  # type: ignore[import-not-found]
    except ImportError as exc:
        raise RuntimeError("pyserial is not available; cannot open live USBDBG") from exc
    lines: list[str] = []
    with serial.Serial(port, baudrate=115200, timeout=1.0) as handle:
        deadline = time.monotonic() + max(duration_s, 0.1)
        while time.monotonic() < deadline:
            raw = handle.readline()
            if raw:
                lines.append(raw.decode("utf-8", errors="replace").strip())
    return parse_usbdbg_rows("\n".join(lines))


def build_rows_from_replay(package: dict[str, object], log_text: str, mode: str) -> list[dict[str, object]]:
    """USBDBG 로그 텍스트로부터 행별 타깃/진단 상태 목록을 만든다(리플레이·라이브 공용).
    Build per-row target/diagnostic status from USBDBG log text (replay & live share this).

    각 행마다 로컬 포즈를 뽑아, 가능하면 ``compute_target_status``, 불가하면
    ``diagnostic_status`` 를 낸다. 로그가 비면 단일 진단 행(NO_USBDBG_ROWS)을 반환한다.
    Per row: compute target if pose available, else diagnostic; empty log → one
    diagnostic row (NO_USBDBG_ROWS)."""
    rows = parse_usbdbg_rows(log_text)
    firmware_source = _latest_firmware_target_source(rows)
    output: list[dict[str, object]] = []
    for index, row in enumerate(rows):
        local_available, reason, local_pose_source, x_m, y_m, heading = local_pose_from_row(package, row)
        lat = _optional_float(row.get("current_lat", row.get("lat")))
        lon = _optional_float(row.get("current_lon", row.get("lon")))
        firmware_for_row = row.get("active_target_source", firmware_source)
        if not local_available:
            output.append(
                diagnostic_status(
                    mode=mode,
                    row_index=index,
                    reason=reason,
                    firmware_active_target_source=firmware_for_row,
                    current_lat=lat,
                    current_lon=lon,
                )
            )
            continue
        output.append(
            compute_target_status(
                package,
                current_x=x_m,
                current_y=y_m,
                current_heading_deg=heading,
                mode=mode,
                row_index=index,
                firmware_active_target_source=firmware_for_row,
                current_lat=lat,
                current_lon=lon,
                local_pose_source=local_pose_source,
            )
        )
    if output:
        return output
    return [diagnostic_status(mode=mode, row_index=0, reason="NO_USBDBG_ROWS", firmware_active_target_source="unknown")]


# ── CLI 진입점 / CLI entry point ──


def build_parser() -> argparse.ArgumentParser:
    """CLI 인자 파서를 만든다(모드, 경로 패키지, 포즈/로그/포트, 출력 디렉터리).
    Build the argparse parser (mode, path package, pose/log/port, out dir)."""
    parser = argparse.ArgumentParser(description="Track station-side targets from path_package.json without motor output.")
    parser.add_argument("--path-package", default="latest")
    parser.add_argument("--mode", choices=("offline_pose", "replay_log", "live_usbdbg"), required=True)
    parser.add_argument("--current-x", type=float)
    parser.add_argument("--current-y", type=float)
    parser.add_argument("--current-heading-deg", type=float)
    parser.add_argument("--log")
    parser.add_argument("--port")
    parser.add_argument("--duration-s", type=float, default=60.0)
    parser.add_argument("--out-dir", required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """모드에 맞게 행을 만들고 CSV/요약/프리뷰를 써서 스테이션 타깃 프리뷰를 완성한다.
    Run the tracker: build rows per mode, write CSV/summary/preview, print status.

    반환값: 정상 0, 경로 패키지 해석 실패 2. 부수효과: out-dir 생성 및 파일 3종 쓰기,
    stdout 출력. 모터 명령은 어떤 경로로도 생성되지 않는다(무모터 불변식).
    Returns 0 on success, 2 if the path package can't be resolved. Side effects:
    creates out-dir, writes 3 files, prints. No motor commands are ever produced."""
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
    if args.mode == "offline_pose":
        if args.current_x is None or args.current_y is None:
            raise SystemExit("--current-x and --current-y are required in offline_pose mode")
        rows = [
            compute_target_status(
                package,
                current_x=float(args.current_x),
                current_y=float(args.current_y),
                current_heading_deg=args.current_heading_deg,
                mode=args.mode,
                firmware_active_target_source="not_checked_offline_pose",
                local_pose_source="manual_local",
            )
        ]
    elif args.mode == "replay_log":
        if not args.log:
            raise SystemExit("--log is required in replay_log mode")
        rows = build_rows_from_replay(package, Path(args.log).read_text(encoding="utf-8", errors="replace"), args.mode)
    else:
        if not args.port:
            raise SystemExit("--port is required in live_usbdbg mode")
        rows = build_rows_from_replay(package, "\n".join(" ".join(f"{k}={v}" for k, v in row.items()) for row in _read_live_usbdbg(args.port, args.duration_s)), args.mode)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / "station_target_status.csv"
    summary_path = out_dir / "summary.md"
    preview_path = out_dir / "preview_current_target.png"
    _write_csv(csv_path, rows)
    summary = _summary_from_rows(package, selected, rows)
    _write_summary(summary_path, summary)
    _plot_current_target(preview_path, package, rows[0])
    print("Stage 12 station-side path package tracker complete.")
    print(f"selected_path_package={selected}")
    print(f"station_target_status_csv={csv_path}")
    print(f"summary_md={summary_path}")
    print(f"preview_current_target_png={preview_path if preview_path.exists() else 'not_generated'}")
    for key in (
        "station_package_target_source",
        "firmware_active_target_source",
        "firmware_still_compile_time",
        "local_pose_available",
        "local_pose_source",
        "active_primitive_index",
        "target_distance_m",
        "target_bearing_deg",
        "ready_for_station_side_target_preview",
        "ready_for_motor_test",
        "motor_command_generated",
        "physical_output_active",
    ):
        print(f"{key}={summary[key]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
