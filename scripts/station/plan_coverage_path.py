"""스테이션측 커버리지 경로 드라이런 CLI / Station-side coverage-path dry-run CLI.

목적/역할:
    운용자가 지정한 두 위경도 점(A, B)과 레인 간격으로부터 사행(보스트로페돈, boustrophedon =
    왕복 지그재그) 커버리지 경로를 계획하고, 그 결과를 **미션 파일 3종(JSON/CSV/미리보기 PNG)** 으로
    저장한다. 이름 그대로 **드라이런**이다 — 시리얼 포트를 열지 않고, HC-12 프레임을 보내지 않으며,
    로버 명령을 일절 생성하지 않는다(이 불변식은 파일의 metadata·safety 블록과 테스트로 강제된다).

시스템 내 위치:
    - 라우팅 코어 `gps_coverage_core.planner`의 generate_corner_rectangle_path /
      generate_coverage_path / latlon_to_xy를 호출한다(실제 기하 계산은 거기서).
    - 출력은 outputs/missions/<mission_name>/ 아래로 떨어지며 이후 사람이 검토하거나 다른 도구가 읽는다.
    - `tests/test_station_plan_coverage_path.py`가 이 CLI의 출력 계약(파일 존재, JSON 키, CSV 행)을 잠근다.

핵심 개념·불변식 (두 계획 모드):
    - corner-rectangle(기본): A와 B를 **대각 코너**로 하는 직사각형을 훑는다. sweep_width는 무시되고
      (지정 시 경고), 커버리지 폭은 |B_y|(북 방향 성분)에서 나온다. 경로는 정확히 B에서 끝나도록 보정된다.
    - baseline-width: A→B를 레인 진행 **기준선**으로 삼고, 그에 수직으로 --sweep-width-m 만큼 레인을
      복제한다. 이 모드에서는 --sweep-width-m이 필수(validate_args가 강제).
    - 로컬 프레임: A=원점(0,0), x=East, y=North. JSON에 로컬 미터와 위경도를 함께 기록한다.
    - segment_type 규약: 짝수 order=lane_start, 홀수=lane_end(planner가 다른 값을 주면 그대로 존중).

사용법/진입점:
    CLI: ``python scripts/station/plan_coverage_path.py --point-a LAT,LON --point-b LAT,LON
    --lane-spacing-m N [--planner-mode ...] [--sweep-width-m N] [--speed-mps N]
    [--mission-name NAME] [--output-dir DIR]``. main(argv)가 진입점이며 종료 코드 int를 반환.

리팩토링 노트:
    출력 스키마 문자열("station_coverage_path.v1"), CSV 헤더(CSV_FIELDS), safety 문자열 목록,
    콘솔 key=value 출력은 모두 테스트가 검증한다 — 바꾸면 테스트도 함께 갱신할 것. 웨이포인트 dict의
    키 집합은 planner의 반환 계약과 결합되어 있다.

Purpose: from two operator lat/lon points (A, B) and a lane spacing, plan a boustrophedon (back-and-
forth) coverage path and write it as three mission files (JSON/CSV/preview PNG). It is a DRY-RUN: no
serial port is opened, no HC-12 frames are sent, and no rover commands are generated — an invariant
enforced by the metadata/safety blocks and by the tests. System: delegates the real geometry to the
routing core ``gps_coverage_core.planner``; outputs land under outputs/missions/<name>/; the contract
(files present, JSON keys, CSV rows) is locked by tests/test_station_plan_coverage_path.py. Two modes:
``corner-rectangle`` (default) sweeps the rectangle whose diagonal corners are A and B and ignores
sweep width; ``baseline-width`` treats A->B as the lane baseline and requires --sweep-width-m. Local
frame: A at origin, x=East, y=North. The schema string, CSV header, safety list and console key=value
output are all test-checked — update the tests if you change them.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import re
import sys
from pathlib import Path
from typing import Any

# 리포지토리 루트를 sys.path에 넣어 스크립트를 직접 실행해도 gps_coverage_core를 import할 수 있게 한다.
# / Put the repo root on sys.path so gps_coverage_core imports when this file is run as a script.
ROOT_DIR = Path(__file__).resolve().parents[2]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

import matplotlib

# 헤드리스(디스플레이 없는) 환경에서 미리보기 PNG를 저장하려면 Agg 백엔드를 import 전에 지정해야 한다.
# / Select the non-interactive Agg backend before pyplot import so preview PNGs save headlessly.
matplotlib.use("Agg")

import matplotlib.pyplot as plt

from gps_coverage_core.planner import (
    generate_corner_rectangle_path,
    generate_coverage_path,
    latlon_to_xy,
)

# CSV 출력 열 순서·이름. 테스트가 이 헤더를 확인하므로 임의로 바꾸지 말 것.
# / Column order/names for the CSV output; tests assert this header, so do not change casually.
CSV_FIELDS = (
    "index",
    "lat",
    "lon",
    "x_m",
    "y_m",
    "segment_type",
    "lane",
    "offset_m",
    "speed_mps",
    "notes",
)


# ── 인자 파싱·검증 / Argument parsing & validation ──
def parse_lat_lon(text: str) -> dict[str, float]:
    """"LAT,LON" 문자열을 검증해 {lat, lon} dict로 파싱. / Parse & validate a "LAT,LON" string into a {lat, lon} dict.

    argparse의 type 콜백으로 쓰인다. 형식·수치·범위(-90..90, -180..180) 위반 시 ArgumentTypeError.
    Used as an argparse ``type``; raises ArgumentTypeError on bad format, non-numeric, or out-of-range.
    """
    parts = [part.strip() for part in text.split(",")]
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("point must be LAT,LON")

    try:
        lat = float(parts[0])
        lon = float(parts[1])
    except ValueError as exc:
        raise argparse.ArgumentTypeError("point must contain numeric LAT,LON") from exc

    if not -90.0 <= lat <= 90.0:
        raise argparse.ArgumentTypeError("latitude must be between -90 and 90")
    if not -180.0 <= lon <= 180.0:
        raise argparse.ArgumentTypeError("longitude must be between -180 and 180")

    return {"lat": lat, "lon": lon}


def parse_mission_name(text: str) -> str:
    """미션 이름이 안전한 문자만 쓰는지 검증. / Validate that the mission name uses only safe characters.

    영숫자로 시작하고 점·밑줄·하이픈만 허용 — 경로 주입/이상한 디렉터리명을 막는다(디렉터리로 쓰이므로).
    Alnum start plus dot/underscore/hyphen only; guards against odd/injection directory names.
    """
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", text):
        raise argparse.ArgumentTypeError(
            "mission name must contain only letters, numbers, dot, underscore, or hyphen"
        )
    return text


def default_mission_name() -> str:
    """--mission-name 미지정 시 타임스탬프 기반 기본 이름 생성. / Default timestamped mission name when none is given."""
    return f"coverage_{dt.datetime.now(tz=dt.UTC):%Y%m%d_%H%M%S}"


def build_parser() -> argparse.ArgumentParser:
    """CLI 인자 파서를 구성해 반환. / Build and return the CLI argument parser.

    필수: --point-a, --point-b, --lane-spacing-m. 모드별 선택 인자는 validate_args가 추가 검증한다.
    Required: point-a/point-b/lane-spacing; mode-specific rules are enforced in ``validate_args``.
    """
    parser = argparse.ArgumentParser(
        description=(
            "Generate a station-side coverage path dry-run. "
            "This writes mission files only and sends no rover commands."
        )
    )
    parser.add_argument(
        "--point-a",
        type=parse_lat_lon,
        required=True,
        help="Start corner as LAT,LON.",
    )
    parser.add_argument(
        "--point-b",
        type=parse_lat_lon,
        required=True,
        help="Opposite/end corner as LAT,LON in the default corner-rectangle mode.",
    )
    parser.add_argument(
        "--planner-mode",
        choices=("corner-rectangle", "baseline-width"),
        default="corner-rectangle",
        help="Planning geometry. Default: corner-rectangle.",
    )
    parser.add_argument(
        "--sweep-width-m",
        type=float,
        default=None,
        help="Coverage width in meters. Required only for --planner-mode baseline-width.",
    )
    parser.add_argument(
        "--lane-spacing-m",
        type=float,
        required=True,
        help="Lane spacing / sweep interval in meters.",
    )
    parser.add_argument("--speed-mps", type=float, default=None, help="Optional dry-run speed metadata")
    parser.add_argument(
        "--mission-name",
        type=parse_mission_name,
        default=None,
        help="Mission directory name. Defaults to coverage_YYYYmmdd_HHMMSS.",
    )
    parser.add_argument(
        "--output-dir",
        default="outputs/missions",
        help="Base output directory. Default: outputs/missions",
    )
    return parser


def validate_args(args: argparse.Namespace) -> None:
    """모드 간 인자 정합성 검사(argparse만으로는 표현 불가). / Cross-argument checks argparse alone cannot express.

    baseline-width 모드는 --sweep-width-m이 필수(없으면 SystemExit). corner-rectangle 모드에서
    --sweep-width-m을 주면 무시된다는 경고만 낸다(에러 아님). / baseline-width requires sweep width;
    corner-rectangle merely warns that a supplied sweep width is ignored.
    """
    if args.planner_mode == "baseline-width" and args.sweep_width_m is None:
        raise SystemExit("--sweep-width-m is required when --planner-mode baseline-width")
    if args.planner_mode == "corner-rectangle" and args.sweep_width_m is not None:
        print("warning: --sweep-width-m is ignored in corner-rectangle mode", file=sys.stderr)


# ── 미션 기하·경계 계산 / Mission geometry & boundary ──
def _lane_length_m(point_a: dict[str, float], point_b: dict[str, float]) -> float:
    """A→B 직선 거리(레인 길이) 계산. / Straight-line A->B distance = lane length, in meters."""
    bx_m, by_m = latlon_to_xy(point_b["lat"], point_b["lon"], point_a["lat"], point_a["lon"])
    return (bx_m**2 + by_m**2) ** 0.5


def _coverage_boundary(waypoints: list[dict[str, float | int | str]]) -> list[tuple[float, float]]:
    """baseline-width 경로에서 커버리지 사각형 외곽선을 도출. / Derive the coverage polygon from a baseline-width path.

    첫 레인(lane 0)과 마지막 레인의 양 끝점을 코너로 잡아 닫힌 폴리곤(마지막 점=첫 점)을 만든다.
    사행 때문에 마지막 레인의 방향이 홀짝에 따라 뒤집히므로, max_lane이 홀수면 시작/끝을 바꿔 잡아야
    폴리곤이 자기교차(bow-tie) 없이 그려진다. / First and last lanes' endpoints form a closed polygon;
    because alternate lanes reverse, an odd last-lane index swaps start/end to avoid a bow-tie.
    """
    lane0 = [waypoint for waypoint in waypoints if int(waypoint["lane"]) == 0]
    max_lane = max(int(waypoint["lane"]) for waypoint in waypoints)
    lane_last = [waypoint for waypoint in waypoints if int(waypoint["lane"]) == max_lane]

    start0, end0 = lane0[0], lane0[1]
    # 홀수 인덱스 레인은 방향이 반전되어 있으니 코너 순서를 맞바꿔 폴리곤 교차를 방지.
    # / Odd-indexed lanes run reversed; swap the corners so the polygon does not self-cross.
    if max_lane % 2 == 0:
        start_last, end_last = lane_last[0], lane_last[1]
    else:
        end_last, start_last = lane_last[0], lane_last[1]

    return [
        (float(start0["x_m"]), float(start0["y_m"])),
        (float(end0["x_m"]), float(end0["y_m"])),
        (float(end_last["x_m"]), float(end_last["y_m"])),
        (float(start_last["x_m"]), float(start_last["y_m"])),
        (float(start0["x_m"]), float(start0["y_m"])),
    ]


def _corner_rectangle_boundary(point_b_local: dict[str, float]) -> list[tuple[float, float]]:
    """A=(0,0)와 B를 대각 코너로 하는 축정렬 직사각형 외곽선. / Axis-aligned rectangle with A=(0,0) and B as diagonal corners.

    B가 원점의 어느 사분면에 있어도 되도록 min/max로 축정렬 상자를 만든다(닫힌 폴리곤, 시계 반대 방향).
    Uses min/max so B may lie in any quadrant; returns a closed, axis-aligned polygon.
    """
    min_x = min(0.0, point_b_local["x_m"])
    max_x = max(0.0, point_b_local["x_m"])
    min_y = min(0.0, point_b_local["y_m"])
    max_y = max(0.0, point_b_local["y_m"])
    return [
        (min_x, min_y),
        (max_x, min_y),
        (max_x, max_y),
        (min_x, max_y),
        (min_x, min_y),
    ]


def _mission_waypoints(
    waypoints: list[dict[str, float | int | str]],
) -> list[dict[str, float | int | str]]:
    """planner 원시 웨이포인트를 미션 스키마 dict로 정규화. / Normalize raw planner waypoints into mission-schema dicts.

    order를 index로 옮기고, 필요한 필드를 float/int로 강제하며 누락 필드에 기본값을 채운다.
    speed_mps는 원본에 있을 때만 포함(선택 필드). / Coerces types, fills defaults; speed is optional.
    """
    mission_waypoints: list[dict[str, float | int | str]] = []
    for waypoint in waypoints:
        index = int(waypoint["order"])
        # planner가 segment_type을 주면 존중, 없으면 order 홀짝으로 lane_start/lane_end 기본 부여.
        # / Respect planner-provided segment_type; otherwise default by order parity (start/end).
        segment_type = str(
            waypoint.get("segment_type", "lane_start" if index % 2 == 0 else "lane_end")
        )
        mission_waypoint: dict[str, float | int | str] = {
            "index": index,
            "lat": float(waypoint["lat"]),
            "lon": float(waypoint["lon"]),
            "x_m": float(waypoint["x_m"]),
            "y_m": float(waypoint["y_m"]),
            "segment_type": segment_type,
            "lane": int(waypoint["lane"]),
            "offset_m": float(waypoint["offset_m"]),
            "notes": str(waypoint.get("notes", "")),
        }
        # 속도는 선택 필드 — 원본에 있을 때만 넣어 스키마에 불필요한 None을 남기지 않는다.
        # / Speed is optional; include it only when present so the schema has no spurious None.
        if "speed_mps" in waypoint:
            mission_waypoint["speed_mps"] = float(waypoint["speed_mps"])
        mission_waypoints.append(mission_waypoint)
    return mission_waypoints


def build_mission(
    *,
    mission_name: str,
    point_a: dict[str, float],
    point_b: dict[str, float],
    planner_mode: str,
    sweep_width_m: float | None,
    lane_spacing_m: float,
    speed_mps: float | None,
    waypoints: list[dict[str, float | int | str]],
) -> dict[str, Any]:
    """웨이포인트+입력을 직렬화 가능한 미션 dict로 조립. / Assemble waypoints + inputs into a serializable mission dict.

    메타데이터(드라이런 플래그 포함), 입력, 로컬 프레임 정보, 요약 통계, 커버리지 경계, 안전 문구,
    정규화된 웨이포인트를 담은 'station_coverage_path.v1' 스키마 dict를 반환한다. 경계·요약 필드는
    planner_mode에 따라 달라진다. 부수효과 없음(순수 조립). / Returns the schema dict; boundary and
    summary fields are mode-dependent. Pure assembly, no side effects.
    """
    mission_waypoints = _mission_waypoints(waypoints)
    lane_count = len({int(waypoint["lane"]) for waypoint in waypoints})
    # A를 원점으로 한 B의 로컬 좌표. 모든 로컬 미터 값의 기준이며 요약·경계 계산에 재사용.
    # / B in A-origin local frame; the basis for every local-meter value and the summary/boundary.
    point_b_x_m, point_b_y_m = latlon_to_xy(
        point_b["lat"], point_b["lon"], point_a["lat"], point_a["lon"]
    )
    point_b_local = {"x_m": point_b_x_m, "y_m": point_b_y_m}
    # 모드별로 경계·요약 통계가 갈린다: 직사각형은 축정렬 상자와 x/y 변 길이, baseline은 사행 폴리곤.
    # / Mode branch: rectangle uses an axis-aligned box + x/y extents; baseline uses the serpentine hull.
    if planner_mode == "corner-rectangle":
        boundary_points = _corner_rectangle_boundary(point_b_local)
        lane_length_m = abs(point_b_x_m)
        rectangle_x_extent_m = abs(point_b_x_m)
        rectangle_y_extent_m = abs(point_b_y_m)
    else:
        boundary_points = _coverage_boundary(waypoints)
        lane_length_m = _lane_length_m(point_a, point_b)
        # 직사각형 변 길이는 baseline 모드에서 의미가 없으므로 명시적으로 None(요약 스키마 유지).
        # / Rectangle extents are meaningless in baseline mode; set None to keep the summary schema stable.
        rectangle_x_extent_m = None
        rectangle_y_extent_m = None
    boundary = [{"x_m": x_m, "y_m": y_m} for x_m, y_m in boundary_points]

    return {
        "schema": "station_coverage_path.v1",
        "metadata": {
            "mission_name": mission_name,
            "generated_at": dt.datetime.now(tz=dt.UTC).isoformat(),
            # 드라이런 불변식을 파일에 명시적으로 못박는다(테스트가 이 두 값을 검증). / Pin the dry-run invariant in the file; tests assert both.
            "dry_run": True,
            "sends_rover_commands": False,
            "planner_mode": planner_mode,
        },
        "inputs": {
            "point_a": point_a,
            "point_b": point_b,
            "sweep_width_m": sweep_width_m,
            "lane_spacing_m": lane_spacing_m,
            "speed_mps": speed_mps,
        },
        "local_origin": {
            "lat": point_a["lat"],
            "lon": point_a["lon"],
            "description": "point_a",
        },
        "input_points_local": {
            # A는 로컬 원점 정의상 항상 (0,0). 테스트가 이 값을 확인한다. / A is always (0,0) by definition of the local origin; asserted by tests.
            "point_a": {"x_m": 0.0, "y_m": 0.0, "role": "start corner"},
            "point_b": {
                "x_m": point_b_x_m,
                "y_m": point_b_y_m,
                "role": "opposite/end corner",
            },
        },
        "local_frame": {
            "x_m": "east",
            "y_m": "north",
            "planner_mode": planner_mode,
            "corner_rectangle_rule": "point_a and point_b are opposite corners",
        },
        "summary": {
            "lane_count": lane_count,
            "waypoint_count": len(mission_waypoints),
            "lane_length_m": lane_length_m,
            "sweep_width_m": sweep_width_m,
            "rectangle_x_extent_m": rectangle_x_extent_m,
            "rectangle_y_extent_m": rectangle_y_extent_m,
        },
        "coverage_boundary": boundary,
        # 사람이 읽는 안전 보증 목록. 문구가 테스트에 하드코딩되어 있으니 그대로 유지.
        # / Human-readable safety assurances; the exact strings are hardcoded in the test, keep them.
        "safety": [
            "PC/Mac-side dry-run only",
            "no serial port opened",
            "no HC-12 frames sent",
            "no rover commands generated",
        ],
        "waypoints": mission_waypoints,
    }


# ── 출력 파일 쓰기 / Output writers ──
def save_mission_json(path: Path, mission: dict[str, Any]) -> None:
    """미션 dict를 들여쓰기 2칸 JSON으로 저장(끝에 개행). / Write the mission dict as 2-space-indented JSON with a trailing newline."""
    with path.open("w", encoding="utf-8") as handle:
        json.dump(mission, handle, indent=2)
        handle.write("\n")


def save_mission_csv(path: Path, waypoints: list[dict[str, float | int | str]]) -> None:
    """웨이포인트를 CSV_FIELDS 열 순서대로 CSV 저장. / Write waypoints to CSV using the fixed CSV_FIELDS column order.

    누락된 필드는 빈 문자열로 채운다(예: speed_mps 없을 때). / Missing fields become empty strings.
    """
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for waypoint in waypoints:
            writer.writerow({field: waypoint.get(field, "") for field in CSV_FIELDS})


def save_preview_png(path: Path, mission: dict[str, Any]) -> None:
    """미션 경로·경계·A/B 점을 미리보기 PNG로 렌더링. / Render the path, boundary and A/B points to a preview PNG.

    사행 경로, 커버리지 경계(점선), A(초록)·B(빨강) 마커, 웨이포인트 번호, 레인 간격/잔여폭 주석을
    담는다. 부수효과: path에 그림 저장 후 figure를 닫는다. / Draws the path/boundary/markers/labels and
    a spacing note; saves to ``path`` and closes the figure.
    """
    waypoints = mission["waypoints"]
    xs = [float(waypoint["x_m"]) for waypoint in waypoints]
    ys = [float(waypoint["y_m"]) for waypoint in waypoints]

    boundary = mission["coverage_boundary"]
    bx = [float(point["x_m"]) for point in boundary]
    by = [float(point["y_m"]) for point in boundary]

    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot(bx, by, linestyle="--", color="tab:gray", linewidth=1.2, label="coverage boundary")
    ax.plot(xs, ys, marker="o", linewidth=1.5, label="boustrophedon path")
    point_a_local = mission["input_points_local"]["point_a"]
    point_b_local = mission["input_points_local"]["point_b"]
    ax.scatter(
        [float(point_a_local["x_m"])],
        [float(point_a_local["y_m"])],
        color="tab:green",
        s=70,
        label="Point A start",
        zorder=5,
    )
    ax.scatter(
        [float(point_b_local["x_m"])],
        [float(point_b_local["y_m"])],
        color="tab:red",
        s=70,
        label="Point B final/end corner",
        zorder=5,
    )

    for waypoint in waypoints:
        ax.annotate(
            str(waypoint["index"]),
            (float(waypoint["x_m"]), float(waypoint["y_m"])),
            textcoords="offset points",
            xytext=(4, 4),
            fontsize=8,
        )

    lane_spacing_m = float(mission["inputs"]["lane_spacing_m"])
    # 커버리지 폭의 출처가 모드마다 다르다: 직사각형은 y 변 길이, baseline은 명시된 sweep_width.
    # / Where the coverage width comes from differs by mode: rectangle y-extent vs the given sweep width.
    if mission["metadata"]["planner_mode"] == "corner-rectangle":
        sweep_extent_m = float(mission["summary"]["rectangle_y_extent_m"])
    else:
        sweep_extent_m = float(mission["summary"]["sweep_width_m"])
    # 폭이 레인 간격으로 딱 나눠떨어지지 않을 때 남는 마지막 잔여 폭 — 운용자에게 커버리지 여백을 알린다.
    # / Leftover width when the extent isn't an exact multiple of spacing; flags the final coverage margin.
    residual_m = sweep_extent_m % lane_spacing_m
    if residual_m > 1e-6:
        spacing_note = f"lane spacing={lane_spacing_m:.2f} m, final residual={residual_m:.2f} m"
    else:
        spacing_note = f"lane spacing={lane_spacing_m:.2f} m"
    ax.text(
        0.02,
        0.02,
        spacing_note,
        transform=ax.transAxes,
        fontsize=9,
        bbox={"boxstyle": "round,pad=0.25", "facecolor": "white", "alpha": 0.75},
    )
    ax.set_title("Station Coverage Path Dry-Run")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


# ── 오케스트레이션 / Orchestration ──
def write_outputs(output_dir: Path, mission_name: str, mission: dict[str, Any]) -> dict[str, Path]:
    """미션 디렉터리를 만들고 JSON/CSV/PNG 3종을 저장. / Create the mission directory and write the JSON/CSV/PNG trio.

    반환: 종류별 파일 경로 dict("json"/"csv"/"preview"). 부수효과: 디렉터리 생성 및 파일 쓰기.
    Returns a dict of output paths; side effect: makes the directory and writes files.
    """
    mission_dir = output_dir / mission_name
    mission_dir.mkdir(parents=True, exist_ok=True)

    paths = {
        "json": mission_dir / "mission.json",
        "csv": mission_dir / "mission.csv",
        "preview": mission_dir / "preview.png",
    }
    save_mission_json(paths["json"], mission)
    save_mission_csv(paths["csv"], mission["waypoints"])
    save_preview_png(paths["preview"], mission)
    return paths


def main(argv: list[str] | None = None) -> int:
    """CLI 진입점: 파싱→검증→경로 계획→미션 조립→파일 저장→요약 출력. / CLI entry point: parse, validate, plan, assemble, write, report.

    argv=None이면 sys.argv를 쓴다(테스트는 리스트를 직접 넘김). 선택한 planner_mode에 따라 라우팅
    코어의 다른 생성기를 호출한다. 반환: 종료 코드(성공 0). 부수효과: 미션 파일 3종 기록·stdout 출력.
    Uses sys.argv when argv is None (tests pass a list). Dispatches to the routing core by mode.
    Returns an exit code (0 on success); writes the mission files and prints a key=value summary.
    """
    args = build_parser().parse_args(argv)
    validate_args(args)
    mission_name = args.mission_name or default_mission_name()

    # 선택한 모드에 맞는 라우팅 코어 생성기를 호출한다(기하 계산은 planner가 담당). / Dispatch to the mode's planner generator.
    if args.planner_mode == "corner-rectangle":
        raw_waypoints = generate_corner_rectangle_path(
            point_a=args.point_a,
            point_b=args.point_b,
            lane_spacing_m=args.lane_spacing_m,
            speed_mps=args.speed_mps,
        )
    else:
        raw_waypoints = generate_coverage_path(
            point_a=args.point_a,
            point_b=args.point_b,
            sweep_width_m=args.sweep_width_m,
            lane_spacing_m=args.lane_spacing_m,
            speed_mps=args.speed_mps,
        )
    mission = build_mission(
        mission_name=mission_name,
        point_a=args.point_a,
        point_b=args.point_b,
        planner_mode=args.planner_mode,
        sweep_width_m=args.sweep_width_m,
        lane_spacing_m=args.lane_spacing_m,
        speed_mps=args.speed_mps,
        waypoints=raw_waypoints,
    )
    paths = write_outputs(Path(args.output_dir), mission_name, mission)

    # 기계가 파싱하기 쉬운 key=value 요약. 상위 스크립트/도구가 이 출력을 읽으므로 형식을 유지.
    # / Machine-parseable key=value summary; wrapper scripts read this, so keep the format stable.
    print("dry_run=true")
    print("sends_rover_commands=false")
    print(f"mission_name={mission_name}")
    print(f"lane_count={mission['summary']['lane_count']}")
    print(f"waypoint_count={mission['summary']['waypoint_count']}")
    print(f"mission_json={paths['json'].resolve()}")
    print(f"mission_csv={paths['csv'].resolve()}")
    print(f"preview_png={paths['preview'].resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
