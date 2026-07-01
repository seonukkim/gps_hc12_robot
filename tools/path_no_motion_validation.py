"""야외 경로 패키지의 무동작(no-motion) 검증기 — GPS/헤딩 준비도 게이트.

목적/역할:
    `field_ab_to_serpentine.py`가 만든 `path_package.json`을 읽어, 실제 로봇을 움직이기
    "전에" 야외에서 만족해야 할 조건들을 점검한다: GPS 품질(위성수/HDOP), 헤딩(코스) 준비,
    IMU/RC 상태, 그리고 각 프리미티브 목표점까지의 거리/방위(bearing) 미리보기. 결과를
    CSV/Markdown으로 남기고 `ready_for_outdoor_no_motion_validation` 플래그를 계산한다.
    이 도구는 **절대 모터를 돌리지 않는다**(motor_command_generated는 항상 False).

시스템 내 위치:
    - 상류: `tools/field_ab_to_serpentine.py`의 경로 패키지를 소비.
    - 동료 도구가 재사용: `PathPackageResolutionError`와 `resolve_path_package`는
      `tools/inspect_path_package.py`, `tools/physical_path_preview_from_package.py`가
      import 한다(패키지 탐색 로직의 단일 출처).
    - 세 가지 실행 모드: `package_check`(패키지만, 시리얼 없음),
      `live_serial`(OpenRB에서 상태 로그 수신), `auto`(포트·duration 유무로 자동 선택).

핵심 개념·불변식(invariant):
    - "무동작": 이 파일의 어떤 경로도 모터/물리 출력을 만들지 않는다. `physical_output_active`는
      입력 상태에서 읽기만 하며, True이면 준비 실패로 처리한다.
    - 준비도(readiness)는 AND 조합: GPS OK + 목표 거리/방위 존재 + 프리미티브 유효 +
      모터 미생성 + 물리출력 비활성 + (live_serial이면 시리얼 열림). 하나라도 어긋나면 False.
    - live_serial 안전장치: `active_target_source`가 compile_time/unknown이면 절대 통과시키지
      않는다(펌웨어 하드코딩 타깃으로 모터 시험 진입 방지).
    - 상태 파싱은 관대함(lenient): `key=value` 정규식 우선, 없으면 CSV 마지막 행. 누락값은
      None/기본값. 이는 다양한 펌웨어 로그 포맷을 견디기 위함.

사용법/진입점:
    CLI. `main()`이 진입점. 예:
    `python tools/path_no_motion_validation.py --path-package latest --mode package_check`.
    라이브: `--port <dev> --duration-s <초>`.

리팩토링 노트:
    - CSV 필드(CONCISE_CSV_FIELDS/VERBOSE_EXTRA_FIELDS) 순서는 산출물 계약.
    - `no_motion_gps_mode`의 별칭 `course_required`는 내부적으로 `require_course`로 접힌다.
    - 게이트 로직은 `build_readiness`에 집중; 새 안전 조건은 여기서 AND에 추가하라.

Outdoor no-motion validator for a serpentine path package. Reads the
`path_package.json` produced upstream and checks the conditions that must hold
outdoors BEFORE any real motion: GPS quality (sats/HDOP), heading/course
readiness, IMU/RC status, and a per-primitive target distance/bearing preview.
Emits CSV/Markdown and computes `ready_for_outdoor_no_motion_validation`.
Never generates motor commands. `resolve_path_package` /
`PathPackageResolutionError` here are the shared package-discovery logic reused
by the other tools/ path scripts. Modes: package_check / live_serial / auto.
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


# ── CSV 필드 계약 / CSV field contracts (concise + verbose extras) ──
# 기본(concise) CSV 컬럼 순서 = 산출물 계약. 순서/이름 변경 시 소비자 확인 필요.
# Default (concise) CSV column order is an output contract; check consumers before changing.
CONCISE_CSV_FIELDS = (
    "validation_mode",
    "serial_opened",
    "selected_path_package",
    "path_package_loaded",
    "gps_status",
    "gps_sats",
    "gps_hdop",
    "position_source",
    "current_lat",
    "current_lon",
    "current_x_m",
    "current_y_m",
    "imu_status",
    "imu_type",
    "imu_chip_id",
    "imu_data_plausible",
    "rc_status",
    "heading_status",
    "active_target_source",
    "live_path_package_connected",
    "active_primitive_index",
    "target_distance_m",
    "target_bearing_deg",
    "cross_track_error_m",
    "heading_error_deg",
    "motor_command_generated",
    "physical_output_active",
    "ready_for_outdoor_no_motion_validation",
    "reason",
    "next_action",
)

# --verbose일 때 concise 뒤에 덧붙는 추가 컬럼 / extra columns appended in verbose mode.
VERBOSE_EXTRA_FIELDS = (
    "current_heading",
    "target_point",
    "along_track_progress_m",
    "tool_active_expected",
)


# ── 예외 / Exception (shared with sibling tools) ──
class PathPackageResolutionError(Exception):
    """경로 패키지를 찾지 못했을 때의 예외. 시도한 값과 후보 목록을 함께 담는다.

    무엇을/왜: `resolve_path_package`가 실패하면 이 예외로 provided(요청 문자열)와
    candidates(가까운 후보 Path들)를 전달해, 호출측이 친절한 진단을 출력하게 한다.
    형제 도구(inspect/physical_path_preview)도 이 예외를 import 해 동일 처리한다.

    Raised when a path package cannot be resolved; carries the provided value and
    nearby candidate paths so callers can print a helpful diagnostic. Shared with
    the sibling tools.
    """

    def __init__(self, message: str, *, provided: str, candidates: Sequence[Path]) -> None:
        super().__init__(message)
        self.provided = provided
        self.candidates = list(candidates)


# ── 관대한 파싱 유틸 / Lenient parse helpers (tolerant of varied log formats) ──
def _parse_bool(value: object, default: bool = False) -> bool:
    """느슨한 불리언 파싱: 참으로 볼 토큰 집합에 들면 True(없으면 default).

    Lenient boolean: True if the token is in the truthy set, else default.
    """
    if value is None:
        return default
    return str(value).strip().lower() in {"1", "true", "ok", "yes", "manual", "ready"}


def _parse_optional_float(value: object) -> float | None:
    """float로 파싱하되 빈값/NA/NaN/비유한수는 None으로 / Parse float, mapping blanks/NA/NaN/inf to None."""
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


def _parse_optional_int(value: object) -> int | None:
    """float 파싱 후 정수로 절단(파싱 실패 시 None) / Parse as float then truncate to int, else None."""
    parsed = _parse_optional_float(value)
    return int(parsed) if parsed is not None else None


def _format_optional(value: object) -> object:
    """None을 CSV용 빈 문자열로 치환 / Map None to an empty string for CSV output."""
    return "" if value is None else value


def _normalize_deg(angle_deg: float) -> float:
    """각도를 (-180, 180] 범위로 접어 반환 / Wrap an angle into (-180, 180] degrees."""
    return ((angle_deg + 180.0) % 360.0) - 180.0


# ── 패키지 탐색·해석 / Package discovery & resolution (shared entry points) ──
def _dedupe_existing(paths: Sequence[Path]) -> list[Path]:
    """존재하는 경로만 남기고 resolve() 기준 중복 제거(입력 순서 유지).

    Keep only existing paths, de-duplicated by resolved path, preserving order.
    """
    seen: set[Path] = set()
    result: list[Path] = []
    for path in paths:
        if not path.exists():
            continue
        resolved = path.resolve()
        if resolved not in seen:
            seen.add(resolved)
            result.append(path)
    return result


def discover_path_package_candidates(base_dir: Path | None = None) -> list[Path]:
    """알려진 위치들에서 path_package.json 후보를 최신순으로 모아 반환.

    무엇을/왜: 관례적 경로(outputs/field_ab_serpentine/latest/...)를 최우선으로 두고,
    field_runs와 outputs 전체를 수정시각 내림차순으로 훑어 후보를 만든다. 존재·중복은
    _dedupe_existing로 정리한다. 'latest' 해석과 오류 진단에 함께 쓰인다.

    Collect path_package.json candidates from known locations, newest first
    (canonical latest path preferred), de-duplicated to existing files.
    """
    base = base_dir or Path.cwd()
    candidates: list[Path] = []
    candidates.append(base / "outputs/field_ab_serpentine/latest/path_package.json")
    candidates.extend(
        sorted(
            (base / "outputs/field_runs").glob("*/field_ab_serpentine/path_package.json"),
            key=lambda path: path.stat().st_mtime if path.exists() else 0.0,
            reverse=True,
        )
    )
    outputs_dir = base / "outputs"
    if outputs_dir.exists():
        candidates.extend(
            sorted(
                outputs_dir.rglob("path_package.json"),
                key=lambda path: path.stat().st_mtime if path.exists() else 0.0,
                reverse=True,
            )
        )
    return _dedupe_existing(candidates)


def resolve_path_package(path_arg: str, base_dir: Path | None = None) -> Path:
    """--path-package 인자를 실제 파일 경로로 해석(형제 도구가 공유하는 진입점).

    무엇을/왜: "latest"면 최신 후보를 반환하고, 명시 경로면 상대경로를 base 기준으로
    붙여 존재하면 반환한다. 실패 시 후보 목록을 담은 PathPackageResolutionError.
    반환: 존재하는 path_package.json의 Path.

    Resolve the --path-package argument to a concrete file. "latest" picks the
    newest candidate; an explicit path is resolved against base. Raises
    PathPackageResolutionError (with candidates) if nothing matches.
    """
    base = base_dir or Path.cwd()
    candidates = discover_path_package_candidates(base)
    if path_arg == "latest":
        if candidates:
            return candidates[0]
        raise PathPackageResolutionError(
            "No path_package.json found. Run tools/field_ab_to_serpentine.py first.",
            provided=path_arg,
            candidates=[],
        )
    provided = Path(path_arg).expanduser()
    if not provided.is_absolute():
        provided = base / provided
    if provided.exists():
        return provided
    raise PathPackageResolutionError(
        "Provided path_package.json does not exist.",
        provided=path_arg,
        candidates=candidates[:5],
    )


# ── 상태 수집 / Status ingestion (log parsing, package-preview, live serial) ──
def parse_status_log(text: str) -> dict[str, object]:
    """펌웨어 상태 텍스트/CSV를 표준 status dict로 파싱(관대하게).

    무엇을/왜: 여러 포맷의 로그에서 GPS/IMU/RC/헤딩/위치 등 필드를 추출한다. 먼저
    `key=value` 토큰을 정규식으로 긁고, 하나도 없으면 CSV의 마지막 행을 사용한다.
    누락 필드는 안전 기본값(False/None/0.0)으로 채워 준비도 계산이 견고하게 동작하게 한다.
    반환: 하류(_gps_status/_heading_status/build_target_rows)가 소비하는 status dict.
    함정: 별칭 키가 많다(gps_ok|gps_fix, bmi160_ok|imu_ok 등). 새 필드 추가 시 별칭 고려.

    Parse firmware status text/CSV into a normalised status dict (lenient):
    prefer `key=value` tokens, fall back to the last CSV row, and fill missing
    fields with safe defaults so readiness computation stays robust.
    """
    status: dict[str, object] = {
        "gps_ok": False,
        "bmi160_ok": False,
        "rc_manual_ok": False,
        "current_pose_known": False,
        "position_source": "unknown",
        "gps_sats": None,
        "gps_hdop": None,
        "current_lat": None,
        "current_lon": None,
        "x_m": 0.0,
        "y_m": 0.0,
        "heading_deg": None,
        "heading_ready": False,
        "gps_course_deg": None,
        "course_displacement_m": None,
        "gps_course_min_displacement_m": None,
        "imu_heading_diag": None,
        "imu_type": "",
        "imu_chip_id": "",
        "imu_data_plausible": False,
        "physical_output_active": False,
        "active_target_source": "unknown",
    }
    # 1차: 모든 줄에서 key=value 토큰을 수집(마지막 값이 이김) / primary: gather key=value tokens.
    key_values: dict[str, str] = {}
    for line in text.splitlines():
        for key, value in re.findall(r"([A-Za-z0-9_]+)=([^,\s]+)", line):
            key_values[key.lower()] = value
    # 2차 대안: key=value가 전혀 없으면 CSV로 보고 마지막(가장 최근) 행을 사용.
    # Fallback: if no key=value pairs, treat as CSV and use the last (latest) row.
    if not key_values:
        try:
            rows = list(csv.DictReader(text.splitlines()))
        except csv.Error:
            rows = []
        if rows:
            key_values.update({key.lower(): value for key, value in rows[-1].items() if value is not None})

    status["gps_ok"] = _parse_bool(key_values.get("gps_ok", key_values.get("gps_fix")))
    status["bmi160_ok"] = _parse_bool(key_values.get("bmi160_ok", key_values.get("imu_ok")))
    status["rc_manual_ok"] = _parse_bool(key_values.get("rc_manual_ok", key_values.get("rc_ok")))
    status["imu_data_plausible"] = _parse_bool(key_values.get("imu_data_plausible", key_values.get("imu_ok")))
    status["imu_type"] = key_values.get("imu_type", "")
    status["imu_chip_id"] = key_values.get("imu_chip_id", "")
    if "position_source" in key_values:
        status["position_source"] = key_values["position_source"].lower()
    elif bool(status["gps_ok"]):
        status["position_source"] = "gps"

    status["gps_sats"] = _parse_optional_int(key_values.get("gps_sats", key_values.get("sats")))
    status["gps_hdop"] = _parse_optional_float(key_values.get("gps_hdop", key_values.get("hdop")))
    status["current_lat"] = _parse_optional_float(key_values.get("current_lat", key_values.get("lat")))
    status["current_lon"] = _parse_optional_float(key_values.get("current_lon", key_values.get("lon")))

    # 로컬 미터 좌표가 있으면 그것으로 포즈 확정(두 가지 키 이름 지원).
    # If local-metre coords exist, mark the pose known (accepts two key spellings).
    for x_key, y_key in (("x_m", "y_m"), ("current_x_m", "current_y_m")):
        if x_key in key_values and y_key in key_values:
            status["x_m"] = float(key_values[x_key])
            status["y_m"] = float(key_values[y_key])
            status["current_pose_known"] = True
            break
    # 위경도만 있어도 포즈는 알려진 것으로 간주(로컬 좌표는 별도 변환 필요).
    # Lat/lon alone also counts as a known pose (local conversion handled elsewhere).
    if status["current_lat"] is not None and status["current_lon"] is not None:
        status["current_pose_known"] = True

    if "heading_deg" in key_values:
        status["heading_deg"] = _parse_optional_float(key_values["heading_deg"])
    elif "current_heading_deg" in key_values:
        status["heading_deg"] = _parse_optional_float(key_values["current_heading_deg"])
    status["heading_ready"] = _parse_bool(
        key_values.get("heading_ready", key_values.get("gps_course_output_valid"))
    )
    status["gps_course_deg"] = _parse_optional_float(
        key_values.get("gps_course_deg", key_values.get("gps_course_output_deg"))
    )
    status["course_displacement_m"] = _parse_optional_float(key_values.get("course_displacement_m"))
    status["gps_course_min_displacement_m"] = _parse_optional_float(key_values.get("gps_course_min_displacement_m"))
    status["imu_heading_diag"] = _parse_optional_float(
        key_values.get("imu_heading_diag", key_values.get("imu_relative_yaw_deg", key_values.get("yaw_deg")))
    )
    status["physical_output_active"] = _parse_bool(key_values.get("physical_output_active"))
    status["active_target_source"] = key_values.get("active_target_source", "unknown")
    return status


def package_preview_status(package: dict[str, object]) -> dict[str, object]:
    """시리얼 없이 패키지만으로 미리보기용 합성 status를 만든다(첫 프리미티브 기준).

    무엇을/왜: `package_check` 모드에서 실제 센서 없이도 목표 거리/방위를 계산하려면
    현재 포즈가 필요하다. 여기서는 첫 프리미티브의 시작점/헤딩을 "현재 위치"로 삼고
    position_source="package_preview"로 표시한다. GPS/IMU/RC는 모두 미검(False).
    반환: parse_status_log와 동일한 형식의 status dict.

    Build a synthetic preview status from the package alone (no serial), using
    the first primitive's start pose as the "current" pose. Marks the source as
    package_preview; GPS/IMU/RC left unchecked.
    """
    primitives = package["primitive_sequence"]  # type: ignore[index]
    first = primitives[0] if primitives else {"start_x_m": 0.0, "start_y_m": 0.0, "start_heading_deg": 0.0}
    return {
        "gps_ok": False,
        "bmi160_ok": False,
        "rc_manual_ok": False,
        "current_pose_known": True,
        "position_source": "package_preview",
        "gps_sats": None,
        "gps_hdop": None,
        "current_lat": None,
        "current_lon": None,
        "x_m": float(first["start_x_m"]),
        "y_m": float(first["start_y_m"]),
        "heading_deg": float(first["start_heading_deg"]),
        "heading_ready": False,
        "gps_course_deg": None,
        "course_displacement_m": None,
        "gps_course_min_displacement_m": None,
        "imu_heading_diag": float(first["start_heading_deg"]),
        "imu_type": "",
        "imu_chip_id": "",
        "imu_data_plausible": False,
        "physical_output_active": False,
        "active_target_source": "package_preview",
    }


def read_live_serial_status(port: str, duration_s: float, baud: int = 115200) -> dict[str, object]:
    """지정 시리얼 포트를 duration_s 동안 읽어 상태 로그를 수집·파싱한다.

    무엇을/왜: `live_serial` 모드에서 OpenRB 등으로부터 상태 라인을 수신해
    parse_status_log로 넘긴다. 이 함수는 **읽기 전용**이며 어떤 명령도 쓰지 않는다.
    부수효과: 시리얼 포트를 연다(읽기만). pyserial 미설치/포트 오류 시 RuntimeError.
    반환: parse_status_log 결과 status dict.

    Read the given serial port for duration_s and parse the collected lines.
    Read-only: opens the port but never writes commands. Raises RuntimeError if
    pyserial is missing or the port cannot be opened/read.
    """
    try:
        import serial  # type: ignore[import-not-found]
    except ImportError as exc:
        raise RuntimeError("pyserial is not available; cannot open live serial") from exc
    try:
        with serial.Serial(port, baudrate=baud, timeout=1.0) as handle:
            lines: list[str] = []
            deadline = time.monotonic() + max(duration_s, 0.1)
            while time.monotonic() < deadline:
                raw = handle.readline()
                if raw:
                    lines.append(raw.decode("utf-8", errors="replace").strip())
    except Exception as exc:  # pragma: no cover - exact exception varies by platform.
        raise RuntimeError(f"could not open/read serial port {port}: {exc}") from exc
    return parse_status_log("\n".join(lines))


# ── 상태 판정 / Status classifiers (GPS, heading readiness) ──
def _gps_status(
    status: dict[str, object],
    *,
    validation_mode: str,
    min_sats: int,
    max_hdop: float,
    no_motion_gps_mode: str,
) -> str:
    """GPS 준비 상태를 문자열로 판정: OK/WAIT/FAIL/NOT_CHECKED_PACKAGE_ONLY.

    무엇을/왜: 모드에 따라 규칙이 다르다. package_preview면 검사 생략, require_course면
    코스가 있어야 OK, 그 외에는 위성수>=min_sats & HDOP<=max_hdop면 OK(부족하면 WAIT,
    소스가 gps가 아니거나 포즈 미상이면 FAIL). 준비도 AND 게이트의 핵심 입력.

    Classify GPS readiness as OK/WAIT/FAIL/NOT_CHECKED_PACKAGE_ONLY. Rules vary
    by mode (package preview skipped; require_course needs a course; otherwise
    sats>=min_sats and HDOP<=max_hdop).
    """
    if validation_mode == "package_check" and status.get("position_source") == "package_preview":
        return "NOT_CHECKED_PACKAGE_ONLY"
    if no_motion_gps_mode == "require_course":
        if not bool(status["gps_ok"]) or status.get("gps_course_deg") is None:
            return "FAIL"
        return "OK"
    if status.get("position_source") != "gps" or not bool(status["current_pose_known"]):
        return "FAIL"
    sats = status.get("gps_sats")
    hdop = status.get("gps_hdop")
    if not isinstance(sats, int) or not isinstance(hdop, float):
        return "WAIT"
    if sats >= min_sats and hdop <= max_hdop:
        return "OK"
    return "WAIT"


def _status_label(ok: object) -> str:
    """불리언을 OK/WAIT 라벨로 / Map a boolean to an OK/WAIT label."""
    return "OK" if bool(ok) else "WAIT"


def _heading_status(status: dict[str, object], no_motion_gps_mode: str) -> str:
    """헤딩(코스) 준비 상태를 라벨로 판정.

    무엇을/왜: require_course면 heading_ready에 따라 OK/FAIL. 아니면 GPS 코스가 없을 때
    "움직여야(또는 진단만) 코스가 나온다"는 WAITING_... 라벨, 코스가 있으면 DIAG_ONLY.
    GPS 코스는 정지 상태에서 산출되지 않으므로 무동작 검증에서는 진단 목적에 가깝다.

    Classify heading/course readiness. require_course -> OK/FAIL by heading_ready;
    otherwise waiting-for-motion vs DIAG_ONLY (course needs motion to be valid).
    """
    if no_motion_gps_mode == "require_course":
        return "OK" if bool(status.get("heading_ready")) else "FAIL"
    if status.get("gps_course_deg") is None:
        displacement = status.get("course_displacement_m")
        minimum = status.get("gps_course_min_displacement_m")
        if isinstance(displacement, float) and isinstance(minimum, float) and displacement < minimum:
            return "WAITING_FOR_MOTION_OR_DIAG_ONLY"
        return "WAITING_FOR_MOTION_OR_DIAG_ONLY"
    return "DIAG_ONLY"


# ── 목표 미리보기 행 / Per-primitive target preview rows ──
def build_target_rows(
    package: dict[str, object],
    status: dict[str, object],
    ready: dict[str, object],
    *,
    concise: bool,
) -> list[dict[str, object]]:
    """프리미티브를 순회하며 각 목표점까지의 거리/방위/헤딩오차 미리보기 행을 만든다.

    무엇을/왜: 현재 포즈에서 시작해 각 프리미티브의 종점(end_x/y)을 차례로 "다음 목표"로
    삼아 거리·방위·(가능하면)헤딩오차와 누적 진행거리를 계산한다. 매 행마다 현재 포즈를
    그 종점/종점헤딩으로 갱신하며 시퀀스를 시뮬레이션한다(모터는 돌리지 않음).
    핵심 인자: ready(각 행에 복사될 준비도 필드), concise(추가 컬럼 포함 여부).
    반환: CSV로 쓰일 행 dict 리스트. 모든 행의 motor_command_generated는 False.
    함정: 헤딩을 모르면(heading_available False) heading_error는 "NA_DIAG_ONLY".

    Walk the primitives, treating each primitive's end point as the next target,
    and compute distance/bearing/heading-error plus cumulative progress, updating
    the simulated pose each step. Motor never engaged; motor flag always False.
    """
    rows: list[dict[str, object]] = []
    current_x = float(status["x_m"])
    current_y = float(status["y_m"])
    current_heading = status.get("heading_deg")
    heading_available = isinstance(current_heading, float)
    progress = 0.0
    for primitive in package["primitive_sequence"]:  # type: ignore[index]
        target_x = float(primitive["end_x_m"])
        target_y = float(primitive["end_y_m"])
        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.hypot(dx, dy)
        bearing = math.degrees(math.atan2(dy, dx)) if distance > 1e-9 else (
            float(current_heading) if heading_available else 0.0
        )
        progress += distance
        heading_error = (
            f"{_normalize_deg(bearing - float(current_heading)):.3f}" if heading_available else "NA_DIAG_ONLY"
        )
        row = {
            "validation_mode": ready["validation_mode"],
            "serial_opened": ready["serial_opened"],
            "selected_path_package": ready["selected_path_package"],
            "path_package_loaded": ready["path_package_loaded"],
            "gps_status": ready["gps_status"],
            "gps_sats": _format_optional(status.get("gps_sats")),
            "gps_hdop": _format_optional(status.get("gps_hdop")),
            "position_source": status.get("position_source", "unknown"),
            "current_lat": _format_optional(status.get("current_lat")),
            "current_lon": _format_optional(status.get("current_lon")),
            "current_x_m": f"{current_x:.3f}",
            "current_y_m": f"{current_y:.3f}",
            "imu_status": ready["imu_status"],
            "imu_type": _format_optional(status.get("imu_type")),
            "imu_chip_id": _format_optional(status.get("imu_chip_id")),
            "imu_data_plausible": ready["imu_data_plausible"],
            "rc_status": ready["rc_status"],
            "heading_status": ready["heading_status"],
            "active_target_source": ready["active_target_source"],
            "live_path_package_connected": ready["live_path_package_connected"],
            "active_primitive_index": primitive["primitive_index"],
            "target_distance_m": f"{distance:.3f}",
            "target_bearing_deg": f"{bearing:.3f}",
            "cross_track_error_m": "0.000",
            "heading_error_deg": heading_error,
            "motor_command_generated": False,
            "physical_output_active": ready["physical_output_active"],
            "ready_for_outdoor_no_motion_validation": ready["ready_for_outdoor_no_motion_validation"],
            "reason": ready["reason"],
            "next_action": ready["next_action"],
        }
        if not concise:
            row |= {
                "current_heading": "" if not heading_available else f"{float(current_heading):.3f}",
                "target_point": f"{target_x:.3f},{target_y:.3f}",
                "along_track_progress_m": f"{progress:.3f}",
                "tool_active_expected": primitive["tool_active"],
            }
        rows.append(row)
        # 다음 반복을 위해 시뮬레이션 포즈를 이번 종점으로 전진 / advance simulated pose to this end.
        current_x = target_x
        current_y = target_y
        current_heading = primitive["end_heading_deg"] if heading_available else None
    return rows


def _first_target_values(rows: Sequence[dict[str, object]]) -> tuple[float | None, float | None]:
    """첫 행의 목표 거리/방위를 float(또는 None)로 추출 / Extract first row's target distance/bearing."""
    if not rows:
        return None, None
    return (
        _parse_optional_float(rows[0].get("target_distance_m")),
        _parse_optional_float(rows[0].get("target_bearing_deg")),
    )


# ── 준비도 게이트(핵심 안전) / Readiness gate (core safety decision) ──
def build_readiness(
    package: dict[str, object],
    status: dict[str, object],
    rows: Sequence[dict[str, object]],
    *,
    validation_mode: str,
    serial_opened: bool,
    selected_path_package: Path,
    min_sats: int,
    max_hdop: float,
    no_motion_gps_mode: str,
) -> dict[str, object]:
    """모든 조건을 AND로 묶어 야외 무동작 검증 준비도를 산출하는 핵심 게이트.

    무엇을/왜: GPS OK + 목표 거리/방위 존재 + 프리미티브 유효 + 모터 미생성 +
    물리출력 비활성 + (live_serial이면 시리얼 열림)이 모두 참이어야 ready=True.
    live_serial에서는 추가로 active_target_source가 compile_time/unknown이면 차단하고
    사유(reason)와 다음행동(next_action)을 설정한다(펌웨어 하드코딩 타깃 방지).
    반환: CSV/요약/표준출력에 쓰이는 준비도 dict(상태 라벨·목표값·플래그·reason·next_action).
    리팩토링 주의: 새 안전 조건은 반드시 이 AND에 추가하고 next_action 분기도 갱신할 것.

    Core safety gate: AND-combine every condition (GPS OK, target present, valid
    primitives, no motor, no physical output, serial open for live) into
    ready_for_outdoor_no_motion_validation. For live_serial, also block a
    compile_time/unknown active_target_source. Returns the readiness dict.
    """
    summary = package["summary"]  # type: ignore[index]
    primitive_sequence_valid = bool(summary["primitive_sequence_valid"])
    motor_command_generated = bool(summary["motor_command_generated"])
    physical_output_active = bool(status["physical_output_active"])
    target_distance, target_bearing = _first_target_values(rows)
    gps_status = _gps_status(
        status,
        validation_mode=validation_mode,
        min_sats=min_sats,
        max_hdop=max_hdop,
        no_motion_gps_mode=no_motion_gps_mode,
    )
    # 준비도 = 모든 안전 조건의 AND. 하나라도 거짓이면 통과 불가.
    # Readiness = AND of all safety conditions; any false blocks it.
    ready = (
        gps_status == "OK"
        and target_distance is not None
        and target_bearing is not None
        and primitive_sequence_valid
        and not motor_command_generated
        and not physical_output_active
        and (validation_mode != "live_serial" or serial_opened)
    )
    active_target_source = str(status.get("active_target_source", "unknown"))
    current_target_source_is_compile_time = active_target_source == "compile_time"
    live_path_package_connected: bool | str = "not_checked_package_only"
    reason = "OK"
    # live_serial 전용 차단: 타깃 소스가 컴파일타임/불명이면 준비 실패로 강제.
    # live_serial-only guard: force not-ready if the target source is compile-time/unknown.
    if validation_mode == "live_serial":
        live_path_package_connected = active_target_source not in {"compile_time", "unknown", ""}
        if current_target_source_is_compile_time:
            reason = "ACTIVE_TARGET_SOURCE_COMPILE_TIME"
            ready = False
        elif not live_path_package_connected:
            reason = "ACTIVE_TARGET_SOURCE_UNKNOWN"
            ready = False
    next_action = "Proceed with no-motion target preview."
    if validation_mode == "package_check" and status.get("position_source") == "package_preview":
        next_action = "Package-only check complete; run live_serial with --duration-s for outdoor GPS target validation."
    elif gps_status == "WAIT":
        next_action = f"Wait for GPS quality: sats >= {min_sats}, hdop <= {max_hdop:g}."
    elif gps_status == "FAIL":
        next_action = "Acquire GPS position before target validation."
    elif physical_output_active or motor_command_generated:
        next_action = "Stop: motor or physical output is active."
    elif target_distance is None or target_bearing is None:
        next_action = "Regenerate the path package; target distance/bearing are NA."
    elif reason == "ACTIVE_TARGET_SOURCE_COMPILE_TIME":
        next_action = "Do not proceed to motor tests. Validate path package offline and implement package-to-firmware/station target bridge."
    elif reason == "ACTIVE_TARGET_SOURCE_UNKNOWN":
        next_action = "Do not proceed to motor tests. Live target source is not a generated path package."

    return {
        "validation_mode": validation_mode,
        "serial_opened": serial_opened,
        "selected_path_package": str(selected_path_package),
        "path_package_loaded": True,
        "gps_status": gps_status,
        "gps_sats": status.get("gps_sats"),
        "gps_hdop": status.get("gps_hdop"),
        "position_source": status.get("position_source", "unknown"),
        "current_lat": status.get("current_lat"),
        "current_lon": status.get("current_lon"),
        "imu_status": _status_label(status["bmi160_ok"]),
        "imu_type": status.get("imu_type", ""),
        "imu_chip_id": status.get("imu_chip_id", ""),
        "imu_data_plausible": bool(status.get("imu_data_plausible")),
        "rc_status": _status_label(status["rc_manual_ok"]),
        "heading_status": _heading_status(status, no_motion_gps_mode),
        "active_target_source": active_target_source,
        "live_path_package_connected": live_path_package_connected,
        "current_target_source_is_compile_time": current_target_source_is_compile_time,
        "active_primitive_index": rows[0]["active_primitive_index"] if rows else "NA",
        "target_distance_m": "NA" if target_distance is None else f"{target_distance:.3f}",
        "target_bearing_deg": "NA" if target_bearing is None else f"{target_bearing:.3f}",
        "cross_track_error_m": rows[0]["cross_track_error_m"] if rows else "NA",
        "heading_error_deg": rows[0]["heading_error_deg"] if rows else "NA_DIAG_ONLY",
        "motor_command_generated": motor_command_generated,
        "physical_output_active": physical_output_active,
        "ready_for_outdoor_no_motion_validation": ready,
        "reason": reason,
        "next_action": next_action,
    }


# ── 산출물 writer / Artifact writers (CSV, Markdown, failure summary) ──
def _write_csv(path: Path, rows: Sequence[dict[str, object]], fields: Sequence[str]) -> None:
    """지정한 필드 순서로 행들을 CSV로 기록(누락 필드는 빈 문자열).

    Write rows to CSV using the given field order; missing fields become "".
    """
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in fields})


def _write_summary(path: Path, ready: dict[str, object], csv_path: Path, *, concise: bool) -> None:
    """준비도 결과를 Markdown 요약으로 기록(concise면 고정 키만, 아니면 전체 키).

    Write the readiness result as a Markdown summary (fixed keys if concise,
    otherwise all keys). Headed by a no-motion banner.
    """
    lines = [
        "# Outdoor Path No-Motion Validation",
        "",
        "No-motion preview only. This tool does not generate motor commands.",
        "",
    ]
    if concise:
        keys = [
            "validation_mode",
            "serial_opened",
            "selected_path_package",
            "path_package_loaded",
            "gps_status",
            "gps_sats",
            "gps_hdop",
            "position_source",
            "current_lat",
            "current_lon",
            "imu_status",
            "imu_type",
            "imu_chip_id",
            "imu_data_plausible",
            "rc_status",
            "heading_status",
            "active_target_source",
            "live_path_package_connected",
            "active_primitive_index",
            "target_distance_m",
            "target_bearing_deg",
            "cross_track_error_m",
            "heading_error_deg",
            "motor_command_generated",
            "physical_output_active",
            "ready_for_outdoor_no_motion_validation",
            "reason",
            "next_action",
        ]
    else:
        keys = list(ready.keys())
    for key in keys:
        lines.append(f"- {key}: `{ready.get(key, '')}`")
    lines.append(f"- validation_csv: `{csv_path}`")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_failure_summary(
    out_dir: Path,
    *,
    validation_mode: str,
    selected_path_package: Path | None,
    serial_opened: bool,
    reason: str,
) -> None:
    """오류(패키지 없음/시리얼 실패 등) 발생 시 최소 실패 요약 Markdown을 남긴다.

    무엇을/왜: 정상 경로로 진행 못 해도 항상 무동작·미준비 상태를 기록해, 상위 자동화가
    산출물 부재로 혼동하지 않게 한다. next_action에 실패 사유를 담는다.

    Write a minimal failure summary (always no-motion, not-ready) when a normal
    run cannot proceed, carrying the reason in next_action.
    """
    out_dir.mkdir(parents=True, exist_ok=True)
    lines = [
        "# Outdoor Path No-Motion Validation",
        "",
        "- validation_mode: `" + validation_mode + "`",
        f"- serial_opened: `{serial_opened}`",
        f"- selected_path_package: `{'' if selected_path_package is None else selected_path_package}`",
        "- path_package_loaded: `" + str(selected_path_package is not None) + "`",
        "- motor_command_generated: `False`",
        "- physical_output_active: `False`",
        "- ready_for_outdoor_no_motion_validation: `False`",
        f"- next_action: `{reason}`",
    ]
    (out_dir / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def _effective_mode(mode: str, port: str | None, duration_s: float) -> str:
    """auto 모드를 실제 모드로 해석: 포트+양수 duration이면 live_serial, 아니면 package_check.

    Resolve auto mode: live_serial if a port and positive duration are given,
    otherwise package_check. Non-auto modes pass through unchanged.
    """
    if mode != "auto":
        return mode
    if port and duration_s > 0:
        return "live_serial"
    return "package_check"


def _print_package_error(error: PathPackageResolutionError) -> None:
    """패키지 해석 실패를 표준출력에 진단 형식으로 출력(가까운 후보 포함).

    Print a package-resolution failure diagnostic to stdout, including nearby
    candidates.
    """
    print(f"provided_path_package={error.provided}")
    print("file_exists=false")
    print(f"error={error}")
    print("nearest_candidates:")
    if error.candidates:
        for candidate in error.candidates:
            print(f"- {candidate}")
    else:
        print("- none")


# ── CLI 진입점 / CLI entry point (argument parsing, main) ──
def build_parser() -> argparse.ArgumentParser:
    """CLI 인자 파서를 구성해 반환 / Build and return the CLI argument parser."""
    parser = argparse.ArgumentParser(
        description="Validate a field serpentine path package in no-motion preview mode."
    )
    parser.add_argument("--path-package", required=True, help="Path to path_package.json, or 'latest'.")
    parser.add_argument("--sample-log", help="Optional text/CSV status log. No serial port is opened.")
    parser.add_argument("--port", default=None, help="OpenRB port for live_serial mode.")
    parser.add_argument("--mode", choices=("package_check", "live_serial", "auto"), default="auto")
    parser.add_argument("--duration-s", type=float, default=0.0)
    parser.add_argument("--out-dir", default="outputs/path_no_motion_validation/latest")
    parser.add_argument("--concise", choices=("true", "false"), default="true")
    parser.add_argument("--no-motion-gps-mode", choices=("position_only", "require_course", "course_required"), default="position_only")
    parser.add_argument("--min-sats", type=int, default=4)
    parser.add_argument("--max-hdop", type=float, default=3.0)
    parser.add_argument("--verbose", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """CLI 진입점: 패키지 해석→상태 수집(모드별)→준비도 산출→CSV/요약 기록·출력.

    무엇을/왜: 모드에 따라 상태 소스를 고른다(live_serial=시리얼, sample_log=파일,
    아니면 package_preview). 두 번 행 생성(초기 rows로 준비도 산출 후 최종 rows 재생성)은
    ready 필드를 각 행에 반영하기 위함. 반환 종료코드: 성공 0, 오류 2.
    부수효과: out_dir에 CSV/Markdown을 쓰고 표준출력에 상태를 찍는다. 모터 출력 없음.

    CLI entry point: resolve the package, gather status per mode, compute
    readiness, then write/print CSV and summary. Returns 0 on success, 2 on
    error. No motor output.
    """
    args = build_parser().parse_args(argv)
    out_dir = Path(args.out_dir)
    try:
        selected_path_package = resolve_path_package(args.path_package)
    except PathPackageResolutionError as error:
        _print_package_error(error)
        _write_failure_summary(
            out_dir,
            validation_mode=args.mode,
            selected_path_package=None,
            serial_opened=False,
            reason="No path_package.json found. Run tools/field_ab_to_serpentine.py first.",
        )
        return 2

    # 별칭 정규화: course_required는 require_course와 동의어 / alias: course_required == require_course.
    no_motion_gps_mode = "require_course" if args.no_motion_gps_mode == "course_required" else args.no_motion_gps_mode
    validation_mode = _effective_mode(args.mode, args.port, args.duration_s)
    package = json.loads(selected_path_package.read_text(encoding="utf-8"))
    serial_opened = False
    try:
        if validation_mode == "live_serial":
            if not args.port:
                raise RuntimeError("--port is required in live_serial mode")
            status = read_live_serial_status(args.port, args.duration_s if args.duration_s > 0 else 120.0)
            serial_opened = True
        elif args.sample_log:
            status = parse_status_log(Path(args.sample_log).read_text(encoding="utf-8"))
        else:
            status = package_preview_status(package)
    except RuntimeError as exc:
        print(f"validation_mode={validation_mode}")
        print("serial_opened=false")
        print(f"selected_path_package={selected_path_package}")
        print(f"error={exc}")
        _write_failure_summary(
            out_dir,
            validation_mode=validation_mode,
            selected_path_package=selected_path_package,
            serial_opened=False,
            reason=str(exc),
        )
        return 2

    preview_ready = {
        "validation_mode": validation_mode,
        "serial_opened": serial_opened,
        "selected_path_package": str(selected_path_package),
        "path_package_loaded": True,
        "gps_status": "",
        "imu_status": "",
        "imu_data_plausible": False,
        "rc_status": "",
        "heading_status": "",
        "active_target_source": "",
        "live_path_package_connected": "not_checked_package_only",
        "active_primitive_index": "",
        "target_distance_m": "",
        "target_bearing_deg": "",
        "cross_track_error_m": "",
        "heading_error_deg": "",
        "motor_command_generated": bool(package["summary"]["motor_command_generated"]),  # type: ignore[index]
        "physical_output_active": bool(status["physical_output_active"]),
        "ready_for_outdoor_no_motion_validation": False,
        "reason": "",
        "next_action": "",
    }
    # 1차 rows로 준비도를 계산한 뒤, 그 ready를 반영해 최종 rows를 다시 만든다(2-pass).
    # First pass computes readiness; then re-build rows so ready fields land in each row.
    initial_rows = build_target_rows(package, status, preview_ready, concise=True)
    ready = build_readiness(
        package,
        status,
        initial_rows,
        validation_mode=validation_mode,
        serial_opened=serial_opened,
        selected_path_package=selected_path_package,
        min_sats=args.min_sats,
        max_hdop=args.max_hdop,
        no_motion_gps_mode=no_motion_gps_mode,
    )
    concise = args.concise == "true" and not args.verbose
    rows = build_target_rows(package, status, ready, concise=concise)
    fields = CONCISE_CSV_FIELDS if concise else CONCISE_CSV_FIELDS + VERBOSE_EXTRA_FIELDS
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / "no_motion_validation.csv"
    summary_path = out_dir / "summary.md"
    _write_csv(csv_path, rows, fields)
    _write_summary(summary_path, ready, csv_path, concise=concise)

    print("Outdoor path no-motion validation generated.")
    print(f"validation_mode={validation_mode}")
    print(f"serial_opened={str(serial_opened).lower()}")
    if validation_mode == "package_check" and args.port:
        print(f"port_reported_not_opened={args.port}")
        print("reason=package_check_does_not_use_serial")
    print(f"selected_path_package={selected_path_package}")
    print("motor_command_generated=false")
    print(f"physical_output_active={str(bool(status['physical_output_active'])).lower()}")
    print(f"gps_status={ready['gps_status']}")
    print(f"heading_status={ready['heading_status']}")
    print(f"ready_for_outdoor_no_motion_validation={ready['ready_for_outdoor_no_motion_validation']}")
    print(f"next_action={ready['next_action']}")
    print(f"validation_csv={csv_path}")
    print(f"summary_md={summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
