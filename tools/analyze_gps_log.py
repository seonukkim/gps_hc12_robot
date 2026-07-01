"""USBDBG GPS 건강도 로그 오프라인 분석기 / Offline USBDBG GPS-health log analyzer.

목적/역할:
    OpenRB 펌웨어가 USB 디버그 포트로 흘려보낸 ``USBDBG ...`` 로그(키=값 형식)를 읽어
    GPS 수신 상태를 요약한다. 하드웨어에 전혀 접속하지 않고, 이미 수집된 로그 파일만
    입력으로 받는 순수 오프라인 도구다. gps_fix 성공률, 위성 수/HDOP/보정 지연(age)의
    최소·평균·최대, 그리고 fix가 잡힌 좌표들의 대략적인 위치 표류(drift)를 출력한다.

시스템 내 위치 (import 결합 주의):
    - CLI 진입점: ``python tools/analyze_gps_log.py <log...>`` → ``main()``.
    - **그림 파이프라인 결합**: ``scripts/analysis/_figure_common.py`` 의
      ``_load_usbdbg_gps_records()`` 가 이 모듈에서 ``ParseError`` 와
      ``parse_usbdbg_line`` 을 직접 import 한다. 따라서 ``USBDBGRecord`` 의 필드 이름·
      타입과 ``parse_usbdbg_line``/``ParseError`` 의 시그니처는 CLI 뿐 아니라 리포트 그림
      생성 코드가 의존하는 공개 API다. 함부로 이름/필드를 바꾸면 그림 생성이 깨진다.

핵심 개념·불변식:
    - USBDBG 한 줄은 ``key=value`` 토큰의 나열이며 ``REQUIRED_FIELDS`` 가 모두 있어야
      유효 레코드로 인정된다. 하나라도 없으면 ``ParseError`` (그 줄만 건너뛴다).
    - ``NA`` 는 "값 없음"을 뜻하며 optional 파서가 ``None`` 으로 변환한다.
    - 통계는 **실제 fix(gps_fix=true) 인 샘플만** 집계한다. no-fix 자리표시값
      (예: sats=0, hdop=99.99)이 평균을 왜곡하지 않게 하려는 의도적 불변식이다.

리팩토링 노트:
    필드 추가/개명 시 위 그림 파이프라인 결합을 먼저 확인할 것. 거리 계산은 등거리
    직사각형(equirectangular) 근사라 소규모 텃밭 스케일에서만 정확하다.

English:
    Offline summarizer for ``USBDBG`` GPS-health lines emitted by the OpenRB firmware
    over USB debug. Never touches hardware; consumes already-captured log files. Reports
    gps_fix ratio, sats/HDOP/age min-mean-max, and approximate position drift of fixed
    points. NOTE: ``scripts/analysis/_figure_common.py`` imports ``ParseError`` and
    ``parse_usbdbg_line`` from here, so ``USBDBGRecord`` fields and those symbols are
    load-bearing public API for the report-figure pipeline, not just this CLI. Statistics
    aggregate real fixes only so no-fix placeholders do not distort the report.
"""
from __future__ import annotations

import argparse
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path


# ── 정규식·상수 / Regex and constants ──
# 로그 한 줄에서 key=value 토큰을 뽑는 정규식. 값은 공백이 없는 토큰으로 본다.
# / Extract key=value tokens from a line; a value is a whitespace-free token.
KEY_VALUE_RE = re.compile(r"([A-Za-z0-9_]+)=([^\s]+)")
# 유효 레코드로 인정되려면 이 필드들이 모두 있어야 한다(하나라도 없으면 ParseError).
# / A line must contain all of these to count as a valid record.
REQUIRED_FIELDS = (
    "gps_fix",
    "gps_lat",
    "gps_lon",
    "gps_sats",
    "gps_hdop",
    "gps_age_ms",
    "rc_ok",
    "mode",
    "control_source",
)
EARTH_RADIUS_M = 6_371_000.0
# 경고 폭주 방지: 이 개수까지만 출력하고 나머지는 집계만. / cap on printed parse warnings.
MAX_PARSE_WARNINGS = 10


class ParseError(ValueError):
    """USBDBG 줄에 필수 필드/값이 없을 때 발생. / Raised when a USBDBG line lacks required fields.

    그림 파이프라인(_figure_common.py)도 이 예외를 import 해 잡으므로 공개 API다.
    / Also imported and caught by the figure pipeline, so it is public API.
    """


@dataclass(slots=True)
class USBDBGRecord:
    """파싱된 USBDBG 한 줄의 GPS 관련 필드 스냅샷. / One parsed USBDBG line (GPS fields).

    ``NA`` 로 온 optional 값은 ``None`` 으로 담긴다. 필드 이름/타입은 그림 파이프라인이
    의존하는 계약이므로 변경 시 ``_figure_common.py`` 를 함께 확인할 것.
    / Optional ``NA`` values become ``None``. Field names/types are a contract the figure
    pipeline relies on; check ``_figure_common.py`` before changing them.
    """

    gps_fix: bool
    gps_lat: float | None
    gps_lon: float | None
    gps_sats: int | None
    gps_hdop: float | None
    gps_age_ms: int | None
    rc_ok: bool
    mode: str
    control_source: str


@dataclass(slots=True)
class ParseWarnings:
    """경고 출력/억제 카운터. / Counter for shown vs. suppressed parse warnings."""

    shown: int = 0
    suppressed: int = 0


def build_parser() -> argparse.ArgumentParser:
    """CLI 인자 파서 생성(로그 경로 1개 이상). / Build the CLI parser (one or more log paths)."""
    parser = argparse.ArgumentParser(
        description="Summarize USBDBG GPS health from one or more OpenRB USB debug logs."
    )
    parser.add_argument("paths", nargs="+", help="One or more log files to analyze")
    return parser


def parse_bool(name: str, value: str) -> bool:
    """'true'/'false' 문자열을 bool 로. 그 외엔 ParseError. / Parse 'true'/'false', else ParseError."""
    lowered = value.lower()
    if lowered == "true":
        return True
    if lowered == "false":
        return False
    raise ParseError(f"{name} must be true/false, got {value!r}")


def parse_optional_int(name: str, value: str) -> int | None:
    """정수 또는 'NA'(→None) 파싱. / Parse an int, or 'NA' → None; else ParseError."""
    if value == "NA":
        return None
    try:
        return int(value)
    except ValueError as exc:
        raise ParseError(f"{name} must be an integer or NA, got {value!r}") from exc


def parse_optional_float(name: str, value: str) -> float | None:
    """실수 또는 'NA'(→None) 파싱. / Parse a float, or 'NA' → None; else ParseError."""
    if value == "NA":
        return None
    try:
        return float(value)
    except ValueError as exc:
        raise ParseError(f"{name} must be a float or NA, got {value!r}") from exc


def parse_usbdbg_line(text: str) -> USBDBGRecord:
    """한 줄에서 ``USBDBG`` 이후를 파싱해 레코드 반환. / Parse the ``USBDBG`` payload of a line.

    ``USBDBG`` 마커 없거나 필수 필드가 빠지면 ``ParseError``. 그림 파이프라인도 이 함수를
    호출하므로 시그니처는 공개 API다. / Raises ``ParseError`` if the marker or a required
    field is missing. Also called by the figure pipeline, so the signature is public API.
    """
    usbdbg_offset = text.find("USBDBG")
    if usbdbg_offset < 0:
        raise ParseError("line does not contain USBDBG")

    # 타임스탬프 등 접두부를 잘라내 key=value 오탐을 막는다. / slice off any prefix (timestamps, ...)
    payload = text[usbdbg_offset:]
    fields = dict(KEY_VALUE_RE.findall(payload))
    missing = [field_name for field_name in REQUIRED_FIELDS if field_name not in fields]
    if missing:
        raise ParseError(f"missing fields: {', '.join(missing)}")

    return USBDBGRecord(
        gps_fix=parse_bool("gps_fix", fields["gps_fix"]),
        gps_lat=parse_optional_float("gps_lat", fields["gps_lat"]),
        gps_lon=parse_optional_float("gps_lon", fields["gps_lon"]),
        gps_sats=parse_optional_int("gps_sats", fields["gps_sats"]),
        gps_hdop=parse_optional_float("gps_hdop", fields["gps_hdop"]),
        gps_age_ms=parse_optional_int("gps_age_ms", fields["gps_age_ms"]),
        rc_ok=parse_bool("rc_ok", fields["rc_ok"]),
        mode=fields["mode"],
        control_source=fields["control_source"],
    )


def emit_parse_warning(warnings: ParseWarnings, path: Path, line_number: int, message: str) -> None:
    """파싱 경고를 stderr 로 출력하되 상한까지만. / Emit a parse warning, capped at MAX_PARSE_WARNINGS.

    상한 초과분은 출력하지 않고 ``suppressed`` 만 늘려 마지막에 요약한다.
    / Beyond the cap, only bumps ``suppressed`` for a final summary line.
    """
    if warnings.shown < MAX_PARSE_WARNINGS:
        print(f"Warning: {path}:{line_number}: {message}", file=sys.stderr)
        warnings.shown += 1
        return
    warnings.suppressed += 1


def finish_parse_warnings(warnings: ParseWarnings) -> None:
    """억제된 경고 개수를 마지막에 한 줄로 알림. / Print a final line if warnings were suppressed."""
    if warnings.suppressed:
        print(
            f"Warning: suppressed {warnings.suppressed} additional parse warning(s).",
            file=sys.stderr,
        )


# ── 출력 포매팅 헬퍼 / Output-formatting helpers ──
def format_int(value: float) -> str:
    """반올림 후 정수 문자열. / Round to nearest integer, render as string."""
    return str(int(round(value)))


def format_float(value: float, digits: int = 2) -> str:
    """고정 소수 자릿수 문자열. / Fixed-precision float string."""
    return f"{value:.{digits}f}"


def format_min_mean_max(
    values: list[int | float], *, digits: int = 2, integer_bounds: bool = False
) -> str:
    """min/mean/max 를 한 줄 문자열로. / Format min/mean/max as one line.

    빈 리스트면 'n/a'. ``integer_bounds`` 면 min·max 는 정수, 평균만 소수로 낸다(위성 수처럼
    개수형 지표용). / 'n/a' when empty; with ``integer_bounds`` the bounds are integers and
    only the mean is decimal (for count-like metrics such as satellite count).
    """
    if not values:
        return "n/a"

    low = min(values)
    high = max(values)
    mean = sum(values) / len(values)

    if integer_bounds:
        return (
            f"{format_int(low)} / {format_float(mean, digits)} / {format_int(high)}"
        )
    return f"{format_float(low, digits)} / {format_float(mean, digits)} / {format_float(high, digits)}"


def equirectangular_distance_m(lat_a: float, lon_a: float, lat_b: float, lon_b: float) -> float:
    """두 위경도 사이 거리(m)를 등거리 직사각형 근사로 계산. / Equirectangular distance in metres.

    소규모(텃밭) 스케일에서만 정확한 근사다. 여기서는 위/경도 최소·최대 모서리 사이 거리를
    위치 표류의 대략치로 쓴다. / Small-scale approximation only; used here as an approximate
    position-drift figure between the min/max lat-lon corners.
    """
    lat_a_rad = math.radians(lat_a)
    lat_b_rad = math.radians(lat_b)
    lon_a_rad = math.radians(lon_a)
    lon_b_rad = math.radians(lon_b)
    x = (lon_b_rad - lon_a_rad) * math.cos((lat_a_rad + lat_b_rad) / 2.0)
    y = lat_b_rad - lat_a_rad
    return math.hypot(x, y) * EARTH_RADIUS_M


def main() -> int:
    """CLI 진입점: 로그들을 읽어 GPS 건강도 요약을 출력. / CLI entry: read logs, print GPS-health summary.

    반환값(종료 코드): 0=정상, 1=읽을 파일 없음/USBDBG 없음/전부 파싱 실패.
    / Returns exit code: 0 on success; 1 if no readable files, no USBDBG lines, or none parsed.
    """
    args = build_parser().parse_args()

    # ── 누적 카운터·수집 버퍼 초기화 / Initialize counters and collection buffers ──
    usbdbg_lines_seen = 0
    parsed_records = 0
    malformed_usbdbg_lines = 0
    files_read = 0
    file_errors = 0
    warnings = ParseWarnings()

    gps_fix_true = 0
    sats_values: list[int] = []
    hdop_values: list[float] = []
    age_values: list[int] = []
    fixed_points: list[tuple[float, float]] = []

    # ── 입력 파일 순회·라인 파싱 / Iterate files and parse lines ──
    for raw_path in args.paths:
        path = Path(raw_path)
        try:
            with path.open("r", encoding="utf-8", errors="replace") as handle:
                files_read += 1
                for line_number, line in enumerate(handle, start=1):
                    # 값싼 사전 필터: USBDBG 없는 줄은 정규식 돌리기 전에 건너뜀. / cheap prefilter
                    if "USBDBG" not in line:
                        continue

                    usbdbg_lines_seen += 1
                    try:
                        record = parse_usbdbg_line(line)
                    except ParseError as exc:
                        malformed_usbdbg_lines += 1
                        emit_parse_warning(warnings, path, line_number, str(exc))
                        continue

                    parsed_records += 1
                    if record.gps_fix:
                        gps_fix_true += 1

                        # Summarize only real fixes so placeholder no-fix values
                        # such as sats=0 or hdop=99.99 do not distort the report.
                        if record.gps_sats is not None:
                            sats_values.append(record.gps_sats)
                        if record.gps_hdop is not None:
                            hdop_values.append(record.gps_hdop)
                        if record.gps_age_ms is not None:
                            age_values.append(record.gps_age_ms)
                        if record.gps_lat is not None and record.gps_lon is not None:
                            fixed_points.append((record.gps_lat, record.gps_lon))
        except FileNotFoundError:
            print(f"Error: file not found: {path}", file=sys.stderr)
            file_errors += 1
        except IsADirectoryError:
            print(f"Error: expected a file, got directory: {path}", file=sys.stderr)
            file_errors += 1
        except OSError as exc:
            print(f"Error: failed to read {path}: {exc}", file=sys.stderr)
            file_errors += 1

    finish_parse_warnings(warnings)

    # ── 입력 유효성 게이트 / Input-validity gates ──
    if files_read == 0:
        print("Error: no readable input files.", file=sys.stderr)
        return 1

    if usbdbg_lines_seen == 0:
        print("Error: no USBDBG lines found in the provided input file(s).", file=sys.stderr)
        return 1

    if parsed_records == 0:
        print("Error: found USBDBG lines, but none could be parsed successfully.", file=sys.stderr)
        return 1

    # ── 요약 리포트 출력 / Emit the summary report ──
    gps_fix_ratio = gps_fix_true / parsed_records

    print(f"Parsed USBDBG lines: {parsed_records}")
    if malformed_usbdbg_lines:
        print(f"Skipped malformed USBDBG lines: {malformed_usbdbg_lines}")
    if file_errors:
        print(f"Files with read errors: {file_errors}")
    print(f"gps_fix=true: {gps_fix_true}/{parsed_records} ({gps_fix_ratio:.1%})")
    print(f"sats min/mean/max: {format_min_mean_max(sats_values, digits=2, integer_bounds=True)}")
    print(f"hdop min/mean/max: {format_min_mean_max(hdop_values, digits=2)}")
    print(
        "gps_age_ms min/mean/max: "
        f"{format_min_mean_max(age_values, digits=1, integer_bounds=True)}"
    )

    if fixed_points:
        latitudes = [point[0] for point in fixed_points]
        longitudes = [point[1] for point in fixed_points]
        drift_m = equirectangular_distance_m(
            min(latitudes),
            min(longitudes),
            max(latitudes),
            max(longitudes),
        )
        print(f"lat min/max: {format_float(min(latitudes), 6)} / {format_float(max(latitudes), 6)}")
        print(f"lon min/max: {format_float(min(longitudes), 6)} / {format_float(max(longitudes), 6)}")
        print(f"approximate position drift: {format_float(drift_m, 2)} m")
    else:
        print("lat min/max: n/a")
        print("lon min/max: n/a")
        print("approximate position drift: n/a")
        print("WARNING: no GPS fix lines found; lat/lon range and drift are unavailable.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
