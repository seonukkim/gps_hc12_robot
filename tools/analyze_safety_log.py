"""USBDBG 안전 동작 로그 오프라인 분석기 / Offline USBDBG safety-behavior log analyzer.

목적/역할:
    OpenRB 펌웨어의 ``USBDBG ...`` 로그를 읽어 **안전 계약이 지켜졌는지** 판정한다.
    모드·제어원(control_source) 목록, rc_ok 참/거짓 횟수, 좌/우 모터 명령의 최대 절댓값을
    집계하고, 특히 두 가지 위반을 검사한다: (1) rc_ok=false 인데 모터 명령이 0이 아님,
    (2) control_source=STOP 인데 모터 명령이 0이 아님. 둘 다 없으면 ``Safety: PASS``.
    하드웨어 접속 없이 캡처된 로그 파일만 입력으로 받는 순수 오프라인 도구다.

시스템 내 위치 (import 결합 주의):
    - CLI 진입점: ``python tools/analyze_safety_log.py <log...>`` → ``main()``.
    - **그림 파이프라인 결합**: ``scripts/analysis/_figure_common.py`` 의
      ``_load_usbdbg_safety_records()`` 가 이 모듈에서 ``ParseError`` 와
      ``parse_usbdbg_line`` 을 직접 import 한다. 그래서 ``USBDBGRecord`` 의 필드 이름·타입,
      그리고 ``parse_usbdbg_line``/``ParseError`` 시그니처는 리포트 그림 생성 코드가 의존하는
      공개 API다. (analyze_gps_log.py 와 같은 결합 패턴.)

핵심 개념·불변식:
    - USBDBG 한 줄은 ``key=value`` 나열이고 ``REQUIRED_FIELDS`` 가 모두 있어야 유효하다.
    - ``NA`` → optional 정수 파서에서 ``None``.
    - 안전 판정은 "위반이 하나라도 관측되면 FAIL" 이라는 단조(monotonic) 규칙이다. 즉 한 줄만
      위반해도 플래그가 켜지고 다시 꺼지지 않는다. 로그가 안전하려면 **모든** 샘플이
      계약을 지켜야 한다.

리팩토링 노트:
    필드 추가/개명 시 위 그림 파이프라인 결합을 먼저 확인할 것. 새 위반 조건을 넣을 때는
    main() 의 누적 플래그 패턴을 따르고 ``safety_pass`` 계산에 반영할 것.

English:
    Offline pass/fail checker for ``USBDBG`` safety lines from the OpenRB firmware. Reports
    the set of modes/control sources, rc_ok true/false counts, and max |left_cmd|/|right_cmd|,
    and flags two violations: nonzero motor command while rc_ok=false, and while
    control_source=STOP. Clean logs yield ``Safety: PASS``. NOTE: ``_figure_common.py`` imports
    ``ParseError`` and ``parse_usbdbg_line`` from here, so ``USBDBGRecord`` fields and those
    symbols are load-bearing public API for the figure pipeline (same coupling as
    analyze_gps_log.py). Violation flags are monotonic: one bad sample fails the whole log.
"""
from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path


# ── 정규식·상수 / Regex and constants ──
# 로그 한 줄에서 key=value 토큰을 뽑는다(값은 공백 없는 토큰). / Extract key=value tokens per line.
KEY_VALUE_RE = re.compile(r"([A-Za-z0-9_]+)=([^\s]+)")
# 유효 레코드에 반드시 있어야 하는 필드들(하나라도 없으면 ParseError). / required for a valid record.
REQUIRED_FIELDS = (
    "mode",
    "rc_ok",
    "auto_sw",
    "ppm_age_ms",
    "steer_us",
    "throttle_us",
    "mode_us",
    "steer_norm",
    "throttle_norm",
    "station_age_ms",
    "station_manual_valid",
    "station_deadman",
    "station_estop",
    "control_source",
    "left_cmd",
    "right_cmd",
    "gps_fix",
)
# 경고 폭주 방지 상한. / cap on printed parse warnings.
MAX_PARSE_WARNINGS = 10


class ParseError(ValueError):
    """USBDBG 줄에 필수 필드/값이 없을 때 발생. / Raised when a USBDBG line lacks required fields.

    그림 파이프라인(_figure_common.py)도 이 예외를 import 해 잡으므로 공개 API다.
    / Also imported and caught by the figure pipeline, so it is public API.
    """


@dataclass(slots=True)
class USBDBGRecord:
    """파싱된 USBDBG 한 줄의 안전 관련 필드 스냅샷. / One parsed USBDBG line (safety fields).

    ``NA`` optional 값은 ``None``. 필드 이름/타입은 그림 파이프라인 계약이므로 변경 시
    ``_figure_common.py`` 확인. / Optional ``NA`` values become ``None``; field names/types are
    a figure-pipeline contract, so check ``_figure_common.py`` before changing them.
    """

    mode: str
    rc_ok: bool
    auto_sw: bool
    ppm_age_ms: int | None
    steer_us: int
    throttle_us: int
    mode_us: int
    steer_norm: float
    throttle_norm: float
    station_age_ms: int | None
    station_manual_valid: bool
    station_deadman: bool
    station_estop: bool
    control_source: str
    left_cmd: float
    right_cmd: float
    gps_fix: bool


@dataclass(slots=True)
class ParseWarnings:
    """경고 출력/억제 카운터. / Counter for shown vs. suppressed parse warnings."""

    shown: int = 0
    suppressed: int = 0


def build_parser() -> argparse.ArgumentParser:
    """CLI 인자 파서 생성(로그 경로 1개 이상). / Build the CLI parser (one or more log paths)."""
    parser = argparse.ArgumentParser(
        description="Summarize USBDBG safety behavior from one or more OpenRB logs."
    )
    parser.add_argument("paths", nargs="+", help="One or more log files to analyze")
    return parser


def parse_bool(name: str, value: str) -> bool:
    """'true'/'false' → bool, 그 외 ParseError. / Parse 'true'/'false', else ParseError."""
    lowered = value.lower()
    if lowered == "true":
        return True
    if lowered == "false":
        return False
    raise ParseError(f"{name} must be true/false, got {value!r}")


def parse_int(name: str, value: str) -> int:
    """정수 파싱, 실패 시 ParseError. / Parse an int, else ParseError."""
    try:
        return int(value)
    except ValueError as exc:
        raise ParseError(f"{name} must be an integer, got {value!r}") from exc


def parse_float(name: str, value: str) -> float:
    """실수 파싱, 실패 시 ParseError. / Parse a float, else ParseError."""
    try:
        return float(value)
    except ValueError as exc:
        raise ParseError(f"{name} must be a float, got {value!r}") from exc


def parse_optional_int(name: str, value: str) -> int | None:
    """정수 또는 'NA'(→None). / Parse an int, or 'NA' → None."""
    if value == "NA":
        return None
    return parse_int(name, value)


def parse_usbdbg_line(text: str) -> USBDBGRecord:
    """한 줄에서 ``USBDBG`` 페이로드를 파싱해 레코드 반환. / Parse the ``USBDBG`` payload of a line.

    마커 없거나 필수 필드 누락 시 ``ParseError``. 그림 파이프라인도 호출하므로 공개 API.
    / Raises ``ParseError`` on missing marker/fields. Also called by the figure pipeline (public API).
    """
    usbdbg_offset = text.find("USBDBG")
    if usbdbg_offset < 0:
        raise ParseError("line does not contain USBDBG")

    # 접두부(타임스탬프 등)를 잘라 key=value 오탐 방지. / slice off any prefix before token parse
    payload = text[usbdbg_offset:]
    fields = dict(KEY_VALUE_RE.findall(payload))
    missing = [field_name for field_name in REQUIRED_FIELDS if field_name not in fields]
    if missing:
        raise ParseError(f"missing fields: {', '.join(missing)}")

    return USBDBGRecord(
        mode=fields["mode"],
        rc_ok=parse_bool("rc_ok", fields["rc_ok"]),
        auto_sw=parse_bool("auto_sw", fields["auto_sw"]),
        ppm_age_ms=parse_optional_int("ppm_age_ms", fields["ppm_age_ms"]),
        steer_us=parse_int("steer_us", fields["steer_us"]),
        throttle_us=parse_int("throttle_us", fields["throttle_us"]),
        mode_us=parse_int("mode_us", fields["mode_us"]),
        steer_norm=parse_float("steer_norm", fields["steer_norm"]),
        throttle_norm=parse_float("throttle_norm", fields["throttle_norm"]),
        station_age_ms=parse_optional_int("station_age_ms", fields["station_age_ms"]),
        station_manual_valid=parse_bool(
            "station_manual_valid", fields["station_manual_valid"]
        ),
        station_deadman=parse_bool("station_deadman", fields["station_deadman"]),
        station_estop=parse_bool("station_estop", fields["station_estop"]),
        control_source=fields["control_source"],
        left_cmd=parse_float("left_cmd", fields["left_cmd"]),
        right_cmd=parse_float("right_cmd", fields["right_cmd"]),
        gps_fix=parse_bool("gps_fix", fields["gps_fix"]),
    )


def emit_parse_warning(warnings: ParseWarnings, path: Path, line_number: int, message: str) -> None:
    """파싱 경고를 상한까지만 stderr 출력. / Emit a parse warning, capped at MAX_PARSE_WARNINGS.

    상한 초과분은 ``suppressed`` 로만 집계. / Beyond the cap, only bumps ``suppressed``.
    """
    if warnings.shown < MAX_PARSE_WARNINGS:
        print(f"Warning: {path}:{line_number}: {message}", file=sys.stderr)
        warnings.shown += 1
        return
    warnings.suppressed += 1


def finish_parse_warnings(warnings: ParseWarnings) -> None:
    """억제된 경고 수를 마지막에 요약. / Print a final line if warnings were suppressed."""
    if warnings.suppressed:
        print(
            f"Warning: suppressed {warnings.suppressed} additional parse warning(s).",
            file=sys.stderr,
        )


# ── 출력·판정 헬퍼 / Formatting and predicate helpers ──
def format_values(values: set[str]) -> str:
    """문자열 집합을 정렬해 콤마로 연결(빈 집합은 'n/a'). / Sorted comma-join of a set ('n/a' if empty)."""
    if not values:
        return "n/a"
    return ", ".join(sorted(values))


def format_yes_no(value: bool) -> str:
    """bool 을 'yes'/'no' 로. / Render a bool as 'yes'/'no'."""
    return "yes" if value else "no"


def command_is_nonzero(record: USBDBGRecord) -> bool:
    """좌·우 모터 명령 중 하나라도 0이 아니면 True. / True if either motor command is nonzero.

    안전 위반 검사(정지 상태에서 모터가 도는지)의 핵심 술어다.
    / Core predicate for the safety-violation checks (motion while it should be stopped).
    """
    return record.left_cmd != 0.0 or record.right_cmd != 0.0


def main() -> int:
    """CLI 진입점: 로그를 읽어 안전 요약과 PASS/FAIL 판정을 출력. / CLI entry: safety summary + verdict.

    반환값(종료 코드): 0=정상 실행(판정은 PASS/FAIL 로 출력), 1=읽을 파일 없음/USBDBG 없음/
    전부 파싱 실패. 주의: 안전 FAIL 여도 종료 코드는 0이다(판정은 stdout 텍스트로 전달).
    / Returns 0 on a successful run (verdict printed as PASS/FAIL); 1 if no readable files, no
    USBDBG lines, or none parsed. NOTE: a safety FAIL still returns 0; the verdict is text on stdout.
    """
    args = build_parser().parse_args()

    # ── 누적 카운터·플래그 초기화 / Initialize counters and violation flags ──
    usbdbg_lines_seen = 0
    parsed_records = 0
    malformed_usbdbg_lines = 0
    files_read = 0
    file_errors = 0
    warnings = ParseWarnings()

    modes: set[str] = set()
    control_sources: set[str] = set()
    rc_ok_true = 0
    rc_ok_false = 0
    max_abs_left_cmd = 0.0
    max_abs_right_cmd = 0.0
    nonzero_cmd_while_rc_bad = False
    nonzero_cmd_while_stop = False
    # 단조 플래그: 한 줄이라도 no-fix 면 영구히 False. / monotonic flag; once no-fix, stays False.
    gps_remained_fix = True

    # ── 입력 파일 순회·라인 파싱·안전 위반 누적 / Iterate files, parse, accumulate violations ──
    for raw_path in args.paths:
        path = Path(raw_path)
        try:
            with path.open("r", encoding="utf-8", errors="replace") as handle:
                files_read += 1
                for line_number, line in enumerate(handle, start=1):
                    # 값싼 사전 필터. / cheap prefilter before regex
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
                    modes.add(record.mode)
                    control_sources.add(record.control_source)
                    if record.rc_ok:
                        rc_ok_true += 1
                    else:
                        rc_ok_false += 1
                    max_abs_left_cmd = max(max_abs_left_cmd, abs(record.left_cmd))
                    max_abs_right_cmd = max(max_abs_right_cmd, abs(record.right_cmd))
                    if not record.gps_fix:
                        gps_remained_fix = False

                    # 안전 계약 위반 탐지: 멈춰 있어야 할 상황에서 모터 명령이 살아 있는지.
                    # / Safety-contract violations: motor command alive when it must be zero.
                    if command_is_nonzero(record):
                        if not record.rc_ok:
                            nonzero_cmd_while_rc_bad = True
                        if record.control_source == "STOP":
                            nonzero_cmd_while_stop = True
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

    # ── 판정 및 요약 리포트 출력 / Compute verdict and emit the summary report ──
    # 두 위반 중 하나라도 관측되면 FAIL(단조 규칙). / FAIL if either violation was ever seen.
    safety_pass = not nonzero_cmd_while_stop and not nonzero_cmd_while_rc_bad

    print(f"Parsed USBDBG lines: {parsed_records}")
    if malformed_usbdbg_lines:
        print(f"Skipped malformed USBDBG lines: {malformed_usbdbg_lines}")
    if file_errors:
        print(f"Files with read errors: {file_errors}")
    print(f"Unique modes: {format_values(modes)}")
    print(f"Unique control_source values: {format_values(control_sources)}")
    print(f"rc_ok=true: {rc_ok_true}")
    print(f"rc_ok=false: {rc_ok_false}")
    print(f"Max |left_cmd|: {max_abs_left_cmd:.3f}")
    print(f"Max |right_cmd|: {max_abs_right_cmd:.3f}")
    print(
        "Nonzero motor command while rc_ok=false: "
        f"{format_yes_no(nonzero_cmd_while_rc_bad)}"
    )
    print(
        "Nonzero motor command while control_source=STOP: "
        f"{format_yes_no(nonzero_cmd_while_stop)}"
    )
    print(f"GPS remained fix throughout test: {format_yes_no(gps_remained_fix)}")
    print(f"Safety: {'PASS' if safety_pass else 'FAIL'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
