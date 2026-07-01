"""무동작 경로 검증 요약 판정 도구 / Verdict checker for a no-motion path validation summary.

목적/역할:
    경로 검증 실행이 남긴 `summary.md`(마크다운 `- key: value` 목록)를 읽어 하나의 판정을
    낸다: PASS / WAIT / FAIL. 안전 게이트(모터 비활성, 물리 출력 비활성)를 먼저 확인하고, 이어
    경로 패키지 로드·GPS 위치·타깃 거리/방위·GPS 품질(위성 수, HDOP)을 순서대로 검사한다.
    stationary(정지) 상태로 헤딩이 없을 때는 WAIT로 처리한다.

    Reads the `summary.md` (a markdown `- key: value` list) written by a path-validation run
    and produces a single verdict: PASS / WAIT / FAIL. It checks safety gates (motor and
    physical output disabled) first, then path-package load, GPS fix, target distance/bearing,
    and GPS quality (satellite count, HDOP); a stationary/no-heading state yields WAIT.

시스템 내 위치:
    `tools/` CLI 진입점이며 프로젝트 모듈 의존성이 없는 순수 유틸이다(_bootstrap도 import 안 함).
    상위 검증 단계가 만든 요약 파일을 소비하는 하류 게이트로, CI나 사람이 "타깃 미리보기 준비됨"을
    빠르게 확인할 때 쓴다.

    Entry-point script in `tools/` with no project-module dependencies (a pure utility). It is
    a downstream gate consuming the summary file produced by an upstream validation step,
    used by CI or an operator to confirm "target preview ready".

핵심 개념·불변식:
    - 판정 우선순위가 곧 안전 계약이다: 안전 플래그(motor/physical) 확인이 GPS 품질보다 먼저다.
    - FAIL은 안전·전제 조건 위반, WAIT는 조건이 아직 충족되지 않음(회복 가능), PASS는 준비 완료.
    - 종료 코드: PASS/WAIT는 0, FAIL은 1 — WAIT는 실패가 아니라 '대기'임에 주의.
    - `_is_true`/`_is_false`는 관대한 문자열 파싱을 하고, `_finite_float`는 NA/NaN/빈값을
      None으로 정규화한다.

    - The check ordering *is* the safety contract: motor/physical flags are verified before
      GPS quality. FAIL = safety/precondition violated, WAIT = not-yet-satisfied (recoverable),
      PASS = ready. Exit codes: 0 for PASS/WAIT, 1 for FAIL — WAIT is not a failure.

사용법/진입점:
    CLI 진입점은 main(). 예: `python -m tools.check_path_no_motion_summary OUTDIR` 또는
    `--summary path/to/summary.md`. --min-sats(기본 4), --max-hdop(기본 3.0)로 임계값 조정.

    CLI entry point is main(); pass an output dir or --summary, tune --min-sats / --max-hdop.

리팩토링 노트:
    파서 정규식은 `- key: value` 마크다운 라인과, 요약 생성 측의 키 이름(validation_mode,
    motor_command_generated 등)에 결합되어 있다. 요약 포맷을 바꾸면 여기 검사도 함께 고칠 것.

    The parser regex is coupled to the `- key: value` markdown lines and to the exact key
    names emitted by the summary producer. Update both together if the summary format changes.
"""

from __future__ import annotations

import argparse
import math
import re
from pathlib import Path
from typing import Sequence


def _parse_summary(text: str) -> dict[str, str]:
    """마크다운 `- key: value` 라인들을 {key: value} dict로 파싱한다(백틱 제거).

    Parse markdown `- key: value` lines into a {key: value} dict, stripping backticks.
    """
    values: dict[str, str] = {}
    for line in text.splitlines():
        match = re.match(r"^- ([A-Za-z0-9_]+): `?(.*?)`?$", line.strip())
        if match:
            values[match.group(1)] = match.group(2).strip("`")
    return values


def _is_true(value: str | None) -> bool:
    """관대한 참 판정: 1/true/ok/yes/ready를 True로 본다 / lenient truthy check."""
    return str(value).strip().lower() in {"1", "true", "ok", "yes", "ready"}


def _is_false(value: str | None) -> bool:
    """관대한 거짓 판정: 0/false/no/off를 False로 본다 / lenient falsy check.

    주의: _is_true의 여집합이 아니다 — 미상 값은 둘 다 False일 수 있다.
    Note: not the complement of _is_true — an unknown value can be neither.
    """
    return str(value).strip().lower() in {"0", "false", "no", "off"}


def _finite_float(value: str | None) -> float | None:
    """유한 float로 파싱하되, NA/NaN/빈값/파싱 실패는 None으로 정규화한다.

    Parse to a finite float; NA/NaN/empty/unparseable all normalize to None.
    """
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


def evaluate_summary(values: dict[str, str], *, min_sats: int = 4, max_hdop: float = 3.0) -> tuple[str, str]:
    """요약 값 dict를 검사해 (판정, 사유) 튜플을 반환한다.

    Evaluate the parsed summary dict and return a (verdict, reason) tuple.
    verdict은 PASS/WAIT/FAIL 중 하나 / verdict is one of PASS/WAIT/FAIL.
    검사 순서가 곧 안전 우선순위다(아래 배너 참조) / order encodes safety priority.
    """
    # ── 안전 게이트 (하드 FAIL) / Safety gates (hard FAIL) ──
    # 라이브 검증은 시리얼이 열려 있어야 하고 타깃 소스가 패키지에 연결돼야 한다
    # live validation requires the serial to be open and the target source connected
    if values.get("validation_mode") == "live_serial" and not _is_true(values.get("serial_opened")):
        return "FAIL", "serial was expected for live validation but was not opened"
    if values.get("validation_mode") == "live_serial" and values.get("live_path_package_connected") == "False":
        return "FAIL", "live target source is not connected to the generated path package"
    # 모터/물리 출력이 '비활성'으로 명시 확인되지 않으면 즉시 FAIL / must be provably disabled
    if not _is_false(values.get("motor_command_generated")):
        return "FAIL", "motor output is not confirmed disabled"
    if not _is_false(values.get("physical_output_active")):
        return "FAIL", "physical output is active or not reported false"

    # ── 전제 조건 (FAIL) / Preconditions (FAIL) ──
    if not _is_true(values.get("path_package_loaded")):
        return "FAIL", "load a path_package.json before validation"
    if values.get("position_source") != "gps" or values.get("gps_status") == "FAIL":
        return "FAIL", "wait for a GPS position fix before target preview"
    if not _is_true(values.get("target_distance_finite")) or not _is_true(values.get("target_bearing_finite")):
        return "FAIL", "target distance or bearing is NA"

    # ── GPS 품질 게이트 (회복 가능 → WAIT) / GPS quality gates (recoverable → WAIT) ──
    sats = _finite_float(values.get("gps_sats"))
    hdop = _finite_float(values.get("gps_hdop"))
    if sats is None or hdop is None:
        return "WAIT", "GPS position exists, but sats/HDOP quality is not reported"
    if sats < min_sats:
        return "WAIT", f"wait for at least {min_sats} GPS satellites"
    if hdop > max_hdop:
        return "WAIT", f"wait for GPS HDOP <= {max_hdop:g}"

    # ── 헤딩 가용성 / Heading availability ──
    # 정지 상태에서는 GPS 코스 헤딩이 없어 거리/방위가 진단용에 머문다 → WAIT
    # while stationary the GPS course heading is absent, so keep values diagnostic → WAIT
    if values.get("heading_status") in {"DIAG_ONLY", "WAITING_FOR_MOTION_OR_DIAG_ONLY"}:
        return "WAIT", "GPS course heading is unavailable while stationary; target distance/bearing remain diagnostic"
    return "PASS", "target preview ready"


def resolve_summary_path(path_arg: str | None, summary_arg: str | None) -> Path:
    """위치 인자/--summary로부터 실제 summary.md 경로를 결정한다.

    Resolve the actual summary.md path from the positional arg and/or --summary.
    --summary가 최우선, 디렉터리면 그 안의 summary.md를 사용 / --summary wins; dir -> dir/summary.md.
    둘 다 없으면 SystemExit / raises SystemExit if neither is given.
    """
    if summary_arg:
        return Path(summary_arg)
    if not path_arg:
        raise SystemExit("ERROR: provide an output directory or --summary path.")
    path = Path(path_arg)
    return path / "summary.md" if path.is_dir() else path


def build_parser() -> argparse.ArgumentParser:
    """이 도구의 argparse 파서를 구성한다 / build the argparse parser for this tool."""
    parser = argparse.ArgumentParser(description="Check a concise no-motion path validation summary.")
    parser.add_argument("path", nargs="?", help="Validation output directory or summary.md path.")
    parser.add_argument("--summary", help="Explicit summary.md path.")
    parser.add_argument("--min-sats", type=int, default=4)
    parser.add_argument("--max-hdop", type=float, default=3.0)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """CLI 진입점: 요약 경로 해석→파싱→판정 출력. PASS/WAIT면 0, FAIL이면 1 반환.

    CLI entry point: resolve path, parse, print verdict; returns 0 for PASS/WAIT, 1 for FAIL.
    """
    args = build_parser().parse_args(argv)
    summary_path = resolve_summary_path(args.path, args.summary)
    values = _parse_summary(summary_path.read_text(encoding="utf-8"))
    verdict, action = evaluate_summary(values, min_sats=args.min_sats, max_hdop=args.max_hdop)
    print(f"{verdict}: {action}")
    return 0 if verdict in {"PASS", "WAIT"} else 1


if __name__ == "__main__":
    raise SystemExit(main())
