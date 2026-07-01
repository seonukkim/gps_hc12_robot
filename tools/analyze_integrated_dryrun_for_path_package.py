"""통합 드라이런 로그의 경로패키지 연동 분석기 (Stage 11) / Path-package linkage dry-run analyzer.

목적/역할:
    Stage 11 통합 "드라이런(dry-run)" 로그를 읽어, 자율주행에 쓰일 목표(target)가 **실행
    중 생성된 경로 패키지에서 오는지, 아니면 컴파일 타임에 박힌 상수인지**를 하드웨어 없이
    판정한다. 아울러 GPS 위치 품질, IMU, 그리고 모터 안전 게이트(모두 닫혀 있어야 함)를
    함께 점검해 "모터 테스트로 넘어가도 되는가"에 대한 권고를 낸다.

시스템 내 위치:
    - 독립 실행 CLI: ``python tools/analyze_integrated_dryrun_for_path_package.py <log...>``
      → ``main()``. ``--json-out`` 으로 결과 dict 를 JSON 파일로도 저장할 수 있다.
    - 다른 분석기들과 달리 그림 파이프라인/테스트에서 import 되지 않는(현재) 단독 도구다.

핵심 개념·불변식:
    - 파서는 각 줄에서 ``key=value``(콤마/공백 구분) 토큰을 뽑고, ``_last_values`` 로 **뒤 값이
      앞 값을 덮어써** 로그의 최종 상태 스냅샷을 만든다. 즉 마지막으로 관측된 값이 판정 기준.
    - 안전 불변식: 이 도구는 어떤 경우에도 모터를 돌리지 않으며 결과에 항상
      ``motor_command_generated=False`` 를 포함한다. 목표원이 compile_time 이면 "모터 테스트
      금지" 권고를 낸다.
    - 종료 코드로 결과를 신호한다(아래 main 참고).

사용법/진입점:
    ``analyze_files(paths)`` → 결과 dict. ``main()`` 이 선택 키들을 ``key=value`` 로 출력하고
    ``--json-out`` 이 있으면 전체 dict 를 JSON 으로 기록한다.

리팩토링 노트:
    GPS/IMU 컷 상수(위성 ≥4, HDOP ≤3.0 등)와 안전 게이트 플래그 이름은 ``analyze_rows`` 안에
    인라인되어 있다. 종료 코드 의미(0/1/2)를 바꾸면 이 도구를 호출하는 스크립트/CI 가 영향을
    받으니 함께 확인할 것.

English:
    Stage 11 dry-run analyzer that decides, without hardware, whether the live autonomy target comes
    from a generated path package or a compile-time constant, and cross-checks GPS quality, IMU, and
    motor-safety gates (all must be closed) to recommend whether motor tests may proceed. Standalone
    CLI (``main()``); ``--json-out`` also dumps the result dict. Parser merges later ``key=value``
    tokens over earlier ones (``_last_values``) to snapshot the log's final state. Never runs motors:
    the result always carries ``motor_command_generated=False``. Exit code signals the outcome.
"""
from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path
from typing import Sequence


# ── 값 파싱 헬퍼 / Value-parsing helpers ──
def _parse_bool(value: object) -> bool:
    """느슨한 불리언 파싱: 참으로 볼 토큰 집합에 속하면 True. / Lenient bool: true for a set of tokens.

    '1','true','ok','yes','ready' 를 참으로 본다(대소문자·공백 무시). / Case/space-insensitive.
    """
    return str(value).strip().lower() in {"1", "true", "ok", "yes", "ready"}


def _parse_float(value: object) -> float | None:
    """실수 파싱: 빈값/NA/NaN/None/Null 및 비유한수는 None. / Parse float; None for NA-likes/non-finite.

    유한 실수만 반환한다(inf/nan 은 None). / Returns only finite floats (inf/nan → None).
    """
    text = str(value).strip()
    if text.upper() in {"", "NA", "NAN", "NONE", "NULL"}:
        return None
    try:
        parsed = float(text)
    except ValueError:
        return None
    return parsed if math.isfinite(parsed) else None


# ── 로그 파싱 / Log parsing ──
def parse_log_rows(text: str) -> list[dict[str, str]]:
    """각 줄의 ``key=value`` 토큰을 dict 로 파싱(값 없는 줄은 제외). / Parse ``key=value`` tokens per line.

    값은 콤마/공백을 만나기 전까지의 토큰이다(로그가 콤마 구분일 수도 있어서).
    / A value runs until a comma or whitespace (the log may be comma-separated).
    """
    rows: list[dict[str, str]] = []
    for line in text.splitlines():
        values = {key: value for key, value in re.findall(r"([A-Za-z0-9_]+)=([^,\s]+)", line)}
        if values:
            rows.append(values)
    return rows


def _last_values(rows: Sequence[dict[str, str]]) -> dict[str, str]:
    """모든 행을 순서대로 병합해 최종 상태 스냅샷 생성. / Merge rows in order into a final-state snapshot.

    뒤 행의 값이 앞 행을 덮어쓰므로, 결과는 각 키의 "마지막으로 관측된 값"이다. 판정은 이
    스냅샷을 기준으로 한다. / Later rows overwrite earlier ones, so each key holds its last observed
    value; the verdict is based on this snapshot.
    """
    merged: dict[str, str] = {}
    for row in rows:
        merged.update(row)
    return merged


# ── 판정 로직 / Verdict logic ──
def analyze_rows(rows: Sequence[dict[str, str]]) -> dict[str, object]:
    """최종 상태 스냅샷으로부터 안전·연동 판정 dict 를 만든다. / Build the safety/linkage verdict dict.

    점검 항목: GPS 위치 품질(block 이유 OK·source gps·위성 ≥4·HDOP ≤3.0), IMU(BMI160·데이터
    타당·캘리브레이션), 모터 안전(4개 게이트가 모두 꺼짐), GPS 코스 가용성, 그리고 활성
    목표원(active_target_source)이 compile_time/미지/생성 패키지 중 무엇인지. 목표원에 따라
    ``reason``/``recommendation`` 을 정하고, 항상 ``motor_command_generated=False`` 를 넣는다.
    부수효과 없음(순수 함수).
    / Checks GPS position quality, IMU health, motor-safety gates (all off), GPS course availability,
    and whether the active target source is compile_time / unknown / a generated package; sets
    ``reason``/``recommendation`` accordingly and always includes ``motor_command_generated=False``.
    Pure (no side effects).
    """
    values = _last_values(rows)
    gps_sats = _parse_float(values.get("gps_sats", "NA"))
    gps_hdop = _parse_float(values.get("gps_hdop", "NA"))
    gps_position_ok = (
        values.get("gps_block_reason") == "OK"
        and values.get("position_source") == "gps"
        and gps_sats is not None
        and gps_sats >= 4
        and gps_hdop is not None
        and gps_hdop <= 3.0
    )
    imu_ok = (
        values.get("imu_type") == "BMI160"
        and _parse_bool(values.get("imu_data_plausible"))
        and _parse_bool(values.get("imu_calibrated", "true"))
    )
    # 무동작 안전: 네 개의 모터 게이트가 "모두 꺼져" 있어야 안전으로 본다. / all four gates must be OFF
    motor_safety_ok = (
        not _parse_bool(values.get("physical_compile_gate"))
        and not _parse_bool(values.get("physical_path_following_enable"))
        and not _parse_bool(values.get("allow_motor_output"))
        and not _parse_bool(values.get("physical_output_active"))
    )
    # 코스 값(deg)이 있거나 출력 유효 플래그가 참이면 코스 가용으로 본다. / numeric course OR valid flag
    gps_course_available = _parse_float(values.get("gps_course_deg", "NA")) is not None or _parse_bool(
        values.get("gps_course_output_valid")
    )
    gps_course_block_reason = values.get("gps_course_output_block_reason", "")
    path_block_reason = values.get("path_following_block_reason", "")
    if gps_course_available:
        gps_course_status = "AVAILABLE"
        gps_course_skip_reason = ""
    elif gps_course_block_reason == "NO_ACCEPTED_COURSE_YET" or path_block_reason == "NO_HEADING":
        gps_course_status = "SKIPPED_NO_MOTION_OR_TETHERED"
        gps_course_skip_reason = gps_course_block_reason or path_block_reason
    else:
        gps_course_status = "NOT_AVAILABLE"
        gps_course_skip_reason = gps_course_block_reason or path_block_reason or "UNKNOWN"

    # 핵심 판정 대상: 실행 중 목표가 어디서 오는가. compile_time = 펌웨어 상수(연동 안 됨),
    # 생성 패키지 이름 = 실 연동. unknown/빈값은 미연결로 본다.
    # / The crux: where the live target comes from. compile_time = baked-in constant (not linked);
    #   a generated package name = truly linked; unknown/empty = not connected.
    active_target_source = values.get("active_target_source", "unknown")
    current_target_source_is_compile_time = active_target_source == "compile_time"
    live_path_package_connected = active_target_source not in {"compile_time", "unknown", ""}
    if current_target_source_is_compile_time:
        recommendation = (
            "Do not proceed to motor tests. Validate path package offline and implement "
            "package-to-firmware/station target bridge."
        )
        reason = "ACTIVE_TARGET_SOURCE_COMPILE_TIME"
    elif not live_path_package_connected:
        recommendation = "Do not proceed to motor tests. Live target source is not a generated path package."
        reason = "ACTIVE_TARGET_SOURCE_UNKNOWN"
    else:
        recommendation = "Live target source is not compile_time; continue no-motion validation only."
        reason = "OK"

    return {
        "gps_position_ok": gps_position_ok,
        "gps_sats": gps_sats,
        "gps_hdop": gps_hdop,
        "position_source": values.get("position_source", "unknown"),
        "imu_ok": imu_ok,
        "imu_type": values.get("imu_type", ""),
        "imu_data_plausible": _parse_bool(values.get("imu_data_plausible")),
        "imu_calibrated": _parse_bool(values.get("imu_calibrated", "true")),
        "motor_safety_ok": motor_safety_ok,
        "physical_output_active": _parse_bool(values.get("physical_output_active")),
        "gps_course_status": gps_course_status,
        "gps_course_skip_reason": gps_course_skip_reason,
        "gps_course_output_block_reason": gps_course_block_reason,
        "path_following_block_reason": path_block_reason,
        "active_target_source": active_target_source,
        "live_path_package_connected": live_path_package_connected,
        "current_target_source_is_compile_time": current_target_source_is_compile_time,
        "reason": reason,
        "recommendation": recommendation,
        # 이 도구는 분석 전용이라 모터 명령을 절대 만들지 않는다(안전 명시적 표기).
        # / Analysis-only tool: it never generates a motor command (explicit safety marker).
        "motor_command_generated": False,
    }


def analyze_files(paths: Sequence[Path]) -> dict[str, object]:
    """여러 로그 파일을 이어 붙여 분석하고 경로 목록을 결과에 추가. / Analyze several logs; record their paths.

    파일들을 순서대로 읽어 rows 로 이어 붙인 뒤 ``analyze_rows`` 를 적용한다(마지막 값이 우선).
    부수효과: 각 파일을 읽는다. / Reads files in order, concatenates rows, applies ``analyze_rows``
    (last value wins). Side effect: reads each file.
    """
    rows: list[dict[str, str]] = []
    for path in paths:
        rows.extend(parse_log_rows(path.read_text(encoding="utf-8", errors="replace")))
    result = analyze_rows(rows)
    result["log_paths"] = [str(path) for path in paths]
    return result


# ── CLI 진입점 / CLI entry point ──
def build_parser() -> argparse.ArgumentParser:
    """CLI 파서 생성(로그 1개 이상, 선택적 ``--json-out``). / Build the CLI parser (logs, optional --json-out)."""
    parser = argparse.ArgumentParser(description="Analyze integrated dry-run logs for Stage 11 path-package target linkage.")
    parser.add_argument("logs", nargs="+")
    parser.add_argument("--json-out")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """CLI 진입점: 로그 분석 후 선택 키를 출력. / CLI entry: analyze logs and print selected keys.

    종료 코드: 2=입력 로그 중 없는 파일 존재, 1=목표원이 compile_time(연동 미완, 모터 테스트
    금지), 0=그 외. ``--json-out`` 지정 시 전체 결과 dict 를 JSON 파일로도 기록한다.
    / Exit code: 2 if any input log is missing; 1 if the target source is compile_time (not linked
    — do not run motor tests); 0 otherwise. With ``--json-out`` it also writes the full dict as JSON.
    """
    args = build_parser().parse_args(argv)
    paths = [Path(arg) for arg in args.logs]
    # 존재하지 않는 로그가 하나라도 있으면 즉시 실패(코드 2). / fail fast (code 2) on any missing log
    missing = [path for path in paths if not path.exists()]
    if missing:
        for path in missing:
            print(f"missing_log={path}")
        return 2
    result = analyze_files(paths)
    if args.json_out:
        Path(args.json_out).write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    for key in (
        "gps_position_ok",
        "imu_ok",
        "motor_safety_ok",
        "gps_course_status",
        "gps_course_skip_reason",
        "active_target_source",
        "live_path_package_connected",
        "current_target_source_is_compile_time",
        "reason",
        "recommendation",
        "motor_command_generated",
    ):
        print(f"{key}={result[key]}")
    # compile_time 목표(연동 미완)면 비정상 종료(1)로 CI/스크립트에 실패 신호. / signal not-yet-linked
    return 1 if result["current_target_source_is_compile_time"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
