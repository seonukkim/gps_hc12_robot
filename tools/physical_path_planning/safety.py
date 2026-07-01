"""Guarded-motion safety predicates over USBDBG telemetry rows.

These are the abort/wait conditions for guarded USB motion, lifted here as
named pure functions so the executor and controller
share one definition. Every predicate takes already-parsed rows (or a single
row) and returns a plain bool / reason string -- no serial, no side effects --
which makes them directly unit-testable with mock telemetry.

Semantics are covered in ``tests/test_ppp_safety.py``.

목적/역할 (KO):
    가드된 USB 이동(guarded motion)의 "중단(abort) / 대기(wait)" 판단을 이름
    붙은 순수 함수로 모아 둔 잎 모듈이다. 직렬 통신도, 부수효과도 없다 -- 이미
    파싱된 행(들)을 받아 순수하게 bool 또는 사유 문자열만 돌려준다. 그래서 mock
    텔레메트리로 곧바로 단위 테스트가 가능하고, executor 와 controller 가 "무엇이
    위험 상태인가"에 대한 단일 정의를 공유한다.

시스템 내 위치 (KO):
    telemetry(접근자/헬퍼) 위에 얹히고, executor·controller 가 이 술어들을
    호출한다. 방향은 telemetry -> safety -> executor -> controller -> cli.
    이 모듈은 checks 의 ``ready_for_full_path_following=false`` 불변식을 실행
    시점(runtime)에서 받쳐 준다: 핸드셰이크(ACK/STOP) 누락, STOP 이후에도 출력
    잔존, 0 이 아닌 잔여 명령 같은 위험 상태를 잡아 상위가 즉시 멈추게 한다.

핵심 개념 (KO):
    - STOP 계열 이벤트 집합 ``STOP_EVENTS`` = 펄스가 "통제된 종료"에 도달했음을
      뜻하며, 그 뒤 모터 출력은 0 이어야 한다.
    - reject 사유는 현재 USB 펄스 필드를 우선하고, 없으면 하위호환 필드로 폴백.

Purpose (EN):
    Leaf module: guarded-motion abort/wait conditions as named pure functions
    over already-parsed telemetry rows -- no serial, no side effects -- so
    executor and controller share one definition and each predicate is directly
    unit-testable with mock telemetry. Sits on top of ``telemetry`` and backs the
    ``ready_for_full_path_following=false`` invariant at runtime by catching a
    missing ACK/STOP handshake, output still active after STOP, or non-zero
    residual commands. Semantics are locked in ``tests/test_ppp_safety.py``.
"""
from __future__ import annotations

from typing import Sequence

from tools.physical_path_planning import telemetry

# Events that mark the controlled end of a pulse (motor output must be zero after).
# KO: 펄스의 "통제된 종료"를 표시하는 이벤트 집합. 이 뒤에는 모터 출력이 0 이어야 한다.
STOP_EVENTS = {"STOP", "STOP_ALREADY_ZERO", "PULSE_COMPLETE", "PULSE_DONE"}

_ZERO_TOLERANCE = 1e-9
# KO: 문자열을 일부러 조각내 합친다("stage"+"20...") -- 스테이지 번호로 코드를
#     자동 스캔/치환하는 도구가 이 하위호환 키를 건드리지 못하게 하는 의도.
_COMPAT_REJECT_REASON_PRIMARY = "stage" + "20_reject_reason"
_COMPAT_REJECT_REASON_FALLBACK = "stage" + "16_reject_reason"


def latest_reject_reason(rows: Sequence[dict[str, str]]) -> str:
    """Most recent reject reason; prefers current USB pulse fields, then compatibility fields.

    KO: 최신 reject 사유를 반환한다. 현재 USB 펄스 필드를 최우선으로 보고, 없으면
    station_drive 필드, 그다음 하위호환(stage20/16) 필드 순으로 폴백한다.
    """
    reason = telemetry._latest(rows, "usb_pulse_test_reject_reason", "")
    if reason:
        return reason
    reason = telemetry._latest(rows, "station_drive_reject_reason", "")
    if reason:
        return reason
    reason = telemetry._latest(rows, _COMPAT_REJECT_REASON_PRIMARY, "")
    return reason if reason else telemetry._latest(rows, _COMPAT_REJECT_REASON_FALLBACK, "NONE")


def rc_invalid_abort(rows: Sequence[dict[str, str]]) -> bool:
    """True => an RC_INVALID reject was reported; an active pulse must hard-abort.

    KO: RC_INVALID reject 가 보고되었는가. 진행 중인 펄스는 즉시 강제 중단해야 한다.
    """
    return latest_reject_reason(rows) == "RC_INVALID"


def missing_ack_or_stop_abort(rows: Sequence[dict[str, str]]) -> str | None:
    """Return ``"ACK_MISSING"`` / ``"STOP_MISSING"`` if the handshake is incomplete, else None.

    KO: ARM->ACK->STOP 핸드셰이크가 불완전하면 어느 쪽이 빠졌는지 사유를 돌려준다.
    ACK 가 하나도 없으면 "ACK_MISSING", STOP 계열이 하나도 없으면 "STOP_MISSING".
    둘 다 관측되면 None(정상).
    """
    if not any(telemetry.event(row) == "ACK" for row in rows):
        return "ACK_MISSING"
    if not any(telemetry.event(row) in STOP_EVENTS for row in rows):
        return "STOP_MISSING"
    return None


def output_active_after_stop(rows: Sequence[dict[str, str]]) -> bool:
    """True => a STOP row still reported physical output active (motors not cut).

    KO: STOP 이벤트 행인데도 물리 출력이 여전히 활성이면 True -- 모터가 실제로
    끊기지 않은 위험 상태이므로 상위가 명시적 STOP 재전송/중단을 하게 만든다.
    """
    return any(
        telemetry.event(row) == "STOP" and telemetry.physical_output_active(row)
        for row in rows
    )


def nonzero_final_cmd(rows: Sequence[dict[str, str]]) -> bool:
    """True => the latest final left/right command is not (within tolerance) zero.

    KO: 최신 좌/우 최종 명령이 (허용오차 내) 0 이 아니면 True. 펄스 종료 후에도
    잔여 구동 명령이 남아 있는지 확인하는 안전 검사. 필드가 없으면 0 으로 본다.
    """
    left = telemetry._optional_float(telemetry._latest(rows, "final_left_cmd", "0")) or 0.0
    right = telemetry._optional_float(telemetry._latest(rows, "final_right_cmd", "0")) or 0.0
    return abs(left) > _ZERO_TOLERANCE or abs(right) > _ZERO_TOLERANCE


def rc_neutral_wait(row: dict[str, str]) -> bool:
    """True => RC is not yet neutral/ready; the caller must keep waiting before pulsing.

    Ready means: RC link ok, sticks neutral, and physical output not already active.

    KO: RC 가 아직 중립/준비 상태가 아니면 True -- 펄스를 쏘기 전에 계속 기다려야
    한다. "준비"란 RC 링크 정상 + 스틱 중립 + 물리 출력 비활성을 모두 만족.
    """
    rc_ok = telemetry._parse_bool(row.get("rc_ok"))
    neutral_ok = telemetry._parse_bool(row.get("neutral_ok"))
    output_active = telemetry.physical_output_active(row)
    return not (rc_ok and neutral_ok and not output_active)


def preflight_heartbeat(row: dict[str, str]) -> bool:
    """True => the row is a healthy preflight HEARTBEAT safe to pulse from.

    Requires a HEARTBEAT event with RC ok, sticks neutral, physical output
    inactive, and both the path-following enable and motor-output gates OFF. This
    is the firmware-side safety state; it intentionally does NOT assert the
    guarded-mode role/compat fields (callers add those if they need them).

    KO: 이 행이 펄스를 시작해도 되는 건강한 프리플라이트 HEARTBEAT 인가. 조건은
    HEARTBEAT 이벤트 + RC 정상 + 스틱 중립 + 물리 출력 비활성 + 경로추종
    enable 게이트 OFF + 모터출력 허용 게이트 OFF. 이는 펌웨어 쪽 안전 상태이며,
    가드모드 role/compat 필드는 일부러 검사하지 않는다(필요한 호출부가 따로 추가).
    """
    return (
        telemetry.event(row) == "HEARTBEAT"
        and telemetry._parse_bool(row.get("rc_ok")) is True
        and telemetry._parse_bool(row.get("neutral_ok")) is True
        and telemetry._parse_bool(row.get("physical_output_active")) is False
        and telemetry._parse_bool(row.get("physical_path_following_enable")) is False
        and telemetry._parse_bool(row.get("allow_motor_output")) is False
    )
