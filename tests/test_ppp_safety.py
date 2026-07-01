"""가드된 모션(guarded-motion) 안전 판정 술어(predicate)들의 계약을 고정하는 테스트.

목적/역할 (KO):
    ``safety`` 모듈의 안전 술어들이 텔레메트리 행 목록을 보고 "중단(abort)/대기
    (wait)/이상(flag)" 을 올바르게 판정하는지 못 박는다. 이 술어들은 물리 모션이
    안전 조건을 벗어났는지 판정하는 마지막 소프트웨어 관문이므로, 그 판정 규칙이
    회귀하면 곧바로 안전 사고로 이어질 수 있다.

핵심 계약·불변식 (KO):
    - ``rc_invalid_abort`` / ``latest_reject_reason``: 현재 거부 사유 채널
      (``usb_pulse_test_reject_reason``) 을 우선 보고, 비어 있으면 하위 호환
      채널(``station_drive_reject_reason``) 을 참조한다. 값이 없으면 ``"NONE"``.
    - ``missing_ack_or_stop_abort``: ARM 후 ACK 가 없으면 ``"ACK_MISSING"``,
      ACK 뒤 정지류 이벤트가 없으면 ``"STOP_MISSING"``, 둘 다 있으면 ``None``.
      정지류 이벤트는 세 종류(PULSE_COMPLETE/PULSE_DONE/... 등) 중 무엇이든 인정.
    - ``output_active_after_stop``: *STOP 행에서* 출력이 살아 있을 때만 이상으로
      본다(STOP 아닌 행의 활성 출력은 이 술어의 대상 아님).
    - ``nonzero_final_cmd``: 좌/우 최종 명령 중 하나라도 0 이 아니면 이상. 필드가
      없으면 0 으로 간주 -> 이상 아님.
    - ``rc_neutral_wait`` / ``preflight_heartbeat``: RC/중립/출력/자율 게이트가
      "안전한 대기 가능" 상태인지 판정. 아래 ``_heartbeat`` 헬퍼가 만드는 "모두
      정상" 행을 기준선으로 삼아 각 필드를 하나씩 뒤집어 반응을 확인한다.

Purpose (EN):
    Locks the decision rules of the guarded-motion safety predicates: the
    current-then-compatibility reject-reason precedence, the ACK/STOP sequence
    requirement, "output active only counts on a STOP row", nonzero-final-command
    detection with missing-field-as-zero, and the RC/neutral/preflight gates. A
    fully-nominal heartbeat is the baseline; each field is flipped in turn.
"""
from __future__ import annotations

from tools.physical_path_planning import safety


# ── 픽스처 헬퍼 / Fixture helper ──
def _heartbeat(**overrides: str) -> dict[str, str]:
    """"모두 정상" 하트비트 행을 만든다(개별 필드는 kwargs 로 뒤집음) / build a
    fully-nominal heartbeat row; override individual fields via kwargs to probe
    a single failing condition at a time."""
    row = {
        "event": "HEARTBEAT",
        "rc_ok": "1",
        "neutral_ok": "1",
        "physical_output_active": "0",
        "physical_path_following_enable": "0",
        "allow_motor_output": "0",
    }
    row.update(overrides)
    return row


# ── 안전 술어 테스트 / Safety-predicate tests ──
def test_rc_invalid_abort() -> None:
    """RC_INVALID 사유면 중단; 현재 채널이 비면 호환 채널도 참조 / aborts on an
    ``RC_INVALID`` reject reason, consulting the compatibility channel when the
    current one is blank."""
    assert safety.rc_invalid_abort([{"usb_pulse_test_reject_reason": "RC_INVALID"}]) is True
    assert safety.rc_invalid_abort([{"usb_pulse_test_reject_reason": "NONE"}]) is False
    assert safety.rc_invalid_abort([]) is False
    # Compatibility fallback channel is still consulted when current reason is blank.
    assert safety.rc_invalid_abort([{"station_drive_reject_reason": "RC_INVALID"}]) is True


def test_latest_reject_reason_prefers_current_then_compatibility() -> None:
    """거부 사유는 현재 채널 우선, 없으면 호환 채널, 그래도 없으면 "NONE" / latest
    reject reason prefers the current channel, then the compatibility one, else
    ``"NONE"``."""
    rows = [{"station_drive_reject_reason": "FOO"}, {"usb_pulse_test_reject_reason": "BAR"}]
    assert safety.latest_reject_reason(rows) == "BAR"
    assert safety.latest_reject_reason([{"station_drive_reject_reason": "FOO"}]) == "FOO"
    assert safety.latest_reject_reason([]) == "NONE"


def test_missing_ack_or_stop_abort() -> None:
    """ARM->ACK->정지 시퀀스 누락을 사유 문자열로, 완성되면 None 으로 판정 / reports
    a missing ACK or missing stop as a reason string, ``None`` once the
    ARM->ACK->stop sequence is complete (any stop-class event qualifies)."""
    assert safety.missing_ack_or_stop_abort([{"event": "ARM"}]) == "ACK_MISSING"
    assert safety.missing_ack_or_stop_abort([{"event": "ACK"}]) == "STOP_MISSING"
    assert (
        safety.missing_ack_or_stop_abort(
            [{"event": "ACK"}, {"event": "PULSE_COMPLETE"}]
        )
        is None
    )
    # Any of the three stop-class events satisfies the stop requirement.
    assert (
        safety.missing_ack_or_stop_abort([{"event": "ACK"}, {"event": "PULSE_DONE"}])
        is None
    )


def test_output_active_after_stop() -> None:
    """STOP 행에서 출력이 살아 있으면 이상; STOP 아닌 행은 대상 아님 / flags output
    still active on a STOP row; an active output on a non-STOP row is ignored."""
    assert safety.output_active_after_stop(
        [{"event": "STOP", "physical_output_active": "active"}]
    ) is True
    assert safety.output_active_after_stop(
        [{"event": "STOP", "physical_output_active": "0"}]
    ) is False
    # Output active on a non-STOP row is not flagged by this predicate.
    assert safety.output_active_after_stop(
        [{"event": "PULSE", "physical_output_active": "1"}]
    ) is False


def test_nonzero_final_cmd() -> None:
    """좌/우 최종 명령 중 하나라도 0 이 아니면 이상; 필드 없으면 0 으로 간주 / flags
    when either final command is nonzero; a missing field defaults to zero."""
    assert safety.nonzero_final_cmd(
        [{"final_left_cmd": "0.0", "final_right_cmd": "0.0"}]
    ) is False
    assert safety.nonzero_final_cmd([{"final_left_cmd": "0.2"}]) is True
    assert safety.nonzero_final_cmd([{"final_right_cmd": "-0.05"}]) is True
    # Missing fields default to zero -> not flagged.
    assert safety.nonzero_final_cmd([{}]) is False


def test_rc_neutral_wait() -> None:
    """RC/중립/출력 중 어느 하나라도 비정상이면 대기 True, 모두 정상이면 False /
    waits (``True``) if RC, neutral, or output is off-nominal; ``False`` only
    when all are nominal."""
    assert safety.rc_neutral_wait(_heartbeat()) is False  # ready -> no wait
    assert safety.rc_neutral_wait(_heartbeat(rc_ok="0")) is True
    assert safety.rc_neutral_wait(_heartbeat(neutral_ok="0")) is True
    assert safety.rc_neutral_wait(_heartbeat(physical_output_active="1")) is True


def test_preflight_heartbeat() -> None:
    """프리플라이트 통과 조건: HEARTBEAT 이벤트 + 모든 게이트가 안전 상태여야 함 /
    a preflight heartbeat passes only when the event is ``HEARTBEAT`` and every
    gate (RC, motor-output, path-following) is in its safe state."""
    assert safety.preflight_heartbeat(_heartbeat()) is True
    assert safety.preflight_heartbeat(_heartbeat(event="PULSE")) is False
    assert safety.preflight_heartbeat(_heartbeat(rc_ok="0")) is False
    assert safety.preflight_heartbeat(_heartbeat(allow_motor_output="1")) is False
    assert safety.preflight_heartbeat(_heartbeat(physical_path_following_enable="1")) is False
