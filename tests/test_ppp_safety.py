"""Tests for the guarded-motion safety predicates."""
from __future__ import annotations

from tools.physical_path_planning import safety


def _heartbeat(**overrides: str) -> dict[str, str]:
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


def test_rc_invalid_abort() -> None:
    assert safety.rc_invalid_abort([{"stage20_reject_reason": "RC_INVALID"}]) is True
    assert safety.rc_invalid_abort([{"stage20_reject_reason": "NONE"}]) is False
    assert safety.rc_invalid_abort([]) is False
    # stage16 fallback channel is consulted when stage20 reason is blank.
    assert safety.rc_invalid_abort([{"stage16_reject_reason": "RC_INVALID"}]) is True


def test_latest_reject_reason_prefers_stage20_then_stage16() -> None:
    rows = [{"stage16_reject_reason": "FOO"}, {"stage20_reject_reason": "BAR"}]
    assert safety.latest_reject_reason(rows) == "BAR"
    assert safety.latest_reject_reason([{"stage16_reject_reason": "FOO"}]) == "FOO"
    assert safety.latest_reject_reason([]) == "NONE"


def test_missing_ack_or_stop_abort() -> None:
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
    assert safety.nonzero_final_cmd(
        [{"final_left_cmd": "0.0", "final_right_cmd": "0.0"}]
    ) is False
    assert safety.nonzero_final_cmd([{"final_left_cmd": "0.2"}]) is True
    assert safety.nonzero_final_cmd([{"final_right_cmd": "-0.05"}]) is True
    # Missing fields default to zero -> not flagged.
    assert safety.nonzero_final_cmd([{}]) is False


def test_rc_neutral_wait() -> None:
    assert safety.rc_neutral_wait(_heartbeat()) is False  # ready -> no wait
    assert safety.rc_neutral_wait(_heartbeat(rc_ok="0")) is True
    assert safety.rc_neutral_wait(_heartbeat(neutral_ok="0")) is True
    assert safety.rc_neutral_wait(_heartbeat(physical_output_active="1")) is True


def test_preflight_heartbeat() -> None:
    assert safety.preflight_heartbeat(_heartbeat()) is True
    assert safety.preflight_heartbeat(_heartbeat(event="PULSE")) is False
    assert safety.preflight_heartbeat(_heartbeat(rc_ok="0")) is False
    assert safety.preflight_heartbeat(_heartbeat(allow_motor_output="1")) is False
    assert safety.preflight_heartbeat(_heartbeat(physical_path_following_enable="1")) is False
