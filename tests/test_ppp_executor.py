"""Tests for the discrete pulse FSM, driven by a fake in-memory serial handle.

No real serial: the fake handle records every written command and replays a
scripted sequence of telemetry lines, so we can assert the exact ARM -> command
-> pulse-complete -> STOP ordering without hardware.
"""
from __future__ import annotations

from tools.physical_path_planning import executor


class FakeSerial:
    """Records writes and yields scripted response lines on each readline()."""

    def __init__(self, responses: list[bytes]) -> None:
        self._responses = list(responses)
        self.writes: list[str] = []

    def write(self, data: bytes) -> int:
        self.writes.append(data.decode("ascii").strip())
        return len(data)

    def flush(self) -> None:  # noqa: D401 - no-op for the fake
        pass

    def readline(self) -> bytes:
        if self._responses:
            return self._responses.pop(0)
        return b""


PLANNED = {
    "arm_command_text": "ARMCMD",
    "stage20_command_text": "PULSECMD",
    "stop_command_text": "STOPCMD",
    "pulse_ms": 200,
}


def test_send_pulse_writes_arm_command_stop_in_order() -> None:
    handle = FakeSerial(
        [
            b"event=ARM\n",
            b"event=ACK\n",
            b"event=PULSE_COMPLETE\n",
            b"event=STOP final_left_cmd=0.0 final_right_cmd=0.0\n",
        ]
    )
    raw_lines: list[str] = []
    rows = executor.send_pulse(handle, PLANNED, raw_lines, event_timeout_s=2.0)

    # The three commands are issued in the safety-critical order.
    assert handle.writes == ["ARMCMD", "PULSECMD", "STOPCMD"]
    # All scripted telemetry lines were captured into the shared buffer.
    assert raw_lines == [
        "event=ARM",
        "event=ACK",
        "event=PULSE_COMPLETE",
        "event=STOP final_left_cmd=0.0 final_right_cmd=0.0",
    ]
    # Returned rows cover the pulse window (start index was 0).
    assert [executor.telemetry.event(r) for r in rows] == [
        "ARM",
        "ACK",
        "PULSE_COMPLETE",
        "STOP",
    ]


def test_send_pulse_can_suppress_raw_console(capsys) -> None:
    handle = FakeSerial(
        [
            b"event=ARM\n",
            b"event=ACK\n",
            b"event=HEARTBEAT stage20_cmd_state=ACTIVE\n",
            b"event=PULSE_COMPLETE\n",
            b"event=STOP final_left_cmd=0.0 final_right_cmd=0.0\n",
        ]
    )
    raw_lines: list[str] = []
    executor.send_pulse(handle, PLANNED, raw_lines, event_timeout_s=2.0, verbose_raw=False)
    assert "HEARTBEAT" in "\n".join(raw_lines)
    assert capsys.readouterr().out == ""


def test_send_pulse_stop_command_follows_pulse_complete() -> None:
    handle = FakeSerial(
        [b"event=ARM\n", b"event=ACK\n", b"event=PULSE_DONE\n", b"event=STOP\n"]
    )
    executor.send_pulse(handle, PLANNED, [], event_timeout_s=2.0)
    # STOPCMD is only written after the pulse-complete event is observed.
    assert handle.writes.index("STOPCMD") == 2


def test_wait_for_event_returns_when_wanted_seen() -> None:
    handle = FakeSerial([b"event=HEARTBEAT\n", b"event=ACK\n"])
    raw_lines: list[str] = []
    rows = executor.wait_for_event(handle, raw_lines, {"ACK"}, timeout_s=2.0)
    assert any(executor.telemetry.event(r) == "ACK" for r in rows)


def test_wait_for_row_applies_predicate() -> None:
    handle = FakeSerial([b"event=HEARTBEAT rc_ok=0\n", b"event=HEARTBEAT rc_ok=1\n"])
    raw_lines: list[str] = []
    row = executor.wait_for_row(
        handle,
        raw_lines,
        lambda r: r.get("rc_ok") == "1",
        timeout_s=2.0,
    )
    assert row is not None and row.get("rc_ok") == "1"


def test_pulse_complete_events_match_safety_stop_events() -> None:
    # Executor reuses the safety stop-class set rather than redefining it.
    from tools.physical_path_planning import safety

    assert executor.PULSE_COMPLETE_EVENTS is safety.STOP_EVENTS
