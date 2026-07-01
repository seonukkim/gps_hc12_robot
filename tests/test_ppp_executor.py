"""이산 펄스 FSM 계약 테스트 — 가짜 인메모리 시리얼 핸들로 구동.

목적/역할: ``tools.physical_path_planning.executor`` 의 저수준 명령 시퀀싱을 잠근다.
실제 시리얼 없이, 가짜 핸들이 기록한 write 순서와 재생된 텔레메트리 라인으로
정확한 ARM -> command -> pulse-complete -> STOP 순서를 하드웨어 없이 검증한다.

시스템 내 위치: executor 는 CLI/controller 아래에서 펌웨어와 대화하는 계층이며,
``safety`` 의 stop-class 이벤트 집합과 ``telemetry`` 파서를 재사용한다. 이 테스트는
그 상위 계층이 의존하는 executor 의 관측 가능 행동(계약)을 고정한다.

핵심 개념·불변식:
  - stop-class 완료(PULSE_COMPLETE/STOP 등)를 이미 펌웨어가 보고했으면 host 는
    중복 STOP 을 보내지 않는다(불필요한 두 번째 중립 write 방지).
  - 완료 이벤트가 시간 내 오지 않으면 안전을 위해 STOP 을 반드시 전송한다.
  - executor 는 stop-class 집합을 재정의하지 않고 ``safety.STOP_EVENTS`` 를 그대로 쓴다.

Contract test for the discrete pulse FSM, driven by a fake in-memory serial
handle. No real serial: the fake records every written command and replays a
scripted telemetry sequence so we can assert the exact ARM -> command ->
pulse-complete -> STOP ordering without hardware. Locks in the low-level command
sequencing that CLI/controller depend on, plus the "skip redundant STOP after a
stop-class completion / always STOP on timeout" invariants and the reuse of
``safety.STOP_EVENTS``.
"""
from __future__ import annotations

from tools.physical_path_planning import executor


# ── 픽스처·헬퍼 / Fixtures & helpers ──────────────────────────────────────────


class FakeSerial:
    """write 를 기록하고 readline() 마다 대본 응답을 하나씩 반환하는 시리얼 대역.

    Records writes and yields scripted response lines on each readline()."""

    def __init__(self, responses: list[bytes]) -> None:
        # 대본 응답 큐(소비되며 줄어듦)와 관측용 write 로그. / Scripted queue + write log.
        self._responses = list(responses)
        self.writes: list[str] = []

    def write(self, data: bytes) -> int:
        """보낸 명령을 디코드해 로그에 남긴다. / Record the decoded outgoing command."""
        self.writes.append(data.decode("ascii").strip())
        return len(data)

    def flush(self) -> None:  # noqa: D401 - no-op for the fake
        """대역에서는 no-op. / No-op for the fake."""
        pass

    def readline(self) -> bytes:
        """다음 대본 라인, 소진되면 b"". / Next scripted line, or b"" when drained."""
        if self._responses:
            return self._responses.pop(0)
        return b""


# 계획된 펄스의 최소 스펙(ARM/명령/STOP 텍스트 + 펄스 길이). 각 테스트가 공유.
# Minimal planned-pulse spec (ARM/command/STOP texts + pulse length) shared by tests.
PLANNED = {
    "arm_command_text": "ARMCMD",
    "command_text": "PULSECMD",
    "stop_command_text": "STOPCMD",
    "pulse_ms": 200,
}


def test_send_pulse_writes_arm_command_stop_in_order() -> None:
    """정상 펄스: ARM -> 명령만 write 되고, 펌웨어 STOP 보고 후 중복 STOP 없음.

    또한 모든 대본 라인이 공유 버퍼에 캡처되고, 반환 rows 가 펄스 창 전체를 덮는지 확인.
    Happy path: writes ARM -> command only, no redundant STOP after firmware STOP;
    all scripted lines are buffered and the returned rows span the pulse window."""
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

    # Firmware already reported a stop-class completion, so the host does not
    # send a redundant STOP command that could create a second neutral write.
    assert handle.writes == ["ARMCMD", "PULSECMD"]
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
    """verbose_raw=False 는 콘솔 출력을 억제하되 raw 버퍼 캡처는 유지한다.

    verbose_raw=False silences console echo yet still captures lines into the buffer."""
    handle = FakeSerial(
        [
            b"event=ARM\n",
            b"event=ACK\n",
            b"event=HEARTBEAT usb_pulse_test_cmd_state=ACTIVE\n",
            b"event=PULSE_COMPLETE\n",
            b"event=STOP final_left_cmd=0.0 final_right_cmd=0.0\n",
        ]
    )
    raw_lines: list[str] = []
    executor.send_pulse(handle, PLANNED, raw_lines, event_timeout_s=2.0, verbose_raw=False)
    assert "HEARTBEAT" in "\n".join(raw_lines)
    assert capsys.readouterr().out == ""


def test_send_pulse_stop_command_is_skipped_after_stop_class_completion() -> None:
    """PULSE_DONE 같은 다른 stop-class 완료여도 중복 STOP 명령을 보내지 않음.

    Any stop-class completion (e.g. PULSE_DONE) suppresses the redundant STOP write."""
    handle = FakeSerial(
        [b"event=ARM\n", b"event=ACK\n", b"event=PULSE_DONE\n", b"event=STOP\n"]
    )
    executor.send_pulse(handle, PLANNED, [], event_timeout_s=2.0)
    assert handle.writes == ["ARMCMD", "PULSECMD"]


def test_send_pulse_sends_stop_if_completion_missing() -> None:
    """완료 이벤트가 타임아웃 내 없으면 안전을 위해 STOP 명령을 반드시 전송.

    Safety fallback: with no completion event before timeout, STOP must be sent."""
    handle = FakeSerial([b"event=ARM\n", b"event=ACK\n"])
    executor.send_pulse(handle, PLANNED, [], event_timeout_s=0.01)
    assert handle.writes == ["ARMCMD", "PULSECMD", "STOPCMD"]


def test_send_live_drive_sends_repeated_setpoints_then_stop() -> None:
    """라이브 드라이브: 반복 setpoint(seq/a/b) write 후 마지막에 LIVE_STOP 전송.

    Live drive: repeated setpoints (seq/a/b) then a final LIVE_STOP for the seq."""
    handle = FakeSerial(
        [
            b"USB_DRIVE_LIVE event=ACTIVE\n",
            b"USB_DRIVE_LIVE event=ACTIVE\n",
            b"USB_DRIVE_LIVE event=STOP final_left_cmd=0.0 final_right_cmd=0.0\n",
        ]
    )
    raw_lines: list[str] = []
    rows = executor.send_live_drive(
        handle,
        seq=7,
        duration_s=0.02,
        update_hz=100.0,
        ttl_ms=350,
        command_fn=lambda _row: (0.3, 0.0),
        raw_lines=raw_lines,
        event_timeout_s=0.1,
        verbose_raw=False,
    )
    assert handle.writes[0].startswith("USB_DRIVE_LIVE_SET seq=7 a=0.300 b=0.000")
    assert handle.writes[-1] == "USB_DRIVE_LIVE_STOP seq=7"
    assert any(executor.telemetry.event(row) == "STOP" for row in rows)


def test_wait_for_event_returns_when_wanted_seen() -> None:
    """원하는 이벤트(ACK)가 나올 때까지 읽고, 나오면 그 row 를 포함해 반환.

    wait_for_event reads until the wanted event (ACK) appears, returning its row."""
    handle = FakeSerial([b"event=HEARTBEAT\n", b"event=ACK\n"])
    raw_lines: list[str] = []
    rows = executor.wait_for_event(handle, raw_lines, {"ACK"}, timeout_s=2.0)
    assert any(executor.telemetry.event(r) == "ACK" for r in rows)


def test_wait_for_row_applies_predicate() -> None:
    """wait_for_row 는 술어(predicate)를 만족하는 첫 row 까지 읽어 그 row 를 반환.

    wait_for_row reads until a row satisfies the predicate, then returns that row."""
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
    """executor 는 stop-class 집합을 자체 정의하지 않고 safety.STOP_EVENTS 를 재사용(동일 객체).

    executor reuses (is-identical to) safety.STOP_EVENTS instead of redefining it."""
    # Executor reuses the safety stop-class set rather than redefining it.
    from tools.physical_path_planning import safety

    assert executor.PULSE_COMPLETE_EVENTS is safety.STOP_EVENTS
