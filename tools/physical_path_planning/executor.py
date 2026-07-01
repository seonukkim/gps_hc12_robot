"""Discrete guarded pulse execution: the ARM -> ACK -> STOP finite-state machine.

This owns the serial-facing mechanics: line-reading
wait loops and the four-step pulse handshake. ``send_pulse`` issues exactly one
guarded pulse (arm, command, await pulse-complete, stop) and returns the rows
captured during that pulse window. Higher layers own the reporting / IMU / GPS
logic around each pulse.

The firmware still owns the real motor-output safety gate; this module only
sequences commands and waits for the matching telemetry events.

목적/역할 (KO):
    이 모듈은 "가드된 펄스(guarded pulse)"의 직렬 유한상태기계(FSM)를 소유한다.
    즉 직렬 포트에 명령 한 줄을 쓰고, 대응하는 텔레메트리 이벤트가 올 때까지 라인을
    읽으며 기다리는 저수준 기계를 담당한다. ``send_pulse`` 는 정확히 한 번의 가드된
    펄스를 낸다: (1) ARM 명령 -> ARM/REJECT 대기, (2) 구동 명령 -> ACK/REJECT 대기,
    (3) 펄스 완료(STOP/PULSE_COMPLETE/PULSE_DONE) 대기, (4) STOP 명령 -> STOP 확인.
    그리고 그 펄스 창(window) 동안 수집된 행들을 반환한다. IMU/GPS/리포트 같은 상위
    로직은 이 모듈이 아니라 controller 가 각 펄스 주위에 두른다.

직렬 프로토콜 (KO) -- ARM -> ACK -> STOP:
    호스트(PC)는 명령을 "쓰고 나서 기다린다". 각 단계는 자신을 인정(ack)하는
    이벤트가 오기 전에는 다음 단계로 넘어가지 않는다. ARM 또는 구동 단계에서
    REJECT 가 오면 펄스는 조기 종료된다. 완료 이벤트에 STOP 확인이 섞여 있지
    않으면 짧게 한 번 더 STOP 확인을 기다린다. 마지막에는 필요 시(또는 강제 시)
    명시적 STOP 을 보내 모터가 실제로 0 이 되도록 만든다. 실제 모터 출력 차단은
    여전히 펌웨어 안전 게이트의 몫이며, 이 모듈은 순서화와 대기만 한다.

시스템 내 위치 (KO):
    telemetry(파싱/접근자)와 safety(정지 이벤트 집합)를 임포트한다. 임포트 방향은
    telemetry, safety -> executor -> controller -> cli. executor 는 거꾸로 상위를
    임포트하지 않는다. ``PULSE_COMPLETE_EVENTS`` 는 safety 의 STOP 집합을 그대로
    재사용해 "정지 판정"의 단일 출처를 유지한다.

함정/주의 (KO):
    - wait 루프는 ``time.monotonic()`` 기반 데드라인을 쓴다(월클록 역행 영향 없음).
    - 완료 대기 타임아웃은 최소 ``pulse_ms`` + 1초 여유를 보장한다.
    - handle 은 ``readline``/``write``/``flush`` 를 가진 오리 타이핑 객체다
      (실직렬 또는 mock). 그래서 이 파일은 하드웨어 없이도 테스트된다.

Purpose (EN):
    Owns the guarded-pulse serial FSM: it writes one command line to the serial
    handle and reads lines until the matching telemetry event arrives. Protocol
    is ARM -> ACK -> STOP: each step blocks for its acknowledging event before the
    next; a REJECT at the arm/command step ends the pulse early; an explicit STOP
    is sent at the end (or when forced) so motor output actually reaches zero.
    ``send_pulse`` runs exactly one pulse and returns the rows from its window;
    higher layers (controller) own IMU/GPS/reporting. Imports telemetry + safety
    only (one-way: -> executor -> controller -> cli) and reuses ``safety.STOP_EVENTS``
    for the completion set. Wait loops use ``time.monotonic()`` deadlines; the
    handle is duck-typed (``readline``/``write``/``flush``) so this is testable
    against a mock without hardware. Firmware still owns the real motor-output gate.
"""
from __future__ import annotations

import time
from typing import Callable, Sequence

from tools.physical_path_planning import safety, telemetry

# ── FSM 전이 이벤트 집합 / FSM transition event sets ──
# KO: 각 단계가 기다리는 이벤트. arm/command 단계의 REJECT 는 펄스를 조기 종료시킨다.
ARM_EVENTS = {"ARM", "REJECT"}
COMMAND_ACK_EVENTS = {"ACK", "REJECT"}
STOP_CONFIRM_EVENTS = {"STOP", "STOP_ALREADY_ZERO"}
# Pulse-complete events are shared with the safety layer's stop-class set.
# KO: 완료 이벤트 집합은 safety 의 STOP 집합을 재사용 -- 정지 판정의 단일 출처.
PULSE_COMPLETE_EVENTS = safety.STOP_EVENTS


def should_send_stop_after_completion(rows: Sequence[dict[str, str]]) -> bool:
    """True when the host should send an explicit STOP after the completion wait.

    KO: 완료 대기 이후에도 호스트가 명시적 STOP 을 보내야 하는가. 관측된 행이 아예
    없거나(=완료 확증 불가), STOP 계열을 못 봤거나, STOP 이후에도 출력이 남아 있으면
    True -- 안전을 위해 STOP 을 한 번 더 확실히 보낸다.
    """
    if not rows:
        return True
    stop_like_seen = any(telemetry.event(row) in safety.STOP_EVENTS for row in rows)
    active_after_stop = safety.output_active_after_stop(rows)
    return (not stop_like_seen) or active_after_stop


def serial_rows(raw_lines: Sequence[str], start_index: int = 0) -> list[dict[str, str]]:
    """Parse raw serial lines (from ``start_index`` on) into telemetry rows.

    KO: 수집된 원시 라인 버퍼에서 ``start_index`` 이후 구간만 골라 다시 합쳐 파싱한다.
    펄스 창(window) 단위로 "이 펄스 동안의 행들"을 잘라 보는 데 쓰인다.
    """
    return telemetry.parse_usbdbg_rows("\n".join(raw_lines[start_index:]))


def _write_line(handle: object, text: object) -> None:
    """Write one newline-terminated ASCII line to the serial handle and flush.

    KO: 명령 한 줄을 개행 종료 ASCII 로 인코딩해 직렬 handle 에 쓰고 즉시 flush.
    flush 를 반드시 해야 펌웨어가 그 줄을 지연 없이 받는다(버퍼링 방지).
    """
    handle.write((str(text) + "\n").encode("ascii"))  # type: ignore[attr-defined]
    handle.flush()  # type: ignore[attr-defined]


def write_command(handle: object, text: object) -> None:
    """Write one newline-terminated command to the serial handle and flush.

    Public wrapper around :func:`_write_line` so peer modules (e.g. heading
    alignment) can issue their own bounded live-drive SET/STOP commands without
    reaching into a private helper.
    """
    _write_line(handle, text)


def wait_for_row(
    handle: object,
    raw_lines: list[str],
    predicate: Callable[[dict[str, str]], bool],
    timeout_s: float,
    *,
    verbose_raw: bool = True,
) -> dict[str, str] | None:
    """Read serial lines until one parses to a row satisfying ``predicate``.

    무엇을/왜 (KO): 데드라인까지 한 줄씩 읽어, 파싱된 행이 ``predicate`` 를 만족하면
    그 행을 반환한다. 시간 초과면 ``None``. 읽은 원시 라인은 ``raw_lines`` 에 계속
    append 되며(부수효과), ``verbose_raw`` 면 그대로 화면에 출력한다.
    반환/부수효과 (EN): returns the first matching row or ``None`` on timeout;
    appends every read line to ``raw_lines`` (in place) and optionally echoes it.
    """
    # KO: 월클록 대신 monotonic 데드라인 -- 시스템 시간이 뒤로 점프해도 안전.
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        raw = handle.readline()  # type: ignore[attr-defined]
        if raw:
            line = raw.decode("utf-8", errors="replace").strip()
            if verbose_raw:
                print(line)
            raw_lines.append(line)
            rows = serial_rows([line])
            row = rows[0] if rows else None
            if row is not None and predicate(row):
                return row
    return None


def wait_for_event(
    handle: object,
    raw_lines: list[str],
    wanted: set[str],
    timeout_s: float,
    *,
    verbose_raw: bool = True,
) -> list[dict[str, str]]:
    """Read serial lines until any row carries an event in ``wanted``.

    무엇을/왜 (KO): 데드라인까지 라인을 읽어, 이번 호출에서 새로 쌓인 행들 중 하나라도
    ``wanted`` 이벤트를 담으면 그 시점까지의 "이 호출분" 행들을 반환한다. 타임아웃이면
    그때까지 쌓인 이번 호출분 행들을 반환한다(빈 리스트일 수 있음). FSM 각 단계가
    ARM/ACK/STOP 같은 특정 이벤트를 기다리는 핵심 프리미티브.
    부수효과 (EN): appends read lines to ``raw_lines``; returns rows parsed from
    this call's slice (from the pre-call length onward), whether matched or timed out.
    """
    # KO: monotonic 데드라인 + 이번 호출 시작 지점 기록(이번 호출분만 잘라 파싱).
    deadline = time.monotonic() + timeout_s
    start_index = len(raw_lines)
    while time.monotonic() < deadline:
        raw = handle.readline()  # type: ignore[attr-defined]
        if raw:
            line = raw.decode("utf-8", errors="replace").strip()
            if verbose_raw:
                print(line)
            raw_lines.append(line)
            rows = serial_rows(raw_lines, start_index)
            if any(telemetry.event(row) in wanted for row in rows):
                return rows
    return serial_rows(raw_lines, start_index)


# ── 가드된 펄스 FSM / Guarded-pulse FSM (ARM -> ACK -> STOP) ──


def send_pulse(
    handle: object,
    planned: dict[str, object],
    raw_lines: list[str],
    *,
    event_timeout_s: float,
    verbose_raw: bool = True,
) -> list[dict[str, str]]:
    """Run one guarded pulse and return the telemetry rows from its window.

    Sequence (each step waits for its acknowledging event before the next):
    arm command -> {ARM,REJECT}; pulse command -> {ACK,REJECT}; await pulse
    completion -> {STOP,PULSE_COMPLETE,PULSE_DONE}; stop command -> {STOP}.
    The pulse-completion wait is given at least ``pulse_ms`` plus a second of slack.

    무엇을/왜 (KO): 정확히 한 번의 가드된 펄스를 순서대로 실행하고, 그 펄스 창 동안
    수집된 텔레메트리 행들을 반환한다. ``planned`` 는 arm/구동/stop 명령 텍스트와
    ``pulse_ms`` 등을 담은 사전이다. 각 단계는 자신을 인정하는 이벤트가 오기 전에는
    다음으로 넘어가지 않으며, arm/구동 단계의 REJECT 는 펄스를 조기 종료시킨다.
    부수효과 (EN): writes commands to ``handle`` and appends read lines to
    ``raw_lines`` in place; returns the rows parsed from this pulse's window.
    """
    # 이 펄스 창의 시작점 / window start index for slicing this pulse's rows:
    pulse_start = len(raw_lines)
    # (1) ARM: arm 명령을 쓰고 ARM/REJECT 를 기다린다.
    _write_line(handle, planned["arm_command_text"])
    wait_for_event(handle, raw_lines, ARM_EVENTS, event_timeout_s, verbose_raw=verbose_raw)
    # KO: 구동 명령 키 폴백 순서 -- 신규(command_text) -> station_drive -> 하위호환.
    #     "stage"+"20..." 는 자동 스테이지 치환 도구가 건드리지 못하게 조각낸 것.
    command_text = (
        planned.get("command_text")
        or planned.get("station_drive_command_text")
        or planned.get("stage" + "20_command_text")
    )
    # (2) ACK: 구동 명령을 쓰고 ACK/REJECT 를 기다린다.
    _write_line(handle, command_text)
    wait_for_event(handle, raw_lines, COMMAND_ACK_EVENTS, event_timeout_s, verbose_raw=verbose_raw)
    pulse_ms = int(planned["pulse_ms"])
    # (3) 완료 대기: 최소 pulse_ms + 1초 여유를 보장(펄스가 끝날 시간을 준다).
    completion_rows = wait_for_event(
        handle,
        raw_lines,
        PULSE_COMPLETE_EVENTS,
        max(event_timeout_s, pulse_ms / 1000.0 + 1.0),
        verbose_raw=verbose_raw,
    )
    # KO: 완료는 봤지만 STOP 확인 이벤트가 안 섞였으면, 짧게(≤0.25s) 한 번 더 STOP 확인 대기.
    if completion_rows and not any(telemetry.event(row) in STOP_CONFIRM_EVENTS for row in completion_rows):
        wait_for_event(
            handle,
            raw_lines,
            STOP_CONFIRM_EVENTS,
            min(event_timeout_s, 0.25),
            verbose_raw=verbose_raw,
        )
    rows_so_far = serial_rows(raw_lines, pulse_start)
    # (4) STOP: 강제 옵션이거나 완료 후 STOP 이 미흡하면 명시적 STOP 을 보낸다.
    send_explicit_stop = bool(planned.get("force_stop_command")) or should_send_stop_after_completion(completion_rows)
    if send_explicit_stop:
        _write_line(handle, planned["stop_command_text"])
        # KO: 아직 STOP 확인을 못 봤을 때만 추가로 STOP 확인을 기다린다(중복 대기 방지).
        if not any(telemetry.event(row) in STOP_CONFIRM_EVENTS for row in rows_so_far):
            wait_for_event(handle, raw_lines, STOP_CONFIRM_EVENTS, event_timeout_s, verbose_raw=verbose_raw)
    return serial_rows(raw_lines, pulse_start)


def send_live_drive(
    handle: object,
    *,
    seq: int,
    duration_s: float,
    update_hz: float,
    ttl_ms: int,
    command_fn: Callable[[dict[str, str] | None], tuple[float, float]],
    raw_lines: list[str],
    event_timeout_s: float,
    verbose_raw: bool = True,
) -> list[dict[str, str]]:
    """Send continuous USB A/B setpoints with a firmware-side deadman TTL.

    무엇을/왜 (KO): ``send_pulse`` 의 이산(discrete) 펄스와 달리, 여기서는 duration 동안
    ``update_hz`` 주기로 A/B 세트포인트를 연속 전송한다. 각 프레임의 (a,b)는
    ``command_fn(latest_row)`` 이 결정하므로 최신 텔레메트리에 반응하는 폐루프가 된다.
    ``ttl_ms`` 는 펌웨어 쪽 데드맨(deadman): 갱신이 끊기면 펌웨어가 스스로 정지하므로,
    호스트가 죽거나 링크가 끊겨도 로버가 계속 달리지 않는다. REJECT 를 받으면 즉시 중단.
    부수효과 (EN): writes LIVE_SET frames plus a final LIVE_STOP to ``handle`` and
    appends read lines to ``raw_lines``; returns this call's rows.
    """
    start_index = len(raw_lines)
    duration_ms = max(1, int(duration_s * 1000.0))
    update_period_s = 1.0 / max(1.0, float(update_hz))
    deadline = time.monotonic() + max(0.0, float(duration_s))
    latest_row: dict[str, str] | None = None
    while time.monotonic() < deadline:
        a_cmd, b_cmd = command_fn(latest_row)
        _write_line(
            handle,
            (
                f"USB_DRIVE_LIVE_SET seq={seq} a={float(a_cmd):.3f} b={float(b_cmd):.3f} "
                f"duration_ms={duration_ms} ttl_ms={int(ttl_ms)}"
            ),
        )
        row = wait_for_row(
            handle,
            raw_lines,
            lambda r: telemetry.event(r) in {"ACTIVE", "REJECT"} or "MOTOR_TRACE" in str(r.get("_raw", "")),
            min(update_period_s, event_timeout_s),
            verbose_raw=verbose_raw,
        )
        if row is not None:
            latest_row = row
            if telemetry.event(row) == "REJECT":
                break
        else:
            time.sleep(min(update_period_s, 0.05))
    # KO: 루프 종료 후 반드시 명시적 LIVE_STOP -- TTL 만료를 기다리지 않고 즉시 정지시킨다.
    _write_line(handle, f"USB_DRIVE_LIVE_STOP seq={seq}")
    wait_for_event(handle, raw_lines, STOP_CONFIRM_EVENTS, event_timeout_s, verbose_raw=verbose_raw)
    return serial_rows(raw_lines, start_index)
