"""Discrete guarded pulse execution: the ARM -> ACK -> STOP finite-state machine.

This owns the serial-facing mechanics: line-reading
wait loops and the four-step pulse handshake. ``send_pulse`` issues exactly one
guarded pulse (arm, command, await pulse-complete, stop) and returns the rows
captured during that pulse window. Higher layers own the reporting / IMU / GPS
logic around each pulse.

The firmware still owns the real motor-output safety gate; this module only
sequences commands and waits for the matching telemetry events.
"""
from __future__ import annotations

import time
from typing import Callable, Sequence

from tools.physical_path_planning import safety, telemetry

# FSM transition event sets (a REJECT at the arm/command step ends the pulse early).
ARM_EVENTS = {"ARM", "REJECT"}
COMMAND_ACK_EVENTS = {"ACK", "REJECT"}
STOP_CONFIRM_EVENTS = {"STOP"}
# Pulse-complete events are shared with the safety layer's stop-class set.
PULSE_COMPLETE_EVENTS = safety.STOP_EVENTS


def serial_rows(raw_lines: Sequence[str], start_index: int = 0) -> list[dict[str, str]]:
    return telemetry.parse_usbdbg_rows("\n".join(raw_lines[start_index:]))


def _write_line(handle: object, text: object) -> None:
    handle.write((str(text) + "\n").encode("ascii"))  # type: ignore[attr-defined]
    handle.flush()  # type: ignore[attr-defined]


def wait_for_row(
    handle: object,
    raw_lines: list[str],
    predicate: Callable[[dict[str, str]], bool],
    timeout_s: float,
    *,
    verbose_raw: bool = True,
) -> dict[str, str] | None:
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
    """
    pulse_start = len(raw_lines)
    _write_line(handle, planned["arm_command_text"])
    wait_for_event(handle, raw_lines, ARM_EVENTS, event_timeout_s, verbose_raw=verbose_raw)
    command_text = (
        planned.get("command_text")
        or planned.get("station_drive_command_text")
        or planned.get("stage20_command_text")
    )
    _write_line(handle, command_text)
    wait_for_event(handle, raw_lines, COMMAND_ACK_EVENTS, event_timeout_s, verbose_raw=verbose_raw)
    pulse_ms = int(planned["pulse_ms"])
    wait_for_event(
        handle,
        raw_lines,
        PULSE_COMPLETE_EVENTS,
        max(event_timeout_s, pulse_ms / 1000.0 + 1.0),
        verbose_raw=verbose_raw,
    )
    _write_line(handle, planned["stop_command_text"])
    wait_for_event(handle, raw_lines, STOP_CONFIRM_EVENTS, event_timeout_s, verbose_raw=verbose_raw)
    return serial_rows(raw_lines, pulse_start)
