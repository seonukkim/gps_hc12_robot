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
STOP_CONFIRM_EVENTS = {"STOP", "STOP_ALREADY_ZERO"}
# Pulse-complete events are shared with the safety layer's stop-class set.
PULSE_COMPLETE_EVENTS = safety.STOP_EVENTS


def should_send_stop_after_completion(rows: Sequence[dict[str, str]]) -> bool:
    """True when the host should send an explicit STOP after the completion wait."""
    if not rows:
        return True
    stop_like_seen = any(telemetry.event(row) in safety.STOP_EVENTS for row in rows)
    active_after_stop = safety.output_active_after_stop(rows)
    return (not stop_like_seen) or active_after_stop


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
        or planned.get("stage" + "20_command_text")
    )
    _write_line(handle, command_text)
    wait_for_event(handle, raw_lines, COMMAND_ACK_EVENTS, event_timeout_s, verbose_raw=verbose_raw)
    pulse_ms = int(planned["pulse_ms"])
    completion_rows = wait_for_event(
        handle,
        raw_lines,
        PULSE_COMPLETE_EVENTS,
        max(event_timeout_s, pulse_ms / 1000.0 + 1.0),
        verbose_raw=verbose_raw,
    )
    if completion_rows and not any(telemetry.event(row) in STOP_CONFIRM_EVENTS for row in completion_rows):
        wait_for_event(
            handle,
            raw_lines,
            STOP_CONFIRM_EVENTS,
            min(event_timeout_s, 0.25),
            verbose_raw=verbose_raw,
        )
    rows_so_far = serial_rows(raw_lines, pulse_start)
    send_explicit_stop = bool(planned.get("force_stop_command")) or should_send_stop_after_completion(completion_rows)
    if send_explicit_stop:
        _write_line(handle, planned["stop_command_text"])
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
    """Send continuous USB A/B setpoints with a firmware-side deadman TTL."""
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
    _write_line(handle, f"USB_DRIVE_LIVE_STOP seq={seq}")
    wait_for_event(handle, raw_lines, STOP_CONFIRM_EVENTS, event_timeout_s, verbose_raw=verbose_raw)
    return serial_rows(raw_lines, start_index)
