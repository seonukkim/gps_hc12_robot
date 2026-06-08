"""Guarded-motion safety predicates over USBDBG telemetry rows.

These are the abort/wait conditions that were copy-pasted across the stage
runners, lifted here as named pure functions so the executor and controller
share one definition. Every predicate takes already-parsed rows (or a single
row) and returns a plain bool / reason string -- no serial, no side effects --
which makes them directly unit-testable with mock telemetry.

Semantics mirror the legacy stage30 inline checks exactly (parity is asserted
in ``tests/test_ppp_safety.py``) so stage30 can delegate without behavior drift.
"""
from __future__ import annotations

from typing import Sequence

from tools.physical_path_planning import telemetry

# Events that mark the controlled end of a pulse (motor output must be zero after).
STOP_EVENTS = {"STOP", "PULSE_COMPLETE", "PULSE_DONE"}

_ZERO_TOLERANCE = 1e-9


def latest_reject_reason(rows: Sequence[dict[str, str]]) -> str:
    """Most recent reject reason; prefers the stage20 channel, falls back to stage16."""
    reason = telemetry._latest(rows, "stage20_reject_reason", "")
    return reason if reason else telemetry._latest(rows, "stage16_reject_reason", "NONE")


def rc_invalid_abort(rows: Sequence[dict[str, str]]) -> bool:
    """True => an RC_INVALID reject was reported; an active pulse must hard-abort."""
    return latest_reject_reason(rows) == "RC_INVALID"


def missing_ack_or_stop_abort(rows: Sequence[dict[str, str]]) -> str | None:
    """Return ``"ACK_MISSING"`` / ``"STOP_MISSING"`` if the handshake is incomplete, else None."""
    if not any(telemetry.event(row) == "ACK" for row in rows):
        return "ACK_MISSING"
    if not any(telemetry.event(row) in STOP_EVENTS for row in rows):
        return "STOP_MISSING"
    return None


def output_active_after_stop(rows: Sequence[dict[str, str]]) -> bool:
    """True => a STOP row still reported physical output active (motors not cut)."""
    return any(
        telemetry.event(row) == "STOP" and telemetry.physical_output_active(row)
        for row in rows
    )


def nonzero_final_cmd(rows: Sequence[dict[str, str]]) -> bool:
    """True => the latest final left/right command is not (within tolerance) zero."""
    left = telemetry._optional_float(telemetry._latest(rows, "final_left_cmd", "0")) or 0.0
    right = telemetry._optional_float(telemetry._latest(rows, "final_right_cmd", "0")) or 0.0
    return abs(left) > _ZERO_TOLERANCE or abs(right) > _ZERO_TOLERANCE


def rc_neutral_wait(row: dict[str, str]) -> bool:
    """True => RC is not yet neutral/ready; the caller must keep waiting before pulsing.

    Ready means: RC link ok, sticks neutral, and physical output not already active.
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
    stage20 role/compat fields (callers add those if they need them).
    """
    return (
        telemetry.event(row) == "HEARTBEAT"
        and telemetry._parse_bool(row.get("rc_ok")) is True
        and telemetry._parse_bool(row.get("neutral_ok")) is True
        and telemetry._parse_bool(row.get("physical_output_active")) is False
        and telemetry._parse_bool(row.get("physical_path_following_enable")) is False
        and telemetry._parse_bool(row.get("allow_motor_output")) is False
    )
