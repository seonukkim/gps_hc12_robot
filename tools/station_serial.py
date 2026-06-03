"""DTR/RTS-safe serial open helper shared by the station-side diagnostic tools.

Background: on a Mac temporary station, some USB-Serial bridges reset or report
``OSError Errno 6 Device not configured`` when DTR/RTS toggle on open (the
pyserial default asserts DTR/RTS). A manual test succeeded only with hardware
flow control off and DTR/RTS forced low. ``safe_open_serial`` reproduces that
known-good sequence: ``rtscts=False``, ``dsrdtr=False``, a bounded
``write_timeout``, and DTR/RTS forced low both pre- and post-open.

The pure helpers (``serial_kwargs``, ``control_line_state``, ``mode_should_set``,
``mode_level``, ``apply_control_lines``) are importable and unit-tested without
pyserial installed.
"""

from __future__ import annotations

from typing import Any

# Valid CLI values for --dtr / --rts. "default" means: do not touch the line.
DTR_RTS_MODES = ("low", "high", "default")


def mode_should_set(mode: str) -> bool:
    """True if the mode requests an explicit line level ("low"/"high")."""
    return mode in ("low", "high")


def mode_level(mode: str) -> bool:
    """Logical line level for the mode: "low" -> False, "high" -> True."""
    return mode == "high"


def serial_kwargs(
    baud: int, *, write_timeout_s: float = 1.0, read_timeout_s: float = 0.2
) -> dict[str, Any]:
    """pyserial constructor kwargs for a DTR/RTS-safe station port.

    Hardware flow control is disabled so the adapter cannot stall or auto-reset on
    DTR/RTS handshaking, and writes are bounded by ``write_timeout``.
    """
    return {
        "baudrate": baud,
        "timeout": read_timeout_s,
        "write_timeout": write_timeout_s,
        "rtscts": False,
        "dsrdtr": False,
    }


def control_line_state(dtr: str, rts: str) -> dict[str, Any]:
    """Structured control-line state for status/summary printing."""
    return {
        "dtr_mode": dtr,
        "rts_mode": rts,
        "rtscts": False,
        "dsrdtr": False,
    }


def apply_control_lines(ser: Any, dtr: str, rts: str) -> list[str]:
    """Force DTR/RTS per mode on an open port. Returns non-fatal warnings.

    Adapters/platforms that cannot set DTR/RTS produce a warning string instead of
    raising, so a missing control line never fails the diagnosis.
    """
    warnings: list[str] = []
    if mode_should_set(dtr):
        try:
            ser.dtr = mode_level(dtr)
        except (OSError, ValueError, AttributeError, NotImplementedError) as exc:
            warnings.append(f"set_dtr_unsupported:{type(exc).__name__}")
    if mode_should_set(rts):
        try:
            ser.rts = mode_level(rts)
        except (OSError, ValueError, AttributeError, NotImplementedError) as exc:
            warnings.append(f"set_rts_unsupported:{type(exc).__name__}")
    return warnings


def safe_open_serial(
    port: str,
    baud: int,
    *,
    dtr: str = "low",
    rts: str = "low",
    write_timeout_s: float = 1.0,
    read_timeout_s: float = 0.2,
) -> tuple[Any, dict[str, Any]]:
    """Open a station serial port the known-good way and return (serial, state).

    Sequence: build a closed ``Serial`` with HW flow control off, pre-set DTR/RTS
    (so ``open()`` does not pulse them where the backend supports it), open, then
    force DTR/RTS again per ``--dtr``/``--rts``. Real open errors propagate so the
    caller can classify/reconnect.
    """
    import serial  # lazy import so --help works without pyserial

    ser = serial.Serial()
    ser.port = port
    for key, value in serial_kwargs(
        baud, write_timeout_s=write_timeout_s, read_timeout_s=read_timeout_s
    ).items():
        setattr(ser, key, value)

    # Best-effort: set the desired DTR/RTS before opening so the open does not emit
    # a reset pulse on adapters that latch the pre-open state.
    if mode_should_set(dtr):
        try:
            ser.dtr = mode_level(dtr)
        except (OSError, ValueError, AttributeError, NotImplementedError):
            pass
    if mode_should_set(rts):
        try:
            ser.rts = mode_level(rts)
        except (OSError, ValueError, AttributeError, NotImplementedError):
            pass

    ser.open()

    warnings = apply_control_lines(ser, dtr, rts)
    state = control_line_state(dtr, rts)
    state["warnings"] = warnings
    return ser, state
