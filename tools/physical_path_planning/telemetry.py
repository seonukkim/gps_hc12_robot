"""Shared USBDBG telemetry parsing and canonical row-field accessors.

This is the single home for the scalar coercion helpers (``_parse_bool``,
``_optional_float``, ``_latest``, ``_fmt``) and the named accessors for the
USBDBG telemetry fields that the executor / safety / controller layers read.
The raw line parser ``parse_usbdbg_rows`` is re-exported (not moved) from
``station_path_package_tracker``, which keeps its own large test surface.

Accessors take a single already-parsed row (``dict[str, str]``) and return a
normalized value, so call sites stop re-deriving field names and coercions.
"""
from __future__ import annotations

import math
from typing import Sequence

from tools import station_path_package_tracker

# Re-export the canonical raw parser; do not reimplement it here.
parse_usbdbg_rows = station_path_package_tracker.parse_usbdbg_rows


def _parse_bool(value: object, default: bool = False) -> bool:
    if value is None:
        return default
    return str(value).strip().lower() in {"1", "true", "yes", "y", "ok", "active"}


def _optional_float(value: object) -> float | None:
    if value is None:
        return None
    text = str(value).strip()
    if text.upper() in {"", "NA", "NAN", "NONE", "NULL"}:
        return None
    try:
        parsed = float(text)
    except ValueError:
        return None
    return parsed if math.isfinite(parsed) else None


def _latest(rows: Sequence[dict[str, str]], key: str, default: str = "NA") -> str:
    for row in reversed(rows):
        if key in row:
            return row[key]
    return default


def _fmt(value: float | None, digits: int = 6) -> str:
    return "NA" if value is None else f"{value:.{digits}f}"


# --- Canonical single-row field accessors -------------------------------------


def event(row: dict[str, str]) -> str:
    """Upper-cased ``event`` field (ARM / PULSE / ACK / STOP / ...), ``""`` if absent."""
    return str(row.get("event", "")).upper()


def final_left_cmd(row: dict[str, str]) -> float | None:
    return _optional_float(row.get("final_left_cmd"))


def final_right_cmd(row: dict[str, str]) -> float | None:
    return _optional_float(row.get("final_right_cmd"))


def physical_output_active(row: dict[str, str]) -> bool:
    return _parse_bool(row.get("physical_output_active"))


def imu_relative_yaw_deg(row: dict[str, str]) -> float | None:
    return _optional_float(row.get("imu_relative_yaw_deg"))


def gps_block_reason(row: dict[str, str], default: str = "NA") -> str:
    value = row.get("gps_block_reason")
    return default if value is None else str(value)


def gps_sats(row: dict[str, str]) -> float | None:
    return _optional_float(row.get("gps_sats"))


def gps_hdop(row: dict[str, str]) -> float | None:
    return _optional_float(row.get("gps_hdop"))
