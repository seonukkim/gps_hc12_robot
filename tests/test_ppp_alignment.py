"""Tests for tools.physical_path_planning.alignment.

Pure helpers are tested directly. The serial-facing ``align_heading`` routine is
exercised with a *time-scheduled* fake serial: ``readline()`` yields each line
only after its wall-clock offset has elapsed (``b""`` before then). This matches
the deadline-bound read loops in ``executor``/``alignment`` -- a probe wait that
runs for its full ``probe_duration_s`` must not over-consume the queue intended
for the post-probe GPS read.
"""
from __future__ import annotations

import time

import pytest

from tools.physical_path_planning import alignment, geometry

START_LAT, START_LON = 37.5665000, 126.9780000


def _displaced(east_m: float, north_m: float) -> tuple[float, float]:
    """Lat/lon of a point ``(east_m, north_m)`` metres from the shared start."""
    end = geometry.local_to_latlon(
        geometry.GeoPoint(START_LAT, START_LON), geometry.LocalPoint(east_m, north_m)
    )
    return end.lat, end.lon


def _hb(lat: float | None = None, lon: float | None = None, yaw: float | None = None, event: str = "HEARTBEAT") -> str:
    """Build one USBDBG telemetry line (parsed by the simple key=value regex)."""
    parts = [f"event={event}"]
    if lat is not None:
        parts.append(f"current_lat={lat:.7f}")
    if lon is not None:
        parts.append(f"current_lon={lon:.7f}")
    if yaw is not None:
        parts.append(f"imu_relative_yaw_deg={yaw:.4f}")
    return " ".join(parts)


class TimedFakeSerial:
    """Serial fake whose readline() reveals each scheduled line only after its
    wall-clock offset has elapsed, returning b"" before then."""

    def __init__(self, schedule: list[tuple[float, str]]):
        self._schedule = sorted(((float(o), str(t)) for o, t in schedule), key=lambda x: x[0])
        self._idx = 0
        self._start: float | None = None
        self.written: list[str] = []

    def _elapsed(self) -> float:
        if self._start is None:
            self._start = time.monotonic()
        return time.monotonic() - self._start

    def readline(self) -> bytes:
        elapsed = self._elapsed()
        if self._idx < len(self._schedule):
            offset, text = self._schedule[self._idx]
            if elapsed >= offset:
                self._idx += 1
                return (text + "\n").encode("ascii")
        return b""

    def write(self, data: object) -> None:
        if isinstance(data, (bytes, bytearray)):
            self.written.append(bytes(data).decode("ascii", errors="replace"))
        else:
            self.written.append(str(data))

    def flush(self) -> None:  # pragma: no cover - trivial
        pass


# Short, fast probe timings; loops are wall-clock bound so keep durations tiny.
PROBE_KW = dict(
    strategy="gps_probe",
    probe_a=0.25,
    probe_duration_s=0.02,
    update_hz=50.0,
    event_timeout_s=0.1,
    heartbeat_timeout_s=1.0,
    chunk_ms=10,
    ttl_ms=100,
    max_turn_duration_s=1.0,
    verbose_raw=False,
)


# --- Pure helpers -------------------------------------------------------------


def test_first_segment_target_heading_returns_first():
    segments = [{"target_heading_deg": 42.0}, {"target_heading_deg": 7.0}]
    assert alignment.first_segment_target_heading(segments) == 42.0


def test_first_segment_target_heading_empty_raises():
    with pytest.raises(ValueError):
        alignment.first_segment_target_heading([])


def test_estimate_heading_north_is_90():
    end_lat, end_lon = _displaced(0.0, 0.6)
    heading, dist = alignment.estimate_heading_deg(START_LAT, START_LON, end_lat, end_lon)
    assert abs(heading - 90.0) < 1.0
    assert dist == pytest.approx(0.6, abs=0.05)


def test_estimate_heading_east_is_0():
    end_lat, end_lon = _displaced(0.6, 0.0)
    heading, dist = alignment.estimate_heading_deg(START_LAT, START_LON, end_lat, end_lon)
    assert abs(heading - 0.0) < 1.0
    assert dist == pytest.approx(0.6, abs=0.05)


def test_shortest_heading_error_wraps_across_180():
    assert alignment.shortest_heading_error_deg(170.0, -170.0) == pytest.approx(-20.0)
    assert alignment.shortest_heading_error_deg(10.0, 350.0) == pytest.approx(20.0)


def test_select_turn_direction_from_error_sign():
    assert alignment.select_turn_direction(5.0) == "left"
    assert alignment.select_turn_direction(-5.0) == "right"
    assert alignment.select_turn_direction(0.0) == "right"


def test_turn_b_command_picks_side():
    assert alignment.turn_b_command("left", 0.24, -0.12) == 0.24
    assert alignment.turn_b_command("right", 0.24, -0.12) == -0.12


def test_remaining_heading_error_reduces_toward_zero():
    assert alignment.remaining_heading_error_deg(30.0, 20.0) == pytest.approx(10.0)
    assert alignment.remaining_heading_error_deg(-30.0, 20.0) == pytest.approx(-10.0)
    # Overshoot: rotating 35 deg past a 30 deg error leaves -5 deg.
    assert alignment.remaining_heading_error_deg(30.0, 35.0) == pytest.approx(-5.0)


def test_within_tolerance_uses_magnitude():
    assert alignment.within_tolerance(7.0, 8.0) is True
    assert alignment.within_tolerance(-8.0, 8.0) is True
    assert alignment.within_tolerance(9.0, 8.0) is False


def test_plan_alignment_displacement_too_small():
    d = alignment.plan_alignment(
        target_heading_deg=90.0, current_heading_deg=0.0, probe_distance_m=0.1,
        min_probe_distance_m=0.3, heading_tolerance_deg=8.0, turn_b_left=0.24, turn_b_right=-0.12,
    )
    assert d["ok"] is False
    assert d["reason"] == "PROBE_GPS_DISPLACEMENT_TOO_SMALL"
    assert d["needs_turn"] is False


def test_plan_alignment_already_aligned():
    d = alignment.plan_alignment(
        target_heading_deg=90.0, current_heading_deg=85.0, probe_distance_m=0.5,
        min_probe_distance_m=0.3, heading_tolerance_deg=8.0, turn_b_left=0.24, turn_b_right=-0.12,
    )
    assert d["ok"] is True
    assert d["reason"] == "ALREADY_ALIGNED"
    assert d["needs_turn"] is False
    assert d["initial_heading_error_deg"] == pytest.approx(5.0)


def test_plan_alignment_turn_required_left():
    d = alignment.plan_alignment(
        target_heading_deg=120.0, current_heading_deg=80.0, probe_distance_m=0.5,
        min_probe_distance_m=0.3, heading_tolerance_deg=8.0, turn_b_left=0.24, turn_b_right=-0.12,
    )
    assert d["ok"] is True
    assert d["reason"] == "TURN_REQUIRED"
    assert d["needs_turn"] is True
    assert d["turn_direction"] == "left"
    assert d["turn_b_cmd"] == 0.24


def test_plan_alignment_turn_required_right():
    d = alignment.plan_alignment(
        target_heading_deg=40.0, current_heading_deg=80.0, probe_distance_m=0.5,
        min_probe_distance_m=0.3, heading_tolerance_deg=8.0, turn_b_left=0.24, turn_b_right=-0.12,
    )
    assert d["turn_direction"] == "right"
    assert d["turn_b_cmd"] == -0.12


# --- Serial-facing align_heading ----------------------------------------------


def test_align_skip_no_handle():
    segments = [{"target_heading_deg": 42.0}]
    result, trace = alignment.align_heading(None, segments=segments, strategy="skip", verbose_raw=False)
    assert result["reason"] == "ALIGNMENT_SKIPPED"
    assert result["alignment_success"] is True
    assert result["ready_for_execute_plan"] is True
    assert result["aligned_yaw_deg"] is None
    assert result["ready_for_full_path_following"] is False
    assert trace == []


def test_align_user_confirmed_captures_yaw():
    segments = [{"target_heading_deg": 42.0}]
    fake = TimedFakeSerial([(0.0, _hb(yaw=33.0))])
    prompts: list[str] = []
    result, _ = alignment.align_heading(
        fake, segments=segments, strategy="user_confirmed",
        input_fn=lambda p: (prompts.append(p), "")[1],
        heartbeat_timeout_s=1.0, verbose_raw=False,
    )
    assert prompts, "operator should be prompted to point the rover"
    assert result["reason"] == "USER_CONFIRMED_HEADING"
    assert result["aligned_yaw_deg"] == 33.0
    assert result["alignment_success"] is True
    assert result["ready_for_execute_plan"] is True
    assert result["ready_for_full_path_following"] is False


def test_align_user_confirmed_no_imu():
    segments = [{"target_heading_deg": 42.0}]
    fake = TimedFakeSerial([(0.0, _hb(lat=START_LAT, lon=START_LON))])
    result, _ = alignment.align_heading(
        fake, segments=segments, strategy="user_confirmed",
        input_fn=lambda p: "", heartbeat_timeout_s=1.0, verbose_raw=False,
    )
    assert result["reason"] == "USER_CONFIRMED_HEADING_NO_IMU"
    assert result["aligned_yaw_deg"] is None
    assert result["alignment_success"] is True


def test_align_gps_probe_unavailable_before():
    segments = [{"target_heading_deg": 90.0}]
    fake = TimedFakeSerial([(0.0, _hb(yaw=0.0))])  # heartbeat without GPS
    result, _ = alignment.align_heading(fake, segments=segments, **PROBE_KW)
    assert result["reason"] == "PROBE_GPS_UNAVAILABLE_BEFORE"
    assert result["alignment_success"] is False
    assert result["ready_for_full_path_following"] is False


def test_align_gps_probe_already_aligned():
    end_lat, end_lon = _displaced(0.0, 0.6)
    target, _ = alignment.estimate_heading_deg(START_LAT, START_LON, end_lat, end_lon)
    segments = [{"target_heading_deg": target}]
    fake = TimedFakeSerial([
        (0.0, _hb(START_LAT, START_LON, yaw=0.0)),
        (0.20, _hb(end_lat, end_lon, yaw=12.0)),
    ])
    result, _ = alignment.align_heading(fake, segments=segments, **PROBE_KW)
    assert result["reason"] == "ALREADY_ALIGNED"
    assert result["alignment_success"] is True
    assert result["ready_for_execute_plan"] is True
    assert result["turn_direction"] == "none"
    assert result["turn_duration_ms"] == 0
    assert result["aligned_yaw_deg"] == 12.0
    assert abs(result["initial_heading_error_deg"]) <= 8.0
    assert result["ready_for_full_path_following"] is False


def test_align_gps_probe_displacement_too_small():
    end_lat, end_lon = _displaced(0.0, 0.05)
    segments = [{"target_heading_deg": 90.0}]
    fake = TimedFakeSerial([
        (0.0, _hb(START_LAT, START_LON, yaw=0.0)),
        (0.20, _hb(end_lat, end_lon, yaw=0.0)),
    ])
    result, _ = alignment.align_heading(fake, segments=segments, **PROBE_KW)
    assert result["reason"] == "PROBE_GPS_DISPLACEMENT_TOO_SMALL"
    assert result["alignment_success"] is False
    assert result["ready_for_execute_plan"] is False
    assert result["ready_for_full_path_following"] is False


def test_align_gps_probe_imu_unavailable_for_turn():
    end_lat, end_lon = _displaced(0.0, 0.6)
    est, _ = alignment.estimate_heading_deg(START_LAT, START_LON, end_lat, end_lon)
    segments = [{"target_heading_deg": est + 30.0}]
    fake = TimedFakeSerial([
        (0.0, _hb(START_LAT, START_LON, yaw=0.0)),
        (0.20, _hb(end_lat, end_lon)),  # GPS present, IMU yaw missing
    ])
    result, _ = alignment.align_heading(fake, segments=segments, **PROBE_KW)
    assert result["reason"] == "IMU_UNAVAILABLE_FOR_ALIGNMENT"
    assert result["alignment_success"] is False
    assert result["ready_for_full_path_following"] is False


def test_align_gps_probe_turn_reduces_error():
    end_lat, end_lon = _displaced(0.0, 0.6)
    est, _ = alignment.estimate_heading_deg(START_LAT, START_LON, end_lat, end_lon)
    segments = [{"target_heading_deg": est + 30.0}]
    fake = TimedFakeSerial([
        (0.0, _hb(START_LAT, START_LON, yaw=0.0)),
        (0.20, _hb(end_lat, end_lon, yaw=0.0)),          # yaw_turn_start = 0
        (0.30, _hb(end_lat, end_lon, yaw=10.0, event="ACTIVE")),
        (0.34, _hb(end_lat, end_lon, yaw=20.0, event="ACTIVE")),
        (0.38, _hb(end_lat, end_lon, yaw=26.0, event="ACTIVE")),
    ])
    result, _ = alignment.align_heading(fake, segments=segments, **PROBE_KW)
    assert result["reason"] == "ALIGNED"
    assert result["alignment_success"] is True
    assert result["turn_direction"] == "left"
    assert result["turn_b_cmd"] == round(alignment.DEFAULT_TURN_B_LEFT, 3)
    assert abs(result["final_heading_error_deg"]) <= 8.0
    assert result["aligned_yaw_deg"] == 26.0
    assert result["ready_for_full_path_following"] is False
