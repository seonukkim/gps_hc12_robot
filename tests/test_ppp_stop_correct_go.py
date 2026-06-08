"""Tests for the stop_correct_go controller mode (pure helpers + mock-serial loop).

The pure decision helpers are exercised directly; the supervised loop is driven
with a scripted fake serial and ``settle_after_move_ms=0`` /
``telemetry_stabilize_ms=0`` so no wall-clock sleeping occurs.
"""

from __future__ import annotations

from tools.physical_path_planning import controller, geometry


# --- scripted serial -----------------------------------------------------------


class FakeSerial:
    """Replays scripted telemetry lines and records every written command."""

    def __init__(self, responses: list[bytes]) -> None:
        self._responses = list(responses)
        self.writes: list[str] = []

    def write(self, data: bytes) -> int:
        self.writes.append(data.decode("ascii").strip())
        return len(data)

    def flush(self) -> None:
        pass

    def readline(self) -> bytes:
        return self._responses.pop(0) if self._responses else b""


def _hb(
    lat: float | None = None,
    lon: float | None = None,
    *,
    imu_yaw_deg: float = 90.0,
    rc_ok: bool = True,
    neutral_ok: bool = True,
    gps_block_reason: str = "OK",
    with_gps: bool = True,
    with_imu: bool = True,
) -> bytes:
    parts = [
        "USB_PULSE_TEST event=HEARTBEAT usb_pulse_test_mode=true",
        f"rc_ok={str(rc_ok).lower()} neutral_ok={str(neutral_ok).lower()}",
        "physical_output_active=false",
        f"gps_block_reason={gps_block_reason}",
    ]
    if with_gps and lat is not None and lon is not None:
        parts.append(f"gps_lat={lat:.7f} gps_lon={lon:.7f}")
    if with_imu:
        parts.append(f"imu_relative_yaw_deg={imu_yaw_deg:.1f}")
    return (" ".join(parts) + "\n").encode("ascii")


# A guarded pulse: ARM -> ACK -> PULSE_COMPLETE -> STOP (final motors zeroed).
_PULSE_OK = [
    b"USB_PULSE_TEST event=ARM\n",
    b"USB_PULSE_TEST event=ACK\n",
    b"USB_PULSE_TEST event=PULSE_COMPLETE\n",
    b"USB_PULSE_TEST event=STOP final_left_cmd=0.000 final_right_cmd=0.000 physical_output_active=false\n",
]


def _north_lane(length_m: float = 1.0) -> dict[str, object]:
    # Lane running due north (ENU heading 90 deg) from the origin; +Y is north.
    return {
        "segment_index": 1,
        "segment_type": "forward_lane",
        "start_x_m": 0.0,
        "start_y_m": 0.0,
        "end_x_m": 0.0,
        "end_y_m": length_m,
        "length_m": length_m,
        "target_heading_deg": 90.0,
        "expected_motion_direction": "forward",
        "pulse_budget": 1,
    }


# --- pure helper: sensor source ------------------------------------------------


def test_sensor_source_uses_both_when_available() -> None:
    out = controller.stop_correct_go_sensor_source(gps_valid=True, imu_valid=True)
    assert out["ok"] is True
    assert out["source"] == "gps_imu"
    assert out["fallback_used"] is False


def test_sensor_source_single_sensor_labels() -> None:
    assert controller.stop_correct_go_sensor_source(gps_valid=True, imu_valid=False)["source"] == "gps"
    assert controller.stop_correct_go_sensor_source(gps_valid=False, imu_valid=True)["source"] == "imu"


def test_sensor_source_aborts_when_both_gone_and_fallback_disallowed() -> None:
    out = controller.stop_correct_go_sensor_source(
        gps_valid=False, imu_valid=False, trust_mode="imu_gps_first", allow_calibration_fallback=False
    )
    assert out["ok"] is False
    assert out["reason"] == "SENSOR_UNAVAILABLE"


def test_sensor_source_calibration_fallback_when_allowed_or_trusted() -> None:
    explicit = controller.stop_correct_go_sensor_source(
        gps_valid=False, imu_valid=False, trust_mode="imu_gps_first", allow_calibration_fallback=True
    )
    assert explicit["ok"] is True
    assert explicit["source"] == "calibration"
    assert explicit["fallback_used"] is True
    trusted = controller.stop_correct_go_sensor_source(
        gps_valid=False, imu_valid=False, trust_mode="calibration_fallback", allow_calibration_fallback=False
    )
    assert trusted["ok"] is True
    assert trusted["fallback_used"] is True


# --- pure helper: heading decision ---------------------------------------------


def test_heading_decision_turns_left_for_positive_error() -> None:
    out = controller.stop_correct_go_heading_decision(
        heading_error_deg=20.0, threshold_deg=8.0, imu_valid=True, b_left=0.24, b_right=-0.12
    )
    assert out["needs_correction"] is True
    assert out["turn_direction"] == "left"
    assert out["correction_b_cmd"] == 0.24  # B > 0 is left, per the locked A/B mapping


def test_heading_decision_turns_right_for_negative_error() -> None:
    out = controller.stop_correct_go_heading_decision(
        heading_error_deg=-20.0, threshold_deg=8.0, imu_valid=True, b_left=0.24, b_right=-0.12
    )
    assert out["turn_direction"] == "right"
    assert out["correction_b_cmd"] == -0.12  # B < 0 is right


def test_heading_decision_skipped_below_threshold_or_without_imu() -> None:
    below = controller.stop_correct_go_heading_decision(
        heading_error_deg=5.0, threshold_deg=8.0, imu_valid=True, b_left=0.24, b_right=-0.12
    )
    assert below["needs_correction"] is False
    no_imu = controller.stop_correct_go_heading_decision(
        heading_error_deg=40.0, threshold_deg=8.0, imu_valid=False, b_left=0.24, b_right=-0.12
    )
    assert no_imu["needs_correction"] is False


# --- pure helper: cross-track trim (B clamp +/-0.08) ---------------------------


def test_cross_track_trim_zero_below_threshold() -> None:
    assert (
        controller.stop_correct_go_cross_track_trim(cross_track_error_m=0.2, threshold_m=0.35) == 0.0
    )


def test_cross_track_trim_clamped_to_max_and_signed() -> None:
    # Large positive cross-track -> positive B, clamped to +max (steer back to line).
    high = controller.stop_correct_go_cross_track_trim(
        cross_track_error_m=5.0, threshold_m=0.35, k_cross_track=0.20, max_correction_b=0.08
    )
    assert high == 0.08
    low = controller.stop_correct_go_cross_track_trim(
        cross_track_error_m=-5.0, threshold_m=0.35, k_cross_track=0.20, max_correction_b=0.08
    )
    assert low == -0.08


def test_remaining_turn_error_shrinks_toward_zero() -> None:
    # Started 20 deg off, rotated 12 deg toward target -> 8 deg remains (same sign).
    assert controller.remaining_turn_error_deg(20.0, 12.0) == 8.0
    assert controller.remaining_turn_error_deg(-20.0, 12.0) == -8.0


# --- loop: clean single-chunk lane completion ----------------------------------


def test_run_stop_correct_go_completes_one_lane_clean() -> None:
    handle = FakeSerial(
        [
            _hb(35.0, 129.0, imu_yaw_deg=90.0),  # preflight heartbeat
            *_PULSE_OK,
            _hb(35.0000100, 129.0, imu_yaw_deg=90.0),  # settled pose ~1.1 m north -> lane done
        ]
    )
    rows, _raw, abort_reason = controller.run_stop_correct_go(
        handle,
        segments=[_north_lane()],
        resolved_calibration=geometry.FALLBACK_RESOLVED_CALIBRATION,
        start_lat=35.0,
        start_lon=129.0,
        start_yaw_deg=90.0,
        goal_lat=35.0000100,
        goal_lon=129.0,
        settle_after_move_ms=0,
        telemetry_stabilize_ms=0,
        event_timeout_s=1.0,
        heartbeat_timeout_s=1.0,
    )

    assert abort_reason == "NONE"
    assert len(rows) == 1
    row = rows[0]
    assert row["valid_pulse"] is True
    assert row["drive_mode"] == "stop_correct_go"
    assert row["phase"] == "move"
    assert row["sensor_source"] == "gps_imu"
    assert row["gps_valid"] is True
    assert row["fallback_used"] is False
    assert row["ready_for_full_path_following"] is False
    # MOVE keeps the calibrated forward A and never lowers it; B starts at neutral.
    assert handle.writes[0].startswith("USB_PULSE_TEST_ARM")
    assert handle.writes[1].startswith("USB_PULSE_TEST_CMD") and "a=0.300" in handle.writes[1]
    assert "b=0.000" in handle.writes[1]
    assert handle.writes[2].startswith("USB_PULSE_TEST_STOP")


def test_run_stop_correct_go_aborts_sensor_unavailable() -> None:
    handle = FakeSerial(
        [
            _hb(35.0, 129.0, imu_yaw_deg=90.0),  # preflight healthy -> move proceeds
            *_PULSE_OK,
            _hb(gps_block_reason="BAD_HDOP", with_gps=False, with_imu=False),  # both sensors gone
        ]
    )
    rows, _raw, abort_reason = controller.run_stop_correct_go(
        handle,
        segments=[_north_lane()],
        resolved_calibration=geometry.FALLBACK_RESOLVED_CALIBRATION,
        start_lat=35.0,
        start_lon=129.0,
        start_yaw_deg=90.0,
        goal_lat=35.0000100,
        goal_lon=129.0,
        settle_after_move_ms=0,
        telemetry_stabilize_ms=0,
        sensor_trust_mode="imu_gps_first",
        allow_calibration_fallback=False,
        event_timeout_s=1.0,
        heartbeat_timeout_s=1.0,
    )

    assert abort_reason == "SENSOR_UNAVAILABLE"
    assert rows == []  # aborted before emitting a row for the unusable cycle


def test_run_stop_correct_go_dead_reckons_when_gps_degraded() -> None:
    # Short lane so one calibrated dead-reckon advance (~0.30 m) completes it.
    handle = FakeSerial(
        [
            _hb(35.0, 129.0, imu_yaw_deg=90.0),  # preflight healthy -> move proceeds
            *_PULSE_OK,
            _hb(35.0, 129.0, imu_yaw_deg=90.0, gps_block_reason="BAD_HDOP"),  # GPS degraded, IMU live
        ]
    )
    rows, _raw, abort_reason = controller.run_stop_correct_go(
        handle,
        segments=[_north_lane(length_m=0.2)],
        resolved_calibration=geometry.FALLBACK_RESOLVED_CALIBRATION,
        start_lat=35.0,
        start_lon=129.0,
        start_yaw_deg=90.0,
        goal_lat=35.0000100,
        goal_lon=129.0,
        settle_after_move_ms=0,
        telemetry_stabilize_ms=0,
        allow_calibration_fallback=True,
        event_timeout_s=1.0,
        heartbeat_timeout_s=1.0,
    )

    assert abort_reason == "NONE"
    assert len(rows) == 1
    row = rows[0]
    assert row["valid_pulse"] is True
    assert row["gps_valid"] is False
    assert row["sensor_source"] == "imu"  # IMU still drives heading; GPS dead-reckoned
    assert row["fallback_used"] is False  # IMU available -> not a calibration-only fallback
    assert float(row["along_track_progress_m"]) > 0.0


def test_build_stop_correct_go_summary_keeps_ready_false_and_counters() -> None:
    handle = FakeSerial(
        [
            _hb(35.0, 129.0, imu_yaw_deg=90.0),
            *_PULSE_OK,
            _hb(35.0000100, 129.0, imu_yaw_deg=90.0),
        ]
    )
    rows, _raw, abort_reason = controller.run_stop_correct_go(
        handle,
        segments=[_north_lane()],
        resolved_calibration=geometry.FALLBACK_RESOLVED_CALIBRATION,
        start_lat=35.0,
        start_lon=129.0,
        start_yaw_deg=90.0,
        goal_lat=35.0000100,
        goal_lon=129.0,
        settle_after_move_ms=0,
        telemetry_stabilize_ms=0,
        event_timeout_s=1.0,
        heartbeat_timeout_s=1.0,
    )
    summary = controller.build_stop_correct_go_summary(
        rows,
        start_lat=35.0,
        start_lon=129.0,
        goal_lat=35.0000100,
        goal_lon=129.0,
        goal_distance_m=1.11,
        fallback_to_repeated_pulses=False,
        sensor_trust_mode="imu_gps_first",
        allow_calibration_fallback=True,
        abort_reason=abort_reason,
    )
    assert summary["path_control_mode"] == "stop_correct_go"
    assert summary["sensor_trust_mode"] == "imu_gps_first"
    assert summary["heading_correction_count"] == 0
    assert summary["sensor_fallback_used_count"] == 0
    assert summary["ready_for_full_path_following"] is False
