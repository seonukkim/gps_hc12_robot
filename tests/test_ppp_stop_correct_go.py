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


# --- connector turns: target_angle_deg-aware multi-pulse + IMU stop ------------


def _small_left_turn_calibration(target_angle_deg: float = 30.0) -> dict[str, object]:
    cal = dict(geometry.FALLBACK_RESOLVED_CALIBRATION)
    cal["connector_mode_effective"] = "angle_calibrated"
    cal["turn_left_90"] = {
        "available": True,
        "a": 0.0,
        "b": 0.24,
        "ms": 600,
        "target_angle_deg": target_angle_deg,
        "source": "manual_small_pulse",
    }
    return cal


def _left_connector(target_heading_deg: float = 90.0) -> dict[str, object]:
    # Quarter-turn corner at the end of an east lane: rotate from east to north.
    return {
        "segment_index": 2,
        "segment_type": "path_connector",
        "start_x_m": 1.0,
        "start_y_m": 0.0,
        "end_x_m": 1.0,
        "end_y_m": 1.2,
        "length_m": 1.2,
        "target_heading_deg": target_heading_deg,
        "expected_motion_direction": "turn_left",
        "pulse_budget": 1,
    }


def test_connector_turn_planning_helpers() -> None:
    connector = {"target_angle_deg": 30.0}
    assert controller.per_pulse_turn_angle_deg(connector) == 30.0
    assert controller.per_pulse_turn_angle_deg(connector, turn_angle_policy="assume_90") == 90.0
    assert controller.per_pulse_turn_angle_deg(connector, turn_angle_override=45.0) == 45.0
    assert controller.per_pulse_turn_angle_deg({"target_angle_deg": None}) is None
    assert controller.connector_planned_pulses(90.0, 30.0) == 3
    assert controller.connector_planned_pulses(90.0, 90.0) == 1
    assert controller.connector_planned_pulses(-90.0, 30.0) == 3
    assert controller.connector_planned_pulses(90.0, None) == 1
    # IMU feedback earns a verified margin; open loop must stop at the plan.
    assert controller.connector_pulse_budget(90.0, 30.0, imu_available=True) == 5
    assert controller.connector_pulse_budget(90.0, 30.0, imu_available=False) == 3
    assert controller.connector_pulse_budget(90.0, 30.0, max_pulses=4, imu_available=True) == 4
    assert controller.connector_turn_angle_deg({"expected_motion_direction": "turn_left"}) == 90.0
    assert controller.connector_turn_angle_deg({"expected_motion_direction": "turn_right"}) == -90.0
    assert controller.connector_turn_angle_deg({"turn_angle_deg": "-90.0"}) == -90.0


def test_connector_small_pulse_repeats_until_imu_reports_90() -> None:
    # turn_left_90 physically turns ~30 deg per pulse: the corner must pulse
    # repeatedly and stop on measured yaw, not assume one pulse completed it.
    handle = FakeSerial(
        [
            _hb(35.0, 129.0, imu_yaw_deg=0.0),
            *_PULSE_OK,
            _hb(35.0, 129.0, imu_yaw_deg=30.0),
            _hb(35.0, 129.0, imu_yaw_deg=30.0),
            *_PULSE_OK,
            _hb(35.0, 129.0, imu_yaw_deg=60.0),
            _hb(35.0, 129.0, imu_yaw_deg=60.0),
            *_PULSE_OK,
            _hb(35.0, 129.0, imu_yaw_deg=88.0),
        ]
    )
    rows, _raw, abort_reason = controller.run_stop_correct_go(
        handle,
        segments=[_left_connector()],
        resolved_calibration=_small_left_turn_calibration(30.0),
        start_lat=35.0,
        start_lon=129.0,
        start_yaw_deg=None,
        goal_lat=35.0000100,
        goal_lon=129.0,
        settle_after_move_ms=0,
        telemetry_stabilize_ms=0,
        event_timeout_s=1.0,
        heartbeat_timeout_s=1.0,
    )

    assert abort_reason == "NONE"
    connector_rows = [r for r in rows if r.get("phase") == "connector"]
    assert len(connector_rows) == 3
    assert all(r["turn_pulse_budget"] == 5 for r in connector_rows)  # ceil(90/30)+2
    assert connector_rows[-1]["connector_turn_completed"] is True
    assert connector_rows[-1]["turn_measured_by_imu"] is True
    assert float(connector_rows[-1]["applied_turn_delta_deg"]) == 88.0
    cmd_writes = [w for w in handle.writes if w.startswith("USB_PULSE_TEST_CMD")]
    assert len(cmd_writes) == 3
    # Pivot pulses: calibrated B only, A stays zero.
    assert all("a=0.000" in w and "b=0.240" in w for w in cmd_writes)


def test_connector_without_imu_uses_open_loop_pulse_count_from_target_angle() -> None:
    responses: list[bytes] = []
    for _ in range(3):
        responses.append(_hb(35.0, 129.0, with_imu=False))
        responses.extend(_PULSE_OK)
        responses.append(_hb(35.0, 129.0, with_imu=False))
    handle = FakeSerial(responses)
    rows, _raw, abort_reason = controller.run_stop_correct_go(
        handle,
        segments=[_left_connector()],
        resolved_calibration=_small_left_turn_calibration(30.0),
        start_lat=35.0,
        start_lon=129.0,
        start_yaw_deg=None,
        goal_lat=35.0000100,
        goal_lon=129.0,
        settle_after_move_ms=0,
        telemetry_stabilize_ms=0,
        event_timeout_s=1.0,
        heartbeat_timeout_s=1.0,
    )

    assert abort_reason == "NONE"
    connector_rows = [r for r in rows if r.get("phase") == "connector"]
    assert len(connector_rows) == 3  # round-up of 90/30, no blind extras
    assert connector_rows[-1]["connector_turn_completed"] is True
    assert connector_rows[-1]["turn_measured_by_imu"] is False


def test_connector_rotation_loop_guard_stops_at_pulse_budget() -> None:
    # IMU reports no rotation at all (stalled motors): the turn must stop at the
    # budget instead of pulsing forever, and be reported as incomplete.
    responses: list[bytes] = []
    for _ in range(5):  # budget = min(6, ceil(90/30)+2) = 5
        responses.append(_hb(35.0, 129.0, imu_yaw_deg=0.0))
        responses.extend(_PULSE_OK)
        responses.append(_hb(35.0, 129.0, imu_yaw_deg=0.0))
    handle = FakeSerial(responses)
    rows, _raw, abort_reason = controller.run_stop_correct_go(
        handle,
        segments=[_left_connector()],
        resolved_calibration=_small_left_turn_calibration(30.0),
        start_lat=35.0,
        start_lon=129.0,
        start_yaw_deg=None,
        goal_lat=35.0000100,
        goal_lon=129.0,
        settle_after_move_ms=0,
        telemetry_stabilize_ms=0,
        event_timeout_s=1.0,
        heartbeat_timeout_s=1.0,
    )

    assert abort_reason == "NONE"
    connector_rows = [r for r in rows if r.get("phase") == "connector"]
    assert len(connector_rows) == 5
    assert connector_rows[-1]["connector_turn_completed"] is False
    summary = controller.build_stop_correct_go_summary(
        connector_rows,
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
    assert summary["connector_turn_count"] == 1
    assert summary["connector_pulse_count"] == 5
    assert summary["connector_incomplete_count"] == 1


# --- heading reference: mission chain vs legacy per-lane re-capture ------------


_EAST_LANE_1M = {
    "segment_index": 1,
    "segment_type": "forward_lane",
    "start_x_m": 0.0,
    "start_y_m": 0.0,
    "end_x_m": 1.0,
    "end_y_m": 0.0,
    "length_m": 1.0,
    "target_heading_deg": 0.0,
    "expected_motion_direction": "forward",
    "pulse_budget": 1,
}

_NORTH_LANE_AFTER_CORNER = {
    "segment_index": 3,
    "segment_type": "forward_lane",
    "start_x_m": 1.0,
    "start_y_m": 0.0,
    "end_x_m": 1.0,
    "end_y_m": 1.0,
    "length_m": 1.0,
    "target_heading_deg": 90.0,
    "expected_motion_direction": "forward",
    "pulse_budget": 1,
}

_LON_1M_EAST = 129.0000110  # ~1.0 m east of 129.0 at lat 35
_LAT_HALF_M_NORTH = 35.0000045
_LAT_1M_NORTH = 35.0000090


def _under_turned_corner_responses(*, with_correction_rows: bool) -> list[bytes]:
    """Lane east -> connector that only turns 40 of 90 deg -> lane north."""
    responses = [
        # lane 1, chunk 1: completes at the east end, yaw matches lane heading.
        _hb(35.0, 129.0, imu_yaw_deg=0.0),
        *_PULSE_OK,
        _hb(35.0, _LON_1M_EAST, imu_yaw_deg=0.0),
        # connector (budget forced to 1): under-turns to 40 deg.
        _hb(35.0, _LON_1M_EAST, imu_yaw_deg=0.0),
        *_PULSE_OK,
        _hb(35.0, _LON_1M_EAST, imu_yaw_deg=40.0),
        # lane 2, chunk 1: still pointing 40 deg while the lane needs 90.
        _hb(35.0, _LON_1M_EAST, imu_yaw_deg=40.0),
        *_PULSE_OK,
        _hb(_LAT_HALF_M_NORTH, _LON_1M_EAST, imu_yaw_deg=40.0),
    ]
    if with_correction_rows:
        responses.extend(
            [
                _hb(_LAT_HALF_M_NORTH, _LON_1M_EAST, imu_yaw_deg=88.0),  # turn feedback
                b"USB_PULSE_TEST event=STOP final_left_cmd=0.000 final_right_cmd=0.000 physical_output_active=false\n",
            ]
        )
    # Without a correction the body keeps pointing 40 deg; with one it ends ~88.
    final_yaw = 88.0 if with_correction_rows else 40.0
    responses.extend(
        [
            # lane 2, chunk 2: lane completes.
            _hb(_LAT_1M_NORTH, _LON_1M_EAST, imu_yaw_deg=final_yaw),
            *_PULSE_OK,
            _hb(_LAT_1M_NORTH, _LON_1M_EAST, imu_yaw_deg=final_yaw),
        ]
    )
    return responses


def _run_under_turned_corner(handle: FakeSerial, heading_reference: str):
    return controller.run_stop_correct_go(
        handle,
        segments=[dict(_EAST_LANE_1M), _left_connector(), dict(_NORTH_LANE_AFTER_CORNER)],
        resolved_calibration=_small_left_turn_calibration(30.0),
        start_lat=35.0,
        start_lon=129.0,
        start_yaw_deg=None,
        goal_lat=35.0000090,
        goal_lon=129.0000110,
        settle_after_move_ms=0,
        telemetry_stabilize_ms=0,
        event_timeout_s=1.0,
        heartbeat_timeout_s=1.0,
        max_connector_pulses_per_turn=1,
        heading_reference=heading_reference,
    )


def test_mission_heading_reference_exposes_connector_under_turn() -> None:
    handle = FakeSerial(_under_turned_corner_responses(with_correction_rows=True))
    rows, _raw, abort_reason = _run_under_turned_corner(handle, "mission")

    assert abort_reason == "NONE"
    lane2_rows = [r for r in rows if r["segment_index"] == 3]
    assert lane2_rows, "expected lane 2 cycles"
    # The 50 deg residual from the under-turned corner is visible and corrected.
    assert lane2_rows[0]["phase"] == "correction"
    assert float(lane2_rows[0]["heading_error_deg"]) == 50.0
    assert lane2_rows[0]["correction_success"] is True
    assert any(w.startswith("USB_DRIVE_LIVE_SET") for w in handle.writes)


def test_per_lane_heading_reference_absorbs_connector_under_turn() -> None:
    # Legacy behavior kept behind --heading-reference per_lane: the lane
    # re-captures its reference yaw, so the same under-turn reads as zero error.
    handle = FakeSerial(_under_turned_corner_responses(with_correction_rows=False))
    rows, _raw, abort_reason = _run_under_turned_corner(handle, "per_lane")

    assert abort_reason == "NONE"
    lane2_rows = [r for r in rows if r["segment_index"] == 3]
    assert lane2_rows
    assert all(r["phase"] == "move" for r in lane2_rows)
    assert all(float(r["heading_error_deg"]) == 0.0 for r in lane2_rows)
    assert not any(w.startswith("USB_DRIVE_LIVE_SET") for w in handle.writes)


def test_manual_switch_during_pulse_aborts_immediately() -> None:
    # AUTO-switch runs must stop as soon as telemetry inside the pulse window
    # reports the physical switch back in MANUAL.
    pulse_with_manual = [
        b"USB_PULSE_TEST event=ARM\n",
        b"USB_PULSE_TEST event=ACK\n",
        b"USB_PULSE_TEST event=PULSE_COMPLETE mode_switch=MANUAL\n",
        b"USB_PULSE_TEST event=STOP final_left_cmd=0.000 final_right_cmd=0.000 physical_output_active=false\n",
    ]
    handle = FakeSerial(
        [
            _hb(35.0, 129.0, imu_yaw_deg=90.0),
            *pulse_with_manual,
            _hb(35.0, 129.0, imu_yaw_deg=90.0),
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
        require_auto_switch=True,
    )

    assert abort_reason == controller.MANUAL_SWITCH_ABORT_REASON
    assert len(rows) == 1
    assert rows[0]["phase"] == "move"  # no correction is attempted after a manual grab


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


def test_mission_heading_holds_body_heading_on_backward_lane() -> None:
    # A reverse-driven lane travels west while the BODY faces east. Mission
    # heading must hold the body at 0 deg (east), so a +20 deg body drift is
    # corrected with a RIGHT turn even though the travel heading is 180.
    backward_lane = {
        "segment_index": 1,
        "segment_type": "backward_lane",
        "start_x_m": 1.0,
        "start_y_m": 0.0,
        "end_x_m": 0.0,
        "end_y_m": 0.0,
        "length_m": 1.0,
        "target_heading_deg": 180.0,
        "body_heading_deg": 0.0,
        "expected_motion_direction": "backward",
        "pulse_budget": 1,
    }
    handle = FakeSerial(
        [
            _hb(35.0, _LON_1M_EAST, imu_yaw_deg=0.0),  # preflight at lane start
            *_PULSE_OK,
            _hb(35.0, 129.0000055, imu_yaw_deg=20.0),  # mid-lane, body drifted +20
            _hb(None, None, imu_yaw_deg=2.0, with_gps=False),  # correction feedback
            b"USB_PULSE_TEST event=STOP final_left_cmd=0.000 final_right_cmd=0.000 physical_output_active=false\n",
            _hb(35.0, 129.0000055, imu_yaw_deg=2.0),  # preflight chunk 2
            *_PULSE_OK,
            _hb(35.0, 129.0, imu_yaw_deg=2.0),  # lane end reached
        ]
    )
    rows, _raw, abort_reason = controller.run_stop_correct_go(
        handle,
        segments=[backward_lane],
        resolved_calibration=geometry.FALLBACK_RESOLVED_CALIBRATION,
        start_lat=35.0,
        start_lon=129.0,
        start_yaw_deg=None,
        goal_lat=35.0,
        goal_lon=129.0,
        settle_after_move_ms=0,
        telemetry_stabilize_ms=0,
        event_timeout_s=1.0,
        heartbeat_timeout_s=1.0,
        heading_reference="mission",
    )

    assert abort_reason == "NONE"
    assert rows[0]["phase"] == "correction"
    assert float(rows[0]["target_heading_deg"]) == 0.0  # body target, not travel 180
    assert float(rows[0]["heading_error_deg"]) == -20.0
    assert rows[0]["correction_b_cmd"] == "-0.120"  # right turn fixes +20 body drift
    assert rows[0]["correction_success"] is True
    move_cmds = [w for w in handle.writes if w.startswith("USB_PULSE_TEST_CMD")]
    assert all("a=-0.080" in w for w in move_cmds)  # reverse-driven chunks


def test_connector_overshoot_stops_same_direction_pulsing() -> None:
    # High-variance motors can blow past the corner in one pulse (field data:
    # 1.6-43 deg from the same command). Once the measured remaining error
    # flips sign, pulsing the same direction again must stop; the next lane's
    # heading correction owns the cleanup.
    handle = FakeSerial(
        [
            _hb(35.0, 129.0, imu_yaw_deg=0.0),
            *_PULSE_OK,
            _hb(35.0, 129.0, imu_yaw_deg=60.0),  # strong pulse
            _hb(35.0, 129.0, imu_yaw_deg=60.0),
            *_PULSE_OK,
            _hb(35.0, 129.0, imu_yaw_deg=115.0),  # overshoot: 25 past the 90 target
        ]
    )
    rows, _raw, abort_reason = controller.run_stop_correct_go(
        handle,
        segments=[_left_connector()],
        resolved_calibration=_small_left_turn_calibration(30.0),
        start_lat=35.0,
        start_lon=129.0,
        start_yaw_deg=None,
        goal_lat=35.0000100,
        goal_lon=129.0,
        settle_after_move_ms=0,
        telemetry_stabilize_ms=0,
        event_timeout_s=1.0,
        heartbeat_timeout_s=1.0,
    )

    assert abort_reason == "NONE"
    connector_rows = [r for r in rows if r.get("phase") == "connector"]
    assert len(connector_rows) == 2  # budget was 5 but the overshoot stops it
    last = connector_rows[-1]
    assert last["turn_overshoot"] is True
    assert last["connector_turn_completed"] is False
    assert float(last["remaining_turn_error_deg"]) == -25.0
    summary = controller.build_stop_correct_go_summary(
        connector_rows,
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
    assert summary["connector_overshoot_count"] == 1


def test_connector_hands_off_small_remainder_instead_of_coarse_pulse() -> None:
    # Right-turn field data: one ~90 deg-class pulse with 2-3x variance. When a
    # pulse lands close (remaining 12 deg < half of the 30 deg per-pulse
    # angle), firing another full pulse would end FARTHER from the corner, so
    # the connector stops and hands the remainder to lane heading correction.
    handle = FakeSerial(
        [
            _hb(35.0, 129.0, imu_yaw_deg=0.0),
            *_PULSE_OK,
            _hb(35.0, 129.0, imu_yaw_deg=50.0),  # strong-ish pulse
            _hb(35.0, 129.0, imu_yaw_deg=50.0),
            *_PULSE_OK,
            _hb(35.0, 129.0, imu_yaw_deg=78.0),  # remaining 12 < 30/2
        ]
    )
    rows, _raw, abort_reason = controller.run_stop_correct_go(
        handle,
        segments=[_left_connector()],
        resolved_calibration=_small_left_turn_calibration(30.0),
        start_lat=35.0,
        start_lon=129.0,
        start_yaw_deg=None,
        goal_lat=35.0000100,
        goal_lon=129.0,
        settle_after_move_ms=0,
        telemetry_stabilize_ms=0,
        event_timeout_s=1.0,
        heartbeat_timeout_s=1.0,
    )

    assert abort_reason == "NONE"
    connector_rows = [r for r in rows if r.get("phase") == "connector"]
    assert len(connector_rows) == 2
    last = connector_rows[-1]
    assert last["turn_handed_off"] is True
    assert last["connector_turn_completed"] is False
    assert last["turn_overshoot"] is False
    assert float(last["remaining_turn_error_deg"]) == 12.0
    summary = controller.build_stop_correct_go_summary(
        connector_rows,
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
    assert summary["connector_handed_off_count"] == 1
