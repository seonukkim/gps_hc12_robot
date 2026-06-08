"""Tests for the continuous-motion controller.

The four guarantees the plan calls out are exercised as pure logic with mock
telemetry (no serial): cross-track sign, the +/-0.08 B-command clamp, GPS-degraded
dead-reckoning, and the connector repeated-pulses fallback flag. One end-to-end
loop test drives ``run_controller`` through a fake serial handle (scripted
heartbeat -> pulse -> heartbeat) to prove the pieces compose without hardware.
"""
from __future__ import annotations

import pytest

from tools.physical_path_planning import checks, controller, geometry
from tools.physical_path_planning.checks import FullPathFollowingNotAllowed


def _east_lane() -> dict[str, object]:
    # A lane running due east from the origin; cross-track is the +/-Y offset.
    return {
        "segment_index": 1,
        "segment_type": "forward_lane",
        "start_x_m": 0.0,
        "start_y_m": 0.0,
        "end_x_m": 1.0,
        "end_y_m": 0.0,
        "length_m": 1.0,
        "target_heading_deg": 90.0,
        "expected_motion_direction": "forward",
        "pulse_budget": 1,
    }


# --- 1. cross-track sign ------------------------------------------------------


def test_cross_track_sign_is_signed_and_correction_opposes_offset() -> None:
    seg = _east_lane()
    north = controller.pulse_correction(
        segment=seg, x=0.5, y=0.2, target_heading_deg=90.0, yaw=None, start_yaw_deg=None
    )
    south = controller.pulse_correction(
        segment=seg, x=0.5, y=-0.2, target_heading_deg=90.0, yaw=None, start_yaw_deg=None
    )
    # A point north of an eastbound lane has negative signed cross-track; south positive.
    assert north["cross_track_error_m"] < 0 < south["cross_track_error_m"]
    # With no heading error the B command is purely the cross-track correction,
    # and the two mirrored offsets produce mirror-image corrections.
    assert north["heading_error_deg"] == 0.0
    assert north["b_cte_component"] < 0 < south["b_cte_component"]
    assert north["b_cmd"] == pytest.approx(-south["b_cmd"])


# --- 2. B-command clamp at +/-0.08 -------------------------------------------


def test_b_command_clamped_to_plus_minus_0_08() -> None:
    seg = _east_lane()
    # yaw=0, start_yaw=90 -> heading_error = +90 -> heading component saturates +0.08;
    # a large southward offset adds positive cross-track so the raw sum exceeds 0.08.
    pos = controller.pulse_correction(
        segment=seg, x=0.5, y=-10.0, target_heading_deg=90.0, yaw=0.0, start_yaw_deg=90.0
    )
    assert pos["heading_error_deg"] == pytest.approx(90.0)
    assert pos["b_heading_component"] == pytest.approx(0.08)
    assert pos["b_cte_component"] == pytest.approx(0.04)
    assert pos["b_cmd"] == pytest.approx(0.08)  # 0.08 + 0.04 clamped back to 0.08

    neg = controller.pulse_correction(
        segment=seg, x=0.5, y=10.0, target_heading_deg=90.0, yaw=0.0, start_yaw_deg=-90.0
    )
    assert neg["b_cmd"] == pytest.approx(-0.08)
    assert abs(neg["b_cmd"]) <= 0.08 + 1e-9


def test_connector_b_command_bypasses_correction() -> None:
    seg = _east_lane()
    # A connector commands the calibrated turn value; steering components are zero.
    corr = controller.pulse_correction(
        segment=seg,
        x=0.5,
        y=5.0,
        target_heading_deg=90.0,
        yaw=10.0,
        start_yaw_deg=0.0,
        is_connector=True,
        connector_b_cmd=0.26,
    )
    assert corr["b_cmd"] == 0.26
    assert corr["b_heading_component"] == 0.0
    assert corr["b_cte_component"] == 0.0


# --- 3. GPS-degraded dead-reckon ---------------------------------------------


def test_gps_degraded_dead_reckons_from_cache() -> None:
    cache: dict[str, object] = {"lat": 35.0, "lon": 129.0, "degraded": False}

    fresh = controller.dead_reckon_gps(
        {"gps_lat": "35.0010000", "gps_lon": "129.0010000", "gps_block_reason": "OK"}, cache
    )
    assert fresh["gps_degraded"] is False
    assert fresh["gps_cached_used"] is False
    assert cache["lat"] == 35.001 and cache["lon"] == 129.001

    # BAD_HDOP with no usable fix -> reuse the cached position, flagged degraded.
    degraded = controller.dead_reckon_gps({"gps_block_reason": "BAD_HDOP"}, cache)
    assert degraded["gps_degraded"] is True
    assert degraded["gps_cached_used"] is True
    assert degraded["lat"] == 35.001 and degraded["lon"] == 129.001

    # A fresh fix after degradation reports recovery.
    recovered = controller.dead_reckon_gps(
        {"gps_lat": "35.0020000", "gps_lon": "129.0020000", "gps_block_reason": "OK"}, cache
    )
    assert recovered["gps_degraded"] is False
    assert recovered["gps_recovered"] is True


def test_gps_policy_default_continue_keeps_running_when_degraded() -> None:
    # The default policy continues (dead-reckoned) rather than aborting/pausing.
    assert geometry.gps_policy_action(True, controller.DEFAULT_GPS_DEGRADATION_POLICY) == "continue"
    assert geometry.gps_policy_action(True, "abort") == "abort"
    assert geometry.gps_policy_action(True, "pause") == "pause"


# --- 4. connector repeated-pulses fallback flag ------------------------------


def test_connector_uses_repeated_pulses_fallback_without_angle_calibration() -> None:
    cal = geometry.FALLBACK_RESOLVED_CALIBRATION  # no turn-angle calibration
    left = controller.connector_command(cal, "left")
    right = controller.connector_command(cal, "right")

    assert left["connector_mode"] == "repeated_pulses"
    assert left["fallback_to_repeated_pulses"] is True
    assert right["fallback_to_repeated_pulses"] is True
    # The fallback uses the known turn primitives -- NOT an invented 15-degree
    # micro-turn -- and left/right are distinct calibrated commands.
    assert (left["b_cmd"], left["pulse_ms"]) == (0.26, 700)
    assert (right["b_cmd"], right["pulse_ms"]) == (-0.08, 250)


# --- pulse outcome classification --------------------------------------------


def test_pulse_block_reason_orders_rc_invalid_first() -> None:
    rows = controller.telemetry.parse_usbdbg_rows(
        "USB_PULSE_TEST event=ACK\n"
        "USB_PULSE_TEST event=STOP usb_pulse_test_reject_reason=RC_INVALID final_left_cmd=0.000 "
        "final_right_cmd=0.000 physical_output_active=false\n"
    )
    assert controller.pulse_block_reason(rows) == "RC_INVALID"


def test_pulse_block_reason_none_for_clean_pulse() -> None:
    rows = controller.telemetry.parse_usbdbg_rows(
        "USB_PULSE_TEST event=ACK\n"
        "USB_PULSE_TEST event=STOP final_left_cmd=0.000 final_right_cmd=0.000 physical_output_active=false\n"
    )
    assert controller.pulse_block_reason(rows) is None


# --- summary guard ------------------------------------------------------------


def test_controller_summary_is_guarded_not_ready() -> None:
    summary = controller.build_controller_summary(
        [],
        start_lat=35.0,
        start_lon=129.0,
        goal_lat=35.001,
        goal_lon=129.001,
        goal_distance_m=10.0,
        fallback_to_repeated_pulses=True,
    )
    assert checks.assert_not_ready_for_full_path_following(summary) is summary
    summary["ready_for_full_path_following"] = True
    with pytest.raises(FullPathFollowingNotAllowed):
        checks.assert_not_ready_for_full_path_following(summary)


def test_controller_summary_reports_continuous_drive_and_imu_heading() -> None:
    summary = controller.build_controller_summary(
        [
            {
                "row_type": "pulse",
                "valid_pulse": True,
                "drive_mode": "continuous",
                "imu_relative_yaw_deg": 3.5,
                "gps_degraded": True,
            }
        ],
        start_lat=35.0,
        start_lon=129.0,
        goal_lat=35.001,
        goal_lon=129.001,
        goal_distance_m=10.0,
        fallback_to_repeated_pulses=False,
    )
    assert summary["continuous_drive_used"] is True
    assert summary["continuous_drive_count"] == 1
    assert summary["imu_heading_used_count"] == 1
    assert summary["gps_degraded_count"] == 1


# --- end-to-end loop over a fake serial handle -------------------------------


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


def _heartbeat(lat: float, lon: float) -> bytes:
    return (
        f"USB_PULSE_TEST event=HEARTBEAT usb_pulse_test_mode=true rc_ok=true "
        f"neutral_ok=true physical_output_active=false gps_block_reason=OK "
        f"gps_lat={lat:.7f} gps_lon={lon:.7f} imu_relative_yaw_deg=0.0\n"
    ).encode("ascii")


def test_run_controller_completes_one_clean_lane_pulse() -> None:
    handle = FakeSerial(
        [
            _heartbeat(35.0, 129.0),  # pre-pulse heartbeat (neutral, fresh GPS)
            b"USB_PULSE_TEST event=ARM\n",
            b"USB_PULSE_TEST event=ACK\n",
            b"USB_PULSE_TEST event=PULSE_COMPLETE\n",
            b"USB_PULSE_TEST event=STOP final_left_cmd=0.000 final_right_cmd=0.000 physical_output_active=false\n",
            _heartbeat(35.0000050, 129.0),  # post-pulse heartbeat
        ]
    )
    rows, raw_lines, abort_reason = controller.run_controller(
        handle,
        segments=[_east_lane()],
        resolved_calibration=geometry.FALLBACK_RESOLVED_CALIBRATION,
        start_lat=35.0,
        start_lon=129.0,
        start_yaw_deg=0.0,
        goal_lat=35.0000100,
        goal_lon=129.0,
        event_timeout_s=1.0,
        heartbeat_timeout_s=1.0,
    )

    assert abort_reason == "NONE"
    assert len(rows) == 1
    row = rows[0]
    assert row["valid_pulse"] is True
    assert row["invalid_reason"] == "NONE"
    assert row["gps_degraded"] is False
    assert row["ready_for_full_path_following"] is False
    # The guarded pulse issued ARM, the forward command, then STOP in order.
    assert handle.writes[0].startswith("USB_PULSE_TEST_ARM")
    assert handle.writes[1].startswith("USB_PULSE_TEST_CMD") and "a=0.300" in handle.writes[1]
    assert handle.writes[2].startswith("USB_PULSE_TEST_STOP")


def test_run_controller_aborts_on_rc_invalid_during_pulse() -> None:
    handle = FakeSerial(
        [
            _heartbeat(35.0, 129.0),
            b"USB_PULSE_TEST event=ARM\n",
            b"USB_PULSE_TEST event=ACK\n",
            b"USB_PULSE_TEST event=PULSE_COMPLETE\n",
            b"USB_PULSE_TEST event=STOP usb_pulse_test_reject_reason=RC_INVALID final_left_cmd=0.000 "
            b"final_right_cmd=0.000 physical_output_active=false\n",
            _heartbeat(35.0, 129.0),
        ]
    )
    rows, _raw, abort_reason = controller.run_controller(
        handle,
        segments=[_east_lane()],
        resolved_calibration=geometry.FALLBACK_RESOLVED_CALIBRATION,
        start_lat=35.0,
        start_lon=129.0,
        start_yaw_deg=0.0,
        goal_lat=35.0000100,
        goal_lon=129.0,
        event_timeout_s=1.0,
        heartbeat_timeout_s=1.0,
    )
    assert abort_reason == "RC_INVALID"
    assert rows[-1]["invalid_reason"] == "RC_INVALID"


def test_run_controller_aborts_when_no_heartbeat() -> None:
    handle = FakeSerial([])  # nothing ever arrives
    rows, _raw, abort_reason = controller.run_controller(
        handle,
        segments=[_east_lane()],
        resolved_calibration=geometry.FALLBACK_RESOLVED_CALIBRATION,
        start_lat=35.0,
        start_lon=129.0,
        start_yaw_deg=0.0,
        goal_lat=35.0000100,
        goal_lon=129.0,
        event_timeout_s=0.2,
        heartbeat_timeout_s=0.2,
    )
    assert abort_reason == "NO_GUARDED_PULSE_HEARTBEAT"
    assert rows == []
