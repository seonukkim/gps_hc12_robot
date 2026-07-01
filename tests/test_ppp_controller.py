"""연속-모션 컨트롤러(controller.run_controller) 계약 테스트.

목적/역할:
    계획서가 명시한 네 가지 보장을 순수 로직으로(시리얼 없이 목 텔레메트리로) 검증한다:
      1) 크로스트랙 부호(보정이 오프셋을 상쇄하는 방향),
      2) B-명령 +/-0.08 클램프,
      3) GPS 열화 시 캐시 기반 추측항법(dead-reckoning),
      4) 각도 캘리브레이션 부재 시 커넥터 repeated_pulses 폴백 플래그.
    끝으로 엔드투엔드 루프 테스트가 가짜 시리얼 핸들(스크립트된
    heartbeat -> pulse -> heartbeat)로 ``run_controller`` 를 돌려 하드웨어 없이
    구성요소가 조립됨을 증명한다.

시스템 내 위치:
    ``tools.physical_path_planning`` 의 ``checks`` / ``controller`` / ``geometry`` 만
    import 한다. stop_correct_go 경로는 자매 파일 ``test_ppp_stop_correct_go`` 가 담당하며,
    이 파일은 연속-드라이브(run_controller) 경로에 집중한다.

핵심 개념·불변식:
  - 조향은 B(turn) 축만, +/-0.08 클램프; 전진 A(throttle)와 펄스 길이는 캘리브레이션에서
    오며 컨트롤러가 절대 낮추지 않는다.
  - 모든 요약은 ``checks.assert_not_ready_for_full_path_following`` 를 통과해야 하므로
    컨트롤러는 full-path-following 준비 완료를 주장할 수 없다.
  - RC_INVALID / 하트비트 부재 / 수동 전환 복귀는 하드 abort 사유다(트레이스백 없이 중단).

------------------------------------------------------------------------------
Tests for the continuous-motion controller.

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


# ── 공유 픽스처 헬퍼 / Shared fixture helpers ──


def _east_lane() -> dict[str, object]:
    """정동(ENU 90도) 방향 직선 lane; 크로스트랙은 +/-Y 오프셋 / A due-east forward lane."""
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


# ── 1. 크로스트랙 부호 / Cross-track sign ──


def test_cross_track_sign_is_signed_and_correction_opposes_offset() -> None:
    """크로스트랙은 부호가 있고 보정 B 가 오프셋을 상쇄(북/남 대칭) / Signed CTE; correction opposes offset."""
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


# ── 2. B-명령 +/-0.08 클램프 / B-command clamp at +/-0.08 ──


def test_b_command_clamped_to_plus_minus_0_08() -> None:
    """헤딩+크로스트랙 합이 커도 최종 B 는 +/-0.08 로 클램프 / Raw B sum clamps back to +/-0.08."""
    seg = _east_lane()
    # yaw=0, start_yaw=90 -> heading_error = +90 -> heading component saturates +0.08;
    # a large southward offset adds positive cross-track so the raw sum exceeds 0.08.
    pos = controller.pulse_correction(
        segment=seg, x=0.5, y=-10.0, target_heading_deg=90.0, yaw=0.0, start_yaw_deg=90.0
    )
    assert pos["heading_error_deg"] == pytest.approx(90.0)
    assert pos["b_heading_component"] == pytest.approx(0.54)
    assert pos["b_cte_component"] == pytest.approx(2.0)
    assert pos["b_cmd"] == pytest.approx(0.08)  # raw correction clamped back to 0.08

    neg = controller.pulse_correction(
        segment=seg, x=0.5, y=10.0, target_heading_deg=90.0, yaw=0.0, start_yaw_deg=-90.0
    )
    assert neg["b_cmd"] == pytest.approx(-0.08)
    assert abs(neg["b_cmd"]) <= 0.08 + 1e-9


def test_connector_b_command_bypasses_correction() -> None:
    """커넥터는 캘리브레이션 회전값을 그대로 명령하고 조향 성분을 우회 / Connector bypasses steering correction."""
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


def test_gps_imu_closed_loop_computes_heading_and_cross_track_correction() -> None:
    """gps_imu_closed_loop 은 헤딩+크로스트랙 성분을 합산해 B 산출(source=gps_imu) / Closed loop sums both terms."""
    seg = _east_lane()
    corr = controller.pulse_correction(
        segment=seg,
        x=0.5,
        y=-0.2,
        target_heading_deg=90.0,
        yaw=0.0,
        start_yaw_deg=-10.0,
        path_control_mode="gps_imu_closed_loop",
        k_heading=0.006,
        k_cross_track=0.20,
        max_correction_b=0.08,
    )
    assert corr["heading_error_deg"] == pytest.approx(-10.0)
    assert corr["cross_track_error_m"] > 0
    assert corr["b_heading_component"] == pytest.approx(-0.06)
    assert corr["b_cte_component"] == pytest.approx(0.04)
    assert corr["b_cmd"] == pytest.approx(-0.02)
    assert corr["correction_source"] == "gps_imu"


def test_open_loop_chunks_does_not_apply_correction() -> None:
    """open_loop_chunks 는 보정 없이 base_b 를 그대로 사용(source=open_loop) / Open loop applies no correction."""
    seg = _east_lane()
    corr = controller.pulse_correction(
        segment=seg,
        x=0.5,
        y=-0.2,
        target_heading_deg=90.0,
        yaw=0.0,
        start_yaw_deg=-10.0,
        path_control_mode="open_loop_chunks",
        base_b_cmd=0.01,
    )
    assert corr["heading_error_deg"] == 0.0
    assert corr["cross_track_error_m"] == 0.0
    assert corr["b_heading_component"] == 0.0
    assert corr["b_cte_component"] == 0.0
    assert corr["b_cmd"] == pytest.approx(0.01)
    assert corr["correction_source"] == "open_loop"


# ── 3. GPS 열화 시 추측항법 / GPS-degraded dead-reckon ──


def test_gps_degraded_dead_reckons_from_cache() -> None:
    """신선 fix->캐시 갱신, BAD_HDOP->캐시 재사용(degraded), 이후 재신선->recovered / Dead-reckon lifecycle."""
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
    """기본 GPS 열화 정책은 continue(추측항법 지속), abort/pause 도 매핑 확인 / Default policy is 'continue'."""
    # The default policy continues (dead-reckoned) rather than aborting/pausing.
    assert geometry.gps_policy_action(True, controller.DEFAULT_GPS_DEGRADATION_POLICY) == "continue"
    assert geometry.gps_policy_action(True, "abort") == "abort"
    assert geometry.gps_policy_action(True, "pause") == "pause"


# ── 4. 커넥터 repeated_pulses 폴백 플래그 / Connector repeated-pulses fallback flag ──


def test_connector_uses_repeated_pulses_fallback_without_angle_calibration() -> None:
    """각도 캘리브레이션 없으면 알려진 회전 프리미티브로 repeated_pulses 폴백(15도 마이크로턴 금지) / Fallback uses known primitives."""
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


# ── 펄스 결과 분류 / Pulse outcome classification ──


def test_pulse_block_reason_orders_rc_invalid_first() -> None:
    """정지 이벤트에 RC_INVALID 가 있으면 이를 우선 차단 사유로 보고 / RC_INVALID takes precedence as block reason."""
    rows = controller.telemetry.parse_usbdbg_rows(
        "USB_PULSE_TEST event=ACK\n"
        "USB_PULSE_TEST event=STOP usb_pulse_test_reject_reason=RC_INVALID final_left_cmd=0.000 "
        "final_right_cmd=0.000 physical_output_active=false\n"
    )
    assert controller.pulse_block_reason(rows) == "RC_INVALID"


def test_pulse_block_reason_none_for_clean_pulse() -> None:
    """정상(모터 0으로 정지) 펄스는 차단 사유 없음(None) / A clean, zeroed pulse has no block reason."""
    rows = controller.telemetry.parse_usbdbg_rows(
        "USB_PULSE_TEST event=ACK\n"
        "USB_PULSE_TEST event=STOP final_left_cmd=0.000 final_right_cmd=0.000 physical_output_active=false\n"
    )
    assert controller.pulse_block_reason(rows) is None


# ── 요약 가드 / Summary guard ──


def test_controller_summary_is_guarded_not_ready() -> None:
    """요약은 ready=False 로 통과하되, True 로 위조하면 FullPathFollowingNotAllowed / Guard rejects a forged ready flag."""
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
    """요약이 연속-드라이브/ IMU-헤딩/ GPS-열화 카운터를 집계 / Summary aggregates continuous/IMU/degraded counts."""
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


# ── 가짜 시리얼 핸들 위 엔드투엔드 루프 / End-to-end loop over a fake serial handle ──


class FakeSerial:
    """스크립트 텔레메트리를 재생하고 write 를 전부 기록하는 가짜 핸들.

    Replays scripted telemetry lines and records every written command.
    """

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


def _heartbeat(
    lat: float,
    lon: float,
    *,
    rc_ok: bool = True,
    neutral_ok: bool = True,
    usb_ignore_rc: bool = False,
    gps_block_reason: str = "OK",
    imu_yaw_deg: float = 0.0,
) -> bytes:
    """한 줄짜리 USB 감독-모드 하트비트 바이트 생성 / Build one USB-supervised heartbeat line."""
    return (
        f"USB_PULSE_TEST event=HEARTBEAT usb_pulse_test_mode=true "
        f"usb_pulse_test_ignore_rc_input={str(usb_ignore_rc).lower()} "
        f"usb_drive_live_mode={str(usb_ignore_rc).lower()} "
        f"rc_ok={str(rc_ok).lower()} neutral_ok={str(neutral_ok).lower()} "
        f"physical_output_active=false gps_block_reason={gps_block_reason} "
        f"gps_lat={lat:.7f} gps_lon={lon:.7f} imu_relative_yaw_deg={imu_yaw_deg:.1f}\n"
    ).encode("ascii")


def test_run_controller_completes_one_clean_lane_pulse() -> None:
    """정상 lane 펄스 1회 완주: ARM -> 전진 CMD(a=0.300) -> STOP 순서 / One clean guarded lane pulse, in order."""
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


def test_run_controller_ignores_rc_not_ok_for_usb_supervised_continuous_drive() -> None:
    """USB 감독 연속-드라이브에서는 rc_ok=false 를 무시하고 진행(경고만 기록) / RC-not-ok ignored in USB-supervised mode."""
    handle = FakeSerial(
        [
            _heartbeat(35.0, 129.0, rc_ok=False, neutral_ok=False, usb_ignore_rc=True),
            b"USB_DRIVE_LIVE event=ACTIVE\n",
            b"USB_DRIVE_LIVE event=STOP final_left_cmd=0.000 final_right_cmd=0.000 physical_output_active=false\n",
            _heartbeat(35.0000050, 129.0, rc_ok=False, neutral_ok=False, usb_ignore_rc=True),
        ]
    )
    rows, _raw_lines, abort_reason = controller.run_controller(
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
        straight_motion_mode="continuous",
        live_update_hz=100.0,
        live_chunk_ms=1000,
    )
    assert abort_reason == "NONE"
    assert rows and rows[0]["valid_pulse"] is True
    assert rows[0]["rc_ignored_for_usb_supervised"] is True
    assert rows[0]["rc_warning"] == "RC_NOT_OK_IGNORED_FOR_MAC_USB_SUPERVISED_MODE"
    assert handle.writes[0].startswith("USB_DRIVE_LIVE_SET")


def test_run_controller_splits_live_drive_segments_under_max_duration(monkeypatch: pytest.MonkeyPatch) -> None:
    """연속 lane 을 live_max_ms 이하 청크로 분할하고 각 청크에 폐루프 보정 부여 / Splits live drive under max, closed-loop per chunk."""
    def fake_send_live_drive(
        handle,
        *,
        seq,
        duration_s,
        update_hz,
        ttl_ms,
        command_fn,
        raw_lines,
        event_timeout_s,
        verbose_raw=True,
    ):
        a_cmd, b_cmd = command_fn(None)
        handle.write(
            (
                f"USB_DRIVE_LIVE_SET seq={seq} a={a_cmd:.3f} b={b_cmd:.3f} "
                f"duration_ms={int(duration_s * 1000.0)} ttl_ms={ttl_ms}\n"
            ).encode("ascii")
        )
        handle.write(f"USB_DRIVE_LIVE_STOP seq={seq}\n".encode("ascii"))
        return controller.telemetry.parse_usbdbg_rows(
            "USB_DRIVE_LIVE event=ACTIVE\n"
            "USB_DRIVE_LIVE event=STOP final_left_cmd=0.000 final_right_cmd=0.000 physical_output_active=false\n"
        )

    monkeypatch.setattr(controller.executor, "send_live_drive", fake_send_live_drive)
    responses: list[bytes] = []
    for idx in range(3):
        responses.extend(
            [
                _heartbeat(35.0 + idx * 0.000001, 129.0, usb_ignore_rc=True, imu_yaw_deg=4.0),
                _heartbeat(35.0 + (idx + 1) * 0.000001, 129.0, usb_ignore_rc=True, imu_yaw_deg=4.0),
            ]
        )
    handle = FakeSerial(responses)
    cal = dict(geometry.FALLBACK_RESOLVED_CALIBRATION)
    cal["forward"] = {"a": 0.30, "b": 0.0, "ms": 3, "source": "approved_test"}
    rows, _raw_lines, abort_reason = controller.run_controller(
        handle,
        segments=[_east_lane()],
        resolved_calibration=cal,
        start_lat=35.0,
        start_lon=129.0,
        start_yaw_deg=-10.0,
        goal_lat=35.0000100,
        goal_lon=129.0,
        event_timeout_s=0.1,
        heartbeat_timeout_s=0.1,
        straight_motion_mode="continuous",
        live_update_hz=1000.0,
        live_chunk_ms=1,
        live_max_ms=1,
        max_segment_chunks=3,
    )
    assert abort_reason == "NONE"
    setpoint_writes = [write for write in handle.writes if write.startswith("USB_DRIVE_LIVE_SET")]
    assert setpoint_writes
    assert all("duration_ms=1" in write for write in setpoint_writes)
    assert max(int(write.split("duration_ms=")[1].split()[0]) for write in setpoint_writes) <= 1
    assert len(rows) == 3
    assert all(row["valid_pulse"] is True for row in rows)
    assert any(abs(float(row["b_heading_component"])) > 0.0 for row in rows)
    assert rows[0]["path_control_mode"] == "gps_imu_closed_loop"
    assert "current_x_m" in rows[0]
    assert "remaining_distance_m" in rows[0]
    assert "b_cross_track_correction" in rows[0]
    assert "correction_source" in rows[0]
    summary = controller.build_controller_summary(
        rows,
        start_lat=35.0,
        start_lon=129.0,
        goal_lat=35.0000100,
        goal_lon=129.0,
        goal_distance_m=1.0,
        fallback_to_repeated_pulses=False,
        abort_reason=abort_reason,
    )
    assert summary["chunk_count"] == 3
    assert summary["valid_chunk_count"] == 3
    assert summary["imu_heading_used_count"] > 0


def test_run_controller_maps_live_drive_duration_reject_to_host_error() -> None:
    """펌웨어의 DURATION_EXCEEDS_MAX 거부를 호스트 오류로 매핑해 중단 / Firmware duration reject maps to host abort."""
    handle = FakeSerial(
        [
            _heartbeat(35.0, 129.0, usb_ignore_rc=True),
            b"USB_DRIVE_LIVE event=REJECT usb_pulse_test_reject_reason=USB_DRIVE_LIVE_DURATION_EXCEEDS_MAX\n",
            b"USB_DRIVE_LIVE event=STOP final_left_cmd=0.000 final_right_cmd=0.000 physical_output_active=false\n",
            _heartbeat(35.0, 129.0, usb_ignore_rc=True),
        ]
    )
    rows, _raw_lines, abort_reason = controller.run_controller(
        handle,
        segments=[_east_lane()],
        resolved_calibration=geometry.FALLBACK_RESOLVED_CALIBRATION,
        start_lat=35.0,
        start_lon=129.0,
        start_yaw_deg=0.0,
        goal_lat=35.0000100,
        goal_lon=129.0,
        event_timeout_s=0.1,
        heartbeat_timeout_s=0.1,
        straight_motion_mode="continuous",
        live_update_hz=1000.0,
        live_chunk_ms=1500,
        live_max_ms=1500,
    )
    assert abort_reason == "HOST_SENT_DURATION_OVER_MAX"
    assert rows[-1]["invalid_reason"] == "HOST_SENT_DURATION_OVER_MAX"
    assert rows[-1]["offending_duration_ms"] == 800
    summary = controller.build_controller_summary(
        rows,
        start_lat=35.0,
        start_lon=129.0,
        goal_lat=35.0000100,
        goal_lon=129.0,
        goal_distance_m=1.0,
        fallback_to_repeated_pulses=False,
        abort_reason=abort_reason,
    )
    assert summary["abort_reason"] == "HOST_SENT_DURATION_OVER_MAX"
    assert summary["offending_duration_ms"] == 800


def test_run_controller_continues_when_gps_degraded_policy_continue() -> None:
    """GPS 열화(BAD_HDOP) + policy=continue 면 유효 펄스로 계속 진행(gps_degraded 플래그) / Continue policy keeps driving degraded."""
    handle = FakeSerial(
        [
            _heartbeat(35.0, 129.0, usb_ignore_rc=True, gps_block_reason="BAD_HDOP"),
            b"USB_DRIVE_LIVE event=ACTIVE\n",
            b"USB_DRIVE_LIVE event=STOP final_left_cmd=0.000 final_right_cmd=0.000 physical_output_active=false\n",
            _heartbeat(35.0, 129.0, usb_ignore_rc=True, gps_block_reason="BAD_HDOP"),
        ]
    )
    rows, _raw_lines, abort_reason = controller.run_controller(
        handle,
        segments=[_east_lane()],
        resolved_calibration=geometry.FALLBACK_RESOLVED_CALIBRATION,
        start_lat=35.0,
        start_lon=129.0,
        start_yaw_deg=0.0,
        goal_lat=35.0000100,
        goal_lon=129.0,
        event_timeout_s=0.1,
        heartbeat_timeout_s=0.1,
        gps_degradation_policy="continue",
        straight_motion_mode="continuous",
        live_update_hz=1000.0,
        live_chunk_ms=1000,
        live_max_ms=1000,
    )
    assert abort_reason == "NONE"
    assert rows[0]["valid_pulse"] is True
    assert rows[0]["gps_degraded"] is True


def test_run_controller_reanchors_pose_when_gps_recovers(monkeypatch: pytest.MonkeyPatch) -> None:
    """열화 후 GPS 복구 시 추측항법 포즈를 실측으로 재고정(gps_reanchored) / Re-anchors dead-reckoned pose on GPS recovery."""
    def fake_send_live_drive(
        handle,
        *,
        seq,
        duration_s,
        update_hz,
        ttl_ms,
        command_fn,
        raw_lines,
        event_timeout_s,
        verbose_raw=True,
    ):
        command_fn(None)
        return controller.telemetry.parse_usbdbg_rows(
            "USB_DRIVE_LIVE event=ACTIVE\n"
            "USB_DRIVE_LIVE event=STOP final_left_cmd=0.000 final_right_cmd=0.000 physical_output_active=false\n"
        )

    monkeypatch.setattr(controller.executor, "send_live_drive", fake_send_live_drive)
    handle = FakeSerial(
        [
            _heartbeat(35.0, 129.0, usb_ignore_rc=True, gps_block_reason="BAD_HDOP"),
            _heartbeat(35.0, 129.0, usb_ignore_rc=True, gps_block_reason="BAD_HDOP"),
            _heartbeat(35.0000010, 129.0, usb_ignore_rc=True, gps_block_reason="OK"),
            _heartbeat(35.0000010, 129.0, usb_ignore_rc=True, gps_block_reason="OK"),
        ]
    )
    cal = dict(geometry.FALLBACK_RESOLVED_CALIBRATION)
    cal["forward"] = {"a": 0.30, "b": 0.0, "ms": 2, "source": "approved_test"}
    rows, _raw_lines, abort_reason = controller.run_controller(
        handle,
        segments=[_east_lane()],
        resolved_calibration=cal,
        start_lat=35.0,
        start_lon=129.0,
        start_yaw_deg=-90.0,
        goal_lat=35.0000100,
        goal_lon=129.0,
        event_timeout_s=0.1,
        heartbeat_timeout_s=0.1,
        gps_degradation_policy="continue",
        straight_motion_mode="continuous",
        live_chunk_ms=1,
        live_max_ms=1,
        max_segment_chunks=2,
    )
    assert abort_reason == "NONE"
    assert rows[0]["gps_degraded"] is True
    assert rows[-1]["gps_reanchored"] is True
    summary = controller.build_controller_summary(
        rows,
        start_lat=35.0,
        start_lon=129.0,
        goal_lat=35.0000100,
        goal_lon=129.0,
        goal_distance_m=1.0,
        fallback_to_repeated_pulses=False,
        abort_reason=abort_reason,
    )
    assert summary["gps_reanchor_count"] >= 1


def test_run_controller_splits_approved_connector_turn_under_max_duration() -> None:
    """승인된 커넥터 회전을 live_max_ms 이하 펄스들로 분할(ms 합=캘리브레이션) / Splits approved connector turn under max ms."""
    connector = {
        "segment_index": 2,
        "segment_type": "connector_turn",
        "start_x_m": 1.0,
        "start_y_m": 0.0,
        "end_x_m": 1.0,
        "end_y_m": 1.0,
        "length_m": 1.0,
        "target_heading_deg": 180.0,
        "expected_motion_direction": "turn_left",
        "pulse_budget": 1,
    }
    responses: list[bytes] = []
    for idx in range(3):
        responses.extend(
            [
                _heartbeat(35.0, 129.0),
                b"USB_PULSE_TEST event=ARM\n",
                b"USB_PULSE_TEST event=ACK\n",
                b"USB_PULSE_TEST event=STOP final_left_cmd=0.000 final_right_cmd=0.000 physical_output_active=false\n",
                _heartbeat(35.0, 129.0),
            ]
        )
    handle = FakeSerial(responses)
    cal = dict(geometry.FALLBACK_RESOLVED_CALIBRATION)
    cal["connector_mode_effective"] = "angle_calibrated"
    cal["turn_left_90"] = {"a": 0.0, "b": 0.24, "ms": 25, "source": "approved_turn"}
    rows, _raw_lines, abort_reason = controller.run_controller(
        handle,
        segments=[connector],
        resolved_calibration=cal,
        start_lat=35.0,
        start_lon=129.0,
        start_yaw_deg=-90.0,
        goal_lat=35.0000100,
        goal_lon=129.0,
        event_timeout_s=0.1,
        heartbeat_timeout_s=0.1,
        live_max_ms=10,
    )
    assert abort_reason == "NONE"
    command_writes = [write for write in handle.writes if write.startswith("USB_PULSE_TEST_CMD")]
    assert [int(write.split("ms=")[1]) for write in command_writes] == [10, 10, 5]
    assert all(row["valid_pulse"] is True for row in rows)


def test_run_controller_aborts_on_rc_invalid_during_pulse() -> None:
    """활성 펄스 중 RC_INVALID 보고 시 하드 abort / RC_INVALID during the active pulse forces a hard abort."""
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
    """하트비트가 전혀 없으면 NO_GUARDED_PULSE_HEARTBEAT 로 중단, 행 미기록 / No heartbeat -> abort, no rows."""
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


# ── 헤딩 기준 자동 캡처 (필드 "보정 죽음" 버그 픽스) / Heading reference auto-capture ──


def _fake_send_live_drive_uses_command(handle, *, seq, duration_s, update_hz, ttl_ms, command_fn, raw_lines, event_timeout_s, verbose_raw=True):
    """command_fn 을 호출해 계산된 a/b 를 SET 로 기록하는 executor.send_live_drive 스텁 / send_live_drive stub."""
    a_cmd, b_cmd = command_fn(None)
    handle.write(
        (
            f"USB_DRIVE_LIVE_SET seq={seq} a={a_cmd:.3f} b={b_cmd:.3f} "
            f"duration_ms={int(duration_s * 1000.0)} ttl_ms={ttl_ms}\n"
        ).encode("ascii")
    )
    return controller.telemetry.parse_usbdbg_rows(
        "USB_DRIVE_LIVE event=ACTIVE\n"
        "USB_DRIVE_LIVE event=STOP final_left_cmd=0.000 final_right_cmd=0.000 physical_output_active=false\n"
    )


def test_run_controller_auto_captures_reference_yaw_so_heading_correction_is_live(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """start_yaw 미지정 시 첫 lane 하트비트 yaw 를 기준으로 자동 캡처 -> 이후 드리프트가 살아있는 B 보정 생성 / Auto-captures reference yaw."""
    # No --start-yaw-deg (the field default). The first lane heartbeat yaw becomes
    # the heading-hold reference, so a later yaw drift produces a nonzero B and a
    # final_b_cmd that changes over time -- the bug was a permanently zero error.
    monkeypatch.setattr(controller.executor, "send_live_drive", _fake_send_live_drive_uses_command)
    handle = FakeSerial(
        [
            _heartbeat(35.0, 129.0, usb_ignore_rc=True, imu_yaw_deg=0.0),  # chunk1 ref capture
            _heartbeat(35.0, 129.0, usb_ignore_rc=True, imu_yaw_deg=0.0),  # chunk1 post
            _heartbeat(35.0, 129.0, usb_ignore_rc=True, imu_yaw_deg=8.0),  # chunk2 drift
            _heartbeat(35.0, 129.0, usb_ignore_rc=True, imu_yaw_deg=8.0),  # chunk2 post
        ]
    )
    cal = dict(geometry.FALLBACK_RESOLVED_CALIBRATION)
    cal["forward"] = {"a": 0.30, "b": 0.0, "ms": 2, "source": "approved_test"}
    rows, _raw, abort_reason = controller.run_controller(
        handle,
        segments=[_east_lane()],
        resolved_calibration=cal,
        start_lat=35.0,
        start_lon=129.0,
        start_yaw_deg=None,  # <-- field default; reference must be auto-captured
        goal_lat=35.0000100,
        goal_lon=129.0,
        event_timeout_s=0.1,
        heartbeat_timeout_s=0.1,
        straight_motion_mode="continuous",
        live_update_hz=1000.0,
        live_chunk_ms=1,
        live_max_ms=1,
        max_segment_chunks=2,
    )
    assert abort_reason == "NONE"
    assert len(rows) == 2
    # First chunk holds the captured reference (≈0 error); second chunk drifted.
    assert abs(float(rows[1]["b_heading_correction"])) > 0.0
    assert float(rows[0]["final_b_cmd"]) != float(rows[1]["final_b_cmd"])
    assert rows[0]["heading_error_deg"] != "NA"


# 실행 트레이스 CSV 가 반드시 담아야 하는 열 계약 / Columns the execution-trace CSV must always contain.
REQUIRED_TRACE_COLUMNS = [
    "segment_index", "chunk_index", "segment_type", "path_control_mode",
    "gps_valid", "gps_degraded", "gps_reanchored", "current_lat", "current_lon",
    "current_x_m", "current_y_m", "target_x_m", "target_y_m",
    "segment_start_x_m", "segment_start_y_m", "segment_end_x_m", "segment_end_y_m",
    "target_heading_deg", "imu_yaw_deg", "heading_error_deg", "cross_track_error_m",
    "along_track_progress_m", "remaining_distance_m", "base_a_cmd", "b_trim",
    "b_heading_correction", "b_cross_track_correction", "final_a_cmd", "final_b_cmd",
    "correction_source", "ack_seen", "active_seen", "stop_seen", "final_zero",
]


def test_execution_row_contains_all_required_trace_columns() -> None:
    """build_execution_row 출력에 트레이스 필수 열이 모두 존재 / Every required trace column is present in a built row."""
    seg = _east_lane()
    cache: dict[str, object] = {"lat": 35.0, "lon": 129.0, "degraded": False}
    gps = controller.dead_reckon_gps(
        {"gps_lat": "35.0", "gps_lon": "129.0", "gps_block_reason": "OK"}, cache
    )
    corr = controller.pulse_correction(
        segment=seg, x=0.1, y=0.05, target_heading_deg=90.0, yaw=5.0, start_yaw_deg=0.0
    )
    after = controller.telemetry.parse_usbdbg_rows(
        "USB_PULSE_TEST event=HEARTBEAT imu_relative_yaw_deg=5.0 gps_lat=35.0 gps_lon=129.0"
    )[0]
    pulse_rows = controller.telemetry.parse_usbdbg_rows(
        "USB_PULSE_TEST event=ACK\n"
        "USB_PULSE_TEST event=STOP final_left_cmd=0.000 final_right_cmd=0.000 physical_output_active=false\n"
    )
    row = controller.build_execution_row(
        segment=seg,
        primitive_index=1,
        after_row=after,
        pulse_rows=pulse_rows,
        start_lat=35.0,
        start_lon=129.0,
        goal_lat=35.001,
        goal_lon=129.0,
        start_yaw_deg=0.0,
        target_heading_deg=90.0,
        a_cmd=0.30,
        correction=corr,
        pulse_ms=800,
        gps=gps,
        calibration_source="test",
        connector_mode="lane",
        pose={"x": 0.1, "y": 0.05, "gps_valid": True, "gps_reanchored": False},
    )
    for column in REQUIRED_TRACE_COLUMNS:
        assert column in row, column


# ── 모드 전환 + 요약 보정 플래그 / Mode-switch + summary correction flags ──


def _heartbeat_with_mode(mode_switch: str) -> bytes:
    """모드 스위치(AUTO/MANUAL) 상태를 실은 하트비트 바이트 생성 / Heartbeat carrying a mode-switch state."""
    return (
        "USB_PULSE_TEST event=HEARTBEAT usb_pulse_test_mode=true usb_drive_live_mode=true "
        "rc_ok=false neutral_ok=false physical_output_active=false gps_block_reason=OK "
        f"gps_lat=35.0000000 gps_lon=129.0000000 imu_relative_yaw_deg=0.0 "
        f"mode_switch={mode_switch} mode_us={'1700' if mode_switch == 'AUTO' else '1100'} "
        f"auto_sw={'true' if mode_switch == 'AUTO' else 'false'} mode_channel_present=true\n"
    ).encode("ascii")


def test_mode_switch_state_reads_auto_manual_absent() -> None:
    """mode_switch_state 가 AUTO/MANUAL/ABSENT 를 정확히 판별 / Reads AUTO, MANUAL, and ABSENT states."""
    auto = controller.telemetry.parse_usbdbg_rows(_heartbeat_with_mode("AUTO").decode())[0]
    manual = controller.telemetry.parse_usbdbg_rows(_heartbeat_with_mode("MANUAL").decode())[0]
    assert controller.mode_switch_state(auto) == "AUTO"
    assert controller.mode_switch_state(manual) == "MANUAL"
    assert controller.mode_switch_state({"event": "HEARTBEAT"}) == "ABSENT"


def test_run_controller_aborts_when_mode_switch_returns_to_manual() -> None:
    """require_auto_switch 에서 스위치가 MANUAL 이면 USER_SWITCHED_TO_MANUAL 로 중단 / Manual switch aborts an AUTO-gated run."""
    handle = FakeSerial([_heartbeat_with_mode("MANUAL")])
    rows, _raw, abort_reason = controller.run_controller(
        handle,
        segments=[_east_lane()],
        resolved_calibration=geometry.FALLBACK_RESOLVED_CALIBRATION,
        start_lat=35.0,
        start_lon=129.0,
        start_yaw_deg=0.0,
        goal_lat=35.0000100,
        goal_lon=129.0,
        event_timeout_s=0.1,
        heartbeat_timeout_s=0.1,
        straight_motion_mode="continuous",
        require_auto_switch=True,
    )
    assert abort_reason == controller.MANUAL_SWITCH_ABORT_REASON == "USER_SWITCHED_TO_MANUAL"
    assert rows == []


def test_controller_summary_reports_correction_enabled_and_applied() -> None:
    """폐루프 모드+비영 보정성분->enabled&applied=True; 개루프->둘 다 False / Summary flags closed-loop correction usage."""
    applied = controller.build_controller_summary(
        [
            {
                "row_type": "pulse",
                "valid_pulse": True,
                "path_control_mode": "gps_imu_closed_loop",
                "b_heading_component": "-0.05",
                "b_cte_component": "0.0",
                "drive_mode": "continuous",
            }
        ],
        start_lat=35.0, start_lon=129.0, goal_lat=35.001, goal_lon=129.0,
        goal_distance_m=5.0, fallback_to_repeated_pulses=False,
    )
    assert applied["closed_loop_correction_enabled"] is True
    assert applied["closed_loop_correction_applied"] is True

    open_loop = controller.build_controller_summary(
        [
            {
                "row_type": "pulse",
                "valid_pulse": True,
                "path_control_mode": "open_loop_chunks",
                "b_heading_component": "0.0",
                "b_cte_component": "0.0",
            }
        ],
        start_lat=35.0, start_lon=129.0, goal_lat=35.001, goal_lon=129.0,
        goal_distance_m=5.0, fallback_to_repeated_pulses=False,
    )
    assert open_loop["closed_loop_correction_enabled"] is False
    assert open_loop["closed_loop_correction_applied"] is False
