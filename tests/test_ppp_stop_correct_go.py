"""stop_correct_go 컨트롤러 모드 계약 테스트 — 순수 헬퍼 + 목(mock) 시리얼 루프.

목적/역할:
    ``controller.run_stop_correct_go`` 가 고정하는 "멈춤 -> 보정 -> 전진" 이산 루프의
    관측 가능 행동(계약)을 잠근다. 순수 결정 헬퍼(sensor source / heading decision /
    cross-track trim / turn-error)는 직접 호출해 검증하고, 감독 루프는 스크립트된
    가짜 시리얼로 구동한다.

시스템 내 위치:
    ``tools.physical_path_planning.controller`` 와 ``geometry`` 만 import 한다. 실제
    펌웨어/시리얼은 건드리지 않으며, run_controller 쪽 연속-드라이브 계약은 자매
    파일 ``test_ppp_controller`` 가 담당한다. 이 파일은 stop_correct_go 경로 전용이다.

핵심 개념·불변식:
  - MOVE 는 보정된 전진 A(throttle)를 절대 낮추지 않는다; 조향은 B(turn) 축만 만진다.
  - A/B 매핑 고정: B > 0 = 좌회전, B < 0 = 우회전. cross-track/heading 보정은 +/-0.08 클램프.
  - 코너/헤딩 보정은 burst -> stop -> measure 사이클(정지 후 IMU 측정)로만 돈다. 이유(WHY):
    모터가 도는 동안 MOTOR_TRACE 가 UART 를 포화시켜 yaw 하트비트가 살아남지 못하기 때문.
  - 모든 요약(summary)은 ``ready_for_full_path_following=False`` 를 유지해야 한다.

테스트 실행상 불변식(중요):
    자매 fixture ``_no_turn_sleep`` 로 회전 버스트의 실시간 sleep 을 무력화하고,
    루프 인자 ``settle_after_move_ms=0`` / ``telemetry_stabilize_ms=0`` 으로 벽시계
    대기를 제거한다 -- 따라서 테스트는 즉시 실행된다(시간에 의존하지 않는다).

------------------------------------------------------------------------------
Tests for the stop_correct_go controller mode (pure helpers + mock-serial loop).

The pure decision helpers are exercised directly; the supervised loop is driven
with a scripted fake serial and ``settle_after_move_ms=0`` /
``telemetry_stabilize_ms=0`` so no wall-clock sleeping occurs.
"""

from __future__ import annotations

import pytest

from tools.physical_path_planning import controller, geometry


# ── Fixtures / 픽스처 ──


@pytest.fixture(autouse=True)
def _no_turn_sleep(monkeypatch: pytest.MonkeyPatch) -> None:
    """회전 버스트의 실시간 sleep 을 제거해 전 테스트를 즉시 실행 / Stub the turn-burst sleep."""
    # Turn bursts sleep in real time on hardware; tests run them instantly.
    monkeypatch.setattr(controller, "_turn_sleep", lambda _s: None)


# ── 스크립트된 시리얼 / Scripted serial (fake handle + telemetry builders) ──


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
    """한 줄짜리 하트비트 텔레메트리 바이트 생성 / Build one heartbeat telemetry line.

    ``with_gps`` / ``with_imu`` 를 끄면 해당 센서 필드를 생략해 "센서 없음" 상황을 흉내낸다.
    """
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
    """정북(ENU 90도) 방향 직선 lane 세그먼트 / Build a due-north forward lane."""
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


# ── 순수 헬퍼: 센서 소스 선택 / Pure helper: sensor source ──


def test_sensor_source_uses_both_when_available() -> None:
    """GPS+IMU 둘 다 살아 있으면 source=gps_imu, 폴백 아님 / Both sensors -> 'gps_imu', no fallback."""
    out = controller.stop_correct_go_sensor_source(gps_valid=True, imu_valid=True)
    assert out["ok"] is True
    assert out["source"] == "gps_imu"
    assert out["fallback_used"] is False


def test_sensor_source_single_sensor_labels() -> None:
    """센서 하나만 있으면 그 이름으로 라벨링(gps / imu) / One-sensor case labels 'gps' or 'imu'."""
    assert controller.stop_correct_go_sensor_source(gps_valid=True, imu_valid=False)["source"] == "gps"
    assert controller.stop_correct_go_sensor_source(gps_valid=False, imu_valid=True)["source"] == "imu"


def test_sensor_source_aborts_when_both_gone_and_fallback_disallowed() -> None:
    """둘 다 사라지고 폴백 불허 -> SENSOR_UNAVAILABLE 로 중단 / Both gone + no fallback -> abort."""
    out = controller.stop_correct_go_sensor_source(
        gps_valid=False, imu_valid=False, trust_mode="imu_gps_first", allow_calibration_fallback=False
    )
    assert out["ok"] is False
    assert out["reason"] == "SENSOR_UNAVAILABLE"


def test_sensor_source_calibration_fallback_when_allowed_or_trusted() -> None:
    """명시 허용 또는 trust=calibration_fallback 이면 캘리브레이션 폴백 사용 / Fallback when allowed or trusted."""
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


# ── 순수 헬퍼: 헤딩 보정 결정 / Pure helper: heading decision ──


def test_heading_decision_turns_left_for_positive_error() -> None:
    """양(+) 헤딩 오차 -> 좌회전(B=+0.24) / Positive heading error commands a left turn."""
    out = controller.stop_correct_go_heading_decision(
        heading_error_deg=20.0, threshold_deg=8.0, imu_valid=True, b_left=0.24, b_right=-0.12
    )
    assert out["needs_correction"] is True
    assert out["turn_direction"] == "left"
    assert out["correction_b_cmd"] == 0.24  # B > 0 is left, per the locked A/B mapping


def test_heading_decision_turns_right_for_negative_error() -> None:
    """음(-) 헤딩 오차 -> 우회전(B=-0.12) / Negative heading error commands a right turn."""
    out = controller.stop_correct_go_heading_decision(
        heading_error_deg=-20.0, threshold_deg=8.0, imu_valid=True, b_left=0.24, b_right=-0.12
    )
    assert out["turn_direction"] == "right"
    assert out["correction_b_cmd"] == -0.12  # B < 0 is right


def test_heading_decision_skipped_below_threshold_or_without_imu() -> None:
    """임계값 미만이거나 IMU 없음 -> 보정 안 함 / Skip correction below threshold or without IMU."""
    below = controller.stop_correct_go_heading_decision(
        heading_error_deg=5.0, threshold_deg=8.0, imu_valid=True, b_left=0.24, b_right=-0.12
    )
    assert below["needs_correction"] is False
    no_imu = controller.stop_correct_go_heading_decision(
        heading_error_deg=40.0, threshold_deg=8.0, imu_valid=False, b_left=0.24, b_right=-0.12
    )
    assert no_imu["needs_correction"] is False


# ── 순수 헬퍼: 크로스트랙 트림 (B 클램프 +/-0.08) / Pure helper: cross-track trim ──


def test_cross_track_trim_zero_below_threshold() -> None:
    """임계 거리 미만 크로스트랙은 트림 0 / Cross-track below threshold yields zero trim."""
    assert (
        controller.stop_correct_go_cross_track_trim(cross_track_error_m=0.2, threshold_m=0.35) == 0.0
    )


def test_cross_track_trim_clamped_to_max_and_signed() -> None:
    """큰 크로스트랙은 부호 유지한 채 +/-max 로 클램프 / Large offset clamps to signed +/-max."""
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
    """회전한 만큼 남은 오차가 부호 유지하며 줄어듦 / Remaining turn error shrinks, keeping sign."""
    # Started 20 deg off, rotated 12 deg toward target -> 8 deg remains (same sign).
    assert controller.remaining_turn_error_deg(20.0, 12.0) == 8.0
    assert controller.remaining_turn_error_deg(-20.0, 12.0) == -8.0


# ── 루프: 단일 청크 lane 정상 완주 / Loop: clean single-chunk lane completion ──


def test_run_stop_correct_go_completes_one_lane_clean() -> None:
    """정상 lane 1개를 한 청크로 완주: A=0.300 유지, B 중립 시작 / Clean one-lane completion."""
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
    """전진 후 두 센서 모두 상실 + 폴백 불허 -> 중단, 행(row) 미기록 / Both sensors lost -> abort, no row."""
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
    """GPS 열화 시 IMU 로 헤딩 유지하며 캘리브레이션 추측항법으로 전진 / Dead-reckon on GPS degrade."""
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


# ── 커넥터 회전: target_angle 인지 다중 펄스 + IMU 정지 / Connector turns ──


def _small_left_turn_calibration(target_angle_deg: float = 30.0) -> dict[str, object]:
    """작은-각(기본 30도) 좌회전 캘리브레이션 dict / Small-angle left-turn calibration."""
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
    """동->북 90도 좌회전 코너(커넥터) 세그먼트 / A quarter-turn left connector segment."""
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
    """커넥터 회전 계획 헬퍼(펄스당 각도/펄스 수/예산/방향)의 산술 계약 / Turn-planning helper arithmetic."""
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


def test_connector_live_turn_stops_on_measured_imu_target() -> None:
    """IMU 있으면 코너를 라이브 SET 피벗으로 돌려 측정 각도 도달 시 정지(가드펄스 미사용) / Live IMU turn stops at target."""
    # A corner is ONE continuous IMU-feedback pivot: bounded live SET commands
    # at the calibrated B until the measured yaw delta reaches the requested
    # angle. No long guarded pulse is sent, so the firmware's guarded-pulse
    # max (COMMAND_EXCEEDS_MAX_MS) can never reject a 2200 ms calibration.
    handle = FakeSerial(
        [
            _hb(35.0, 129.0, imu_yaw_deg=0.0),  # connector preflight
            b"USB_PULSE_TEST event=STOP final_left_cmd=0.000 final_right_cmd=0.000 physical_output_active=false\n",  # burst 1 stop confirm
            _hb(35.0, 129.0, imu_yaw_deg=88.0),  # stationary measure: within tolerance
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
    assert len(connector_rows) == 1
    row = connector_rows[0]
    assert row["turn_mode"] == "live_imu"
    assert row["connector_turn_completed"] is True
    assert row["turn_measured_by_imu"] is True
    assert float(row["applied_turn_delta_deg"]) == 88.0
    set_writes = [w for w in handle.writes if w.startswith("USB_DRIVE_LIVE_SET")]
    assert len(set_writes) == 1  # one burst was enough
    assert "a=0.000" in set_writes[0] and "b=0.240" in set_writes[0]
    assert "duration_ms=1000" in set_writes[0]  # 90 deg at ~50 dps, capped at the burst max
    assert any(w.startswith("USB_DRIVE_LIVE_STOP") for w in handle.writes)
    # The connector never issues a guarded pulse command.
    assert not any(w.startswith("USB_PULSE_TEST_CMD") for w in handle.writes)


def test_connector_without_imu_uses_open_loop_pulse_count_from_target_angle() -> None:
    """IMU 없으면 90/30 올림=3펄스 개루프로 회전(추가 blind 펄스 없음) / No IMU -> open-loop pulse count."""
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


def test_connector_live_turn_times_out_on_stalled_motors() -> None:
    """모터 정지(회전 0)면 라이브 회전이 시간 상한에서 멈추고 미완료로 보고 / Stalled motors -> timeout, incomplete."""
    # IMU reports no rotation (stalled motors): the live turn must stop at its
    # duration cap instead of spinning forever, and be reported as incomplete.
    handle = FakeSerial(
        [
            _hb(35.0, 129.0, imu_yaw_deg=0.0),  # connector preflight
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
        event_timeout_s=0.2,
        heartbeat_timeout_s=0.2,
        max_connector_turn_ms=200,
    )

    assert abort_reason == "NONE"
    connector_rows = [r for r in rows if r.get("phase") == "connector"]
    assert len(connector_rows) == 1
    row = connector_rows[0]
    assert row["turn_timed_out"] is True
    assert row["connector_turn_completed"] is False
    assert float(row["applied_turn_delta_deg"]) == 0.0
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
    assert summary["connector_incomplete_count"] == 1


# ── 헤딩 기준: 미션 체인 vs 레거시 lane별 재캡처 / Heading reference: mission vs per-lane ──


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
    """덜-회전한 코너 시나리오의 스크립트 시리얼 생성 (동 lane -> 40/90도 커넥터 -> 북 lane).

    ``with_correction_rows`` 가 True 면 다음 lane 이 잔여 오차를 보정하는 버스트 응답까지 포함한다.

    Lane east -> connector that only turns 40 of 90 deg -> lane north.
    """
    responses = [
        # lane 1, chunk 1: completes at the east end, yaw matches lane heading.
        _hb(35.0, 129.0, imu_yaw_deg=0.0),
        *_PULSE_OK,
        _hb(35.0, _LON_1M_EAST, imu_yaw_deg=0.0),
        # connector live bursts: rotation stalls at 40 of 90 deg -- burst 2
        # measures no progress, so the turn stops and records the under-turn.
        _hb(35.0, _LON_1M_EAST, imu_yaw_deg=0.0),  # connector preflight
        b"USB_PULSE_TEST event=STOP final_left_cmd=0.000 final_right_cmd=0.000 physical_output_active=false\n",  # burst 1 stop confirm
        _hb(35.0, _LON_1M_EAST, imu_yaw_deg=40.0),  # measure: 40 of 90
        b"USB_PULSE_TEST event=STOP final_left_cmd=0.000 final_right_cmd=0.000 physical_output_active=false\n",  # burst 2 stop confirm
        _hb(35.0, _LON_1M_EAST, imu_yaw_deg=40.0),  # measure: stalled
        # lane 2, chunk 1: still pointing 40 deg while the lane needs 90.
        _hb(35.0, _LON_1M_EAST, imu_yaw_deg=40.0),
        *_PULSE_OK,
        _hb(_LAT_HALF_M_NORTH, _LON_1M_EAST, imu_yaw_deg=40.0),
    ]
    if with_correction_rows:
        responses.extend(
            [
                b"USB_PULSE_TEST event=STOP final_left_cmd=0.000 final_right_cmd=0.000 physical_output_active=false\n",  # correction burst stop confirm
                _hb(_LAT_HALF_M_NORTH, _LON_1M_EAST, imu_yaw_deg=88.0),  # measure
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
    """공유 3-세그먼트 미션을 주어진 heading_reference 모드로 실행 / Run the shared 3-segment mission."""
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
        heading_reference=heading_reference,
    )


def test_mission_heading_reference_exposes_connector_under_turn() -> None:
    """mission 기준: 커넥터 덜-회전 50도 잔차가 다음 lane 에서 드러나 보정됨 / Mission ref exposes under-turn."""
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
    """per_lane 기준(레거시): lane 이 기준 yaw 를 재캡처해 같은 덜-회전이 0 오차로 흡수됨 / Per-lane absorbs under-turn."""
    # Legacy behavior kept behind --heading-reference per_lane: the lane
    # re-captures its reference yaw, so the same under-turn reads as zero error.
    handle = FakeSerial(_under_turned_corner_responses(with_correction_rows=False))
    rows, _raw, abort_reason = _run_under_turned_corner(handle, "per_lane")

    assert abort_reason == "NONE"
    lane2_rows = [r for r in rows if r["segment_index"] == 3]
    assert lane2_rows
    assert all(r["phase"] == "move" for r in lane2_rows)
    assert all(float(r["heading_error_deg"]) == 0.0 for r in lane2_rows)
    # No lane heading-correction turn ever ran (the connector's live pivot is
    # the only live-drive user in this script).
    assert all(int(r.get("correction_duration_ms") or 0) == 0 for r in lane2_rows)


# ── 수동 전환 중단 · 요약 카운터 · 후진 lane 헤딩 유지 / Manual-abort, summary, backward-lane ──


def test_manual_switch_during_pulse_aborts_immediately() -> None:
    """펄스 창 안에서 스위치가 MANUAL 로 돌아오면 즉시 중단(보정 미시도) / Manual grab mid-pulse aborts at once."""
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
    """요약이 mode·trust·카운터를 채우고 ready_for_full_path_following=False 유지 / Summary keeps ready False."""
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
    """후진 lane 은 몸체(body) 헤딩(0도)을 유지: +20도 드리프트를 우회전으로 보정 / Mission holds body heading in reverse."""
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
            b"USB_PULSE_TEST event=STOP final_left_cmd=0.000 final_right_cmd=0.000 physical_output_active=false\n",  # correction burst stop confirm
            _hb(None, None, imu_yaw_deg=2.0, with_gps=False),  # stationary measure
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


def test_connector_live_turn_stops_on_overshoot() -> None:
    """오버슈트로 잔여 오차 부호가 뒤집히면 같은 방향 회전을 멈추고 다음 lane 이 정리 / Stop turning on overshoot."""
    # High-variance motors can blow past the corner between feedback samples
    # (field data: ~80 deg/s right turns). Once the measured remaining error
    # flips sign the live turn must stop turning the same direction; the next
    # lane's heading correction owns the cleanup.
    handle = FakeSerial(
        [
            _hb(35.0, 129.0, imu_yaw_deg=0.0),  # connector preflight
            b"USB_PULSE_TEST event=STOP final_left_cmd=0.000 final_right_cmd=0.000 physical_output_active=false\n",  # burst 1 stop confirm
            _hb(35.0, 129.0, imu_yaw_deg=115.0),  # stationary measure: 25 past the target
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
    assert len(connector_rows) == 1
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
