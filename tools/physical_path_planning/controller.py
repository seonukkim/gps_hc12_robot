"""Continuous-motion controller: the supervised pulse loop that follows a planned
serpentine/direct path toward the goal.

This is the one genuinely new module in the consolidation, but it invents no new
control law. Each step REUSES the already-tested pieces:

* :func:`executor.send_pulse` issues the guarded ARM->command->STOP pulse;
* :func:`geometry.projection_metrics` + :func:`geometry.compute_b_correction`
  produce the steering correction -- applied to the **B axis only**, clamped to
  +/-0.08; the forward A command and pulse duration come straight from
  calibration and are **never lowered** by the controller;
* :func:`geometry.gps_policy_action` decides what to do when GPS degrades.

New (loop-level) behavior layered on top of those primitives:

* IMU yaw is sampled on every heartbeat to hold the segment's target heading
  between pulses (``current_heading`` is the segment heading plus the measured
  yaw delta from the start yaw).
* GPS degraded (``BAD_HDOP`` / no fix / cached) -> by default *continue
  dead-reckoned* from the last good fix and flag ``gps_degraded=True``; it does
  not silently stop. The policy is configurable (continue/pause/abort).
* Serial disconnect mid-loop -> hard abort (no traceback): the loop records
  ``abort_reason`` and stops.
* ``RC_INVALID`` reported during the active pulse -> hard abort.
* RC sticks not neutral before a pulse -> wait up to ``rc_neutral_wait_s``
  (default 5s) for neutral; if it never settles, abort rather than pulse.
* Connectors prefer angle-calibrated turns; when no turn-angle calibration
  exists the resolver's ``repeated_pulses`` fallback is used and
  ``fallback_to_repeated_pulses`` is surfaced. The controller never invents 15
  degree micro-turns as a preferred connector.

Every emitted summary is routed through
:func:`checks.assert_not_ready_for_full_path_following`, so the controller can
never claim full-path-following readiness.

--------------------------------------------------------------------------------
[한국어 개요 / Korean overview]

목적/역할:
    계획된 커버리지 경로(직선 lane + 코너 connector)를 감독하에 따라가는 세그먼트
    루프. 이 파일은 두 개의 상위 드라이브 루프를 담는다:
      * :func:`run_controller` -- 연속 모션(gps_imu_closed_loop 등) 청크 드라이브.
      * :func:`run_stop_correct_go` -- 이 모듈의 핵심 개념인 `stop_correct_go` 모드.
    새로운 제어 법칙(control law)을 발명하지 않고, 이미 검증된 하위 프리미티브
    (executor.send_pulse / geometry / safety / telemetry)를 재사용한다.

핵심 개념 -- stop_correct_go:
    "보정된(calibrated) 한 청크만큼 전진 -> 완전 정지 -> 안정화된 GPS/IMU
    하트비트를 읽음 -> **정지한 상태에서** 헤딩을 보정"하는 이산(discrete) 루프.
    코너 회전과 헤딩 보정은 burst->stop->measure(버스트 회전 -> 정지 -> 측정)
    사이클로 돌린다. 이유(WHY): 모터가 도는 동안 펌웨어의 MOTOR_TRACE 스트림이
    시리얼 링크(UART)를 포화시켜 yaw가 실린 하트비트가 거의 살아남지 못하기
    때문 -- 즉 "움직이면서 보는" 연속 피드백은 실전에서 사실상 눈먼 상태다
    (필드 데이터: 모든 커넥터가 시간 상한까지 돌았음). 그래서 반드시 멈춘 뒤에
    측정한다. 전진 A(throttle)는 절대 낮추지 않고 조향 B(turn)만 만진다.

불변식/함정(invariants & gotchas):
    * 조향은 **B 축에만** 적용하고 +/-max_correction_b(기본 0.08)로 clamp한다.
      전진 A와 펄스 길이는 캘리브레이션 값 그대로이며 컨트롤러가 낮추지 않는다.
    * "mission" 헤딩 프레임은 하나의 yaw 기준을 런 전체에 이어 붙여, 덜 돈
      커넥터 오차가 다음 lane의 헤딩 오차로 드러나 보정되게 한다. "per_lane"은
      lane마다 기준 yaw를 다시 잡아 커넥터 오차를 조용히 흡수하는 레거시 동작.
    * 예상되는 필드 결함(시리얼 끊김, RC invalid, GPS 정책, 하트비트 없음)은
      raise하지 않고 abort_reason만 세팅한 뒤 루프를 깨끗이 멈춘다 -- 마지막
      STOP/영점 핸드셰이크가 여전히 실행되도록.
    * 모든 요약(summary)은 assert_not_ready_for_full_path_following를 통과하므로,
      이 컨트롤러는 절대 "전체 경로 추종 준비 완료"를 주장할 수 없다(안전 규약).

시스템 내 위치:
    CLI(tools.physical_path_planning cli)의 상위 실행 모드가 이 두 진입점을
    호출한다. 이 파일은 executor(시리얼 펄스), geometry(투영/보정 수학),
    safety(정지/무효 판정), telemetry(행 파싱), checks(준비도 가드)를 import.

리팩토링 노트:
    순수 결정 헬퍼(no serial) 영역과 시리얼을 만지는 루프 영역이 나뉘어 있다.
    새 제어 규칙을 넣을 때는 순수 헬퍼(pulse_correction, *_decision 등)에 로직을
    두고 루프는 조립만 하도록 유지하면 단위 테스트가 쉽다. burst 길이/상한 상수,
    turn-rate 추정, MOTOR_TRACE 포화 가정이 강하게 결합돼 있으니 함께 바꿀 것.

[EN] Supervised segment loop that follows a planned coverage path. Two top-level
drive loops live here: :func:`run_controller` (continuous chunked drive) and
:func:`run_stop_correct_go` (the headline `stop_correct_go` mode: drive one
bounded calibrated chunk, fully stop, read a stabilized GPS/IMU heartbeat, then
correct heading *while stopped*). Turns and heading corrections run as
burst->stop->measure cycles because the firmware's MOTOR_TRACE stream saturates
the serial link during motion, so in-motion yaw feedback is effectively blind.
Steering touches the B axis only (clamped); forward A and pulse length come from
calibration and are never lowered. Expected field faults set ``abort_reason``
instead of raising, and every summary is gated so readiness can never be claimed.
"""
from __future__ import annotations

import math
import time
from typing import Sequence

from tools.physical_path_planning import checks, executor, geometry, safety, telemetry

DEFAULT_EVENT_TIMEOUT_S = 3.0
DEFAULT_HEARTBEAT_TIMEOUT_S = 3.0
DEFAULT_RC_NEUTRAL_WAIT_S = 5.0
DEFAULT_GPS_DEGRADATION_POLICY = "continue"
DEFAULT_MANUAL_OVERRIDE_MODE = "abort"
_ZERO_TOLERANCE = 1e-9
# ── 경로 제어 모드 상수 / Path-control mode constants ──
# 폐루프 강도가 낮은 순 -> 높은 순: open_loop_chunks(보정 없음) < imu_heading
# (헤딩만) < gps_imu_closed_loop(헤딩+횡오차) / stop_correct_go(정지-보정-전진).
# EN: modes ordered by closed-loop strength; stop_correct_go is the discrete
# stop->correct->go variant driven by run_stop_correct_go.
DEFAULT_PATH_CONTROL_MODE = "gps_imu_closed_loop"
PATH_CONTROL_MODES = {
    "open_loop_chunks",
    "imu_heading",
    "gps_imu_closed_loop",
    "stop_correct_go",
}

# Abort reason raised when the operator flips the physical mode switch back to
# MANUAL during an AUTO-triggered run (auto-relative-run).
MANUAL_SWITCH_ABORT_REASON = "USER_SWITCHED_TO_MANUAL"

# ── stop_correct_go 기본값 / stop_correct_go defaults ──
# 이산 "전진 -> 정지 -> 측정 -> 보정" 루프의 튜닝값. 조향(B 축)만 하고 제자리
# 회전만 하며, 보정된 전진 A는 절대 낮추지 않는다.
# stop_correct_go defaults: a discrete move -> stop -> measure -> correct loop
# that only steers (B axis) and turns in place, never lowering the calibrated A.
DEFAULT_MOVE_CHUNK_MS = 700
DEFAULT_SETTLE_AFTER_MOVE_MS = 300
DEFAULT_TELEMETRY_STABILIZE_MS = 500
DEFAULT_HEADING_CORRECTION_THRESHOLD_DEG = 8.0
DEFAULT_HEADING_CORRECTION_TOLERANCE_DEG = 5.0
DEFAULT_CROSS_TRACK_CORRECTION_THRESHOLD_M = 0.35
DEFAULT_HEADING_CORRECTION_B_LEFT = 0.24
DEFAULT_HEADING_CORRECTION_B_RIGHT = -0.12
# Turns run as burst -> stop -> measure cycles (see _run_stop_correct_go_heading_turn);
# each cycle costs the burst plus ~0.8 s of settle/stabilize, so the caps cover
# several cycles. The slow left turn (~13 deg/s at B=0.24) needs ~7 s of rotation
# alone for a quarter turn.
DEFAULT_MAX_HEADING_CORRECTION_MS = 8000
DEFAULT_HEADING_CORRECTION_CHUNK_MS = 300
MIN_TURN_BURST_MS = 250
# One burst is capped at the firmware-safe live-drive duration (upload-baked).
DEFAULT_TURN_BURST_MAX_MS = 1000
DEFAULT_TURN_RATE_GUESS_DPS = 45.0
TURN_STALL_MIN_PROGRESS_DEG = 3.0
# Test hook: bursts sleep in real time on hardware; unit tests stub this out.
_turn_sleep = time.sleep
SENSOR_TRUST_MODES = {"imu_gps_first", "calibration_fallback"}
DEFAULT_SENSOR_TRUST_MODE = "imu_gps_first"

# Connector turn semantics: the calibrated turn primitive is a PULSE whose real
# per-pulse rotation is target_angle_deg (often 15-45 degrees on this rover,
# despite the turn_*_90 key name). A 90 degree planned corner therefore takes
# repeated pulses, stopped early by IMU yaw feedback when available.
TURN_ANGLE_POLICIES = {"from_json", "assume_90"}
DEFAULT_TURN_ANGLE_POLICY = "from_json"
DEFAULT_MAX_CONNECTOR_PULSES_PER_TURN = 6
DEFAULT_CONNECTOR_TURN_TOLERANCE_DEG = 10.0
# Cap for one connector's burst->measure pivot sequence. The slow left turn on
# this rover (~13 deg/s at B=0.24) needs ~7 s of rotation plus measure pauses.
DEFAULT_MAX_CONNECTOR_TURN_MS = 20000
# Firmware-safe single guarded-pulse duration for the no-IMU fallback: the
# flashed guarded-pulse firmware rejects longer commands (COMMAND_EXCEEDS_MAX_MS).
OPEN_LOOP_CONNECTOR_PULSE_MS_CAP = 1000
# How lane heading errors are referenced. "mission" chains one yaw reference
# across the whole run, so an under-turned connector shows up (and gets
# corrected) on the next lane. "per_lane" is the legacy behavior: each lane
# re-captures its reference yaw, silently absorbing any connector error.
HEADING_REFERENCES = {"mission", "per_lane"}
DEFAULT_HEADING_REFERENCE = "mission"


# ── 순수 결정 헬퍼 / Pure decision helpers (no serial; directly unit-testable) ──
# 시리얼을 만지지 않는 순수 함수들 -- 제어 판단 로직을 여기 모아 단위 테스트가
# 하드웨어 없이 가능하게 한다. 루프는 이들을 조립만 한다.
# EN: side-effect-free decision logic kept separate from the serial loop so it
# can be unit-tested without hardware.


def _row_lat_lon(row: dict[str, str] | None) -> tuple[float | None, float | None]:
    """텔레메트리 행에서 위도/경도 추출 (current_*/gps_* 키 모두 허용).

    Latitude/longitude from a telemetry row, accepting ``current_*``/``gps_*``.
    """
    if row is None:
        return None, None
    lat = telemetry._optional_float(row.get("current_lat", row.get("gps_lat")))
    lon = telemetry._optional_float(row.get("current_lon", row.get("gps_lon")))
    return lat, lon


def mode_switch_state(row: dict[str, str] | None) -> str:
    """하트비트 행에서 물리 PPM 모드 스위치 상태를 읽는다: AUTO / MANUAL / ABSENT.

    Physical PPM mode switch state from a heartbeat row: AUTO / MANUAL / ABSENT.

    Prefers the firmware's ``mode_switch=AUTO|MANUAL`` string; falls back to the
    ``auto_sw`` boolean gated by ``mode_channel_present``. Returns ``ABSENT`` when
    no usable mode channel is reported (no PPM receiver), which lets the caller
    offer the keyboard-start fallback instead of waiting forever for a switch.
    """
    if row is None:
        return "ABSENT"
    label = str(row.get("mode_switch", "")).strip().upper()
    if label in {"AUTO", "MANUAL"}:
        return label
    present_value = row.get("mode_channel_present")
    if present_value is not None and not telemetry._parse_bool(present_value, default=False):
        return "ABSENT"
    auto_value = row.get("auto_sw")
    if auto_value is None:
        return "ABSENT"
    return "AUTO" if telemetry._parse_bool(auto_value, default=False) else "MANUAL"


def dead_reckon_gps(row: dict[str, str] | None, cache: dict[str, object]) -> dict[str, object]:
    """이번 스텝의 사용 가능한 위경도 확정 -- 열화 시 캐시로 추측항법(dead-reckon).

    Resolve usable lat/lon for this step, dead-reckoning from cache when degraded.

    Mirrors the legacy stage35 ``_gps_fields`` semantics: a fresh fix updates the
    cache and clears degradation; a missing/blocked fix falls back to the last
    cached fix (``gps_cached_used``); ``BAD_HDOP`` or any cache use marks
    ``gps_degraded`` so the caller can flag the step without stopping.
    """
    lat, lon = _row_lat_lon(row)
    reason = (row or {}).get("gps_block_reason", "NA")
    cached_used = False
    recovered = False
    if lat is not None and lon is not None:
        recovered = bool(cache.get("degraded"))
        cache["lat"] = lat
        cache["lon"] = lon
        cache["degraded"] = False
    elif cache.get("lat") is not None and cache.get("lon") is not None:
        lat = float(cache["lat"])
        lon = float(cache["lon"])
        cached_used = True
        cache["degraded"] = True
    else:
        cache["degraded"] = True
    degraded = reason == "BAD_HDOP" or cached_used or lat is None or lon is None
    return {
        "lat": lat,
        "lon": lon,
        "gps_cached_used": cached_used,
        "gps_degraded": degraded,
        "gps_recovered": recovered,
    }


def current_heading_deg(
    target_heading_deg: float, yaw: float | None, start_yaw_deg: float | None
) -> float:
    """세그먼트 목표 헤딩을 시작 yaw 대비 측정된 yaw 드리프트만큼 보정한 값.

    Segment heading adjusted by the measured yaw drift since the start yaw.
    """
    if yaw is None or start_yaw_deg is None:
        return target_heading_deg
    return geometry.wrap_deg(target_heading_deg + geometry.wrap_deg(yaw - start_yaw_deg))


def reference_yaw_for_segment(
    heartbeat_row: dict[str, str] | None,
    *,
    provided_start_yaw: float | None,
    use_provided: bool,
) -> float | None:
    """lane가 헤딩을 유지할 기준이 되는 IMU yaw 기준값을 확정한다.

    [KO] 헤딩 유지는 lane을 시작할 때의 방향에서 벗어난 드리프트를 측정하므로,
    기준은 lane마다 다시 잡아야 한다(커넥터 회전 후 절대 yaw가 ~90도 바뀌므로).
    --start-yaw-deg가 없으면(현장 기본값) 각 lane의 첫 하트비트 yaw를 기준으로
    삼아 IMU 헤딩 보정이 살아 있게 한다 -- 그렇지 않으면 조용히 헤딩 오차 0을 낸다.

    Heading-hold measures drift away from the orientation the robot had when it
    started the lane, so the reference must be re-captured per lane (after each
    connector turn the absolute yaw has changed by ~90 degrees).

    * ``use_provided`` (the first lane) with an explicit ``--start-yaw-deg`` keeps
      that operator-supplied reference.
    * Otherwise the lane's first heartbeat yaw becomes the reference -- this is
      what makes IMU heading correction live when no ``--start-yaw-deg`` is given
      (the field default), instead of silently producing a zero heading error.
    * When no IMU yaw is available the provided value (possibly ``None``) is used,
      which disables heading correction for that lane rather than guessing.
    """
    if use_provided and provided_start_yaw is not None:
        return provided_start_yaw
    yaw = telemetry.imu_relative_yaw_deg(heartbeat_row) if heartbeat_row else None
    if yaw is not None:
        return yaw
    return provided_start_yaw


def pulse_correction(
    *,
    segment: dict[str, object],
    x: float,
    y: float,
    target_heading_deg: float,
    yaw: float | None,
    start_yaw_deg: float | None,
    is_connector: bool = False,
    base_b_cmd: float = 0.0,
    connector_b_cmd: float = 0.0,
    imu_heading_hold: bool = True,
    cross_track_correction: bool = True,
    path_control_mode: str = DEFAULT_PATH_CONTROL_MODE,
    k_heading: float = 0.006,
    k_cross_track: float = 0.20,
    max_correction_b: float = 0.08,
) -> dict[str, float]:
    """펄스마다의 조향 보정을 계산한다 (B 축 전용).

    [KO] lane 세그먼트에서 B 명령 = 헤딩 오차 + 횡오차(cross-track) 보정
    (+/-0.08로 clamp). 커넥터에서는 B = 보정된 회전값이고 보정 성분은 0으로 둔다
    -- 커넥터는 조향 nudge가 아니라 의도된 회전이기 때문. 모드에 따라 헤딩/횡오차
    성분을 끄는(0으로 만드는) 게이팅이 여기서 일어난다(open_loop/imu_heading 등).

    For lane segments the B command is the heading+cross-track correction from
    :func:`geometry.compute_b_correction` (clamped to +/-0.08). For connectors the
    B command is the calibrated turn value and the correction components are zero
    (a connector is a deliberate turn, not a steering nudge).
    """
    if path_control_mode not in PATH_CONTROL_MODES:
        path_control_mode = DEFAULT_PATH_CONTROL_MODE
    heading_enabled = imu_heading_hold and path_control_mode in {"imu_heading", "gps_imu_closed_loop"}
    cte_enabled = cross_track_correction and path_control_mode == "gps_imu_closed_loop"
    heading = current_heading_deg(target_heading_deg, yaw if heading_enabled else None, start_yaw_deg)
    heading_error = geometry.wrap_deg(target_heading_deg - heading)
    along, signed_cte, _ = geometry.projection_metrics(segment, x, y)
    if not heading_enabled:
        heading_error = 0.0
    if not cte_enabled:
        signed_cte = 0.0
    b_heading = float(k_heading) * heading_error
    b_cte = float(k_cross_track) * signed_cte
    correction = geometry.clamp(b_heading + b_cte, -abs(max_correction_b), abs(max_correction_b))
    if is_connector:
        b_cmd = float(connector_b_cmd)
        b_heading = 0.0
        b_cte = 0.0
    else:
        b_cmd = geometry.clamp(
            float(base_b_cmd) + correction,
            -abs(max_correction_b),
            abs(max_correction_b),
        )
    if is_connector:
        correction_source = "connector_calibration"
    elif path_control_mode == "open_loop_chunks":
        correction_source = "open_loop"
    elif path_control_mode == "imu_heading":
        correction_source = "imu_heading"
    else:
        correction_source = "gps_imu"
    return {
        "current_heading_deg": heading,
        "heading_error_deg": heading_error,
        "cross_track_error_m": signed_cte,
        "along_track_progress_m": along,
        "remaining_distance_m": max(0.0, float(segment.get("length_m", 0.0)) - along),
        "b_cmd": b_cmd,
        "b_heading_component": b_heading,
        "b_cte_component": b_cte,
        "correction_source": correction_source,
    }


def connector_command(
    resolved_calibration: dict[str, object], direction: str
) -> dict[str, object]:
    """커넥터 회전 프리미티브를 선택하고, repeated-pulses 폴백 여부를 드러낸다.

    Select the connector turn primitive, surfacing the repeated-pulses fallback.

    Prefers the angle-calibrated 90 degree turn; when no turn-angle calibration
    exists the resolver's ``repeated_pulses`` fallback is used and
    ``fallback_to_repeated_pulses`` is reported True so the caller (and operator)
    can see that connectors are uncalibrated.
    """
    primitive = geometry._turn_calibrated(resolved_calibration, direction)
    effective = str(resolved_calibration.get("connector_mode_effective", "repeated_pulses"))
    fallback = effective == "repeated_pulses"
    target_angle = primitive.get("target_angle_deg")
    return {
        "a_cmd": float(primitive["a_cmd"]),
        "b_cmd": float(primitive["b_cmd"]),
        "pulse_ms": int(primitive["pulse_ms"]),
        "calibration_source": str(primitive.get("calibration_source", "unknown")),
        "connector_mode": effective,
        "fallback_to_repeated_pulses": fallback,
        "target_angle_deg": float(target_angle) if target_angle is not None else None,
    }


def connector_turn_angle_deg(segment: dict[str, object]) -> float:
    """커넥터 세그먼트가 요구하는 부호 있는 회전각 (+왼쪽 / -오른쪽).

    Signed rotation a connector segment requests (+left / -right).

    Plans generated after the turn/step decomposition carry an explicit
    ``turn_angle_deg``; older plans fall back to +/-90 from the expected motion
    direction (the lawnmower corner has always been a quarter turn).
    """
    explicit = telemetry._optional_float(segment.get("turn_angle_deg"))
    if explicit is not None:
        return float(explicit)
    return 90.0 if str(segment.get("expected_motion_direction")) == "turn_left" else -90.0


def body_heading_target_deg(segment: dict[str, object]) -> float:
    """이 세그먼트에서 로봇 몸체(BODY)가 유지해야 할 헤딩.

    [KO] 후진(reverse) lane은 몸체가 향하는 방향과 반대로 이동하므로 몸체 목표는
    이동 헤딩 + 180이다. mission 프레임이 추적하는 것은 이동 방향이 아니라 몸체다.

    Heading the robot BODY should hold on this segment.

    Backward (reverse-driven) lanes travel opposite to where the body points,
    so their body target is the travel heading plus 180. New plans carry an
    explicit ``body_heading_deg``; older plans derive it from the expected
    motion direction.
    """
    explicit = telemetry._optional_float(segment.get("body_heading_deg"))
    if explicit is not None:
        return float(explicit)
    travel = float(segment.get("target_heading_deg", 0.0) or 0.0)
    if str(segment.get("expected_motion_direction")) == "backward":
        return geometry.wrap_deg(travel + 180.0)
    return travel


def per_pulse_turn_angle_deg(
    connector: dict[str, object],
    *,
    turn_angle_policy: str = DEFAULT_TURN_ANGLE_POLICY,
    turn_angle_override: float | None = None,
) -> float | None:
    """보정된 회전 펄스 1회가 만들 것으로 기대되는 회전각, 또는 None.

    Rotation one calibrated turn pulse is expected to produce, or None.

    ``None`` means unknown (repeated-pulse twitch calibration); callers then keep
    the fixed-pulse budget. ``assume_90`` reproduces the legacy one-pulse-per-90
    assumption; ``from_json`` (default) trusts the calibration's target_angle_deg.
    """
    if turn_angle_override is not None and float(turn_angle_override) > 0.0:
        return float(turn_angle_override)
    if turn_angle_policy == "assume_90":
        return 90.0
    target = connector.get("target_angle_deg")
    if target is None:
        return None
    angle = abs(float(target))
    return angle if angle > 1e-6 else None


def connector_planned_pulses(requested_angle_deg: float, per_pulse_angle_deg: float | None) -> int:
    """요청된 회전을 덮는 개루프 펄스 개수 (>= 1). / Open-loop pulse count (>= 1)."""
    if per_pulse_angle_deg is None or per_pulse_angle_deg <= 0.0:
        return 1
    return max(1, math.ceil(abs(requested_angle_deg) / per_pulse_angle_deg - 1e-9))


def connector_pulse_budget(
    requested_angle_deg: float,
    per_pulse_angle_deg: float | None,
    *,
    max_pulses: int = DEFAULT_MAX_CONNECTOR_PULSES_PER_TURN,
    imu_available: bool,
) -> int:
    """커넥터 회전 1회의 펄스 상한 (무한 회전 방지 가드).

    Pulse cap for one connector turn (anti-rotation-loop guard).

    With IMU feedback the loop may use a couple of extra pulses beyond the
    planned count because each one is verified against measured yaw; without
    IMU it must stop exactly at the open-loop count rather than guess.
    """
    planned = connector_planned_pulses(requested_angle_deg, per_pulse_angle_deg)
    budget = planned + 2 if imu_available else planned
    return max(1, min(int(max_pulses), budget))


def manual_switch_seen(rows: Sequence[dict[str, str]]) -> bool:
    """텔레메트리 행 중 하나라도 물리 모드 스위치를 MANUAL로 보고하면 True.

    True when any telemetry row reports the physical mode switch in MANUAL.

    Rows without mode-channel fields (pulse ACK/STOP events) report ABSENT and
    never count, so this only triggers on explicit MANUAL evidence.
    """
    return any(mode_switch_state(row) == "MANUAL" for row in rows if isinstance(row, dict))


def manual_override_detected(row: dict[str, str] | None) -> bool:
    """운영자가 수동 RC 제어를 가져갔으면 True (스틱이 중립 이탈 / RC 소스).

    True => the operator has taken manual RC control (sticks off neutral / RC source).
    """
    if row is None:
        return False
    if telemetry._parse_bool(row.get("neutral_ok"), default=True) is False:
        return True
    for key in ("manual_forward_cmd", "manual_turn_cmd"):
        value = telemetry._optional_float(row.get(key))
        if value is not None and abs(value) > 1e-4:
            return True
    if row.get("control_source") == "RC_MANUAL" and telemetry.physical_output_active(row):
        return True
    return False


def rc_ignored_for_usb_supervised(row: dict[str, str] | None) -> bool:
    """활성 Mac USB 모션 펌웨어가 RC 입력을 명시적으로 무시하면 True.

    True when the active Mac USB motion firmware explicitly ignores RC input.
    """
    if row is None:
        return False
    return any(
        telemetry._parse_bool(row.get(key))
        for key in (
            "usb_pulse_test_ignore_rc_input",
            "usb_drive_live_ignore_rc_input",
            "usb_drive_live_mode",
            "usb_pulse_test_mode",
            "mac_physical_supervised",
        )
    ) or str(row.get("firmware_profile", "")).upper() == "MAC_PHYSICAL_SUPERVISED"


def rc_warning_for_usb_supervised(row: dict[str, str] | None) -> str:
    """USB 감독 모드에서의 비치명적 RC 경고 문자열 반환.

    Return non-fatal RC warning for USB-supervised modes.
    """
    if row is None or not rc_ignored_for_usb_supervised(row):
        return "NONE"
    if telemetry._parse_bool(row.get("rc_ok")) is not True:
        return "RC_NOT_OK_IGNORED_FOR_MAC_USB_SUPERVISED_MODE"
    if telemetry._parse_bool(row.get("neutral_ok"), default=True) is not True:
        return "RC_NOT_NEUTRAL_IGNORED_FOR_MAC_USB_SUPERVISED_MODE"
    return "NONE"


_USB_PULSE_COMPAT_MODE_KEY = "stage" + "20_physical_ab_guarded_crawl"
_USB_PULSE_COMPAT_READY_KEY = "stage" + "20_firmware_ready"


def guarded_pulse_compatible(row: dict[str, str]) -> bool:
    """펌웨어 하트비트가 guarded 물리 A/B 펄스 모드를 노출하면 True.

    True => the firmware heartbeat exposes guarded physical A/B pulse mode.
    """
    return (
        telemetry._parse_bool(row.get("usb_pulse_test_mode")) is True
        or telemetry._parse_bool(row.get(_USB_PULSE_COMPAT_MODE_KEY)) is True
        or (
            (
                telemetry._parse_bool(row.get("usb_pulse_ready")) is True
                or telemetry._parse_bool(row.get(_USB_PULSE_COMPAT_READY_KEY)) is True
            )
            and row.get("physical_a_role", "throttle") == "throttle"
            and row.get("physical_b_role", "turn") == "turn"
        )
    )


def pulse_block_reason(pulse_rows: Sequence[dict[str, str]]) -> str | None:
    """완료된 펄스 윈도우의 abort/무효 사유를 반환, 없으면 None.

    Return the abort/invalid reason for a completed pulse window, else None.

    Ordered so the most safety-critical condition wins: an RC_INVALID reject
    aborts the run; a missing ACK/STOP handshake, output still active after STOP,
    or nonzero final wheel commands each invalidate the pulse.
    """
    if safety.rc_invalid_abort(pulse_rows):
        return "RC_INVALID"
    missing = safety.missing_ack_or_stop_abort(pulse_rows)
    if missing is not None:
        return missing
    if safety.output_active_after_stop(pulse_rows):
        return "OUTPUT_ACTIVE_AFTER_STOP"
    if safety.nonzero_final_cmd(pulse_rows):
        return "FINAL_COMMANDS_NONZERO"
    return None


def live_drive_block_reason(rows: Sequence[dict[str, str]]) -> str | None:
    """라이브 드라이브(연속) 윈도우의 무효 사유, 없으면 None.

    [KO] pulse_block_reason의 라이브-드라이브 대응판. ARM/ACK 펄스 대신
    ACTIVE/STOP 이벤트와 REJECT를 검사한다(연속 SET에는 ARM/ACK가 없음).

    Invalidation reason for a live-drive (continuous) window, else None -- the
    live-drive counterpart of :func:`pulse_block_reason`.
    """
    if safety.rc_invalid_abort(rows):
        return "RC_INVALID"
    if any(telemetry.event(row) == "REJECT" for row in rows):
        return safety.latest_reject_reason(rows)
    if not any(telemetry.event(row) == "ACTIVE" for row in rows):
        return "ACTIVE_MISSING"
    if not any(telemetry.event(row) in safety.STOP_EVENTS for row in rows):
        return "STOP_MISSING"
    if safety.output_active_after_stop(rows):
        return "OUTPUT_ACTIVE_AFTER_STOP"
    if safety.nonzero_final_cmd(rows):
        return "FINAL_COMMANDS_NONZERO"
    return None


def _last_row_value(rows: Sequence[dict[str, str]], key: str) -> str | None:
    """뒤에서부터 첫 유효값(None/""/NA 아님)을 찾는다. / Last non-empty value for key."""
    for row in reversed(rows):
        value = row.get(key)
        if value not in (None, "", "NA"):
            return value
    return None


def _final_zero(rows: Sequence[dict[str, str]]) -> bool:
    """마지막 좌/우 바퀴 명령이 실질적으로 0인지 (정지 후 영점 확인).

    Whether the final left/right wheel commands are effectively zero (post-stop).
    """
    left = telemetry._optional_float(_last_row_value(rows, "final_left_cmd"))
    right = telemetry._optional_float(_last_row_value(rows, "final_right_cmd"))
    if left is None and right is None:
        return False
    return abs(left or 0.0) <= 1e-3 and abs(right or 0.0) <= 1e-3


def _segment_point(segment: dict[str, object], along_m: float) -> tuple[float, float]:
    """세그먼트 시작점에서 along_m만큼 진행한 라인 위 점 (구간에 clamp).

    Point on the segment line at ``along_m`` from the start (clamped to segment).
    """
    sx = float(segment["start_x_m"])
    sy = float(segment["start_y_m"])
    ex = float(segment["end_x_m"])
    ey = float(segment["end_y_m"])
    length = math.hypot(ex - sx, ey - sy)
    if length <= 1e-9:
        return sx, sy
    t = geometry.clamp(along_m / length, 0.0, 1.0)
    return sx + (ex - sx) * t, sy + (ey - sy) * t


def _dead_reckon_pose_along_segment(
    *,
    segment: dict[str, object],
    x_m: float,
    y_m: float,
    advance_m: float,
) -> tuple[float, float]:
    """현재 pose를 세그먼트 방향으로 advance_m 전진시킨 추측항법 pose.

    [KO] GPS가 없을 때, 캘리브레이션으로 추정한 전진량만큼 라인 방향으로 밀고
    기존 횡오차는 (좌법선 부호 규약으로) 유지한다.

    Dead-reckon the pose forward by ``advance_m`` along the segment, preserving
    the current signed cross-track offset (projection_metrics' left-normal sign).
    """
    along, signed_cte, _ = geometry.projection_metrics(segment, x_m, y_m)
    new_along = min(float(segment.get("length_m", 0.0)), along + max(0.0, advance_m))
    px, py = _segment_point(segment, new_along)
    sx = float(segment["start_x_m"])
    sy = float(segment["start_y_m"])
    ex = float(segment["end_x_m"])
    ey = float(segment["end_y_m"])
    length = math.hypot(ex - sx, ey - sy)
    if length <= 1e-9:
        return px, py
    # Signed cross-track uses the left-normal convention from projection_metrics.
    nx = -(ey - sy) / length
    ny = (ex - sx) / length
    return px + nx * signed_cte, py + ny * signed_cte


def _pose_from_gps_or_dead_reckon(
    *,
    row: dict[str, str] | None,
    gps: dict[str, object],
    pose_state: dict[str, object],
    segment: dict[str, object],
    start_lat: float,
    start_lon: float,
    advance_m: float = 0.0,
    gps_reanchor: bool = True,
) -> dict[str, object]:
    """유효 GPS면 로컬 좌표로 재앵커(re-anchor), 아니면 추측항법으로 pose 갱신.

    [KO] GPS가 유효하면 시작 위경도 기준 로컬 x/y로 다시 고정하고 pose_state를
    갱신한다. 열화 시 마지막 pose에서 advance_m만큼 추측항법으로 밀며,
    gps_reanchored는 GPS가 열화 이후 처음 복구되어 재앵커됐음을 표시한다.
    부수효과: 인자로 받은 ``pose_state`` 딕셔너리를 in-place로 갱신한다.

    Update pose from GPS when valid (re-anchoring to local x/y), otherwise dead-
    reckon forward by ``advance_m``. Side effect: mutates ``pose_state`` in place.
    """
    lat = gps.get("lat")
    lon = gps.get("lon")
    gps_valid = bool(lat is not None and lon is not None and not gps.get("gps_degraded"))
    gps_reanchored = False
    if gps_valid and gps_reanchor:
        x, y = geometry.goal_to_local(start_lat, start_lon, float(lat), float(lon))
        gps_reanchored = bool(pose_state.get("gps_degraded") or pose_state.get("source") != "gps")
        pose_state.update({"x": x, "y": y, "source": "gps", "gps_degraded": False})
    elif gps_valid and pose_state.get("x") is None:
        x, y = geometry.goal_to_local(start_lat, start_lon, float(lat), float(lon))
        pose_state.update({"x": x, "y": y, "source": "gps", "gps_degraded": False})
    else:
        x = float(pose_state.get("x", float(segment.get("start_x_m", 0.0))) or 0.0)
        y = float(pose_state.get("y", float(segment.get("start_y_m", 0.0))) or 0.0)
        if advance_m > 0.0:
            x, y = _dead_reckon_pose_along_segment(
                segment=segment,
                x_m=x,
                y_m=y,
                advance_m=advance_m,
            )
        pose_state.update({"x": x, "y": y, "source": "dead_reckoning", "gps_degraded": True})
    return {
        "x": float(pose_state.get("x", 0.0)),
        "y": float(pose_state.get("y", 0.0)),
        "gps_valid": gps_valid,
        "gps_reanchored": gps_reanchored,
        "source": pose_state.get("source", "dead_reckoning"),
    }


def planned_pulse(
    *, seq: int, a_cmd: float, b_cmd: float, pulse_ms: int
) -> dict[str, object]:
    """executor.send_pulse에 넘길 ARM/CMD/STOP 명령 텍스트를 구성한다.

    Build the ARM/CMD/STOP command texts for :func:`executor.send_pulse`.
    """
    return {
        "arm_command_text": f"USB_PULSE_TEST_ARM seq={seq}",
        "command_text": (
            f"USB_PULSE_TEST_CMD seq={seq} a={a_cmd:.3f} b={b_cmd:.3f} ms={int(pulse_ms)}"
        ),
        "stop_command_text": f"USB_PULSE_TEST_STOP seq={seq}",
        "pulse_ms": int(pulse_ms),
        "force_stop_command": True,
    }


# ── 시리얼 대기 헬퍼 / Serial-facing waits (small; exercised with a fake handle) ──
# 시리얼 핸들을 만지는 얇은 대기 함수들. 단위 테스트는 fake handle로 검증한다.
# EN: thin blocking waits on the serial handle; tested with a fake handle.


def _is_guarded_pulse_heartbeat(row: dict[str, str]) -> bool:
    """행이 guarded 펄스 모드를 노출하는 HEARTBEAT인지. / Guarded-pulse HEARTBEAT row?"""
    return telemetry.event(row) == "HEARTBEAT" and guarded_pulse_compatible(row)


def wait_for_guarded_pulse_heartbeat(
    handle: object, raw_lines: list[str], timeout_s: float
) -> dict[str, str] | None:
    """guarded 펄스 하트비트가 올 때까지 대기 (없으면 None).

    Block until a guarded-pulse heartbeat arrives, else None on timeout.
    """
    return executor.wait_for_row(handle, raw_lines, _is_guarded_pulse_heartbeat, timeout_s)


def wait_for_neutral_rc(
    handle: object, raw_lines: list[str], timeout_s: float
) -> dict[str, str] | None:
    """RC 스틱이 중립이고 준비된 guarded 펄스 하트비트를 대기.

    Wait for a guarded pulse heartbeat whose RC sticks are neutral and ready.
    """
    return executor.wait_for_row(
        handle,
        raw_lines,
        lambda row: _is_guarded_pulse_heartbeat(row) and not safety.rc_neutral_wait(row),
        timeout_s,
    )


# ── 실행 행 + 요약 / Execution row + summary ──
# 청크/펄스 1회를 CSV 한 행(dict)으로, 그리고 런 전체를 요약 dict으로 만든다.
# 두 드라이브 루프(run_controller / run_stop_correct_go)가 공유하는 출력 스키마.
# EN: turn one chunk/pulse into a CSV row dict and the whole run into a summary
# dict -- the shared output schema for both drive loops.


def build_execution_row(
    *,
    segment: dict[str, object],
    primitive_index: int,
    after_row: dict[str, str] | None,
    pulse_rows: Sequence[dict[str, str]],
    start_lat: float,
    start_lon: float,
    goal_lat: float,
    goal_lon: float,
    start_yaw_deg: float | None,
    target_heading_deg: float,
    a_cmd: float,
    correction: dict[str, float],
    pulse_ms: int,
    gps: dict[str, object],
    calibration_source: str,
    connector_mode: str,
    block_reason_override: str | None = None,
    drive_mode: str = "pulse",
    chunk_index: int | None = None,
    live_chunk_ms: int | None = None,
    max_ms: int | None = None,
    path_control_mode: str = DEFAULT_PATH_CONTROL_MODE,
    pose: dict[str, object] | None = None,
    base_a_cmd: float | None = None,
    b_trim: float = 0.0,
) -> dict[str, object]:
    """한 청크/펄스의 실행 결과를 표준 CSV 행(dict)으로 조립한다.

    [KO] pose/GPS/IMU/보정 성분/명령/유효성 판정을 한 행에 담는다. drive_mode가
    "continuous"면 무효 사유는 호출자가 넘긴 override를 쓰고(라이브 드라이브에는
    펄스 ACK/STOP 윈도우가 없으므로), 그 외에는 pulse_block_reason으로 계산한다.
    ``valid_pulse``/``invalid_reason``이 상위 루프의 중단 판단에 쓰인다.
    ``ready_for_full_path_following``은 항상 False(안전 규약).

    Assemble one chunk/pulse result into the standard CSV row dict (pose, GPS,
    IMU, correction components, commands, validity). Continuous rows take the
    caller's ``block_reason_override``; pulse rows compute it from the ACK/STOP
    window. ``ready_for_full_path_following`` is always False by contract.
    """
    if drive_mode == "continuous":
        block_reason = block_reason_override
    else:
        block_reason = block_reason_override if block_reason_override is not None else pulse_block_reason(pulse_rows)
    yaw = telemetry.imu_relative_yaw_deg(after_row) if after_row else None
    rc_ignored = rc_ignored_for_usb_supervised(after_row)
    invalid_reason = block_reason or "NONE"
    offending_duration_ms = "NA"
    if invalid_reason == "USB_DRIVE_LIVE_DURATION_EXCEEDS_MAX":
        invalid_reason = "HOST_SENT_DURATION_OVER_MAX"
        offending_duration_ms = int(pulse_ms)
    pose = pose or {}
    current_x = float(pose.get("x", 0.0))
    current_y = float(pose.get("y", 0.0))
    segment_start_x = float(segment.get("start_x_m", 0.0))
    segment_start_y = float(segment.get("start_y_m", 0.0))
    segment_end_x = float(segment.get("end_x_m", 0.0))
    segment_end_y = float(segment.get("end_y_m", 0.0))
    target_x = segment_end_x
    target_y = segment_end_y
    return {
        "row_type": "pulse",
        "segment_index": segment["segment_index"],
        "primitive_index": primitive_index,
        "chunk_index": chunk_index if chunk_index is not None else primitive_index,
        "path_control_mode": path_control_mode,
        "segment_type": segment["segment_type"],
        "start_lat": f"{start_lat:.7f}",
        "start_lon": f"{start_lon:.7f}",
        "goal_lat": f"{goal_lat:.7f}",
        "goal_lon": f"{goal_lon:.7f}",
        "current_lat": telemetry._fmt(gps["lat"], 7) if gps["lat"] is not None else "NA",
        "current_lon": telemetry._fmt(gps["lon"], 7) if gps["lon"] is not None else "NA",
        "gps_block_reason": (after_row or {}).get("gps_block_reason", "NA"),
        "firmware_profile": (after_row or {}).get("firmware_profile", "NA"),
        "gps_valid": bool(pose.get("gps_valid", False)),
        "gps_degraded": gps["gps_degraded"],
        "gps_reanchored": bool(pose.get("gps_reanchored", False)),
        "gps_cached_used": gps["gps_cached_used"],
        "imu_relative_yaw_deg": telemetry._fmt(yaw),
        "imu_yaw_deg": telemetry._fmt(yaw),
        "current_x_m": telemetry._fmt(current_x),
        "current_y_m": telemetry._fmt(current_y),
        "target_x_m": telemetry._fmt(target_x),
        "target_y_m": telemetry._fmt(target_y),
        "segment_start_x_m": telemetry._fmt(segment_start_x),
        "segment_start_y_m": telemetry._fmt(segment_start_y),
        "segment_end_x_m": telemetry._fmt(segment_end_x),
        "segment_end_y_m": telemetry._fmt(segment_end_y),
        "current_heading_deg": telemetry._fmt(correction["current_heading_deg"]),
        "target_heading_deg": telemetry._fmt(target_heading_deg),
        "heading_error_deg": telemetry._fmt(correction["heading_error_deg"]),
        "cross_track_error_m": telemetry._fmt(correction["cross_track_error_m"]),
        "along_track_progress_m": telemetry._fmt(correction["along_track_progress_m"]),
        "remaining_distance_m": telemetry._fmt(correction["remaining_distance_m"]),
        "a_cmd": f"{a_cmd:.3f}",
        "b_cmd": f"{correction['b_cmd']:.3f}",
        "base_a_cmd": f"{(base_a_cmd if base_a_cmd is not None else a_cmd):.3f}",
        "b_trim": f"{b_trim:.3f}",
        "b_heading_component": f"{correction['b_heading_component']:.3f}",
        "b_cte_component": f"{correction['b_cte_component']:.3f}",
        "b_heading_correction": f"{correction['b_heading_component']:.3f}",
        "b_cross_track_correction": f"{correction['b_cte_component']:.3f}",
        "final_a_cmd": f"{a_cmd:.3f}",
        "final_b_cmd": f"{correction['b_cmd']:.3f}",
        "pulse_ms": int(pulse_ms),
        "live_chunk_ms": live_chunk_ms if live_chunk_ms is not None else "NA",
        "max_ms": max_ms if max_ms is not None else "NA",
        "offending_duration_ms": offending_duration_ms,
        "ack_seen": any(telemetry.event(r) == "ACK" for r in pulse_rows),
        "active_seen": any(telemetry.event(r) == "ACTIVE" for r in pulse_rows),
        "stop_seen": any(telemetry.event(r) in safety.STOP_EVENTS for r in pulse_rows),
        "final_zero": _final_zero(pulse_rows),
        "manual_override_detected": False if rc_ignored else manual_override_detected(after_row),
        "rc_ignored_for_usb_supervised": rc_ignored,
        "rc_warning": rc_warning_for_usb_supervised(after_row),
        "calibration_source": calibration_source,
        "connector_mode": connector_mode,
        "drive_mode": drive_mode,
        "correction_source": str(correction.get("correction_source", "unknown")),
        "valid_pulse": block_reason is None,
        "invalid_reason": invalid_reason,
        "ready_for_full_path_following": False,
    }


def build_controller_summary(
    rows: Sequence[dict[str, object]],
    *,
    start_lat: float,
    start_lon: float,
    goal_lat: float,
    goal_lon: float,
    goal_distance_m: float,
    fallback_to_repeated_pulses: bool,
    abort_reason: str = "NONE",
) -> dict[str, object]:
    """guarded 컨트롤러 런 요약을 만든다 (준비도 체크를 통과시킨다).

    [KO] 행들을 집계해 유효/무효 카운트, GPS/IMU 사용 카운트, 평균/최대 헤딩·횡오차,
    폐루프 보정 enabled(설정 의도) vs applied(실제 B에 반영된 증거) 등을 뽑는다.
    반드시 assert_not_ready_for_full_path_following를 통과해 반환 -- 준비 완료를
    주장할 수 없게 하는 안전 게이트.

    Build the guarded controller run summary; routed through the readiness check.
    """
    pulse_rows = [r for r in rows if r.get("row_type") == "pulse"]
    valid = sum(1 for r in pulse_rows if r.get("valid_pulse") is True)
    continuous_drive_count = sum(1 for r in pulse_rows if r.get("drive_mode") == "continuous")
    rc_warning_count = sum(1 for r in pulse_rows if r.get("rc_warning") not in (None, "", "NONE"))
    cross_track_correction_used_count = sum(
        1
        for r in pulse_rows
        if abs(float(r.get("b_cte_component") or 0.0)) > _ZERO_TOLERANCE
    )
    imu_heading_used_count = sum(
        1
        for r in pulse_rows
        if r.get("imu_relative_yaw_deg") not in (None, "", "NA")
    )
    offending_durations = [
        int(r["offending_duration_ms"])
        for r in pulse_rows
        if str(r.get("offending_duration_ms", "NA")) not in {"", "NA"}
    ]
    max_ms_values = [
        int(r["max_ms"])
        for r in pulse_rows
        if str(r.get("max_ms", "NA")) not in {"", "NA"}
    ]
    heading_errors = [
        abs(float(r["heading_error_deg"]))
        for r in pulse_rows
        if str(r.get("heading_error_deg", "NA")) not in {"", "NA"}
    ]
    cross_track_errors = [
        abs(float(r["cross_track_error_m"]))
        for r in pulse_rows
        if str(r.get("cross_track_error_m", "NA")) not in {"", "NA"}
    ]
    completed_segment_count = len(
        {
            r.get("segment_index")
            for r in pulse_rows
            if r.get("valid_pulse") is True
        }
    )
    path_control_modes = [
        str(r.get("path_control_mode"))
        for r in pulse_rows
        if r.get("path_control_mode") not in (None, "", "NA")
    ]
    path_control_mode = path_control_modes[-1] if path_control_modes else DEFAULT_PATH_CONTROL_MODE
    # "enabled" is the configured intent (closed-loop modes); "applied" is the
    # runtime evidence that a nonzero steering correction actually reached B.
    correction_enabled = path_control_mode in {"imu_heading", "gps_imu_closed_loop"}
    correction_applied = any(
        abs(float(r.get("b_heading_component") or 0.0)) > _ZERO_TOLERANCE
        or abs(float(r.get("b_cte_component") or 0.0)) > _ZERO_TOLERANCE
        for r in pulse_rows
    )
    final_remaining = [
        float(r["remaining_distance_m"])
        for r in pulse_rows
        if str(r.get("remaining_distance_m", "NA")) not in {"", "NA"}
    ]
    summary = {
        "controller_mode": "continuous_motion",
        "path_control_mode": path_control_mode,
        "closed_loop_correction_enabled": correction_enabled,
        "closed_loop_correction_applied": correction_applied,
        "closed_loop_correction_disabled_reason": (
            "OPEN_LOOP_CHUNKS"
            if path_control_mode == "open_loop_chunks"
            else ("NO_NONZERO_HEADING_OR_CROSS_TRACK_ERROR" if not correction_applied else "NONE")
        ),
        "start_lat": start_lat,
        "start_lon": start_lon,
        "goal_lat": goal_lat,
        "goal_lon": goal_lon,
        "goal_distance_m": goal_distance_m,
        "pulse_count": len(pulse_rows),
        "valid_pulse_count": valid,
        "invalid_pulse_count": len(pulse_rows) - valid,
        "segment_count": len({r.get("segment_index") for r in pulse_rows}),
        "chunk_count": len(pulse_rows),
        "valid_chunk_count": valid,
        "completed_segment_count": completed_segment_count,
        "completed_chunk_count": valid,
        "gps_chunk_count": sum(1 for r in pulse_rows if r.get("gps_valid") is True),
        "gps_degraded_count": sum(1 for r in pulse_rows if r.get("gps_degraded") is True),
        "gps_reanchor_count": sum(1 for r in pulse_rows if r.get("gps_reanchored") is True),
        "continuous_drive_used": continuous_drive_count > 0,
        "continuous_drive_count": continuous_drive_count,
        "imu_heading_used_count": imu_heading_used_count,
        "cross_track_correction_used_count": cross_track_correction_used_count,
        "average_abs_heading_error_deg": (
            sum(heading_errors) / len(heading_errors) if heading_errors else "NA"
        ),
        "max_abs_heading_error_deg": max(heading_errors) if heading_errors else "NA",
        "average_abs_cross_track_error_m": (
            sum(cross_track_errors) / len(cross_track_errors) if cross_track_errors else "NA"
        ),
        "max_abs_cross_track_error_m": max(cross_track_errors) if cross_track_errors else "NA",
        "final_distance_to_goal_m": final_remaining[-1] if final_remaining else "NA",
        "rc_ignored_for_usb_supervised": any(r.get("rc_ignored_for_usb_supervised") is True for r in pulse_rows),
        "rc_warning_count": rc_warning_count,
        "offending_duration_ms": offending_durations[-1] if offending_durations else "NA",
        "max_ms": max_ms_values[-1] if max_ms_values else "NA",
        "fallback_to_repeated_pulses": bool(fallback_to_repeated_pulses),
        "abort_reason": abort_reason,
        "aborted": abort_reason != "NONE",
        "ready_for_full_path_following": False,
    }
    return checks.assert_not_ready_for_full_path_following(summary)


# ── 연속 모션 루프 / The continuous-motion loop ──
# run_controller: 멈추지 않고 이어지는 청크 드라이브. stop_correct_go와 달리
# 조향 보정을 정지 없이 B에 연속으로 실어 보낸다(하트비트 사이 IMU yaw로 헤딩 유지).
# EN: run_controller drives in continuous chunks, applying B corrections without
# stopping (contrast with the discrete run_stop_correct_go below).


def correction_token(row: dict[str, object]) -> str:
    """청크별 콘솔 라인에 쓰는 한 단어짜리 보정 요약.

    One-word correction summary for the per-chunk console line.

    ``dead_reckon`` when GPS is degraded (pose came from IMU + calibrated
    progress, not GPS); otherwise ``both`` / ``cross_track`` / ``heading`` for
    whichever steering components are nonzero, or ``none`` when B was untouched
    (open-loop chunks, or a chunk with zero error).
    """
    if row.get("gps_degraded") is True:
        return "dead_reckon"
    heading = abs(telemetry._optional_float(row.get("b_heading_correction")) or 0.0) > _ZERO_TOLERANCE
    cross_track = abs(telemetry._optional_float(row.get("b_cross_track_correction")) or 0.0) > _ZERO_TOLERANCE
    if heading and cross_track:
        return "both"
    if cross_track:
        return "cross_track"
    if heading:
        return "heading"
    return "none"


def chunk_status_line(row: dict[str, object], *, a_cmd: float, b_cmd: float) -> str:
    """빌드된 행으로부터 청크별 한 줄 상태 문자열을 렌더링.

    Render the required one-line per-chunk status string for a built row.
    """
    imu_ok = row.get("imu_relative_yaw_deg") not in (None, "", "NA")
    return (
        f"seg={row['segment_index']} chunk={row['chunk_index']} "
        f"mode={row['path_control_mode']} "
        f"gps={'DEGRADED' if row.get('gps_degraded') else 'OK'} "
        f"imu={'OK' if imu_ok else 'NA'} "
        f"heading_err={row['heading_error_deg']} cte={row['cross_track_error_m']} "
        f"progress={row['along_track_progress_m']} remaining={row['remaining_distance_m']} "
        f"A={a_cmd:.3f} B={b_cmd:.3f} correction={correction_token(row)}"
    )


def _segment_pulse_budget(
    segment: dict[str, object],
    resolved_calibration: dict[str, object],
    *,
    left_fixed_pulses: int,
    right_fixed_pulses: int,
    max_connector_pulses: int = DEFAULT_MAX_CONNECTOR_PULSES_PER_TURN,
    turn_angle_policy: str = DEFAULT_TURN_ANGLE_POLICY,
    turn_angle_override: float | None = None,
    imu_available: bool = False,
) -> tuple[int, bool, str]:
    """계획된 세그먼트에 대해 (펄스 예산, 커넥터 여부, 방향)을 반환.

    Return (pulse_budget, is_connector, direction) for a planned segment.

    Angle-calibrated connectors budget ``ceil(|requested| / per-pulse angle)``
    pulses from the calibration's target_angle_deg instead of assuming one pulse
    completes the corner; repeated-pulse twitch calibration keeps the fixed count.
    """
    if str(segment["segment_type"]) in {"connector_turn", "path_connector"}:
        direction = "left" if str(segment["expected_motion_direction"]) == "turn_left" else "right"
        if str(resolved_calibration.get("connector_mode_effective")) == "repeated_pulses":
            budget = int(left_fixed_pulses if direction == "left" else right_fixed_pulses)
        else:
            connector = connector_command(resolved_calibration, direction)
            per_pulse = per_pulse_turn_angle_deg(
                connector,
                turn_angle_policy=turn_angle_policy,
                turn_angle_override=turn_angle_override,
            )
            budget = connector_pulse_budget(
                connector_turn_angle_deg(segment),
                per_pulse,
                max_pulses=max_connector_pulses,
                imu_available=imu_available,
            )
        return budget, True, direction
    return int(segment.get("pulse_budget", 1)), False, "forward"


def run_controller(
    handle: object,
    *,
    segments: Sequence[dict[str, object]],
    resolved_calibration: dict[str, object],
    start_lat: float,
    start_lon: float,
    start_yaw_deg: float | None,
    goal_lat: float,
    goal_lon: float,
    event_timeout_s: float = DEFAULT_EVENT_TIMEOUT_S,
    heartbeat_timeout_s: float = DEFAULT_HEARTBEAT_TIMEOUT_S,
    rc_neutral_wait_s: float = DEFAULT_RC_NEUTRAL_WAIT_S,
    gps_degradation_policy: str = DEFAULT_GPS_DEGRADATION_POLICY,
    manual_override_mode: str = DEFAULT_MANUAL_OVERRIDE_MODE,
    left_fixed_pulses: int = 12,
    right_fixed_pulses: int = 12,
    straight_motion_mode: str = "pulse",
    live_update_hz: float = 8.0,
    live_ttl_ms: int = 350,
    live_chunk_ms: int = 700,
    max_segment_chunks: int = 20,
    live_max_ms: int = 1000,
    imu_heading_hold: bool = True,
    cross_track_correction: bool = True,
    path_control_mode: str = DEFAULT_PATH_CONTROL_MODE,
    k_heading: float = 0.006,
    k_cross_track: float = 0.20,
    max_correction_b: float = 0.08,
    gps_reanchor: bool = True,
    require_auto_switch: bool = False,
    max_connector_pulses_per_turn: int = DEFAULT_MAX_CONNECTOR_PULSES_PER_TURN,
    turn_angle_policy: str = DEFAULT_TURN_ANGLE_POLICY,
    turn_angle_override: float | None = None,
) -> tuple[list[dict[str, object]], list[str], str]:
    """``segments`` 위에서 감독 펄스 루프를 돌린다; (rows, raw_lines, abort_reason) 반환.

    [KO] 세그먼트마다: 펄스/청크 예산을 정하고, 매 청크 전에 하트비트를 기다려
    RC/모드 스위치/GPS 정책 안전 게이트를 통과시킨 뒤, 조향 보정을 얹어 전진
    청크(또는 라이브 드라이브)를 보내고, 이후 하트비트로 pose를 갱신해 행을 쌓는다.
    커넥터는 IMU yaw 조기 종료(측정된 회전이 요청각에 도달하면 중단)를 적용한다.
    예상 결함은 raise하지 않고 abort_reason만 세팅한 뒤 깨끗이 멈춘다(마지막
    STOP/영점 핸드셰이크는 여전히 실행됨).

    Run the supervised pulse loop over ``segments``; return (rows, raw_lines, abort_reason).

    Never raises on the expected field faults (serial disconnect, RC invalid,
    GPS abort policy, missing heartbeat): each sets ``abort_reason`` and stops the
    loop cleanly so the caller can still write a guarded summary.

    ``require_auto_switch`` makes the loop abort with ``USER_SWITCHED_TO_MANUAL``
    if a heartbeat reports the physical mode switch back in MANUAL -- used by the
    AUTO-switch-triggered ``auto-relative-run`` so flipping the switch stops the
    rover safely (the final STOP/zero handshake still runs).
    """
    rows: list[dict[str, object]] = []
    raw_lines: list[str] = []
    gps_cache: dict[str, object] = {"lat": start_lat, "lon": start_lon, "degraded": False}
    abort_reason = "NONE"
    primitive_index = 0
    effective_live_max_ms = max(1, int(live_max_ms))
    effective_live_chunk_ms = max(1, min(int(live_chunk_ms), effective_live_max_ms))
    effective_max_segment_chunks = max(1, int(max_segment_chunks))
    if path_control_mode not in PATH_CONTROL_MODES:
        path_control_mode = DEFAULT_PATH_CONTROL_MODE
    pose_state: dict[str, object] = {"x": 0.0, "y": 0.0, "source": "plan_start", "gps_degraded": False}
    # Heading-hold reference: honor an explicit --start-yaw-deg for the first lane,
    # then re-capture per lane from each lane's first heartbeat yaw so heading
    # correction stays live without the operator supplying a start yaw.
    provided_start_yaw = start_yaw_deg
    first_lane_pending = True
    if path_control_mode == "open_loop_chunks":
        imu_heading_hold = False
        cross_track_correction = False
    elif path_control_mode == "imu_heading":
        cross_track_correction = False

    for segment in segments:
        segment_ref_yaw: float | None = None
        connector_yaw_ref: float | None = None
        budget, is_connector, direction = _segment_pulse_budget(
            segment,
            resolved_calibration,
            left_fixed_pulses=left_fixed_pulses,
            right_fixed_pulses=right_fixed_pulses,
            max_connector_pulses=max_connector_pulses_per_turn,
            turn_angle_policy=turn_angle_policy,
            turn_angle_override=turn_angle_override,
        )
        connector_requested_angle = connector_turn_angle_deg(segment) if is_connector else 0.0
        target_heading = float(segment["target_heading_deg"])
        if is_connector:
            connector = connector_command(resolved_calibration, direction)
            a_cmd = float(connector["a_cmd"])
            pulse_ms = int(connector["pulse_ms"])
            calibration_source = str(connector["calibration_source"])
            connector_mode = str(connector["connector_mode"])
            connector_b = float(connector["b_cmd"])
            base_b = 0.0
        else:
            motion = geometry._motion_calibrated(
                resolved_calibration, str(segment["expected_motion_direction"])
            )
            a_cmd = float(motion["a_cmd"])
            pulse_ms = int(motion["pulse_ms"])
            calibration_source = str(motion.get("calibration_source", "unknown"))
            connector_mode = "lane"
            connector_b = 0.0
            base_b = float(motion.get("b_cmd", 0.0))

        # ── 직선 lane: 연속 청크 드라이브 / Straight lane: continuous chunk drive ──
        # 전체 이동 시간을 firmware-safe 상한(effective_live_max_ms)으로 잘게 쪼갠
        # 라이브 드라이브 청크들로 흘려보내고, 콜백이 최신 텔레메트리로 매 SET마다
        # B 보정을 다시 계산한다. remaining이 충분히 줄면 조기 종료.
        if not is_connector and straight_motion_mode == "continuous":
            total_duration_ms = max(1, int(pulse_ms) * max(1, budget))
            bounded_chunk_ms = effective_live_chunk_ms
            chunk_count = max(1, math.ceil(total_duration_ms / bounded_chunk_ms))
            chunk_count = min(chunk_count, effective_max_segment_chunks)
            remaining_ms = total_duration_ms
            chunk_progress_m = float(segment.get("length_m", 0.0)) / max(1, chunk_count)
            for chunk_index in range(1, chunk_count + 1):
                primitive_index += 1
                chunk_ms = min(bounded_chunk_ms, remaining_ms)
                if chunk_index == chunk_count:
                    chunk_ms = min(bounded_chunk_ms, max(1, remaining_ms))
                remaining_ms = max(0, remaining_ms - chunk_ms)
                try:
                    heartbeat = wait_for_guarded_pulse_heartbeat(handle, raw_lines, heartbeat_timeout_s)
                    if heartbeat is None:
                        abort_reason = "NO_GUARDED_PULSE_HEARTBEAT"
                        break
                    usb_supervised_rc_ignored = rc_ignored_for_usb_supervised(heartbeat)
                    if telemetry._parse_bool(heartbeat.get("rc_ok")) is not True and not usb_supervised_rc_ignored:
                        abort_reason = "RC_NOT_OK"
                        break
                    if (
                        not usb_supervised_rc_ignored
                        and manual_override_detected(heartbeat)
                        and manual_override_mode == "abort"
                    ):
                        abort_reason = "MANUAL_OVERRIDE"
                        break
                    if require_auto_switch and mode_switch_state(heartbeat) == "MANUAL":
                        abort_reason = MANUAL_SWITCH_ABORT_REASON
                        break
                    if segment_ref_yaw is None:
                        segment_ref_yaw = reference_yaw_for_segment(
                            heartbeat,
                            provided_start_yaw=provided_start_yaw,
                            use_provided=first_lane_pending,
                        )
                        first_lane_pending = False
                    gps = dead_reckon_gps(heartbeat, gps_cache)
                    gps_action = geometry.gps_policy_action(bool(gps["gps_degraded"]), gps_degradation_policy)
                    if gps_action == "abort":
                        abort_reason = "GPS_DEGRADED"
                        break
                    if gps_action == "pause":
                        continue
                    pose_before = _pose_from_gps_or_dead_reckon(
                        row=heartbeat,
                        gps=gps,
                        pose_state=pose_state,
                        segment=segment,
                        start_lat=start_lat,
                        start_lon=start_lon,
                        gps_reanchor=gps_reanchor,
                    )
                    latest_correction: dict[str, float] | None = None
                    latest_pose: dict[str, object] | None = pose_before
                    chunk_gps_reanchored = bool(pose_before.get("gps_reanchored"))

                    def command_from_row(row: dict[str, str] | None) -> tuple[float, float]:
                        # 라이브 드라이브 콜백: 매 SET 갱신마다 최신 활성 텔레메트리로
                        # pose와 B 보정을 다시 계산해 (a_cmd, b_cmd)를 돌려준다. A는 고정.
                        # EN: recompute pose + B correction from the freshest active
                        # telemetry on each live-drive SET; forward A stays fixed.
                        nonlocal latest_correction, latest_pose, chunk_gps_reanchored
                        source = row or heartbeat
                        local_gps = dead_reckon_gps(source, gps_cache)
                        latest_pose = _pose_from_gps_or_dead_reckon(
                            row=source,
                            gps=local_gps,
                            pose_state=pose_state,
                            segment=segment,
                            start_lat=start_lat,
                            start_lon=start_lon,
                            gps_reanchor=gps_reanchor,
                        )
                        chunk_gps_reanchored = chunk_gps_reanchored or bool(latest_pose.get("gps_reanchored"))
                        yaw = telemetry.imu_relative_yaw_deg(source)
                        latest_correction = pulse_correction(
                            segment=segment,
                            x=float(latest_pose["x"]),
                            y=float(latest_pose["y"]),
                            target_heading_deg=target_heading,
                            yaw=yaw,
                            start_yaw_deg=segment_ref_yaw,
                            is_connector=False,
                            base_b_cmd=base_b,
                            imu_heading_hold=imu_heading_hold,
                            cross_track_correction=cross_track_correction,
                            path_control_mode=path_control_mode,
                            k_heading=k_heading,
                            k_cross_track=k_cross_track,
                            max_correction_b=max_correction_b,
                        )
                        return a_cmd, float(latest_correction["b_cmd"])

                    duration_s = max(0.001, chunk_ms / 1000.0)
                    live_rows = executor.send_live_drive(
                        handle,
                        seq=primitive_index,
                        duration_s=duration_s,
                        update_hz=live_update_hz,
                        ttl_ms=live_ttl_ms,
                        command_fn=command_from_row,
                        raw_lines=raw_lines,
                        event_timeout_s=event_timeout_s,
                    )
                    after = wait_for_guarded_pulse_heartbeat(handle, raw_lines, heartbeat_timeout_s) or heartbeat
                except OSError:
                    abort_reason = "SERIAL_DISCONNECT"
                    break

                gps_after = dead_reckon_gps(after, gps_cache)
                pose_after = _pose_from_gps_or_dead_reckon(
                    row=after,
                    gps=gps_after,
                    pose_state=pose_state,
                    segment=segment,
                    start_lat=start_lat,
                    start_lon=start_lon,
                    advance_m=0.0 if not gps_after["gps_degraded"] else chunk_progress_m,
                    gps_reanchor=gps_reanchor,
                )
                chunk_gps_reanchored = chunk_gps_reanchored or bool(pose_after.get("gps_reanchored"))
                pose_after["gps_reanchored"] = chunk_gps_reanchored
                if latest_correction is None:
                    latest_correction = pulse_correction(
                        segment=segment,
                        x=float(pose_after["x"]),
                        y=float(pose_after["y"]),
                        target_heading_deg=target_heading,
                        yaw=telemetry.imu_relative_yaw_deg(after),
                        start_yaw_deg=segment_ref_yaw,
                        is_connector=False,
                        base_b_cmd=base_b,
                        imu_heading_hold=imu_heading_hold,
                        cross_track_correction=cross_track_correction,
                        path_control_mode=path_control_mode,
                        k_heading=k_heading,
                        k_cross_track=k_cross_track,
                        max_correction_b=max_correction_b,
                    )
                    latest_pose = pose_after
                else:
                    # The command callback computed the command from the latest active
                    # telemetry; rows should report the post-chunk pose when available.
                    latest_pose = pose_after
                row = build_execution_row(
                    segment=segment,
                    primitive_index=primitive_index,
                    after_row=after,
                    pulse_rows=live_rows,
                    start_lat=start_lat,
                    start_lon=start_lon,
                    goal_lat=goal_lat,
                    goal_lon=goal_lon,
                    start_yaw_deg=start_yaw_deg,
                    target_heading_deg=target_heading,
                    a_cmd=a_cmd,
                    correction=latest_correction,
                    pulse_ms=chunk_ms,
                    gps=gps_after,
                    calibration_source=calibration_source,
                    connector_mode=connector_mode,
                    block_reason_override=live_drive_block_reason(live_rows),
                    drive_mode="continuous",
                    chunk_index=chunk_index,
                    live_chunk_ms=bounded_chunk_ms,
                    max_ms=effective_live_max_ms,
                    path_control_mode=path_control_mode,
                    pose=latest_pose,
                    base_a_cmd=a_cmd,
                    b_trim=base_b,
                )
                rows.append(row)
                print(chunk_status_line(row, a_cmd=a_cmd, b_cmd=float(latest_correction["b_cmd"])))
                if row["valid_pulse"] is not True:
                    abort_reason = str(row["invalid_reason"])
                    break
                if require_auto_switch and manual_switch_seen(live_rows):
                    abort_reason = MANUAL_SWITCH_ABORT_REASON
                    break
                remaining = telemetry._optional_float(row.get("remaining_distance_m")) or 0.0
                if remaining <= max(0.05, chunk_progress_m * 0.5):
                    break
            if abort_reason != "NONE":
                break
            continue

        # ── 펄스 청크 루프 / Guarded-pulse chunk loop ──
        # 연속 모드가 아닌 경로: 각 계획 펄스를 firmware-safe 상한으로 쪼갠
        # guarded 펄스(ARM->CMD->STOP)들로 실행. 커넥터는 IMU yaw 조기 종료 적용.
        # EN: non-continuous path -- run each planned pulse as firmware-safe-capped
        # guarded pulses; connectors early-stop on measured IMU yaw.
        pulse_chunk_ms_values: list[int] = []
        for _ in range(budget):
            remaining_pulse_ms = int(pulse_ms)
            while remaining_pulse_ms > 0:
                chunk_ms = min(effective_live_max_ms, remaining_pulse_ms)
                pulse_chunk_ms_values.append(chunk_ms)
                remaining_pulse_ms -= chunk_ms
        for chunk_index, pulse_chunk_ms in enumerate(pulse_chunk_ms_values, 1):
            primitive_index += 1
            try:
                heartbeat = wait_for_guarded_pulse_heartbeat(handle, raw_lines, heartbeat_timeout_s)
                if heartbeat is None:
                    abort_reason = "NO_GUARDED_PULSE_HEARTBEAT"
                    break
                usb_supervised_rc_ignored = rc_ignored_for_usb_supervised(heartbeat)
                if telemetry._parse_bool(heartbeat.get("rc_ok")) is not True and not usb_supervised_rc_ignored:
                    abort_reason = "RC_NOT_OK"
                    break
                if (
                    not usb_supervised_rc_ignored
                    and manual_override_detected(heartbeat)
                    and manual_override_mode == "abort"
                ):
                    abort_reason = "MANUAL_OVERRIDE"
                    break
                if require_auto_switch and mode_switch_state(heartbeat) == "MANUAL":
                    abort_reason = MANUAL_SWITCH_ABORT_REASON
                    break
                if not usb_supervised_rc_ignored and safety.rc_neutral_wait(heartbeat):
                    neutral = wait_for_neutral_rc(handle, raw_lines, rc_neutral_wait_s)
                    if neutral is None:
                        abort_reason = "RC_NOT_NEUTRAL"
                        break
                    heartbeat = neutral

                gps = dead_reckon_gps(heartbeat, gps_cache)
                gps_action = geometry.gps_policy_action(
                    bool(gps["gps_degraded"]), gps_degradation_policy
                )
                if gps_action == "abort":
                    abort_reason = "GPS_DEGRADED"
                    break
                if gps_action == "pause":
                    continue

                lat = float(gps["lat"]) if gps["lat"] is not None else start_lat
                lon = float(gps["lon"]) if gps["lon"] is not None else start_lon
                x, y = geometry.goal_to_local(start_lat, start_lon, lat, lon)
                # Connectors hold against the global start yaw (the turn rotates the
                # body); lanes hold against the per-lane captured reference.
                if is_connector:
                    if connector_yaw_ref is None:
                        connector_yaw_ref = telemetry.imu_relative_yaw_deg(heartbeat)
                    effective_ref_yaw = provided_start_yaw
                else:
                    if segment_ref_yaw is None:
                        segment_ref_yaw = reference_yaw_for_segment(
                            heartbeat,
                            provided_start_yaw=provided_start_yaw,
                            use_provided=first_lane_pending,
                        )
                        first_lane_pending = False
                    effective_ref_yaw = segment_ref_yaw
                yaw = telemetry.imu_relative_yaw_deg(heartbeat)
                correction = pulse_correction(
                    segment=segment,
                    x=x,
                    y=y,
                    target_heading_deg=target_heading,
                    yaw=yaw,
                    start_yaw_deg=effective_ref_yaw,
                    is_connector=is_connector,
                    base_b_cmd=base_b,
                    connector_b_cmd=connector_b,
                    imu_heading_hold=imu_heading_hold,
                    cross_track_correction=cross_track_correction,
                    path_control_mode=path_control_mode,
                    k_heading=k_heading,
                    k_cross_track=k_cross_track,
                    max_correction_b=max_correction_b,
                )
                planned = planned_pulse(
                    seq=primitive_index,
                    a_cmd=a_cmd,
                    b_cmd=float(correction["b_cmd"]),
                    pulse_ms=pulse_chunk_ms,
                )
                pulse_rows = executor.send_pulse(
                    handle, planned, raw_lines, event_timeout_s=event_timeout_s
                )
                after = (
                    wait_for_guarded_pulse_heartbeat(handle, raw_lines, heartbeat_timeout_s)
                    or heartbeat
                )
            except OSError:
                abort_reason = "SERIAL_DISCONNECT"
                break

            gps_after = dead_reckon_gps(after, gps_cache)
            row = build_execution_row(
                segment=segment,
                primitive_index=primitive_index,
                after_row=after,
                pulse_rows=pulse_rows,
                start_lat=start_lat,
                start_lon=start_lon,
                goal_lat=goal_lat,
                goal_lon=goal_lon,
                start_yaw_deg=start_yaw_deg,
                target_heading_deg=target_heading,
                a_cmd=a_cmd,
                correction=correction,
                pulse_ms=pulse_chunk_ms,
                gps=gps_after,
                calibration_source=calibration_source,
                connector_mode=connector_mode,
                chunk_index=chunk_index,
                max_ms=effective_live_max_ms,
                path_control_mode=path_control_mode,
                pose={
                    "x": x,
                    "y": y,
                    "gps_valid": not bool(gps_after["gps_degraded"]),
                    "gps_reanchored": False,
                },
                base_a_cmd=a_cmd,
                b_trim=base_b,
            )
            rows.append(row)
            print(chunk_status_line(row, a_cmd=a_cmd, b_cmd=float(correction["b_cmd"])))
            if row["valid_pulse"] is not True:
                abort_reason = str(row["invalid_reason"])
                break
            if require_auto_switch and manual_switch_seen(pulse_rows):
                abort_reason = MANUAL_SWITCH_ABORT_REASON
                break
            # Connector early-stop: measure the rotation actually applied since
            # the connector started (local yaw reference), not against a global
            # start yaw that is usually absent. Without IMU evidence the loop
            # must run its full angle-derived budget instead of assuming done.
            if is_connector and connector_mode != "repeated_pulses":
                yaw_after_turn = telemetry.imu_relative_yaw_deg(after)
                if connector_yaw_ref is not None and yaw_after_turn is not None:
                    applied_turn = geometry.wrap_deg(yaw_after_turn - connector_yaw_ref)
                    remaining_turn = geometry.wrap_deg(connector_requested_angle - applied_turn)
                    # Done within tolerance, or overshot: never keep pulsing the
                    # same direction past the corner.
                    if (
                        abs(remaining_turn) <= DEFAULT_CONNECTOR_TURN_TOLERANCE_DEG
                        or remaining_turn * connector_requested_angle < 0.0
                    ):
                        break
            progress = telemetry._optional_float(row.get("along_track_progress_m")) or 0.0
            if str(segment["segment_type"]).endswith("_lane") and progress >= float(
                segment.get("length_m", 0.0)
            ):
                break

        if abort_reason != "NONE":
            break
        if rows and rows[-1]["valid_pulse"] is not True:
            break

    return rows, raw_lines, abort_reason


# ── stop_correct_go: 이산 전진->정지->측정->보정 루프 ──
# stop_correct_go: discrete move -> stop -> measure -> correct loop
#
# [KO] 이 모듈의 핵심 모드. 짧은 필드 테스트용으로 일부러 보수적으로 만든 경로
# 추종 방식이다. 연속 gps_imu_closed_loop과 달리, 각 직선 청크는 완전히 멈추고
# 영점을 확인한 뒤에야 pose를 읽는다. 그래서 헤딩 보정은 연속 B nudge가 아니라
# **정지 상태의 이산 제자리 IMU 회전**이고, 횡오차는 *다음* 청크의 B로 넘겨 트림한다.
#
# 왜 멈춰서 측정하는가(WHY): 모터가 도는 동안 펌웨어가 MOTOR_TRACE 라인으로
# 시리얼(UART)을 포화시켜 yaw가 실린 하트비트가 거의 살아남지 못한다 -- 움직이며
# 보는 연속 피드백은 실전에서 눈먼 상태다. 그래서 회전(커넥터/헤딩 보정)은
# burst -> stop -> 텔레메트리 회복 대기 -> 정지 yaw 측정 사이클로 돈다.
#
# 재사용(primitives): executor.send_pulse(guarded 전진), projection_metrics(기하),
# dead_reckon_gps / _pose_from_gps_or_dead_reckon(pose), build_execution_row
# (표준 요약/CSV 경로 그대로) -- 새 제어 법칙은 없다.
#
# A deliberately conservative path-following mode for short field tests. Unlike
# the continuous gps_imu_closed_loop drive, each straight chunk fully stops and
# confirms zero before the rover reads its pose, so heading is corrected by a
# discrete in-place IMU turn (not a continuous B nudge) and cross-track error is
# trimmed onto the *next* chunk's B. It reuses the same primitives as the rest of
# the controller: executor.send_pulse for the guarded move, projection_metrics for
# the geometry, dead_reckon_gps / _pose_from_gps_or_dead_reckon for pose, and
# build_execution_row so the standard summary/CSV path works unchanged.


def stop_correct_go_sensor_source(
    *,
    gps_valid: bool,
    imu_valid: bool,
    trust_mode: str = DEFAULT_SENSOR_TRUST_MODE,
    allow_calibration_fallback: bool = True,
) -> dict[str, object]:
    """이번 사이클의 pose/헤딩 추정을 어떤 센서로 할지 결정.

    Decide which sensor drives this cycle's pose/heading estimate.

    GPS and/or IMU are used whenever available. When *both* are unavailable the
    cycle may only continue on calibrated dead-reckoning if it is allowed --
    either explicitly (``allow_calibration_fallback``) or implicitly by the
    ``calibration_fallback`` trust mode. Otherwise the run aborts
    ``SENSOR_UNAVAILABLE`` rather than driving blind.
    """
    if trust_mode not in SENSOR_TRUST_MODES:
        trust_mode = DEFAULT_SENSOR_TRUST_MODE
    if gps_valid or imu_valid:
        source = "gps_imu" if (gps_valid and imu_valid) else ("gps" if gps_valid else "imu")
        return {"ok": True, "source": source, "fallback_used": False, "reason": "NONE"}
    fallback_allowed = bool(allow_calibration_fallback) or trust_mode == "calibration_fallback"
    if fallback_allowed:
        return {
            "ok": True,
            "source": "calibration",
            "fallback_used": True,
            "reason": "CALIBRATION_DEAD_RECKON",
        }
    return {"ok": False, "source": "none", "fallback_used": False, "reason": "SENSOR_UNAVAILABLE"}


def stop_correct_go_heading_decision(
    *,
    heading_error_deg: float,
    threshold_deg: float,
    imu_valid: bool,
    b_left: float,
    b_right: float,
) -> dict[str, object]:
    """제자리 헤딩 보정이 필요한지, 그리고 그 회전 명령을 결정한다.

    Whether an in-place heading correction is needed, and its turn command.

    The correction is an IMU-feedback turn, so it is only attempted when IMU yaw
    is available; without IMU the rover keeps its heading on the next straight
    chunk instead of guessing a blind rotation.
    """
    needs = bool(imu_valid) and abs(heading_error_deg) > abs(threshold_deg)
    if not needs:
        return {"needs_correction": False, "turn_direction": "none", "correction_b_cmd": 0.0}
    direction = "left" if heading_error_deg > 0.0 else "right"
    return {
        "needs_correction": True,
        "turn_direction": direction,
        "correction_b_cmd": float(b_left) if direction == "left" else float(b_right),
    }


def stop_correct_go_cross_track_trim(
    *,
    cross_track_error_m: float,
    threshold_m: float,
    k_cross_track: float = 0.20,
    max_correction_b: float = 0.08,
) -> float:
    """횡오차가 임계값을 넘을 때 *다음* 청크에 적용할 B 트림.

    B trim applied to the *next* chunk when cross-track error exceeds threshold.

    Uses the same sign/gain convention as the continuous closed-loop B correction
    (positive signed cross-track -> positive B -> steer back onto the line),
    clamped to +/-``max_correction_b``. Below threshold the trim is zero.
    """
    if abs(cross_track_error_m) <= abs(threshold_m):
        return 0.0
    return geometry.clamp(
        float(k_cross_track) * float(cross_track_error_m),
        -abs(max_correction_b),
        abs(max_correction_b),
    )


def remaining_turn_error_deg(initial_heading_error_deg: float, yaw_delta_deg: float) -> float:
    """목표 쪽으로 ``|yaw_delta|``만큼 회전한 뒤 남은 부호 있는 헤딩 오차.

    Signed heading error left after rotating ``|yaw_delta|`` toward the target.

    The turn direction comes from the error sign and the IMU yaw magnitude
    measures how far we actually rotated, so the applied rotation is
    ``copysign(|yaw_delta|, initial_error)`` (mirrors the alignment turn math).
    """
    applied = math.copysign(abs(yaw_delta_deg), initial_heading_error_deg)
    return geometry.wrap_deg(initial_heading_error_deg - applied)


def _collect_stabilized_heartbeat(
    handle: object,
    raw_lines: list[str],
    *,
    settle_after_move_ms: int,
    telemetry_stabilize_ms: int,
    heartbeat_timeout_s: float,
    verbose_raw: bool = True,
) -> dict[str, str] | None:
    """모션이 잦아들길 기다린 뒤, 안정화 윈도우 동안 하트비트를 읽는다.

    [KO] 정지 직후의 과도(transient) 프레임이 아니라, 안정화 윈도우 안에서 본
    *마지막* guarded 하트비트를 반환한다 -- 정지·안정된 표본에서 pose/헤딩을 읽기
    위함. stop_correct_go의 "정지 후 측정"을 실제로 구현하는 지점.

    Let the motion settle, then read heartbeats over the stabilize window.

    Returns the *last* guarded-pulse heartbeat seen inside the window so pose and
    heading are read from a stationary, settled telemetry sample rather than the
    transient first frame after the move stops.
    """
    if settle_after_move_ms > 0:
        time.sleep(settle_after_move_ms / 1000.0)
    last = wait_for_guarded_pulse_heartbeat(handle, raw_lines, heartbeat_timeout_s)
    deadline = time.monotonic() + max(0.0, telemetry_stabilize_ms / 1000.0)
    while time.monotonic() < deadline:
        budget = max(0.02, min(float(heartbeat_timeout_s), deadline - time.monotonic()))
        nxt = wait_for_guarded_pulse_heartbeat(handle, raw_lines, budget)
        if nxt is None:
            break
        last = nxt
    return last


def _run_stop_correct_go_heading_turn(
    handle: object,
    *,
    correction_b_cmd: float,
    yaw_turn_start: float,
    initial_heading_error_deg: float,
    heading_tolerance_deg: float,
    max_correction_ms: int,
    update_hz: float,
    ttl_ms: int,
    chunk_ms: int,
    event_timeout_s: float,
    raw_lines: list[str],
    verbose_raw: bool = True,
    abort_on_manual_switch: bool = False,
    settle_after_move_ms: int = DEFAULT_SETTLE_AFTER_MOVE_MS,
    telemetry_stabilize_ms: int = DEFAULT_TELEMETRY_STABILIZE_MS,
    heartbeat_timeout_s: float = DEFAULT_HEARTBEAT_TIMEOUT_S,
    rate_dps_hint: float | None = None,
    max_burst_ms: int = DEFAULT_TURN_BURST_MAX_MS,
) -> dict[str, object]:
    """목표를 향해 burst -> stop -> measure 사이클로 제자리 회전한다.

    [KO] stop_correct_go 회전 엔진(커넥터 pivot과 헤딩 보정이 공유). 모터가 도는
    동안 MOTOR_TRACE 홍수로 시리얼이 포화돼 yaw 하트비트가 살아남지 못하므로,
    각 사이클은 남은 각도와 회전율 추정으로 크기를 정한 데드맨 라이브 SET 버스트
    1회를 쏘고 -> 멈추고 -> 텔레메트리 회복을 기다린 뒤 -> 정지 yaw를 읽어 다음
    버스트를 정한다. 종료 조건: 허용오차 도달 / 오버슈트(부호 반전) / 정체(측정
    가능한 진행 없음) / REJECT / MANUAL 스위치 / max_correction_ms 마감.
    긴 guarded 펄스를 쓰지 않으므로 펌웨어의 guarded-pulse 상한은 트리거되지 않는다.

    Rotate in place toward the target in burst -> stop -> measure cycles.

    While the motors run, the firmware floods the serial link with MOTOR_TRACE
    lines and saturates the UART, so yaw-bearing heartbeats rarely survive --
    continuous in-motion feedback is blind in practice (field data: every
    connector ran to its time cap). Each cycle therefore issues ONE bounded
    deadman live SET (the burst, sized from the remaining angle and a turn-rate
    estimate), stops, lets telemetry recover, and reads a settled stationary
    yaw before deciding the next burst. Stops on: tolerance reached, overshoot
    (sign flip), stall (no measurable progress), REJECT, MANUAL switch
    (``abort_on_manual_switch``), or the ``max_correction_ms`` deadline.
    """
    seq = 1
    start_time = time.monotonic()
    deadline = start_time + max(0.0, max_correction_ms / 1000.0)
    rate_dps = abs(rate_dps_hint) if rate_dps_hint else DEFAULT_TURN_RATE_GUESS_DPS
    rate_dps = max(rate_dps, 1.0)
    yaw_latest = yaw_turn_start
    remaining = initial_heading_error_deg
    bursts = 0
    timed_out = False
    rejected = False
    manual_switch = False
    overshoot = False
    no_progress = False
    last_row: dict[str, str] | None = None
    while abs(remaining) > abs(heading_tolerance_deg):
        budget_ms = int((deadline - time.monotonic()) * 1000.0)
        if budget_ms < MIN_TURN_BURST_MS:
            timed_out = True
            break
        # Aim ~80% of the estimated remaining-time so a fast motor lands short
        # of the target instead of past it; the next cycle finishes the rest.
        burst_ms = int(abs(remaining) / rate_dps * 1000.0 * 0.8)
        burst_ms = max(MIN_TURN_BURST_MS, min(burst_ms, int(max_burst_ms), budget_ms))
        bursts += 1
        executor.write_command(
            handle,
            (
                f"USB_DRIVE_LIVE_SET seq={seq} a=0.000 b={float(correction_b_cmd):.3f} "
                f"duration_ms={burst_ms} ttl_ms={burst_ms + max(int(ttl_ms), 300)}"
            ),
        )
        _turn_sleep(burst_ms / 1000.0)
        executor.write_command(handle, f"USB_DRIVE_LIVE_STOP seq={seq}")
        confirm_rows = executor.wait_for_event(
            handle, raw_lines, executor.STOP_CONFIRM_EVENTS, event_timeout_s, verbose_raw=verbose_raw
        )
        if any(telemetry.event(r) == "REJECT" for r in confirm_rows):
            rejected = True
            break
        row = _collect_stabilized_heartbeat(
            handle,
            raw_lines,
            settle_after_move_ms=settle_after_move_ms,
            telemetry_stabilize_ms=telemetry_stabilize_ms,
            heartbeat_timeout_s=heartbeat_timeout_s,
            verbose_raw=verbose_raw,
        )
        if row is None:
            no_progress = True
            break
        last_row = row
        if abort_on_manual_switch and mode_switch_state(row) == "MANUAL":
            manual_switch = True
            break
        yaw = telemetry.imu_relative_yaw_deg(row)
        if yaw is None:
            no_progress = True
            break
        yaw_latest = yaw
        previous_remaining = remaining
        remaining = geometry.wrap_deg(
            initial_heading_error_deg - geometry.wrap_deg(yaw - yaw_turn_start)
        )
        if abs(remaining) <= abs(heading_tolerance_deg):
            break
        if remaining * initial_heading_error_deg < 0.0:
            # Rotated past the target during the burst: keeping the same turn
            # direction would only widen the error, so stop here.
            overshoot = True
            break
        if abs(previous_remaining) - abs(remaining) < TURN_STALL_MIN_PROGRESS_DEG:
            # Stalled motors or a wrong-direction response: stop instead of
            # burning the whole time budget on bursts that change nothing.
            no_progress = True
            break
    return {
        "yaw_turn_start": yaw_turn_start,
        "yaw_final": yaw_latest,
        "initial_heading_error_deg": initial_heading_error_deg,
        "final_heading_error_deg": remaining,
        "turn_duration_ms": int((time.monotonic() - start_time) * 1000.0),
        "correction_success": abs(remaining) <= abs(heading_tolerance_deg)
        and not rejected
        and not manual_switch,
        "timed_out": timed_out,
        "rejected": rejected,
        "manual_switch": manual_switch,
        "overshoot": overshoot,
        "no_progress": no_progress,
        "bursts": bursts,
        "last_row": last_row,
    }


def stop_correct_go_status_line(row: dict[str, object]) -> str:
    """빌드된 stop_correct_go 행의 사이클별 한 줄 콘솔 상태.

    One-line per-cycle console status for a built stop_correct_go row.
    """
    imu_ok = bool(row.get("imu_valid"))
    return (
        f"seg={row['segment_index']} chunk={row['chunk_index']} "
        f"phase={row.get('phase', 'move')} mode=stop_correct_go "
        f"sensor={row.get('sensor_source', 'NA')} fallback={str(bool(row.get('fallback_used'))).lower()} "
        f"gps={'OK' if row.get('gps_valid') else 'DEGRADED'} imu={'OK' if imu_ok else 'NA'} "
        f"heading_err={row['heading_error_deg']} cte={row['cross_track_error_m']} "
        f"progress={row['along_track_progress_m']} remaining={row['remaining_distance_m']} "
        f"moveA={row.get('move_a_cmd')} moveB={row.get('move_b_cmd')} "
        f"corrB={row.get('correction_b_cmd')} corr_ok={row.get('correction_success')}"
    )


def _stop_correct_go_preflight(
    handle: object,
    raw_lines: list[str],
    *,
    heartbeat_timeout_s: float,
    rc_neutral_wait_s: float,
    manual_override_mode: str,
    require_auto_switch: bool,
) -> tuple[dict[str, str] | None, str]:
    """guarded 하트비트를 기다린 뒤 공통 이동-전 안전 게이트를 적용한다.

    Wait for a guarded heartbeat and apply the shared pre-move safety gates.

    Returns ``(heartbeat, abort_reason)`` where ``abort_reason`` is ``"NONE"`` when
    it is safe to move. Mirrors the per-pulse preamble of :func:`run_controller`.
    """
    heartbeat = wait_for_guarded_pulse_heartbeat(handle, raw_lines, heartbeat_timeout_s)
    if heartbeat is None:
        return None, "NO_GUARDED_PULSE_HEARTBEAT"
    rc_ignored = rc_ignored_for_usb_supervised(heartbeat)
    if telemetry._parse_bool(heartbeat.get("rc_ok")) is not True and not rc_ignored:
        return heartbeat, "RC_NOT_OK"
    if not rc_ignored and manual_override_detected(heartbeat) and manual_override_mode == "abort":
        return heartbeat, "MANUAL_OVERRIDE"
    if require_auto_switch and mode_switch_state(heartbeat) == "MANUAL":
        return heartbeat, MANUAL_SWITCH_ABORT_REASON
    if not rc_ignored and safety.rc_neutral_wait(heartbeat):
        neutral = wait_for_neutral_rc(handle, raw_lines, rc_neutral_wait_s)
        if neutral is None:
            return heartbeat, "RC_NOT_NEUTRAL"
        heartbeat = neutral
    return heartbeat, "NONE"


def build_stop_correct_go_summary(
    rows: Sequence[dict[str, object]],
    *,
    start_lat: float,
    start_lon: float,
    goal_lat: float,
    goal_lon: float,
    goal_distance_m: float,
    fallback_to_repeated_pulses: bool,
    sensor_trust_mode: str,
    allow_calibration_fallback: bool,
    abort_reason: str = "NONE",
    heading_reference: str = DEFAULT_HEADING_REFERENCE,
    turn_angle_policy: str = DEFAULT_TURN_ANGLE_POLICY,
    turn_calibration_angles: dict[str, object] | None = None,
) -> dict[str, object]:
    """컨트롤러 요약에 stop_correct_go 전용 카운터를 덧붙인다.

    [KO] 헤딩 보정 횟수/성공, 횡오차 트림 적용 수, 센서 폴백 수, 커넥터 회전
    완료/미완/오버슈트/IMU 측정 수, GPS 점프 거부 수 등을 집계해 기본 요약에 병합.
    반환은 준비도 게이트를 통과한다.

    Controller summary augmented with stop_correct_go-specific counters.
    """
    summary = build_controller_summary(
        rows,
        start_lat=start_lat,
        start_lon=start_lon,
        goal_lat=goal_lat,
        goal_lon=goal_lon,
        goal_distance_m=goal_distance_m,
        fallback_to_repeated_pulses=fallback_to_repeated_pulses,
        abort_reason=abort_reason,
    )
    cycle_rows = [r for r in rows if r.get("row_type") == "pulse"]
    heading_corrections = [r for r in cycle_rows if int(r.get("correction_duration_ms") or 0) > 0]
    connector_rows = [r for r in cycle_rows if r.get("phase") == "connector"]
    connector_segments = {r.get("segment_index") for r in connector_rows}
    connector_completed_segments = {
        r.get("segment_index")
        for r in connector_rows
        if r.get("connector_turn_completed") is True
    }
    summary.update(
        {
            "path_control_mode": "stop_correct_go",
            "sensor_trust_mode": sensor_trust_mode,
            "allow_calibration_fallback": bool(allow_calibration_fallback),
            "heading_reference": heading_reference,
            "turn_calibration_angle_policy": turn_angle_policy,
            "heading_correction_count": len(heading_corrections),
            "heading_correction_success_count": sum(
                1 for r in heading_corrections if r.get("correction_success") is True
            ),
            "cross_track_trim_applied_count": sum(
                1
                for r in cycle_rows
                if abs(telemetry._optional_float(r.get("move_b_cmd")) or 0.0) > _ZERO_TOLERANCE
            ),
            "sensor_fallback_used_count": sum(1 for r in cycle_rows if r.get("fallback_used") is True),
            "connector_turn_count": len(connector_segments),
            "connector_pulse_count": len(connector_rows),
            "connector_completed_count": len(connector_completed_segments),
            "connector_incomplete_count": len(connector_segments - connector_completed_segments),
            "connector_imu_measured_pulse_count": sum(
                1 for r in connector_rows if r.get("turn_measured_by_imu") is True
            ),
            "connector_overshoot_count": sum(
                1 for r in connector_rows if r.get("turn_overshoot") is True
            ),
            "gps_jump_rejected_count": sum(
                1 for r in cycle_rows if r.get("gps_jump_rejected") is True
            ),
        }
    )
    if turn_calibration_angles:
        summary.update(turn_calibration_angles)
    return checks.assert_not_ready_for_full_path_following(summary)


def run_stop_correct_go(
    handle: object,
    *,
    segments: Sequence[dict[str, object]],
    resolved_calibration: dict[str, object],
    start_lat: float,
    start_lon: float,
    start_yaw_deg: float | None,
    goal_lat: float,
    goal_lon: float,
    move_chunk_ms: int = DEFAULT_MOVE_CHUNK_MS,
    settle_after_move_ms: int = DEFAULT_SETTLE_AFTER_MOVE_MS,
    telemetry_stabilize_ms: int = DEFAULT_TELEMETRY_STABILIZE_MS,
    heading_correction_threshold_deg: float = DEFAULT_HEADING_CORRECTION_THRESHOLD_DEG,
    heading_correction_tolerance_deg: float = DEFAULT_HEADING_CORRECTION_TOLERANCE_DEG,
    cross_track_correction_threshold_m: float = DEFAULT_CROSS_TRACK_CORRECTION_THRESHOLD_M,
    heading_correction_b_left: float = DEFAULT_HEADING_CORRECTION_B_LEFT,
    heading_correction_b_right: float = DEFAULT_HEADING_CORRECTION_B_RIGHT,
    max_heading_correction_ms: int = DEFAULT_MAX_HEADING_CORRECTION_MS,
    heading_correction_chunk_ms: int = DEFAULT_HEADING_CORRECTION_CHUNK_MS,
    sensor_trust_mode: str = DEFAULT_SENSOR_TRUST_MODE,
    allow_calibration_fallback: bool = True,
    event_timeout_s: float = DEFAULT_EVENT_TIMEOUT_S,
    heartbeat_timeout_s: float = DEFAULT_HEARTBEAT_TIMEOUT_S,
    rc_neutral_wait_s: float = DEFAULT_RC_NEUTRAL_WAIT_S,
    gps_degradation_policy: str = DEFAULT_GPS_DEGRADATION_POLICY,
    manual_override_mode: str = DEFAULT_MANUAL_OVERRIDE_MODE,
    left_fixed_pulses: int = 12,
    right_fixed_pulses: int = 12,
    live_update_hz: float = 8.0,
    live_ttl_ms: int = 350,
    max_segment_chunks: int = 20,
    k_cross_track: float = 0.20,
    max_correction_b: float = 0.08,
    gps_reanchor: bool = True,
    require_auto_switch: bool = False,
    verbose_raw: bool = True,
    max_connector_pulses_per_turn: int = DEFAULT_MAX_CONNECTOR_PULSES_PER_TURN,
    connector_turn_tolerance_deg: float = DEFAULT_CONNECTOR_TURN_TOLERANCE_DEG,
    max_connector_turn_ms: int = DEFAULT_MAX_CONNECTOR_TURN_MS,
    turn_angle_policy: str = DEFAULT_TURN_ANGLE_POLICY,
    turn_angle_override: float | None = None,
    heading_reference: str = DEFAULT_HEADING_REFERENCE,
    max_gps_jump_m: float | None = None,
) -> tuple[list[dict[str, object]], list[str], str]:
    """``segments`` 위에서 stop_correct_go 루프를 돌린다; (rows, raw_lines, abort) 반환.

    [KO] 이 모듈의 핵심 진입점. lane 세그먼트는 bounded guarded 청크를 한 번에
    하나씩 전진하며, 완전히 멈추고 영점을 확인한 뒤 안정화된 pose를 읽는다. 임계값을
    넘는 헤딩 오차는 이산 제자리 IMU 회전으로 보정하고, 횡오차는 다음 청크의 B로
    트림한다. 커넥터는 코너마다 IMU 피드백 pivot(회전 엔진 공유)을 돌리되, IMU yaw가
    없으면 캘리브레이션 target_angle_deg에서 유도한 개루프 펄스 수로 대체한다.
    mission 헤딩 프레임(heading = yaw + offset)은 첫 lane 정렬 상태에서 한 번만 캡처해
    런 전체에 이어 붙인다. ``max_gps_jump_m``는 lane 길이 스케일의 GPS 텔레포트를
    거부하고 그 사이클을 추측항법으로 처리한다.

    Run the stop_correct_go loop over ``segments``; return (rows, raw_lines, abort).

    Lane segments advance one bounded guarded chunk at a time, fully stopping and
    confirming zero before reading a settled pose; heading errors over threshold
    are corrected by a discrete IMU turn-in-place and cross-track error is trimmed
    onto the next chunk's B.

    Connector turns run as ONE continuous IMU-feedback pivot per corner (the
    same bounded live-drive mechanism as the heading correction), stopped when
    the measured yaw delta reaches the segment's requested angle within
    ``connector_turn_tolerance_deg`` and capped by ``max_connector_turn_ms``.
    Without IMU yaw an open-loop pulse count derived from the calibration's
    ``target_angle_deg`` is used, each pulse clamped to the firmware-safe
    duration. ``max_gps_jump_m`` (off by default) rejects teleport-scale GPS
    steps between cycles, falling back to dead reckoning for that cycle --
    essential when lane lengths are close to the GPS noise floor. Expected
    field faults set ``abort_reason`` and stop the loop cleanly rather than
    raising.
    """
    rows: list[dict[str, object]] = []
    raw_lines: list[str] = []
    gps_cache: dict[str, object] = {"lat": start_lat, "lon": start_lon, "degraded": False}
    pose_state: dict[str, object] = {"x": 0.0, "y": 0.0, "source": "plan_start", "gps_degraded": False}
    abort_reason = "NONE"
    primitive_index = 0
    provided_start_yaw = start_yaw_deg
    first_lane_pending = True
    if sensor_trust_mode not in SENSOR_TRUST_MODES:
        sensor_trust_mode = DEFAULT_SENSOR_TRUST_MODE
    if heading_reference not in HEADING_REFERENCES:
        heading_reference = DEFAULT_HEADING_REFERENCE
    if turn_angle_policy not in TURN_ANGLE_POLICIES:
        turn_angle_policy = DEFAULT_TURN_ANGLE_POLICY
    effective_max_chunks = max(1, int(max_segment_chunks))
    dead_reckon_advance_m = max(0.01, 0.30 * (float(move_chunk_ms) / 700.0))

    # ── 미션 헤딩 프레임 / Mission-heading frame ──
    # [KO] heading = yaw + offset. 오프셋은 로버가 아직 첫 lane에 정렬돼 있을 때
    # (운영자 정렬 또는 초기 헤딩 정렬 스텝) 딱 한 번 캡처한다. 하나의 기준을 런
    # 전체에 이어 붙이므로 덜 돈 커넥터가 다음 lane의 헤딩 오차로 드러나 보정된다
    # (per_lane처럼 조용히 재영점하지 않음). 프레임은 첫 커넥터 회전 전에만 캡처
    # 가능하며, 이후에야 IMU yaw가 나타나면 회전 도중 프레임을 잡는 대신 레거시
    # per_lane 동작으로 강등된다.
    #
    # Mission heading frame: heading = yaw + offset, captured once while the
    # rover is still aligned with the first lane (operator alignment or the
    # initial-heading-align step). Chaining one reference across the whole run
    # keeps a connector under-turn visible as next-lane heading error instead of
    # silently re-zeroing it per lane. The frame may only be captured before the
    # first connector turn; if IMU yaw appears later the run degrades to the
    # legacy per-lane behavior rather than capturing a frame mid-rotation.
    first_lane_heading = next(
        (
            body_heading_target_deg(s)
            for s in segments
            if str(s.get("segment_type")) not in {"connector_turn", "path_connector"}
        ),
        body_heading_target_deg(segments[0]) if segments else 0.0,
    )
    mission_frame_offset: float | None = None
    mission_capture_allowed = True
    if provided_start_yaw is not None:
        mission_frame_offset = geometry.wrap_deg(first_lane_heading - provided_start_yaw)

    def maybe_capture_mission_frame(row: dict[str, str] | None) -> None:
        """아직 미캡처이고 허용될 때, 첫 하트비트 yaw로 미션 프레임 오프셋을 잡는다.

        Capture the mission-frame offset from the first heartbeat yaw, only while
        still allowed (before the first connector) and not yet captured.
        """
        nonlocal mission_frame_offset
        if (
            heading_reference != "mission"
            or mission_frame_offset is not None
            or not mission_capture_allowed
        ):
            return
        yaw_now = telemetry.imu_relative_yaw_deg(row) if row else None
        if yaw_now is not None:
            mission_frame_offset = geometry.wrap_deg(first_lane_heading - yaw_now)

    def mission_heading(yaw_value: float | None) -> float | None:
        """미션 프레임에서 절대 헤딩 = wrap(yaw + offset); 사용 불가면 None.

        Absolute heading in the mission frame = wrap(yaw + offset), or None when
        the mission frame is unavailable/disabled.
        """
        if (
            heading_reference != "mission"
            or yaw_value is None
            or mission_frame_offset is None
        ):
            return None
        return geometry.wrap_deg(yaw_value + mission_frame_offset)

    for segment in segments:
        is_connector = str(segment["segment_type"]) in {"connector_turn", "path_connector"}
        target_heading = float(segment["target_heading_deg"])
        length_m = float(segment.get("length_m", 0.0))

        # ── 커넥터 회전 / Connector turns ──
        # 코너마다 제자리 pivot. IMU yaw가 있으면 burst->stop->measure 피드백
        # 회전(회전 엔진 공유), 없으면 캘리브레이션 각도에서 유도한 개루프 펄스.
        # 첫 커넥터에 진입하는 순간 mission 프레임 캡처를 봉인한다.
        # EN: per-corner in-place pivot -- IMU-feedback burst cycles when yaw is
        # available, else calibration-derived open-loop pulses.
        if is_connector:
            mission_capture_allowed = False
            direction = "left" if str(segment["expected_motion_direction"]) == "turn_left" else "right"
            connector = connector_command(resolved_calibration, direction)
            a_cmd = float(connector["a_cmd"])
            connector_b = float(connector["b_cmd"])
            pulse_ms = int(connector["pulse_ms"])
            calibration_source = str(connector["calibration_source"])
            connector_mode = str(connector["connector_mode"])
            requested_angle = connector_turn_angle_deg(segment)
            per_pulse = per_pulse_turn_angle_deg(
                connector,
                turn_angle_policy=turn_angle_policy,
                turn_angle_override=turn_angle_override,
            )
            try:
                heartbeat, gate = _stop_correct_go_preflight(
                    handle,
                    raw_lines,
                    heartbeat_timeout_s=heartbeat_timeout_s,
                    rc_neutral_wait_s=rc_neutral_wait_s,
                    manual_override_mode=manual_override_mode,
                    require_auto_switch=require_auto_switch,
                )
            except OSError:
                abort_reason = "SERIAL_DISCONNECT"
                break
            if gate != "NONE":
                abort_reason = gate
                break
            connector_yaw_ref = telemetry.imu_relative_yaw_deg(heartbeat)

            if connector_yaw_ref is not None:
                # IMU-feedback pivot in burst -> stop -> measure cycles (the
                # same mechanism as the lane heading correction). Yaw is read
                # only while stopped, because the firmware's MOTOR_TRACE flood
                # saturates the serial link during motion and starves
                # heartbeats; no long guarded pulse is sent, so the firmware's
                # guarded-pulse max (COMMAND_EXCEEDS_MAX_MS) cannot trigger.
                primitive_index += 1
                rate_hint = None
                if per_pulse is not None and pulse_ms > 0:
                    rate_hint = per_pulse / float(pulse_ms) * 1000.0
                try:
                    turn = _run_stop_correct_go_heading_turn(
                        handle,
                        correction_b_cmd=connector_b,
                        yaw_turn_start=float(connector_yaw_ref),
                        initial_heading_error_deg=requested_angle,
                        heading_tolerance_deg=connector_turn_tolerance_deg,
                        max_correction_ms=max_connector_turn_ms,
                        update_hz=live_update_hz,
                        ttl_ms=live_ttl_ms,
                        chunk_ms=heading_correction_chunk_ms,
                        event_timeout_s=event_timeout_s,
                        raw_lines=raw_lines,
                        verbose_raw=verbose_raw,
                        abort_on_manual_switch=require_auto_switch,
                        settle_after_move_ms=settle_after_move_ms,
                        telemetry_stabilize_ms=telemetry_stabilize_ms,
                        heartbeat_timeout_s=heartbeat_timeout_s,
                        rate_dps_hint=rate_hint,
                    )
                except OSError:
                    abort_reason = "SERIAL_DISCONNECT"
                    break
                after = turn.get("last_row") or heartbeat
                gps_after = dead_reckon_gps(after, gps_cache)
                lat = float(gps_after["lat"]) if gps_after["lat"] is not None else start_lat
                lon = float(gps_after["lon"]) if gps_after["lon"] is not None else start_lon
                x, y = geometry.goal_to_local(start_lat, start_lon, lat, lon)
                along, signed_cte, _ = geometry.projection_metrics(segment, x, y)
                yaw = telemetry.imu_relative_yaw_deg(after)
                imu_valid = yaw is not None
                yaw_for_delta = yaw if imu_valid else float(turn["yaw_final"])
                applied_delta = geometry.wrap_deg(yaw_for_delta - float(connector_yaw_ref))
                remaining_angle = geometry.wrap_deg(requested_angle - applied_delta)
                turn_completed = abs(remaining_angle) <= abs(connector_turn_tolerance_deg)
                turn_overshoot = bool(turn.get("overshoot")) or (
                    not turn_completed and remaining_angle * requested_angle < 0.0
                )
                mission_h = mission_heading(yaw)
                if mission_h is not None:
                    current_heading = mission_h
                    heading_target_for_control = body_heading_target_deg(segment)
                else:
                    current_heading = current_heading_deg(target_heading, yaw, provided_start_yaw)
                    heading_target_for_control = target_heading
                correction = {
                    "current_heading_deg": current_heading,
                    "heading_error_deg": geometry.wrap_deg(heading_target_for_control - current_heading),
                    "cross_track_error_m": signed_cte,
                    "along_track_progress_m": along,
                    "remaining_distance_m": max(0.0, length_m - along),
                    "b_cmd": connector_b,
                    "b_heading_component": 0.0,
                    "b_cte_component": 0.0,
                    "correction_source": "connector_calibration",
                }
                row = build_execution_row(
                    segment=segment,
                    primitive_index=primitive_index,
                    after_row=after,
                    pulse_rows=[],
                    start_lat=start_lat,
                    start_lon=start_lon,
                    goal_lat=goal_lat,
                    goal_lon=goal_lon,
                    start_yaw_deg=start_yaw_deg,
                    target_heading_deg=heading_target_for_control,
                    a_cmd=a_cmd,
                    correction=correction,
                    pulse_ms=int(turn["turn_duration_ms"]),
                    gps=gps_after,
                    calibration_source=calibration_source,
                    connector_mode=connector_mode,
                    # The live turn has no ARM/ACK pulse window; a REJECT ends
                    # the turn (recorded as turn_rejected) like a rejected
                    # correction turn, and the next preflight gates the mission.
                    block_reason_override=None,
                    drive_mode="continuous",
                    chunk_index=1,
                    path_control_mode="stop_correct_go",
                    pose={
                        "x": x,
                        "y": y,
                        "gps_valid": not bool(gps_after["gps_degraded"]),
                        "gps_reanchored": False,
                    },
                    base_a_cmd=a_cmd,
                    b_trim=0.0,
                )
                row.update(
                    {
                        "drive_mode": "stop_correct_go",
                        "phase": "connector",
                        "imu_valid": imu_valid,
                        "move_a_cmd": "0.000",
                        "move_b_cmd": f"{connector_b:.3f}",
                        "correction_b_cmd": "0.000",
                        "correction_duration_ms": 0,
                        "correction_success": "NA",
                        "post_correction_heading_error_deg": telemetry._fmt(correction["heading_error_deg"]),
                        "sensor_source": "imu",
                        "fallback_used": False,
                        "heading_reference": heading_reference,
                        "turn_angle_policy": turn_angle_policy,
                        "turn_mode": "live_imu",
                        "requested_turn_angle_deg": telemetry._fmt(requested_angle),
                        "calibration_target_angle_deg": (
                            telemetry._fmt(per_pulse) if per_pulse is not None else "NA"
                        ),
                        "turn_pulse_index": 1,
                        "turn_pulse_budget": 1,
                        "turn_planned_pulses": connector_planned_pulses(requested_angle, per_pulse),
                        "yaw_turn_ref_deg": telemetry._fmt(connector_yaw_ref),
                        "applied_turn_delta_deg": telemetry._fmt(applied_delta),
                        "remaining_turn_error_deg": telemetry._fmt(remaining_angle),
                        "turn_measured_by_imu": True,
                        "turn_duration_ms": int(turn["turn_duration_ms"]),
                        "turn_timed_out": bool(turn["timed_out"]),
                        "turn_rejected": bool(turn["rejected"]),
                        "connector_turn_completed": turn_completed,
                        "turn_overshoot": turn_overshoot,
                    }
                )
                rows.append(row)
                if verbose_raw:
                    print(stop_correct_go_status_line(row))
                if row["valid_pulse"] is not True:
                    abort_reason = str(row["invalid_reason"])
                    break
                if bool(turn.get("manual_switch")):
                    abort_reason = MANUAL_SWITCH_ABORT_REASON
                    break
                continue

            # No IMU yaw: bounded open-loop pulses derived from the calibrated
            # per-pulse angle. The firmware caps a single guarded pulse, so a
            # long calibrated duration is clamped and the planned pulse count is
            # scaled to keep the total commanded rotation roughly unchanged.
            safe_pulse_ms = max(1, min(int(pulse_ms), OPEN_LOOP_CONNECTOR_PULSE_MS_CAP))
            effective_per_pulse = None
            if per_pulse is not None and pulse_ms > 0:
                effective_per_pulse = per_pulse * (safe_pulse_ms / float(pulse_ms))
            if connector_mode == "repeated_pulses":
                budget = int(left_fixed_pulses if direction == "left" else right_fixed_pulses)
            else:
                budget = max(
                    1,
                    min(
                        int(max_connector_pulses_per_turn),
                        connector_planned_pulses(requested_angle, effective_per_pulse),
                    ),
                )
            applied_delta = 0.0
            remaining_angle = requested_angle
            chunk_index = 0
            while chunk_index < budget:
                chunk_index += 1
                primitive_index += 1
                if chunk_index > 1:
                    try:
                        heartbeat, gate = _stop_correct_go_preflight(
                            handle,
                            raw_lines,
                            heartbeat_timeout_s=heartbeat_timeout_s,
                            rc_neutral_wait_s=rc_neutral_wait_s,
                            manual_override_mode=manual_override_mode,
                            require_auto_switch=require_auto_switch,
                        )
                    except OSError:
                        abort_reason = "SERIAL_DISCONNECT"
                        break
                    if gate != "NONE":
                        abort_reason = gate
                        break
                try:
                    planned = planned_pulse(
                        seq=primitive_index, a_cmd=a_cmd, b_cmd=connector_b, pulse_ms=safe_pulse_ms
                    )
                    pulse_rows = executor.send_pulse(
                        handle, planned, raw_lines, event_timeout_s=event_timeout_s, verbose_raw=verbose_raw
                    )
                    manual_after_pulse = require_auto_switch and manual_switch_seen(pulse_rows)
                    after = _collect_stabilized_heartbeat(
                        handle,
                        raw_lines,
                        settle_after_move_ms=settle_after_move_ms,
                        telemetry_stabilize_ms=telemetry_stabilize_ms,
                        heartbeat_timeout_s=heartbeat_timeout_s,
                        verbose_raw=verbose_raw,
                    ) or heartbeat
                except OSError:
                    abort_reason = "SERIAL_DISCONNECT"
                    break
                gps_after = dead_reckon_gps(after, gps_cache)
                lat = float(gps_after["lat"]) if gps_after["lat"] is not None else start_lat
                lon = float(gps_after["lon"]) if gps_after["lon"] is not None else start_lon
                x, y = geometry.goal_to_local(start_lat, start_lon, lat, lon)
                along, signed_cte, _ = geometry.projection_metrics(segment, x, y)
                if effective_per_pulse is not None:
                    applied_delta = math.copysign(effective_per_pulse * chunk_index, requested_angle)
                    remaining_angle = geometry.wrap_deg(requested_angle - applied_delta)
                turn_completed = chunk_index >= budget
                current_heading = current_heading_deg(target_heading, None, provided_start_yaw)
                correction = {
                    "current_heading_deg": current_heading,
                    "heading_error_deg": geometry.wrap_deg(target_heading - current_heading),
                    "cross_track_error_m": signed_cte,
                    "along_track_progress_m": along,
                    "remaining_distance_m": max(0.0, length_m - along),
                    "b_cmd": connector_b,
                    "b_heading_component": 0.0,
                    "b_cte_component": 0.0,
                    "correction_source": "connector_calibration",
                }
                row = build_execution_row(
                    segment=segment,
                    primitive_index=primitive_index,
                    after_row=after,
                    pulse_rows=pulse_rows,
                    start_lat=start_lat,
                    start_lon=start_lon,
                    goal_lat=goal_lat,
                    goal_lon=goal_lon,
                    start_yaw_deg=start_yaw_deg,
                    target_heading_deg=target_heading,
                    a_cmd=a_cmd,
                    correction=correction,
                    pulse_ms=safe_pulse_ms,
                    gps=gps_after,
                    calibration_source=calibration_source,
                    connector_mode=connector_mode,
                    chunk_index=chunk_index,
                    path_control_mode="stop_correct_go",
                    pose={
                        "x": x,
                        "y": y,
                        "gps_valid": not bool(gps_after["gps_degraded"]),
                        "gps_reanchored": False,
                    },
                    base_a_cmd=a_cmd,
                    b_trim=0.0,
                )
                row.update(
                    {
                        "drive_mode": "stop_correct_go",
                        "phase": "connector",
                        "imu_valid": False,
                        "move_a_cmd": f"{a_cmd:.3f}",
                        "move_b_cmd": f"{connector_b:.3f}",
                        "correction_b_cmd": "0.000",
                        "correction_duration_ms": 0,
                        "correction_success": "NA",
                        "post_correction_heading_error_deg": telemetry._fmt(correction["heading_error_deg"]),
                        "sensor_source": "calibration",
                        "fallback_used": True,
                        "heading_reference": heading_reference,
                        "turn_angle_policy": turn_angle_policy,
                        "turn_mode": "open_loop_pulses",
                        "requested_turn_angle_deg": telemetry._fmt(requested_angle),
                        "calibration_target_angle_deg": (
                            telemetry._fmt(effective_per_pulse)
                            if effective_per_pulse is not None
                            else "NA"
                        ),
                        "turn_pulse_index": chunk_index,
                        "turn_pulse_budget": budget,
                        "turn_planned_pulses": budget,
                        "yaw_turn_ref_deg": "NA",
                        "applied_turn_delta_deg": telemetry._fmt(applied_delta),
                        "remaining_turn_error_deg": telemetry._fmt(remaining_angle),
                        "turn_measured_by_imu": False,
                        "turn_duration_ms": safe_pulse_ms,
                        "turn_timed_out": False,
                        "connector_turn_completed": turn_completed,
                        "turn_overshoot": False,
                    }
                )
                rows.append(row)
                if verbose_raw:
                    print(stop_correct_go_status_line(row))
                if row["valid_pulse"] is not True:
                    abort_reason = str(row["invalid_reason"])
                    break
                if manual_after_pulse:
                    abort_reason = MANUAL_SWITCH_ABORT_REASON
                    break
            if abort_reason != "NONE":
                break
            continue

        # ── 직선 lane: 이산 전진 -> 정지 -> 측정 -> 보정 ──
        # straight lane: discrete move -> stop -> measure -> correct
        # [KO] 청크마다: 안전 게이트 -> 이전 사이클의 횡오차 트림을 B에 실은 bounded
        # guarded 전진 -> 정착/안정화 후 pose 읽기 -> (임계 초과 시) 이산 IMU 제자리
        # 헤딩 보정 -> 다음 청크용 횡오차 트림 계산 -> 행 기록. GPS 점프 가드 포함.
        motion = geometry._motion_calibrated(
            resolved_calibration, str(segment["expected_motion_direction"])
        )
        a_cmd = float(motion["a_cmd"])
        calibration_source = str(motion.get("calibration_source", "unknown"))
        segment_ref_yaw: float | None = None
        next_b_trim = 0.0
        for chunk_index in range(1, effective_max_chunks + 1):
            primitive_index += 1
            try:
                heartbeat, gate = _stop_correct_go_preflight(
                    handle,
                    raw_lines,
                    heartbeat_timeout_s=heartbeat_timeout_s,
                    rc_neutral_wait_s=rc_neutral_wait_s,
                    manual_override_mode=manual_override_mode,
                    require_auto_switch=require_auto_switch,
                )
                if gate != "NONE":
                    abort_reason = gate
                    break
                maybe_capture_mission_frame(heartbeat)
                if segment_ref_yaw is None:
                    segment_ref_yaw = reference_yaw_for_segment(
                        heartbeat,
                        provided_start_yaw=provided_start_yaw,
                        use_provided=first_lane_pending,
                    )
                    first_lane_pending = False
                gps_before = dead_reckon_gps(heartbeat, gps_cache)
                gps_action = geometry.gps_policy_action(
                    bool(gps_before["gps_degraded"]), gps_degradation_policy
                )
                if gps_action == "abort":
                    abort_reason = "GPS_DEGRADED"
                    break
                if gps_action == "pause":
                    continue
                # MOVE: one bounded guarded chunk (B = cross-track trim from prior cycle).
                move_b = float(next_b_trim)
                planned = planned_pulse(
                    seq=primitive_index, a_cmd=a_cmd, b_cmd=move_b, pulse_ms=int(move_chunk_ms)
                )
                pulse_rows = executor.send_pulse(
                    handle, planned, raw_lines, event_timeout_s=event_timeout_s, verbose_raw=verbose_raw
                )
                manual_after_pulse = require_auto_switch and manual_switch_seen(pulse_rows)
                # SETTLE + STABILIZE telemetry, then read a settled pose.
                stabilized = _collect_stabilized_heartbeat(
                    handle,
                    raw_lines,
                    settle_after_move_ms=settle_after_move_ms,
                    telemetry_stabilize_ms=telemetry_stabilize_ms,
                    heartbeat_timeout_s=heartbeat_timeout_s,
                    verbose_raw=verbose_raw,
                ) or heartbeat
            except OSError:
                abort_reason = "SERIAL_DISCONNECT"
                break

            gps_after = dead_reckon_gps(stabilized, gps_cache)
            yaw = telemetry.imu_relative_yaw_deg(stabilized)
            imu_valid = yaw is not None
            gps_valid = bool(
                gps_after["lat"] is not None
                and gps_after["lon"] is not None
                and not gps_after["gps_degraded"]
            )
            prev_pose_x = telemetry._optional_float(pose_state.get("x"))
            prev_pose_y = telemetry._optional_float(pose_state.get("y"))
            pose = _pose_from_gps_or_dead_reckon(
                row=stabilized,
                gps=gps_after,
                pose_state=pose_state,
                segment=segment,
                start_lat=start_lat,
                start_lon=start_lon,
                advance_m=0.0 if gps_valid else dead_reckon_advance_m,
                gps_reanchor=gps_reanchor,
            )
            # GPS jump guard: on small fields a noisy fix can "teleport" the
            # pose by lane-length scale in one cycle (instantly completing a
            # lane). Reject steps far beyond what one chunk can physically
            # drive and dead-reckon this cycle instead.
            gps_jump_rejected = False
            if (
                max_gps_jump_m is not None
                and bool(pose.get("gps_valid"))
                and prev_pose_x is not None
                and prev_pose_y is not None
            ):
                jump_m = math.hypot(float(pose["x"]) - prev_pose_x, float(pose["y"]) - prev_pose_y)
                allowed_jump_m = max(float(max_gps_jump_m), dead_reckon_advance_m * 3.0)
                if jump_m > allowed_jump_m:
                    dr_x, dr_y = _dead_reckon_pose_along_segment(
                        segment=segment,
                        x_m=prev_pose_x,
                        y_m=prev_pose_y,
                        advance_m=dead_reckon_advance_m,
                    )
                    pose_state.update(
                        {"x": dr_x, "y": dr_y, "source": "gps_jump_rejected", "gps_degraded": True}
                    )
                    pose = {
                        "x": dr_x,
                        "y": dr_y,
                        "gps_valid": False,
                        "gps_reanchored": False,
                        "source": "gps_jump_rejected",
                    }
                    gps_valid = False
                    gps_jump_rejected = True
            sensor = stop_correct_go_sensor_source(
                gps_valid=gps_valid,
                imu_valid=imu_valid,
                trust_mode=sensor_trust_mode,
                allow_calibration_fallback=allow_calibration_fallback,
            )
            if not sensor["ok"]:
                abort_reason = "SENSOR_UNAVAILABLE"
                break
            x = float(pose["x"])
            y = float(pose["y"])
            along, signed_cte, _ = geometry.projection_metrics(segment, x, y)
            mission_h = mission_heading(yaw if imu_valid else None)
            if mission_h is not None:
                # Mission frame tracks the BODY: backward lanes hold travel+180.
                current_heading = mission_h
                heading_target_for_control = body_heading_target_deg(segment)
            else:
                current_heading = current_heading_deg(
                    target_heading, yaw if imu_valid else None, segment_ref_yaw
                )
                heading_target_for_control = target_heading
            heading_error = geometry.wrap_deg(heading_target_for_control - current_heading)
            remaining_m = max(0.0, length_m - along)

            # HEADING CORRECTION: discrete IMU turn-in-place when over threshold.
            decision = stop_correct_go_heading_decision(
                heading_error_deg=heading_error,
                threshold_deg=heading_correction_threshold_deg,
                imu_valid=imu_valid,
                b_left=heading_correction_b_left,
                b_right=heading_correction_b_right,
            )
            correction_b_cmd = 0.0
            correction_duration_ms = 0
            correction_success: object = "NA"
            post_correction_error = heading_error
            correction_manual_switch = False
            phase = "move"
            if decision["needs_correction"] and not manual_after_pulse:
                phase = "correction"
                correction_turn = connector_command(
                    resolved_calibration, str(decision["turn_direction"])
                )
                correction_rate_hint = None
                correction_target = correction_turn.get("target_angle_deg")
                if correction_target is not None and int(correction_turn["pulse_ms"]) > 0:
                    correction_rate_hint = (
                        float(correction_target) / float(correction_turn["pulse_ms"]) * 1000.0
                    )
                try:
                    turn = _run_stop_correct_go_heading_turn(
                        handle,
                        correction_b_cmd=float(decision["correction_b_cmd"]),
                        yaw_turn_start=float(yaw),
                        initial_heading_error_deg=heading_error,
                        heading_tolerance_deg=heading_correction_tolerance_deg,
                        max_correction_ms=max_heading_correction_ms,
                        update_hz=live_update_hz,
                        ttl_ms=live_ttl_ms,
                        chunk_ms=heading_correction_chunk_ms,
                        event_timeout_s=event_timeout_s,
                        raw_lines=raw_lines,
                        verbose_raw=verbose_raw,
                        abort_on_manual_switch=require_auto_switch,
                        settle_after_move_ms=settle_after_move_ms,
                        telemetry_stabilize_ms=telemetry_stabilize_ms,
                        heartbeat_timeout_s=heartbeat_timeout_s,
                        rate_dps_hint=correction_rate_hint,
                    )
                except OSError:
                    abort_reason = "SERIAL_DISCONNECT"
                    break
                correction_b_cmd = float(decision["correction_b_cmd"])
                correction_duration_ms = int(turn["turn_duration_ms"])
                correction_success = bool(turn["correction_success"])
                post_correction_error = float(turn["final_heading_error_deg"])
                correction_manual_switch = bool(turn.get("manual_switch"))

            # CROSS-TRACK: trim the next chunk's B when off the line.
            next_b_trim = stop_correct_go_cross_track_trim(
                cross_track_error_m=signed_cte,
                threshold_m=cross_track_correction_threshold_m,
                k_cross_track=k_cross_track,
                max_correction_b=max_correction_b,
            )

            correction = {
                "current_heading_deg": current_heading,
                "heading_error_deg": heading_error,
                "cross_track_error_m": signed_cte,
                "along_track_progress_m": along,
                "remaining_distance_m": remaining_m,
                "b_cmd": move_b,
                "b_heading_component": 0.0,
                "b_cte_component": move_b,
                "correction_source": "stop_correct_go",
            }
            row = build_execution_row(
                segment=segment,
                primitive_index=primitive_index,
                after_row=stabilized,
                pulse_rows=pulse_rows,
                start_lat=start_lat,
                start_lon=start_lon,
                goal_lat=goal_lat,
                goal_lon=goal_lon,
                start_yaw_deg=start_yaw_deg,
                target_heading_deg=heading_target_for_control,
                a_cmd=a_cmd,
                correction=correction,
                pulse_ms=int(move_chunk_ms),
                gps=gps_after,
                calibration_source=calibration_source,
                connector_mode="lane",
                chunk_index=chunk_index,
                path_control_mode="stop_correct_go",
                pose={
                    "x": x,
                    "y": y,
                    "gps_valid": gps_valid,
                    "gps_reanchored": bool(pose.get("gps_reanchored")),
                },
                base_a_cmd=a_cmd,
                b_trim=move_b,
            )
            row.update(
                {
                    "drive_mode": "stop_correct_go",
                    "phase": phase,
                    "imu_valid": imu_valid,
                    "move_a_cmd": f"{a_cmd:.3f}",
                    "move_b_cmd": f"{move_b:.3f}",
                    "correction_b_cmd": f"{correction_b_cmd:.3f}",
                    "correction_duration_ms": correction_duration_ms,
                    "correction_success": correction_success,
                    "post_correction_heading_error_deg": telemetry._fmt(post_correction_error),
                    "next_b_trim": f"{next_b_trim:.3f}",
                    "sensor_source": str(sensor["source"]),
                    "fallback_used": bool(sensor["fallback_used"]),
                    "heading_reference": heading_reference,
                    "turn_angle_policy": turn_angle_policy,
                    "gps_jump_rejected": gps_jump_rejected,
                }
            )
            rows.append(row)
            if verbose_raw:
                print(stop_correct_go_status_line(row))
            if row["valid_pulse"] is not True:
                abort_reason = str(row["invalid_reason"])
                break
            if manual_after_pulse or correction_manual_switch:
                abort_reason = MANUAL_SWITCH_ABORT_REASON
                break
            if remaining_m <= max(0.05, dead_reckon_advance_m * 0.5) or along >= length_m:
                break

        if abort_reason != "NONE":
            break
        if rows and rows[-1]["valid_pulse"] is not True:
            break

    return rows, raw_lines, abort_reason
