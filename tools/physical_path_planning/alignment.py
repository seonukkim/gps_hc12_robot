"""감독형 물리 경로 실행을 위한 초기 헤딩 정렬.
Initial heading alignment for supervised physical path execution.

목적/역할 (Purpose):
    로버가 첫 번째 계획 레인을 따라가기 전에 그 레인의 목표 헤딩을 향하도록 몸을
    돌려 맞추는 모듈. IMU 상대 요(yaw)는 **절대 ENU 헤딩이 아니므로**, 짧은
    경계-GPS 프로브(살짝 전진 → GPS 변위 측정)로 현재 절대 헤딩을 복원한 뒤, IMU
    요는 오직 *회전 피드백*으로만 써서 최단 부호 헤딩오차만큼 돌리고 남은 오차가
    허용범위에 들면 멈춘다.

    Points the rover at the first planned lane's target heading before following
    it. IMU relative yaw is NOT an absolute ENU heading, so the absolute current
    heading is recovered from a short bounded GPS probe (nudge forward, measure
    GPS displacement); IMU yaw is then used only as turn feedback.

시스템 내 위치 (Where it sits):
    :mod:`controller` 의 동급(peer) 모듈. leaf 수학(:mod:`geometry`), 텔레메트리
    접근자(:mod:`telemetry`), 시리얼 대면 경계 실행기(:mod:`executor`),
    준비-상태 가드(:mod:`checks`)를 재사용한다. CLI 가 이 모듈을 연결하며, 패키지
    내 어떤 모듈도 이걸 되-import 하지 않아 순환이 없다.

    Peer of :mod:`controller`; reuses :mod:`geometry`, :mod:`telemetry`,
    :mod:`executor`, :mod:`checks`. Wired by the CLI; nothing imports it back.

핵심 개념·불변식 (Key concepts / invariants):
    * 헤딩 규약은 플래너와 동일: ``atan2(north, east)`` (동 0°, 반시계 증가).
    * ``pure helpers`` 섹션은 시리얼 없이 단위 테스트 가능. 실제 회전만
      ``executor`` 로 경계 명령을 보낸다.
    * 예상되는 현장 결함(GPS 없음/IMU 없음/프로브 변위 미달)에는 예외를 던지지
      않고 ``reason`` 을 세팅한 가드된 결과를 돌려준다 — 호출자가 요약을 남기고
      중단 여부를 결정할 수 있게.
    * 모든 결과는 :func:`checks.assert_not_ready_for_full_path_following` 를 거쳐
      ``ready_for_full_path_following=False`` 를 보장한다.

    Heading convention matches the planner (``atan2(north, east)``). Pure helpers
    are serial-free/testable; only the turn sends bounded ``executor`` commands.
    Expected field faults never raise — they set ``reason`` and return a guarded
    result. Every result carries ``ready_for_full_path_following=False``.

세 가지 전략 (Three strategies):

* ``gps_probe`` -- 자동: 전진 프로브 → GPS 변위로 헤딩 추정 → IMU 피드백 회전.
  automatic: probe forward, estimate heading from GPS displacement, rotate with
  IMU feedback.
* ``user_confirmed`` -- 운영자가 로버를 A→B 첫 레인 방향으로 직접 맞추고 Enter;
  현재 IMU 요를 레인 기준으로 포착(프로브·자동 회전 없음).
  operator points the rover manually and presses Enter; current IMU yaw is
  captured as the lane reference (no probe, no automatic turn).
* ``skip`` -- 정렬을 수행하지 않음 / no alignment is performed.

리팩토링 노트 (Refactoring notes):
    회전 방향은 (GPS 기반) 헤딩오차 부호로 정하고, 회전량은 IMU 요 크기로 측정한다
    — 이 분리를 :func:`remaining_heading_error_deg` 가 캡슐화한다. 새 전략을 더할
    때는 :data:`ALIGNMENT_STRATEGIES` 와 :func:`align_heading` 의 분기를 함께
    갱신하고, 새 결과 키는 :func:`_base_result` 에 먼저 선언할 것.

    Turn direction comes from the GPS heading-error sign; the amount from IMU yaw
    magnitude (see :func:`remaining_heading_error_deg`). Adding a strategy means
    updating :data:`ALIGNMENT_STRATEGIES`, the branch in :func:`align_heading`,
    and declaring any new result key in :func:`_base_result`.
"""
from __future__ import annotations

import math
import time
from typing import Callable, Sequence

from tools.physical_path_planning import checks, executor, geometry, telemetry

# ── 기본 파라미터 (프로브·회전·타임아웃) / Default params (probe, turn, timeouts) ──
DEFAULT_PROBE_A = 0.25
DEFAULT_PROBE_DURATION_S = 1.0
DEFAULT_MIN_PROBE_DISTANCE_M = 0.30
DEFAULT_HEADING_TOLERANCE_DEG = 8.0
DEFAULT_TURN_B_LEFT = 0.24
DEFAULT_TURN_B_RIGHT = -0.12
DEFAULT_MAX_TURN_DURATION_S = 3.0
DEFAULT_TURN_UPDATE_HZ = 8.0
DEFAULT_TURN_TTL_MS = 350
DEFAULT_TURN_CHUNK_MS = 300
DEFAULT_EVENT_TIMEOUT_S = 3.0
DEFAULT_HEARTBEAT_TIMEOUT_S = 5.0

ALIGNMENT_STRATEGIES = ("gps_probe", "user_confirmed", "skip")


# --- Pure helpers (no serial; directly unit-testable) -------------------------
# ── 순수 헬퍼 (시리얼 없음, 단위 테스트 가능) / Pure helpers (serial-free, testable) ──


def first_segment_target_heading(segments: Sequence[dict[str, object]]) -> float:
    """첫 레인이 정렬돼야 할 목표 헤딩(ENU°) / Target heading (ENU deg) of the first lane.

    세그먼트가 없으면 ``ValueError``. Raises if there are no segments.
    """
    if not segments:
        raise ValueError("no planned segments: cannot determine target heading")
    return float(segments[0]["target_heading_deg"])


def estimate_heading_deg(
    start_lat: float, start_lon: float, end_lat: float, end_lon: float
) -> tuple[float, float]:
    """시작→끝 GPS 쌍에서 ENU 헤딩+변위 추정 / Estimate ENU heading + displacement from a GPS pair.

    반환 ``(heading_deg, distance_m)``: 헤딩은 동쪽 기준 반시계
    (``atan2(d_north, d_east)``)로 플래너의 ``target_heading_deg`` 규약과 일치.
    :func:`geometry.goal_to_local` (로컬 (동,북) 미터)을 재사용한다.

    Returns ``(heading_deg, distance_m)`` where ``heading_deg`` is measured CCW
    from East (``atan2(d_north, d_east)``) and matches the planner's segment
    ``target_heading_deg`` convention. Reuses :func:`geometry.goal_to_local`,
    which returns local ``(east, north)`` metres.
    """
    east_m, north_m = geometry.goal_to_local(start_lat, start_lon, end_lat, end_lon)
    distance_m = math.hypot(east_m, north_m)
    heading_deg = math.degrees(math.atan2(north_m, east_m))
    return heading_deg, distance_m


def shortest_heading_error_deg(target_heading_deg: float, current_heading_deg: float) -> float:
    """최단 부호 헤딩오차 / Shortest signed error (target - current), wrapped to [-180, 180]."""
    return geometry.wrap_deg(target_heading_deg - current_heading_deg)


def select_turn_direction(heading_error_deg: float) -> str:
    """오차 부호로 회전 방향 결정 / Positive error => ``left`` (B>0, CCW raises heading) else ``right``."""
    return "left" if heading_error_deg > 0.0 else "right"


def turn_b_command(turn_direction: str, turn_b_left: float, turn_b_right: float) -> float:
    """회전 방향에 맞는 경계 B 명령 선택 / Bounded B steering command for the chosen direction."""
    return float(turn_b_left) if turn_direction == "left" else float(turn_b_right)


def remaining_heading_error_deg(initial_heading_error_deg: float, yaw_delta_deg: float) -> float:
    """회전 후 남은 부호 오차 / Signed error left after rotating ``|yaw_delta|`` in the error's direction.

    회전 방향은 (GPS 기반) 헤딩오차 부호로 정하고, 실제 회전량은 IMU 요 크기로
    재므로 적용 회전은 ``copysign(|yaw_delta|, initial_error)`` 이다. 이 부호/크기
    분리가 이 함수의 핵심.

    The turn direction is chosen from the (GPS-derived) heading-error sign and the
    IMU yaw magnitude measures how far we have actually rotated, so the applied
    rotation is ``copysign(|yaw_delta|, initial_error)``.
    """
    applied = math.copysign(abs(yaw_delta_deg), initial_heading_error_deg)
    return geometry.wrap_deg(initial_heading_error_deg - applied)


def within_tolerance(heading_error_deg: float, heading_tolerance_deg: float) -> bool:
    """오차 크기가 허용범위 이내인가 / True when the heading error magnitude is within tolerance."""
    return abs(heading_error_deg) <= abs(heading_tolerance_deg)


def plan_alignment(
    *,
    target_heading_deg: float,
    current_heading_deg: float,
    probe_distance_m: float,
    min_probe_distance_m: float,
    heading_tolerance_deg: float,
    turn_b_left: float,
    turn_b_right: float,
) -> dict[str, object]:
    """측정 헤딩+프로브 변위로부터 순수 정렬 결정 / Pure alignment decision from heading + probe.

    회전 필요 여부·방향/명령·부호 있는 초기 헤딩오차를 담은 dict 를 돌려준다.
    프로브 변위가 최소치 미만이면(GPS 헤딩을 신뢰 불가)
    ``PROBE_GPS_DISPLACEMENT_TOO_SMALL`` 실패를, 이미 허용범위면 ``ALREADY_ALIGNED``
    를 반환한다. 시리얼 접근 없음 — 순수 판단 함수.

    Returns a dict: needs-turn, direction/command, and the signed initial error;
    or a ``PROBE_GPS_DISPLACEMENT_TOO_SMALL`` failure when the probe did not move
    far enough to trust the GPS heading (``ALREADY_ALIGNED`` if within tolerance).
    """
    if probe_distance_m < min_probe_distance_m:
        return {
            "ok": False,
            "reason": "PROBE_GPS_DISPLACEMENT_TOO_SMALL",
            "initial_heading_error_deg": None,
            "needs_turn": False,
            "turn_direction": "none",
            "turn_b_cmd": 0.0,
        }
    error = shortest_heading_error_deg(target_heading_deg, current_heading_deg)
    if within_tolerance(error, heading_tolerance_deg):
        return {
            "ok": True,
            "reason": "ALREADY_ALIGNED",
            "initial_heading_error_deg": error,
            "needs_turn": False,
            "turn_direction": "none",
            "turn_b_cmd": 0.0,
        }
    direction = select_turn_direction(error)
    return {
        "ok": True,
        "reason": "TURN_REQUIRED",
        "initial_heading_error_deg": error,
        "needs_turn": True,
        "turn_direction": direction,
        "turn_b_cmd": turn_b_command(direction, turn_b_left, turn_b_right),
    }


# --- Result shaping -----------------------------------------------------------
# ── 결과 dict 형성 / Result-dict shaping ──


def _base_result(strategy: str, target_heading_deg: float | None) -> dict[str, object]:
    """모든 정렬 결과의 기본 스키마 / Base schema for every alignment result.

    모든 가능한 출력 키를 안전한 기본값으로 미리 선언한다. 새 결과 필드는 반드시
    여기에 먼저 추가할 것(호출처마다 키 존재를 가정하므로).
    Declares every output key with a safe default; add new fields here first.
    """
    return {
        "mode": "align-heading",
        "strategy": strategy,
        "target_heading_source": "first_segment",
        "target_heading_deg": round(target_heading_deg, 3) if target_heading_deg is not None else None,
        "probe_start_lat": None,
        "probe_start_lon": None,
        "probe_end_lat": None,
        "probe_end_lon": None,
        "probe_distance_m": None,
        "estimated_current_heading_deg": None,
        "initial_heading_error_deg": None,
        "final_heading_error_deg": None,
        "turn_direction": "none",
        "turn_b_cmd": 0.0,
        "turn_duration_ms": 0,
        "aligned_yaw_deg": None,
        "alignment_success": False,
        "ready_for_execute_plan": False,
        "success": False,
        "reason": "NOT_RUN",
        "ready_for_full_path_following": False,
    }


def _finalize(result: dict[str, object]) -> dict[str, object]:
    """성공 플래그 세팅 + 준비-금지 가드 확인 후 반환 / Stamp success + never-ready guard, then return."""
    result["success"] = bool(result.get("alignment_success"))
    checks.assert_not_ready_for_full_path_following(result)
    return result


# --- Serial-facing routine ----------------------------------------------------
# ── 시리얼 대면 루틴 (executor 로 경계 명령) / Serial-facing routine (bounded via executor) ──


def _wait_for_heartbeat(
    handle: object, raw_lines: list[str], timeout_s: float, *, verbose_raw: bool
) -> dict[str, str] | None:
    """다음 HEARTBEAT 텔레메트리 행 대기 / Wait for the next HEARTBEAT telemetry row (or None)."""
    return executor.wait_for_row(
        handle,
        raw_lines,
        lambda row: telemetry.event(row) == "HEARTBEAT",
        timeout_s,
        verbose_raw=verbose_raw,
    )


def _row_lat_lon(row: dict[str, str] | None) -> tuple[float | None, float | None]:
    """행에서 위경도 추출(별칭 키 허용) / Extract lat/lon from a row, tolerating alias keys."""
    if row is None:
        return None, None
    lat = telemetry._optional_float(row.get("current_lat", row.get("gps_lat")))
    lon = telemetry._optional_float(row.get("current_lon", row.get("gps_lon")))
    return lat, lon


def _execute_alignment_turn(
    handle: object,
    *,
    turn_b_cmd: float,
    yaw_turn_start: float,
    initial_heading_error_deg: float,
    heading_tolerance_deg: float,
    max_turn_duration_s: float,
    update_hz: float,
    ttl_ms: int,
    chunk_ms: int,
    event_timeout_s: float,
    raw_lines: list[str],
    trace: list[dict[str, str]],
    verbose_raw: bool,
) -> dict[str, object]:
    """IMU 요 피드백으로 회전, 남은 오차가 허용범위면 정지 / Rotate with IMU feedback, stop within tolerance.

    경계 ``USB_DRIVE_LIVE_SET`` 회전 setpoint(A=0, B=turn_b_cmd)를 반복 발행하고
    마지막에 ``USB_DRIVE_LIVE_STOP`` 을 보낸다 — :func:`executor.send_live_drive`
    와 같되, 고정 시간이 아니라 **IMU 정지 조건**으로 조기 종료한다. 안전장치:
    ``max_turn_duration_s`` 데드라인과 ``REJECT`` 이벤트에서도 중단한다.
    반환: 최종 요·남은 오차·소요 ms·타임아웃/거부 플래그.

    Issues bounded turn setpoints and a final stop like
    :func:`executor.send_live_drive`, but breaks early on the IMU stop condition
    (not a fixed duration). Also bounded by ``max_turn_duration_s`` and a
    ``REJECT`` event. Returns final yaw, remaining error, ms, and flags.
    """
    seq = 1
    duration_ms = max(1, int(chunk_ms))
    update_period_s = 1.0 / max(1.0, float(update_hz))
    start_time = time.monotonic()
    deadline = start_time + max(0.0, float(max_turn_duration_s))
    yaw_latest = yaw_turn_start
    remaining = initial_heading_error_deg
    timed_out = False
    rejected = False
    while True:
        # 안전 상한: 허용범위에 못 들어도 최대 회전 시간이 지나면 무조건 정지.
        # Safety cap: stop after the max turn time even if tolerance is unmet.
        if time.monotonic() >= deadline:
            timed_out = True
            break
        executor.write_command(
            handle,
            (
                f"USB_DRIVE_LIVE_SET seq={seq} a=0.000 b={float(turn_b_cmd):.3f} "
                f"duration_ms={duration_ms} ttl_ms={int(ttl_ms)}"
            ),
        )
        row = executor.wait_for_row(
            handle,
            raw_lines,
            lambda r: telemetry.imu_relative_yaw_deg(r) is not None
            or telemetry.event(r) in {"ACTIVE", "REJECT"},
            min(update_period_s, event_timeout_s),
            verbose_raw=verbose_raw,
        )
        if row is not None:
            trace.append(row)
            if telemetry.event(row) == "REJECT":
                rejected = True
                break
            yaw = telemetry.imu_relative_yaw_deg(row)
            if yaw is not None:
                yaw_latest = yaw
                # 회전 시작 기준 요 변화량으로 남은 오차를 갱신 → 허용범위면 정지.
                # 이것이 이 루프의 핵심 정지 조건(고정 시간이 아님).
                # Yaw delta since turn start updates remaining error; the IMU
                # stop condition (not a fixed duration) ends the loop.
                yaw_delta = geometry.wrap_deg(yaw - yaw_turn_start)
                remaining = remaining_heading_error_deg(initial_heading_error_deg, yaw_delta)
                if within_tolerance(remaining, heading_tolerance_deg):
                    break
        else:
            time.sleep(min(update_period_s, 0.05))
    executor.write_command(handle, f"USB_DRIVE_LIVE_STOP seq={seq}")
    executor.wait_for_event(
        handle, raw_lines, executor.STOP_CONFIRM_EVENTS, event_timeout_s, verbose_raw=verbose_raw
    )
    turn_duration_ms = int((time.monotonic() - start_time) * 1000.0)
    return {
        "yaw_final": yaw_latest,
        "final_heading_error_deg": remaining,
        "turn_duration_ms": turn_duration_ms,
        "timed_out": timed_out,
        "rejected": rejected,
    }


def align_heading(
    handle: object,
    *,
    segments: Sequence[dict[str, object]],
    strategy: str = "gps_probe",
    target_heading_source: str = "first_segment",
    probe_a: float = DEFAULT_PROBE_A,
    probe_duration_s: float = DEFAULT_PROBE_DURATION_S,
    min_probe_distance_m: float = DEFAULT_MIN_PROBE_DISTANCE_M,
    heading_tolerance_deg: float = DEFAULT_HEADING_TOLERANCE_DEG,
    turn_b_left: float = DEFAULT_TURN_B_LEFT,
    turn_b_right: float = DEFAULT_TURN_B_RIGHT,
    max_turn_duration_s: float = DEFAULT_MAX_TURN_DURATION_S,
    update_hz: float = DEFAULT_TURN_UPDATE_HZ,
    ttl_ms: int = DEFAULT_TURN_TTL_MS,
    chunk_ms: int = DEFAULT_TURN_CHUNK_MS,
    event_timeout_s: float = DEFAULT_EVENT_TIMEOUT_S,
    heartbeat_timeout_s: float = DEFAULT_HEARTBEAT_TIMEOUT_S,
    input_fn: Callable[[str], str] = input,
    raw_lines: list[str] | None = None,
    verbose_raw: bool = True,
) -> tuple[dict[str, object], list[dict[str, str]]]:
    """로버를 첫 레인 헤딩으로 정렬; ``(result, trace_rows)`` 반환 / Align to first lane heading.

    ``strategy`` 에 따라 skip / user_confirmed / gps_probe 를 수행한다. gps_probe
    흐름: 프로브 전 GPS·IMU 읽기 → 짧은 전진 프로브 → 프로브 후 GPS 읽기 →
    변위로 절대 헤딩 추정 → :func:`plan_alignment` 결정 → 필요 시 IMU 피드백 회전.
    예상 현장 결함(GPS 없음/IMU 없음/프로브 변위 미달)에는 예외를 던지지 않고
    ``reason`` 을 세팅한 가드된 결과(``alignment_success=False``)를 돌려줘, 호출자가
    요약을 남기고 중단 여부를 결정할 수 있게 한다.

    Runs skip / user_confirmed / gps_probe per ``strategy``. gps_probe: read
    GPS+IMU, forward probe, read GPS, estimate absolute heading from the
    displacement, decide via :func:`plan_alignment`, then rotate with IMU
    feedback if needed. Never raises on expected field faults (no GPS, no IMU,
    tiny probe) — each sets ``reason`` and returns a guarded result so the caller
    can still summarize and choose to abort.
    """
    if raw_lines is None:
        raw_lines = []
    trace: list[dict[str, str]] = []
    target_heading = first_segment_target_heading(segments)
    result = _base_result(strategy, target_heading)
    result["target_heading_source"] = target_heading_source

    # ── 전략 분기 / Strategy branches ──
    if strategy == "skip":
        result["reason"] = "ALIGNMENT_SKIPPED"
        result["alignment_success"] = True
        result["ready_for_execute_plan"] = True
        return _finalize(result), trace

    if strategy == "user_confirmed":
        input_fn(
            "Point the rover toward the A->B first lane heading, then press Enter to capture IMU yaw..."
        )
        heartbeat = _wait_for_heartbeat(handle, raw_lines, heartbeat_timeout_s, verbose_raw=verbose_raw)
        if heartbeat is not None:
            trace.append(heartbeat)
        yaw = telemetry.imu_relative_yaw_deg(heartbeat) if heartbeat else None
        result["estimated_current_heading_deg"] = round(target_heading, 3)
        result["initial_heading_error_deg"] = 0.0
        result["final_heading_error_deg"] = 0.0
        result["turn_direction"] = "user_confirmed"
        result["aligned_yaw_deg"] = round(yaw, 3) if yaw is not None else None
        result["alignment_success"] = True
        result["ready_for_execute_plan"] = True
        result["reason"] = "USER_CONFIRMED_HEADING" if yaw is not None else "USER_CONFIRMED_HEADING_NO_IMU"
        return _finalize(result), trace

    if strategy != "gps_probe":
        result["reason"] = "UNKNOWN_ALIGNMENT_STRATEGY"
        return _finalize(result), trace

    # ── gps_probe: 자동 정렬 절차 / gps_probe: automatic alignment sequence ──
    # 1) 프로브 전 GPS+IMU 읽기 / read GPS+IMU before the probe.
    before = _wait_for_heartbeat(handle, raw_lines, heartbeat_timeout_s, verbose_raw=verbose_raw)
    if before is not None:
        trace.append(before)
    start_lat, start_lon = _row_lat_lon(before)
    if start_lat is None or start_lon is None:
        result["reason"] = "PROBE_GPS_UNAVAILABLE_BEFORE"
        return _finalize(result), trace
    result["probe_start_lat"] = round(start_lat, 7)
    result["probe_start_lon"] = round(start_lon, 7)

    # 2) 짧은 경계 전진 프로브(A만, B 중립) / short bounded forward probe (A only, B neutral).
    probe_rows = executor.send_live_drive(
        handle,
        seq=1,
        duration_s=float(probe_duration_s),
        update_hz=float(update_hz),
        ttl_ms=int(ttl_ms),
        command_fn=lambda _row: (float(probe_a), 0.0),
        raw_lines=raw_lines,
        event_timeout_s=float(event_timeout_s),
        verbose_raw=verbose_raw,
    )
    trace.extend(probe_rows)

    # 3) 프로브 후 GPS 읽기 / read GPS after the probe.
    after = _wait_for_heartbeat(handle, raw_lines, heartbeat_timeout_s, verbose_raw=verbose_raw)
    if after is not None:
        trace.append(after)
    end_lat, end_lon = _row_lat_lon(after)
    if end_lat is None or end_lon is None:
        result["reason"] = "PROBE_GPS_UNAVAILABLE_AFTER"
        return _finalize(result), trace
    result["probe_end_lat"] = round(end_lat, 7)
    result["probe_end_lon"] = round(end_lon, 7)

    # 4) GPS 변위로 현재 절대 헤딩 추정 / estimate absolute current heading from GPS displacement.
    current_heading, distance_m = estimate_heading_deg(start_lat, start_lon, end_lat, end_lon)
    result["probe_distance_m"] = round(distance_m, 4)
    result["estimated_current_heading_deg"] = round(current_heading, 3)

    decision = plan_alignment(
        target_heading_deg=target_heading,
        current_heading_deg=current_heading,
        probe_distance_m=distance_m,
        min_probe_distance_m=float(min_probe_distance_m),
        heading_tolerance_deg=float(heading_tolerance_deg),
        turn_b_left=float(turn_b_left),
        turn_b_right=float(turn_b_right),
    )
    if not decision["ok"]:
        result["reason"] = str(decision["reason"])
        result["next_recommended_action"] = (
            "Increase --probe-duration-s/--probe-a so the GPS probe moves at least "
            "--min-probe-distance-m, or use --strategy user_confirmed."
        )
        return _finalize(result), trace

    initial_error = float(decision["initial_heading_error_deg"])
    result["initial_heading_error_deg"] = round(initial_error, 3)
    yaw_after_probe = telemetry.imu_relative_yaw_deg(after)

    if not decision["needs_turn"]:
        result["final_heading_error_deg"] = round(initial_error, 3)
        result["aligned_yaw_deg"] = round(yaw_after_probe, 3) if yaw_after_probe is not None else None
        result["alignment_success"] = True
        result["ready_for_execute_plan"] = True
        result["reason"] = "ALREADY_ALIGNED"
        return _finalize(result), trace

    # 5) 회전 필요: gps_probe 에서 IMU 요는 회전 피드백으로 필수.
    # A turn is required: IMU yaw is mandatory as turn feedback for gps_probe.
    if yaw_after_probe is None:
        result["reason"] = "IMU_UNAVAILABLE_FOR_ALIGNMENT"
        result["next_recommended_action"] = (
            "Enable IMU telemetry (BMI160) or use --strategy user_confirmed to point the rover manually."
        )
        return _finalize(result), trace

    turn_direction = str(decision["turn_direction"])
    turn_b_cmd = float(decision["turn_b_cmd"])
    result["turn_direction"] = turn_direction
    result["turn_b_cmd"] = round(turn_b_cmd, 3)

    turn = _execute_alignment_turn(
        handle,
        turn_b_cmd=turn_b_cmd,
        yaw_turn_start=float(yaw_after_probe),
        initial_heading_error_deg=initial_error,
        heading_tolerance_deg=float(heading_tolerance_deg),
        max_turn_duration_s=float(max_turn_duration_s),
        update_hz=float(update_hz),
        ttl_ms=int(ttl_ms),
        chunk_ms=int(chunk_ms),
        event_timeout_s=float(event_timeout_s),
        raw_lines=raw_lines,
        trace=trace,
        verbose_raw=verbose_raw,
    )
    final_error = float(turn["final_heading_error_deg"])
    result["final_heading_error_deg"] = round(final_error, 3)
    result["turn_duration_ms"] = int(turn["turn_duration_ms"])
    result["aligned_yaw_deg"] = round(float(turn["yaw_final"]), 3)
    success = within_tolerance(final_error, heading_tolerance_deg) and not turn["rejected"]
    result["alignment_success"] = success
    result["ready_for_execute_plan"] = success
    if turn["rejected"]:
        result["reason"] = "ALIGN_TURN_REJECTED"
    elif success:
        result["reason"] = "ALIGNED"
    elif turn["timed_out"]:
        result["reason"] = "ALIGN_TURN_TIMEOUT_BEFORE_TOLERANCE"
    else:
        result["reason"] = "ALIGN_TURN_INCOMPLETE"
    return _finalize(result), trace
