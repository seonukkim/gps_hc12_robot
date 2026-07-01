"""``tools.physical_path_planning.alignment`` 계약 테스트 — 순수 헬퍼 + 시리얼 정렬 루틴.

목적/역할: 초기 헤딩 정렬(alignment)의 두 부분을 잠근다. (1) 각도/거리/부호를 다루는
순수 헬퍼들, (2) 시리얼과 대화하는 ``align_heading`` 루틴(스킵/사용자확인/gps_probe 전략).

시스템 내 위치: alignment 는 CLI 의 run/auto-relative-run 이 모션 시작 전 로버 헤딩을
목표 헤딩에 맞추는 단계. 상위 게이트는 여기 반환하는 result dict 의 reason/success 키에 의존.

핵심 개념·불변식(테스트 하네스):
  - ``TimedFakeSerial`` 은 각 라인을 지정된 벽시계 오프셋 이후에만 readline() 으로 노출하고
    그 전에는 b"" 를 준다. 이는 executor/alignment 의 데드라인-바운드 읽기 루프를 모사한다.
  - 그래서 probe 대기가 자기 ``probe_duration_s`` 를 다 쓰는 동안, post-probe GPS 읽기용
    큐를 과소비하면 안 된다(타이밍 불변식).
  - ``ready_for_full_path_following`` 은 모든 경로에서 False.

Contract tests for ``tools.physical_path_planning.alignment``. Pure helpers are
tested directly. The serial-facing ``align_heading`` routine is exercised with a
*time-scheduled* fake serial: ``readline()`` yields each line only after its
wall-clock offset has elapsed (``b""`` before then). This matches the
deadline-bound read loops in ``executor``/``alignment`` -- a probe wait that runs
for its full ``probe_duration_s`` must not over-consume the queue intended for
the post-probe GPS read.
"""
from __future__ import annotations

import time

import pytest

from tools.physical_path_planning import alignment, geometry

START_LAT, START_LON = 37.5665000, 126.9780000


# ── 픽스처·헬퍼 / Fixtures & helpers ──────────────────────────────────────────


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
    """벽시계 스케줄 기반 시리얼 대역: 각 라인을 지정 오프셋 이후에만 readline() 으로 노출.

    데드라인-바운드 읽기 루프를 재현해 타이밍 불변식을 시험한다.
    Serial fake whose readline() reveals each scheduled line only after its
    wall-clock offset has elapsed, returning b"" before then."""

    def __init__(self, schedule: list[tuple[float, str]]):
        # (오프셋초, 텍스트) 목록을 오프셋 오름차순으로 정렬해 소비 인덱스로 순회.
        # Sort (offset_s, text) ascending; walk it with a consume index.
        self._schedule = sorted(((float(o), str(t)) for o, t in schedule), key=lambda x: x[0])
        self._idx = 0
        self._start: float | None = None
        self.written: list[str] = []

    def _elapsed(self) -> float:
        """첫 접근 시각을 기준으로 한 경과초(지연 시작). / Elapsed seconds since first access (lazy start)."""
        if self._start is None:
            self._start = time.monotonic()
        return time.monotonic() - self._start

    def readline(self) -> bytes:
        """오프셋이 도달한 다음 스케줄 라인만 반환, 아니면 b"". / Next due scheduled line, else b""."""
        elapsed = self._elapsed()
        if self._idx < len(self._schedule):
            offset, text = self._schedule[self._idx]
            if elapsed >= offset:
                self._idx += 1
                return (text + "\n").encode("ascii")
        return b""

    def write(self, data: object) -> None:
        """보낸 명령을 관측용으로 기록. / Record the outgoing command for inspection."""
        if isinstance(data, (bytes, bytearray)):
            self.written.append(bytes(data).decode("ascii", errors="replace"))
        else:
            self.written.append(str(data))

    def flush(self) -> None:  # pragma: no cover - trivial
        """no-op. / No-op."""
        pass


# gps_probe 정렬 공용 인자. 루프가 벽시계에 묶여 있어 지속시간을 아주 짧게 유지한다.
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


# ── 순수 헬퍼 / Pure helpers ──────────────────────────────────────────────────


def test_first_segment_target_heading_returns_first():
    """첫 세그먼트의 target_heading_deg 를 반환. / Returns the first segment's target heading."""
    segments = [{"target_heading_deg": 42.0}, {"target_heading_deg": 7.0}]
    assert alignment.first_segment_target_heading(segments) == 42.0


def test_first_segment_target_heading_empty_raises():
    """세그먼트가 비면 ValueError. / Empty segments raise ValueError."""
    with pytest.raises(ValueError):
        alignment.first_segment_target_heading([])


def test_estimate_heading_north_is_90():
    """정북 변위는 헤딩 ~90°(ENU 규약), 거리도 근사 일치. / Due-north displacement => heading ~90deg."""
    end_lat, end_lon = _displaced(0.0, 0.6)
    heading, dist = alignment.estimate_heading_deg(START_LAT, START_LON, end_lat, end_lon)
    assert abs(heading - 90.0) < 1.0
    assert dist == pytest.approx(0.6, abs=0.05)


def test_estimate_heading_east_is_0():
    """정동 변위는 헤딩 ~0°. / Due-east displacement => heading ~0deg."""
    end_lat, end_lon = _displaced(0.6, 0.0)
    heading, dist = alignment.estimate_heading_deg(START_LAT, START_LON, end_lat, end_lon)
    assert abs(heading - 0.0) < 1.0
    assert dist == pytest.approx(0.6, abs=0.05)


def test_shortest_heading_error_wraps_across_180():
    """헤딩 오차는 ±180° 경계를 감싸 최단 부호값으로 계산. / Heading error wraps across ±180 to shortest signed."""
    assert alignment.shortest_heading_error_deg(170.0, -170.0) == pytest.approx(-20.0)
    assert alignment.shortest_heading_error_deg(10.0, 350.0) == pytest.approx(20.0)


def test_select_turn_direction_from_error_sign():
    """오차 부호로 회전 방향 선택: +는 left, -/0 은 right(0 은 right 로 규약). / Sign picks turn side."""
    assert alignment.select_turn_direction(5.0) == "left"
    assert alignment.select_turn_direction(-5.0) == "right"
    assert alignment.select_turn_direction(0.0) == "right"


def test_turn_b_command_picks_side():
    """방향에 따라 좌/우 b 명령을 선택. / Picks the left/right b command by direction."""
    assert alignment.turn_b_command("left", 0.24, -0.12) == 0.24
    assert alignment.turn_b_command("right", 0.24, -0.12) == -0.12


def test_remaining_heading_error_reduces_toward_zero():
    """회전량만큼 잔여 오차가 0 을 향해 줄고, 초과 회전은 부호를 뒤집는다.

    Remaining error shrinks toward zero by the rotated amount; overshoot flips sign."""
    assert alignment.remaining_heading_error_deg(30.0, 20.0) == pytest.approx(10.0)
    assert alignment.remaining_heading_error_deg(-30.0, 20.0) == pytest.approx(-10.0)
    # Overshoot: rotating 35 deg past a 30 deg error leaves -5 deg.
    assert alignment.remaining_heading_error_deg(30.0, 35.0) == pytest.approx(-5.0)


def test_within_tolerance_uses_magnitude():
    """허용오차 판정은 부호와 무관하게 절대값 기준. / Tolerance check uses magnitude, ignoring sign."""
    assert alignment.within_tolerance(7.0, 8.0) is True
    assert alignment.within_tolerance(-8.0, 8.0) is True
    assert alignment.within_tolerance(9.0, 8.0) is False


def test_plan_alignment_displacement_too_small():
    """probe 변위가 최소값 미만이면 ok=False + DISPLACEMENT_TOO_SMALL, 회전 불필요.

    Below the minimum probe displacement: ok=False, reason DISPLACEMENT_TOO_SMALL."""
    d = alignment.plan_alignment(
        target_heading_deg=90.0, current_heading_deg=0.0, probe_distance_m=0.1,
        min_probe_distance_m=0.3, heading_tolerance_deg=8.0, turn_b_left=0.24, turn_b_right=-0.12,
    )
    assert d["ok"] is False
    assert d["reason"] == "PROBE_GPS_DISPLACEMENT_TOO_SMALL"
    assert d["needs_turn"] is False


def test_plan_alignment_already_aligned():
    """오차가 허용범위 안이면 ALREADY_ALIGNED, 회전 불필요, 초기오차 노출.

    Within tolerance => ALREADY_ALIGNED, no turn, initial error reported."""
    d = alignment.plan_alignment(
        target_heading_deg=90.0, current_heading_deg=85.0, probe_distance_m=0.5,
        min_probe_distance_m=0.3, heading_tolerance_deg=8.0, turn_b_left=0.24, turn_b_right=-0.12,
    )
    assert d["ok"] is True
    assert d["reason"] == "ALREADY_ALIGNED"
    assert d["needs_turn"] is False
    assert d["initial_heading_error_deg"] == pytest.approx(5.0)


def test_plan_alignment_turn_required_left():
    """목표가 현재보다 반시계(+)면 TURN_REQUIRED + 좌회전 + 좌 b 명령.

    Target CCW of current => TURN_REQUIRED, left turn, left b command."""
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
    """목표가 현재보다 시계(-)면 우회전 + 우 b 명령. / Target CW of current => right turn, right b."""
    d = alignment.plan_alignment(
        target_heading_deg=40.0, current_heading_deg=80.0, probe_distance_m=0.5,
        min_probe_distance_m=0.3, heading_tolerance_deg=8.0, turn_b_left=0.24, turn_b_right=-0.12,
    )
    assert d["turn_direction"] == "right"
    assert d["turn_b_cmd"] == -0.12


# ── 시리얼 대상 align_heading / Serial-facing align_heading ───────────────────


def test_align_skip_no_handle():
    """skip 전략은 핸들 없이도 성공 처리하고 execute-plan 을 허용(정렬 생략).

    strategy=skip succeeds without a handle and greenlights execute-plan."""
    segments = [{"target_heading_deg": 42.0}]
    result, trace = alignment.align_heading(None, segments=segments, strategy="skip", verbose_raw=False)
    assert result["reason"] == "ALIGNMENT_SKIPPED"
    assert result["alignment_success"] is True
    assert result["ready_for_execute_plan"] is True
    assert result["aligned_yaw_deg"] is None
    assert result["ready_for_full_path_following"] is False
    assert trace == []


def test_align_user_confirmed_captures_yaw():
    """user_confirmed: 조작자에게 프롬프트 후, IMU yaw 가 있으면 그 yaw 를 정렬값으로 캡처.

    user_confirmed prompts the operator and captures the IMU yaw as aligned_yaw."""
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
    """user_confirmed 인데 IMU yaw 가 없으면 ..._NO_IMU 로 성공하되 aligned_yaw 는 None.

    user_confirmed without IMU yaw succeeds as ..._NO_IMU with aligned_yaw=None."""
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
    """gps_probe 시작 전 GPS 가 없으면 PROBE_GPS_UNAVAILABLE_BEFORE 로 실패.

    gps_probe with no GPS fix before the probe fails PROBE_GPS_UNAVAILABLE_BEFORE."""
    segments = [{"target_heading_deg": 90.0}]
    fake = TimedFakeSerial([(0.0, _hb(yaw=0.0))])  # heartbeat without GPS
    result, _ = alignment.align_heading(fake, segments=segments, **PROBE_KW)
    assert result["reason"] == "PROBE_GPS_UNAVAILABLE_BEFORE"
    assert result["alignment_success"] is False
    assert result["ready_for_full_path_following"] is False


def test_align_gps_probe_already_aligned():
    """probe 변위로 추정한 헤딩이 이미 목표에 맞으면 ALREADY_ALIGNED(회전 0ms).

    gps_probe: if the probe-estimated heading already matches, ALREADY_ALIGNED (0ms)."""
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
    """probe 이동이 너무 작아 헤딩 추정 불가 -> DISPLACEMENT_TOO_SMALL 로 실패.

    Too little probe motion to estimate heading => PROBE_GPS_DISPLACEMENT_TOO_SMALL."""
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
    """헤딩 추정은 됐지만 회전 폐루프에 IMU yaw 가 없으면 IMU_UNAVAILABLE_FOR_ALIGNMENT.

    Heading estimated but no IMU yaw for the turn loop => IMU_UNAVAILABLE_FOR_ALIGNMENT."""
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
    """전체 성공 경로: probe 로 헤딩 추정 -> 좌회전 폐루프로 오차를 허용범위까지 줄여 ALIGNED.

    turn_b_cmd 는 기본 좌회전 b, 최종 yaw 는 마지막 관측값을 반영.
    Full success path: estimate heading, then a left-turn loop reduces error into
    tolerance => ALIGNED with the default left b and the last observed yaw."""
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
