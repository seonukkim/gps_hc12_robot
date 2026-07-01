"""대화형 시각/IMU 보조 모션 튜닝 헬퍼.
Interactive visual/IMU-assisted motion tuning helpers.

목적/역할 (Purpose):
    로버의 각 기본 동작(전진/후진/좌·우/좌·우 90° 회전)에 대한 A/B/ms 명령을
    현장에서 조금씩 조정해 "승인된 캘리브레이션"으로 저장하는 순수 헬퍼 모음.
    운영자 피드백(약함/강함/좌/우 등)이나 IMU 요(yaw) 델타를 받아 다음 후보를
    산출한다. 시리얼 통신 자체는 하지 않는다.

    Pure helpers to hand-tune the A/B/ms command for each primitive
    (forward/backward/left/right/turn-*-90) and persist an approved calibration.
    Given operator feedback or an IMU yaw delta, they compute the next
    candidate. No serial I/O here.

시스템 내 위치 (Where it sits):
    CLI 의 ``tune-motion`` 모드가 이 헬퍼들을 **이미 확인된 USB 경계-펄스
    실행기**에 연결한다. :mod:`geometry`(clamp/wrap 등 leaf 수학),
    :mod:`calibration`(경로 상수), :mod:`telemetry`(IMU/모터 트레이스 파싱)를
    import 한다. 순수 모듈이라 단위 테스트가 쉽다.

    The ``tune-motion`` CLI wires these to the already-confirmed USB
    bounded-pulse executor. Imports :mod:`geometry`, :mod:`calibration`,
    :mod:`telemetry`. Deliberately pure => easily unit-tested.

핵심 개념·불변식 (Key concepts / invariants):
    * 물리 매핑 부호 규약: 전진 A>0, 후진 A<0, 좌/좌회전 B>0, 우/우회전 B<0.
      :func:`clamp_candidate` 와 :func:`validate_manual_calibration_entry` 가 이를
      강제한다 — 부호를 뒤집으면 로버가 반대로 움직인다(함정).
    * 명령은 항상 경계 안: |A|,|B| ≤ 0.35, ms ∈ [100, 3000].
    * 저장되는 모든 캘리브레이션 dict 는 ``ready_for_full_path_following=False``.

    Sign convention: forward A>0, backward A<0, left/turn-left B>0,
    right/turn-right B<0 (enforced by clamp/validate; flipping a sign reverses
    the rover). Commands stay bounded (|A|,|B| ≤ 0.35, ms in [100, 3000]). Every
    saved dict carries ``ready_for_full_path_following=False``.

리팩토링 노트 (Refactoring notes):
    프리미티브 이름은 여러 표기(``turn_left_90``/``turn-left-90`` 등)가 섞이므로
    항상 :func:`normalize_primitive` 로 정규화한 뒤 비교할 것. 캘리브레이션 파일
    키는 :data:`CALIBRATION_KEYS` 를 단일 출처로 삼는다.

    Primitive names come in several spellings; always run
    :func:`normalize_primitive` before comparing. :data:`CALIBRATION_KEYS` is the
    single source of truth for on-disk calibration keys.
"""
from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Sequence

from tools.physical_path_planning import calibration, geometry, telemetry

# ── 경계값·초기 후보·별칭·프리셋 테이블 / Bounds, seed candidates, aliases, presets ──
# 명령 경계(모터 안전 한계)와 90° 회전 목표/허용오차. 튜닝은 이 안에서만 움직인다.
# Command bounds (motor safety) and 90-deg turn target/tolerance.
MAX_ABS_A = 0.35
MAX_ABS_B = 0.35
MAX_MS = 3000
MIN_MS = 100
TURN_TARGET_ANGLE_DEG = 90.0
TURN_ANGLE_TOLERANCE_DEG = 10.0

# 각 프리미티브의 튜닝 시작점(부호 규약 준수). 조정은 이 값에서 출발한다.
# Seed candidate per primitive (obeys the sign convention).
INITIAL_CANDIDATES: dict[str, dict[str, object]] = {
    "forward": {"primitive": "forward", "a": 0.30, "b": 0.0, "ms": 800},
    "backward": {"primitive": "backward", "a": -0.08, "b": 0.0, "ms": 300},
    "left": {"primitive": "left", "a": 0.0, "b": 0.26, "ms": 700},
    "right": {"primitive": "right", "a": 0.0, "b": -0.08, "ms": 250},
    "turn-left-90": {
        "primitive": "turn-left-90",
        "a": 0.0,
        "b": 0.24,
        "ms": 2200,
        "target_angle_deg": TURN_TARGET_ANGLE_DEG,
        "angle_tolerance_deg": TURN_ANGLE_TOLERANCE_DEG,
    },
    "turn-right-90": {
        "primitive": "turn-right-90",
        "a": 0.0,
        "b": -0.12,
        "ms": 2200,
        "target_angle_deg": TURN_TARGET_ANGLE_DEG,
        "angle_tolerance_deg": TURN_ANGLE_TOLERANCE_DEG,
    },
}

ALIASES = {
    "turn_left": "left",
    "turn_right": "right",
    "turn_left_90": "turn-left-90",
    "turn-right_90": "turn-right-90",
    "turn_right_90": "turn-right-90",
}

CALIBRATION_KEYS = {
    "forward": "forward",
    "backward": "backward",
    "left": "left",
    "right": "right",
    "turn-left-90": "turn_left_90",
    "turn-right-90": "turn_right_90",
}

MANUAL_CALIBRATION_PRESETS: dict[str, dict[str, dict[str, object]]] = {
    "field_manual_high_except_soft_right": {
        "forward": {
            "a": 0.30,
            "b": 0.0,
            "ms": 1000,
            "approved_by_user": True,
            "source": "manual_high_preset",
        },
        "backward": {
            "a": -0.08,
            "b": 0.0,
            "ms": 350,
            "approved_by_user": True,
            "source": "manual_high_preset",
        },
        "turn_left_90": {
            "a": 0.0,
            "b": 0.26,
            "ms": 2400,
            "target_angle_deg": 90,
            "approved_by_user": True,
            "source": "manual_high_preset",
        },
        "turn_right_90": {
            "a": 0.0,
            "b": -0.08,
            "ms": 1000,
            "target_angle_deg": 90,
            "approved_by_user": True,
            "source": "manual_soft_right_preset",
        },
    },
}


# ── 이름 정규화 & 키 매핑 / Name normalization & key mapping ──


def normalize_primitive(name: str) -> str:
    """프리미티브 이름을 정규 형태로 / Normalize a primitive name (handles aliases, dashes)."""
    normalized = name.strip().lower().replace("_", "-")
    return ALIASES.get(normalized, normalized)


def initial_candidate(primitive: str) -> dict[str, object]:
    """프리미티브의 초기 후보 복사본 / Fresh copy of the seed candidate; raise if unknown."""
    key = normalize_primitive(primitive)
    if key not in INITIAL_CANDIDATES:
        raise ValueError(f"unsupported tune-motion primitive: {primitive}")
    return dict(INITIAL_CANDIDATES[key])


def calibration_key_for_primitive(primitive: str) -> str:
    """프리미티브 → 캘리브레이션 파일 키 / Map a primitive to its on-disk calibration key."""
    key = normalize_primitive(primitive)
    if key not in CALIBRATION_KEYS:
        raise ValueError(f"unsupported motion calibration primitive: {primitive}")
    return CALIBRATION_KEYS[key]


def primitive_for_calibration_key(key: str) -> str:
    """캘리브레이션 키 → 프리미티브(역매핑) / Reverse-map a calibration key to a primitive name."""
    normalized = key.strip().lower()
    for primitive, calibration_key in CALIBRATION_KEYS.items():
        if normalized == calibration_key:
            return primitive
    return normalize_primitive(normalized)


# ── 수동 캘리브레이션 항목/프리셋 검증·저장 / Manual calibration validate & persist ──


def validate_manual_calibration_entry(primitive: str, entry: dict[str, object]) -> None:
    """오버라이드 기록 전 A/B 부호·경계 검증 / Validate A/B signs+bounds before writing overrides.

    물리 매핑 규약을 강제한다: forward A>0, backward A<0, left/turn-left B>0,
    right/turn-right B<0, 0<ms≤MAX_MS, |A|,|B|≤경계. 위반 시 ``ValueError``.
    부수효과 없음(검증만). Raises on any sign/bound violation.
    """
    normalized = normalize_primitive(primitive_for_calibration_key(primitive))
    a_cmd = float(entry["a"])
    b_cmd = float(entry["b"])
    ms = int(entry["ms"])
    if ms <= 0 or ms > MAX_MS:
        raise ValueError(f"{primitive} ms must be > 0 and <= {MAX_MS}")
    if abs(a_cmd) > MAX_ABS_A or abs(b_cmd) > MAX_ABS_B:
        raise ValueError(f"{primitive} exceeds max |A|={MAX_ABS_A} or |B|={MAX_ABS_B}")
    if normalized == "forward" and a_cmd <= 0.0:
        raise ValueError("forward calibration requires A > 0")
    if normalized == "backward" and a_cmd >= 0.0:
        raise ValueError("backward calibration requires A < 0")
    if normalized in {"left", "turn-left-90"} and b_cmd <= 0.0:
        raise ValueError("left/turn-left calibration requires B > 0")
    if normalized in {"right", "turn-right-90"} and b_cmd >= 0.0:
        raise ValueError("right/turn-right calibration requires B < 0")


def manual_calibration_entry(
    primitive: str,
    *,
    a: float,
    b: float,
    ms: int,
    source: str,
    target_angle_deg: float | None = None,
) -> dict[str, object]:
    """수동 값으로 검증된 캘리브레이션 항목 dict 구성 / Build a validated manual calibration entry.

    A/B 를 소수 3자리로 반올림하고 90° 회전이면 ``target_angle_deg`` 를 채운 뒤,
    :func:`validate_manual_calibration_entry` 로 부호/경계를 확인해 반환한다.
    Rounds A/B, adds ``target_angle_deg`` for turns, then validates.
    """
    normalized = normalize_primitive(primitive)
    entry: dict[str, object] = {
        "a": round(float(a), 3),
        "b": round(float(b), 3),
        "ms": int(ms),
        "approved_by_user": True,
        "source": source,
    }
    if normalized in {"turn-left-90", "turn-right-90"}:
        entry["target_angle_deg"] = float(target_angle_deg if target_angle_deg is not None else TURN_TARGET_ANGLE_DEG)
    elif target_angle_deg is not None:
        entry["target_angle_deg"] = float(target_angle_deg)
    validate_manual_calibration_entry(normalized, entry)
    return entry


def manual_calibration_preset(name: str) -> dict[str, dict[str, object]]:
    """이름으로 사전 정의 프리셋을 검증된 복사본으로 반환 / Named preset as a validated deep copy.

    미지의 이름이면 ``ValueError``. 원본이 변형되지 않도록 항목마다 복사하고 각
    항목을 검증한다. Raises on unknown name; copies+validates each entry.
    """
    if name not in MANUAL_CALIBRATION_PRESETS:
        raise ValueError(f"unknown motion calibration preset: {name}")
    preset = {
        key: dict(value)
        for key, value in MANUAL_CALIBRATION_PRESETS[name].items()
    }
    for key, entry in preset.items():
        validate_manual_calibration_entry(key, entry)
    return preset


def apply_manual_calibration_updates(
    path: Path,
    updates: dict[str, dict[str, object]],
    *,
    timestamp: str | None = None,
) -> tuple[dict[str, object], Path | None]:
    """검증된 수동 항목들을 캘리브레이션 파일에 병합 저장 / Merge validated manual entries into the file.

    기존 파일을 먼저 백업하고, 각 항목을 검증한 뒤 키별로 병합해 정렬-JSON 으로
    다시 쓴다. 반환: ``(병합된 data, 백업 경로 또는 None)``. 부수효과: 디렉터리
    생성 · 백업 · 파일 쓰기.
    Backs up, validates+merges each entry, rewrites sorted JSON. Returns
    ``(data, backup_path)``. Side effects: mkdir, backup, file write.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    backup_path = backup_calibration(path, timestamp=timestamp)
    if path.exists():
        loaded = json.loads(path.read_text(encoding="utf-8"))
        data = loaded if isinstance(loaded, dict) else {}
    else:
        data = {}
    for key, entry in updates.items():
        validate_manual_calibration_entry(key, entry)
        data[key] = dict(entry)
    # 부분 튜닝 결과는 절대 "완전 경로추종 준비 완료"로 표시하지 않는다(안전 불변식).
    # Partial tuning must never claim full-path readiness (safety invariant).
    data["ready_for_full_path_following"] = False
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return data, backup_path


# ── 후보 조정 루프 (피드백/IMU → 다음 후보) / Candidate-adjust loop (feedback/IMU → next) ──


def clamp_candidate(candidate: dict[str, object]) -> dict[str, object]:
    """후보를 경계+부호 규약으로 정규화 / Clamp a candidate to bounds and enforce sign per primitive.

    |A|,|B| 를 경계로 자르고 프리미티브별 부호(전진 A>0, 후진 A<0, 좌 B>0, 우
    B<0)를 강제하며 ms 를 [MIN_MS, MAX_MS] 로 제한한다. 원본은 변형하지 않는다.
    Bounds A/B, forces the per-primitive sign, clamps ms; returns a new dict.
    """
    out = dict(candidate)
    primitive = str(out["primitive"])
    a = geometry.clamp(float(out["a"]), -MAX_ABS_A, MAX_ABS_A)
    b = geometry.clamp(float(out["b"]), -MAX_ABS_B, MAX_ABS_B)
    if primitive == "forward":
        a = abs(a)
    elif primitive == "backward":
        a = -abs(a)
    elif primitive in {"left", "turn-left-90"}:
        b = abs(b)
    elif primitive in {"right", "turn-right-90"}:
        b = -abs(b)
    out["a"] = round(a, 3)
    out["b"] = round(b, 3)
    out["ms"] = int(geometry.clamp(float(out["ms"]), MIN_MS, MAX_MS))
    return out


def yaw_delta_from_rows(rows: Sequence[dict[str, str]]) -> float | None:
    """트레이스 행들에서 IMU 요(yaw) 순변화량 / Net IMU yaw delta across trace rows (or None).

    첫·마지막 유효 상대 요를 래핑 차분해 실제 회전량을 추정한다. 유효 값이 2개
    미만이면 ``None``. 회전 튜닝의 강약 판정에 쓰인다.
    Wrapped difference of first/last valid relative yaw; None if <2 samples.
    """
    yaws = [
        yaw for yaw in (telemetry.imu_relative_yaw_deg(row) for row in rows)
        if yaw is not None
    ]
    if len(yaws) < 2:
        return None
    return round(geometry.wrap_deg(yaws[-1] - yaws[0]), 3)


def adjust_candidate(
    candidate: dict[str, object],
    feedback: str,
    *,
    yaw_delta_deg: float | None = None,
) -> dict[str, object]:
    """운영자 피드백/IMU 로부터 다음 경계 후보 산출 / Next bounded candidate from feedback/IMU.

    ``retry``/``approve``/``good`` 은 현 후보를 그대로(clamp만) 돌려준다. 90° 회전은
    ``yaw_delta_deg`` 가 있으면 목표±허용오차와 비교해 약함/강함으로 자동 판정한다.
    조정 전략: 먼저 ms(지속시간)를 키우거나 줄이고, ms 가 상한을 넘어서야 A/B
    크기를 조금 올린다 — 즉 세기보다 시간을 먼저 조절(부드러운 수렴).
    항상 :func:`clamp_candidate` 를 거쳐 경계·부호를 보장한다.

    ``retry``/``approve``/``good`` return the current candidate (clamped only).
    For 90-deg turns, a supplied ``yaw_delta_deg`` auto-classifies weak/strong vs
    target±tolerance. Strategy: adjust ms first, only bump |A|/|B| once ms hits
    its cap (prefer time over magnitude). Always clamped before return.
    """
    feedback = feedback.strip().lower()
    out = dict(candidate)
    primitive = str(out["primitive"])
    if feedback in {"retry", "approve"}:
        return clamp_candidate(out)

    target = float(out.get("target_angle_deg", TURN_TARGET_ANGLE_DEG))
    tolerance = float(out.get("angle_tolerance_deg", TURN_ANGLE_TOLERANCE_DEG))
    # 회전은 IMU 요 델타가 있으면 목표±허용오차로 약함/강함을 자동 판정.
    # For turns, auto-classify weak/strong from the measured yaw delta.
    if primitive in {"turn-left-90", "turn-right-90"} and yaw_delta_deg is not None:
        yaw_abs = abs(float(yaw_delta_deg))
        if yaw_abs < target - tolerance:
            feedback = "weak"
        elif yaw_abs > target + tolerance:
            feedback = "strong"
        else:
            return clamp_candidate(out)
    elif feedback == "good":
        return clamp_candidate(out)

    if primitive == "forward":
        if feedback in {"weak", "too_short", "none"}:
            out["ms"] = int(out["ms"]) + 100
            if int(out["ms"]) > MAX_MS:
                out["a"] = float(out["a"]) + 0.02
        elif feedback in {"strong", "too_long"}:
            out["ms"] = int(out["ms"]) - 100
        elif feedback == "left":
            out["b"] = float(out["b"]) - 0.02
        elif feedback == "right":
            out["b"] = float(out["b"]) + 0.02
    elif primitive == "backward":
        if feedback in {"weak", "too_short", "none"}:
            out["ms"] = int(out["ms"]) + 50
            if int(out["ms"]) > MAX_MS:
                out["a"] = float(out["a"]) - 0.01
        elif feedback in {"strong", "too_long"}:
            out["ms"] = int(out["ms"]) - 50
        elif feedback == "left":
            out["b"] = float(out["b"]) + 0.02
        elif feedback == "right":
            out["b"] = float(out["b"]) - 0.02
    elif primitive in {"left", "right", "turn-left-90", "turn-right-90"}:
        sign = 1.0 if primitive in {"left", "turn-left-90"} else -1.0
        if feedback in {"weak", "too_short", "none"}:
            out["ms"] = int(out["ms"]) + (150 if "90" in primitive else 100)
            if int(out["ms"]) > MAX_MS:
                out["b"] = float(out["b"]) + sign * 0.02
        elif feedback in {"strong", "too_long"}:
            out["ms"] = int(out["ms"]) - (150 if "90" in primitive else 100)
    return clamp_candidate(out)


def approved_calibration_entry(
    candidate: dict[str, object],
    *,
    yaw_delta_deg: float | None = None,
    heading_drift_deg: float | None = None,
) -> dict[str, object]:
    """승인된 후보를 저장용 캘리브레이션 항목으로 / Approved candidate -> persisted calibration entry.

    A/B/ms 와 승인 플래그·출처를 담고, 프리미티브 종류에 따라 진단 필드를 덧붙인다:
    좌/우엔 마지막 IMU 요 델타, 전/후진엔 헤딩 드리프트, 90° 회전엔 목표각+요 델타.
    Adds per-primitive diagnostics (yaw delta / heading drift / target angle).
    """
    primitive = str(candidate["primitive"])
    entry: dict[str, object] = {
        "a": round(float(candidate["a"]), 3),
        "b": round(float(candidate["b"]), 3),
        "ms": int(candidate["ms"]),
        "approved_by_user": True,
        "source": "interactive_visual_tuning",
    }
    if primitive in {"left", "right"}:
        entry["last_imu_yaw_delta_deg"] = yaw_delta_deg
    if primitive in {"forward", "backward"}:
        entry["heading_drift_deg"] = heading_drift_deg
    if primitive in {"turn-left-90", "turn-right-90"}:
        entry["target_angle_deg"] = float(candidate.get("target_angle_deg", TURN_TARGET_ANGLE_DEG))
        entry["last_imu_yaw_delta_deg"] = yaw_delta_deg
    return entry


def opposite_sign_transient(primitive: str, rows: Sequence[dict[str, str]]) -> bool:
    """전/후진 시도에서 모터 트레이스 부호 반전 감지 / Detect a motor-trace sign reversal (fwd/back).

    기대 부호(전진 +, 후진 -)와 반대되는 물리 A 명령 또는 좌/우 평균에서 유도한 A
    가 한 번이라도 나타나면 ``True``. 급출발/역방향 트랜지언트를 잡아 튜닝 품질을
    경고하는 용도. 전/후진 외 프리미티브는 항상 ``False``.
    True if any commanded/derived A opposes the expected forward/backward sign.
    """
    normalized = normalize_primitive(primitive)
    if normalized not in {"forward", "backward"}:
        return False
    expected_sign = 1.0 if normalized == "forward" else -1.0
    for row in rows:
        if telemetry._parse_bool(row.get("motor_write_called")) is not True:
            continue
        physical_a = telemetry._optional_float(row.get("physical_a_cmd"))
        if physical_a is not None and physical_a * expected_sign < -1e-4:
            return True
        final_left = telemetry._optional_float(row.get("final_left_cmd"))
        final_right = telemetry._optional_float(row.get("final_right_cmd"))
        if final_left is not None and final_right is not None:
            inferred_a = (final_left + final_right) * 0.5
            if inferred_a * expected_sign < -1e-4:
                return True
    return False


# ── 승인·경로·백업·리셋 (파일 I/O) / Persist, path, backup, reset (file I/O) ──


def save_approved_calibration(
    path: Path,
    candidate: dict[str, object],
    *,
    yaw_delta_deg: float | None = None,
    heading_drift_deg: float | None = None,
) -> dict[str, object]:
    """승인된 한 프리미티브를 캘리브레이션 파일에 저장 / Save one approved primitive to the file.

    기존 파일을 읽어 해당 프리미티브 키만 갱신하고 정렬-JSON 으로 다시 쓴다.
    :func:`apply_manual_calibration_updates` 와 달리 백업은 하지 않는다.
    반환: 병합된 data dict. 부수효과: 디렉터리 생성 · 파일 쓰기.
    Updates just this primitive's key and rewrites sorted JSON (no backup).
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        loaded = json.loads(path.read_text(encoding="utf-8"))
        data = loaded if isinstance(loaded, dict) else {}
    else:
        data = {}
    primitive = str(candidate["primitive"])
    data[CALIBRATION_KEYS[primitive]] = approved_calibration_entry(
        candidate,
        yaw_delta_deg=yaw_delta_deg,
        heading_drift_deg=heading_drift_deg,
    )
    data["ready_for_full_path_following"] = False
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return data


def motion_calibration_path(path: str | None = None) -> Path:
    """모션 캘리브레이션 파일 경로 해석 / Resolve the motion calibration path (explicit or default)."""
    return Path(path) if path else calibration.DEFAULT_MOTION_CALIBRATION


def backup_calibration(path: Path, *, timestamp: str | None = None) -> Path | None:
    """기존 캘리브레이션 JSON 을 타임스탬프 사본으로 백업 / Back up calibration JSON to a timestamped sibling.

    백업 경로를 반환하며, 백업할 파일이 없으면 ``None``. 타임스탬프는 주입 가능해
    호출자(및 테스트)가 결정적 파일명을 얻는다.
    Returns the backup path, or ``None`` when there is nothing to back up. The
    timestamp is injectable so callers (and tests) get deterministic names.
    """
    if not path.exists():
        return None
    stamp = timestamp or time.strftime("%Y%m%d_%H%M%S")
    backup_path = path.with_name(f"{path.stem}.backup_{stamp}{path.suffix}")
    backup_path.write_text(path.read_text(encoding="utf-8"), encoding="utf-8")
    return backup_path


def reset_calibration(path: Path, *, timestamp: str | None = None) -> tuple[Path | None, bool]:
    """전체 재캘리브레이션 전 백업 후 파일 삭제 / Back up then delete before a full recalibration.

    반환: ``(backup_path, removed)`` — 이전 파일이 없으면 ``backup_path`` 는
    ``None``, 실제로 삭제됐을 때만 ``removed`` 가 ``True``. 이후 승인되는
    ``tune-motion`` 값은 깨끗한 상태에서 시작하고, 이전 캘리브레이션은
    타임스탬프 백업으로 보존된다.
    Returns ``(backup_path, removed)``: ``backup_path`` is ``None`` when no prior
    file existed, ``removed`` is ``True`` only when an existing file was deleted.
    Approved ``tune-motion`` values written afterward start from a clean slate
    while the previous calibration is preserved as a timestamped backup.
    """
    backup_path = backup_calibration(path, timestamp=timestamp)
    removed = False
    if path.exists():
        path.unlink()
        removed = True
    return backup_path, removed
