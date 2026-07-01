"""Physical motion/turn calibration resolver and calibrated-primitive accessors.

Reads the calibration JSON sources (interactive motion, fine motion, turn twitch,
turn angle, smooth connector) in priority order and produces a normalized schema with a
``connector_mode_effective`` plus ``ready_*`` flags. ``ready_for_full_path_following``
is always False.

목적/역할 (KO):
    로버의 물리 이동/회전 "보정(calibration)"을 해석하는 잎(leaf) 모듈이다. 여러
    보정 JSON 소스(대화형 모션, 파인 모션, 회전 트위치, 회전 각도, 스무스 커넥터)를
    우선순위대로 읽어, 전진/후진/좌회전/우회전과 90도 커넥터 프리미티브를 담은
    정규화 스키마 dict 으로 합친다. 결과에는 ``connector_mode_effective`` (실제로
    쓸 커넥터 방식)와 각종 ``ready_*`` 플래그가 들어간다. 어떤 경우에도
    ``ready_for_full_path_following`` 은 항상 False 다(자율 전체 경로 추종 미승인).

시스템 내 위치 (KO):
    표준 라이브러리(json/math/pathlib)만 의존하는 잎 모듈. geometry 가 폴백 보정을
    얻으려 이 모듈을 임포트하고, controller/cli 는 프리미티브 접근자
    (``planner_primitive``/``connector_primitive`` 등)로 실제 명령 값을 뽑는다.
    방향: calibration -> (geometry) -> controller -> cli.

핵심 개념 (KO):
    - 우선순위 병합: "대화형 모션(사용자 승인)" 오버라이드가 있으면 그것을 쓰고,
      없으면 파인/트위치 계산값, 그것도 없으면 내장 안전 기본값으로 폴백한다.
    - source 규약: 폴백에서 나온 프리미티브는 ``source`` 가 ``fallback_known_`` 으로
      시작한다. 이 접두어가 곧 "측정 보정 아님"의 표식이다(아래 완결성 판정에 사용).
    - 커넥터 모드: angle_calibrated > smooth_imu > repeated_pulses 순으로 권장하며,
      가능한 것이 없으면 반복 펄스로 폴백한다.
    - ``target_angle_deg`` 함정: ``turn_*_90`` 키라도 실제로는 훨씬 작은 각(15~45도)
      트위치일 수 있으므로, 키 이름의 "90"을 믿지 말고 측정값을 예산으로 삼아야 한다.

Purpose (EN):
    Leaf module that resolves physical motion/turn calibration. It reads the
    calibration JSON sources in priority order and merges them into one normalized
    schema (forward/backward/turn primitives, 90-degree connectors,
    ``connector_mode_effective``, ``ready_*`` flags); ``ready_for_full_path_following``
    is always False. Depends only on the stdlib; geometry imports it for a fallback
    calibration and controller/cli read command values through the primitive
    accessors. Key rules: an approved interactive override wins, else computed
    fine/twitch values, else built-in safe fallbacks whose ``source`` starts with
    ``fallback_known_`` (that prefix means "not measured"); connector preference is
    angle_calibrated > smooth_imu > repeated_pulses; and a ``turn_*_90`` entry may
    hold a much smaller measured pulse, so budget by ``target_angle_deg``/measured
    yaw, never by the "90" in the key name.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Sequence


# ── 기본 보정 파일 경로 / Default calibration source paths ──
# KO: 우선순위대로 읽는 보정 JSON 소스들의 기본 경로. 경로 문자열의 "stage"+"20..."
#     조각내기는 스테이지 자동 치환 도구가 이 경로를 건드리지 못하게 하는 의도.
DEFAULT_FINE_CALIBRATION = Path("outputs/" + "stage" + "20_physical_ab_probe/calibration/physical_ab_fine_motion_calibration.json")
DEFAULT_TURN_CALIBRATION = Path("outputs/stage23_turn_calibration/calibration/physical_ab_turn_twitch_calibration.json")
DEFAULT_TURN_ANGLE_CALIBRATION = Path("outputs/stage23_turn_calibration/calibration/physical_ab_turn_angle_calibration.json")
DEFAULT_SMOOTH_TURN_CALIBRATION = Path("outputs/stage36_smooth_turn_connector/calibration/smooth_turn_connector_calibration.json")
DEFAULT_MOTION_CALIBRATION = Path("outputs/physical_path_planning/calibration/motion_calibration.json")

DEFAULT_FORWARD_A_CMD = 0.30
DEFAULT_FORWARD_MS = 800
DEFAULT_BACKWARD_A_CMD = -0.08
DEFAULT_BACKWARD_MS = 300
DEFAULT_TURN_LEFT_B_CMD = 0.26
DEFAULT_TURN_LEFT_MS = 700
DEFAULT_TURN_RIGHT_B_CMD = -0.08
DEFAULT_TURN_RIGHT_MS = 250
DEFAULT_LEFT_FIXED_PULSES = 12
DEFAULT_RIGHT_FIXED_PULSES = 12


# ── 강제 변환·조회 헬퍼 / Coercion + lookup helpers ──


def _parse_bool(value: object, default: bool = False) -> bool:
    """Tri-state bool coercion from calibration JSON text.

    KO: 참/거짓 화이트리스트를 각각 두어, 인식 못 하는 값은 ``default`` 로 둔다
    (telemetry._parse_bool 과 달리 "false" 계열도 명시적으로 False 로 잡는다).
    """
    if value is None:
        return default
    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "y", "ok", "ready"}:
        return True
    if text in {"0", "false", "no", "n", "off", "inactive"}:
        return False
    return default


def _optional_float(value: object) -> float | None:
    """Parse float; empty/NA sentinels, parse failure, non-finite -> ``None``.

    KO: 빈 문자열/NA/NAN/NONE/NULL, 파싱 실패, 비유한 값은 모두 ``None``.
    """
    if value is None:
        return None
    text = str(value).strip()
    if text.upper() in {"", "NA", "NAN", "NONE", "NULL"}:
        return None
    try:
        parsed = float(text)
    except ValueError:
        return None
    return parsed if math.isfinite(parsed) else None


def _optional_int(value: object) -> int | None:
    """Parse int via ``_optional_float`` (truncates) / 실수로 파싱 후 정수 절삭, 실패 시 None."""
    parsed = _optional_float(value)
    return int(parsed) if parsed is not None else None


def _load_json_if_present(path: Path | None) -> tuple[dict[str, object], str | None]:
    """Load a JSON dict if the file exists, returning (data, source_name).

    KO: 파일이 없거나 dict 가 아니면 ``({}, None)``. source_name(파일명)은 나중에
    프리미티브의 ``source`` 로 붙어 "어느 보정 파일에서 왔는지" 추적에 쓰인다.
    """
    if path is None or not path.exists():
        return {}, None
    loaded = json.loads(path.read_text(encoding="utf-8"))
    return (loaded if isinstance(loaded, dict) else {}), path.name


def _first_float(data: dict[str, object], keys: Sequence[str]) -> tuple[float | None, str | None]:
    """First non-zero float among ``keys`` (in order) with the key that matched.

    KO: 키를 순서대로 보며 처음으로 나오는 0 이 아닌 유효 실수와 그 키를 돌려준다.
    0.0 을 건너뛰는 이유: 보정 파일의 0 은 "미측정/미사용"을 뜻하는 관례라서.
    """
    for key in keys:
        parsed = _optional_float(data.get(key))
        if parsed is not None and parsed != 0.0:
            return parsed, key
    return None, None


def _first_int(data: dict[str, object], keys: Sequence[str]) -> tuple[int | None, str | None]:
    """First positive int among ``keys`` (in order) with the key that matched.

    KO: 처음으로 나오는 양의 정수(펄스 길이 ms 등)와 그 키. 0/음수는 무효로 건너뛴다.
    """
    for key in keys:
        parsed = _optional_int(data.get(key))
        if parsed is not None and parsed > 0:
            return parsed, key
    return None, None


def _primitive(
    *,
    a: float,
    b: float,
    ms: int,
    source: str,
) -> dict[str, object]:
    """Build a normalized motion primitive dict {a, b, ms, source}.

    KO: a(전/후진 축)·b(회전 축) 명령을 소수 3자리로 반올림하고 ms/source 를 붙인
    표준 프리미티브 사전을 만든다. ``source`` 는 출처 추적/폴백 판정의 근거.
    """
    return {"a": round(a, 3), "b": round(b, 3), "ms": int(ms), "source": source}


# ── 모션/회전 프리미티브 해석 / Motion & turn primitive resolution ──


def _approved_entry(data: dict[str, object], key: str) -> dict[str, object] | None:
    """Return the entry at ``key`` only if user-approved and fully specified.

    KO: ``approved_by_user`` 가 참이고 a/b/ms 가 모두 있는 항목만 그대로 반환한다.
    승인 안 됐거나 값이 빠지면 ``None`` -- 즉 사용자 승인 오버라이드의 게이트.
    """
    entry = data.get(key)
    if not isinstance(entry, dict) or not _parse_bool(entry.get("approved_by_user")):
        return None
    a = _optional_float(entry.get("a_cmd", entry.get("a")))
    b = _optional_float(entry.get("b_cmd", entry.get("b")))
    ms = _optional_int(entry.get("pulse_ms", entry.get("ms")))
    if a is None or b is None or ms is None:
        return None
    return entry


def _interactive_motion_override(
    data: dict[str, object],
    source_name: str | None,
    key: str,
    fallback: dict[str, object],
) -> dict[str, object]:
    """Prefer a user-approved interactive primitive, else return ``fallback``.

    KO: 대화형(사용자 승인) 보정 항목이 있으면 그것으로 프리미티브를 만들고, 없으면
    미리 계산된 ``fallback`` 을 그대로 쓴다. 방향별로 부호를 강제 정규화한다.
    """
    entry = _approved_entry(data, key)
    if entry is None:
        return fallback
    a = _optional_float(entry.get("a_cmd", entry.get("a")))
    b = _optional_float(entry.get("b_cmd", entry.get("b")))
    ms = _optional_int(entry.get("pulse_ms", entry.get("ms")))
    if a is None or b is None or ms is None:
        return fallback
    # KO: 방향과 부호를 강제로 일치 -- 입력 부호 실수와 무관하게 전진은 +a, 후진은 -a,
    #     좌회전은 +b, 우회전은 -b 가 되도록 한다(안전한 정규화).
    if key == "forward":
        a = abs(a)
    elif key == "backward":
        a = -abs(a)
    elif key == "left":
        b = abs(b)
    elif key == "right":
        b = -abs(b)
    return _primitive(
        a=a,
        b=b,
        ms=ms,
        source=source_name or "interactive_motion_calibration",
    )


def _motion_primitive(
    data: dict[str, object],
    source_name: str | None,
    *,
    direction: str,
) -> dict[str, object]:
    """Compute a forward/backward primitive from fine-motion keys, else safe fallback.

    KO: 파인 모션 보정에서 direction(전진/후진)에 해당하는 a_cmd/pulse_ms 를 우선순위
    키 목록으로 찾는다. 하나도 못 찾으면 내장 안전 기본값을 쓰고 ``source`` 를
    ``fallback_known_...`` 으로 표시한다(=측정 보정 아님의 표식).
    """
    if direction == "forward":
        value, value_key = _first_float(
            data,
            (
                "stage26_forward_a_cmd",
                "stage22_forward_a_cmd",
                "stage21_forward_a_cmd",
                "forward_recommended_crawl_a_cmd",
                "forward_visible_a_cmd",
            ),
        )
        ms, ms_key = _first_int(
            data,
            (
                "stage26_forward_pulse_ms",
                "stage22_forward_pulse_ms",
                "stage21_forward_pulse_ms",
                "forward_recommended_crawl_pulse_ms",
                "forward_visible_pulse_ms",
            ),
        )
        return _primitive(
            a=abs(value) if value is not None else DEFAULT_FORWARD_A_CMD,
            b=0.0,
            ms=ms if ms is not None else DEFAULT_FORWARD_MS,
            source=source_name if source_name and (value_key or ms_key) else "fallback_known_forward",
        )
    value, value_key = _first_float(
        data,
        (
            "stage26_backward_a_cmd",
            "stage21_backward_a_cmd",
            "backward_recommended_crawl_a_cmd",
            "backward_visible_a_cmd",
            "backward_min_a_cmd",
        ),
    )
    ms, ms_key = _first_int(
        data,
        (
            "stage26_backward_pulse_ms",
            "stage21_backward_pulse_ms",
            "backward_recommended_crawl_pulse_ms",
            "backward_visible_pulse_ms",
            "backward_min_pulse_ms",
        ),
    )
    return _primitive(
        a=-abs(value) if value is not None else DEFAULT_BACKWARD_A_CMD,
        b=0.0,
        ms=ms if ms is not None else DEFAULT_BACKWARD_MS,
        source=source_name if source_name and (value_key or ms_key) else "fallback_known_backward",
    )


def _turn_primitive(
    data: dict[str, object],
    source_name: str | None,
    *,
    direction: str,
) -> dict[str, object]:
    """Compute a left/right turn-twitch primitive from turn keys, else safe fallback.

    KO: 회전 트위치 보정에서 direction(좌/우)에 맞는 b_cmd/pulse_ms 를 우선순위 키로
    찾는다. 좌회전은 +b, 우회전은 -b 로 부호를 강제. 없으면 안전 기본값 + fallback source.
    """
    if direction == "left":
        value, value_key = _first_float(
            data,
            ("stage26_turn_left_b_cmd", "turn_left_usable_b_cmd", "turn_left_min_b_cmd", "safe_turn_left_cmd"),
        )
        ms, ms_key = _first_int(
            data,
            ("stage26_turn_left_pulse_ms", "turn_left_usable_pulse_ms", "turn_left_min_pulse_ms", "safe_turn_left_pulse_ms"),
        )
        return _primitive(
            a=0.0,
            b=abs(value) if value is not None else DEFAULT_TURN_LEFT_B_CMD,
            ms=ms if ms is not None else DEFAULT_TURN_LEFT_MS,
            source=source_name if source_name and (value_key or ms_key) else "fallback_known_turn_left",
        )
    value, value_key = _first_float(
        data,
        ("stage26_turn_right_b_cmd", "turn_right_small_b_cmd", "turn_right_min_b_cmd", "safe_turn_right_cmd"),
    )
    ms, ms_key = _first_int(
        data,
        ("stage26_turn_right_pulse_ms", "turn_right_small_pulse_ms", "turn_right_min_pulse_ms", "safe_turn_right_pulse_ms"),
    )
    return _primitive(
        a=0.0,
        b=-abs(value) if value is not None else DEFAULT_TURN_RIGHT_B_CMD,
        ms=ms if ms is not None else DEFAULT_TURN_RIGHT_MS,
        source=source_name if source_name and (value_key or ms_key) else "fallback_known_turn_right",
    )


# ── 각도 기반 / 스무스 커넥터 항목 / Angle-based & smooth connector entries ──


def _angle_turn_entry(data: dict[str, object], source_name: str | None, key: str) -> dict[str, object]:
    """Normalize a turn-angle calibration entry into an ``available`` connector dict.

    KO: 회전-각도 보정 항목을 정규화한다. "사용 가능"의 조건은 ``ready`` 참 AND
    ``visual_confirmation == "yes"`` (사람이 눈으로 확인). 조건 미달이거나 a/b/ms 가
    빠지면 ``available=False`` 로, 왜 안 되는지 힌트 필드와 함께 반환한다.
    """
    entry = data.get(key)
    if not isinstance(entry, dict):
        return {"available": False, "source": source_name or "missing"}
    # KO: 자동화가 실수로 켜지지 않도록 사람 눈 확인("yes")까지 요구하는 이중 게이트.
    ready = _parse_bool(entry.get("ready")) and str(entry.get("visual_confirmation", "")).strip().lower() == "yes"
    a = _optional_float(entry.get("a_cmd", entry.get("a")))
    b = _optional_float(entry.get("b_cmd", entry.get("b")))
    ms = _optional_int(entry.get("pulse_ms", entry.get("ms")))
    yaw = _optional_float(entry.get("imu_yaw_delta_deg"))
    if not ready or a is None or b is None or ms is None:
        return {
            "available": False,
            "visual_confirmation": entry.get("visual_confirmation", "unknown"),
            "ready": _parse_bool(entry.get("ready")),
            "source": source_name or "missing",
        }
    return {
        "available": True,
        "a": round(a, 3),
        "b": round(b, 3),
        "ms": ms,
        "imu_yaw_delta_deg": None if yaw is None else round(yaw, 3),
        "visual_confirmation": str(entry.get("visual_confirmation", "unknown")),
        "source": source_name or "missing",
    }


def _interactive_angle_turn_entry(
    data: dict[str, object],
    source_name: str | None,
    key: str,
    fallback: dict[str, object],
) -> dict[str, object]:
    """Prefer a user-approved 90-degree turn entry, else ``fallback`` (the angle-file entry).

    KO: 대화형(사용자 승인) 90도 회전 항목이 있으면 그것을 정규화해 쓰고, 없으면
    ``fallback`` (보통 회전-각도 파일에서 만든 항목)을 그대로 반환한다. 좌90은 +b,
    우90은 -b 로 부호 강제. target_angle_deg 기본값은 90.
    """
    entry = _approved_entry(data, key)
    if entry is None:
        return fallback
    a = _optional_float(entry.get("a_cmd", entry.get("a")))
    b = _optional_float(entry.get("b_cmd", entry.get("b")))
    ms = _optional_int(entry.get("pulse_ms", entry.get("ms")))
    yaw = _optional_float(entry.get("last_imu_yaw_delta_deg", entry.get("imu_yaw_delta_deg")))
    target = _optional_float(entry.get("target_angle_deg"))
    if a is None or b is None or ms is None:
        return fallback
    if key == "turn_left_90":
        b = abs(b)
    else:
        b = -abs(b)
    return {
        "available": True,
        "a": round(a, 3),
        "b": round(b, 3),
        "ms": ms,
        "target_angle_deg": target if target is not None else 90.0,
        "imu_yaw_delta_deg": None if yaw is None else round(yaw, 3),
        "visual_confirmation": str(entry.get("visual_confirmation", "approved")),
        "ready": True,
        "source": source_name or "interactive_motion_calibration",
    }


def _smooth_turn_entry(data: dict[str, object], source_name: str | None, *, direction: str, allow_uncalibrated: bool) -> dict[str, object]:
    """Normalize a smooth-IMU connector entry (per direction) into a connector dict.

    KO: 스무스(IMU 폐루프) 커넥터 항목을 정규화한다. ``ready_for_smooth_connectors``
    가 참이고 소스가 있으면 사용 가능. 그렇지 않아도 ``allow_uncalibrated`` 면 안전
    기본값으로 사용 가능 처리하고 source 를 ``uncalibrated_smooth_defaults`` 로 표시한다.
    좌는 +b, 우는 -b 로 부호 강제.
    """
    prefix = "smooth_left" if direction == "left" else "smooth_right"
    default_b = 0.20 if direction == "left" else -0.08
    b_value = _optional_float(data.get(f"{prefix}_b_cmd"))
    ms = _optional_int(data.get(f"{prefix}_max_ms"))
    ready = _parse_bool(data.get("ready_for_smooth_connectors")) and source_name is not None
    if not ready and not allow_uncalibrated:
        return {"available": False, "source": source_name or "missing"}
    signed_b = abs(b_value) if b_value is not None else abs(default_b)
    if direction == "right":
        signed_b = -signed_b
    return {
        "available": bool(ready or allow_uncalibrated),
        "a": 0.0,
        "b": round(signed_b, 3),
        "ms": ms if ms is not None else 3000,
        "target_angle_deg": _optional_float(data.get(f"{prefix}_target_angle_deg")) or 90.0,
        "angle_tolerance_deg": _optional_float(data.get(f"{prefix}_angle_tolerance_deg")) or 10.0,
        "source": source_name if ready else "uncalibrated_smooth_defaults",
    }


# ── 최상위 해석기 / Top-level resolver ──


def resolve_physical_calibration(
    *,
    motion_calibration_json: Path | None = None,
    fine_calibration_json: Path | None = DEFAULT_FINE_CALIBRATION,
    turn_calibration_json: Path | None = DEFAULT_TURN_CALIBRATION,
    turn_angle_calibration_json: Path | None = DEFAULT_TURN_ANGLE_CALIBRATION,
    smooth_turn_calibration_json: Path | None = DEFAULT_SMOOTH_TURN_CALIBRATION,
    calibration_mode: str = "auto",
    allow_uncalibrated_smooth: bool = False,
    left_fixed_pulses: int = DEFAULT_LEFT_FIXED_PULSES,
    right_fixed_pulses: int = DEFAULT_RIGHT_FIXED_PULSES,
) -> dict[str, object]:
    """Merge all calibration sources into one normalized calibration schema dict.

    무엇을/왜 (KO): 이 모듈의 메인 진입점. 다섯 개 보정 소스를 읽어 전진/후진/좌우
    회전 프리미티브, 좌/우 90도 커넥터(각도·스무스), 그리고 실제로 쓸 커넥터 방식
    ``connector_mode_effective`` 와 각종 ``ready_*`` 플래그를 담은 dict 을 만든다.
    핵심 인자 (KO): ``calibration_mode`` 는 auto|angle_calibrated|smooth_imu|
    repeated_pulses. auto 면 권장 모드를 자동 선택하고, 특정 모드를 강제했는데 준비가
    안 됐으면 ``RuntimeError`` 를 던진다. 알 수 없는 모드는 ``ValueError``.
    반환 (EN): a schema dict; ``ready_for_full_path_following`` is always False.
    """
    motion, motion_source = _load_json_if_present(motion_calibration_json)
    fine, fine_source = _load_json_if_present(fine_calibration_json)
    turn, turn_source = _load_json_if_present(turn_calibration_json)
    angle, angle_source = _load_json_if_present(turn_angle_calibration_json)
    smooth, smooth_source = _load_json_if_present(smooth_turn_calibration_json)

    left_90 = _interactive_angle_turn_entry(
        motion, motion_source, "turn_left_90", _angle_turn_entry(angle, angle_source, "turn_left_90")
    )
    right_90 = _interactive_angle_turn_entry(
        motion, motion_source, "turn_right_90", _angle_turn_entry(angle, angle_source, "turn_right_90")
    )
    angle_ready = bool(left_90.get("available")) and bool(right_90.get("available"))
    smooth_left = _smooth_turn_entry(smooth, smooth_source, direction="left", allow_uncalibrated=allow_uncalibrated_smooth)
    smooth_right = _smooth_turn_entry(smooth, smooth_source, direction="right", allow_uncalibrated=allow_uncalibrated_smooth)
    smooth_ready = bool(smooth_left.get("available")) and bool(smooth_right.get("available")) and (
        bool(smooth.get("ready_for_smooth_connectors")) or allow_uncalibrated_smooth
    )

    # KO: 커넥터 방식 권장 우선순위 -- 각도 보정 > 스무스(IMU) > 반복 펄스.
    #     반복 펄스는 항상 가능한 "최후의 폴백"이므로 이동을 막지 않는다.
    if angle_ready:
        recommended = "angle_calibrated"
    elif smooth_ready:
        recommended = "smooth_imu"
    else:
        recommended = "repeated_pulses"

    # KO: auto 는 권장을 그대로 채택. 특정 모드를 강제했는데 준비 미달이면 즉시 실패
    #     (조용히 폴백하지 않는다 -- 운영자가 명시적으로 요청한 것이므로).
    if calibration_mode == "auto":
        effective = recommended
    elif calibration_mode == "angle_calibrated":
        if not angle_ready:
            raise RuntimeError("angle-calibrated connector requested but physical_ab_turn_angle_calibration.json is missing or incomplete.")
        effective = "angle_calibrated"
    elif calibration_mode == "smooth_imu":
        if not smooth_ready:
            raise RuntimeError("smooth-imu connector requested but smooth_turn_connector_calibration.json is missing or incomplete.")
        effective = "smooth_imu"
    elif calibration_mode == "repeated_pulses":
        effective = "repeated_pulses"
    else:
        raise ValueError(f"unsupported calibration_mode: {calibration_mode}")

    return {
        "forward": _interactive_motion_override(
            motion, motion_source, "forward", _motion_primitive(fine, fine_source, direction="forward")
        ),
        "backward": _interactive_motion_override(
            motion, motion_source, "backward", _motion_primitive(fine, fine_source, direction="backward")
        ),
        "turn_left": _interactive_motion_override(
            motion, motion_source, "left", _turn_primitive(turn, turn_source, direction="left")
        ),
        "turn_right": _interactive_motion_override(
            motion, motion_source, "right", _turn_primitive(turn, turn_source, direction="right")
        ),
        "turn_left_90": left_90,
        "turn_right_90": right_90,
        "smooth_turn_left_90": smooth_left,
        "smooth_turn_right_90": smooth_right,
        "connector_mode_requested": calibration_mode,
        "connector_mode_recommended": recommended,
        "connector_mode_effective": effective,
        "ready_for_angle_based_connectors": angle_ready,
        "angle_based_connector_available": angle_ready,
        "ready_for_smooth_connectors": smooth_ready,
        "smooth_connector_available": smooth_ready,
        "fallback_to_repeated_pulses": effective == "repeated_pulses",
        "left_fixed_pulses": int(left_fixed_pulses),
        "right_fixed_pulses": int(right_fixed_pulses),
        "calibration_files": {
            "motion": str(motion_calibration_json) if motion_calibration_json else None,
            "fine": str(fine_calibration_json) if fine_calibration_json else None,
            "turn_twitch": str(turn_calibration_json) if turn_calibration_json else None,
            "turn_angle": str(turn_angle_calibration_json) if turn_angle_calibration_json else None,
            "smooth_turn": str(smooth_turn_calibration_json) if smooth_turn_calibration_json else None,
        },
        # KO: 패키지 전역 불변식 -- 자율 전체 경로 추종은 미승인이므로 항상 False.
        "ready_for_full_path_following": False,
    }


# ── 프리미티브 접근자 / Calibrated-primitive accessors ──


def planner_primitive(calibration: dict[str, object], name: str) -> dict[str, object]:
    """Extract one motion primitive as {a_cmd, b_cmd, pulse_ms, calibration_source}.

    KO: 해석된 보정 dict 에서 이름(move_forward/move_backward/turn_left/turn_right
    또는 원 키)으로 프리미티브를 뽑아 실행부가 바로 쓰는 명령 형태로 변환한다.
    항목이 없거나 dict 가 아니면 ``KeyError``.
    """
    key = {
        "move_forward": "forward",
        "move_backward": "backward",
        "turn_left": "turn_left",
        "turn_right": "turn_right",
    }.get(name, name)
    entry = calibration[key]
    if not isinstance(entry, dict):
        raise KeyError(key)
    return {
        "a_cmd": float(entry["a"]),
        "b_cmd": float(entry["b"]),
        "pulse_ms": int(entry["ms"]),
        "calibration_source": str(entry.get("source", "unknown")),
    }


def _is_motion_calibrated(primitive: object) -> bool:
    """A motion primitive is user-calibrated when it is not a known-safe fallback.

    The resolver always returns a usable ``forward``/``backward`` primitive, but
    falls back to the built-in safe values (``source`` prefixed ``fallback_known_``)
    when no real calibration file supplied them. Driving the rover physically should
    use measured calibration, so a fallback source counts as *not calibrated*.

    KO: 프리미티브의 ``source`` 가 ``fallback_known_`` 로 시작하면 "측정 보정 아님"
    으로 보아 False. 즉 실제 측정에서 온 값만 "보정됨"으로 친다.
    """
    if not isinstance(primitive, dict):
        return False
    return not str(primitive.get("source", "")).startswith("fallback_known_")


def plan_requires_backward(segments: Sequence[dict[str, object]] | None) -> bool:
    """True when any planned lane drives backward (serpentine return lanes).

    KO: 계획된 차선 중 ``expected_motion_direction == "backward"`` 가 하나라도 있으면
    True. 뱀형(serpentine) 복귀 차선처럼 후진이 필요한 계획인지 판정한다.
    """
    for segment in segments or []:
        if str(segment.get("expected_motion_direction", "")) == "backward":
            return True
    return False


def calibration_completeness(
    calibration: dict[str, object],
    *,
    segments: Sequence[dict[str, object]] | None = None,
) -> dict[str, object]:
    """Report which motion primitives are calibrated and which the plan requires.

    ``forward`` is always required to drive. ``backward`` is required only when the
    plan contains a backward (serpentine return) lane. ``turn_left_90`` /
    ``turn_right_90`` are reported but never required: a missing turn-angle
    calibration falls back to repeated fixed pulses, so it cannot block motion.

    KO: 어떤 프리미티브가 보정되었고 현재 계획이 무엇을 요구하는지 보고한다. 전진은
    항상 필수, 후진은 후진 차선이 있을 때만 필수, 회전 90은 보고만 하고 필수는 아니다
    (미보정이면 반복 펄스로 폴백하므로 이동을 막지 못한다).
    ``can_run_stop_correct_go`` = 필수 항목이 모두 갖춰졌는가.
    """
    left_90 = calibration.get("turn_left_90")
    right_90 = calibration.get("turn_right_90")
    present_and_approved = {
        "forward": _is_motion_calibrated(calibration.get("forward")),
        "backward": _is_motion_calibrated(calibration.get("backward")),
        "turn_left_90": bool(isinstance(left_90, dict) and left_90.get("available")),
        "turn_right_90": bool(isinstance(right_90, dict) and right_90.get("available")),
    }
    needs_backward = plan_requires_backward(segments)
    required = ["forward"] + (["backward"] if needs_backward else [])
    missing_required = [name for name in required if not present_and_approved[name]]
    return {
        "present_and_approved": present_and_approved,
        "required_for_current_plan": required,
        "missing_required": missing_required,
        "plan_requires_backward": needs_backward,
        "can_run_stop_correct_go": not missing_required,
        "ready_for_full_path_following": False,
    }


def connector_primitive(calibration: dict[str, object], direction: str) -> dict[str, object]:
    """Extract the corner-turn primitive for ``direction`` under the effective mode.

    무엇을/왜 (KO): 현재 유효 커넥터 모드(angle_calibrated/smooth_imu/repeated_pulses)에
    따라 좌/우 코너 회전에 쓸 프리미티브를 골라 명령 형태로 반환한다. 반환 dict 에는
    ``connector_mode`` 와 ``target_angle_deg`` (이 프리미티브 한 펄스의 측정/선언 회전각)
    가 포함된다. 실행부는 이 각을 예산으로 반복 펄스/IMU 피드백을 계산한다.
    함정 (KO): ``turn_*_90`` 키라도 실제로는 작은 트위치일 수 있으니 키 이름의 "90"이
    아니라 아래에서 결정한 ``target_angle_deg`` 를 신뢰해야 한다.
    """
    effective = str(calibration.get("connector_mode_effective", "repeated_pulses"))
    if effective == "angle_calibrated":
        key = "turn_left_90" if direction == "left" else "turn_right_90"
    elif effective == "smooth_imu":
        key = "smooth_turn_left_90" if direction == "left" else "smooth_turn_right_90"
    else:
        key = "turn_left" if direction == "left" else "turn_right"
    entry = calibration[key]
    if not isinstance(entry, dict):
        raise KeyError(key)
    # target_angle_deg is the measured/declared rotation of ONE pulse of this
    # primitive. A turn_*_90 entry may legally hold a much smaller pulse (the
    # operator calibrated a 15-45 degree twitch); the executor must budget
    # repeated pulses / IMU feedback from this value instead of trusting the
    # "90" in the key name. Repeated-pulse twitch entries carry no angle (None).
    target_angle: float | None = None
    if effective in {"angle_calibrated", "smooth_imu"}:
        target_angle = _optional_float(entry.get("target_angle_deg"))
        if target_angle is None:
            # Older turn-angle calibrations carry only the measured IMU yaw
            # delta of one pulse; that measurement beats assuming 90.
            measured = _optional_float(entry.get("imu_yaw_delta_deg"))
            if measured is not None and abs(measured) >= 5.0:
                target_angle = abs(measured)
        if target_angle is None:
            target_angle = 90.0
    return {
        "a_cmd": float(entry["a"]),
        "b_cmd": float(entry["b"]),
        "pulse_ms": int(entry["ms"]),
        "calibration_source": str(entry.get("source", "unknown")),
        "connector_mode": effective,
        "target_angle_deg": target_angle,
    }


TURN_SMALL_PULSE_WARNING = "TURN_CALIBRATION_IS_SMALL_PULSE_NOT_90"
SMALL_PULSE_ANGLE_THRESHOLD_DEG = 60.0


def turn_angle_summary(calibration: dict[str, object]) -> dict[str, object]:
    """Per-direction turn pulse angles plus small-pulse warnings for reports.

    A ``turn_*_90`` entry whose ``target_angle_deg`` is below 60 degrees is a
    small turn pulse, not a one-shot 90 degree turn; connectors then need
    repeated pulses (or IMU feedback) to actually reach the planned corner
    angle, and the warning makes that visible in calibration-check output.

    KO: 좌/우 회전 펄스의 각도와 "작은 펄스" 경고를 리포트용으로 모은다. ``turn_*_90``
    항목의 각이 60도 미만이면 한 방에 90도를 도는 게 아니라 작은 트위치이므로, 커넥터는
    반복 펄스/IMU 피드백이 필요하다는 경고를 붙여 보정 점검 출력에 드러낸다.
    """
    out: dict[str, object] = {}
    warnings: list[str] = []
    for key in ("turn_left_90", "turn_right_90"):
        entry = calibration.get(key)
        angle: float | None = None
        if isinstance(entry, dict) and entry.get("available"):
            angle = _optional_float(entry.get("target_angle_deg"))
            if angle is None:
                angle = 90.0
            if abs(angle) < SMALL_PULSE_ANGLE_THRESHOLD_DEG:
                warnings.append(f"{TURN_SMALL_PULSE_WARNING}:{key}:target_angle_deg={angle:g}")
        out[f"{key}_target_angle_deg"] = angle if angle is not None else "NA"
    out["turn_angle_warnings"] = warnings
    return out
