"""통합 경로계획 패키지의 보정 리졸버(calibration resolver) 계약 테스트.

목적/역할: ``tools.physical_path_planning.calibration.resolve_physical_calibration`` 이
여러 보정 파일(fine/twitch/angle/smooth/motion)을 어떻게 로드·병합하고, 커넥터 모드를
어떻게 자동 선택하는지 잠근다.

시스템 내 위치: 리졸버는 CLI 의 preview/run/execute 및 calibration-check 이 공유하는
"해석된 보정" 계층. 상위 로직은 여기서 나온 딕셔너리 형태/키에 의존한다.

핵심 개념·불변식:
  - auto 모드 커넥터 우선순위: angle_calibrated -> smooth_imu -> repeated_pulses 폴백.
  - angle_calibrated 를 명시 요청했는데 각도 보정이 없으면 RuntimeError.
  - ``connector_primitive`` 는 커넥터 프리미티브(b_cmd/pulse_ms/target_angle_deg/source)를 제공.
  - target_angle_deg: 실제 회전각을 표면화(키 이름 turn_*_90 을 맹신하지 않음); 폴백은 None.
  - ``ready_for_full_path_following`` 은 모든 경로에서 False 로 유지된다.

Calibration resolver contract tests for the consolidated path-planning package.
Cover fine/twitch/angle/smooth source loading, the auto-mode connector selection
(angle-calibrated -> smooth -> repeated-pulses fallback), the explicit
angle-calibrated guard, and the connector-primitive accessor -- all against
``tools.physical_path_planning.calibration``. ``ready_for_full_path_following``
stays false on every path.
"""
from __future__ import annotations

import json
from pathlib import Path

import pytest

from tools.physical_path_planning import calibration as resolver


# ── 픽스처·헬퍼 / Fixtures & helpers ──────────────────────────────────────────


def _write_json(path: Path, payload: dict[str, object]) -> Path:
    """보정 JSON 픽스처를 디스크에 쓰고 경로 반환. / Write a calibration JSON fixture, return path."""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return path


# ── 소스 로딩 / Source loading ────────────────────────────────────────────────


def test_resolver_loads_fine_calibration(tmp_path: Path) -> None:
    """fine 보정 파일에서 forward/backward 의 a/ms 를 읽어 source 태그와 함께 채운다.

    Loads forward/backward a/ms from the fine-calibration file, tagged by source."""
    fine = _write_json(
        tmp_path / "physical_ab_fine_motion_calibration.json",
        {
            "stage22_forward_a_cmd": 0.31,
            "stage22_forward_pulse_ms": 820,
            "stage21_backward_a_cmd": -0.09,
            "stage21_backward_pulse_ms": 320,
        },
    )
    resolved = resolver.resolve_physical_calibration(
        fine_calibration_json=fine,
        turn_calibration_json=tmp_path / "missing_turn.json",
        turn_angle_calibration_json=tmp_path / "missing_angle.json",
        smooth_turn_calibration_json=tmp_path / "missing_smooth.json",
    )
    assert resolved["forward"] == {"a": 0.31, "b": 0.0, "ms": 820, "source": "physical_ab_fine_motion_calibration.json"}
    assert resolved["backward"] == {"a": -0.09, "b": 0.0, "ms": 320, "source": "physical_ab_fine_motion_calibration.json"}
    assert resolved["ready_for_full_path_following"] is False


def test_resolver_loads_turn_twitch_calibration(tmp_path: Path) -> None:
    """turn twitch 파일에서 turn_left/right 의 b/ms(최소 트위치)를 로드.

    Loads turn_left/right b/ms (minimum twitch) from the turn-twitch file."""
    turn = _write_json(
        tmp_path / "physical_ab_turn_twitch_calibration.json",
        {
            "turn_left_min_b_cmd": 0.27,
            "turn_left_min_pulse_ms": 710,
            "turn_right_min_b_cmd": -0.10,
            "turn_right_min_pulse_ms": 260,
        },
    )
    resolved = resolver.resolve_physical_calibration(
        fine_calibration_json=tmp_path / "missing_fine.json",
        turn_calibration_json=turn,
        turn_angle_calibration_json=tmp_path / "missing_angle.json",
        smooth_turn_calibration_json=tmp_path / "missing_smooth.json",
    )
    assert resolved["turn_left"] == {"a": 0.0, "b": 0.27, "ms": 710, "source": "physical_ab_turn_twitch_calibration.json"}
    assert resolved["turn_right"] == {"a": 0.0, "b": -0.10, "ms": 260, "source": "physical_ab_turn_twitch_calibration.json"}


def test_resolver_loads_angle_calibration(tmp_path: Path) -> None:
    """각도 보정(turn_*_90, ready=True)을 로드하면 available=True + angle 커넥터 준비 완료.

    Loading angle calibration (turn_*_90, ready) marks it available and enables
    angle-based connectors."""
    angle = _write_json(
        tmp_path / "physical_ab_turn_angle_calibration.json",
        {
            "turn_left_90": {
                "a_cmd": 0.0,
                "b_cmd": 0.26,
                "pulse_ms": 700,
                "imu_yaw_delta_deg": 87.5,
                "visual_confirmation": "yes",
                "ready": True,
            },
            "turn_right_90": {
                "a_cmd": 0.0,
                "b_cmd": -0.08,
                "pulse_ms": 250,
                "imu_yaw_delta_deg": -91.2,
                "visual_confirmation": "yes",
                "ready": True,
            },
        },
    )
    resolved = resolver.resolve_physical_calibration(
        fine_calibration_json=tmp_path / "missing_fine.json",
        turn_calibration_json=tmp_path / "missing_turn.json",
        turn_angle_calibration_json=angle,
        smooth_turn_calibration_json=tmp_path / "missing_smooth.json",
    )
    assert resolved["turn_left_90"]["available"] is True
    assert resolved["turn_left_90"]["imu_yaw_delta_deg"] == 87.5
    assert resolved["turn_right_90"]["available"] is True
    assert resolved["ready_for_angle_based_connectors"] is True


# ── auto 모드 커넥터 선택 / auto-mode connector selection ──────────────────────


def test_auto_mode_chooses_angle_calibrated_when_both_90_turns_ready(tmp_path: Path) -> None:
    """양쪽 90° 회전이 준비되면 auto 모드는 angle_calibrated 를 추천·채택.

    커넥터 프리미티브가 각도 보정 소스의 b_cmd 를 실어 오는지 확인.
    With both 90-deg turns ready, auto mode recommends/uses angle_calibrated and the
    connector primitive carries the angle-calibration source's b_cmd."""
    angle = _write_json(
        tmp_path / "physical_ab_turn_angle_calibration.json",
        {
            "turn_left_90": {"a_cmd": 0, "b_cmd": 0.26, "pulse_ms": 700, "visual_confirmation": "yes", "ready": True},
            "turn_right_90": {"a_cmd": 0, "b_cmd": -0.08, "pulse_ms": 250, "visual_confirmation": "yes", "ready": True},
        },
    )
    resolved = resolver.resolve_physical_calibration(
        fine_calibration_json=tmp_path / "missing_fine.json",
        turn_calibration_json=tmp_path / "missing_turn.json",
        turn_angle_calibration_json=angle,
        smooth_turn_calibration_json=tmp_path / "missing_smooth.json",
        calibration_mode="auto",
    )
    assert resolved["connector_mode_recommended"] == "angle_calibrated"
    assert resolved["connector_mode_effective"] == "angle_calibrated"
    left = resolver.connector_primitive(resolved, "left")
    assert left["b_cmd"] == 0.26
    assert left["calibration_source"] == "physical_ab_turn_angle_calibration.json"


def test_auto_mode_falls_back_to_repeated_pulses_without_connector_calibration(tmp_path: Path) -> None:
    """커넥터 보정이 전혀 없으면 auto 모드는 안전한 repeated_pulses 로 폴백.

    폴백에서도 connector_primitive 는 내장 기본 b_cmd 를 제공.
    Without any connector calibration, auto mode falls back to repeated_pulses; the
    connector primitive still exposes built-in default b_cmd values."""
    resolved = resolver.resolve_physical_calibration(
        fine_calibration_json=tmp_path / "missing_fine.json",
        turn_calibration_json=tmp_path / "missing_turn.json",
        turn_angle_calibration_json=tmp_path / "missing_angle.json",
        smooth_turn_calibration_json=tmp_path / "missing_smooth.json",
        calibration_mode="auto",
    )
    assert resolved["connector_mode_recommended"] == "repeated_pulses"
    assert resolved["connector_mode_effective"] == "repeated_pulses"
    assert resolved["fallback_to_repeated_pulses"] is True
    assert resolver.connector_primitive(resolved, "right")["b_cmd"] == -0.08


def test_angle_calibrated_mode_fails_if_angle_calibration_missing(tmp_path: Path) -> None:
    """angle_calibrated 를 명시했는데 각도 보정이 없으면 조용히 폴백하지 않고 RuntimeError.

    Explicit angle_calibrated with no angle calibration raises (no silent fallback)."""
    with pytest.raises(RuntimeError, match="angle-calibrated connector requested"):
        resolver.resolve_physical_calibration(
            fine_calibration_json=tmp_path / "missing_fine.json",
            turn_calibration_json=tmp_path / "missing_turn.json",
            turn_angle_calibration_json=tmp_path / "missing_angle.json",
            smooth_turn_calibration_json=tmp_path / "missing_smooth.json",
            calibration_mode="angle_calibrated",
        )


def test_smooth_calibration_is_used_when_angle_missing(tmp_path: Path) -> None:
    """각도 보정이 없고 smooth 보정만 있으면 auto 모드는 smooth_imu 커넥터를 채택.

    커넥터 프리미티브의 pulse_ms 가 smooth 의 max_ms 를 반영하는지 확인.
    With angle missing but smooth present, auto mode selects smooth_imu connectors;
    the connector primitive's pulse_ms reflects smooth's max_ms."""
    smooth = _write_json(
        tmp_path / "smooth_turn_connector_calibration.json",
        {
            "smooth_left_b_cmd": 0.20,
            "smooth_left_max_ms": 3000,
            "smooth_right_b_cmd": 0.09,
            "smooth_right_max_ms": 2800,
            "ready_for_smooth_connectors": True,
            "ready_for_full_path_following": False,
        },
    )
    resolved = resolver.resolve_physical_calibration(
        fine_calibration_json=tmp_path / "missing_fine.json",
        turn_calibration_json=tmp_path / "missing_turn.json",
        turn_angle_calibration_json=tmp_path / "missing_angle.json",
        smooth_turn_calibration_json=smooth,
        calibration_mode="auto",
    )
    assert resolved["connector_mode_effective"] == "smooth_imu"
    assert resolved["ready_for_smooth_connectors"] is True
    assert resolver.connector_primitive(resolved, "left")["pulse_ms"] == 3000


# ── 커넥터 프리미티브·목표각 / connector primitive & target angle ──────────────


def test_connector_primitive_carries_target_angle_from_interactive_calibration(tmp_path: Path) -> None:
    """실제로 ~30° 도는 펄스가 turn_*_90 키에 저장돼도 target_angle_deg=30 을 표면화.

    executor 가 키 이름(90)을 맹신하지 않고 반복 펄스를 예산하도록 실제 각을 노출.
    A turn_*_90 entry whose pulse really turns ~30 deg surfaces target_angle_deg=30
    so the executor budgets repeated pulses instead of trusting the key name."""
    # A turn_*_90 key whose pulse really turns ~30 deg must surface that angle so
    # the executor budgets repeated pulses instead of trusting the key name.
    motion = _write_json(
        tmp_path / "motion_calibration.json",
        {
            "turn_left_90": {
                "approved_by_user": True,
                "a_cmd": 0.0,
                "b_cmd": 0.24,
                "pulse_ms": 700,
                "target_angle_deg": 30.0,
            },
            "turn_right_90": {
                "approved_by_user": True,
                "a_cmd": 0.0,
                "b_cmd": -0.08,
                "pulse_ms": 600,
                "target_angle_deg": 30.0,
            },
        },
    )
    resolved = resolver.resolve_physical_calibration(
        motion_calibration_json=motion,
        fine_calibration_json=tmp_path / "missing_fine.json",
        turn_calibration_json=tmp_path / "missing_turn.json",
        turn_angle_calibration_json=tmp_path / "missing_angle.json",
        smooth_turn_calibration_json=tmp_path / "missing_smooth.json",
        calibration_mode="auto",
    )
    assert resolved["connector_mode_effective"] == "angle_calibrated"
    left = resolver.connector_primitive(resolved, "left")
    right = resolver.connector_primitive(resolved, "right")
    assert left["target_angle_deg"] == 30.0
    assert right["target_angle_deg"] == 30.0
    assert right["b_cmd"] == -0.08


def test_connector_primitive_defaults_to_90_and_repeated_pulses_has_no_angle(tmp_path: Path) -> None:
    """target_angle_deg 이 없으면 각도-보정 프리미티브는 90° 로 디폴트; repeated_pulses 폴백은 None.

    Absent target_angle_deg defaults angle-calibrated primitives to 90; the
    repeated_pulses fallback exposes target_angle_deg=None."""
    motion = _write_json(
        tmp_path / "motion_calibration.json",
        {
            "turn_left_90": {"approved_by_user": True, "a_cmd": 0.0, "b_cmd": 0.24, "pulse_ms": 700},
            "turn_right_90": {"approved_by_user": True, "a_cmd": 0.0, "b_cmd": -0.08, "pulse_ms": 600},
        },
    )
    resolved = resolver.resolve_physical_calibration(
        motion_calibration_json=motion,
        fine_calibration_json=tmp_path / "missing_fine.json",
        turn_calibration_json=tmp_path / "missing_turn.json",
        turn_angle_calibration_json=tmp_path / "missing_angle.json",
        smooth_turn_calibration_json=tmp_path / "missing_smooth.json",
        calibration_mode="auto",
    )
    assert resolver.connector_primitive(resolved, "left")["target_angle_deg"] == 90.0
    fallback = resolver.resolve_physical_calibration(
        motion_calibration_json=None,
        fine_calibration_json=tmp_path / "missing_fine.json",
        turn_calibration_json=tmp_path / "missing_turn.json",
        turn_angle_calibration_json=tmp_path / "missing_angle.json",
        smooth_turn_calibration_json=tmp_path / "missing_smooth.json",
        calibration_mode="repeated_pulses",
    )
    assert resolver.connector_primitive(fallback, "left")["target_angle_deg"] is None


def test_turn_angle_summary_warns_on_small_pulse_stored_under_90_key(tmp_path: Path) -> None:
    """turn_angle_summary 는 각 90 키의 실제 target 각을 요약하고, 작은 각(30°)엔 경고 1건을 낸다.

    turn_angle_summary reports each 90-key's real target angle and emits exactly one
    warning for the small (30-deg) pulse stored under a _90 key."""
    motion = _write_json(
        tmp_path / "motion_calibration.json",
        {
            "turn_left_90": {
                "approved_by_user": True,
                "a_cmd": 0.0,
                "b_cmd": 0.24,
                "pulse_ms": 700,
                "target_angle_deg": 30.0,
            },
            "turn_right_90": {
                "approved_by_user": True,
                "a_cmd": 0.0,
                "b_cmd": -0.08,
                "pulse_ms": 600,
                "target_angle_deg": 90.0,
            },
        },
    )
    resolved = resolver.resolve_physical_calibration(
        motion_calibration_json=motion,
        fine_calibration_json=tmp_path / "missing_fine.json",
        turn_calibration_json=tmp_path / "missing_turn.json",
        turn_angle_calibration_json=tmp_path / "missing_angle.json",
        smooth_turn_calibration_json=tmp_path / "missing_smooth.json",
        calibration_mode="auto",
    )
    summary = resolver.turn_angle_summary(resolved)
    assert summary["turn_left_90_target_angle_deg"] == 30.0
    assert summary["turn_right_90_target_angle_deg"] == 90.0
    warnings = summary["turn_angle_warnings"]
    assert len(warnings) == 1
    assert resolver.TURN_SMALL_PULSE_WARNING in warnings[0]
    assert "turn_left_90" in warnings[0]
