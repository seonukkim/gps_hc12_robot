"""모션 튜닝/수동 보정 오버라이드 계약 테스트(tuning + set-motion-calibration CLI).

목적/역할: 대화형 모션 튜닝의 순수 로직과, ``set-motion-calibration`` 이 디스크의
motion_calibration.json 을 어떻게 안전하게 갱신하는지 잠근다.

시스템 내 위치:
  - ``tuning`` : 후보(candidate) 생성/피드백 반영/승인 저장/부호 검증 — CLI 의
    ``tune-motion`` / ``set-motion-calibration`` 이 사용.
  - ``cli`` : 위 모드들의 진입점(요약 JSON/백업/변경 파일 산출).
  - ``calibration`` 리졸버 + ``controller`` 의 dead-reckon/GPS 정책도 부분적으로 교차 검증.

핵심 개념·불변식:
  - 프리미티브별 부호 규약: forward a>0, backward a<0, turn-left b>0, turn-right b<0.
  - 승인 저장은 approved_by_user=True 로 기록하고, 프리셋/명시 오버라이드는 지정하지 않은
    다른 프리미티브와 operator_note 를 보존한다(비파괴적 병합 + 백업 생성).
  - 회전 튜닝은 가능하면 IMU yaw delta 를 사용해 pulse_ms 를 조정(목표각 근처면 유지).
  - ``ready_for_full_path_following`` 은 모든 경로에서 False.

Contract tests for motion tuning and manual calibration override (the ``tuning``
module + ``tune-motion`` / ``set-motion-calibration`` CLI modes). Locks in the
pure tuning logic (candidate init/feedback/approval-save/sign-validation), the
per-primitive sign convention (forward a>0, backward a<0, left b>0, right b<0),
and the non-destructive, backed-up disk merge performed by set-motion-calibration
(other primitives and operator_note preserved). ``ready_for_full_path_following``
stays false everywhere.
"""
from __future__ import annotations

import json
from pathlib import Path

import pytest

from tools.physical_path_planning import calibration, cli, controller, geometry, tuning


# ── 픽스처·헬퍼 / Fixtures & helpers ──────────────────────────────────────────


def _write_json(path: Path, payload: dict[str, object]) -> Path:
    """보정 JSON 픽스처를 디스크에 쓰고 경로 반환. / Write a calibration JSON fixture, return path."""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return path


# ── 순수 튜닝 로직 / Pure tuning logic ────────────────────────────────────────


def test_tune_motion_adjusts_candidate_from_user_feedback() -> None:
    """사용자 피드백(weak/strong)에 따라 후보의 a/ms(회전은 b/ms)를 규칙대로 조정.

    User feedback (weak/strong) nudges the candidate's a/ms (b/ms for turns) per rule."""
    candidate = tuning.initial_candidate("forward")
    adjusted = tuning.adjust_candidate(candidate, "weak")
    assert adjusted["a"] == 0.30
    assert adjusted["ms"] == 900

    adjusted = tuning.adjust_candidate(candidate, "strong")
    assert adjusted["ms"] == 700

    turn = tuning.initial_candidate("left")
    adjusted_turn = tuning.adjust_candidate(turn, "weak")
    assert adjusted_turn["b"] == 0.26
    assert adjusted_turn["ms"] == 800


def test_approve_saves_motion_calibration_json(tmp_path: Path) -> None:
    """승인 저장: 후보를 turn_right_90 항목으로 기록(목표각 90, IMU yaw delta, approved 플래그).

    Approval writes the candidate as a turn_right_90 entry (target 90, IMU delta,
    approved_by_user=True) and keeps ready_for_full_path_following False."""
    candidate = tuning.initial_candidate("turn-right-90")
    candidate["b"] = -0.12
    candidate["ms"] = 1600
    path = tmp_path / "motion_calibration.json"

    saved = tuning.save_approved_calibration(path, candidate, yaw_delta_deg=-91.0)

    assert saved["turn_right_90"]["a"] == 0.0
    assert saved["turn_right_90"]["b"] == -0.12
    assert saved["turn_right_90"]["ms"] == 1600
    assert saved["turn_right_90"]["target_angle_deg"] == 90.0
    assert saved["turn_right_90"]["last_imu_yaw_delta_deg"] == -91.0
    assert saved["turn_right_90"]["approved_by_user"] is True
    assert saved["ready_for_full_path_following"] is False


def test_turn_tuning_uses_imu_yaw_delta_when_available() -> None:
    """회전 튜닝은 IMU yaw delta(rows 에서 계산)를 이용: 목표에 못 미치면 ms↑, 근접하면 유지.

    Turn tuning uses the IMU yaw delta (from rows): under target => longer ms; near
    target => ms unchanged."""
    rows = [
        {"imu_relative_yaw_deg": "175.0"},
        {"imu_relative_yaw_deg": "-100.0"},
    ]
    assert tuning.yaw_delta_from_rows(rows) == 85.0

    candidate = tuning.initial_candidate("turn-left-90")
    adjusted = tuning.adjust_candidate(candidate, "good", yaw_delta_deg=45.0)
    assert adjusted["ms"] > candidate["ms"]

    near_target = tuning.adjust_candidate(candidate, "good", yaw_delta_deg=88.0)
    assert near_target["ms"] == candidate["ms"]


def test_tune_motion_has_no_observed_distance_option() -> None:
    """CLI 도움말에 tune-motion/set-motion-calibration 은 있고, 제거된 observed-distance 는 없다.

    Help exposes tune-motion/set-motion-calibration but not the removed observed-distance."""
    parser = cli.build_parser()
    help_text = parser.format_help()
    assert "tune-motion" in help_text
    assert "set-motion-calibration" in help_text
    assert "observed-distance" not in help_text


def test_opposite_sign_transient_detects_reverse_kick() -> None:
    """명령 부호와 반대인 모터 출력(역방향 킥)을 감지: forward 기대에 음수 명령이면 True.

    Detects a motor output opposite the intended sign (reverse kick): negative cmd
    under a forward primitive => True."""
    rows = [
        {
            "motor_write_called": "true",
            "physical_a_cmd": "-0.020",
            "final_left_cmd": "-0.020",
            "final_right_cmd": "-0.020",
        }
    ]
    assert tuning.opposite_sign_transient("forward", rows) is True
    assert tuning.opposite_sign_transient("backward", rows) is False


# ── CLI: tune-motion / set-motion-calibration ────────────────────────────────


def test_tune_motion_print_candidate_writes_summary(tmp_path: Path) -> None:
    """tune-motion --print-candidate 는 rc=0 + CANDIDATE_PRINTED 요약과 후보 JSON 을 쓴다.

    tune-motion --print-candidate: rc=0, CANDIDATE_PRINTED summary + candidate JSON."""
    rc = cli.main(
        [
            "tune-motion",
            "--primitive",
            "forward",
            "--print-candidate",
            "true",
            "--out-dir",
            str(tmp_path),
        ]
    )
    assert rc == 0
    summary = json.loads((tmp_path / "summary.json").read_text())
    assert summary["mode"] == "tune-motion"
    assert summary["reason"] == "CANDIDATE_PRINTED"
    assert summary["ready_for_full_path_following"] is False
    assert (tmp_path / "tune_motion_candidate.json").exists()


# ── 리졸버·컨트롤러 교차검증 / Resolver & controller cross-checks ─────────────


def test_motion_calibration_resolver_prefers_approved_values(tmp_path: Path) -> None:
    """리졸버는 approved 된 motion_calibration.json 값을 우선 사용하고 커넥터를 angle_calibrated 로.

    폴백(repeated_pulses)이 아니며 커넥터 프리미티브 pulse_ms 가 저장값을 반영.
    The resolver prefers approved motion_calibration.json values and selects
    angle_calibrated connectors (no repeated-pulses fallback)."""
    motion = _write_json(
        tmp_path / "motion_calibration.json",
        {
            "forward": {"a": 0.32, "b": 0.01, "ms": 900, "approved_by_user": True},
            "backward": {"a": -0.10, "b": 0.0, "ms": 350, "approved_by_user": True},
            "turn_left_90": {
                "a": 0.0,
                "b": 0.24,
                "ms": 1800,
                "target_angle_deg": 90,
                "last_imu_yaw_delta_deg": 88.5,
                "approved_by_user": True,
            },
            "turn_right_90": {
                "a": 0.0,
                "b": -0.12,
                "ms": 1600,
                "target_angle_deg": 90,
                "last_imu_yaw_delta_deg": -91.0,
                "approved_by_user": True,
            },
            "ready_for_full_path_following": False,
        },
    )

    resolved = calibration.resolve_physical_calibration(
        motion_calibration_json=motion,
        fine_calibration_json=tmp_path / "missing_fine.json",
        turn_calibration_json=tmp_path / "missing_turn.json",
        turn_angle_calibration_json=tmp_path / "missing_angle.json",
        smooth_turn_calibration_json=tmp_path / "missing_smooth.json",
        calibration_mode="auto",
    )

    assert resolved["forward"] == {"a": 0.32, "b": 0.01, "ms": 900, "source": "motion_calibration.json"}
    assert resolved["backward"] == {"a": -0.10, "b": 0.0, "ms": 350, "source": "motion_calibration.json"}
    assert resolved["connector_mode_effective"] == "angle_calibrated"
    assert resolved["fallback_to_repeated_pulses"] is False
    assert calibration.connector_primitive(resolved, "left")["pulse_ms"] == 1800
    assert resolved["ready_for_full_path_following"] is False


def test_set_motion_calibration_preset_writes_manual_high_soft_right(tmp_path: Path) -> None:
    """프리셋(field_manual_high_except_soft_right)은 forward/backward/turn 값을 기록하되,

    기존 operator_note 를 보존하고 백업/변경/갱신 파일을 남긴다(비파괴적 오버라이드).
    The manual-high/soft-right preset writes forward/backward/turn values while
    preserving operator_note and emitting backup/change/updated artifacts."""
    cal_path = tmp_path / "cal" / "motion_calibration.json"
    _write_json(
        cal_path,
        {
            "forward": {"a": 0.25, "b": 0.0, "ms": 700, "approved_by_user": True},
            "operator_note": "preserve me",
            "ready_for_full_path_following": False,
        },
    )
    out_dir = tmp_path / "manual_override"

    rc = cli.main(
        [
            "set-motion-calibration",
            "--preset",
            "field_manual_high_except_soft_right",
            "--calibration-out",
            str(cal_path),
            "--out-dir",
            str(out_dir),
        ]
    )

    assert rc == 0
    updated = json.loads(cal_path.read_text())
    assert updated["forward"] == {
        "a": 0.30,
        "b": 0.0,
        "ms": 1000,
        "approved_by_user": True,
        "source": "manual_high_preset",
    }
    assert updated["backward"]["a"] == -0.08
    assert updated["backward"]["ms"] == 350
    assert updated["turn_left_90"]["b"] == 0.26
    assert updated["turn_left_90"]["ms"] == 2400
    assert updated["turn_right_90"]["b"] == -0.08
    assert updated["turn_right_90"]["ms"] == 1000
    assert updated["turn_right_90"]["source"] == "manual_soft_right_preset"
    assert updated["operator_note"] == "preserve me"
    assert updated["ready_for_full_path_following"] is False

    summary = json.loads((out_dir / "summary.json").read_text())
    assert summary["mode"] == "set-motion-calibration"
    assert summary["preset"] == "field_manual_high_except_soft_right"
    assert summary["backup_path"] != "NONE"
    assert Path(summary["backup_path"]).exists()
    assert (out_dir / "manual_motion_calibration_change.json").exists()
    assert (out_dir / "motion_calibration_updated.json").exists()
    assert summary["ready_for_full_path_following"] is False


def test_set_motion_calibration_explicit_override_preserves_other_primitives(tmp_path: Path) -> None:
    """단일 프리미티브 명시 오버라이드(turn-right-90)는 그 항목만 바꾸고 forward 등은 그대로 둔다.

    change 파일의 updated_primitives 에 바뀐 항목만 기록됨.
    An explicit single-primitive override (turn-right-90) changes only that entry,
    leaving others intact; the change file lists just the updated primitive."""
    cal_path = _write_json(
        tmp_path / "cal" / "motion_calibration.json",
        {
            "forward": {
                "a": 0.30,
                "b": 0.0,
                "ms": 1000,
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
            "ready_for_full_path_following": False,
        },
    )
    out_dir = tmp_path / "right_soft"

    rc = cli.main(
        [
            "set-motion-calibration",
            "--primitive",
            "turn-right-90",
            "--a",
            "0.0",
            "--b",
            "-0.06",
            "--ms",
            "800",
            "--target-angle-deg",
            "90",
            "--source",
            "manual_soft_right_test",
            "--calibration-out",
            str(cal_path),
            "--out-dir",
            str(out_dir),
        ]
    )

    assert rc == 0
    updated = json.loads(cal_path.read_text())
    assert updated["forward"]["ms"] == 1000
    assert updated["turn_right_90"] == {
        "a": 0.0,
        "b": -0.06,
        "ms": 800,
        "target_angle_deg": 90.0,
        "approved_by_user": True,
        "source": "manual_soft_right_test",
    }
    change = json.loads((out_dir / "manual_motion_calibration_change.json").read_text())
    assert change["updated_primitives"] == ["turn_right_90"]


def test_set_motion_calibration_rejects_wrong_turn_sign(tmp_path: Path) -> None:
    """turn-right-90 에 양수 b(잘못된 부호)를 주면 rc=2 + OVERRIDE_INVALID, 저장하지 않음.

    A positive b for turn-right-90 (wrong sign) => rc=2, OVERRIDE_INVALID, no write."""
    out_dir = tmp_path / "bad"
    rc = cli.main(
        [
            "set-motion-calibration",
            "--primitive",
            "turn-right-90",
            "--a",
            "0.0",
            "--b",
            "0.06",
            "--ms",
            "800",
            "--target-angle-deg",
            "90",
            "--calibration-out",
            str(tmp_path / "cal" / "motion_calibration.json"),
            "--out-dir",
            str(out_dir),
        ]
    )

    assert rc == 2
    summary = json.loads((out_dir / "summary.json").read_text())
    assert summary["reason"] == "MOTION_CALIBRATION_OVERRIDE_INVALID"
    assert "B < 0" in summary["message"]
    assert summary["ready_for_full_path_following"] is False


def test_manual_calibration_sign_validation() -> None:
    """수동 보정 항목의 부호 검증: forward 는 A>0, turn-right 는 B<0 를 강제(위반 시 ValueError).

    manual_calibration_entry enforces sign rules (forward A>0, turn-right B<0)."""
    with pytest.raises(ValueError, match="forward calibration requires A > 0"):
        tuning.manual_calibration_entry("forward", a=-0.30, b=0.0, ms=1000, source="bad")
    with pytest.raises(ValueError, match="right/turn-right calibration requires B < 0"):
        tuning.manual_calibration_entry("turn-right-90", a=0.0, b=0.08, ms=1000, source="bad")


def test_mac_execute_plan_loads_approved_motion_calibration(tmp_path: Path) -> None:
    """execute-plan 인자에 motion-calibration-json 을 주면 resolve_calibration 이 그 승인값을 로드.

    execute-plan resolves the passed motion-calibration-json's approved forward values."""
    motion = _write_json(
        tmp_path / "motion_calibration.json",
        {"forward": {"a": 0.33, "b": 0.02, "ms": 950, "approved_by_user": True}},
    )
    parser = cli.build_parser()
    args = parser.parse_args(
        [
            "execute-plan",
            "--motion-calibration-json",
            str(motion),
            "--print-plan",
            "--start-lat",
            "35.1",
            "--start-lon",
            "129.1",
            "--goal-mode",
            "bearing_distance",
            "--goal-bearing-deg",
            "90",
            "--goal-distance-m",
            "4",
            "--path-shape",
            "direct_line",
        ]
    )
    resolved = cli.resolve_calibration(args)
    assert resolved["forward"]["a"] == 0.33
    assert resolved["forward"]["b"] == 0.02
    assert resolved["forward"]["ms"] == 950


def test_gps_degraded_continue_policy_does_not_abort_when_imu_available() -> None:
    """GPS 열화 시 dead_reckon_gps 가 캐시로 대체(degraded 표시)하고, continue 정책이면 중단 안 함.

    On degraded GPS, dead_reckon_gps uses the cache (flagged degraded); the continue
    policy keeps going rather than aborting."""
    cache: dict[str, object] = {"lat": 35.1, "lon": 129.1, "degraded": False}
    gps = controller.dead_reckon_gps(
        {"gps_block_reason": "BAD_HDOP", "current_lat": "NA", "current_lon": "NA"},
        cache,
    )
    assert gps["gps_degraded"] is True
    assert gps["gps_cached_used"] is True
    assert geometry.gps_policy_action(bool(gps["gps_degraded"]), "continue") == "continue"
