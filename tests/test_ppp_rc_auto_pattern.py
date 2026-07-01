"""무선-끊김 없이 도는 rc-auto-pattern 모드: 펌웨어 플래그 빌더 + print-cmd 경로.

목적/역할 (KO):
    RC 스위치 하나로 온보드 자동 패턴(레인/스텝 서펜타인)을 돌리는 "untethered
    (무선 미연결)" 모드를 검증한다. 이 테스트는 두 가지를 못 박는다:
    (1) ``cli.rc_auto_pattern_firmware_flags`` 가 만드는 ``-D...`` 펌웨어 컴파일
    플래그 집합 -- 실전에서 검증된 수동 프로파일(PPM 디코드), IMU yaw 피드백,
    온보드 패턴 파라미터, 헤딩 홀드/RC-손실 유예 기본값, 그리고 자율 게이트는
    반드시 꺼짐 -- 을, (2) ``--print-cmd`` 경로가 시리얼 없이도 설정/프리뷰
    파일을 써 내려가는 것.

핵심 계약·불변식 (KO):
    - 기본 베이스는 full-telemetry-ppm(이 수신기 채널을 실전에서 유일하게 디코드
      한 설정): ``MANUAL_CONTROL_PPM=1``, ``MANUAL_FORWARD_SIGN=-1``, IMU 켜짐.
    - 이 구동계는 *주행 중* B 에 대한 yaw 응답이 정지 피벗과 반대(2026-06-12 현장
      로그) -> ``DRIVE_STEER_SIGN=-1``. 직각 코너는 coast 보정 정지 + settle
      재시도 + 도넛-방지 레인 중단으로 확보.
    - 헤딩 홀드/RC-손실 유예 기본값은 항상 컴파일에 포함(무선에서도 레인이 곧게,
      전파 글리치가 주행을 죽이지 못하게). 이 값들은 kwargs 로 오버라이드 가능.
    - 자율 게이트(``PHYSICAL_PATH_FOLLOWING_ENABLE``, ``AUTO_MOTION_ARMED``)는
      항상 0 -- 이것은 RC 스위치 패턴이지 경로 추종이 아니다.
    - ``--print-cmd`` 는 시리얼 접속 없이 ``rc_auto_pattern_config.json`` +
      ``_preview.png`` + ``summary.json`` 을 남기며, 설정은 절대 ready 아님.

Purpose (EN):
    Verifies the untethered rc-auto-pattern mode: the ``-D`` firmware compile
    flags emitted by ``rc_auto_pattern_firmware_flags`` (field-proven manual PPM
    profile, IMU yaw feedback, the onboard pattern params, always-baked heading-
    hold / RC-loss-grace defaults, and autonomy gates forced off), plus the
    ``--print-cmd`` path that writes config/preview/summary files without serial.
    Note the drive-steer sign defaults to -1 because this drivetrain's yaw
    response to B while DRIVING is inverted vs a stationary pivot.
"""

from __future__ import annotations

import json
from pathlib import Path

from tools.physical_path_planning import cli


def test_rc_auto_pattern_flags_combine_manual_profile_imu_and_pattern() -> None:
    """기본 플래그가 수동 프로파일+IMU+온보드 패턴+홀드/유예 기본값을 결합하고 자율
    게이트는 끈다 / the default flags combine the manual profile, IMU, onboard
    pattern params and hold/grace defaults, with autonomy gates off."""
    flags = cli.rc_auto_pattern_firmware_flags(
        lanes=4,
        lane_ms=4200,
        step_ms=1400,
        forward_a=0.30,
        reverse_a=-0.30,
        turn_b_left=0.24,
        turn_b_right=-0.12,
        turn_target_deg=90.0,
        turn_tol_deg=8.0,
        turn_timeout_ms=15000,
        pause_ms=500,
    )
    # Proven manual profile basis (RC manual keeps working in MANUAL). The
    # default base is full-telemetry-ppm: firmware-default PPM decode, the
    # only configuration field-proven to decode this receiver's channels.
    assert "-DMANUAL_CONTROL_PPM=1" in flags
    assert "-DMANUAL_FORWARD_SIGN=-1" in flags
    assert "-DPPM_INTERRUPT_EDGE_FALLING=1" not in flags
    # IMU yaw must be compiled in for the pivot feedback.
    assert "-DIMU_ENABLE=1" in flags
    assert "-DIMU_ENABLE=0" not in flags
    # The onboard pattern and its parameters.
    assert "-DRC_AUTO_PATTERN=1" in flags
    assert "-DRC_AUTO_PATTERN_LANES=4" in flags
    assert "-DRC_AUTO_PATTERN_LANE_MS=4200" in flags
    assert "-DRC_AUTO_PATTERN_STEP_MS=1400" in flags
    assert "-DRC_AUTO_PATTERN_TURN_B_RIGHT=-0.12f" in flags
    # Heading-hold straights + RC-loss grace defaults are always baked in so
    # lanes stay straight untethered and a radio glitch cannot kill a run.
    assert "-DRC_AUTO_PATTERN_HEADING_KP=0.015f" in flags
    assert "-DRC_AUTO_PATTERN_HEADING_MAX_B=0.25f" in flags
    assert "-DRC_AUTO_PATTERN_DRIVE_B_TRIM=0.0f" in flags
    assert "-DRC_AUTO_PATTERN_RC_LOSS_GRACE_MS=1500" in flags
    # This drivetrain's yaw response to B while DRIVING is inverted vs a
    # stationary pivot (2026-06-12 field log), so the lane feedback sign
    # defaults to -1. Square corners come from coast-compensated stops plus
    # settle-verify retries, with an anti-donut lane abort as the safety net.
    assert "-DRC_AUTO_PATTERN_DRIVE_STEER_SIGN=-1.0f" in flags
    assert "-DRC_AUTO_PATTERN_DRIVE_ABORT_ERR_DEG=60.0" in flags
    assert "-DRC_AUTO_PATTERN_TURN_COAST_S=0.15f" in flags
    assert "-DRC_AUTO_PATTERN_TURN_SETTLE_RETRIES=2" in flags
    # Autonomy gates stay off: this is the RC-switch pattern, not path following.
    assert "-DPHYSICAL_PATH_FOLLOWING_ENABLE=0" in flags
    assert "-DAUTO_MOTION_ARMED=0" in flags


def test_rc_auto_pattern_flags_accept_heading_hold_overrides() -> None:
    """헤딩 홀드/스티어/유예 파라미터를 kwargs 로 오버라이드하면 플래그에 반영된다 /
    heading-hold / steer / grace parameters supplied as kwargs are reflected in
    the emitted flags."""
    flags = cli.rc_auto_pattern_firmware_flags(
        lanes=4,
        lane_ms=4200,
        step_ms=1400,
        forward_a=0.30,
        reverse_a=-0.30,
        turn_b_left=0.24,
        turn_b_right=-0.12,
        turn_target_deg=90.0,
        turn_tol_deg=8.0,
        turn_timeout_ms=15000,
        pause_ms=500,
        heading_kp=0.02,
        heading_hold_max_b=0.3,
        drive_b_trim=-0.05,
        drive_steer_sign=1.0,
        drive_abort_err_deg=45.0,
        turn_coast_s=0.2,
        turn_settle_retries=3,
        rc_loss_grace_ms=2000,
    )
    assert "-DRC_AUTO_PATTERN_HEADING_KP=0.02f" in flags
    assert "-DRC_AUTO_PATTERN_HEADING_MAX_B=0.3f" in flags
    assert "-DRC_AUTO_PATTERN_DRIVE_B_TRIM=-0.05f" in flags
    assert "-DRC_AUTO_PATTERN_DRIVE_STEER_SIGN=1.0f" in flags
    assert "-DRC_AUTO_PATTERN_DRIVE_ABORT_ERR_DEG=45.0" in flags
    assert "-DRC_AUTO_PATTERN_TURN_COAST_S=0.2f" in flags
    assert "-DRC_AUTO_PATTERN_TURN_SETTLE_RETRIES=3" in flags
    assert "-DRC_AUTO_PATTERN_RC_LOSS_GRACE_MS=2000" in flags


def test_rc_auto_pattern_print_cmd_writes_config_without_serial(tmp_path: Path) -> None:
    """--print-cmd 는 시리얼 없이 config/preview/summary 를 쓰고 설정은 절대 ready
    아님 / ``--print-cmd`` writes config/preview/summary files without any serial
    connection, and the config is never ready for full-path following."""
    rc = cli.main(
        [
            "rc-auto-pattern",
            "--print-cmd",
            "--lane-ms", "4200",
            "--step-ms", "1400",
            "--out-dir", str(tmp_path),
        ]
    )
    assert rc == 0
    config = json.loads((tmp_path / "rc_auto_pattern_config.json").read_text())
    assert config["mode"] == "rc-auto-pattern"
    assert config["untethered"] is True
    assert config["lane_ms"] == 4200
    assert config["heading_kp"] == 0.015
    assert config["rc_loss_grace_ms"] == 1500
    assert config["drive_steer_sign"] == -1.0
    assert config["turn_settle_retries"] == 2
    assert config["ready_for_full_path_following"] is False
    # The operator can SEE the expected path before going untethered.
    assert config["estimated_lane_m"] > 0
    assert (tmp_path / "rc_auto_pattern_preview.png").exists()
    summary = json.loads((tmp_path / "summary.json").read_text())
    assert summary["reason"] == "COMMAND_PRINTED"
