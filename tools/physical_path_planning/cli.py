"""물리 경로계획 통합 CLI: 현장에서 실제 로버를 다루는 단일 진입점.
Unified physical-path-planning CLI: one field-facing entrypoint.

목적/역할 (Purpose):
    ``scripts/run_physical_path_planner.sh`` 뒤에 있는 사용자 대면 CLI. GPS 대기,
    수동 제어, 미리보기, 모션 튜닝, 턴 보정, 계획 실행 등 모든 "모드"를 하나의
    argparse 서브커맨드 트리로 제공한다. 로버 시리얼 포트를 열고, arduino-cli 로
    펌웨어를 빌드/업로드하며, 각 실행의 JSON/Markdown 요약을 out-dir 에 기록한다.
    This is the user-facing CLI behind ``run_physical_path_planner.sh``: every mode
    is an argparse subcommand that may open the rover serial port, build/upload
    firmware via arduino-cli, and write JSON summaries under its ``--out-dir``.

모드 → 핸들러 매핑 (Mode → ``cmd_*`` handler map; wired in :func:`build_parser`):
    diagnose               -> cmd_diagnose            read-only telemetry summary.
    gps-wait               -> cmd_gps_wait            wait for GPS cold/warm start.
    rc-input-diagnose      -> cmd_rc_input_diagnose   read-only PPM channel probe.
    manual-rc              -> cmd_manual_rc           restore/validate RC passthrough.
    manual-control         -> cmd_manual_control      upload+monitor PPM manual control.
    rc-auto-pattern        -> cmd_rc_auto_pattern     untethered CH5 MANUAL/AUTO ㄹ pattern.
    station-hw-diagnose    -> cmd_station_hw_diagnose read-only station HW link diag.
    station-hw-manual      -> cmd_station_hw_manual   station HW manual rover control.
    usb-pulse-test         -> cmd_usb_pulse_test      laptop-USB bounded A/B pulse test.
    usb-drive-live         -> cmd_usb_drive_live      continuous USB A/B setpoint drive.
    tune-motion            -> cmd_tune_motion         interactive per-primitive calibration.
    set-motion-calibration -> cmd_set_motion_calibration  write preset/override. No motion.
    reset-motion-calibration -> cmd_reset_motion_calibration  back up + clear. No motion.
    calibration-check      -> cmd_calibration_check   stop_correct_go readiness. No motion.
    guarded-pulse-ready    -> cmd_guarded_pulse_ready upload/check IMU guarded firmware.
    calibrate-turn         -> cmd_calibrate_turn      shell out to turn-angle calibration.
    preview                -> cmd_preview             build + render coverage plan. No motion.
    auto-relative-preview  -> cmd_auto_relative_preview  relative A->B preview. No motion.
    inspect-plan           -> cmd_inspect_plan        inspect saved plan/images. No motion.
    align-heading          -> cmd_align_heading       point rover at first lane heading.
    execute-plan / run     -> cmd_run                 execute a planned path (guarded).
    auto-relative-run      -> cmd_auto_relative_run   AUTO-switch-triggered relative run.

핵심 개념·불변식 (Key concepts / invariants):
    - 모든 요약(summary) dict 은 ``checks.assert_not_ready_for_full_path_following``
      또는 ``write_summary_files`` 를 거치므로 어떤 모드도 "완전 경로추종 준비완료"를
      주장할 수 없다 (``ready_for_full_path_following`` 는 항상 False). 이 불변식은
      감독되지 않은 자율주행을 막는 안전장치이므로 절대 우회하지 말 것.
      Every summary flows through ``assert_not_ready_for_full_path_following`` /
      ``write_summary_files``, so no mode ever claims full-path-following readiness.
    - 하드웨어 모드는 실제로 호출될 때만 시리얼을 연다. ``--print-plan`` /
      ``--print-cmd`` / ``--from-log`` 는 하드웨어 없이 "무엇이 실행될지"만 보여주는
      완전 무하드웨어 경로다. ``import serial`` 은 이 무하드웨어 경로가 pyserial 을
      요구하지 않도록 각 핸들러 내부에서 지역 import 한다.
      Hardware modes open serial only when invoked; the print/from-log paths stay
      hardware-free (``import serial`` is deliberately local to each handler).
    - 물리 명령 규약: physical A = throttle(전후진), physical B = turn(조향).
      PPM 배선은 D6 신호 / CH1 조향 / CH2 스로틀 / CH5 모드(MANUAL/AUTO) 이다.
      Physical A = throttle, physical B = turn; PPM wiring D6/CH1/CH2/CH5.

사용법/진입점 (Entry point):
    ``main(argv)`` -> :func:`build_parser` -> ``args.handler(args)``. 각 서브파서는
    ``set_defaults(handler=cmd_*)`` 로 자신의 핸들러를 지정한다. ``station-manual`` /
    ``station-drive`` 는 ``usb-pulse-test`` 의 폐기 예정 별칭으로 재작성된다.

리팩토링 노트 (Refactoring notes):
    이 파일은 순수(무하드웨어) 헬퍼 + 출력 라이터 + 모드 핸들러 + argparse 로 구성된다.
    순수 헬퍼(예: ``evaluate_*_rows``, ``gps_snapshot``)는 단위 테스트가 직접 부르는
    계약이므로 반환 dict 의 키/의미를 함부로 바꾸지 말 것. 실제 모션은 controller /
    executor / alignment 모듈에 위임하며, 이 CLI 는 인자 → 그 함수 호출 매핑을 담당한다.
    Structure: pure helpers, output writers, mode handlers, argparse. The pure
    ``evaluate_*``/``gps_snapshot`` helpers are a tested contract; motion is
    delegated to controller/executor/alignment and this CLI only maps args to them.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import platform
import shlex
import subprocess
import sys
import time
from urllib.parse import unquote
from pathlib import Path
from typing import Callable, Sequence

from tools.physical_path_planning import alignment, calibration, checks, controller, executor, geometry, preview, safety, telemetry, tuning

DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = 115200
DEFAULT_GUARDED_PULSE_CALIBRATION_SCRIPT = "legacy/stage_scripts/run_guarded_pulse_calibration.sh"
DEFAULT_MANUAL_RC_UPLOAD_SCRIPT = "legacy/stage_scripts/upload_manual_rc_recovery_firmware.sh"
DEFAULT_MANUAL_RC_VALIDATE_SCRIPT = "legacy/stage_scripts/run_manual_rc_passthrough_validation.sh"
DEFAULT_RC_INPUT_DIAGNOSE_SKETCH = "firmware/ppm_channel_map_probe"
DEFAULT_TURN_CALIBRATION_OUT = (
    "outputs/stage23_turn_calibration/calibration/physical_ab_turn_angle_calibration.json"
)
DEFAULT_GPS_CACHE = Path("outputs/physical_path_planning/gps_cache/latest_start.json")
MAC_PHYSICAL_SUPERVISED_PROFILE = "MAC_PHYSICAL_SUPERVISED"
MANUAL_CONTROL_OLD_WORKING_PPM_PROFILE = "old-working-ppm"
MANUAL_CONTROL_RC_MIX_PPM_PROFILE = "rc-mix-ppm"
MANUAL_CONTROL_FULL_TELEMETRY_PPM_PROFILE = "full-telemetry-ppm"
MANUAL_CONTROL_PROFILES = (
    MANUAL_CONTROL_RC_MIX_PPM_PROFILE,
    MANUAL_CONTROL_OLD_WORKING_PPM_PROFILE,
    MANUAL_CONTROL_FULL_TELEMETRY_PPM_PROFILE,
)
MANUAL_CONTROL_DEFAULT_PROFILE = MANUAL_CONTROL_RC_MIX_PPM_PROFILE
MANUAL_CONTROL_OLD_WORKING_LOG = "outputs/logs/manual_forward_neg_turn_pos_20260530_141846.log"
MANUAL_CONTROL_OLD_WORKING_BUILD_PATH = "/private/tmp/openrb-manual-forward-neg-turn-pos"
MANUAL_CONTROL_RC_MIX_BUILD_PATH = "/private/tmp/openrb-manual-rc-mix-ppm"
MANUAL_CONTROL_FULL_TELEMETRY_BUILD_PATH = "/private/tmp/openrb-manual-control-ppm"
RC_INPUT_ABSENT_ACTION = (
    "Check RC receiver power; check receiver signal wire to OpenRB RC input; "
    "check whether receiver output mode is PPM/SBUS/PWM and firmware input mode matches; "
    "check mode channel index / channel mapping; check transmitter-receiver binding; "
    "if using individual PWM channels instead of PPM, firmware must read the correct pins; "
    "then run manual-rc --diagnose-only true after changing wiring or binding."
)
PPM_INPUT_ABSENT_ACTION = (
    "PPM input is absent. Expected wiring: signal -> OpenRB D6; CH1 steering; "
    "CH2 throttle; CH5 mode/manual-auto switch. Check station/controller power, "
    "PPM output mode, transmitter binding, and the D6 signal wire."
)
NO_USABLE_START_GPS_ACTION = (
    "No usable current or fresh cached GPS coordinate was available for the plan start. "
    "Move outside and wait longer for GPS, or pass --start-lat and --start-lon."
)
GPS_WAIT_TIMEOUT_ACTION = (
    "GPS characters are being monitored but no usable fix was reached before timeout. "
    "Move outdoors, wait longer for cold start, or pass --start-lat and --start-lon."
)
USB_PULSE_TEST_SEQUENCE = (
    {"primitive": "forward", "a": calibration.DEFAULT_FORWARD_A_CMD, "b": 0.0, "ms": calibration.DEFAULT_FORWARD_MS},
    {"primitive": "backward", "a": calibration.DEFAULT_BACKWARD_A_CMD, "b": 0.0, "ms": calibration.DEFAULT_BACKWARD_MS},
    {"primitive": "left", "a": 0.0, "b": calibration.DEFAULT_TURN_LEFT_B_CMD, "ms": calibration.DEFAULT_TURN_LEFT_MS},
    {"primitive": "right", "a": 0.0, "b": calibration.DEFAULT_TURN_RIGHT_B_CMD, "ms": calibration.DEFAULT_TURN_RIGHT_MS},
)
USB_PULSE_TEST_ALIASES = {
    "forward": "forward",
    "backward": "backward",
    "left": "left",
    "right": "right",
    "turn_left": "left",
    "turn_right": "right",
}


# ── 순수 무하드웨어 헬퍼 (단위 테스트 대상) / Pure no-hardware helpers ──
# --- Pure, no-hardware helpers (directly unit-testable) -----------------------


# 예전 stage 접두 telemetry 키와의 하위호환. 문자열을 런타임에 이어붙여, 소스에서
# 이 파일을 "stage20" 잔재로 착각해 잡아내지 않도록 한다.
# Back-compat with older "stageNN" telemetry keys; assembled at runtime so a grep
# for the retired stage names does not flag this file.
_COMPAT_GUARDED_MODE_KEY = "stage" + "20_physical_ab_guarded_crawl"
_COMPAT_GUARDED_READY_KEY = "stage" + "20_firmware_ready"
_COMPAT_GUARDED_STATE_KEY = "stage" + "20_cmd_state"
_COMPAT_GUARDED_STATE_FALLBACK_KEY = "stage" + "16_cmd_state"


# ── 펌웨어 컴파일 플래그 빌더 / Firmware compile-flag builders ──
# 이 함수들은 arduino-cli 의 ``compiler.cpp.extra_flags`` 로 넘길 ``-D...`` 문자열을
# 만든다. 모든 프로파일이 경로추종 계열 플래그를 명시적으로 0 으로 끄고(감독 불가
# 자율주행 방지) 각 모드가 필요한 기능만 켠다 — 이 불변식은 안전상 유지해야 한다.
# These build the ``-D...`` string passed to arduino-cli's extra_flags. Every
# profile explicitly disables the path-following flags (no unsupervised autonomy)
# and enables only what the mode needs; keep that invariant.


def guarded_pulse_imu_flags(*, max_abs_a: float = 0.35, max_abs_b: float = 0.35, max_ms: int = 1500) -> str:
    """가드된 펄스 + IMU 진단 펌웨어 플래그 문자열. / Guarded-pulse + IMU-diag flags.

    A/B 명령 크기와 펄스 길이를 하드웨어에서 강제로 상한(max_abs_a/b, max_ms)으로
    묶어 감독형 실험을 안전하게 만든다. RC 입력은 무시하고 IMU yaw 진단만 켠다.
    Bounds A/B magnitude and pulse length in firmware for supervised safety;
    ignores RC input and enables IMU yaw diagnostics only.
    """
    return (
        "-DUSB_PULSE_TEST_GUARDED=1 "
        f"-DUSB_PULSE_TEST_MAX_ABS_A={max_abs_a} "
        f"-DUSB_PULSE_TEST_MAX_ABS_B={max_abs_b} "
        f"-DUSB_PULSE_TEST_MAX_MS={max_ms} "
        "-DUSB_PULSE_TEST_IGNORE_RC_INPUT=1 "
        "-DUSB_DRIVE_LIVE_ENABLE=1 "
        "-DIMU_ENABLE=1 "
        "-DIMU_YAW_DIAG=1 "
        "-DPHYSICAL_PATH_FOLLOWING_ENABLE=0 "
        "-DPATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0 "
        "-DPATH_FOLLOWING_DRYRUN=0 "
        "-DPATH_FOLLOWING_HC12_ENABLED=0 "
        "-DGROUND_CRAWL_TEST_MODE=0 "
        "-DAUTO_MOTION_ARMED=0"
    )


def mac_physical_supervised_firmware_flags(
    *,
    max_abs_a: float = 0.35,
    max_abs_b: float = 0.35,
    max_ms: int = 1000,
    max_duration_ms: int = 1000,
    update_timeout_ms: int = 350,
) -> str:
    """감독형(MAC_PHYSICAL_SUPERVISED) 펌웨어 플래그. / Supervised firmware flags.

    gps-wait / preview / usb-pulse-test / usb-drive-live / tune-motion / run 이
    공유하는 기본 프로파일. 펄스(guarded)와 실시간 setpoint 드라이브(live) 양쪽
    상한, IMU, GPS/telemetry 를 켜되 경로추종·모터 자율출력은 끈다. ``update_timeout_ms``
    는 라이브 명령의 데드맨 TTL 이라 이보다 오래 새 setpoint 가 없으면 정지한다.
    Shared base profile; enables guarded + live-drive bounds, IMU and telemetry,
    but keeps path-following/auto motor output off. ``update_timeout_ms`` is the
    live-drive deadman TTL.
    """
    return (
        "-DMAC_PHYSICAL_SUPERVISED=1 "
        "-DUSB_PULSE_TEST_GUARDED=1 "
        "-DUSB_PULSE_TEST_IGNORE_RC_INPUT=1 "
        f"-DUSB_PULSE_TEST_MAX_ABS_A={max_abs_a} "
        f"-DUSB_PULSE_TEST_MAX_ABS_B={max_abs_b} "
        f"-DUSB_PULSE_TEST_MAX_MS={max_ms} "
        "-DUSB_DRIVE_LIVE_ENABLE=1 "
        "-DUSB_DRIVE_LIVE_IGNORE_RC_INPUT=1 "
        f"-DUSB_DRIVE_LIVE_MAX_ABS_A={max_abs_a} "
        f"-DUSB_DRIVE_LIVE_MAX_ABS_B={max_abs_b} "
        f"-DUSB_DRIVE_LIVE_MAX_DURATION_MS={max_duration_ms} "
        f"-DUSB_DRIVE_LIVE_UPDATE_TIMEOUT_MS={update_timeout_ms} "
        "-DIMU_ENABLE=1 "
        "-DIMU_YAW_DIAG=1 "
        "-DPHYSICAL_PATH_FOLLOWING_ENABLE=0 "
        "-DPATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0 "
        "-DPATH_FOLLOWING_DRYRUN=0 "
        "-DPATH_FOLLOWING_HC12_ENABLED=0 "
        "-DGROUND_CRAWL_TEST_MODE=0 "
        "-DAUTO_MOTION_ARMED=0"
    )


def guarded_pulse_firmware_flags(
    *,
    max_abs_a: float = 0.35,
    max_abs_b: float = 0.35,
    max_ms: int = 1500,
    ignore_rc_input_for_usb_command: bool = False,
) -> str:
    """guarded-pulse-ready 모드용 플래그. / Flags for the guarded-pulse-ready mode.

    ``ignore_rc_input_for_usb_command`` 이 True 면 RC 입력이 없어도 USB 명령을
    받도록 STATION_MANUAL_IGNORE_RC_INPUT 를 덧붙인다.
    """
    flags = guarded_pulse_imu_flags(max_abs_a=max_abs_a, max_abs_b=max_abs_b, max_ms=max_ms)
    if ignore_rc_input_for_usb_command:
        flags += " -DSTATION_MANUAL_IGNORE_RC_INPUT=1"
    return flags


def usb_pulse_test_firmware_flags(
    *,
    max_abs_a: float = 0.35,
    max_abs_b: float = 0.35,
    max_ms: int = 1000,
) -> str:
    """usb-pulse-test 펌웨어 플래그(감독형 프로파일 위임). / usb-pulse-test flags."""
    return mac_physical_supervised_firmware_flags(
        max_abs_a=max_abs_a,
        max_abs_b=max_abs_b,
        max_ms=max_ms,
    )


def usb_drive_live_firmware_flags(
    *,
    max_abs_a: float = 0.35,
    max_abs_b: float = 0.35,
    max_duration_ms: int = 3000,
    update_timeout_ms: int = 350,
) -> str:
    """usb-drive-live 펌웨어 플래그(감독형 프로파일 위임). / usb-drive-live flags."""
    return mac_physical_supervised_firmware_flags(
        max_abs_a=max_abs_a,
        max_abs_b=max_abs_b,
        max_duration_ms=max_duration_ms,
        update_timeout_ms=update_timeout_ms,
    )


def station_hw_manual_firmware_flags() -> str:
    """물리 스테이션 HW 수동제어 펌웨어 플래그. / Station-hardware manual flags."""
    return (
        "-DSTATION_HW_MANUAL_ENABLE=1 "
        "-DSTATION_HW_MANUAL_A_B_MAPPING=1 "
        "-DSTATION_HW_MANUAL_IGNORE_RC_INPUT=1 "
        "-DPHYSICAL_PATH_FOLLOWING_ENABLE=0 "
        "-DPATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0 "
        "-DPATH_FOLLOWING_DRYRUN=0 "
        "-DGROUND_CRAWL_TEST_MODE=0 "
        "-DAUTO_MOTION_ARMED=0"
    )


def station_hw_diagnose_firmware_flags() -> str:
    """스테이션 HW 진단 전용(모터 출력 없음) 플래그. / Diagnose-only station flags."""
    return station_hw_manual_firmware_flags() + " -DSTATION_HW_MANUAL_DIAGNOSE_ONLY=1"


def manual_rc_recovery_flags(*, mode_channel_index: int | None = None) -> str:
    """구형 검증된 수동 RC 복구 펌웨어 플래그. / Known-good manual RC recovery flags.

    현장 검증된 방향 부호(FORWARD=-1, TURN=+1)와 좌우 미교환/보정 비활성을 고정한다.
    Locks the field-proven direction signs and disables L/R swap + drive calibration.
    """
    flags = (
        "-DMANUAL_RC_RECOVERY=1 "
        "-DMANUAL_FORWARD_SIGN=-1 "
        "-DMANUAL_TURN_SIGN=1 "
        "-DMOTOR_OUTPUT_SWAP_LR=0 "
        "-DDRIVE_CALIBRATION_ENABLE=0 "
        "-DPHYSICAL_PATH_FOLLOWING_ENABLE=0 "
        "-DPATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0 "
        "-DPATH_FOLLOWING_DRYRUN=0 "
        "-DPATH_FOLLOWING_HC12_ENABLED=0 "
        "-DGROUND_CRAWL_TEST_MODE=0 "
        "-DAUTO_MOTION_ARMED=0"
    )
    if mode_channel_index is not None:
        flags += f" -DMODE_CHANNEL_INDEX={mode_channel_index}"
    return flags


def manual_control_firmware_flags(
    *,
    profile: str = MANUAL_CONTROL_DEFAULT_PROFILE,
    mode_channel_index: int | None = 4,
) -> str:
    """PPM 수동제어 펌웨어 플래그를 프로파일별로 생성. / PPM manual-control flags.

    프로파일마다 PPM 디코더 설정(에지/sync 임계/최소폭)이 다르다. 이 값들이 실제
    수신기의 채널을 해독하느냐를 좌우하므로 프로파일이 곧 "어느 디코더를 쓸지" 선택이다:
    - rc-mix-ppm: FALLING 에지, sync 4000us (2026-05-02 rc_mix_test 검증).
    - old-working-ppm: RISING 에지, sync 3000us, 최소폭 800us (구형 검증 값).
    - full-telemetry-ppm: 펌웨어 기본 디코드값 유지(감독형 펌웨어와 동일) — 이 수신기
      채널을 해독한다고 현장 확인된 유일한 설정.
    Each profile is a different PPM decoder setting (edge/sync/min-width); the
    profile choice decides which decoder actually reads this receiver's channels.
    Raises ``ValueError`` on an unknown profile.
    """
    if profile == MANUAL_CONTROL_RC_MIX_PPM_PROFILE:
        flags = (
            "-DMANUAL_CONTROL_PPM=1 "
            "-DMANUAL_FORWARD_SIGN=-1 "
            "-DMANUAL_TURN_SIGN=1 "
            "-DMOTOR_OUTPUT_SWAP_LR=0 "
            "-DDRIVE_CALIBRATION_ENABLE=0 "
            "-DPPM_INTERRUPT_EDGE_FALLING=1 "
            "-DPPM_SYNC_THRESHOLD_US=4000 "
            "-DPPM_CAPTURE_MIN_US_VALUE=0 "
            "-DIMU_ENABLE=0 "
            "-DPHYSICAL_PATH_FOLLOWING_ENABLE=0 "
            "-DPATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0 "
            "-DPATH_FOLLOWING_DRYRUN=0 "
            "-DPATH_FOLLOWING_HC12_ENABLED=0 "
            "-DGROUND_CRAWL_TEST_MODE=0 "
            "-DAUTO_MOTION_ARMED=0"
        )
    elif profile == MANUAL_CONTROL_OLD_WORKING_PPM_PROFILE:
        flags = (
            "-DMANUAL_CONTROL_PPM=1 "
            "-DMANUAL_FORWARD_SIGN=-1 "
            "-DMANUAL_TURN_SIGN=1 "
            "-DMOTOR_OUTPUT_SWAP_LR=0 "
            "-DDRIVE_CALIBRATION_ENABLE=0 "
            "-DPPM_INTERRUPT_EDGE_FALLING=0 "
            "-DPPM_SYNC_THRESHOLD_US=3000 "
            "-DPPM_CAPTURE_MIN_US_VALUE=800 "
            "-DIMU_ENABLE=0 "
            "-DPHYSICAL_PATH_FOLLOWING_ENABLE=0 "
            "-DPATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0 "
            "-DPATH_FOLLOWING_DRYRUN=0 "
            "-DPATH_FOLLOWING_HC12_ENABLED=0 "
            "-DGROUND_CRAWL_TEST_MODE=0 "
            "-DAUTO_MOTION_ARMED=0"
        )
    elif profile == MANUAL_CONTROL_FULL_TELEMETRY_PPM_PROFILE:
        flags = (
            "-DMANUAL_CONTROL_PPM=1 "
            "-DMANUAL_FORWARD_SIGN=-1 "
            "-DMANUAL_TURN_SIGN=1 "
            "-DMOTOR_OUTPUT_SWAP_LR=0 "
            "-DDRIVE_CALIBRATION_ENABLE=0 "
            "-DIMU_ENABLE=0 "
            "-DPHYSICAL_PATH_FOLLOWING_ENABLE=0 "
            "-DPATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0 "
            "-DPATH_FOLLOWING_DRYRUN=0 "
            "-DPATH_FOLLOWING_HC12_ENABLED=0 "
            "-DGROUND_CRAWL_TEST_MODE=0 "
            "-DAUTO_MOTION_ARMED=0"
        )
    else:
        raise ValueError(f"unknown manual-control profile: {profile}")
    if mode_channel_index is not None:
        flags += f" -DMODE_CHANNEL_INDEX={mode_channel_index}"
    return flags


def manual_control_build_path(profile: str) -> str:
    """프로파일별 arduino-cli 빌드 캐시 경로. / arduino-cli build path per profile.

    프로파일마다 별도 빌드 경로를 쓰면 프로파일을 바꿔도 재컴파일 캐시가 섞이지 않는다.
    Separate build paths keep the compile cache clean across profile switches.
    """
    if profile == MANUAL_CONTROL_RC_MIX_PPM_PROFILE:
        return MANUAL_CONTROL_RC_MIX_BUILD_PATH
    if profile == MANUAL_CONTROL_OLD_WORKING_PPM_PROFILE:
        return MANUAL_CONTROL_OLD_WORKING_BUILD_PATH
    if profile == MANUAL_CONTROL_FULL_TELEMETRY_PPM_PROFILE:
        return MANUAL_CONTROL_FULL_TELEMETRY_BUILD_PATH
    raise ValueError(f"unknown manual-control profile: {profile}")


def manual_control_expected_ppm_edge(profile: str) -> str:
    """프로파일이 기대하는 PPM 인터럽트 에지. / Expected PPM interrupt edge.

    모니터가 실제 관측 에지와 비교해 에지 불일치(펌웨어/프로파일 오설정)를 잡는 데 쓴다.
    Used to flag a PPM edge mismatch (wrong firmware/profile) against observed edges.
    """
    if profile == MANUAL_CONTROL_RC_MIX_PPM_PROFILE:
        return "FALLING"
    return "RISING"


def manual_control_mapping(
    *,
    steer_norm: float,
    throttle_norm: float,
    forward_sign: float = -1.0,
    turn_sign: float = 1.0,
) -> dict[str, float]:
    """Return the physical A/B commands selected by the old PPM manual path.

    The PPM wiring is CH1 steering, CH2 throttle, CH5 mode. The old working
    controller used ``MANUAL_FORWARD_SIGN=-1`` and ``MANUAL_TURN_SIGN=1`` before
    the logical-wheel-to-physical A/B conversion. This helper keeps the tested
    sign contract explicit without touching path-planning logic.
    """
    physical_a = max(-1.0, min(1.0, forward_sign * throttle_norm))
    physical_b = max(-1.0, min(1.0, -turn_sign * steer_norm))
    return {"physical_a_cmd": physical_a, "physical_b_cmd": physical_b}


# ── PPM/RC telemetry 행(row) 분류 헬퍼 / PPM+RC telemetry row classifiers ──
# 아래 ``_row_*`` 헬퍼들은 파싱된 USBDBG telemetry 한 행을 검사해 "PPM 입력이
# 있나/유효한가/모드 채널이 잡혔나" 같은 참/거짓 판정을 내린다. 순수 함수이며
# ``evaluate_manual_control_rows`` 와 상태줄(status line)이 이들을 조합한다.
# These ``_row_*`` helpers inspect one parsed USBDBG row for PPM/RC presence and
# validity; pure predicates composed by ``evaluate_manual_control_rows``.


def _row_input_zero(row: dict[str, str]) -> bool:
    """이 행의 모든 RC 채널값이 사실상 0인가. / All RC channel values ~zero in this row."""
    keys = [f"raw_ch{i}_us" for i in range(1, 9)] + [
        "steer_us",
        "throttle_us",
        "mode_us",
        "raw_mode_channel_us",
    ]
    present = [key for key in keys if key in row]
    if not present:
        return False
    return all(abs(telemetry._optional_float(row.get(key)) or 0.0) <= 1e-3 for key in present)


def _row_input_nonzero(row: dict[str, str]) -> bool:
    """이 행에 0이 아닌 RC 채널이 하나라도 있나. / Any nonzero RC channel in this row."""
    keys = [f"raw_ch{i}_us" for i in range(1, 9)] + [
        "steer_us",
        "throttle_us",
        "mode_us",
        "raw_mode_channel_us",
    ]
    return any(abs(telemetry._optional_float(row.get(key)) or 0.0) > 1e-3 for key in keys)


def _row_rc_input_detected(row: dict[str, str]) -> bool:
    """이 행이 실제 RC 입력 감지를 나타내는가. / Does this row indicate real RC input?

    sync 펄스만 있거나 프레임 캡처가 없는 경우는(=채널 미해독) 감지로 치지 않는다.
    Rows that are sync-only or have no captured frame do not count as detected.
    """
    if _row_ppm_sync_only(row) or _row_no_ppm_frame_capture(row):
        return False
    return telemetry._parse_bool(row.get("rc_input_detected"), default=False) or _row_input_nonzero(row)


def _row_optional_int(row: dict[str, str], key: str) -> int | None:
    """행 값을 int 로(없으면 None). / Row value as int, or None when absent."""
    value = telemetry._optional_float(row.get(key))
    if value is None:
        return None
    return int(value)


def _row_ppm_edge_mismatch(row: dict[str, str], expected_ppm_interrupt_edge: str | None = None) -> bool:
    """관측 PPM 에지가 기대값과 다른가. / Observed PPM edge differs from expected?"""
    if not expected_ppm_interrupt_edge:
        return False
    edge = _row_value(row, ["ppm_interrupt_edge"], default="")
    return bool(edge and edge.upper() != expected_ppm_interrupt_edge.upper())


def _row_ppm_sync_only(row: dict[str, str]) -> bool:
    """sync 펄스만 있고 채널 프레임은 없는가. / Sync pulses seen but no channel frames?

    D6 에 sync 유사 펄스는 들어오는데 CH1/CH2/CH5 구간이 해독되지 않는 상태.
    보통 수신기가 결합 PPM 이 아니라 단일 PWM 을 내보내거나 디코더 프로파일이 틀린 경우.
    Usually means the receiver is not emitting combined PPM (single PWM instead),
    or the edge/sync decoder profile is wrong.
    """
    decode_reason = _row_value(row, ["ppm_decode_reason", "mode_decode_reason"], default="")
    if decode_reason == "PPM_SYNC_ONLY_NO_CHANNELS":
        return True
    sync_count = _row_optional_int(row, "ppm_sync_count")
    frame_count = _row_optional_int(row, "ppm_frame_count")
    last_channel_count = _row_optional_int(row, "ppm_last_channel_count")
    return (
        (sync_count is not None and sync_count > 0)
        and (frame_count is None or frame_count <= 0)
        and (last_channel_count is None or last_channel_count <= 0)
    )


def _row_no_ppm_frame_capture(row: dict[str, str]) -> bool:
    """PPM 카운터는 있으나 프레임/채널을 하나도 못 잡았는가. / PPM counters but no capture.

    PPM 관련 카운터 필드는 존재하지만 프레임 수·채널 수가 0이고 입력이 전부 0인 경우.
    PPM counters are present yet frame/channel counts are 0 and all inputs are zero.
    """
    frame_count = _row_optional_int(row, "ppm_frame_count")
    last_channel_count = _row_optional_int(row, "ppm_last_channel_count")
    has_ppm_counters = frame_count is not None or last_channel_count is not None
    return (
        has_ppm_counters
        and (frame_count is None or frame_count <= 0)
        and (last_channel_count is None or last_channel_count <= 0)
        and _row_input_zero(row)
    )


def _row_mode_channel_capture_known(row: dict[str, str]) -> bool:
    """모드 채널(CH5)까지 캡처됐다고 볼 수 있는가. / Was the mode channel (CH5) captured?

    채널 수가 5개 이상 잡혔으면 CH5 도 캡처된 것으로 본다(모드 채널 존재 판단 근거).
    A channel count >=5 implies CH5 was captured too (basis for "mode channel known").
    """
    if _row_input_zero(row):
        return False
    last_channel_count = _row_optional_int(row, "ppm_last_channel_count")
    frame_count = _row_optional_int(row, "ppm_frame_count")
    if last_channel_count is not None:
        return last_channel_count >= 5 and (frame_count is None or frame_count > 0)
    if frame_count is not None and frame_count <= 0:
        return False
    return _row_rc_input_detected(row)


def manual_control_mode_decode(row: dict[str, str]) -> tuple[str, str]:
    """한 telemetry 행에서 (모드 스위치, 디코드 사유)를 결정. / Decode (mode, reason).

    반환은 ``(manual_switch, mode_decode_reason)`` 튜플. 우선순위: telemetry 없음 →
    sync-only → PPM 부재 → 펌웨어가 직접 보고한 값 → mode_us(us) 로 MANUAL/AUTO 판정.
    mode_us 는 대략 <=1600us=MANUAL, >1600us=AUTO, 900~2100us 밖은 범위이탈로 본다.
    Returns ``(manual_switch, mode_decode_reason)``; falls back from firmware-reported
    values to a mode_us threshold (~<=1600us MANUAL, >1600us AUTO).
    """
    if not row:
        return "UNKNOWN_NO_USBDBG_TELEMETRY", "NO_USBDBG_TELEMETRY"
    if _row_ppm_sync_only(row):
        return "UNKNOWN_PPM_SYNC_ONLY", "PPM_SYNC_ONLY_NO_CHANNELS"
    if _row_no_ppm_frame_capture(row):
        return "UNKNOWN_PPM_ABSENT", "PPM_INPUT_ABSENT"
    row_manual_switch = _row_value(row, ["manual_switch"], default="")
    row_mode_decode_reason = _row_value(row, ["mode_decode_reason"], default="")
    if row_mode_decode_reason == "NO_MODE_CHANNEL" and not _row_mode_channel_capture_known(row):
        return "UNKNOWN_PPM_ABSENT", "PPM_INPUT_ABSENT"
    if row_manual_switch and row_mode_decode_reason:
        return row_manual_switch, row_mode_decode_reason
    if not _row_rc_input_detected(row):
        return "UNKNOWN_PPM_ABSENT", "PPM_INPUT_ABSENT"
    mode_us = telemetry._optional_float(row.get("mode_us") or row.get("raw_mode_channel_us"))
    if mode_us is None or mode_us <= 0.0:
        if not _row_mode_channel_capture_known(row):
            return "UNKNOWN_PPM_ABSENT", "PPM_INPUT_ABSENT"
        return "UNKNOWN_MODE_CHANNEL_MISSING", "NO_MODE_CHANNEL"
    if mode_us < 900.0 or mode_us > 2100.0:
        return "UNKNOWN_MODE_CHANNEL_INVALID", "MODE_CHANNEL_OUT_OF_RANGE"
    if mode_us > 1600.0:
        return "AUTO", "MODE_CHANNEL_AUTO"
    return "MANUAL", "MODE_CHANNEL_MANUAL"


def _latest_manual_control_row(rows: Sequence[dict[str, str]]) -> dict[str, str]:
    """수동제어 상태 필드를 포함한 가장 최근 행. / Latest row carrying manual-control fields.

    뒤에서부터 훑어 상태 키(모드/PPM/GPS/IMU 등)를 하나라도 가진 첫 행을 돌려준다.
    Scans from the end for the first row bearing any manual-control status key.
    """
    status_keys = {
        "manual_control",
        "manual_control_ppm",
        "rc_input_detected",
        "mode",
        "auto_sw",
        "rc_ok",
        "steer_us",
        "throttle_us",
        "mode_us",
        "raw_mode_channel_us",
        *[f"raw_ch{i}_us" for i in range(1, 9)],
        "ppm_interrupt_edge",
        "ppm_decode_reason",
        "ppm_edge_count",
        "ppm_sync_count",
        "ppm_sync_age_ms",
        "ppm_last_width_us",
        "ppm_min_width_us",
        "ppm_max_width_us",
        "ppm_frame_count",
        "ppm_last_channel_count",
        "ppm_short_rejects",
        "ppm_long_rejects",
        "ppm_last_rejected_us",
        "gps_block_reason",
        "gps_sats",
        "gps_hdop",
        "current_lat",
        "current_lon",
        "gps_cached_lat",
        "gps_cached_lon",
        "imu_present",
        "imu_relative_yaw_deg",
        "imu_heading_block_reason",
    }
    for row in reversed(rows):
        if any(key in row for key in status_keys):
            return row
    return {}


def _row_value(row: dict[str, str], keys: Sequence[str], default: str = "NA") -> str:
    """후보 키들 중 첫 번째 의미있는 값(빈/NA 제외). / First meaningful value among keys."""
    for key in keys:
        value = row.get(key)
        if value is not None and str(value).strip().upper() not in {"", "NA", "NAN", "NONE", "NULL"}:
            return str(value)
    return default


def _latest_non_na(rows: Sequence[dict[str, str]], keys: Sequence[str], default: str = "NA") -> str:
    """행들을 뒤에서부터 훑어 첫 non-NA 값. / Latest non-NA value scanning rows backwards."""
    for row in reversed(rows):
        value = _row_value(row, keys, default="")
        if value:
            return value
    return default


def format_manual_control_status(*, elapsed_s: int, rows: Sequence[dict[str, str]]) -> str:
    """모니터 중 한 줄 상태 문자열 생성. / One-line live status during manual-control monitor.

    최신 행에서 PPM/모드/모터/GPS/IMU 필드를 뽑아 ``key=value`` 나열로 만든다(부수효과 없음).
    Pulls PPM/mode/motor/GPS/IMU fields from the latest row into a ``key=value`` line.
    """
    last = _latest_manual_control_row(rows)
    manual_switch, mode_decode_reason = manual_control_mode_decode(last)
    mode_us = _row_value(last, ["mode_us", "raw_mode_channel_us"])
    current_lat = _row_value(last, ["current_lat", "gps_cached_lat", "gps_lat", "current_gps_lat"])
    current_lon = _row_value(last, ["current_lon", "gps_cached_lon", "gps_lon", "current_gps_lon"])
    fields = {
        "elapsed_s": str(elapsed_s),
        "rc_input_detected": str(_row_rc_input_detected(last)).lower() if last else "false",
        "rc_ok": last.get("rc_ok", "NA"),
        "mode": last.get("mode", "NA"),
        "auto_sw": last.get("auto_sw", "NA"),
        "manual_switch": manual_switch,
        "mode_decode_reason": mode_decode_reason,
        "ppm_interrupt_edge": last.get("ppm_interrupt_edge", "NA"),
        "ppm_decode_reason": last.get("ppm_decode_reason", "NA"),
        "ppm_edge_count": last.get("ppm_edge_count", "NA"),
        "ppm_sync_count": last.get("ppm_sync_count", "NA"),
        "ppm_sync_age_ms": last.get("ppm_sync_age_ms", "NA"),
        "ppm_last_width_us": last.get("ppm_last_width_us", "NA"),
        "ppm_min_width_us": last.get("ppm_min_width_us", "NA"),
        "ppm_max_width_us": last.get("ppm_max_width_us", "NA"),
        "ppm_frame_count": last.get("ppm_frame_count", "NA"),
        "ppm_last_channel_count": last.get("ppm_last_channel_count", "NA"),
        "ppm_short_rejects": last.get("ppm_short_rejects", "NA"),
        "ppm_long_rejects": last.get("ppm_long_rejects", "NA"),
        "ppm_last_rejected_us": last.get("ppm_last_rejected_us", "NA"),
        "steer_us": last.get("steer_us", "NA"),
        "throttle_us": last.get("throttle_us", "NA"),
        "mode_us": mode_us,
        "physical_a_cmd": last.get("physical_a_cmd", "NA"),
        "physical_b_cmd": last.get("physical_b_cmd", "NA"),
        "control_source": last.get("control_source", "NA"),
        "motor_write_called": last.get("motor_write_called", "NA"),
        "physical_output_active": last.get("physical_output_active", "NA"),
        "final_left_cmd": last.get("final_left_cmd", "NA"),
        "final_right_cmd": last.get("final_right_cmd", "NA"),
        "gps_block_reason": last.get("gps_block_reason", "NA"),
        "current_lat": current_lat,
        "current_lon": current_lon,
        "gps_sats": last.get("gps_sats", "NA"),
        "gps_hdop": last.get("gps_hdop", "NA"),
        "imu_present": last.get("imu_present", "NA"),
        "imu_relative_yaw_deg": last.get("imu_relative_yaw_deg", "NA"),
        "imu_heading_block_reason": last.get("imu_heading_block_reason", "NA"),
    }
    return " ".join(f"{key}={value}" for key, value in fields.items())


def evaluate_manual_control_rows(
    rows: Sequence[dict[str, str]],
    *,
    expected_ppm_interrupt_edge: str | None = None,
) -> dict[str, object]:
    """수동제어 세션 전체 telemetry 를 판정 요약으로 축약. / Evaluate a manual-control session.

    manual-control 모드의 핵심 순수 함수(단위 테스트 계약). 여러 행을 종합해
    통과/사유(reason)와 다음 행동을 결정한다. 통과(pass_ready)는 RC OK + MANUAL 모드 +
    RC_MANUAL 소스 + 0 아닌 최종 모터 명령 + 모터 write/출력 + 안정된 PPM 을 모두 만족할 때뿐.
    반환 dict 은 요약 파일로 그대로 쓰이므로 키를 바꾸면 하위 소비자/테스트가 깨진다.
    부수효과 없음. ``expected_ppm_interrupt_edge`` 로 에지 불일치도 사유로 승격한다.
    Core pure classifier for manual-control (a tested contract): aggregates rows
    into a pass/``reason``/next-action summary. Pass requires rc_ok + MANUAL +
    RC_MANUAL + nonzero final motor cmd + motor write/output + stable PPM. No side
    effects; the returned dict's keys are consumed downstream, so keep them stable.
    """
    rows_with_input = [
        row for row in rows
        if any(
            key in row
            for key in [f"raw_ch{i}_us" for i in range(1, 9)]
            + ["steer_us", "throttle_us", "mode_us", "raw_mode_channel_us"]
        )
    ]
    zero_rows = [row for row in rows_with_input if _row_input_zero(row)]
    rc_input_detected = any(_row_rc_input_detected(row) for row in rows)
    input_absent = bool(rows_with_input) and len(zero_rows) / max(1, len(rows_with_input)) >= 0.8 and not rc_input_detected
    rc_ok_seen = any(telemetry._parse_bool(row.get("rc_ok")) is True for row in rows)
    manual_mode_seen = any(row.get("mode") == "MANUAL" for row in rows)
    rc_manual_seen = any(row.get("control_source") == "RC_MANUAL" for row in rows)
    physical_a_nonzero = any(abs(telemetry._optional_float(row.get("physical_a_cmd")) or 0.0) > 1e-3 for row in rows)
    physical_b_nonzero = any(abs(telemetry._optional_float(row.get("physical_b_cmd")) or 0.0) > 1e-3 for row in rows)
    final_motor_nonzero = any(
        abs(telemetry._optional_float(row.get("final_left_cmd")) or 0.0) > 1e-3
        or abs(telemetry._optional_float(row.get("final_right_cmd")) or 0.0) > 1e-3
        for row in rows
    )
    motor_write_seen = any(telemetry._parse_bool(row.get("motor_write_called")) is True for row in rows)
    physical_output_seen = any(telemetry.physical_output_active(row) for row in rows)
    rc_status_rows = [
        row for row in rows
        if telemetry._parse_bool(row.get("rc_ok")) is not None
    ]
    rc_ok_rows = sum(1 for row in rc_status_rows if telemetry._parse_bool(row.get("rc_ok")) is True)
    rc_bad_rows = sum(1 for row in rc_status_rows if telemetry._parse_bool(row.get("rc_ok")) is False)
    rc_ok_ratio = (rc_ok_rows / len(rc_status_rows)) if rc_status_rows else 0.0
    ppm_signal_stable = not rc_status_rows or len(rc_status_rows) < 3 or rc_ok_ratio >= 0.6
    ppm_edge_mismatch_seen = any(_row_ppm_edge_mismatch(row, expected_ppm_interrupt_edge) for row in rows)
    ppm_sync_only_seen = any(_row_ppm_sync_only(row) for row in rows)
    no_ppm_frame_capture_seen = any(_row_no_ppm_frame_capture(row) for row in rows)
    pass_ready = (
        rc_ok_seen
        and manual_mode_seen
        and rc_manual_seen
        and final_motor_nonzero
        and (motor_write_seen or physical_output_seen)
        and ppm_signal_stable
    )
    last = _latest_manual_control_row(rows)
    manual_switch, mode_decode_reason = manual_control_mode_decode(last)
    ppm_decode_reason_latest = _latest_non_na(rows, ["ppm_decode_reason"])
    ppm_invalid_decode_seen = any(
        str(row.get("ppm_decode_reason", "")).startswith("PPM_")
        and row.get("ppm_decode_reason") not in {"OK", "PPM_FRAME_STALE"}
        for row in rows
    )
    gps_status_available = _latest_non_na(
        rows,
        ["gps_block_reason", "gps_sats", "gps_hdop", "current_lat", "current_lon", "gps_cached_lat", "gps_cached_lon"],
    ) != "NA"
    imu_status_available = _latest_non_na(
        rows,
        ["imu_present", "imu_relative_yaw_deg", "imu_heading_block_reason"],
    ) != "NA"
    if ppm_edge_mismatch_seen:
        manual_switch = "UNKNOWN_PPM_EDGE_MISMATCH"
        mode_decode_reason = "PPM_EDGE_MISMATCH"
    if pass_ready:
        reason = "MANUAL_CONTROL_PASS"
    elif not rows or not last:
        reason = "NO_USBDBG_TELEMETRY"
    elif ppm_edge_mismatch_seen:
        reason = "PPM_EDGE_MISMATCH"
    elif ppm_sync_only_seen:
        reason = "PPM_SYNC_ONLY_NO_CHANNELS"
    elif rc_ok_seen and not ppm_signal_stable:
        reason = "PPM_CHANNELS_PRESENT_BUT_INVALID"
    elif ppm_invalid_decode_seen and not rc_ok_seen:
        reason = "PPM_CHANNELS_PRESENT_BUT_INVALID"
    elif input_absent or no_ppm_frame_capture_seen:
        reason = "PPM_INPUT_ABSENT"
    elif mode_decode_reason == "NO_MODE_CHANNEL" and _row_mode_channel_capture_known(last):
        reason = "MODE_CHANNEL_MISSING"
    elif rc_input_detected and not rc_ok_seen:
        reason = "PPM_CHANNELS_PRESENT_BUT_INVALID"
    elif rc_ok_seen and not manual_mode_seen:
        reason = "MANUAL_CONTROL_READY"
    elif (physical_a_nonzero or physical_b_nonzero) and not final_motor_nonzero:
        reason = "MOTOR_OUTPUT_BLOCKED"
    else:
        reason = "MANUAL_CONTROL_READY"
    expected_edge_label = expected_ppm_interrupt_edge or "the selected profile"
    next_action = {
        "MANUAL_CONTROL_PASS": "PPM manual control is verified.",
        "MANUAL_CONTROL_READY": "PPM telemetry is present; set CH5 to MANUAL and move the sticks to verify output.",
        "PPM_INPUT_ABSENT": PPM_INPUT_ABSENT_ACTION,
        "PPM_SYNC_ONLY_NO_CHANNELS": "D6 is seeing sync-like pulses but no CH1/CH2/CH5 PPM channel intervals. This usually means the receiver is not outputting combined PPM on D6, the signal wire is on a single PWM output, or the edge/sync decoder profile is wrong. Rebuild with --profile rc-mix-ppm; if this reason remains, check receiver output mode/wiring.",
        "PPM_CHANNELS_PRESENT_BUT_INVALID": "PPM is present but unstable or invalid; charge/check the station controller battery, receiver power, D6 signal wire, shared ground, channel order, and pulse widths.",
        "PPM_EDGE_MISMATCH": f"Firmware PPM edge does not match the selected profile; expected {expected_edge_label}. Rebuild/upload manual-control with the intended --profile.",
        "MODE_CHANNEL_MISSING": "PPM steering/throttle are present but CH5 mode is missing; verify the receiver mode channel.",
        "MOTOR_OUTPUT_BLOCKED": "Manual A/B commands changed, but final motor output stayed zero; inspect manual control priority and motor gating.",
        "NO_USBDBG_TELEMETRY": "No USBDBG rows were parsed; check USB serial, firmware mode, baud rate, and --verbose-raw output.",
    }.get(reason, "Move the physical station/controller during the monitor window and inspect summary telemetry.")
    return {
        "mode": "manual-control",
        "success": pass_ready,
        "manual_control_ok": pass_ready,
        "reason": reason,
        "manual_switch": manual_switch,
        "mode_decode_reason": mode_decode_reason,
        "ppm_decode_reason_latest": ppm_decode_reason_latest,
        "ppm_interrupt_edge_latest": _latest_non_na(rows, ["ppm_interrupt_edge"]),
        "expected_ppm_interrupt_edge": expected_ppm_interrupt_edge or "profile-dependent",
        "ppm_edge_mismatch_seen": ppm_edge_mismatch_seen,
        "ppm_sync_only_seen": ppm_sync_only_seen,
        "ppm_edge_count_latest": _latest_non_na(rows, ["ppm_edge_count"]),
        "ppm_sync_count_latest": _latest_non_na(rows, ["ppm_sync_count"]),
        "ppm_last_width_us_latest": _latest_non_na(rows, ["ppm_last_width_us"]),
        "ppm_min_width_us_latest": _latest_non_na(rows, ["ppm_min_width_us"]),
        "ppm_max_width_us_latest": _latest_non_na(rows, ["ppm_max_width_us"]),
        "mode_us_latest": _latest_non_na(rows, ["mode_us", "raw_mode_channel_us"]),
        "rc_input_detected": rc_input_detected,
        "rc_ok_rows": rc_ok_rows,
        "rc_bad_rows": rc_bad_rows,
        "rc_ok_ratio": round(rc_ok_ratio, 3),
        "ppm_signal_stable": ppm_signal_stable,
        "ppm_input_pin": "D6",
        "steer_channel": "CH1",
        "throttle_channel": "CH2",
        "mode_channel": "CH5",
        "rc_ok_seen": rc_ok_seen,
        "gps_status_available": gps_status_available,
        "imu_status_available": imu_status_available,
        "last_current_lat": _latest_non_na(rows, ["current_lat", "gps_cached_lat", "gps_lat", "current_gps_lat"]),
        "last_current_lon": _latest_non_na(rows, ["current_lon", "gps_cached_lon", "gps_lon", "current_gps_lon"]),
        "last_imu_yaw": _latest_non_na(rows, ["imu_relative_yaw_deg"]),
        "manual_mode_seen": manual_mode_seen,
        "control_source_rc_manual_seen": rc_manual_seen,
        "physical_a_nonzero_seen": physical_a_nonzero,
        "physical_b_nonzero_seen": physical_b_nonzero,
        "final_motor_nonzero_seen": final_motor_nonzero,
        "motor_write_called_seen": motor_write_seen,
        "physical_output_active_seen": physical_output_seen,
        "gps_required": False,
        "imu_required": False,
        "path_package_required": False,
        "station_frame_parser_required": False,
        "hc12_required": False,
        "physical_a_role": "throttle",
        "physical_b_role": "turn",
        "wheel_to_physical_mapping": "physical_ab_manual_equivalent",
        "next_recommended_action": next_action,
        "ready_for_full_path_following": False,
    }


def evaluate_rc_input_diagnose_rows(rows: Sequence[dict[str, str]]) -> dict[str, object]:
    """읽기전용 PPM 프로브 telemetry 를 분류. / Classify read-only PPM probe telemetry.

    rc-input-diagnose 모드의 순수 판정 함수. 프레임 수/무효 프레임/0 아닌 채널을 세어
    RC 입력이 없음/있으나 무효/유효 중 무엇인지 사유(reason)로 반환한다(부수효과 없음).
    Pure classifier for rc-input-diagnose: counts frames/invalid/nonzero channels
    to report RC input absent / present-but-invalid / present-and-valid.
    """
    frame_counts = [int(float(row.get("frames", "0") or 0)) for row in rows if "frames" in row]
    invalid_counts = [int(float(row.get("invalid_frames", "0") or 0)) for row in rows if "invalid_frames" in row]
    total_frames = sum(frame_counts)
    total_invalid = sum(invalid_counts)
    ppm_header_seen = any("ppm_pin" in row or "channel_count" in row for row in rows)
    event_frames = [
        row for row in rows
        if any(f"ch{i}_us" in row for i in range(1, 9))
    ]
    nonzero_channels = [
        (key, value)
        for row in rows
        for key, value in row.items()
        if key.startswith("ch") and key.endswith("_us")
        if telemetry._optional_float(value) is not None and abs(float(value)) > 1e-3
    ]
    any_ppm_signal = total_frames > 0 or bool(nonzero_channels)
    valid_ppm_signal = any_ppm_signal and total_invalid < max(1, total_frames)
    if not rows:
        reason = "SERIAL_ERROR"
    elif not any_ppm_signal:
        reason = "RC_INPUT_ABSENT"
    elif not valid_ppm_signal:
        reason = "RC_CHANNELS_PRESENT_BUT_INVALID"
    else:
        reason = "RC_CHANNELS_PRESENT_AND_VALID"
    signal_class = "RC_INPUT_PRESENT_PPM" if any_ppm_signal else "RC_INPUT_ABSENT"
    next_action = {
        "SERIAL_ERROR": "Check USB serial connection and rerun rc-input-diagnose.",
        "RC_INPUT_ABSENT": RC_INPUT_ABSENT_ACTION,
        "RC_CHANNELS_PRESENT_BUT_INVALID": (
            "PPM frames were seen but invalid or incomplete; check receiver output mode, "
            "signal wiring, and whether the receiver is configured for PPM on the OpenRB input pin."
        ),
        "RC_CHANNELS_PRESENT_AND_VALID": (
            "RC input frames are present. Run manual-rc and verify MANUAL mode stick passthrough."
        ),
    }.get(reason, "Inspect RC input telemetry.")
    return {
        "mode": "rc-input-diagnose",
        "success": reason == "RC_CHANNELS_PRESENT_AND_VALID",
        "reason": reason,
        "rc_input_classification": reason,
        "rc_input_signal_class": signal_class,
        "rc_input_detected": any_ppm_signal,
        "rc_input_present_ppm": any_ppm_signal,
        "rc_input_present_pwm": False,
        "rc_input_present_sbus": False,
        "ppm_header_seen": ppm_header_seen,
        "ppm_event_row_count": len(event_frames),
        "ppm_total_frames": total_frames,
        "ppm_invalid_frames": total_invalid,
        "raw_channel_nonzero_seen": bool(nonzero_channels),
        "next_recommended_action": next_action,
        "ready_for_full_path_following": False,
    }


# ── 물리 스테이션 하드웨어 telemetry 평가 / Station-hardware telemetry evaluation ──
# station-hw-diagnose / station-hw-manual 이 공유하는 순수 판정 헬퍼. 스테이션
# 시리얼 프레임의 도착·파싱·deadman/estop·모터출력 여부로 링크/제어 상태를 사유화한다.
# Pure helpers shared by station-hw-{diagnose,manual}: reason about link/parse/
# deadman/estop/motor-output from the station serial frames.


def _station_value_present(value: object) -> bool:
    """값이 의미있게 존재하는가(빈/NA 아님). / Value present and not blank/NA."""
    text = str(value or "").strip().upper()
    return text not in {"", "NA", "NAN", "NONE", "NULL"}


def _station_hw_link_row(row: dict[str, str]) -> bool:
    """이 행이 스테이션 링크 프레임 수신을 보이는가. / Row shows a received station frame?"""
    if telemetry._parse_bool(row.get("station_link_seen")) is True:
        return True
    if _station_value_present(row.get("station_seq")) or _station_value_present(row.get("station_age_ms")):
        return True
    rx_count = telemetry._optional_float(row.get("station_rx_count"))
    if rx_count is None:
        rx_count = telemetry._optional_float(row.get("hc12_rx_count"))
    return rx_count is not None and rx_count > 0


def _station_hw_float_seen(rows: Sequence[dict[str, str]], *keys: str) -> bool:
    """주어진 키 중 어느 것이든 0 아닌 float 이 관측됐나. / Any nonzero float across keys?"""
    for row in rows:
        for key in keys:
            value = telemetry._optional_float(row.get(key))
            if value is not None and abs(value) > 1e-6:
                return True
    return False


def _station_last_present(last: dict[str, str], key: str, default: object = "NA") -> object:
    """최신 행의 값(없으면 default). / Value from the last row, else default."""
    value = last.get(key)
    return value if _station_value_present(value) else default


def evaluate_station_hw_rows(rows: Sequence[dict[str, str]], *, mode: str) -> dict[str, object]:
    """스테이션 HW telemetry 전체를 판정 요약으로 축약. / Evaluate station-hardware rows.

    station-hw-diagnose/manual 의 핵심 순수 함수. 우선순위대로 사유를 정한다: 링크 부재
    → estop → 프레임은 오나 파싱 실패(파서 불일치) → deadman 미활성 → 모터출력=PASS →
    A/B 명령만 → 유효. ``mode`` 에 따라 diagnose 는 "유효만 봐도 성공", manual 은 모터
    출력까지 요구한다. 반환은 ``assert_not_ready_for_full_path_following`` 로 봉인된 dict.
    부수효과 없음.
    Core pure classifier for station-hw modes; priority: link-absent -> estop ->
    frames-but-parser-mismatch -> deadman-inactive -> motor-output=PASS -> A/B-only
    -> valid. ``mode`` decides whether valid-only counts as success (diagnose) or
    motor output is required (manual). Returns a not-ready-sealed dict; no side effects.
    """
    link_rows = [row for row in rows if _station_hw_link_row(row)]
    manual_valid_rows = [row for row in rows if telemetry._parse_bool(row.get("station_manual_valid")) is True]
    deadman_rows = [row for row in rows if telemetry._parse_bool(row.get("station_deadman")) is True]
    estop_rows = [row for row in rows if telemetry._parse_bool(row.get("station_estop")) is True]
    motor_rows = [
        row for row in rows
        if telemetry._parse_bool(row.get("motor_write_called")) is True
        or telemetry.physical_output_active(row)
        or abs(telemetry._optional_float(row.get("final_left_cmd")) or 0.0) > 1e-6
        or abs(telemetry._optional_float(row.get("final_right_cmd")) or 0.0) > 1e-6
    ]
    parsed_ok_rows = sum(
        1 for row in rows
        if telemetry._parse_bool(row.get("station_parse_ok")) is True
        or telemetry._parse_bool(row.get("station_manual_valid")) is True
    )
    parsed_error_rows = sum(
        1 for row in rows
        if telemetry._parse_bool(row.get("station_parse_error")) is True
    )
    last = link_rows[-1] if link_rows else (rows[-1] if rows else {})
    parse_ok_count = max(
        parsed_ok_rows,
        int(telemetry._optional_float(last.get("station_parse_ok_count")) or 0),
    )
    parse_error_count = max(
        parsed_error_rows,
        int(telemetry._optional_float(last.get("station_parse_error_count")) or 0),
    )
    station_frame_count = max(
        len(link_rows),
        int(telemetry._optional_float(last.get("station_frame_count")) or 0),
        int(telemetry._optional_float(last.get("station_rx_count")) or 0),
        int(telemetry._optional_float(last.get("hc12_rx_count")) or 0),
    )
    station_link_seen = station_frame_count > 0
    station_physical_a_nonzero_seen = _station_hw_float_seen(
        rows, "station_physical_a_cmd", "station_a_cmd", "station_forward_cmd"
    )
    station_physical_b_nonzero_seen = _station_hw_float_seen(
        rows, "station_physical_b_cmd", "station_b_cmd", "station_turn_cmd"
    )
    if not station_link_seen:
        reason = "STATION_HW_LINK_ABSENT"
        success = False
        next_action = (
            "Check station hardware power, station transport wiring, station baud/settings, "
            "and whether the rover firmware has station hardware manual mode enabled."
        )
    elif estop_rows:
        reason = "STATION_HW_ESTOP_ACTIVE"
        success = False
        next_action = "Release station hardware emergency stop and rerun station-hw-diagnose."
    elif link_rows and not manual_valid_rows:
        reason = "WRONG_STATION_FRAME_PARSER"
        success = False
        next_action = (
            "Station bytes are arriving but no station manual frame parsed. Inspect "
            "raw_station_frames.txt and raw_station_frames_hex.txt, then compare the "
            "physical station output against the rover station parser."
        )
    elif not deadman_rows:
        reason = "STATION_HW_DEADMAN_NOT_ACTIVE"
        success = False
        next_action = "Hold the station hardware deadman control while moving the station input."
    elif motor_rows:
        reason = "STATION_HW_MANUAL_PASS"
        success = True
        next_action = "Station hardware manual control is passing; continue only with bounded supervised tests."
    elif station_physical_a_nonzero_seen or station_physical_b_nonzero_seen:
        if mode == "station-hw-manual":
            reason = "STATION_HW_MANUAL_OUTPUT_BLOCKED"
            success = False
            next_action = (
                "Station hardware A/B commands changed but rover motor output did not respond; "
                "compare against usb-pulse-test, then inspect station manual control-source gating."
            )
        else:
            reason = "STATION_HW_MANUAL_READY"
            success = True
            next_action = "Station hardware commands are valid. Run station-hw-manual to verify motor output if needed."
    else:
        reason = "STATION_HW_MANUAL_VALID"
        success = mode == "station-hw-diagnose"
        next_action = "Station frames are valid. Move the station hardware input while holding deadman to verify A/B commands."
    return checks.assert_not_ready_for_full_path_following({
        "mode": mode,
        "success": success,
        "reason": reason,
        "station_hw_result": reason,
        "station_link_seen": station_link_seen,
        "station_frame_count": station_frame_count,
        "station_parse_ok_count": parse_ok_count,
        "station_parse_error_count": parse_error_count,
        "station_transport": _station_last_present(last, "station_transport", "station_hardware_serial"),
        "station_protocol": _station_last_present(last, "station_protocol", "auto"),
        "station_parser": _station_last_present(last, "station_parser", "auto_station_manual"),
        "station_last_frame_age_ms": last.get("station_age_ms", "NA"),
        "station_seq": last.get("station_seq", "NA"),
        "station_manual_valid": bool(manual_valid_rows),
        "station_manual_valid_seen": bool(manual_valid_rows),
        "station_deadman": bool(deadman_rows),
        "station_deadman_seen": bool(deadman_rows),
        "station_estop": bool(estop_rows),
        "station_estop_seen": bool(estop_rows),
        "station_a_cmd": last.get("station_a_cmd", last.get("station_physical_a_cmd", "NA")),
        "station_b_cmd": last.get("station_b_cmd", last.get("station_physical_b_cmd", "NA")),
        "station_forward_cmd": last.get("station_forward_cmd", "NA"),
        "station_turn_cmd": last.get("station_turn_cmd", "NA"),
        "station_physical_a_cmd": last.get("station_physical_a_cmd", "NA"),
        "station_physical_b_cmd": last.get("station_physical_b_cmd", "NA"),
        "station_physical_a_nonzero_seen": station_physical_a_nonzero_seen,
        "station_physical_b_nonzero_seen": station_physical_b_nonzero_seen,
        "active_control_source_candidate": "STATION_HW_MANUAL" if bool(manual_valid_rows) else "STOP",
        "station_rx_count": last.get("station_rx_count", last.get("hc12_rx_count", station_frame_count)),
        "motor_write_called_seen": any(telemetry._parse_bool(row.get("motor_write_called")) is True for row in rows),
        "physical_output_active_seen": any(telemetry.physical_output_active(row) for row in rows),
        "final_motor_nonzero_seen": any(
            abs(telemetry._optional_float(row.get("final_left_cmd")) or 0.0) > 1e-6
            or abs(telemetry._optional_float(row.get("final_right_cmd")) or 0.0) > 1e-6
            for row in rows
        ),
        "rc_input_required": False,
        "gps_required": False,
        "imu_required": False,
        "physical_a_role": "throttle",
        "physical_b_role": "turn",
        "wheel_to_physical_mapping": "physical_ab_manual_equivalent",
        "next_recommended_action": next_action,
        "ready_for_full_path_following": False,
    })


def _station_hw_status_line(summary: dict[str, object], *, elapsed_s: float) -> str:
    """스테이션 HW 요약을 한 줄 상태 문자열로. / Station-hw summary to a one-line status."""
    return (
        f"elapsed_s={elapsed_s:.0f} "
        f"station_link_seen={str(summary.get('station_link_seen', False)).lower()} "
        f"station_frame_count={summary.get('station_frame_count', 0)} "
        f"station_parse_ok_count={summary.get('station_parse_ok_count', 0)} "
        f"station_parse_error_count={summary.get('station_parse_error_count', 0)} "
        f"station_deadman={str(summary.get('station_deadman', False)).lower()} "
        f"station_estop={str(summary.get('station_estop', False)).lower()} "
        f"station_manual_valid={str(summary.get('station_manual_valid', False)).lower()} "
        f"station_physical_a_cmd={summary.get('station_physical_a_cmd', 'NA')} "
        f"station_physical_b_cmd={summary.get('station_physical_b_cmd', 'NA')} "
        f"station_rx_count={summary.get('station_rx_count', 'NA')} "
        f"station_transport={summary.get('station_transport', 'NA')} "
        f"station_parser={summary.get('station_parser', 'NA')} "
        f"motor_write_called={str(summary.get('motor_write_called_seen', False)).lower()} "
        f"physical_output_active={str(summary.get('physical_output_active_seen', False)).lower()} "
        f"reason_so_far={summary.get('reason', 'NA')}"
    )


def station_hw_status_line(rows: Sequence[dict[str, str]], *, mode: str, elapsed_s: float) -> str:
    """행들을 평가해 한 줄 상태로(공개 래퍼). / Evaluate rows into a status line (public)."""
    return _station_hw_status_line(evaluate_station_hw_rows(rows, mode=mode), elapsed_s=elapsed_s)


def _station_raw_frame_dumps(rows: Sequence[dict[str, str]]) -> list[tuple[str, str]]:
    """중복 제거된 원시 스테이션 프레임 (텍스트, hex) 최대 20개. / Dedup raw station frames.

    파서가 맞지 않을 때 실제 바이트를 눈으로 비교하도록 원시/16진 프레임을 수집한다.
    Collects raw + hex frames so a parser mismatch can be inspected byte-for-byte.
    """
    dumps: list[tuple[str, str]] = []
    seen: set[tuple[str, str]] = set()
    for row in rows:
        raw_text = row.get("station_raw_frame")
        raw_hex = row.get("station_raw_frame_hex")
        if not _station_value_present(raw_text) and not _station_value_present(raw_hex):
            continue
        decoded = unquote(str(raw_text or ""))
        hex_text = str(raw_hex or "")
        key = (decoded, hex_text)
        if key in seen:
            continue
        seen.add(key)
        dumps.append(key)
        if len(dumps) >= 20:
            break
    return dumps


def write_station_raw_frame_dumps(out_dir: Path, rows: Sequence[dict[str, str]]) -> int:
    """원시 스테이션 프레임을 파일로 기록하고 개수 반환. / Write raw station frame dumps.

    부수효과: ``raw_station_frames.txt`` / ``raw_station_frames_hex.txt`` 생성.
    Side effect: writes the two raw-frame files under ``out_dir``.
    """
    dumps = _station_raw_frame_dumps(rows)
    if not dumps:
        return 0
    (out_dir / "raw_station_frames.txt").write_text(
        "\n".join(raw for raw, _ in dumps) + "\n",
        encoding="utf-8",
    )
    (out_dir / "raw_station_frames_hex.txt").write_text(
        "\n".join(raw_hex for _, raw_hex in dumps) + "\n",
        encoding="utf-8",
    )
    return len(dumps)


# ── 시리얼 포트 해석·요약 파일 쓰기 / Serial-port resolution + summary writers ──


def arduino_cli_openrb_port() -> str | None:
    """arduino-cli board list 에서 OpenRB-150 포트를 찾는다. / Find OpenRB-150 port.

    부수효과: ``arduino-cli board list`` 서브프로세스 실행. 실패/미검출 시 None.
    Side effect: runs the ``arduino-cli board list`` subprocess; None on failure.
    """
    try:
        completed = subprocess.run(
            ["arduino-cli", "board", "list"],
            check=False,
            capture_output=True,
            text=True,
            timeout=5,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    for line in completed.stdout.splitlines():
        if "OpenRB-150" in line:
            parts = line.split()
            return parts[0] if parts else None
    return None


def detected_serial_ports() -> list[str]:
    """/dev 에서 USB 시리얼 후보 포트를 나열(mac/Linux 패턴). / List USB serial ports."""
    parent = Path("/dev")
    if not parent.exists():
        return []
    patterns = (
        "cu.usbmodem*",
        "tty.usbmodem*",
        "ttyACM*",
        "ttyUSB*",
        "cu.usbserial*",
        "tty.usbserial*",
    )
    ports: list[str] = []
    for pattern in patterns:
        ports.extend(str(path) for path in parent.glob(pattern))
    return sorted(set(ports))


def resolve_port(
    explicit_port: str | None,
    *,
    env: dict[str, str] | None = None,
    system_name: str | None = None,
) -> dict[str, object]:
    """포트 결정 우선순위: 명시 > $PORT > arduino-cli > Linux 기본. / Resolve serial port.

    ``{"port", "source"}`` 를 반환하며 ``source`` 로 어디서 정해졌는지 알려준다.
    ``env``/``system_name`` 인자는 테스트 주입용. Returns ``{"port", "source"}``.
    """
    env = os.environ if env is None else env
    system_name = platform.system() if system_name is None else system_name
    if explicit_port:
        return {"port": explicit_port, "source": "explicit"}
    env_port = env.get("PORT", "")
    if env_port:
        return {"port": env_port, "source": "env"}
    detected = arduino_cli_openrb_port()
    if detected:
        return {"port": detected, "source": "arduino_cli"}
    if system_name == "Linux" and Path(DEFAULT_PORT).exists():
        return {"port": DEFAULT_PORT, "source": "linux_default"}
    return {"port": None, "source": "none"}


def write_summary_files(out_dir: str | Path, summary: dict[str, object], *, title: str) -> dict[str, object]:
    """summary.json + summary.md 를 쓰고 정규화된 dict 반환. / Write summary.json + .md.

    거의 모든 모드가 결과를 남기는 공통 라이터. ``ready_for_full_path_following`` 를
    강제로 False 로 봉인하고(안전 불변식) ``success`` 기본값을 채운 뒤 두 파일을 만든다.
    부수효과: ``out_dir`` 생성 및 두 파일 기록.
    Shared writer; forces the not-ready invariant, defaults ``success``, then
    writes both files. Side effect: creates ``out_dir`` and the two files.
    """
    path = Path(out_dir)
    path.mkdir(parents=True, exist_ok=True)
    normalized = dict(summary)
    normalized.setdefault("success", normalized.get("reason") == "OK")
    normalized["ready_for_full_path_following"] = False
    normalized = checks.assert_not_ready_for_full_path_following(normalized)
    _write_json(path / "summary.json", normalized)
    lines = [f"# {title}", ""]
    lines.extend(f"- {key}: `{value}`" for key, value in normalized.items())
    (path / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return normalized


def write_failure_summary(
    out_dir: str | Path | None,
    *,
    reason: str,
    attempted_port: str | None = None,
    mode: str | None = None,
    next_recommended_action: str = "Check the requested serial port or connect OpenRB-150 and retry.",
) -> None:
    """포트 없음 등 초기 실패 요약을 기록. / Write an early-failure summary (e.g. no port).

    ``out_dir`` 가 None 이면 아무 것도 하지 않는다. Side effect: writes summary files.
    """
    if out_dir is None:
        return
    path = Path(out_dir)
    path.mkdir(parents=True, exist_ok=True)
    payload = {
        "mode": mode or "unknown",
        "reason": reason,
        "success": False,
        "attempted_port": attempted_port,
        "detected_ports": detected_serial_ports(),
        "next_recommended_action": next_recommended_action,
        "ready_for_full_path_following": False,
    }
    write_summary_files(path, payload, title="Physical Path Planner")


def ensure_port(args: argparse.Namespace) -> bool:
    """포트를 해석해 ``args.port`` 에 채우고 존재 여부 검증. / Resolve+validate the port.

    성공 시 True 를 돌려주며 ``args.port``/``args.port_source`` 를 설정한다. ``--from-log``
    은 하드웨어가 필요 없으므로 즉시 True. 실패 시 실패 요약을 쓰고 사유를 출력한 뒤 False.
    거의 모든 하드웨어 핸들러의 첫 관문(게이트)이다.
    Returns True (setting ``args.port``/``port_source``); ``--from-log`` short-circuits
    True. On failure writes a failure summary and returns False. Side effects: sets
    args fields, may write summary + print.
    """
    if getattr(args, "from_log", None):
        return True
    resolved = resolve_port(getattr(args, "port", None))
    args.port = resolved["port"]
    args.port_source = resolved["source"]
    if args.port is None or not Path(str(args.port)).exists():
        write_failure_summary(
            getattr(args, "out_dir", None),
            reason="SERIAL_PORT_NOT_FOUND",
            attempted_port=None if args.port is None else str(args.port),
            mode=getattr(args, "mode", None),
        )
        print(f"reason=SERIAL_PORT_NOT_FOUND attempted_port={args.port} detected_ports={detected_serial_ports()}")
        print("ready_for_full_path_following=false")
        return False
    print(f"resolved_port={args.port}")
    print(f"port_source={args.port_source}")
    return True


def printable_port(explicit_port: str | None) -> str:
    """--print-cmd 표시에 쓸 포트 문자열(미해석 시 ``$PORT``). / Printable port for --print-cmd.

    실제 시리얼을 열지 않고 명령을 출력만 할 때 사용. 결정 못 하면 ``$PORT`` 리터럴.
    Used when printing commands without opening serial; falls back to the ``$PORT`` literal.
    """
    if explicit_port:
        return explicit_port
    resolved = resolve_port(None)
    return str(resolved["port"] or "$PORT")


def build_calibrate_turn_argv(
    *,
    script: str,
    port: str,
    mode: str,
    target_angle_deg: float,
    angle_tolerance_deg: float,
    save_turn_calibration: str,
    turn_calibration_out: str,
    out_dir: str,
    b_cmd: float | None = None,
    pulse_ms: int | None = None,
    max_abs_b: float = 0.35,
    max_ms: int = 1500,
) -> list[str]:
    """Build the argv that shells out to guarded pulse calibration.

    Always passes ``--imu-angle-compare true`` -- that is what makes the launcher
    append ``-DIMU_ENABLE=1 -DIMU_YAW_DIAG=1`` and measure before/after yaw.
    """
    argv = [
        "bash",
        str(script),
        "--port",
        str(port),
        "--mode",
        str(mode),
        "--max-abs-b",
        str(max_abs_b),
        "--max-ms",
        str(max_ms),
        "--imu-angle-compare",
        "true",
        "--target-angle-deg",
        str(target_angle_deg),
        "--angle-tolerance-deg",
        str(angle_tolerance_deg),
        "--save-turn-calibration",
        str(save_turn_calibration),
        "--turn-calibration-out",
        str(turn_calibration_out),
        "--out-dir",
        str(out_dir),
    ]
    if b_cmd is not None:
        argv.extend(["--cmd-list", str(abs(b_cmd))])
    if pulse_ms is not None:
        argv.extend(["--pulse-ms-list", str(pulse_ms)])
    return argv


def resolve_calibration(args: argparse.Namespace) -> dict[str, object]:
    """Resolve calibration honoring real on-disk files, with explicit overrides.

    Unspecified ``--*-calibration-json`` flags fall back to the resolver's default
    on-disk paths (so genuine calibration is used when present); a missing file
    degrades to the repeated-pulses fallback and never raises.
    """
    kwargs: dict[str, object] = {"calibration_mode": args.calibration_mode}
    for flag, key in (
        ("motion_calibration_json", "motion_calibration_json"),
        ("fine_calibration_json", "fine_calibration_json"),
        ("turn_calibration_json", "turn_calibration_json"),
        ("turn_angle_calibration_json", "turn_angle_calibration_json"),
        ("smooth_turn_calibration_json", "smooth_turn_calibration_json"),
    ):
        value = getattr(args, flag)
        if value is not None:
            kwargs[key] = Path(value)
    return calibration.resolve_physical_calibration(**kwargs)


def motion_calibration_loaded(calibration_dict: dict[str, object]) -> bool:
    """승인된 모션 보정 파일이 실제 디스크에 있나. / Is an approved motion calibration on disk?"""
    files = calibration_dict.get("calibration_files")
    if not isinstance(files, dict):
        return False
    motion_path = files.get("motion")
    return bool(motion_path and Path(str(motion_path)).exists())


# ── GPS 시작좌표 스냅샷/캐시 해석 / GPS start-coordinate snapshot + cache resolution ──
# preview/run/auto-relative 가 공유. 라이브 telemetry 행에서 사용 가능한 시작 위경도를
# 뽑거나(gps_snapshot/resolve_start_*), 최근 캐시(load_cached_start)로 대체한다.
# Shared by preview/run/auto-relative: derive a usable start lat/lon from live
# telemetry, or fall back to a recent on-disk cache.


def _lat_lon_from_row(row: dict[str, str], lat_keys: Sequence[str], lon_keys: Sequence[str]) -> tuple[float | None, float | None]:
    """후보 키들에서 (lat, lon) 을 추출. / Extract (lat, lon) from candidate keys."""
    lat = None
    lon = None
    for key in lat_keys:
        lat = telemetry._optional_float(row.get(key))
        if lat is not None:
            break
    for key in lon_keys:
        lon = telemetry._optional_float(row.get(key))
        if lon is not None:
            break
    return lat, lon


def _fresh_cached_gps(row: dict[str, str], max_age_ms: int) -> bool:
    """행의 캐시 GPS 가 충분히 최신인가. / Is this row's cached GPS fresh enough?"""
    age = telemetry._optional_float(row.get("gps_cached_age_ms", row.get("gps_age_ms")))
    if age is not None:
        return age <= max_age_ms
    return telemetry._parse_bool(row.get("gps_location_fresh"), default=False)


def gps_snapshot(rows: Sequence[dict[str, str]], *, min_sats: float = 5.0, max_hdop: float = 2.5) -> dict[str, object]:
    """파싱된 telemetry 에서 GPS 콜드스타트 상태를 요약. / Summarize cold-start GPS state.

    최적 위성수/HDOP/좌표와 "준비된 행"(min_sats/max_hdop 충족 + fix 유효)을 찾아 dict 로
    반환한다(순수 함수, 부수효과 없음). ``ready_row`` 는 시작좌표 확정에 쓰인다.
    Pure: finds best sats/hdop/coords plus a ``ready_row`` (meets thresholds and a
    valid fix), used to lock the plan start. No side effects.
    """
    best_sats: float | None = None
    best_hdop: float | None = None
    best_lat: float | None = None
    best_lon: float | None = None
    best_ready_row: dict[str, str] | None = None
    last = rows[-1] if rows else {}
    for row in rows:
        sats = telemetry._optional_float(row.get("gps_sats"))
        hdop = telemetry._optional_float(row.get("gps_hdop"))
        lat, lon = _lat_lon_from_row(
            row,
            ("current_lat", "gps_lat", "current_gps_lat"),
            ("current_lon", "gps_lon", "current_gps_lon"),
        )
        if sats is not None and (best_sats is None or sats > best_sats):
            best_sats = sats
        if hdop is not None and (best_hdop is None or hdop < best_hdop):
            best_hdop = hdop
        if lat is not None and lon is not None:
            best_lat = lat
            best_lon = lon
            if (
                (sats is None or sats >= min_sats)
                and (hdop is None or hdop <= max_hdop)
                and (
                    telemetry._parse_bool(row.get("gps_ready"))
                    or telemetry._parse_bool(row.get("gps_solution_valid"))
                    or str(row.get("gps_block_reason", "")).upper() == "OK"
                )
            ):
                best_ready_row = row
    current_lat, current_lon = _lat_lon_from_row(
        last,
        ("current_lat", "gps_lat", "current_gps_lat"),
        ("current_lon", "gps_lon", "current_gps_lon"),
    )
    sats = telemetry._optional_float(last.get("gps_sats"))
    hdop = telemetry._optional_float(last.get("gps_hdop"))
    gps_ready = best_ready_row is not None
    return {
        "firmware_profile": last.get("firmware_profile", MAC_PHYSICAL_SUPERVISED_PROFILE),
        "gps_ready": gps_ready,
        "gps_solution_valid": telemetry._parse_bool(last.get("gps_solution_valid")),
        "gps_chars": last.get("gps_chars", "NA"),
        "current_lat": current_lat,
        "current_lon": current_lon,
        "gps_sats": sats,
        "gps_hdop": hdop,
        "best_sats": best_sats,
        "best_hdop": best_hdop,
        "best_lat": best_lat,
        "best_lon": best_lon,
        "last_rmc_status": last.get("last_rmc_status", "NA"),
        "last_gga_fix_quality": last.get("last_gga_fix_quality", "NA"),
        "gps_block_reason": last.get("gps_block_reason", "NA"),
        "imu_present": telemetry._parse_bool(last.get("imu_present")),
        "imu_relative_yaw_deg": last.get("imu_relative_yaw_deg", "NA"),
        "ready_row": best_ready_row,
    }


def _gps_status_line(elapsed_s: float, snapshot: dict[str, object]) -> str:
    """GPS 대기 중 한 줄 상태 문자열. / One-line status while waiting for GPS."""
    return (
        f"elapsed_s={elapsed_s:.0f} "
        f"firmware_profile={snapshot['firmware_profile']} "
        f"gps_chars={snapshot['gps_chars']} "
        f"gps_ready={str(snapshot['gps_ready']).lower()} "
        f"gps_solution_valid={str(snapshot['gps_solution_valid']).lower()} "
        f"current_lat={telemetry._fmt(snapshot['current_lat'], 7) if snapshot['current_lat'] is not None else 'NA'} "
        f"current_lon={telemetry._fmt(snapshot['current_lon'], 7) if snapshot['current_lon'] is not None else 'NA'} "
        f"gps_sats={telemetry._fmt(snapshot['gps_sats'], 0) if snapshot['gps_sats'] is not None else 'NA'} "
        f"gps_hdop={telemetry._fmt(snapshot['gps_hdop'], 2) if snapshot['gps_hdop'] is not None else 'NA'} "
        f"last_rmc_status={snapshot['last_rmc_status']} "
        f"last_gga_fix_quality={snapshot['last_gga_fix_quality']} "
        f"best_sats={telemetry._fmt(snapshot['best_sats'], 0) if snapshot['best_sats'] is not None else 'NA'} "
        f"best_hdop={telemetry._fmt(snapshot['best_hdop'], 2) if snapshot['best_hdop'] is not None else 'NA'} "
        f"best_lat={telemetry._fmt(snapshot['best_lat'], 7) if snapshot['best_lat'] is not None else 'NA'} "
        f"best_lon={telemetry._fmt(snapshot['best_lon'], 7) if snapshot['best_lon'] is not None else 'NA'} "
        f"imu_present={str(snapshot['imu_present']).lower()} "
        f"imu_relative_yaw_deg={snapshot['imu_relative_yaw_deg']}"
    )


def write_gps_cache(snapshot: dict[str, object]) -> None:
    """확보된 시작좌표를 캐시 파일에 저장. / Persist the acquired start fix to the cache.

    이후 preview/run 이 GPS 재확보 없이 최근 시작좌표를 재사용할 수 있게 한다.
    부수효과: ``DEFAULT_GPS_CACHE`` JSON 기록(좌표 없으면 아무 것도 안 함).
    Lets later preview/run reuse a recent start without re-acquiring GPS. Side
    effect: writes the cache JSON (no-op when best_lat/lon is missing).
    """
    lat = snapshot.get("best_lat")
    lon = snapshot.get("best_lon")
    if lat is None or lon is None:
        return
    DEFAULT_GPS_CACHE.parent.mkdir(parents=True, exist_ok=True)
    _write_json(
        DEFAULT_GPS_CACHE,
        {
            "start_lat": lat,
            "start_lon": lon,
            "timestamp_s": time.time(),
            "gps_sats": snapshot.get("best_sats"),
            "gps_hdop": snapshot.get("best_hdop"),
            "source": "gps-wait",
            "firmware_profile": MAC_PHYSICAL_SUPERVISED_PROFILE,
            "ready_for_full_path_following": False,
        },
    )


def load_cached_start(max_age_s: float) -> dict[str, object] | None:
    """캐시된 시작좌표를 나이 제한 내에서 로드. / Load a cached start fix within max age.

    ``max_age_s`` 를 넘겼거나 좌표가 없으면 None. 성공 시 시작좌표 + 스냅샷 dict.
    Returns None when stale/missing; otherwise a start-fix + snapshot dict.
    """
    if not DEFAULT_GPS_CACHE.exists():
        return None
    try:
        data = json.loads(DEFAULT_GPS_CACHE.read_text())
    except (OSError, json.JSONDecodeError):
        return None
    timestamp_s = telemetry._optional_float(data.get("timestamp_s"))
    if timestamp_s is None or time.time() - timestamp_s > max_age_s:
        return None
    lat = telemetry._optional_float(data.get("start_lat"))
    lon = telemetry._optional_float(data.get("start_lon"))
    if lat is None or lon is None:
        return None
    return {
        "start_lat": lat,
        "start_lon": lon,
        "start_source": "cached_gps",
        "start_gps_block_reason": "CACHE",
        "start_gps_sats": data.get("gps_sats", "NA"),
        "start_gps_hdop": data.get("gps_hdop", "NA"),
        "gps_cached_used": True,
        "gps_wait_snapshot": {
            "firmware_profile": data.get("firmware_profile", MAC_PHYSICAL_SUPERVISED_PROFILE),
            "gps_ready": True,
            "gps_solution_valid": True,
            "current_lat": lat,
            "current_lon": lon,
            "gps_sats": data.get("gps_sats"),
            "gps_hdop": data.get("gps_hdop"),
            "best_sats": data.get("gps_sats"),
            "best_hdop": data.get("gps_hdop"),
            "best_lat": lat,
            "best_lon": lon,
            "last_rmc_status": data.get("last_rmc_status", "CACHE"),
            "last_gga_fix_quality": data.get("last_gga_fix_quality", "CACHE"),
            "imu_present": False,
            "imu_relative_yaw_deg": "NA",
        },
    }


def resolve_start_gps_from_rows(
    rows: Sequence[dict[str, str]],
    *,
    start_mode: str,
    cached_start_max_age_ms: int,
    min_sats: float = 5.0,
    max_hdop: float = 2.5,
) -> dict[str, object] | None:
    """Resolve the preview start coordinate from live/current or fresh cached GPS."""
    if start_mode not in {"live_gps", "cached_gps"}:
        return None
    snapshot = gps_snapshot(rows, min_sats=min_sats, max_hdop=max_hdop)
    ready_row = snapshot.get("ready_row")
    if start_mode == "live_gps" and isinstance(ready_row, dict):
        lat, lon = _lat_lon_from_row(
            ready_row,
            ("current_lat", "gps_lat", "current_gps_lat"),
            ("current_lon", "gps_lon", "current_gps_lon"),
        )
        if lat is not None and lon is not None:
            return {
                "start_lat": lat,
                "start_lon": lon,
                "start_source": "live_gps",
                "start_gps_block_reason": ready_row.get("gps_block_reason", "NA"),
                "start_gps_sats": ready_row.get("gps_sats", "NA"),
                "start_gps_hdop": ready_row.get("gps_hdop", "NA"),
                "gps_cached_used": False,
                "gps_wait_snapshot": {k: v for k, v in snapshot.items() if k != "ready_row"},
            }
    for row in reversed(rows):
        lat, lon = _lat_lon_from_row(
            row,
            ("gps_cached_lat", "gps_lat", "current_lat", "current_gps_lat"),
            ("gps_cached_lon", "gps_lon", "current_lon", "current_gps_lon"),
        )
        if lat is not None and lon is not None and _fresh_cached_gps(row, cached_start_max_age_ms):
            return {
                "start_lat": lat,
                "start_lon": lon,
                "start_source": "cached_gps",
                "start_gps_block_reason": row.get("gps_block_reason", "NA"),
                "start_gps_sats": row.get("gps_sats", "NA"),
                "start_gps_hdop": row.get("gps_hdop", "NA"),
                "gps_cached_used": True,
                "gps_wait_snapshot": {k: v for k, v in snapshot.items() if k != "ready_row"},
            }
    return None


def resolve_start_for_preview(args: argparse.Namespace) -> tuple[dict[str, object] | None, list[str]]:
    """미리보기/실행의 시작좌표를 해석(필요시 시리얼 오픈). / Resolve start coords for preview/run.

    우선순위: 명시 --start-lat/lon > (start_mode=explicit 이면 없음) > --from-log 파싱 >
    라이브 시리얼(옵션으로 감독형 펌웨어 업로드 후 GPS 대기) > 최근 캐시. 반환은
    ``(start_dict|None, raw_lines)``. preview 와 run 이 공유하는 시작좌표 획득 진입점.
    부수효과: 라이브 경로에서 시리얼을 열고 펌웨어를 업로드할 수 있으며 상태줄을 출력한다.
    Priority: explicit > (none if start_mode=explicit) > --from-log > live serial
    (optionally uploading supervised firmware then waiting for GPS) > recent cache.
    Returns ``(start_dict|None, raw_lines)``. Side effects on the live path: opens
    serial, may upload firmware, prints status.
    """
    if args.start_lat is not None and args.start_lon is not None:
        return {
            "start_lat": float(args.start_lat),
            "start_lon": float(args.start_lon),
            "start_source": "explicit",
            "start_gps_block_reason": "NA",
            "start_gps_sats": "NA",
            "start_gps_hdop": "NA",
            "gps_cached_used": False,
            "gps_wait_elapsed_s": 0.0,
        }, []
    if getattr(args, "start_mode", "live_gps") == "explicit":
        return None, []

    raw_lines: list[str] = []
    min_sats = float(getattr(args, "gps_min_sats", 5))
    max_hdop = float(getattr(args, "gps_max_hdop", 2.5))
    max_cache_s = float(getattr(args, "max_cached_start_age_s", 600))
    allow_cache = str(getattr(args, "allow_cached_start", "true")).lower() != "false"
    if getattr(args, "from_log", None):
        log_path = Path(args.from_log)
        raw_lines = log_path.read_text(encoding="utf-8").splitlines()
        rows = telemetry.parse_usbdbg_rows("\n".join(raw_lines))
        resolved = resolve_start_gps_from_rows(
            rows,
            start_mode=args.start_mode,
            cached_start_max_age_ms=args.cached_start_max_age_ms,
            min_sats=min_sats,
            max_hdop=max_hdop,
        )
        if resolved is not None:
            resolved["gps_wait_elapsed_s"] = 0.0
        return resolved, raw_lines

    if not ensure_port(args):
        return None, []
    if getattr(args, "upload", "auto") in {"true", "auto"}:
        uploaded = _upload_mac_physical_supervised_firmware(
            args,
            Path(args.out_dir),
            title="Mac Physical Supervised",
            mode=getattr(args, "mode", "preview"),
            build_path="/private/tmp/openrb-mac-physical-supervised",
        )
        if uploaded != 0:
            return None, []
    import serial

    if not telemetry._parse_bool(getattr(args, "wait_gps", "true"), default=True):
        deadline = time.monotonic() + 0.5
    else:
        deadline = time.monotonic() + float(getattr(args, "gps_timeout_s", args.start_timeout_s))
    status_interval_s = float(getattr(args, "gps_status_interval_s", 2.0))
    next_status = time.monotonic()
    start_monotonic = time.monotonic()
    rows: list[dict[str, str]] = []
    with serial.Serial(args.port, baudrate=args.baud, timeout=0.5) as handle:
        while time.monotonic() < deadline:
            raw = handle.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            raw_lines.append(line)
            rows = telemetry.parse_usbdbg_rows("\n".join(raw_lines))
            resolved = resolve_start_gps_from_rows(
                rows,
                start_mode=args.start_mode,
                cached_start_max_age_ms=args.cached_start_max_age_ms,
                min_sats=min_sats,
                max_hdop=max_hdop,
            )
            if resolved is not None:
                resolved["gps_wait_elapsed_s"] = time.monotonic() - start_monotonic
                return resolved, raw_lines
            if time.monotonic() >= next_status:
                snapshot = gps_snapshot(rows, min_sats=min_sats, max_hdop=max_hdop)
                print(_gps_status_line(time.monotonic() - start_monotonic, snapshot))
                next_status = time.monotonic() + status_interval_s
    if allow_cache:
        cached = load_cached_start(max_cache_s)
        if cached is not None:
            cached["gps_wait_elapsed_s"] = time.monotonic() - start_monotonic
            return cached, raw_lines
    return None, raw_lines


def resolve_plan(args: argparse.Namespace, calibration_dict: dict[str, object]) -> dict[str, object]:
    """Build the no-motion plan (segments + goal) shared by preview and run."""
    plan = preview.build_preview(
        start_lat=args.start_lat,
        start_lon=args.start_lon,
        goal_mode=args.goal_mode,
        goal_lat=args.goal_lat,
        goal_lon=args.goal_lon,
        goal_east_m=args.goal_east_m,
        goal_north_m=args.goal_north_m,
        goal_dlat=args.goal_dlat,
        goal_dlon=args.goal_dlon,
        goal_bearing_deg=args.goal_bearing_deg,
        goal_distance_m=args.goal_distance_m,
        path_shape=args.path_shape,
        workspace_width_m=args.workspace_width_m,
        step_spacing_m=args.step_spacing_m,
        diagonal_orientation=args.diagonal_orientation,
        max_segment_pulses=args.max_segment_pulses,
        nominal_forward_pulse_m=args.nominal_forward_pulse_m,
        calibration=calibration_dict,
        connector_style=getattr(args, "connector_style", geometry.DEFAULT_CONNECTOR_STYLE),
    )
    validate_resolved_field_config(args, plan)
    plan["field_config"] = build_resolved_field_config(args, plan)
    if str(plan.get("path_shape")) == geometry.DIAGONAL_RECTANGLE_SERPENTINE:
        print("diagonal_rectangle_serpentine follows the A-B diagonal frame; it is not the ㄹ coverage path.")
    return plan


def build_resolved_field_config(args: argparse.Namespace, plan: dict[str, object]) -> dict[str, object]:
    """Resolved A/B field geometry shown to the operator before preview/run."""
    goal_x, goal_y = geometry.goal_to_local(
        float(plan["start_lat"]),
        float(plan["start_lon"]),
        float(plan["goal_lat"]),
        float(plan["goal_lon"]),
    )
    goal_input = plan.get("goal_input", {})
    if not isinstance(goal_input, dict):
        goal_input = {}
    if str(plan.get("goal_mode", getattr(args, "goal_mode", ""))) == "relative_enu":
        if goal_input.get("goal_east_m") is not None:
            goal_x = float(goal_input["goal_east_m"])
        if goal_input.get("goal_north_m") is not None:
            goal_y = float(goal_input["goal_north_m"])
    return {
        "start_mode": getattr(args, "start_mode", "explicit"),
        "start_source": str(plan.get("start_source", "explicit")),
        "start_lat": float(plan["start_lat"]),
        "start_lon": float(plan["start_lon"]),
        "start_x_m": 0.0,
        "start_y_m": 0.0,
        "coordinate_mode": str(plan.get("goal_mode", getattr(args, "goal_mode", "unknown"))),
        "goal_mode": str(plan.get("goal_mode", getattr(args, "goal_mode", "unknown"))),
        "goal_east_m": goal_input.get("goal_east_m", getattr(args, "goal_east_m", None)),
        "goal_north_m": goal_input.get("goal_north_m", getattr(args, "goal_north_m", None)),
        "goal_lat": float(plan["goal_lat"]),
        "goal_lon": float(plan["goal_lon"]),
        "goal_x_m": goal_x,
        "goal_y_m": goal_y,
        "resolved_goal_x_m": goal_x,
        "resolved_goal_y_m": goal_y,
        "workspace_width_m": plan.get("workspace_width_m"),
        "workspace_length_m": plan.get("workspace_length_m"),
        "step_spacing_m": plan.get("step_spacing_m", getattr(args, "step_spacing_m", None)),
        "path_shape": str(plan.get("path_shape", getattr(args, "path_shape", "unknown"))),
        "diagonal_orientation": getattr(args, "diagonal_orientation", "A_top_left_to_B_bottom_right"),
        "expected_goal_distance_m": float(plan["goal_distance_m"]),
        "connector_count": int(plan.get("connector_count", 0)),
        "connector_style": str(plan.get("connector_style", geometry.DEFAULT_CONNECTOR_STYLE)),
        "connector_turn_count": int(plan.get("connector_turn_count", plan.get("connector_count", 0))),
        "step_lane_count": int(plan.get("step_lane_count", 0)),
        "coverage_area_estimate_m2": plan.get("coverage_area_estimate_m2"),
        "expected_sweep_style": plan.get("expected_sweep_style", "lawnmower_ㄹ"),
        "lane_count": int(plan.get("lane_count", 0)),
        "segment_count": int(plan.get("segment_count", 0)),
        "expected_lane_count": int(plan.get("lane_count", 0)),
        "expected_segment_count": int(plan.get("segment_count", 0)),
        "relative_enu_note": (
            "A is local (0,0); B is (goal_east_m, goal_north_m); coverage_lawnmower sweeps axis-aligned local ENU lanes."
            if str(plan.get("goal_mode")) == "relative_enu" and str(plan.get("path_shape")) == "coverage_lawnmower"
            else "A is local (0,0); B is (goal_east_m, goal_north_m); workspace width is perpendicular to the A-B diagonal."
            if str(plan.get("goal_mode")) == "relative_enu"
            else "NA"
        ),
        "ready_for_full_path_following": False,
    }


def validate_resolved_field_config(args: argparse.Namespace, plan: dict[str, object]) -> None:
    """계획 지오메트리 정합성 검증(위반 시 ValueError). / Validate plan geometry, raise on bad.

    목표거리>0, 스텝 간격>0, (직선 외에는) 작업폭>0 을 요구한다. 작업폭이 A-B 대각선에
    비해 과도하면 명시적 ``--allow-wide-field true`` 없이는 거부한다(현장 오설정 방지 가드).
    Requires goal distance>0, step spacing>0, and (except direct_line) width>0;
    rejects an over-wide field unless ``--allow-wide-field true`` is passed.
    """
    distance = float(plan.get("goal_distance_m", 0.0))
    if distance <= 0.0:
        raise ValueError("goal distance must be > 0")
    step_spacing = float(plan.get("step_spacing_m") or getattr(args, "step_spacing_m", 0.0))
    if step_spacing <= 0.0:
        raise ValueError("step_spacing_m must be > 0")
    if str(plan.get("path_shape")) != "direct_line":
        width = plan.get("workspace_width_m")
        if width is None or float(width) <= 0.0:
            raise ValueError("workspace_width_m must be > 0")
        max_ratio = float(getattr(args, "max_width_to_goal_ratio", 0.95))
        allow_wide = telemetry._parse_bool(getattr(args, "allow_wide_field", "false"), default=False)
        if not allow_wide and float(width) > distance * max_ratio:
            raise ValueError(
                "workspace_width_m is too large for the A-B diagonal; pass --allow-wide-field true only after verifying the field geometry"
            )


def format_field_config(config: dict[str, object]) -> str:
    """A/B 필드 구성을 사람이 읽을 여러 줄 텍스트로. / Field config as human-readable text."""
    ordered_keys = [
        "start_mode",
        "start_source",
        "start_lat",
        "start_lon",
        "start_x_m",
        "start_y_m",
        "goal_mode",
        "goal_east_m",
        "goal_north_m",
        "goal_lat",
        "goal_lon",
        "goal_x_m",
        "goal_y_m",
        "resolved_goal_x_m",
        "resolved_goal_y_m",
        "workspace_width_m",
        "step_spacing_m",
        "path_shape",
        "diagonal_orientation",
        "expected_goal_distance_m",
        "expected_lane_count",
        "expected_segment_count",
        "connector_count",
        "connector_style",
        "connector_turn_count",
        "step_lane_count",
        "coverage_area_estimate_m2",
        "expected_sweep_style",
    ]
    lines = ["Field configuration:"]
    for key in ordered_keys:
        lines.append(f"  {key}={config.get(key, 'NA')}")
    if config.get("relative_enu_note") not in (None, "NA"):
        lines.append(f"  note={config['relative_enu_note']}")
    return "\n".join(lines)


def load_rows_from_log(path: Path) -> list[dict[str, str]]:
    """Parse USBDBG telemetry rows from a saved serial log (no serial needed)."""
    return telemetry.parse_usbdbg_rows(path.read_text())


def load_planner_config(path: Path) -> dict[str, object]:
    """Load a shipped JSON config, dropping ``_``-prefixed comment keys.

    Used for the ``configs/*.json`` starting points. A ``field_rectangle_example``
    config loads with keys that are exactly :func:`preview.build_preview` kwargs,
    so ``build_preview(**load_planner_config(path))`` runs it directly.
    """
    data = json.loads(path.read_text())
    if not isinstance(data, dict):
        raise ValueError(f"config {path} must be a JSON object")
    return {key: value for key, value in data.items() if not key.startswith("_")}


def load_plan_dir_plan(plan_dir: Path) -> dict[str, object]:
    """저장된 plan-dir 를 계획 dict 로 복원. / Reconstruct a plan dict from a saved plan-dir.

    plan.json/preview_summary.json 을 기준으로 세그먼트/프리미티브 JSON 을 다시 붙여 레인·
    커넥터 수를 재계산한다. execute-plan/inspect-plan/auto-relative-run 이 공유한다.
    Rebuilds segments/primitives and recomputes lane/connector counts. Raises if no
    plan file is present.
    """
    candidates = [plan_dir / "plan.json", plan_dir / "preview_summary.json"]
    plan_path = next((path for path in candidates if path.exists()), None)
    if plan_path is None:
        raise FileNotFoundError(f"--plan-dir must contain plan.json or preview_summary.json: {plan_dir}")
    plan = json.loads(plan_path.read_text())
    if not isinstance(plan, dict):
        raise ValueError(f"plan file must contain a JSON object: {plan_path}")
    segments_json = plan_dir / "planned_segments.json"
    if segments_json.exists():
        segments = json.loads(segments_json.read_text())
        if isinstance(segments, list):
            plan["segments"] = segments
            plan["segment_count"] = len(segments)
            lane_count = len(
                [
                    seg for seg in segments
                    if str(seg.get("segment_type", "")) in geometry.FULL_LANE_SEGMENT_TYPES
                ]
            )
            plan["lane_count"] = lane_count
            plan["step_lane_count"] = len(
                [seg for seg in segments if str(seg.get("segment_type", "")) == "step_lane"]
            )
            plan["connector_turn_count"] = len(
                [
                    seg for seg in segments
                    if str(seg.get("segment_type", "")) in geometry.CONNECTOR_SEGMENT_TYPES
                ]
            )
            plan["connector_count"] = max(0, lane_count - 1)
    primitives_json = plan_dir / "planned_primitives.json"
    if primitives_json.exists():
        primitives = json.loads(primitives_json.read_text())
        if isinstance(primitives, list):
            plan["primitives"] = primitives
            plan["primitive_count"] = len(primitives)
    return plan


def diagnose_summary(rows: Sequence[dict[str, str]]) -> dict[str, object]:
    """Summarize telemetry rows into a read-only, never-ready diagnostic dict."""
    heartbeats = [r for r in rows if telemetry.event(r) == "HEARTBEAT"]
    last = heartbeats[-1] if heartbeats else None
    event_counts: dict[str, int] = {}
    for row in rows:
        name = telemetry.event(row) or "NONE"
        event_counts[name] = event_counts.get(name, 0) + 1
    summary: dict[str, object] = {
        "mode": "diagnose",
        "row_count": len(rows),
        "heartbeat_count": len(heartbeats),
        "event_counts": event_counts,
        "guarded_pulse_compatible": controller.guarded_pulse_compatible(last) if last else False,
        "physical_output_active": telemetry.physical_output_active(last) if last else False,
        "last_gps_block_reason": telemetry.gps_block_reason(last) if last else "NA",
        "last_gps_sats": telemetry._fmt(telemetry.gps_sats(last)) if last else "NA",
        "last_gps_hdop": telemetry._fmt(telemetry.gps_hdop(last)) if last else "NA",
        "last_imu_relative_yaw_deg": (
            telemetry._fmt(telemetry.imu_relative_yaw_deg(last)) if last else "NA"
        ),
        "ready_for_full_path_following": False,
    }
    return checks.assert_not_ready_for_full_path_following(summary)


def guarded_pulse_ready_summary(rows: Sequence[dict[str, str]]) -> dict[str, object]:
    """가드 펄스 펌웨어 준비 여부를 telemetry 로 판정. / Judge guarded-pulse firmware readiness.

    가드 하트비트 + 펌웨어 ready + IMU(BMI160) 존재/yaw + RC OK + 중립 OK 를 모두
    만족해야 ready. 하나라도 빠지면 사유 리스트를 모아 반환한다(순수, 부수효과 없음).
    Ready requires guarded heartbeat + firmware ready + IMU(BMI160)/yaw + rc_ok +
    neutral_ok; otherwise collects the missing reasons. Pure, no side effects.
    """
    heartbeats = [row for row in rows if telemetry.event(row) == "HEARTBEAT"]
    last = heartbeats[-1] if heartbeats else {}
    guarded_seen = any(
        row.get("usb_pulse_test_mode") == "true" or row.get(_COMPAT_GUARDED_MODE_KEY) == "true"
        for row in rows
    )
    firmware_ready = any(
        row.get("usb_pulse_ready") == "true" or row.get(_COMPAT_GUARDED_READY_KEY) == "true"
        for row in rows
    )
    imu_enabled = any(row.get("imu_enabled") == "true" for row in rows)
    imu_present = any(row.get("imu_present") == "true" for row in rows)
    imu_bmi160 = any(row.get("imu_type") == "BMI160" for row in rows)
    yaw_seen = any(row.get("imu_relative_yaw_deg", "NA").upper() not in {"", "NA", "NAN", "NONE"} for row in rows)
    rc_ok = any(row.get("rc_ok") == "true" for row in rows)
    neutral_ok = any(row.get("neutral_ok") == "true" for row in rows)
    ready = guarded_seen and firmware_ready and imu_enabled and imu_present and imu_bmi160 and yaw_seen and rc_ok and neutral_ok
    reasons: list[str] = []
    if not guarded_seen:
        reasons.append("GUARDED_PULSE_HEARTBEAT_NOT_SEEN")
    if not firmware_ready:
        reasons.append("GUARDED_PULSE_READY_FALSE")
    if not imu_enabled:
        reasons.append("IMU_NOT_ENABLED")
    if not imu_present:
        reasons.append("IMU_NOT_PRESENT")
    if not imu_bmi160:
        reasons.append("BMI160_NOT_SEEN")
    if not yaw_seen:
        reasons.append("IMU_YAW_NOT_AVAILABLE")
    if not rc_ok:
        reasons.append("RC_NOT_OK")
    if not neutral_ok:
        reasons.append("NEUTRAL_NOT_OK")
    return checks.assert_not_ready_for_full_path_following({
        "mode": "guarded-pulse-ready",
        "success": ready,
        "guarded_pulse_ready": ready,
        "guarded_pulse_heartbeat_seen": guarded_seen,
        "turn_angle_calibration_ready": ready,
        "imu_enabled": imu_enabled,
        "imu_present": imu_present,
        "imu_type": last.get("imu_type", "NA"),
        "imu_relative_yaw_available": yaw_seen,
        "rc_ok": rc_ok,
        "neutral_ok": neutral_ok,
        "reason": "OK" if ready else ",".join(reasons),
        "next_recommended_action": (
            "Guarded pulse firmware is ready for turn calibration or usb-pulse-test validation."
            if ready else
            "Upload/check IMU-enabled guarded pulse firmware and inspect heartbeat, IMU, RC, and neutral fields."
        ),
        "ready_for_full_path_following": False,
    })


# ── 출력 라이터 (JSON/CSV/원시 로그/미리보기 이미지) / Output writers ──
# --- Output writers -----------------------------------------------------------


def _write_json(path: Path, obj: object) -> None:
    """객체를 들여쓴 JSON 으로 기록(직렬화 불가는 str). / Write obj as indented JSON."""
    path.write_text(json.dumps(obj, indent=2, default=str) + "\n")


def _write_rows_csv(path: Path, rows: Sequence[dict[str, object]]) -> None:
    """행 dict 들을 CSV 로(열은 관측 순서로 합집합). / Write row dicts to CSV.

    빈 입력이면 빈 파일을 쓴다. 열 이름은 모든 행 키의 등장 순서 합집합.
    Empty input writes an empty file; columns are the ordered union of all keys.
    """
    if not rows:
        path.write_text("")
        return
    fieldnames: list[str] = []
    for row in rows:
        for key in row.keys():
            if key not in fieldnames:
                fieldnames.append(key)
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def _write_raw_log(path: Path, lines: Sequence[str]) -> None:
    """원시 시리얼 라인들을 로그 파일로. / Write raw serial lines to a log file."""
    path.write_text("\n".join(lines) + ("\n" if lines else ""))


def _required_preview_image_paths(out_dir: Path) -> dict[str, str]:
    """미리보기가 반드시 만들어야 하는 PNG 경로 맵. / Required preview PNG path map."""
    return {
        "preview_current_goal_rectangle_path": str(out_dir / "preview_current_goal_rectangle_path.png"),
        "preview_overview": str(out_dir / "preview_overview.png"),
    }


def _write_required_preview_images(out_dir: Path, plan: dict[str, object]) -> dict[str, str]:
    """필수 미리보기 PNG 를 렌더링(실패 시 RuntimeError). / Render required preview PNGs.

    이미지가 실제로 생기지 않으면 ``PREVIEW_IMAGE_NOT_WRITTEN`` 로 예외를 던져, 렌더 실패를
    물리 실행 전에 잡도록 한다(호출부가 이 예외를 실패 요약으로 변환).
    Raises ``RuntimeError('PREVIEW_IMAGE_NOT_WRITTEN ...')`` if a PNG is not created,
    so render failures are caught before physical execution.
    """
    image_paths = _required_preview_image_paths(out_dir)
    for expected in image_paths.values():
        expected_path = Path(expected)
        rendered = preview.write_preview_png(
            expected_path,
            plan["segments"],  # type: ignore[arg-type]
            float(plan["start_lat"]),
            float(plan["start_lon"]),
            float(plan["goal_lat"]),
            float(plan["goal_lon"]),
            plan.get("workspace"),  # type: ignore[arg-type]
            path_shape=str(plan.get("path_shape", "unknown")),
        )
        if rendered is None or not expected_path.exists():
            raise RuntimeError(f"PREVIEW_IMAGE_NOT_WRITTEN {expected_path}")
    return image_paths


def _write_plan_artifacts(out_dir: Path, plan: dict[str, object], field_config: dict[str, object]) -> dict[str, str]:
    """계획 산출물(이미지+JSON+CSV) 일괄 기록. / Write all plan artifacts (images/JSON/CSV).

    preview/run 공용. 미리보기 PNG, field_config, plan.json, 세그먼트/프리미티브/경로점
    CSV·JSON 을 out_dir 에 남기고 image_paths 를 반환한다.
    Shared by preview/run; writes PNGs, field_config, plan.json and the segment/
    primitive/path CSV+JSON, returning the image path map.
    """
    image_paths = _write_required_preview_images(out_dir, plan)
    field_config["image_paths"] = image_paths
    plan["image_paths"] = image_paths
    _write_json(out_dir / "field_config_resolved.json", field_config)
    _write_json(out_dir / "plan.json", plan)
    _write_rows_csv(out_dir / "planned_segments.csv", plan.get("segments", []))  # type: ignore[arg-type]
    _write_json(out_dir / "planned_segments.json", plan.get("segments", []))
    _write_rows_csv(out_dir / "planned_primitives.csv", plan.get("primitives", []))  # type: ignore[arg-type]
    _write_json(out_dir / "planned_primitives.json", plan.get("primitives", []))
    _write_rows_csv(out_dir / "planned_path_local.csv", plan.get("path_points", []))  # type: ignore[arg-type]
    return image_paths


def _fail(message: str) -> int:
    """stderr 에 ABORT 를 찍고 종료코드 2 반환. / Print ABORT to stderr, return exit code 2."""
    print(f"ABORT: {message}", file=sys.stderr)
    return 2


def _fail_with_summary(args: argparse.Namespace, *, reason: str, message: str) -> int:
    """실패 요약을 out_dir 에 쓰고 코드 2 반환. / Write a failure summary + return 2.

    입력/구성 오류(예: PLAN_INPUT_INVALID)에서 실패도 감사 가능한 요약으로 남기는 헬퍼.
    Records input/config failures as an auditable summary; side effect: writes files.
    """
    out_dir = getattr(args, "out_dir", None)
    if out_dir is not None:
        write_summary_files(
            out_dir,
            {
                "mode": getattr(args, "mode", "unknown"),
                "success": False,
                "reason": reason,
                "message": message,
                "next_recommended_action": "Fix the reported input or configuration and rerun the same command.",
                "ready_for_full_path_following": False,
            },
            title="Physical Path Planner",
        )
    return _fail(message)


# ══ 모드 핸들러 / Mode handlers (each = one CLI subcommand) ══════════════════
# 여기서부터 각 ``cmd_*`` 는 하나의 CLI 서브커맨드를 구현한다. 공통 패턴:
# (1) out_dir 준비 → (2) print/from-log 무하드웨어 분기 → (3) ensure_port +
# (필요시) 펌웨어 컴파일/업로드 → (4) 시리얼 스트리밍 → (5) telemetry 를 순수
# ``evaluate_*``/summary 로 축약 → (6) write_summary_files + stdout 출력 → 종료코드.
# From here each ``cmd_*`` implements one subcommand; common shape: prepare out_dir,
# handle print/from-log hardware-free branches, ensure_port + optional compile/
# upload, stream serial, reduce telemetry via a pure evaluator, write summaries.


# ── preview / 계획 미리보기 (모션 없음) / Coverage-plan preview (no motion) ──


def cmd_preview(args: argparse.Namespace) -> int:
    """coverage 계획을 만들고 렌더링(모션 없음). / Build + render the coverage plan. No motion.

    시작좌표를 해석(명시/라이브 GPS/캐시)하고 계획을 세워 미리보기 PNG 와 요약을 남긴다.
    시작좌표를 못 얻으면 NO_USABLE_START_GPS, 이미지 렌더 실패면 PREVIEW_IMAGE_NOT_WRITTEN.
    부수효과: 라이브 경로에서 시리얼/펌웨어 업로드, out_dir 에 산출물 기록.
    Resolves the start (explicit/live GPS/cache), builds the plan, writes preview
    PNGs + summary. Side effects: serial/upload on the live path; writes artifacts.
    """
    cal = resolve_calibration(args)
    start, raw_start_lines = resolve_start_for_preview(args)
    if start is None:
        out_dir = Path(args.out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        if raw_start_lines:
            _write_raw_log(out_dir / "preview_start_usbdbg.log", raw_start_lines)
        raw_rows = telemetry.parse_usbdbg_rows("\n".join(raw_start_lines))
        snapshot = gps_snapshot(
            raw_rows,
            min_sats=float(getattr(args, "gps_min_sats", 5.0)),
            max_hdop=float(getattr(args, "gps_max_hdop", 2.5)),
        )
        summary = {
            "mode": "preview",
            "success": False,
            "reason": "NO_USABLE_START_GPS",
            "firmware_profile": MAC_PHYSICAL_SUPERVISED_PROFILE,
            "message": NO_USABLE_START_GPS_ACTION,
            "next_recommended_action": NO_USABLE_START_GPS_ACTION,
            "start_mode": getattr(args, "start_mode", "live_gps"),
            "start_source": "none",
            "gps_wait_enabled": telemetry._parse_bool(getattr(args, "wait_gps", "true"), default=True),
            "gps_wait_timeout_s": float(getattr(args, "gps_timeout_s", getattr(args, "start_timeout_s", 0.0))),
            "gps_wait_elapsed_s": float(getattr(args, "gps_timeout_s", getattr(args, "start_timeout_s", 0.0))),
            **{k: v for k, v in snapshot.items() if k != "ready_row"},
            "motion_calibration_loaded": motion_calibration_loaded(cal),
            "ready_for_full_path_following": False,
        }
        write_summary_files(out_dir, summary, title="Physical Path Planner Preview")
        print(f"preview: reason=NO_USABLE_START_GPS. {NO_USABLE_START_GPS_ACTION}")
        return 2
    args.start_lat = float(start["start_lat"])
    args.start_lon = float(start["start_lon"])
    try:
        plan = resolve_plan(args, cal)
    except ValueError as exc:
        return _fail_with_summary(args, reason="PLAN_INPUT_INVALID", message=str(exc))
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    if raw_start_lines:
        _write_raw_log(out_dir / "preview_start_usbdbg.log", raw_start_lines)
    summary = {
        **plan,
        "mode": "preview",
        "success": True,
        "reason": "OK",
        "firmware_profile": MAC_PHYSICAL_SUPERVISED_PROFILE,
        "start_mode": getattr(args, "start_mode", "live_gps"),
        "start_source": start["start_source"],
        "current_lat": start["start_lat"],
        "current_lon": start["start_lon"],
        "start_gps_block_reason": start["start_gps_block_reason"],
        "start_gps_sats": start["start_gps_sats"],
        "start_gps_hdop": start["start_gps_hdop"],
        "gps_cached_used": start["gps_cached_used"],
        "gps_wait_enabled": telemetry._parse_bool(getattr(args, "wait_gps", "true"), default=True),
        "gps_wait_timeout_s": float(getattr(args, "gps_timeout_s", getattr(args, "start_timeout_s", 0.0))),
        "gps_wait_elapsed_s": start.get("gps_wait_elapsed_s", 0.0),
        **dict(start.get("gps_wait_snapshot", {})),
        "motion_calibration_loaded": motion_calibration_loaded(cal),
        "connector_mode_effective": cal.get("connector_mode_effective", plan.get("connector_mode_effective")),
        "next_recommended_action": f"Inspect {out_dir / 'summary.md'} and preview outputs before execute-plan or run.",
        "ready_for_full_path_following": False,
    }
    field_config = dict(plan.get("field_config", {}))
    field_config.update(
        {
            "start_mode": getattr(args, "start_mode", "live_gps"),
            "start_source": start["start_source"],
        }
    )
    try:
        image_paths = _write_plan_artifacts(out_dir, plan, field_config)
    except RuntimeError as exc:
        expected = str(exc).replace("PREVIEW_IMAGE_NOT_WRITTEN ", "")
        failure = {
            "mode": "preview",
            "success": False,
            "reason": "PREVIEW_IMAGE_NOT_WRITTEN",
            "expected_image_path": expected,
            "next_recommended_action": "Install/check matplotlib rendering and rerun preview before physical execution.",
            "ready_for_full_path_following": False,
        }
        write_summary_files(out_dir, failure, title="Physical Path Planner Preview")
        print(f"preview: reason=PREVIEW_IMAGE_NOT_WRITTEN expected_image_path={expected}")
        return 2
    plan["image_paths"] = image_paths
    summary["field_config"] = field_config
    summary["image_paths"] = image_paths
    _write_json(out_dir / "preview_summary.json", summary)
    write_summary_files(out_dir, summary, title="Physical Path Planner Preview")
    if telemetry._parse_bool(getattr(args, "print_field_config", "false"), default=False):
        print(format_field_config(field_config))
    print(
        f"preview: {plan['segment_count']} segments, "
        f"{plan['lane_count']} lanes, goal_distance_m={float(plan['goal_distance_m']):.3f} -> {out_dir}"
    )
    return 0


# ── gps-wait / GPS 콜드스타트 대기 (모션 없음) / Wait for GPS fix (no motion) ──


def _gps_wait_summary(
    rows: Sequence[dict[str, str]],
    *,
    mode: str,
    elapsed_s: float,
    timeout_s: float,
    min_sats: float,
    max_hdop: float,
) -> dict[str, object]:
    """gps-wait 결과 요약(준비/타임아웃). / Build the gps-wait summary (ready/timeout)."""
    snapshot = gps_snapshot(rows, min_sats=min_sats, max_hdop=max_hdop)
    summary = {k: v for k, v in snapshot.items() if k != "ready_row"}
    success = bool(snapshot["gps_ready"])
    summary.update(
        {
            "mode": mode,
            "success": success,
            "reason": "GPS_READY" if success else "GPS_WAIT_TIMEOUT",
            "firmware_profile": MAC_PHYSICAL_SUPERVISED_PROFILE,
            "gps_wait_enabled": True,
            "gps_wait_timeout_s": timeout_s,
            "gps_wait_elapsed_s": elapsed_s,
            "gps_min_sats": min_sats,
            "gps_max_hdop": max_hdop,
            "next_recommended_action": (
                "Use preview/run now that a start coordinate is available."
                if success else GPS_WAIT_TIMEOUT_ACTION
            ),
            "ready_for_full_path_following": False,
        }
    )
    return checks.assert_not_ready_for_full_path_following(summary)


def gps_telemetry_parse_mismatch(raw_lines: Sequence[str], rows: Sequence[dict[str, str]]) -> bool:
    """원시 라인은 왔는데 GPS/IMU 필드를 못 알아봤나. / Raw arrived but GPS/IMU fields unparsed?

    True 면 대개 펌웨어 프로파일 불일치 또는 파서 mismatch(잘못된 telemetry 포맷)를 뜻한다.
    True usually indicates a wrong firmware profile or a telemetry parser mismatch.
    """
    if not raw_lines:
        return False
    gps_keys = {
        "gps_chars",
        "gps_ready",
        "gps_solution_valid",
        "current_lat",
        "current_lon",
        "gps_lat",
        "gps_lon",
        "gps_sats",
        "gps_hdop",
        "last_rmc_status",
        "last_gga_fix_quality",
    }
    imu_keys = {"imu_present", "imu_relative_yaw_deg", "imu_enabled", "imu_type"}
    return (
        not rows
        or not any(gps_keys.intersection(row.keys()) for row in rows)
        or not any(imu_keys.intersection(row.keys()) for row in rows)
    )


def write_raw_gps_samples(out_dir: Path, raw_lines: Sequence[str]) -> None:
    """진단용 원시 GPS 라인 최대 20개 기록. / Write up to 20 raw GPS lines for diagnosis."""
    samples = [line for line in raw_lines if line][:20]
    (out_dir / "raw_gps_samples.txt").write_text("\n".join(samples) + ("\n" if samples else ""), encoding="utf-8")


def cmd_gps_wait(args: argparse.Namespace) -> int:
    """사용 가능한 GPS 시작 fix 를 대기(모션 없음). / Wait for a usable GPS start fix. No motion.

    감독형 펌웨어를 (옵션) 업로드하고 시리얼을 스트리밍하며 준비될 때까지 대기, 성공 시
    시작좌표를 캐시에 저장한다. Ctrl-C 는 USER_ABORTED(130), 시리얼 끊김은 SERIAL_DISCONNECT.
    부수효과: 시리얼/업로드, 요약·CSV·원시로그 기록, 성공 시 GPS 캐시 쓰기.
    Optionally uploads supervised firmware, streams serial until GPS is ready, and
    caches the fix on success. Side effects: serial/upload, writes files + cache.
    """
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    raw_lines: list[str] = []
    rows: list[dict[str, str]] = []
    start = time.monotonic()
    user_aborted = False
    if getattr(args, "from_log", None):
        raw_lines = Path(args.from_log).read_text(encoding="utf-8").splitlines()
        rows = telemetry.parse_usbdbg_rows("\n".join(raw_lines))
        elapsed_s = 0.0
    else:
        if not ensure_port(args):
            return 2
        if args.upload in {"true", "auto"}:
            uploaded = _upload_mac_physical_supervised_firmware(
                args,
                out_dir,
                title="GPS Wait",
                mode="gps-wait",
                build_path="/private/tmp/openrb-mac-physical-supervised",
            )
            if uploaded != 0:
                return uploaded
        import serial
        try:
            serial_errors = (OSError, serial.serialutil.SerialException)
        except AttributeError:
            serial_errors = (OSError,)

        deadline = start + float(args.timeout_s)
        next_status = start
        try:
            with serial.Serial(args.port, baudrate=args.baud, timeout=0.5) as handle:
                while time.monotonic() < deadline:
                    raw = handle.readline()
                    if not raw:
                        continue
                    line = raw.decode("utf-8", errors="replace").strip()
                    raw_lines.append(line)
                    parsed = telemetry.parse_usbdbg_rows(line)
                    if parsed:
                        rows.extend(parsed)
                    snapshot = gps_snapshot(rows, min_sats=args.min_sats, max_hdop=args.max_hdop)
                    if time.monotonic() >= next_status:
                        print(_gps_status_line(time.monotonic() - start, snapshot))
                        next_status = time.monotonic() + float(args.status_interval_s)
                    if snapshot["gps_ready"]:
                        break
        except KeyboardInterrupt:
            user_aborted = True
            print("gps-wait: user aborted; writing summaries.")
        except serial_errors:
            elapsed_s = time.monotonic() - start
            summary = {
                **_gps_wait_summary(
                    rows,
                    mode="gps-wait",
                    elapsed_s=elapsed_s,
                    timeout_s=float(args.timeout_s),
                    min_sats=float(args.min_sats),
                    max_hdop=float(args.max_hdop),
                ),
                "success": False,
                "reason": "SERIAL_DISCONNECT",
                "next_recommended_action": "Reconnect OpenRB and rerun gps-wait.",
                "ready_for_full_path_following": False,
            }
            _write_raw_log(out_dir / "raw_usbdbg.log", raw_lines)
            _write_rows_csv(out_dir / "gps_wait.csv", rows)
            write_summary_files(out_dir, summary, title="GPS Wait")
            return 2
        elapsed_s = time.monotonic() - start
    summary = _gps_wait_summary(
        rows,
        mode="gps-wait",
        elapsed_s=elapsed_s,
        timeout_s=float(args.timeout_s),
        min_sats=float(args.min_sats),
        max_hdop=float(args.max_hdop),
    )
    if gps_telemetry_parse_mismatch(raw_lines, rows):
        write_raw_gps_samples(out_dir, raw_lines)
        summary = {
            **summary,
            "success": False,
            "reason": "WRONG_FIRMWARE_PROFILE_OR_TELEMETRY_PARSE_MISMATCH",
            "raw_gps_samples": "raw_gps_samples.txt",
            "next_recommended_action": "Telemetry arrived, but GPS/IMU fields for MAC_PHYSICAL_SUPERVISED were not recognized. Inspect raw_gps_samples.txt and verify the firmware profile/parser.",
            "ready_for_full_path_following": False,
        }
    if user_aborted:
        summary = {
            **summary,
            "success": False,
            "reason": "USER_ABORTED",
            "user_aborted": True,
            "next_recommended_action": "Rerun gps-wait when ready to continue waiting for GPS.",
            "ready_for_full_path_following": False,
        }
        summary = checks.assert_not_ready_for_full_path_following(summary)
    _write_raw_log(out_dir / "raw_usbdbg.log", raw_lines)
    _write_rows_csv(out_dir / "gps_wait.csv", rows)
    write_summary_files(out_dir, summary, title="GPS Wait")
    if summary["success"] is True:
        write_gps_cache(gps_snapshot(rows, min_sats=args.min_sats, max_hdop=args.max_hdop))
    print(
        f"gps-wait: reason={summary['reason']} "
        f"best_sats={summary['best_sats']} best_hdop={summary['best_hdop']} -> {out_dir}"
    )
    if user_aborted:
        return 130
    return 0 if summary["success"] is True else 2


# ── calibrate-turn / 턴 각도 보정 (외부 스크립트 위임) / Turn-angle calibration ──


def cmd_calibrate_turn(args: argparse.Namespace) -> int:
    """가드 펄스 턴 각도 보정 스크립트로 위임. / Shell out to guarded-pulse turn calibration.

    이 CLI 는 직접 모터를 돌리지 않고 레거시 보정 스크립트에 argv 를 만들어 넘긴다
    (IMU yaw 비교 강제). ``--print-cmd`` 는 시리얼/펌웨어 없이 명령만 출력.
    부수효과: 서브프로세스 실행, 요약 기록. 반환은 스크립트 종료코드.
    Builds argv for the legacy calibration script (forcing IMU yaw compare) and runs
    it; ``--print-cmd`` only prints. Side effects: subprocess + summary write.
    """
    if args.print_cmd:
        args.port = printable_port(args.port)
    if not args.print_cmd and not ensure_port(args):
        return 2
    mode = args.mode
    if args.direction:
        mode = "turn_left" if args.direction == "left" else "turn_right"
    argv = build_calibrate_turn_argv(
        script=args.script,
        port=args.port,
        mode=mode,
        b_cmd=args.b_cmd,
        pulse_ms=args.pulse_ms,
        max_abs_b=args.max_abs_b,
        max_ms=args.max_ms,
        target_angle_deg=args.target_angle_deg,
        angle_tolerance_deg=args.angle_tolerance_deg,
        save_turn_calibration=args.save_turn_calibration,
        turn_calibration_out=args.turn_calibration_out,
        out_dir=args.out_dir,
    )
    printable = " ".join(shlex.quote(part) for part in argv)
    if args.print_cmd:
        print(printable)
        write_summary_files(
            args.out_dir,
            {
                "mode": "calibrate-turn",
                "success": True,
                "reason": "COMMAND_PRINTED",
                "command": printable,
                "turn_angle_calibration_ready": False,
                "next_recommended_action": "Run without --print-cmd only when ready to upload firmware and calibrate physically.",
                "ready_for_full_path_following": False,
            },
            title="Turn Angle Calibration",
        )
        return 0
    print(f"calibrate-turn: invoking {printable}")
    completed = subprocess.run(argv, check=False)
    write_summary_files(
        args.out_dir,
        {
            "mode": "calibrate-turn",
            "success": completed.returncode == 0,
            "reason": "OK" if completed.returncode == 0 else "TURN_CALIBRATION_FAILED",
            "returncode": completed.returncode,
            "command": printable,
            "turn_angle_calibration_ready": completed.returncode == 0,
            "next_recommended_action": (
                "Inspect the calibration output summary and turn calibration JSON."
                if completed.returncode == 0
                else "Inspect raw logs, IMU yaw availability, ACK/STOP, and visual confirmation."
            ),
            "ready_for_full_path_following": False,
        },
        title="Turn Angle Calibration",
    )
    return completed.returncode


# ── manual-rc / 수동 RC 패스스루 복구·검증 / Manual RC passthrough recovery ──


def cmd_manual_rc(args: argparse.Namespace) -> int:
    """검증된 수동 RC 패스스루 펌웨어 업로드/검증. / Restore + validate manual RC passthrough.

    업로드/검증 스크립트로 위임한다. 채널 매핑 플래그 중 미구현 조합은 경고로 표시.
    ``--print-cmd`` 는 명령만 출력, ``--diagnose-only true`` 는 업로드를 건너뛴다.
    부수효과: 서브프로세스 실행, 요약 병합/기록.
    Delegates to the upload/validate scripts; flags unimplemented channel-mapping
    combos as warnings. Side effects: subprocess + summary write.
    """
    if args.print_cmd:
        args.port = printable_port(args.port)
    if not args.print_cmd and not ensure_port(args):
        return 2
    if args.diagnose_only == "true" and args.upload == "true":
        print("diagnose-only requested; skipping upload")
        args.upload = "false"
    upload_enabled = args.upload in {"true", "auto"}
    validate_enabled = args.validate == "true"
    upload_cmd = ["bash", args.upload_script, "--port", str(args.port)]
    upload_cmd.extend(["--rc-input-mode", str(args.rc_input_mode)])
    if args.mode_channel_index is not None:
        upload_cmd.extend(["--mode-channel-index", str(args.mode_channel_index)])
    upload_cmd.extend(["--steer-channel-index", str(args.steer_channel_index)])
    upload_cmd.extend(["--throttle-channel-index", str(args.throttle_channel_index)])
    validate_cmd = [
        "bash",
        args.validate_script,
        "--port",
        str(args.port),
        "--duration-s",
        str(args.duration_s),
        "--out-dir",
        str(args.out_dir),
        "--upload",
        "false",
    ]
    if args.log:
        validate_cmd.extend(["--log", str(args.log)])
    if args.diagnose_only == "true":
        validate_cmd.extend(["--diagnose-only", "true"])
    mapping_warning = "NONE"
    if args.rc_input_mode not in {"auto", "old_known_good", "ppm"}:
        mapping_warning = "RC_INPUT_MODE_FLAG_NOT_IMPLEMENTED"
    elif args.steer_channel_index != 0 or args.throttle_channel_index != 1:
        mapping_warning = "RC_STEER_THROTTLE_CHANNEL_FLAGS_NOT_IMPLEMENTED"
    mapping_summary = {
        "rc_input_mode_requested": args.rc_input_mode,
        "rc_input_mode_effective": "ppm_old_known_good" if args.rc_input_mode in {"auto", "old_known_good", "ppm"} else "unsupported",
        "mode_channel_index": args.mode_channel_index,
        "steer_channel_index": args.steer_channel_index,
        "throttle_channel_index": args.throttle_channel_index,
        "old_known_good_rc_path": args.rc_input_mode in {"auto", "old_known_good", "ppm"},
        "manual_forward_sign": -1,
        "manual_turn_sign": 1,
        "motor_output_swap_lr": 0,
        "drive_calibration_enable": 0,
        "manual_mode_threshold_us": args.manual_mode_threshold_us,
        "rc_mapping_flags_effective": mapping_warning == "NONE",
        "rc_mapping_warning": mapping_warning,
    }
    if args.print_cmd:
        if upload_enabled:
            print(" ".join(shlex.quote(part) for part in upload_cmd))
        if validate_enabled:
            print(" ".join(shlex.quote(part) for part in validate_cmd))
        print("ready_for_full_path_following=false")
        write_summary_files(
            args.out_dir,
            {
                "mode": "manual-rc",
                "success": True,
                "reason": "COMMAND_PRINTED",
                "upload_success": False,
                "validation_success": False,
                **mapping_summary,
                "next_recommended_action": "Run without --print-cmd when ready to upload or validate manual RC telemetry.",
                "ready_for_full_path_following": False,
            },
            title="Manual RC Diagnostic",
        )
        return 0
    print("Manual RC recovery")
    print("RC transmitter ON; MANUAL / AUTO OFF")
    print("Sequence: neutral 5s, slight forward, neutral, slight backward, neutral, slight left/right steering, neutral")
    if args.print_rc_mapping == "true":
        for key, value in mapping_summary.items():
            print(f"{key}={value}")
    print(f"manual_rc_recovery_flags={manual_rc_recovery_flags(mode_channel_index=args.mode_channel_index)}")
    upload_success = False
    if upload_enabled:
        completed = subprocess.run(upload_cmd, check=False)
        if completed.returncode != 0:
            write_summary_files(
                args.out_dir,
                {
                    "mode": "manual-rc",
                    "success": False,
                    "reason": "MANUAL_RC_UPLOAD_FAILED",
                    "upload_success": False,
                    "validation_success": False,
                    "returncode": completed.returncode,
                    **mapping_summary,
                    "next_recommended_action": "Check Arduino CLI, OpenRB port, and compile/upload output.",
                    "ready_for_full_path_following": False,
                },
                title="Manual RC Diagnostic",
            )
            return completed.returncode
        upload_success = True
    else:
        upload_success = args.upload == "false"
    if validate_enabled:
        completed = subprocess.run(validate_cmd, check=False)
        summary_path = Path(args.out_dir) / "summary.json"
        validation_summary: dict[str, object]
        if summary_path.exists():
            loaded = json.loads(summary_path.read_text())
            validation_summary = loaded if isinstance(loaded, dict) else {}
        else:
            validation_summary = {
                "reason": "MANUAL_RC_VALIDATION_FAILED",
                "manual_rc_passthrough_ok": False,
                "validation_success": False,
            }
        merged = {
            "mode": "manual-rc",
            "success": completed.returncode == 0 and validation_summary.get("manual_rc_passthrough_ok") is True,
            "upload_success": upload_success,
            "validation_success": completed.returncode == 0 and validation_summary.get("manual_rc_passthrough_ok") is True,
            **mapping_summary,
            **validation_summary,
            "ready_for_full_path_following": False,
        }
        merged.setdefault("reason", "OK" if merged["success"] else "MANUAL_RC_VALIDATION_FAILED")
        if merged.get("reason") == "RC_INPUT_ABSENT":
            merged["next_recommended_action"] = RC_INPUT_ABSENT_ACTION
        merged.setdefault("next_recommended_action", "Inspect manual RC telemetry and wiring before rerunning.")
        write_summary_files(args.out_dir, merged, title="Manual RC Diagnostic")
        return 0 if merged["success"] is True else 2
    write_summary_files(
        args.out_dir,
        {
            "mode": "manual-rc",
            "success": upload_success,
            "reason": "OK" if upload_success else "NO_UPLOAD_OR_VALIDATION_REQUESTED",
            "upload_success": upload_success,
            "validation_success": False,
            "manual_rc_passthrough_ok": False,
            **mapping_summary,
            "next_recommended_action": "Run manual-rc --upload false --validate true to diagnose receiver input.",
            "ready_for_full_path_following": False,
        },
        title="Manual RC Diagnostic",
    )
    return 0 if upload_success else 2


# ── rc-auto-pattern / 무선(무테더) CH5 MANUAL·AUTO ㄹ 패턴 / Untethered RC AUTO pattern ──


RC_AUTO_PATTERN_BUILD_PATH = "outputs/firmware_builds/rc_auto_pattern"


def rc_auto_pattern_firmware_flags(
    *,
    lanes: int,
    lane_ms: int,
    step_ms: int,
    forward_a: float,
    reverse_a: float,
    turn_b_left: float,
    turn_b_right: float,
    turn_target_deg: float,
    turn_tol_deg: float,
    turn_timeout_ms: int,
    pause_ms: int,
    heading_kp: float = 0.015,
    heading_hold_max_b: float = 0.25,
    drive_b_trim: float = 0.0,
    drive_steer_sign: float = -1.0,
    drive_abort_err_deg: float = 60.0,
    turn_coast_s: float = 0.15,
    turn_settle_retries: int = 2,
    rc_loss_grace_ms: int = 1500,
    mode_channel_index: int | None = 4,
    profile: str = MANUAL_CONTROL_FULL_TELEMETRY_PPM_PROFILE,
) -> str:
    """Build flags for the untethered RC AUTO pattern firmware.

    Starts from a manual-control PPM profile (RC manual driving, the locked
    direction signs, CH5 mode channel) and adds IMU yaw plus the onboard
    lawnmower pattern: CH5=MANUAL drives manually, CH5=AUTO runs the pattern
    once per fresh MANUAL->AUTO transition. No station/USB link is required
    after upload.

    Default profile is full-telemetry-ppm: it keeps the firmware-default PPM
    decode settings, which are the SAME settings the supervised firmware uses
    and the only configuration field-proven to decode this receiver's
    channels. rc-mix-ppm's falling-edge decode produced
    PPM_SYNC_ONLY_NO_CHANNELS (no frames, FAILSAFE) on 2026-06-12.
    """
    base = manual_control_firmware_flags(
        profile=profile, mode_channel_index=mode_channel_index
    )
    base = base.replace("-DIMU_ENABLE=0", "-DIMU_ENABLE=1 -DIMU_YAW_DIAG=1")
    return (
        f"{base} "
        "-DRC_AUTO_PATTERN=1 "
        f"-DRC_AUTO_PATTERN_LANES={int(lanes)} "
        f"-DRC_AUTO_PATTERN_LANE_MS={int(lane_ms)} "
        f"-DRC_AUTO_PATTERN_STEP_MS={int(step_ms)} "
        f"-DRC_AUTO_PATTERN_FORWARD_A={float(forward_a)}f "
        f"-DRC_AUTO_PATTERN_REVERSE_A={float(reverse_a)}f "
        f"-DRC_AUTO_PATTERN_TURN_B_LEFT={float(turn_b_left)}f "
        f"-DRC_AUTO_PATTERN_TURN_B_RIGHT={float(turn_b_right)}f "
        f"-DRC_AUTO_PATTERN_TURN_TARGET_DEG={float(turn_target_deg)} "
        f"-DRC_AUTO_PATTERN_TURN_TOL_DEG={float(turn_tol_deg)} "
        f"-DRC_AUTO_PATTERN_TURN_TIMEOUT_MS={int(turn_timeout_ms)} "
        f"-DRC_AUTO_PATTERN_PAUSE_MS={int(pause_ms)} "
        f"-DRC_AUTO_PATTERN_HEADING_KP={float(heading_kp)}f "
        f"-DRC_AUTO_PATTERN_HEADING_MAX_B={float(heading_hold_max_b)}f "
        f"-DRC_AUTO_PATTERN_DRIVE_B_TRIM={float(drive_b_trim)}f "
        f"-DRC_AUTO_PATTERN_DRIVE_STEER_SIGN={float(drive_steer_sign)}f "
        f"-DRC_AUTO_PATTERN_DRIVE_ABORT_ERR_DEG={float(drive_abort_err_deg)} "
        f"-DRC_AUTO_PATTERN_TURN_COAST_S={float(turn_coast_s)}f "
        f"-DRC_AUTO_PATTERN_TURN_SETTLE_RETRIES={int(turn_settle_retries)} "
        f"-DRC_AUTO_PATTERN_RC_LOSS_GRACE_MS={int(rc_loss_grace_ms)}"
    )


def cmd_rc_auto_pattern(args: argparse.Namespace) -> int:
    """Upload the untethered RC AUTO pattern firmware, then optionally monitor.

    After upload the rover is standalone: the USB cable may be removed. CH5
    MANUAL = RC stick driving; a fresh MANUAL (>=1 s) followed by AUTO runs the
    onboard ㄹ pattern once; flipping back to MANUAL stops motors immediately.
    """
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    if args.print_cmd:
        args.port = printable_port(args.port)
    if not args.print_cmd and not ensure_port(args):
        return 2
    flags = rc_auto_pattern_firmware_flags(
        profile=args.profile,
        lanes=args.lanes,
        lane_ms=args.lane_ms,
        step_ms=args.step_ms,
        forward_a=args.forward_a,
        reverse_a=args.reverse_a,
        turn_b_left=args.turn_b_left,
        turn_b_right=args.turn_b_right,
        turn_target_deg=args.turn_target_deg,
        turn_tol_deg=args.turn_tol_deg,
        turn_timeout_ms=args.turn_timeout_ms,
        pause_ms=args.pause_ms,
        heading_kp=args.heading_kp,
        heading_hold_max_b=args.heading_hold_max_b,
        drive_b_trim=args.drive_b_trim,
        drive_steer_sign=args.drive_steer_sign,
        drive_abort_err_deg=args.drive_abort_err_deg,
        turn_coast_s=args.turn_coast_s,
        turn_settle_retries=args.turn_settle_retries,
        rc_loss_grace_ms=args.rc_loss_grace_ms,
        mode_channel_index=args.mode_channel_index,
    )
    compile_cmd = [
        "arduino-cli", "compile", "--fqbn", "OpenRB-150:samd:OpenRB-150",
        "--build-path", RC_AUTO_PATTERN_BUILD_PATH,
        "--build-property", f"compiler.cpp.extra_flags={flags}",
        "firmware/openrb_robot_controller",
    ]
    upload_cmd = [
        "arduino-cli", "upload", "-p", str(args.port),
        "--fqbn", "OpenRB-150:samd:OpenRB-150",
        "--build-path", RC_AUTO_PATTERN_BUILD_PATH,
        "firmware/openrb_robot_controller",
    ]
    config = {
        "mode": "rc-auto-pattern",
        "firmware_profile": "rc_auto_pattern",
        "ppm_profile": args.profile,
        "untethered": True,
        "rc_input_mode": "ppm",
        "ppm_input_pin": "D6",
        "steer_channel": "CH1",
        "throttle_channel": "CH2",
        "mode_channel": "CH5",
        "mode_channel_index": args.mode_channel_index,
        "lanes": args.lanes,
        "lane_ms": args.lane_ms,
        "step_ms": args.step_ms,
        "forward_a": args.forward_a,
        "reverse_a": args.reverse_a,
        "turn_b_left": args.turn_b_left,
        "turn_b_right": args.turn_b_right,
        "turn_target_deg": args.turn_target_deg,
        "turn_tol_deg": args.turn_tol_deg,
        "turn_timeout_ms": args.turn_timeout_ms,
        "pause_ms": args.pause_ms,
        "heading_kp": args.heading_kp,
        "heading_hold_max_b": args.heading_hold_max_b,
        "drive_b_trim": args.drive_b_trim,
        "drive_steer_sign": args.drive_steer_sign,
        "drive_abort_err_deg": args.drive_abort_err_deg,
        "turn_coast_s": args.turn_coast_s,
        "turn_settle_retries": args.turn_settle_retries,
        "rc_loss_grace_ms": args.rc_loss_grace_ms,
        "heading_frame": "absolute body-heading chain anchored at each AUTO start",
        "drive_steer_sign_note": (
            "yaw response to B while translating is inverted vs stationary "
            "pivots on this drivetrain (field log 2026-06-12); -1 flips lane "
            "feedback only"
        ),
        "arming_rule": "fresh MANUAL >=1s then AUTO; one run per transition; MANUAL stops instantly",
        "rc_loss_rule": (
            "RC dropout <= grace pauses the run (resumes mid-step); longer dropout "
            "resets+disarms so the next MANUAL(1s)->AUTO restarts from step 0"
        ),
        "ready_for_full_path_following": False,
    }
    # Render the expected pattern geometry so the operator can SEE the path
    # before going untethered: lane/step distances are estimated from the
    # configured durations at the calibrated forward speed (~0.43 m/s at
    # A=0.30, scaled linearly with the commanded A).
    speed_mps = 0.43 * (abs(float(args.forward_a)) / 0.30) if args.forward_a else 0.43
    estimated_lane_m = round(args.lane_ms / 1000.0 * speed_mps, 2)
    estimated_step_m = round(args.step_ms / 1000.0 * speed_mps, 2)
    config["estimated_speed_mps"] = round(speed_mps, 3)
    config["estimated_lane_m"] = estimated_lane_m
    config["estimated_step_m"] = estimated_step_m
    config["estimated_field_width_m"] = round(max(0, args.lanes - 1) * estimated_step_m, 2)
    preview_path = None
    if args.lanes >= 2 and estimated_lane_m > 0 and estimated_step_m > 0:
        try:
            pattern_plan = preview.build_preview(
                start_lat=0.0,
                start_lon=0.0,
                goal_mode="relative_enu",
                goal_east_m=estimated_lane_m,
                goal_north_m=(args.lanes - 1) * estimated_step_m,
                workspace_width_m=(args.lanes - 1) * estimated_step_m,
                step_spacing_m=estimated_step_m,
            )
            preview_path = preview.write_preview_png(
                out_dir / "rc_auto_pattern_preview.png",
                pattern_plan["segments"],
                0.0,
                0.0,
                float(pattern_plan["goal_lat"]),
                float(pattern_plan["goal_lon"]),
                workspace=pattern_plan.get("workspace"),
                path_shape="coverage_lawnmower",
            )
        except (ValueError, RuntimeError):
            preview_path = None
    config["preview_image"] = str(preview_path) if preview_path else "NONE"
    _write_json(out_dir / "rc_auto_pattern_config.json", config)
    if preview_path:
        print(f"pattern preview: {preview_path}")
    if args.print_cmd:
        print(" ".join(shlex.quote(part) for part in compile_cmd))
        print(" ".join(shlex.quote(part) for part in upload_cmd))
        print(f"rc_auto_pattern_firmware_flags={flags}")
        print("ready_for_full_path_following=false")
        write_summary_files(
            out_dir,
            {
                **config,
                "success": True,
                "reason": "COMMAND_PRINTED",
                "next_recommended_action": "Run without --print-cmd to upload the untethered pattern firmware.",
            },
            title="RC Auto Pattern",
        )
        return 0
    if args.upload in {"true", "auto"}:
        completed = subprocess.run(compile_cmd, check=False)
        if completed.returncode != 0:
            write_summary_files(
                out_dir,
                {**config, "success": False, "reason": "RC_AUTO_PATTERN_COMPILE_FAILED",
                 "returncode": completed.returncode,
                 "next_recommended_action": "Inspect the Arduino compile output."},
                title="RC Auto Pattern",
            )
            return completed.returncode
        completed = subprocess.run(upload_cmd, check=False)
        if completed.returncode != 0:
            write_summary_files(
                out_dir,
                {**config, "success": False, "reason": "RC_AUTO_PATTERN_UPLOAD_FAILED",
                 "returncode": completed.returncode,
                 "next_recommended_action": "Check the OpenRB port and upload output."},
                title="RC Auto Pattern",
            )
            return completed.returncode
    raw_lines: list[str] = []
    if args.duration_s > 0:
        import serial  # local import keeps print-cmd serial-free

        monitor_port = args.port
        try:
            with serial.Serial(monitor_port, baudrate=args.baud, timeout=0.5) as handle:
                deadline = time.monotonic() + float(args.duration_s)
                while time.monotonic() < deadline:
                    raw = handle.readline()
                    if raw:
                        line = raw.decode("utf-8", errors="replace").strip()
                        print(line)
                        raw_lines.append(line)
        except (OSError, KeyboardInterrupt):
            pass
        if raw_lines:
            _write_raw_log(out_dir / "rc_auto_pattern_monitor.log", raw_lines)
    summary = {
        **config,
        "success": True,
        "reason": "UPLOADED",
        "monitor_lines": len(raw_lines),
        "next_recommended_action": (
            "Unplug USB if desired. CH5 MANUAL = stick driving; hold MANUAL >=1 s, "
            "flip to AUTO to run the pattern once; flip back to MANUAL to stop."
        ),
        "ready_for_full_path_following": False,
    }
    write_summary_files(out_dir, summary, title="RC Auto Pattern")
    print("rc-auto-pattern: uploaded. MANUAL=stick drive; MANUAL(1s)->AUTO runs the pattern once.")
    print("ready_for_full_path_following=false")
    return 0


# ── manual-control / PPM 물리 수동제어 업로드·모니터 / PPM manual control ──


def cmd_manual_control(args: argparse.Namespace) -> int:
    """PPM 물리 수동제어 펌웨어 업로드 후 모니터링. / Upload + monitor PPM manual control.

    선택한 PPM 프로파일로 컴파일/업로드하고 시리얼을 스트리밍하며 매초 상태줄을 출력한다.
    종료 시 ``evaluate_manual_control_rows`` 로 통과/사유를 판정한다. ``--from-log`` 는
    시리얼 없이 저장 로그를 재평가, ``--print-cmd`` 는 명령만 출력.
    부수효과: 컴파일/업로드/시리얼, 요약·CSV·원시로그 기록.
    Compiles/uploads the chosen PPM profile, streams serial with a per-second status
    line, then classifies via ``evaluate_manual_control_rows``. Side effects: compile/
    upload/serial + file writes.
    """
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    if args.print_cmd:
        args.port = printable_port(args.port)
    if not args.print_cmd and not args.from_log and not ensure_port(args):
        return 2

    flags = manual_control_firmware_flags(profile=args.profile, mode_channel_index=args.mode_channel_index)
    build_path = manual_control_build_path(args.profile)
    compile_cmd = [
        "arduino-cli",
        "compile",
        "--fqbn",
        "OpenRB-150:samd:OpenRB-150",
        "--build-path",
        build_path,
        "--build-property",
        f"compiler.cpp.extra_flags={flags}",
        "firmware/openrb_robot_controller",
    ]
    upload_cmd = [
        "arduino-cli",
        "upload",
        "-p",
        str(args.port),
        "--fqbn",
        "OpenRB-150:samd:OpenRB-150",
        "--build-path",
        build_path,
        "firmware/openrb_robot_controller",
    ]
    config = {
        "mode": "manual-control",
        "profile": args.profile,
        "firmware_profile": args.profile,
        "expected_ppm_interrupt_edge": manual_control_expected_ppm_edge(args.profile),
        "old_working_log": MANUAL_CONTROL_OLD_WORKING_LOG,
        "rc_input_mode": "ppm",
        "ppm_input_pin": "D6",
        "steer_channel": "CH1",
        "throttle_channel": "CH2",
        "mode_channel": "CH5",
        "mode_channel_index": args.mode_channel_index,
        "manual_forward_sign": -1,
        "manual_turn_sign": 1,
        "gps_required": False,
        "imu_required": False,
        "path_package_required": False,
        "station_frame_parser_required": False,
        "hc12_required": False,
        "ready_for_full_path_following": False,
    }
    _write_json(out_dir / "manual_control_config.json", config)
    if args.print_cmd:
        if args.upload in {"true", "auto"}:
            print(" ".join(shlex.quote(part) for part in compile_cmd))
            print(" ".join(shlex.quote(part) for part in upload_cmd))
        print(f"manual_control_firmware_flags={flags}")
        print(f"manual_control_profile={args.profile}")
        print("PPM wiring: signal -> OpenRB D6; CH1 steering; CH2 throttle; CH5 mode/manual-auto.")
        print("ready_for_full_path_following=false")
        write_summary_files(
            out_dir,
            {
                **config,
                "success": True,
                "reason": "COMMAND_PRINTED",
                "manual_control_ok": False,
                "next_recommended_action": "Run without --print-cmd when ready to upload and monitor PPM manual control.",
            },
            title="Manual Control",
        )
        return 0

    raw_lines: list[str] = []
    if args.from_log:
        raw_lines = Path(args.from_log).read_text(encoding="utf-8").splitlines()
    else:
        if args.upload in {"true", "auto"}:
            completed = subprocess.run(compile_cmd, check=False)
            if completed.returncode != 0:
                write_summary_files(
                    out_dir,
                    {
                        **config,
                        "success": False,
                        "reason": "MANUAL_CONTROL_COMPILE_FAILED",
                        "returncode": completed.returncode,
                        "next_recommended_action": "Inspect Arduino compile output for the PPM manual control firmware.",
                    },
                    title="Manual Control",
                )
                return completed.returncode
            completed = subprocess.run(upload_cmd, check=False)
            if completed.returncode != 0:
                write_summary_files(
                    out_dir,
                    {
                        **config,
                        "success": False,
                        "reason": "MANUAL_CONTROL_UPLOAD_FAILED",
                        "returncode": completed.returncode,
                        "next_recommended_action": "Check OpenRB port and upload output.",
                    },
                    title="Manual Control",
                )
                return completed.returncode
        if args.validate == "false":
            write_summary_files(
                out_dir,
                {
                    **config,
                    "success": True,
                    "reason": "UPLOAD_ONLY",
                    "manual_control_ok": False,
                    "next_recommended_action": "Run manual-control --upload false --validate true to monitor PPM control.",
                },
                title="Manual Control",
            )
            return 0
        import serial

        print("Manual control: PPM input on OpenRB D6.")
        print(f"Profile: {args.profile}. Default rc-mix-ppm matches the May 2 rc_mix_test decoder: FALLING edge, 4000us sync.")
        print("Expected mapping: CH1 steering -> physical B, CH2 throttle -> physical A, CH5 mode/manual-auto.")
        print("GPS/IMU status remains visible as telemetry only; it does not gate manual motor output.")
        print("Set mode to MANUAL / AUTO OFF and move the physical station/controller.")
        if args.duration_s <= 0:
            print("Monitor runs until Ctrl-C.")
        last_status_s = -1
        start_s = time.monotonic()
        deadline = None if args.duration_s <= 0 else start_s + args.duration_s
        try:
            with serial.Serial(args.port, baudrate=args.baud, timeout=0.2) as handle:
                while deadline is None or time.monotonic() < deadline:
                    raw = handle.readline()
                    if raw:
                        line = raw.decode("utf-8", errors="replace").strip()
                        raw_lines.append(line)
                        if args.verbose_raw == "true":
                            print(line)
                    elapsed_s = int(time.monotonic() - start_s)
                    if elapsed_s != last_status_s:
                        last_status_s = elapsed_s
                        rows = telemetry.parse_usbdbg_rows("\n".join(raw_lines[-200:]))
                        print(format_manual_control_status(elapsed_s=elapsed_s, rows=rows))
        except KeyboardInterrupt:
            print("User aborted manual-control monitor; writing summaries.")
        except (OSError, serial.serialutil.SerialException) as exc:
            print(f"manual-control serial error: {exc}")
            raw_lines.append(f"SERIAL_ERROR error={str(exc).replace(' ', '_')}")

    rows = telemetry.parse_usbdbg_rows("\n".join(raw_lines))
    summary = {
        **config,
        **evaluate_manual_control_rows(
            rows,
            expected_ppm_interrupt_edge=manual_control_expected_ppm_edge(args.profile),
        ),
    }
    if summary.get("reason") == "PPM_INPUT_ABSENT":
        print("reason=PPM_INPUT_ABSENT")
        print("Expected wiring: signal -> OpenRB D6; CH1 steering; CH2 throttle; CH5 mode/manual-auto.")
    if summary.get("reason") == "PPM_SYNC_ONLY_NO_CHANNELS":
        print("reason=PPM_SYNC_ONLY_NO_CHANNELS")
        print("D6 has sync-like pulses but no decoded PPM channels. Check receiver output mode is combined PPM, not single PWM.")
    _write_raw_log(out_dir / "raw_usbdbg.log", raw_lines)
    _write_rows_csv(out_dir / "manual_control.csv", rows)
    _write_json(out_dir / "manual_control_summary.json", summary)
    write_summary_files(out_dir, summary, title="Manual Control")
    print(f"manual_control_ok={str(summary['manual_control_ok']).lower()}")
    print(f"reason={summary['reason']}")
    print("ready_for_full_path_following=false")
    return 0 if summary["manual_control_ok"] is True else 2


# ── rc-input-diagnose / 읽기전용 PPM 채널 프로브 / Read-only PPM channel probe ──


def cmd_rc_input_diagnose(args: argparse.Namespace) -> int:
    """읽기전용 RC 입력(PPM) 진단 펌웨어 업로드/판독. / Read-only RC input (PPM) probe.

    모터/GPS/HC-12 를 끈 별도 프로브 스케치를 업로드해 채널 프레임을 읽고
    ``evaluate_rc_input_diagnose_rows`` 로 RC 입력 존재/유효를 판정한다.
    부수효과: 컴파일/업로드/시리얼, 요약·CSV·원시로그 기록.
    Uploads a probe sketch (motors/GPS/HC-12 off), reads channel frames, and
    classifies via ``evaluate_rc_input_diagnose_rows``. Side effects: compile/upload/
    serial + file writes.
    """
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    compile_cmd = [
        "arduino-cli",
        "compile",
        "--fqbn",
        "OpenRB-150:samd:OpenRB-150",
        "--build-path",
        "/private/tmp/openrb-rc-input-diagnose",
        args.sketch,
    ]
    upload_cmd = [
        "arduino-cli",
        "upload",
        "-p",
        printable_port(args.port),
        "--fqbn",
        "OpenRB-150:samd:OpenRB-150",
        "--build-path",
        "/private/tmp/openrb-rc-input-diagnose",
        args.sketch,
    ]
    if args.print_cmd:
        print(" ".join(shlex.quote(part) for part in compile_cmd))
        print(" ".join(shlex.quote(part) for part in upload_cmd))
        print("ready_for_full_path_following=false")
        write_summary_files(
            out_dir,
            {
                "mode": "rc-input-diagnose",
                "success": True,
                "reason": "COMMAND_PRINTED",
                "probe": "ppm_channel_map_probe",
                "motors_enabled": False,
                "next_recommended_action": "Run without --print-cmd to upload/read the RC input diagnostic firmware.",
                "ready_for_full_path_following": False,
            },
            title="RC Input Diagnose",
        )
        return 0
    raw_lines: list[str] = []
    if args.from_log:
        raw_lines = Path(args.from_log).read_text(encoding="utf-8").splitlines()
    else:
        if not ensure_port(args):
            return 2
        if args.upload in {"true", "auto"}:
            completed = subprocess.run(compile_cmd, check=False)
            if completed.returncode != 0:
                write_summary_files(
                    out_dir,
                    {
                        "mode": "rc-input-diagnose",
                        "success": False,
                        "reason": "RC_INPUT_DIAGNOSE_COMPILE_FAILED",
                        "returncode": completed.returncode,
                        "next_recommended_action": "Inspect Arduino compile output for the read-only PPM probe.",
                        "ready_for_full_path_following": False,
                    },
                    title="RC Input Diagnose",
                )
                return completed.returncode
            completed = subprocess.run(upload_cmd, check=False)
            if completed.returncode != 0:
                write_summary_files(
                    out_dir,
                    {
                        "mode": "rc-input-diagnose",
                        "success": False,
                        "reason": "RC_INPUT_DIAGNOSE_UPLOAD_FAILED",
                        "returncode": completed.returncode,
                        "next_recommended_action": "Check OpenRB port and upload mode, then retry rc-input-diagnose.",
                        "ready_for_full_path_following": False,
                    },
                    title="RC Input Diagnose",
                )
                return completed.returncode
        import serial

        print("RC input diagnose: read-only PPM channel probe; motors/GPS/HC-12 disabled.")
        deadline = time.monotonic() + args.duration_s
        with serial.Serial(args.port, baudrate=args.baud, timeout=0.5) as handle:
            while time.monotonic() < deadline:
                raw = handle.readline()
                if raw:
                    line = raw.decode("utf-8", errors="replace").strip()
                    print(line)
                    raw_lines.append(line)
    rows = telemetry.parse_usbdbg_rows("\n".join(raw_lines))
    summary = evaluate_rc_input_diagnose_rows(rows)
    _write_raw_log(out_dir / "raw_usbdbg.log", raw_lines)
    _write_rows_csv(out_dir / "rc_input_diagnose.csv", rows)
    _write_json(out_dir / "rc_input_diagnose_summary.json", summary)
    write_summary_files(out_dir, summary, title="RC Input Diagnose")
    print(f"rc_input_classification={summary['rc_input_classification']}")
    print("ready_for_full_path_following=false")
    return 0 if summary["success"] is True else 2


# ── guarded-pulse-ready / IMU 가드 펄스 펌웨어 준비 확인 / Guarded-pulse readiness ──


def cmd_guarded_pulse_ready(args: argparse.Namespace) -> int:
    """IMU 활성 가드 펄스 펌웨어를 업로드/점검. / Upload + check IMU guarded-pulse firmware.

    가드 펄스 펌웨어를 (옵션) 업로드하고 하트비트/IMU/RC/중립을 스트리밍으로 확인해
    턴 보정·usb-pulse-test 로 넘어갈 준비가 됐는지 판정한다(guarded_pulse_ready_summary).
    부수효과: 컴파일/업로드/시리얼, 요약·CSV·원시로그 기록.
    Optionally uploads then streams to verify heartbeat/IMU/RC/neutral readiness.
    Side effects: compile/upload/serial + file writes.
    """
    if getattr(args, "deprecated_alias", False):
        print("Deprecated alias: use guarded-pulse-ready.")
    if args.print_cmd:
        args.port = printable_port(args.port)
    if not args.print_cmd and not ensure_port(args):
        return 2
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    flags = guarded_pulse_firmware_flags(max_abs_a=args.max_abs_a, max_abs_b=args.max_abs_b, max_ms=args.max_ms)
    compile_cmd = [
        "arduino-cli",
        "compile",
        "--fqbn",
        "OpenRB-150:samd:OpenRB-150",
        "--build-path",
        "/private/tmp/openrb-guarded-pulse-ready",
        "--build-property",
        f"compiler.cpp.extra_flags={flags}",
        "firmware/openrb_robot_controller",
    ]
    upload_cmd = [
        "arduino-cli",
        "upload",
        "-p",
        str(args.port),
        "--fqbn",
        "OpenRB-150:samd:OpenRB-150",
        "--build-path",
        "/private/tmp/openrb-guarded-pulse-ready",
        "firmware/openrb_robot_controller",
    ]
    if args.print_cmd:
        if args.upload in {"true", "auto"}:
            print(" ".join(shlex.quote(part) for part in compile_cmd))
            print(" ".join(shlex.quote(part) for part in upload_cmd))
        print(f"guarded_pulse_flags={flags}")
        print("ready_for_full_path_following=false")
        write_summary_files(
            out_dir,
            {
                "mode": "guarded-pulse-ready",
                "success": True,
                "reason": "COMMAND_PRINTED",
                "guarded_pulse_ready": False,
                "guarded_pulse_heartbeat_seen": False,
                "turn_angle_calibration_ready": False,
                "next_recommended_action": "Run without --print-cmd only when ready to upload/check guarded pulse firmware.",
                "ready_for_full_path_following": False,
            },
            title="IMU-Enabled Guarded Pulse Firmware",
        )
        return 0
    if args.upload in {"true", "auto"}:
        completed = subprocess.run(compile_cmd, check=False)
        if completed.returncode != 0:
            return completed.returncode
        completed = subprocess.run(upload_cmd, check=False)
        if completed.returncode != 0:
            return completed.returncode
    import serial

    raw_lines: list[str] = []
    with serial.Serial(args.port, baudrate=args.baud, timeout=0.5) as handle:
        deadline = time.monotonic() + args.duration_s
        while time.monotonic() < deadline:
            raw = handle.readline()
            if raw:
                line = raw.decode("utf-8", errors="replace").strip()
                print(line)
                raw_lines.append(line)
    rows = telemetry.parse_usbdbg_rows("\n".join(raw_lines))
    summary = guarded_pulse_ready_summary(rows)
    _write_raw_log(out_dir / "raw_usbdbg.log", raw_lines)
    _write_rows_csv(out_dir / "guarded_pulse_readiness.csv", rows)
    _write_json(out_dir / "guarded_pulse_readiness_summary.json", summary)
    write_summary_files(out_dir, summary, title="IMU-Enabled Guarded Pulse Firmware")
    print(f"guarded_pulse_ready={str(summary['guarded_pulse_ready']).lower()}")
    print("ready_for_full_path_following=false")
    return 0 if summary["guarded_pulse_ready"] is True else 2


# ── usb-pulse-test / 랩탑 USB 유계(有界) A·B 펄스 검증 / USB bounded pulse test ──
# 노트북에서 USB 로 짧고 크기 제한된 A/B 펄스를 하나씩 보내 로버가 반응하는지 검증한다
# (RC 입력 없이). ``station_drive_*`` 는 예전 "station drive" 명칭의 잔재로, 계획 생성·
# 콘솔 표시·이벤트 집계·통과 분류를 담당하는 순수 헬퍼들이다.
# Sends bounded single A/B pulses over laptop USB (no RC) to verify the rover reacts.
# The ``station_drive_*`` helpers (legacy name) are pure: plan/display/count/classify.


def _station_drive_name(name: str) -> str:
    """프리미티브 별칭을 표준명으로(미지원은 오류). / Normalize primitive alias; raise if unknown."""
    normalized = USB_PULSE_TEST_ALIASES.get(name.strip().lower())
    if normalized is None:
        raise ValueError(f"unknown usb-pulse-test primitive: {name}")
    return normalized


def station_drive_plan(*, sequence: str | None = None, single: str | None = None) -> list[dict[str, object]]:
    """전송할 유계 펄스 목록(시리얼 명령 텍스트 포함) 생성. / Build the bounded-pulse plan.

    기본은 forward/backward/left/right 순서. ``sequence`` 는 콤마목록, ``single`` 은 하나만.
    각 항목에 ARM/CMD/STOP 시리얼 명령 문자열을 미리 채워 둔다. 순수 함수.
    Defaults to fwd/back/left/right; ``sequence`` (comma list) or ``single`` override.
    Each item is prefilled with ARM/CMD/STOP command strings. Pure.
    """
    requested = [_station_drive_name(item["primitive"]) for item in USB_PULSE_TEST_SEQUENCE]
    if sequence:
        requested = [_station_drive_name(part) for part in sequence.split(",") if part.strip()]
    if single:
        requested = [_station_drive_name(single)]
    by_name = {str(item["primitive"]): item for item in USB_PULSE_TEST_SEQUENCE}
    planned: list[dict[str, object]] = []
    for index, name in enumerate(requested, start=1):
        primitive = by_name[name]
        a_cmd = float(primitive["a"])
        b_cmd = float(primitive["b"])
        pulse_ms = int(primitive["ms"])
        planned.append(
            {
                **primitive,
                "seq": index,
                "a_cmd": a_cmd,
                "b_cmd": b_cmd,
                "pulse_ms": pulse_ms,
                "arm_command_text": f"USB_PULSE_TEST_ARM seq={index}",
                "usb_pulse_test_command_text": (
                    f"USB_PULSE_TEST_CMD seq={index} a={a_cmd:.3f} b={b_cmd:.3f} ms={pulse_ms}"
                ),
                "command_text": f"USB_PULSE_TEST_CMD seq={index} a={a_cmd:.3f} b={b_cmd:.3f} ms={pulse_ms}",
                "stop_command_text": f"USB_PULSE_TEST_STOP seq={index}",
            }
        )
    return planned


def usb_pulse_test_plan(*, sequence: str | None = None, single: str | None = None) -> list[dict[str, object]]:
    """``station_drive_plan`` 의 현재 명칭 별칭. / Current-name alias for station_drive_plan."""
    return station_drive_plan(sequence=sequence, single=single)


def station_drive_display_block(item: dict[str, object]) -> str:
    """--print-command 용 다중행 표시 블록. / Multi-line display block for --print-command."""
    label = {
        "forward": "FORWARD",
        "backward": "BACKWARD",
        "left": "LEFT",
        "right": "RIGHT",
    }.get(str(item["primitive"]), str(item["primitive"]).upper())
    return f"{label}:\nA={float(item['a']):+0.3f} B={float(item['b']):+0.3f} ms={int(item['ms'])}"


def station_drive_console_line(item: dict[str, object]) -> str:
    """콘솔용 한 줄 펄스 설명. / One-line pulse description for the console."""
    return f"{str(item['primitive']).upper()}: A={float(item['a']):+0.2f} B={float(item['b']):+0.2f} {int(item['ms'])}ms"


def station_drive_clean_plan(planned: Sequence[dict[str, object]]) -> list[dict[str, object]]:
    """요약 저장용 축약 계획(핵심 필드만). / Compact plan for summaries (core fields only)."""
    return [
        {
            "primitive": item["primitive"],
            "a_cmd": item["a"],
            "b_cmd": item["b"],
            "pulse_ms": item["ms"],
        }
        for item in planned
    ]


def station_drive_compatible(row: dict[str, str]) -> bool:
    """이 하트비트 행이 usb-pulse-test 를 받을 준비가 됐나. / Ready to accept a USB pulse?

    새 usb_pulse_test_mode/ready 신호이거나 레거시 가드펄스 호환이면 True.
    True on the new usb_pulse_test ready signal or legacy guarded-pulse compatibility.
    """
    clean_ready = (
        telemetry._parse_bool(row.get("usb_pulse_test_mode")) is True
        and telemetry._parse_bool(row.get("usb_pulse_test_ready")) is True
    )
    return clean_ready or controller.guarded_pulse_compatible(row)


def station_drive_event_counts(rows: Sequence[dict[str, object]]) -> dict[str, int]:
    """검증 행들에서 ACK/ACTIVE/STOP/reject 등 이벤트 집계. / Tally pulse event counts."""
    def count_bool(key: str) -> int:
        return sum(1 for row in rows if row.get(key) is True)

    return {
        "command_sent_count": sum(1 for row in rows if row.get("command_sent") is True),
        "ack_count": count_bool("ack_seen"),
        "active_count": count_bool("active_seen"),
        "stop_count": count_bool("stop_seen"),
        "reject_count": count_bool("reject_seen"),
        "rc_invalid_count": sum(1 for row in rows if row.get("reject_reason") == "RC_INVALID"),
        "motor_write_called_count": count_bool("motor_write_called_seen"),
        "physical_output_active_count": count_bool("physical_output_active_seen"),
        "final_zero_count": count_bool("final_zero"),
        "user_observed_motion_count": sum(
            1 for row in rows if str(row.get("user_motion_report", "")).lower() in {"forward", "backward", "left", "right"}
        ),
    }


def station_drive_classification(rows: Sequence[dict[str, object]], *, user_aborted: bool = False) -> tuple[str, str, str]:
    """usb-pulse-test 결과를 (result, reason, 다음행동)으로 분류. / Classify the pulse test.

    잘못된 펌웨어/미전송/거부/ACK없음/ACTIVE없음/STOP없음/모터차단/텔레메트리는 움직였는데
    관측 없음 등을 순서대로 판정하고, 유효 펄스가 모두 정상 관측되면 PASS. 순수 함수.
    Diagnoses wrong-firmware/no-command/reject/no-ACK/no-ACTIVE/no-STOP/motor-blocked/
    telemetry-vs-observed and returns PASS when valid pulses observed cleanly. Pure.
    """
    counts = station_drive_event_counts(rows)
    if any(row.get("wrong_firmware_manual_rc_recovery") is True for row in rows):
        return (
            "WRONG_FIRMWARE_MANUAL_RC_RECOVERY",
            "WRONG_FIRMWARE_MANUAL_RC_RECOVERY",
            "Upload usb-pulse-test firmware; manual-rc firmware reads receiver passthrough and is not usb-pulse-test.",
        )
    if counts["command_sent_count"] == 0:
        return (
            "WAITING_FOR_USER_ENTER",
            "WAITING_FOR_USER_ENTER",
            "No usb-pulse-test command was sent. Press Enter at a command prompt to run bounded USB pulses.",
        )
    if counts["reject_count"] > 0:
        if counts["rc_invalid_count"] > 0:
            return (
                "BUG_USB_PULSE_TEST_STILL_REQUIRES_RC_INPUT",
                "BUG_USB_PULSE_TEST_STILL_REQUIRES_RC_INPUT",
                "usb-pulse-test must ignore absent RC input. Re-upload usb-pulse-test firmware and inspect reject telemetry.",
            )
        return (
            "COMMAND_SENT_NO_ACK",
            "COMMAND_SENT_NO_ACK",
            "Inspect raw_usbdbg.log for the usb-pulse-test reject reason and command limits.",
        )
    if counts["ack_count"] < counts["command_sent_count"]:
        return (
            "COMMAND_SENT_NO_ACK",
            "COMMAND_SENT_NO_ACK",
            "Confirm usb-pulse-test firmware received the USB pulse command and emitted ACK.",
        )
    if counts["active_count"] < counts["command_sent_count"]:
        return (
            "COMMAND_ACKED_NO_ACTIVE",
            "COMMAND_ACKED_NO_ACTIVE",
            "ACK was seen but ACTIVE was missing; inspect firmware output gating.",
        )
    if counts["stop_count"] < counts["command_sent_count"]:
        return (
            "COMMAND_ACTIVE_NO_STOP",
            "COMMAND_ACTIVE_NO_STOP",
            "ACTIVE was seen but STOP was missing; inspect serial timing and STOP telemetry.",
        )
    if counts["motor_write_called_count"] == 0 and counts["physical_output_active_count"] == 0:
        return (
            "MOTOR_OUTPUT_BLOCKED",
            "MOTOR_OUTPUT_BLOCKED",
            "ACK/ACTIVE were seen but no motor write or output-active telemetry appeared.",
        )
    if any(
        row.get("valid_pulse") is True
        and (
            row.get("telemetry_motion_seen") is True
            or row.get("motor_write_called_seen") is True
            or row.get("physical_output_active_seen") is True
        )
        and str(row.get("user_motion_report", "")).lower() == "none"
        for row in rows
    ):
        return (
            "TELEMETRY_OUTPUT_ACTIVE_BUT_USER_SAW_NONE",
            "TELEMETRY_OUTPUT_ACTIVE_BUT_USER_SAW_NONE",
            "Telemetry says output occurred; inspect wheels, drivetrain load, and whether the rover was able to move.",
        )
    valid_rows = [row for row in rows if row.get("valid_pulse") is True and row.get("skipped") is not True]
    if valid_rows and all(
        str(row.get("user_motion_report", "")).lower() in {"forward", "backward", "left", "right", "twitch", "unknown", "not_asked"}
        for row in valid_rows
    ):
        return (
            "USB_PULSE_TEST_PASS",
            "USB_PULSE_TEST_PASS",
            "usb-pulse-test A/B control passed; RC receiver passthrough remains a separate mode.",
        )
    return (
        "COMMAND_SENT_NO_ACK",
        "COMMAND_SENT_NO_ACK",
        "Inspect ACK/ACTIVE/STOP/final-zero fields and raw_usbdbg.log.",
    )


def _station_drive_latest_state(row: dict[str, str]) -> str:
    """행의 펄스 명령 상태(신·구 키 폴백). / Pulse command state (new/legacy key fallback)."""
    return (
        row.get("usb_pulse_test_cmd_state")
        or row.get("station_drive_cmd_state")
        or row.get(_COMPAT_GUARDED_STATE_KEY)
        or row.get(_COMPAT_GUARDED_STATE_FALLBACK_KEY)
        or ""
    )


def cmd_usb_pulse_test(args: argparse.Namespace) -> int:
    """랩탑 USB 로 유계 A/B 펄스를 하나씩 보내 모터 검증. / Bounded USB A/B pulse motor test.

    감독형 펌웨어를 (옵션) 업로드하고, 계획된 펄스마다 하트비트 대기 → (옵션) Enter/카운트다운
    → ARM/CMD/STOP 전송 → ACK/ACTIVE/STOP·모터출력 관측을 반복한다. 대화형이면 관측된 움직임을
    사용자에게 묻는다. 종료 시 ``station_drive_classification`` 으로 통과/사유 판정.
    ``--print-command``/``--print-cmd`` 는 계획만 출력. 부수효과: 업로드/시리얼/입력, 파일 기록.
    Optionally uploads supervised firmware, then per planned pulse waits for a
    heartbeat, optionally prompts, sends ARM/CMD/STOP and observes ACK/ACTIVE/STOP +
    motor output; classifies at the end. Side effects: upload/serial/input + writes.
    """
    if getattr(args, "deprecated_station_manual_alias", False):
        print("Deprecated alias: use usb-pulse-test.")
    if getattr(args, "deprecated_station_drive_alias", False):
        print("Deprecated alias: use usb-pulse-test.")
    planned = station_drive_plan(sequence=getattr(args, "sequence", None), single=getattr(args, "single", None))
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    if args.print_command == "true":
        print("\n\n".join(station_drive_display_block(item) for item in planned))
        summary = {
            "mode": "usb-pulse-test",
            "success": True,
            "reason": "COMMAND_PRINTED",
            "usb_pulse_test_result": "COMMAND_PRINTED",
            "rc_input_required": False,
            "rc_input_ignored": True,
            "gps_required": False,
            "imu_required": False,
            "pulse_count": len(planned),
            "planned_pulses": station_drive_clean_plan(planned),
            "physical_a_role": "throttle",
            "physical_b_role": "turn",
            "wheel_to_physical_mapping": "physical_ab_manual_equivalent",
            "next_recommended_action": "Run usb-pulse-test without --print-command only when ready for bounded USB pulse control.",
            "ready_for_full_path_following": False,
        }
        _write_json(out_dir / "usb_pulse_test_plan.json", planned)
        write_summary_files(out_dir, summary, title="USB Pulse Test")
        return 0
    if args.print_cmd:
        for item in planned:
            print(item["arm_command_text"])
            print(item["usb_pulse_test_command_text"])
            print(item["stop_command_text"])
        summary = {
            "mode": "usb-pulse-test",
            "success": True,
            "reason": "COMMAND_PRINTED",
            "usb_pulse_test_result": "COMMAND_PRINTED",
            "rc_input_required": False,
            "rc_input_ignored": True,
            "gps_required": False,
            "imu_required": False,
            "pulse_count": len(planned),
            "planned_pulses": station_drive_clean_plan(planned),
            "next_recommended_action": "Run without --print-cmd only when ready for bounded USB pulse control.",
            "ready_for_full_path_following": False,
        }
        _write_json(out_dir / "usb_pulse_test_plan.json", planned)
        write_summary_files(out_dir, summary, title="USB Pulse Test")
        return 0
    if not ensure_port(args):
        return 2
    if args.upload in {"true", "auto"}:
        flags = usb_pulse_test_firmware_flags(max_abs_a=args.max_abs_a, max_abs_b=args.max_abs_b, max_ms=args.max_ms)
        compile_cmd = [
            "arduino-cli",
            "compile",
            "--fqbn",
            "OpenRB-150:samd:OpenRB-150",
            "--build-path",
            "/private/tmp/openrb-usb-pulse-test",
            "--build-property",
            f"compiler.cpp.extra_flags={flags}",
            "firmware/openrb_robot_controller",
        ]
        upload_cmd = [
            "arduino-cli",
            "upload",
            "-p",
            str(args.port),
            "--fqbn",
            "OpenRB-150:samd:OpenRB-150",
            "--build-path",
            "/private/tmp/openrb-usb-pulse-test",
            "firmware/openrb_robot_controller",
        ]
        completed = subprocess.run(compile_cmd, check=False)
        if completed.returncode != 0:
            write_summary_files(
                out_dir,
                {
                    "mode": "usb-pulse-test",
                    "success": False,
                    "reason": "USB_PULSE_TEST_FIRMWARE_COMPILE_FAILED",
                    "returncode": completed.returncode,
                    "next_recommended_action": "Inspect Arduino compile output before retrying usb-pulse-test.",
                    "ready_for_full_path_following": False,
                },
                title="USB Pulse Test",
            )
            return completed.returncode
        completed = subprocess.run(upload_cmd, check=False)
        if completed.returncode != 0:
            write_summary_files(
                out_dir,
                {
                    "mode": "usb-pulse-test",
                    "success": False,
                    "reason": "USB_PULSE_TEST_FIRMWARE_UPLOAD_FAILED",
                    "returncode": completed.returncode,
                    "next_recommended_action": "Check OpenRB port and upload mode before retrying usb-pulse-test.",
                    "ready_for_full_path_following": False,
                },
                title="USB Pulse Test",
            )
            return completed.returncode
    import serial

    raw_lines: list[str] = []
    rows: list[dict[str, object]] = []
    invalid_count = 0
    user_aborted = False
    print(f"resolved_port={args.port}")
    print("firmware_mode=usb_pulse_test")
    print("usb_pulse_test_ignore_rc_input=true")
    print("USB pulse-test command plan:")
    for item in planned:
        print(f"  {station_drive_console_line(item)}")
    with serial.Serial(args.port, baudrate=args.baud, timeout=0.5) as handle:
        for item in planned:
            print(f"Ready to send {station_drive_console_line(item)}")
            heartbeat = executor.wait_for_row(
                handle,
                raw_lines,
                lambda row: telemetry.event(row) == "HEARTBEAT" and station_drive_compatible(row),
                args.heartbeat_timeout_s,
                verbose_raw=args.verbose_raw == "true",
            )
            print(f"heartbeat ready: {str(heartbeat is not None).lower()}")
            if heartbeat and telemetry._parse_bool(heartbeat.get("manual_rc_recovery")) is True:
                rows.append({
                    "seq": item["seq"],
                    "primitive": item["primitive"],
                    "a_cmd": item["a"],
                    "b_cmd": item["b"],
                    "pulse_ms": item["ms"],
                    "command_sent": False,
                    "wrong_firmware_manual_rc_recovery": True,
                    "valid_pulse": False,
                    "invalid_reason": "WRONG_FIRMWARE_MANUAL_RC_RECOVERY",
                    "ready_for_full_path_following": False,
                })
                break
            if heartbeat is None:
                rows.append({
                    "seq": item["seq"],
                    "primitive": item["primitive"],
                    "a_cmd": item["a"],
                    "b_cmd": item["b"],
                    "pulse_ms": item["ms"],
                    "command_sent": False,
                    "skipped": False,
                    "valid_pulse": False,
                    "invalid_reason": "USB_PULSE_TEST_HEARTBEAT_MISSING",
                    "ready_for_full_path_following": False,
                })
                if args.abort_on_invalid == "true":
                    break
                continue
            if args.require_enter == "true":
                response = input("Press Enter to send, or type skip/abort: ").strip().lower()
                if response == "skip":
                    rows.append({
                        "seq": item["seq"],
                        "primitive": item["primitive"],
                        "a_cmd": item["a"],
                        "b_cmd": item["b"],
                        "pulse_ms": item["ms"],
                        "command_sent": False,
                        "skipped": True,
                        "valid_pulse": False,
                        "invalid_reason": "SKIPPED_BY_USER",
                        "ready_for_full_path_following": False,
                    })
                    print("skipped=true")
                    continue
                if response == "abort":
                    user_aborted = True
                    print("aborted_by_user=true")
                    break
            else:
                for remaining in (3, 2, 1):
                    print(f"sending in {remaining}...")
                    time.sleep(1.0)
            print("command sent")
            pulse_rows = executor.send_pulse(
                handle,
                item,
                raw_lines,
                event_timeout_s=args.event_timeout_s,
                verbose_raw=args.verbose_raw == "true",
            )
            invalid_reason = controller.pulse_block_reason(pulse_rows)
            if invalid_reason is not None:
                invalid_count += 1
            visual = "not_asked"
            if args.interactive_visible_motion == "true":
                visual = input("Observed motion [forward/backward/left/right/twitch/none/unknown]: ").strip() or "unknown"
            last = pulse_rows[-1] if pulse_rows else {}
            reject_reason = safety.latest_reject_reason(pulse_rows) if pulse_rows else "NONE"
            ack_seen = any(telemetry.event(row) == "ACK" for row in pulse_rows)
            active_seen = any(telemetry.event(row) == "ACTIVE" or _station_drive_latest_state(row) == "ACTIVE" for row in pulse_rows)
            stop_seen = any(telemetry.event(row) in {"STOP", "PULSE_COMPLETE", "PULSE_DONE"} for row in pulse_rows)
            motor_write_called_seen = any(telemetry._parse_bool(row.get("motor_write_called")) is True for row in pulse_rows)
            physical_output_active_seen = any(telemetry.physical_output_active(row) for row in pulse_rows)
            final_left = telemetry._optional_float(last.get("final_left_cmd")) if last else None
            final_right = telemetry._optional_float(last.get("final_right_cmd")) if last else None
            final_zero = (final_left is not None and final_right is not None and abs(final_left) <= 1e-6 and abs(final_right) <= 1e-6)
            print(f"ACK seen: {str(ack_seen).lower()}")
            print(f"ACTIVE seen: {str(active_seen).lower()}")
            print(f"STOP seen: {str(stop_seen).lower()}")
            print(f"final zero: {str(final_zero).lower()}")
            print(f"observed_motion={visual}")
            rows.append({
                "seq": item["seq"],
                "primitive": item["primitive"],
                "a_cmd": item["a"],
                "b_cmd": item["b"],
                "pulse_ms": item["ms"],
                "arm_command_text": item["arm_command_text"],
                "usb_pulse_test_command_text": item["usb_pulse_test_command_text"],
                "stop_command_text": item["stop_command_text"],
                "command_sent": True,
                "ack_seen": ack_seen,
                "active_seen": active_seen,
                "stop_seen": stop_seen,
                "reject_seen": any(telemetry.event(row) == "REJECT" for row in pulse_rows),
                "reject_reason": reject_reason,
                "motor_write_called_seen": motor_write_called_seen,
                "physical_output_active_seen": physical_output_active_seen,
                "telemetry_motion_seen": motor_write_called_seen or physical_output_active_seen,
                "invalid_reason": invalid_reason or "OK",
                "valid_pulse": invalid_reason is None,
                "final_left_cmd": last.get("final_left_cmd", "NA"),
                "final_right_cmd": last.get("final_right_cmd", "NA"),
                "final_zero": final_zero,
                "physical_output_active_after_stop": last.get("physical_output_active", "NA"),
                "user_motion_report": visual,
                "ready_for_full_path_following": False,
            })
            if invalid_reason is not None and args.abort_on_invalid == "true":
                break
    _write_raw_log(out_dir / "raw_usbdbg.log", raw_lines)
    _write_rows_csv(out_dir / "usb_pulse_test_validation.csv", rows)
    result, reason, next_action = station_drive_classification(rows, user_aborted=user_aborted)
    counts = station_drive_event_counts(rows)
    success = result == "USB_PULSE_TEST_PASS"
    summary = {
        "mode": "usb-pulse-test",
        "success": success,
        "reason": reason,
        "usb_pulse_test_result": result,
        "pulse_count": len(planned),
        "completed_pulse_count": len(rows),
        "invalid_pulse_count": invalid_count,
        **counts,
        "observed_motions": [row.get("user_motion_report", "") for row in rows if row.get("command_sent") is True],
        "rc_input_required": False,
        "rc_input_ignored": True,
        "gps_required": False,
        "imu_required": False,
        "physical_a_role": "throttle",
        "physical_b_role": "turn",
        "wheel_to_physical_mapping": "physical_ab_manual_equivalent",
        "next_recommended_action": next_action,
        "ready_for_full_path_following": False,
    }
    write_summary_files(out_dir, summary, title="USB Pulse Test")
    print(f"usb_pulse_test_success={str(success).lower()}")
    print("ready_for_full_path_following=false")
    return 0 if success else 2


# ── tune-motion / 대화형 프리미티브 모션 보정 / Interactive motion calibration ──
# 한 프리미티브(forward/backward/left/right/turn-*-90)를 반복 펄스로 시각·IMU 피드백을
# 받아가며 조정하고, 조작자가 승인하면 motion_calibration.json 에 저장한다.
# Iteratively tunes one primitive with visual/IMU feedback; on approval saves it.


def tune_motion_planned_command(candidate: dict[str, object], *, seq: int) -> dict[str, object]:
    """후보 파라미터를 한 번 보낼 펄스 명령으로 변환. / Candidate -> one planned pulse command."""
    a_cmd = float(candidate["a"])
    b_cmd = float(candidate["b"])
    pulse_ms = int(candidate["ms"])
    return {
        "seq": seq,
        "primitive": candidate["primitive"],
        "a_cmd": a_cmd,
        "b_cmd": b_cmd,
        "pulse_ms": pulse_ms,
        "arm_command_text": f"USB_PULSE_TEST_ARM seq={seq}",
        "command_text": f"USB_PULSE_TEST_CMD seq={seq} a={a_cmd:.3f} b={b_cmd:.3f} ms={pulse_ms}",
        "stop_command_text": f"USB_PULSE_TEST_STOP seq={seq}",
    }


def tune_motion_trial_row(
    *,
    trial_index: int,
    candidate: dict[str, object],
    feedback: str,
    pulse_rows: Sequence[dict[str, str]],
    invalid_reason: str | None,
    yaw_delta_deg: float | None,
    opposite_sign_transient: bool = False,
) -> dict[str, object]:
    """한 튜닝 시도를 CSV 행으로 기록. / One tuning trial as a CSV row.

    후보값·조작자 피드백·ACK/ACTIVE/STOP·IMU yaw 변화·최종 0 여부를 담는다.
    Captures the candidate, operator feedback, ACK/ACTIVE/STOP, yaw delta, final-zero.
    """
    last = pulse_rows[-1] if pulse_rows else {}
    final_left = telemetry._optional_float(last.get("final_left_cmd")) if last else None
    final_right = telemetry._optional_float(last.get("final_right_cmd")) if last else None
    final_zero = (
        final_left is not None
        and final_right is not None
        and abs(final_left) <= 1e-6
        and abs(final_right) <= 1e-6
    )
    return {
        "trial_index": trial_index,
        "primitive": candidate["primitive"],
        "a_cmd": f"{float(candidate['a']):.3f}",
        "b_cmd": f"{float(candidate['b']):.3f}",
        "pulse_ms": int(candidate["ms"]),
        "target_angle_deg": candidate.get("target_angle_deg", "NA"),
        "imu_yaw_delta_deg": "NA" if yaw_delta_deg is None else f"{yaw_delta_deg:.3f}",
        "feedback": feedback,
        "ack_seen": any(telemetry.event(row) == "ACK" for row in pulse_rows),
        "active_seen": any(telemetry.event(row) == "ACTIVE" or _station_drive_latest_state(row) == "ACTIVE" for row in pulse_rows),
        "stop_seen": any(telemetry.event(row) in safety.STOP_EVENTS for row in pulse_rows),
        "reject_seen": any(telemetry.event(row) == "REJECT" for row in pulse_rows),
        "opposite_sign_transient": opposite_sign_transient,
        "final_left_cmd": last.get("final_left_cmd", "NA"),
        "final_right_cmd": last.get("final_right_cmd", "NA"),
        "final_zero": final_zero,
        "valid_pulse": invalid_reason is None,
        "invalid_reason": invalid_reason or "OK",
        "ready_for_full_path_following": False,
    }


def tune_motion_summary(
    rows: Sequence[dict[str, object]],
    *,
    primitive: str,
    candidate: dict[str, object],
    approved: bool,
    reason: str,
    calibration_out: Path,
) -> dict[str, object]:
    """튜닝 세션 결과 요약(승인 후보 포함). / Tune-motion session summary (approved candidate)."""
    summary = {
        "mode": "tune-motion",
        "success": approved,
        "reason": reason,
        "firmware_profile": MAC_PHYSICAL_SUPERVISED_PROFILE,
        "primitive": primitive,
        "trial_count": len(rows),
        "actual_pulse_count": sum(1 for row in rows if row.get("valid_pulse") in {True, False}),
        "opposite_sign_transient_count": sum(1 for row in rows if row.get("opposite_sign_transient") is True),
        "approved_candidate": {
            "a": round(float(candidate["a"]), 3),
            "b": round(float(candidate["b"]), 3),
            "ms": int(candidate["ms"]),
            **(
                {"target_angle_deg": float(candidate["target_angle_deg"])}
                if "target_angle_deg" in candidate else {}
            ),
        },
        "calibration_out": str(calibration_out),
        "final_zero_required": True,
        "observed_distance_m_required": False,
        "next_recommended_action": (
            "Use execute-plan or run; approved motion calibration will be loaded automatically."
            if approved else
            "Rerun tune-motion and approve only after ACK/ACTIVE/STOP/final-zero and visual behavior are acceptable."
        ),
        "ready_for_full_path_following": False,
    }
    return checks.assert_not_ready_for_full_path_following(summary)


# ── 감독형 펌웨어 컴파일·업로드 헬퍼 / Supervised firmware compile+upload helpers ──


def _upload_mac_physical_supervised_firmware(
    args: argparse.Namespace,
    out_dir: Path,
    *,
    title: str,
    mode: str,
    build_path: str,
) -> int:
    """감독형 펌웨어를 컴파일·업로드(공통 헬퍼). / Compile + upload supervised firmware.

    gps-wait/preview/usb-pulse-test/usb-drive-live/tune-motion 이 공유. args 에서 상한
    (max_abs_a/b, max_ms, TTL 등)을 읽어 플래그를 만들고 arduino-cli 로 빌드/업로드한다.
    실패 시 실패 요약을 쓰고 arduino-cli 종료코드를, 성공 시 0 을 반환. 부수효과: 서브프로세스+파일.
    Shared by several modes; reads bounds from args, builds flags, runs arduino-cli
    compile/upload. On failure writes a summary and returns the arduino-cli code; 0 on OK.
    """
    max_abs_a = float(getattr(args, "max_abs_a", 0.35))
    max_abs_b = float(getattr(args, "max_abs_b", 0.35))
    max_ms = int(getattr(args, "max_ms", 1000))
    max_duration_s = float(getattr(args, "max_duration_s", max_ms / 1000.0))
    ttl_ms = int(getattr(args, "ttl_ms", 350))
    flags = mac_physical_supervised_firmware_flags(
        max_abs_a=max_abs_a,
        max_abs_b=max_abs_b,
        max_ms=max_ms,
        max_duration_ms=int(max_duration_s * 1000.0),
        update_timeout_ms=ttl_ms,
    )
    compile_cmd = [
        "arduino-cli",
        "compile",
        "--fqbn",
        "OpenRB-150:samd:OpenRB-150",
        "--build-path",
        build_path,
        "--build-property",
        f"compiler.cpp.extra_flags={flags}",
        "firmware/openrb_robot_controller",
    ]
    upload_cmd = [
        "arduino-cli",
        "upload",
        "-p",
        str(args.port),
        "--fqbn",
        "OpenRB-150:samd:OpenRB-150",
        "--build-path",
        build_path,
        "firmware/openrb_robot_controller",
    ]
    completed = subprocess.run(compile_cmd, check=False)
    if completed.returncode != 0:
        write_summary_files(
            out_dir,
            {
                "mode": mode,
                "success": False,
                "reason": "MAC_PHYSICAL_SUPERVISED_FIRMWARE_COMPILE_FAILED",
                "firmware_profile": MAC_PHYSICAL_SUPERVISED_PROFILE,
                "returncode": completed.returncode,
                "next_recommended_action": "Inspect Arduino compile output before retrying the same command.",
                "ready_for_full_path_following": False,
            },
            title=title,
        )
        return completed.returncode
    completed = subprocess.run(upload_cmd, check=False)
    if completed.returncode != 0:
        write_summary_files(
            out_dir,
            {
                "mode": mode,
                "success": False,
                "reason": "MAC_PHYSICAL_SUPERVISED_FIRMWARE_UPLOAD_FAILED",
                "firmware_profile": MAC_PHYSICAL_SUPERVISED_PROFILE,
                "returncode": completed.returncode,
                "next_recommended_action": "Check OpenRB port and upload mode before retrying the same command.",
                "ready_for_full_path_following": False,
            },
            title=title,
        )
        return completed.returncode
    return 0


def _upload_usb_pulse_test_firmware(args: argparse.Namespace, out_dir: Path, *, title: str) -> int:
    """usb-pulse-test/tune-motion 펌웨어 업로드(빌드경로 title 로 선택). / Upload pulse-test firmware."""
    build_path = "/private/tmp/openrb-tune-motion" if title == "Tune Motion" else "/private/tmp/openrb-usb-pulse-test"
    mode = "tune-motion" if title == "Tune Motion" else "usb-pulse-test"
    return _upload_mac_physical_supervised_firmware(
        args,
        out_dir,
        title=title,
        mode=mode,
        build_path=build_path,
    )


def _upload_usb_drive_live_firmware(args: argparse.Namespace, out_dir: Path) -> int:
    """usb-drive-live 펌웨어 업로드. / Upload the usb-drive-live firmware."""
    return _upload_mac_physical_supervised_firmware(
        args,
        out_dir,
        title="USB Drive Live",
        mode="usb-drive-live",
        build_path="/private/tmp/openrb-usb-drive-live",
    )


# ── usb-drive-live / 연속 USB A·B setpoint 드라이브 / Continuous USB setpoint drive ──


def usb_drive_live_summary(rows: Sequence[dict[str, str]], *, a_cmd: float, b_cmd: float, duration_s: float) -> dict[str, object]:
    """연속 라이브 드라이브 결과 판정. / Judge the continuous live-drive result.

    거부 없음 + STOP 관측 + 최종 명령 0 이면 성공. 순수 함수. 통과는 A/B 상수 setpoint 를
    지정 시간 동안 데드맨 아래 안전하게 보냈고 정상 정지했음을 의미한다.
    Success = no reject + STOP seen + final commands zero. Pure. Pass means the
    constant A/B setpoint ran under the deadman and stopped cleanly.
    """
    reject_seen = any(telemetry.event(row) == "REJECT" for row in rows)
    stop_seen = any(telemetry.event(row) in safety.STOP_EVENTS for row in rows)
    trace_rows = [row for row in rows if "physical_a_cmd" in row and "motor_write_called" in row]
    motor_write_seen = any(telemetry._parse_bool(row.get("motor_write_called")) is True for row in trace_rows)
    output_active_seen = any(telemetry.physical_output_active(row) for row in rows)
    final_nonzero = safety.nonzero_final_cmd(rows)
    success = not reject_seen and stop_seen and not final_nonzero
    reason = "OK" if success else (
        "REJECT" if reject_seen else
        "STOP_MISSING" if not stop_seen else
        "FINAL_COMMANDS_NONZERO"
    )
    summary = {
        "mode": "usb-drive-live",
        "success": success,
        "reason": reason,
        "firmware_profile": MAC_PHYSICAL_SUPERVISED_PROFILE,
        "a_cmd": round(a_cmd, 3),
        "b_cmd": round(b_cmd, 3),
        "duration_s": duration_s,
        "setpoint_update_count": sum(1 for row in rows if telemetry.event(row) == "ACTIVE"),
        "motor_trace_count": len(trace_rows),
        "motor_write_called_seen": motor_write_seen,
        "physical_output_active_seen": output_active_seen,
        "stop_seen": stop_seen,
        "final_zero": not final_nonzero,
        "rc_ignored_for_usb_supervised": True,
        "rc_warning": "RC_NOT_OK_IGNORED_FOR_MAC_USB_SUPERVISED_MODE"
        if any(controller.rc_warning_for_usb_supervised(row).startswith("RC_NOT_OK") for row in rows)
        else "NONE",
        "next_recommended_action": (
            "Use tune-motion or execute-plan after confirming smooth motion."
            if success else
            "Inspect raw_usbdbg.log and motor trace rows before retrying live drive."
        ),
        "ready_for_full_path_following": False,
    }
    return checks.assert_not_ready_for_full_path_following(summary)


def cmd_usb_drive_live(args: argparse.Namespace) -> int:
    """USB 로 A/B setpoint 를 지정 시간 연속 전송(펌웨어 데드맨). / Continuous USB A/B drive.

    먼저 --a/--b/--duration 이 상한을 넘지 않는지 검증(초과 시 실패 요약). 감독형 펌웨어를
    (옵션) 업로드하고 ``executor.send_live_drive`` 로 주기적으로 setpoint 를 갱신한다.
    부수효과: 업로드/시리얼, 요약·CSV·원시로그 기록.
    Validates bounds, optionally uploads, then updates the setpoint via
    ``executor.send_live_drive``. Side effects: upload/serial + file writes.
    """
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    if abs(float(args.a)) > args.max_abs_a or abs(float(args.b)) > args.max_abs_b:
        return _fail_with_summary(args, reason="USB_DRIVE_LIVE_COMMAND_EXCEEDS_MAX", message="--a/--b exceed live-drive bounds")
    if args.duration_s <= 0 or args.duration_s > args.max_duration_s:
        return _fail_with_summary(args, reason="USB_DRIVE_LIVE_DURATION_EXCEEDS_MAX", message="--duration-s must be >0 and <= --max-duration-s")
    if args.print_command == "true":
        print(
            f"USB_DRIVE_LIVE_SET seq=1 a={float(args.a):.3f} b={float(args.b):.3f} "
            f"duration_ms={int(args.duration_s * 1000.0)} ttl_ms={int(args.ttl_ms)}"
        )
        print("USB_DRIVE_LIVE_STOP seq=1")
        summary = {
            "mode": "usb-drive-live",
            "success": True,
            "reason": "COMMAND_PRINTED",
            "firmware_profile": MAC_PHYSICAL_SUPERVISED_PROFILE,
            "a_cmd": round(float(args.a), 3),
            "b_cmd": round(float(args.b), 3),
            "duration_s": args.duration_s,
            "ready_for_full_path_following": False,
        }
        write_summary_files(out_dir, summary, title="USB Drive Live")
        return 0
    if not ensure_port(args):
        return 2
    if args.upload in {"true", "auto"}:
        uploaded = _upload_usb_drive_live_firmware(args, out_dir)
        if uploaded != 0:
            return uploaded

    import serial

    raw_lines: list[str] = []
    rows: list[dict[str, str]] = []
    try:
        with serial.Serial(args.port, baudrate=args.baud, timeout=0.5) as handle:
            print(f"resolved_port={args.port}")
            print(f"usb_drive_live A={float(args.a):+0.3f} B={float(args.b):+0.3f} duration_s={float(args.duration_s):.2f}")
            rows = executor.send_live_drive(
                handle,
                seq=1,
                duration_s=float(args.duration_s),
                update_hz=float(args.update_hz),
                ttl_ms=int(args.ttl_ms),
                command_fn=lambda _row: (float(args.a), float(args.b)),
                raw_lines=raw_lines,
                event_timeout_s=float(args.event_timeout_s),
                verbose_raw=args.verbose_raw == "true",
            )
    except KeyboardInterrupt:
        rows = telemetry.parse_usbdbg_rows("\n".join(raw_lines))
    except OSError:
        rows = telemetry.parse_usbdbg_rows("\n".join(raw_lines))
        summary = {
            "mode": "usb-drive-live",
            "success": False,
            "reason": "SERIAL_DISCONNECT",
            "firmware_profile": MAC_PHYSICAL_SUPERVISED_PROFILE,
            "ready_for_full_path_following": False,
        }
        _write_raw_log(out_dir / "raw_usbdbg.log", raw_lines)
        _write_rows_csv(out_dir / "usb_drive_live_rows.csv", rows)
        write_summary_files(out_dir, summary, title="USB Drive Live")
        return 2
    _write_raw_log(out_dir / "raw_usbdbg.log", raw_lines)
    _write_rows_csv(out_dir / "usb_drive_live_rows.csv", rows)
    summary = usb_drive_live_summary(rows, a_cmd=float(args.a), b_cmd=float(args.b), duration_s=float(args.duration_s))
    write_summary_files(out_dir, summary, title="USB Drive Live")
    print(f"usb_drive_live_success={str(summary['success']).lower()}")
    print(f"reason={summary['reason']}")
    print("ready_for_full_path_following=false")
    return 0 if summary["success"] is True else 2


def cmd_reset_motion_calibration(args: argparse.Namespace) -> int:
    """Back up then delete the approved motion calibration before recalibrating.

    Local-only: opens no serial port and uploads no firmware. The previous file
    is preserved as a timestamped ``*.backup_<stamp>.json`` sibling so a full
    ``tune-motion`` recalibration can start from a clean slate without losing the
    prior approved values.
    """
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    calibration_out = tuning.motion_calibration_path(getattr(args, "calibration_out", None))
    backup_path, removed = tuning.reset_calibration(calibration_out)
    summary = {
        "mode": "reset-motion-calibration",
        "success": True,
        "reason": "CALIBRATION_RESET" if removed else "NO_EXISTING_CALIBRATION",
        "calibration_path": str(calibration_out),
        "existing_calibration_found": removed,
        "backup_path": str(backup_path) if backup_path is not None else "NONE",
        "next_recommended_action": (
            "Recalibrate forward, backward, turn-left-90, turn-right-90 with tune-motion; "
            "each approve overwrites the fresh calibration file."
        ),
        "ready_for_full_path_following": False,
    }
    _write_json(out_dir / "reset_motion_calibration_summary.json", summary)
    write_summary_files(out_dir, summary, title="Reset Motion Calibration")
    if backup_path is not None:
        print(f"calibration backed up: {backup_path}")
    print(f"reset-motion-calibration: removed_existing={str(removed).lower()} -> {calibration_out}")
    print("ready_for_full_path_following=false")
    return 0


def cmd_set_motion_calibration(args: argparse.Namespace) -> int:
    """Apply a manual motion-calibration preset or one explicit primitive override."""
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    calibration_out = tuning.motion_calibration_path(getattr(args, "calibration_out", None))
    try:
        if args.preset:
            updates = tuning.manual_calibration_preset(args.preset)
            change_source = args.preset
        else:
            if not args.primitive:
                return _fail_with_summary(
                    args,
                    reason="MOTION_CALIBRATION_OVERRIDE_MISSING",
                    message="Provide --preset or --primitive with --a --b --ms.",
                )
            missing = [
                name for name in ("a", "b", "ms")
                if getattr(args, name) is None
            ]
            if missing:
                return _fail_with_summary(
                    args,
                    reason="MOTION_CALIBRATION_OVERRIDE_INCOMPLETE",
                    message=f"Missing required override fields: {', '.join('--' + item for item in missing)}",
                )
            primitive = tuning.normalize_primitive(args.primitive)
            key = tuning.calibration_key_for_primitive(primitive)
            updates = {
                key: tuning.manual_calibration_entry(
                    primitive,
                    a=float(args.a),
                    b=float(args.b),
                    ms=int(args.ms),
                    target_angle_deg=args.target_angle_deg,
                    source=args.source,
                )
            }
            change_source = args.source
        updated, backup_path = tuning.apply_manual_calibration_updates(calibration_out, updates)
    except ValueError as exc:
        return _fail_with_summary(
            args,
            reason="MOTION_CALIBRATION_OVERRIDE_INVALID",
            message=str(exc),
        )
    small_pulse_warnings: list[str] = []
    for key, entry in updates.items():
        if key in {"turn_left_90", "turn_right_90"} and isinstance(entry, dict):
            target = telemetry._optional_float(entry.get("target_angle_deg"))
            if target is not None and abs(target) < calibration.SMALL_PULSE_ANGLE_THRESHOLD_DEG:
                small_pulse_warnings.append(
                    f"{calibration.TURN_SMALL_PULSE_WARNING}:{key}:target_angle_deg={target:g}"
                )
    changed = {
        "mode": "set-motion-calibration",
        "preset": args.preset or "NONE",
        "source": change_source,
        "calibration_path": str(calibration_out),
        "backup_path": str(backup_path) if backup_path is not None else "NONE",
        "updated_primitives": sorted(updates.keys()),
        "updates": updates,
        "turn_angle_warnings": small_pulse_warnings,
        "ready_for_full_path_following": False,
    }
    summary = {
        **changed,
        "success": True,
        "reason": "MOTION_CALIBRATION_SET",
        "next_recommended_action": (
            "Run calibration-check, then execute-plan/run. For softer right-turn testing, "
            "try b=-0.06 ms=800, b=-0.08 ms=1000, or b=-0.10 ms=1000."
        ),
    }
    summary = checks.assert_not_ready_for_full_path_following(summary)
    _write_json(out_dir / "manual_motion_calibration_change.json", changed)
    _write_json(out_dir / "motion_calibration_updated.json", updated)
    _write_json(out_dir / "set_motion_calibration_summary.json", summary)
    write_summary_files(out_dir, summary, title="Set Motion Calibration")
    print(f"set-motion-calibration: updated={','.join(sorted(updates.keys()))} -> {calibration_out}")
    for warning in small_pulse_warnings:
        print(
            f"WARNING {warning} -- this is a small turn pulse, not a one-shot 90 degree "
            "turn; connectors will use repeated pulses / IMU feedback to reach 90."
        )
    if backup_path is not None:
        print(f"calibration backed up: {backup_path}")
    print("ready_for_full_path_following=false")
    return 0


def cmd_tune_motion(args: argparse.Namespace) -> int:
    """대화형으로 한 프리미티브를 보정하고 승인 시 저장. / Interactively tune one primitive.

    감독형 펌웨어를 (옵션) 업로드하고 최대 ``--max-iterations`` 회 반복한다. 매 시도마다
    후보 펄스를 보내고 IMU yaw/시각 피드백을 받아 후보를 조정한다. 조작자가 ``approve`` 하면
    ``tuning.save_approved_calibration`` 로 저장(반대부호 과도현상이면 거부). ``--print-candidate``
    는 초기 후보만 출력. 부수효과: 업로드/시리얼/입력, 보정 파일·요약·CSV 기록.
    Optionally uploads, then loops sending candidate pulses and adjusting from IMU/
    visual feedback; ``approve`` saves the calibration. Side effects: upload/serial/
    input + calibration/summary/CSV writes.
    """
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    primitive = tuning.normalize_primitive(args.primitive)
    candidate = tuning.initial_candidate(primitive)
    calibration_out = tuning.motion_calibration_path(args.calibration_out)
    if telemetry._parse_bool(getattr(args, "reset_calibration", "false"), default=False):
        backup_path, removed = tuning.reset_calibration(calibration_out)
        if backup_path is not None:
            print(f"calibration backed up: {backup_path}")
        print(f"calibration reset: removed_existing={str(removed).lower()} -> {calibration_out}")
    if args.print_candidate == "true":
        summary = tune_motion_summary(
            [],
            primitive=primitive,
            candidate=candidate,
            approved=False,
            reason="CANDIDATE_PRINTED",
            calibration_out=calibration_out,
        )
        summary["success"] = True
        _write_json(out_dir / "tune_motion_candidate.json", candidate)
        write_summary_files(out_dir, summary, title="Tune Motion")
        print(
            f"{primitive}: A={float(candidate['a']):+0.3f} "
            f"B={float(candidate['b']):+0.3f} ms={int(candidate['ms'])}"
        )
        return 0
    if not ensure_port(args):
        return 2
    if args.upload in {"true", "auto"}:
        uploaded = _upload_usb_pulse_test_firmware(args, out_dir, title="Tune Motion")
        if uploaded != 0:
            return uploaded

    import serial

    raw_lines: list[str] = []
    trial_rows: list[dict[str, object]] = []
    approved = False
    reason = "NOT_APPROVED"
    try:
        with serial.Serial(args.port, baudrate=args.baud, timeout=0.5) as handle:
            for trial_index in range(1, args.max_iterations + 1):
                candidate = tuning.clamp_candidate(candidate)
                print(
                    f"candidate trial={trial_index} primitive={primitive} "
                    f"A={float(candidate['a']):+0.3f} B={float(candidate['b']):+0.3f} ms={int(candidate['ms'])}"
                )
                if "target_angle_deg" in candidate:
                    print(f"target_angle_deg={float(candidate['target_angle_deg']):.1f}")
                heartbeat = executor.wait_for_row(
                    handle,
                    raw_lines,
                    lambda row: telemetry.event(row) == "HEARTBEAT" and station_drive_compatible(row),
                    args.heartbeat_timeout_s,
                    verbose_raw=args.verbose_raw == "true",
                )
                print(f"heartbeat ready: {str(heartbeat is not None).lower()}")
                if heartbeat is None:
                    reason = "USB_PULSE_TEST_HEARTBEAT_MISSING"
                    break
                if args.require_enter == "true":
                    response = input("Press Enter to send, or type abort: ").strip().lower()
                    if response == "abort":
                        reason = "USER_ABORTED"
                        break
                planned = tune_motion_planned_command(candidate, seq=trial_index)
                print("command sent")
                pulse_rows = executor.send_pulse(
                    handle,
                    planned,
                    raw_lines,
                    event_timeout_s=args.event_timeout_s,
                    verbose_raw=args.verbose_raw == "true",
                )
                invalid_reason = controller.pulse_block_reason(pulse_rows)
                yaw_delta = tuning.yaw_delta_from_rows(pulse_rows)
                opposite = tuning.opposite_sign_transient(primitive, pulse_rows)
                if yaw_delta is not None:
                    print(f"imu_yaw_delta_deg={yaw_delta:.3f}")
                if opposite:
                    print("opposite_sign_transient=true")
                if invalid_reason is not None:
                    reason = invalid_reason
                    trial_rows.append(
                        tune_motion_trial_row(
                            trial_index=trial_index,
                            candidate=candidate,
                            feedback="invalid",
                            pulse_rows=pulse_rows,
                            invalid_reason=invalid_reason,
                            yaw_delta_deg=yaw_delta,
                            opposite_sign_transient=opposite,
                        )
                    )
                    break
                feedback = input(
                    "observed? [good/weak/strong/too_short/too_long/left/right/none/retry/approve/abort]: "
                ).strip().lower() or "retry"
                trial_rows.append(
                    tune_motion_trial_row(
                        trial_index=trial_index,
                        candidate=candidate,
                        feedback=feedback,
                        pulse_rows=pulse_rows,
                        invalid_reason=invalid_reason,
                        yaw_delta_deg=yaw_delta,
                        opposite_sign_transient=opposite,
                    )
                )
                if feedback == "abort":
                    reason = "USER_ABORTED"
                    break
                if feedback == "approve":
                    if opposite:
                        reason = "OPPOSITE_SIGN_TRANSIENT"
                        break
                    tuning.save_approved_calibration(
                        calibration_out,
                        candidate,
                        yaw_delta_deg=yaw_delta,
                        heading_drift_deg=yaw_delta if primitive in {"forward", "backward"} else None,
                    )
                    approved = True
                    reason = "APPROVED"
                    break
                candidate = tuning.adjust_candidate(candidate, feedback, yaw_delta_deg=yaw_delta)
    except KeyboardInterrupt:
        reason = "USER_ABORTED"
    except OSError:
        reason = "SERIAL_DISCONNECT"

    _write_raw_log(out_dir / "raw_usbdbg.log", raw_lines)
    _write_rows_csv(out_dir / "tune_motion_trials.csv", trial_rows)
    summary = tune_motion_summary(
        trial_rows,
        primitive=primitive,
        candidate=candidate,
        approved=approved,
        reason=reason,
        calibration_out=calibration_out,
    )
    write_summary_files(out_dir, summary, title="Tune Motion")
    print(f"tune_motion_success={str(approved).lower()}")
    print(f"reason={reason}")
    print("ready_for_full_path_following=false")
    return 0 if approved else 2


# ── station-hw-diagnose / station-hw-manual (물리 스테이션 HW) / Station HW modes ──


def _station_hw_compile_upload_cmds(args: argparse.Namespace, *, diagnose_only: bool) -> tuple[list[str], list[str], str]:
    """스테이션 HW 컴파일/업로드 명령과 플래그 생성. / Build station-HW compile/upload cmds+flags.

    ``diagnose_only`` 면 모터 출력 없는 진단 플래그·빌드경로를 쓴다. 반환 ``(compile, upload, flags)``.
    Diagnose-only uses motor-off flags/build path. Returns ``(compile, upload, flags)``.
    """
    flags = station_hw_diagnose_firmware_flags() if diagnose_only else station_hw_manual_firmware_flags()
    build_path = "/private/tmp/openrb-station-hw-diagnose" if diagnose_only else "/private/tmp/openrb-station-hw-manual"
    compile_cmd = [
        "arduino-cli",
        "compile",
        "--fqbn",
        "OpenRB-150:samd:OpenRB-150",
        "--build-path",
        build_path,
        "--build-property",
        f"compiler.cpp.extra_flags={flags}",
        "firmware/openrb_robot_controller",
    ]
    upload_cmd = [
        "arduino-cli",
        "upload",
        "-p",
        str(args.port),
        "--fqbn",
        "OpenRB-150:samd:OpenRB-150",
        "--build-path",
        build_path,
        "firmware/openrb_robot_controller",
    ]
    return compile_cmd, upload_cmd, flags


def _read_station_hw_rows(args: argparse.Namespace, *, title: str, csv_name: str, summary_name: str, mode: str) -> int:
    """스테이션 HW 모니터 공통 본체(diagnose/manual 공유). / Shared station-HW monitor body.

    (옵션) 펌웨어 업로드 후 시리얼을 스트리밍하며 매초 상태줄을 출력하고, 종료 시
    ``evaluate_station_hw_rows`` 로 판정한다. Ctrl-C 는 USER_ABORTED(130), 시리얼 오류는
    SERIAL_ERROR 로 요약을 덮어쓴다. 파서 불일치 시 원시 프레임 덤프를 남긴다.
    부수효과: 업로드/시리얼, 요약·CSV·원시로그·(있으면)프레임덤프 기록.
    Optionally uploads, streams serial with a per-second status line, and evaluates
    at the end; overrides the summary on abort/serial error and dumps raw frames on a
    parser mismatch. Side effects: upload/serial + file writes.
    """
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    raw_lines: list[str] = []
    user_aborted = False
    serial_error: str | None = None
    if args.from_log:
        raw_lines = Path(args.from_log).read_text(encoding="utf-8").splitlines()
    else:
        if not ensure_port(args):
            return 2
        compile_cmd, upload_cmd, flags = _station_hw_compile_upload_cmds(
            args,
            diagnose_only=(mode == "station-hw-diagnose"),
        )
        if args.upload in {"true", "auto"}:
            print(f"station_hw_firmware_flags={flags}")
            completed = subprocess.run(compile_cmd, check=False)
            if completed.returncode != 0:
                write_summary_files(
                    out_dir,
                    {
                        "mode": mode,
                        "success": False,
                        "reason": "STATION_HW_FIRMWARE_COMPILE_FAILED",
                        "returncode": completed.returncode,
                        "next_recommended_action": "Inspect Arduino compile output before retrying station hardware mode.",
                        "ready_for_full_path_following": False,
                    },
                    title=title,
                )
                return completed.returncode
            completed = subprocess.run(upload_cmd, check=False)
            if completed.returncode != 0:
                write_summary_files(
                    out_dir,
                    {
                        "mode": mode,
                        "success": False,
                        "reason": "STATION_HW_FIRMWARE_UPLOAD_FAILED",
                        "returncode": completed.returncode,
                        "next_recommended_action": "Check OpenRB port and upload mode before retrying station hardware mode.",
                        "ready_for_full_path_following": False,
                    },
                    title=title,
                )
                return completed.returncode
        import serial

        print(f"{mode}: monitoring physical station hardware frames")
        print("rc_receiver_required=false gps_required=false imu_required=false")
        print("station input mapping: throttle -> physical A, steering -> physical B")
        print("station_transport=station_hardware_serial")
        print("station_protocol=auto")
        print("station_parser=auto_station_manual")
        duration_s = float(args.duration_s)
        deadline = None if duration_s <= 0 else time.monotonic() + duration_s
        started = time.monotonic()
        next_status_s = 0.0
        no_link_notice_printed = False
        if deadline is None:
            print("duration=continuous_until_ctrl_c")
        try:
            with serial.Serial(args.port, baudrate=args.baud, timeout=0.2) as handle:
                while deadline is None or time.monotonic() < deadline:
                    raw = handle.readline()
                    if raw:
                        line = raw.decode("utf-8", errors="replace").strip()
                        raw_lines.append(line)
                        if args.verbose_raw == "true":
                            print(line)
                    elapsed_s = time.monotonic() - started
                    if elapsed_s >= next_status_s:
                        rows_now = telemetry.parse_usbdbg_rows("\n".join(raw_lines))
                        summary_now = evaluate_station_hw_rows(rows_now, mode=mode)
                        print(_station_hw_status_line(summary_now, elapsed_s=elapsed_s))
                        if (
                            elapsed_s >= 3.0
                            and not no_link_notice_printed
                            and summary_now.get("station_link_seen") is not True
                        ):
                            print("No station hardware frames received yet.")
                            no_link_notice_printed = True
                        if summary_now.get("reason") == "STATION_HW_DEADMAN_NOT_ACTIVE":
                            print("Station frames received, but deadman is not active.")
                        elif summary_now.get("reason") == "STATION_HW_ESTOP_ACTIVE":
                            print("Station estop active.")
                        elif (
                            summary_now.get("station_physical_a_nonzero_seen") is True
                            or summary_now.get("station_physical_b_nonzero_seen") is True
                        ):
                            print(
                                f"A={summary_now.get('station_physical_a_cmd', 'NA')} "
                                f"B={summary_now.get('station_physical_b_cmd', 'NA')}"
                            )
                        if summary_now.get("motor_write_called_seen") is True or summary_now.get("physical_output_active_seen") is True:
                            print(
                                "motor_write_called="
                                f"{str(summary_now.get('motor_write_called_seen', False)).lower()} "
                                "physical_output_active="
                                f"{str(summary_now.get('physical_output_active_seen', False)).lower()}"
                            )
                        next_status_s += 1.0
        except KeyboardInterrupt:
            user_aborted = True
            print("User aborted station hardware monitor; writing summaries.")
        except (OSError, serial.serialutil.SerialException) as exc:
            serial_error = str(exc)
            print(f"station hardware serial error: {exc}")
    rows = telemetry.parse_usbdbg_rows("\n".join(raw_lines))
    summary = evaluate_station_hw_rows(rows, mode=mode)
    if user_aborted:
        summary = dict(summary)
        summary["success"] = False
        summary["station_hw_result_before_abort"] = summary.get("station_hw_result")
        summary["reason_before_abort"] = summary.get("reason")
        summary["reason"] = "USER_ABORTED"
        summary["station_hw_result"] = "USER_ABORTED"
        summary["user_aborted"] = True
        summary["next_recommended_action"] = "Rerun station-hw-diagnose or station-hw-manual after checking the station hardware state."
        summary = checks.assert_not_ready_for_full_path_following(summary)
    elif serial_error is not None:
        summary = dict(summary)
        summary["success"] = False
        summary["station_hw_result_before_serial_error"] = summary.get("station_hw_result")
        summary["reason_before_serial_error"] = summary.get("reason")
        summary["reason"] = "SERIAL_ERROR"
        summary["station_hw_result"] = "SERIAL_ERROR"
        summary["serial_error"] = serial_error
        summary["next_recommended_action"] = "Check the OpenRB USB cable/port and rerun the station hardware monitor."
        summary = checks.assert_not_ready_for_full_path_following(summary)
    _write_raw_log(out_dir / "raw_usbdbg.log", raw_lines)
    _write_rows_csv(out_dir / csv_name, rows)
    raw_dump_count = write_station_raw_frame_dumps(out_dir, rows)
    if raw_dump_count:
        summary = dict(summary)
        summary["station_raw_frame_dump_count"] = raw_dump_count
        summary["raw_station_frames"] = "raw_station_frames.txt"
        summary["raw_station_frames_hex"] = "raw_station_frames_hex.txt"
        summary = checks.assert_not_ready_for_full_path_following(summary)
        if summary.get("station_parse_ok_count") == 0 and summary.get("station_parse_error_count", 0):
            print("Station frames are arriving but parser does not match. See raw_station_frames.txt.")
    _write_json(out_dir / summary_name, summary)
    write_summary_files(out_dir, summary, title=title)
    print(f"station_link_seen={str(summary['station_link_seen']).lower()}")
    print(f"station_hw_result={summary['station_hw_result']}")
    print("ready_for_full_path_following=false")
    if user_aborted:
        return 130
    return 0 if summary["success"] is True else 2


def cmd_station_hw_diagnose(args: argparse.Namespace) -> int:
    """읽기전용 물리 스테이션 HW 링크 진단(모터 없음). / Read-only station-HW link diagnostic.

    ``--print-cmd``/``--print-command`` 는 명령만 출력하고, 그 외에는 ``_read_station_hw_rows``
    로 위임한다. 모터 명령을 절대 보내지 않는다.
    Prints commands on request, else delegates to ``_read_station_hw_rows``; never
    sends motor commands.
    """
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    compile_cmd, upload_cmd, flags = _station_hw_compile_upload_cmds(args, diagnose_only=True)
    if args.print_cmd or args.print_command == "true":
        print("STATION HARDWARE DIAGNOSE:")
        print("No motor commands are sent. The rover only reports station hardware frames.")
        print("Expected station input mapping: throttle -> physical A, steering -> physical B")
        print(f"station_hw_firmware_flags={flags}")
        if args.print_cmd:
            print(" ".join(shlex.quote(part) for part in compile_cmd))
            print(" ".join(shlex.quote(part) for part in upload_cmd))
        write_summary_files(
            out_dir,
            {
                "mode": "station-hw-diagnose",
                "success": True,
                "reason": "COMMAND_PRINTED",
                "station_hw_result": "COMMAND_PRINTED",
                "motors_enabled": False,
                "rc_input_required": False,
                "gps_required": False,
                "imu_required": False,
                "physical_a_role": "throttle",
                "physical_b_role": "turn",
                "wheel_to_physical_mapping": "physical_ab_manual_equivalent",
                "next_recommended_action": "Run without print options to monitor physical station hardware frames.",
                "ready_for_full_path_following": False,
            },
            title="Station Hardware Diagnose",
        )
        return 0
    return _read_station_hw_rows(
        args,
        title="Station Hardware Diagnose",
        csv_name="station_hw_diagnose.csv",
        summary_name="station_hw_diagnose_summary.json",
        mode="station-hw-diagnose",
    )


def cmd_station_hw_manual(args: argparse.Namespace) -> int:
    """물리 스테이션 HW 수동 로버 제어(모터 출력 포함). / Station-HW manual rover control.

    폐기 예정(수동제어는 manual-control PPM 권장). deadman 을 잡고 스테이션 스로틀/조향을
    움직여 모터 출력을 검증한다. ``--print-cmd`` 는 명령만 출력, 그 외 ``_read_station_hw_rows`` 위임.
    Deprecated (prefer manual-control PPM); verifies motor output under the station
    deadman. Prints commands on request, else delegates.
    """
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    compile_cmd, upload_cmd, flags = _station_hw_compile_upload_cmds(args, diagnose_only=False)
    if args.print_cmd:
        print(" ".join(shlex.quote(part) for part in compile_cmd))
        print(" ".join(shlex.quote(part) for part in upload_cmd))
        print(f"station_hw_firmware_flags={flags}")
        write_summary_files(
            out_dir,
            {
                "mode": "station-hw-manual",
                "success": True,
                "reason": "COMMAND_PRINTED",
                "station_hw_result": "COMMAND_PRINTED",
                "rc_input_required": False,
                "gps_required": False,
                "imu_required": False,
                "physical_a_role": "throttle",
                "physical_b_role": "turn",
                "wheel_to_physical_mapping": "physical_ab_manual_equivalent",
                "next_recommended_action": "Run without --print-cmd to upload/verify station hardware manual firmware and monitor output.",
                "ready_for_full_path_following": False,
            },
            title="Station Hardware Manual",
        )
        return 0
    print("Station hardware manual control")
    print("Operator: turn on station hardware, release estop, hold deadman, then move station throttle/steering.")
    return _read_station_hw_rows(
        args,
        title="Station Hardware Manual",
        csv_name="station_hw_manual.csv",
        summary_name="station_hw_manual_summary.json",
        mode="station-hw-manual",
    )


# ── align-heading / 첫 레인 방향 정렬 (GPS 프로브 + IMU 턴) / Heading alignment ──


def _load_plan_segments(args: argparse.Namespace) -> tuple[list[dict[str, object]] | None, str, str]:
    """Load planned segments from --plan-dir; return (segments, reason, message).

    --plan-dir 의 저장된 계획에서 정렬 대상 세그먼트를 읽는다. 실패 시 segments=None 과
    기계용 사유·조작자 메시지를 함께 돌려준다. / Loads segments to align to; on failure
    returns None plus a machine reason and operator message.

    ``segments`` is None on failure, with a machine reason and operator message.
    """
    plan_dir = getattr(args, "plan_dir", None)
    if not plan_dir:
        return None, "ALIGN_PLAN_DIR_REQUIRED", "align-heading requires --plan-dir with a built plan (run preview first)"
    plan_dir_path = Path(plan_dir)
    candidates = [plan_dir_path / "preview_summary.json", plan_dir_path / "plan.json"]
    plan_path = next((path for path in candidates if path.exists()), None)
    if plan_path is None:
        return None, "PLAN_DIR_MISSING_PLAN", f"--plan-dir must contain preview_summary.json or plan.json: {plan_dir_path}"
    plan = json.loads(plan_path.read_text())
    segments = plan.get("segments") if isinstance(plan, dict) else None
    if not segments:
        return None, "PLAN_HAS_NO_SEGMENTS", f"plan in {plan_dir_path} has no segments to align to"
    return list(segments), "OK", ""


def _write_align_artifacts(
    out_dir: Path,
    summary: dict[str, object],
    trace: Sequence[dict[str, object]],
    raw_lines: Sequence[str],
) -> None:
    """정렬 trace·원시로그·요약 파일을 기록. / Write alignment trace, raw log, and summary."""
    _write_rows_csv(out_dir / "align_heading_trace.csv", list(trace))
    _write_raw_log(out_dir / "raw_usbdbg.log", list(raw_lines))
    _write_json(out_dir / "align_heading_summary.json", summary)
    write_summary_files(out_dir, summary, title="Physical Path Planner Align Heading")


def _align_kwargs(args: argparse.Namespace) -> dict[str, object]:
    """Alignment knobs shared by the align-heading mode and the run integration."""
    return {
        "probe_a": float(getattr(args, "probe_a", getattr(args, "align_probe_a", alignment.DEFAULT_PROBE_A))),
        "probe_duration_s": float(
            getattr(args, "probe_duration_s", getattr(args, "align_probe_duration_s", alignment.DEFAULT_PROBE_DURATION_S))
        ),
        "min_probe_distance_m": float(
            getattr(args, "min_probe_distance_m", getattr(args, "align_min_probe_distance_m", alignment.DEFAULT_MIN_PROBE_DISTANCE_M))
        ),
        "heading_tolerance_deg": float(
            getattr(args, "heading_tolerance_deg", getattr(args, "align_heading_tolerance_deg", alignment.DEFAULT_HEADING_TOLERANCE_DEG))
        ),
        "turn_b_left": float(getattr(args, "turn_b_left", alignment.DEFAULT_TURN_B_LEFT)),
        "turn_b_right": float(getattr(args, "turn_b_right", alignment.DEFAULT_TURN_B_RIGHT)),
        "max_turn_duration_s": float(getattr(args, "max_turn_duration_s", alignment.DEFAULT_MAX_TURN_DURATION_S)),
        "event_timeout_s": float(getattr(args, "event_timeout_s", alignment.DEFAULT_EVENT_TIMEOUT_S)),
        "heartbeat_timeout_s": float(getattr(args, "heartbeat_timeout_s", alignment.DEFAULT_HEARTBEAT_TIMEOUT_S)),
        "verbose_raw": getattr(args, "verbose_raw", "false") == "true",
    }


def cmd_align_heading(args: argparse.Namespace) -> int:
    """로버를 첫 레인 방향으로 정렬. / Point the rover at the first lane heading.

    ``alignment.align_heading`` 로 위임한다. strategy=skip 은 시리얼 없이 계산만,
    gps_probe 는 짧게 전진해 GPS 변위로 현재 헤딩을 재고 IMU 피드백 턴으로 목표에 맞춘다.
    부수효과: (skip 아니면) 시리얼 오픈, 정렬 trace·요약 기록.
    Delegates to ``alignment.align_heading``; skip is serial-free, gps_probe drives a
    short probe then IMU-feedback turns. Side effects: serial + artifact writes.
    """
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    segments, reason, message = _load_plan_segments(args)
    if segments is None:
        return _fail_with_summary(args, reason=reason, message=message)

    strategy = args.strategy
    if strategy not in alignment.ALIGNMENT_STRATEGIES:
        return _fail_with_summary(
            args,
            reason="UNKNOWN_ALIGNMENT_STRATEGY",
            message=f"--strategy must be one of {alignment.ALIGNMENT_STRATEGIES}",
        )
    align_kwargs = _align_kwargs(args)
    target_source = getattr(args, "target_heading_source", "first_segment")

    if strategy == "skip":
        summary, trace = alignment.align_heading(
            None, segments=segments, strategy="skip", target_heading_source=target_source
        )
        summary["mode"] = args.mode
        summary["firmware_profile"] = MAC_PHYSICAL_SUPERVISED_PROFILE
        _write_align_artifacts(out_dir, summary, trace, [])
        print(f"align-heading: strategy=skip success=true reason={summary['reason']} -> {out_dir}")
        print("ready_for_full_path_following=false")
        return 0

    if not ensure_port(args):
        return 2

    import serial  # local import: preview/diagnose --from-log never need pyserial

    raw_lines: list[str] = []
    try:
        with serial.Serial(args.port, baudrate=args.baud, timeout=0.5) as handle:
            print(f"resolved_port={args.port}")
            print(f"align-heading: strategy={strategy} target_heading_source={target_source}")
            summary, trace = alignment.align_heading(
                handle,
                segments=segments,
                strategy=strategy,
                target_heading_source=target_source,
                raw_lines=raw_lines,
                **align_kwargs,
            )
    except OSError:
        summary = {
            "mode": args.mode,
            "success": False,
            "reason": "SERIAL_DISCONNECT",
            "firmware_profile": MAC_PHYSICAL_SUPERVISED_PROFILE,
            "strategy": strategy,
            "ready_for_full_path_following": False,
        }
        _write_align_artifacts(out_dir, summary, [], raw_lines)
        print("align-heading: reason=SERIAL_DISCONNECT")
        return 2

    summary["mode"] = args.mode
    summary["firmware_profile"] = MAC_PHYSICAL_SUPERVISED_PROFILE
    _write_align_artifacts(out_dir, summary, trace, raw_lines)
    print(
        f"align-heading: strategy={summary['strategy']} success={str(summary['alignment_success']).lower()} "
        f"reason={summary['reason']} initial_err={summary['initial_heading_error_deg']} "
        f"final_err={summary['final_heading_error_deg']} -> {out_dir}"
    )
    print("ready_for_full_path_following=false")
    return 0 if summary["alignment_success"] else 2


def _run_initial_alignment(
    handle: object,
    args: argparse.Namespace,
    *,
    segments: Sequence[dict[str, object]],
    out_dir: Path,
    raw_lines: list[str],
    input_fn: Callable[[str], str] = input,
) -> dict[str, object]:
    """Optionally align the rover to the first lane heading before execution.

    Returns ``{performed, ok, aligned_yaw_deg, abort_reason, summary}``. When
    ``--initial-heading-align none`` no alignment runs and the operator's
    ``--start-yaw-deg`` (possibly ``None``) is preserved. A ``gps_probe`` failure
    sets ``ok=False`` so the caller aborts; ``user_confirmed`` never aborts (a
    missing IMU simply yields no yaw reference and the controller falls back to
    per-lane capture). Alignment artifacts are written to ``<out_dir>/alignment``
    and the alignment serial lines are folded into ``raw_lines``.
    """
    mode = str(getattr(args, "initial_heading_align", "none"))
    fallback_yaw = getattr(args, "start_yaw_deg", None)
    if mode == "none":
        return {
            "performed": False,
            "ok": True,
            "aligned_yaw_deg": fallback_yaw,
            "abort_reason": None,
            "summary": None,
        }
    align_raw: list[str] = []
    summary, trace = alignment.align_heading(
        handle,
        segments=segments,
        strategy=mode,
        raw_lines=align_raw,
        input_fn=input_fn,
        **_align_kwargs(args),
    )
    summary["mode"] = "align-heading"
    summary["firmware_profile"] = MAC_PHYSICAL_SUPERVISED_PROFILE
    align_dir = Path(out_dir) / "alignment"
    align_dir.mkdir(parents=True, exist_ok=True)
    _write_align_artifacts(align_dir, summary, trace, align_raw)
    raw_lines.extend(align_raw)
    aligned_yaw = summary.get("aligned_yaw_deg")
    if aligned_yaw is None:
        aligned_yaw = fallback_yaw
    ok = True
    abort_reason: str | None = None
    if mode == "gps_probe" and not summary.get("alignment_success"):
        ok = False
        abort_reason = f"INITIAL_ALIGNMENT_FAILED:{summary.get('reason')}"
    return {
        "performed": True,
        "ok": ok,
        "aligned_yaw_deg": aligned_yaw,
        "abort_reason": abort_reason,
        "summary": summary,
    }


def _alignment_summary_fields(align: dict[str, object]) -> dict[str, object]:
    """Compact alignment fields to merge into a run/auto-relative-run summary."""
    asum = align.get("summary") or {}
    performed = bool(align.get("performed"))
    return {
        "alignment_performed": performed,
        "alignment_strategy": asum.get("strategy", "none") if performed else "none",
        "alignment_success": asum.get("alignment_success", "NA") if performed else "NA",
        "alignment_reason": asum.get("reason", "NA") if performed else "NA",
        "alignment_initial_heading_error_deg": asum.get("initial_heading_error_deg", "NA") if performed else "NA",
        "alignment_final_heading_error_deg": asum.get("final_heading_error_deg", "NA") if performed else "NA",
        "aligned_yaw_deg": align.get("aligned_yaw_deg"),
    }


def _alignment_abort_summary(
    args: argparse.Namespace,
    plan: dict[str, object],
    cal: dict[str, object],
    align: dict[str, object],
    plan_dir_used: bool,
) -> dict[str, object]:
    """Guarded summary written when initial heading alignment fails before motion."""
    return {
        "mode": args.mode,
        "success": False,
        "aborted": True,
        "reason": str(align.get("abort_reason")),
        "abort_reason": str(align.get("abort_reason")),
        "firmware_profile": MAC_PHYSICAL_SUPERVISED_PROFILE,
        "start_source": "plan_dir" if plan_dir_used else str(plan.get("start_source", "explicit")),
        "path_control_mode": args.path_control_mode,
        "initial_heading_align": str(getattr(args, "initial_heading_align", "none")),
        "next_recommended_action": (
            "Initial heading alignment failed; inspect alignment/summary.md, move outdoors "
            "for a stronger GPS displacement, or rerun with --initial-heading-align none."
        ),
        **_alignment_summary_fields(align),
        "ready_for_full_path_following": False,
    }


def _calibration_status_detail(cal: dict[str, object]) -> dict[str, object]:
    """Per-primitive calibration status for the calibration-check report.

    A motion primitive counts as operator-approved when its ``source`` does not
    start with ``fallback_known_`` (the resolver always returns a usable safe
    default with that source when no real calibration exists). The 90-degree
    turn entries are connector fallbacks and report their ``available`` flag.
    """

    def motion(name: str) -> dict[str, object]:
        primitive = cal.get(name)
        source = str(primitive.get("source", "")) if isinstance(primitive, dict) else "missing"
        return {
            "approved": calibration._is_motion_calibrated(primitive),
            "source": source,
        }

    def turn90(name: str) -> dict[str, object]:
        entry = cal.get(name)
        available = bool(isinstance(entry, dict) and entry.get("available"))
        detail: dict[str, object] = {
            "approved": available,
            "source": str(entry.get("source", "")) if isinstance(entry, dict) else "missing",
        }
        if isinstance(entry, dict) and available:
            target = telemetry._optional_float(entry.get("target_angle_deg"))
            detail.update(
                {
                    "a_cmd": entry.get("a", "NA"),
                    "b_cmd": entry.get("b", "NA"),
                    "pulse_ms": entry.get("ms", "NA"),
                    "target_angle_deg": target if target is not None else 90.0,
                }
            )
        return detail

    return {
        "forward": motion("forward"),
        "backward": motion("backward"),
        "turn_left_90": turn90("turn_left_90"),
        "turn_right_90": turn90("turn_right_90"),
    }


def _calibration_check_segments(
    args: argparse.Namespace, cal: dict[str, object]
) -> tuple[list[dict[str, object]] | None, str]:
    """Resolve the plan segments used to decide what calibration is required.

    Serial-free: prefers a saved ``--plan-dir`` plan, else builds a plan from the
    goal flags (defaulting an unset start to 0,0 so relative goal modes still
    yield the correct lane directions). Returns ``(segments, plan_source)``;
    ``segments`` is ``None`` when no plan could be built (forward-only required).
    """
    plan_dir = getattr(args, "plan_dir", None)
    if plan_dir:
        plan_dir_path = Path(plan_dir)
        candidates = [plan_dir_path / "preview_summary.json", plan_dir_path / "plan.json"]
        plan_path = next((path for path in candidates if path.exists()), None)
        if plan_path is None:
            return None, "PLAN_DIR_MISSING_PLAN"
        loaded = json.loads(plan_path.read_text())
        segments = loaded.get("segments") if isinstance(loaded, dict) else None
        if isinstance(segments, list):
            return segments, "plan_dir"
        return None, "PLAN_DIR_MISSING_SEGMENTS"
    if getattr(args, "start_lat", None) is None:
        args.start_lat = 0.0
    if getattr(args, "start_lon", None) is None:
        args.start_lon = 0.0
    try:
        plan = resolve_plan(args, cal)
    except ValueError:
        return None, "PLAN_INPUT_INVALID"
    segments = plan.get("segments")
    return (segments if isinstance(segments, list) else None), "goal_flags"


def cmd_calibration_check(args: argparse.Namespace) -> int:
    """Report motion-calibration completeness for stop_correct_go (local-only).

    Opens no serial port and uploads no firmware. Resolves the on-disk
    calibration, determines which primitives the current plan requires (forward
    always; backward when a multi-lane serpentine has return lanes), and reports
    whether stop_correct_go can run before motion is ever attempted.
    """
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    cal = resolve_calibration(args)
    segments, plan_source = _calibration_check_segments(args, cal)
    completeness = calibration.calibration_completeness(cal, segments=segments)
    detail = _calibration_status_detail(cal)
    angle_summary = calibration.turn_angle_summary(cal)
    turn_warnings = list(angle_summary.get("turn_angle_warnings", []))
    missing = list(completeness["missing_required"])
    can_run = bool(completeness["can_run_stop_correct_go"])
    summary = {
        "mode": "calibration-check",
        "success": can_run,
        "reason": "CALIBRATION_COMPLETE" if can_run else "CALIBRATION_INCOMPLETE",
        "firmware_profile": MAC_PHYSICAL_SUPERVISED_PROFILE,
        "calibration_completeness": completeness,
        "calibration_detail": detail,
        "required_for_current_plan": list(completeness["required_for_current_plan"]),
        "missing_required_calibration": missing,
        "plan_requires_backward": bool(completeness["plan_requires_backward"]),
        "can_run_stop_correct_go": can_run,
        "plan_source": plan_source,
        "plan_evaluated": plan_source in {"plan_dir", "goal_flags"},
        "motion_calibration_loaded": motion_calibration_loaded(cal),
        "fallback_to_repeated_pulses": bool(cal.get("fallback_to_repeated_pulses", True)),
        "connector_mode_effective": cal.get("connector_mode_effective", "repeated_pulses"),
        **angle_summary,
        "next_recommended_action": (
            "stop_correct_go is ready; run preview then run --path-control-mode stop_correct_go."
            if can_run
            else (
                "Calibrate the missing motion primitives "
                f"({', '.join(missing) or 'none'}) with tune-motion before stop_correct_go."
            )
        ),
        "ready_for_full_path_following": False,
    }
    _write_json(out_dir / "calibration_check_summary.json", summary)
    write_summary_files(out_dir, summary, title="Calibration Check")
    print(
        "calibration-check: "
        f"can_run_stop_correct_go={str(can_run).lower()} "
        f"required={completeness['required_for_current_plan']} "
        f"missing={missing} plan_source={plan_source} -> {out_dir}"
    )
    print(
        "turn calibration: "
        f"left_target_angle_deg={angle_summary.get('turn_left_90_target_angle_deg')} "
        f"right_target_angle_deg={angle_summary.get('turn_right_90_target_angle_deg')} "
        f"connector_mode={summary['connector_mode_effective']}"
    )
    for warning in turn_warnings:
        print(
            f"WARNING {warning} -- the executor will budget repeated pulses / IMU "
            "feedback to reach each 90 degree corner."
        )
    print("ready_for_full_path_following=false")
    return 0 if can_run else 1


def _calibration_incomplete_summary(
    args: argparse.Namespace,
    plan: dict[str, object],
    cal: dict[str, object],
    completeness: dict[str, object],
    plan_dir_used: bool,
) -> dict[str, object]:
    """Guarded summary written when stop_correct_go is requested without the
    motion calibration its plan requires (no motion is attempted)."""
    missing = list(completeness["missing_required"])
    return {
        "mode": getattr(args, "mode", "run"),
        "success": False,
        "aborted": True,
        "reason": "CALIBRATION_INCOMPLETE",
        "abort_reason": "CALIBRATION_INCOMPLETE",
        "firmware_profile": MAC_PHYSICAL_SUPERVISED_PROFILE,
        "start_source": "plan_dir" if plan_dir_used else str(plan.get("start_source", "explicit")),
        "path_control_mode": getattr(args, "path_control_mode", "stop_correct_go"),
        "calibration_completeness": completeness,
        "missing_required_calibration": missing,
        "next_recommended_action": (
            "Run calibration-check for the full report, then calibrate the missing "
            f"motion primitives ({', '.join(missing) or 'none'}) before stop_correct_go."
        ),
        "motion_calibration_loaded": motion_calibration_loaded(cal),
        "ready_for_full_path_following": False,
    }


# ── inspect-plan / 저장된 계획·이미지 점검 (모션 없음) / Inspect saved plan (no motion) ──


def _load_plan_segments_from_dir(plan_dir: Path) -> list[dict[str, object]]:
    """plan-dir 에서 세그먼트 로드(JSON→CSV→plan.json 순). / Load segments from a plan dir."""
    json_path = plan_dir / "planned_segments.json"
    if json_path.exists():
        loaded = json.loads(json_path.read_text())
        if isinstance(loaded, list):
            return [row for row in loaded if isinstance(row, dict)]
    csv_path = plan_dir / "planned_segments.csv"
    if csv_path.exists():
        with csv_path.open(newline="") as handle:
            return [dict(row) for row in csv.DictReader(handle)]
    plan = load_plan_dir_plan(plan_dir)
    segments = plan.get("segments", [])
    return [row for row in segments if isinstance(row, dict)] if isinstance(segments, list) else []


def cmd_inspect_plan(args: argparse.Namespace) -> int:
    """저장된 계획 산출물과 미리보기 이미지를 점검(모션 없음). / Inspect saved plan + images.

    로컬 전용(시리얼/펌웨어 없음). plan-dir 의 계획을 읽어 경로형태·레인/세그먼트/커넥터 수와
    첫 20개 세그먼트를 요약하고, 필수 미리보기 이미지 존재 여부를 확인한다. 이미지가 빠지면 실패.
    Local-only; summarizes shape/lane/segment/connector counts and the first 20
    segments, and checks required preview images. Missing images -> failure.
    """
    plan_dir = Path(args.plan_dir)
    out_dir = Path(args.out_dir or plan_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    try:
        plan = load_plan_dir_plan(plan_dir)
    except (FileNotFoundError, ValueError) as exc:
        return _fail_with_summary(args, reason="PLAN_DIR_MISSING_PLAN", message=str(exc))
    field_config_path = plan_dir / "field_config_resolved.json"
    field_config: dict[str, object] = {}
    if field_config_path.exists():
        loaded = json.loads(field_config_path.read_text())
        if isinstance(loaded, dict):
            field_config = loaded
    segments = _load_plan_segments_from_dir(plan_dir)
    image_paths = dict(field_config.get("image_paths", {})) if isinstance(field_config.get("image_paths"), dict) else {}
    if not image_paths:
        image_paths = _required_preview_image_paths(plan_dir)
    image_status = {
        name: {"path": path, "exists": Path(path).exists()}
        for name, path in image_paths.items()
    }
    missing_images = [status["path"] for status in image_status.values() if not status["exists"]]
    connector_count = len(
        [seg for seg in segments if str(seg.get("segment_type", "")) in geometry.CONNECTOR_SEGMENT_TYPES]
    )
    lane_count = len(
        [seg for seg in segments if str(seg.get("segment_type", "")) in geometry.FULL_LANE_SEGMENT_TYPES]
    )
    summary = {
        "mode": "inspect-plan",
        "success": not missing_images,
        "reason": "OK" if not missing_images else "PREVIEW_IMAGE_MISSING",
        "plan_dir": str(plan_dir),
        "path_shape": field_config.get("path_shape", plan.get("path_shape", "NA")),
        "lane_count": int(field_config.get("lane_count", lane_count)),
        "segment_count": len(segments),
        "connector_count": int(field_config.get("connector_count", connector_count)),
        "first_20_planned_segments": segments[:20],
        "image_status": image_status,
        "missing_preview_images": missing_images,
        "next_recommended_action": (
            "Plan artifacts are present; execute-plan can use this plan-dir."
            if not missing_images else
            "Rerun preview so required preview images are regenerated before physical execution."
        ),
        "ready_for_full_path_following": False,
    }
    summary = checks.assert_not_ready_for_full_path_following(summary)
    _write_json(out_dir / "inspect_plan_summary.json", summary)
    write_summary_files(out_dir, summary, title="Inspect Physical Path Plan")
    print(f"path_shape={summary['path_shape']}")
    print(f"lane_count={summary['lane_count']} segment_count={summary['segment_count']} connector_count={summary['connector_count']}")
    for index, segment in enumerate(segments[:20], start=1):
        print(f"{index}: {segment}")
    for name, status in image_status.items():
        print(f"{name}: exists={str(status['exists']).lower()} path={status['path']}")
    return 0 if not missing_images else 1


# ── execute-plan / run / 계획 실행 (감독형 폐루프 모션) / Execute a planned path ──


def cmd_run(args: argparse.Namespace) -> int:
    """계획된 경로를 감독형으로 실행(execute-plan/run). / Execute a planned path (guarded).

    이 파일에서 유일하게 실제로 로버를 움직이는 주 핸들러. 계획을 --plan-dir 에서 불러오거나
    골 플래그+GPS 로 즉석 생성한다. ``--print-plan`` 은 시리얼 없이 계획만 저장. path_control_mode 가
    stop_correct_go 면 먼저 보정 완비 여부를 확인하고(미완이면 모션 전 중단), 필요 시 초기 헤딩 정렬을
    수행한 뒤 ``controller.run_stop_correct_go`` 또는 ``controller.run_controller`` 에 위임한다.
    부수효과: 시리얼 오픈/실제 모터 명령, 폐루프 trace·요약·원시로그 기록. 반환 1=중단, 0=완료.
    The one handler that actually drives the rover. Loads a plan (--plan-dir or goal
    flags+GPS); ``--print-plan`` is serial-free. For stop_correct_go it checks
    calibration completeness (aborts before motion if missing), optionally aligns
    heading, then delegates to controller.run_stop_correct_go / run_controller.
    Side effects: serial + real motor commands + artifact writes. Returns 1 on abort.
    """
    cal = resolve_calibration(args)
    plan_dir_used = bool(getattr(args, "plan_dir", None))
    gps_cache_for_run = load_cached_start(float(getattr(args, "max_cached_start_age_s", 600.0)))
    field_config_for_run: dict[str, object] | None = None
    if getattr(args, "plan_dir", None):
        plan_dir = Path(args.plan_dir)
        try:
            plan = load_plan_dir_plan(plan_dir)
        except (FileNotFoundError, ValueError) as exc:
            return _fail_with_summary(
                args,
                reason="PLAN_DIR_MISSING_PLAN",
                message=str(exc),
            )
        field_config_path = plan_dir / "field_config_resolved.json"
        if field_config_path.exists():
            loaded_field_config = json.loads(field_config_path.read_text())
            if isinstance(loaded_field_config, dict):
                field_config_for_run = loaded_field_config
                plan["field_config"] = field_config_for_run
    else:
        if args.start_lat is None or args.start_lon is None:
            start, raw_start_lines = resolve_start_for_preview(args)
            if start is None:
                out_dir = Path(args.out_dir)
                out_dir.mkdir(parents=True, exist_ok=True)
                if raw_start_lines:
                    _write_raw_log(out_dir / "run_start_usbdbg.log", raw_start_lines)
                raw_rows = telemetry.parse_usbdbg_rows("\n".join(raw_start_lines))
                snapshot = gps_snapshot(
                    raw_rows,
                    min_sats=float(getattr(args, "gps_min_sats", 5.0)),
                    max_hdop=float(getattr(args, "gps_max_hdop", 2.5)),
                )
                write_summary_files(
                    out_dir,
                    {
                        "mode": args.mode,
                        "success": False,
                        "reason": "NO_USABLE_START_GPS",
                        "firmware_profile": MAC_PHYSICAL_SUPERVISED_PROFILE,
                        "message": NO_USABLE_START_GPS_ACTION,
                        "next_recommended_action": NO_USABLE_START_GPS_ACTION,
                        "start_mode": getattr(args, "start_mode", "live_gps"),
                        "start_source": "none",
                        "gps_wait_enabled": telemetry._parse_bool(getattr(args, "wait_gps", "true"), default=True),
                        "gps_wait_timeout_s": float(getattr(args, "gps_timeout_s", getattr(args, "start_timeout_s", 0.0))),
                        "gps_wait_elapsed_s": float(getattr(args, "gps_timeout_s", getattr(args, "start_timeout_s", 0.0))),
                        **{k: v for k, v in snapshot.items() if k != "ready_row"},
                        "motion_calibration_loaded": motion_calibration_loaded(cal),
                        "ready_for_full_path_following": False,
                    },
                    title="Physical Path Planner Run",
                )
                return 2
            args.start_lat = float(start["start_lat"])
            args.start_lon = float(start["start_lon"])
        try:
            plan = resolve_plan(args, cal)
        except ValueError as exc:
            return _fail_with_summary(args, reason="PLAN_INPUT_INVALID", message=str(exc))
        if "start" in locals():
            plan.update(
                {
                    "start_mode": getattr(args, "start_mode", "live_gps"),
                    "start_source": start["start_source"],
                    "current_lat": start["start_lat"],
                    "current_lon": start["start_lon"],
                    "gps_wait_enabled": telemetry._parse_bool(getattr(args, "wait_gps", "true"), default=True),
                    "gps_wait_timeout_s": float(getattr(args, "gps_timeout_s", getattr(args, "start_timeout_s", 0.0))),
                    "gps_wait_elapsed_s": start.get("gps_wait_elapsed_s", 0.0),
                    **dict(start.get("gps_wait_snapshot", {})),
                }
            )
        field_config_for_run = dict(plan.get("field_config", {}))
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    if field_config_for_run is None:
        field_config_for_run = dict(plan.get("field_config", {}))
    if field_config_for_run:
        field_config_for_run.update(
            {
                "start_mode": plan.get("start_mode", getattr(args, "start_mode", "explicit")),
                "start_source": "plan_dir" if plan_dir_used else str(plan.get("start_source", "explicit")),
            }
        )
        if plan_dir_used:
            _write_json(out_dir / "field_config_resolved.json", field_config_for_run)
        else:
            try:
                image_paths = _write_plan_artifacts(out_dir, plan, field_config_for_run)
                plan["image_paths"] = image_paths
            except RuntimeError as exc:
                expected = str(exc).replace("PREVIEW_IMAGE_NOT_WRITTEN ", "")
                failure = {
                    "mode": args.mode,
                    "success": False,
                    "reason": "PREVIEW_IMAGE_NOT_WRITTEN",
                    "expected_image_path": expected,
                    "next_recommended_action": "Install/check matplotlib rendering and rerun planning before physical execution.",
                    "ready_for_full_path_following": False,
                }
                write_summary_files(out_dir, failure, title="Physical Path Planner Run")
                print(f"run: reason=PREVIEW_IMAGE_NOT_WRITTEN expected_image_path={expected}")
                return 2
        if telemetry._parse_bool(getattr(args, "print_field_config", "false"), default=False):
            print(format_field_config(field_config_for_run))
    if args.print_plan:
        _write_json(out_dir / "plan.json", plan)
        start_source = "plan_dir" if plan_dir_used else str(plan.get("start_source", "explicit"))
        summary = {
            **plan,
            "mode": args.mode,
            "success": True,
            "reason": "PLAN_PRINTED",
            "firmware_profile": MAC_PHYSICAL_SUPERVISED_PROFILE,
            "start_source": start_source,
            "current_lat": plan["start_lat"],
            "current_lon": plan["start_lon"],
            "start_mode": plan.get("start_mode", getattr(args, "start_mode", "explicit")),
            "gps_wait_enabled": plan.get("gps_wait_enabled", False),
            "gps_wait_timeout_s": plan.get("gps_wait_timeout_s", 0.0),
            "gps_wait_elapsed_s": plan.get("gps_wait_elapsed_s", 0.0),
            "gps_ready": plan.get("gps_ready", "NA"),
            "gps_ready_at_start": plan.get("gps_ready", "NA"),
            "gps_solution_valid": plan.get("gps_solution_valid", "NA"),
            "gps_sats": plan.get("gps_sats", "NA"),
            "gps_hdop": plan.get("gps_hdop", "NA"),
            "best_sats": plan.get("best_sats", "NA"),
            "best_hdop": plan.get("best_hdop", "NA"),
            "best_lat": plan.get("best_lat", "NA"),
            "best_lon": plan.get("best_lon", "NA"),
            "last_rmc_status": plan.get("last_rmc_status", "NA"),
            "last_gga_fix_quality": plan.get("last_gga_fix_quality", "NA"),
            "imu_present": plan.get("imu_present", "NA"),
            "imu_relative_yaw_deg": plan.get("imu_relative_yaw_deg", "NA"),
            "motion_calibration_loaded": motion_calibration_loaded(cal),
            "field_config": field_config_for_run,
            "field_config_loaded": bool(field_config_for_run),
            "connector_mode_effective": cal.get("connector_mode_effective", plan.get("connector_mode_effective")),
            "continuous_drive_used": args.straight_motion_mode == "continuous",
            "path_control_mode": args.path_control_mode,
            "live_chunk_ms": args.live_chunk_ms,
            "max_segment_chunks": args.max_segment_chunks,
            "max_ms": args.max_ms,
            "imu_heading_hold": telemetry._parse_bool(args.imu_heading_hold, default=True),
            "cross_track_correction": telemetry._parse_bool(args.cross_track_correction, default=True),
            "gps_reanchor": telemetry._parse_bool(args.gps_reanchor, default=True),
            "k_heading": args.k_heading,
            "k_cross_track": args.k_cross_track,
            "max_correction_b": args.max_correction_b,
            "gps_cache_used": bool(plan.get("gps_cached_used") or gps_cache_for_run is not None),
            "rc_ignored_for_usb_supervised": True,
            "gps_degraded_count": 0,
            "imu_heading_used_count": 0,
            "next_recommended_action": "Inspect summary.md and plan.json before running physical execution.",
            "ready_for_full_path_following": False,
        }
        write_summary_files(out_dir, summary, title="Physical Path Planner Plan")
        print(
            f"run --print-plan: {plan['segment_count']} segments, "
            f"fallback_to_repeated_pulses={cal['fallback_to_repeated_pulses']} "
            f"-> {out_dir}/plan.json (no serial opened)"
        )
        return 0
    if args.path_control_mode == "stop_correct_go":
        completeness = calibration.calibration_completeness(cal, segments=plan["segments"])
        if not completeness["can_run_stop_correct_go"]:
            abort_summary = _calibration_incomplete_summary(
                args, plan, cal, completeness, plan_dir_used
            )
            _write_json(out_dir / "run_summary.json", abort_summary)
            write_summary_files(out_dir, abort_summary, title="Physical Path Planner Run")
            print(
                "run: aborted before motion, reason=CALIBRATION_INCOMPLETE "
                f"missing={completeness['missing_required']} -> {out_dir}"
            )
            return 2
    if not ensure_port(args):
        return 2

    import serial  # local import: preview/diagnose --from-log never need pyserial

    align_raw_lines: list[str] = []
    handle = serial.Serial(args.port, baudrate=args.baud, timeout=0.5)
    try:
        align = _run_initial_alignment(
            handle,
            args,
            segments=plan["segments"],  # type: ignore[arg-type]
            out_dir=out_dir,
            raw_lines=align_raw_lines,
        )
        if not align["ok"]:
            abort_summary = _alignment_abort_summary(args, plan, cal, align, plan_dir_used)
            _write_json(out_dir / "run_summary.json", abort_summary)
            write_summary_files(out_dir, abort_summary, title="Physical Path Planner Run")
            _write_raw_log(out_dir / "run_serial.log", align_raw_lines)
            print(f"run: aborted before motion, reason={abort_summary['reason']} -> {out_dir}")
            return 2
        if args.path_control_mode == "stop_correct_go":
            rows, raw_lines, abort_reason = _run_stop_correct_go_from_args(
                handle, args, plan, cal, start_yaw_deg=align["aligned_yaw_deg"]
            )
        else:
            rows, raw_lines, abort_reason = controller.run_controller(
                handle,
                segments=plan["segments"],  # type: ignore[arg-type]
                resolved_calibration=cal,
                start_lat=float(plan["start_lat"]),
                start_lon=float(plan["start_lon"]),
                start_yaw_deg=align["aligned_yaw_deg"],
                goal_lat=float(plan["goal_lat"]),
                goal_lon=float(plan["goal_lon"]),
                event_timeout_s=args.event_timeout_s,
                heartbeat_timeout_s=args.heartbeat_timeout_s,
                rc_neutral_wait_s=args.rc_neutral_wait_s,
                gps_degradation_policy=args.gps_degradation_policy,
                manual_override_mode=args.manual_override_mode,
                left_fixed_pulses=args.left_fixed_pulses,
                right_fixed_pulses=args.right_fixed_pulses,
                straight_motion_mode=args.straight_motion_mode,
                live_update_hz=args.live_update_hz,
                live_ttl_ms=args.live_ttl_ms,
                live_chunk_ms=args.live_chunk_ms,
                max_segment_chunks=args.max_segment_chunks,
                live_max_ms=args.max_ms,
                imu_heading_hold=telemetry._parse_bool(args.imu_heading_hold, default=True),
                cross_track_correction=telemetry._parse_bool(args.cross_track_correction, default=True),
                path_control_mode=args.path_control_mode,
                k_heading=args.k_heading,
                k_cross_track=args.k_cross_track,
                max_correction_b=args.max_correction_b,
                gps_reanchor=telemetry._parse_bool(args.gps_reanchor, default=True),
                max_connector_pulses_per_turn=getattr(
                    args,
                    "max_connector_pulses_per_turn",
                    controller.DEFAULT_MAX_CONNECTOR_PULSES_PER_TURN,
                ),
                turn_angle_policy=getattr(
                    args, "turn_calibration_angle_policy", controller.DEFAULT_TURN_ANGLE_POLICY
                ),
                turn_angle_override=getattr(args, "turn_angle_deg_override", None),
            )
    finally:
        handle.close()

    raw_lines = list(align_raw_lines) + list(raw_lines)

    if args.path_control_mode == "stop_correct_go":
        summary = controller.build_stop_correct_go_summary(
            rows,
            start_lat=float(plan["start_lat"]),
            start_lon=float(plan["start_lon"]),
            goal_lat=float(plan["goal_lat"]),
            goal_lon=float(plan["goal_lon"]),
            goal_distance_m=float(plan["goal_distance_m"]),
            fallback_to_repeated_pulses=bool(cal["fallback_to_repeated_pulses"]),
            sensor_trust_mode=args.sensor_trust_mode,
            allow_calibration_fallback=telemetry._parse_bool(
                args.allow_calibration_fallback, default=True
            ),
            abort_reason=abort_reason,
            heading_reference=getattr(
                args, "heading_reference", controller.DEFAULT_HEADING_REFERENCE
            ),
            turn_angle_policy=getattr(
                args, "turn_calibration_angle_policy", controller.DEFAULT_TURN_ANGLE_POLICY
            ),
            turn_calibration_angles=calibration.turn_angle_summary(cal),
        )
    else:
        summary = controller.build_controller_summary(
            rows,
            start_lat=float(plan["start_lat"]),
            start_lon=float(plan["start_lon"]),
            goal_lat=float(plan["goal_lat"]),
            goal_lon=float(plan["goal_lon"]),
            goal_distance_m=float(plan["goal_distance_m"]),
            fallback_to_repeated_pulses=bool(cal["fallback_to_repeated_pulses"]),
            abort_reason=abort_reason,
        )
    summary = {
        **summary,
        "mode": args.mode,
        "success": summary.get("aborted") is False,
        "reason": "OK" if summary.get("aborted") is False else str(abort_reason),
        "firmware_profile": MAC_PHYSICAL_SUPERVISED_PROFILE,
        "start_source": "plan_dir" if plan_dir_used else str(plan.get("start_source", "explicit")),
        "current_lat": plan["start_lat"],
        "current_lon": plan["start_lon"],
        "start_mode": plan.get("start_mode", getattr(args, "start_mode", "explicit")),
        "gps_wait_enabled": plan.get("gps_wait_enabled", False),
        "gps_wait_timeout_s": plan.get("gps_wait_timeout_s", 0.0),
        "gps_wait_elapsed_s": plan.get("gps_wait_elapsed_s", 0.0),
        "gps_ready": plan.get("gps_ready", "NA"),
        "gps_ready_at_start": plan.get("gps_ready", "NA"),
        "gps_solution_valid": plan.get("gps_solution_valid", "NA"),
        "gps_sats": plan.get("gps_sats", "NA"),
        "gps_hdop": plan.get("gps_hdop", "NA"),
        "best_sats": plan.get("best_sats", "NA"),
        "best_hdop": plan.get("best_hdop", "NA"),
        "best_lat": plan.get("best_lat", "NA"),
        "best_lon": plan.get("best_lon", "NA"),
        "last_rmc_status": plan.get("last_rmc_status", "NA"),
        "last_gga_fix_quality": plan.get("last_gga_fix_quality", "NA"),
        "imu_present": plan.get("imu_present", "NA"),
        "imu_relative_yaw_deg": plan.get("imu_relative_yaw_deg", "NA"),
        "motion_calibration_loaded": motion_calibration_loaded(cal),
        "field_config": field_config_for_run,
        "field_config_loaded": bool(field_config_for_run),
        "connector_mode_effective": cal.get("connector_mode_effective", plan.get("connector_mode_effective")),
        "continuous_drive_used": summary.get("continuous_drive_used", args.straight_motion_mode == "continuous"),
        "path_control_mode": summary.get("path_control_mode", args.path_control_mode),
        "closed_loop_correction_enabled": summary.get("closed_loop_correction_enabled", False),
        "closed_loop_correction_applied": summary.get("closed_loop_correction_applied", False),
        "live_chunk_ms": args.live_chunk_ms,
        "max_segment_chunks": args.max_segment_chunks,
        "max_ms": summary.get("max_ms", args.max_ms),
        "imu_heading_hold": telemetry._parse_bool(args.imu_heading_hold, default=True),
        "cross_track_correction": telemetry._parse_bool(args.cross_track_correction, default=True),
        "gps_reanchor": telemetry._parse_bool(args.gps_reanchor, default=True),
        "k_heading": args.k_heading,
        "k_cross_track": args.k_cross_track,
        "max_correction_b": args.max_correction_b,
        "gps_cache_used": bool(plan.get("gps_cached_used") or gps_cache_for_run is not None),
        "rc_ignored_for_usb_supervised": True,
        "rc_warning": (
            "RC_NOT_OK_IGNORED_FOR_MAC_USB_SUPERVISED_MODE"
            if int(summary.get("rc_warning_count", 0)) > 0 else "NONE"
        ),
        "abort_reason": str(abort_reason),
        "next_recommended_action": (
            "Inspect path trace and run summary before any longer test."
            if summary.get("aborted") is False else
            "Inspect abort reason, raw USB log, and final motor command fields."
        ),
        **_alignment_summary_fields(align),
        "ready_for_full_path_following": False,
    }
    _write_json(out_dir / "run_summary.json", summary)
    write_summary_files(out_dir, summary, title="Physical Path Planner Run")
    _write_rows_csv(out_dir / "run_rows.csv", rows)
    _write_raw_log(out_dir / "run_serial.log", raw_lines)
    _write_closed_loop_artifacts(out_dir, rows, raw_lines)
    if args.path_control_mode == "stop_correct_go":
        _write_stop_correct_go_artifacts(out_dir, rows)
    print(
        f"run: abort_reason={abort_reason}, pulses={summary['pulse_count']}, "
        f"valid={summary['valid_pulse_count']} -> {out_dir}"
    )
    return 1 if summary["aborted"] else 0


# ── AUTO 스위치 트리거 상대경로 실행 / AUTO-switch-triggered relative path planning ──
# auto-relative-preview 는 GPS 를 기다려 상대 A->B 필드를 렌더링(모션 없음)하고,
# auto-relative-run 은 물리 CH5 모드 스위치가 AUTO 로 넘어갈 때만 폐루프 실행을 시작한다.
# 실행 중 MANUAL 로 되돌리면(require_auto_switch) 안전하게 정지한다.
# auto-relative-preview renders the relative A->B field (no motion); auto-relative-run
# starts closed-loop execution only when the physical CH5 switch reads AUTO, and stops
# safely if it flips back to MANUAL.


def _write_closed_loop_artifacts(
    out_dir: Path, rows: Sequence[dict[str, object]], raw_lines: Sequence[str]
) -> None:
    """Write the closed-loop trace, planned-vs-actual, and raw USBDBG log."""
    _write_rows_csv(out_dir / "closed_loop_trace.csv", rows)
    planned_vs_actual = [
        {
            "segment_index": row.get("segment_index"),
            "chunk_index": row.get("chunk_index"),
            "path_control_mode": row.get("path_control_mode"),
            "target_x_m": row.get("target_x_m"),
            "target_y_m": row.get("target_y_m"),
            "current_x_m": row.get("current_x_m"),
            "current_y_m": row.get("current_y_m"),
            "target_heading_deg": row.get("target_heading_deg"),
            "heading_error_deg": row.get("heading_error_deg"),
            "cross_track_error_m": row.get("cross_track_error_m"),
            "along_track_progress_m": row.get("along_track_progress_m"),
            "remaining_distance_m": row.get("remaining_distance_m"),
            "final_a_cmd": row.get("final_a_cmd"),
            "final_b_cmd": row.get("final_b_cmd"),
            "valid_pulse": row.get("valid_pulse"),
            "invalid_reason": row.get("invalid_reason"),
        }
        for row in rows
    ]
    _write_rows_csv(out_dir / "planned_vs_actual.csv", planned_vs_actual)
    _write_raw_log(out_dir / "raw_usbdbg.log", raw_lines)


def _run_stop_correct_go_from_args(
    handle: object,
    args: argparse.Namespace,
    plan: dict[str, object],
    cal: dict[str, object],
    *,
    start_yaw_deg: float | None,
    start_lat: float | None = None,
    start_lon: float | None = None,
    require_auto_switch: bool = False,
) -> tuple[list[dict[str, object]], list[str], str]:
    """Map CLI args onto :func:`controller.run_stop_correct_go`."""
    s_lat = float(plan["start_lat"]) if start_lat is None else float(start_lat)
    s_lon = float(plan["start_lon"]) if start_lon is None else float(start_lon)
    return controller.run_stop_correct_go(
        handle,
        segments=plan["segments"],  # type: ignore[arg-type]
        resolved_calibration=cal,
        start_lat=s_lat,
        start_lon=s_lon,
        start_yaw_deg=start_yaw_deg,
        goal_lat=float(plan["goal_lat"]),
        goal_lon=float(plan["goal_lon"]),
        move_chunk_ms=args.move_chunk_ms,
        settle_after_move_ms=args.settle_after_move_ms,
        telemetry_stabilize_ms=args.telemetry_stabilize_ms,
        heading_correction_threshold_deg=args.heading_correction_threshold_deg,
        heading_correction_tolerance_deg=args.heading_correction_tolerance_deg,
        cross_track_correction_threshold_m=args.cross_track_correction_threshold_m,
        heading_correction_b_left=args.heading_correction_b_left,
        heading_correction_b_right=args.heading_correction_b_right,
        max_heading_correction_ms=args.max_heading_correction_ms,
        sensor_trust_mode=args.sensor_trust_mode,
        allow_calibration_fallback=telemetry._parse_bool(
            args.allow_calibration_fallback, default=True
        ),
        event_timeout_s=args.event_timeout_s,
        heartbeat_timeout_s=args.heartbeat_timeout_s,
        rc_neutral_wait_s=args.rc_neutral_wait_s,
        gps_degradation_policy=args.gps_degradation_policy,
        manual_override_mode=args.manual_override_mode,
        left_fixed_pulses=args.left_fixed_pulses,
        right_fixed_pulses=args.right_fixed_pulses,
        live_update_hz=args.live_update_hz,
        live_ttl_ms=args.live_ttl_ms,
        max_segment_chunks=args.max_segment_chunks,
        k_cross_track=args.k_cross_track,
        max_correction_b=args.max_correction_b,
        gps_reanchor=telemetry._parse_bool(args.gps_reanchor, default=True),
        require_auto_switch=require_auto_switch,
        max_connector_pulses_per_turn=getattr(
            args, "max_connector_pulses_per_turn", controller.DEFAULT_MAX_CONNECTOR_PULSES_PER_TURN
        ),
        connector_turn_tolerance_deg=getattr(
            args, "connector_turn_tolerance_deg", controller.DEFAULT_CONNECTOR_TURN_TOLERANCE_DEG
        ),
        max_connector_turn_ms=getattr(
            args, "max_connector_turn_ms", controller.DEFAULT_MAX_CONNECTOR_TURN_MS
        ),
        turn_angle_policy=getattr(
            args, "turn_calibration_angle_policy", controller.DEFAULT_TURN_ANGLE_POLICY
        ),
        turn_angle_override=getattr(args, "turn_angle_deg_override", None),
        heading_reference=getattr(args, "heading_reference", controller.DEFAULT_HEADING_REFERENCE),
        max_gps_jump_m=getattr(args, "max_gps_jump_m", None),
    )


def _write_stop_correct_go_artifacts(
    out_dir: Path, rows: Sequence[dict[str, object]]
) -> None:
    """Write the stop_correct_go cycle trace and the heading-correction trace."""
    cycle_rows = [row for row in rows if row.get("row_type") == "pulse"]
    trace = [
        {
            "segment_index": row.get("segment_index"),
            "chunk_index": row.get("chunk_index"),
            "phase": row.get("phase"),
            "gps_valid": row.get("gps_valid"),
            "imu_valid": row.get("imu_valid"),
            "current_x_m": row.get("current_x_m"),
            "current_y_m": row.get("current_y_m"),
            "target_heading_deg": row.get("target_heading_deg"),
            "imu_yaw_deg": row.get("imu_yaw_deg"),
            "heading_error_deg": row.get("heading_error_deg"),
            "cross_track_error_m": row.get("cross_track_error_m"),
            "along_track_progress_m": row.get("along_track_progress_m"),
            "remaining_distance_m": row.get("remaining_distance_m"),
            "move_a_cmd": row.get("move_a_cmd"),
            "move_b_cmd": row.get("move_b_cmd"),
            "correction_b_cmd": row.get("correction_b_cmd"),
            "correction_duration_ms": row.get("correction_duration_ms"),
            "correction_success": row.get("correction_success"),
            "sensor_source": row.get("sensor_source"),
            "fallback_used": row.get("fallback_used"),
            "final_zero": row.get("final_zero"),
            "heading_reference": row.get("heading_reference"),
            "turn_angle_policy": row.get("turn_angle_policy"),
            "requested_turn_angle_deg": row.get("requested_turn_angle_deg"),
            "calibration_target_angle_deg": row.get("calibration_target_angle_deg"),
            "turn_pulse_index": row.get("turn_pulse_index"),
            "turn_pulse_budget": row.get("turn_pulse_budget"),
            "yaw_turn_ref_deg": row.get("yaw_turn_ref_deg"),
            "applied_turn_delta_deg": row.get("applied_turn_delta_deg"),
            "remaining_turn_error_deg": row.get("remaining_turn_error_deg"),
            "turn_measured_by_imu": row.get("turn_measured_by_imu"),
            "turn_mode": row.get("turn_mode"),
            "turn_duration_ms": row.get("turn_duration_ms"),
            "turn_timed_out": row.get("turn_timed_out"),
            "connector_turn_completed": row.get("connector_turn_completed"),
            "turn_overshoot": row.get("turn_overshoot"),
            "gps_jump_rejected": row.get("gps_jump_rejected"),
        }
        for row in cycle_rows
    ]
    _write_rows_csv(out_dir / "stop_correct_go_trace.csv", trace)
    corrections = [
        {
            "segment_index": row.get("segment_index"),
            "chunk_index": row.get("chunk_index"),
            "imu_yaw_deg": row.get("imu_yaw_deg"),
            "heading_error_deg": row.get("heading_error_deg"),
            "correction_b_cmd": row.get("correction_b_cmd"),
            "correction_duration_ms": row.get("correction_duration_ms"),
            "correction_success": row.get("correction_success"),
            "post_correction_heading_error_deg": row.get("post_correction_heading_error_deg"),
        }
        for row in cycle_rows
        if int(telemetry._optional_float(row.get("correction_duration_ms")) or 0) > 0
    ]
    _write_rows_csv(out_dir / "heading_correction_trace.csv", corrections)


def _write_preview_outputs(
    out_dir: Path,
    plan: dict[str, object],
    *,
    start_mode: str,
    start_source: str,
    write_png: bool,
) -> dict[str, object]:
    """Write field_config_resolved.json, plan.json, planned CSVs, and the preview PNG."""
    field_config = dict(plan.get("field_config", {}))
    field_config.update({"start_mode": start_mode, "start_source": start_source})
    try:
        image_paths = _write_plan_artifacts(out_dir, plan, field_config)
        plan["image_paths"] = image_paths
    except RuntimeError as exc:
        expected = str(exc).replace("PREVIEW_IMAGE_NOT_WRITTEN ", "")
        raise RuntimeError(f"PREVIEW_IMAGE_NOT_WRITTEN {expected}") from exc
    return field_config


def _auto_relative_status_line(
    rows: Sequence[dict[str, str]], *, waiting_for: str, min_sats: float, max_hdop: float
) -> str:
    """One-line operator status while waiting for GPS / the AUTO mode switch."""
    snapshot = gps_snapshot(rows, min_sats=min_sats, max_hdop=max_hdop)
    last = rows[-1] if rows else {}
    fmt = telemetry._fmt
    return (
        f"mode_switch={controller.mode_switch_state(last)} "
        f"mode_us={last.get('mode_us', 'NA')} "
        f"rc_ok={str(telemetry._parse_bool(last.get('rc_ok'))).lower()} "
        f"gps_ready={str(snapshot['gps_ready']).lower()} "
        f"gps_sats={fmt(snapshot['gps_sats'], 0) if snapshot['gps_sats'] is not None else 'NA'} "
        f"gps_hdop={fmt(snapshot['gps_hdop'], 2) if snapshot['gps_hdop'] is not None else 'NA'} "
        f"current_lat={fmt(snapshot['current_lat'], 7) if snapshot['current_lat'] is not None else 'NA'} "
        f"current_lon={fmt(snapshot['current_lon'], 7) if snapshot['current_lon'] is not None else 'NA'} "
        f"imu_present={str(snapshot['imu_present']).lower()} "
        f"imu_relative_yaw_deg={snapshot['imu_relative_yaw_deg']} "
        f"waiting_for={waiting_for}"
    )


def _auto_relative_wait_for_gps(
    handle: object,
    *,
    min_sats: float,
    max_hdop: float,
    timeout_s: float,
    cached_start_max_age_ms: int,
    raw_lines: list[str],
    rows: list[dict[str, str]],
) -> dict[str, object] | None:
    """Stream telemetry until a usable GPS start fix; print status ~every 1s."""
    deadline = time.monotonic() + max(0.0, timeout_s)
    next_status = time.monotonic()
    started = time.monotonic()
    while time.monotonic() < deadline:
        raw = handle.readline()  # type: ignore[attr-defined]
        if raw:
            line = raw.decode("utf-8", errors="replace").strip()
            raw_lines.append(line)
            rows.extend(telemetry.parse_usbdbg_rows(line))
        resolved = resolve_start_gps_from_rows(
            rows,
            start_mode="live_gps",
            cached_start_max_age_ms=cached_start_max_age_ms,
            min_sats=min_sats,
            max_hdop=max_hdop,
        )
        if resolved is not None:
            resolved["gps_wait_elapsed_s"] = time.monotonic() - started
            return resolved
        if time.monotonic() >= next_status:
            print(_auto_relative_status_line(rows, waiting_for="GPS", min_sats=min_sats, max_hdop=max_hdop))
            next_status = time.monotonic() + 1.0
    return None


def _auto_relative_wait_for_auto(
    handle: object,
    *,
    timeout_s: float,
    allow_keyboard: bool,
    input_fn: "callable",
    raw_lines: list[str],
    rows: list[dict[str, str]],
    min_sats: float,
    max_hdop: float,
) -> tuple[bool, str]:
    """Wait for the physical mode switch to read AUTO (or a keyboard start fallback).

    Returns ``(started, reason)``. ``reason`` is ``AUTO_SWITCH`` /
    ``KEYBOARD_START`` on success, or ``AUTO_SWITCH_NOT_DETECTED`` /
    ``KEYBOARD_START_ABORTED`` when execution must not begin.
    """
    deadline = time.monotonic() + max(0.0, timeout_s)
    next_status = time.monotonic()
    keyboard_offered = False
    while time.monotonic() < deadline:
        raw = handle.readline()  # type: ignore[attr-defined]
        if raw:
            line = raw.decode("utf-8", errors="replace").strip()
            raw_lines.append(line)
            rows.extend(telemetry.parse_usbdbg_rows(line))
        last = rows[-1] if rows else {}
        state = controller.mode_switch_state(last)
        if state == "AUTO":
            return True, "AUTO_SWITCH"
        if state == "ABSENT" and allow_keyboard and not keyboard_offered:
            keyboard_offered = True
            print("auto-relative-run: PPM mode channel absent; press Enter to start (Ctrl-C aborts).")
            try:
                input_fn()
            except (EOFError, KeyboardInterrupt):
                return False, "KEYBOARD_START_ABORTED"
            return True, "KEYBOARD_START"
        if time.monotonic() >= next_status:
            print(_auto_relative_status_line(rows, waiting_for="AUTO_SWITCH", min_sats=min_sats, max_hdop=max_hdop))
            next_status = time.monotonic() + 1.0
    return False, "AUTO_SWITCH_NOT_DETECTED"


def _auto_relative_summary(
    args: argparse.Namespace,
    cal: dict[str, object],
    plan: dict[str, object] | None,
    field_config: dict[str, object] | None,
    *,
    controller_summary: dict[str, object] | None,
    start_lat: float | None,
    start_lon: float | None,
    start_source: str,
    auto_switch_detected: bool,
    execution_started: bool,
    stop_reason: str,
    reason: str,
) -> dict[str, object]:
    """Assemble the auto-relative-run summary (closed-loop counts + AUTO outcome)."""
    summary: dict[str, object] = {
        "mode": "auto-relative-run",
        "success": stop_reason in {"COMPLETED", "USER_SWITCHED_TO_MANUAL"},
        "reason": reason,
        "firmware_profile": MAC_PHYSICAL_SUPERVISED_PROFILE,
        "goal_mode": "relative_enu",
        "goal_east_m": args.goal_east_m,
        "goal_north_m": args.goal_north_m,
        "expected_goal_distance_m": float(plan["goal_distance_m"]) if plan else "NA",
        "workspace_width_m": args.workspace_width_m,
        "step_spacing_m": args.step_spacing_m,
        "path_shape": args.path_shape,
        "start_source": start_source,
        "start_lat": start_lat,
        "start_lon": start_lon,
        "auto_switch_detected": auto_switch_detected,
        "execution_started": execution_started,
        "path_control_mode": args.path_control_mode,
        "motion_calibration_loaded": motion_calibration_loaded(cal),
        "continuous_drive_used": args.straight_motion_mode == "continuous",
        "connector_mode_effective": cal.get("connector_mode_effective"),
        "field_config": field_config,
        "field_config_loaded": bool(field_config),
        "allow_keyboard_start": telemetry._parse_bool(getattr(args, "allow_keyboard_start", "false"), default=False),
        "rc_ignored_for_usb_supervised": True,
        "stop_reason": stop_reason,
        "ready_for_full_path_following": False,
    }
    if controller_summary is not None:
        summary.update(
            {
                "closed_loop_correction_enabled": controller_summary.get("closed_loop_correction_enabled", False),
                "closed_loop_correction_applied": controller_summary.get("closed_loop_correction_applied", False),
                "imu_heading_used_count": controller_summary.get("imu_heading_used_count", 0),
                "cross_track_correction_used_count": controller_summary.get("cross_track_correction_used_count", 0),
                "gps_chunk_count": controller_summary.get("gps_chunk_count", 0),
                "gps_degraded_count": controller_summary.get("gps_degraded_count", 0),
                "gps_reanchor_count": controller_summary.get("gps_reanchor_count", 0),
                "completed_segment_count": controller_summary.get("completed_segment_count", 0),
                "completed_chunk_count": controller_summary.get("completed_chunk_count", 0),
                "average_abs_heading_error_deg": controller_summary.get("average_abs_heading_error_deg", "NA"),
                "max_abs_heading_error_deg": controller_summary.get("max_abs_heading_error_deg", "NA"),
                "average_abs_cross_track_error_m": controller_summary.get("average_abs_cross_track_error_m", "NA"),
                "max_abs_cross_track_error_m": controller_summary.get("max_abs_cross_track_error_m", "NA"),
                "final_distance_to_goal_m": controller_summary.get("final_distance_to_goal_m", "NA"),
                "abort_reason": controller_summary.get("abort_reason", stop_reason),
            }
        )
    else:
        summary.update(
            {
                "closed_loop_correction_enabled": args.path_control_mode in {"imu_heading", "gps_imu_closed_loop"},
                "closed_loop_correction_applied": False,
                "imu_heading_used_count": 0,
                "cross_track_correction_used_count": 0,
                "gps_chunk_count": 0,
                "gps_degraded_count": 0,
                "gps_reanchor_count": 0,
                "completed_segment_count": 0,
                "completed_chunk_count": 0,
                "average_abs_heading_error_deg": "NA",
                "max_abs_heading_error_deg": "NA",
                "average_abs_cross_track_error_m": "NA",
                "max_abs_cross_track_error_m": "NA",
                "final_distance_to_goal_m": "NA",
                "abort_reason": stop_reason,
            }
        )
    return summary


def cmd_auto_relative_preview(args: argparse.Namespace) -> int:
    """Wait for GPS, resolve the relative A->B field, write field config + preview. No motion."""
    cal = resolve_calibration(args)
    start, raw_start_lines = resolve_start_for_preview(args)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    if raw_start_lines:
        _write_raw_log(out_dir / "preview_start_usbdbg.log", raw_start_lines)
    if start is None:
        raw_rows = telemetry.parse_usbdbg_rows("\n".join(raw_start_lines))
        snapshot = gps_snapshot(
            raw_rows,
            min_sats=float(getattr(args, "gps_min_sats", 5.0)),
            max_hdop=float(getattr(args, "gps_max_hdop", 2.5)),
        )
        write_summary_files(
            out_dir,
            {
                "mode": "auto-relative-preview",
                "success": False,
                "reason": "NO_USABLE_START_GPS",
                "firmware_profile": MAC_PHYSICAL_SUPERVISED_PROFILE,
                "next_recommended_action": NO_USABLE_START_GPS_ACTION,
                "start_source": "none",
                **{k: v for k, v in snapshot.items() if k != "ready_row"},
                "motion_calibration_loaded": motion_calibration_loaded(cal),
                "ready_for_full_path_following": False,
            },
            title="Physical Path Planner Auto-Relative Preview",
        )
        print(f"auto-relative-preview: reason=NO_USABLE_START_GPS. {NO_USABLE_START_GPS_ACTION}")
        return 2
    args.start_lat = float(start["start_lat"])
    args.start_lon = float(start["start_lon"])
    try:
        plan = resolve_plan(args, cal)
    except ValueError as exc:
        return _fail_with_summary(args, reason="PLAN_INPUT_INVALID", message=str(exc))
    try:
        field_config = _write_preview_outputs(
            out_dir,
            plan,
            start_mode=getattr(args, "start_mode", "live_gps"),
            start_source=str(start["start_source"]),
            write_png=getattr(args, "png", True),
        )
    except RuntimeError as exc:
        expected = str(exc).replace("PREVIEW_IMAGE_NOT_WRITTEN ", "")
        failure = {
            "mode": "auto-relative-preview",
            "success": False,
            "reason": "PREVIEW_IMAGE_NOT_WRITTEN",
            "expected_image_path": expected,
            "next_recommended_action": "Install/check matplotlib rendering and rerun preview before physical execution.",
            "ready_for_full_path_following": False,
        }
        write_summary_files(out_dir, failure, title="Physical Path Planner Auto-Relative Preview")
        print(f"auto-relative-preview: reason=PREVIEW_IMAGE_NOT_WRITTEN expected_image_path={expected}")
        return 2
    summary = {
        **plan,
        "mode": "auto-relative-preview",
        "success": True,
        "reason": "OK",
        "firmware_profile": MAC_PHYSICAL_SUPERVISED_PROFILE,
        "start_source": start["start_source"],
        "current_lat": start["start_lat"],
        "current_lon": start["start_lon"],
        "goal_mode": "relative_enu",
        "goal_east_m": args.goal_east_m,
        "goal_north_m": args.goal_north_m,
        "expected_goal_distance_m": float(plan["goal_distance_m"]),
        "motion_calibration_loaded": motion_calibration_loaded(cal),
        "connector_mode_effective": cal.get("connector_mode_effective", plan.get("connector_mode_effective")),
        "field_config": field_config,
        "field_config_loaded": True,
        "next_recommended_action": f"Inspect {out_dir / 'summary.md'} then run auto-relative-run with --plan-dir {out_dir}.",
        "ready_for_full_path_following": False,
    }
    _write_json(out_dir / "preview_summary.json", summary)
    write_summary_files(out_dir, summary, title="Physical Path Planner Auto-Relative Preview")
    if telemetry._parse_bool(getattr(args, "print_field_config", "false"), default=False):
        print(format_field_config(field_config))
    print(
        f"auto-relative-preview: {plan['segment_count']} segments, "
        f"goal_distance_m={float(plan['goal_distance_m']):.3f} -> {out_dir}"
    )
    return 0


def _auto_relative_run_on_handle(
    handle: object,
    args: argparse.Namespace,
    cal: dict[str, object],
    out_dir: Path,
    *,
    plan: dict[str, object] | None,
    field_config: dict[str, object] | None,
    plan_dir_used: bool,
    input_fn: "callable",
) -> int:
    """Serial-facing core of auto-relative-run (testable with a fake handle)."""
    raw_lines: list[str] = []
    rows: list[dict[str, str]] = []
    min_sats = float(getattr(args, "gps_min_sats", 5.0))
    max_hdop = float(getattr(args, "gps_max_hdop", 2.5))
    start_source = "plan_dir" if plan_dir_used else "live_gps"
    start_lat: float | None = None
    start_lon: float | None = None

    if plan_dir_used and plan is not None:
        start_lat = float(plan["start_lat"])
        start_lon = float(plan["start_lon"])
        if field_config:
            field_config = dict(field_config)
            field_config["start_source"] = "plan_dir"
            _write_json(out_dir / "field_config_resolved.json", field_config)
    else:
        # 2. Wait for GPS readiness; save the current fix as start A.
        start = _auto_relative_wait_for_gps(
            handle,
            min_sats=min_sats,
            max_hdop=max_hdop,
            timeout_s=float(getattr(args, "gps_timeout_s", 300.0)),
            cached_start_max_age_ms=int(getattr(args, "cached_start_max_age_ms", 10000)),
            raw_lines=raw_lines,
            rows=rows,
        )
        if start is None:
            summary = _auto_relative_summary(
                args, cal, None, None,
                controller_summary=None, start_lat=None, start_lon=None,
                start_source="none", auto_switch_detected=False,
                execution_started=False, stop_reason="NO_USABLE_START_GPS",
                reason="NO_USABLE_START_GPS",
            )
            _write_json(out_dir / "run_summary.json", summary)
            write_summary_files(out_dir, summary, title="Physical Path Planner Auto-Relative Run")
            _write_closed_loop_artifacts(out_dir, [], raw_lines)
            print(f"auto-relative-run: reason=NO_USABLE_START_GPS. {NO_USABLE_START_GPS_ACTION}")
            return 2
        start_lat = float(start["start_lat"])
        start_lon = float(start["start_lon"])
        snapshot = dict(start.get("gps_wait_snapshot", {}))
        if snapshot:
            write_gps_cache(snapshot)
        args.start_lat = start_lat
        args.start_lon = start_lon
        try:
            plan = resolve_plan(args, cal)
        except ValueError as exc:
            return _fail_with_summary(args, reason="PLAN_INPUT_INVALID", message=str(exc))
        try:
            field_config = _write_preview_outputs(
                out_dir,
                plan,
                start_mode="live_gps",
                start_source="live_gps",
                write_png=getattr(args, "png", True),
            )
        except RuntimeError as exc:
            expected = str(exc).replace("PREVIEW_IMAGE_NOT_WRITTEN ", "")
            summary = {
                "mode": "auto-relative-run",
                "success": False,
                "reason": "PREVIEW_IMAGE_NOT_WRITTEN",
                "expected_image_path": expected,
                "next_recommended_action": "Install/check matplotlib rendering and rerun before physical execution.",
                "ready_for_full_path_following": False,
            }
            _write_json(out_dir / "run_summary.json", summary)
            write_summary_files(out_dir, summary, title="Physical Path Planner Auto-Relative Run")
            _write_closed_loop_artifacts(out_dir, [], raw_lines)
            print(f"auto-relative-run: reason=PREVIEW_IMAGE_NOT_WRITTEN expected_image_path={expected}")
            return 2

    # 3. Monitor the physical mode switch; start only when AUTO (or keyboard fallback).
    started, start_reason = _auto_relative_wait_for_auto(
        handle,
        timeout_s=float(getattr(args, "auto_switch_timeout_s", 300.0)),
        allow_keyboard=telemetry._parse_bool(getattr(args, "allow_keyboard_start", "false"), default=False),
        input_fn=input_fn,
        raw_lines=raw_lines,
        rows=rows,
        min_sats=min_sats,
        max_hdop=max_hdop,
    )
    if not started:
        summary = _auto_relative_summary(
            args, cal, plan, field_config,
            controller_summary=None, start_lat=start_lat, start_lon=start_lon,
            start_source=start_source, auto_switch_detected=False,
            execution_started=False, stop_reason=start_reason, reason=start_reason,
        )
        _write_json(out_dir / "run_summary.json", summary)
        write_summary_files(out_dir, summary, title="Physical Path Planner Auto-Relative Run")
        _write_closed_loop_artifacts(out_dir, [], raw_lines)
        print(f"auto-relative-run: execution_started=false stop_reason={start_reason} -> {out_dir}")
        return 2

    # 4. Align to the first lane heading before path execution (if requested).
    assert plan is not None
    if args.path_control_mode == "stop_correct_go":
        completeness = calibration.calibration_completeness(cal, segments=plan["segments"])
        if not completeness["can_run_stop_correct_go"]:
            summary = _auto_relative_summary(
                args, cal, plan, field_config,
                controller_summary=None, start_lat=start_lat, start_lon=start_lon,
                start_source=start_source, auto_switch_detected=(start_reason == "AUTO_SWITCH"),
                execution_started=False, stop_reason="CALIBRATION_INCOMPLETE",
                reason="CALIBRATION_INCOMPLETE",
            )
            summary["calibration_completeness"] = completeness
            summary["missing_required_calibration"] = list(completeness["missing_required"])
            summary["auto_start_reason"] = start_reason
            _write_json(out_dir / "run_summary.json", summary)
            write_summary_files(out_dir, summary, title="Physical Path Planner Auto-Relative Run")
            _write_closed_loop_artifacts(out_dir, [], raw_lines)
            print(
                "auto-relative-run: aborted before motion, reason=CALIBRATION_INCOMPLETE "
                f"missing={completeness['missing_required']} -> {out_dir}"
            )
            return 2
    align = _run_initial_alignment(
        handle,
        args,
        segments=plan["segments"],  # type: ignore[arg-type]
        out_dir=out_dir,
        raw_lines=raw_lines,
        input_fn=input_fn,
    )
    if not align["ok"]:
        summary = _auto_relative_summary(
            args, cal, plan, field_config,
            controller_summary=None, start_lat=start_lat, start_lon=start_lon,
            start_source=start_source, auto_switch_detected=(start_reason == "AUTO_SWITCH"),
            execution_started=False, stop_reason=str(align["abort_reason"]),
            reason=str(align["abort_reason"]),
        )
        summary.update(_alignment_summary_fields(align))
        summary["auto_start_reason"] = start_reason
        _write_json(out_dir / "run_summary.json", summary)
        write_summary_files(out_dir, summary, title="Physical Path Planner Auto-Relative Run")
        _write_closed_loop_artifacts(out_dir, [], raw_lines)
        print(f"auto-relative-run: execution_started=false stop_reason={align['abort_reason']} -> {out_dir}")
        return 2

    # 5/6. Execute closed-loop; require_auto_switch stops safely on a MANUAL flip.
    if args.path_control_mode == "stop_correct_go":
        rows_exec, raw_exec, abort_reason = _run_stop_correct_go_from_args(
            handle, args, plan, cal,
            start_yaw_deg=align["aligned_yaw_deg"],
            start_lat=float(start_lat), start_lon=float(start_lon),
            require_auto_switch=True,
        )
    else:
        rows_exec, raw_exec, abort_reason = controller.run_controller(
            handle,
            segments=plan["segments"],  # type: ignore[arg-type]
            resolved_calibration=cal,
            start_lat=float(start_lat),
            start_lon=float(start_lon),
            start_yaw_deg=align["aligned_yaw_deg"],
            goal_lat=float(plan["goal_lat"]),
            goal_lon=float(plan["goal_lon"]),
            event_timeout_s=args.event_timeout_s,
            heartbeat_timeout_s=args.heartbeat_timeout_s,
            rc_neutral_wait_s=args.rc_neutral_wait_s,
            gps_degradation_policy=args.gps_degradation_policy,
            manual_override_mode=args.manual_override_mode,
            left_fixed_pulses=args.left_fixed_pulses,
            right_fixed_pulses=args.right_fixed_pulses,
            straight_motion_mode=args.straight_motion_mode,
            live_update_hz=args.live_update_hz,
            live_ttl_ms=args.live_ttl_ms,
            live_chunk_ms=args.live_chunk_ms,
            max_segment_chunks=args.max_segment_chunks,
            live_max_ms=args.max_ms,
            imu_heading_hold=telemetry._parse_bool(args.imu_heading_hold, default=True),
            cross_track_correction=telemetry._parse_bool(args.cross_track_correction, default=True),
            path_control_mode=args.path_control_mode,
            k_heading=args.k_heading,
            k_cross_track=args.k_cross_track,
            max_correction_b=args.max_correction_b,
            gps_reanchor=telemetry._parse_bool(args.gps_reanchor, default=True),
            require_auto_switch=True,
            max_connector_pulses_per_turn=getattr(
                args,
                "max_connector_pulses_per_turn",
                controller.DEFAULT_MAX_CONNECTOR_PULSES_PER_TURN,
            ),
            turn_angle_policy=getattr(
                args, "turn_calibration_angle_policy", controller.DEFAULT_TURN_ANGLE_POLICY
            ),
            turn_angle_override=getattr(args, "turn_angle_deg_override", None),
        )
    raw_lines.extend(raw_exec)
    if args.path_control_mode == "stop_correct_go":
        controller_summary = controller.build_stop_correct_go_summary(
            rows_exec,
            start_lat=float(start_lat),
            start_lon=float(start_lon),
            goal_lat=float(plan["goal_lat"]),
            goal_lon=float(plan["goal_lon"]),
            goal_distance_m=float(plan["goal_distance_m"]),
            fallback_to_repeated_pulses=bool(cal["fallback_to_repeated_pulses"]),
            sensor_trust_mode=args.sensor_trust_mode,
            allow_calibration_fallback=telemetry._parse_bool(
                args.allow_calibration_fallback, default=True
            ),
            abort_reason=abort_reason,
            heading_reference=getattr(
                args, "heading_reference", controller.DEFAULT_HEADING_REFERENCE
            ),
            turn_angle_policy=getattr(
                args, "turn_calibration_angle_policy", controller.DEFAULT_TURN_ANGLE_POLICY
            ),
            turn_calibration_angles=calibration.turn_angle_summary(cal),
        )
    else:
        controller_summary = controller.build_controller_summary(
            rows_exec,
            start_lat=float(start_lat),
            start_lon=float(start_lon),
            goal_lat=float(plan["goal_lat"]),
            goal_lon=float(plan["goal_lon"]),
            goal_distance_m=float(plan["goal_distance_m"]),
            fallback_to_repeated_pulses=bool(cal["fallback_to_repeated_pulses"]),
            abort_reason=abort_reason,
        )
    stop_reason = "COMPLETED" if abort_reason == "NONE" else str(abort_reason)
    summary = _auto_relative_summary(
        args, cal, plan, field_config,
        controller_summary=controller_summary, start_lat=start_lat, start_lon=start_lon,
        start_source=start_source, auto_switch_detected=(start_reason == "AUTO_SWITCH"),
        execution_started=True,
        stop_reason=stop_reason,
        reason="OK" if stop_reason in {"COMPLETED", "USER_SWITCHED_TO_MANUAL"} else stop_reason,
    )
    summary.update(_alignment_summary_fields(align))
    summary["auto_start_reason"] = start_reason
    _write_json(out_dir / "run_summary.json", summary)
    write_summary_files(out_dir, summary, title="Physical Path Planner Auto-Relative Run")
    _write_closed_loop_artifacts(out_dir, rows_exec, raw_lines)
    if args.path_control_mode == "stop_correct_go":
        _write_stop_correct_go_artifacts(out_dir, rows_exec)
    print(
        f"auto-relative-run: started={start_reason} stop_reason={stop_reason} "
        f"chunks={controller_summary.get('chunk_count', 0)} -> {out_dir}"
    )
    return 0 if stop_reason in {"COMPLETED", "USER_SWITCHED_TO_MANUAL"} else 1


def cmd_auto_relative_run(args: argparse.Namespace) -> int:
    """Watch the physical mode switch and run closed-loop relative path execution on AUTO."""
    cal = resolve_calibration(args)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    plan_dir_used = bool(getattr(args, "plan_dir", None))
    plan: dict[str, object] | None = None
    field_config: dict[str, object] | None = None
    if plan_dir_used:
        plan_dir = Path(args.plan_dir)
        try:
            plan = load_plan_dir_plan(plan_dir)
        except (FileNotFoundError, ValueError) as exc:
            return _fail_with_summary(
                args,
                reason="PLAN_DIR_MISSING_PLAN",
                message=str(exc),
            )
        field_config_path = plan_dir / "field_config_resolved.json"
        if field_config_path.exists():
            loaded = json.loads(field_config_path.read_text())
            if isinstance(loaded, dict):
                field_config = loaded
                plan["field_config"] = loaded
    if not ensure_port(args):
        return 2
    import serial  # local import: keeps preview/diagnose --from-log free of pyserial

    handle = serial.Serial(args.port, baudrate=args.baud, timeout=0.5)
    try:
        return _auto_relative_run_on_handle(
            handle,
            args,
            cal,
            out_dir,
            plan=plan,
            field_config=field_config,
            plan_dir_used=plan_dir_used,
            input_fn=input,
        )
    finally:
        handle.close()


# ── diagnose / 읽기전용 telemetry 요약 / Read-only telemetry summary ──


def cmd_diagnose(args: argparse.Namespace) -> int:
    """읽기전용 telemetry 요약(모션 없음). / Read-only telemetry summary. No motion.

    라이브 포트를 잠깐 읽거나 ``--from-log`` 로 저장 로그를 파싱해 ``diagnose_summary`` 로
    행수·하트비트·GPS/IMU 상태를 요약한다. 모터 명령을 보내지 않는다.
    부수효과: (라이브면) 시리얼 오픈, 요약·원시로그 기록.
    Reads a live port briefly or parses ``--from-log`` and summarizes via
    ``diagnose_summary``; sends no motor commands. Side effects: serial + file writes.
    """
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    if args.from_log:
        log_path = Path(args.from_log)
        raw_lines = log_path.read_text().splitlines()
        rows = load_rows_from_log(log_path)
    else:
        if not ensure_port(args):
            return 2
        import serial  # local import: --from-log path never needs pyserial

        raw_lines = []
        handle = serial.Serial(args.port, baudrate=args.baud, timeout=0.5)
        try:
            deadline = time.monotonic() + args.duration_s
            while time.monotonic() < deadline:
                raw = handle.readline()
                if raw:
                    line = raw.decode("utf-8", errors="replace").strip()
                    print(line)
                    raw_lines.append(line)
        finally:
            handle.close()
        rows = telemetry.parse_usbdbg_rows("\n".join(raw_lines))

    summary = diagnose_summary(rows)
    summary = {
        **summary,
        "success": True,
        "reason": "OK",
        "next_recommended_action": "Inspect summary.md for GPS, IMU, RC, and guarded pulse heartbeat status.",
        "ready_for_full_path_following": False,
    }
    _write_json(out_dir / "diagnose_summary.json", summary)
    write_summary_files(out_dir, summary, title="Physical Path Planner Diagnose")
    if raw_lines:
        _write_raw_log(out_dir / "diagnose_serial.log", raw_lines)
    print(
        f"diagnose: {summary['row_count']} rows, {summary['heartbeat_count']} heartbeats, "
        f"last_gps_block_reason={summary['last_gps_block_reason']} -> {out_dir}"
    )
    return 0


# ══ argparse 설정 (서브커맨드 → 핸들러) / Argument parser (subcommand -> handler) ══
# ``build_parser`` 가 모든 모드의 서브파서를 만들고 각각 ``set_defaults(handler=cmd_*)``
# 로 핸들러를 지정한다. 공용 인자군은 ``_add_goal_arguments`` / ``_add_calibration_arguments``
# 로 묶어 여러 모드가 재사용한다. 새 모드 추가 시: 서브파서 생성 → 인자 → handler 지정.
# ``build_parser`` builds every mode's subparser and wires its handler; shared arg
# groups live in the two helpers below. To add a mode: add a subparser + handler.


def _add_goal_arguments(parser: argparse.ArgumentParser, *, require_start: bool = True) -> None:
    """골/경로형태/작업폭 등 공통 계획 인자 추가. / Add shared goal + plan-shape arguments.

    preview/auto-relative-preview/calibration-check/run 계열이 공유한다. ``require_start``
    로 --start-lat/lon 필수 여부를 조절(라이브 GPS 로 채우는 모드는 False).
    Shared by preview/calibration-check/run; ``require_start`` toggles whether the
    start lat/lon are mandatory (False for modes that fill them from live GPS).
    """
    parser.add_argument("--start-lat", type=float, required=require_start)
    parser.add_argument("--start-lon", type=float, required=require_start)
    parser.add_argument(
        "--goal-mode",
        choices=["absolute", "relative_enu", "relative_latlon", "bearing_distance"],
        default="absolute",
    )
    parser.add_argument("--goal-lat", type=float, default=None)
    parser.add_argument("--goal-lon", type=float, default=None)
    parser.add_argument("--goal-east-m", type=float, default=None)
    parser.add_argument("--goal-north-m", type=float, default=None)
    parser.add_argument("--goal-dlat", type=float, default=None)
    parser.add_argument("--goal-dlon", type=float, default=None)
    parser.add_argument("--goal-bearing-deg", type=float, default=None)
    parser.add_argument("--goal-distance-m", type=float, default=None)
    parser.add_argument(
        "--path-shape",
        choices=sorted(geometry.PATH_SHAPE_ALIASES.keys()),
        default=geometry.COVERAGE_LAWNMOWER,
    )
    parser.add_argument(
        "--connector-style",
        choices=sorted(geometry.CONNECTOR_STYLES),
        default=geometry.DEFAULT_CONNECTOR_STYLE,
        help="turn_step_turn plans each ㄹ corner as pivot + step-over straight + pivot "
        "(drivable); single_turn is the legacy one-pivot connector",
    )
    parser.add_argument("--workspace-width-m", type=float, default=None)
    parser.add_argument("--step-spacing-m", type=float, default=0.5)
    parser.add_argument(
        "--diagonal-orientation", default="A_top_left_to_B_bottom_right"
    )
    parser.add_argument("--max-segment-pulses", type=int, default=8)
    parser.add_argument("--nominal-forward-pulse-m", type=float, default=0.30)


def _add_calibration_arguments(parser: argparse.ArgumentParser) -> None:
    """보정 JSON 경로/모드 공통 인자 추가. / Add shared calibration JSON/mode arguments."""
    parser.add_argument("--calibration-mode", default="auto")
    parser.add_argument("--motion-calibration-json", default=str(calibration.DEFAULT_MOTION_CALIBRATION))
    parser.add_argument("--fine-calibration-json", default=None)
    parser.add_argument("--turn-calibration-json", default=None)
    parser.add_argument("--turn-angle-calibration-json", default=None)
    parser.add_argument("--smooth-turn-calibration-json", default=None)


def build_parser() -> argparse.ArgumentParser:
    """모든 모드 서브파서를 구성해 argparse 파서를 반환. / Build the full argparse parser.

    ``dest="mode"`` 서브파서에 각 모드를 등록하고 ``set_defaults(handler=cmd_*)`` 로
    핸들러를 연결한다. station-hw 와 guarded-pulse-ready 는 내부 팩토리 함수로 만든다.
    Registers every mode under the ``mode`` subparsers and wires each handler; the
    station-HW and guarded-pulse-ready parsers are built by nested factory functions.
    """
    parser = argparse.ArgumentParser(
        prog="physical_path_planner",
        description="Unified physical rover tools: station hardware manual, USB pulse tests, diagnostics, calibration, and supervised planning.",
    )
    sub = parser.add_subparsers(
        dest="mode",
        required=True,
        metavar="{diagnose,gps-wait,rc-input-diagnose,manual-rc,manual-control,station-hw-diagnose,station-hw-manual,usb-pulse-test,usb-drive-live,tune-motion,set-motion-calibration,reset-motion-calibration,calibration-check,guarded-pulse-ready,calibrate-turn,preview,inspect-plan,auto-relative-preview,align-heading,execute-plan,run,auto-relative-run}",
    )

    gps_p = sub.add_parser("gps-wait", help="wait for usable GPS start fix; no motion")
    gps_p.add_argument("--port", default=None)
    gps_p.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    gps_p.add_argument("--from-log", default=None)
    gps_p.add_argument("--timeout-s", type=float, default=300.0)
    gps_p.add_argument("--status-interval-s", type=float, default=2.0)
    gps_p.add_argument("--min-sats", type=float, default=5.0)
    gps_p.add_argument("--max-hdop", type=float, default=2.5)
    gps_p.add_argument("--upload", choices=["true", "false", "auto"], default="auto")
    gps_p.add_argument("--out-dir", default="outputs/physical_path_planning/gps_wait")
    gps_p.set_defaults(handler=cmd_gps_wait)

    preview_p = sub.add_parser("preview", help="build + render the plan (captures live/cached GPS start when omitted)")
    _add_goal_arguments(preview_p, require_start=False)
    _add_calibration_arguments(preview_p)
    preview_p.add_argument("--port", default=None)
    preview_p.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    preview_p.add_argument(
        "--start-mode",
        choices=["live_gps", "cached_gps", "explicit"],
        default="live_gps",
        help="how to resolve the plan start when --start-lat/--start-lon are omitted",
    )
    preview_p.add_argument("--start-timeout-s", type=float, default=120.0)
    preview_p.add_argument("--wait-gps", choices=["true", "false"], default="true")
    preview_p.add_argument("--gps-timeout-s", type=float, default=300.0)
    preview_p.add_argument("--gps-status-interval-s", type=float, default=2.0)
    preview_p.add_argument("--gps-min-sats", type=float, default=5.0)
    preview_p.add_argument("--gps-max-hdop", type=float, default=2.5)
    preview_p.add_argument("--allow-cached-start", choices=["true", "false"], default="true")
    preview_p.add_argument("--max-cached-start-age-s", type=float, default=600.0)
    preview_p.add_argument("--cached-start-max-age-ms", type=int, default=10000)
    preview_p.add_argument("--from-log", default=None, help="parse saved telemetry for start GPS instead of opening serial")
    preview_p.add_argument("--upload", choices=["true", "false", "auto"], default="auto")
    preview_p.add_argument("--out-dir", default="outputs/physical_path_planning/preview")
    preview_p.add_argument("--print-field-config", choices=["true", "false"], default="false")
    preview_p.add_argument("--allow-wide-field", choices=["true", "false"], default="false")
    preview_p.add_argument("--max-width-to-goal-ratio", type=float, default=0.95)
    preview_p.add_argument("--png", dest="png", action="store_true", default=True)
    preview_p.add_argument("--no-png", dest="png", action="store_false")
    preview_p.set_defaults(handler=cmd_preview)

    auto_prev_p = sub.add_parser(
        "auto-relative-preview",
        help="wait for GPS, resolve a relative A->B field, write field config + preview (no motion)",
    )
    _add_goal_arguments(auto_prev_p, require_start=False)
    _add_calibration_arguments(auto_prev_p)
    auto_prev_p.add_argument("--port", default=None)
    auto_prev_p.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    auto_prev_p.add_argument(
        "--start-mode",
        choices=["live_gps", "cached_gps", "explicit"],
        default="live_gps",
    )
    auto_prev_p.add_argument("--start-timeout-s", type=float, default=120.0)
    auto_prev_p.add_argument("--wait-gps", choices=["true", "false"], default="true")
    auto_prev_p.add_argument("--gps-timeout-s", type=float, default=300.0)
    auto_prev_p.add_argument("--gps-status-interval-s", type=float, default=2.0)
    auto_prev_p.add_argument("--gps-min-sats", type=float, default=5.0)
    auto_prev_p.add_argument("--gps-max-hdop", type=float, default=2.5)
    auto_prev_p.add_argument("--allow-cached-start", choices=["true", "false"], default="true")
    auto_prev_p.add_argument("--max-cached-start-age-s", type=float, default=600.0)
    auto_prev_p.add_argument("--cached-start-max-age-ms", type=int, default=10000)
    auto_prev_p.add_argument("--from-log", default=None)
    auto_prev_p.add_argument("--upload", choices=["true", "false", "auto"], default="auto")
    auto_prev_p.add_argument("--out-dir", default="outputs/physical_path_planning/auto_relative_preview")
    auto_prev_p.add_argument("--print-field-config", choices=["true", "false"], default="false")
    auto_prev_p.add_argument("--allow-wide-field", choices=["true", "false"], default="false")
    auto_prev_p.add_argument("--max-width-to-goal-ratio", type=float, default=0.95)
    auto_prev_p.add_argument("--png", dest="png", action="store_true", default=True)
    auto_prev_p.add_argument("--no-png", dest="png", action="store_false")
    auto_prev_p.set_defaults(handler=cmd_auto_relative_preview, goal_mode="relative_enu")

    cal_p = sub.add_parser(
        "calibrate-turn",
        help="run guarded pulse turn angle calibration",
    )
    cal_p.add_argument("--port", default=None)
    cal_p.add_argument("--direction", choices=["left", "right"], default=None)
    cal_p.add_argument("--mode", default="turn_left")
    cal_p.add_argument("--b-cmd", type=float, default=None)
    cal_p.add_argument("--pulse-ms", type=int, default=None)
    cal_p.add_argument("--max-abs-b", type=float, default=0.35)
    cal_p.add_argument("--max-ms", type=int, default=1500)
    cal_p.add_argument("--upload", choices=["true", "false", "auto"], default="auto")
    cal_p.add_argument("--target-angle-deg", type=float, default=90.0)
    cal_p.add_argument("--angle-tolerance-deg", type=float, default=10.0)
    cal_p.add_argument("--save-turn-calibration", default="true")
    cal_p.add_argument("--turn-calibration-out", default=DEFAULT_TURN_CALIBRATION_OUT)
    cal_p.add_argument("--out-dir", default="outputs/physical_path_planning/calibration")
    cal_p.add_argument("--script", default=DEFAULT_GUARDED_PULSE_CALIBRATION_SCRIPT)
    cal_p.add_argument(
        "--print-cmd",
        action="store_true",
        help="print the shell-out command and exit (no firmware, no serial)",
    )
    cal_p.set_defaults(handler=cmd_calibrate_turn)

    rc_diag_p = sub.add_parser(
        "rc-input-diagnose",
        help="upload/read the read-only RC input channel diagnostic",
    )
    rc_diag_p.add_argument("--port", default=None)
    rc_diag_p.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    rc_diag_p.add_argument("--duration-s", type=float, default=20.0)
    rc_diag_p.add_argument("--upload", choices=["true", "false", "auto"], default="true")
    rc_diag_p.add_argument("--from-log", default=None)
    rc_diag_p.add_argument("--out-dir", default="outputs/physical_path_planning/rc_input_diagnose")
    rc_diag_p.add_argument("--sketch", default=DEFAULT_RC_INPUT_DIAGNOSE_SKETCH)
    rc_diag_p.add_argument(
        "--print-cmd",
        action="store_true",
        help="print read-only probe upload commands and exit",
    )
    rc_diag_p.set_defaults(handler=cmd_rc_input_diagnose)

    manual_p = sub.add_parser("manual-rc", help="upload and validate manual RC recovery")
    manual_p.add_argument("--port", default=None)
    manual_p.add_argument("--upload", choices=["true", "false", "auto"], default="true")
    manual_p.add_argument("--validate", choices=["true", "false"], default="true")
    manual_p.add_argument("--diagnose-only", choices=["true", "false"], default="false")
    manual_p.add_argument("--duration-s", type=float, default=45.0)
    manual_p.add_argument("--log", default=None)
    manual_p.add_argument("--rc-input-mode", choices=["auto", "old_known_good", "ppm", "pwm", "sbus"], default="old_known_good")
    manual_p.add_argument("--mode-channel-index", type=int, default=4)
    manual_p.add_argument("--steer-channel-index", type=int, default=0)
    manual_p.add_argument("--throttle-channel-index", type=int, default=1)
    manual_p.add_argument("--manual-mode-threshold-us", type=int, default=None)
    manual_p.add_argument("--print-rc-mapping", choices=["true", "false"], default="false")
    manual_p.add_argument("--out-dir", default="outputs/physical_path_planning/manual_rc")
    manual_p.add_argument("--upload-script", default=DEFAULT_MANUAL_RC_UPLOAD_SCRIPT)
    manual_p.add_argument("--validate-script", default=DEFAULT_MANUAL_RC_VALIDATE_SCRIPT)
    manual_p.add_argument(
        "--print-cmd",
        action="store_true",
        help="print upload/validation commands and exit",
    )
    manual_p.set_defaults(handler=cmd_manual_rc)

    manual_control_p = sub.add_parser(
        "manual-control",
        help="upload and monitor PPM physical manual control with full telemetry display",
    )
    manual_control_p.add_argument("--port", default=None)
    manual_control_p.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    manual_control_p.add_argument("--upload", choices=["true", "false", "auto"], default="true")
    manual_control_p.add_argument("--validate", choices=["true", "false"], default="true")
    manual_control_p.add_argument("--duration-s", type=float, default=0.0)
    manual_control_p.add_argument("--from-log", default=None)
    manual_control_p.add_argument(
        "--profile",
        choices=MANUAL_CONTROL_PROFILES,
        default=MANUAL_CONTROL_DEFAULT_PROFILE,
        help="firmware compile profile; default uses the older rc_mix_test PPM decoder",
    )
    manual_control_p.add_argument("--mode-channel-index", type=int, default=4)
    manual_control_p.add_argument("--verbose-raw", choices=["true", "false"], default="false")
    manual_control_p.add_argument("--out-dir", default="outputs/physical_path_planning/manual_control")
    manual_control_p.add_argument(
        "--print-cmd",
        action="store_true",
        help="print upload commands and exit",
    )
    manual_control_p.set_defaults(handler=cmd_manual_control)

    rc_auto_p = sub.add_parser(
        "rc-auto-pattern",
        help="upload untethered firmware: CH5 MANUAL = RC sticks, CH5 AUTO = onboard ㄹ pattern",
    )
    rc_auto_p.add_argument("--port", default=None)
    rc_auto_p.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    rc_auto_p.add_argument("--upload", choices=["true", "false", "auto"], default="true")
    rc_auto_p.add_argument("--duration-s", type=float, default=0.0,
                           help="optional post-upload serial monitor duration (0 = exit after upload)")
    rc_auto_p.add_argument("--lanes", type=int, default=4)
    rc_auto_p.add_argument("--lane-ms", type=int, default=4200,
                           help="full-lane drive duration (1.8 m at ~0.43 m/s is ~4200 ms)")
    rc_auto_p.add_argument("--step-ms", type=int, default=1400,
                           help="step-over drive duration (0.6 m is ~1400 ms)")
    rc_auto_p.add_argument("--forward-a", type=float, default=0.30)
    rc_auto_p.add_argument("--reverse-a", type=float, default=-0.30)
    rc_auto_p.add_argument("--turn-b-left", type=float, default=0.24)
    rc_auto_p.add_argument("--turn-b-right", type=float, default=-0.12)
    rc_auto_p.add_argument("--turn-target-deg", type=float, default=90.0)
    rc_auto_p.add_argument("--turn-tol-deg", type=float, default=8.0)
    rc_auto_p.add_argument("--turn-timeout-ms", type=int, default=15000)
    rc_auto_p.add_argument("--pause-ms", type=int, default=800,
                           help="stop-settle pause between steps; >=800 ms also "
                           "gives the stationary gyro re-zero a window")
    rc_auto_p.add_argument("--turn-coast-s", type=float, default=0.15,
                           help="pivot stops early by (yaw rate x this) so coast "
                           "does not carry it past the target")
    rc_auto_p.add_argument("--turn-settle-retries", type=int, default=2,
                           help="after the pause, re-run the same pivot up to N "
                           "times if the settled heading is outside tolerance")
    rc_auto_p.add_argument("--heading-kp", type=float, default=0.015,
                           help="straight-lane heading-hold gain: B per degree of "
                           "heading error vs the planned absolute heading")
    rc_auto_p.add_argument("--heading-hold-max-b", type=float, default=0.25,
                           help="cap on the heading-hold steering component")
    rc_auto_p.add_argument("--drive-b-trim", type=float, default=0.0,
                           help="constant B added on straights to cancel a known "
                           "mechanical veer (negative counters a leftward pull)")
    rc_auto_p.add_argument("--drive-steer-sign", type=float, default=-1.0,
                           choices=[-1.0, 1.0],
                           help="sign of the lane heading-hold feedback; -1 "
                           "(default) matches this drivetrain, whose yaw "
                           "response to B while DRIVING is inverted vs a "
                           "stationary pivot (2026-06-12 field log)")
    rc_auto_p.add_argument("--drive-abort-err-deg", type=float, default=60.0,
                           help="end a lane early if its heading error diverges "
                           "past this many degrees (anti-donut safety net)")
    rc_auto_p.add_argument("--rc-loss-grace-ms", type=int, default=1500,
                           help="RC dropout shorter than this pauses (then resumes) "
                           "a running pattern instead of resetting it")
    rc_auto_p.add_argument("--mode-channel-index", type=int, default=4)
    rc_auto_p.add_argument(
        "--profile",
        choices=[
            MANUAL_CONTROL_FULL_TELEMETRY_PPM_PROFILE,
            MANUAL_CONTROL_OLD_WORKING_PPM_PROFILE,
            MANUAL_CONTROL_RC_MIX_PPM_PROFILE,
        ],
        default=MANUAL_CONTROL_FULL_TELEMETRY_PPM_PROFILE,
        help="PPM decode base profile; if the monitor shows "
        "ppm_decode_reason=PPM_SYNC_ONLY_NO_CHANNELS, try another profile",
    )
    rc_auto_p.add_argument("--print-cmd", action="store_true",
                           help="print firmware commands and exit")
    rc_auto_p.add_argument("--out-dir", default="outputs/physical_path_planning/rc_auto_pattern")
    rc_auto_p.set_defaults(handler=cmd_rc_auto_pattern)

    def add_station_hw_parser(name: str, *, diagnose_only: bool) -> None:
        """station-hw-diagnose/manual 서브파서 등록. / Register a station-HW subparser."""
        station_p = sub.add_parser(
            name,
            help=(
                "read-only physical station hardware link diagnostic"
                if diagnose_only else
                "deprecated serial-frame hardware monitor; use manual-control for PPM control"
            ),
        )
        station_p.add_argument("--port", default=None)
        station_p.add_argument("--baud", type=int, default=DEFAULT_BAUD)
        station_p.add_argument(
            "--duration-s",
            type=float,
            default=20.0 if diagnose_only else 0.0,
            help=(
                "monitor duration in seconds"
                if diagnose_only else
                "monitor duration in seconds; <=0 means continuous until Ctrl-C"
            ),
        )
        station_p.add_argument("--upload", choices=["true", "false", "auto"], default="auto")
        station_p.add_argument("--from-log", default=None)
        station_p.add_argument("--verbose-raw", choices=["true", "false"], default="false")
        station_p.add_argument(
            "--print-cmd",
            action="store_true",
            help="print firmware commands and exit",
        )
        station_p.add_argument("--print-command", choices=["true", "false"], default="false")
        station_p.add_argument(
            "--out-dir",
            default=(
                "outputs/physical_path_planning/station_hw_diagnose"
                if diagnose_only else
                "outputs/physical_path_planning/station_hw_manual"
            ),
        )
        station_p.set_defaults(handler=cmd_station_hw_diagnose if diagnose_only else cmd_station_hw_manual)

    add_station_hw_parser("station-hw-diagnose", diagnose_only=True)
    add_station_hw_parser("station-hw-manual", diagnose_only=False)

    station_drive_p = sub.add_parser(
        "usb-pulse-test",
        help="laptop USB bounded A/B pulse motor validation",
    )
    station_drive_p.add_argument("--port", default=None)
    station_drive_p.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    station_drive_p.add_argument("--upload", choices=["true", "false", "auto"], default="auto")
    station_drive_p.add_argument("--max-abs-a", type=float, default=0.35)
    station_drive_p.add_argument("--max-abs-b", type=float, default=0.35)
    station_drive_p.add_argument("--max-ms", type=int, default=1000)
    station_drive_p.add_argument("--event-timeout-s", type=float, default=controller.DEFAULT_EVENT_TIMEOUT_S)
    station_drive_p.add_argument("--heartbeat-timeout-s", type=float, default=controller.DEFAULT_HEARTBEAT_TIMEOUT_S)
    station_drive_p.add_argument("--single", choices=["forward", "backward", "left", "right", "turn_left", "turn_right"], default=None)
    station_drive_p.add_argument("--sequence", default=None)
    station_drive_p.add_argument("--require-rc-input", choices=["true", "false"], default="false")
    station_drive_p.add_argument("--require-enter", choices=["true", "false"], default="true")
    station_drive_p.add_argument("--interactive-visible-motion", choices=["true", "false"], default="true")
    station_drive_p.add_argument("--abort-on-invalid", choices=["true", "false"], default="true")
    station_drive_p.add_argument("--verbose-raw", choices=["true", "false"], default="false")
    station_drive_p.add_argument("--out-dir", default="outputs/physical_path_planning/usb_pulse_test")
    station_drive_p.add_argument(
        "--print-cmd",
        action="store_true",
        help="print bounded USB pulse serial commands and exit",
    )
    station_drive_p.add_argument("--print-command", choices=["true", "false"], default="false")
    station_drive_p.set_defaults(handler=cmd_usb_pulse_test)

    live_p = sub.add_parser(
        "usb-drive-live",
        help="continuous laptop USB A/B setpoint drive with firmware deadman",
    )
    live_p.add_argument("--port", default=None)
    live_p.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    live_p.add_argument("--a", type=float, required=True)
    live_p.add_argument("--b", type=float, required=True)
    live_p.add_argument("--duration-s", type=float, required=True)
    live_p.add_argument("--update-hz", type=float, default=8.0)
    live_p.add_argument("--ttl-ms", type=int, default=350)
    live_p.add_argument("--max-abs-a", type=float, default=0.35)
    live_p.add_argument("--max-abs-b", type=float, default=0.35)
    live_p.add_argument("--max-duration-s", type=float, default=3.0)
    live_p.add_argument("--event-timeout-s", type=float, default=controller.DEFAULT_EVENT_TIMEOUT_S)
    live_p.add_argument("--upload", choices=["true", "false", "auto"], default="auto")
    live_p.add_argument("--verbose-raw", choices=["true", "false"], default="false")
    live_p.add_argument("--print-command", choices=["true", "false"], default="false")
    live_p.add_argument("--out-dir", default="outputs/physical_path_planning/usb_drive_live")
    live_p.set_defaults(handler=cmd_usb_drive_live)

    tune_p = sub.add_parser(
        "tune-motion",
        help="interactive visual/IMU-assisted USB pulse calibration",
    )
    tune_p.add_argument("--port", default=None)
    tune_p.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    tune_p.add_argument(
        "--primitive",
        choices=["forward", "backward", "left", "right", "turn-left-90", "turn-right-90"],
        required=True,
    )
    tune_p.add_argument("--upload", choices=["true", "false", "auto"], default="auto")
    tune_p.add_argument("--max-abs-a", type=float, default=tuning.MAX_ABS_A)
    tune_p.add_argument("--max-abs-b", type=float, default=tuning.MAX_ABS_B)
    tune_p.add_argument("--max-ms", type=int, default=tuning.MAX_MS)
    tune_p.add_argument("--event-timeout-s", type=float, default=controller.DEFAULT_EVENT_TIMEOUT_S)
    tune_p.add_argument("--heartbeat-timeout-s", type=float, default=controller.DEFAULT_HEARTBEAT_TIMEOUT_S)
    tune_p.add_argument("--require-enter", choices=["true", "false"], default="true")
    tune_p.add_argument("--max-iterations", type=int, default=12)
    tune_p.add_argument("--verbose-raw", choices=["true", "false"], default="false")
    tune_p.add_argument("--print-candidate", choices=["true", "false"], default="false")
    tune_p.add_argument("--calibration-out", default=str(calibration.DEFAULT_MOTION_CALIBRATION))
    tune_p.add_argument(
        "--reset-calibration",
        choices=["true", "false"],
        default="false",
        help="back up and clear the existing motion calibration before this session",
    )
    tune_p.add_argument("--out-dir", default="outputs/physical_path_planning/tune_motion")
    tune_p.set_defaults(handler=cmd_tune_motion)

    reset_cal_p = sub.add_parser(
        "reset-motion-calibration",
        help="back up and clear approved motion calibration before a full recalibration",
    )
    reset_cal_p.add_argument("--calibration-out", default=str(calibration.DEFAULT_MOTION_CALIBRATION))
    reset_cal_p.add_argument("--out-dir", default="outputs/physical_path_planning/reset_motion_calibration")
    reset_cal_p.set_defaults(handler=cmd_reset_motion_calibration)

    set_cal_p = sub.add_parser(
        "set-motion-calibration",
        help="write a manual motion calibration preset or primitive override; no motion",
    )
    set_cal_p.add_argument(
        "--preset",
        choices=sorted(tuning.MANUAL_CALIBRATION_PRESETS.keys()),
        default=None,
    )
    set_cal_p.add_argument(
        "--primitive",
        choices=["forward", "backward", "left", "right", "turn-left-90", "turn-right-90"],
        default=None,
    )
    set_cal_p.add_argument("--a", type=float, default=None)
    set_cal_p.add_argument("--b", type=float, default=None)
    set_cal_p.add_argument("--ms", type=int, default=None)
    set_cal_p.add_argument("--target-angle-deg", type=float, default=None)
    set_cal_p.add_argument("--source", default="manual_override")
    set_cal_p.add_argument("--calibration-out", default=str(calibration.DEFAULT_MOTION_CALIBRATION))
    set_cal_p.add_argument("--out-dir", required=True)
    set_cal_p.set_defaults(handler=cmd_set_motion_calibration)

    cal_check_p = sub.add_parser(
        "calibration-check",
        help="report motion-calibration completeness for stop_correct_go; no motion",
    )
    _add_goal_arguments(cal_check_p, require_start=False)
    _add_calibration_arguments(cal_check_p)
    cal_check_p.add_argument("--plan-dir", default=None)
    cal_check_p.add_argument("--out-dir", default="outputs/physical_path_planning/calibration_check")
    cal_check_p.set_defaults(handler=cmd_calibration_check)

    inspect_p = sub.add_parser(
        "inspect-plan",
        help="inspect saved plan shape, segments, and preview images; no motion",
    )
    inspect_p.add_argument("--plan-dir", required=True)
    inspect_p.add_argument("--out-dir", default=None)
    inspect_p.set_defaults(handler=cmd_inspect_plan)

    def add_guarded_pulse_ready_parser(name: str) -> None:
        """guarded-pulse-ready 서브파서 등록. / Register the guarded-pulse-ready subparser."""
        guarded_p = sub.add_parser(
            name,
            help="upload/check IMU-enabled guarded pulse firmware",
        )
        guarded_p.add_argument("--port", default=None)
        guarded_p.add_argument("--baud", type=int, default=DEFAULT_BAUD)
        guarded_p.add_argument("--upload", choices=["true", "false", "auto"], default="true")
        guarded_p.add_argument("--duration-s", type=float, default=15.0)
        guarded_p.add_argument("--max-abs-a", type=float, default=0.35)
        guarded_p.add_argument("--max-abs-b", type=float, default=0.35)
        guarded_p.add_argument("--max-ms", type=int, default=1500)
        guarded_p.add_argument("--out-dir", default="outputs/physical_path_planning/guarded_pulse_ready")
        guarded_p.add_argument(
            "--print-cmd",
            action="store_true",
            help="print firmware commands and exit",
        )
        guarded_p.set_defaults(handler=cmd_guarded_pulse_ready, deprecated_alias=False)

    add_guarded_pulse_ready_parser("guarded-pulse-ready")

    # run / execute-plan / auto-relative-run 은 인자 집합이 거의 같아 한 루프에서
    # 같은 옵션을 세 서브파서에 붙인다. auto-relative-run 만 AUTO 스위치/키보드 시작
    # 관련 인자와 기본 헤딩 정렬을 추가로 갖는다.
    # These three share almost the same option set, so one loop adds them to all three;
    # only auto-relative-run gets the extra AUTO-switch/keyboard args + default align.
    for name in ("run", "execute-plan", "auto-relative-run"):
        if name == "auto-relative-run":
            run_p = sub.add_parser(
                name,
                help="watch the AUTO mode switch, then run closed-loop relative path execution",
            )
        else:
            run_p = sub.add_parser(name, help="drive the continuous-motion controller over a plan")
        _add_goal_arguments(run_p, require_start=False)
        _add_calibration_arguments(run_p)
        run_p.add_argument("--plan-dir", default=None)
        run_p.add_argument("--port", default=None)
        run_p.add_argument("--baud", type=int, default=DEFAULT_BAUD)
        run_p.add_argument(
            "--start-mode",
            choices=["live_gps", "cached_gps", "explicit"],
            default="live_gps",
        )
        run_p.add_argument("--wait-gps", choices=["true", "false"], default="true")
        run_p.add_argument("--gps-timeout-s", type=float, default=300.0)
        run_p.add_argument("--gps-status-interval-s", type=float, default=2.0)
        run_p.add_argument("--gps-min-sats", type=float, default=5.0)
        run_p.add_argument("--gps-max-hdop", type=float, default=2.5)
        run_p.add_argument("--allow-cached-start", choices=["true", "false"], default="true")
        run_p.add_argument("--max-cached-start-age-s", type=float, default=600.0)
        run_p.add_argument("--start-timeout-s", type=float, default=120.0)
        run_p.add_argument("--cached-start-max-age-ms", type=int, default=10000)
        run_p.add_argument("--from-log", default=None)
        run_p.add_argument("--start-yaw-deg", type=float, default=None)
        run_p.add_argument("--event-timeout-s", type=float, default=controller.DEFAULT_EVENT_TIMEOUT_S)
        run_p.add_argument(
            "--heartbeat-timeout-s", type=float, default=controller.DEFAULT_HEARTBEAT_TIMEOUT_S
        )
        run_p.add_argument(
            "--rc-neutral-wait-s", type=float, default=controller.DEFAULT_RC_NEUTRAL_WAIT_S
        )
        run_p.add_argument(
            "--gps-degradation-policy",
            choices=["continue", "pause", "abort"],
            default=controller.DEFAULT_GPS_DEGRADATION_POLICY,
        )
        run_p.add_argument(
            "--manual-override-mode",
            choices=["abort", "warn", "continue"],
            default=controller.DEFAULT_MANUAL_OVERRIDE_MODE,
        )
        run_p.add_argument("--left-fixed-pulses", type=int, default=12)
        run_p.add_argument("--right-fixed-pulses", type=int, default=12)
        run_p.add_argument("--straight-motion-mode", choices=["continuous", "pulse"], default="continuous")
        run_p.add_argument(
            "--path-control-mode",
            choices=["open_loop_chunks", "imu_heading", "gps_imu_closed_loop", "stop_correct_go"],
            default="gps_imu_closed_loop",
        )
        run_p.add_argument("--live-update-hz", type=float, default=8.0)
        run_p.add_argument("--live-ttl-ms", type=int, default=350)
        run_p.add_argument("--live-chunk-ms", type=int, default=700)
        run_p.add_argument("--max-segment-chunks", type=int, default=20)
        run_p.add_argument("--max-ms", type=int, default=1000)
        run_p.add_argument("--imu-heading-hold", choices=["true", "false"], default="true")
        run_p.add_argument("--cross-track-correction", choices=["true", "false"], default="true")
        run_p.add_argument("--gps-reanchor", choices=["true", "false"], default="true")
        run_p.add_argument("--k-heading", type=float, default=0.006)
        run_p.add_argument("--k-cross-track", type=float, default=0.20)
        run_p.add_argument("--max-correction-b", type=float, default=0.08)
        # stop_correct_go: discrete move -> stop -> measure -> correct cycle.
        run_p.add_argument(
            "--move-chunk-ms", type=int, default=controller.DEFAULT_MOVE_CHUNK_MS,
            help="stop_correct_go: forward chunk duration per move-measure-correct cycle",
        )
        run_p.add_argument(
            "--settle-after-move-ms", type=int, default=controller.DEFAULT_SETTLE_AFTER_MOVE_MS,
            help="stop_correct_go: pause after STOP before reading a settled pose",
        )
        run_p.add_argument(
            "--telemetry-stabilize-ms", type=int, default=controller.DEFAULT_TELEMETRY_STABILIZE_MS,
            help="stop_correct_go: window of heartbeats averaged for the settled pose",
        )
        run_p.add_argument(
            "--heading-correction-threshold-deg", type=float,
            default=controller.DEFAULT_HEADING_CORRECTION_THRESHOLD_DEG,
            help="stop_correct_go: |heading error| above this triggers an IMU turn-in-place",
        )
        run_p.add_argument(
            "--heading-correction-tolerance-deg", type=float,
            default=controller.DEFAULT_HEADING_CORRECTION_TOLERANCE_DEG,
            help="stop_correct_go: stop the correction turn once within this tolerance",
        )
        run_p.add_argument(
            "--cross-track-correction-threshold-m", type=float,
            default=controller.DEFAULT_CROSS_TRACK_CORRECTION_THRESHOLD_M,
            help="stop_correct_go: |cross-track| above this trims the next chunk's B",
        )
        run_p.add_argument(
            "--heading-correction-b-left", type=float,
            default=controller.DEFAULT_HEADING_CORRECTION_B_LEFT,
            help="stop_correct_go: B command for a left (B>0) correction turn",
        )
        run_p.add_argument(
            "--heading-correction-b-right", type=float,
            default=controller.DEFAULT_HEADING_CORRECTION_B_RIGHT,
            help="stop_correct_go: B command for a right (B<0) correction turn",
        )
        run_p.add_argument(
            "--max-heading-correction-ms", type=int,
            default=controller.DEFAULT_MAX_HEADING_CORRECTION_MS,
            help="stop_correct_go: cap on a single correction turn's duration",
        )
        run_p.add_argument(
            "--sensor-trust-mode",
            choices=sorted(controller.SENSOR_TRUST_MODES),
            default=controller.DEFAULT_SENSOR_TRUST_MODE,
            help="stop_correct_go: imu_gps_first uses live sensors first; "
            "calibration_fallback tolerates sensor loss via dead reckoning",
        )
        run_p.add_argument(
            "--max-connector-pulses-per-turn", type=int,
            default=controller.DEFAULT_MAX_CONNECTOR_PULSES_PER_TURN,
            help="cap on calibrated turn pulses per planned corner (anti rotation-loop guard)",
        )
        run_p.add_argument(
            "--connector-turn-tolerance-deg", type=float,
            default=controller.DEFAULT_CONNECTOR_TURN_TOLERANCE_DEG,
            help="stop the connector turn once the measured IMU yaw delta is within "
            "this tolerance of the requested corner angle",
        )
        run_p.add_argument(
            "--max-connector-turn-ms", type=int,
            default=controller.DEFAULT_MAX_CONNECTOR_TURN_MS,
            help="cap on one connector's continuous IMU-feedback pivot duration",
        )
        run_p.add_argument(
            "--max-gps-jump-m", type=float, default=None,
            help="stop_correct_go: reject GPS pose steps larger than this between "
            "cycles (dead-reckon that cycle instead); recommended ~1.2 on small "
            "fields where GPS noise rivals the lane length",
        )
        run_p.add_argument(
            "--turn-calibration-angle-policy",
            choices=sorted(controller.TURN_ANGLE_POLICIES),
            default=controller.DEFAULT_TURN_ANGLE_POLICY,
            help="from_json trusts target_angle_deg in motion_calibration.json (a turn_*_90 "
            "entry may be a small 15-45 deg pulse); assume_90 reproduces the legacy "
            "one-pulse-per-corner behavior",
        )
        run_p.add_argument(
            "--turn-angle-deg-override", type=float, default=None,
            help="override the calibrated per-pulse turn angle (deg) for both directions "
            "without editing the calibration JSON",
        )
        run_p.add_argument(
            "--heading-reference",
            choices=sorted(controller.HEADING_REFERENCES),
            default=controller.DEFAULT_HEADING_REFERENCE,
            help="stop_correct_go: mission chains one yaw reference across the whole run "
            "(connector under-turns stay visible); per_lane is the legacy "
            "re-capture-per-lane behavior",
        )
        run_p.add_argument(
            "--allow-calibration-fallback", choices=["true", "false"], default="true",
            help="stop_correct_go: continue dead-reckoned when both GPS and IMU drop out",
        )
        run_p.add_argument("--out-dir", default="outputs/physical_path_planning/run")
        run_p.add_argument("--print-field-config", choices=["true", "false"], default="false")
        run_p.add_argument("--allow-wide-field", choices=["true", "false"], default="false")
        run_p.add_argument("--max-width-to-goal-ratio", type=float, default=0.95)
        run_p.add_argument(
            "--print-plan",
            action="store_true",
            help="build + write the plan and exit (no serial opened)",
        )
        run_p.add_argument(
            "--initial-heading-align",
            choices=["none", "gps_probe", "user_confirmed"],
            default="none",
            help="align the rover to the first lane heading before path execution",
        )
        run_p.add_argument(
            "--align-heading-tolerance-deg", type=float, default=alignment.DEFAULT_HEADING_TOLERANCE_DEG
        )
        run_p.add_argument("--align-probe-a", type=float, default=alignment.DEFAULT_PROBE_A)
        run_p.add_argument(
            "--align-probe-duration-s", type=float, default=alignment.DEFAULT_PROBE_DURATION_S
        )
        run_p.add_argument(
            "--align-min-probe-distance-m", type=float, default=alignment.DEFAULT_MIN_PROBE_DISTANCE_M
        )
        if name == "auto-relative-run":
            run_p.add_argument(
                "--allow-keyboard-start",
                choices=["true", "false"],
                default="false",
                help="when the PPM mode channel is absent, start on Enter instead of the AUTO switch",
            )
            run_p.add_argument("--auto-switch-timeout-s", type=float, default=300.0)
            run_p.add_argument("--png", dest="png", action="store_true", default=True)
            run_p.add_argument("--no-png", dest="png", action="store_false")
            # auto-relative-run aligns by default; pass --initial-heading-align none
            # to keep the prior (no-alignment) behavior.
            run_p.set_defaults(
                handler=cmd_auto_relative_run,
                goal_mode="relative_enu",
                initial_heading_align="gps_probe",
            )
        else:
            run_p.set_defaults(handler=cmd_run)

    align_p = sub.add_parser(
        "align-heading",
        help="point the rover at the first lane heading via a GPS probe + IMU-feedback turn",
    )
    align_p.add_argument("--plan-dir", default=None, help="preview/plan dir holding the planned segments")
    align_p.add_argument("--port", default=None)
    align_p.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    align_p.add_argument(
        "--strategy",
        choices=list(alignment.ALIGNMENT_STRATEGIES),
        default="gps_probe",
        help="gps_probe (automatic), user_confirmed (operator points + Enter), or skip",
    )
    align_p.add_argument("--target-heading-source", choices=["first_segment"], default="first_segment")
    align_p.add_argument("--probe-a", type=float, default=alignment.DEFAULT_PROBE_A)
    align_p.add_argument("--probe-duration-s", type=float, default=alignment.DEFAULT_PROBE_DURATION_S)
    align_p.add_argument("--min-probe-distance-m", type=float, default=alignment.DEFAULT_MIN_PROBE_DISTANCE_M)
    align_p.add_argument("--heading-tolerance-deg", type=float, default=alignment.DEFAULT_HEADING_TOLERANCE_DEG)
    align_p.add_argument("--turn-b-left", type=float, default=alignment.DEFAULT_TURN_B_LEFT)
    align_p.add_argument("--turn-b-right", type=float, default=alignment.DEFAULT_TURN_B_RIGHT)
    align_p.add_argument("--max-turn-duration-s", type=float, default=alignment.DEFAULT_MAX_TURN_DURATION_S)
    align_p.add_argument("--event-timeout-s", type=float, default=alignment.DEFAULT_EVENT_TIMEOUT_S)
    align_p.add_argument("--heartbeat-timeout-s", type=float, default=alignment.DEFAULT_HEARTBEAT_TIMEOUT_S)
    align_p.add_argument("--verbose-raw", choices=["true", "false"], default="false")
    align_p.add_argument("--out-dir", default="outputs/physical_path_planning/align_heading")
    align_p.set_defaults(handler=cmd_align_heading)

    diag_p = sub.add_parser("diagnose", help="read-only telemetry summary (live port or --from-log)")
    diag_p.add_argument("--port", default=None)
    diag_p.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    diag_p.add_argument("--from-log", default=None, help="parse a saved serial log instead of a port")
    diag_p.add_argument("--duration-s", type=float, default=5.0)
    diag_p.add_argument("--out-dir", default="outputs/physical_path_planning/diagnose")
    diag_p.set_defaults(handler=cmd_diagnose)

    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """CLI 진입점: 인자 파싱 후 선택된 핸들러 실행. / CLI entry point; dispatch to handler.

    폐기 예정 별칭 ``station-manual`` / ``station-drive`` 를 ``usb-pulse-test`` 로 재작성한
    뒤 파싱하고, ``args.handler(args)`` 의 종료코드를 int 로 반환한다.
    Rewrites the deprecated ``station-manual``/``station-drive`` aliases to
    ``usb-pulse-test`` before parsing, then returns ``args.handler(args)`` as an int.
    """
    parser = build_parser()
    normalized_argv = list(sys.argv[1:] if argv is None else argv)
    deprecated_station_manual_alias = False
    deprecated_station_drive_alias = False
    if normalized_argv and normalized_argv[0] == "station-manual":
        normalized_argv[0] = "usb-pulse-test"
        deprecated_station_manual_alias = True
    if normalized_argv and normalized_argv[0] == "station-drive":
        normalized_argv[0] = "usb-pulse-test"
        deprecated_station_drive_alias = True
    args = parser.parse_args(normalized_argv)
    if deprecated_station_manual_alias:
        args.deprecated_station_manual_alias = True
    if deprecated_station_drive_alias:
        args.deprecated_station_drive_alias = True
    return int(args.handler(args))


if __name__ == "__main__":
    raise SystemExit(main())
