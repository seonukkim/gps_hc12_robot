"""tools.analyze_heading_readiness_log 의 판정·요약 계약 검증 / Verdict + summary contract tests.

무엇을/왜 (What/why):
  무동작 헤딩 준비도 분석기의 오프라인 로직을 하드웨어 없이 고정한다. 합성 ``USBDBG`` 로그
  줄들을 만들어 ``parse_usbdbg_rows`` -> ``summarize`` 파이프라인을 태우고, 요약 텍스트 안의
  특정 필드와 최종 ``verdict`` 를 단언한다. 이 두 함수의 출력 형식은 도구의 공개 계약이므로
  여기서 못 박는다.
  Locks the analyzer's offline logic without hardware by feeding synthetic ``USBDBG`` lines
  through parse->summarize and asserting on summary fields and the final verdict.

고정하는 불변식 (Invariants locked):
  - 판정 심각도 우선순위(위가 우선): MOTOR_SAFETY_FAIL > SENSOR_BASELINE_FAIL >
    HEADING_READY_PASS > HEADING_COURSE_LATCH_AVAILABLE > HEADING_COURSE_ACCEPTED_BUT_NOT_OUTPUT
    > HEADING_NOT_READY_BUT_MOVEMENT_SUFFICIENT > SENSOR_BASELINE_PASS.
  - GPS 품질/IMU 실패는 모두 단일 SENSOR_BASELINE_FAIL 로 접힌다.
  - 물리 출력이 활성이면(모터 미차단) 다른 무엇보다 먼저 MOTOR_SAFETY_FAIL.
  - 이동량이 임계값을 넘고 리셋 유사 이벤트가 잡히면 movement 요약과 설명 문구가 나타난다.
  - summarize 출력의 key=value 형식 자체가 계약(아래 부분 문자열 단언이 그것을 고정).

리팩토링 노트 (Refactoring notes):
  _base_usbdbg 의 기본 필드 묶음은 "센서 기준선 통과 + 헤딩 미준비"의 최소 정상 로그를 뜻한다.
  요약 키 이름/순서나 임계값을 바꾸면 이 파일의 부분 문자열 단언을 함께 갱신할 것.
"""
from tools.analyze_heading_readiness_log import parse_usbdbg_rows, summarize


# ── 테스트 헬퍼: 합성 USBDBG 로그 줄 빌더 / Test helpers: synthetic USBDBG line builders ──
def _base_usbdbg(**overrides: str) -> str:
    """"정상 기준선 + 헤딩 미준비"의 한 줄 USBDBG 로그를 만든다(필드는 kwargs 로 덮어씀).
    Build one baseline-healthy, heading-not-ready USBDBG line; override any field via kwargs."""
    fields = {
        "gps_block_reason": "OK",
        "position_source": "gps",
        "heading_agreement_diag": "WAITING_GPS_COURSE",
        "path_following_block_reason": "NO_HEADING",
        "imu_type": "BMI160",
        "imu_present": "true",
        "imu_i2c_addr": "0x68",
        "imu_chip_id": "0xD1",
        "imu_pmu_normal": "true",
        "imu_data_plausible": "true",
        "physical_block_reason": "COMPILE_GATE_OFF",
        "physical_output_active": "false",
        "mode": "MANUAL",
        "control_source": "RC_MANUAL",
        "rc_ok": "true",
        "neutral_ok": "true",
        "gps_sats": "9",
        "gps_hdop": "1.0",
        "course_displacement_m": "0.0",
        "estimated_course_deg": "10.0",
        "gps_course_deg": "NA",
        "heading_ready": "false",
        "heading_agreement_error_deg": "NA",
        "imu_accel_mag_g": "1.0",
        "imu_gyro_mag_dps": "0.1",
        "final_left_cmd": "0.000",
        "final_right_cmd": "0.000",
        "current_lat": "35.000000",
        "current_lon": "129.000000",
    }
    fields.update(overrides)
    return "USBDBG " + " ".join(f"{key}={value}" for key, value in fields.items())


def _verdict_for(lines: list[str]) -> str:
    """USBDBG 줄들을 parse->summarize 로 태우고 요약에서 verdict= 값만 뽑아 반환.
    Run the lines through parse->summarize and extract just the verdict= value."""
    rows = parse_usbdbg_rows("\n".join(lines))
    summary = summarize(rows)
    return next(line.split("=", 1)[1] for line in summary if line.startswith("verdict="))


# ── 판정 시나리오별 테스트 / Per-scenario verdict tests ──
def test_heading_analyzer_reports_movement_and_transient_course_block() -> None:
    """충분히 움직였으나 코스가 스치듯만 잡히는 케이스: 이동량·리셋이벤트 요약과
    "MOVEMENT_SUFFICIENT" 판정, 그리고 일시적 코스 계산 설명 문구가 나오는지 확인.
    Moved enough but course is only transient: checks movement/reset-event summary,
    the MOVEMENT_SUFFICIENT verdict, and the transient-course explanation line."""
    text = "\n".join(
        [
            "noise",
            _base_usbdbg(),
            _base_usbdbg(
                gps_sats="9",
                gps_hdop="0.9",
                course_displacement_m="2.1",
                estimated_course_deg="20.0",
                heading_agreement_error_deg="5.0",
                current_lat="35.000150",
            ),
            _base_usbdbg(
                gps_sats="9",
                gps_hdop="0.9",
                course_displacement_m="0.0",
                estimated_course_deg="20.0",
                heading_agreement_error_deg="5.0",
                current_lat="35.000300",
            ),
        ]
    )

    rows = parse_usbdbg_rows(text)
    lines = summarize(rows)
    joined = "\n".join(lines)

    assert "usbdbg_rows=3" in joined
    assert "gps_block_reason_counts={'OK': 3}" in joined
    assert "moved_enough=true" in joined
    assert "reset_like_event_count=1" in joined
    assert "gps_course_deg_non_na_count=0" in joined
    assert "verdict=HEADING_NOT_READY_BUT_MOVEMENT_SUFFICIENT" in joined
    assert "course heading is computed only briefly" in joined


def test_heading_analyzer_verdict_motor_safety_fail() -> None:
    """물리 출력이 활성(physical_output_active=true)이면 다른 무엇보다 우선해 MOTOR_SAFETY_FAIL.
    An active physical output forces MOTOR_SAFETY_FAIL ahead of every other check."""
    assert (
        _verdict_for(
            [
                _base_usbdbg(
                    physical_output_active="true",
                    gps_course_anchor_reset_reason="COURSE_ACCEPTED_RESEED",
                )
            ]
        )
        == "MOTOR_SAFETY_FAIL"
    )


def test_heading_analyzer_verdict_gps_quality_fail() -> None:
    """GPS 차단 이유가 OK 가 아니면(NO_LOCATION) 센서 기준선 실패로 접힘.
    A non-OK GPS block reason collapses into SENSOR_BASELINE_FAIL."""
    assert _verdict_for([_base_usbdbg(gps_block_reason="NO_LOCATION")]) == "SENSOR_BASELINE_FAIL"


def test_heading_analyzer_verdict_imu_fail() -> None:
    """IMU 데이터 타당성 실패(imu_data_plausible=false)도 동일하게 센서 기준선 실패.
    An implausible-IMU flag likewise yields SENSOR_BASELINE_FAIL."""
    assert _verdict_for([_base_usbdbg(imu_data_plausible="false")]) == "SENSOR_BASELINE_FAIL"


def test_heading_analyzer_verdict_heading_ready_pass() -> None:
    """heading_ready=true 이고 GPS 코스가 있으면 최상위 통과 HEADING_READY_PASS.
    heading_ready=true with a GPS course gives the top-level HEADING_READY_PASS."""
    assert (
        _verdict_for([_base_usbdbg(heading_ready="true", gps_course_deg="22.0")])
        == "HEADING_READY_PASS"
    )


def test_heading_analyzer_verdict_course_accepted_but_not_output() -> None:
    """코스는 수락(reseed)됐지만 출력 코스가 없으면 ACCEPTED_BUT_NOT_OUTPUT 판정.
    Course accepted (reseed) but no output course -> HEADING_COURSE_ACCEPTED_BUT_NOT_OUTPUT."""
    verdict = _verdict_for(
        [
            _base_usbdbg(current_lat="35.000000"),
            _base_usbdbg(
                current_lat="35.000300",
                course_displacement_m="2.2",
                gps_course_anchor_reset_reason="COURSE_ACCEPTED_RESEED",
                gps_course_estimated_deg="224.8",
                gps_course_output_deg="NA",
                gps_course_deg="NA",
            ),
        ]
    )

    assert verdict == "HEADING_COURSE_ACCEPTED_BUT_NOT_OUTPUT"


def test_heading_analyzer_verdict_output_valid_is_latch_available() -> None:
    """출력 코스가 유효(gps_course_output_valid=true)하면 출력 가능한 래치 확보로 판정.
    A valid output course means an output-ready latch -> HEADING_COURSE_LATCH_AVAILABLE."""
    verdict = _verdict_for(
        [
            _base_usbdbg(
                gps_course_last_accepted_valid="true",
                gps_course_last_accepted_deg="71.0",
                gps_course_last_accepted_age_ms="700",
                gps_course_output_valid="true",
                gps_course_output_deg="71.0",
                gps_course_output_age_ms="700",
                heading_agreement_error_deg="8.5",
            )
        ]
    )

    assert verdict == "HEADING_COURSE_LATCH_AVAILABLE"


def test_heading_analyzer_verdict_sensor_baseline_pass() -> None:
    """이동/코스 증거가 없는 순수 기준선 로그는 SENSOR_BASELINE_PASS(기본 통과)로 판정.
    A pure baseline log with no movement/course evidence yields SENSOR_BASELINE_PASS."""
    assert _verdict_for([_base_usbdbg()]) == "SENSOR_BASELINE_PASS"
