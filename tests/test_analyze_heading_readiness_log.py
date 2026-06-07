from tools.analyze_heading_readiness_log import parse_usbdbg_rows, summarize


def _base_usbdbg(**overrides: str) -> str:
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
    rows = parse_usbdbg_rows("\n".join(lines))
    summary = summarize(rows)
    return next(line.split("=", 1)[1] for line in summary if line.startswith("verdict="))


def test_heading_analyzer_reports_movement_and_transient_course_block() -> None:
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
    assert _verdict_for([_base_usbdbg(gps_block_reason="NO_LOCATION")]) == "SENSOR_BASELINE_FAIL"


def test_heading_analyzer_verdict_imu_fail() -> None:
    assert _verdict_for([_base_usbdbg(imu_data_plausible="false")]) == "SENSOR_BASELINE_FAIL"


def test_heading_analyzer_verdict_heading_ready_pass() -> None:
    assert (
        _verdict_for([_base_usbdbg(heading_ready="true", gps_course_deg="22.0")])
        == "HEADING_READY_PASS"
    )


def test_heading_analyzer_verdict_course_accepted_but_not_output() -> None:
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
    assert _verdict_for([_base_usbdbg()]) == "SENSOR_BASELINE_PASS"
