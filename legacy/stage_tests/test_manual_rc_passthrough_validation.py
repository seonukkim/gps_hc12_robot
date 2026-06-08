from __future__ import annotations

from pathlib import Path

from tools import manual_rc_passthrough_validation


REPO = Path(__file__).resolve().parents[1]
UPLOAD_SCRIPT = REPO / "scripts" / "upload_manual_rc_recovery_firmware.sh"


def test_manual_rc_parser_detects_manual_motion_and_zero_return() -> None:
    rows = manual_rc_passthrough_validation.parse_rows(
        "USBDBG manual_rc_recovery=true mode=MANUAL control_source=RC_MANUAL rc_ok=true neutral_ok=false "
        "manual_forward_cmd=0.220 manual_turn_cmd=0.000 physical_a_cmd=0.220 physical_b_cmd=0.000 "
        "final_left_cmd=0.220 final_right_cmd=0.220 motor_write_called=true physical_output_active=true "
        "active_target_source=manual_rc latched_stop=true\n"
        "USBDBG manual_rc_recovery=true mode=MANUAL control_source=RC_MANUAL rc_ok=true neutral_ok=true "
        "manual_forward_cmd=0.000 manual_turn_cmd=0.000 physical_a_cmd=0.000 physical_b_cmd=0.000 "
        "final_left_cmd=0.000 final_right_cmd=0.000 motor_write_called=false physical_output_active=false "
        "active_target_source=manual_rc latched_stop=true\n"
    )
    summary = manual_rc_passthrough_validation.evaluate_rows(rows)
    assert summary["manual_rc_passthrough_pass"] is True
    assert summary["reason"] == "MANUAL_RC_PASS"
    assert summary["control_source_rc_manual_seen"] is True
    assert summary["physical_a_nonzero_seen"] is True
    assert summary["nonzero_final_command_rows"] == 1
    assert summary["neutral_zero_return_seen"] is True
    assert summary["station_command_used"] is False
    assert summary["ready_for_full_path_following"] is False


def test_manual_rc_parser_rejects_autonomous_target_source() -> None:
    rows = manual_rc_passthrough_validation.parse_rows(
        "USBDBG mode=MANUAL control_source=RC_MANUAL rc_ok=true neutral_ok=false "
        "manual_forward_cmd=0.200 final_left_cmd=0.200 final_right_cmd=0.200 "
        "physical_output_active=true active_target_source=compile_time\n"
        "USBDBG mode=MANUAL control_source=RC_MANUAL rc_ok=true neutral_ok=true "
        "manual_forward_cmd=0.000 final_left_cmd=0.000 final_right_cmd=0.000 "
        "physical_output_active=false active_target_source=manual_rc\n"
    )
    summary = manual_rc_passthrough_validation.evaluate_rows(rows)
    assert summary["manual_rc_passthrough_pass"] is False
    assert "AUTONOMOUS_TARGET_SOURCE_SEEN" in summary["reasons"]
    assert summary["ready_for_full_path_following"] is False


def test_manual_rc_all_raw_channels_zero_is_rc_input_absent() -> None:
    rows = manual_rc_passthrough_validation.parse_rows(
        "\n".join(
            [
                "USBDBG manual_rc_recovery=true mode=FAILSAFE rc_ok=false auto_sw=false "
                "steer_us=0 throttle_us=0 mode_us=0 raw_ch1_us=0 raw_ch2_us=0 raw_ch3_us=0 raw_ch4_us=0 "
                "raw_ch5_us=0 raw_ch6_us=0 raw_ch7_us=0 raw_ch8_us=0 control_source=STOP "
                "final_left_cmd=0 final_right_cmd=0 physical_output_active=false",
                "USBDBG manual_rc_recovery=true mode=FAILSAFE rc_ok=false auto_sw=false "
                "steer_us=0 throttle_us=0 mode_us=0 raw_ch1_us=0 raw_ch2_us=0 raw_ch3_us=0 raw_ch4_us=0 "
                "raw_ch5_us=0 raw_ch6_us=0 raw_ch7_us=0 raw_ch8_us=0 control_source=STOP "
                "final_left_cmd=0 final_right_cmd=0 physical_output_active=false",
            ]
        )
    )
    summary = manual_rc_passthrough_validation.evaluate_rows(rows)
    assert summary["manual_rc_passthrough_ok"] is False
    assert summary["rc_input_detected"] is False
    assert summary["reason"] == "RC_INPUT_ABSENT"
    assert "receiver signal wire" in str(summary["next_recommended_action"])


def test_valid_rc_manual_steering_maps_to_physical_b() -> None:
    rows = manual_rc_passthrough_validation.parse_rows(
        "USBDBG manual_rc_recovery=true mode=MANUAL control_source=RC_MANUAL rc_ok=true neutral_ok=false "
        "manual_forward_cmd=0.000 manual_turn_cmd=0.180 physical_a_cmd=0.000 physical_b_cmd=0.180 "
        "final_left_cmd=-0.180 final_right_cmd=0.180 motor_write_called=true physical_output_active=true "
        "active_target_source=manual_rc\n"
        "USBDBG manual_rc_recovery=true mode=MANUAL control_source=RC_MANUAL rc_ok=true neutral_ok=true "
        "manual_forward_cmd=0.000 manual_turn_cmd=0.000 physical_a_cmd=0.000 physical_b_cmd=0.000 "
        "final_left_cmd=0.000 final_right_cmd=0.000 motor_write_called=false physical_output_active=false "
        "active_target_source=manual_rc\n"
    )
    summary = manual_rc_passthrough_validation.evaluate_rows(rows)
    assert summary["manual_rc_passthrough_ok"] is True
    assert summary["physical_b_nonzero_seen"] is True
    assert summary["ready_for_full_path_following"] is False


def test_rc_ok_false_keeps_manual_outputs_zero() -> None:
    rows = manual_rc_passthrough_validation.parse_rows(
        "USBDBG manual_rc_recovery=true mode=FAILSAFE control_source=STOP rc_ok=false neutral_ok=false "
        "manual_forward_cmd=0.000 manual_turn_cmd=0.000 physical_a_cmd=0.000 physical_b_cmd=0.000 "
        "final_left_cmd=0.000 final_right_cmd=0.000 motor_write_called=false physical_output_active=false\n"
    )
    summary = manual_rc_passthrough_validation.evaluate_rows(rows)
    assert summary["manual_rc_passthrough_ok"] is False
    assert summary["physical_a_nonzero_seen"] is False
    assert summary["physical_b_nonzero_seen"] is False
    assert summary["nonzero_final_motor_command_seen"] is False


def test_manual_rc_recovery_compile_flags_disable_stage20_and_path_following() -> None:
    text = UPLOAD_SCRIPT.read_text(encoding="utf-8")
    assert "MANUAL_RC_RECOVERY=1" in text
    assert "MANUAL_FORWARD_SIGN=-1" in text
    assert "MANUAL_TURN_SIGN=1" in text
    assert "MOTOR_OUTPUT_SWAP_LR=0" in text
    assert "DRIVE_CALIBRATION_ENABLE=0" in text
    assert "PHYSICAL_PATH_FOLLOWING_ENABLE=0" in text
    assert "PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0" in text
    assert "PATH_FOLLOWING_HC12_ENABLED=0" in text
    assert "STAGE20_PHYSICAL_AB_GUARDED_CRAWL=0" in text
    assert "AUTO_MOTION_ARMED=0" in text
