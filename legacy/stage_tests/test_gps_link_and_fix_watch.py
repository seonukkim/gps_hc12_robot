from __future__ import annotations

from tools import check_gps_link_and_fix_watch
from tools import gps_link_and_fix_watch


def test_gps_watch_distinguishes_no_serial_data_from_no_fix() -> None:
    no_serial_summary, _ = gps_link_and_fix_watch.summarize_rows(
        gps_link_and_fix_watch.parse_rows(
            "USBDBG gps_block_reason=NO_LOCATION position_source=NA current_lat=NA current_lon=NA\n"
        ),
        requested_duration_s=1200,
        elapsed_s=1200,
    )
    assert no_serial_summary["gps_status"] == "GPS_NO_SERIAL_DATA"

    no_fix_summary, _ = gps_link_and_fix_watch.summarize_rows(
        gps_link_and_fix_watch.parse_rows(
            "USBDBG gps_chars=120 gps_block_reason=NO_LOCATION position_source=NA current_lat=NA current_lon=NA\n"
        ),
        requested_duration_s=1200,
        elapsed_s=1200,
    )
    assert no_fix_summary["gps_status"] == "GPS_SERIAL_PRESENT_NO_FIX"


def test_gps_watch_does_not_fail_early_before_timeout() -> None:
    summary, _ = gps_link_and_fix_watch.summarize_rows(
        gps_link_and_fix_watch.parse_rows(
            "USBDBG gps_block_reason=NO_LOCATION position_source=NA current_lat=NA current_lon=NA\n"
        ),
        requested_duration_s=1200,
        elapsed_s=30,
    )
    assert summary["gps_status"] == "GPS_WAIT_LONGER"
    result = check_gps_link_and_fix_watch.evaluate_summary(summary)
    assert result["verdict"] == "PASS"
    assert result["ready_for_full_path_following"] is False


def test_gps_watch_ready_full_with_late_fix() -> None:
    summary, _ = gps_link_and_fix_watch.summarize_rows(
        gps_link_and_fix_watch.parse_rows(
            "USBDBG gps_chars=12 gps_block_reason=NO_LOCATION position_source=NA current_lat=NA current_lon=NA\n"
            "USBDBG gps_chars=240 gps_block_reason=OK position_source=gps current_lat=35.0 current_lon=129.0 gps_sats=6 gps_hdop=1.6\n"
        ),
        requested_duration_s=1200,
        elapsed_s=900,
    )
    assert summary["gps_status"] == "GPS_READY_FULL"
    assert summary["gps_fix_rows"] == 1
    assert summary["motor_command_generated"] is False
    assert summary["ready_for_full_path_following"] is False


def test_gps_watch_accepts_nmea_counter_as_serial_presence() -> None:
    summary, _ = gps_link_and_fix_watch.summarize_rows(
        gps_link_and_fix_watch.parse_rows(
            "USBDBG nmea_sentence_count=22 gps_block_reason=NO_LOCATION position_source=NA current_lat=NA current_lon=NA\n"
        ),
        requested_duration_s=1200,
        elapsed_s=1200,
    )
    assert summary["gps_status"] == "GPS_SERIAL_PRESENT_NO_FIX"
