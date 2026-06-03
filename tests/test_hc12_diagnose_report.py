from gps_coverage_core.protocol import decode_frame, encode_frame
from tools import hc12_diagnose_report as report
from tools.hc12_link_probe import ping_frame
from tools.serial_raw_read import detect_uart_ports


def test_report_verdict_station_off() -> None:
    assert report.report_verdict({"station_off": True}) == "TEST_INVALID_STATION_OFF"


def test_report_verdict_link_ok_beats_sweep() -> None:
    ev = {"link_ok": True, "detected_uart_ports": [3]}
    assert report.report_verdict(ev) == "HC12_LINK_OK"


def test_report_verdict_sweep_serial3() -> None:
    assert report.report_verdict({"detected_uart_ports": [3]}) == "UART_SWEEP_RECEIVED_ON_SERIAL3"


def test_report_verdict_openrb_rx() -> None:
    ev = {"detected_uart_ports": [], "openrb_rx_detected": True}
    assert report.report_verdict(ev) == "STATION_TO_OPENRB_RX_DETECTED"


def test_report_verdict_stable_no_rf() -> None:
    ev = {"station_opened": True, "station_total_bytes": 0}
    assert report.report_verdict(ev) == "USB_SERIAL_STABLE_NO_RF_BYTES"


def test_report_verdict_no_rf_default() -> None:
    ev = {"station_opened": True, "station_total_bytes": 5}
    assert report.report_verdict(ev) == "HC12_NO_RF_RX"


def test_next_action_covers_all_verdicts() -> None:
    for verdict in report.VERDICTS:
        assert report.next_action(verdict)  # non-empty guidance


def test_parse_sweep_counters_takes_latest_and_rx_first() -> None:
    text = (
        "uart_sweep_alive=true Serial1_tx=1 Serial1_rx=0 Serial2_tx=1 Serial2_rx=10 Serial3_tx=1 Serial3_rx=0\n"
        "RX_FIRST_DETECTED port=Serial3 pins=...\n"
        "uart_sweep_alive=true Serial1_tx=5 Serial1_rx=0 Serial2_tx=5 Serial2_rx=99 Serial3_tx=5 Serial3_rx=7\n"
    )
    parsed = report.parse_sweep_counters(text)
    assert parsed["serial3_tx"] == 5
    assert parsed["serial3_rx"] == 7
    assert parsed["serial2_rx"] == 99
    assert parsed["rx_first_detected"] == [3]


def test_existing_hc12_frame_format_remains_compatible() -> None:
    # Sweep frame, STATION frame, and PING frame all use @TYPE,SEQ,PAYLOAD*CS.
    sweep = encode_frame("UART", 3, "TX_TEST")
    assert detect_uart_ports(sweep.decode()) == [3]
    decoded = decode_frame(encode_frame("STATION", 7, "TX_TEST"))
    assert decoded["type"] == "STATION" and decoded["seq"] == 7
    assert decode_frame(ping_frame(2))["type"] == "PING"


def test_report_verdict_station_tx_ok_no_rx() -> None:
    ev = {
        "station_tx_count": 5,
        "serial_error_count": 0,
        "station_total_bytes": 0,
        "station_opened": True,
    }
    assert report.report_verdict(ev) == "STATION_TX_OK_NO_RX"


def test_report_verdict_clean_tx_not_unstable() -> None:
    ev = {
        "station_tx_count": 5,
        "serial_error_count": 0,
        "station_total_bytes": 0,
        "station_usb_unstable": True,  # even if heuristically set, clean tx wins
        "station_opened": True,
    }
    assert report.report_verdict(ev) == "STATION_TX_OK_NO_RX"
