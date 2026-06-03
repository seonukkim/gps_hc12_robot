from tools import hc12_operational_diagnose as diag


def test_filter_station_ports_excludes_usbmodem_and_dedups() -> None:
    paths = [
        "/dev/cu.usbserial-02442CA5",
        "/dev/cu.usbmodem12101",  # OpenRB, must be excluded
        "/dev/cu.usbserial-02442CA5",  # duplicate
        "/dev/cu.wchusbserial-1",
    ]
    assert diag.filter_station_ports(paths) == [
        "/dev/cu.usbserial-02442CA5",
        "/dev/cu.wchusbserial-1",
    ]


def test_classify_serial_error_device_not_configured() -> None:
    assert diag.classify_serial_error(OSError(6, "Device not configured")) == "DEVICE_NOT_CONFIGURED"


def test_classify_serial_error_port_not_found() -> None:
    assert diag.classify_serial_error(OSError(2, "No such file or directory")) == "PORT_NOT_FOUND"


def test_classify_serial_error_serial_exception_by_name() -> None:
    class SerialException(Exception):
        pass

    assert diag.classify_serial_error(SerialException("could not open port")) == "SERIAL_EXCEPTION"


def test_classify_serial_error_generic_oserror() -> None:
    assert diag.classify_serial_error(OSError("weird")) == "OS_ERROR"


def test_diagnose_verdict_station_off_is_invalid() -> None:
    v = diag.diagnose_verdict(
        mode="read-only", opened=True, serial_error_count=0, total_bytes=0,
        detected_uart_ports=[], pong_rx=0, station_off=True,
    )
    assert v == "TEST_INVALID_STATION_OFF"


def test_diagnose_verdict_ping_pong_link_ok() -> None:
    v = diag.diagnose_verdict(
        mode="ping-pong", opened=True, serial_error_count=0, total_bytes=40,
        detected_uart_ports=[], pong_rx=3,
    )
    assert v == "HC12_LINK_OK"


def test_diagnose_verdict_sweep_serial3() -> None:
    v = diag.diagnose_verdict(
        mode="read-only", opened=True, serial_error_count=0, total_bytes=80,
        detected_uart_ports=[3], pong_rx=0,
    )
    assert v == "UART_SWEEP_RECEIVED_ON_SERIAL3"


def test_diagnose_verdict_stable_no_rf_bytes() -> None:
    v = diag.diagnose_verdict(
        mode="read-only", opened=True, serial_error_count=0, total_bytes=0,
        detected_uart_ports=[], pong_rx=0,
    )
    assert v == "USB_SERIAL_STABLE_NO_RF_BYTES"


def test_diagnose_verdict_unstable_when_never_opened() -> None:
    v = diag.diagnose_verdict(
        mode="stability", opened=False, serial_error_count=3, total_bytes=0,
        detected_uart_ports=[], pong_rx=0,
    )
    assert v == "STATION_USB_UNSTABLE"


def test_dtr_rts_write_timeout_defaults() -> None:
    args = diag.build_parser().parse_args([])
    assert args.dtr == "low"
    assert args.rts == "low"
    assert args.write_timeout_s == 1.0


def test_dtr_rts_choices_parse() -> None:
    args = diag.build_parser().parse_args(
        ["--dtr", "high", "--rts", "default", "--write-timeout-s", "2"]
    )
    assert args.dtr == "high"
    assert args.rts == "default"
    assert args.write_timeout_s == 2.0


def test_verdict_write_only_tx_ok_no_rx() -> None:
    # Manual-equivalent write test: frames sent, no serial errors, nothing back.
    v = diag.diagnose_verdict(
        mode="write-only", opened=True, serial_error_count=0, total_bytes=0,
        detected_uart_ports=[], pong_rx=0, tx_count=5, rx_count=0,
    )
    assert v == "STATION_TX_OK_NO_RX"


def test_verdict_clean_tx_is_not_usb_unstable() -> None:
    # tx succeeded with zero serial errors -> must never be STATION_USB_UNSTABLE.
    v = diag.diagnose_verdict(
        mode="ping-pong", opened=True, serial_error_count=0, total_bytes=0,
        detected_uart_ports=[], pong_rx=0, tx_count=10, rx_count=0,
    )
    assert v == "HC12_NO_RF_RX"
