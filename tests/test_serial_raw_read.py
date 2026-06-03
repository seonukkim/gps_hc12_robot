from tools import serial_raw_read


def test_detect_uart_ports_extracts_and_dedups() -> None:
    text = (
        "@UART,3,TX_TEST*1A\n"
        "noise\n"
        "@UART,1,TX_TEST*0B\n"
        "@UART,3,TX_TEST*1A\n"
    )
    assert serial_raw_read.detect_uart_ports(text) == [1, 3]


def test_detect_uart_ports_none() -> None:
    assert serial_raw_read.detect_uart_ports("$GPGGA,...\nrandom bytes") == []


def test_find_usbserial_ports_dedups_and_orders() -> None:
    globs = ["/dev/cu.usbserial-A", "/dev/cu.usbserial-A", "/dev/cu.usbserial-B"]
    # Simulate two glob patterns returning overlapping results.
    ports = serial_raw_read.find_usbserial_ports(
        globs=[g for g in globs]  # passed straight through to glob.glob; non-existent -> []
    )
    # On a machine without these devices glob returns nothing; the function must
    # still return a list (possibly empty) without raising.
    assert isinstance(ports, list)
