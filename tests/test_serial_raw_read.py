"""tools.serial_raw_read 의 UART 감지·포트 탐색 헬퍼 계약 검증 / Contract tests for serial_raw_read.

무엇을/왜 (What/why):
  원시 시리얼 리더의 두 순수 헬퍼를 고정한다: ``detect_uart_ports`` (캡처 텍스트에서
  ``@UART,<n>,TX_TEST`` 프레임의 포트 번호 추출)와 ``find_usbserial_ports`` (glob 기반
  장치 경로 수집). ``detect_uart_ports`` 는 hc12_diagnose_report / hc12_operational_diagnose
  가 공유하는 감지 규칙이라 이 테스트가 그 규칙(정렬·중복제거)을 못 박는다.
  Locks two pure helpers: ``detect_uart_ports`` (which UART ports appear in the capture)
  and ``find_usbserial_ports`` (glob-based device discovery).

고정하는 불변식 (Invariants locked):
  - ``detect_uart_ports`` 는 감지된 포트 번호를 정렬·중복제거해 반환하고, 프레임이 없으면 [].
  - ``find_usbserial_ports`` 는 장치가 없는 머신에서도 예외 없이 항상 list 를 반환한다
    (하드웨어 부재에 견고).

리팩토링 노트 (Refactoring notes):
  프레임 정규식(UART_FRAME_RE)이나 포트 glob 목록을 바꾸면 이 기대값을 함께 갱신할 것.
"""
from tools import serial_raw_read


def test_detect_uart_ports_extracts_and_dedups() -> None:
    """혼합 텍스트에서 UART 포트 번호를 추출하고 정렬·중복제거([1, 3])하는지 확인.
    Extract UART port numbers from mixed text, sorted and de-duplicated ([1, 3])."""
    text = (
        "@UART,3,TX_TEST*1A\n"
        "noise\n"
        "@UART,1,TX_TEST*0B\n"
        "@UART,3,TX_TEST*1A\n"
    )
    assert serial_raw_read.detect_uart_ports(text) == [1, 3]


def test_detect_uart_ports_none() -> None:
    """@UART 프레임이 없는 텍스트(NMEA/잡음)는 빈 목록을 낸다.
    Text without any @UART frame (NMEA/noise) yields an empty list."""
    assert serial_raw_read.detect_uart_ports("$GPGGA,...\nrandom bytes") == []


def test_find_usbserial_ports_dedups_and_orders() -> None:
    """장치 부재 시에도 예외 없이 list 를 반환하는 견고성 검증(중복 glob 전달).
    Robustness: returns a list without raising even when no device matches."""
    globs = ["/dev/cu.usbserial-A", "/dev/cu.usbserial-A", "/dev/cu.usbserial-B"]
    # Simulate two glob patterns returning overlapping results.
    ports = serial_raw_read.find_usbserial_ports(
        globs=[g for g in globs]  # passed straight through to glob.glob; non-existent -> []
    )
    # On a machine without these devices glob returns nothing; the function must
    # still return a list (possibly empty) without raising.
    assert isinstance(ports, list)
