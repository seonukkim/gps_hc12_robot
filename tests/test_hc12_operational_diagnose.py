"""tools.hc12_operational_diagnose 의 포트필터·에러분류·단일실행 판정·CLI 파서 검증.
Port-filter, error-classify, single-run verdict, and CLI-parser tests for hc12_operational_diagnose.

무엇을/왜 (What/why):
  현장 진단 도구의 순수 로직을 하드웨어 없이 고정한다: OpenRB(usbmodem) 제외 포트 필터,
  errno/메시지 기반 에러 분류(pyserial 미의존), ``diagnose_verdict`` 의 우선순위 분기, 그리고
  --dtr/--rts/--write-timeout-s 의 known-good 기본값과 파싱.
  Locks the field tool's pure logic without hardware: station-port filtering,
  pyserial-free error classification, single-run verdict priority, and CLI defaults.

고정하는 불변식 (Invariants locked):
  - filter_station_ports 는 usbmodem(OpenRB 자신)을 제외하고 순서 보존 중복제거한다.
  - classify_serial_error 는 errno 6/2 및 메시지·타입명으로 안정적 코드를 낸다(pyserial 불필요).
  - diagnose_verdict 우선순위: station_off > 미개통 > ping-pong+pong > 포트감지 >
    깨끗한 송신(write-only=TX_OK_NO_RX / 그 외=NO_RF_RX) > 진짜 불안정 > ...
    특히 "에러 0의 깨끗한 송신"은 절대 STATION_USB_UNSTABLE 이 되지 않는다.
  - --dtr/--rts 기본값은 "low"(known-good 수동 테스트와 일치), write_timeout 기본 1.0.

리팩토링 노트 (Refactoring notes):
  새 모드/verdict 추가 시 MODES·diagnose_verdict·리포트 어휘를 함께 갱신할 것. 기본 인자값을
  바꾸면 파서 테스트의 기대값도 갱신.
"""
from tools import hc12_operational_diagnose as diag


# ── 포트 선택 필터 / Station-port selection filter ──
def test_filter_station_ports_excludes_usbmodem_and_dedups() -> None:
    """usbmodem(OpenRB)을 제외하고 순서 보존 중복제거로 스테이션 어댑터만 남기는지 확인.
    Excludes usbmodem (OpenRB) and de-dups in order, leaving only station adapters."""
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


# ── 시리얼/OS 에러 분류 / Serial and OS error classification ──
def test_classify_serial_error_device_not_configured() -> None:
    """errno 6 -> DEVICE_NOT_CONFIGURED (DTR/RTS 미구성 어댑터의 대표 증상).
    errno 6 maps to DEVICE_NOT_CONFIGURED."""
    assert diag.classify_serial_error(OSError(6, "Device not configured")) == "DEVICE_NOT_CONFIGURED"


def test_classify_serial_error_port_not_found() -> None:
    """errno 2 -> PORT_NOT_FOUND (장치 경로 없음).
    errno 2 maps to PORT_NOT_FOUND."""
    assert diag.classify_serial_error(OSError(2, "No such file or directory")) == "PORT_NOT_FOUND"


def test_classify_serial_error_serial_exception_by_name() -> None:
    """pyserial 미설치라도 예외 '타입 이름'만으로 SERIAL_EXCEPTION 분류(로컬 더미 클래스 사용).
    Classifies SERIAL_EXCEPTION by type name only, so it works without pyserial installed."""
    class SerialException(Exception):
        pass

    assert diag.classify_serial_error(SerialException("could not open port")) == "SERIAL_EXCEPTION"


def test_classify_serial_error_generic_oserror() -> None:
    """errno/메시지 단서가 없는 일반 OSError 는 OS_ERROR 로 폴백.
    A generic OSError with no errno/message clue falls back to OS_ERROR."""
    assert diag.classify_serial_error(OSError("weird")) == "OS_ERROR"


# ── 단일 실행 판정 diagnose_verdict / Single-run diagnose_verdict ──
def test_diagnose_verdict_station_off_is_invalid() -> None:
    """station_off=True 는 다른 관측을 무시하고 무효 판정으로 강제.
    station_off overrides observations and forces the INVALID verdict."""
    v = diag.diagnose_verdict(
        mode="read-only", opened=True, serial_error_count=0, total_bytes=0,
        detected_uart_ports=[], pong_rx=0, station_off=True,
    )
    assert v == "TEST_INVALID_STATION_OFF"


def test_diagnose_verdict_ping_pong_link_ok() -> None:
    """ping-pong 모드에서 pong 수신(>0)이면 양방향 링크 OK.
    In ping-pong mode, any received pong means the bidirectional link is OK."""
    v = diag.diagnose_verdict(
        mode="ping-pong", opened=True, serial_error_count=0, total_bytes=40,
        detected_uart_ports=[], pong_rx=3,
    )
    assert v == "HC12_LINK_OK"


def test_diagnose_verdict_sweep_serial3() -> None:
    """감지된 UART 포트에 3 이 있으면 UART_SWEEP_RECEIVED_ON_SERIAL3.
    A detected UART port set containing 3 yields the Serial3 sweep verdict."""
    v = diag.diagnose_verdict(
        mode="read-only", opened=True, serial_error_count=0, total_bytes=80,
        detected_uart_ports=[3], pong_rx=0,
    )
    assert v == "UART_SWEEP_RECEIVED_ON_SERIAL3"


def test_diagnose_verdict_stable_no_rf_bytes() -> None:
    """read-only 로 열렸으나 수신 0 이면 USB 안정+RF 무수신 판정.
    Opened read-only with zero bytes -> USB stable, no RF bytes."""
    v = diag.diagnose_verdict(
        mode="read-only", opened=True, serial_error_count=0, total_bytes=0,
        detected_uart_ports=[], pong_rx=0,
    )
    assert v == "USB_SERIAL_STABLE_NO_RF_BYTES"


def test_diagnose_verdict_unstable_when_never_opened() -> None:
    """포트를 한 번도 못 열었으면(opened=False) USB 브리지 불안정으로 판정.
    Never opening the port (opened=False) yields STATION_USB_UNSTABLE."""
    v = diag.diagnose_verdict(
        mode="stability", opened=False, serial_error_count=3, total_bytes=0,
        detected_uart_ports=[], pong_rx=0,
    )
    assert v == "STATION_USB_UNSTABLE"


# ── CLI 파서 기본값·선택지 / CLI parser defaults and choices ──
def test_dtr_rts_write_timeout_defaults() -> None:
    """인자 없이 파싱 시 --dtr/--rts 기본 "low", write_timeout 기본 1.0(known-good).
    With no args, --dtr/--rts default to "low" and write_timeout to 1.0."""
    args = diag.build_parser().parse_args([])
    assert args.dtr == "low"
    assert args.rts == "low"
    assert args.write_timeout_s == 1.0


def test_dtr_rts_choices_parse() -> None:
    """명시 옵션(--dtr high --rts default --write-timeout-s 2)이 정상 파싱되는지 확인.
    Explicit --dtr/--rts/--write-timeout-s options parse into the expected values."""
    args = diag.build_parser().parse_args(
        ["--dtr", "high", "--rts", "default", "--write-timeout-s", "2"]
    )
    assert args.dtr == "high"
    assert args.rts == "default"
    assert args.write_timeout_s == 2.0


def test_verdict_write_only_tx_ok_no_rx() -> None:
    """write-only 에서 프레임 송신+에러 0+무응답이면 STATION_TX_OK_NO_RX(정상 송신, RF 무응답).
    write-only with frames sent, no errors, nothing back -> STATION_TX_OK_NO_RX."""
    # Manual-equivalent write test: frames sent, no serial errors, nothing back.
    v = diag.diagnose_verdict(
        mode="write-only", opened=True, serial_error_count=0, total_bytes=0,
        detected_uart_ports=[], pong_rx=0, tx_count=5, rx_count=0,
    )
    assert v == "STATION_TX_OK_NO_RX"


def test_verdict_clean_tx_is_not_usb_unstable() -> None:
    """에러 0의 깨끗한 송신은 write-only 가 아닌 모드에서도 절대 USB 불안정이 아니라 NO_RF_RX.
    A clean, error-free TX is never STATION_USB_UNSTABLE; here it maps to HC12_NO_RF_RX."""
    # tx succeeded with zero serial errors -> must never be STATION_USB_UNSTABLE.
    v = diag.diagnose_verdict(
        mode="ping-pong", opened=True, serial_error_count=0, total_bytes=0,
        detected_uart_ports=[], pong_rx=0, tx_count=10, rx_count=0,
    )
    assert v == "HC12_NO_RF_RX"
