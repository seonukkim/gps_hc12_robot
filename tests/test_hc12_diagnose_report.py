"""tools.hc12_diagnose_report 의 판정 우선순위·스윕 파싱·프레임 호환성 검증.
Verdict-priority, sweep-parsing, and frame-compatibility tests for hc12_diagnose_report.

무엇을/왜 (What/why):
  리포트 취합기의 순수 로직을 로그 파일 없이 고정한다: ``report_verdict`` 의 우선순위 분기,
  ``next_action`` 의 전수 커버리지, ``parse_sweep_counters`` 의 최신값 취득, 그리고 스윕/
  STATION/PING 프레임이 여전히 공통 ``@TYPE,SEQ,PAYLOAD*CS`` 포맷을 공유한다는 상호운용성.
  Locks the report collector's pure logic without log files: verdict priority,
  next_action coverage, sweep counter parsing, and cross-tool frame compatibility.

고정하는 불변식 (Invariants locked):
  - 판정 우선순위: station_off > link_ok > 포트별 수신(Serial3>1>2) > openrb_rx >
    STATION_TX_OK_NO_RX > usb_unstable > stable_no_rf > 기본. 분기 순서가 곧 우선순위다.
  - "깨끗한 송신(tx>0, 에러 0, 수신 0)"은 USB 불안정이 아니라 STATION_TX_OK_NO_RX 로 이긴다.
  - VERDICTS 의 모든 문자열은 next_action 매핑을 가진다(빈 안내 금지).
  - parse_sweep_counters 는 필드가 여러 번 나오면 항상 마지막(최신) 값을 취한다.

리팩토링 노트 (Refactoring notes):
  새 verdict 추가 시 VERDICTS·report_verdict·next_action 을 함께 갱신하면 여기 커버리지
  테스트가 자동으로 그 정합성을 지켜준다. 프레임 포맷 변경 시 마지막 호환성 테스트도 확인.
"""
from gps_coverage_core.protocol import decode_frame, encode_frame
from tools import hc12_diagnose_report as report
from tools.hc12_link_probe import ping_frame
from tools.serial_raw_read import detect_uart_ports


def test_report_verdict_station_off() -> None:
    """station_off 는 다른 증거를 모두 무시하고 무효(TEST_INVALID) 판정으로 강제됨.
    station_off overrides all other evidence and forces the INVALID verdict."""
    assert report.report_verdict({"station_off": True}) == "TEST_INVALID_STATION_OFF"


def test_report_verdict_link_ok_beats_sweep() -> None:
    """양방향 링크(link_ok)가 포트 스윕 감지보다 우선순위가 높음을 확인.
    A confirmed link (link_ok) outranks a UART-sweep detection."""
    ev = {"link_ok": True, "detected_uart_ports": [3]}
    assert report.report_verdict(ev) == "HC12_LINK_OK"


def test_report_verdict_sweep_serial3() -> None:
    """Serial3 에서 스윕 프레임 감지 -> UART_SWEEP_RECEIVED_ON_SERIAL3.
    A sweep frame seen on Serial3 yields the Serial3 verdict."""
    assert report.report_verdict({"detected_uart_ports": [3]}) == "UART_SWEEP_RECEIVED_ON_SERIAL3"


def test_report_verdict_openrb_rx() -> None:
    """포트 감지는 없지만 OpenRB 측 수신 증거가 있으면 STATION_TO_OPENRB_RX_DETECTED.
    No port detection but OpenRB-side RX evidence -> station-to-OpenRB verdict."""
    ev = {"detected_uart_ports": [], "openrb_rx_detected": True}
    assert report.report_verdict(ev) == "STATION_TO_OPENRB_RX_DETECTED"


def test_report_verdict_stable_no_rf() -> None:
    """포트는 열렸으나 수신 0 바이트면 USB 안정+RF 무수신(STABLE_NO_RF_BYTES).
    Opened but zero bytes received -> USB stable, no RF bytes."""
    ev = {"station_opened": True, "station_total_bytes": 0}
    assert report.report_verdict(ev) == "USB_SERIAL_STABLE_NO_RF_BYTES"


def test_report_verdict_no_rf_default() -> None:
    """송신 흔적 없이 바이트만 수신된 잔여 케이스는 기본 판정 HC12_NO_RF_RX 로 폴백.
    Bytes received with no clean-TX evidence falls through to the default HC12_NO_RF_RX."""
    ev = {"station_opened": True, "station_total_bytes": 5}
    assert report.report_verdict(ev) == "HC12_NO_RF_RX"


def test_next_action_covers_all_verdicts() -> None:
    """VERDICTS 의 모든 판정에 비어있지 않은 next_action 안내가 존재하는지 전수 검증.
    Every verdict in VERDICTS maps to non-empty next-action guidance."""
    for verdict in report.VERDICTS:
        assert report.next_action(verdict)  # non-empty guidance


def test_parse_sweep_counters_takes_latest_and_rx_first() -> None:
    """스윕 로그에서 각 UART tx/rx 는 마지막 값을, RX_FIRST_DETECTED 는 포트 집합을 취함.
    Sweep parsing takes the latest tx/rx per UART and collects RX_FIRST_DETECTED ports."""
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
    """스윕·STATION·PING 프레임이 모두 공통 @TYPE,SEQ,PAYLOAD*CS 포맷과 호환되는지 확인.
    Sweep, STATION, and PING frames all stay compatible with the shared frame format."""
    # Sweep frame, STATION frame, and PING frame all use @TYPE,SEQ,PAYLOAD*CS.
    sweep = encode_frame("UART", 3, "TX_TEST")
    assert detect_uart_ports(sweep.decode()) == [3]
    decoded = decode_frame(encode_frame("STATION", 7, "TX_TEST"))
    assert decoded["type"] == "STATION" and decoded["seq"] == 7
    assert decode_frame(ping_frame(2))["type"] == "PING"


def test_report_verdict_station_tx_ok_no_rx() -> None:
    """깨끗한 송신(tx>0, 에러 0, 수신 0)은 STATION_TX_OK_NO_RX 로 판정됨.
    A clean write (tx>0, no errors, no RX) is judged STATION_TX_OK_NO_RX."""
    ev = {
        "station_tx_count": 5,
        "serial_error_count": 0,
        "station_total_bytes": 0,
        "station_opened": True,
    }
    assert report.report_verdict(ev) == "STATION_TX_OK_NO_RX"


def test_report_verdict_clean_tx_not_unstable() -> None:
    """usb_unstable 힌트가 있어도 깨끗한 송신이 우선 -> USB 불안정으로 오판하지 않는 불변식.
    Even with a usb_unstable hint, a clean TX wins and is never mislabeled unstable."""
    ev = {
        "station_tx_count": 5,
        "serial_error_count": 0,
        "station_total_bytes": 0,
        "station_usb_unstable": True,  # even if heuristically set, clean tx wins
        "station_opened": True,
    }
    assert report.report_verdict(ev) == "STATION_TX_OK_NO_RX"
