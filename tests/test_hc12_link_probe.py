"""tools.hc12_link_probe 의 순수 헬퍼 계약 검증 / Pure-helper contract tests for hc12_link_probe.

무엇을/왜 (What/why):
  안전 링크 프로브의 두 순수 함수 ``ping_frame`` 과 ``link_status`` 의 계약을 고정한다.
  포트를 열거나 프레임을 실제로 보내지 않고(부수효과 없는 부분만) 검증하므로 하드웨어/
  pyserial 없이 돌아간다. ``ping_frame`` 은 hc12_operational_diagnose 도 재사용하고
  ``link_status`` 의 임계값은 LINK_OK_MAX_AGE_S(=3초)에 묶여 있어, 이 테스트가 그 두
  불변식을 지킨다.
  Locks the contract of the two pure helpers ``ping_frame`` and ``link_status``; runs
  without hardware since neither touches the serial port.

고정하는 불변식 (Invariants locked):
  - ``ping_frame`` 은 항상 PING 타입 프레임을 만든다(모터 명령 아님 — 안전 불변식).
  - PING payload 는 기본 "PROBE" 이며 인자로 덮어쓸 수 있고, encode/decode 왕복이 보존된다.
  - ``link_status``: 수신 0 이면 NO_RX_YET, 최근 수신이 3초 이내면 LINK_OK, 초과면 LINK_STALE.

리팩토링 노트 (Refactoring notes):
  경계값(3초)을 바꾸면 여기 5.0/0.5 케이스의 기대값도 함께 재검토할 것.
"""
from gps_coverage_core.protocol import decode_frame
from tools import hc12_link_probe


def test_ping_frame_is_valid_ping() -> None:
    """기본 PING 프레임: 타입=PING, seq 보존, payload 기본값 "PROBE".
    Default PING frame decodes to type=PING with the given seq and payload "PROBE"."""
    decoded = decode_frame(hc12_link_probe.ping_frame(7))
    assert decoded["type"] == "PING"
    assert decoded["seq"] == 7
    assert decoded["payload"] == "PROBE"


def test_ping_frame_custom_payload_roundtrips() -> None:
    """사용자 payload 를 넘기면 그대로 인코딩·디코딩되어 왕복 보존됨.
    A custom payload round-trips through encode/decode unchanged."""
    decoded = decode_frame(hc12_link_probe.ping_frame(12, "HELLO"))
    assert decoded["type"] == "PING"
    assert decoded["payload"] == "HELLO"


def test_link_status_transitions() -> None:
    """수신 활동 -> 상태 전이 3종(NO_RX_YET / LINK_OK / LINK_STALE)을 3초 경계로 검증.
    The three link-state transitions across the LINK_OK_MAX_AGE_S (3s) boundary."""
    assert hc12_link_probe.link_status(0, None) == "NO_RX_YET"
    assert hc12_link_probe.link_status(3, 0.5) == "LINK_OK"
    assert hc12_link_probe.link_status(3, 5.0) == "LINK_STALE"
