from gps_coverage_core.protocol import decode_frame
from tools import hc12_link_probe


def test_ping_frame_is_valid_ping() -> None:
    decoded = decode_frame(hc12_link_probe.ping_frame(7))
    assert decoded["type"] == "PING"
    assert decoded["seq"] == 7
    assert decoded["payload"] == "PROBE"


def test_ping_frame_custom_payload_roundtrips() -> None:
    decoded = decode_frame(hc12_link_probe.ping_frame(12, "HELLO"))
    assert decoded["type"] == "PING"
    assert decoded["payload"] == "HELLO"


def test_link_status_transitions() -> None:
    assert hc12_link_probe.link_status(0, None) == "NO_RX_YET"
    assert hc12_link_probe.link_status(3, 0.5) == "LINK_OK"
    assert hc12_link_probe.link_status(3, 5.0) == "LINK_STALE"
