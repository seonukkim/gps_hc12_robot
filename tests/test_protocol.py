import pytest

from gps_coverage_core.protocol import checksum_xor, decode_frame, encode_frame


def test_valid_cmd_frame() -> None:
    frame = encode_frame("CMD", 103, "AUTO", 0.25, -0.03)
    assert frame.startswith(b"@CMD,103,AUTO,0.25,-0.03*")
    assert decode_frame(frame) == ("CMD", 103, ["AUTO", "0.25", "-0.03"])


def test_valid_gps_frame() -> None:
    frame = encode_frame("GPS", 520, 35.123456, 129.123456, 48.2, 8, 1.4, 1)
    assert decode_frame(frame) == (
        "GPS",
        520,
        ["35.123456", "129.123456", "48.2", "8", "1.4", "1"],
    )


def test_invalid_checksum() -> None:
    with pytest.raises(ValueError, match="checksum mismatch"):
        decode_frame("@CMD,103,STOP,0,0*00\n")


def test_missing_at() -> None:
    body = "CMD,103,STOP,0,0"
    checksum = checksum_xor(body)
    with pytest.raises(ValueError, match="start with @"):
        decode_frame(f"{body}*{checksum:02X}\n")


def test_missing_checksum() -> None:
    with pytest.raises(ValueError, match="checksum separator"):
        decode_frame("@CMD,103,STOP,0,0\n")


def test_non_integer_sequence() -> None:
    body = "CMD,ABC,STOP,0,0"
    checksum = checksum_xor(body)
    with pytest.raises(ValueError, match="SEQ must be an integer"):
        decode_frame(f"@{body}*{checksum:02X}\n")
