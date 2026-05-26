import pytest

from gps_coverage_core.protocol import (
    checksum_xor,
    decode_frame,
    encode_frame,
    manual_command_fields,
)


def test_valid_gps_frame() -> None:
    payload = "fix=1,lat=35.1,lon=129.2"
    body = f"GPS,12,{payload}"
    frame = encode_frame("GPS", 12, payload)

    assert frame == f"@{body}*{checksum_xor(body):02X}\n".encode("ascii")
    assert decode_frame(frame) == {
        "type": "GPS",
        "seq": 12,
        "payload": payload,
        "checksum": f"{checksum_xor(body):02X}",
        "raw": frame.decode("ascii"),
    }


def test_valid_stat_frame() -> None:
    payload = "AUTO_RUNNING,RC_OK,LINK_OK,103"
    body = f"STAT,521,{payload}"
    frame = f"@{body}*{checksum_xor(body):02X}\n"

    assert decode_frame(frame) == {
        "type": "STAT",
        "seq": 521,
        "payload": payload,
        "checksum": f"{checksum_xor(body):02X}",
        "raw": frame,
    }


def test_invalid_checksum() -> None:
    with pytest.raises(ValueError, match="checksum mismatch"):
        decode_frame("@GPS,12,fix=1,lat=35.1,lon=129.2*00\n")


def test_malformed_missing_at() -> None:
    body = "GPS,12,fix=1,lat=35.1,lon=129.2"

    with pytest.raises(ValueError, match="start with @"):
        decode_frame(f"{body}*{checksum_xor(body):02X}\n")


def test_malformed_missing_checksum_separator() -> None:
    with pytest.raises(ValueError, match="checksum separator"):
        decode_frame("@GPS,12,fix=1,lat=35.1,lon=129.2AA\n")


def test_non_integer_seq() -> None:
    body = "GPS,abc,fix=1,lat=35.1,lon=129.2"

    with pytest.raises(ValueError, match="SEQ must be an integer"):
        decode_frame(f"@{body}*{checksum_xor(body):02X}\n")


def test_payload_with_comma_round_trip() -> None:
    payload = "AUTO,0.25,-0.03"
    frame = encode_frame("CMD", 103, payload)
    decoded = decode_frame(frame)

    assert decoded["type"] == "CMD"
    assert decoded["seq"] == 103
    assert decoded["payload"] == payload


def test_manual_command_fields_clamp_and_encode_booleans() -> None:
    assert manual_command_fields(0.7, -0.5, True, False, limit=0.25) == (
        "MANUAL",
        "0.25",
        "-0.25",
        "1",
        "0",
    )
