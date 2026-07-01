"""HC-12 프레임/체크섬 프로토콜 계약 테스트 / Contract test for the HC-12 frame + checksum protocol.

목적/역할:
    `gps_coverage_core.protocol`의 와이어 포맷 `@TYPE,SEQ,PAYLOAD*CS\\n` 를 잠근다.
    encode/decode 왕복, XOR 체크섬 검증, 그리고 손상 프레임에 대한 명확한 ValueError
    (checksum mismatch / start with @ / checksum separator / SEQ must be an integer)를 검증한다.

시스템 내 위치:
    protocol.py는 PC↔로버 무선 링크의 "전송 계층"이고, 이 파일은 그 와이어 계약을 고정한다.
    로버 펌웨어 파서와 반드시 일치해야 하는 양방향 계약이므로, 문법을 바꾸면 여기가 먼저 깨진다.

핵심 개념·불변식:
    - CS는 body(별표 앞부분)를 ASCII로 인코드한 바이트들의 XOR을 대문자 2자리 16진수로 표기.
    - encode와 decode는 **동일한** checksum_xor를 쓰므로 왕복 시 반드시 일치한다.
    - 명령값은 [-limit,1]로 clamp 후 compact 문자열로 포맷(manual_command_fields).

리팩토링 노트:
    예약 문자(@ , * \\n)나 SEQ 규칙, 에러 메시지 문구를 바꾸면 이 테스트의 match 패턴이 깨진다.

Contract test: locks protocol.py's wire format ``@TYPE,SEQ,PAYLOAD*CS\\n`` — encode/decode round-trip,
XOR checksum agreement, and clear ValueErrors for corrupt frames (bad checksum, missing @, missing
``*`` separator, non-integer SEQ). This is the transport-layer contract shared with the rover
firmware parser, so grammar changes surface here first.
"""

import pytest

from gps_coverage_core.protocol import (
    checksum_xor,
    decode_frame,
    encode_frame,
    manual_command_fields,
)


def test_valid_gps_frame() -> None:
    """GPS 프레임 encode 결과와 decode 필드 딕셔너리를 함께 검증. / encode() bytes and decode() field dict both match for a GPS frame."""
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
    """직접 만든 STAT 문자열 프레임이 올바르게 디코드됨을 검증. / A hand-built STAT string frame decodes into the expected fields."""
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
    """체크섬 불일치 프레임은 ValueError로 거부됨을 검증. / A frame with a wrong checksum is rejected with ValueError."""
    with pytest.raises(ValueError, match="checksum mismatch"):
        decode_frame("@GPS,12,fix=1,lat=35.1,lon=129.2*00\n")


def test_malformed_missing_at() -> None:
    """시작 문자 @ 누락 프레임 거부 검증. / A frame missing the leading @ start byte is rejected."""
    body = "GPS,12,fix=1,lat=35.1,lon=129.2"

    with pytest.raises(ValueError, match="start with @"):
        decode_frame(f"{body}*{checksum_xor(body):02X}\n")


def test_malformed_missing_checksum_separator() -> None:
    """체크섬 구분자(*) 누락 프레임 거부 검증. / A frame missing the ``*`` checksum separator is rejected."""
    with pytest.raises(ValueError, match="checksum separator"):
        decode_frame("@GPS,12,fix=1,lat=35.1,lon=129.2AA\n")


def test_non_integer_seq() -> None:
    """SEQ가 정수가 아니면 거부됨을 검증. / A non-integer SEQ field is rejected."""
    body = "GPS,abc,fix=1,lat=35.1,lon=129.2"

    with pytest.raises(ValueError, match="SEQ must be an integer"):
        decode_frame(f"@{body}*{checksum_xor(body):02X}\n")


def test_payload_with_comma_round_trip() -> None:
    """payload 안의 콤마가 필드 분리로 오인되지 않고 왕복 보존됨을 검증. / Commas inside the payload survive encode/decode without being split as fields."""
    payload = "AUTO,0.25,-0.03"
    frame = encode_frame("CMD", 103, payload)
    decoded = decode_frame(frame)

    assert decoded["type"] == "CMD"
    assert decoded["seq"] == 103
    assert decoded["payload"] == payload


def test_manual_command_fields_clamp_and_encode_booleans() -> None:
    """조종값이 limit로 clamp되고 bool이 "1"/"0"으로 인코드됨을 검증. / Steer/throttle clamped to limit and booleans encoded as "1"/"0"."""
    assert manual_command_fields(0.7, -0.5, True, False, limit=0.25) == (
        "MANUAL",
        "0.25",
        "-0.25",
        "1",
        "0",
    )
