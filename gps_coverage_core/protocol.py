from __future__ import annotations


def checksum_xor(text: str) -> int:
    """Return the XOR checksum of an ASCII frame body."""
    checksum = 0
    for value in text.encode("ascii"):
        checksum ^= value
    return checksum


def _validate_token(token: str, label: str, *, allow_empty: bool = False) -> str:
    if not isinstance(token, str):
        raise ValueError(f"{label} must be a string")
    if not allow_empty and token == "":
        raise ValueError(f"{label} cannot be empty")
    if any(char in token for char in "@*,\r\n"):
        raise ValueError(f"{label} contains reserved characters")
    try:
        token.encode("ascii")
    except UnicodeEncodeError as exc:
        raise ValueError(f"{label} must be ASCII") from exc
    return token


def encode_frame(msg_type: str, seq: int, *fields: object) -> bytes:
    """Encode a message frame as ``@TYPE,SEQ,PAYLOAD*CS\\n``."""
    if isinstance(seq, bool) or not isinstance(seq, int):
        raise ValueError("SEQ must be an integer")
    msg_type = _validate_token(msg_type, "msg_type")
    payload = [_validate_token(str(field), f"field[{index}]") for index, field in enumerate(fields)]
    body = ",".join([msg_type, str(seq), *payload])
    checksum = checksum_xor(body)
    return f"@{body}*{checksum:02X}\n".encode("ascii")


def decode_frame(line: bytes | str) -> tuple[str, int, list[str]]:
    """Decode a frame and return ``(msg_type, seq, payload_fields)``."""
    if isinstance(line, bytes):
        try:
            text = line.decode("ascii")
        except UnicodeDecodeError as exc:
            raise ValueError("frame must be ASCII") from exc
    elif isinstance(line, str):
        text = line
    else:
        raise ValueError("line must be bytes or str")

    text = text.strip()
    if not text.startswith("@"):
        raise ValueError("frame must start with @")
    if "*" not in text:
        raise ValueError("frame must include checksum separator")

    body, checksum_text = text[1:].rsplit("*", 1)
    if not checksum_text:
        raise ValueError("frame checksum is missing")

    try:
        expected = int(checksum_text, 16)
    except ValueError as exc:
        raise ValueError("frame checksum must be hex") from exc

    actual = checksum_xor(body)
    if actual != expected:
        raise ValueError("frame checksum mismatch")

    parts = body.split(",")
    if len(parts) < 2:
        raise ValueError("frame body must contain TYPE and SEQ")

    msg_type = parts[0]
    if not msg_type:
        raise ValueError("TYPE is missing")
    _validate_token(msg_type, "msg_type")

    try:
        seq = int(parts[1])
    except ValueError as exc:
        raise ValueError("SEQ must be an integer") from exc

    return msg_type, seq, parts[2:]
