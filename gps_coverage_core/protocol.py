from __future__ import annotations


def checksum_xor(text: str) -> int:
    """Return the XOR checksum of an ASCII frame body."""
    checksum = 0
    for value in text.encode("ascii"):
        checksum ^= value
    return checksum


def clamp_unit(value: float, *, limit: float = 1.0) -> float:
    """Clamp a numeric command value to ``[-limit, +limit]``."""
    if limit <= 0:
        raise ValueError("limit must be positive")
    limit = min(limit, 1.0)
    value = float(value)
    if value > limit:
        return limit
    if value < -limit:
        return -limit
    return value


def format_command_float(value: float) -> str:
    """Return a compact decimal string for command payload fields."""
    text = f"{float(value):.3f}".rstrip("0").rstrip(".")
    if text in {"", "-0"}:
        return "0"
    if "." not in text:
        return f"{text}.0"
    return text


def manual_command_fields(
    steer: float,
    throttle: float,
    deadman: bool,
    estop: bool,
    *,
    limit: float = 1.0,
) -> tuple[str, str, str, str, str]:
    """Return payload fields for a ``CMD,MANUAL`` frame."""
    return (
        "MANUAL",
        format_command_float(clamp_unit(steer, limit=limit)),
        format_command_float(clamp_unit(throttle, limit=limit)),
        "1" if deadman else "0",
        "1" if estop else "0",
    )


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


def _validate_frame_type(frame_type: str) -> str:
    if not isinstance(frame_type, str):
        raise ValueError("TYPE must be a string")
    if frame_type == "":
        raise ValueError("TYPE must be non-empty")
    try:
        frame_type.encode("ascii")
    except UnicodeEncodeError as exc:
        raise ValueError("TYPE must be ASCII") from exc
    if any(char not in "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_" for char in frame_type):
        raise ValueError("TYPE must be an uppercase ASCII token")
    return frame_type


def _coerce_payload_text(payload: str, extra_fields: tuple[object, ...]) -> str:
    payload_parts = [str(payload), *(str(field) for field in extra_fields)]
    payload_text = ",".join(payload_parts)
    try:
        payload_text.encode("ascii")
    except UnicodeEncodeError as exc:
        raise ValueError("PAYLOAD must be ASCII") from exc
    if "\r" in payload_text or "\n" in payload_text:
        raise ValueError("PAYLOAD must not contain newline characters")
    return payload_text


def encode_frame(frame_type: str, seq: int, payload: str, *extra_fields: object) -> bytes:
    """Encode a message frame as ``@TYPE,SEQ,PAYLOAD*CS\\n``."""
    if isinstance(seq, bool) or not isinstance(seq, int):
        raise ValueError("SEQ must be an integer")
    frame_type = _validate_frame_type(frame_type)
    payload_text = _coerce_payload_text(payload, extra_fields)
    body = f"{frame_type},{seq},{payload_text}"
    checksum = checksum_xor(body)
    return f"@{body}*{checksum:02X}\n".encode("ascii")


def decode_frame(line: bytes | str) -> dict[str, str | int]:
    """Decode a frame and return the parsed fields plus the raw line."""
    if isinstance(line, bytes):
        try:
            text = line.decode("ascii")
        except UnicodeDecodeError as exc:
            raise ValueError("frame must be ASCII") from exc
    elif isinstance(line, str):
        text = line
    else:
        raise ValueError("line must be bytes or str")

    raw = text
    text = text.rstrip("\r\n")
    if not text.startswith("@"):
        raise ValueError("frame must start with @")
    if "*" not in text:
        raise ValueError("frame must include checksum separator")

    body, checksum_text = text[1:].rsplit("*", 1)
    if checksum_text == "":
        raise ValueError("frame checksum is missing")
    if len(checksum_text) != 2:
        raise ValueError("frame checksum must be two hex digits")

    try:
        expected = int(checksum_text, 16)
    except ValueError as exc:
        raise ValueError("frame checksum must be hexadecimal") from exc

    actual = checksum_xor(body)
    if actual != expected:
        raise ValueError("frame checksum mismatch")

    parts = body.split(",", 2)
    if len(parts) != 3:
        raise ValueError("frame body must contain TYPE, SEQ, and PAYLOAD")

    frame_type, seq_text, payload = parts
    frame_type = _validate_frame_type(frame_type)
    if "\r" in payload or "\n" in payload:
        raise ValueError("PAYLOAD must not contain newline characters")

    try:
        seq = int(seq_text)
    except ValueError as exc:
        raise ValueError("SEQ must be an integer") from exc

    return {
        "type": frame_type,
        "seq": seq,
        "payload": payload,
        "checksum": checksum_text.upper(),
        "raw": raw,
    }
