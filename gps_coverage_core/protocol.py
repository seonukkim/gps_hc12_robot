"""PC↔로버 명령 프레임/체크섬 프로토콜 / PC<->rover command frame + checksum protocol.

목적/역할:
    PC(스테이션)와 로버가 HC-12 무선 링크로 주고받는 ASCII 라인 프레임을 인코드/디코드한다.
    와이어 포맷은 `@TYPE,SEQ,PAYLOAD*CS\\n` 한 줄이며, CS는 body(별표 앞부분)의 XOR 체크섬을 대문자
    2자리 16진수로 표기한 것이다. 잡음이 섞이는 무선에서 프레임 손상을 잡아내는 것이 목적이다.

시스템 내 위치:
    이 패키지 안에서 가장 널리 쓰이는 모듈: `tools/hc12_link_probe.py`,
    `tools/hc12_operational_diagnose.py`, `archive/tools/station_controller.py`, `nmea_replay.py`,
    `gps_logger.py`, 그리고 다수 테스트가 `encode_frame`/`decode_frame`을 import 한다. GPS 프레임의
    payload는 telemetry.GPSTelemetry가, 명령 프레임 필드는 이 파일의 manual_command_fields가 만든다.
    파이프라인상 "전송 계층"(직렬화/역직렬화 + 무결성 검사)에 해당한다.

핵심 개념·불변식(와이어 계약 — 로버 펌웨어와 반드시 일치):
    - 시작 문자 `@`, 필드 구분자 `,`, 체크섬 구분자 `*`, 종료 `\\n`. 이 문자들은 예약(reserved)이라
      TYPE/토큰 안에 올 수 없다(_validate_token이 강제).
    - TYPE는 대문자/숫자/밑줄만 허용. SEQ는 정수(단, bool은 int의 하위형이라 명시적으로 거부).
    - 체크섬은 body를 ASCII로 인코드한 바이트들의 XOR. encode와 decode가 **같은** checksum_xor로
      계산하므로 왕복(round-trip) 시 반드시 일치해야 한다.
    - 명령값(steer/throttle)은 [-1,1]로 clamp 후 compact 십진 문자열로 포맷("0", "1.0", "-0.5" 등).

리팩토링 노트:
    프레임 문법이나 예약 문자를 바꾸면 로버 펌웨어 파서도 함께 바꿔야 한다(양방향 계약). 오류는 모두
    ValueError로 통일되어 있어 호출측이 손상 프레임을 일관되게 처리할 수 있다. `_`로 시작하는 함수는
    내부 검증 헬퍼다.

Purpose: encode/decode the ASCII line frames exchanged over the HC-12 radio between the PC station
and the rover. Wire format is one line ``@TYPE,SEQ,PAYLOAD*CS\\n`` where CS is the XOR checksum of the
body (everything before ``*``) as two uppercase hex digits — this catches frame corruption on a noisy
link. Most-imported module here: used by hc12_link_probe, hc12_operational_diagnose, station tools,
nmea_replay, gps_logger and many tests. This is the transport layer (serialize/deserialize +
integrity). Wire contract (must match rover firmware): ``@`` start, ``,`` field sep, ``*`` checksum
sep, ``\\n`` end are reserved and forbidden inside TYPE/tokens; TYPE is an uppercase/digit/underscore
token; SEQ is int (bool explicitly rejected since ``bool`` subclasses ``int``); encode and decode use
the same ``checksum_xor`` so round-trips must agree. Command values are clamped to [-1,1] and
formatted compactly. Changing the grammar/reserved chars means changing the firmware parser too. All
errors are raised as ``ValueError``; leading-underscore functions are internal validation helpers.
"""

from __future__ import annotations


# ── 체크섬 / Checksum ──
def checksum_xor(text: str) -> int:
    """ASCII 프레임 body의 XOR 체크섬 계산. / Return the XOR checksum of an ASCII frame body.

    인자/Args: text=별표(*) 앞의 body 문자열. 반환/Returns: 0..255 정수(모든 바이트 XOR).
    encode/decode가 공유하는 무결성 기준. / Shared integrity basis for encode and decode.
    """
    checksum = 0
    for value in text.encode("ascii"):
        checksum ^= value
    return checksum


# ── 명령값 정규화·포맷 / Command value clamping & formatting ──
def clamp_unit(value: float, *, limit: float = 1.0) -> float:
    """명령값을 ``[-limit, +limit]``로 제한(상한은 1.0로 캡). / Clamp a command value to [-limit, +limit].

    안전상 limit은 1.0을 넘을 수 없다(min으로 캡). limit<=0이면 ValueError.
    For safety ``limit`` is capped at 1.0; ``limit<=0`` raises ValueError.
    """
    if limit <= 0:
        raise ValueError("limit must be positive")
    # 호출측이 1.0보다 큰 limit을 줘도 물리적으로 허용치를 넘지 못하게 강제 상한.
    # / Hard cap: even if the caller passes >1.0, never exceed full scale.
    limit = min(limit, 1.0)
    value = float(value)
    if value > limit:
        return limit
    if value < -limit:
        return -limit
    return value


def format_command_float(value: float) -> str:
    """명령 필드용 compact 십진 문자열 생성. / Return a compact decimal string for command payload fields.

    소수 3자리로 반올림 후 잉여 0/소수점을 제거하되, 정수는 "1.0"처럼 소수점을 남긴다. "-0"과 빈
    문자열은 "0"으로 정규화(음의 0 방지). / Rounds to 3 decimals, trims trailing zeros, keeps a ".0"
    on integers, and normalizes "-0"/"" to "0" (no negative zero).
    """
    text = f"{float(value):.3f}".rstrip("0").rstrip(".")
    # 빈 문자열이나 "-0"은 파서 혼란을 막기 위해 표준 "0"으로 통일. / Normalize ""/"-0" to canonical "0".
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
    """``CMD,MANUAL`` 프레임의 payload 필드 튜플 생성. / Return payload fields for a ``CMD,MANUAL`` frame.

    인자/Args: steer/throttle=[-1,1] 조향·구동 명령, deadman/estop=안전 스위치 상태.
    반환/Returns: ("MANUAL", steer, throttle, deadman, estop) 문자열 5-튜플. steer/throttle은
    clamp 후 포맷되고, 불리언은 "1"/"0"으로 인코드된다.
    """
    return (
        "MANUAL",
        format_command_float(clamp_unit(steer, limit=limit)),
        format_command_float(clamp_unit(throttle, limit=limit)),
        "1" if deadman else "0",
        "1" if estop else "0",
    )


# ── 내부 검증 헬퍼 / Internal validation helpers ──
def _validate_token(token: str, label: str, *, allow_empty: bool = False) -> str:
    """예약 문자·비ASCII를 막는 범용 토큰 검증. / Generic token check rejecting reserved chars / non-ASCII.

    label은 오류 메시지용 필드 이름. 통과 시 token을 그대로 반환. / ``label`` names the field in errors.
    """
    if not isinstance(token, str):
        raise ValueError(f"{label} must be a string")
    if not allow_empty and token == "":
        raise ValueError(f"{label} cannot be empty")
    # 프레임 문법 문자(@ * , CR LF)가 토큰에 섞이면 파싱이 깨지므로 원천 차단.
    # / Frame-grammar chars (@ * , CR LF) inside a token would break parsing — reject up front.
    if any(char in token for char in "@*,\r\n"):
        raise ValueError(f"{label} contains reserved characters")
    try:
        token.encode("ascii")
    except UnicodeEncodeError as exc:
        raise ValueError(f"{label} must be ASCII") from exc
    return token


def _validate_frame_type(frame_type: str) -> str:
    """TYPE 필드 검증: 비어있지 않은 대문자 ASCII 토큰. / Validate TYPE: non-empty uppercase ASCII token.

    허용 문자는 A-Z, 0-9, `_`. encode와 decode 양쪽에서 호출되어 대칭성을 보장한다.
    Allowed: A-Z, 0-9, underscore. Called by both encode and decode for symmetry.
    """
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
    """payload와 가변 필드를 쉼표 결합한 ASCII 문자열로 정규화. / Join payload + extra fields into one ASCII, comma-separated string.

    ASCII가 아니거나 개행이 포함되면 ValueError. / Raises ValueError on non-ASCII or embedded newlines.
    """
    # 각 필드를 str로 강제해 숫자/불리언 등 어떤 타입이 와도 프레임에 안전히 실을 수 있게 한다.
    # / Coerce every field to str so numbers/bools/etc. can all ride in the frame safely.
    payload_parts = [str(payload), *(str(field) for field in extra_fields)]
    payload_text = ",".join(payload_parts)
    try:
        payload_text.encode("ascii")
    except UnicodeEncodeError as exc:
        raise ValueError("PAYLOAD must be ASCII") from exc
    if "\r" in payload_text or "\n" in payload_text:
        raise ValueError("PAYLOAD must not contain newline characters")
    return payload_text


# ── 프레임 인코드/디코드 / Frame encode & decode ──
def encode_frame(frame_type: str, seq: int, payload: str, *extra_fields: object) -> bytes:
    """메시지 프레임을 ``@TYPE,SEQ,PAYLOAD*CS\\n`` 바이트로 인코드. / Encode a frame as ``@TYPE,SEQ,PAYLOAD*CS\\n`` bytes.

    인자/Args: frame_type=대문자 토큰, seq=정수 시퀀스, payload와 *extra_fields=쉼표로 이어붙일 값들.
    반환/Returns: 개행으로 끝나는 ASCII bytes(그대로 링크에 write 가능). 잘못된 입력은 ValueError.
    ``decode_frame``과 왕복 대칭. / Round-trips with ``decode_frame``.
    """
    # bool은 int의 하위형이라 True/False가 SEQ로 새어들 수 있어 명시적으로 먼저 거부한다.
    # / bool subclasses int, so True/False could leak in as SEQ — reject it explicitly first.
    if isinstance(seq, bool) or not isinstance(seq, int):
        raise ValueError("SEQ must be an integer")
    frame_type = _validate_frame_type(frame_type)
    payload_text = _coerce_payload_text(payload, extra_fields)
    body = f"{frame_type},{seq},{payload_text}"
    # 체크섬은 body(별표 앞)만 대상으로 계산 — decode도 동일 범위를 검사해야 일치한다.
    # / Checksum covers only the body (before ``*``); decode must check the same span to match.
    checksum = checksum_xor(body)
    return f"@{body}*{checksum:02X}\n".encode("ascii")


def decode_frame(line: bytes | str) -> dict[str, str | int]:
    """프레임을 검증·파싱해 필드 dict 반환(원문 포함). / Decode + validate a frame into a field dict (raw included).

    인자/Args: line=수신한 프레임(bytes 또는 str). 반환/Returns: {"type","seq","payload","checksum","raw"}.
    체크섬 불일치·문법 위반·비ASCII 등 모든 손상은 ValueError. 부수효과 없음.
    Any corruption (checksum mismatch, grammar violation, non-ASCII) raises ValueError; no side effects.
    """
    if isinstance(line, bytes):
        try:
            text = line.decode("ascii")
        except UnicodeDecodeError as exc:
            raise ValueError("frame must be ASCII") from exc
    elif isinstance(line, str):
        text = line
    else:
        raise ValueError("line must be bytes or str")

    # 원문(raw)은 개행 제거 전에 보존 — 디버깅/재현용으로 결과 dict에 그대로 실어 보낸다.
    # / Preserve the raw line before stripping newlines; passed back in the dict for debugging/replay.
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

    # 재계산한 체크섬이 프레임에 실린 값과 다르면 링크 손상 — 여기서 거부해 상위로 오염 전파 차단.
    # / Recomputed checksum must equal the transmitted one; mismatch = link corruption, reject here.
    actual = checksum_xor(body)
    if actual != expected:
        raise ValueError("frame checksum mismatch")

    # maxsplit=2 라서 TYPE,SEQ 뒤의 나머지는 통째로 PAYLOAD가 된다 → payload 내부의 쉼표 허용.
    # / maxsplit=2 keeps everything after TYPE,SEQ as one PAYLOAD, so commas inside payload survive.
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
