"""스테이션 진단 도구들이 공유하는 DTR/RTS 안전 시리얼 오픈 헬퍼.
DTR/RTS-safe serial open helper shared by the station-side diagnostic tools.

배경/역할:
    맥 임시 스테이션에서 일부 USB-Serial 브리지는 오픈 시 DTR/RTS 토글에 반응해 리셋되거나
    ``OSError Errno 6 Device not configured`` 를 낸다(pyserial 기본값이 DTR/RTS 를
    assert 하기 때문). 수동 실험 결과 하드웨어 흐름제어를 끄고 DTR/RTS 를 low 로 강제할 때만
    안정적으로 열렸다. ``safe_open_serial`` 은 그 known-good 시퀀스를 재현한다:
    ``rtscts=False``, ``dsrdtr=False``, 유한한 ``write_timeout``, 그리고 오픈 전·후로
    DTR/RTS 를 강제.
    Background: some USB-serial bridges reset or raise ``OSError Errno 6`` when
    DTR/RTS toggle on open. Known-good sequence = HW flow control off + DTR/RTS
    forced low, reproduced by ``safe_open_serial``.

시스템 내 위치:
    ``serial_raw_read.py`` 등 스테이션 측 진단 도구가 ``DTR_RTS_MODES`` 와
    ``safe_open_serial`` 을 import 해서 쓴다. ``--dtr`` / ``--rts`` CLI 인자의 선택지가
    곧 ``DTR_RTS_MODES`` 이다.
    Imported by station-side diagnostics (e.g. ``serial_raw_read.py``);
    ``DTR_RTS_MODES`` supplies the ``--dtr`` / ``--rts`` CLI choices.

핵심 개념·불변식:
    - ``DTR_RTS_MODES`` = ("low", "high", "default"); "default" 는 "라인을 건드리지
      않는다" 를 뜻한다 — high 로 강제하는 것과 다르다 (함정).
    - 제어 라인 설정 실패는 치명적이지 않다: 경고 문자열로 수집될 뿐 예외를 올리지 않는다.
      반대로 진짜 오픈 오류(``ser.open()``)는 그대로 전파되어 호출자가 분류/재접속하게 한다.
    - 순수 헬퍼(``serial_kwargs``, ``control_line_state``, ``mode_should_set``,
      ``mode_level``, ``apply_control_lines``)는 pyserial 없이도 import·단위테스트 가능하다.
      ``serial`` 은 ``safe_open_serial`` 안에서 지연 import 하므로 ``--help`` 는 pyserial
      미설치에서도 동작한다.
    - Invariants: "default" means *do not touch* the line; control-line failures
      warn (never raise) while real ``open()`` errors propagate; pure helpers work
      without pyserial (which is lazily imported inside ``safe_open_serial``).

리팩토링 노트:
    ``safe_open_serial`` 은 부수효과(포트 오픈)를 갖는 유일한 함수다. 나머지는 부수효과가
    없어 테스트하기 쉽다. 오픈 시퀀스(pre-set → open → post-set)의 순서 자체가 하드웨어
    회피책이므로 순서를 바꾸지 말 것.
    Only ``safe_open_serial`` has side effects (opens the port); the rest are pure.
    The pre-set → open → post-set ordering is the hardware workaround — keep it.
"""

from __future__ import annotations

from typing import Any

# Valid CLI values for --dtr / --rts. "default" means: do not touch the line.
DTR_RTS_MODES = ("low", "high", "default")


def mode_should_set(mode: str) -> bool:
    """모드가 명시적 라인 레벨("low"/"high")을 요구하는지 여부. "default"면 False.
    True if the mode requests an explicit line level ("low"/"high")."""
    return mode in ("low", "high")


def mode_level(mode: str) -> bool:
    """모드의 논리 레벨: "low" -> False, "high" -> True.
    Logical line level for the mode: "low" -> False, "high" -> True."""
    return mode == "high"


def serial_kwargs(
    baud: int, *, write_timeout_s: float = 1.0, read_timeout_s: float = 0.2
) -> dict[str, Any]:
    """DTR/RTS 안전 스테이션 포트를 위한 pyserial 생성자 kwargs를 만든다.
    pyserial constructor kwargs for a DTR/RTS-safe station port.

    하드웨어 흐름제어를 꺼서 어댑터가 DTR/RTS 핸드셰이크로 멈추거나 자동 리셋되지 않게 하고,
    쓰기는 ``write_timeout`` 으로 유한하게 제한한다.
    Hardware flow control is disabled so the adapter cannot stall or auto-reset on
    DTR/RTS handshaking, and writes are bounded by ``write_timeout``.
    """
    return {
        "baudrate": baud,
        "timeout": read_timeout_s,
        "write_timeout": write_timeout_s,
        "rtscts": False,
        "dsrdtr": False,
    }


def control_line_state(dtr: str, rts: str) -> dict[str, Any]:
    """상태/요약 출력을 위한 구조화된 제어 라인 상태 딕셔너리.
    Structured control-line state for status/summary printing."""
    return {
        "dtr_mode": dtr,
        "rts_mode": rts,
        "rtscts": False,
        "dsrdtr": False,
    }


def apply_control_lines(ser: Any, dtr: str, rts: str) -> list[str]:
    """열린 포트에서 모드대로 DTR/RTS를 강제한다. 치명적이지 않은 경고 목록을 반환.
    Force DTR/RTS per mode on an open port. Returns non-fatal warnings.

    DTR/RTS 를 설정할 수 없는 어댑터/플랫폼에서는 예외 대신 경고 문자열을 남기므로, 제어 라인
    부재가 진단 자체를 실패시키지 않는다.
    Adapters/platforms that cannot set DTR/RTS produce a warning string instead of
    raising, so a missing control line never fails the diagnosis.
    """
    warnings: list[str] = []
    if mode_should_set(dtr):
        try:
            ser.dtr = mode_level(dtr)
        except (OSError, ValueError, AttributeError, NotImplementedError) as exc:
            warnings.append(f"set_dtr_unsupported:{type(exc).__name__}")
    if mode_should_set(rts):
        try:
            ser.rts = mode_level(rts)
        except (OSError, ValueError, AttributeError, NotImplementedError) as exc:
            warnings.append(f"set_rts_unsupported:{type(exc).__name__}")
    return warnings


def safe_open_serial(
    port: str,
    baud: int,
    *,
    dtr: str = "low",
    rts: str = "low",
    write_timeout_s: float = 1.0,
    read_timeout_s: float = 0.2,
) -> tuple[Any, dict[str, Any]]:
    """스테이션 시리얼 포트를 known-good 방식으로 열고 (serial, state) 튜플을 반환한다.
    Open a station serial port the known-good way and return (serial, state).

    시퀀스: 흐름제어를 끈 닫힌 ``Serial`` 을 만들고, DTR/RTS 를 미리 설정한 뒤(백엔드가
    지원하면 ``open()`` 이 라인을 펄스하지 않게), 열고, 다시 ``--dtr``/``--rts`` 대로
    DTR/RTS 를 강제한다. 진짜 오픈 오류는 그대로 전파되어 호출자가 분류/재접속할 수 있다.
    부수효과: OS 시리얼 포트를 연다(호출자가 close 책임).
    Sequence: build a closed ``Serial`` with HW flow control off, pre-set DTR/RTS
    (so ``open()`` does not pulse them where the backend supports it), open, then
    force DTR/RTS again per ``--dtr``/``--rts``. Real open errors propagate so the
    caller can classify/reconnect. Side effect: opens an OS serial port.
    """
    import serial  # lazy import so --help works without pyserial

    ser = serial.Serial()
    ser.port = port
    for key, value in serial_kwargs(
        baud, write_timeout_s=write_timeout_s, read_timeout_s=read_timeout_s
    ).items():
        setattr(ser, key, value)

    # Best-effort: set the desired DTR/RTS before opening so the open does not emit
    # a reset pulse on adapters that latch the pre-open state.
    if mode_should_set(dtr):
        try:
            ser.dtr = mode_level(dtr)
        except (OSError, ValueError, AttributeError, NotImplementedError):
            pass
    if mode_should_set(rts):
        try:
            ser.rts = mode_level(rts)
        except (OSError, ValueError, AttributeError, NotImplementedError):
            pass

    ser.open()

    warnings = apply_control_lines(ser, dtr, rts)
    state = control_line_state(dtr, rts)
    state["warnings"] = warnings
    return ser, state
