"""tools.station_serial 의 순수 헬퍼 + DTR/RTS 적용 계약 검증 / Contract tests for station_serial.

무엇을/왜 (What/why):
  진단 도구들이 공유하는 안전 시리얼 헬퍼의 계약을 pyserial 없이 고정한다. 부수효과가 있는
  ``safe_open_serial`` 은 직접 열지 않고, 대신 순수 헬퍼(``serial_kwargs``,
  ``control_line_state``, ``mode_should_set``, ``mode_level``)와 가짜/예외 시리얼 객체를 쓴
  ``apply_control_lines`` 만 검증한다.
  Locks the shared safe-serial helpers without pyserial: the pure helpers plus
  ``apply_control_lines`` exercised with fake/raising serial doubles.

고정하는 불변식 (Invariants locked):
  - ``serial_kwargs`` 는 하드웨어 흐름제어를 끄고(rtscts/dsrdtr=False) 유한한 write_timeout 을
    준다(어댑터가 DTR/RTS 로 멈추거나 자동리셋되지 않게 하는 known-good 값).
  - "low"/"high" 는 명시 레벨 설정을 요구하고 "default" 는 라인을 건드리지 않는다(함정).
  - ``apply_control_lines`` 는 DTR/RTS 설정이 불가능한 어댑터에서 예외 대신 경고 문자열을 남긴다
    — 제어선 부재가 진단을 실패시키지 않는다는 안전 불변식.

리팩토링 노트 (Refactoring notes):
  기본 timeout/write_timeout 값이나 경고 문자열 접두어(set_dtr_unsupported/set_rts_unsupported)를
  바꾸면 아래 기대값을 함께 갱신할 것.
"""
from tools import station_serial as ss


def test_serial_kwargs_defaults_disable_flow_control() -> None:
    """기본 serial_kwargs: 흐름제어 off + 정해진 read/write timeout 값 전체를 못 박음.
    Default serial_kwargs disable flow control and pin the read/write timeouts."""
    assert ss.serial_kwargs(9600) == {
        "baudrate": 9600,
        "timeout": 0.2,
        "write_timeout": 1.0,
        "rtscts": False,
        "dsrdtr": False,
    }


def test_serial_kwargs_write_timeout_override() -> None:
    """write_timeout_s 인자가 write_timeout 값을 덮어쓰는지 확인.
    The write_timeout_s argument overrides the write_timeout entry."""
    assert ss.serial_kwargs(9600, write_timeout_s=2.5)["write_timeout"] == 2.5


def test_control_line_state() -> None:
    """control_line_state 는 dtr/rts 모드와 흐름제어 off 를 구조화된 dict 로 요약.
    control_line_state summarizes dtr/rts modes plus flow-control-off as a dict."""
    assert ss.control_line_state("low", "high") == {
        "dtr_mode": "low",
        "rts_mode": "high",
        "rtscts": False,
        "dsrdtr": False,
    }


def test_mode_helpers() -> None:
    """모드 헬퍼: "low"/"high"만 설정 대상(should_set)이고, level 은 high->True/low->False.
    Mode helpers: only low/high are set-worthy; level maps high->True, low->False."""
    assert ss.mode_should_set("low") and ss.mode_should_set("high")
    assert not ss.mode_should_set("default")
    assert ss.mode_level("high") is True
    assert ss.mode_level("low") is False


# ── 테스트 더블: 정상 동작하는 가짜 시리얼 / Test double: a well-behaved fake serial ──
class _FakeSerial:
    """DTR/RTS 설정을 그대로 받아 저장하는 최소 시리얼 스텁 / Minimal serial stub storing dtr/rts."""
    def __init__(self) -> None:
        self.dtr = None
        self.rts = None


def test_apply_control_lines_low_sets_false() -> None:
    """"low" 모드는 DTR/RTS 를 False 로 설정하고 경고를 남기지 않는다.
    "low" mode sets both DTR and RTS to False with no warnings."""
    fake = _FakeSerial()
    warnings = ss.apply_control_lines(fake, "low", "low")
    assert fake.dtr is False and fake.rts is False and warnings == []


def test_apply_control_lines_high_sets_true() -> None:
    """"high" 모드는 DTR/RTS 를 True 로 설정한다.
    "high" mode sets both DTR and RTS to True."""
    fake = _FakeSerial()
    ss.apply_control_lines(fake, "high", "high")
    assert fake.dtr is True and fake.rts is True


def test_apply_control_lines_default_leaves_untouched() -> None:
    """"default" 모드는 라인을 전혀 건드리지 않는다(초기 None 유지) — 함정 방지 검증.
    "default" mode leaves the lines untouched (stays None)."""
    fake = _FakeSerial()
    ss.apply_control_lines(fake, "default", "default")
    assert fake.dtr is None and fake.rts is None


# ── 테스트 더블: DTR/RTS 설정 시 OSError 를 던지는 시리얼 / Test double: serial that raises on set ──
class _RaisingSerial:
    """dtr/rts setter 가 항상 OSError 6 을 던지는 스텁(설정 불가 어댑터 모사).
    Stub whose dtr/rts setters always raise OSError 6 (an adapter that cannot set them)."""
    @property
    def dtr(self):
        return None

    @dtr.setter
    def dtr(self, value):
        raise OSError(6, "Device not configured")

    @property
    def rts(self):
        return None

    @rts.setter
    def rts(self, value):
        raise OSError(6, "Device not configured")


def test_apply_control_lines_warns_instead_of_raising() -> None:
    """설정 실패 시 예외를 삼키고 두 라인 각각의 경고 문자열을 반환하는 안전 불변식 검증.
    Setter failures are swallowed into per-line warnings rather than raising."""
    warnings = ss.apply_control_lines(_RaisingSerial(), "low", "low")
    assert any("set_dtr_unsupported" in w for w in warnings)
    assert any("set_rts_unsupported" in w for w in warnings)
