from tools import station_serial as ss


def test_serial_kwargs_defaults_disable_flow_control() -> None:
    assert ss.serial_kwargs(9600) == {
        "baudrate": 9600,
        "timeout": 0.2,
        "write_timeout": 1.0,
        "rtscts": False,
        "dsrdtr": False,
    }


def test_serial_kwargs_write_timeout_override() -> None:
    assert ss.serial_kwargs(9600, write_timeout_s=2.5)["write_timeout"] == 2.5


def test_control_line_state() -> None:
    assert ss.control_line_state("low", "high") == {
        "dtr_mode": "low",
        "rts_mode": "high",
        "rtscts": False,
        "dsrdtr": False,
    }


def test_mode_helpers() -> None:
    assert ss.mode_should_set("low") and ss.mode_should_set("high")
    assert not ss.mode_should_set("default")
    assert ss.mode_level("high") is True
    assert ss.mode_level("low") is False


class _FakeSerial:
    def __init__(self) -> None:
        self.dtr = None
        self.rts = None


def test_apply_control_lines_low_sets_false() -> None:
    fake = _FakeSerial()
    warnings = ss.apply_control_lines(fake, "low", "low")
    assert fake.dtr is False and fake.rts is False and warnings == []


def test_apply_control_lines_high_sets_true() -> None:
    fake = _FakeSerial()
    ss.apply_control_lines(fake, "high", "high")
    assert fake.dtr is True and fake.rts is True


def test_apply_control_lines_default_leaves_untouched() -> None:
    fake = _FakeSerial()
    ss.apply_control_lines(fake, "default", "default")
    assert fake.dtr is None and fake.rts is None


class _RaisingSerial:
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
    warnings = ss.apply_control_lines(_RaisingSerial(), "low", "low")
    assert any("set_dtr_unsupported" in w for w in warnings)
    assert any("set_rts_unsupported" in w for w in warnings)
