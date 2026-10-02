"""Tests du module des moteurs, sans robot : un faux bus imite pypot.

Lancer avec : python3 -m pytest tests   (ou simplement : python3 tests/test_motors.py)

Le verrou du bus (deux programmes à la fois) et le chien de garde ont aussi été vérifiés sur
MobileRobot-1 ; ici, on vérifie la logique de Motors.
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from holorobot import motors  # noqa: E402


class FakeIO:
    """Imite pypot.dynamixel.DxlIO : par défaut, quatre MX-12 dont le 2 en mode articulation."""

    def __init__(self, ids=(1, 2, 4, 8), joint=(2,)):
        self.present = list(ids)
        self.modes = {i: "joint" if i in joint else "wheel" for i in ids}
        self.torque = set()
        self.speeds = {i: 0.0 for i in ids}
        self.commands = []  # consignes de vitesse reçues, dans l'ordre
        self.wheel_mode_calls = []
        self.closed = False

    def scan(self, ids):
        return [i for i in ids if i in self.present]

    def get_model(self, ids):
        return tuple("MX-12" for _ in ids)

    def get_control_mode(self, ids):
        return tuple(self.modes[i] for i in ids)

    def set_wheel_mode(self, ids):
        self.wheel_mode_calls.append(list(ids))
        self.modes.update(dict.fromkeys(ids, "wheel"))

    def enable_torque(self, ids):
        self.torque.update(ids)

    def disable_torque(self, ids):
        self.torque.difference_update(ids)

    def set_moving_speed(self, speeds):
        self.speeds.update(speeds)
        self.commands.append(dict(speeds))

    def get_present_speed(self, ids):
        return tuple(self.speeds[i] for i in ids)

    def get_present_voltage(self, ids):
        return tuple(11.7 for _ in ids)

    def close(self):
        self.closed = True


def fake_bus(**kwargs):
    """Remplace le vrai bus par un FakeIO ; renvoie ce FakeIO pour l'examiner."""
    io = FakeIO(**kwargs)

    class FakeBus:
        def __init__(self, port, baudrate):
            self.io = io

        def close(self):
            io.close()

    motors._Bus = FakeBus
    return io


def test_opening_sets_wheel_mode_only_where_needed():
    io = fake_bus()
    with motors.Motors() as m:
        assert m.ids == [1, 2, 4, 8]
        assert m.models == {1: "MX-12", 2: "MX-12", 4: "MX-12", 8: "MX-12"}
        assert io.wheel_mode_calls == [[2]]
        assert m.is_wheel_mode()
        assert m.get_voltages() == {1: 11.7, 2: 11.7, 4: 11.7, 8: 11.7}


def test_wheel_mode_untouched_on_request():
    io = fake_bus()
    with motors.Motors(wheel_mode=False) as m:
        assert io.wheel_mode_calls == []
        assert not m.is_wheel_mode()


def test_set_speeds_limits_and_enables_torque():
    io = fake_bus()
    with motors.Motors() as m:
        m.set_speeds({1: 1000, 2: -1000, 4: 90})
        assert io.speeds == {1: 720.0, 2: -720.0, 4: 90.0, 8: 0.0}
        assert io.torque == {1, 2, 4}  # le moteur 8, sans consigne, reste libre
        try:
            m.set_speeds({3: 90})
        except ValueError as error:
            assert "3" in str(error)
        else:
            raise AssertionError("un moteur inconnu doit être refusé")


def test_run_repeats_the_command_then_stops():
    io = fake_bus()
    with motors.Motors() as m:
        m.run({1: 90, 8: -90}, duration=0.35)
        assert sum(c == {1: 90.0, 8: -90.0} for c in io.commands) >= 3
        assert io.commands[-1] == {1: 0.0, 2: 0.0, 4: 0.0, 8: 0.0}
        assert m.watchdog_stops == 0


def test_watchdog_stops_the_wheels():
    io = fake_bus()
    with motors.Motors(watchdog=0.2) as m:
        m.set_speeds({1: 90, 4: 90})
        time.sleep(0.1)
        assert m.watchdog_stops == 0 and io.speeds[1] == 90.0
        time.sleep(0.4)
        assert m.watchdog_stops == 1
        assert io.speeds == {1: 0, 2: 0, 4: 0, 8: 0}


def test_close_releases_everything():
    io = fake_bus()
    m = motors.Motors()
    m.set_speeds({1: 90})
    m.close()
    assert io.closed and io.torque == set() and io.speeds[1] == 0
    m.close()  # sans effet la seconde fois
    try:
        m.set_speeds({1: 90})
    except RuntimeError:
        pass
    else:
        raise AssertionError("des moteurs fermés doivent refuser les consignes")


def test_no_motor_on_the_bus():
    io = fake_bus(ids=())
    try:
        motors.Motors()
    except RuntimeError as error:
        assert "alimentés" in str(error)
    else:
        raise AssertionError("sans moteur, Motors() doit échouer")
    assert io.closed


def test_missing_requested_motor():
    io = fake_bus()
    try:
        motors.Motors(ids=[1, 3])
    except RuntimeError as error:
        assert "[3]" in str(error)
    else:
        raise AssertionError("un moteur demandé et absent doit être signalé")
    assert io.closed


if __name__ == "__main__":
    for name, test in list(globals().items()):
        if name.startswith("test_"):
            test()
            print("ok", name)
