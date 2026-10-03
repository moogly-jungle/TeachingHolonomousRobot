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
        self.positions = {i: 0.0 for i in ids}
        self.commands = []  # consignes de vitesse reçues, dans l'ordre
        self.events = []  # toutes les écritures, dans l'ordre : vitesses, positions visées, modes
        self.wheel_mode_calls = []
        self.closed = False

    def scan(self, ids):
        return [i for i in ids if i in self.present]

    def ping(self, motor_id):
        return motor_id in self.present

    def change_id(self, new_id_for_id):
        for old, new in new_id_for_id.items():
            self.present[self.present.index(old)] = new
            for table in (self.modes, self.speeds, self.positions):
                table[new] = table.pop(old)

    def get_model(self, ids):
        return tuple("MX-12" for _ in ids)

    def get_control_mode(self, ids):
        return tuple(self.modes[i] for i in ids)

    def set_wheel_mode(self, ids):
        self.wheel_mode_calls.append(list(ids))
        self.events.append(("mode", "wheel", list(ids)))
        self.modes.update(dict.fromkeys(ids, "wheel"))

    def set_joint_mode(self, ids):
        self.events.append(("mode", "joint", list(ids)))
        self.modes.update(dict.fromkeys(ids, "joint"))

    def enable_torque(self, ids):
        self.torque.update(ids)

    def disable_torque(self, ids):
        self.torque.difference_update(ids)

    def set_moving_speed(self, speeds):
        self.speeds.update(speeds)
        self.commands.append(dict(speeds))
        self.events.append(("vitesse", dict(speeds)))

    def set_goal_position(self, goals):
        self.events.append(("position", dict(goals)))
        for i, goal in goals.items():
            if i in self.torque and self.modes[i] == "joint":
                self.positions[i] = goal  # un moteur idéal : il arrive tout de suite

    def get_present_position(self, ids):
        return tuple(self.positions[i] for i in ids)

    def get_present_speed(self, ids):
        return tuple(self.speeds[i] for i in ids)

    def get_present_load(self, ids):
        return tuple(0.0 for _ in ids)

    def get_present_temperature(self, ids):
        return tuple(40.0 for _ in ids)

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


def test_joint_mode_switch_does_not_jump():
    io = fake_bus()
    with motors.Motors() as m:
        io.positions[1] = 30.0
        io.events.clear()
        m.set_joint_mode([1])
        assert m.modes[1] == "joint" and io.modes[1] == "joint"
        # roue arrêtée, puis position visée = position actuelle, puis seulement le changement de mode
        assert io.events == [("vitesse", {1: 0}), ("position", {1: 30.0}), ("mode", "joint", [1]),
                             ("vitesse", {1: motors.JOINT_SPEED})]


def test_move_to_reaches_the_position():
    io = fake_bus()
    with motors.Motors() as m:
        m.set_joint_mode([1])
        assert m.move_to({1: 90}, speed=120) == {1: 90.0}
        assert io.speeds[1] == 120.0 and 1 in io.torque
        assert m.move_to({1: 500}) == {1: 180.0}  # ramené à la course du moteur


def test_modes_are_not_mixed_up():
    io = fake_bus()
    with motors.Motors() as m:
        m.set_joint_mode([1])
        before = list(io.events)
        for call, args in ((m.set_speeds, {1: 0}), (m.set_positions, {4: 90})):
            try:
                call(args)
            except ValueError:
                pass
            else:
                raise AssertionError(f"{call.__name__}{args} aurait dû être refusé")
        assert io.events == before  # rien n'a été écrit : en mode articulation, une vitesse 0 veut dire « au plus vite »


def test_unsafe_values_are_refused():
    fake_bus()
    with motors.Motors() as m:
        m.set_joint_mode([1])
        attempts = {
            "une vitesse nan (calcul raté)": lambda: m.set_speeds({4: float("nan")}),
            "une position infinie": lambda: m.set_positions({1: float("inf")}),
            "une vitesse nulle en mode articulation": lambda: m.set_joint_mode([4], speed=0),
            "une vitesse que le moteur arrondit à 0": lambda: m.set_joint_mode([4], speed=2),
        }
        for label, attempt in attempts.items():
            try:
                attempt()
            except ValueError:
                pass
            else:
                raise AssertionError(f"{label} aurait dû être refusée")


def test_back_to_wheel_mode_does_not_spin():
    io = fake_bus()
    with motors.Motors() as m:
        m.set_joint_mode([1])
        m.move_to({1: 45})
        io.events.clear()
        m.set_wheel_mode([1])
        # se tient là où il est, vitesse nulle, puis mode roue : il repart à l'arrêt
        assert io.events == [("position", {1: 45.0}), ("vitesse", {1: 0}), ("mode", "wheel", [1])]
        assert m.modes[1] == "wheel" and io.speeds[1] == 0


def test_stop_and_watchdog_leave_joint_motors_alone():
    io = fake_bus()
    with motors.Motors(watchdog=0.2) as m:
        m.set_joint_mode([1], speed=90)
        m.move_to({1: 60})
        m.set_speeds({4: 90})
        time.sleep(0.5)
        assert m.watchdog_stops == 1 and io.speeds[4] == 0
        assert io.speeds[1] == 90.0  # jamais 0, qui voudrait dire « au plus vite » en mode articulation
        io.events.clear()
        m.stop()
        assert ("position", {1: 60.0}) in io.events and all(1 not in e[1] for e in io.events if e[0] == "vitesse")


def test_change_id():
    io = fake_bus()
    with motors.Motors() as m:
        m.set_speeds({2: 90})
        m.change_id(2, 3)
        assert m.ids == [1, 3, 4, 8] and io.present == [1, 3, 4, 8]
        assert m.models[3] == "MX-12" and m.modes[3] == "wheel" and 2 not in m.models
        assert io.speeds[3] == 0 and 3 not in io.torque  # arrêté et libéré avant le changement
        for old, new in ((3, 4), (5, 6), (3, 300)):  # pris, inconnu, hors limites
            try:
                m.change_id(old, new)
            except ValueError:
                pass
            else:
                raise AssertionError(f"change_id({old}, {new}) aurait dû être refusé")
        m.set_speeds({3: 45})  # le moteur se commande sous son nouvel identifiant
        assert io.speeds[3] == 45.0


def test_move_to_survives_a_missed_reply():
    io = fake_bus()
    with motors.Motors() as m:
        m.set_joint_mode([1])
        real_read = io.get_present_position
        misses = [1]

        def flaky_read(ids):  # le moteur ne répond pas une fois, comme sur un vrai bus
            if misses:
                misses.pop()
                raise RuntimeError("motors 1 did not respond")
            return real_read(ids)

        io.get_present_position = flaky_read
        assert m.move_to({1: 30}) == {1: 30.0}
        assert m.get_positions([1, 4]) == {1: 30.0, 4: 0.0}


def test_single_ids_and_read_only_helpers():
    io = fake_bus()
    with motors.Motors() as m:
        io.positions[4] = 12.5
        assert m.get_positions(4) == {4: 12.5}  # un seul identifiant, sans liste
        m.set_joint_mode(4)
        assert m.modes[4] == "joint"
    try:
        m.get_positions()
    except RuntimeError:
        pass
    else:
        raise AssertionError("après close(), une lecture doit donner un message clair")
    io = fake_bus()
    assert motors.find_ids() == [1, 2, 4, 8]
    assert motors.voltages() == {1: 11.7, 2: 11.7, 4: 11.7, 8: 11.7}
    assert io.torque == set() and io.closed  # rien n'a été changé, le bus est rendu


def test_change_id_forgets_the_old_command():
    fake_bus()
    with motors.Motors(watchdog=0.2) as m:
        m.set_speeds({2: 90})
        m.change_id(2, 3)
        time.sleep(0.4)
        assert m.watchdog_stops == 0  # pas d'arrêt fantôme : la consigne du moteur renommé est retombée à 0


def test_status_reads_without_changing_anything():
    io = fake_bus()
    io.positions[4] = 30.0
    state = motors.status()
    assert sorted(state) == [1, 2, 4, 8] and state[4]["position"] == 30.0
    assert state[2]["mode"] == "joint" and state[1]["model"] == "MX-12" and state[8]["voltage"] == 11.7
    assert io.wheel_mode_calls == [] and io.torque == set() and io.closed
    assert motors.status(4) == {4: state[4]}


def test_close_can_keep_the_motors_holding():
    io = fake_bus()
    m = motors.Motors()
    m.set_joint_mode([1])
    m.move_to({1: 40})
    m.set_speeds({4: 90})
    before = len(io.events)
    m.close(hold=True)
    assert io.closed and 1 in io.torque and io.speeds[4] == 0  # roue arrêtée, articulation tenue
    assert ("position", {1: 40.0}) in io.events
    assert not any(e[0] == "position" for e in io.events[before:])  # sa consigne n'est pas changée


def test_move_to_waits_until_the_motor_stops():
    io = fake_bus()
    with motors.Motors() as m:
        m.set_joint_mode([1])
        trajectory = [10.0, 25.0, 38.0, 39.4, 39.8, 39.8]  # un vrai moteur ralentit en arrivant

        def gradual_read(ids):
            return tuple(trajectory.pop(0) if trajectory else 39.8 for _ in ids)

        io.get_present_position = gradual_read
        assert m.move_to({1: 40}) == {1: 39.8}  # pas 38.0 : déjà à moins de 3°, mais encore en mouvement


def test_a_lost_ping_does_not_hide_a_motor():
    io = fake_bus()
    real_scan = io.scan
    lost = [4]

    def lossy_scan(ids):  # le premier ping du moteur 4 se perd
        found = real_scan(ids)
        if 4 in found and lost:
            lost.pop()
            found.remove(4)
        return found

    io.scan = lossy_scan
    with motors.Motors(ids=[1, 4], wheel_mode=False) as m:
        assert m.ids == [1, 4]


if __name__ == "__main__":
    for name, test in list(globals().items()):
        if name.startswith("test_"):
            test()
            print("ok", name)
