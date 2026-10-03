"""Moteurs des roues : Dynamixel MX-12W, sur le bus série du robot.

Exemple :

    from holorobot.motors import Motors

    with Motors() as motors:               # ouvre le bus, trouve les moteurs, les met en mode roue
        print(motors.ids)                  # par exemple [1, 2, 4, 8]
        motors.run({4: 90}, duration=2)    # le moteur 4 fait tourner sa roue à 90 °/s pendant 2 s

Conventions :
- un moteur est désigné par son identifiant (ID) sur le bus ;
- les vitesses sont en degrés par seconde (°/s) : c'est la vitesse de rotation de la roue ;
- une vitesse positive fait tourner la roue dans le sens inverse des aiguilles d'une montre, vue
  du côté de la roue. Selon le côté où le moteur est monté, cela fait avancer ou reculer le robot.

Deux modes :
- mode roue, par défaut : le moteur tourne sans fin à la vitesse demandée (set_speeds, run) ;
- mode articulation : le moteur va à une position, en degrés de -180 à 180, et s'y tient
  (set_positions, move_to). C'est le mode d'un bras robotique, ou d'une tourelle pan-tilt qui
  oriente la caméra.

Sécurité : un chien de garde arrête toutes les roues si aucune consigne n'arrive pendant
`watchdog` secondes (0,5 s par défaut). Une consigne de set_speeds() ne vaut donc que 0,5 s : pour
tourner plus longtemps, utilisez run(), ou renvoyez la consigne régulièrement dans une boucle. Un
moteur en mode articulation n'en a pas besoin : arrivé à sa position, il s'y tient. Le chien de
garde vit dans votre programme : si celui-ci est tué brutalement, les roues gardent leur dernière
vitesse. Il faut alors couper l'alimentation.

Ce module s'appuie sur pypot (pypot.dynamixel.DxlIO).
"""
import atexit
import fcntl
import os
import threading
import time
import weakref

import numpy as np

PORT = "/dev/serial0"
BAUDRATE = 57600
MAX_SPEED = 720.0  # °/s, soit 2 tours de roue par seconde : limite de sécurité, réglable
JOINT_SPEED = 60.0  # °/s, vitesse des mouvements en mode articulation si on n'en donne pas
# En mode articulation, une vitesse nulle veut dire « aussi vite que possible ». Le MX-12W compte sa
# vitesse par pas de 5,5 °/s et arrondit à 0 ce qui est plus lent : on refuse donc moins de 6 °/s.
MIN_JOINT_SPEED = 6.0  # °/s
SCAN_IDS = range(21)  # identifiants cherchés par défaut : 0 à 20
# Le mode et l'identifiant sont écrits dans la mémoire permanente du moteur (EEPROM) : pendant
# l'écriture, il ignore ce qu'on lui envoie. On lui laisse ce temps avant de lui parler à nouveau.
EEPROM_DELAY = 0.1  # s

_open_motors = weakref.WeakSet()


def _read(getter, ids):
    """Lecture sur le bus : {identifiant: valeur}. Une réponse se perd de temps en temps sur un bus
    Dynamixel : on relit alors, jusqu'à trois essais."""
    for attempt in range(3):
        try:
            return dict(zip(ids, getter(ids)))
        except Exception:
            if attempt == 2:
                raise
            time.sleep(0.01)


def find_ids(ids=SCAN_IDS, port=PORT, baudrate=BAUDRATE):
    """Identifiants des moteurs qui répondent sur le bus, sans rien faire bouger."""
    bus = _Bus(port, baudrate)
    try:
        return bus.io.scan(list(ids))
    finally:
        bus.close()


def voltages(ids=None, port=PORT, baudrate=BAUDRATE):
    """Tension lue par chaque moteur, en V : celle de la batterie. Ne change rien aux moteurs.

    Sans `ids`, interroge tous les moteurs présents. Renvoie un dictionnaire {identifiant: tension}.
    """
    bus = _Bus(port, baudrate)
    try:
        ids = bus.io.scan(list(SCAN_IDS)) if ids is None else list(ids)
        return _read(bus.io.get_present_voltage, ids)
    finally:
        bus.close()


def status(ids=None, port=PORT, baudrate=BAUDRATE):
    """État des moteurs, lu sans rien leur changer : {identifiant: {nom: valeur}}.

    Pour chaque moteur : "model", "mode" (« wheel » ou « joint »), "position" (degrés), "speed"
    (°/s), "load" (% du couple maximal), "temperature" (°C) et "voltage" (V). Sans `ids`, interroge
    tous les moteurs présents.
    """
    bus = _Bus(port, baudrate)
    try:
        io = bus.io
        ids = io.scan(list(SCAN_IDS)) if ids is None else ([ids] if isinstance(ids, int) else list(ids))
        if not ids:
            return {}
        columns = {
            "model": _read(io.get_model, ids),
            "mode": _read(io.get_control_mode, ids),
            "position": _read(io.get_present_position, ids),
            "speed": _read(io.get_present_speed, ids),
            "load": _read(io.get_present_load, ids),
            "temperature": _read(io.get_present_temperature, ids),
            "voltage": _read(io.get_present_voltage, ids),
        }
        return {i: {name: values[i] for name, values in columns.items()} for i in ids}
    finally:
        bus.close()


class _Bus:
    """Le bus des moteurs, réservé à un seul utilisateur à la fois, tous programmes confondus.

    pypot ne fait qu'afficher un avertissement si le port est déjà ouvert, alors que deux
    programmes qui parlent en même temps sur le bus brouillent les échanges. On verrouille donc le
    port (flock) : le verrou est rendu à la fermeture, ou par le système si le programme s'arrête.
    """

    def __init__(self, port, baudrate):
        try:
            self._lock = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError as error:
            raise RuntimeError(f"Impossible d'ouvrir le bus des moteurs sur {port} : {error}") from error
        try:
            fcntl.flock(self._lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            os.close(self._lock)
            raise RuntimeError(
                f"Le bus des moteurs ({port}) est déjà utilisé, par ce programme ou par un autre "
                "(un autre carnet ?) : fermez-le (motors.close()) ou redémarrez le noyau qui "
                "l'utilise.") from None
        try:
            import pypot.dynamixel as dxl

            self.io = dxl.DxlIO(port, baudrate=baudrate)
        except BaseException as error:  # y compris une interruption : le verrou doit être rendu
            os.close(self._lock)
            if isinstance(error, Exception):
                raise RuntimeError(f"Impossible d'ouvrir le bus des moteurs sur {port} : {error}") from error
            raise

    def close(self):
        try:
            self.io.close()
        finally:
            os.close(self._lock)


class Motors:
    """Les moteurs, trouvés sur le bus à l'ouverture.

    `Motors()` cherche les moteurs d'identifiants 0 à 20 (ou ceux de `ids`) et, sauf si
    `wheel_mode=False`, les met en mode roue. Attributs : `ids` (identifiants trouvés), `models`
    (modèle de chaque moteur), `modes` (« wheel » ou « joint » pour chaque moteur), `watchdog`
    (délai du chien de garde, en s), `max_speed` (vitesse maximale autorisée, en °/s),
    `watchdog_stops` (nombre d'arrêts déclenchés par le chien de garde).

    `with Motors() as motors:` ferme tout à la fin, même en cas d'erreur : roues arrêtées, couple
    coupé, bus libéré.
    """

    def __init__(self, ids=None, port=PORT, baudrate=BAUDRATE, wheel_mode=True, watchdog=0.5,
                 max_speed=MAX_SPEED):
        self.watchdog = watchdog
        self.max_speed = max_speed
        self.watchdog_stops = 0
        self._torque = set()  # moteurs dont le couple est actif
        self._lock = threading.RLock()  # un seul échange à la fois avec le bus ; les modes en dépendent
        self._closing = threading.Event()
        self._closed = False
        self._bus = _Bus(port, baudrate)
        self._io = self._bus.io
        try:
            found = self._io.scan(list(SCAN_IDS if ids is None else ids))
            for _ in range(2):  # une réponse perdue ne doit pas faire croire à un moteur absent
                missing = [] if ids is None else sorted(set(ids) - set(found))
                if not missing:
                    break
                found += self._io.scan(missing)
            if not found:
                raise RuntimeError(
                    "Aucun moteur ne répond : les moteurs sont-ils alimentés (batterie) ? "
                    "Un autre programme utilise-t-il le bus ?")
            if ids is not None and set(found) != set(ids):
                raise RuntimeError(f"Moteurs absents du bus : {sorted(set(ids) - set(found))}")
            self.ids = sorted(found)
            self.models = _read(self._io.get_model, self.ids)
            self.modes = _read(self._io.get_control_mode, self.ids)
            self._speeds = {i: 0.0 for i in self.ids}  # dernières consignes de vitesse (mode roue)
            self._last_command = time.monotonic()
            if wheel_mode:
                self.set_wheel_mode()
        except BaseException:  # y compris une interruption : le bus doit être libéré
            self._bus.close()
            raise
        self._guard = threading.Thread(target=self._watch, name="chien-de-garde", daemon=True)
        self._guard.start()
        _open_motors.add(self)

    # Modes de fonctionnement

    def is_wheel_mode(self):
        """Vrai si tous les moteurs sont en mode roue."""
        return all(mode == "wheel" for mode in self.modes.values())

    def set_wheel_mode(self, ids=None):
        """Met les moteurs (tous, ou ceux de `ids`) en mode roue : ils tournent sans fin, à la vitesse demandée.

        Un moteur qui quitte le mode articulation repart à l'arrêt. Le réglage est mémorisé par le
        moteur, même hors tension.
        """
        ids = self._select(ids)
        with self._lock:
            self._check_open()
            joint = [i for i in ids if self.modes[i] != "wheel"]
            if joint:
                # il se tient d'abord là où il est : la vitesse nulle, qui en mode roue veut dire
                # « arrêt », ne le lance pas vers une ancienne position
                self._hold(joint)
                self._io.set_moving_speed(dict.fromkeys(joint, 0))
                self._io.set_wheel_mode(joint)
                time.sleep(EEPROM_DELAY)
                self.modes.update(dict.fromkeys(joint, "wheel"))
                self._speeds.update(dict.fromkeys(joint, 0.0))

    def set_joint_mode(self, ids=None, speed=JOINT_SPEED):
        """Met les moteurs (tous, ou ceux de `ids`) en mode articulation : chacun va à la position demandée et s'y tient.

        Le moteur ne bouge pas au changement de mode : il garde sa position. `speed` (°/s, au moins
        6) est la vitesse de ses mouvements, jusqu'à nouvel ordre. C'est le mode d'un bras
        robotique, ou d'une tourelle pan-tilt.
        """
        ids = self._select(ids)
        speed = self._check_speed(speed)
        with self._lock:
            self._check_open()
            wheel = [i for i in ids if self.modes[i] != "joint"]
            if wheel:
                # la roue s'arrête, puis la position visée devient la position actuelle : au
                # changement de mode, le moteur ne saute pas vers une ancienne consigne
                self._io.set_moving_speed(dict.fromkeys(wheel, 0))
                self._speeds.update(dict.fromkeys(wheel, 0.0))
                time.sleep(0.1)
                self._hold(wheel)
                self._io.set_joint_mode(wheel)
                time.sleep(EEPROM_DELAY)
                self.modes.update(dict.fromkeys(wheel, "joint"))
            self._io.set_moving_speed(dict.fromkeys(ids, speed))

    def change_id(self, old_id, new_id):
        """Change l'identifiant d'un moteur. Le moteur le garde en mémoire, même hors tension.

        Le nouvel identifiant doit être libre sur le bus, de 0 à 252 : deux moteurs de même
        identifiant se brouillent. Le moteur est arrêté et libéré avant le changement.
        """
        self._check_ids([old_id])
        new_id = int(new_id)
        if not 0 <= new_id <= 252:
            raise ValueError(f"Identifiant {new_id} impossible : de 0 à 252")
        if new_id in self.ids:
            raise ValueError(f"L'identifiant {new_id} est déjà pris (moteurs présents : {self.ids})")
        with self._lock:
            self._check_open()
            if self._io.ping(new_id):
                raise ValueError(f"Un autre moteur répond déjà à l'identifiant {new_id}")
            if self.modes[old_id] == "wheel":
                self._io.set_moving_speed({old_id: 0})
            self._speeds[old_id] = 0.0
            self._io.disable_torque([old_id])
            self._torque.discard(old_id)
            self._io.change_id({old_id: new_id})
            time.sleep(EEPROM_DELAY)
            if not self._io.ping(new_id):
                raise RuntimeError(f"Le moteur ne répond pas à son nouvel identifiant {new_id}")
            self.ids = sorted(new_id if i == old_id else i for i in self.ids)
            for table in (self.models, self.modes, self._speeds):
                table[new_id] = table.pop(old_id)

    # Commandes en mode roue

    def set_speeds(self, speeds):
        """Vitesses des roues en °/s, par exemple {1: 90, 2: -90}. Valable `watchdog` secondes.

        Les moteurs absents du dictionnaire gardent leur consigne. Une vitesse au-delà de
        `max_speed` est ramenée à `max_speed`.
        """
        self._check_ids(speeds)
        values = {i: self._finite(v, "vitesse") for i, v in speeds.items()}
        limited = {i: max(-self.max_speed, min(self.max_speed, v)) for i, v in values.items()}
        with self._lock:  # le mode est vérifié sous le verrou : un autre fil ne peut pas le changer entre-temps
            self._check_open()
            joint = [i for i in limited if self.modes[i] != "wheel"]
            if joint:
                raise ValueError(f"Moteurs en mode articulation : {sorted(joint)} ; utilisez set_positions(), "
                                 "ou repassez-les en mode roue (set_wheel_mode)")
            self._enable_torque(limited)
            self._io.set_moving_speed(limited)
            self._speeds.update(limited)
            self._last_command = time.monotonic()

    def run(self, speeds, duration):
        """Fait tourner les roues aux vitesses données pendant `duration` secondes, puis les arrête.

        La consigne est renvoyée toutes les 0,1 s, ce qui nourrit le chien de garde. Une
        interruption (Ctrl-C, ou « Interrupt Kernel » dans Jupyter) arrête aussi les roues.
        """
        end = time.monotonic() + duration
        try:
            while True:
                self.set_speeds(speeds)
                remaining = end - time.monotonic()
                if remaining <= 0:
                    break
                time.sleep(min(0.1, remaining))
        finally:
            self.stop()

    # Commandes en mode articulation

    def set_positions(self, positions, speed=None):
        """Envoie des moteurs en mode articulation à des positions, en degrés : {1: 90, 2: -45}.

        Les positions vont de -180 à 180, 0 au milieu de la course. `speed` (°/s) règle la vitesse
        du mouvement ; sans elle, c'est la dernière vitesse réglée. Ne bloque pas : le moteur part
        vers sa position et s'y tient une fois arrivé. Pour attendre l'arrivée : move_to().
        """
        self._check_ids(positions)
        goals = self._goals(positions)
        speed = None if speed is None else self._check_speed(speed)
        with self._lock:
            self._check_open()
            wheel = [i for i in goals if self.modes[i] != "joint"]
            if wheel:
                raise ValueError(f"Moteurs en mode roue : {sorted(wheel)} ; passez-les d'abord en mode "
                                 "articulation (set_joint_mode)")
            if speed is not None:
                self._io.set_moving_speed(dict.fromkeys(goals, speed))
            self._enable_torque(goals)
            self._io.set_goal_position(goals)

    def move_to(self, positions, speed=None, timeout=10.0, tolerance=3.0):
        """Comme set_positions(), mais attend que les moteurs soient arrivés, à `tolerance` degrés près, et arrêtés.

        Un moteur arrêté garde souvent un petit écart avec sa consigne, de l'ordre du degré : d'où la
        tolérance de 3°.

        Renvoie les positions atteintes, moteurs arrêtés. Lève TimeoutError s'ils n'y sont pas au bout de `timeout`
        secondes (bloqués par un obstacle ?). En cas d'interruption (Ctrl-C, « Interrupt Kernel »)
        ou d'erreur, les moteurs s'arrêtent et se tiennent là où ils sont.
        """
        self.set_positions(positions, speed)
        goals = self._goals(positions)
        end = time.monotonic() + timeout
        failures = 0
        previous = None  # positions lues au tour précédent : pour savoir si les moteurs bougent encore
        try:
            while True:
                try:
                    current = self.get_positions(list(goals))
                    failures = 0
                except Exception:  # un échange raté sur le bus arrive : on relit, jusqu'à 3 fois de suite
                    failures += 1
                    if failures > 3:
                        raise
                    time.sleep(0.02)
                    continue
                near = all(abs((current[i] - goal + 180) % 360 - 180) <= tolerance for i, goal in goals.items())
                stopped = previous is not None and all(abs(current[i] - previous[i]) <= 0.2 for i in goals)
                if near and (stopped or time.monotonic() > end):
                    return current
                previous = current
                if time.monotonic() > end:
                    raise TimeoutError(f"Positions non atteintes en {timeout} s : visées {goals}, "
                                       f"atteintes {current}")
                time.sleep(0.05)
        except BaseException:
            try:
                with self._lock:
                    if not self._closed:
                        self._hold(list(goals))
            except Exception:  # bus en panne : on garde l'erreur d'origine
                pass
            raise

    # Arrêt

    def stop(self):
        """Arrête tout : les roues freinent (couple actif), les moteurs en mode articulation se tiennent là où ils sont."""
        with self._lock:
            self._check_open()
            wheel = [i for i in self.ids if self.modes[i] == "wheel"]
            joint = [i for i in self.ids if self.modes[i] != "wheel"]
            if wheel:
                self.set_speeds(dict.fromkeys(wheel, 0))
            if joint:
                self._hold(joint)

    def release(self):
        """Arrête tout et coupe le couple : les moteurs tournent alors librement à la main."""
        with self._lock:
            self._check_open()
            wheel = [i for i in self.ids if self.modes[i] == "wheel"]
            if wheel:
                self._io.set_moving_speed(dict.fromkeys(wheel, 0))
            self._io.disable_torque(self.ids)
            self._torque.clear()
            self._speeds = {i: 0.0 for i in self.ids}

    # Mesures

    def get_speeds(self, ids=None):
        """Vitesses mesurées des moteurs (tous, ou ceux de `ids`), en °/s."""
        ids = self._select(ids)
        self._check_open()
        return _read(self._io.get_present_speed, ids)

    def get_positions(self, ids=None):
        """Positions des moteurs (tous, ou ceux de `ids`), en degrés, entre -180 et 180."""
        ids = self._select(ids)
        self._check_open()
        return _read(self._io.get_present_position, ids)

    def get_loads(self, ids=None):
        """Charge des moteurs (tous, ou ceux de `ids`), en % du couple maximal : une roue qui force ou qui est bloquée."""
        ids = self._select(ids)
        self._check_open()
        return _read(self._io.get_present_load, ids)

    def get_temperatures(self, ids=None):
        """Températures des moteurs (tous, ou ceux de `ids`), en °C. Ils se coupent d'eux-mêmes à 70 °C."""
        ids = self._select(ids)
        self._check_open()
        return _read(self._io.get_present_temperature, ids)

    def get_voltages(self, ids=None):
        """Tension d'alimentation lue par les moteurs (tous, ou ceux de `ids`), en V : celle de la batterie."""
        ids = self._select(ids)
        self._check_open()
        return _read(self._io.get_present_voltage, ids)

    # Fermeture et sécurité

    def close(self, hold=False):
        """Libère le bus. Arrête les moteurs et coupe leur couple ; avec hold=True, ils gardent leur couple :
        les roues s'arrêtent en freinant, les moteurs en mode articulation gardent leur consigne et s'y tiennent."""
        if self._closed:
            return
        self._closing.set()
        self._guard.join(timeout=1.0)
        with self._lock:
            try:
                if hold:
                    wheel = [i for i in self.ids if self.modes[i] == "wheel"]
                    if wheel:
                        self.set_speeds(dict.fromkeys(wheel, 0))
                else:
                    self.release()
            finally:
                self._closed = True
                self._bus.close()
                _open_motors.discard(self)

    def _select(self, ids):
        """Liste des identifiants demandés : tous si `ids` vaut None ; un seul nombre est accepté."""
        if ids is None:
            return list(self.ids)
        ids = [ids] if isinstance(ids, int) else list(ids)
        self._check_ids(ids)
        return ids

    def _check_ids(self, ids):
        unknown = set(ids) - set(self.ids)
        if unknown:
            raise ValueError(f"Moteurs inconnus : {sorted(unknown)} (moteurs présents : {self.ids})")

    @staticmethod
    def _finite(value, what):
        value = float(value)
        if not np.isfinite(value):
            raise ValueError(f"{what} invalide : {value} (un calcul a-t-il donné nan ou inf ?)")
        return value

    def _goals(self, positions):
        """Positions visées, vérifiées et ramenées à la course du moteur (-180 à 180°)."""
        return {i: max(-180.0, min(180.0, self._finite(a, "position"))) for i, a in positions.items()}

    def _check_speed(self, speed):
        speed = self._finite(speed, "vitesse")
        if speed < MIN_JOINT_SPEED:
            raise ValueError(f"Vitesse de {speed} °/s trop faible en mode articulation : au moins {MIN_JOINT_SPEED} °/s. "
                             "En dessous, le moteur l'arrondit à 0, qui veut dire « aussi vite que possible ».")
        return min(speed, self.max_speed)

    def _check_open(self):
        if self._closed:
            raise RuntimeError("Moteurs fermés : créez un nouvel objet Motors()")

    def _enable_torque(self, ids):
        """Active le couple des moteurs qui ne l'ont pas encore (à appeler sous self._lock)."""
        to_enable = [i for i in ids if i not in self._torque]
        if to_enable:
            self._io.enable_torque(to_enable)
            self._torque.update(to_enable)

    def _hold(self, ids):
        """La position visée devient la position actuelle (à appeler sous self._lock)."""
        self._io.set_goal_position(_read(self._io.get_present_position, ids))

    def _watch(self):
        """Chien de garde : arrête les roues si les consignes cessent d'arriver."""
        while not self._closing.wait(0.05):
            with self._lock:
                wheel = [i for i in self.ids if self.modes[i] == "wheel"]
                moving = any(self._speeds[i] != 0 for i in wheel)
                if moving and time.monotonic() - self._last_command > self.watchdog:
                    try:
                        self._io.set_moving_speed(dict.fromkeys(wheel, 0))
                        self._speeds.update(dict.fromkeys(wheel, 0.0))
                        self.watchdog_stops += 1
                    except Exception:  # bus momentanément indisponible : on réessaiera
                        pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


@atexit.register
def _close_all():
    """En fin de programme (et au redémarrage d'un noyau Jupyter), arrête les moteurs encore ouverts."""
    for motors in list(_open_motors):
        try:
            motors.close()
        except Exception:
            pass
