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

Sécurité : un chien de garde arrête toutes les roues si aucune consigne n'arrive pendant
`watchdog` secondes (0,5 s par défaut). Une consigne de set_speeds() ne vaut donc que 0,5 s : pour
tourner plus longtemps, utilisez run(), ou renvoyez la consigne régulièrement dans une boucle.

Ce module s'appuie sur pypot (pypot.dynamixel.DxlIO).
"""
import atexit
import fcntl
import os
import threading
import time
import weakref

PORT = "/dev/serial0"
BAUDRATE = 57600
MAX_SPEED = 720.0  # °/s, soit 2 tours de roue par seconde : limite de sécurité, réglable
SCAN_IDS = range(21)  # identifiants cherchés par défaut : 0 à 20

_open_motors = weakref.WeakSet()


def scan(ids=SCAN_IDS, port=PORT, baudrate=BAUDRATE):
    """Identifiants des moteurs qui répondent sur le bus, sans rien faire bouger."""
    bus = _Bus(port, baudrate)
    try:
        return bus.io.scan(list(ids))
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
        except Exception as error:
            os.close(self._lock)
            raise RuntimeError(f"Impossible d'ouvrir le bus des moteurs sur {port} : {error}") from error

    def close(self):
        try:
            self.io.close()
        finally:
            os.close(self._lock)


class Motors:
    """Les moteurs des roues, trouvés sur le bus à l'ouverture.

    `Motors()` cherche les moteurs d'identifiants 0 à 20 (ou ceux de `ids`) et, sauf si
    `wheel_mode=False`, les met en mode roue. Attributs : `ids` (identifiants trouvés), `models`
    (modèle de chaque moteur), `watchdog` (délai du chien de garde, en s), `max_speed` (vitesse
    maximale autorisée, en °/s).

    `with Motors() as motors:` ferme tout à la fin, même en cas d'erreur : roues arrêtées, couple
    coupé, bus libéré.
    """

    def __init__(self, ids=None, port=PORT, baudrate=BAUDRATE, wheel_mode=True, watchdog=0.5,
                 max_speed=MAX_SPEED):
        self._bus = _Bus(port, baudrate)
        self._io = self._bus.io
        try:
            found = self._io.scan(list(SCAN_IDS if ids is None else ids))
            if not found:
                raise RuntimeError(
                    "Aucun moteur ne répond : les moteurs sont-ils alimentés (batterie) ? "
                    "Un autre programme utilise-t-il le bus ?")
            if ids is not None and set(found) != set(ids):
                raise RuntimeError(f"Moteurs absents du bus : {sorted(set(ids) - set(found))}")
            self.ids = sorted(found)
            self.models = dict(zip(self.ids, self._io.get_model(self.ids)))
            if wheel_mode:
                self.set_wheel_mode()
        except Exception:
            self._bus.close()
            raise
        self.watchdog = watchdog
        self.max_speed = max_speed
        self.watchdog_stops = 0  # nombre d'arrêts déclenchés par le chien de garde
        self._speeds = {i: 0.0 for i in self.ids}  # dernières consignes envoyées
        self._torque = set()  # moteurs dont le couple est actif
        self._last_command = time.monotonic()
        self._lock = threading.Lock()
        self._closing = threading.Event()
        self._closed = False
        self._guard = threading.Thread(target=self._watch, name="chien-de-garde", daemon=True)
        self._guard.start()
        _open_motors.add(self)

    # Mode de fonctionnement

    def is_wheel_mode(self):
        """Vrai si tous les moteurs sont en mode roue."""
        return all(mode == "wheel" for mode in self._io.get_control_mode(self.ids))

    def set_wheel_mode(self):
        """Met les moteurs en mode roue : ils tournent sans fin, à la vitesse demandée.

        En mode articulation, l'autre mode des Dynamixel, un moteur va à une position et s'y
        arrête : c'est celui d'un bras robotique, pas d'une roue. Le réglage est mémorisé par le
        moteur, même hors tension.
        """
        to_change = [i for i, mode in zip(self.ids, self._io.get_control_mode(self.ids)) if mode != "wheel"]
        if to_change:
            self._io.set_wheel_mode(to_change)

    # Commandes

    def set_speeds(self, speeds):
        """Vitesses des roues en °/s, par exemple {1: 90, 2: -90}. Valable `watchdog` secondes.

        Les moteurs absents du dictionnaire gardent leur consigne. Une vitesse au-delà de
        `max_speed` est ramenée à `max_speed`.
        """
        unknown = set(speeds) - set(self.ids)
        if unknown:
            raise ValueError(f"Moteurs inconnus : {sorted(unknown)} (moteurs présents : {self.ids})")
        limited = {i: max(-self.max_speed, min(self.max_speed, float(v))) for i, v in speeds.items()}
        with self._lock:
            self._check_open()
            to_enable = [i for i in limited if i not in self._torque]
            if to_enable:
                self._io.enable_torque(to_enable)
                self._torque.update(to_enable)
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

    def stop(self):
        """Arrête toutes les roues. Le couple reste actif : les roues freinent."""
        self.set_speeds({i: 0 for i in self.ids})

    def release(self):
        """Arrête toutes les roues et coupe le couple : elles tournent alors librement à la main."""
        with self._lock:
            self._check_open()
            self._io.set_moving_speed({i: 0 for i in self.ids})
            self._io.disable_torque(self.ids)
            self._torque.clear()
            self._speeds = {i: 0.0 for i in self.ids}

    # Mesures

    def get_speeds(self):
        """Vitesses mesurées des roues, en °/s."""
        return dict(zip(self.ids, self._io.get_present_speed(self.ids)))

    def get_positions(self):
        """Positions des roues, en degrés, entre -180 et 180."""
        return dict(zip(self.ids, self._io.get_present_position(self.ids)))

    def get_temperatures(self):
        """Températures des moteurs, en °C. Ils se coupent d'eux-mêmes à 70 °C."""
        return dict(zip(self.ids, self._io.get_present_temperature(self.ids)))

    def get_voltages(self):
        """Tension d'alimentation lue par chaque moteur, en V : celle de la batterie."""
        return dict(zip(self.ids, self._io.get_present_voltage(self.ids)))

    # Fermeture et sécurité

    def close(self):
        """Arrête les roues, coupe le couple et libère le bus."""
        if self._closed:
            return
        self._closing.set()
        self._guard.join(timeout=1.0)
        try:
            self.release()
        finally:
            self._closed = True
            self._bus.close()
            _open_motors.discard(self)

    def _check_open(self):
        if self._closed:
            raise RuntimeError("Moteurs fermés : créez un nouvel objet Motors()")

    def _watch(self):
        """Chien de garde : arrête les roues si les consignes cessent d'arriver."""
        while not self._closing.wait(0.05):
            with self._lock:
                moving = any(v != 0 for v in self._speeds.values())
                if moving and time.monotonic() - self._last_command > self.watchdog:
                    try:
                        self._io.set_moving_speed({i: 0 for i in self.ids})
                        self._speeds = {i: 0.0 for i in self.ids}
                        self.watchdog_stops += 1
                    except Exception:  # bus momentanément indisponible : on réessaiera
                        pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


@atexit.register
def _close_all():
    """En fin de programme (et au redémarrage d'un noyau Jupyter), arrête les roues encore ouvertes."""
    for motors in list(_open_motors):
        try:
            motors.close()
        except Exception:
            pass
