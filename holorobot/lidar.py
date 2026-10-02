"""Pilote du lidar YDLidar X4.

Le lidar tourne sur lui-même, environ 7 tours par seconde, et mesure la distance des obstacles
tout autour de lui, dans un plan horizontal. Ce module lit son flux de données en tâche de fond
et fournit un scan par tour.

Exemple :

    from holorobot.lidar import Lidar

    with Lidar() as lidar:
        scan = lidar.get_scan()
        print(len(scan), "points ; le plus proche (angle, distance) :", scan.nearest())

Conventions :
- les angles sont en degrés dans [0, 360), comptés dans le sens des aiguilles d'une montre vu de
  dessus, à partir de la direction 0° du lidar ;
- les distances sont en mètres ; les directions sans mesure (rien vu, ou obstacle à moins de
  12 cm) sont absentes du scan, elles ne valent jamais 0.

Le décodage suit le protocole officiel du X4 : manuel de développement YDLIDAR et pilote
YDLidar-SDK (src/YDlidarDriver.cpp : waitPackage, calcCheckSum, parsePoints).
"""
import math
import threading
import time
from dataclasses import dataclass

BAUDRATE = 128000
CMD_START, CMD_STOP = b"\xa5\x60", b"\xa5\x65"
CMD_INFO, CMD_HEALTH = b"\xa5\x90", b"\xa5\x91"
HEALTH = {0: "bon", 1: "avertissement", 2: "erreur"}


@dataclass(frozen=True)
class Scan:
    """Un tour complet du lidar, dans l'ordre de mesure."""

    angles: tuple  # degrés, sens horaire, dans [0, 360)
    distances: tuple  # mètres
    timestamp: float | None = None  # time.time() à la fin du tour ; None pour un enregistrement

    def __len__(self):
        return len(self.angles)

    def points(self):
        """Liste des (angle en degrés, distance en m)."""
        return list(zip(self.angles, self.distances))

    def nearest(self):
        """(angle, distance) du point le plus proche, ou None si le scan est vide."""
        return min(self.points(), key=lambda point: point[1], default=None)

    def xy(self):
        """Coordonnées cartésiennes en m : x vers le 0° du lidar, y vers sa gauche."""
        return [(d * math.cos(math.radians(a)), -d * math.sin(math.radians(a))) for a, d in self.points()]


@dataclass(frozen=True)
class Packet:
    """Paquet de mesures du flux de scan."""

    start_of_turn: bool  # bit 0 de CT : paquet de position zéro, le premier d'un tour
    first_angle: float  # angle du premier échantillon, en degrés, avant correction
    last_angle: float  # angle du dernier échantillon, en degrés, avant correction
    samples: tuple  # valeurs brutes Si : distance = Si / 4 mm, 0 = pas de mesure


def parse_packets(buffer, start=0):
    """Paquets complets et valides trouvés dans `buffer` à partir de `start`.

    Renvoie (paquets, position) : `position` est l'endroit où reprendre la lecture, au début d'un
    éventuel paquet incomplet. Un paquet a cette forme :
        AA 55 | CT | LSN | FSA (2) | LSA (2) | CS (2) | LSN échantillons de 2 octets
    Les octets qui ne forment pas un paquet valide sont sautés un par un : la lecture se recale
    ainsi d'elle-même sur l'en-tête suivant.
    """
    packets = []
    i, n = start, len(buffer)
    while i + 10 <= n:
        if buffer[i] != 0xAA or buffer[i + 1] != 0x55:
            i += 1
            continue
        ct, lsn = buffer[i + 2], buffer[i + 3]
        fsa = buffer[i + 4] | buffer[i + 5] << 8
        lsa = buffer[i + 6] | buffer[i + 7] << 8
        checksum = buffer[i + 8] | buffer[i + 9] << 8
        # Dans un vrai paquet, LSN > 0 et le bit 0 de FSA et de LSA vaut 1
        if lsn == 0 or not fsa & 1 or not lsa & 1:
            i += 1
            continue
        end = i + 10 + 2 * lsn
        if end > n:
            break  # paquet pas encore arrivé en entier
        samples = tuple(buffer[j] | buffer[j + 1] << 8 for j in range(i + 10, end, 2))
        check = 0x55AA ^ (ct | lsn << 8) ^ fsa ^ lsa
        for sample in samples:
            check ^= sample
        if check != checksum:
            i += 1
            continue
        packets.append(Packet(bool(ct & 1), (fsa >> 1) / 64.0, (lsa >> 1) / 64.0, samples))
        i = end
    return packets, i


def angle_correction(distance_mm):
    """Correction d'angle du X4, en degrés : l'émetteur laser et le capteur sont décalés."""
    return math.degrees(math.atan(21.8 * (155.3 - distance_mm) / (155.3 * distance_mm)))


def packet_points(packet):
    """Points mesurés d'un paquet : liste de (angle en degrés, distance en m), sans les zéros."""
    first, last = packet.first_angle, packet.last_angle
    if last < first:
        last += 360.0  # le paquet passe par 0°
    count = len(packet.samples)
    step = (last - first) / (count - 1) if count > 1 else 0.0
    points = []
    for k, raw in enumerate(packet.samples):
        if raw == 0:  # pas de mesure : on n'invente pas de point
            continue
        distance = raw / 4.0
        # Chaque échantillon est corrigé avec SA distance
        angle = (first + k * step + angle_correction(distance)) % 360.0
        points.append((angle, distance / 1000.0))
    return points


MIN_TURN_COVERAGE = 350.0  # degrés : en dessous, ce n'est qu'un morceau de tour


class TurnAssembler:
    """Regroupe les points par tour.

    Un tour commence au paquet de position zéro (bit 0 de CT). Sur le X4, ce paquet arrive vers
    347° (repère du codeur), pas à 0° : un tour passe donc par 0° en son milieu. Juste après la
    commande de démarrage, le premier de ces paquets arrive à un autre angle et le moteur n'est
    pas encore à sa vitesse : le premier tour est écarté, comme tout morceau de tour (moins de
    350° couverts).
    """

    def __init__(self):
        self._skip = 1  # tours complets à écarter au démarrage
        self._points = None  # None tant que le premier tour n'a pas commencé
        self._start_angle = None  # angle du premier paquet du tour en cours
        self._progress = 0.0  # avancée du dernier paquet depuis le début du tour, en degrés
        self._coverage = 0.0  # angle couvert par le tour en cours, en degrés

    def add(self, packet):
        """Ajoute un paquet ; renvoie les points du tour qui vient de se terminer, sinon None."""
        finished = None
        progress = None
        if self._start_angle is not None:
            progress = (packet.first_angle - self._start_angle) % 360.0
        # Si le paquet de position zéro a été perdu (paquet abîmé), on coupe quand même le tour
        # en voyant l'angle repasser par celui du début du tour
        if packet.start_of_turn or (progress is not None and progress < self._progress - 180.0):
            if self._points is not None and self._coverage >= MIN_TURN_COVERAGE:
                if self._skip:
                    self._skip -= 1
                else:
                    finished = self._points
            self._points, self._start_angle = [], packet.first_angle
            progress = 0.0
        if self._points is not None:
            self._progress = progress
            self._coverage = progress + (packet.last_angle - packet.first_angle) % 360.0
            self._points.extend(packet_points(packet))
        return finished


def make_scan(points, timestamp=None):
    return Scan(tuple(a for a, _ in points), tuple(d for _, d in points), timestamp)


def decode(raw):
    """Tours complets contenus dans un enregistrement d'octets bruts du lidar (liste de Scan)."""
    packets, _ = parse_packets(raw)
    assembler = TurnAssembler()
    scans = []
    for packet in packets:
        points = assembler.add(packet)
        if points is not None:
            scans.append(make_scan(points))
    return scans


class Lidar:
    """Lidar YDLidar X4 branché en USB.

    À l'ouverture, le pilote lit les informations de l'appareil (`model`, `firmware`, `hardware`,
    `serial_number`) et son état (`health`). `start()` lance le moteur et la lecture des tours en
    tâche de fond, `close()` arrête tout. `with Lidar() as lidar:` fait les deux automatiquement.

    Le pilote lit le port dans son propre thread, en continu : un programme lent ne fait pas
    prendre de retard aux mesures, et `get_scan()` ou `latest()` peuvent être appelés depuis
    n'importe quel thread, même plusieurs à la fois.
    """

    def __init__(self, port="/dev/ttyUSB0"):
        # Importé ici : le décodage (decode, Scan…) reste utilisable sans pyserial
        import serial

        self.port = port
        self._serial = serial.Serial()
        self._serial.port = port
        self._serial.baudrate = BAUDRATE
        self._serial.timeout = 0.1
        self._serial.dtr = False  # la ligne DTR commande le moteur : arrêté pour l'instant
        try:
            self._serial.open()
        except serial.SerialException as error:
            raise RuntimeError(f"Impossible d'ouvrir le lidar sur {port} : {error}") from error
        self._thread = None
        self._control = threading.Lock()  # start(), stop() et close() un seul à la fois
        self._stopping = threading.Event()
        self._new_scan = threading.Condition()
        self._latest = None
        self._count = 0
        self._error = None
        try:
            # Au cas où un programme précédent aurait laissé le lidar en train de scanner
            self._serial.write(CMD_STOP)
            time.sleep(0.1)
            self._serial.reset_input_buffer()
            info = self._request(CMD_INFO, 0x04, 20)
            status = self._request(CMD_HEALTH, 0x06, 3)
        except Exception:
            self._serial.close()
            raise
        self.model = info[0]
        self.firmware = f"{info[2]}.{info[1]}"
        self.hardware = info[3]
        self.serial_number = "".join(str(b) for b in info[4:20])
        self.health = HEALTH.get(status[0], f"inconnu ({status[0]})")
        self.error_code = status[1] | status[2] << 8

    def _request(self, command, response_type, length):
        """Envoie une commande hors scan et renvoie les `length` octets de données de la réponse.

        Réponse : A5 5A | longueur (4 octets) | type | données.
        """
        self._serial.reset_input_buffer()
        self._serial.write(command)
        expected = b"\xa5\x5a"
        received = bytearray()
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            received += self._serial.read(max(1, self._serial.in_waiting))
            i = received.find(expected)
            if i >= 0 and len(received) >= i + 7 + length and received[i + 6] == response_type:
                return bytes(received[i + 7:i + 7 + length])
        raise RuntimeError(f"Le lidar ne répond pas sur {self.port} (commande {command.hex(' ')})")

    def start(self):
        """Lance le moteur et la lecture des tours ; attend le premier tour complet."""
        with self._control:
            if self._thread is not None:
                return
            self._error = None
            self._serial.reset_input_buffer()
            self._serial.dtr = True  # moteur en marche
            self._serial.write(CMD_START)
            self._stopping.clear()
            self._thread = threading.Thread(target=self._read_loop, name="lidar", daemon=True)
            self._thread.start()
        try:
            self.get_scan(timeout=5.0)  # vérifie que les mesures arrivent bien
        except Exception:
            self.stop()
            raise

    def _read_loop(self):
        buffer = bytearray()
        assembler = TurnAssembler()
        try:
            while not self._stopping.is_set():
                chunk = self._serial.read(max(1, self._serial.in_waiting))
                if not chunk:
                    continue
                buffer += chunk
                packets, position = parse_packets(buffer)
                del buffer[:position]
                for packet in packets:
                    points = assembler.add(packet)
                    if points is not None:
                        self._publish(make_scan(points, time.time()))
        except Exception as error:  # câble débranché, port fermé…
            with self._new_scan:
                self._error = error
                self._new_scan.notify_all()

    def _publish(self, scan):
        with self._new_scan:
            self._latest = scan
            self._count += 1
            self._new_scan.notify_all()

    def get_scan(self, timeout=2.0):
        """Attend le prochain tour complet et le renvoie (Scan) : les données sont toujours fraîches."""
        if self._thread is None:
            raise RuntimeError("Lidar arrêté : appeler start(), ou utiliser « with Lidar() as lidar: »")
        with self._new_scan:
            seen = self._count
            done = lambda: self._count > seen or self._error is not None or self._stopping.is_set()  # noqa: E731
            if not self._new_scan.wait_for(done, timeout):
                raise TimeoutError(f"Aucun tour complet reçu du lidar en {timeout} s")
            if self._error is not None:
                raise RuntimeError(f"Le lidar a cessé de répondre : {self._error}") from self._error
            if self._count == seen:  # réveillé par stop() depuis un autre thread
                raise RuntimeError("Lidar arrêté pendant l'attente d'un tour")
            return self._latest

    def latest(self):
        """Dernier tour complet reçu, sans attendre : il peut avoir déjà été renvoyé. None au tout début."""
        with self._new_scan:
            if self._error is not None:
                raise RuntimeError(f"Le lidar a cessé de répondre : {self._error}") from self._error
            return self._latest

    def scans(self):
        """Tours successifs, sans fin : for scan in lidar.scans(): ..."""
        while True:
            yield self.get_scan()

    def stop(self):
        """Arrête la lecture des tours et le moteur ; le port reste ouvert."""
        with self._control:
            if self._thread is None:
                return
            self._stopping.set()
            with self._new_scan:
                self._new_scan.notify_all()  # réveille les get_scan() en attente dans d'autres threads
            self._thread.join(timeout=1.0)
            self._thread = None
            self._serial.write(CMD_STOP)
            self._serial.dtr = False
            time.sleep(0.1)
            self._serial.reset_input_buffer()

    def close(self):
        """Arrête tout et libère le port."""
        try:
            self.stop()
        finally:
            self._serial.close()

    def __enter__(self):
        try:
            self.start()
        except Exception:
            self._serial.close()
            raise
        return self

    def __exit__(self, *exc):
        self.close()
