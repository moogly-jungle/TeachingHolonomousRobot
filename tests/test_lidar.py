"""Tests du pilote du lidar, sans robot : on rejoue 8 s d'octets bruts enregistrés sur MobileRobot-1.

Lancer avec : python3 -m pytest tests   (ou simplement : python3 tests/test_lidar.py)

L'enregistrement tests/data/x4_raw.bin a été décodé une première fois par un décodeur de référence
écrit d'après le pilote officiel YDLidar-SDK : les valeurs attendues ci-dessous en viennent.
"""
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np  # noqa: E402

from holorobot import lidar  # noqa: E402

RAW = (Path(__file__).parent / "data" / "x4_raw.bin").read_bytes()


def packet_bytes(ct, first_angle, last_angle, samples):
    """Fabrique un paquet de scan valide, comme l'enverrait le X4."""
    fsa = int(first_angle * 64) << 1 | 1
    lsa = int(last_angle * 64) << 1 | 1
    checksum = 0x55AA ^ (ct | len(samples) << 8) ^ fsa ^ lsa
    for sample in samples:
        checksum ^= sample
    words = [fsa, lsa, checksum, *samples]
    return bytes([0xAA, 0x55, ct, len(samples)]) + b"".join(w.to_bytes(2, "little") for w in words)


def stream(raw, chunks):
    """Décode `raw` morceau par morceau, comme le fait la tâche de fond du pilote."""
    buffer, assembler, scans = bytearray(), lidar.TurnAssembler(), []
    for chunk in chunks:
        buffer += chunk
        packets, position = lidar.parse_packets(buffer)
        del buffer[:position]
        for packet in packets:
            points = assembler.add(packet)
            if points is not None:
                scans.append(lidar.make_scan(points))
    return scans


def test_recording():
    scans = lidar.decode(RAW)
    assert len(scans) == 53
    assert sum(map(len, scans)) == 27848
    for scan in scans:
        assert 500 <= len(scan) <= 560
        assert all(0 <= a < 360 for a in scan.angles)
        assert all(0.12 < d < 10 for d in scan.distances)  # jamais de zéro ni de point fantôme
    first = scans[0]
    assert [(round(a, 4), round(d, 4)) for a, d in first.points()[:3]] == [
        (339.3956, 2.198), (339.9132, 2.19), (340.4181, 2.188)]
    assert tuple(round(x, 4) for x in first.nearest()) == (201.8684, 0.315)


def test_streaming_matches_whole_recording():
    rng = random.Random(1)
    chunks, i = [], 0
    while i < len(RAW):
        n = rng.randint(1, 700)
        chunks.append(RAW[i:i + n])
        i += n
    assert [s.points() for s in stream(RAW, chunks)] == [s.points() for s in lidar.decode(RAW)]


def test_garbage_between_packets_is_skipped():
    expected = [s.points() for s in lidar.decode(RAW)]
    packets, _ = lidar.parse_packets(RAW)
    rebuilt = [packet_bytes(int(p.start_of_turn), p.first_angle, p.last_angle, list(p.samples)) for p in packets]
    assert [s.points() for s in lidar.decode(b"".join(rebuilt))] == expected
    # Des octets parasites glissés devant un paquet sur vingt ne changent rien au résultat
    rng = random.Random(2)
    noisy = bytearray()
    for chunk in rebuilt:
        if rng.random() < 0.05:
            noisy += bytes(rng.randrange(256) for _ in range(rng.randint(1, 30)))
        noisy += chunk
    assert [s.points() for s in lidar.decode(bytes(noisy))] == expected


def test_corrupted_packet_is_rejected():
    packets, _ = lidar.parse_packets(RAW)
    good = len(packets)
    corrupted = bytearray(RAW)
    corrupted[len(RAW) // 2] ^= 0x10  # un seul octet abîmé au milieu du flux
    packets, _ = lidar.parse_packets(corrupted)
    assert good - 1 <= len(packets) < good + 1  # au plus le paquet touché disparaît


def test_lost_start_packets_still_split_turns():
    packets, _ = lidar.parse_packets(RAW)
    assembler, turns, kept_first = lidar.TurnAssembler(), [], False
    for packet in packets:
        if packet.start_of_turn:
            if kept_first:
                continue  # on « perd » tous les paquets de position zéro sauf le premier
            kept_first = True
        points = assembler.add(packet)
        if points is not None:
            turns.append(points)
    assert len(turns) >= 50
    assert max(map(len, turns)) < 600  # aucun tour n'en contient deux


def test_handmade_packet():
    # Une distance de 155,25 mm rend la correction d'angle quasi nulle (elle s'annule à 155,3 mm)
    raw = packet_bytes(0, 10.0, 12.0, [621, 0, 621])
    packets, position = lidar.parse_packets(raw)
    assert position == len(raw) and len(packets) == 1
    points = lidar.packet_points(packets[0])
    assert len(points) == 2  # l'échantillon nul n'est pas un point
    assert [round(a, 2) for a, _ in points] == [10.0, 12.0]
    assert all(d == 0.15525 for _, d in points)
    # Paquet incomplet : rien n'est consommé, on attend la suite
    assert lidar.parse_packets(raw[:-1]) == ([], 0)


def test_scan_helpers():
    scan = lidar.Scan(angles=(0.0, 90.0, 180.0), distances=(1.0, 2.0, 0.5))
    assert scan.nearest() == (180.0, 0.5)
    x, y = zip(*scan.xy())
    # 0° devant, 90° (sens horaire) à droite donc y négatif, 180° derrière
    assert [round(v, 9) for v in x] == [1.0, 0.0, -0.5]
    assert [round(v, 9) for v in y] == [0.0, -2.0, 0.0]
    assert lidar.Scan(angles=(), distances=()).nearest() is None
    assert np.isclose(lidar.angle_correction(155.3), 0.0, atol=1e-12)


def simulated_scan(walls, noise=0.005, step=0.5, dropout=0.1, seed=0):
    """Scan simulé : un rayon tous les `step` degrés (sens horaire), arrêté par le mur le plus proche."""
    rng = random.Random(seed)
    points = []
    for k in range(int(round(360 / step))):
        a = k * step
        dx, dy = np.cos(np.radians(a)), -np.sin(np.radians(a))
        best = None
        for (x1, y1), (x2, y2) in walls:
            ex, ey = x2 - x1, y2 - y1
            den = dx * ey - dy * ex
            if abs(den) < 1e-12:
                continue
            t = (x1 * ey - y1 * ex) / den  # distance le long du rayon
            u = (x1 * dy - y1 * dx) / den  # position sur le mur, de 0 à 1
            if t > 0 and 0 <= u <= 1:
                best = t if best is None else min(best, t)
        if best is not None and rng.random() >= dropout:
            points.append((a, best + rng.gauss(0, noise)))
    return points


def angle_gap(a, b):
    return abs((a - b + 180) % 360 - 180)


def test_segment_helpers():
    segment = lidar.Segment(start=(1.0, 1.0), end=(1.0, -1.0), points=20, rms=0.002, confidence=0.9)
    assert segment.length == 2.0 and segment.midpoint == (1.0, 0.0)
    assert np.isclose(segment.angle, 90.0) and np.isclose(segment.distance, 1.0)
    assert angle_gap(segment.bearing, 0.0) < 1e-9
    assert [round(v, 12) for v in segment.equation] == [1.0, 0.0, -1.0]


def test_segments_of_a_room():
    # Pièce de 3,5 m sur 3 m, lidar décentré : murs à x = 2 (devant), x = -1,5, y = 1,2 (à gauche), y = -1,8
    corners = [(2.0, 1.2), (2.0, -1.8), (-1.5, -1.8), (-1.5, 1.2)]
    walls = list(zip(corners, corners[1:] + corners[:1]))
    segments = lidar.find_segments(simulated_scan(walls))
    confident = [s for s in segments if s.confidence > 0.5]
    assert len(confident) == 4, [(round(s.distance, 2), round(s.bearing), round(s.confidence, 2)) for s in segments]
    for distance, bearing, length in ((2.0, 0, 3.0), (1.8, 90, 3.5), (1.5, 180, 3.0), (1.2, 270, 3.5)):
        match = [s for s in confident if abs(s.distance - distance) < 0.02 and angle_gap(s.bearing, bearing) < 2]
        assert len(match) == 1, (distance, bearing)
        s = match[0]
        assert abs(s.length - length) < 0.2 and s.confidence > 0.8
        a, b, c = s.equation
        assert np.isclose(a * a + b * b, 1.0) and np.isclose(-c, s.distance)
        for x, y in (s.start, s.end):
            assert abs(a * x + b * y + c) < 1e-9


def test_wall_across_zero_degrees():
    # Le tour commence et finit à 0° : un mur droit devant ne doit pas être coupé en deux
    segments = lidar.find_segments(simulated_scan([((1.5, 1.0), (1.5, -1.0))]))
    assert len(segments) == 1
    assert abs(segments[0].length - 2.0) < 0.1 and angle_gap(segments[0].bearing, 0.0) < 1


def test_clutter_gives_no_confident_segment():
    rng = random.Random(3)
    points = [(rng.uniform(0, 360), rng.uniform(0.3, 3.0)) for _ in range(300)]
    assert all(s.confidence < 0.5 for s in lidar.find_segments(points))


def test_holes_lower_the_confidence():
    # Dans un même tour, un mur bien mesuré devant, et derrière un mur qui renvoie mal le laser
    rng = random.Random(1)
    points = [(a, d) for a, d in simulated_scan([((1.0, 1.5), (1.0, -1.5)), ((-1.0, -1.5), (-1.0, 1.5))], dropout=0.05)
              if not 90 < a < 270 or rng.random() > 0.6]
    segments = lidar.find_segments(points)
    front = [s for s in segments if angle_gap(s.bearing, 0) < 5]
    back = [s for s in segments if angle_gap(s.bearing, 180) < 5]
    assert len(front) == len(back) == 1
    assert back[0].confidence < front[0].confidence - 0.2


def test_recorded_room_segments():
    # Sur MobileRobot-1 immobile, un mur à 1,43 m presque droit devant, vu à chaque tour
    scans = lidar.decode(RAW)
    seen = 0
    for scan in scans:
        segments = scan.segments()
        assert 8 <= len(segments) <= 25
        seen += any(abs(s.distance - 1.43) < 0.03 and angle_gap(s.bearing, 356) < 3 and s.confidence > 0.8
                    for s in segments)
    assert seen >= 0.9 * len(scans)


def test_degenerate_scans():
    # angles arrondis au degré (le pas angulaire médian vaut 0) et points tous identiques : pas d'erreur
    rounded = [(float(round(k / 3)), 1.0 + 0.001 * (k % 3)) for k in range(360 * 3)]
    lidar.find_segments(rounded)
    assert lidar.find_segments([(10.0, 1.0)] * 20, min_length=0) == []
    assert lidar.find_segments([]) == []


if __name__ == "__main__":
    for name, test in list(globals().items()):
        if name.startswith("test_"):
            test()
            print("ok", name)
