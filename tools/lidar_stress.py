#!/usr/bin/env python3
"""Essais de robustesse du pilote du lidar avec des threads, sur le vrai lidar (environ 40 s).

À lancer sur un robot préparé, pour vérifier que le lidar et le pilote tiennent la charge :
consommateur dans un thread pendant un calcul intensif, plusieurs threads en parallèle,
consommateur lent, latest() à 20 Hz, arrêt pendant une attente, cycles démarrage/arrêt.
Les roues ne bougent pas : seul le moteur du lidar tourne.

Usage : python3 tools/lidar_stress.py
"""
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from holorobot.lidar import Lidar  # noqa: E402


def age(scan):
    return time.time() - scan.timestamp


def consume(lidar, duration, results, key, pause=0.0):
    """Lit des tours pendant `duration` s ; note le nombre, l'âge max et les erreurs."""
    count, oldest, errors = 0, 0.0, []
    end = time.monotonic() + duration
    while time.monotonic() < end:
        try:
            scan = lidar.get_scan()
            count += 1
            oldest = max(oldest, age(scan))
        except Exception as error:  # noqa: BLE001
            errors.append(repr(error))
        if pause:
            time.sleep(pause)
    results[key] = (count, oldest, errors)


with Lidar() as lidar:
    # A. consommateur dans un thread, thread principal qui sature le processeur
    results = {}
    worker = threading.Thread(target=consume, args=(lidar, 10, results, "A"))
    worker.start()
    end = time.monotonic() + 10
    x = 0
    while time.monotonic() < end:
        x = (x * 31 + 7) % 1000003  # calcul inutile pour occuper le thread principal
    worker.join()
    count, oldest, errors = results["A"]
    print(f"A. thread + calcul intensif : {count} tours en 10 s ({count / 10:.1f}/s), âge max {oldest * 1000:.0f} ms, erreurs {len(errors)}")

    # B. trois consommateurs en parallèle
    results = {}
    workers = [threading.Thread(target=consume, args=(lidar, 5, results, k)) for k in "123"]
    for w in workers:
        w.start()
    for w in workers:
        w.join()
    print("B. 3 threads en parallèle : " + " | ".join(
        f"thread {k} : {c} tours, âge max {o * 1000:.0f} ms, erreurs {len(e)}" for k, (c, o, e) in sorted(results.items())))

    # C. consommateur lent : un tour par seconde, les données doivent rester fraîches
    results = {}
    consume(lidar, 6, results, "C", pause=1.0)
    count, oldest, errors = results["C"]
    print(f"C. consommateur lent (1 lecture/s) : {count} tours, âge max {oldest * 1000:.0f} ms, erreurs {len(errors)}")

    # D. latest() à 20 Hz
    seen, oldest, nones = set(), 0.0, 0
    end = time.monotonic() + 5
    while time.monotonic() < end:
        scan = lidar.latest()
        if scan is None:
            nones += 1
        else:
            seen.add(scan.timestamp)  # pas id() : Python réutilise les identifiants des objets libérés
            oldest = max(oldest, age(scan))
        time.sleep(0.05)
    print(f"D. latest() à 20 Hz pendant 5 s : {len(seen)} tours différents vus, âge max {oldest * 1000:.0f} ms, None : {nones}")

    # E. arrêt depuis un autre thread pendant qu'un consommateur attend
    outcome = {}

    def waiter():
        try:
            while True:
                lidar.get_scan(timeout=10)
        except Exception as error:  # noqa: BLE001
            outcome["exception"] = (type(error).__name__, str(error), time.monotonic())

    t = threading.Thread(target=waiter)
    t.start()
    time.sleep(0.5)
    stop_time = time.monotonic()
    lidar.stop()
    t.join(timeout=12)
    name, message, when = outcome.get("exception", ("aucune", "", stop_time))
    print(f"E. stop() pendant une attente : {name} « {message} » {1000 * (when - stop_time):.0f} ms après stop(), thread terminé : {not t.is_alive()}")

    # F. cycles démarrage / arrêt
    start = time.monotonic()
    sizes = []
    for _ in range(5):
        lidar.start()
        sizes.append(len(lidar.get_scan()))
        lidar.stop()
    print(f"F. 5 cycles start/get_scan/stop : {time.monotonic() - start:.1f} s, points par tour {sizes}")
    lidar.start()  # pour que le « with » se termine normalement

    # G. coût processeur d'un consommateur normal
    wall, cpu = time.monotonic(), time.process_time()
    n = sum(1 for _ in range(35) if lidar.get_scan())
    print(f"G. processeur pour {n} tours : {100 * (time.process_time() - cpu) / (time.monotonic() - wall):.0f} % d'un cœur")
print("lidar fermé proprement")
