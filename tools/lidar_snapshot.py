#!/usr/bin/env python3
"""Teste le lidar : informations de l'appareil, état, puis image de quelques tours.

Le moteur du lidar tourne pendant quelques secondes. L'image montre les points vus de dessus,
le lidar au centre : 0° en haut, angles croissants dans le sens des aiguilles d'une montre,
comme les mesure le lidar.

Usage : python3 tools/lidar_snapshot.py [--port /dev/ttyUSB0] [--tours 5] [--image lidar.png]
"""
import argparse
import statistics
import sys
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")  # pas besoin d'écran
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from holorobot.lidar import Lidar  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--tours", type=int, default=5, help="nombre de tours à afficher")
    parser.add_argument("--image", default="lidar.png")
    args = parser.parse_args()

    with Lidar(args.port) as lidar:
        print(f"lidar : modèle {lidar.model}, firmware {lidar.firmware}, matériel {lidar.hardware},"
              f" n° de série {lidar.serial_number}, état {lidar.health}")
        scans = [lidar.get_scan() for _ in range(args.tours)]

    distances = [d for scan in scans for d in scan.distances]
    if len(scans) > 1:
        rate = (len(scans) - 1) / (scans[-1].timestamp - scans[0].timestamp)
        print(f"{rate:.1f} tours par seconde", end=" | ")
    print(f"{statistics.median(map(len, scans)):.0f} points par tour | distance min {min(distances):.2f} m,"
          f" médiane {statistics.median(distances):.2f} m, max {max(distances):.2f} m")

    fig = plt.figure(figsize=(7, 7))
    ax = fig.add_subplot(projection="polar")
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)
    for scan in scans:
        ax.scatter(np.radians(scan.angles), scan.distances, s=2, c="tab:blue")
    ax.plot(0, 0, "r^", markersize=10)
    ax.set_rmax(min(max(distances) * 1.05, 10))
    ax.set_title(f"Lidar : {len(scans)} tours superposés (distances en m)")
    fig.savefig(args.image, dpi=110, bbox_inches="tight")
    print("image enregistrée :", args.image)


if __name__ == "__main__":
    main()
