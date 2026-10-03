#!/usr/bin/env python3
"""Tension et charge estimée de la batterie, lues par les moteurs Dynamixel.

Les moteurs sont branchés directement sur le pack (3 éléments Li-ion en série) et mesurent
sa tension au dixième de volt. La charge est estimée d'après la courbe de décharge d'un
élément 18650 de 2500 mAh : comptez ±10 %, et la tension baisse un peu quand les moteurs
forcent.

Usage :
    python3 tools/battery.py              # une lecture
    python3 tools/battery.py --watch 60   # surveillance : une ligne à chaque baisse de 0,1 V ou seuil franchi
"""
import argparse
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from holorobot.motors import voltages  # noqa: E402

CELLS = 3
# Tension d'un élément (V) -> charge (%), sous faible débit
CURVE = [(3.00, 0), (3.30, 5), (3.50, 12), (3.60, 22), (3.70, 36), (3.80, 52),
         (3.90, 68), (4.00, 82), (4.10, 92), (4.20, 100)]
# Seuils par élément, du moins grave au plus grave
THRESHOLDS = [(3.60, "batterie à recharger bientôt"),
              (3.45, "batterie faible : arrêter les essais moteurs et recharger"),
              (3.30, "batterie critique : éteindre et débrancher, sinon décharge profonde")]


def charge(cell_voltage):
    """Charge estimée (%) par interpolation linéaire sur la courbe."""
    if cell_voltage <= CURVE[0][0]:
        return 0
    for (v0, c0), (v1, c1) in zip(CURVE, CURVE[1:]):
        if cell_voltage <= v1:
            return round(c0 + (c1 - c0) * (cell_voltage - v0) / (v1 - v0))
    return 100


def read_pack(port, baudrate, ids):
    """Tension du pack (V) : médiane des tensions lues par les moteurs (tous si `ids` vaut None)."""
    return statistics.median(voltages(ids, port, baudrate).values())


def describe(pack):
    cell = pack / CELLS
    return f"{pack:.1f} V ({cell:.2f} V par élément), charge estimée {charge(cell)} %"


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--baudrate", type=int, default=57600)
    parser.add_argument("--ids", type=int, nargs="+", help="moteurs à interroger (par défaut : tous)")
    parser.add_argument("--watch", type=float, metavar="SECONDES",
                        help="surveiller en relisant toutes les SECONDES")
    args = parser.parse_args()

    if args.watch is None:
        print(describe(read_pack(args.port, args.baudrate, args.ids)))
        return

    recent, reported, alerted, failures = [], None, 0, 0
    while True:
        try:
            recent = (recent + [read_pack(args.port, args.baudrate, args.ids)])[-5:]
            failures = 0
        except Exception as error:  # bus occupé par un autre programme, moteurs hors tension…
            failures += 1
            if failures == 3:
                print(time.strftime("%H:%M:%S"), f"lecture impossible depuis 3 essais ({error})", flush=True)
            time.sleep(args.watch)
            continue
        # Médiane glissante : ignore les creux brefs quand les moteurs forcent
        pack = statistics.median(recent)
        level = sum(pack / CELLS <= limit for limit, _ in THRESHOLDS)
        # Une baisse de 0,1 V se signale ; une hausse seulement si elle est nette (batterie changée) :
        # sinon une tension à la limite d'un arrondi ferait des allers-retours
        dropped = reported is not None and reported - pack >= 0.1 - 1e-9
        replaced = reported is not None and pack - reported >= 0.3 - 1e-9
        if reported is None or dropped or replaced or level > alerted:
            if replaced:
                alerted = 0
            warning = f" | ATTENTION : {THRESHOLDS[level - 1][1]}" if level > alerted else ""
            print(time.strftime("%H:%M:%S"), describe(pack) + warning, flush=True)
            reported, alerted = pack, max(alerted, level)
        time.sleep(args.watch)


if __name__ == "__main__":
    main()
