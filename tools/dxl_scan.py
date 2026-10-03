#!/usr/bin/env python3
"""Cherche les moteurs Dynamixel (protocole 1.0) présents sur le bus série.

L'outil ne fait que des lectures : aucun moteur ne bouge. Pour chaque vitesse de bus
testée, il envoie un PING à chaque identifiant, puis lit la table de contrôle des
moteurs qui répondent : modèle, vitesse configurée, mode (roue ou articulation),
tension et température.

Usage :
    python3 tools/dxl_scan.py                      # bus /dev/serial0, 57600 et 1000000 bauds
    python3 tools/dxl_scan.py --baud all           # toutes les vitesses des MX
    python3 tools/dxl_scan.py --ids 1-10 --baud 57600
"""
import argparse
import time

import serial

PING, READ = 0x01, 0x02
MODELS = {360: "MX-12W", 29: "MX-28", 310: "MX-64", 320: "MX-106", 12: "AX-12A", 300: "AX-12W", 18: "AX-18A"}
# Valeur du registre « Baud Rate » (adresse 4) des MX -> vitesse en bauds
BAUD_REGISTER = {1: 1000000, 3: 500000, 4: 400000, 7: 250000, 9: 200000, 16: 115200,
                 34: 57600, 103: 19200, 207: 9600, 250: 2250000, 251: 2500000, 252: 3000000}
ERRORS = ["tension", "angle", "surchauffe", "plage", "checksum", "surcharge", "instruction"]


def packet(dxl_id, instruction, params=()):
    body = [dxl_id, len(params) + 2, instruction, *params]
    return bytes([0xFF, 0xFF, *body, ~sum(body) & 0xFF])


def exchange(port, tx, timeout):
    """Envoie un paquet et renvoie tout ce qui revient pendant `timeout` secondes."""
    port.reset_input_buffer()
    port.write(tx)
    port.flush()
    rx = b""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rx += port.read(port.in_waiting or 1)
    return rx


def parse_status(rx, dxl_id):
    """Premier paquet de statut valide de ce moteur dans `rx` : (erreur, paramètres) ou None."""
    i = rx.find(b"\xff\xff")
    while i != -1 and i + 5 < len(rx):
        if rx[i + 2] == dxl_id:
            end = i + 4 + rx[i + 3]
            if end <= len(rx) and (~sum(rx[i + 2:end - 1]) & 0xFF) == rx[end - 1]:
                return rx[i + 4], rx[i + 5:end - 1]
        i = rx.find(b"\xff\xff", i + 1)
    return None


def request(port, dxl_id, instruction, params, echo, timeout):
    tx = packet(dxl_id, instruction, params)
    rx = exchange(port, tx, timeout)
    # Sur un bus demi-duplex câblé sans commande de direction, la Pi relit ce qu'elle envoie
    if echo and rx.startswith(tx):
        rx = rx[len(tx):]
    return parse_status(rx, dxl_id), rx


def detect_echo(port, timeout):
    """Vrai si la Pi relit ses propres paquets (écho du demi-duplex) : test sur des IDs improbables."""
    hits = 0
    for dxl_id in (253, 252, 251):
        tx = packet(dxl_id, PING)
        if exchange(port, tx, timeout) == tx:
            hits += 1
    return hits >= 2


def bus_speed(register):
    """Vitesse réelle du bus pour une valeur du registre « Baud Rate » des MX : 2 Mbit/s / (valeur + 1)."""
    if register >= 250:
        return {250: 2250000, 251: 2500000, 252: 3000000}.get(register)
    return round(2000000 / (register + 1))


def describe(table):
    word = lambda a: table[a] | table[a + 1] << 8
    model = word(0)
    cw, ccw = word(6), word(8)
    mode = "roue" if cw == 0 and ccw == 0 else ("multitour" if cw == 4095 and ccw == 4095 else f"articulation {cw}-{ccw}")
    return (f"modèle {MODELS.get(model, model)} (firmware {table[2]}) | registre vitesse {table[4]}"
            f" = {bus_speed(table[4])} bauds | mode {mode} | couple {'actif' if table[24] else 'relâché'}"
            f" | {table[42] / 10:.1f} V | {table[43]} °C | position {word(36)}")


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--baud", nargs="+", default=["57600", "1000000"],
                        help="vitesses à tester, ou « all » pour toutes celles des MX")
    parser.add_argument("--ids", default="0-253", help="plage d'identifiants, par ex. 1-10")
    args = parser.parse_args()

    bauds = sorted(BAUD_REGISTER.values()) if args.baud == ["all"] else [int(b) for b in args.baud]
    first, last = (int(x) for x in args.ids.split("-")) if "-" in args.ids else (int(args.ids),) * 2
    found = 0
    for baud in bauds:
        timeout = 0.012 if baud >= 200000 else 0.03
        with serial.Serial(args.port, baud, timeout=0, exclusive=True) as port:  # échoue si le bus est déjà utilisé
            echo = detect_echo(port, timeout)
            garbage = 0
            print(f"--- {baud} bauds (écho {'présent' if echo else 'absent'})", flush=True)
            for dxl_id in range(first, last + 1):
                status, rx = request(port, dxl_id, PING, (), echo, timeout)
                if status is None:
                    garbage += bool(rx) and not (echo and rx == packet(dxl_id, PING))
                    continue
                found += 1
                error, _ = status
                errors = ", ".join(name for bit, name in enumerate(ERRORS) if error >> bit & 1)
                read, _ = request(port, dxl_id, READ, (0, 50), echo, timeout * 3)
                details = describe(read[1]) if read and len(read[1]) == 50 else "lecture de la table impossible"
                print(f"  ID {dxl_id:3d} : {details}" + (f" | ERREURS : {errors}" if errors else ""), flush=True)
            if garbage:
                print(f"  ({garbage} réponses illisibles à cette vitesse)")
    print(f"{found} moteur(s) trouvé(s)")


if __name__ == "__main__":
    main()
