#!/usr/bin/env python3
"""Fait tourner chaque roue l'une après l'autre et vérifie que le moteur suit la consigne.

Pour chaque moteur : rotation dans un sens puis dans l'autre, à vitesse modérée, en relevant
la position (pour mesurer l'angle réellement parcouru), la vitesse et la charge. Le couple est
toujours relâché à la fin, même en cas d'erreur ou de Ctrl-C.

À lancer roues en l'air : le robot ne doit pas pouvoir partir.

Usage : python3 tools/wheel_test.py [--ids 1 2 4 8] [--speed 60] [--duration 2]
"""
import argparse
import statistics
import time

import pypot.dynamixel as dxl


def angle_step(previous, current):
    """Plus petit écart entre deux positions en degrés, mesurées dans [-180, 180]."""
    return (current - previous + 180) % 360 - 180


def spin(io, dxl_id, speed, duration):
    """Fait tourner une roue à `speed` °/s pendant `duration` s.

    Renvoie l'angle parcouru (°, arrêt compris), la vitesse lue médiane (°/s) et la charge
    maximale (% du couple max).
    """
    last, = io.get_present_position([dxl_id])
    turned, speeds, loads = 0.0, [], []
    io.set_moving_speed({dxl_id: speed})
    end = time.monotonic() + duration
    while time.monotonic() < end:
        position, = io.get_present_position([dxl_id])
        turned += angle_step(last, position)
        last = position
        speeds.append(io.get_present_speed([dxl_id])[0])
        loads.append(abs(io.get_present_load([dxl_id])[0]))
    io.set_moving_speed({dxl_id: 0})
    time.sleep(0.3)
    position, = io.get_present_position([dxl_id])
    turned += angle_step(last, position)
    return turned, statistics.median(speeds), max(loads)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--baudrate", type=int, default=57600)
    parser.add_argument("--ids", type=int, nargs="+", default=[1, 2, 4, 8])
    parser.add_argument("--speed", type=float, default=60.0, help="vitesse de la roue en °/s")
    parser.add_argument("--duration", type=float, default=2.0, help="durée de chaque rotation en s")
    args = parser.parse_args()

    io = dxl.DxlIO(args.port, baudrate=args.baudrate)
    try:
        missing = set(args.ids) - set(io.scan(args.ids))
        if missing:
            raise SystemExit(f"Moteurs absents du bus : {sorted(missing)}")
        for dxl_id, limits in zip(args.ids, io.get_angle_limit(args.ids)):
            # En mode roue, les deux limites valent 0 en brut, soit -180° pour pypot
            if limits != (-180.0, -180.0):
                raise SystemExit(f"ID {dxl_id} n'est pas en mode roue (limites {limits}) : test annulé")

        expected = args.speed * args.duration
        for dxl_id in args.ids:
            temperature, = io.get_present_temperature([dxl_id])
            io.enable_torque([dxl_id])
            for speed in (args.speed, -args.speed):
                turned, measured, load = spin(io, dxl_id, speed, args.duration)
                print(f"ID {dxl_id} : consigne {speed:+.0f} °/s pendant {args.duration:.1f} s -> "
                      f"{turned:+.0f}° parcourus (attendu {expected if speed > 0 else -expected:+.0f}°), "
                      f"vitesse lue {measured:+.0f} °/s, charge max {load:.0f} %", flush=True)
            io.disable_torque([dxl_id])
            print(f"      température {temperature:.0f} °C avant, "
                  f"{io.get_present_temperature([dxl_id])[0]:.0f} °C après", flush=True)
    finally:
        io.set_moving_speed({dxl_id: 0 for dxl_id in args.ids})
        io.disable_torque(args.ids)
        io.close()


if __name__ == "__main__":
    main()
