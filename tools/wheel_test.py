#!/usr/bin/env python3
"""Fait tourner chaque roue l'une après l'autre et vérifie que le moteur suit la consigne.

Pour chaque moteur : rotation dans un sens puis dans l'autre, à vitesse modérée, en relevant
la position (pour mesurer l'angle réellement parcouru), la vitesse et la charge. Le couple est
toujours relâché à la fin, même en cas d'erreur ou de Ctrl-C, et le chien de garde de
holorobot.motors arrête les roues si le programme s'interrompt.

À lancer roues en l'air : le robot ne doit pas pouvoir partir.

Usage : python3 tools/wheel_test.py [--ids 1 2 4 8] [--speed 60] [--duration 2]
"""
import argparse
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from holorobot.motors import Motors  # noqa: E402


def angle_step(previous, current):
    """Plus petit écart entre deux positions en degrés, mesurées dans [-180, 180]."""
    return (current - previous + 180) % 360 - 180


def spin(motors, dxl_id, speed, duration):
    """Fait tourner une roue à `speed` °/s pendant `duration` s.

    Renvoie l'angle parcouru (°, arrêt compris), la vitesse lue médiane (°/s) et la charge
    maximale (% du couple max).
    """
    last = motors.get_positions(dxl_id)[dxl_id]
    turned, speeds, loads = 0.0, [], []
    end = time.monotonic() + duration
    while time.monotonic() < end:
        motors.set_speeds({dxl_id: speed})  # renvoyée à chaque tour : le chien de garde est nourri
        position = motors.get_positions(dxl_id)[dxl_id]
        turned += angle_step(last, position)
        last = position
        speeds.append(motors.get_speeds(dxl_id)[dxl_id])
        loads.append(abs(motors.get_loads(dxl_id)[dxl_id]))
    motors.set_speeds({dxl_id: 0})
    time.sleep(0.3)
    turned += angle_step(last, motors.get_positions(dxl_id)[dxl_id])
    return turned, statistics.median(speeds), max(loads)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--baudrate", type=int, default=57600)
    parser.add_argument("--ids", type=int, nargs="+", help="moteurs à tester (par défaut : tous)")
    parser.add_argument("--speed", type=float, default=60.0, help="vitesse de la roue en °/s")
    parser.add_argument("--duration", type=float, default=2.0, help="durée de chaque rotation en s")
    args = parser.parse_args()

    with Motors(ids=args.ids, port=args.port, baudrate=args.baudrate, wheel_mode=False) as motors:
        not_wheels = [i for i in motors.ids if motors.modes[i] != "wheel"]
        if not_wheels:
            raise SystemExit(f"Moteurs pas en mode roue : {not_wheels}. Test annulé (motors.set_wheel_mode() les y met).")
        expected = args.speed * args.duration
        for dxl_id in motors.ids:
            before = motors.get_temperatures(dxl_id)[dxl_id]
            for speed in (args.speed, -args.speed):
                turned, measured, load = spin(motors, dxl_id, speed, args.duration)
                print(f"ID {dxl_id} : consigne {speed:+.0f} °/s pendant {args.duration:.1f} s -> "
                      f"{turned:+.0f}° parcourus (attendu {expected if speed > 0 else -expected:+.0f}°), "
                      f"vitesse lue {measured:+.0f} °/s, charge max {load:.0f} %", flush=True)
            print(f"      température {before:.0f} °C avant, {motors.get_temperatures(dxl_id)[dxl_id]:.0f} °C après",
                  flush=True)


if __name__ == "__main__":
    main()
