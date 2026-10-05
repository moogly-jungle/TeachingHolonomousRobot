#!/usr/bin/env python3
"""Supervision probe: the state of a robot, seen from the inside.

The supervision (robots.py) sends this program to the robot over SSH and runs it with the robot's
Python. It answers in JSON.
- System: temperature of the Pi, power supply, memory, disk, wifi, services, open notebooks.
- USB: are the lidar and the camera plugged in?
- Motors: those answering on the bus, only if no other program is using it.

It only reads, with a few exceptions, each asked for explicitly: the lidar spins during a lidar
scan, the camera lights its infrared projector during a capture, set-motor-id and set-motor-speed
write the new ID or the new bus speed of a motor, and spin-motors turns the motors.

It needs the standard library and pyserial; the lidar scan and spin-motors also use holorobot, and
the capture pyrealsense2, both installed on every robot. By hand, on a robot:

    python3 probe.py state                 # the state of the robot
    python3 probe.py motors                # the motors and all their characteristics
    python3 probe.py scan-motor            # the motors, at every speed of the bus
    python3 probe.py lidar                 # three turns of the lidar and their segments
    python3 probe.py camera                # an image of each stream of the camera, and what YOLO sees
    python3 probe.py set-motor-id --old 1 --new 4
    python3 probe.py set-motor-speed --id 1 --new 57600 --baud 1000000
    python3 probe.py spin-motors --speed 90 --duration 2
"""
import argparse
import base64
import io
import json
import os
import shutil
import signal
import socket
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path

MOTOR_PORT = "/dev/serial0"
MOTOR_BAUDRATE = 57600
MOTOR_IDS = range(21)  # IDs searched by default, as in holorobot.motors
SPIN_MAX_SPEED = 720  # °/s, two turns of wheel per second: the limit of holorobot.motors
SPIN_MAX_DURATION = 30  # s
LIDAR_USB_ID = "10c4:ea60"  # the USB-serial adapter (CP2102) that comes with the YDLidar X4
LIDAR_BAUDRATE = 128000
SERVICES = ("jupyterlab", "tailscaled", "funnel-watchdog")


def robot_state(motors=True, lidar=False, tailnet=False):
    """The full state of the robot. A failing measurement gives an error without blocking the others."""
    started = time.monotonic()
    usb = usb_devices()
    state = {
        "system": measure(system_state),
        "wifi": measure(wifi_state),
        "services": {name: run("systemctl", "is-active", name) for name in SERVICES},
        "kernels": measure(jupyter_kernels),
        "lidar": measure(lidar_state, usb, lidar),
        "camera": measure(camera_state, usb),
        "motors": measure(motors_state) if motors else None,
    }
    if tailnet:
        state["tailnet"] = measure(tailnet_state)
    state["duration"] = round(time.monotonic() - started, 2)
    return state


def measure(function, *args):
    """function(*args), or {"error": ...} if it fails."""
    try:
        return function(*args)
    except Exception as error:
        return {"error": f"{type(error).__name__}: {error}"}


def run(*command):
    """The output of a command, or an empty string if it fails."""
    try:
        return subprocess.run(command, capture_output=True, text=True, timeout=10).stdout.strip()
    except (OSError, subprocess.TimeoutExpired):
        return ""


def read_text(path):
    """The content of a file (from /proc or /sys), or an empty string if it does not exist."""
    try:
        return Path(path).read_text().strip()
    except OSError:
        return ""


# System and network

def system_state():
    """The Pi: temperature, power supply, load, memory and disk."""
    uptime = float(read_text("/proc/uptime").split()[0])  # seconds since boot
    return {
        "hostname": socket.gethostname(),
        "uptime": uptime,
        "load": os.getloadavg()[0],
        "temperature": int(read_text("/sys/class/thermal/thermal_zone0/temp")) / 1000,  # °C
        "power": power_flags(run("vcgencmd", "get_throttled")),
        "undervoltage_alerts": undervoltage_alerts(uptime),
        "memory_available": meminfo("MemAvailable"),
        "disk_free": shutil.disk_usage("/").free,
    }


def power_flags(answer):
    """Decodes the answer of "vcgencmd get_throttled" (for example throttled=0x50000): one bit per problem."""
    if "=" not in answer:
        return None
    flags = int(answer.split("=")[1], 16)
    return {
        "value": f"0x{flags:x}",
        "undervoltage": bool(flags & 1 << 0),  # power supply too weak right now
        "throttled": bool(flags & 1 << 2),  # processor slowed down right now
        "undervoltage_since_boot": bool(flags & 1 << 16),
        "throttled_since_boot": bool(flags & 1 << 18),
    }


def undervoltage_alerts(uptime, journal=None):
    """When the power supply of the Pi dropped too low since boot: dates in UTC, like those of Tailscale.

    A Pi 4 does not measure its supply voltage: it only detects when it drops below about 4.63 V. The
    kernel notes each of these alerts, with its time since boot. `journal`: for the tests.
    """
    if journal is None:  # this boot, times in seconds since boot
        journal = run("journalctl", "-k", "-b", "-o", "short-monotonic", "--no-pager",
                      "--grep", "Undervoltage detected")
    boot = time.time() - uptime
    seconds = [float(line[1:line.index("]")]) for line in journal.splitlines() if line.startswith("[")]
    return [datetime.fromtimestamp(boot + second, timezone.utc).isoformat() for second in seconds]


def meminfo(field):
    """A value from /proc/meminfo, in bytes."""
    for line in read_text("/proc/meminfo").splitlines():
        name, _, value = line.partition(":")
        if name == field:
            return int(value.split()[0]) * 1024
    return None


def wifi_state():
    """The wifi network, the signal strength (0 to 100) and the IP addresses of the robot."""
    output = run("nmcli", "-t", "-f", "ACTIVE,SSID,SIGNAL", "device", "wifi", "list", "--rescan", "no")
    return {**parse_wifi(output), "addresses": run("hostname", "-I").split()}


def parse_wifi(nmcli_output):
    """The active network in the output of nmcli, made of lines like "yes:DU_ROBOT_1:99"."""
    for line in nmcli_output.splitlines():
        active, _, rest = line.partition(":")
        if active == "yes":
            ssid, _, signal = rest.rpartition(":")
            return {"ssid": ssid.replace("\\:", ":"), "signal": int(signal)}
    return {"ssid": None, "signal": None}


def jupyter_kernels():
    """The number of Jupyter kernels of the etudiant account: one per running notebook."""
    return int(run("pgrep", "-c", "-u", "etudiant", "-f", "ipykernel_launcher") or 0)


def tailnet_state():
    """What Tailscale knows about the robots: online or not, and when it last saw them."""
    peers = json.loads(run("tailscale", "status", "--json") or "{}").get("Peer", {}).values()
    robots = {}
    for peer in peers:
        name = peer.get("DNSName", "").split(".")[0]
        if name.startswith("holobot"):
            robots[name] = {"online": peer.get("Online", False), "last_seen": peer.get("LastSeen")}
    return robots


# Lidar and camera: we look for them among the USB devices

def usb_devices():
    """The USB devices plugged in, according to /sys: ID, name, serial number, speed, serial ports."""
    devices = []
    for path in sorted(Path("/sys/bus/usb/devices").glob("*")):
        if not (path / "idVendor").exists():
            continue  # an interface of a device, not the device itself
        devices.append({
            "id": f"{read_text(path / 'idVendor')}:{read_text(path / 'idProduct')}",
            "name": read_text(path / "product"),
            "serial": read_text(path / "serial"),
            "speed": int(read_text(path / "speed") or 0),  # Mbit/s: 480 in USB 2, 5000 in USB 3
            "ports": sorted(tty.name for tty in path.glob("*/tty[A-Z]*")),
        })
    return devices


def lidar_state(usb, ask=False):
    """Is the lidar plugged in? With ask=True, we also ask it who it is and how it is doing."""
    adapter = next((device for device in usb if device["id"] == LIDAR_USB_ID), None)
    if adapter is None:
        return {"connected": False}
    state = {"connected": True, "port": f"/dev/{adapter['ports'][0]}" if adapter["ports"] else None}
    if ask and state["port"]:
        state.update(ask_lidar(state["port"]))
    return state


def open_lidar(port):
    """Opens the serial port of the lidar, without starting its motor, if no other program is using it."""
    import serial  # imported here: the rest of the probe does not need it

    link = serial.Serial()
    link.port, link.baudrate, link.timeout = port, LIDAR_BAUDRATE, 1.0
    link.dtr = False  # the DTR line drives the motor of the lidar: it stays off
    link.exclusive = True  # the lock of holorobot.lidar: never while another program is using it
    link.open()
    return link


def ask_lidar(port):
    """The information and health of the lidar, asked without starting its motor (YDLidar protocol)."""
    try:
        link = open_lidar(port)
    except OSError as error:
        return port_failure(error)
    with link:
        link.write(b"\xa5\x65")  # stop scanning, in case a program left the lidar running
        time.sleep(0.1)
        info = lidar_request(link, b"\xa5\x90", 20)  # model, firmware, hardware, serial number
        health = lidar_request(link, b"\xa5\x91", 3)  # 0 good, 1 warning, 2 error
    if info is None or health is None:
        return {"responds": False}
    return {"responds": True, "model": info[0], "firmware": f"{info[2]}.{info[1]}", "health": health[0]}


def lidar_scan(turns=3):
    """Three turns of the lidar: the points and segments of the fullest one, and the size of each.

    Read with holorobot.lidar, as in the notebooks; the motor of the lidar spins for about a second.
    Once in a while, a turn comes back almost empty: hence several turns. Points and segments are in
    meters, in the frame of the lidar: x toward its 0°, y toward its left.
    """
    state = lidar_state(usb_devices())
    if not state["connected"] or not state["port"]:
        return {"error": "lidar not plugged in"}
    try:
        open_lidar(state["port"]).close()  # free? holorobot would refuse it anyway, with a message in French
    except OSError as error:
        return port_failure(error)
    from holorobot.lidar import Lidar  # installed on every robot, with numpy

    with Lidar(state["port"]) as lidar:  # starts the motor, and stops it at the end
        scans = [lidar.get_scan() for _ in range(turns)]
    scan = max(scans, key=len)
    segments = [{"start": list(s.start), "end": list(s.end), "confidence": s.confidence} for s in scan.segments()]
    return {"points": scan.xy().round(3).tolist(), "segments": segments, "turns": [len(s) for s in scans]}


def port_failure(error):
    """Why a serial port could not be opened: busy with another program, or another error."""
    if "lock" in str(error).lower():
        return {"busy": True}  # a notebook uses it: we do not disturb it
    return {"error": str(error)}


def lidar_request(link, command, length):
    """Sends a command to the lidar and returns the `length` data bytes of its answer, or None.

    An answer: A5 5A, length (4 bytes), answer type (1 byte), then the data.
    """
    link.reset_input_buffer()
    link.write(command)
    received = link.read(7 + length)
    start = received.find(b"\xa5\x5a")
    if start < 0 or len(received) < start + 7 + length:
        return None
    return received[start + 7:start + 7 + length]


def camera_frames(rotation=0, yolo=True):
    """One image of each stream of the RealSense (color, depth, left infrared, 640 x 480), and the
    objects that YOLO recognizes in the color image.

    `rotation` is 180 for a camera mounted upside down: the images are turned upright first. They are
    sent as PNG (JPEG for the color) in base64; the depth is in units of `depth_scale` meters. The
    infrared projector of the camera lights up during the capture.
    """
    camera = camera_state(usb_devices())
    if not camera["connected"]:
        return {"error": "camera not plugged in"}
    import numpy as np  # imported here, like pyrealsense2: installed on the robots only
    import pyrealsense2 as rs

    fps = 30 if camera["usb3"] else 15  # USB 2 has less bandwidth
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, fps)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, fps)
    config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, fps)
    pipeline = rs.pipeline()
    try:
        profile = pipeline.start(config)
    except RuntimeError as error:  # another program uses the camera (a notebook?)
        return {"busy": True} if "busy" in str(error).lower() else {"error": str(error)}
    try:
        for _ in range(30):  # one second: the automatic exposure settles
            pipeline.wait_for_frames()
        frames = pipeline.wait_for_frames()
        color = np.asanyarray(frames.get_color_frame().get_data())
        depth = np.asanyarray(frames.get_depth_frame().get_data())
        infrared = np.asanyarray(frames.get_infrared_frame(1).get_data())
        scale = profile.get_device().first_depth_sensor().get_depth_scale()
    finally:
        pipeline.stop()
    if rotation == 180:
        color, depth, infrared = (np.ascontiguousarray(image[::-1, ::-1]) for image in (color, depth, infrared))
    frames = {"color": encode(color, "JPEG"), "depth": encode(depth), "infrared": encode(infrared),
              "depth_scale": scale, "usb3": camera["usb3"]}
    if yolo:
        frames["detections"] = measure(detect_objects, color)
    return frames


def detect_objects(rgb):
    """The objects that YOLO recognizes in a color image, as in the dashboard notebook: name, confidence, box."""
    import numpy as np
    from ultralytics import YOLO  # installed on the robots; slow to load

    model = YOLO(str(Path.home() / "yolo26n.pt"))  # downloaded there the first time
    bgr = np.ascontiguousarray(rgb[..., ::-1])  # YOLO expects blue, green, red
    result = model.predict(bgr, imgsz=640, conf=0.25, verbose=False)[0]
    return [{"name": model.names[int(box.cls)], "confidence": round(float(box.conf), 2),
             "box": [round(value) for value in box.xyxy[0].tolist()]} for box in result.boxes]


def encode(image, image_format="PNG"):
    """An image (numpy array) as a PNG or JPEG file, in base64: it fits in the JSON answer."""
    from PIL import Image  # comes with matplotlib, on the robots

    buffer = io.BytesIO()
    Image.fromarray(image).save(buffer, image_format)
    return base64.b64encode(buffer.getvalue()).decode("ascii")


def camera_state(usb):
    """Is the RealSense camera plugged in, and on a USB 3 port? On USB 2, it is throttled."""
    camera = next((device for device in usb if "RealSense" in device["name"]), None)
    if camera is None:
        return {"connected": False}
    return {"connected": True, "name": camera["name"], "serial": camera["serial"], "usb3": camera["speed"] >= 5000}


# Dynamixel motors, protocol 1.0
#
# A packet: FF FF | ID | length | instruction | parameters | checksum.
# The motor answers with a packet of the same shape, where the instruction is replaced by its alarms.

PING, READ, WRITE = 0x01, 0x02, 0x03
ID_REGISTER, BAUD_REGISTER, TORQUE_REGISTER = 3, 4, 24
# The ID and the speed are written in the permanent memory of the motor (EEPROM): it ignores the bus meanwhile
EEPROM_DELAY = 0.1  # s, as in holorobot.motors
# Model number: name, speed unit (rpm), and resolution of the position: steps for how many degrees.
# MX motors turn 360° in 4096 steps, AX motors 300° in 1024; the MX-12W is a fast MX.
MODELS = {
    360: ("MX-12W", 0.916, 4096, 360), 29: ("MX-28", 0.114, 4096, 360), 310: ("MX-64", 0.114, 4096, 360),
    320: ("MX-106", 0.114, 4096, 360), 12: ("AX-12A", 0.111, 1024, 300), 300: ("AX-12W", 0.111, 1024, 300),
    18: ("AX-18A", 0.111, 1024, 300),
}
# Possible speeds of the bus, in bauds, according to the value of the "Baud Rate" register of MX motors
BAUDRATES = {1: 1000000, 3: 500000, 4: 400000, 7: 250000, 9: 200000, 16: 115200, 34: 57600,
             103: 19200, 207: 9600, 250: 2250000, 251: 2500000, 252: 3000000}


def packet(motor_id, instruction, params=()):
    """A Dynamixel 1.0 packet, ready to send."""
    body = [motor_id, len(params) + 2, instruction, *params]
    return bytes([0xFF, 0xFF, *body, ~sum(body) & 0xFF])


def parse_status(data, motor_id):
    """The status packet of this motor in `data`: (alarms, parameters), or None."""
    start = data.find(b"\xff\xff")
    while start >= 0 and start + 5 < len(data):
        end = start + 4 + data[start + 3]  # the length byte counts the alarms, the parameters and the checksum
        body = data[start + 2:end - 1]
        if data[start + 2] == motor_id and end <= len(data) and (~sum(body) & 0xFF) == data[end - 1]:
            return data[start + 4], data[start + 5:end - 1]
        start = data.find(b"\xff\xff", start + 1)
    return None


class MotorBus:
    """The motor bus. The probe sends reads to it (PING and READ); only set-motor-id and set-motor-speed write."""

    def __init__(self, link):
        self.link = link  # an open serial port, or a fake one for the tests
        self.wait = 0.012 if link.baudrate >= 200000 else 0.03  # time to wait for an answer, in s

    @classmethod
    def open(cls, port, baudrate):
        """Opens the bus, if it is free.

        The lock (exclusive) is the one of holorobot.motors: if a notebook drives the motors, opening
        fails instead of scrambling its exchanges.
        """
        import serial

        return cls(serial.Serial(port, baudrate, exclusive=True))

    def request(self, motor_id, instruction, params=(), wait=1):
        """Sends a packet and returns the answer of the motor, (alarms, parameters), or None."""
        sent = packet(motor_id, instruction, params)
        self.link.reset_input_buffer()
        self.link.write(sent)
        self.link.timeout = self.wait * wait
        received = self.link.read(256)
        if received.startswith(sent):  # on some boards, the Pi reads back what it sends (echo)
            received = received[len(sent):]
        return parse_status(received, motor_id)

    def write(self, motor_id, register, value):
        """Writes one byte in a register of a motor."""
        return self.request(motor_id, WRITE, (register, value))

    def scan(self, ids):
        """The motors that answer, with what we read in their control table."""
        motors = []
        for motor_id in ids:
            status = self.request(motor_id, PING)
            if status is not None:
                table = self.request(motor_id, READ, (0, 50), wait=3)  # the first 50 registers
                motors.append(describe_motor(motor_id, status[0], table[1] if table else None))
        return motors

    def close(self):
        self.link.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def describe_motor(motor_id, alarms, table):
    """What we keep from a motor, according to the first 50 bytes of its control table.

    Angles are in degrees, 0 in the middle of the range; speeds in degrees per second, positive
    counterclockwise; load and torques in % of the maximal torque.
    """
    motor = {"id": motor_id, "alarms": alarms}  # one bit per alarm (voltage, overheating…)
    if table is None or len(table) < 50:
        return motor  # it answered the PING, but not the read
    number = word(table, 0)
    name, rpm, steps, degrees = MODELS.get(number, (f"model {number}", 0.114, 4096, 360))  # unknown: read as an MX

    def angle(address):
        return round((word(table, address) - steps / 2) * degrees / steps, 1)

    def speed(address):
        return round(signed(word(table, address)) * rpm * 6, 1)  # 1 rpm = 6 °/s

    cw_limit, ccw_limit = word(table, 6), word(table, 8)
    motor.update(
        model=name,
        firmware=table[2],
        baud_register=table[4],
        return_delay=table[5] * 2,  # µs before the motor answers
        mode="wheel" if cw_limit == ccw_limit == 0 else "multi-turn" if cw_limit == ccw_limit == steps - 1 else "joint",
        angle_limits=[angle(6), angle(8)],
        temperature_limit=table[11],
        voltage_limits=[table[12] / 10, table[13] / 10],
        max_torque=percent(word(table, 14)),
        status_return_level=table[16],  # 0: answers PING only, 1: also READ, 2: every instruction
        alarm_shutdown=table[18],  # the alarms that cut the torque, one bit each
        torque=bool(table[24]),
        led=bool(table[25]),
        goal_position=angle(30),
        speed_command=speed(32),  # in joint mode, 0 means "as fast as possible"
        torque_limit=percent(word(table, 34)),
        position=angle(36),
        speed=speed(38),
        load=round(signed(word(table, 40)) / 10.23),
        voltage=table[42] / 10,
        temperature=table[43],
        moving=bool(table[46]),
    )
    return motor


def word(table, address):
    """A two-byte register, low byte first."""
    return table[address] | table[address + 1] << 8


def signed(value):
    """A speed or load register: 10 bits of size, then the direction bit (set: clockwise, counted negative)."""
    return -(value & 1023) if value & 1024 else value


def percent(value):
    """A torque register (0 to 1023) in % of the maximal torque."""
    return round(value / 10.23)


def register_baudrate(register):
    """The real speed of the bus, in bauds, for a value of the "Baud Rate" register."""
    return {250: 2250000, 251: 2500000, 252: 3000000}.get(register, round(2000000 / (register + 1)))


def motors_state(port=MOTOR_PORT, baudrate=MOTOR_BAUDRATE, ids=MOTOR_IDS):
    """The motors that answer on the bus, at its usual speed."""
    try:
        with MotorBus.open(port, baudrate) as bus:
            return {"bus": "free", "baudrate": baudrate, "motors": bus.scan(ids)}
    except OSError as error:
        return bus_failure(error)


def scan_motors(baudrates, ids, port=MOTOR_PORT):
    """Searches the motors at each speed of the bus: one result per speed, as the scan goes."""
    for baudrate in baudrates:
        try:
            with MotorBus.open(port, baudrate) as bus:
                yield {"baudrate": baudrate, "motors": bus.scan(ids)}
        except OSError as error:
            yield {"baudrate": baudrate, **bus_failure(error)}
            return


def set_motor_id(old, new, baudrate=MOTOR_BAUDRATE, port=MOTOR_PORT):
    """Gives the ID `new` to the motor `old`. The motor keeps it, even without power.

    Refused if no motor answers `old`, or if `new` is already taken: two motors with the same ID
    scramble each other. The motor is freed first (torque off), as holorobot.motors does.
    """
    if not 0 <= new <= 252:
        return {"error": f"impossible ID {new}: from 0 to 252"}
    try:
        with MotorBus.open(port, baudrate) as bus:
            if bus.request(old, PING) is None:
                return {"error": f"no motor answers ID {old} at {baudrate} bauds"}
            if bus.request(new, PING) is not None:
                return {"error": f"ID {new} is already taken by another motor"}
            bus.write(old, TORQUE_REGISTER, 0)
            bus.write(old, ID_REGISTER, new)
            time.sleep(EEPROM_DELAY)
            if bus.request(new, PING) is None:
                return {"error": f"the motor does not answer its new ID {new}"}
            return {"motor": bus.scan([new])[0], "baudrate": baudrate}
    except OSError as error:
        return bus_failure(error)


def set_motor_baudrate(motor_id, new, baudrate=MOTOR_BAUDRATE, port=MOTOR_PORT):
    """Changes the speed of the bus at which a motor talks, from `baudrate` to `new` bauds.

    The motor keeps it, even without power. Refused if no motor answers `motor_id`, or if another
    motor with the same ID already answers at the new speed: they would scramble each other.
    """
    register = next((register for register, value in BAUDRATES.items() if value == new), None)
    if register is None:
        return {"error": f"impossible speed {new}: choose among {', '.join(map(str, sorted(BAUDRATES.values())))}"}
    try:
        with MotorBus.open(port, new) as bus:
            if new != baudrate and bus.request(motor_id, PING) is not None:
                return {"error": f"another motor with ID {motor_id} already answers at {new} bauds"}
        with MotorBus.open(port, baudrate) as bus:
            if bus.request(motor_id, PING) is None:
                return {"error": f"no motor answers ID {motor_id} at {baudrate} bauds"}
            bus.write(motor_id, TORQUE_REGISTER, 0)
            bus.write(motor_id, BAUD_REGISTER, register)
            time.sleep(EEPROM_DELAY)
        with MotorBus.open(port, new) as bus:  # from now on, the motor talks at its new speed
            if bus.request(motor_id, PING) is None:
                return {"error": f"the motor does not answer at its new speed, {new} bauds"}
            return {"motor": bus.scan([motor_id])[0], "baudrate": new}
    except OSError as error:
        return bus_failure(error)


def spin_motors(speed, duration, ids, report, port=MOTOR_PORT):
    """Turns the motors at `speed` °/s for `duration` seconds, then stops them and turns their torque off.

    With holorobot.motors, as in the notebooks: the motors go to wheel mode, and its watchdog stops
    them if the speed stops coming. `report` receives the measured speeds, twice a second: when the
    supervision is gone (Ctrl-C), `report` fails and the wheels stop at once. A connection lost
    without being closed goes unnoticed: the wheels then stop at the end of `duration`.
    """
    try:
        with MotorBus.open(port, MOTOR_BAUDRATE) as bus:  # is the bus free, and which motors answer?
            found = [motor_id for motor_id in ids if bus.request(motor_id, PING) is not None]
    except OSError as error:
        return report(bus_failure(error))
    if not found:
        return report({"error": f"no motor answers at {MOTOR_BAUDRATE} bauds"})
    from holorobot.motors import Motors  # installed on every robot, with pypot

    with Motors(ids=found) as motors:  # at the end, even after an error: wheels stopped, torque off, bus free
        start = time.monotonic()
        for step in range(round(duration * 10)):
            motors.set_speeds(dict.fromkeys(found, speed))  # every 0.1 s: the watchdog wants one every 0.5 s
            if step % 5 == 0:
                report({"elapsed": round(time.monotonic() - start, 1), "speeds": motors.get_speeds()})
            time.sleep(0.1)
    report({"done": True, "ids": found, "watchdog_stops": motors.watchdog_stops})


def stop_probe(signal_number, frame):
    """A signal (connection closed, robot shutting down) becomes an exception: the wheels stop on the way out."""
    raise SystemExit(128 + signal_number)


def bus_failure(error):
    """Why the bus could not be opened: busy with another program, or another error."""
    if "lock" in str(error).lower():
        return {"bus": "busy"}  # a notebook drives the motors: we do not disturb it
    return {"bus": "error", "error": str(error)}


def id_range(text):
    """A range of IDs: "0-20" gives range(0, 21), "5" gives range(5, 6)."""
    first, _, last = text.partition("-")
    return range(int(first), int(last or first) + 1)


def main():
    parser = argparse.ArgumentParser(description="Supervision probe of a robot: answers in JSON.")
    commands = parser.add_subparsers(dest="command", required=True)
    state = commands.add_parser("state", help="state of the robot")
    state.add_argument("--no-motors", action="store_true", help="do not query the motor bus")
    state.add_argument("--lidar", action="store_true", help="query the lidar, without starting its motor")
    state.add_argument("--tailnet", action="store_true", help="add what Tailscale knows about the other robots")
    scan = commands.add_parser("scan-motor", help="search the motors at several speeds of the bus")
    scan.add_argument("--bauds", type=int, nargs="+", metavar="BAUD", default=sorted(BAUDRATES.values()),
                      help="bus speeds to try (default: all of them)")
    scan.add_argument("--ids", type=id_range, default=range(254),
                      help="motor IDs to try, for example 0-20 (default: 0-253)")
    commands.add_parser("lidar", help="three turns of the lidar and their segments (the lidar spins about a second)")
    camera = commands.add_parser("camera", help="an image of each stream of the camera, and what YOLO recognizes")
    camera.add_argument("--rotation", type=int, choices=(0, 180), default=0,
                        help="180 for a camera mounted upside down")
    camera.add_argument("--no-yolo", action="store_true", help="do not run YOLO on the color image")
    motors = commands.add_parser("motors", help="the motors and all their characteristics")
    motors.add_argument("--baud", type=int, default=MOTOR_BAUDRATE,
                        help=f"speed of the bus (default: {MOTOR_BAUDRATE})")
    motors.add_argument("--ids", type=id_range, default=MOTOR_IDS, help="motor IDs to try (default: 0-20)")
    set_id = commands.add_parser("set-motor-id", help="change the ID of a motor (kept even without power)")
    set_id.add_argument("--old", type=int, required=True, help="current ID of the motor")
    set_id.add_argument("--new", type=int, required=True, help="new ID, from 0 to 252, not taken by another motor")
    set_id.add_argument("--baud", type=int, default=MOTOR_BAUDRATE,
                        help=f"speed of the bus where the motor answers (default: {MOTOR_BAUDRATE})")
    set_speed = commands.add_parser("set-motor-speed", help="change the bus speed of a motor (kept even without power)")
    set_speed.add_argument("--id", type=int, required=True, help="ID of the motor")
    set_speed.add_argument("--new", type=int, required=True, choices=sorted(BAUDRATES.values()),
                           help="new speed of the bus, in bauds")
    set_speed.add_argument("--baud", type=int, default=MOTOR_BAUDRATE,
                           help=f"speed of the bus where the motor answers now (default: {MOTOR_BAUDRATE})")
    spin = commands.add_parser("spin-motors", help="turn the motors for a few seconds, then stop them")
    spin.add_argument("--speed", type=float, required=True, help="speed of the wheels, in °/s")
    spin.add_argument("--duration", type=float, default=2, help="in seconds (default: 2)")
    spin.add_argument("--ids", type=int, nargs="+", default=list(MOTOR_IDS),
                      help="motor IDs to turn (default: those of 0 to 20 that answer)")
    args = parser.parse_args()

    # The standard output carries the answer only: what the libraries print there (YOLO does) goes
    # to the standard error instead
    output = os.fdopen(os.dup(1), "w")
    os.dup2(2, 1)

    def answer(result):
        print(json.dumps(result), file=output, flush=True)

    if args.command == "state":
        answer(robot_state(not args.no_motors, args.lidar, args.tailnet))
    elif args.command == "lidar":
        answer(measure(lidar_scan))
    elif args.command == "camera":
        answer(measure(camera_frames, args.rotation, not args.no_yolo))
    elif args.command == "motors":
        answer(motors_state(MOTOR_PORT, args.baud, args.ids))
    elif args.command == "set-motor-id":
        answer(set_motor_id(args.old, args.new, args.baud))
    elif args.command == "set-motor-speed":
        answer(set_motor_baudrate(args.id, args.new, args.baud))
    elif args.command == "spin-motors":
        signal.signal(signal.SIGHUP, stop_probe)
        signal.signal(signal.SIGTERM, stop_probe)
        spin_motors(args.speed, args.duration, args.ids, answer)
    else:
        for result in scan_motors(args.bauds, args.ids):
            answer(result)  # as the scan goes, one line per speed


if __name__ == "__main__":
    main()
