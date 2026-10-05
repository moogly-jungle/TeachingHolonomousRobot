"""Display in the terminal, with rich: tables, colors, and what the measurements mean.

The thresholds (battery, temperatures, wifi) are at the top of the file: this is where to tune them.
"""
from datetime import datetime, timezone
from statistics import median

from rich import box
from rich.console import Group
from rich.table import Table
from rich.text import Text

from .probe import MOTOR_BAUDRATE
from .robots import Unreachable

CELLS = 3  # Li-ion cells in series in the battery
# Voltage of a cell (V) -> charge (%), under a light load: same curve as tools/battery.py
CHARGE_CURVE = [(3.00, 0), (3.30, 5), (3.50, 12), (3.60, 22), (3.70, 36), (3.80, 52),
                (3.90, 68), (4.00, 82), (4.10, 92), (4.20, 100)]
CELL_LOW, CELL_CRITICAL = 3.60, 3.45  # V per cell: recharge soon, recharge now
MOTOR_WARM, MOTOR_HOT = 55, 65  # °C; an MX-12W shuts down at 70 °C
PI_WARM, PI_HOT = 70, 80  # °C; the Pi slows down from 80 °C
WIFI_WEAK, WIFI_BAD = 50, 30  # signal strength, out of 100
DISK_LOW = 2e9  # free bytes
ALARMS = ("input voltage", "angle limit", "overheating", "range", "checksum", "overload", "instruction")
LIDAR_HEALTH = ("good", "warning", "error")


# The dashboard: one line per robot

def dashboard(states, period):
    """The watch screen: the time of the last update, then the table."""
    header = Text(f"Robots of the course · {datetime.now():%H:%M:%S} · updated every {period} s · Ctrl-C to quit",
                  "dim")
    return Group(header, overview(states))


def overview(states):
    """A table, one line per robot."""
    table = Table(box=box.SIMPLE_HEAD, header_style="bold")
    for title in ("Robot", "Access", "Battery", "Motors", "Lidar", "Camera", "Pi", "Wifi", "Notebooks"):
        table.add_column(title)
    for robot, state in states.items():
        if isinstance(state, Unreachable):
            table.add_row(robot.hostname, Text(str(state), "red"))
            continue
        table.add_row(robot.hostname, state["route"], battery(state["motors"]), motors_summary(state["motors"]),
                      lidar_summary(state["lidar"]), camera_summary(state["camera"]),
                      pi_summary(state["system"], state["services"]), wifi_summary(state["wifi"]),
                      str(state["kernels"]))
    return table


def battery(motors):
    """The voltage of the battery, read by the motors, and its estimated charge."""
    voltages = [motor["voltage"] for motor in found_motors(motors) if "voltage" in motor]
    if not voltages:
        return Text("?", "dim")
    pack = median(voltages)
    cell = pack / CELLS
    return Text(f"{pack:.1f} V · {charge(cell)} %", alert_low(cell, CELL_LOW, CELL_CRITICAL))


def charge(cell_voltage):
    """Estimated charge (%) of a cell, interpolated on the discharge curve."""
    if cell_voltage <= CHARGE_CURVE[0][0]:
        return 0
    for (v0, c0), (v1, c1) in zip(CHARGE_CURVE, CHARGE_CURVE[1:]):
        if cell_voltage <= v1:
            return round(c0 + (c1 - c0) * (cell_voltage - v0) / (v1 - v0))
    return 100


def motors_summary(motors):
    """The IDs of the motors, the hottest one, and a warning if there are alarms."""
    if motors is None:
        return Text("not queried", "dim")
    if "error" in motors or motors["bus"] == "error":
        return Text("bus unreachable", "red")
    if motors["bus"] == "busy":
        return Text("bus busy", "yellow")  # a notebook drives the motors
    if not motors["motors"]:
        return Text("none", "red")
    text = Text(" ".join(str(motor["id"]) for motor in motors["motors"]))
    hottest = max(motor.get("temperature", 0) for motor in motors["motors"])
    text.append(f" · {hottest} °C", alert_high(hottest, MOTOR_WARM, MOTOR_HOT))
    if any(motor["alarms"] for motor in motors["motors"]):
        text.append(" · alarm", "red")
    return text


def lidar_summary(lidar):
    if "error" in lidar or not lidar["connected"]:
        return Text("missing", "red")
    if lidar.get("responds") is False:
        return Text("connected, silent", "red")  # the USB adapter is there, but not the lidar
    return Text("connected", "green")


def camera_summary(camera):
    if "error" in camera or not camera["connected"]:
        return Text("missing", "red")
    return Text("USB 3", "green") if camera["usb3"] else Text("USB 2: throttled", "yellow")


def pi_summary(system, services, power=True):
    """Temperature of the Pi, then what is wrong: power supply (unless `power` is False), disk, stopped services."""
    if "error" in system:
        return Text("?", "red")
    text = Text(f"{system['temperature']:.0f} °C", alert_high(system["temperature"], PI_WARM, PI_HOT))
    supply = power_summary(system)
    if power and supply.plain != "OK":
        text.append(" · ").append(supply)
    if system["disk_free"] < DISK_LOW:
        text.append(" · disk full", "red")
    for name, state in services.items():
        if state != "active":
            text.append(f" · {name} stopped", "red")
    return text


def power_summary(system, detailed=False):
    """The power supply of the Pi: too low now, too low earlier (when), or OK.

    A Pi 4 does not measure its supply voltage: it only detects drops below about 4.63 V, and the
    kernel notes the time of each one.
    """
    power, alerts = system["power"] or {}, system.get("undervoltage_alerts") or []
    if power.get("undervoltage"):
        return Text("undervoltage now", "red")
    if not alerts:
        return Text("undervoltage earlier", "yellow") if power.get("undervoltage_since_boot") else Text("OK", "green")
    if detailed:
        count = f"{len(alerts)} undervoltage alerts" if len(alerts) > 1 else "an undervoltage alert"
        return Text(f"OK now · {count} since boot, the last {when(alerts[-1])}", "yellow")
    return Text(f"undervoltage {when(alerts[-1])}" + (f" (+{len(alerts) - 1})" if len(alerts) > 1 else ""), "yellow")


def wifi_summary(wifi):
    if "error" in wifi or wifi["ssid"] is None:
        return Text("?", "dim")
    return Text(f"{wifi['ssid']} · {wifi['signal']} %", alert_low(wifi["signal"], WIFI_WEAK, WIFI_BAD))


# One robot in detail

def details(robot, state):
    """Everything about a robot: system, motors, lidar, camera."""
    system = state["system"]
    title = Text(f"{robot.hostname} · {robot.jupyter_url} · via {state['route']}", "bold")
    if "error" not in system:
        title.append(f" · up for {duration(system['uptime'])}", "dim")
    return Group(title, system_table(state), motors_table(state["motors"]), devices_table(state))


def system_table(state):
    table = Table(box=box.SIMPLE, show_header=False, title="System", title_justify="left")
    system, wifi = state["system"], state["wifi"]
    if "error" not in system:
        table.add_row("Pi", pi_summary(system, state["services"], power=False))
        value = (system["power"] or {}).get("value", "?")
        table.add_row("Power supply", Text.assemble(power_summary(system, detailed=True),
                                                    (f" · get_throttled {value}", "dim")))
        table.add_row("Load", f"{system['load']:.2f}")
        table.add_row("Free memory", size(system["memory_available"]))
        table.add_row("Free disk", size(system["disk_free"]))
    if "error" not in wifi:
        ipv4 = [address for address in wifi["addresses"] if "." in address]
        table.add_row("Wifi", Text.assemble(wifi_summary(wifi), f" · {' '.join(ipv4)}"))
    table.add_row("Services", " ".join(f"{name} {'✓' if status == 'active' else '✗'}"
                                       for name, status in state["services"].items()))
    table.add_row("Running notebooks", str(state["kernels"]))
    return table


def motors_table(motors):
    table = Table(box=box.SIMPLE_HEAD, title="Motors", title_justify="left")
    if motors is None or "error" in motors or motors["bus"] != "free":
        table.add_column("")
        table.add_row(motors_summary(motors))
        return table
    for title in ("ID", "Model", "Mode", "Torque", "Voltage", "Temperature", "Position", "Alarms"):
        table.add_column(title)
    for motor in motors["motors"]:
        table.add_row(*motor_row(motor))
    if not motors["motors"]:
        table.add_row(Text(f"no motor at {motors['baudrate']} bauds: try the scan-motor command", "red"))
    return table


def motor_row(motor):
    """The cells of a motor in a table; a motor that only answered the PING fills part of them."""
    if "model" not in motor:
        return str(motor["id"]), Text("answers the PING, not the read", "yellow")
    return (str(motor["id"]), motor["model"], motor["mode"], "on" if motor["torque"] else "off",
            f"{motor['voltage']:.1f} V",
            Text(f"{motor['temperature']} °C", alert_high(motor["temperature"], MOTOR_WARM, MOTOR_HOT)),
            f"{motor['position']:.0f}°", alarms(motor["alarms"]))


def devices_table(state):
    table = Table(box=box.SIMPLE, show_header=False, title="Lidar and camera", title_justify="left")
    lidar, camera = state["lidar"], state["camera"]
    lidar_text = lidar_summary(lidar)
    if lidar.get("connected"):
        lidar_text.append(f" · {lidar['port']}")
    if lidar.get("busy"):
        lidar_text.append(" · used by a program", "yellow")
    if lidar.get("responds"):
        lidar_text.append(f" · model {lidar['model']}, firmware {lidar['firmware']}, "
                          f"health {LIDAR_HEALTH[lidar['health']]}")
    table.add_row("Lidar", lidar_text)
    camera_text = camera_summary(camera)
    if camera.get("connected"):
        camera_text.append(f" · {camera['name']} · serial {camera['serial']}")
    table.add_row("Camera", camera_text)
    return table


# The list of robots

def robot_list(states, jupyter):
    """Who is online, since when, through which route; for the others, when Tailscale last saw them."""
    seen = {}  # what the online robots know about the others, through Tailscale
    for state in states.values():
        tailnet = {} if isinstance(state, Unreachable) else state.get("tailnet", {})
        if "error" not in tailnet:
            seen.update(tailnet)
    table = Table(box=box.SIMPLE_HEAD, header_style="bold")
    for title in ("Robot", "State", "Since", "Access", "JupyterLab"):
        table.add_column(title)
    for robot, state in states.items():
        notebook = Text("responds", "green") if jupyter[robot] else Text("does not respond", "red")
        if isinstance(state, Unreachable):
            last = seen.get(robot.tailscale_name, {}).get("last_seen")
            table.add_row(robot.hostname, Text(state.reason, "red"), f"seen {when(last)}" if last else "?",
                          Text(state.route, "dim"), notebook)
        else:
            table.add_row(robot.hostname, Text("online", "green"), duration(state["system"]["uptime"]),
                          state["route"], notebook)
    return table


# The lidar

def lidar_turns(turns):
    """The number of points of each turn read: one of them can come back almost empty."""
    text = Text(f"Turns read: {', '.join(map(str, turns))} points", "dim")
    if min(turns) < max(turns) / 2:
        text.append(" · a turn came back almost empty: the image shows the fullest one", "yellow")
    return text


# The camera

def detections_summary(detections):
    """The objects recognized by YOLO, with their confidence."""
    if detections is None:
        return Text("YOLO: not run", "dim")
    if isinstance(detections, dict):  # the probe could not run YOLO
        return Text(f"YOLO: {detections['error']}", "red")
    if not detections:
        return Text("YOLO: no object recognized", "dim")
    return Text("YOLO: " + ", ".join(f"{d['name']} {d['confidence']:.0%}" for d in detections))


# The motor scan

def scan_step(result):
    """One line per speed tried, as the scan goes."""
    line = Text(f"{result['baudrate']:>8} bauds: ")
    if result.get("bus") == "busy":
        return line.append("bus busy with another program, scan stopped", "yellow")
    if result.get("bus") == "error":
        return line.append(f"bus unreachable ({result['error']})", "red")
    if not result["motors"]:
        return line.append("nothing", "dim")
    ids = ", ".join(str(motor["id"]) for motor in result["motors"])
    return line.append(f"{len(result['motors'])} motor(s): {ids}", "green")


def scan_summary(results, usual_baudrate=MOTOR_BAUDRATE):
    """The motors found, all speeds together, and what to make of it."""
    found = [(result["baudrate"], motor) for result in results for motor in result.get("motors", [])]
    if not found:
        return Text("No motor found: are they powered (battery) and plugged in?", "red")
    table = Table(box=box.SIMPLE_HEAD, title="Motors found", title_justify="left")
    for title in ("Speed", "ID", "Model", "Mode", "Torque", "Voltage", "Temperature", "Position", "Alarms"):
        table.add_column(title)
    for baudrate, motor in found:
        table.add_row(f"{baudrate} bauds", *motor_row(motor))
    elsewhere = sorted({baudrate for baudrate, _ in found if baudrate != usual_baudrate})
    if not elsewhere:
        return table
    advice = Text(f"Some motors are not at {usual_baudrate} bauds, the speed of the robot: holorobot does not "
                  "see them. Set their speed with the set-motor-speed command.", "yellow")
    return Group(table, advice)


# Shutting robots down

def shutdown_preview(states):
    """What a shutdown would do: the robots online, with their running notebooks; the others are left alone."""
    lines = []
    for robot, state in states.items():
        if isinstance(state, Unreachable):
            lines.append(Text(f"{robot.hostname}: {state}, nothing to do", "dim"))
        elif state["kernels"]:
            lines.append(Text(f"{robot.hostname}: online, {state['kernels']} running notebook(s): "
                              "unsaved work would be lost", "yellow"))
        else:
            lines.append(Text(f"{robot.hostname}: online, no running notebook", "green"))
    return Group(*lines)


# Turning the motors

def spin_step(result):
    """One line of spin-motors: the speed measured on each motor, or why they could not turn."""
    if result.get("bus") == "busy":
        return Text("Bus busy with another program (a notebook?): nothing turned.", "yellow")
    if "error" in result:
        return Text(f"{result['error']}: nothing turned.", "red")
    if result.get("done"):
        text = Text("Wheels stopped, torque off.", "green")
        if result["watchdog_stops"]:
            text.append(f" The watchdog stopped them {result['watchdog_stops']} time(s) on the way.", "yellow")
        return text
    speeds = "   ".join(f"ID {motor}: {speed:+4.0f} °/s" for motor, speed in result["speeds"].items())
    return Text(f"{result['elapsed']:4.1f} s   {speeds}")


# Changing the ID of a motor

def id_change(old, new, result):
    """The motor under its new ID, or why the change was refused."""
    done = Text(f"Motor {old} is now motor {new}.", "green")
    if new > 20:
        done.append(" holorobot.motors looks for IDs 0 to 20 by default: use Motors(ids=[...]).", "yellow")
    return motor_change(result, done)


def speed_change(motor_id, result):
    """The motor at its new bus speed, or why the change was refused."""
    done = Text(f"Motor {motor_id} now talks at {result.get('baudrate')} bauds.", "green")
    if result.get("baudrate") != MOTOR_BAUDRATE:
        done.append(f" holorobot.motors talks at {MOTOR_BAUDRATE} bauds: it no longer sees this motor.", "yellow")
    return motor_change(result, done)


def motor_change(result, done):
    """The motor after a change of ID or speed, with the message `done`; or why nothing changed."""
    if result.get("bus") == "busy":
        return Text("Bus busy with another program (a notebook?): nothing changed.", "yellow")
    if "error" in result:
        return Text(f"Nothing changed: {result['error']}.", "red")
    table = Table(box=box.SIMPLE_HEAD)
    for title in ("ID", "Model", "Mode", "Torque", "Voltage", "Temperature", "Position", "Alarms"):
        table.add_column(title)
    table.add_row(*motor_row(result["motor"]))
    return Group(table, done)


# All the characteristics of the motors

DETAILS = ("Model", "Mode", "Angle limits", "Torque", "Position", "Goal position", "Speed", "Speed command", "Load",
           "Voltage", "Temperature", "Torque limit", "Moving", "Alarms", "Torque cut by",
           "Return delay", "Answers", "LED")
ANSWERS = ("PING only", "PING and READ", "every instruction")  # the "Status Return Level" of the motor


def motors_details(motors):
    """The motors as a table: one column per motor, one row per characteristic."""
    if motors.get("bus") != "free":
        return motors_summary(motors)
    if not motors["motors"]:
        return Text(f"No motor answers at {motors['baudrate']} bauds: try the scan-motor command.", "red")
    table = Table(box=box.SIMPLE_HEAD, title=f"Motors at {motors['baudrate']} bauds", title_justify="left")
    table.add_column("")
    for motor in motors["motors"]:
        table.add_column(f"ID {motor['id']}")
    columns = [motor_details(motor) for motor in motors["motors"]]
    for label, *cells in zip(DETAILS, *columns):  # one row per characteristic
        table.add_row(label, *cells)
    return table


def motor_details(motor):
    """The cells of one motor, in the order of DETAILS."""
    if "model" not in motor:
        return [Text("answers the PING, not the read", "yellow")] + [""] * (len(DETAILS) - 1)
    joint = motor["mode"] == "joint"
    first, last = motor["angle_limits"]
    low, high = motor["voltage_limits"]
    in_range = low <= motor["voltage"] <= high
    return [
        f"{motor['model']}, firmware {motor['firmware']}",
        motor["mode"],
        f"{first:.0f}° to {last:.0f}°" if joint else "none",
        "on" if motor["torque"] else "off",
        f"{motor['position']:.1f}°",
        f"{motor['goal_position']:.1f}°" if joint else "-",
        f"{motor['speed']:.0f} °/s",
        "maximum" if joint and motor["speed_command"] == 0 else f"{motor['speed_command']:.0f} °/s",
        f"{motor['load']} %",
        Text(f"{motor['voltage']:.1f} V (limits {low:g}-{high:g} V)", "green" if in_range else "red"),
        Text(f"{motor['temperature']} °C (limit {motor['temperature_limit']} °C)",
             alert_high(motor["temperature"], MOTOR_WARM, MOTOR_HOT)),
        Text(f"{motor['torque_limit']} % (maximum {motor['max_torque']} %)",
             "green" if motor["torque_limit"] == 100 else "yellow"),  # 0 % after an alarm that cuts the torque
        "yes" if motor["moving"] else "no",
        alarms(motor["alarms"]),
        alarm_names(motor["alarm_shutdown"]),
        f"{motor['return_delay']} µs",
        ANSWERS[motor["status_return_level"]] if motor["status_return_level"] < len(ANSWERS) else "?",
        "on" if motor["led"] else "off",
    ]


# Small formatting tools

def found_motors(motors):
    """The motors read on the bus, or an empty list if the bus could not be queried."""
    if motors is None or "error" in motors or motors["bus"] != "free":
        return []
    return motors["motors"]


def alert_high(value, warning, alert):
    """The color of a measurement that is worrying when it rises (temperature)."""
    return "red" if value >= alert else "yellow" if value >= warning else "green"


def alert_low(value, warning, alert):
    """The color of a measurement that is worrying when it falls (battery, wifi)."""
    return "red" if value < alert else "yellow" if value < warning else "green"


def alarm_names(bits):
    """The names of the alarms whose bits are set, or "none"."""
    return ", ".join(name for bit, name in enumerate(ALARMS) if bits >> bit & 1) or "none"


def alarms(bits):
    """The alarms of a motor right now: in red, if there are any."""
    names = alarm_names(bits)
    return Text(names, "dim" if names == "none" else "red")


def duration(seconds):
    """A readable duration: "3 d 4 h", "2 h 05", "12 min"."""
    minutes = int(seconds // 60)
    days, hours = divmod(minutes // 60, 24)
    if days:
        return f"{days} d {hours} h"
    if hours:
        return f"{hours} h {minutes % 60:02d}"
    return f"{minutes} min"


def when(timestamp):
    """A Tailscale date (UTC), in local time: "at 07:00" today, "on 04 Oct at 20:20" otherwise."""
    moment = datetime.strptime(timestamp[:19], "%Y-%m-%dT%H:%M:%S").replace(tzinfo=timezone.utc).astimezone()
    if moment.date() == datetime.now().date():
        return f"at {moment:%H:%M}"
    return f"on {moment:%d %b} at {moment:%H:%M}"


def size(octets):
    return "?" if octets is None else f"{octets / 1e9:.1f} GB"
