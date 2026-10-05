"""Tests of the supervision, without a robot: motor protocol, decoding, SSH commands, display.

They need the environment of the supervision (rich, PyYAML, matplotlib):
    .venv/bin/python tests/test_supervision.py       (bin/supervision creates .venv the first time)
"""
import base64
import io
import os
import sys
import tempfile
from contextlib import contextmanager, redirect_stderr, redirect_stdout
from datetime import datetime, timedelta, timezone
from pathlib import Path
from unittest import mock

os.environ.setdefault("MPLBACKEND", "Agg")  # no window during the tests
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402
from rich.console import Console  # noqa: E402

from supervision import camera_image, cli, display, lidar_image, probe  # noqa: E402
from supervision.robots import Robot, Unreachable, explain, load_robots  # noqa: E402


def status_packet(motor_id, alarms, params=b""):
    """The answer of a motor: FF FF, ID, length, alarms, parameters, checksum."""
    body = [motor_id, len(params) + 2, alarms, *params]
    return bytes([0xFF, 0xFF, *body, ~sum(body) & 0xFF])


def mx12_table(motor_id, voltage=116, temperature=45, wheel=True):
    """The first 50 registers of an MX-12W at rest, in the middle of its range, as it comes from the factory."""
    table = bytearray(50)
    table[0:2] = (360).to_bytes(2, "little")
    table[2], table[3], table[4], table[5] = 36, motor_id, 33, 250  # firmware, ID, speed, return delay
    if not wheel:
        table[8:10] = (4095).to_bytes(2, "little")  # angle limit: joint mode
    table[11], table[12], table[13] = 70, 60, 160  # temperature limit, voltage limits
    table[14:16] = table[34:36] = (1023).to_bytes(2, "little")  # maximal torque, torque limit: 100 %
    table[16], table[18] = 2, 0b100100  # answers every instruction; overheating and overload cut the torque
    table[36:38] = (2048).to_bytes(2, "little")
    table[42], table[43] = voltage, temperature
    return bytes(table)


class FakeLink:
    """A fake serial bus at one speed: the motors of `buses[baudrate]` answer PING, READ and WRITE.

    `buses` gives the motors at each speed, {baudrate: {ID: control table}}: a motor that changes its
    speed moves from one to the other.
    """

    def __init__(self, buses, baudrate=57600, echo=False):
        self.buses, self.baudrate, self.echo = buses, baudrate, echo
        self.tables = buses.setdefault(baudrate, {})
        self.timeout = None
        self.pending = b""

    def reset_input_buffer(self):
        self.pending = b""

    def write(self, data):
        motor_id, instruction = data[2], data[4]
        self.pending = data if self.echo else b""
        if motor_id not in self.tables:
            return
        if instruction == probe.WRITE and data[5] == probe.ID_REGISTER:
            self.tables[data[6]] = self.tables.pop(motor_id)  # from now on, the motor answers its new ID
            return
        if instruction == probe.WRITE and data[5] == probe.BAUD_REGISTER:
            self.buses.setdefault(probe.BAUDRATES[data[6]], {})[motor_id] = self.tables.pop(motor_id)
            return
        params = self.tables[motor_id][data[5]:data[5] + data[6]] if instruction == probe.READ else b""
        self.pending += status_packet(motor_id, 0, params)

    def read(self, size):
        data, self.pending = self.pending[:size], self.pending[size:]
        return data

    def close(self):
        pass


@contextmanager
def fake_bus(buses):
    """Replaces the real bus with fake ones, one per speed, while the block runs."""
    original = probe.MotorBus.open
    probe.MotorBus.open = classmethod(lambda cls, port, baudrate: cls(FakeLink(buses, baudrate)))
    try:
        yield
    finally:
        probe.MotorBus.open = original


class FakeMotors:
    """Imitates holorobot.motors.Motors: records the speeds it receives, and whether it was closed."""

    def __init__(self, ids):
        self.ids, self.speeds, self.closed, self.watchdog_stops = list(ids), [], False, 0
        FakeMotors.last = self

    def set_speeds(self, speeds):
        self.speeds.append(speeds)

    def get_speeds(self):
        return {motor_id: 88.0 for motor_id in self.ids}

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.closed = True  # Motors.close(): wheels stopped, torque off


def render(renderable):
    """The text rich would display, without colors."""
    console = Console(file=io.StringIO(), width=200, color_system=None)
    console.print(renderable)
    return console.file.getvalue()


def robot(number=1):
    return Robot(number, f"MobileRobot-{number}", f"holobot{number}", "100.1.2.3", "tail0000.ts.net", 10000, "admin")


def state(motors):
    """The state of a robot, as the probe returns it."""
    return {
        "route": "funnel",
        "system": {"hostname": "MobileRobot-1", "uptime": 5400, "load": 0.1, "temperature": 47.2,
                   "power": probe.power_flags("throttled=0x0"), "memory_available": 3.5e9, "disk_free": 4.6e9},
        "wifi": {"ssid": "DU_ROBOT_1", "signal": 99, "addresses": ["192.168.9.100", "fd7a::1"]},
        "services": {"jupyterlab": "active", "tailscaled": "active", "funnel-watchdog": "inactive"},
        "kernels": 2,
        "lidar": {"connected": True, "port": "/dev/ttyUSB0"},
        "camera": {"connected": True, "name": "Intel(R) RealSense(TM) Depth Camera 435i", "serial": "1", "usb3": False},
        "motors": motors,
    }


# The probe: Dynamixel protocol

def test_packet_checksum():
    assert probe.packet(1, probe.PING) == bytes([0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB])
    assert probe.packet(4, probe.READ, (0, 50)) == bytes([0xFF, 0xFF, 0x04, 0x04, 0x02, 0x00, 0x32, 0xC3])


def test_parse_status_finds_the_right_motor():
    answer = status_packet(4, 0b100, b"\x2a")
    assert probe.parse_status(b"\x00\x13" + answer, 4) == (0b100, b"\x2a")  # stray bytes before
    assert probe.parse_status(answer, 1) is None  # another motor
    assert probe.parse_status(answer[:-1] + b"\x00", 4) is None  # wrong checksum
    assert probe.parse_status(answer[:4], 4) is None  # cut packet


def test_scan_reads_the_motors_with_or_without_echo():
    tables = {1: mx12_table(1), 4: mx12_table(4, voltage=112, temperature=61, wheel=False)}
    for echo in (False, True):
        motors = probe.MotorBus(FakeLink({57600: tables}, echo=echo)).scan(range(10))
        assert [motor["id"] for motor in motors] == [1, 4]
        first, second = motors
        assert first["model"] == "MX-12W" and first["mode"] == "wheel" and first["baud_register"] == 33
        assert first["voltage"] == 11.6 and first["position"] == 0 and first["alarms"] == 0
        assert second["mode"] == "joint" and second["temperature"] == 61


def test_waiting_time_depends_on_the_speed():
    assert probe.MotorBus(FakeLink({}, baudrate=57600)).wait > probe.MotorBus(FakeLink({}, baudrate=1000000)).wait


def test_describe_motor_decodes_the_control_table():
    table = bytearray(mx12_table(1))
    table[38:40] = (1024 + 100).to_bytes(2, "little")  # 100 steps of speed, clockwise
    table[40:42] = (512).to_bytes(2, "little")  # half of the torque, counterclockwise
    motor = probe.describe_motor(1, 0, bytes(table))
    assert motor["speed"] == round(-100 * 0.916 * 6, 1) and motor["load"] == 50 and motor["return_delay"] == 500
    assert motor["temperature_limit"] == 70 and motor["voltage_limits"] == [6.0, 16.0]
    assert motor["torque_limit"] == motor["max_torque"] == 100 and motor["alarm_shutdown"] == 0b100100
    table[6:10] = (4095).to_bytes(2, "little") * 2
    assert probe.describe_motor(1, 0, bytes(table))["mode"] == "multi-turn"
    assert probe.register_baudrate(1) == 1000000 and probe.register_baudrate(33) == 58824


def test_set_motor_id():
    buses = {57600: {1: mx12_table(1), 2: mx12_table(2)}}
    with fake_bus(buses):
        assert probe.set_motor_id(1, 4)["motor"]["id"] == 4 and sorted(buses[57600]) == [2, 4]
        assert "already taken" in probe.set_motor_id(4, 2)["error"]
        assert "no motor answers ID 7" in probe.set_motor_id(7, 9)["error"]
        assert "impossible ID" in probe.set_motor_id(2, 300)["error"]


def test_set_motor_baudrate():
    buses = {1000000: {1: mx12_table(1)}, 57600: {2: mx12_table(2)}}  # a new motor, at its factory speed
    with fake_bus(buses):
        assert probe.set_motor_baudrate(1, 57600, baudrate=1000000)["baudrate"] == 57600
        assert sorted(buses[57600]) == [1, 2] and not buses[1000000]
        buses[1000000][2] = mx12_table(2)  # another motor 2, which would collide with the first one
        assert "already answers" in probe.set_motor_baudrate(2, 57600, baudrate=1000000)["error"]
        assert "impossible speed" in probe.set_motor_baudrate(1, 12345)["error"]


def test_busy_bus_and_busy_port():
    locked = OSError("Could not exclusively lock port /dev/serial0")
    assert probe.bus_failure(locked) == {"bus": "busy"} and probe.port_failure(locked) == {"busy": True}
    assert probe.bus_failure(OSError("No such file")) == {"bus": "error", "error": "No such file"}


def test_spin_motors_stops_the_wheels_at_the_end():
    reports = []
    with fake_bus({57600: {1: mx12_table(1), 4: mx12_table(4)}}), \
            mock.patch.dict(sys.modules, {"holorobot.motors": mock.Mock(Motors=FakeMotors)}):
        probe.spin_motors(90, 0.6, range(21), reports.append)
    motors = FakeMotors.last
    assert motors.ids == [1, 4] and motors.closed and len(motors.speeds) == 6
    assert all(speeds == {1: 90, 4: 90} for speeds in motors.speeds)
    assert [report["speeds"] for report in reports[:2]] == [{1: 88.0, 4: 88.0}] * 2  # at 0 s and 0.5 s
    assert reports[-1] == {"done": True, "ids": [1, 4], "watchdog_stops": 0}


def test_spin_motors_stops_the_wheels_when_the_supervision_is_gone():
    def report(result):
        raise BrokenPipeError  # Ctrl-C or connection lost: the answer can no longer be sent

    with fake_bus({57600: {1: mx12_table(1)}}), \
            mock.patch.dict(sys.modules, {"holorobot.motors": mock.Mock(Motors=FakeMotors)}):
        try:
            probe.spin_motors(90, 10, range(21), report)
            raise AssertionError("the probe should stop at once")
        except BrokenPipeError:
            pass
    assert FakeMotors.last.closed and len(FakeMotors.last.speeds) == 1


def test_spin_motors_refusals():
    reports = []
    with fake_bus({57600: {}}):
        probe.spin_motors(90, 1, range(21), reports.append)
    assert reports == [{"error": "no motor answers at 57600 bauds"}]
    assert "nothing turned" in render(display.spin_step(reports[0]))
    assert "Bus busy" in render(display.spin_step({"bus": "busy"}))
    assert render(display.spin_step({"elapsed": 0.5, "speeds": {"1": 88.0}})).strip() == "0.5 s   ID 1:  +88 °/s"
    parser = cli.build_parser()
    assert parser.parse_args(["spin-motors", "2", "-90"]).speed == -90  # a negative speed is not an option
    for wrong in (["spin-motors", "2", "900"], ["spin-motors", "2", "90", "--duration", "60"]):
        with redirect_stderr(io.StringIO()):
            try:
                parser.parse_args(wrong)
                raise AssertionError(f"accepted: {wrong}")
            except SystemExit:
                pass


# The probe: system, USB

def test_power_flags():
    assert probe.power_flags("throttled=0x50005") == {"value": "0x50005", "undervoltage": True, "throttled": True,
                                                      "undervoltage_since_boot": True, "throttled_since_boot": True}
    flags = probe.power_flags("throttled=0x0")
    assert flags.pop("value") == "0x0" and not any(flags.values())
    assert probe.power_flags("") is None  # no vcgencmd: this is not a Pi


def test_undervoltage_alerts_are_dated():
    journal = ("[    8.096042] MobileRobot-2 kernel: hwmon hwmon1: Undervoltage detected!\n"
               "[ 2602.688085] MobileRobot-2 kernel: hwmon hwmon1: Undervoltage detected!\n")
    first, last = probe.undervoltage_alerts(2816.0, journal)
    gap = datetime.fromisoformat(last) - datetime.fromisoformat(first)
    assert abs(gap.total_seconds() - 2594.592043) < 1e-3
    assert probe.undervoltage_alerts(10.0, "-- No entries --") == []
    now = datetime.now(timezone.utc)
    system = {"power": probe.power_flags("throttled=0x50000"),
              "undervoltage_alerts": [(now - timedelta(hours=1)).isoformat(), now.isoformat()]}
    at = f"at {now.astimezone():%H:%M}"
    assert render(display.power_summary(system)).strip() == f"undervoltage {at} (+1)"
    assert f"OK now · 2 undervoltage alerts since boot, the last {at}" in render(display.power_summary(system, True))
    assert "undervoltage now" in render(display.power_summary({"power": probe.power_flags("throttled=0x50005")}))
    assert render(display.power_summary({"power": probe.power_flags("throttled=0x0")})).strip() == "OK"


def test_parse_wifi():
    assert probe.parse_wifi("no:RHOBAN:40\nyes:DU_ROBOT_1:99") == {"ssid": "DU_ROBOT_1", "signal": 99}
    assert probe.parse_wifi("yes:my\\:network:55")["ssid"] == "my:network"
    assert probe.parse_wifi("") == {"ssid": None, "signal": None}


def test_lidar_and_camera_in_the_usb_list():
    usb = [{"id": "10c4:ea60", "name": "CP2102 USB to UART Bridge Controller", "serial": "0001", "speed": 12,
            "ports": ["ttyUSB0"]},
           {"id": "8086:0b3a", "name": "Intel(R) RealSense(TM) Depth Camera 435i", "serial": "42", "speed": 480,
            "ports": []}]
    assert probe.lidar_state(usb) == {"connected": True, "port": "/dev/ttyUSB0"}
    assert probe.camera_state(usb)["usb3"] is False
    assert probe.lidar_state([]) == {"connected": False} and probe.camera_state([]) == {"connected": False}


def test_id_range():
    assert probe.id_range("0-20") == range(0, 21) and probe.id_range("5") == range(5, 6)


# The robots: configuration and SSH commands

def test_ssh_command_for_each_route():
    by_funnel = robot().ssh_command(robot().route("funnel"), "scan-motor", "--ids", "0-20")
    assert "admin@holobot1.tail0000.ts.net" in by_funnel and "10000" in by_funnel
    assert any(part.startswith("ProxyCommand=") and "tunnel.py" in part for part in by_funnel)
    assert "HostKeyAlias=MobileRobot-1" in by_funnel
    assert by_funnel[-1].endswith("scan-motor --ids 0-20")
    by_network = robot().ssh_command(robot().route("local"), "state")
    assert "admin@MobileRobot-1.local" in by_network and "ProxyCommand" not in " ".join(by_network)


def test_shutdown_runs_in_the_terminal_after_a_confirmation():
    command = robot().terminal_command(robot().route("local"), cli.SHUTDOWN)
    assert command[:2] == ["ssh", "-t"] and command[-2:] == ["admin@MobileRobot-1.local", cli.SHUTDOWN]
    for answer, expected in (("", False), ("n", False), ("no", False), ("y", True), (" Yes ", True)):
        with mock.patch("builtins.input", return_value=answer):
            assert cli.confirm("Shut down?") is expected
    states = {robot(1): {"kernels": 2}, robot(2): {"kernels": 0}, robot(3): Unreachable("offline", "funnel")}
    text = render(display.shutdown_preview(states))
    assert "MobileRobot-1: online, 2 running notebook(s)" in text and "MobileRobot-2: online, no running" in text
    assert "MobileRobot-3: offline (funnel), nothing to do" in text


def test_load_robots():
    config = """
tailscale: {tailnet: tail0000.ts.net, funnel_ssh_port: 10000}
ssh: {user: admin, password: secret}
robots:
  1: {hostname: MobileRobot-1, tailscale: holobot1, tailscale_ip: 100.1.1.1}
  2: {hostname: MobileRobot-2, tailscale: holobot2, tailscale_ip: 100.1.1.2}
"""
    with tempfile.TemporaryDirectory() as folder:
        path = Path(folder) / "robots.yaml"
        path.write_text(config, encoding="utf-8")
        assert [r.number for r in load_robots(path=path)] == [1, 2]
        second = load_robots([2], path=path)[0]
        assert second.public_name == "holobot2.tail0000.ts.net" and second.tailscale_ip == "100.1.1.2"
        try:
            load_robots([7], path=path)
        except SystemExit as error:
            assert "7" in str(error)
        else:
            raise AssertionError("an unknown robot must be refused")


def test_the_template_has_the_expected_keys():
    template = Path(__file__).resolve().parent.parent / "supervision" / "robots.example.yaml"
    assert [robot.hostname for robot in load_robots(path=template)] == ["MobileRobot-1"]


def test_explain_ssh_errors():
    assert explain("admin@x: Permission denied (publickey).") == "SSH key refused"
    assert explain("Connecting to 1.2.3.4\nConnection closed by UNKNOWN port 65535") == "offline"
    assert explain("") == "cannot connect"
    assert str(Unreachable("offline", "funnel")) == "offline (funnel)"


# The display

def test_battery_charge():
    assert display.charge(3.7) == 36 and display.charge(3.65) == 29
    assert display.charge(2.9) == 0 and display.charge(4.3) == 100


def test_colors_follow_the_thresholds():
    assert display.alert_high(40, 55, 65) == "green" and display.alert_high(60, 55, 65) == "yellow"
    assert display.alert_low(3.5, 3.60, 3.45) == "yellow" and display.alert_low(3.4, 3.60, 3.45) == "red"


def test_overview():
    motors = {"bus": "free", "baudrate": 57600,
              "motors": [probe.describe_motor(i, 0, mx12_table(i)) for i in (1, 2, 4, 8)]}
    states = {robot(1): state(motors), robot(2): state({"bus": "busy"}),
              robot(3): Unreachable("offline", "funnel")}
    text = render(display.overview(states))
    assert "11.6 V · 63 %" in text and "1 2 4 8 · 45 °C" in text
    assert "bus busy" in text and "offline (funnel)" in text
    assert "USB 2: throttled" in text and "funnel-watchdog stopped" in text


def test_details():
    motors = {"bus": "free", "baudrate": 57600, "motors": [{"id": 3, "alarms": 0}]}  # only answered the PING
    text = render(display.details(robot(1), state(motors)))
    assert "answers the PING, not the read" in text and "192.168.9.100" in text and "fd7a" not in text


def test_scan_summary_advises_when_a_motor_is_elsewhere():
    found = [{"baudrate": 57600, "motors": []},
             {"baudrate": 1000000, "motors": [probe.describe_motor(1, 0, mx12_table(1))]}]
    text = render(display.scan_summary(found))
    assert "1000000 bauds" in text and "register" not in text and "set-motor-speed" in text
    assert "No motor found" in render(display.scan_summary([{"baudrate": 57600, "motors": []}]))


def test_lidar_image_is_drawn():
    scan = {"points": [[1.0, y / 10] for y in range(-10, 11)] + [[0.5, 1.5], [-2.0, 0.3]],
            "segments": [{"start": [1.0, -1.0], "end": [1.0, 1.0], "confidence": 0.9}]}
    with tempfile.TemporaryDirectory() as folder:
        path = Path(folder) / "lidar.png"
        plt.close(lidar_image.draw(scan, "test", path))
        assert path.read_bytes()[:8] == b"\x89PNG\r\n\x1a\n"
    assert "almost empty" in render(display.lidar_turns([385, 25, 384]))
    assert "almost empty" not in render(display.lidar_turns([385, 390, 384]))


def test_an_empty_lidar_scan_is_explained():
    empty = {"points": [], "segments": [], "turns": [0, 0, 0], "route": "local"}
    args = cli.build_parser().parse_args(["check-lidar", "2"])
    with mock.patch.object(cli, "load_robots", return_value=[robot(2)]), \
            mock.patch.object(cli, "probe_or_exit", return_value=empty), redirect_stdout(io.StringIO()):
        try:
            args.function(args)
            raise AssertionError("an empty scan should end the command")
        except SystemExit as error:
            assert "measured nothing" in str(error)


def encoded(image, image_format="PNG"):
    """An image (numpy array) as the probe sends it: a PNG or JPEG file, in base64."""
    buffer = io.BytesIO()
    Image.fromarray(image).save(buffer, image_format)
    return base64.b64encode(buffer.getvalue()).decode("ascii")


def test_camera_image_is_drawn():
    depth = np.zeros((48, 64), dtype=np.uint16)
    depth[:, 32:] = 1500  # 1.5 m on the right half, no measurement on the left half
    frames = {"color": encoded(np.full((48, 64, 3), 120, dtype=np.uint8), "JPEG"), "depth": encoded(depth),
              "infrared": encoded(np.full((48, 64), 80, dtype=np.uint8)), "depth_scale": 0.001}
    assert camera_image.depth_summary(frames) == (0.5, 1.5)
    frames["detections"] = [{"name": "chair", "confidence": 0.87, "box": [10, 5, 30, 40]}]
    with tempfile.TemporaryDirectory() as folder:
        path = Path(folder) / "camera.png"
        plt.close(camera_image.draw(frames, "test", path))
        assert path.read_bytes()[:8] == b"\x89PNG\r\n\x1a\n"
    assert render(display.detections_summary(frames["detections"])).strip() == "YOLO: chair 87%"
    assert "no object" in render(display.detections_summary([]))
    assert "not run" in render(display.detections_summary(None))
    assert "ModuleNotFoundError" in render(display.detections_summary({"error": "ModuleNotFoundError: ultralytics"}))


def test_motor_changes_display():
    result = {"motor": probe.describe_motor(24, 0, mx12_table(24)), "baudrate": 57600}
    text = render(display.id_change(1, 24, result))
    assert "Motor 1 is now motor 24" in text and "0 to 20" in text
    assert "Nothing changed" in render(display.id_change(1, 2, {"error": "ID 2 is already taken"}))
    elsewhere = {"motor": probe.describe_motor(1, 0, mx12_table(1)), "baudrate": 1000000}
    assert "no longer sees this motor" in render(display.speed_change(1, elsewhere))
    assert "no longer" not in render(display.speed_change(24, result))


def test_motors_details():
    found = [probe.describe_motor(1, 0, mx12_table(1)), probe.describe_motor(4, 0, mx12_table(4, wheel=False))]
    motors = {"bus": "free", "baudrate": 57600, "motors": found}
    text = render(display.motors_details(motors))
    assert "Motors at 57600 bauds" in text and "ID 1" in text and "ID 4" in text
    assert "MX-12W, firmware 36" in text and "45 °C (limit 70 °C)" in text and "100 % (maximum 100 %)" in text
    assert "overheating, overload" in text and "every instruction" in text and "500 µs" in text
    assert "-180° to 180°" in text  # the angle limits of the motor in joint mode
    assert "No motor answers" in render(display.motors_details({"bus": "free", "baudrate": 57600, "motors": []}))
    assert "bus busy" in render(display.motors_details({"bus": "busy"}))


def test_without_command_the_help_is_shown():
    output = io.StringIO()
    with redirect_stdout(output):
        cli.main([])
    commands = ("watch", "list", "status", "robot", "check-motors", "scan-motor", "check-lidar", "check-cam",
                "set-motor-id", "set-motor-speed", "spin-motors", "shutdown")
    assert all(command in output.getvalue() for command in commands)


def test_durations():
    assert display.duration(5400) == "1 h 30" and display.duration(90000) == "1 d 1 h"
    assert display.duration(600) == "10 min"


if __name__ == "__main__":
    for name, test in list(globals().items()):
        if name.startswith("test_"):
            test()
            print("ok", name)
