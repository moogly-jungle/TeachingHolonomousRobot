"""The robots of the course, and how to talk to them.

Their addresses come from private/robots.yaml, the private file of the project (template:
supervision/robots.example.yaml). To reach a robot, the supervision tries in this order:
1. the local network, if the computer is on the same network as the robot (MobileRobot-1.local);
2. Tailscale directly, if the computer itself is in the tailnet;
3. the Tailscale Funnel, which works from anywhere: SSH travels inside TLS, on port 10000 (the TLS
   tunnel is tunnel.py, run by ssh).

It runs the probe (probe.py) there with the robot's Python and reads its JSON answer: nothing has
to be installed on the robots.
"""
import json
import shlex
import socket
import subprocess
import sys
import threading
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path

import yaml

CONFIG = Path(__file__).resolve().parent.parent / "private" / "robots.yaml"
PROBE = Path(__file__).with_name("probe.py")
TUNNEL = Path(__file__).with_name("tunnel.py")
# On the robot: its Python, which has pyserial; python3 otherwise (robot not set up yet)
REMOTE_PYTHON = 'P=/opt/robot/venv/bin/python; [ -x "$P" ] || P=python3; exec "$P" -'
# Messages of ssh -> short explanation
SSH_ERRORS = {
    "Permission denied": "SSH key refused",
    "Connection closed": "offline",
    "Connection refused": "connection refused",
    "timed out": "timed out",
    "Could not resolve": "unknown address",
}


class Unreachable(Exception):
    """The robot does not answer: the reason, and the route that was tried."""

    def __init__(self, reason, route):
        super().__init__(f"{reason} ({route})")
        self.reason = reason
        self.route = route


@dataclass(frozen=True)
class Route:
    """A route to a robot: "local", "tailscale" or "funnel"."""

    name: str
    host: str
    port: int = 22
    tls: bool = False  # SSH inside TLS: this is what the Funnel accepts

    def ssh_options(self):
        """The ssh options for this route: the port, and for the Funnel the TLS tunnel (tunnel.py)."""
        options = ["-p", str(self.port)]
        if self.tls:
            options += ["-o", f'ProxyCommand="{sys.executable}" "{TUNNEL}" %h %p']
        return options

    def is_open(self, timeout=1.5):
        """True if the robot accepts a connection through this route (quick try, without SSH)."""
        if self.tls:
            return True  # the Funnel always answers: SSH will tell whether the robot is there
        return accepts_connection(self.host, self.port, timeout)


@dataclass(frozen=True)
class Robot:
    """A robot of the course, as described in private/robots.yaml."""

    number: int
    hostname: str  # name of the Pi: MobileRobot-1
    tailscale_name: str  # name in the tailnet: holobot1
    tailscale_ip: str
    tailnet: str
    funnel_port: int
    user: str
    camera_rotation: int = 0  # 180 for a camera mounted upside down

    @property
    def public_name(self):
        return f"{self.tailscale_name}.{self.tailnet}"

    @property
    def jupyter_url(self):
        return f"https://{self.public_name}"

    def routes(self):
        """The routes to the robot, from the most direct to the one that works everywhere."""
        return [
            Route("local", f"{self.hostname}.local"),
            Route("tailscale", self.tailscale_ip),
            Route("funnel", self.public_name, self.funnel_port, tls=True),
        ]

    def route(self, access="auto"):
        """The first open route; `access` can force one: "local", "tailscale" or "funnel"."""
        if access != "auto":
            return next(route for route in self.routes() if route.name == access)
        return next(route for route in self.routes() if route.is_open())

    def ssh_command(self, route, *probe_args):
        """The command that runs the probe on the robot; the probe reaches it through standard input."""
        remote_command = " ".join([REMOTE_PYTHON, *map(shlex.quote, probe_args)])
        return ["ssh", *self.ssh_options(route), f"{self.user}@{route.host}", remote_command]

    def terminal_command(self, route, remote_command):
        """The command that runs `remote_command` on the robot in our terminal: sudo can ask the password there."""
        return ["ssh", "-t", *self.ssh_options(route), f"{self.user}@{route.host}", remote_command]

    def ssh_options(self, route):
        return [
            "-o", "BatchMode=yes",  # no password for SSH itself: the SSH key, or nothing
            "-o", "ConnectTimeout=10",
            "-o", "ServerAliveInterval=5",
            "-o", "StrictHostKeyChecking=accept-new",
            "-o", f"HostKeyAlias={self.hostname}",  # the same host key, whatever the route
            *route.ssh_options(),
        ]

    def run_in_terminal(self, remote_command, access="auto"):
        """Runs a command on the robot, attached to our terminal; returns its exit code."""
        return subprocess.run(self.terminal_command(self.route(access), remote_command)).returncode

    def probe(self, *args, access="auto", timeout=90):
        """Runs the probe on the robot and returns its answer, with the route that was taken."""
        route = self.route(access)
        try:
            done = subprocess.run(self.ssh_command(route, *args), input=PROBE.read_text(),
                                  capture_output=True, text=True, timeout=timeout)
        except subprocess.TimeoutExpired:
            raise Unreachable(f"no answer within {timeout} s", route.name) from None
        if done.returncode != 0:
            raise Unreachable(explain(done.stderr), route.name)
        try:
            return {**json.loads(done.stdout), "route": route.name}
        except json.JSONDecodeError:  # not an answer of the probe
            raise Unreachable(f"unexpected answer {done.stdout[:80]!r}", route.name) from None

    def stream(self, *args, access="auto"):
        """Runs the probe and yields its JSON lines as they come: for long scans."""
        route = self.route(access)
        with subprocess.Popen(self.ssh_command(route, *args), stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                              stderr=subprocess.PIPE, text=True) as process:
            process.stdin.write(PROBE.read_text())
            process.stdin.close()
            for line in process.stdout:
                yield json.loads(line)
            if process.wait() != 0:
                raise Unreachable(explain(process.stderr.read()), route.name)

    def jupyter_responds(self, timeout=10):
        """True if JupyterLab answers at its public address, the one the students use."""
        try:
            with urllib.request.urlopen(f"{self.jupyter_url}/lab", timeout=timeout) as reply:
                return reply.status == 200
        except OSError:
            return False


def load_robots(numbers=(), path=CONFIG):
    """The robots described in private/robots.yaml: all of them, or those whose numbers are given."""
    if not path.exists():
        raise SystemExit(f"File not found: {path}\n"
                         "Copy the template supervision/robots.example.yaml to private/robots.yaml and fill it in.")
    config = yaml.safe_load(path.read_text(encoding="utf-8"))
    tailscale = config["tailscale"]
    robots = [
        Robot(number, entry["hostname"], entry["tailscale"], entry["tailscale_ip"], tailscale["tailnet"],
              tailscale["funnel_ssh_port"], config["ssh"]["user"], entry.get("camera_rotation", 0))
        for number, entry in config["robots"].items()
    ]
    unknown = set(numbers) - {robot.number for robot in robots}
    if unknown:
        raise SystemExit(f"Unknown robots: {sorted(unknown)} (see {path})")
    return [robot for robot in robots if not numbers or robot.number in numbers]


def probe_all(robots, *args, access="auto"):
    """Queries the robots in parallel: {robot: answer}, or the Unreachable error instead."""
    def probe(robot):
        try:
            return robot.probe(*args, access=access)
        except Unreachable as error:
            return error

    return in_parallel(probe, robots)


def in_parallel(function, robots):
    """function(robot) for each robot, all at the same time: {robot: result}."""
    with ThreadPoolExecutor(max_workers=len(robots)) as pool:
        return dict(zip(robots, pool.map(function, robots)))


def accepts_connection(host, port, timeout):
    """True if host:port accepts a TCP connection within `timeout` seconds, name lookup included.

    Looking up a .local name can take several seconds when the robot is not on the network: we do
    it in a separate thread, which we do not wait for longer than `timeout`.
    """
    answer = []

    def connect():
        try:
            socket.create_connection((host, port), timeout).close()
            answer.append(True)
        except OSError:
            pass

    attempt = threading.Thread(target=connect, daemon=True)
    attempt.start()
    attempt.join(timeout)
    return bool(answer)


def explain(stderr):
    """The reason why ssh failed, in a few words."""
    for message, meaning in SSH_ERRORS.items():
        if message in stderr:
            return meaning
    lines = [line for line in stderr.splitlines() if line.strip() and not line.startswith("Connecting to")]
    return lines[-1] if lines else "cannot connect"
