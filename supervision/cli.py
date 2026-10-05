"""Supervision of the robots of the course, in the terminal.

    bin/supervision                        the help: the list of commands
    bin/supervision watch                  the dashboard, refreshed every minute
    bin/supervision list                   the robots online, and when the others were last seen
    bin/supervision status 1 3             the state of robots 1 and 3, once
    bin/supervision robot 1                everything about robot 1, lidar included
    bin/supervision check-motors 1         the motors of robot 1, with all their characteristics
    bin/supervision scan-motor 1           the motors of robot 1, searched at every speed of the bus
    bin/supervision check-lidar 1          an image of a lidar scan of robot 1, with its segments
    bin/supervision check-cam 1            an image of the camera of robot 1: color with YOLO, depth, infrared
    bin/supervision set-motor-id 2 1 4     motor 1 of robot 2 becomes motor 4
    bin/supervision set-motor-speed 2 1 57600 --baud 1000000
                                           motor 1 of robot 2, now at 1000000 bauds, goes to 57600
    bin/supervision spin-motors 2 90       turns the motors of robot 2 at 90 °/s for 2 s, then stops them
    bin/supervision shutdown 3 4           shuts robots 3 and 4 down, after a confirmation

Each command accepts --access (local, tailscale or funnel; by default, the first one that works).
"""
import argparse
import json
import time
from datetime import datetime
from pathlib import Path

from rich.console import Console
from rich.live import Live
from rich.text import Text

from . import display
from .probe import BAUDRATES, MOTOR_BAUDRATE, SPIN_MAX_DURATION, SPIN_MAX_SPEED
from .robots import Robot, Unreachable, in_parallel, load_robots, probe_all

IMAGES = Path(__file__).resolve().parent.parent / "tmp"  # in the repository, ignored by git
# The robot shuts down 3 s later: meanwhile, ssh gets the answer and ends normally
SHUTDOWN = "sudo systemd-run --quiet --on-active=3 /bin/systemctl poweroff"
SPEEDS = sorted(BAUDRATES.values())  # every possible speed of the bus, in bauds
console = Console()


def watch(args):
    """The dashboard, refreshed every `period` seconds, until Ctrl-C."""
    robots = load_robots(args.robots)
    with Live(Text("Querying the robots…", "dim"), console=console, auto_refresh=False) as live:
        while True:
            states = probe_all(robots, "state", access=args.access)
            live.update(display.dashboard(states, args.period), refresh=True)
            time.sleep(args.period)


def list_robots(args):
    """The robots online; for the others, when Tailscale last saw them."""
    robots = load_robots(args.robots)
    with console.status("Querying the robots…"):
        states = probe_all(robots, "state", "--no-motors", "--tailnet", access=args.access)
        jupyter = in_parallel(Robot.jupyter_responds, robots)
    show(args, states, display.robot_list(states, jupyter))


def show_status(args):
    """The state of each robot, once."""
    robots = load_robots(args.robots)
    with console.status("Querying the robots…"):
        states = probe_all(robots, "state", access=args.access)
    show(args, states, display.overview(states))


def show_robot(args):
    """Everything about a robot, including what the lidar says about itself."""
    robot = load_robots([args.robot])[0]
    with console.status(f"Querying {robot.hostname}…"):
        state = probe_or_exit(robot, "state", "--lidar", access=args.access)
    show(args, {robot: state}, display.details(robot, state))


def check_motors(args):
    """The motors of a robot, with all their characteristics."""
    robot = load_robots([args.robot])[0]
    with console.status(f"Reading the motors of {robot.hostname}…"):
        motors = probe_or_exit(robot, "motors", "--baud", str(args.baud), "--ids", args.ids, access=args.access)
    show(args, {robot: motors}, display.motors_details(motors))


def scan_motor(args):
    """Searches the motors of a robot at several speeds of the bus, showing the scan as it goes."""
    robot = load_robots([args.robot])[0]
    options = ["--ids", args.ids] + (["--bauds", *map(str, args.bauds)] if args.bauds else [])
    speeds = " ".join(map(str, args.bauds)) if args.bauds else "all (about one minute)"
    console.print(f"Scanning the motors of {robot.hostname} · IDs {args.ids} · speeds: {speeds}")
    results = []
    try:
        for result in robot.stream("scan-motor", *options, access=args.access):
            console.print(display.scan_step(result))
            results.append(result)
    except Unreachable as error:
        raise SystemExit(f"{robot.hostname}: {error}")
    console.print(display.scan_summary(results))


def check_lidar(args):
    """An image of a lidar scan with its segments, saved in tmp/ and shown in a window."""
    from . import lidar_image  # imported here: matplotlib is slow to load, and only this command needs it

    robot = load_robots([args.robot])[0]
    with console.status(f"Reading the lidar of {robot.hostname}…"):
        scan = probe_or_exit(robot, "lidar", access=args.access)
    check_answer(robot, scan, "points", "lidar")
    if not scan["points"]:  # it happens now and then: the lidar turns, but its distances are all 0
        console.print(display.lidar_turns(scan["turns"]))
        raise SystemExit(f"{robot.hostname}: the lidar turned, but measured nothing: run the command again")
    path = image_path("lidar", robot)
    figure = lidar_image.draw(scan, f"{robot.hostname} · lidar scan and segments (color = confidence)", path)
    console.print(f"{len(scan['points'])} points, {len(scan['segments'])} segments · image: {path}")
    console.print(display.lidar_turns(scan["turns"]))
    show_window(figure, path)


def check_cam(args):
    """An image of the camera (color with what YOLO recognizes, depth map, infrared), saved in tmp/ and shown."""
    from . import camera_image  # imported here: matplotlib is slow to load, and only this command needs it

    robot = load_robots([args.robot])[0]
    options = ["--rotation", str(robot.camera_rotation)] + (["--no-yolo"] if args.no_yolo else [])
    with console.status(f"Capturing the camera of {robot.hostname}…"):
        frames = probe_or_exit(robot, "camera", *options, access=args.access)
    check_answer(robot, frames, "color", "camera")
    path = image_path("camera", robot)
    figure = camera_image.draw(frames, f"{robot.hostname} · camera", path)
    measured, center = camera_image.depth_summary(frames)
    at_center = f"{center:.2f} m at the center" if center > 0 else "no measurement at the center"
    console.print(f"Depth: {measured:.0%} of the pixels measured, {at_center} · image: {path}")
    console.print(display.detections_summary(frames.get("detections")))
    show_window(figure, path)


def set_motor_id(args):
    """Changes the ID of a motor, then shows the motor under its new ID."""
    robot = load_robots([args.robot])[0]
    with console.status(f"Motor {args.old} of {robot.hostname} becomes motor {args.new}…"):
        result = probe_or_exit(robot, "set-motor-id", "--old", str(args.old), "--new", str(args.new),
                               "--baud", str(args.baud), access=args.access)
    console.print(display.id_change(args.old, args.new, result))


def set_motor_speed(args):
    """Changes the bus speed of a motor, then shows the motor at its new speed."""
    robot = load_robots([args.robot])[0]
    with console.status(f"Motor {args.id} of {robot.hostname} goes to {args.new} bauds…"):
        result = probe_or_exit(robot, "set-motor-speed", "--id", str(args.id), "--new", str(args.new),
                               "--baud", str(args.baud), access=args.access)
    console.print(display.speed_change(args.id, result))


def spin_motors(args):
    """Turns the motors of a robot for a few seconds, then stops them, showing their measured speed."""
    robot = load_robots([args.robot])[0]
    which = f"motors {' '.join(map(str, args.ids))}" if args.ids else "all the motors"
    console.print(f"{robot.hostname}: {which} at {args.speed:g} °/s for {args.duration:g} s · Ctrl-C stops them")
    options = [f"--speed={args.speed}", f"--duration={args.duration}"]  # "=": a negative speed is not an option
    if args.ids:
        options += ["--ids", *map(str, args.ids)]
    try:
        for result in robot.stream("spin-motors", *options, access=args.access):
            console.print(display.spin_step(result))
    except Unreachable as error:
        raise SystemExit(f"{robot.hostname}: {error}")
    except KeyboardInterrupt:
        console.print(Text("Interrupted: the robot stops the wheels within a second.", "yellow"))


def shutdown(args):
    """Shuts robots down, after a confirmation. On each robot, sudo asks the password of admin."""
    robots = load_robots(args.robots)
    with console.status("Querying the robots…"):
        states = probe_all(robots, "state", "--no-motors", access=args.access)
    console.print(display.shutdown_preview(states))
    online = [robot for robot in robots if not isinstance(states[robot], Unreachable)]
    if not online:
        return
    if not args.yes and not confirm(f"Shut down {', '.join(robot.hostname for robot in online)}?"):
        console.print("Nothing done.")
        return
    for robot in online:
        console.print(f"{robot.hostname}: sudo asks the password of {robot.user}")
        if robot.run_in_terminal(SHUTDOWN, access=args.access) == 0:
            console.print(Text(f"{robot.hostname}: shuts down in 3 seconds", "green"))
        else:
            console.print(Text(f"{robot.hostname}: not shut down", "red"))


def confirm(question):
    """True if the user answers yes; anything else, an empty answer included, means no."""
    return input(f"{question} [y/N] ").strip().lower() in ("y", "yes")


def between(low, high):
    """An argparse type: a number from `low` to `high`."""
    def number(text):
        value = float(text)
        if not low <= value <= high:
            raise argparse.ArgumentTypeError(f"{text} is not between {low:g} and {high:g}")
        return value
    return number


def probe_or_exit(robot, *args, access):
    """The answer of the probe, or the end of the program with the reason if the robot does not answer."""
    try:
        return robot.probe(*args, access=access)
    except Unreachable as error:
        raise SystemExit(f"{robot.hostname}: {error}")


def check_answer(robot, answer, expected, device):
    """Ends the program with the reason if the probe could not use the device (lidar or camera)."""
    if expected not in answer:
        reason = f"{device} busy with another program" if answer.get("busy") else answer.get("error", f"no {device}")
        raise SystemExit(f"{robot.hostname}: {reason}")


def image_path(kind, robot):
    """Where to save a new image: tmp/, created if needed, with the robot and the time in the name."""
    IMAGES.mkdir(exist_ok=True)
    return IMAGES / f"{kind}_{robot.hostname}_{datetime.now():%Y%m%d_%H%M%S}.png"


def show_window(figure, path):
    """Shows the image in a matplotlib window, until it is closed. Without a screen, only the file remains."""
    import matplotlib.pyplot as plt

    if plt.get_backend().lower() == "agg":
        return  # no window possible (or MPLBACKEND=agg): the image is in tmp/
    figure.canvas.manager.set_window_title(path.name)
    plt.show()


def show(args, states, rendering):
    """Shows the rendering, or with --json the raw answers of the probe."""
    if args.json:
        raw = {robot.hostname: str(state) if isinstance(state, Exception) else state for robot, state in states.items()}
        print(json.dumps(raw, indent=2, ensure_ascii=False))
    else:
        console.print(rendering)


def build_parser():
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--access", choices=("auto", "local", "tailscale", "funnel"), default="auto",
                        help="route to the robots: local network, Tailscale or Funnel "
                             "(default: the first one that works)")
    raw = argparse.ArgumentParser(add_help=False)
    raw.add_argument("--json", action="store_true", help="print the raw answers of the probe")

    # RawTextHelpFormatter keeps the spaces: the arguments of the commands line up in a column
    parser = argparse.ArgumentParser(
        prog="supervision", formatter_class=argparse.RawTextHelpFormatter,
        description="Supervision of the robots of the course, from the teacher's computer.",
        epilog="arguments:\n"
               "  N              robot number, from 1 to 6 (all robots by default for watch, list and status)\n"
               "  ID, OLD, NEW   motor IDs, from 0 to 252 (see check-motors)\n"
               "  BAUD           speed of the bus, in bauds: " + ", ".join(map(str, SPEEDS[:6])) + ",\n"
               "                 " + ", ".join(map(str, SPEEDS[6:])) + "\n"
               f"                 (the robots talk at {MOTOR_BAUDRATE}; new motors are often at 1000000)\n"
               f"  SPEED          speed of the wheels, in °/s, from -{SPIN_MAX_SPEED} to {SPIN_MAX_SPEED}\n\n"
               "Arguments and options of a command: supervision COMMAND -h")
    commands = parser.add_subparsers(metavar="command")

    def command(name, function, arguments, help, *parents, description=None):
        """A command; `arguments` is shown before its description, in the list of commands."""
        sub = commands.add_parser(name, parents=[common, *parents], help=f"{arguments:<11}{help}",
                                  description=description or help, formatter_class=argparse.RawTextHelpFormatter)
        sub.set_defaults(function=function)
        return sub

    def motor_options(sub, ids):
        """The options --baud and --ids, for the commands that look for motors."""
        sub.add_argument("--baud", type=int, default=MOTOR_BAUDRATE, metavar="BAUD",
                         help=f"speed of the bus, in bauds (default: {MOTOR_BAUDRATE}, the speed of the robots)")
        sub.add_argument("--ids", default=ids, help=f"motor IDs to try, for example 0-20 (default: {ids})")

    # The robots
    watch_parser = command("watch", watch, "[N ...]", "live dashboard, refreshed every minute")
    watch_parser.add_argument("--period", type=int, default=60, metavar="S",
                              help="seconds between two updates (default: 60)")
    list_parser = command("list", list_robots, "[N ...]", "robots online, and when the others were last seen", raw)
    status_parser = command("status", show_status, "[N ...]", "state of each robot, once", raw)
    for sub in (watch_parser, list_parser, status_parser):
        sub.add_argument("robots", type=int, nargs="*", metavar="N", help="robot numbers (default: all robots)")
    robot_parser = command("robot", show_robot, "N", "everything about robot N, lidar included", raw)

    # The motors
    motors_parser = command("check-motors", check_motors, "N", "the motors of robot N, with all their characteristics",
                            raw)
    motor_options(motors_parser, "0-20")
    scan_parser = command("scan-motor", scan_motor, "N", "search the motors of robot N at every speed of the bus")
    scan_parser.add_argument("--bauds", type=int, nargs="+", metavar="BAUD", choices=SPEEDS,
                             help="bus speeds to try (default: all of them, about one minute)")
    scan_parser.add_argument("--ids", default="0-253", help="motor IDs to try, for example 0-20 (default: 0-253)")
    id_parser = command(
        "set-motor-id", set_motor_id, "N OLD NEW", "change the ID of motor OLD of robot N to NEW",
        description="Change the ID of motor OLD of robot N to NEW. The motor keeps it, even without power.\n"
                    "Refused if a program uses the bus, if no motor answers OLD, or if NEW is taken.\n"
                    "New motors all come with ID 1: plug them in one at a time.")
    speed_parser = command(
        "set-motor-speed", set_motor_speed, "N ID BAUD", "change the bus speed of motor ID of robot N to BAUD",
        description="Change the speed at which motor ID of robot N talks on the bus (its baud rate), to BAUD.\n"
                    "The motor keeps it, even without power. Refused if a program uses the bus, if no motor\n"
                    "answers ID, or if another motor with the same ID already answers at BAUD.\n"
                    f"Possible speeds, in bauds: {', '.join(map(str, SPEEDS))}.\n"
                    f"The robots talk to their motors at {MOTOR_BAUDRATE} bauds; new motors are often at 1000000.")
    for sub in (robot_parser, motors_parser, scan_parser, id_parser, speed_parser):
        sub.add_argument("robot", type=int, metavar="N", help="robot number")
    id_parser.add_argument("old", type=int, metavar="OLD", help="current ID of the motor (see check-motors)")
    id_parser.add_argument("new", type=int, metavar="NEW",
                           help="new ID, from 0 to 252, not taken by another motor\n"
                                "(holorobot.motors looks for IDs 0 to 20 by default)")
    speed_parser.add_argument("id", type=int, metavar="ID", help="ID of the motor (see check-motors or scan-motor)")
    speed_parser.add_argument("new", type=int, metavar="BAUD", choices=SPEEDS,
                              help=f"new speed of the bus, in bauds: one of\n{', '.join(map(str, SPEEDS))}")
    for sub in (id_parser, speed_parser):
        sub.add_argument("--baud", type=int, default=MOTOR_BAUDRATE, metavar="BAUD", choices=SPEEDS,
                         help=f"speed at which the motor talks now, in bauds (default: {MOTOR_BAUDRATE};\n"
                              "a new motor is often at 1000000: see scan-motor)")
    spin_parser = command(
        "spin-motors", spin_motors, "N SPEED", "turn the motors of robot N at SPEED °/s for 2 s, then stop them",
        description="Turn the motors of robot N at SPEED degrees per second for a few seconds, then stop them\n"
                    "and turn their torque off. The motors go to wheel mode, as Motors() does in the notebooks.\n"
                    "Ctrl-C stops them within a second; if the connection is lost, they stop at the end anyway.\n"
                    "On the floor, the robot moves: put it on its stand, wheels in the air.")
    spin_parser.add_argument("robot", type=int, metavar="N", help="robot number")
    spin_parser.add_argument("speed", type=between(-SPIN_MAX_SPEED, SPIN_MAX_SPEED), metavar="SPEED",
                             help=f"speed of the wheels, in °/s, from -{SPIN_MAX_SPEED} to {SPIN_MAX_SPEED};\n"
                                  "positive: counterclockwise, seen from the wheel")
    spin_parser.add_argument("--duration", type=between(0.1, SPIN_MAX_DURATION), default=2, metavar="S",
                             help=f"how long they turn, in seconds, up to {SPIN_MAX_DURATION} (default: 2)")
    spin_parser.add_argument("--ids", type=int, nargs="+", metavar="ID",
                             help="the motors to turn (default: all of them, among IDs 0 to 20)")

    # The lidar and the camera
    lidar_parser = command("check-lidar", check_lidar, "N",
                           "image of a lidar scan of robot N with its segments, shown and saved in tmp/")
    cam_parser = command("check-cam", check_cam, "N",
                         "image of the camera of robot N: color with YOLO, depth, infrared; shown and saved in tmp/")
    cam_parser.add_argument("--no-yolo", action="store_true", help="do not run YOLO (a few seconds faster)")
    for sub in (lidar_parser, cam_parser):
        sub.add_argument("robot", type=int, metavar="N", help="robot number")

    # Shutting robots down
    shutdown_parser = command(
        "shutdown", shutdown, "N [N ...]", "shut robots N down, after a confirmation (sudo asks the password)",
        description="Shut robots N down. The command first shows which robots are online and their running\n"
                    "notebooks, then asks a confirmation. On each robot, sudo asks the password of admin.\n"
                    "A robot shut down stays off until someone turns it on again, on the spot.")
    shutdown_parser.add_argument("robots", type=int, nargs="+", metavar="N", help="numbers of the robots to shut down")
    shutdown_parser.add_argument("--yes", action="store_true", help="do not ask for a confirmation")
    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if "function" not in args:  # no command: the help
        parser.print_help()
        return
    try:
        args.function(args)
    except KeyboardInterrupt:
        pass  # Ctrl-C: quit, without an error trace
