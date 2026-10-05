# Supervision of the robots

A command-line tool for the teacher: it runs on their computer (Linux, macOS or Windows) and shows, for each robot, what works and what does not: battery, motors, lidar, camera, Pi, wifi, running notebooks. It can also list the motors with all their characteristics, search for them at every speed of the bus, change their ID or their bus speed, make them turn, show what the lidar and the camera see, and shut robots down.

## Running it

```
bin/supervision                  # the help: the list of commands
bin/supervision watch            # the dashboard, refreshed every minute; Ctrl-C to quit
bin/supervision watch --period 20   # the same, every 20 seconds
bin/supervision list             # the robots online, and when the others were last seen
bin/supervision status 1 3       # the state of robots 1 and 3, once
bin/supervision robot 1          # everything about robot 1, lidar included
bin/supervision check-motors 1   # the motors of robot 1, with all their characteristics
bin/supervision scan-motor 1     # the motors of robot 1, at every speed of the bus (about one minute)
bin/supervision scan-motor 1 --bauds 57600 1000000 --ids 0-20
bin/supervision set-motor-id 2 1 4               # motor 1 of robot 2 becomes motor 4
bin/supervision set-motor-id 2 1 4 --baud 1000000   # the same, for a motor still at its factory speed
bin/supervision set-motor-speed 2 4 57600 --baud 1000000   # motor 4 of robot 2 goes from 1000000 to 57600 bauds
bin/supervision spin-motors 1 90                 # the motors of robot 1 turn at 90 °/s for 2 s, then stop
bin/supervision spin-motors 1 -90 --ids 4 --duration 5     # motor 4 only, the other way, for 5 s
bin/supervision check-lidar 1    # an image of a lidar scan of robot 1 with its segments
bin/supervision check-cam 1      # an image of the camera of robot 1: color with what YOLO recognizes, depth map, infrared
bin/supervision check-cam 1 --no-yolo   # the same without YOLO, a few seconds faster
bin/supervision shutdown 3 4     # shuts robots 3 and 4 down, after a confirmation
```

A new motor usually comes with ID 1, at 1000000 bauds: plug the new motors in one at a time, give each one its ID with `set-motor-id … --baud 1000000`, then the speed of the robots with `set-motor-speed … 57600 --baud 1000000`. The speeds a motor can take: 9600, 19200, 57600, 115200, 200000, 250000, 400000, 500000, 1000000, 2250000, 2500000 and 3000000 bauds.

On Windows, the launcher is `bin\supervision.cmd`: `bin\supervision status 1`.

N is the number of a robot, from 1 to 6. Without N, `watch`, `list` and `status` show all the robots. `bin/supervision COMMAND -h` describes the arguments and options of a command.

The first time, the launcher creates the Python environment `.venv` at the root of the repository and installs the dependencies of `supervision/requirements.txt` in it (rich, PyYAML, matplotlib, and PyQt6 on Linux for the windows): nothing is installed anywhere else on the computer. It needs Python 3 (on Debian or Ubuntu, with the `python3-venv` package) and the `ssh` client (included in Windows 10 and 11).

The tool reads the addresses of the robots in `private/robots.yaml`, the private file of the project, which also gathers all the passwords. It is not in the repository: on another computer, it has to be copied there (for example with `scp`). Its template, without secrets: `supervision/robots.example.yaml`. A robot whose camera is mounted upside down gets `camera_rotation: 180` there: the probe turns the images upright before YOLO looks at them.

`check-lidar` and `check-cam` show their image in a window (close it to end the command) and save it in `tmp/`, at the root of the repository (created if needed, ignored by git). Without a screen, for example over SSH, only the file is produced. `check-cam` also lists the objects that YOLO recognizes in the color image, with their confidence, as the dashboard notebook does (YOLO26 nano, 80 categories: person, chair, bottle…).

`spin-motors` turns the motors with `holorobot.motors`, as the notebooks do: SPEED in °/s, from -720 to 720, positive counterclockwise seen from the wheel; 2 s by default, 30 s at most. It shows the measured speed of each motor twice a second, then stops the motors and turns their torque off. Ctrl-C stops them within a second (0.7 s measured). If the connection is lost without being closed (wifi down), the robot does not notice: the motors stop at the end of the duration. On the floor, the robot moves: put it on its stand, wheels in the air.

`shutdown` first shows, for each robot, whether it is online and how many notebooks are running there (their unsaved work would be lost), then asks for a confirmation (`--yes` skips it). It then connects to each robot in the terminal, where sudo asks the password of `admin`: the tool never stores nor sends it. A robot shut down stays off until someone turns it on again, on the spot.

## What the dashboard shows

| Column | What it is | Alerts |
|---|---|---|
| Access | the route taken to reach the robot (see below) | "offline", "SSH key refused"… |
| Battery | voltage read by the motors, and estimated charge | yellow below 3.60 V per cell (10.8 V), red below 3.45 V (10.35 V) |
| Motors | IDs of the motors, temperature of the hottest one | yellow from 55 °C, red from 65 °C (shutdown at 70 °C); "bus busy" if a notebook drives the motors |
| Lidar | USB adapter of the lidar plugged in | "missing" |
| Camera | RealSense plugged in, and on which port | yellow on USB 2: it is throttled there |
| Pi | temperature of the processor | "undervoltage now" in red; "undervoltage at 17:06" in yellow, the time of the last alert since boot (+ the number of earlier ones); disk almost full; stopped service |
| Wifi | network and signal strength | yellow below 50 %, red below 30 % |
| Notebooks | running Jupyter kernels | |

The thresholds are at the top of `supervision/display.py`.

A Pi 4 does not measure its supply voltage: it only detects when it drops below about 4.63 V, and the kernel notes the time of each alert. `robot N` shows them all, with the raw value of `vcgencmd get_throttled` (bit 0: undervoltage now; bit 16: since boot). An alert in the past means the supply is too weak at times, typically when a USB device is plugged in: the Pi can then restart on its own.

## How it works

```
teacher's computer                                    robot
  bin/supervision ──── ssh admin@… ──────────────▶  /opt/robot/venv/bin/python -
  (supervision/cli.py)   the probe, on stdin          (supervision/probe.py)
                       ◀── answer in JSON ───────────
```

- `supervision/robots.py` chooses the route to each robot, in this order: the **local network** (`MobileRobot-1.local`, if the computer is on the same network), **Tailscale** directly (if the computer is in the tailnet), otherwise the Tailscale **Funnel**, which works from anywhere: SSH travels inside TLS, on port 10000, and `supervision/tunnel.py` is the TLS tunnel that ssh runs. The option `--access local|tailscale|funnel` forces a route.
- The probe, `supervision/probe.py`, is sent every time: nothing has to be installed on the robots, and the version of the repository is always the one that runs. It needs the standard library and pyserial; the lidar scan also uses `holorobot.lidar`, the camera capture `pyrealsense2`, and YOLO `ultralytics`, all installed on every robot. Its standard output carries its answer only: what the libraries print goes to the standard error. It can also be run by hand on a robot: `python3 probe.py state`.
- `check-motors` shows, for each motor: model and firmware, mode (wheel, joint or multi-turn) and angle limits, torque, position and goal position, speed and speed command, load, voltage and its limits, temperature and its limit, torque limit (it drops to 0 % after an alarm that cuts the torque), current alarms, alarms that cut the torque, return delay, which instructions it answers, LED.
- `supervision/display.py` formats the answers with rich; `supervision/lidar_image.py` and `supervision/camera_image.py` draw the images with matplotlib, as the notebooks do; `supervision/cli.py` defines the commands.
- The connection uses the SSH key, never a password. The first time, the host key of each robot is recorded in `~/.ssh/known_hosts`, under the name of the robot (`MobileRobot-1`) whatever the route.

## What the tool touches on the robots

The tool only reads, except for what each command below says:
- **motors**: the probe only sends reads (PING and READ of the Dynamixel protocol), except `set-motor-id` and `set-motor-speed`, which write the new ID or the new bus speed in the motor (after turning its torque off), and `spin-motors`, which puts the motors in wheel mode and turns them. It opens the bus only if it is free, with the same lock as `holorobot.motors`: if a notebook drives the motors, it answers "bus busy" without disturbing it. `status`, `watch` and `check-motors` keep the bus less than a second; `scan-motor` keeps it during the whole scan, up to one minute;
- **lidar**: `status` and `watch` only check that it is plugged in. `robot N` also asks it for its model and health, without starting its motor. `check-lidar N` reads three turns, and shows the fullest one: the motor of the lidar spins for about a second;
- **camera**: `check-cam N` captures one image of each stream; the infrared projector lights up for about a second. YOLO runs on the robot; the first time, it downloads its model, `yolo26n.pt` (5 MB), into the home of `admin`. The other commands only read what USB says about the camera;
- **shutdown**: `shutdown N` shuts the robot down, with `sudo systemctl poweroff`, once sudo has the password.

Each device is used only if no program is using it. During a course, if a student opens the motors, the lidar or the camera at the very moment the probe reads them, they get the message "déjà utilisé" (already in use): running the cell again is enough.

## Tests

```
.venv/bin/python tests/test_supervision.py
```

They do not need a robot: a fake bus simulates the motors.
