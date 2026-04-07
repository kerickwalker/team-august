# CLAUDE.md — mqtt_python

This file is read by Claude Code at the start of each session. It captures the project
context, conventions, and the state of the codebase so sessions start without re-deriving
what is already known.

---

## Maintenance instructions — READ BEFORE STARTING ANY TASK

**After every session that modifies code, you must update documentation. This is not
optional.** Follow these rules every time, without waiting to be asked:

1. **Update this file (CLAUDE.md)** — keep the file map, code snippets, and "how to run"
   commands accurate. If a file is added, removed, or renamed, update the File map table.
   If constants or APIs change, update the relevant section. If a new entry point is
   added, update the Mission state machines section and Common commands.

2. **Update `../notes/change_log.md`** — add a dated entry (use today's actual date)
   describing what changed, which files were modified, and why. Follow the format of
   existing entries: problem → solution → modified files.

3. **Update the relevant feature doc in `../notes/`** — if the change touches path
   following, update `path_following.md`; if it touches line following, update
   `line_following.md`; if it touches system architecture or topics, update
   `system_architecture.md`. At minimum check that file paths, command examples, and
   parameter values are still correct.

4. **Update `../notes/overview.md`** if the current priority or next-session focus has
   changed.

5. **Do not leave stale information** — a wrong command in docs is worse than no docs.
   If a section is no longer accurate, fix or remove it.

These rules mirror `../notes/llm_operator_rules.md`. Both files say the same thing.

---

---

## What this directory is

`mqtt_python/` is the Python mission-control layer for the DTU Robobot platform (robot
"August", ID 112, type 8). It subscribes to sensor data over MQTT, runs state-machine
missions, and publishes velocity commands back to the robot.

The full system has three layers:
```
Teensy firmware  →  teensy_interface (C++)  →  mqtt_python (Python, this directory)
```
The interface must be running before any Python client is started. Only one MQTT master
is allowed at a time.

MQTT broker: `localhost:1883`

---

## Key conventions

### Singleton module pattern
Every subsystem is instantiated once at the bottom of its own file and imported by name:
```python
# in spose.py
pose = SPose()

# elsewhere
from spose import pose
```
The same pattern applies to: `edge`, `ir`, `robot`, `imu`, `cam`, `gpio`, `path_follow`,
`pursuit`.

### Lazy uservice import
Modules that need to publish (`service.send(...)`) import `uservice` inside a method to
avoid circular imports at load time:
```python
def some_method(self):
    from uservice import service
    service.send("robobot/cmd/ti", "rc 0.200 0.000")
```

### Velocity command format
All motion commands go to `robobot/cmd/ti` as:
```
rc <linvel_m_s> <turnrate_rad_s>
```
Example: `rc 0.200 -1.250`  — forward at 0.2 m/s, turning right at 1.25 rad/s.

### MQTT topic namespaces
| Direction | Prefix | Example |
|-----------|--------|---------|
| Incoming sensor data | `robobot/drive/` | `robobot/drive/T0/pose` |
| Outgoing commands | `robobot/cmd/` | `robobot/cmd/ti` |
| Kalman filter state | `robobot/kalman/state` | JSON pose estimate |

`uservice.on_message` subscribes to `robobot/drive/#` and dispatches via the `elif`
decoder chain in `uservice.decode()`. The Kalman topic is outside this prefix; modules
that need it must create their own MQTT client (see `spath_follow.py`).

### Mission state machines
Missions are explicit integer state machines inside functions like `driveToLine()`.
States 0–98 are mission-internal; state 99 is "exit this mission"; state 100 is the
return-to-idle transition in `loop()`; states 101+ are named missions.

**`mqtt_client_core.py`** — original multi-mission client:
```
101 → driveOneMeter()
102 → driveTurnPi()
103 → driveToLine()         (line following)
104 → driveMotorsPwmDuration()
```

**`mqtt_client_path_follow.py`** — dedicated path-follow client:
```
101 → drivePathFollow()     (Pure Pursuit trajectory following)
```
These are separate entry points. Running both at the same time will cause an MQTT
master conflict — only one can run at a time.

---

## File map

| File | Role |
|------|------|
| `mqtt_client_core.py` | Original entry point; line-follow, meter, pi, PWM missions |
| `mqtt_client_path_follow.py` | **Path-follow entry point** — Pure Pursuit mission only |
| `uservice.py` | MQTT connect/send/receive; argparse; master arbitration |
| `spose.py` | Encoder odometry decoder; `pose.pose[0..2]` = (x, y, heading rad) |
| `sedge.py` | Line-following controller; `edge.lineControl(vel)` arms/disarms |
| `spursuit.py` | **Pure Pursuit math** — no MQTT; fully standalone |
| `spath_follow.py` | **Path-following MQTT layer** — subscribes to Kalman topic, calls `spursuit` |
| `get-pose.py` | Standalone debug utility; subscribes to `robobot/kalman/state` |
| `sir.py` | IR proximity sensor decoder |
| `simu.py` | IMU (gyro, acc, yawg) decoder |
| `srobot.py` | Heartbeat / robot name decoder |
| `scam.py` | Camera setup |
| `sgpio.py` | GPIO (stop button, LED) |
| `ulog.py` | Log file writer |

---

## Pure Pursuit path-following (added 2026-04-07)

### What was added
Two new files and minimal modifications to existing ones.

#### `spursuit.py`
Pure math module. No MQTT. No robot imports. Loads a CSV trajectory and computes
`(linvel, turnrate)` given the current robot pose.

Tunable constants at the top of the file:
```python
LOOKAHEAD_DIST = 0.3   # metres
MAX_LINVEL     = 0.3   # m/s
MAX_TURNRATE   = 4.0   # rad/s
SEARCH_WINDOW  = 30    # points scanned forward to find nearest waypoint
```

Key methods on the `SPurePursuit` class (singleton `pursuit`):
- `load_trajectory(csv_path)` — parse CSV, store as numpy array
- `compute_command(x, y, heading, speed)` — one control step; returns `(linvel, turnrate)`
- `reset()` — call before each new run (resets nearest-point index to 0)
- `at_end()` — True when the nearest index has reached the last waypoint
- `is_loaded()` — True once a CSV has been loaded successfully

#### `spath_follow.py`
MQTT integration. Subscribes to `robobot/kalman/state` with its own MQTT client
(separate from `uservice`). Parses both JSON schemas the Kalman node may emit.

Key constants:
```python
TRAJECTORY_CSV = "trajectory.csv"   # path relative to working directory
```

Key methods on `SPathFollow` (singleton `path_follow`):
- `setup()` — connect MQTT client; call once from `uservice.setup()`
- `pathControl(velocity)` — `velocity > 0` arms controller and loads trajectory;
  `velocity == 0` disarms. Mirrors `edge.lineControl()` API.
- `update()` — call at ~50 Hz from mission loop; reads Kalman pose, calls
  `pursuit.compute_command()`, publishes `rc` command
- `terminate()` — disconnect MQTT client; call from `uservice.terminate()`

#### `mqtt_client_path_follow.py` (new, standalone entry point)
Dedicated main file for the path-following mission. Does not modify or depend on
`mqtt_client_core.py`. Contains `drivePathFollow()` and its own `loop()` that starts
directly in state 101 (path-follow mission). `mqtt_client_core.py` is unchanged.

#### Changes to `uservice.py`
- Added `from spath_follow import path_follow` import
- Added `path_follow.setup()` after `edge.setup()` in `setup()`
- Added `path_follow.terminate()` after `edge.terminate()` in `terminate()`

### Trajectory CSV format
```
x,y,heading,cumulative_dist
0.000,0.000,0.000,0.000
0.050,0.000,0.000,0.050
...
```
Columns must have exactly these names. Arc-length spacing ~0.05 m recommended.
Place the file at `mqtt_python/trajectory.csv` (or edit `TRAJECTORY_CSV`).

### How to run
```bash
python3 mqtt_client_path_follow.py
```
No flags needed — the mission starts immediately on launch.

### To replace with a Stanley controller later
Only `spursuit.py` needs to change. `spath_follow.py` calls `pursuit.compute_command()`
and checks `pursuit.at_end()` — the same interface works for any controller that
exposes these two methods plus `reset()` / `is_loaded()`.

---

## Common commands

```bash
# Path following (trajectory.csv must exist)
python3 mqtt_client_path_follow.py

# Line following
python3 mqtt_client_core.py --edge

# Drive 1 m
python3 mqtt_client_core.py --meter

# Turn 180°
python3 mqtt_client_core.py --pi

# Stationary (default; exits after 300 s)
python3 mqtt_client_core.py

# Silent mode (no console chatter)
python3 mqtt_client_core.py --edge -s

# Test mode (show only controller prints)
python3 mqtt_client_core.py --edge -t
```

---

## Robot hardware reference

| Parameter | Value | Source |
|-----------|-------|--------|
| Wheelbase | 0.22 m | `robot.ini`, `spose.py:92` |
| Wheel radius | 0.075 m | `robot.ini` |
| Gear ratio | 19:1 | `robot.ini` |
| Encoder ticks/rev | 68 | `spose.py:51` |
| Max recommended velocity | 0.8 m/s | `robot.ini` joy limit |
| Pose update rate | ~5 ms | `robot.ini` interval_pose_ms |
| Line sensor update rate | ~10 ms | `robot.ini` interval_livn_ms |

---

## Notes bundle

Detailed documentation lives in `../notes/`. Key files:

| File | Contents |
|------|----------|
| `overview.md` | Current priorities and what each notes file is for |
| `system_architecture.md` | Full data-flow diagram, hardware summary |
| `change_log.md` | Chronological list of all code changes |
| `line_following.md` | Line-following stack, tuning, calibration |
| `path_following.md` | Pure Pursuit controller design and tuning |
| `commands_and_workflow.md` | Copy-paste command sheet, startup sequence |
| `llm_operator_rules.md` | How to work in LLM sessions; note-update rules |
