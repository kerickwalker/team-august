# Path Following

## Purpose
This file documents the Pure Pursuit path-following controller added to the robot.
It covers architecture, algorithm, configuration, calibration workflow, tuning, and edge
cases.  For line following see `line_following.md`.

---

## Overview

The path-following stack lets the robot track a pre-generated trajectory stored as a CSV
file.  It uses the **Pure Pursuit** algorithm, which steers the robot toward a lookahead
point on the trajectory rather than tracking the nearest point directly.  This gives
smooth, lag-tolerant steering that handles curves naturally.

The pose source is the **Kalman-filtered state** published on `robobot/kalman/state`,
which is more accurate than raw encoder odometry for closed-loop trajectory tracking.

---

## File structure

| File | Role |
|------|------|
| `mqtt_python/spursuit.py` | Pure Pursuit math — no MQTT, fully standalone |
| `mqtt_python/spath_follow.py` | MQTT integration — Kalman subscriber + `rc` publisher |
| `mqtt_python/mqtt_client_path_follow.py` | **Entry point** — run this file to start the mission |
| `mqtt_python/trajectory.csv` | Input trajectory — place here before running |

---

## Trajectory CSV format

```
x,y,heading,cumulative_dist
0.000,0.000,0.000,0.000
0.050,0.001,0.020,0.050
0.100,0.003,0.020,0.100
...
```

| Column | Unit | Description |
|--------|------|-------------|
| `x` | m | World-frame x position |
| `y` | m | World-frame y position |
| `heading` | rad | Robot heading (same convention as Kalman `yaw`) |
| `cumulative_dist` | m | Arc length from the first point |

**Arc-length spacing:** ~0.05 m per row recommended.  Coarser spacing degrades lookahead
interpolation; finer spacing increases file size with no benefit.

Place the file at `mqtt_python/trajectory.csv`, or change `TRAJECTORY_CSV` at the top
of `spath_follow.py`.

---

## Pose source: `robobot/kalman/state`

The Kalman node publishes on this topic at whatever rate it runs.  Two JSON schemas are
supported (handled identically in `spath_follow._on_kalman_message()`):

**Schema A** (nested under `"x"`):
```json
{"x": {"x": 1.20, "y": 0.50, "yaw": 0.34,
       "velocity": 0.30, "angular_velocity": 0.10}}
```

**Schema B** (flat sub-objects):
```json
{"position": {"x": 1.20, "y": 0.50},
 "orientation": {"yaw": 0.34},
 "velocity": {"linear": 0.30, "angular": 0.10}}
```

If no Kalman message has arrived when `update()` is called, the controller does nothing
until the first message is received (`_pose_received` flag).

---

## Algorithm — Pure Pursuit

### Step-by-step

```
Given: current pose (x, y, heading), trajectory array, LOOKAHEAD_DIST

1. Nearest point
   Search forward from _nearest_idx (bounded by SEARCH_WINDOW) for the
   trajectory point with minimum Euclidean distance to the robot.
   Advance _nearest_idx to that point.
   → This is O(SEARCH_WINDOW) not O(N); the robot cannot jump backward.

2. Lookahead point
   Walk forward from _nearest_idx until
       traj[i].cumulative_dist  >=  traj[_nearest_idx].cumulative_dist + LOOKAHEAD_DIST
   Use that point (x_l, y_l) as the steering target.
   If the trajectory ends before the lookahead distance, use the final point.

3. Transform to robot frame
   dx_w = x_l − x          (world frame offset)
   dy_w = y_l − y
   dx_r =  cos(heading) * dx_w + sin(heading) * dy_w   (forward in robot frame)
   dy_r = −sin(heading) * dx_w + cos(heading) * dy_w   (left in robot frame)

4. Curvature
   κ = 2 · dy_r / (dx_r² + dy_r²)

   Derivation: for a chord of length L to a point offset laterally by d,
   κ = 2d/L².  Using dx_r and dy_r generalises this to any lookahead direction.

5. Output
   turnrate = clamp(κ · MAX_LINVEL,  −MAX_TURNRATE, +MAX_TURNRATE)
   linvel   = MAX_LINVEL  (constant speed)
   → published as: rc <linvel> <turnrate>
```

### End condition
When `_nearest_idx >= len(trajectory) − 1`, `at_end()` returns `True`.  The mission
function then calls `pathControl(0)` and sends `rc 0.0 0.0`.

---

## Coordinate convention

The stack uses **two** y-axis conventions that must be kept consistent:

| Layer | +y direction | File |
|-------|-------------|------|
| Robot Kalman state (internal) | **Left** (standard math, CCW positive) | `skalman.py`, `spose.py`, `simu.py` |
| Trajectory CSV | **Right** (`+y` = right, `-y` = left) | `trajectory.csv` |
| Status display (silent mode) | **Right** (negated before printing) | `mqtt_client_path_follow.py` |

The Pure Pursuit controller bridges these by **negating trajectory y** inside `compute_command()` before the robot-frame transform (`spursuit.py`, steps 1 and 2).  The Kalman heading follows the same flip: displayed heading is `-Kalman_yaw`.

**rc command sign convention:**
- Positive angular velocity → turn **left**
- Negative angular velocity → turn **right**

---

## Tunable parameters

All in `spursuit.py` (top of file):

| Constant | Current value | Effect |
|----------|---------|--------|
| `LOOKAHEAD_DIST` | 1.0 m | Distance ahead on path to aim for. Larger = smoother but cuts corners more. Smaller = more precise but oscillates on tight curves. Rule of thumb: `LOOKAHEAD_DIST ≥ turn_radius` of tightest curve. |
| `MAX_LINVEL` | 0.3 m/s | Fallback speed if `pathControl()` is not used. In practice the speed is set by `path_follow.pathControl(v)` which passes it through `compute_command(..., linvel=v)`. |
| `MAX_TURNRATE` | 1.0 rad/s | Saturation clamp on angular velocity. At 0.15 m/s, >1.5 rad/s causes the inner wheel to spin backwards. |
| `SEARCH_WINDOW` | 30 pts | Max points scanned forward per step (~1.5 m at 0.05 m spacing). |

**Setting speed at runtime** — use `pathControl()`, not the constant:
```python
path_follow.pathControl(0.15)   # 0.15 m/s
```
This value is forwarded to `compute_command()` as `linvel` and overrides `MAX_LINVEL`.

The trajectory file path is in `spath_follow.py`:
```python
TRAJECTORY_CSV = "trajectory.csv"
```

---

## How to run

### Prerequisites
1. `teensy_interface` must be running.
2. The Kalman filter node must be publishing on `robobot/kalman/state`.
3. `trajectory.csv` must exist in `mqtt_python/`.

### Start the mission
```bash
cd mqtt_python
python3 mqtt_client_path_follow.py
```

### Silent mode — clean status output at 10 Hz
```bash
python3 mqtt_client_path_follow.py -s
```
Suppresses all MQTT/service noise and instead prints one status line per 100 ms:
```
x=0.123 m  y=-0.045 m  hdg=0.038 rad  rc=0.150 m/s  0.250 rad/s
```
- `y` and `hdg` are displayed in the **user convention** (+y = right, positive hdg = turned right)
- `rc` turnrate sign: positive = turning left, negative = turning right

### Debug mode — verify Kalman pose without moving the robot
```bash
python3 mqtt_client_path_follow.py --debug   # or -d
```
Connects to the broker, subscribes to `robobot/kalman/state`, and prints the pose at ~1 Hz.
No `rc` commands are sent.  Use this to confirm the Kalman filter is publishing before
attempting a full run.

Example output:
```
% Debug mode — printing Kalman pose. Ctrl-C to stop.
% Kalman pose:  x=0.000 m  y=0.000 m  heading=0.0000 rad  speed=0.000 m/s
```

### Monitor commands (separate terminal)
```bash
mosquitto_sub -t "robobot/cmd/ti" -v
```
You should see lines like `rc 0.300 1.250` at ~50 Hz once tracking starts.

### Verify Kalman pose is arriving
```bash
mosquitto_sub -t "robobot/kalman/state" -v
```

---

## Tuning guide

### Oscillation / spinning on turns
Increase `LOOKAHEAD_DIST`.  Pure Pursuit is unstable when `LOOKAHEAD_DIST < turn_radius`.
At 0.15 m/s with a 0.9 m turn radius, 1.0 m lookahead is the minimum stable value.
Also ensure `MAX_TURNRATE` is ≤ 1.0 rad/s at low speeds (higher values cause wheel slip and heading overshoot).

### Oscillation / weaving on straights
Reduce `LOOKAHEAD_DIST` (robot reacts to more local path shape) OR reduce `MAX_LINVEL`.
Typical oscillation means the lookahead is too long relative to the straight-line error.

### Cutting corners
Increase `LOOKAHEAD_DIST`.  Cutting corners means the robot is turning late because the
lookahead point reaches around a bend before the robot does.

### Tracking errors at high speed
Either reduce `MAX_LINVEL` or tighten the trajectory arc spacing so there are more
waypoints per metre.

### Robot doesn't respond to turns
Check that `dy_r` is non-zero — if the coordinate frame is wrong (heading sign, axis
order), lateral error will be zero and the robot will drive straight regardless.  Verify
by printing `dx_r`, `dy_r` from `compute_command()` while driving a known curve.

### Stops too early
Check `at_end()`: `_nearest_idx` reaching the last point before the robot is actually
there.  Increase `SEARCH_WINDOW` or add extra endpoint copies at the end of the CSV.

---

## Edge cases and safety

| Situation | Behaviour |
|-----------|-----------|
| Kalman pose not yet received | `update()` silently does nothing until first message |
| Trajectory file missing or malformed | `load_trajectory()` prints a warning; `compute_command()` returns `(0.0, 0.0)` |
| Trajectory fewer than 2 points | Rejected; same safe return |
| Lookahead extends past trajectory end | Uses the final waypoint as target |
| Lookahead point collapses onto robot (l² < 1e-6) | Returns `(MAX_LINVEL, 0.0)` — drive straight |
| `at_end()` True when called | Returns `(0.0, 0.0)` immediately |
| MQTT broker unreachable at `setup()` | Prints a warning; controller waits; rest of system is unaffected |
| Ctrl-C during tracking | `uservice.terminate()` → `path_follow.terminate()` → disarm + MQTT disconnect + `rc 0 0` sent |

---

## Swapping to a Stanley controller

Only `spursuit.py` needs to be replaced.  Keep the same module-level singleton:
```python
pursuit = SStanleyController()
```
and expose the same interface:
```python
def load_trajectory(csv_path) -> bool
def compute_command(x, y, heading, speed) -> tuple[float, float]
def reset()
def at_end() -> bool
def is_loaded() -> bool
```
`spath_follow.py`, `mqtt_client_core.py`, and `uservice.py` need no changes.

---

## Architecture diagram

```
robobot/kalman/state (JSON)
          │
          ▼
  spath_follow._on_kalman_message()
          │  updates _x, _y, _heading, _speed
          │
  mission loop (mqtt_client_core.drivePathFollow)
    t.sleep(0.02) ~50 Hz
          │
          ▼
  path_follow.update()
          │
          ▼
  pursuit.compute_command(x, y, heading, speed)   ◄── trajectory loaded from CSV
          │  spursuit.py: nearest point → lookahead → robot-frame transform → κ
          │
          ▼
  service.send("robobot/cmd/ti",  "rc 0.300 1.250")
          │
          ▼
  teensy_interface → Teensy → motors
```
