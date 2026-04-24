# Full Mission — Architecture & Tuning Guide

**Script:** `mqtt_python/mqtt-full-mission.py`

---

## Mission overview

The full mission combines line-following, a roundabout, and a final crossing sequence
into one script. It is the primary competition script for the robot.

```
[INIT]        drive forward → find line → start line following

[FOLLOW]      follow line, react at each crossing:
                crossing 1 → turn LEFT 45°, resume following
                crossing 2 → go straight
                crossing 3 → go straight
                crossing 4 → hard 90° turn (direction auto-detected)
                crossing 5 → END SEQUENCE

[ROUNDABOUT]  triggered when line physically ends (between crossings 1 and 2):
                drive 20 cm → turn LEFT 45° → arc 450° → turn LEFT 90°
                → find line → resume FOLLOW with crossing_count = 1

[END]         nudge clear → right 45° → drive 1.1 m → right 90° → drive 0.15 m
              → drop flag → stop
```

---

## Crossing-count mapping

The `crossing_count` variable is 0-based internally (increments when the robot *leaves*
a crossing). The table below shows the relationship between the physical crossing number
and what the robot does.

| Physical crossing | `crossing_count` when at it | Action |
|---|---|---|
| 1 | 0 | Turn LEFT 45° (one-time, `first_cross_done` flag) |
| 2 | 1 | Go straight |
| 3 | 2 | Go straight |
| 4 | 3 | Hard 90° turn, direction from sensor |
| 5 | 4 | Stop → end sequence |

After the roundabout, `crossing_count` is reset to `1` so the robot treats
crossing 2 as the next one to handle.

---

## How the code is structured

### Configuration (top of file)

All tunable values are grouped into `SimpleNamespace` objects, one per mission phase.
You only need to edit this section — the mission logic below never needs to change.

```
LINE              — line-following speed (normal + approach), and line-loss timeout
CROSSINGS         — sensor threshold, debounce timing, go-straight window, stop crossing
CROSSING1         — turn angle, direction, and rate for the crossing-1 manoeuvre
ROUNDABOUT_ENTRY  — drive distance and turn before entering the arc
ROUNDABOUT_ARC    — circle diameter, total degrees, speed, direction
ROUNDABOUT_EXIT   — turn angle and direction after arc, seek speed and timeout
LINE_END          — debounce parameters for detecting the physical end of the line
END               — all distances, angles, speeds, and servo positions for the end sequence
```

`LINE` carries two speed fields:
- `speed = 0.25` — normal line-following speed (used after roundabout and on all return-leg crossings)
- `approach_speed = 0.15` — reduced speed used from after crossing 1 until the roundabout is complete

Example — to change the crossing-1 turn from 45° to 30°:
```python
CROSSING1 = SimpleNamespace(
    turn_deg  = 30.0,   # ← change here only
    turn_dir  = "left",
    turn_rate = 0.5,
)
```

### Sequence lists (middle of file)

The two blocking phases are defined as plain ordered lists of steps:

```python
ROUNDABOUT_SEQUENCE = [
    ("drive",     {"dist": 0.20,  "speed": 0.15}),
    ("turn",      {"deg":  45.0,  "dir":   "left"}),
    ("arc",       {}),
    ("turn",      {"deg":  90.0,  "dir":   "left"}),
    ("seek_line", {"speed": 0.15, "timeout": 10.0}),
]
```

To add, remove, or reorder a step — just edit the list. The `run_sequence()` dispatcher
handles execution. The state machine does not need to change.

Supported step actions:

| Action | Required params | Optional params |
|---|---|---|
| `"drive"` | `dist` (m) | `speed` (m/s, default 0.2) |
| `"turn"` | `deg` (°), `dir` (`"left"`/`"right"`) | `rate` (rad/s, default 0.5) |
| `"arc"` | — | — (uses `ROUNDABOUT_ARC` settings) |
| `"seek_line"` | `speed` (m/s), `timeout` (s) | — |
| `"servo"` | `pos` (PWM value) | `hold` (s — waits after moving) |

### State machine (`driveMission`)

| State | What it does |
|---|---|
| 0 | Wait for IR gate (or `--now`), then drive forward at 0.2 m/s |
| 1 | Drive until line sensor is confident (`lineValidCnt > 4`), then start PD controller |
| 2 | Wait for full stop, then exit |
| 10 | Line following + crossing reactions + line-end detection |
| 11 | Run `ROUNDABOUT_SEQUENCE`, reset `crossing_count = 1`, resume state 10 |
| 20 | Run `END_SEQUENCE`, then → state 2 |
| 99 | Mission complete |

---

## State 10 in detail

State 10 is the main reactive loop (100 Hz). Three things run in parallel every cycle:

### 1. Crossing detection

`edge.crossingLineCnt` counts how many crossing lines are currently active.
When it reaches `CROSSINGS.sensor_threshold` (default 2), the robot is considered
to be *at* a crossing.

The count only increments **on leaving**, after the robot has been clear of the
crossing for `CROSSINGS.leave_delay_s` (default 0.5 s). This prevents double-counting
from sensor jitter.

### 2. Crossing reactions

Evaluated only while `at_crossing == True`:

- **Crossing 1** (`crossing_number == 1`, `first_cross_done == False`):
  stop line controller → `driveTurn(45°, left)` → resume line controller at
  `LINE.approach_speed` (0.15 m/s) with `params="slow"` PID gains.
  The flag `first_cross_done` prevents re-entry.
  Speed and PID gains remain in slow mode until the roundabout finishes.

- **Crossings 2, 3** (`crossing_count` 1 and 2, below `CROSSINGS.go_straight_until = 3`):
  no action — PD controller keeps the robot on the line.

- **Crossing 4** (`crossing_count >= CROSSINGS.go_straight_until`):
  hard 90° turn. Direction is auto-detected:
  - left-end sensors lit → turn **left**
  - right-end sensors lit → turn **right**
  A 2 s cooldown (`CROSSINGS.hard_turn_cooldown_s`) prevents the same crossing
  from triggering twice.

- **Crossing 5** (`crossing_count == CROSSINGS.stop_at - 1 = 4`):
  stop line controller → state 20.

### 3. Line-end / line-loss

**Before roundabout** (`roundabout_done == False`):
Counts consecutive samples where `edge.lineValidCnt < LINE_END.lost_cnt`.
After `LINE_END.lost_confirm` (5 × 10 ms = 50 ms) consecutive low-confidence samples,
the line is declared ended → state 11.

**After roundabout** (`roundabout_done == True`):
Standard line-loss recovery: if the line disappears for longer than `LINE.lost_timeout`
(5 s), stop the robot → state 2. This is a safety fallback on the return leg.

---

## Primitive drive helpers

| Function | What it does |
|---|---|
| `driveTurn(deg, dir, rate=0.5)` | Rotate in place; stops when `pose.tripBh` reaches target |
| `driveDistance(dist, speed=0.2)` | Drive straight; stops when `pose.tripB` reaches target |
| `driveRoundabout()` | Drive arc using `ROUNDABOUT_ARC`; stops when `pose.tripBh` reaches target |
| `seekLine(speed, timeout_s)` | Drive forward until `lineValidCnt > 4` or timeout |
| `run_sequence(steps)` | Execute a list of `(action, params)` steps in order |

All primitives use **odometry feedback** (`pose.tripB`, `pose.tripBh`), not timers.
The trip counters are reset at the start of each call with `pose.tripBreset()`.

---

## Tuning workflow

### Adjusting a distance or angle

Find the relevant `SimpleNamespace` block at the top of the file and change the value:

```python
ROUNDABOUT_ENTRY = SimpleNamespace(
    drive_dist_m = 0.25,   # was 0.20 — increased to clear a wider gap
    ...
)
```

### Changing the roundabout circle size

Only `ROUNDABOUT_ARC.diameter_m` needs to change. Turn rate is derived automatically:
```
w = speed / (diameter_m / 2)
```
Never set turn rate by hand.

### Adding a step to a sequence

Insert a tuple in the appropriate list:
```python
ROUNDABOUT_SEQUENCE = [
    ("drive",  {"dist": 0.20, "speed": 0.15}),
    ("turn",   {"deg": 45.0,  "dir":  "left"}),
    ("drive",  {"dist": 0.10, "speed": 0.10}),   # ← new step: extra nudge
    ("arc",    {}),
    ...
]
```

### Changing the crossing-1 turn angle or direction

```python
CROSSING1 = SimpleNamespace(
    turn_deg  = 60.0,     # sharper turn
    turn_dir  = "right",  # now turns right instead
    turn_rate = 0.4,      # slower
)
```

### Tuning PID gains per speed mode

`sedge.py` holds two named PID sets in `SEdge.PARAM_SETS`:

```python
PARAM_SETS = {
    "normal": dict(lineKp=0.4, lineKi=0.0, lineKd=0.1,
                   lineIntegralLimit=2.0, lineOutputAlpha=0.4),
    "slow":   dict(lineKp=0.4, lineKi=0.0, lineKd=0.1,
                   lineIntegralLimit=2.0, lineOutputAlpha=0.4),
}
```

- `"normal"` is applied at mission start and after the roundabout (`LINE.speed = 0.25 m/s`).
- `"slow"` is applied after crossing 1 (`LINE.approach_speed = 0.15 m/s`) and stays active
  through the roundabout approach.

Both sets start with identical values. Tune `"slow"` independently — for example, higher `lineKp`
compensates for the reduced speed, and lower `lineOutputAlpha` gives a sharper response.
Changing `"slow"` has no effect on the return-leg or post-roundabout behaviour.

### Adjusting crossing thresholds

To make crossings 2, 3, and 4 all go straight (turn only at crossing 5):
```python
CROSSINGS = SimpleNamespace(
    go_straight_until = 4,   # was 3
    stop_at           = 6,   # was 5 — adjust if more crossings exist
    ...
)
```

---

## Running the script

```bash
cd mqtt_python

# Normal run (waits for IR gate)
python3 mqtt-full-mission.py -e

# Skip IR gate, start immediately
python3 mqtt-full-mission.py -e --now

# Silent (no debug prints)
python3 mqtt-full-mission.py -e -s

# Both
python3 mqtt-full-mission.py -e --now -s
```

---

## Relationship to other scripts

| Script | Relationship |
|---|---|
| `mqtt-linefollow.py` | Implements crossings 1-4 as a standalone script (no roundabout). `mqtt-full-mission.py` extends this with the roundabout phase and adjusts the crossing count accordingly. |
| `mqtt_roundabout_test.py` | Standalone roundabout test. The arc logic in `mqtt-full-mission.py` (`driveRoundabout`) uses the same physics. |
| `sedge.py` | Provides `edge.lineControl(velocity, refPosition, params="normal")`, `edge.lineValidCnt`, `edge.crossingLineCnt`, and the sensor index fields used for hard-turn direction detection. `PARAM_SETS` in `SEdge` holds named PID gain sets (`"normal"`, `"slow"`); `lineControl()` applies the selected set and resets the integral on every call. |
| `spose.py` | Provides `pose.tripB`, `pose.tripBh`, `pose.tripBreset()` used by all drive primitives. |
