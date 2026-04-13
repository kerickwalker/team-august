# Roundabout Mission — Architecture & Tuning Guide

## Overview

The roundabout functionality lets the robot:
1. Follow a straight line until it ends (tape runs out)
2. Transition to driving a fixed-radius circle for a configurable number of degrees

This is implemented as a **standalone test script** (`mqtt_python/mqtt_roundabout_test.py`) so it can be tuned independently before being merged into the main line-follow mission.

---

## File

```
mqtt_python/mqtt_roundabout_test.py
```

Run with:
```bash
cd mqtt_python

python3 mqtt_roundabout_test.py          # waits for IR start-gate
python3 mqtt_roundabout_test.py --now    # skip IR gate, go immediately
python3 mqtt_roundabout_test.py --now -s # silent (no debug prints)
```

---

## Mission Phases

### Phase 1 — `driveLineUntilEnd()`

- Waits for the IR start-gate (same as main mission), then starts the sedge PD line controller (`edge.lineControl(refPosition=0)`)
- Monitors `edge.lineValidCnt` every 10 ms
- Transitions to phase 2 when `lineValidCnt < LINE_LOST_CNT` for `LINE_LOST_CONFIRM` consecutive samples
- The consecutive-sample counter prevents a brief sensor noise dip from triggering the arc too early
- Safety exit after `LINE_FOLLOW_TIMEOUT` seconds if the line never ends

### Phase 2 — `driveRoundabout()`

- Computes turn rate from diameter and velocity: `w = v / r` where `r = CIRCLE_DIAMETER_M / 2`
- Sends a single `rc v w` command; the Teensy maintains it continuously
- Tracks heading change via `pose.tripBh` (odometry, reset at phase start)
- Exits when `abs(pose.tripBh) >= radians(ARC_TOTAL_DEGREES)`
- Waits for `pose.velocity() < 0.001` before returning (full stop)

---

## Tuning Parameters

All parameters are at the top of `mqtt_roundabout_test.py`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `CIRCLE_DIAMETER_M` | `1.0` | Diameter of the circle the robot drives (metres). Measure the physical platform and set this first. |
| `ARC_TOTAL_DEGREES` | `450.0` | Total heading change. 360 = one full lap. Increase for multiple gate passes (e.g. 720 = two laps). |
| `ARC_VELOCITY` | `0.2` | Forward speed on the arc (m/s). Keep low for initial tests. |
| `ARC_DIRECTION` | `"left"` | `"left"` = CCW (positive turn rate), `"right"` = CW (negative turn rate). |
| `LINE_LOST_CONFIRM` | `5` | Number of consecutive 10 ms samples below `LINE_LOST_CNT` before transitioning. Increase to be less sensitive to sensor noise. |
| `LINE_LOST_CNT` | `2` | `lineValidCnt` threshold; a sample is counted as "no line" when below this. |
| `LINE_FOLLOW_TIMEOUT` | `20.0` | Abort line-follow phase after this many seconds (safety). |

---

## How `rc` turn rate is derived

```
radius    = CIRCLE_DIAMETER_M / 2
turn_rate = ARC_VELOCITY / radius          # rad/s, positive = left/CCW
```

The sign is flipped for `ARC_DIRECTION = "right"`. You never set turn rate directly — only diameter and velocity.

---

## Tuning Workflow

### Step 1 — Get the circle right

1. Set `ARC_TOTAL_DEGREES = 360` (one lap)
2. Set `CIRCLE_DIAMETER_M` to the measured physical platform diameter
3. Run `python3 mqtt_roundabout_test.py --now`
4. Watch whether the robot closes the loop:
   - Ends up to the **left** of start → diameter too small (increase it)
   - Ends up to the **right** of start → diameter too large (decrease it)
   - Overshoots/undershoots along the straight → adjust `ARC_VELOCITY`

### Step 2 — Set number of laps

Once the circle closes cleanly, set `ARC_TOTAL_DEGREES` to the required value:
- 1 lap = 360°
- 1.25 laps = 450° (passes through 2 gates if spaced 90° apart)
- 2 laps = 720°

### Step 3 — Tune line-end detection

If the robot starts arcing too early (noise spike on the sensor):
- Increase `LINE_LOST_CONFIRM` (e.g. to 10)

If it takes too long to transition after the line ends:
- Decrease `LINE_LOST_CONFIRM` (e.g. to 3)

---

## Key Modules Used

| Module | Used for |
|--------|---------|
| `sedge.edge` | Line detection (`lineValidCnt`) and PD line controller |
| `spose.pose` | Odometry: `tripBh` (heading change), `tripB` (distance), `velocity()` |
| `sir.ir` | IR start-gate detection (`ir.ir[0] < 0.2`) |
| `sgpio.gpio` | LED status indicators |
| `uservice.service` | MQTT connection, `send()`, `stop`, `is_quiet()`, `args` |

---

## Integration into Main Mission

When ready to integrate into `mqtt-linefollow.py`:

1. Copy `driveRoundabout()` into `mqtt-linefollow.py`
2. In `driveToLine()`, add a new state after line-follow state 10:
   - When `lineValidCnt < LINE_LOST_CNT` for `LINE_LOST_CONFIRM` samples → call `driveRoundabout()`
3. Keep `CIRCLE_DIAMETER_M` and `ARC_TOTAL_DEGREES` as module-level constants at the top of the file
