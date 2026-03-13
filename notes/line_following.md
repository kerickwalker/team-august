# Line Following

## Purpose
This file describes the line-following stack: where the active controller lives, the data flow, the first Python-only patch, and the recommended calibration and test workflow. Magnetometer work is deferred until after line following is in a good state.

## Priority
Line following is the **current focus**. Magnetometer I2C bypass fix comes **after** line following (calibrate, test, tune).

---

## Main architectural conclusion

**The currently active line-following controller is in Python, not on the Teensy.**

### Division of responsibility

#### Teensy firmware
- Line sensor acquisition, calibration storage, normalization
- Line validity / crossing detection
- Onboard line position estimate (`lip`)

Key files:
- `teensy_firmware_8/src/ulinesensor.cpp`
- `teensy_firmware_8/src/ulinesensor.h`

Teensy publishes (among others):
- `liv` = raw sensor data
- `liw` = white calibration values
- `lib` = black calibration values
- `livn` = normalized line sensor values (0–1000 scale)
- `lip` = line position / validity (onboard estimate)

#### teensy_interface
- Reads Teensy messages over USB
- Republishes to MQTT (e.g. `robobot/drive/T0/edge_livn`, `robobot/drive/T0/edge_liv`)
- Logs to build folder

Key file: `teensy_interface/src/steensy.cpp`. Local `sedge.cpp` is mostly logging/subscription support, not the high-level controller.

#### Python MQTT client (active controller)
- Subscribes to normalized line sensor values (`T0/livn`)
- **Recomputes** line position from `livn` (does **not** use Teensy `lip`)
- Computes steering and sends `rc v w` over MQTT

Key files:
- `mqtt_python/sedge.py` — line detection and control
- `mqtt_python/mqtt-client.py` — mission start/stop, e.g. `driveToLine()` and `--edge`

**Active control path:** Teensy line sensor → teensy_interface (USB/MQTT) → Python line follower → `rc` commands back to robot.

---

## How the current (pre-patch) system works

### Sensor startup (sedge.py)
- `lip 1` to turn line sensor on
- `sub livn 10` to request normalized updates at 10 ms
- Uses `liwi` for one-shot white calibration request

### Original Python line detection
- Uses **brightest sensor** and **average** over 8 sensors
- Line valid if brightest exceeds threshold; crossing if average exceeds threshold
- `posLeft` / `posRight` from scanning from each side until above threshold  
→ **Coarse threshold-based edge follower**, not a continuous center tracker.

### Controller and mission
- `followLine()`: error from `refPosition - posLeft` (left edge) or `posRight` (right edge); P + lead filter; steering saturated at ±4 rad/s.
- `driveToLine()` (mqtt-client.py): drive forward until `edge.lineValidCnt > 4`, then `edge.lineControl(0.2, True)` → **0.2 m/s, left edge**. Stops when `lineValidCnt < 2`.

**Behavior:** Drive toward line → detect line → follow left edge → stop when line lost.

### Important comparison with firmware
The Teensy already computes a **smoother** line estimate in `ulinesensor.cpp` (e.g. weighted position / center-of-gravity). The current Python code **ignores** that and uses a rougher threshold-based estimate from `livn`. So the active follower is coarser than the data the lower layers already provide.

---

## Firmware flashing

**No Teensy reflashing is needed for the first improvements.**

The weak points are in Python (rough edge estimate, simple lost-line behavior, control logic in `sedge.py`). Flashing would only be needed later if we change Teensy-side line processing, use or change the onboard `lip`, or move the control loop onto the Teensy (e.g. `udemo_behave.cpp`).

---

## First Python-only patch (implemented)

### Files edited
- `mqtt_python/sedge.py`
- `mqtt_python/mqtt-client.py`

### Improvements in sedge.py
1. **Smoother line position:** New state `lineCenter`, `lineCenterRaw`, `lineWidth`, `followMode`. Weighted center-of-gravity from sensor intensities above background; low-pass filtered with `linePosAlpha`. Still keeps `posLeft` / `posRight` from active sensor span.
2. **followMode:** Controller can follow `"center"`, `"left"`, or `"right"` instead of only left/right edge.
3. **Short dropout recovery:** `lostLineCnt`, `maxLostLineCnt`, `recoveryTurnGain` — softened previous steering and small recovery turn; stop only after line lost too long.
4. **Steering saturation:** `maxTurnRate` reduced from 4 to **2.5 rad/s** for safer initial tests.
5. **Visualization:** `paint()` can show `posLeft`, `posRight`, `lineCenter` for debugging.

### Change in mqtt-client.py
- `driveToLine()` now calls `edge.lineControl(0.2, True, 0.0, "center")` — follow **line center** instead of left edge.

### Expected effect
- Smoother steering, less sensor-to-sensor jump, less oscillation
- Better tolerance to brief line loss
- No firmware flash required to test

### Caveat
Patch does not yet clean up noisy debug prints in `followLine()` and `paint()`. Before live testing, consider reducing console spam and centralizing tuning parameters.

### Test print format (`-t` or when not `-s`)
When line-following is active, a combined two-line block is printed every 10th livn update (if `--test` or not `--silent`):
- **Line 1:** `% line: livn [ 8 values ] avg=... high=... valid=... validCnt=...`
- **Line 2:** `%       posL=... posR=... cross=... | e=... u=... y=... -> rc <velocity> <turn>`

Fields: **livn** = normalized 0–1000 (Teensy: raw → subtract black, × gain, ×1000). **posL/posR** = line position index -3.5..3.5 (left/right edge of detected line). **e** = error, **u** = P output, **y** = turn rate (pure P); **rc** is the command sent (no timestamp in print). No print when not line-following (e.g. during drive-to-line).

---

## Calibration and testing workflow (recommended order)

**Calibration is persistent.** White (and black) levels are stored in **Teensy EEPROM** when you run `--white`. The client runs a **rolling calibration** (drive forward/back over the line, collect per-sensor max, send `litw` then `eew`). Calibrate **once** (or when conditions change: different tape, lighting, or surface). The “calibrate first” step below is for a fresh robot or after such a change.

If the robot was already calibrated and conditions are unchanged, you can skip Step 2 and run `--meter` then `--edge` directly.

### Step 1: Interface and directory
Ensure `teensy_interface` is running (see `commands_and_workflow.md`), then:

```bash
cd ~/svn/robobot/mqtt_python
```

### Step 2: White calibration
```bash
python3 mqtt-client.py --white
```
(Uses built-in calibration; aligns with wiki calibration flow.)

### Step 3: Motion sanity check
```bash
python3 mqtt-client.py --meter
```
Verifies drive/stop before mixing in line-following issues.

### Step 4: Line-follow mode
```bash
python3 mqtt-client.py --edge
```

### During `--edge`, observe
1. Line detection reliability
2. Left–right oscillation
3. How easily the line is lost
4. Whether `livn` shows clear separation between tape and background

Then compare original vs patched behavior (if patch is applied).

---

## Planned changes (line following)

**Order of implementation:** (1) PID controller → (2) Behavior (recovery, speed) → (3) Lead / lowpass later if needed.

### 1. PID controller — done
- **Implemented:** Integral (I) and derivative (D) added in `sedge.py`; gains `lineKp`, `lineKi`, `lineKd` (defaults 0.5, 0, 0).
- Integral clamped to ±`lineIntegralLimit`; reset when line lost. Derivative from `(e - e_prev)/dt`; output clamped to ±4 rad/s. Test print shows e, p, i, d, u, y.

### 2. Behavior options (after PID)
Implement as chosen; not all required.

**A. Finding the line again when driving off**
- **A1.** When line lost: stop forward, turn in place (alternate left/right or one direction) until line seen again, then resume.
- **A2.** Sweep search: turn 90° one way, then 180° the other (bounded search).
- **A3.** Slow forward + turn: while lost, drive slowly and add constant turn (spiral/curve) until line reacquired.
- **A4.** Remember last side: turn toward the side the line was last on (from last `lineCenterWeighted` or `e` sign).

**B. Speed vs confidence**
- **B1.** Scale forward speed with `lineValidCnt` (e.g. full speed when confident, reduce when low, minimal when just reacquired).
- **B2.** Reduce speed when `|e|` is large (slow in curves, fast on straights).
- **B3.** Combined: speed from both `lineValidCnt` and `|e|`.

**C. When to stop vs keep trying**
- **C1.** Timeout on loss: if line lost for N seconds, stop (and optionally run search A1/A2).
- **C2.** Distance without line: stop after X m without valid line (if odometry available).
- **C3.** No timeout: keep current behavior.

**D. Smoother re-entry**
- **D1.** Ramp speed up when line reacquired (e.g. over 0.5–1 s).
- **D2.** When using PID: reset integral term when line is lost (avoid windup during lost period).

**E. Crossing / junctions**
- **E1.** Use `crossingLine` to detect T-junctions; slow or pause and choose direction (left/right/straight).
- **E2.** Ignore crossing: keep current behavior.

### 3. Lead and lowpass (later, if needed)
- **Lead:** Re-add phase-lead filter (smooths P output, adds slight anticipation). Add only after PID and behavior are satisfactory; tune `tauZ`, `tauP` with clear goal.
- **Lowpass:** Optional low-pass on weighted center or turn rate to reduce jerk; add one at a time and document.

---

## Wiki references
- Robobot calibration guide: white-level calibration; normalized scale 0 = black, 1000 = calibrated white; Python uses rolling motion and `litw` then `eew` to save.
- Robobot MQTT-client line-drive guide: line drive uses normalized `livn`; each update can produce a new turn-rate; line drive ends when velocity set to zero; notes that code can use edge estimates or center-of-gravity.

---

## Short handover summary
- Active line follower: **Python** (`sedge.py`, `mqtt-client.py`).
- Teensy: sensor read, calibration, normalization, onboard `lip` (not used by current Python).
- `teensy_interface`: forwards Teensy messages (e.g. `livn`) to MQTT.
- Original Python: coarse threshold-based edge from `livn`; patch adds weighted center, center-follow mode, dropout recovery, lower turn saturation.
- No firmware flash for first improvements. Test order: `--white` → `--meter` → `--edge`; then compare with/without patch.
