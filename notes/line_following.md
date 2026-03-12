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
