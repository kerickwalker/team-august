# Change Log

Entries are in chronological order (oldest first, newest at bottom).

## 2026-03-10: Button-6 GPIO Fix and Quiet Mode Implementation

### Button-6 stop detection fix in `sgpio.py`

**Problem**
- Button-6 on GPIO pin 6 always returned `False`, so the stop button did not work.

**Root causes**
- `test_stop_button()` used a hardcoded `return False`
- `get_value()` used a hardcoded `return False`
- GPIO pin 6 was configured as input without a pull-down resistor, so it could float high

**Solution**
- changed `test_stop_button()` to return the actual GPIO value
- changed `get_value()` to return `v == 1`
- configured GPIO pin 6 with pull-down:
  ```python
  GPIO.setup(6, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
  ```

**Modified file**
- `/home/local/svn/robobot/mqtt_python/sgpio.py`

### Quiet mode implementation in `uservice.py`

**Problem**
- Console output was too verbose even when using `--silent`, which made data-collection sessions hard to read.

**Root causes**
- `send()` printed every MQTT message
- `on_messageOut()` printed every outgoing MQTT channel message
- `decode()` printed warnings for unhandled messages

**Solution**
Wrapped the verbose prints with:
```python
if not self.args.silent:
```

Affected areas:
- `send()` around line 279
- `on_messageOut()` around line 236
- `decode()` around line 275

**Modified file**
- `/home/local/svn/robobot/mqtt_python/uservice.py`

### Encoder topic handler in `spose.py`

**Problem**
- The newer encoder MQTT topic `robobot/drive/T0/enc` produced "message not used" warnings because Python had no decoder case for it.

**Solution**
Added a decoder branch:
```python
elif topic == "T0/enc":
  # Encoder data: timestamp encoder_left encoder_right
  # Currently just acknowledging receipt, not storing
  pass
```

**Modified file**
- `/home/local/svn/robobot/mqtt_python/spose.py`

### Default stationary mode in `mqtt-client.py`

**Problem**
- The robot used to default into motion, which was unsafe for testing while powered from a wall socket without a battery.

**Solution**
- changed `loop()` so that the default state becomes state 200
- state 200 sends `rc 0.0 0.0`
- motion states are only entered when explicit flags are provided:
  - `-m`
  - `-p`
  - `-e`
  - `-u`

**Modified file**
- `/home/local/svn/robobot/mqtt_python/mqtt-client.py`

### Testing status after these changes
- Button-6 stop detection working correctly
- Quiet mode suppressing verbose output
- Encoder warnings resolved
- Stationary mode safe operation verified

## 2026-03-10: Master Handover Fix and Stationary Timeout

### Master-claim grace period in `uservice.py`

**Problem**
- `mqtt-client` could quit immediately with "I am not robot master, quitting!" when `teensy_interface` was still publishing a stale master ID.

**Solution**
Added a 5-second grace period before confirming non-master status.

New variables mentioned in the notes:
- `masterMismatchCnt`
- `masterMismatchFirst`
- `masterClaimGraceSec = 5.0`

Grace logic added around lines 268–283:
- count consecutive mismatches
- measure elapsed mismatch duration
- only confirm non-master after the grace interval

**Effect**
- the client tolerates brief master-ID mismatch periods during interface timeout or recovery

**Modified file**
- `/home/local/svn/robobot/mqtt_python/uservice.py`

### Stationary mode auto-timeout in `mqtt-client.py`

**Purpose**
Enable unattended stationary runs that stop automatically after 60 seconds.

**Implementation**
Added logic in state 200:
```python
if self.stateTimePassed() > 60.0:
    self.toExit = True
```

**Usage**
```bash
python3 mqtt-client.py -s
```

**Modified file**
- `/home/local/svn/robobot/mqtt_python/mqtt-client.py`

## 2026-03-10: Added gyro-yaw stream `yawg`

### Purpose
Provide a yaw signal derived from gyro integration so it can later be fused with encoder heading.

### What was added

#### Firmware
File:
- `teensy_firmware_8/src/uimu2.cpp`

Published subscription item:
```text
yawg yaw yaw_rate t
```

Definitions:
- `yaw`: integrated `gyro[2]` in radians, wrapped to `±π`
- `yaw_rate`: `gyro[2]` converted to rad/s

#### Interface
File:
- `teensy_interface/src/simu.cpp`

New log file:
- `log_t0_yawg.txt`

Columns:
1. host timestamp in seconds
2. `yaw` in radians
3. `yaw_rate` in rad/s

#### Python decoder
File:
- `mqtt_python/simu.py`

New topic:
- `T0/yawg`

### Config keys in `[imu1teensy0]`
- `interval_yawg_ms = 12`
- `print_yawg = false`

### Important interpretation
- `pose[2]` remains encoder heading
- `yawg` is gyro-only yaw, so it drifts over time
- intended use is fusion in MATLAB or a Kalman workflow

## 2026-03-11: System verification after git revert

### Problem
After several git reverts, it was unclear whether all recent fixes were still present and working.

### Solution
Performed a full re-verification of the relevant modules and features.

### Verified as working
- Button-6 stop detection
- Encoder streaming and logging
- Quiet mode and stationary mode
- Master-claim grace period
- `yawg` stream presence and efficiency
- `robot.ini` no longer being overwritten

### Result
All implemented fixes discussed in the notes were confirmed to still be working after the revert, and the system was considered stable and ready for new work.

## 2026-03-12: Magnetometer debug – AK8963 unreachable, proposed bypass fix

### Problem
The magnetometer MQTT topic `robobot/drive/T0/mag` produced only zeros (e.g. `0 0 0` or `0.000000 0.000000 0.000000`). Gyro and accelerometer worked normally; the issue was isolated to the magnetometer path.

### Verification
- Magnetometer handling is in `teensy_firmware_8/src/uimu2.cpp` (runtime block under `if (useMag)` with `mpu.magUpdate()` and `mag[0..2]`).
- Debug prints in `initMpu()` confirmed the correct IMU module is used and magnetometer init runs.
- Raw serial monitoring (`cat /dev/ttyACM0`) with `teensy_interface` stopped showed custom probe output.

### Serial probe result
A runtime I2C probe to the AK8963 (address `0x0C`) was added. Output:

```text
# AK8963 probe endTx=2 n=0 who=0xffffffff
```

- `endTx=2`: I2C NACK on address.
- `n=0`: no bytes returned.
- `who=0xffffffff`: no valid WHO_AM_I response.

**Conclusion:** The AK8963 magnetometer is not reachable on I2C address `0x0C`. Failure is below `magUpdate()`, at the I2C layer.

### Suspected cause
The AK8963 is only accessible when the MPU9250 register **INT_PIN_CFG (0x37)** has **BYPASS_EN = 1**. If bypass is not enabled, the host MCU cannot talk to the AK8963. The library call `mpu.magEnableSlaveMode()` may not be setting this correctly on this platform.

### Proposed fix (not yet applied)
In `initMpu()` in `teensy_firmware_8/src/uimu2.cpp`, replace reliance on `mpu.magEnableSlaveMode()` with direct register configuration before starting the magnetometer:

```cpp
if (useMag)
{
  // enable I2C bypass so AK8963 is visible
  Wire.beginTransmission(0x68);
  Wire.write(0x37);   // INT_PIN_CFG register
  Wire.write(0x02);   // BYPASS_EN bit
  Wire.endTransmission();

  delay(10);

  mpu.beginMag(MAG_MODE_CONTINUOUS_100HZ);
}
```

After fix, probe output should show something like `endTx=0 n=1 who=0x48` (0x48 is AK8963 WHO_AM_I), and `robobot/drive/T0/mag` should carry real values.

### Current status
- ✔ Accelerometer working  
- ✔ Gyroscope working  
- ✔ IMU detected; magnetometer probe implemented and tested  
- ❌ AK8963 unreachable on I2C (0x0C)  

**Next step:** Verify and fix I2C bypass configuration in MPU9250 initialization. (Deferred until after line-following work.)



## 2026-03-12: Line-following, first Python-only patch (sedge.py, mqtt-client.py)

### Context
The active line-following controller is in Python (`sedge.py`, `mqtt-client.py`), not on the Teensy. The Teensy provides sensor acquisition, calibration, normalization, and an onboard line estimate (`lip`); Python subscribes to `livn` and recomputes line position (it does not use `lip`). The original Python logic was a coarse threshold-based edge follower.

### Goals
- Replace coarse threshold-edge estimate with a smoother weighted center-of-gravity estimate.
- Reduce jumpy behavior and improve line-loss handling.
- No firmware flashing; Python-only changes.

## 2026-03-12: Calibration persistence docs and periodic line-sensor prints

### Calibration persistence (notes)
- **line_following.md:** Clarified that white/black calibration is stored in **Teensy EEPROM** (`eew` after `licw 100`). One calibration is enough unless tape/lighting/surface changes; "calibrate first" is for first run or after such a change.
- **commands_and_workflow.md:** Line-follow workflow now states that calibration is persistent; can skip `--white` when already calibrated.

### Periodic line-sensor prints (sedge.py)
- **Purpose:** During testing (without `--silent`), print raw and normalized sensor values and line state periodically.
- **Implementation:** Print every **50** lines (~0.5 s at 100 Hz). In `decode()` when topic is `T0/livn`, if not `--silent` and `edge_nUpdCnt % 50 == 0`, print one line: raw `edge[]`, normalized `edge_n[]`, `average`, `high`, `lineValid`, `lineValidCnt`, `posLeft`, `posRight`, `crossingLine`.
- **Raw values:** Setup sends `sub liv 50` so raw (`T0/liv`) is requested every 50 ms and `edge[]` is updated for the print.
- **Modified files:** `mqtt_python/sedge.py`, `notes/line_following.md`, `notes/commands_and_workflow.md`.

## 2026-03-12: Rolling white calibration (--white)

### Change
White calibration no longer samples in a single position. When you run `--white`:
1. Place the robot on the line; the script drives **forward** at 0.12 m/s for 2.5 s, then **backward** for 2.5 s, then stops.
2. While moving, it collects raw line-sensor values (`liv`); for each of the 8 sensors it keeps the **maximum** (the line is the brightest).
3. It sends those 8 values to the Teensy with **`litw w1 w2 ... w8`**, then **`eew`** to save to EEPROM.

So the white level reflects the full behaviour over the line, not one spot. Implemented in `sedge.py` (setup() and decode T0/liv); uses existing `litw` and `eew` on the Teensy.

## 2026-03-12: teensy_interface stops robot when master is lost

### Problem
If mqtt-client was closed (or crashed) while the robot was moving, the wheels kept turning because the last `rc` command (e.g. `rc 0.2 0.5`) stayed active. The interface detected "master lost" after 4 s without "alive" but did not stop the motors.

### Fix
In `teensy_interface/src/uservice.cpp`, when master is lost (no "alive" for > 4 s), call `mixer.setVelocity(0, 0)` so the robot stops. Previously this was commented out ("ignored for now").

**Note:** After you close mqtt-client there can be up to ~4 s before the interface declares master lost and stops the robot. For immediate stop, exit the client normally (Ctrl+C or let the mission end) so it sends `rc 0 0` in cleanup.

## 2026-03-13: Silent/test mode fix, quiet startup, and line-print cleanup

### Bug fix: robot not moving with `-s` or `-t`
- **Problem:** With `-e -s` or `-e -t` (or `-m -s`, etc.) the robot did nothing; `loop()` was never run.
- **Cause:** In `uservice.py`, `on_connect` and `on_connectOut` set `self.connected` and `self.connectedOut` only inside `if rc == 0 and not self.is_quiet()`. With `-s` or `-t`, `is_quiet()` is true, so the flags were never set and `if service.connected: loop()` in mqtt-client.py was skipped.
- **Fix:** Set `connected` / `connectedOut` whenever `rc == 0`; use `is_quiet()` only to gate the "Connected to MQTT Broker" print.

### Quiet mode cleanup (`-s` / `-t`)
- **`is_quiet()`** in `uservice.py`: returns true when `--silent` or `--test`; used to gate normal chatter (not errors).
- **`-s` (silent):** No startup messages, no state/mission prints, no test prints. Only critical/error messages.
- **`-t` (test):** Same as silent for chatter; **only** test prints are shown (the periodic line + followLine output during line-following).
- **Gated prints:** Startup (BCM GPIO, GPIO setup, Robot data stream OK, IR got data stream, Pose configured), uservice (Started/Ended, Connected, Setup finished, shutting down, Service thread stopped), mqtt-client (Starting, state changes, mission messages, Main Terminated), and module terminate/log messages are all gated with `if not service.is_quiet():` in uservice, mqtt-client, sedge, spose, srobot, sir, sgpio, scam, ulog, simu. Pose message in non-quiet mode gets a blank line before and after.

### Line-follow test print (sedge.py)
- **Only when line-following:** The single test print runs only inside `followLine()` (when `lineCtrl` is true). No print during drive-to-line or when line control is off.
- **One combined message:** livn + line state + followLine output in one block (no separate "line:" and "Edge::followLine" lines).
- **Content:** livn (8 values 0–1000), avg, high, valid, validCnt, posL, posR, cross, then `|` then e, u, y, and rc (velocity and turn only; timestamp omitted from print). Raw values removed from print (calculation: Teensy does `(raw - black) * gain` → 0–1 → ×1000 for livn; see `teensy_firmware_8/src/ulinesensor.cpp`).
- **posL / posR:** Line position index in [-3.5, 3.5] for left/right edge of the detected line (sensors 1..8).
- **Format:** Two lines, break after validCnt; fixed-width columns so output lines up. First line: `% line: livn [ ... ] avg=... high=... valid=... validCnt=...`; second line: `%       posL=... posR=... cross=... | e=... u=... y=... -> rc ...`. Printed every 10th livn update when test or not silent.
- **Subscription:** `sub liv 20` (raw every 20 ms) for fresher data when printing; livn remains at 10 ms.

### Modified files
- `mqtt_python/uservice.py` — is_quiet(), on_connect/on_connectOut fix, gated prints
- `mqtt_python/mqtt-client.py` — gated mission/state/startup prints
- `mqtt_python/sedge.py` — test print only in followLine(), combined two-line fixed-width format
- `mqtt_python/spose.py`, `srobot.py`, `sir.py`, `sgpio.py`, `scam.py`, `ulog.py`, `simu.py` — gated startup/terminate/periodic prints

---

### Planned line-following changes (order documented)
- **Planned changes** documented in `notes/line_following.md` under "Planned changes (line following)".
- **Order:** (1) PID controller (Kp, Ki, Kd; integral limit/anti-windup; no lead/lowpass) → (2) Behavior (recovery A1–A4, speed B1–B3, stop C1–C3, re-entry D1–D2, crossing E1–E2) → (3) Lead and lowpass later if needed.
- **Next step:** Implement PID in `sedge.py`.

### PID line-following controller implemented
- **File:** `mqtt_python/sedge.py`
- **Gains:** `lineKp`, `lineKi`, `lineKd` (defaults 0.5, 0, 0 so behavior unchanged until tuned).
- **Integral:** `lineIntegral` accumulated when line valid; clamped to ±`lineIntegralLimit` (default 2.0 error·s); reset to 0 when `lineValid` is false (avoids windup when line lost).
- **Derivative:** `(e - e_prev) / dt` with `dt` from `edge_nInterval`; `lineE_prev` set to `e` when line lost so D is zero on re-acquire.
- **Output:** `u = Kp*e + Ki*integral + Kd*dTerm`, then `lineY = clamp(u, -4, 4)` rad/s unchanged.
- **Test print:** `e`, `p`, `i`, `d`, `u`, `y` so P/I/D contributions are visible for tuning.

### Line-follow tuning block and follow-line print options (sedge.py)
- **Single tuning block** at top of `SEdge`: all values to change in one place (line detection thresholds, PID gains, integral limit, output saturation `lineYMax`/`lineYMin`, legacy lead, follow-line print options).
- **Saturation:** Turn rate clamp uses `lineYMax` / `lineYMin` (default ±4 rad/s) instead of hardcoded ±4.
- **Follow-line print:** One master switch `print_follow_line_block`; which values are printed is controlled by tuple `print_follow_line_fields`. Copy field names from the "Available fields" comment into the tuple; order in tuple = order on screen.
- **Available print fields:** Line 1: livn, avg, high, valid, validCnt. Line 2: posL, posR, center, cross, e, p, i, d, dTerm, integral, u, y, rc. Includes P/I/D terms and raw `dTerm` (error/s) and `integral` (error·s) for tuning.
- **Intervals:** `follow_line_print_every_n` = how often the two-line block is printed when enabled; `flog_write_every_n` = how often `flog.write()` is called (file log append). Both placed next to `print_follow_line_block` in the block.
- **Comments:** Available-fields list and short descriptions moved below `print_follow_line_fields`; comment lines kept short (newline per term where needed). `flog_write_every_n` comment explains it appends line/sensor log to file.
- **Other prints:** Setup, calibration, PID recalculate, terminate, paint messages no longer have per-category toggles; they are gated only by `is_quiet()` / `--silent` as before.

### Line-follow recovery and last-line memory (sedge + missions)
- **sedge.py:** `lastLineSide` (-1/0/+1) updated when line valid from sign of error; used for recovery turn direction. Recovery: when line lost and line control on, send `rc <recoveryVelocity> <turn>` with turn = `recoveryTurnRate * lastLineSide`. Tunables: `recoveryTurnRate`, `recoveryVelocity` (m/s forward during recovery; 0 = turn in place, small value = creep forward while turning), `recovery_timeout_s` (mission stops after this many seconds without line).
- **Missions (mqtt-client.py, mqtt-linefollow.py):** In state 10 (following), no longer stop immediately when `lineValidCnt < 2`; start lost-line timer and keep line control on so sedge recovery runs. Stop only after `edge.recovery_timeout_s` (default 5 s) without line; if line re-found before that, resume following. `recovery_timeout_s` moved to sedge tuning block and read as `edge.recovery_timeout_s`.
- **Notes:** Crossing logic (Phase 1 slow, Phase 2 pause/choose) and "Other features we can implement" (B speed vs confidence, D1 ramp, C2 distance stop, min/max speed) added to `notes/line_following.md` for later.

---

## 2026-03-15: Sensor-array state and crossing-by-count (sedge.py)

### Per-sensor threshold and array state
- **Per-sensor above-threshold:** Each of the 8 line sensors is compared to `lineValidThreshold`; results are stored in `sensorAboveThreshold[0..7]` (bool per sensor).
- **Derived values:** `sensorsAboveCount` (0..8), `leftmostAboveIndex` / `rightmostAboveIndex` (first/last sensor index above threshold, or None).
- **Named state:** `lineState` is set each update: `"no_line"` (0 sensors above), `"line"` (1–2 above), `"crossing"` (3+ above). Behavior can branch on `edge.lineState` or on the count/pattern for each sensor-array configuration.

### Crossing detection
- **Crossing = count of sensors above threshold:** `crossingLine` is now true when `sensorsAboveCount >= crossingMinSensors` (no longer based on 8-sensor average). `crossingLineCnt` still debounces (0..20) for downstream behavior (e.g. slow on crossing).

### Tunables (top of sedge.py)
- **`lineValidThreshold`** — used for each sensor’s on/off and for line validity (comment clarified).
- **`crossingMinSensors`** — number of sensors above threshold that define a crossing (default 3). Added to the tuning block at the top of `SEdge` with the other line-detection parameters.

### Bug fix (LineDetect)
- **TypeError: 'int' object is not callable:** A local variable `sum = 0` in `LineDetect()` shadowed the built-in `sum()`, so `sum(1 for b in self.sensorAboveThreshold if b)` failed. Renamed the local variable to `total`.

### Modified files
- `mqtt_python/sedge.py` — tuning block (lineValidThreshold comment, crossingMinSensors); per-sensor state and lineState; crossing by count; LineDetect sum→total.
- `notes/line_following.md` — documented sensor state and crossing-by-count under "E. Crossing / junctions".

---

## 2026-03-15: Mission state machine, hard turns, 4th crossing, buttons

### driveTurn and driveDistance (mqtt-client.py, mqtt-linefollow.py)
- **`driveTurn(angle_deg, direction)`** — Turn in place by angle in degrees; `direction` is `"left"` or `"right"`. Completion when `abs(pose.tripBh) >= angle_rad` (fixes over-rotation on right turns). No timeout; stop only when requested angle reached. `driveTurn90` removed; use `driveTurn(90, "left")` etc.
- **`driveDistance(meters, velocity=0.2)`** — Drive forward for given meters (same pattern as driveOneMeter).

### Crossing-count state machine and leave delay
- **Crossing counter:** Mission tracks `crossing_count`; incremented only **after** we have been clear of a crossing for `CROSSING_LEAVE_DELAY_S` (0.5 s). Timer resets while still at crossing (`crossingLineCnt >= CROSSING_AT_CNT`), so we count one crossing per physical crossing.
- **Constants:** `CROSSING_AT_CNT`, `CROSSINGS_GO_STRAIGHT = 2`, `CROSSING_STOP_AT = 4`, `START_AT_CROSSING` (0 = normal; e.g. 4 for testing from 4th crossing).

### Crossing-only behavior (hard turns and 4th stop)
- All crossing logic runs only when **at a crossing** (`at_crossing_now = edge.crossingLineCnt >= CROSSING_AT_CNT`). Crossings 1 and 2: go straight. Crossing 3 (and 5+): hard turn if pattern matches (leftmost=0 and rightmost≤4 → hard left; rightmost=7 and leftmost≥3 → hard right); cooldown between hard turns. **Crossing 4:** always stop (never hard turn); then state 20: `driveTurn(45, "right")`, `driveDistance(1.3)`, then exit.

### Follow-line print (sedge.py)
- Single-line debug block (no split across lines). Added **crossingCnt** (mission crossing count, set from driveToLine state 10), **leftMost** and **rightMost** (above-threshold indices). `edge.mission_crossing_count` updated in state 10 for display.

### GPIO buttons (sgpio.py, uservice.py)
- **Green (GPIO 13):** `test_start_button()` added; 13 = start (optional; mission can wait for green; currently `-e` starts without green).
- **Red (GPIO 6):** Stop unchanged. In `uservice.runAlive()`, when red pressed: set `service.stop = True` and send `rc 0 0` immediately so robot stops in place.

### Modified files
- `mqtt_python/mqtt-linefollow.py` — driveTurn, driveDistance; crossing state machine with leave delay; hard turns only at crossing and not at 1/2/4; state 20 at 4th crossing; mission_crossing_count in print.
- `mqtt_python/mqtt-client.py` — same drive/crossing/state-machine logic as mqtt-linefollow.
- `mqtt_python/sedge.py` — single-line follow print; leftMost, rightMost, crossingCnt (mission_crossing_count) in print.
- `mqtt_python/sgpio.py` — GPIO 13 green/start, test_start_button(); comment 6=stop, 13=start.
- `mqtt_python/uservice.py` — red button: set stop and send rc 0 0 immediately.
- `notes/line_following.md` — new section "Mission and crossing behavior (implemented)".
- `notes/change_log.md` — this entry.

---

## 2026-03-19: GPIO fix, master stop, teensy_interface auto-management

### GPIO per-pin init fix (sgpio.py)
- **Root cause:** GPIO 13 (hardware PWM1, claimed by `ip_disp`) caused `GPIO.setup()` of the full 8-pin list to throw, leaving `gpioFound = False` and all GPIO disabled.
- **Fix:** Each pin is now set up individually in a `try/except`; busy/unavailable pins are skipped and reported. `gpioFound` is set `True` as long as the GPIO module loaded, so pin 6 (red stop) still works even when pin 13 is busy.
- **Note:** GPIO 13 is used by `ip_disp` to monitor the green button and run `mission_start.bash`. `test_start_button()` in Python will always return False while `ip_disp` holds pin 13 — that is expected and fine.

### Stop robot on master loss (uservice.py)
- When `confirmedNotMaster` is set (grace period expired), `clientOut.publish("robobot/cmd/ti", "rc 0 0")` is called directly (bypassing `service.send()` which refuses to send when not master). Robot now stops immediately instead of continuing the last command until the Python process exits.

### teensy_interface auto-management (mqtt-linefollow.py)
- `start_teensy_interface()`: kills any running `teensy_interface` instance (`pkill -x teensy_interfac`, 15-char kernel comm limit), waits 1.5 s for USB release, starts a fresh subprocess, logs its output to `teensy_interface/build/out_console.txt`, waits 3 s for full init, and checks the process is still alive. Guarantees a clean master every run.
- `stop_teensy_interface()`: sends SIGINT to the subprocess, waits up to 3 s, force-kills if needed, always calls `wait()` to reap the child and prevent zombie processes.
- Called automatically in the `__main__` block: `start_teensy_interface()` before `service.setup()`, `stop_teensy_interface()` after `service.terminate()`.
- **`mission_start.bash` compatibility:** `ip_disp` monitors GPIO 13 and runs the bash script on button press; the script starts `mqtt-linefollow.py` in the background. Auto-management works correctly in this path too.

### Master system explanation
- `teensy_interface` tracks the current master by the Python client's start timestamp (sent in `alive` messages). Only one client can be master at a time.
- If you kill and restart `teensy_interface` while keeping the MQTT broker running, the stale master timestamp persists; new Python sessions see a mismatch and quit after 12 s. Auto-management solves this by always restarting `teensy_interface` fresh.
- On robot reboot both broker and `teensy_interface` start clean, so the first Python session always wins.

### Modified files
- `mqtt_python/sgpio.py` — per-pin GPIO setup with individual try/except; busy pins reported and skipped.
- `mqtt_python/uservice.py` — `clientOut.publish("rc 0 0")` on master loss.
- `mqtt_python/mqtt-linefollow.py` — `start_teensy_interface()`, `stop_teensy_interface()`; zombie-safe `wait()` calls.

