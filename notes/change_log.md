# Change Log

## 2026-03-12: Line-following smoothing (less jagged)

### Changes in sedge.py
- **maxTurnRate = 2.5** rad/s (was hardcoded ±4). Limits how sharply the robot can turn.
- **lineKp = 0.7** (was 1.0). Same error produces a smaller turn rate.
- **Low-pass filtered position:** Added `posLeftFilt`, `posRightFilt` with `linePosAlpha = 0.35`. Error in `followLine()` is now based on the filtered position so sensor-to-sensor jumps don’t translate directly to steering. Filter is updated only when `lineValid`.
- **followLine() debug print** only every 20th update to reduce console spam.

Tuning: Increase `linePosAlpha` (e.g. 0.5) for quicker response, decrease (e.g. 0.25) for smoother. Increase `lineKp` or `maxTurnRate` if following feels sluggish.

## 2026-03-12: teensy_interface stops robot when master is lost

### Problem
If mqtt-client was closed (or crashed) while the robot was moving, the wheels kept turning because the last `rc` command (e.g. `rc 0.2 0.5`) stayed active. The interface detected "master lost" after 4 s without "alive" but did not stop the motors.

### Fix
In `teensy_interface/src/uservice.cpp`, when master is lost (no "alive" for > 4 s), call `mixer.setVelocity(0, 0)` so the robot stops. Previously this was commented out ("ignored for now").

**Note:** After you close mqtt-client there can be up to ~4 s before the interface declares master lost and stops the robot. For immediate stop, exit the client normally (Ctrl+C or let the mission end) so it sends `rc 0 0` in cleanup.

## 2026-03-12: Rolling white calibration (--white)

### Change
White calibration no longer samples in a single position. When you run `--white`:
1. Place the robot on the line; the script drives **forward** at 0.12 m/s for 2.5 s, then **backward** for 2.5 s, then stops.
2. While moving, it collects raw line-sensor values (`liv`); for each of the 8 sensors it keeps the **maximum** (the line is the brightest).
3. It sends those 8 values to the Teensy with **`litw w1 w2 ... w8`**, then **`eew`** to save to EEPROM.

So the white level reflects the full behaviour over the line, not one spot. Implemented in `sedge.py` (setup() and decode T0/liv); uses existing `litw` and `eew` on the Teensy.

## 2026-03-12: Calibration persistence docs and periodic line-sensor prints

### Calibration persistence (notes)
- **line_following.md:** Clarified that white/black calibration is stored in **Teensy EEPROM** (`eew` after `licw 100`). One calibration is enough unless tape/lighting/surface changes; "calibrate first" is for first run or after such a change.
- **commands_and_workflow.md:** Line-follow workflow now states that calibration is persistent; can skip `--white` when already calibrated.

### Periodic line-sensor prints (sedge.py)
- **Purpose:** During testing (without `--silent`), print raw and normalized sensor values and line state periodically.
- **Implementation:** Print every **50** lines (~0.5 s at 100 Hz). In `decode()` when topic is `T0/livn`, if not `--silent` and `edge_nUpdCnt % 50 == 0`, print one line: raw `edge[]`, normalized `edge_n[]`, `average`, `high`, `lineValid`, `lineValidCnt`, `posLeft`, `posRight`, `crossingLine`.
- **Raw values:** Setup sends `sub liv 50` so raw (`T0/liv`) is requested every 50 ms and `edge[]` is updated for the print.
- **Modified files:** `mqtt_python/sedge.py`, `notes/line_following.md`, `notes/commands_and_workflow.md`.

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
- The newer encoder MQTT topic `robobot/drive/T0/enc` produced “message not used” warnings because Python had no decoder case for it.

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
- `mqtt-client` could quit immediately with “I am not robot master, quitting!” when `teensy_interface` was still publishing a stale master ID.

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

## Line-following: first Python-only patch (sedge.py, mqtt-client.py)

### Context
The active line-following controller is in Python (`sedge.py`, `mqtt-client.py`), not on the Teensy. The Teensy provides sensor acquisition, calibration, normalization, and an onboard line estimate (`lip`); Python subscribes to `livn` and recomputes line position (it does not use `lip`). The original Python logic was a coarse threshold-based edge follower.

### Goals
- Replace coarse threshold-edge estimate with a smoother weighted center-of-gravity estimate.
- Reduce jumpy behavior and improve line-loss handling.
- No firmware flashing; Python-only changes.

### Changes in sedge.py
- **Smoother line position:** New state `lineCenter`, `lineCenterRaw`, `lineWidth`, `followMode`. Weighted center-of-gravity from sensor intensities above background; low-pass filtered (`linePosAlpha`). `posLeft`/`posRight` kept from active sensor span.
- **followMode:** Controller can follow `"center"`, `"left"`, or `"right"`.
- **Dropout recovery:** `lostLineCnt`, `maxLostLineCnt`, `recoveryTurnGain` — softened steering and recovery turn on brief loss; stop only after prolonged loss.
- **Steering saturation:** `maxTurnRate` reduced from 4 to 2.5 rad/s.
- **paint():** Updated to show `posLeft`, `posRight`, `lineCenter` for debugging.

### Changes in mqtt-client.py
- `driveToLine()` now calls `edge.lineControl(0.2, True, 0.0, "center")` — follow line center instead of left edge.

### Testing
- No Teensy reflash required. Recommended order: calibrate (`--white`), motion check (`--meter`), then line follow (`--edge`). See `line_following.md`.

### Caveat
Debug prints in `followLine()` and `paint()` may still be verbose; consider reducing console spam before live testing.
