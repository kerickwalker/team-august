# Robot Session Notes

Quick handoff file for SSH sessions and sensor-calibration work.

## Copilot Temporary Files Rule
- All temporary scripts, logs, and outputs created by Copilot should be saved in `.copilot_workspace/` at the root of the robobot repo, not in `/tmp`.
- You do not need to ask for permission to use `.copilot_workspace/` for any temporary or test files.

## Quick Command Sheet (copy/paste)

```bash
ps aux | grep teensy  # check teensy_interface process (run anywhere)
pkill teensy_interfac || pkill teensy_interface  # stop interface (process name can be truncated)

cd ~/svn/robobot/teensy_interface/build  # run teensy_interface commands from build dir
./teensy_interface -l -d  # start interface with logging OFF at boot (recommended)
# ./teensy_interface -d  # alternative: start interface with logging ON immediately

mosquitto_pub -h localhost -t "robobot/cmd/ti" -m "log 1"  # start logging manually (run anywhere)
mosquitto_pub -h localhost -t "robobot/cmd/ti" -m "log 0"  # stop logging manually (run anywhere)
mosquitto_pub -h 'localhost' -t robobot/cmd/T0 -m "servo 1 0 50"


cd ~/svn/robobot/mqtt_python  # run mqtt-client commands from python dir
python3 mqtt-client.py -s  # stationary + quiet mode; auto log1 on setup and log0 on exit
```

- Important: `-l` means `--logging-off` at startup.
- In `mqtt-client.py`, setup sends `log 1` and terminate sends `log 0`.
- Teensy logs appear in: `~/svn/robobot/teensy_interface/build/log_YYYYMMDD_HHMMSS.xxx/`
- Other service/system logs appear in: `~/svn/log/`
- New log folder is created when `teensy_interface` starts, not for every `mqtt-client.py` run.
- If `service.logpath` is static (e.g. `log_stationary/`), files are reused there instead of new timestamp folders.
- Only one active MQTT master is allowed; if another client is running, new client may be rejected.

## Date
- 2026-03-10

## Next Session Start
- First verify magnetometer code path end-to-end (firmware -> teensy_interface -> MQTT -> Python/logs).
- Stationary data already exists without magnetometer; use it as the no-mag baseline.
- After mag path is verified, collect new stationary data with magnetometer enabled and compare against baseline.

## Main Objective
- Collect trial data for sensor calibration/validation.
- Use results to support Kalman filter sensor fusion setup.
- Estimate/tune covariance matrices `Q` (process noise) and `R` (measurement noise).

## Key Context
- Primary logs of interest right now are in `robobot/teensy_interface/build`.
- There is also a separate top-level log tree in `/home/local/svn/log`.
- `mqtt-client.py` depends on `teensy_interface` being active (interface process, not just logging).

## Verified Log Locations

### A) Top-level logs (`/home/local/svn/log`)
- Structure: many timestamped folders like `log_YYYYMMDD_HHMMSS.xxx/`.
- Example files seen inside a run folder:
  - `log_gpio.txt`
  - `log_service.txt`
  - `robot.ini`
- Also contains utility/status files at root:
  - `log_mqtt_ip_disp.txt`
  - `off_by_mqtt1.txt`, `off_by_mqtt2.txt`
  - `rebootinfo.txt`
  - `rename_info.txt`
  - `ip_disp.out`, `robotname`, `robot.ini`

### B) Teensy interface logs (`/home/local/svn/robobot/teensy_interface/build`)
- Also timestamped folders like `log_20260310_093226.617/` and `log_stationary/`.
- Typical files:
  - `log_mqtt.txt`, `log_mqtt_in.txt`
  - `log_joy_drive.txt`, `log_mixer.txt`, `log_service.txt`
  - `log_t0_acc_1.txt`, `log_t0_gyro_1.txt`
  - `log_t0_encoder.txt`, `log_t0_encoder_velocity.txt`
  - `log_t0_pose.txt`, `log_t0_dist.txt`, `log_t0_servo.txt`
  - `log_t0_edge_liv.txt`, `log_t0_edge_livn.txt`, `log_t0_hbt.txt`, `log_t0_teensy_io.txt`
  - `robot.ini`

## Test Command Context
- Script path: `robobot/mqtt_python/mqtt-client.py`.
- Option flags verified in `robobot/mqtt_python/uservice.py`:
  - `-m` / `--meter`: drive 1 m and stop.
  - `-p` / `--pi`: turn 180 degrees and stop.
  - Also relevant: `-e` (`--edge`), `-u` (`--usestate`), `-n` (`--now`), `-s` (`--silent`).

## Current Operating Workflow (as used)
- Check process: `ps aux | grep teensy`
- Stop process: `pkill teensy_interface`
- Start process:
  - `cd ~/svn/robobot/teensy_interface/build`
  - `./teensy_interface -l -d`
- Then run tests via `mqtt-client.py` (e.g., `-m`, `-p`).

## Known Behavior / Constraint
- `mqtt-client.py` will not run correctly if `teensy_interface` is down.
- Reason: `teensy_interface` provides core robot interface, not only logging.

## Logging Control Pipeline (verified)
- Clarification: `./teensy_interface -l` means `--logging-off`.
- Effect of `-l`: teensy_interface starts with logging disabled, and waits for MQTT `log 1`.
- Without `-l`: teensy_interface starts logging immediately on launch.

### Option 1: Automatic Logging via `mqtt-client.py` (recommended for trials)
- Start interface with logging initially off:
  - `cd ~/svn/robobot/teensy_interface/build`
  - `./teensy_interface -l -d`
- Run client:
  - `cd ~/svn/robobot/mqtt_python`
  - `python3 mqtt-client.py -s`
- Behavior:
  - `mqtt-client.py` sends `log 1` during setup (start logging).
  - `mqtt-client.py` sends `log 0` during terminate (stop logging).
- Benefit: clean trial boundaries without restarting teensy_interface.

### Option 2: Manual MQTT Log Toggle (recommended for diagnostics)
- Start interface with logging initially off:
  - `./teensy_interface -l -d`
- Start logging when needed:
  - `mosquitto_pub -h localhost -t "robobot/cmd/ti" -m "log 1"`
- Stop logging when needed:
  - `mosquitto_pub -h localhost -t "robobot/cmd/ti" -m "log 0"`
- Benefit: precise control independent of mqtt-client lifecycle.

### Option 3: Always-On Logging from Startup (simple, more disk usage)
- Start interface without `-l`:
  - `./teensy_interface -d`
- Behavior:
  - logging starts immediately and continues until process stop.
- Benefit: simplest setup.
- Tradeoff: larger logs and less explicit trial segmentation.

### Practice To Avoid
- Avoid using `pkill teensy_interface` just to stop/start logging.
- Reason: teensy_interface also handles robot I/O and control bridge, not only logging.

## Roadmap (proposed)
1. Define trial matrix
	- Motion primitives (`-m`, `-p`, stationary), repetitions, speeds, and surfaces.
2. Standardize run procedure
	- Start/verify `teensy_interface`, run command, stop, archive logs with trial ID.
3. Add run metadata template
	- For each trial: command, date, battery/state, environment notes.
4. Build log map
	- Document each `log_t0_*` signal meaning, units, and expected frequency.
5. Compute initial `R`
	- From stationary and repeatability trials per sensor channel.
6. Compute initial `Q`
	- From process-model residuals during controlled motion trials.
7. Validate and iterate
	- Run fusion, inspect innovations/residuals, retune `Q` and `R`.

## Motion Test Pack (prioritized)

### Minimum set (do these first)
1. Stationary noise (`R` baseline)
- 3 runs: 60s each, robot fully still.
- Use: `python3 mqtt-client.py -s` (auto stop at 60s in state 200).

2. Straight repeatability (`-m`)
- 5 runs on one surface, same start pose.
- If no measurement tool yet: record end-point spread visually/photo.

3. Turn repeatability (`-p`)
- 5 runs from same start pose.
- If no angle tool yet: compare final heading against a floor/wall reference line.

### Next set (when measurement tool is available)
4. Absolute distance accuracy
- Commands: 0.5 m, 1.0 m, 1.5 m; 5 reps each.

5. Absolute turn accuracy
- Commands: 90 deg and 180 deg; 5 reps each direction.

### Optional (if time permits)
6. Speed sensitivity
- Repeat key distance/turn tests at low/medium/high speed.

7. Surface sensitivity
- Repeat on at least 2 surfaces.

## Accuracy Gates (practical)
- Stationary IMU: near-zero mean after bias removal; stable variance across runs.
- Straight repeatability: end-point spread small and consistent run-to-run.
- Turn repeatability: heading spread small and consistent run-to-run.
- Absolute distance error target: start with <= 5% mean error.
- Absolute turn error target: start with <= 5 deg at 90 deg and <= 10 deg at 180 deg.
- If repeatability is poor: fix mechanics/traction first (calibration cannot fix random slip).
- If repeatability is good but biased: tune model params (wheel radius, wheelbase, encoder scale).

## Slip Diagnosis Playbook
- Symptom: commanded distance/angle varies a lot between identical runs.
- Check 1: left/right encoder mismatch during straight runs.
- Check 2: higher error at higher speed (classic traction limit).
- Check 3: larger error on smooth floor than rough floor.
- Mitigation order:
  - Reduce speed/acceleration for calibration runs.
  - Improve tire-floor contact and weight balance.
  - Recalibrate wheel radius/wheelbase only after slip is reduced.

## Data Analysis Workflow
- MATLAB is a good choice; continue with it.
- Per run, compute:
  - Mean and variance per IMU channel (`acc`, `gyro`).
  - End-of-run distance and heading error.
  - Repeatability stats: mean error, std error, RMSE.
- For filter tuning:
  - Build `R` from stationary variances.
  - Build initial diagonal `Q` from motion residual variances.
  - Validate on mixed maneuvers and iterate.

## Pending Inputs
- User can provide wiki links for additional documentation.

## Working Preference
- Keep assistant replies short and manageable.
- Use roadmap/task list first.
- After approval, execute one task at a time.

## Future Investigation
- Edge sensor logging behavior: `log_t0_edge_livn.txt` and `log_t0_edge_liv.txt` output changes depending on logging mode/timing used (observed 2026-03-10)

## Code Changes Log

### 2026-03-10: Button-6 GPIO Fix & Quiet Mode Implementation

#### 1. Button-6 Stop Detection Fix (`sgpio.py`)
**Problem**: Button-6 (GPIO pin 6, stop button) was always returning False, making the stop button non-functional.

**Root causes**:
- `test_stop_button()` line 60: hardcoded `return False` instead of returning actual GPIO value
- `get_value()` line 86: hardcoded `return False` instead of returning GPIO read result
- GPIO pin 6 configured as INPUT without pull-down resistor, causing floating voltage to read as HIGH (false positives)

**Solution**:
- Changed `test_stop_button()`: `return v` (actual GPIO.input() result)
- Changed `get_value()`: `return v == 1` (actual boolean from GPIO read)
- Added pull-down resistor configuration: `GPIO.setup(6, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)` at line 34 and line 50

**Files modified**: 
- `/home/local/svn/robobot/mqtt_python/sgpio.py`

#### 2. Quiet Mode Implementation (`uservice.py`)
**Problem**: Console spam from verbose debug output even when using `--silent` flag, making logs difficult to read during sensor data collection.

**Root causes**:
- `send()` method printing every MQTT message sent with full timestamp and parameters
- `on_messageOut()` printing every MQTT message received on output channel
- `decode()` printing "message not used" warnings for unhandled topics

**Solution**: Wrapped all verbose prints in `if not self.args.silent:` conditionals:
- `send()` line ~279: verbose logging now respects --silent flag
- `on_messageOut()` line ~236: MQTT output channel messages now respects --silent flag  
- `decode()` line ~275: "message not used" warnings now respects --silent flag

**Files modified**:
- `/home/local/svn/robobot/mqtt_python/uservice.py`

#### 3. Encoder Topic Handler (`spose.py`)
**Problem**: New encoder data topic `robobot/drive/T0/enc` recently implemented in teensy_interface was generating "message not used" warnings because no Python decoder existed for it.

**Solution**: Added decoder case in `decode()` method at line ~247:
```python
elif topic == "T0/enc":
  # Encoder data: timestamp encoder_left encoder_right
  # Currently just acknowledging receipt, not storing
  pass
```

**Files modified**:
- `/home/local/svn/robobot/mqtt_python/spose.py`

#### 4. Default Stationary Mode (`mqtt-client.py`)
**Problem**: Robot previously defaulted to starting motion immediately, unsafe for battery-less testing (powered by wall socket only).

**Solution**: Modified `loop()` function to default to state 200 (stationary mode) when no motion flags provided:
- State 200 sends `rc 0.0 0.0` (zero velocity) and green LED indicator
- Only enters motion states (101, 102, 103) if explicit flags given (`-m`, `-p`, `-e`, `-u`)

### Testing Status
✅ Button-6 stop detection working correctly (no false positives)  
✅ Quiet mode (`-s` / `--silent`) suppressing verbose output  
✅ Encoder warnings resolved  
✅ Stationary mode safe operation verified  


### 2026-03-10: Master Handover Fix & Stationary Timeout

#### Master Claim Grace Period (`uservice.py`)
**Problem**: mqtt-client would immediately quit with "I am not robot master, quitting!" when teensy_interface was still broadcasting stale master ID, creating race condition during normal operation.

**Solution**: Added 5-second grace period before setting `confirmedNotMaster` flag:
- New variables at lines 73-75: `masterMismatchCnt`, `masterMismatchFirst`, `masterClaimGraceSec=5.0`
- Grace logic at lines 268-283: counts consecutive mismatches and checks elapsed time before confirming non-master status
- Effect: mqtt-client now tolerates brief master ID mismatches during teensy_interface timeout/recovery

**Files modified**: `/home/local/svn/robobot/mqtt_python/uservice.py`

#### Stationary Mode Auto-Timeout (`mqtt-client.py`)
**Purpose**: Enable unattended stationary data collection with automatic shutdown after 60 seconds.

**Implementation**: Added timeout check in state 200 block:
```python
if self.stateTimePassed() > 60.0:
    self.toExit = True
```

**Usage**: `python3 mqtt-client.py -s` now automatically exits after 60s of stationary operation.

**Files modified**: `/home/local/svn/robobot/mqtt_python/mqtt-client.py`

### 2026-03-10: Added Gyro-Yaw Stream (`yawg`)

**Purpose**: Provide a yaw signal from gyro integration so fusion can combine encoder heading and gyro-based yaw.

**What was added**:
- Firmware publishes new subscription item: `yawg yaw yaw_rate t`
  - File: `teensy_firmware_8/src/uimu2.cpp`
  - `yaw` = integrated `gyro[2]` in radians, wrapped to +/-pi
  - `yaw_rate` = `gyro[2]` converted to rad/s
- Interface subscribes and logs `yawg`
  - File: `teensy_interface/src/simu.cpp`
  - New logfile: `log_t0_yawg.txt` with columns:
    - 1: host timestamp (sec)
    - 2: `yaw` (rad)
    - 3: `yaw_rate` (rad/s)
- Python decoder accepts new topic
  - File: `mqtt_python/simu.py`
  - New MQTT topic: `T0/yawg`

**Config keys** (`[imu1teensy0]`):
- `interval_yawg_ms = 12`
- `print_yawg = false`

**Important**:
- `pose[2]` is still encoder heading (unchanged).
- `yawg` is gyro-only yaw estimate (drifts over time).
- Intended use: fuse `pose[2]` and `yawg` in MATLAB/Kalman workflow.

2026-03-11: System Verification After Git Revert
Full System Re-Verification (all modules)
Problem: After a series of git reverts, it was unclear if all previously fixed issues and solutions were still working as intended.

Solution: Systematically re-verified all recent fixes and features:

Button-6 stop detection is functional
Encoder streaming/logging is correct
Quiet mode and stationary mode work as intended
Master claim grace period is effective
Yawg stream is present and efficient
robot.ini is no longer overwritten
Status: All solutions discussed and implemented are confirmed working after the revert. System is stable and ready for new work.

Files verified: All relevant Python, C++, and config files as listed in previous entries.

---

## System Architecture Reference

**Purpose**: Technical documentation for understanding sensor pipeline, state estimation, and logging infrastructure. Reference this section when answering questions about "how is X calculated" or "what data do we get from Y sensor."

### Overall Architecture (Data Flow)

```
Teensy Microcontroller (firmware)
  ├─ Sensors: quadrature encoders, MPU9250 IMU (accel + gyro + mag)
  ├─ Computation: differential drive odometry, complementary tilt filter
  ├─ Control loop: ~1 kHz
  └─ Output: ASCII strings via USB serial
             ↓
Raspberry Pi: teensy_interface (C++)
  ├─ Decodes Teensy USB strings
  ├─ Publishes to MQTT (localhost:1883)
  ├─ Logs to timestamped folders: build/log_YYYYMMDD_HHMMSS.xxx/
  └─ Topics: robobot/drive/T0/{pose, enc, vel, gyro, acc, edge_liv, edge_livn, ...}
             ↓
Python Client: mqtt-client.py
  ├─ Subscribes to MQTT topics
  ├─ Mission control state machine
  ├─ Sends velocity commands: rc <vel_left> <vel_right>
  └─ Orchestrates logging via "log 1"/"log 0" commands
```

### Robot Hardware: "August" (ID 112, Type 8)
- **Drivetrain**: Differential drive, 2 wheels + caster
- **Encoders**: Quadrature, 68 ticks/rev, 19:1 gear reduction
- **Wheels**: 0.075 m radius, wheelbase configuration in robot.ini
- **IMU**: MPU9250 (Invensense 9-axis: 3-axis accel, 3-axis gyro, 3-axis magnetometer AK8963)
- **Line sensor**: 8-element IR array for edge detection
- **Microcontroller**: Teensy (exact model TBD, likely Teensy 3.x or 4.x)

### Firmware: teensy_firmware_8

**Key files**:
- [teensy_firmware_8/src/uencoder.cpp](teensy_firmware_8/src/uencoder.cpp): Encoder pulse counting, odometry, pose integration
- [teensy_firmware_8/src/uimu2.cpp](teensy_firmware_8/src/uimu2.cpp): IMU data acquisition, complementary tilt filter
- [teensy_firmware_8/src/ulinesensor.cpp](teensy_firmware_8/src/ulinesensor.cpp): Line sensor edge detection

**Control loop**: ~1 kHz, runs in main loop

### Interface: teensy_interface (Raspberry Pi)

**Key files**:
- [teensy_interface/src/steensy.cpp](teensy_interface/src/steensy.cpp): USB serial decode, MQTT publish
- [teensy_interface/src/sencoder.cpp](teensy_interface/src/sencoder.cpp): Encoder/pose logging (log_t0_encoder.txt, log_t0_pose.txt)
- [teensy_interface/src/simu.cpp](teensy_interface/src/simu.cpp): IMU logging (log_t0_gyro_1.txt, log_t0_acc_1.txt)
- [teensy_interface/src/sedge.cpp](teensy_interface/src/sedge.cpp): Edge sensor logging (log_t0_edge_liv.txt, log_t0_edge_livn.txt)

**MQTT topics** (published by teensy_interface):
- `robobot/drive/T0/pose`: timestamp x y heading tilt
- `robobot/drive/T0/enc`: timestamp enc_left enc_right vel_left vel_right update_cnt
- `robobot/drive/T0/vel`: timestamp vel_left vel_right
- `robobot/drive/T0/gyro`: timestamp gx gy gz (deg/s)
- `robobot/drive/T0/acc`: timestamp ax ay az (m/s²)
- `robobot/drive/T0/edge_liv`: raw AD values (50ms interval)
- `robobot/drive/T0/edge_livn`: normalized 0-1000 (10ms interval)

**Configuration**: [teensy_interface/build/robot.ini](teensy_interface/build/robot.ini)
- Subscription intervals: pose 5ms, gyro/acc 12ms, encoder 10ms, edge_liv 50ms, edge_livn 10ms

### Python Client: mqtt_python/mqtt-client.py

**Key files**:
- [mqtt_python/mqtt-client.py](mqtt_python/mqtt-client.py): Main state machine
- [mqtt_python/uservice.py](mqtt_python/uservice.py): MQTT management, master arbitration
- [mqtt_python/spose.py](mqtt_python/spose.py): Pose/encoder decode
- [mqtt_python/simu.py](mqtt_python/simu.py): IMU decode
- [mqtt_python/sedge.py](mqtt_python/sedge.py): Edge sensor decode

---

## Sensor Pipeline Details

### Pose Array: `pose[0..3]`
**Source**: Computed in firmware, logged in interface
- **pose[0]** = x position (meters)
- **pose[1]** = y position (meters)  
- **pose[2]** = heading / yaw (radians, ±π)
- **pose[3]** = tilt / pitch (radians)

**Roll**: NOT computed (would require full 6DOF IMU fusion)

---

### Heading (Yaw) = pose[2]

**Source**: ENCODERS ONLY (no gyro, no magnetometer)  
**File**: [teensy_firmware_8/src/uencoder.cpp](teensy_firmware_8/src/uencoder.cpp#L340) `updatePose()` at line 340  
**Algorithm**: Differential drive odometry with half-angle integration

**Exact equations** (lines 371-394):
```cpp
float d1 = -dp1 * anglePerPuls/gear * odoWheelRadius[0];  // left wheel displacement
float d2 = dp2 * anglePerPuls/gear * odoWheelRadius[1];   // right wheel displacement
float dh = (d2 - d1) / odoWheelBase;                      // heading change
float ds = (d1 + d2) / 2.0;                               // straight displacement

pose[2] += dh/2.0;                    // first half of heading change
pose[0] += cosf(pose[2]) * ds;        // x increment
pose[1] += sinf(pose[2]) * ds;        // y increment
pose[2] += dh/2.0;                    // second half of heading change
if (pose[2] > M_PI) pose[2] -= M_PI*2;      // wrap to ±π
else if (pose[2] < -M_PI) pose[2] += M_PI*2;
```

**Parameters**:
- `anglePerPuls`: encoder resolution (radians per tick)
- `gear`: gear ratio (19:1)
- `odoWheelRadius[0/1]`: wheel radii (meters, typically 0.075 m)
- `odoWheelBase`: distance between wheels (meters)

**Characteristics**:
- ✅ Accurate over short distances
- ❌ Unbounded drift over time (no absolute heading reference)
- ❌ Slip-sensitive (wheel slip causes heading error)
- ❌ NOT corrected by gyro or magnetometer

**Future improvement**: Integrate magnetometer for absolute heading correction

---

### Tilt (Pitch) = pose[3]

**Source**: COMPLEMENTARY FILTER (gyro + accelerometer)  
**File**: [teensy_firmware_8/src/uimu2.cpp](teensy_firmware_8/src/uimu2.cpp#L369) `estimateTilt()` at line 369-447  
**Algorithm**: Complementary filter combining gyro integration with accelerometer angle

**Exact equations** (lines 389-447):
```cpp
// Accelerometer angle (instant tilt from gravity)
float accAng = atan2f(-mpu.acc[0], -mpu.acc[2]);

// Complementary filter (continuous-time equivalent)
// est_dot = a*(gyrorate - bias) + b*(accAng - est)
float u = mpu.gyro[1];  // gyro Y-axis (pitch rate, rad/s)
float est = a * encoder.pose[3] + b * u + b * tiltu1;
encoder.pose[3] = est;
tiltu1 = u;

// where a = (2*tau - dt)/(2*tau + dt)
//       b = dt/(2*tau + dt)
//       tau = filter time constant
```

**Parameters**:
- `tau`: complementary filter time constant (balances gyro vs accel weight)
- Gyro bias correction applied before filtering

**Characteristics**:
- ✅ Bounded (gravity provides absolute reference)
- ✅ Smooth (filters out accelerometer noise)
- ✅ Fast response to actual tilt changes
- ❌ Influenced by linear accelerations (during motion, gravity assumption violated)

---

### Roll

**Source**: NOT COMPUTED  
**Reason**: Would require full 9DOF sensor fusion (Madgwick/Mahony filter)  
**Status**: Madgwick filter code exists but is COMMENTED OUT in [teensy_firmware_8/src/uimu2.h](teensy_firmware_8/src/uimu2.h#L34)

---

### Magnetometer (Absolute Heading Reference)

**Hardware**: AK8963 (integrated in MPU9250)  
**Status**: ❌ DISABLED in firmware  
**File**: [teensy_firmware_8/src/uimu2.h](teensy_firmware_8/src/uimu2.h#L136) `useMag = false`  
**Code**: Madgwick filter (9DOF fusion with mag) COMMENTED OUT at line 34

**Current behavior**:
- Magnetometer initialization code exists but never executes
- `useMag` flag hardcoded to false
- No magnetometer data logged
- Heading comes ONLY from encoders (unbounded drift)

**Future enablement strategy** (recommended):
1. **Phase 1**: Offline MATLAB sensor fusion
   - Add mag data logging to teensy_interface
   - Collect stationary + motion trials
   - Validate mag calibration (hard-iron/soft-iron correction)
   - Test Kalman filter with mag heading correction offline
   
2. **Phase 2**: Onboard firmware integration (if offline validation successful)
   - Uncomment Madgwick filter at uimu2.h:34
   - Set `useMag = true` at uimu2.h:136
   - Implement `imuon M` command parameter handling
   - Calibrate `magOffset` and `magRot` arrays (hard-iron + soft-iron)
   - Store calibration in robot.ini

**Tradeoff**: Magnetometers provide absolute heading but are sensitive to local magnetic disturbances (steel structures, motors, batteries). Must validate environment suitability before relying on mag data.

---

### Encoders (Relative, Incremental)

**Type**: Quadrature encoders  
**Resolution**: 68 ticks/rev × 19:1 gear = 1292 ticks/wheel revolution  
**Measurement**: Incremental pulses (dp1, dp2), NOT absolute position  
**Direction**: Determined by phase relationship between A/B channels

**Velocity computation** (in firmware):
- Computed from encoder delta over time interval
- Published via MQTT topic `T0/vel`
- Logged in `log_t0_encoder_velocity.txt`

**Implementation**: Hardware interrupts with direction tracking in [teensy_firmware_8/src/uencoder.cpp](teensy_firmware_8/src/uencoder.cpp)  
**Functions**: `encPinUpdate()` callbacks for encoder A/B pins

**Characteristics**:
- ✅ High resolution (sub-millimeter wheel displacement)
- ✅ No drift in relative displacement
- ❌ Slip-sensitive (assumes perfect wheel-ground contact)
- ❌ No absolute position reference (reset on power cycle)

---

### IMU: MPU9250 (9-Axis)

**Axes**:
- **Accelerometer**: ax, ay, az (m/s²)
- **Gyroscope**: gx, gy, gz (deg/s)
- **Magnetometer**: mx, my, mz (μT) - **DISABLED**

**Logged data** (raw or processed?):
- **Gyro**: Offset-corrected (bias removed in firmware), NOT raw
- **Accelerometer**: Raw sensor output (no filtering in firmware)
- **Magnetometer**: NOT logged (disabled)

**Bias correction**: Gyro bias estimated and removed in firmware before logging  
**Filtering**: None in firmware (filtering happens in complementary tilt filter for pose[3] only)

**MQTT topics**:
- `T0/gyro`: timestamp gx gy gz (deg/s, bias-corrected)
- `T0/acc`: timestamp ax ay az (m/s², raw)

**Log files**:
- `log_t0_gyro_1.txt`: gyro data (bias-corrected, NOT raw)
- `log_t0_acc_1.txt`: accelerometer data (raw)

**Subscription intervals** (robot.ini):
- Gyro: 12 ms
- Accelerometer: 12 ms

---

### Line Sensor (Edge Detection)

**Hardware**: 8-element IR sensor array  
**Modes**:
- **liv**: Raw AD values (0-4095 or similar)
- **livn**: Normalized (0-1000 scale)

**Normalization equation** (in firmware):
```cpp
lineSensorValue = (raw - blackLevel) * lsGain
where lsGain = 1000.0 / (whiteLevel - blackLevel)
```

**Logging differences**:
- `log_t0_edge_liv.txt`: Raw AD, 50ms interval
- `log_t0_edge_livn.txt`: Normalized, 10ms interval

**Why different rates?**: Different subscription intervals in robot.ini  
- `interval_liv_ms = 50`
- `interval_livn_ms = 10`

**Result**: livn logs are denser (more frequent samples), liv logs are sparser but retain raw sensor DN values

---

## Complete Log File Inventory (24 Types)

**Location**: `~/svn/robobot/teensy_interface/build/log_YYYYMMDD_HHMMSS.xxx/`

**Core sensor logs**:
1. `log_t0_pose.txt`: time x y heading tilt
2. `log_t0_encoder.txt`: time enc_left enc_right vel_left vel_right update_cnt
3. `log_t0_encoder_velocity.txt`: wheel velocities
4. `log_t0_gyro_1.txt`: time gx gy gz (deg/s, bias-corrected)
5. `log_t0_acc_1.txt`: time ax ay az (m/s², raw)

**Line sensor logs**:
6. `log_t0_edge_liv.txt`: raw AD values (50ms)
7. `log_t0_edge_livn.txt`: normalized 0-1000 (10ms)

**Control logs**:
8. `log_mixer.txt`: velocity command mixer output
9. `log_joy_drive.txt`: joystick input
10. `log_t0_motor_voltage.txt`: motor drive voltages
11. `log_t0_motor_0_pid.txt`: left motor PID controller
12. `log_t0_motor_1_pid.txt`: right motor PID controller
13. `log_t0_motor_current.txt`: motor current draw

**Distance/servo logs**:
14. `log_t0_dist.txt`: distance sensor readings
15. `log_t0_force.txt`: force/touch sensor
16. `log_t0_servo.txt`: servo positions

**Status logs**:
17. `log_t0_hbt.txt`: heartbeat (teensy alive indicator)
18. `log_t0_teensy_io.txt`: teensy I/O status
19. `log_service.txt`: service status messages
20. `log_gpio.txt`: GPIO pin states

**MQTT logs**:
21. `log_mqtt.txt`: outgoing MQTT messages
22. `log_mqtt_in.txt`: incoming MQTT messages

**Config**:
23. `robot.ini`: robot configuration snapshot

**Source references**: Grep search `service.logPath` in [teensy_interface/src/*.cpp](teensy_interface/src) returns all 24 log file creation points.

---

## Timestamp Handling (CRITICAL for sensor fusion)

**Teensy time**: Teensy sends timestamps in USB strings  
**Host time**: teensy_interface logs `msgTime` (host receive time)  

**Implementation** (in teensy_interface decode functions):
```cpp
// Example from sencoder.cpp:155
if (sscanf(msg, "%ld %d %d", &t, &enc[0], &enc[1]) == 3) {
    // t is Teensy time, but we log msgTime instead:
    fprintf(service.logPath[LOG_enc], "%ld.%03ld %d %d ...\n", 
            msgTime.tv_sec, msgTime.tv_usec/1000, enc[0], enc[1], ...);
}
```

**Result**: All log files use HOST receive time, NOT Teensy timestamp  
**Implication**: Log timestamps are slightly delayed and jittered relative to true sensor sample time  
**Workaround for fusion**: Use logged timestamps as-is (already synchronized to common host clock)  

**Why useful**: Simplifies multi-sensor time alignment in MATLAB (all logs share same time base)  
**Tradeoff**: Cannot analyze true Teensy-side sample timing or jitter

---

## Key Takeaways for Kalman Filter Design

### State Vector (recommended)
`x = [x, y, θ, v, ω]`  
- x, y: position (m)
- θ: heading (rad)
- v: linear velocity (m/s)
- ω: angular velocity (rad/s)

### Measurements (available)
- **Encoders**: wheel velocities → v, ω (high rate, low accuracy)
- **Gyro**: ω directly (bias-corrected, medium rate, medium accuracy)
- **Accelerometer**: linear acceleration → derives v (high noise)
- **Magnetometer**: absolute θ (if enabled, low rate, environment-sensitive)

### Process Model
Differential drive kinematics:
```
x_dot = v * cos(θ)
y_dot = v * sin(θ)
θ_dot = ω
v_dot = (commanded - actual) / tau_v
ω_dot = (commanded - actual) / tau_ω
```

### Covariance Estimation Strategy
1. **R matrix** (measurement noise): Compute from stationary trials
   - `R_gyro = var(log_t0_gyro_1.txt)` when robot still
   - `R_acc = var(log_t0_acc_1.txt)` when robot still
   - `R_enc = var(encoder_velocity)` when robot still

2. **Q matrix** (process noise): Compute from motion trials
   - Run known commands, compute prediction error
   - `Q = var(measured - predicted)` for each state

3. **Validation**: Check innovation statistics
   - Innovation = measurement - predicted measurement
   - Should be zero-mean with covariance = H*P*H' + R
   - If not, retune Q/R

---

## Next Steps

1. **Collect stationary data**: 3 runs × 60s
2. **Compute R matrix**: MATLAB variance analysis
3. **Design state estimator**: Define state vector, process model, measurement model
4. **Implement in MATLAB**: Offline Kalman filter on logged data
5. **Validate performance**: Compare estimated vs. logged pose
6. **Iterate Q/R tuning**: Minimize innovation residuals
7. **(Future) Enable magnetometer**: If heading drift is unacceptable

# ======================
# === Copilot Instructions (for AI agent, keep at bottom) ===
# ======================

- All temporary scripts, logs, and outputs created by Copilot should be saved in `.copilot_workspace/` at the root of the robobot repo, not in `/tmp`.
- You do not need to ask for permission to use `.copilot_workspace/` for any temporary or test files.
- Use roadmap/task list first.
- After approval, execute one task at a time.
- Keep assistant replies short and manageable.
- Never overwrite the entire file when making edits—only add or change the specific lines needed for the update. Always preserve all existing content unless explicitly told otherwise.
