# Sensor Reference

## Purpose
This file focuses on how the main estimated and measured signals are produced, what they mean, and what limitations matter for calibration or sensor fusion.

## Pose array: `pose[0..3]`

### Definition
- `pose[0]`: x position in meters
- `pose[1]`: y position in meters
- `pose[2]`: heading or yaw in radians, wrapped to `±π`
- `pose[3]`: tilt or pitch in radians

### Important note
Roll is not currently computed.

## Heading: `pose[2]`

### Source
Encoder-only differential-drive odometry.

### File
- `teensy_firmware_8/src/uencoder.cpp`
- function mentioned in the notes: `updatePose()`

### Odometry equations from the notes
```cpp
float d1 = -dp1 * anglePerPuls/gear * odoWheelRadius[0];
float d2 = dp2 * anglePerPuls/gear * odoWheelRadius[1];
float dh = (d2 - d1) / odoWheelBase;
float ds = (d1 + d2) / 2.0;

pose[2] += dh/2.0;
pose[0] += cosf(pose[2]) * ds;
pose[1] += sinf(pose[2]) * ds;
pose[2] += dh/2.0;

if (pose[2] > M_PI) pose[2] -= M_PI*2;
else if (pose[2] < -M_PI) pose[2] += M_PI*2;
```

### Parameters involved
- `anglePerPuls`
- `gear`
- `odoWheelRadius[0/1]`
- `odoWheelBase`

### Interpretation
Strengths:
- good short-horizon relative heading estimate
- directly tied to wheel motion

Limitations:
- drifts over time
- sensitive to wheel slip
- has no absolute heading correction in the current setup
- is not currently corrected by gyro or magnetometer

## Tilt: `pose[3]`

### Source
Complementary filter using gyro and accelerometer.

### File
- `teensy_firmware_8/src/uimu2.cpp`
- function mentioned in the notes: `estimateTilt()`

### Equations quoted in the notes
```cpp
float accAng = atan2f(-mpu.acc[0], -mpu.acc[2]);

float u = mpu.gyro[1];
float est = a * encoder.pose[3] + b * u + b * tiltu1;
encoder.pose[3] = est;
tiltu1 = u;
```

With:
```text
a = (2*tau - dt) / (2*tau + dt)
b = dt / (2*tau + dt)
```

### Interpretation
Strengths:
- bounded because gravity provides a reference
- smoother than raw accelerometer angle
- responds faster than a pure low-pass estimate

Limitations:
- can be disturbed by linear acceleration during motion

## Roll

### Status
Not computed.

### Reason
The notes state that this would require fuller IMU fusion such as Madgwick or Mahony.

### Additional note
Madgwick-related code exists but is commented out in `uimu2.h`.

## Magnetometer

### Hardware
AK8963 inside the MPU9250 package. The board (labeled MPU-9250) exposes XDA/XCL for the internal AK8963 I2C interface, so the hardware should contain a magnetometer.

### MQTT topic
- `robobot/drive/T0/mag`

### Current status (as of 2026-03-12)
- Magnetometer path is **enabled in firmware** (`useMag` can be true) and the topic is published, but readings are **all zeros** (e.g. `mag 0.000000 0.000000 0.000000`).
- Firmware path verified: `teensy_firmware_8/src/uimu2.cpp`; init runs; runtime code uses `mpu.magUpdate()` and `mag[0..2]`.
- Raw serial probe to AK8963 at I2C address `0x0C` gives: `endTx=2` (NACK on address), `n=0`, no valid WHO_AM_I.
- **Conclusion:** AK8963 is **not reachable on the I2C bus**; failure is below `magUpdate()`, at the I2C communication layer.

### Suspected cause
The AK8963 is only accessible when the MPU9250 register **INT_PIN_CFG (0x37)** has **BYPASS_EN = 1**. The library call `mpu.magEnableSlaveMode()` may not be setting this correctly; bypass is likely not enabled.

### Proposed fix (to test)
In `initMpu()` in `uimu2.cpp`, before `mpu.beginMag(...)`, set the bypass bit directly:

```cpp
if (useMag)
{
  Wire.beginTransmission(0x68);
  Wire.write(0x37);   // INT_PIN_CFG
  Wire.write(0x02);   // BYPASS_EN
  Wire.endTransmission();
  delay(10);
  mpu.beginMag(MAG_MODE_CONTINUOUS_100HZ);
}
```

Expected after fix: probe shows `endTx=0 n=1 who=0x48` (0x48 = AK8963 WHO_AM_I); `magUpdate()` returns valid data; MQTT topic carries real values.

### Next step
Verify I2C bypass configuration in MPU9250 initialization (see also `change_log.md`).

### Recommended enablement strategy (after bypass fix)
Phase 1:
- confirm mag topic and logs with non-zero values
- collect stationary and motion data
- validate hard-iron and soft-iron calibration
- test fusion offline in MATLAB

Phase 2:
- enable onboard fusion only if offline results are convincing
- calibrate `magOffset` and `magRot`
- store calibration in `robot.ini`

### Important caution
Magnetometers can provide an absolute heading reference, but they are sensitive to environmental disturbances from steel structures, motors, and other local effects.

## Encoders

### Type
Quadrature encoders.

### Resolution information from the notes
- 68 ticks per revolution
- 19:1 gear ratio
- therefore 1292 ticks per wheel revolution

### What they measure
- incremental pulse changes
- not absolute wheel position across power cycles

### Velocity path
The firmware computes velocity from encoder deltas over time.
This is published on `T0/vel` and logged in `log_t0_encoder_velocity.txt`.

### Interpretation
Strengths:
- high resolution
- strong for short-term relative motion

Limitations:
- slip-sensitive
- no absolute position reference after reset

## IMU: MPU9250

### Channels
- accelerometer: `ax`, `ay`, `az`
- gyroscope: `gx`, `gy`, `gz`
- magnetometer: `mx`, `my`, `mz`, but currently disabled in practice

### What is logged now
Gyro:
- bias-corrected before logging
- units: deg/s

Accelerometer:
- raw sensor output
- units: m/s²

### MQTT topics
- `T0/gyro`
- `T0/acc`

### Log files
- `log_t0_gyro_1.txt`
- `log_t0_acc_1.txt`

### Subscription timing in the notes
- gyro: 12 ms
- accelerometer: 12 ms

## Gyro-yaw stream: `yawg`

### Purpose
Provide a gyro-integrated yaw estimate that can later be fused with encoder heading.

### Source path
- firmware: `teensy_firmware_8/src/uimu2.cpp`
- interface logging: `teensy_interface/src/simu.cpp`
- Python decoder: `mqtt_python/simu.py`

### Logged file
- `log_t0_yawg.txt`

### Columns
1. host timestamp
2. `yaw` in radians
3. `yaw_rate` in rad/s

### Interpretation
- `pose[2]` and `yawg` are not the same signal
- `pose[2]` is encoder heading
- `yawg` is gyro-only yaw and will drift
- the two are useful together for fusion work

## Line sensor

### Hardware
8-element IR array.

### Modes and Teensy messages
- `liv`: raw AD values
- `livn`: normalized values on a 0–1000 scale (0 = black, 1000 = calibrated white)
- `liw` / `lib`: white/black calibration values
- `lip`: Teensy onboard line position and validity (computed in `ulinesensor.cpp`)

### Who uses what
- The **active** line-following controller is in **Python** (`sedge.py`). It subscribes to **`livn`** and **recomputes** line position; it does **not** use the Teensy `lip` estimate.
- The original Python logic used a coarse threshold-based left/right edge from `livn`. A patched version uses a **weighted center-of-gravity** line estimate and can follow center, left, or right. See `line_following.md`.

### Normalization equation from the notes
```cpp
lineSensorValue = (raw - blackLevel) * lsGain
```

with:
```text
lsGain = 1000.0 / (whiteLevel - blackLevel)
```

### Logging difference
- `log_t0_edge_liv.txt`: raw values, 50 ms interval
- `log_t0_edge_livn.txt`: normalized values, 10 ms interval

### Interpretation
- `livn` is denser and easier to compare directly across sensors
- `liv` preserves the raw sensor-domain values
- the rate difference comes from separate subscription intervals, not necessarily from a different sensing process
- calibration (`--white` before `--edge`) is required for reliable line detection

## Most important fusion implications
- heading currently has no absolute correction because magnetometer support is still disabled
- encoder heading and gyro-yaw should be treated as distinct signals with different drift and noise behavior
- host-side timestamps simplify multi-sensor alignment
- stationary data is the right place to estimate baseline measurement noise for `R`
