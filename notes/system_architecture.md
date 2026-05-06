# System Architecture

## Purpose
This file describes the full data path from sensors on the robot to logs and Python control logic.

## End-to-end data flow

```text
Teensy Microcontroller (firmware)
  ├─ Sensors: quadrature encoders, MPU9250 IMU (accelerometer + gyro + magnetometer)
  ├─ Computation: differential-drive odometry, complementary tilt filter
  ├─ Control loop: about 1 kHz
  └─ Output: ASCII strings over USB serial

Raspberry Pi: teensy_interface (C++)
  ├─ Decodes Teensy USB strings
  ├─ Publishes MQTT messages on localhost:1883
  ├─ Writes logs to build/log_YYYYMMDD_HHMMSS.xxx/
  └─ Publishes robot topics such as pose, enc, vel, gyro, acc, edge_liv, edge_livn

Python Client: mqtt-client.py
  ├─ Subscribes to MQTT topics
  ├─ Runs the mission-control state machine (including line-following when using --edge)
  ├─ Sends commands such as rc <vel_left> <vel_right>
  └─ Coordinates logging through "log 1" and "log 0"
```

## Robot hardware summary: "August"
- robot ID: 112
- robot type: 8
- drivetrain: differential drive with two wheels and a caster
- encoders: quadrature
- encoder gearing noted as 68 ticks per revolution with 19:1 gear reduction
- wheel radius: 0.075 m
- wheelbase: configured in `robot.ini`
- IMU: MPU9250, including accelerometer, gyroscope, and AK8963 magnetometer
- line sensor: 8-element IR array
- microcontroller: Teensy, noted as likely 3.x or 4.x but still marked TBD in the source notes

## Firmware layer: `teensy_firmware_8`

### Main responsibilities
- count encoder pulses
- integrate odometry
- read IMU
- estimate tilt
- produce outgoing serial messages

### Key files
- `teensy_firmware_8/src/uencoder.cpp`
- `teensy_firmware_8/src/uimu2.cpp`
- `teensy_firmware_8/src/ulinesensor.cpp`

### Line sensor (firmware)
- Line acquisition, calibration storage, normalization, and onboard line position/validity (`lip`) are in `ulinesensor.cpp` / `ulinesensor.h`.
- Teensy publishes `liv` (raw), `liw`/`lib` (calibration), `livn` (normalized 0–1000), `lip` (onboard estimate). The **active** line-following controller is in **Python** (`sedge.py`), which subscribes to `livn` and recomputes line position (it does not use Teensy `lip`). See `line_following.md`.

### Magnetometer (mag) and I2C bypass
- Magnetometer handling is in `uimu2.cpp`; the mag topic is published but currently returns zeros because the AK8963 is not reachable on I2C (address 0x0C). The MPU9250 must have I2C bypass enabled (INT_PIN_CFG register 0x37, BYPASS_EN bit) for the host to talk to the AK8963. See `sensor_reference.md` and `change_log.md` for the diagnostic summary and proposed bypass fix.

### Runtime characteristic
- main control loop around 1 kHz

## Interface layer: `teensy_interface`

### Main responsibilities
- decode USB serial lines coming from the Teensy
- republish the data as MQTT topics
- log the received values into timestamped folders

### Key files
- `teensy_interface/src/steensy.cpp`
- `teensy_interface/src/sencoder.cpp`
- `teensy_interface/src/simu.cpp`
- `teensy_interface/src/sedge.cpp`

### Important MQTT topics published by the interface
- `robobot/drive/T0/pose`: timestamp, x, y, heading, tilt
- `robobot/drive/T0/enc`: timestamp, enc_left, enc_right, vel_left, vel_right, update_cnt
- `robobot/drive/T0/vel`: timestamp, vel_left, vel_right
- `robobot/drive/T0/gyro`: timestamp, gx, gy, gz in deg/s
- `robobot/drive/T0/acc`: timestamp, ax, ay, az in m/s²
- `robobot/drive/T0/mag`: magnetometer mx, my, mz (currently zeros until I2C bypass fix; see `sensor_reference.md`)
- `robobot/drive/T0/edge_liv`: raw line-sensor values (Teensy `liv`)
- `robobot/drive/T0/edge_livn`: normalized line-sensor values (Teensy `livn`); used by Python line follower
- Line-related Teensy messages also include `liw`, `lib`, `lip` (onboard line position); Python controller uses `livn` only. See `line_following.md`.

### Configuration location
- `teensy_interface/build/robot.ini`

### Subscription intervals noted in the source notes
- pose: 5 ms
- gyro: 12 ms
- accelerometer: 12 ms
- encoder: 10 ms
- edge_liv: 50 ms
- edge_livn: 10 ms

## Python layer: `mqtt_python`

### Main responsibilities
- subscribe to robot data topics
- implement mission logic and test modes
- send motion commands
- manage or influence logging behavior
- arbitrate master ownership through the service layer

### Key files
- `mqtt_python/mqtt-client.py` — mission state machine; line-follow entry point (`--edge`, `driveToLine()`)
- `mqtt_python/uservice.py` — MQTT send/receive, decoding
- `mqtt_python/spose.py` — pose/encoder handling
- `mqtt_python/simu.py` — IMU (gyro, acc, yawg) decoding
- `mqtt_python/sedge.py` — **active line-following controller** (subscribes to `livn`, computes steering, sends `rc`)

## Architecture implications

### Why `teensy_interface` matters so much
The interface is not just a logging process. It is the bridge between:
- USB serial from the Teensy
- MQTT topics on the Pi
- downstream Python control logic

That is why killing it breaks more than logging.

### Why the logging setup matters
Because the logs are written from the interface layer, trial segmentation and timestamp semantics are controlled there, not only in the Python client.

### Why this architecture is good for offline estimation
The structure already gives you:
- centralized host-side timestamps
- separate logs per signal type
- clean topic boundaries
- a natural place to add future sensor streams such as magnetometer-derived outputs
