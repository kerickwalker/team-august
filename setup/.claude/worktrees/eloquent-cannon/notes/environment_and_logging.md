# Environment and Logging

## Core paths

### Main working areas
- `~/svn/robobot/teensy_interface/build`
- `~/svn/robobot/mqtt_python`
- `/home/local/svn/log`

### What each path is used for
- `teensy_interface/build` is the main place for robot-interface runs and timestamped sensor logs.
- `mqtt_python` is where `mqtt-client.py` is launched from.
- `/home/local/svn/log` is a separate top-level system log tree with service and utility logs.

## Verified log locations

### Top-level logs: `/home/local/svn/log`
This tree contains many timestamped folders of the form:

```text
log_YYYYMMDD_HHMMSS.xxx/
```

Typical contents inside a run folder:
- `log_gpio.txt`
- `log_service.txt`
- `robot.ini`

Also seen at the root of the tree:
- `log_mqtt_ip_disp.txt`
- `off_by_mqtt1.txt`
- `off_by_mqtt2.txt`
- `rebootinfo.txt`
- `rename_info.txt`
- `ip_disp.out`
- `robotname`
- `robot.ini`

### Teensy interface logs: `/home/local/svn/robobot/teensy_interface/build`
This area also contains timestamped folders such as:
- `log_20260310_093226.617/`
- `log_stationary/`

Typical files in a run folder:
- `log_mqtt.txt`
- `log_mqtt_in.txt`
- `log_joy_drive.txt`
- `log_mixer.txt`
- `log_service.txt`
- `log_t0_acc_1.txt`
- `log_t0_gyro_1.txt`
- `log_t0_encoder.txt`
- `log_t0_encoder_velocity.txt`
- `log_t0_pose.txt`
- `log_t0_dist.txt`
- `log_t0_servo.txt`
- `log_t0_edge_liv.txt`
- `log_t0_edge_livn.txt`
- `log_t0_hbt.txt`
- `log_t0_teensy_io.txt`
- `robot.ini`

## Logging behavior

### What `-l` means
When launching `teensy_interface`, the flag:

```bash
-l
```

means:

```text
--logging-off
```

So `./teensy_interface -l -d` starts the interface with logging disabled at startup.

### What happens without `-l`
If you start:

```bash
./teensy_interface -d
```

then logging starts immediately and continues until the process stops.

### Important logging rule
A new timestamped log folder is created when `teensy_interface` starts, not every time `mqtt-client.py` is run.

### Static log-path case
If `service.logpath` is configured to a fixed directory such as `log_stationary/`, files are reused there instead of creating a fresh timestamped run directory.

## Logging control modes

### Automatic trial logging via `mqtt-client.py`
Recommended for ordinary test runs.

Behavior:
- Start `teensy_interface` with logging off.
- Start `mqtt-client.py`.
- During setup, `mqtt-client.py` sends `log 1`.
- During terminate, `mqtt-client.py` sends `log 0`.

Benefit:
- Clean trial boundaries without restarting the interface.

### Manual MQTT logging control
Recommended for diagnostics or when you want explicit control over the logging window.

Commands:
```bash
mosquitto_pub -h localhost -t "robobot/cmd/ti" -m "log 1"
mosquitto_pub -h localhost -t "robobot/cmd/ti" -m "log 0"
```

Benefit:
- Precise start and stop control that is independent of client lifecycle.

### Always-on logging
Simplest setup, but less clean for later analysis.

Behavior:
- Launch `teensy_interface` without `-l`.
- Logging starts immediately.
- Logs keep accumulating until the process exits.

Tradeoff:
- Larger files
- Poorer trial segmentation

## Full log file inventory

### Core sensor logs
1. `log_t0_pose.txt`: time, x, y, heading, tilt
2. `log_t0_encoder.txt`: time, enc_left, enc_right, vel_left, vel_right, update_cnt
3. `log_t0_encoder_velocity.txt`: wheel velocities
4. `log_t0_gyro_1.txt`: time, gx, gy, gz in deg/s, bias-corrected
5. `log_t0_acc_1.txt`: time, ax, ay, az in m/s², raw

### Line sensor logs
6. `log_t0_edge_liv.txt`: raw AD values, 50 ms interval
7. `log_t0_edge_livn.txt`: normalized 0–1000 values, 10 ms interval

### Control logs
8. `log_mixer.txt`: velocity-command mixer output
9. `log_joy_drive.txt`: joystick input
10. `log_t0_motor_voltage.txt`: motor drive voltages
11. `log_t0_motor_0_pid.txt`: left motor PID controller
12. `log_t0_motor_1_pid.txt`: right motor PID controller
13. `log_t0_motor_current.txt`: motor current draw

### Distance and servo logs
14. `log_t0_dist.txt`: distance sensor readings
15. `log_t0_force.txt`: force or touch sensor data
16. `log_t0_servo.txt`: servo positions

### Status logs
17. `log_t0_hbt.txt`: heartbeat, used as a teensy-alive indicator
18. `log_t0_teensy_io.txt`: teensy I/O status
19. `log_service.txt`: service status messages
20. `log_gpio.txt`: GPIO states

### MQTT logs
21. `log_mqtt.txt`: outgoing MQTT messages
22. `log_mqtt_in.txt`: incoming MQTT messages

### Configuration snapshot
23. `robot.ini`

The notes mention 24 total log creation points, with the source identified by grepping `service.logPath` in `teensy_interface/src/*.cpp`.

## Timestamp handling for sensor fusion

### What the Teensy sends
The Teensy includes its own timestamps in the USB serial messages.

### What gets written to logs
`teensy_interface` writes host receive time, stored as `msgTime`, rather than the original Teensy timestamp.

Example pattern from the notes:
```cpp
if (sscanf(msg, "%ld %d %d", &t, &enc[0], &enc[1]) == 3) {
    fprintf(service.logPath[LOG_enc], "%ld.%03ld %d %d ...\n",
            msgTime.tv_sec, msgTime.tv_usec/1000, enc[0], enc[1], ...);
}
```

### Consequences
- All logged streams share a common host-side time base.
- This makes multi-sensor alignment in MATLAB easier.
- The cost is that you do not retain the true sensor-sample timestamp from the Teensy.
- Logged times therefore include host-side receive delay and jitter.

### Practical guidance
For fusion work, use the logged timestamps as the common time base unless you specifically decide to redesign the logging path.

## Logging-related investigation still open
- `log_t0_edge_liv.txt` and `log_t0_edge_livn.txt` appeared to change behavior depending on logging mode and timing.
- That remains a follow-up item from 2026-03-10.
