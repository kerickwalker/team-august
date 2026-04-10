# Commands and Workflow

## Quick command sheet

```bash
ps aux | grep teensy  # check teensy_interface process (run anywhere)
pkill teensy_interfac || pkill teensy_interface  # stop interface (process name can be truncated)

cd ~/svn/robobot/teensy_interface/build  # run teensy_interface commands from build dir
./teensy_interface -l -d  # start interface with logging OFF at boot (recommended)
# ./teensy_interface -d  # alternative: start interface with logging ON immediately

mosquitto_pub -h localhost -t "robobot/cmd/ti" -m "log 1"  # start logging manually (run anywhere)
mosquitto_pub -h localhost -t "robobot/cmd/ti" -m "log 0"  # stop logging manually (run anywhere)
mosquitto_pub -h 'localhost' -t robobot/cmd/T0 -m "servo 1 0 100"

cd ~/svn/robobot/mqtt_python  # run mqtt-client commands from python dir
python3 mqtt-client.py -s  # stationary + quiet mode; auto log1 on setup and log0 on exit
```

## Script and flag reference

### Main script
- `robobot/mqtt_python/mqtt-client.py`

### Verified option flags from `uservice.py`
- `-m` / `--meter`: drive 1 meter and stop
- `-p` / `--pi`: turn 180 degrees and stop
- `-e` / `--edge`: line-follow mode (drive to line, then follow; see `line_following.md`)
- `-u` / `--usestate`
- `-n` / `--now`
- `-s` / `--silent`: print less to console
- `-t` / `--test`: quiet like `--silent` but show only test prints (e.g. periodic line sensor, followLine)
- `--white`: run white calibration for line sensor (do this before `--edge`)

## Standard operating workflow

### Basic sequence
1. Check whether `teensy_interface` is already running:
   ```bash
   ps aux | grep teensy
   ```
2. If needed, stop the existing interface:
   ```bash
   pkill teensy_interfac || pkill teensy_interface
   ```
3. Start the interface from the build directory:
   ```bash
   cd ~/svn/robobot/teensy_interface/build
   ./teensy_interface -l -d
   ```
4. Run the Python client from its own directory:
   ```bash
   cd ~/svn/robobot/mqtt_python
   python3 mqtt-client.py -s
   ```

### For motion tests
After the interface is up, run `mqtt-client.py` with the desired motion flag, such as:
- `-m`
- `-p`

### Line-following workflow (recommended order)
Calibration is stored in Teensy EEPROM (one-time unless tape/lighting changes). Then sanity-check motion, then line follow. From `mqtt_python`:

```bash
python3 mqtt-client.py --white    # line sensor white calibration (skip if already done)
python3 mqtt-client.py --meter    # sanity check: drive 1 m and stop
python3 mqtt-client.py --edge     # line-follow mode (no -s so you see sensor prints)
```

See `line_following.md` for details and what to observe during `--edge`. Do **not** use `-s`/`--silent` when testing line following if you want the periodic sensor/line prints (raw, livn, validCnt, posL, posR, etc.).

## What matters operationally

### Interface dependency
`mqtt-client.py` will not run correctly if `teensy_interface` is down.

Reason:
- `teensy_interface` is the robot interface and control bridge.
- It is not just a logger.

### If you close mqtt-client while the robot is moving
The interface waits for "alive" messages. If the client exits (close terminal, kill, crash), the interface declares "master lost" after **4 seconds** without "alive" and then **stops the robot** (velocity 0). So the wheels may keep turning for up to ~4 s after the client is gone. For immediate stop, exit the client normally (e.g. Ctrl+C) so it sends `rc 0 0` in cleanup.

### Master ownership
Only one active MQTT master is allowed.
If another client already owns master control, a new client may be rejected.

## Recommended usage patterns

### Clean stationary trial
Use:
```bash
python3 mqtt-client.py -s
```

Because in the current setup:
- stationary mode is the default when no motion flag is given
- quiet mode suppresses verbose console output
- the client starts logging during setup
- the client stops logging during terminate
- stationary mode now auto-exits after 60 seconds

### Manual servo command
Use:
```bash
mosquitto_pub -h 'localhost' -t robobot/cmd/T0 -m "servo 1 -800 100"
```

### Manual log toggle
Use:
```bash
mosquitto_pub -h localhost -t "robobot/cmd/ti" -m "log 1"
mosquitto_pub -h localhost -t "robobot/cmd/ti" -m "log 0"
```

## Practice to avoid

Do not use `pkill teensy_interface` as your normal way of toggling logging on and off.

Why:
- It tears down the main robot interface as well.
- It is a heavy-handed substitute for a logging control problem.
- MQTT log toggling is the cleaner mechanism.

## Practical notes
- `-l` on `teensy_interface` means logging is off at startup.
- `mqtt-client.py` sends `log 1` in setup and `log 0` in terminate.
- If logging is meant to stay always on, launch `teensy_interface` without `-l`.
