# Robot Teleoperation with Kalman Filter Integration

This guide explains how to use the prompt-based teleoperation system with Kalman filtering for model-based predictions.

## Overview

The teleoperation system has three main components:

1. **teleop_input.py** - Sends velocity commands via MQTT (two modes: velocity or motor)
2. **mqtt-client.py** - Receives and executes commands on the robot
3. **Kalman Filter (skalman.py)** - Fuses teleoperation input with sensors for state estimation
4. **kalman_output.py** - Reads and displays the Kalman-estimated robot state

### Data Flow

```
┌─────────────────────┐
│  Teleoperation Input  │  
│  Prompt or Motor      │  [User: "0.5 0.2" or "1.0 1.0"]
└────────┬─────────────┘
         │ MQTT: robobot/teleop/cmd
         │ {"linear_velocity": 0.5, "angular_velocity": 0.2, 
         │  "v_left": 0.42, "v_right": 0.58, "timestamp": "..."}
         │
         ├──────────────────────┬────────────────────────┐
         │                      │                        │
    ┌────▼────┐          ┌────▼────┐          ┌─────▼──┐
    │  Kalman  │          │  Motor   │          │ Sensors│
    │  Filter  │          │  Control │          │ (IMU,  │
    │  (State  │          │  (RC cmd)│          │ Encoder)
    │ Estimate)│          │          │          │        │
    └────┬────┘          └────┬────┘          └─────┬──┘
         │                    │                     │
         │ State estimate      │ Motor PWM/commands │ Sensor readings
         │                    │                     │
         └─────────┬────────────────┬──────────────┘
                   │                │
              MQTT publish     MQTT publish
              robobot/state    robobot/sensors
```

## Setup: Two Terminal Method

### Terminal 1: Run Teleoperation Input

Start the MQTT Client with the robot (if not already running):

```bash
cd mqtt_python
python3 mqtt-client.py -i localhost
```

The system will connect to MQTT and wait for teleoperation commands.

### Terminal 2: Send Velocity Commands

In a separate terminal, run the teleoperation input client:

#### Velocity Mode (Default)
```bash
cd mqtt_python
python3 teleop_input.py -i localhost
```

Enter linear velocity (m/s) and angular velocity (rad/s):
```
Enter velocity [lin ang] or command: 0.5 0.2
✓ Published: linear=0.500, angular=0.200
```

#### Motor Mode (Direct left/right motor velocities)
```bash
cd mqtt_python
python3 teleop_input.py -i localhost -m
```

Enter left and right motor velocities (m/s):
```
Enter velocity [L/R m/s] or command: 1.0 1.0
✓ Published motor: L=1.000, R=1.000 m/s
```

You'll see interactive prompts for each mode with examples and limits.

### Terminal 3 (Optional): Monitor Kalman State

To view the Kalman-filtered state estimates in real-time:

```bash
cd mqtt_python
python3 kalman_output.py -i localhost
```

## Usage Examples

### Velocity Mode (Linear / Angular)

#### Example 1: Drive Forward

```
Enter velocity [lin ang] or command: 0.5 0.0
✓ Published: linear=0.500, angular=0.000
```

The robot drives forward at 0.5 m/s.

#### Example 2: Turn Left

```
Enter velocity [lin ang] or command: 0.0 0.5
✓ Published: linear=0.000, angular=0.500
```

The robot turns left at 0.5 rad/s.

#### Example 3: Drive Forward-Right

```
Enter velocity [lin ang] or command: 0.4 -0.3
✓ Published: linear=0.400, angular=-0.300
```

The robot drives forward while turning right.

#### Example 4: Stop

```
Enter velocity [lin ang] or command: 0.0 0.0
✓ Published: linear=0.000, angular=0.000
```

### Motor Mode (Direct Left/Right Motor Velocities)

Start with `-m` flag:
```bash
python3 teleop_input.py -i localhost -m
```

#### Example 1: Both Motors Forward at 1.0 m/s

```
Enter velocity [L/R m/s] or command: 1.0 1.0
✓ Published motor: L=1.000, R=1.000 m/s
```

Both wheels drive forward.

#### Example 2: Turn Right (differential speed)

```
Enter velocity [L/R m/s] or command: 0.5 1.0
✓ Published motor: L=0.500, R=1.000 m/s
```

Left wheel slower, right wheel faster → robot turns right.

#### Example 3: Spin Left (opposite speeds)

```
Enter velocity [L/R m/s] or command: -0.5 0.5
✓ Published motor: L=-0.500, R=0.500 m/s
```

Motors in opposite directions → robot spins in place.

#### Example 4: Stop

```
Enter velocity [L/R m/s] or command: 0.0 0.0
✓ Published motor: L=0.000, R=0.000 m/s
```

### General Commands

In either mode, type:
```
help     -> Show velocity examples and limits
q, quit  -> Exit teleoperation
```

## Configuration Options

### teleop_input.py

```bash
# Velocity mode (default): linear and angular velocities
python3 teleop_input.py -i localhost

# Motor mode: direct left and right motor velocities
python3 teleop_input.py -i localhost -m

# Custom MQTT broker host and port
python3 teleop_input.py -i 10.197.217.80 -p 1883

# Adjust velocity limits (velocity mode)
python3 teleop_input.py --max-vel 0.8 --max-turn 1.5

# Motor mode with custom host
python3 teleop_input.py -m -i 192.168.1.100

# All options combined
python3 teleop_input.py -m -i 192.168.1.100 -p 1883 --max-vel 1.2 --max-turn 2.5
```

| Option | Default | Meaning |
|--------|---------|---------|
| `-i, --host` | localhost | MQTT broker hostname |
| `-p, --port` | 1883 | MQTT broker port |
| `-m, --motor` | False | Enable motor mode (direct L/R velocities) |
| `--max-vel` | 1.0 | Max linear velocity (m/s) |
| `--max-turn` | 2.0 | Max angular velocity (rad/s) |

### kalman_output.py

```bash
# Read state with custom hosts
python3 kalman_output.py -i 10.197.217.80

# Different refresh rate (seconds)
python3 kalman_output.py -i localhost -r 0.1
```

## System Integration

### Data Channels (MQTT Topics)

| Topic | Direction | Content | Source |
|-------|-----------|---------|--------|
| `robobot/teleop/cmd` | → | Velocity input with motors `{linear_velocity, angular_velocity, v_left, v_right, timestamp}` | teleop_input.py |
| `robobot/state` | ← | Kalman-filtered state estimate | skalman.py |
| `robobot/cmd/ti` | → | Motor control commands (`rc {v} {w}`) sent directly from teleoperation | uservice.py |
| `robobot/sensors/*` | ← | Raw sensor data (IMU, encoders) | Robot hardware |

### Processing Flow

1. **Teleoperation Input**
   - **Velocity Mode**: User enters: `0.5 0.2` → linear_velocity=0.5, angular_velocity=0.2
   - **Motor Mode**: User enters: `1.0 1.0` → v_left=1.0, v_right=1.0 (converted to linear/angular)
   - Published to: `robobot/teleop/cmd`
   - Message includes both linear/angular and v_left/v_right for flexibility

2. **Direct Motor Control** (uservice.py)
   - `uservice.py` intercepts `robobot/teleop/cmd`
   - Immediately sends `rc {linear_velocity} {angular_velocity}` to teensy_interface
   - Enables real-time motor response without waiting for Kalman filter cycle

3. **Kalman Filter Processing** (skalman.py)
   - Receives teleoperation input via `uservice.on_message()`
   - Calls: `kalman.decode_teleoperation(msg)`
   - Uses input as control input for model-based state prediction
   - Sensor readings update the state estimate (fusion)
   - Publishes merged state estimate to `robobot/state`

3. **Motor Control** (mqtt-client.py)
   - Converts velocity commands to motor control signals
   - Sends: `robobot/cmd/ti` with `rc {linear} {angular}`
   - Teensy interface drives motors

4. **State Feedback** (kalman_output.py)
   - Reads Kalman state from `robobot/state`
   - Displays merged estimate of:
     - Position (x, y, heading)
     - Velocity (linear, angular)
     - Sensor readings (left/right wheel encoders, IMU)

## Recent Updates & Fixes

### Motor Control Integration (Direct RC Commands)
- Teleoperation commands now directly send `rc` motor commands to the teensy interface
- Eliminates the control lag between teleoperation input and motor response
- Both channels publish to MQTT: Kalman filter (for state estimation) and motor controller (for immediate action)

### Dual Input Modes
- **Velocity Mode** (default): Input linear velocity (m/s) and angular velocity (rad/s)
  - Useful for mission planning and high-level control
- **Motor Mode** (`-m` flag): Input left and right motor velocities directly
  - Better for hands-on teleoperation and debugging motor response

### Kalman Filter Robustness
- Fixed startup race condition where MQTT messages could arrive before Kalman filter initialization
- Kalman filter now auto-initializes if measurements arrive early
- Prevents `"NoneType' object has no attribute 'x"` crash

### Paho MQTT v2 Compatibility
- Updated teleoperation input to support both paho-mqtt v1 and v2
- Uses `CallbackAPIVersion.VERSION1` for v2 to maintain compatibility
- Automatic fallback for v1 installations

## Troubleshooting

### "ERROR: Not connected to MQTT broker"
- Ensure MQTT broker is running on `localhost:1883`
- Check that `mqtt-client.py` is running in Terminal 1
- Verify firewall doesn't block port 1883

### No response to velocity commands
- Check `mqtt-client.py` console for errors
- Verify robot is powered on and Teensy interface is connected
- Look at `log_out.txt` and `log_err.txt` for detailed logs

### Kalman state not updating
- Ensure sensors are providing data
- Check that encoder values are being published
- Look at IMU readings in motor control console

### Velocity limits exceeded
- The system automatically clamps values to safe limits:
  - Linear velocity: `[-1.0, 1.0]` m/s
  - Angular velocity: `[-2.0, 2.0]` rad/s
- Values outside this range are clamped without warning

## Advanced Usage

### Batch Testing

Create a test script that sends multiple commands:

```bash
#!/bin/bash
{
    echo "0.2 0.0"   # Move forward slowly
    sleep 2
    echo "0.0 0.5"   # Turn left
    sleep 1
    echo "0.0 0.0"   # Stop
    sleep 1
    echo "q"         # Quit
} | python3 teleop_input.py
```

### Data Logging

Monitor MQTT messages directly:

```bash
# Subscribe to all teleoperation commands
mosquitto_sub -h localhost -p 1883 -t "robobot/teleop/#" -v

# Subscribe to state updates
mosquitto_sub -h localhost -p 1883 -t "robobot/state" -v

# Subscribe to everything
mosquitto_sub -h localhost -p 1883 -t "robobot/#" -v
```

### Performance Monitoring

Check system logs:

```bash
# Show last 50 lines of output log
tail -50 mqtt_python/log_out.txt

# Show last 50 lines of error log
tail -50 mqtt_python/log_err.txt

# Monitor in real-time
tail -f mqtt_python/log_out.txt
```

## Architecture Notes

- **Kalman Filter**: Uses teleoperation input as feedforward control term
- **Motor Control**: Applies velocity commands to differential-drive kinematics
- **Sensor Fusion**: Combines encoder and IMU data with model predictions
- **MQTT Decoupling**: All components communicate via MQTT, allowing:
  - Remote operation over network
  - Asynchronous processing
  - Easy monitoring and logging

## Files Reference

- [teleop_input.py](teleop_input.py) - Prompt-based velocity input
- [mqtt-client.py](mqtt-client.py) - Main robot control loop
- [skalman.py](skalman.py) - Kalman filter state estimation
- [kalman_output.py](kalman_output.py) - State monitor
- [teleop_motor_control.py](teleop_motor_control.py) - Motor control interface
- [uservice.py](uservice.py) - MQTT service wrapper

