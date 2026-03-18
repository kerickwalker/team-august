# Robot Teleoperation with Kalman Filter Integration

This guide explains how to use the prompt-based teleoperation system with Kalman filtering for model-based predictions.

## Overview

The teleoperation system has three main components:

1. **teleop_input.py** - Sends velocity commands via MQTT
2. **mqtt-client.py** - Receives and executes commands on the robot
3. **Kalman Filter (skalman.py)** - Fuses teleoperation input with sensors for state estimation
4. **kalman_output.py** - Reads and displays the Kalman-estimated robot state

### Data Flow

```
┌─────────────┐
│  Teleoperation │  [User inputs: "0.5 0.2"]
│  Input Prompt   │
└────────┬────┘
         │ MQTT: robobot/teleop/cmd
         │ {"linear_velocity": 0.5, "angular_velocity": 0.2}
         │
         ├──────────────────────┬────────────────────────┐
         │                      │                        │
    ┌────▼────┐          ┌────▼────┐          ┌─────▼──┐
    │  Kalman  │          │  Motor   │          │ Sensors│
    │  Filter  │          │ Control  │          │ (IMU,  │
    │          │          │          │          │ Encoder)
    └────┬────┘          └────┬────┘          └─────┬──┘
         │                    │                     │
         │ State estimate      │ Motor commands      │ Sensor readings
         │                    │                     │
         └─────────┬────────────────┬──────────────┘
                   │                │
              MQTT publish     MQTT publish
              robobot/state    robobot/sensors
                   │
         ┌─────────▼──────────┐
         │  kalman_output.py  │
         │  (Display merged   │
         │   state estimate)  │
         └────────────────────┘
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

```bash
cd mqtt_python
python3 teleop_input.py -i localhost
```

You'll see:
```
% Connecting to MQTT broker localhost:1883...
% Connected to MQTT Broker at localhost:1883

% ===== Teleoperation Input (Velocity Prompt) =====
% Connected and ready for velocity commands

% Teleoperation Input Commands:
% ==============================
% Enter velocity input as two space-separated values:
%   <linear_vel> <angular_vel>
%
% Examples:
%   0.5 0.0      -> Forward at 0.5 m/s
%   -0.5 0.0     -> Backward at 0.5 m/s
%   0.0 0.5      -> Turn left at 0.5 rad/s
%   0.0 -0.5     -> Turn right at 0.5 rad/s
%   0.3 0.3      -> Forward-left
%   0.3 -0.3     -> Forward-right
%   0.0 0.0      -> STOP
%
% Limits: linear_vel [-1.0, 1.0] m/s
%         angular_vel [-2.0, 2.0] rad/s
%
% Commands:
%   help, h, ?   -> Show this help
%   q, quit      -> Exit
% ==============================

Enter velocity [linear angular] or command:
```

### Terminal 3 (Optional): Monitor Kalman State

To view the Kalman-filtered state estimates in real-time:

```bash
cd mqtt_python
python3 kalman_output.py -i localhost
```

## Usage Examples

### Example 1: Drive Forward

```
Enter velocity [linear angular] or command: 0.5 0.0
✓ Published: [0.500, 0.000] -> robobot/teleop/cmd
```

The robot drives forward at 0.5 m/s.

### Example 2: Turn Left

```
Enter velocity [linear angular] or command: 0.0 0.5
✓ Published: [0.000, 0.500] -> robobot/teleop/cmd
```

The robot turns left at 0.5 rad/s.

### Example 3: Drive Forward-Right

```
Enter velocity [linear angular] or command: 0.4 -0.3
✓ Published: [0.400, -0.300] -> robobot/teleop/cmd
```

The robot drives forward while turning right.

### Example 4: Stop

```
Enter velocity [linear angular] or command: 0.0 0.0
✓ Published: [0.000, 0.000] -> robobot/teleop/cmd
```

The robot stops.

### Example 5: Show Help

```
Enter velocity [linear angular] or command: help
```

## Configuration Options

### teleop_input.py

```bash
# Custom MQTT broker host and port
python3 teleop_input.py -i 10.197.217.80 -p 1883

# Adjust velocity limits
python3 teleop_input.py --max-vel 0.8 --max-turn 1.5

# Combined
python3 teleop_input.py -i 192.168.1.100 --max-vel 1.2 --max-turn 2.5
```

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
| `robobot/teleop/cmd` | → | Velocity input `{linear_velocity, angular_velocity}` | teleop_input.py |
| `robobot/state` | ← | Kalman-filtered state estimate | skalman.py |
| `robobot/cmd/ti` | → | Motor control commands (`rc {v} {w}`) | mqtt-client.py |
| `robobot/sensors/*` | ← | Raw sensor data (IMU, encoders) | Robot hardware |

### Processing Flow

1. **Teleoperation Input**
   - User enters: `0.5 0.2`
   - Published to: `robobot/teleop/cmd`
   - Message: `{"linear_velocity": 0.5, "angular_velocity": 0.2, "timestamp": "..."}`

2. **Kalman Filter Processing** (skalman.py)
   - Receives teleoperation input via `uservice.on_message()`
   - Calls: `kalman.decode_teleoperation(msg)`
   - Uses input as control input for model-based state prediction
   - Sensor readings update the state estimate (fusion)

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

