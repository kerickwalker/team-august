# Kalman MQTT Commands (Realtime Testing)

This file documents the MQTT topics and commands added for realtime Kalman testing in `team-august`.

## Topics

- Command input topic: `robobot/drive/kalman/cmd`
- Kalman state output topic: `robobot/kalman/state`

## Start The Robot Client

Run this first:

```bash
cd "/home/berko/Documents/Python Codes/BDRS/team-august"
python3 mqtt-client.py
```

## Watch Kalman Output

In another terminal:

```bash
mosquitto_sub -h localhost -t "robobot/kalman/state" -v
```

## Command List

Send commands to `robobot/drive/kalman/cmd`.

### `help`
Shows all supported Kalman commands.

```bash
mosquitto_pub -h localhost -t "robobot/drive/kalman/cmd" -m "help"
```

### `state`
Publishes the current Kalman state immediately.

```bash
mosquitto_pub -h localhost -t "robobot/drive/kalman/cmd" -m "state"
```

### `set_u <left_mps> <right_mps>`
Sets manual Kalman model input (wheel linear velocities in m/s).

```bash
mosquitto_pub -h localhost -t "robobot/drive/kalman/cmd" -m "set_u 0.20 0.24"
```

### `clear_u`
Clears manual input and returns Kalman input source to live wheel velocity sensors.

```bash
mosquitto_pub -h localhost -t "robobot/drive/kalman/cmd" -m "clear_u"
```

### `predict <dt_s> [left_mps right_mps]`
Runs one predict-only model step (no measurement update).

- Use current input source (manual `u` if set, else sensor wheel velocities):

```bash
mosquitto_pub -h localhost -t "robobot/drive/kalman/cmd" -m "predict 0.05"
```

- Override with one-shot input for this predict command only:

```bash
mosquitto_pub -h localhost -t "robobot/drive/kalman/cmd" -m "predict 0.05 0.30 0.10"
```

### `enable <0|1>`
Disables or enables Kalman updates.

```bash
mosquitto_pub -h localhost -t "robobot/drive/kalman/cmd" -m "enable 0"
mosquitto_pub -h localhost -t "robobot/drive/kalman/cmd" -m "enable 1"
```

### `reset`
Resets Kalman state to default start state (`[0, 0, 0, 0, 0, 0, 0]`).

```bash
mosquitto_pub -h localhost -t "robobot/drive/kalman/cmd" -m "reset"
```

### `reset m`
Resets Kalman state from current sensor measurement.

```bash
mosquitto_pub -h localhost -t "robobot/drive/kalman/cmd" -m "reset m"
```

### `reset <x y z v w yaw pitch>`
Resets Kalman state to a custom 7-state vector.

```bash
mosquitto_pub -h localhost -t "robobot/drive/kalman/cmd" -m "reset 0 0 0 0 0 0 0"
mosquitto_pub -h localhost -t "robobot/drive/kalman/cmd" -m "reset 1.0 2.0 0.0 0.1 0.0 0.2 -0.05"
```

## Output Payload Format

Messages on `robobot/kalman/state` are JSON.

Example:

```json
{
  "source": "cmd_state",
  "time": 1710000000.123,
  "has_estimate": true,
  "u": [0.2, 0.24],
  "x": {
    "x": 0.13,
    "y": 0.02,
    "z": 0.0,
    "velocity": 0.21,
    "angular_velocity": 0.17,
    "yaw": 0.05,
    "pitch": -0.01
  }
}
```

### `source` Values You May See

- `sensor_update`: regular update from sensor-driven Kalman cycle
- `cmd_state`: response to `state`
- `cmd_set_u`: response to `set_u`
- `cmd_clear_u`: response to `clear_u`
- `cmd_predict`: response to `predict`
- `cmd_enable`: response to `enable`
- `cmd_reset_default`: response to `reset`
- `cmd_reset_measurement`: response to `reset m`
- `cmd_reset_custom`: response to custom `reset ...`
- `cmd_help`: response to `help`
- `cmd_error`: invalid command or bad arguments

## Quick End-to-End Test

1. Start client: `python3 mqtt-client.py`
2. Subscribe state topic.
3. Send:

```bash
mosquitto_pub -h localhost -t "robobot/drive/kalman/cmd" -m "reset"
mosquitto_pub -h localhost -t "robobot/drive/kalman/cmd" -m "set_u 0.20 0.24"
mosquitto_pub -h localhost -t "robobot/drive/kalman/cmd" -m "predict 0.05"
mosquitto_pub -h localhost -t "robobot/drive/kalman/cmd" -m "state"
```

You should see `robobot/kalman/state` messages for each command.
