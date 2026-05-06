# Code map: line following

---

## Short version (takeaways + code)

**Data path in one line:** Teensy → USB → teensy_interface → MQTT `T0/livn` → **uservice** → **edge.decode()** → **LineDetect()** → **followLine()** → `rc v w` → MQTT `robobot/cmd/ti` → robot moves.

**Where the magic happens:** Every time a line-sensor message arrives, **uservice** calls `edge.decode("T0/livn", msg)`. That parses 8 numbers into `edge_n[]`, runs **LineDetect()** (→ `posLeft`, `posRight`, `lineValidCnt`), then if line-follow is on runs **followLine()**, which sends one `rc` command.

```python
# sedge.decode() when topic == "T0/livn":
for i in range(8):
    self.edge_n[i] = int(gg[i + 1])
self.LineDetect()      # → posLeft, posRight, lineValidCnt
if self.lineCtrl:
    self.followLine()  # → service.send("robobot/cmd/ti", f"rc {velocity} {lineY} ...")
```

**Mission side (`mqtt-client` / driveToLine):** Turn line-follow **on** when you see the line; turn it **off** when you lose it. Everything else is “drive forward until then” and “stop when lost”.

```python
# driveToLine() state 1: saw line?
if edge.lineValidCnt > 4:
    edge.lineControl(0.2, True)   # on: 0.2 m/s, follow left edge → followLine() runs on each livn
    state = 10
# state 10: lost line?
if edge.lineValidCnt < 2:
    edge.lineControl(0, True)     # off → no more rc from followLine
    service.send("robobot/cmd/ti", "rc 0.0 0.0")
```

**Takeaways:**
- **Input:** `edge_n[0..7]` from MQTT `T0/livn` (0–1000 per sensor). **Output:** `rc <velocity> <turn_rate>` to `robobot/cmd/ti`.
- **lineValidCnt** is a smoothed 0–20 “how sure we are we see the line”; mission uses it to start/stop following (>4 → follow, <2 → stop).
- **posLeft / posRight** are the two edge positions; controller error is `refPosition - posLeft` (or posRight) → P-Lead → `lineY` (turn rate) in `followLine()`.
- **lineControl(v, followLeft)** just sets `velocity`, `followLeft`, and `lineCtrl = (v > 0)`. The actual `rc` is sent inside **followLine()**, which is only called from **decode()** when `lineCtrl` is True.

The long sections below are for “where exactly is this variable set” and full call chains when you need them.

---

## End-to-end data flow (one sentence per step)

1. **Teensy** reads 8 IR sensors → normalizes to 0–1000 → publishes ASCII over USB (e.g. `livn timestamp v0 v1 ... v7`).
2. **teensy_interface** (C++) reads USB, decodes, publishes MQTT to `robobot/drive/T0/edge_livn` (and logs).
3. **MQTT broker** (mosquitto on localhost:1883) delivers to subscribed clients.
4. **uservice.py** (in a thread) receives the message; `on_message()` → `decode(topic, payload)`; topic is like `robobot/drive/T0/livn`; stripped to subtopic `T0/livn`; then `edge.decode("T0/livn", msg)` is called.
5. **sedge.py** `decode("T0/livn", msg)` parses the 9 numbers (timestamp + 8 values), stores in `edge_n[]`, calls `LineDetect()` then, if `lineCtrl`, `followLine()`; `followLine()` sends `rc <velocity> <turn_rate>` via `service.send("robobot/cmd/ti", par)`.
6. **uservice.send()** publishes to MQTT topic `robobot/cmd/ti`.
7. **teensy_interface** subscribes to that topic, converts `rc` to motor commands, sends to Teensy over USB → robot turns.

---

## File roles (quick reference)

| File | Role |
|------|------|
| **teensy_firmware_8** (C++) | Line sensor read, calibration, normalization; publishes `liv`, `livn`, `liw`, `lib`, `lip`. |
| **teensy_interface** (C++) | USB↔MQTT bridge. Subscribes to `robobot/cmd/ti` (rc), publishes `robobot/drive/T0/edge_livn` etc. |
| **uservice.py** | MQTT client: subscribe `robobot/drive/#`, dispatch by subtopic to pose, imu, **edge**, etc.; send commands to `robobot/cmd/ti` or `robobot/cmd/T0`. |
| **sedge.py** | Line state from `livn`; `LineDetect()` → posLeft/posRight, lineValid; `followLine()` → P-Lead → `rc v w`. |
| **mqtt-client.py** | Mission: chooses state (meter, pi, edge, stationary); calls `edge.lineControl(v, followLeft)` and `driveToLine()` which waits on `edge.lineValidCnt`. |

---

## Where each variable / command comes from

### Topics and messages (MQTT)

- **`robobot/drive/T0/livn`**  
  Source: teensy_interface (from Teensy USB).  
  Payload: `"timestamp v0 v1 v2 v3 v4 v5 v6 v7"` (float timestamp, then 8 ints 0–1000).  
  Consumer: `sedge.decode("T0/livn", msg)`.

- **`robobot/drive/T0/liv`**  
  Raw AD values; same format. Used in sedge for optional debug; main control uses `livn` only.

- **`robobot/drive/T0/liw`**  
  White calibration values (8 ints). Filled when calibration runs; sedge stores in `edge_n_w[]`.

- **`robobot/cmd/T0`**  
  Commands to the Teensy: `lip 1` (line sensor on), `lip 0` (off), `sub livn 10` (request livn every 10 ms), `liwi` (request white cal), `licw 100`, `eew`, etc.  
  Sender: sedge (setup/terminate) and mqtt-client (servo, leds).

- **`robobot/cmd/ti`**  
  Commands to teensy_interface: `rc <velocity> <turn_rate>` (and `log 1`/`log 0`, `alive`, etc.).  
  Sender: sedge in `followLine()`, mqtt-client in mission states.

### Sedge (sedge.py) – main state variables

| Variable | Set in | Meaning |
|----------|--------|---------|
| `edge_n[0..7]` | `decode("T0/livn", msg)` | Latest normalized line sensor values (0–1000). |
| `edge_nUpdCnt` | same | Number of livn updates received. |
| `lineValid` | `LineDetect()` | True if any sensor ≥ lineValidThreshold (750). |
| `lineValidCnt` | `LineDetect()` | Smoothed 0–20; increments when line valid, decrements when not. |
| `posLeft`, `posRight` | `LineDetect()` | Edge positions in sensor index space (-3.5..3.5); from threshold scan. |
| `average`, `high` | `LineDetect()` | Average and max of edge_n; used for crossing/valid. |
| `crossingLine`, `crossingLineCnt` | `LineDetect()` | Average ≥ crossingThreshold (700). |
| `refPosition`, `followLeft`, `velocity` | `lineControl(v, followLeft, ref)` | Setpoint and which edge to follow; velocity (m/s). |
| `lineCtrl` | `lineControl(v, ...)` | True when velocity > 0.001; enables followLine() in decode. |
| `lineY`, `u`, `lineE1`, `lineY1` | `followLine()` | P-Lead output (turn rate rad/s) and state. |

### Commands sent by sedge

- **Setup:** `service.send("robobot/cmd/T0", "lip 1")` then `"sub livn 10"` (turn line sensor on; request livn every 10 ms).
- **When line following:** `service.send("robobot/cmd/ti", f"rc {velocity} {lineY} {time}")` each time a new `livn` arrives and `lineCtrl` is True.
- **Terminate:** `service.send("robobot/cmd/T0", "lip 0")`.

### mqtt-client.py – line-follow mission

- **driveToLine()**  
  State 0: send `rc 0.2 0.0` (drive forward).  
  State 1: when `edge.lineValidCnt > 4` → call `edge.lineControl(0.2, True)` (follow left edge), then state 10.  
  State 10: when `edge.lineValidCnt < 2` → `edge.lineControl(0, True)` and `rc 0 0`, then state 2 and exit path.  
  So: drive until line detected (lineValidCnt>4), then follow until line lost (lineValidCnt<2).

- **loop()**  
  If `--edge`: state 103 → `driveToLine()` then state 100.  
  Before loop: `edge.lineControl(0, True)` so line control starts off.  
  On exit: `edge.lineControl(0, True)` and `rc 0 0`.

### pose (spose.py) – used by mqtt-client

- **tripB**, **tripBtimePassed()**  
  Used in driveToLine (and driveOneMeter, driveTurnPi) to limit distance/time (e.g. tripB > 1.0 m or time > 15 s).  
  **tripBh**  
  Heading change (rad); used in driveTurnPi (e.g. > π) and in other states.  
  **velocity()**, **turnrate()**  
  Used to wait until robot has stopped before finishing a state.  
  Source: pose gets encoder/velocity from MQTT (e.g. T0/enc, T0/vel) and integrates tripB/tripBh.

---

## Call chain summary

**At startup (uservice.setup):**  
`edge.setup()` → sends `lip 1`, `sub livn 10`, optionally runs white calibration loop (sends `liwi`, `licw 100`, `eew`), then returns.

**On every livn message:**  
MQTT → uservice `on_message` → `decode` → `edge.decode("T0/livn", msg)` → parse into `edge_n[]` → `LineDetect()` (updates posLeft, posRight, lineValid, lineValidCnt, etc.) → if `lineCtrl`: `followLine()` → P-Lead from error → `service.send("robobot/cmd/ti", "rc v w")`.

**When user runs `python3 mqtt-client.py --edge`:**  
`loop()` state 103 → `driveToLine()` → state machine that drives forward until `edge.lineValidCnt > 4`, then calls `edge.lineControl(0.2, True)` so subsequent livn messages trigger followLine(); when `edge.lineValidCnt < 2`, turns off line control and stops.

---

## Note on state 200 timeout (mqtt-client.py)

In the current code, state 200 uses `stateTimePassed() > 300.0` (300 seconds). The notes say 60 s auto-exit for stationary; if that was intended, the constant should be `60.0`.

---

## Core files (for reading / mapping)

- **`mqtt_python/sedge_core.py`** — Stripped `sedge.py`: no debug prints, no paint() body, same logic with section comments. **Not** used at runtime; the real client imports `sedge` (sedge.py). Use this file to trace where each variable is set and where commands go.
- **`mqtt_python/mqtt_client_core.py`** — Stripped `mqtt-client.py`: line-follow + meter + pi + stationary + motpwm, minimal prints, section comments. You can run it instead of mqtt-client: `python3 mqtt_client_core.py --edge` (it still uses `from sedge import edge`, i.e. the original sedge.py).
