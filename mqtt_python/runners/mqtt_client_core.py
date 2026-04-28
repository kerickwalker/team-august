# mqtt_client_core.py - stripped, commented mission control (line-follow path in focus)
# See notes/code_map_line_following.md for data flow. Original: mqtt-client.py.
# Run with: python3 mqtt_client_core.py (uses same uservice; ensure no duplicate process)

import time as t
from datetime import *
from setproctitle import setproctitle
from sensors.spose import pose
from sensors.sir import ir
from sensors.srobot import robot
from sensors.scam import cam
from sensors.sedge import edge
from sgpio import gpio
from util.uservice import service

# ═══════════════════════════════════════════════════════════════════════════
# stateTime / stateTimePassed() - used for stationary timeout and state timing
# ═══════════════════════════════════════════════════════════════════════════
stateTime = datetime.now()

def stateTimePassed():
    return (datetime.now() - stateTime).total_seconds()

# ═══════════════════════════════════════════════════════════════════════════
# driveOneMeter() — state 0 → rc 0.2 0 → state 1; when tripB > 1 m or 15 s → rc 0 0 → state 2;
# when velocity ~ 0 → state 99 and exit. pose.tripB from encoder (spose) via MQTT.
# ═══════════════════════════════════════════════════════════════════════════
def driveOneMeter():
    state = 0
    pose.tripBreset()
    service.send("robobot/cmd/T0", "leds 16 0 100 0")
    while not service.stop:
        if state == 0:
            service.send("robobot/cmd/ti", "rc 0.2 0.0")
            service.send("robobot/cmd/T0", "servo 1 -800 300")
            state = 1
        elif state == 1:
            if pose.tripB > 1.0 or pose.tripBtimePassed() > 15:
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                service.send("robobot/cmd/T0", "servo 1 0 0")
                state = 2
        elif state == 2:
            if abs(pose.velocity()) < 0.001:
                state = 99
        else:
            service.send("robobot/cmd/ti", "rc 0.0 0.0")
            break
        t.sleep(0.05)
    service.send("robobot/cmd/T0", "leds 16 0 0 0")

# ═══════════════════════════════════════════════════════════════════════════
# driveToLine() — LINE-FOLLOW MISSION
# State 0: drive forward (rc 0.2 0) until ir.ir[0] >= 0.2 or we just start.
# State 1: keep driving; when edge.lineValidCnt > 4 → start following: edge.lineControl(0.2);
#           then state 10. Also exit state 1 if tripB > 1 m or 15 s → state 2.
# State 10: follow line (control runs in sedge on each livn). When edge.lineValidCnt < 2 →
#           stop line control, rc 0 0, state 2.
# State 2: wait until velocity ~ 0 then state 99 and exit.
# So: drive to line → follow left edge until line lost → stop.
# ═══════════════════════════════════════════════════════════════════════════
def driveToLine():
    state = 0
    pose.tripBreset()
    service.send("robobot/cmd/T0", "leds 16 0 100 0")
    while not service.stop:
        if state == 0:  
            if ir.ir[0] < 0.2: # start when hand in front
                service.send("robobot/cmd/ti", "rc 0.2 0.0")
                service.send("robobot/cmd/T0", "servo 1 -800 300")
                state = 1
        elif state == 1:
            if pose.tripB > 1.0 or pose.tripBtimePassed() > 15:
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                state = 2
            if edge.lineValidCnt > 4:
                edge.lineControl(0.2)   # 0.2 m/s, follow line center
                service.send("robobot/cmd/T0", "servo 1 0 0")
                pose.tripBreset()
                state = 10
        elif state == 2:
            if abs(pose.velocity()) < 0.001:
                state = 99
        elif state == 10:
            if edge.lineValidCnt < 2:
                edge.lineControl(0)     # turn off line control
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                pose.tripBreset()
                state = 2
        else:
            service.send("robobot/cmd/ti", "rc 0.0 0.0")
            service.send("robobot/cmd/T0", "servo 1 500 200")
            break
        t.sleep(0.01)
    service.send("robobot/cmd/T0", "leds 16 0 0 0")

def driveMotorsPwmDuration(left_pwm, right_pwm, duration_s):
    """Map PWM to rc (linvel, turnrate), send for duration_s, then stop."""
    max_pwm = 4096.0
    max_linvel = 1.0
    wheelbase = 0.22
    left_n = max(-1, min(1, float(left_pwm) / max_pwm))
    right_n = max(-1, min(1, float(right_pwm) / max_pwm))
    linvel = 0.5 * (left_n + right_n) * max_linvel
    turnrate = (right_n - left_n) * max_linvel / wheelbase
    service.send("robobot/cmd/ti", f"rc {linvel:.3f} {turnrate:.3f}")
    t0 = t.time()
    while not service.stop and (t.time() - t0) < max(0, float(duration_s)):
        t.sleep(0.02)
    service.send("robobot/cmd/ti", "rc 0 0")

# ═══════════════════════════════════════════════════════════════════════════
# driveTurnPi() — turn in place until tripBh > π or 15 s; then stop and exit.
# ═══════════════════════════════════════════════════════════════════════════
def driveTurnPi():
    state = 0
    pose.tripBreset()
    service.send("robobot/cmd/T0", "leds 16 0 100 0")
    while not service.stop:
        if state == 0:
            service.send("robobot/cmd/ti", "rc 0.0 0.5")
            state = 1
        elif state == 1:
            if pose.tripBh > 3.14 or pose.tripBtimePassed() > 15:
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                state = 2
        elif state == 2:
            if abs(pose.velocity()) < 0.001 and abs(pose.turnrate()) < 0.001:
                state = 99
        else:
            service.send("robobot/cmd/ti", "rc 0.0 0.0")
            break
        t.sleep(0.05)
    service.send("robobot/cmd/T0", "leds 16 0 0 0")

# ═══════════════════════════════════════════════════════════════════════════
# loop() — main state machine. Initial state from flags: --meter→101, --pi→102,
# --edge→103, --motpwm→104, else 200 (stationary). Then loop:
#   101 → driveOneMeter() → 100
#   102 → driveTurnPi() → 100
#   103 → driveToLine() → 100   ← line-follow entry
#   200 → rc 0 0, sleep 1 s; exit after stateTimePassed() > 300 (note: notes say 60 s)
# On exit: leds off, edge.lineControl(0), rc 0 0.
# ═══════════════════════════════════════════════════════════════════════════
def loop():
    from util.ulog import flog
    state = 0
    oldstate = -1
    service.send("robobot/cmd/T0", "leds 16 30 30 0")
    if service.args.meter:
        state = 101
    elif service.args.pi:
        state = 102
    elif service.args.edge:
        state = 103
    elif service.args.motpwm is not None:
        state = 104
    elif service.args.usestate > 0:
        state = service.args.usestate
    else:
        state = 200
    edge.lineControl(0)   # ensure line control off at start
    while not service.stop:
        if state == 101:
            driveOneMeter()
            state = 100
        elif state == 102:
            driveTurnPi()
            state = 100
        elif state == 103:
            driveToLine()
            state = 100
        elif state == 104:
            left_pwm, right_pwm, duration_s = service.args.motpwm
            driveMotorsPwmDuration(left_pwm, right_pwm, duration_s)
            state = 100
        elif state == 200:
            service.send("robobot/cmd/T0", "leds 16 0 100 0")
            service.send("robobot/cmd/ti", "rc 0.0 0.0")
            if stateTimePassed() > 300.0:   # 300 s in code; notes mention 60 s
                state = 99
            t.sleep(1.0)
        else:
            break
        if state != oldstate:
            flog.writeRemark(f"% State change from {oldstate} to {state}")
            oldstate = state
            stateTime = datetime.now()
        t.sleep(0.1)
    service.send("robobot/cmd/T0", "leds 16 0 0 0")
    try:
        gpio.set_value(20, 0)
    except Exception:
        pass
    edge.lineControl(0)
    service.send("robobot/cmd/ti", "rc 0 0")
    service.send("robobot/cmd/T0", "servo 1 0 0")
    t.sleep(0.05)

# ═══════════════════════════════════════════════════════════════════════════
# __main__ — single process check, setproctitle, service.setup('localhost'),
# then loop(); finally service.terminate() (which calls edge.terminate() etc.).
# ═══════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    if service.process_running("mqtt-client"):
        print("% mqtt-client (or mqtt_client_core) already running - terminating")
    else:
        setproctitle("mqtt-client")
        service.setup("10.197.219.117")
        if service.connected:
            loop()
        service.terminate()
    print("% Main Terminated")
