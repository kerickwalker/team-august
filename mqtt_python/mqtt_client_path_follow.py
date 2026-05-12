# mqtt_client_path_follow.py — Pure Pursuit path-following mission
#
# Run with:
#   python3 mqtt_client_path_follow.py
#
# Prerequisites:
#   - teensy_interface must be running
#   - Kalman filter node must be publishing on robobot/kalman/state
#   - trajectory.csv must exist in this directory (edit TRAJECTORY_CSV in spath_follow.py)
#
# The robot will track the trajectory at 0.3 m/s and stop when it reaches the end.
# Ctrl-C or the hardware stop button will stop the robot cleanly at any time.

import sys as _sys
import time as t
from datetime import *
from setproctitle import setproctitle
from spose import pose
from sgpio import gpio
from uservice import service
from spath_follow import path_follow
from spursuit import pursuit

# ─── optional save-on-exit plot (--plot / -p) ───────────────────────────────
# Records the robot's Kalman pose during the mission and saves a PNG when the
# script stops (normal end or Ctrl-C).  No display server needed — works over
# plain SSH.  Copy the file off the Pi with: scp pi@<ip>:~/robobot/mqtt_python/path_follow_result.png .
_PLOT = '--plot' in _sys.argv or '-p' in _sys.argv
if _PLOT:
    _sys.argv = [a for a in _sys.argv if a not in ('--plot', '-p')]
    _pose_history = []   # (y_right, x_fwd) in display frame, sampled at ~10 Hz

def _plot_save(traj):
    import matplotlib
    matplotlib.use('Agg')           # non-interactive backend — no display needed
    import matplotlib.pyplot as _plt
    import os
    ys = [p[0] for p in _pose_history]
    xs = [p[1] for p in _pose_history]
    fig, ax = _plt.subplots(figsize=(8, 8))
    ax.plot(-traj[:, 1], traj[:, 0], 'b-', lw=1.5, label='desired trajectory')
    if xs:
        ax.plot(ys, xs, color='orange', lw=1.5, label='actual path')
        ax.plot(ys[0], xs[0], 'k^', ms=8, label='robot start')
        ax.plot(ys[-1], xs[-1], 'kx', ms=10, mew=2, label='robot end')
    ax.plot(float(-traj[0, 1]), float(traj[0, 0]), 'go', ms=8, label='traj start')
    ax.plot(float(-traj[-1, 1]), float(traj[-1, 0]), 'rs', ms=8, label='traj end')
    ax.set_xlabel('y  (right →, m)')
    ax.set_ylabel('x  (forward ↑, m)')
    ax.set_title('Pure Pursuit — desired vs actual path')
    ax.legend(loc='upper right')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    _plt.tight_layout()
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'path_follow_result.png')
    _plt.savefig(out, dpi=150)
    _plt.close(fig)
    print(f'% Plot saved → {out}')
# ────────────────────────────────────────────────────────────────────────────

# ═══════════════════════════════════════════════════════════════════════════
# stateTime / stateTimePassed() — mission timing helper
# ═══════════════════════════════════════════════════════════════════════════
stateTime = datetime.now()

def stateTimePassed():
    return (datetime.now() - stateTime).total_seconds()

# ═══════════════════════════════════════════════════════════════════════════
# drivePathFollow()
#
# State machine:
#   0  → arm controller (load trajectory, set velocity), go to state 1
#   1  → call path_follow.update() at ~50 Hz
#        → when pursuit.at_end(): disarm, send rc 0 0, go to state 2
#   2  → wait for robot to coast to a stop (velocity < 1 mm/s)
#        → go to state 99
#   99 → exit
#
# Uses pose.velocity() (encoder odometry) only for the coast-to-stop check.
# All steering feedback comes from robobot/kalman/state via path_follow.
# ═══════════════════════════════════════════════════════════════════════════
def drivePathFollow():
    state = 0
    pose.tripBreset()
    from skalman import kalman
    kalman.reset()                  # zero full Kalman state so heading=0 matches x=0, y=0
    service.send("robobot/cmd/T0", "leds 16 0 100 0")
    path_follow.pathControl(0.2)    # arm: load trajectory.csv, set velocity 0.2 m/s
    last_print = datetime.now()
    _iter = 0
    while not service.stop:
        if state == 0:
            state = 1
        elif state == 1:
            path_follow.update()    # compute Pure Pursuit command and publish rc
            if _PLOT and _iter % 5 == 0:
                _pose_history.append((-path_follow._y, path_follow._x))
            if pursuit.at_end():
                path_follow.pathControl(0)
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                state = 2
        elif state == 2:
            if abs(pose.velocity()) < 0.001:
                if _PLOT:
                    _pose_history.append((-path_follow._y, path_follow._x))
                state = 99
        else:
            break
        if service.is_quiet() and (datetime.now() - last_print).total_seconds() >= 0.1:
            print(f"x={path_follow._x:.3f} m  y={-path_follow._y:.3f} m  "
                  f"hdg={-path_follow._heading:.3f} rad  "
                  f"rc={path_follow._last_linvel:.3f} m/s  {path_follow._last_turnrate:.3f} rad/s")
            last_print = datetime.now()
        _iter += 1
        t.sleep(0.02)               # ~50 Hz control loop
    if _PLOT:
        _plot_save(pursuit._traj)
    service.send("robobot/cmd/T0", "leds 16 0 0 0")

# ═══════════════════════════════════════════════════════════════════════════
# debugPose() — debug mode: print Kalman pose without sending any commands
# ═══════════════════════════════════════════════════════════════════════════
def debugPose():
    print("% Debug mode — printing Kalman pose. Ctrl-C to stop.")
    while not service.stop:
        if path_follow._pose_received:
            print(f"% Kalman pose:  x={path_follow._x:.3f} m  "
                  f"y={path_follow._y:.3f} m  "
                  f"heading={path_follow._heading:.4f} rad  "
                  f"speed={path_follow._speed:.3f} m/s")
        else:
            print("% Waiting for Kalman pose on robobot/kalman/state ...")
        t.sleep(1.0)

# ═══════════════════════════════════════════════════════════════════════════
# loop() — top-level state machine
#
#   101 → drivePathFollow() → 100 (return-to-idle)
#   200 → stationary hold; exit after 300 s (safety timeout)
#   else → exit
# ═══════════════════════════════════════════════════════════════════════════
def loop():
    from ulog import flog
    state = 101             # go straight into path-follow mission
    oldstate = -1
    stateTime = datetime.now()
    service.send("robobot/cmd/T0", "leds 16 30 30 0")
    while not service.stop:
        if state == 101:
            drivePathFollow()
            state = 100
        elif state == 100:
            # return-to-idle transition
            service.send("robobot/cmd/ti", "rc 0.0 0.0")
            state = 200
        elif state == 200:
            # stationary hold — safety timeout
            service.send("robobot/cmd/T0", "leds 16 0 100 0")
            service.send("robobot/cmd/ti", "rc 0.0 0.0")
            if (datetime.now() - stateTime).total_seconds() > 300.0:
                state = 99
            t.sleep(1.0)
        else:
            break
        if state != oldstate:
            flog.writeRemark(f"% State change from {oldstate} to {state}")
            oldstate = state
        t.sleep(0.1)
    # --- cleanup ---
    service.send("robobot/cmd/T0", "leds 16 0 0 0")
    try:
        gpio.set_value(20, 0)
    except Exception:
        pass
    path_follow.pathControl(0)
    service.send("robobot/cmd/ti", "rc 0 0")
    t.sleep(0.05)

# ═══════════════════════════════════════════════════════════════════════════
# __main__
# ═══════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    if service.process_running("mqtt-client"):
        print("% mqtt-client already running — terminating")
    else:
        setproctitle("mqtt-client")
        service.setup("localhost")
        if service.connected:
            if service.args.debug:
                debugPose()
            else:
                loop()
        service.terminate()
    print("% Main Terminated")
