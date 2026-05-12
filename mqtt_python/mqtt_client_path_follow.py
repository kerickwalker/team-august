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

# ─── optional live plot (--plot / -p) ───────────────────────────────────────
_PLOT = '--plot' in _sys.argv or '-p' in _sys.argv
if _PLOT:
    _sys.argv = [a for a in _sys.argv if a not in ('--plot', '-p')]
    import matplotlib.pyplot as _plt
    from math import cos as _cos, sin as _sin
    _plt.ion()
    _fig = _ax = _robot_dot = _quiver = None

def _plot_setup(traj):
    global _fig, _ax, _robot_dot, _quiver
    _fig, _ax = _plt.subplots(figsize=(7, 7))
    _ax.plot(traj[:, 1], traj[:, 0], 'b-', lw=1.5, label='trajectory')
    _ax.plot(traj[0, 1], traj[0, 0], 'go', ms=8, label='start')
    _ax.plot(traj[-1, 1], traj[-1, 0], 'rs', ms=8, label='end')
    y_r0, x0 = float(traj[0, 1]), float(traj[0, 0])
    _robot_dot, = _ax.plot(y_r0, x0, 'ko', ms=10, label='robot', zorder=5)
    _quiver = _ax.quiver([y_r0], [x0], [0.0], [0.2],
                         scale=1.0, scale_units='xy', angles='xy',
                         color='red', width=0.005, zorder=6)
    _ax.set_xlabel('y  (right →, m)')
    _ax.set_ylabel('x  (forward ↑, m)')
    _ax.set_title('Pure Pursuit — live robot pose')
    _ax.legend(loc='upper right')
    _ax.set_aspect('equal')
    _ax.grid(True, alpha=0.3)
    _plt.tight_layout()
    _plt.pause(0.05)

def _plot_update(x, y, heading):
    # Kalman +y=left → plot +y_right=−y_kalman; heading CCW so arrow=(cos θ, −sin θ) in (y_right, x_fwd)
    global _quiver
    py, px = -y, x
    _robot_dot.set_data([py], [px])
    if _quiver is not None:
        _quiver.remove()
    L = 0.2
    _quiver = _ax.quiver([py], [px], [-L * _sin(heading)], [L * _cos(heading)],
                         scale=1.0, scale_units='xy', angles='xy',
                         color='red', width=0.005, zorder=6)
    _fig.canvas.flush_events()
    _fig.canvas.draw_idle()
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
            if _PLOT:
                _plot_setup(pursuit._traj)
            state = 1
        elif state == 1:
            path_follow.update()    # compute Pure Pursuit command and publish rc
            if _PLOT and _iter % 5 == 0:
                _plot_update(path_follow._x, path_follow._y, path_follow._heading)
            if pursuit.at_end():
                path_follow.pathControl(0)
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                state = 2
        elif state == 2:
            if abs(pose.velocity()) < 0.001:
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
        _plt.ioff()
        _plt.show()
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
