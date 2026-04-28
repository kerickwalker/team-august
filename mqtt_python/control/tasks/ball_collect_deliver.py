#!/usr/bin/env python3
"""
ball_collect_deliver.py
=============================================================================
Pick up the balls from the dispenser bowl in zone A and drop them in zone C.

Task summary
------------
Five balls (1 blue, 1 red, 3 white) sit in a bowl at the ball dispenser.
The robot has a 3D-printed tray ("hazne") on a servo-driven arm and a
servo-driven slider gate at the bottom of the tray.

Sequence (state machine):
    0  approach A area      — encoder/Kalman drive to in front of the bowl
    1  align in front        — close the last few cm so the bowl ends up
                               under the tray when the arm comes down
    2  slider close          — make sure the gate is shut (no balls fall out)
    3  arm down              — lower the tray over the bowl
    4  shake                 — alternate yaw to tip the bowl over; balls
                               drop into the tray
    5  arm up                — raise the tray with the balls inside
    6  drive to C            — encoder approach to the sorting square
    7  ArUco fine-align      — markers 14/15 give the final stop
    8  slider open           — drop balls into zone C
    9  reset                 — close slider, raise arm, back off
    99 done

All numeric constants that depend on physical setup (servo PWM values,
exact bowl approach point, shake amplitude) are tagged FIELD_TUNE so
they are easy to find and adjust on the day.

Run from mqtt_python/:
    python3 -m control.tasks.ball_collect_deliver [robot-ip]
"""

from __future__ import annotations
import math
import sys
import time as t

from util.uservice import service
from sensors.spose import pose
from sensors.scam import cam
from perception.camera.saruco import aruco_detector


# ---------------------------------------------------------------------------
# Servo configuration (PWM positions on Teensy)
# ---------------------------------------------------------------------------
SERVO_ARM         = 1     # lowers/raises the tray ("hazne")
# SERVO_SLIDER    = 3     # DISABLED — slider mechanism broken, operator handles it manually

ARM_DOWN_PWM      =  500  # tray fully lowered, sits over the bowl  (calibrated saha)
ARM_UP_PWM        = -700  # tray raised for travel                  (calibrated saha)
# SLIDER_OPEN_PWM   =  900  # gate open  → balls fall out  (DISABLED — manual)
# SLIDER_CLOSED_PWM =    0  # gate closed → balls retained (DISABLED — manual)
SERVO_SPEED       =  100  # slow speed gives smoother motion


# ---------------------------------------------------------------------------
# Field coordinates (from field_map_2026 / field_objects)
# ---------------------------------------------------------------------------
# Ball dispenser bowl centre (FIELD: BallDispenser.position)
BOWL_X = 1.00
BOWL_Y = 2.00

# Approach point in front of the bowl. The robot stops here facing the bowl,
# arm still up. From here we drive straight a small distance so the bowl
# ends up between the body and the tray when the arm comes down.
# FIELD_TUNE: pick a heading that avoids walls/fixtures around the dispenser.
BOWL_APPROACH_X       = 1.00
BOWL_APPROACH_Y       = 1.55       # 0.45 m south of the bowl
BOWL_APPROACH_YAW     = math.radians(90.0)   # facing +Y (toward the bowl)
BOWL_FINAL_STEP_M     = 0.20       # FIELD_TUNE: how far to creep in once aligned

# Zone C (SortingCenter zone "C": centre + (0, +0.30) since zone_size=0.30)
ZONE_C_X = 1.80
ZONE_C_Y = 1.40

# Zone C ArUco markers (SortingCenter.build_aruco: 14 = top-right, 15 = top-left)
ZONE_C_ARUCO_IDS = {14, 15}


# ---------------------------------------------------------------------------
# Navigation gains
# ---------------------------------------------------------------------------
# Encoder-frame approach to a goal pose
KV         = 0.35
KW         = 1.20
MAX_V      = 0.25       # m/s
MAX_W      = 1.00       # rad/s
GOAL_TOL_M = 0.08       # m — encoder phase considers itself "at the goal"
NAV_TIMEOUT_S = 30.0

# Hand-off distance: stop encoder approach this far from C, then look for ArUco.
ARUCO_HANDOFF_DIST_M = 0.50

# ArUco fine-align phase
ARUCO_TARGET_RANGE_M = 0.25                    # FIELD_TUNE: stop this far from marker face
ARUCO_RANGE_TOL_M    = 0.05
ARUCO_BEAR_TOL_RAD   = math.radians(5.0)
ARUCO_KV             = 0.20
ARUCO_KW             = 1.00
ARUCO_SCAN_W         = 0.40                    # rad/s — scan rotation when no marker visible
ARUCO_SCAN_TIMEOUT_S = 8.0


# ---------------------------------------------------------------------------
# Shake reflex (tip the bowl over so balls fall into the tray)
# ---------------------------------------------------------------------------
# FIELD_TUNE: amplitude/duration depends on weight balance and bowl friction.
SHAKE_CYCLES        = 4
SHAKE_OMEGA_RAD_S   = 0.6
SHAKE_HALF_PERIOD_S = 0.18


class BallCollectDeliverTask:
    """State-machine task: bowl pickup at A, drop at C."""

    # ---- low-level helpers --------------------------------------------------
    def _servo(self, sid: int, pwm: int, speed: int = SERVO_SPEED) -> None:
        service.send("robobot/cmd/T0", f"servo {sid} {pwm} {speed}")

    def _drive(self, v: float, w: float) -> None:
        service.send("robobot/cmd/ti", f"rc {v:.3f} {w:.3f}")

    def _stop(self) -> None:
        self._drive(0.0, 0.0)

    @staticmethod
    def _wrap(angle: float) -> float:
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    @staticmethod
    def _pose_xyh():
        # SPose stores the encoder-fused pose in `pose.pose[0..2]`. There are
        # no x()/y()/h() accessors — direct list indexing is the convention.
        p = pose.pose
        return float(p[0]), float(p[1]), float(p[2])

    # ---- shake reflex -------------------------------------------------------
    def _shake(self) -> None:
        for _ in range(SHAKE_CYCLES):
            self._drive(0.0,  SHAKE_OMEGA_RAD_S)
            t.sleep(SHAKE_HALF_PERIOD_S)
            self._drive(0.0, -SHAKE_OMEGA_RAD_S)
            t.sleep(SHAKE_HALF_PERIOD_S)
        self._stop()
        t.sleep(0.3)

    # ---- encoder approach to (tx, ty) --------------------------------------
    def _drive_to_xy(self, tx: float, ty: float, stop_dist: float) -> bool:
        """Drive toward (tx, ty); stop when within stop_dist. Returns True if reached."""
        t0 = t.time()
        while not service.stop:
            if t.time() - t0 > NAV_TIMEOUT_S:
                print(f"% BCD:: nav timeout at ({tx:.2f},{ty:.2f})")
                self._stop()
                return False

            x, y, h = self._pose_xyh()
            dx, dy  = tx - x, ty - y
            dist    = math.hypot(dx, dy)

            if dist <= stop_dist:
                self._stop()
                return True

            bearing   = math.atan2(dy, dx)
            angle_err = self._wrap(bearing - h)

            v = min(KV * dist, MAX_V)
            w = max(-MAX_W, min(MAX_W, KW * angle_err))
            if abs(angle_err) > math.radians(30.0):
                v *= 0.4

            self._drive(v, w)
            t.sleep(0.05)

        self._stop()
        return False

    # ---- turn in place to a target heading ---------------------------------
    def _turn_to_yaw(self, target_yaw: float, tol_rad: float = math.radians(3.0)) -> bool:
        t0 = t.time()
        while not service.stop:
            if t.time() - t0 > NAV_TIMEOUT_S:
                self._stop()
                return False
            _, _, h = self._pose_xyh()
            err = self._wrap(target_yaw - h)
            if abs(err) <= tol_rad:
                self._stop()
                return True
            w = max(-MAX_W, min(MAX_W, 1.6 * err))
            self._drive(0.0, w)
            t.sleep(0.03)
        self._stop()
        return False

    # ---- creep forward a fixed distance using tripB -----------------------
    def _creep_forward(self, distance_m: float, speed: float = 0.10) -> None:
        pose.tripBreset()
        self._drive(speed, 0.0)
        while not service.stop and pose.tripB < distance_m:
            t.sleep(0.03)
        self._stop()
        t.sleep(0.2)

    # ---- ArUco fine-align (zone C) -----------------------------------------
    def _aruco_align_to_zone_c(self) -> bool:
        """Drive to range≈ARUCO_TARGET_RANGE_M, bearing≈0 on the closest C marker."""
        scan_start = None

        while not service.stop:
            ok, frame, _ = cam.getImage()
            if not ok:
                t.sleep(0.03)
                continue

            detections = aruco_detector.process(frame)
            zone_c = [d for d in detections if d['id'] in ZONE_C_ARUCO_IDS]

            if not zone_c:
                if scan_start is None:
                    scan_start = t.time()
                    print("% BCD:: scanning for zone-C ArUco (14/15)…")
                elif t.time() - scan_start > ARUCO_SCAN_TIMEOUT_S:
                    print("% BCD:: ArUco scan timed out — proceeding without fine align")
                    self._stop()
                    return False
                self._drive(0.0, ARUCO_SCAN_W)
                t.sleep(0.03)
                continue

            scan_start = None
            det = min(zone_c, key=lambda d: d['range'])
            rng     = det['range']
            bearing = det['bearing']
            range_err = rng - ARUCO_TARGET_RANGE_M

            if abs(range_err) < ARUCO_RANGE_TOL_M and abs(bearing) < ARUCO_BEAR_TOL_RAD:
                print(f"% BCD:: aligned id={det['id']}  range={rng:.2f}  "
                      f"bearing={math.degrees(bearing):+.1f}°")
                self._stop()
                return True

            v = max(-MAX_V, min(MAX_V, ARUCO_KV * range_err))
            w = max(-MAX_W, min(MAX_W, ARUCO_KW * bearing))
            self._drive(v, w)
            t.sleep(0.03)

        self._stop()
        return False

    # ---- main state machine -------------------------------------------------
    def run(self) -> None:
        # Start with the arm raised. Slider is held closed by the operator.
        self._servo(SERVO_ARM, ARM_UP_PWM)
        t.sleep(0.5)
        print("% BCD:: ⚠ Slider must be CLOSED by hand before starting")

        state = 0
        while not service.stop:

            if state == 0:
                print(f"% BCD:: [0] approach A → ({BOWL_APPROACH_X:.2f}, {BOWL_APPROACH_Y:.2f})")
                self._drive_to_xy(BOWL_APPROACH_X, BOWL_APPROACH_Y, GOAL_TOL_M)
                self._turn_to_yaw(BOWL_APPROACH_YAW)
                state = 1

            elif state == 1:
                print(f"% BCD:: [1] creep forward {BOWL_FINAL_STEP_M:.2f} m onto bowl")
                self._creep_forward(BOWL_FINAL_STEP_M)
                state = 2

            elif state == 2:
                # Slider is held closed manually by the operator — nothing to do here.
                print("% BCD:: [2] slider already closed (manual)")
                state = 3

            elif state == 3:
                print("% BCD:: [3] arm down (tray over bowl)")
                self._servo(SERVO_ARM, ARM_DOWN_PWM)
                t.sleep(1.5)
                state = 4

            elif state == 4:
                print("% BCD:: [4] shake (tip bowl, balls drop into tray)")
                self._shake()
                state = 5

            elif state == 5:
                print("% BCD:: [5] arm up (carry balls)")
                self._servo(SERVO_ARM, ARM_UP_PWM)
                t.sleep(1.2)
                state = 6

            elif state == 6:
                print(f"% BCD:: [6] encoder drive to zone C "
                      f"(stop {ARUCO_HANDOFF_DIST_M:.2f} m short)")
                self._drive_to_xy(ZONE_C_X, ZONE_C_Y, ARUCO_HANDOFF_DIST_M)
                state = 7

            elif state == 7:
                print("% BCD:: [7] ArUco fine-align to zone C")
                self._aruco_align_to_zone_c()  # proceed even if scan fails
                state = 8

            elif state == 8:
                # Slider is broken — operator opens it by hand to drop the balls,
                # then closes it again. Robot just waits.
                print("% BCD:: [8] >>> MANUAL: open slider by hand, drop balls, "
                      "close slider, then press ENTER <<<")
                try:
                    input()
                except EOFError:
                    t.sleep(8.0)  # fallback if no stdin
                state = 9

            elif state == 9:
                print("% BCD:: [9] reset — arm up, back off")
                self._servo(SERVO_ARM, ARM_UP_PWM)
                t.sleep(0.8)
                # Small reverse so we don't drag the tray over the dropped balls.
                self._drive(-0.10, 0.0)
                t.sleep(0.6)
                self._stop()
                state = 99

            elif state == 99:
                print("% BCD:: done")
                break


task = BallCollectDeliverTask()


if __name__ == "__main__":
    host = sys.argv[1] if len(sys.argv) > 1 else "10.197.219.117"
    service.setup(host)
    if service.connected:
        try:
            task.run()
        finally:
            task._stop()
    service.terminate()
