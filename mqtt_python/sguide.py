#!/usr/bin/env python3

# Ball-to-hole guiding module.
# Call guide.guide() from the main mission loop after the ball is collected.
# Expects arm up and gate closed (as left by scollect.py).
# Returns True when the ball is deposited, False if interrupted (service.stop).

import math
import time as t
from uservice import service
from scam import cam
from shole import hole
from spose import pose

class SGuide:

    # --- Servo parameters (must match scollect.py) ---
    servo_id    = 1
    pos_open    = -900   # arm up — for searching
    pos_closed  = 400    # arm down — for deposit
    servo_speed = 300

    # --- Gate parameters (must match scollect.py) ---
    gate_id     = 3
    gate_open   =  200   # gate open — releases ball into hole
    gate_closed = -1024   # gate closed — retains ball during approach

    # --- Drive parameters ---
    drive_speed     = 0.12   # forward speed during approach (m/s)
    turn_gain       = 0.6    # steeringError → turn rate (rad/s)
    align_threshold = 0.15   # max |steeringError| to consider hole centred
    commit_radius    = 35     # hole radius (px) that triggers centering step
    center_threshold = 0.05  # max |steeringError| to consider hole centered before deposit run
    commit_speed     = 0.10  # forward speed during blind deposit run (m/s)
    commit_duration  = 1.0   # seconds to drive straight before depositing

    # --- Wiggle parameters ---
    wiggle_rate     = 0.5    # turn rate during wiggle (rad/s)
    wiggle_duration = 0.5    # seconds per half-swing (~15° at 0.5 rad/s)

    # --- Detection robustness ---
    confirm_frames  = 3    # consecutive frames required before any state transition
    steering_alpha  = 0.4  # EMA weight on new sample (0=frozen, 1=no smoothing)

    # --- Backup (odometry) parameters ---
    backup_drive_speed = 0.10  # forward speed during blind approach (m/s)
    backup_distance    = 0.285   # metres to drive before depositing
    backup_rotate_deg  = 38    # degrees to rotate in place after wiggle

    # --- Debug ---
    verbose = False           # set True in test scripts for per-tick logging

    ##########################################################

    def _servo(self, position):
        service.send("robobot/cmd/T0",
                     f"servo {self.servo_id} {position} {self.servo_speed}")

    def _gate(self, position):
        service.send("robobot/cmd/T0",
                     f"servo {self.gate_id} {position} {self.servo_speed}")

    def _drive(self, velocity, turn_rate):
        service.send("robobot/cmd/ti", f"rc {velocity:.3f} {turn_rate:.3f}")

    ##########################################################

    def guide(self):
        """Run the hole-guiding state machine.
        Blocks until the ball is deposited or service.stop is set.
        Returns True on success, False if stopped early."""

        hole.setup()
        state       = 0
        _confirm    = 0      # consecutive frames meeting the current transition condition
        _smooth_err = 0.0    # EMA of steeringError

        while not service.stop:

            if state == 0:
                # Arm is already up and gate closed from scollect — start searching
                print("% SGuide:: state 0 — starting hole search")
                _confirm    = 0
                _smooth_err = 0.0
                state = 1

            elif state == 1:
                # Search and align on the hole
                ok, img, _ = cam.getImage()
                if not ok:
                    t.sleep(0.02)
                    continue

                hole.detect(img)

                if hole.detected:
                    _smooth_err = (self.steering_alpha * hole.steeringError()
                                   + (1 - self.steering_alpha) * _smooth_err)
                else:
                    _smooth_err = 0.0

                if self.verbose:
                    print(f"% [s1] detected={hole.detected}  "
                          f"r={hole.radius}px  "
                          f"err_raw={hole.steeringError():+.3f}  "
                          f"err_smooth={_smooth_err:+.3f}  "
                          f"confirm={_confirm}/{self.confirm_frames}")

                if not hole.detected:
                    _confirm = 0
                    self._drive(0, 0.3)
                elif abs(_smooth_err) <= self.align_threshold:
                    _confirm += 1
                    if _confirm >= self.confirm_frames:
                        print(f"% SGuide:: state 1 — hole aligned "
                              f"(err={_smooth_err:+.2f}), advancing to approach")
                        _confirm = 0
                        state = 2
                else:
                    _confirm = 0
                    self._drive(self.drive_speed * 0.5, -self.turn_gain * _smooth_err)

                t.sleep(0.02)

            elif state == 2:
                # Drive toward hole with continuous steering correction until commit radius
                ok, img, _ = cam.getImage()
                if not ok:
                    t.sleep(0.02)
                    continue

                hole.detect(img)

                if hole.detected:
                    _smooth_err = (self.steering_alpha * hole.steeringError()
                                   + (1 - self.steering_alpha) * _smooth_err)

                if self.verbose:
                    print(f"% [s2] detected={hole.detected}  "
                          f"r={hole.radius}px  "
                          f"err_smooth={_smooth_err:+.3f}  "
                          f"confirm={_confirm}/{self.confirm_frames}")

                if not hole.detected:
                    _confirm = 0
                    self._drive(self.drive_speed, 0)
                elif hole.radius >= self.commit_radius:
                    _confirm += 1
                    if _confirm >= self.confirm_frames:
                        self._drive(0, 0)
                        print(f"% SGuide:: state 2 — hole at commit radius "
                              f"(r={hole.radius}px >= {self.commit_radius}px), centering")
                        _confirm = 0
                        state = 3
                else:
                    _confirm = 0
                    self._drive(self.drive_speed, -self.turn_gain * _smooth_err)

                t.sleep(0.02)

            elif state == 3:
                # Rotate in place until hole is centered before the deposit run
                ok, img, _ = cam.getImage()
                if not ok:
                    t.sleep(0.02)
                    continue

                hole.detect(img)

                if hole.detected:
                    _smooth_err = (self.steering_alpha * hole.steeringError()
                                   + (1 - self.steering_alpha) * _smooth_err)

                if self.verbose:
                    print(f"% [s3] detected={hole.detected}  "
                          f"r={hole.radius}px  "
                          f"err_smooth={_smooth_err:+.3f}  "
                          f"confirm={_confirm}/{self.confirm_frames}")

                if not hole.detected:
                    _confirm = 0
                    self._drive(0, 0)
                elif abs(_smooth_err) <= self.center_threshold:
                    _confirm += 1
                    if _confirm >= self.confirm_frames:
                        self._drive(0, 0)
                        print(f"% SGuide:: state 3 — hole centered "
                              f"(err={_smooth_err:+.3f}), starting deposit run")
                        _confirm = 0
                        state = 4
                else:
                    _confirm = 0
                    self._drive(0, -self.turn_gain * _smooth_err)

                t.sleep(0.02)

            elif state == 4:
                # Drive straight toward hole for a fixed duration
                print(f"% SGuide:: state 4 — driving straight {self.commit_duration}s")
                self._drive(self.commit_speed, 0)
                t.sleep(self.commit_duration)
                self._drive(0, 0)
                state = 5

            elif state == 5:
                # Lower arm over hole, then open gate to release ball
                self._servo(self.pos_closed)
                print("% SGuide:: state 5 — arm lowered over hole")
                t.sleep(1.0)
                self._gate(self.gate_open)
                print("% SGuide:: state 5 — gate open, ball released")
                t.sleep(0.5)
                state = 6

            elif state == 6:
                # Wiggle left and right to help seat the ball in the hole
                print("% SGuide:: state 6 — wiggling to seat ball")
                self._drive(0, -self.wiggle_rate)          # swing left
                t.sleep(self.wiggle_duration)
                self._drive(0,  self.wiggle_rate)          # swing right (through centre)
                t.sleep(2 * self.wiggle_duration)
                self._drive(0, -self.wiggle_rate)          # return to centre
                t.sleep(self.wiggle_duration)
                self._drive(0, 0)
                print("% SGuide:: ball deposited")
                return True

        # Stopped early
        self._drive(0, 0)
        return False


    def guide_backup(self):
        """Odometry-based fallback deposit: drive a fixed distance, lower arm,
        open gate, then wiggle to seat the ball.
        Returns True on success, False if stopped early."""

        print(f"% SGuide:: backup — driving {self.backup_distance:.2f} m to hole")
        pose.tripBreset()
        self._drive(self.backup_drive_speed, 0)

        while not service.stop:
            if pose.tripB >= self.backup_distance:
                break
            if self.verbose:
                print(f"% [backup] tripB={pose.tripB:.3f} / {self.backup_distance:.2f} m")
            t.sleep(0.02)

        self._drive(0, 0)

        if service.stop:
            return False

        # Rotate in place by backup_rotate_deg
        rotate_duration = math.radians(self.backup_rotate_deg) / self.wiggle_rate
        print(f"% SGuide:: backup — rotating {self.backup_rotate_deg}° "
              f"({rotate_duration:.2f}s at {self.wiggle_rate} rad/s)")
        self._drive(0, self.wiggle_rate)
        t.sleep(rotate_duration)
        self._drive(0, 0)

        # Lower arm over hole
        self._servo(self.pos_closed)
        print("% SGuide:: backup — arm lowered")
        t.sleep(1.0)

        # Release ball
        self._gate(self.gate_open)
        print("% SGuide:: backup — gate open, ball released")
        t.sleep(0.5)

        # Wiggle to seat ball
        print("% SGuide:: backup — wiggling to seat ball")
        self._drive(0, -self.wiggle_rate)
        t.sleep(self.wiggle_duration)
        self._drive(0,  self.wiggle_rate)
        t.sleep(2 * self.wiggle_duration)
        self._drive(0, -self.wiggle_rate)
        t.sleep(self.wiggle_duration)
        self._drive(0, 0)

        print("% SGuide:: backup — ball deposited")
        return True


# singleton
guide = SGuide()
