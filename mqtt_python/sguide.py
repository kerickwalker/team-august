#!/usr/bin/env python3

#/***************************************************************************
#*   Copyright (C) 2025 by DTU
#*   jcan@dtu.dk
#*
#* The MIT License (MIT)  https://mit-license.org/
#***************************************************************************/

# Ball-to-hole guiding module.
# Call guide.guide() from the main mission loop after the ball is collected.
# Expects arm up and gate closed (as left by scollect.py).
# Returns True when the ball is deposited, False if interrupted (service.stop).

import time as t
from uservice import service
from scam import cam
from shole import hole

class SGuide:

    # --- Servo parameters (must match scollect.py) ---
    servo_id    = 1
    pos_open    = -900   # arm up — for searching
    pos_closed  = 400    # arm down — for deposit
    servo_speed = 300

    # --- Gate parameters (must match scollect.py) ---
    gate_id     = 3
    gate_open   =  900   # gate open — releases ball into hole
    gate_closed = -900   # gate closed — retains ball during approach

    # --- Drive parameters ---
    drive_speed     = 0.12   # forward speed during approach (m/s)
    turn_gain       = 1.0    # steeringError → turn rate (rad/s)
    align_threshold = 0.15   # max |steeringError| to consider hole centred
    commit_radius    = 35     # hole radius (px) that triggers centering step
    center_threshold = 0.05  # max |steeringError| to consider hole centered before deposit run
    commit_speed     = 0.10  # forward speed during blind deposit run (m/s)
    commit_duration  = 1.0   # seconds to drive straight before depositing

    # --- Wiggle parameters ---
    wiggle_rate     = 0.5    # turn rate during wiggle (rad/s)
    wiggle_duration = 0.5    # seconds per half-swing (~15° at 0.5 rad/s)

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
        state = 0

        while not service.stop:

            if state == 0:
                # Arm is already up and gate closed from scollect — start searching
                print("% SGuide:: state 0 — starting hole search")
                state = 1

            elif state == 1:
                # Search and align on the hole
                ok, img, _ = cam.getImage()
                if not ok:
                    t.sleep(0.02)
                    continue

                hole.detect(img)

                if self.verbose:
                    print(f"% [s1] detected={hole.detected}  "
                          f"r={hole.radius}px  "
                          f"err={hole.steeringError():+.3f}  "
                          f"commit={hole.detected and hole.radius >= self.commit_radius}")

                if not hole.detected:
                    # No hole in view — rotate slowly to scan
                    self._drive(0, 0.3)
                else:
                    err = hole.steeringError()
                    if abs(err) <= self.align_threshold:
                        print(f"% SGuide:: state 1 — hole found and aligned "
                              f"(err={err:+.2f}), advancing to approach")
                        state = 2
                    else:
                        turn = -self.turn_gain * err
                        self._drive(self.drive_speed * 0.5, turn)

                t.sleep(0.02)

            elif state == 2:
                # Drive toward hole with continuous steering correction until commit radius
                ok, img, _ = cam.getImage()
                if not ok:
                    t.sleep(0.02)
                    continue

                hole.detect(img)

                if self.verbose:
                    print(f"% [s2] detected={hole.detected}  "
                          f"r={hole.radius}px  "
                          f"err={hole.steeringError():+.3f}  "
                          f"commit={hole.detected and hole.radius >= self.commit_radius}")

                if not hole.detected:
                    # Briefly lost hole — hold course
                    self._drive(self.drive_speed, 0)
                elif hole.radius >= self.commit_radius:
                    self._drive(0, 0)
                    print(f"% SGuide:: state 2 — hole at commit radius "
                          f"(r={hole.radius}px >= {self.commit_radius}px), centering")
                    state = 3
                else:
                    turn = -self.turn_gain * hole.steeringError()
                    self._drive(self.drive_speed, turn)

                t.sleep(0.02)

            elif state == 3:
                # Rotate in place until hole is centered before the deposit run
                ok, img, _ = cam.getImage()
                if not ok:
                    t.sleep(0.02)
                    continue

                hole.detect(img)

                if self.verbose:
                    print(f"% [s3] detected={hole.detected}  "
                          f"r={hole.radius}px  "
                          f"err={hole.steeringError():+.3f}")

                if not hole.detected:
                    # Briefly lost hole — hold still
                    self._drive(0, 0)
                else:
                    err = hole.steeringError()
                    if abs(err) <= self.center_threshold:
                        self._drive(0, 0)
                        print(f"% SGuide:: state 3 — hole centered "
                              f"(err={err:+.3f}), starting deposit run")
                        state = 4
                    else:
                        self._drive(0, -self.turn_gain * err)

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


# singleton
guide = SGuide()
