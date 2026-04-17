#!/usr/bin/env python3

#/***************************************************************************
#*   Copyright (C) 2025 by DTU
#*   jcan@dtu.dk
#*
#* The MIT License (MIT)  https://mit-license.org/
#***************************************************************************/

# Ball-to-hole guiding module.
# Call guide.guide() from the main mission loop after the ball is collected.
# The arm remains in the closed/down position throughout — the ball is already
# captured and is guided into the hole by driving over it.
# Returns True when the hole is reached, False if interrupted (service.stop).

import time as t
from uservice import service
from scam import cam
from shole import hole

class SGuide:

    # --- Servo parameters (must match scollect.py) ---
    servo_id    = 1
    pos_closed  = 0      # arm down — ball held in cage throughout
    servo_speed = 300

    # --- Drive parameters ---
    drive_speed     = 0.12   # forward speed during approach (m/s) — slightly
                             # slower than collection to keep ball in cage
    turn_gain       = 1.0    # steeringError → turn rate (rad/s)
    align_threshold = 0.15   # max |steeringError| to consider hole centred
    close_radius    = 60     # hole.isClose() threshold in pixels

    # --- Deposit nudge ---
    nudge_speed    = 0.08    # slow forward speed for final push into hole (m/s)
    nudge_duration = 0.8     # seconds to drive forward after isClose() triggers

    ##########################################################

    def _servo(self, position):
        service.send("robobot/cmd/T0",
                     f"servo {self.servo_id} {position} {self.servo_speed}")

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
                # Confirm arm is down with ball captured, then start searching
                self._servo(self.pos_closed)
                print("% SGuide:: state 0 — arm down, starting hole search")
                state = 1

            elif state == 1:
                # Search and align on the hole
                ok, img, _ = cam.getImage()
                if not ok:
                    t.sleep(0.02)
                    continue

                hole.detect(img)

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
                        # Steer toward hole while creeping forward
                        turn = self.turn_gain * err
                        self._drive(self.drive_speed * 0.5, turn)

                t.sleep(0.02)

            elif state == 2:
                # Drive toward hole with continuous steering correction
                ok, img, _ = cam.getImage()
                if not ok:
                    t.sleep(0.02)
                    continue

                hole.detect(img)

                if not hole.detected:
                    # Briefly lost hole — hold course
                    self._drive(self.drive_speed, 0)
                elif hole.isClose(self.close_radius):
                    self._drive(0, 0)
                    print(f"% SGuide:: state 2 — hole close (r={hole.radius}px), depositing")
                    state = 3
                else:
                    turn = self.turn_gain * hole.steeringError()
                    self._drive(self.drive_speed, turn)

                t.sleep(0.02)

            elif state == 3:
                # Nudge forward to push ball fully into hole, then stop
                print("% SGuide:: state 3 — nudging ball into hole")
                self._drive(self.nudge_speed, 0)
                t.sleep(self.nudge_duration)
                self._drive(0, 0)
                print("% SGuide:: ball deposited")
                return True

        # Stopped early
        self._drive(0, 0)
        return False


# singleton
guide = SGuide()
