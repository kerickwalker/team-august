#!/usr/bin/env python3

#/***************************************************************************
#*   Copyright (C) 2025 by DTU
#*   jcan@dtu.dk
#*
#* The MIT License (MIT)  https://mit-license.org/
#***************************************************************************/

# Ball collection module.
# Call collect.collect() from the main mission loop when the ball is in sight.
# Returns True when the ball is captured, False if interrupted (service.stop).

import time as t
from uservice import service
from scam import cam
from sball import ball

class SCollect:

    # --- Servo parameters ---
    servo_id    = 1
    pos_open    = -900   # arm up — ready to scoop
    pos_closed  = 400   # arm down — ball captured in cage
    servo_speed = 100    # positions per second (0 = full speed)

    # --- Gate parameters ---
    gate_id     = 3
    gate_open   =  900   # gate open — allows ball to enter cage
    gate_closed = -900   # gate closed — retains ball when arm is raised

    # --- Drive parameters ---
    drive_speed     = 0.15   # forward speed during approach (m/s)
    turn_gain       = 1.0    # steeringError → turn rate (rad/s)
    align_threshold = 0.05   # max |steeringError| to consider ball centred
    commit_radius    = 50     # ball radius (px) that triggers centering step
    center_threshold = 0.05  # max |steeringError| to consider ball centered before capture run
    commit_speed     = 0.10  # forward speed during blind capture run (m/s)
    commit_duration  = 0.3   # seconds to drive straight before closing arm

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

    def collect(self):
        """Run the ball-collection state machine.
        Blocks until the ball is captured or service.stop is set.
        Returns True on success, False if stopped early."""

        ball.setup()
        state = 0

        while not service.stop:

            if state == 0:
                # Arm up and gate open, ready to scoop
                self._gate(self.gate_open)
                self._servo(self.pos_open)
                print("% SCollect:: state 0 — gate open, arm up, starting search")
                state = 1

            elif state == 1:
                # Get latest frame and run ball detection
                ok, img, _ = cam.getImage()
                if not ok:
                    t.sleep(0.02)
                    continue

                ball.detect(img)

                if self.verbose:
                    print(f"% [s1] detected={ball.detected}  "
                          f"r={ball.radius}px  "
                          f"clip={ball.clip_fraction:.2f}  "
                          f"err={ball.steeringError():+.3f}  "
                          f"commit={ball.detected and ball.radius >= self.commit_radius}")

                if not ball.detected:
                    # No ball in view — rotate slowly to scan
                    self._drive(0, 0.3)
                else:
                    err = ball.steeringError()
                    if abs(err) <= self.align_threshold:
                        # Ball centred enough — begin approach
                        print(f"% SCollect:: state 1 — ball found and aligned "
                              f"(err={err:+.2f}), advancing to approach")
                        state = 2
                    else:
                        # Steer toward ball while moving forward slowly
                        turn = -self.turn_gain * err
                        self._drive(self.drive_speed * 0.5, turn)

                t.sleep(0.02)

            elif state == 2:
                # Drive forward with continuous steering correction until commit radius
                ok, img, _ = cam.getImage()
                if not ok:
                    t.sleep(0.02)
                    continue

                ball.detect(img)

                if self.verbose:
                    print(f"% [s2] detected={ball.detected}  "
                          f"r={ball.radius}px  "
                          f"clip={ball.clip_fraction:.2f}  "
                          f"err={ball.steeringError():+.3f}  "
                          f"commit={ball.detected and ball.radius >= self.commit_radius}")

                if not ball.detected:
                    # Briefly lost the ball — hold course and keep looking
                    self._drive(self.drive_speed, 0)
                elif ball.radius >= self.commit_radius:
                    # Ball large enough — stop and center before capture run
                    self._drive(0, 0)
                    print(f"% SCollect:: state 2 — ball at commit radius "
                          f"(r={ball.radius}px >= {self.commit_radius}px), centering")
                    state = 3
                else:
                    # Keep steering toward ball as we approach
                    turn = -self.turn_gain * ball.steeringError()
                    self._drive(self.drive_speed, turn)

                t.sleep(0.02)

            elif state == 3:
                # Rotate in place until ball is centered before the capture run
                ok, img, _ = cam.getImage()
                if not ok:
                    t.sleep(0.02)
                    continue

                ball.detect(img)

                if self.verbose:
                    print(f"% [s3] detected={ball.detected}  "
                          f"r={ball.radius}px  "
                          f"err={ball.steeringError():+.3f}")

                if not ball.detected:
                    # Briefly lost ball — hold still
                    self._drive(0, 0)
                else:
                    err = ball.steeringError()
                    if abs(err) <= self.center_threshold:
                        self._drive(0, 0)
                        print(f"% SCollect:: state 3 — ball centered "
                              f"(err={err:+.3f}), starting capture run")
                        state = 4
                    else:
                        self._drive(0, -self.turn_gain * err)

                t.sleep(0.02)

            elif state == 4:
                # Drive straight toward ball for a fixed duration, then capture
                print(f"% SCollect:: state 4 — driving straight {self.commit_duration}s")
                self._drive(self.commit_speed, 0)
                t.sleep(self.commit_duration)
                self._drive(0, 0)
                state = 5

            elif state == 5:
                # Lower arm to capture ball
                self._servo(self.pos_closed)
                print("% SCollect:: state 5 — arm closed, ball captured")
                t.sleep(1.0)   # wait for arm to fully reach down position
                state = 6

            elif state == 6:
                # Close gate to retain ball, then raise arm so camera can see hole
                self._gate(self.gate_closed)
                print("% SCollect:: state 6 — gate closed")
                t.sleep(1.0)
                self._servo(self.pos_open)
                print("% SCollect:: state 6 — arm raised, ready to guide")
                t.sleep(0.5)
                return True

        # Stopped early
        self._drive(0, 0)
        return False


# singleton
collect = SCollect()
