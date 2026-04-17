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
    pos_closed  = 0      # arm down — ball captured in cage
    servo_speed = 300    # positions per second (0 = full speed)

    # --- Drive parameters ---
    drive_speed     = 0.15   # forward speed during approach (m/s)
    turn_gain       = 1.0    # steeringError → turn rate (rad/s)
    align_threshold = 0.15   # max |steeringError| to consider ball centred
    close_radius    = 40     # ball.isClose() threshold in pixels

    ##########################################################

    def _servo(self, position):
        service.send("robobot/cmd/T0",
                     f"servo {self.servo_id} {position} {self.servo_speed}")

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
                # Arm up so it can scoop; transition immediately
                self._servo(self.pos_open)
                print("% SCollect:: state 0 — arm open, starting search")
                state = 1

        # Stopped early
        self._drive(0, 0)
        return False


# singleton
collect = SCollect()
