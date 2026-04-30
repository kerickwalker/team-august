#/***************************************************************************
#*   Copyright (C) 2024 by DTU
#*   jcan@dtu.dk
#*
#*
#* The MIT License (MIT)  https://mit-license.org/
#*
#* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
#* and associated documentation files (the “Software”), to deal in the Software without restriction,
#* including without limitation the rights to use, copy, modify, merge, publish, distribute,
#* sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
#* is furnished to do so, subject to the following conditions:
#*
#* The above copyright notice and this permission notice shall be included in all copies
#* or substantial portions of the Software.
#*
#* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
#* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
#* PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
#* FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
#* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
#* THE SOFTWARE. */


from datetime import *
import time as t
from threading import Thread
import cv2 as cv
from ulog import flog

class SEdge:
    # ============= TUNING & PRINT OPTIONS (edit these) =============
    # Forward speed when following line (m/s). Mission scripts pass this to lineControl(); change here to tune.
    defaultLineVelocity = 0.30  # m/s
    # Line detection (livn 0–1000 scale)
    lineValidThreshold = 500   # each sensor above this → "on line"; line valid when peak >= this
    crossingThreshold = 700    # legacy: was used for average-based crossing; crossing now uses crossingMinSensors
    crossingMinSensors = 4     # T-crossing: this many or more sensors above lineValidThreshold counts as a crossing
    # Y-crossing detection: a crossing can also be declared with FEWER active sensors
    # when the line splits into two narrow branches with a dark gap between them.
    # All three Y conditions must hold:
    #   sensorsAboveCount >= yMinSensors
    #   (rightmostAboveIndex - leftmostAboveIndex) >= yMinSpan
    #   at least one of the centre sensor indices in yMidIndices is NOT above threshold (centre gap)
    yMinSensors = 3            # Y-crossing: minimum sensors above threshold
    yMinSpan    = 4            # Y-crossing: minimum span between leftmost and rightmost active sensor
    yMidIndices = (3, 4)       # centre sensor indices; if any are inactive while above conditions hold → Y
    low = 400                  # unused; was lineValidThreshold-100 for dark threshold; weighted center uses min(edge_n) as floor
    # Pattern-based line position. Bits are sensor 0..7 from left to right,
    # where 1 means edge_n[i] >= lineValidThreshold. Negative center = line left.
    lineUsePatternCenter = True
    linePatternFallbackCenters = (-3.4, -2.3, -1.1, -0.25, 0.25, 1.1, 2.3, 3.4)
    linePatternCenterTable = [None] * 256
    linePatternCenterTable[0b10000000] = -3.4
    linePatternCenterTable[0b01000000] = -2.3
    linePatternCenterTable[0b00100000] = -1.1
    linePatternCenterTable[0b00010000] = -0.25
    linePatternCenterTable[0b00001000] = 0.25
    linePatternCenterTable[0b00000100] = 1.1
    linePatternCenterTable[0b00000010] = 2.3
    linePatternCenterTable[0b00000001] = 3.4
    linePatternCenterTable[0b11000000] = -3.0
    linePatternCenterTable[0b01100000] = -1.8
    linePatternCenterTable[0b00110000] = -0.8
    linePatternCenterTable[0b00011000] = 0.0
    linePatternCenterTable[0b00001100] = 0.8
    linePatternCenterTable[0b00000110] = 1.8
    linePatternCenterTable[0b00000011] = 3.0
    linePatternCenterTable[0b11100000] = -2.6
    linePatternCenterTable[0b01110000] = -1.5
    linePatternCenterTable[0b00111000] = -0.5
    linePatternCenterTable[0b00011100] = 0.5
    linePatternCenterTable[0b00001110] = 1.5
    linePatternCenterTable[0b00000111] = 2.6
    linePatternCenterTable[0b11110000] = -2.0
    linePatternCenterTable[0b01111000] = -0.8
    linePatternCenterTable[0b00111100] = 0.0
    linePatternCenterTable[0b00011110] = 0.8
    linePatternCenterTable[0b00001111] = 2.0
    linePatternCenterTable[0b11111111] = 0.0
    linePatternCenterTable[0b01111110] = 0.0
    # Active line-follow tuning. Start with PI-only so straight-line behavior
    # can be tuned before adding separate turn/edge behavior.
    lineKp = 0.25
    lineKi = 0.005
    lineKd = 0.0
    lineIntegralLimit = 2.0    # clamp integral to ±this (error·s) to limit windup
    lineDerivativeTermLimit = 0.25  # max absolute D contribution to turn-rate output
    lineMinTurnError = 1.10     # if abs(error) is this large, enforce a minimum turn
    lineMinTurnRate = 0.55      # minimum abs(turn rate) while the visible line is far off-center
    lineRecentValidCnt = 5      # below this confidence, recovery may start
    lineNoReverseError = 0.25   # above this abs(error), D may damp but not reverse PI steering
    lineTurnSlewRate = 4.0      # max change in commanded turn rate (rad/s per second); 0 disables
    lineTurnLimit = 0.90        # normal line-following abs(turn rate) cap, below recovery turn cap
    lineVelocityMin = 0.10      # slow to at least this speed when line error is large
    lineSlowdownError = 1.00    # abs(error) where adaptive slowdown reaches lineVelocityMin
    lineControlPeriod = 0.050   # motor command update period (50 ms)
    # Sample period used by the PID (s). T0/livn is published every ~10 ms
    # ("sub livn 3"), so the controller assumes a fixed dt to keep Kd and Ki
    # tuning independent of MQTT arrival jitter / EWMA warm-up. The measured
    # interval (edge_nInterval) is kept only for diagnostics + drift warning.
    TS_NOMINAL = 0.010
    TS_DRIFT_WARN_FRAC = 0.30  # one-shot warn if measured Ts drifts >this fraction from nominal
    # Output saturation (turn rate rad/s)
    # Keep ≤ speed / (half_track_width) so inner wheel never reverses.
    # At 0.25 m/s, ~0.13 m track: max safe ≈ 1.9 rad/s → use 1.5 for margin.
    lineYMax = 1.0
    lineYMin = -1.0
    # Low-pass smoothing on the turn-rate output (0 = no filter, 1 = frozen)
    # Absorbs sensor noise between 30 ms livn updates without adding much lag.
    lineOutputAlpha = 0.25  # new = alpha*raw + (1-alpha)*prev; lower = smoother
    # EWMA on tracking error for the D-term only (does not smooth P or I).
    # e_filt = beta * e_filt + (1 - beta) * e ; beta in [0, 1]. beta=0 → e_filt = e
    # each step (classic derivative on raw error). beta>0 low-pass filters e before
    # differencing, which cuts Kd amplification of single-sample sensor noise; see
    # mqtt_python/PID_explanation.md and --beta on mission scripts.
    lineDerivativeBeta = 0.0
    # Named PID parameter sets — select via lineControl(params="slow"|"normal")
    # "slow" starts as a copy of "normal"; tune independently on the robot.
    PARAM_SETS = {
        "normal": dict(lineKp=0.25, lineKi=0.005, lineKd=0.0, lineVelocity=0.30),
        "slow":   dict(lineKp=0.20, lineKi=0.005, lineKd=0.0, lineVelocity=0.15),
    }
    lineReacquireSettleTime = 3.0     # seconds to use slow profile after line is found again
    lineReacquireSettleParams = "slow"
    # Recovery when line lost (A1/A4: turn toward last line side)
    recoveryTurnRate = 1.0   # rad/s when turning to find line during recovery
    recoveryVelocity = 0.0   # m/s forward during recovery (0 = turn in place; small value = creep forward while turning)
    recoveryTurnTime = 0.50       # seconds to turn in place before trying a forward step
    recoveryForwardDistance = 0.10 # meters to move forward while searching after turn phase
    recoveryForwardVelocity = 0.10 # m/s during forward search step
    recoveryStraightCenter = 0.8      # if last valid center was this close, assume straight-line dropout
    recoveryStraightTime = 0.35       # seconds to creep straight before falling back to turn recovery
    recoveryStraightVelocity = 0.08   # m/s during straight-line dropout recovery
    recovery_timeout_s = 5.0 # mission stops after this many seconds without line (recovery runs until then)
    # Legacy lead (unused when using PID; for optional re-enable later)
    lineTauZ = 0.8
    lineTauP = 0.25
    # --- Follow-line block print (gated by --test/--silent). Print + flog next to each other: ---
    print_follow_line_block = True  # set True to re-enable; False = no print in hot path (test responsiveness)
    follow_line_print_every_n = 5   # diagnostics: denser prints for line-loss debugging
    flog_write_every_n = 1         # flog.write() every Nth livn update (appends line/sensor log line to file)
    
    print_follow_line_fields = (
        'livn', 'pattern', 'high', 'valid', 'validCnt', 'state', 'aboveCnt',
        'crossingCnt', 'leftMost', 'rightMost', 'center', 'wCenter', 'e', 'p', 'i', 'd', 'u',
        'y', 'dGuard', 'minTurn', 'settle', 'recMode', 'recStraight', 'rc'
    )

    # Available fields (copy into tuple above; order = order on screen; single line):
    #   livn, pattern, avg, high, valid, validCnt, center, wCenter, state, aboveCnt,
    #   crossingCnt, leftMost, rightMost, e, p, i, d, u, y, dGuard, minTurn, settle, recMode, recStraight, rc, lastLineSide
    #   e = error (ref - center); u = p+i+d; y = clamped turn rate.
    # ============= end tuning & print options =============

    # raw AD values
    edge = [0, 0, 0 , 0, 0, 0, 0, 0]
    edgeUpdCnt = 0
    edgeTime = datetime.now()
    edgeInterval = 0
    # normalizing white values
    edge_n_w = [0, 0, 0 , 0, 0, 0, 0, 0]
    edge_n_wUpdCnt = 0
    edge_n_wTime = datetime.now()
    # normalized after white calibration
    edge_n = [0, 0, 0 , 0, 0, 0, 0, 0]
    edge_nUpdCnt = 0
    edge_nTime = datetime.now()
    edge_nInterval = 0
    edgeIntervalSetup = 0.1
    # line detection values
    posLeft = 0.0
    posRight = 0.0
    lineCenterWeighted = 0.0  # weighted center of mass from analog values, -3.5..3.5
    lineCenterPattern = 0.0   # threshold-pattern center from linePatternCenterTable
    refPosition = 0.0  # setpoint: 0 = line center under sensor
    lineValid = False
    lineValidCnt = 0 # a value up to 20 for most confident line detect
    crossingLine = False
    crossingLineCnt = 0  # a value up to 20 for most confident crossing line
    average = 0
    high = 0   # highest reflectivity in latest sample
    # Per-sensor above-threshold and array state (for crossing/behavior; tunables: lineValidThreshold, crossingMinSensors above)
    sensorAboveThreshold = [False] * 8   # True if edge_n[i] >= lineValidThreshold
    sensorsAboveCount = 0                # number of sensors above threshold (0..8)
    leftmostAboveIndex = None           # first sensor index 0..7 above threshold, or None
    rightmostAboveIndex = None          # last sensor index 0..7 above threshold, or None
    linePattern = 0                     # 8-bit sensor mask; bit 7 = sensor 0, bit 0 = sensor 7
    lineState = "no_line"               # "no_line" | "line" | "crossing" (set from sensorsAboveCount vs crossingMinSensors)
    mission_crossing_count = 0          # set by mission (driveToLine) so print can show crossing counter
    #
    topicLip = ""
    sendCalibRequest = False
    calibCollecting = False   # True while rolling forward/back to collect white levels
    calibWhiteMax = [0] * 8   # per-sensor max raw value during rolling calib
    #
    # follow line controller
    lineCtrl = False # private
    tauP2pT = 1.0
    tauP2mT = 0.0
    tauZ2pT = 1.0
    tauZ2mT = 0.0
    # PID state
    lineIntegral = 0.0
    lineE_prev = None  # previous raw error (updated every sample; used for diagnostics consistency)
    lineE_filt = 0.0   # EWMA of e for D-term when lineDerivativeBeta > 0
    lineE_filt_prev = None  # previous e_filt for derivative; None resets D spike
    lineE1 = 0.0
    lineY1 = 0.0
    lineY = 0.0  # control output (rad/s), clamped and smoothed
    lineLastControlTime = 0.0
    lineLostStartTime = 0.0
    lineRequestedParams = "normal"
    lineReacquireSettleUntil = 0.0
    lineSettleActive = False
    lineSuppressInitialSettle = False
    lineWasValid = False
    lastValidLineCenter = 0.0
    recoveryMode = "none"
    recoveryPhaseStartTime = 0.0
    recoveryForwardStartTrip = None
    recoveryForwardStartTime = 0.0
    recoveryStraightActive = False
    lineDerivativeGuardActive = False
    lineMinTurnActive = False
    lineTurnSlewActive = False
    lineVelocityAdjusted = False
    # memory for recovery (A4: remember last side)
    lastLineSide = 0   # -1 = line was left, +1 = line was right, 0 = unknown; recovery turns this way
    # management
    topicCmdT0 = ""
    lostLineCnt = 0
    u = 0 # turn rate control signal


    ##########################################################

    def setup(self):
      from uservice import service
      sendBlack = False
      loops = 0
      # turn line sensor on (command 'lip 1')
      if not service.is_quiet():
        print("% Edge (sedge.py):: turns on line sensor")
      self.topicCmdT0 = "robobot/cmd/T0"
      service.send(self.topicCmdT0, "lip 1")
      # request fast update (every 10 ms); raw every 20 ms for combined line+followLine prints
      service.send(self.topicCmdT0,"sub livn 3")
      service.send(self.topicCmdT0,"sub liv 3")
      # request data
      while not service.stop:
        t.sleep(0.02)
        # white calibrate requested
        if service.args.white:
          if not sendBlack:
            # make sure black level is black
            topic = self.topicCmdT0
            param = "litb 0 0 0 0 0 0 0 0"
            sendBlack = service.send(topic, param)
          elif self.edgeUpdCnt < 3:
            # request raw AD reflectivity
            service.send(self.topicCmdT0,"livi")
            pass
          elif not self.sendCalibRequest:
            # Rolling calibration: drive slowly over the line, collect max raw per sensor, set white from that
            if not service.is_quiet():
              print("% Edge (sedge.py):: Place robot on line; will roll forward then back to capture white level")
            self.calibWhiteMax = [0] * 8
            self.calibCollecting = True
            # forward
            service.send("robobot/cmd/ti", "rc 0.12 0.0")
            t.sleep(2.5)
            # back
            service.send("robobot/cmd/ti", "rc -0.12 0.0")
            t.sleep(2.5)
            service.send("robobot/cmd/ti", "rc 0.0 0.0")
            self.calibCollecting = False
            # ensure no zero (firmware needs white > black); use at least 1
            w = [max(1, self.calibWhiteMax[i]) for i in range(8)]
            service.send(self.topicCmdT0, "litw " + " ".join(str(v) for v in w))
            t.sleep(0.1)
            service.send(self.topicCmdT0, "eew")
            self.sendCalibRequest = True
            if not service.is_quiet():
              print(f"% Edge (sedge.py):: white set from rolling calib: {w}")
            service.args.white = False
            service.stop = True
          else:
            t.sleep(0.25)
            service.args.white = False
            if not service.is_quiet():
              print(f"% Edge (sedge.py):: calibration done, terminates.")
            service.stop = True
        elif self.edge_n_wUpdCnt == 0:
          # get calibrated white value
          service.send(self.topicCmdT0,"liwi")
          pass
        elif self.edge_nUpdCnt == 0:
          # wait for line sensor data
          pass
        else:
          if not service.is_quiet():
            print(f"% Edge (sedge.py):: got data stream; after {loops} loops")
          break
        loops += 1
        if loops > 30:
          if not service.is_quiet():
            print(f"% Edge (sedge.py):: got no data after {loops} (continues edge_n_wUpdCnt={self.edge_n_wUpdCnt}, edgeUpdCnt={self.edgeUpdCnt}, edge_nUpdCnt={self.edge_nUpdCnt})")
          break
      pass

    ##########################################################

    def print(self):
      from uservice import service
      if not service.is_quiet():
        print("% Edge (sedge.py):: " + str(self.edgeTime - service.startTime) +
            f" ({self.edge[0]}, " +
            f"{self.edge[1]}, " +
            f"{self.edge[2]}, " +
            f"{self.edge[3]}, " +
            f"{self.edge[4]}, " +
            f"{self.edge[5]}, " +
            f"{self.edge[6]}, " +
            f"{self.edge[7]})" +
            f" {self.edgeInterval:.2f} ms " +
            str(self.edgeUpdCnt))
    def printn(self):
      from uservice import service
      if not service.is_quiet():
        print("% Edge (sedge.py):: normalized " + str(self.edge_nTime - service.startTime) +
              f" ({self.edge_n[0]}, " +
              f"{self.edge_n[1]}, " +
              f"{self.edge_n[2]}, " +
              f"{self.edge_n[3]}, " +
              f"{self.edge_n[4]}, " +
              f"{self.edge_n[5]}, " +
              f"{self.edge_n[6]}, " +
              f"{self.edge_n[7]})" +
              f" {self.edge_nInterval:.2f} ms " +
              str(self.edge_nUpdCnt))
    def printnw(self):
      from uservice import service
      if not service.is_quiet():
        print("% Edge (sedge.py):: white level " + str(self.edge_n_wTime) +
            f" ({self.edge_n_w[0]}, " +
            f"{self.edge_n_w[1]}, " +
            f"{self.edge_n_w[2]}, " +
            f"{self.edge_n_w[3]}, " +
            f"{self.edge_n_w[4]}, " +
            f"{self.edge_n_w[5]}, " +
            f"{self.edge_n_w[6]}, " +
            f"{self.edge_n_w[7]}) " +
            str(self.edge_n_wUpdCnt))

    ##########################################################

    def decode(self, topic, msg):
        # decode MQTT message
        used = True
        if topic == "T0/liv": # raw AD value
          from uservice import service
          gg = msg.split(" ")
          if (len(gg) >= 4):
            t0 = self.edgeTime;
            self.edgeTime = datetime.fromtimestamp(float(gg[0]))
            self.edge[0] = int(gg[1])
            self.edge[1] = int(gg[2])
            self.edge[2] = int(gg[3])
            self.edge[3] = int(gg[4])
            self.edge[4] = int(gg[5])
            self.edge[5] = int(gg[6])
            self.edge[6] = int(gg[7])
            self.edge[7] = int(gg[8])
            t1 = self.edgeTime;
            if self.edgeUpdCnt == 2:
              self.edgeInterval = (t1 -t0).total_seconds()*1000
            elif self.edgeUpdCnt > 2:
              self.edgeInterval = (self.edgeInterval * 99 + (t1 -t0).total_seconds()*1000) / 100
            self.edgeUpdCnt += 1
            # during rolling calibration, keep per-sensor max (line = brightest)
            if getattr(self, 'calibCollecting', False):
              for i in range(8):
                self.calibWhiteMax[i] = max(self.calibWhiteMax[i], self.edge[i])
            # self.print()
        elif topic == "T0/livn": # normalized after calibration range (0..1000)
          from uservice import service
          gg = msg.split(" ")
          if (len(gg) >= 4):
            t0 = self.edge_nTime;
            self.edge_nTime = datetime.fromtimestamp(float(gg[0]))
            self.edge_n[0] = int(gg[1])
            self.edge_n[1] = int(gg[2])
            self.edge_n[2] = int(gg[3])
            self.edge_n[3] = int(gg[4])
            self.edge_n[4] = int(gg[5])
            self.edge_n[5] = int(gg[6])
            self.edge_n[6] = int(gg[7])
            self.edge_n[7] = int(gg[8])
            t1 = self.edge_nTime;
            if self.edge_nUpdCnt == 2:
              self.edge_nInterval = (t1 -t0).total_seconds()*1000
            elif self.edge_nUpdCnt > 2:
              self.edge_nInterval = (self.edge_nInterval * 99 + (t1 -t0).total_seconds()*1000) / 100
            self.edge_nUpdCnt += 1
            # got new normalized values
            # debug save as a remark with timestamp
            # flog.writeDataString(f" {msg}");
            #
            # calculate line values based on new values
            self.LineDetect()
            #
            # use to control, if active
            if self.lineCtrl:
              self.followLine()
            # log relevant line sensor data (disabled for responsiveness test; re-enable to log)
            # if self.edge_nUpdCnt % self.flog_write_every_n == 0:
            #   flog.write()
            ## self.printn()
        elif topic == "T0/liw": # get white level
          from uservice import service
          gg = msg.split(" ")
          if (len(gg) >= 4):
            self.edge_n_wTime = datetime.fromtimestamp(float(gg[0]))
            self.edge_n_w[0] = int(gg[1])
            self.edge_n_w[1] = int(gg[2])
            self.edge_n_w[2] = int(gg[3])
            self.edge_n_w[3] = int(gg[4])
            self.edge_n_w[4] = int(gg[5])
            self.edge_n_w[5] = int(gg[6])
            self.edge_n_w[6] = int(gg[7])
            self.edge_n_w[7] = int(gg[8])
            self.edge_n_wUpdCnt += 1
            # self.printnw()
        else:
          used = False
        return used

    ##########################################################

    def LineDetect(self):
      total = 0
      posSum = 0
      high = int(1)
      # find levels (and average)
      # using normalised readings (0 (no reflection) to 1000 (calibrated white)))
      for i in range(8):
        total += self.edge_n[i] # for average
        if self.edge_n[i] > high:
          high = self.edge_n[i] # most bright value (floor level)
      self.high = high # most white level
      # print(f"% Edge (sedge.py):: {low}, {high} - what")
      # average white level
      self.average = total / 8.0
      # Per-sensor above-threshold state (each sensor checked against lineValidThreshold)
      self.linePattern = 0
      for i in range(8):
        self.sensorAboveThreshold[i] = self.edge_n[i] >= self.lineValidThreshold
        if self.sensorAboveThreshold[i]:
          self.linePattern |= 1 << (7 - i)
      self.sensorsAboveCount = sum(1 for b in self.sensorAboveThreshold if b)
      self.leftmostAboveIndex = None
      self.rightmostAboveIndex = None
      for i in range(8):
        if self.sensorAboveThreshold[i]:
          if self.leftmostAboveIndex is None:
            self.leftmostAboveIndex = i
          self.rightmostAboveIndex = i
      # Crossing detection: T-pattern (many sensors across) OR Y-pattern (wide footprint with centre gap)
      T_pattern = self.sensorsAboveCount >= self.crossingMinSensors
      Y_pattern = False
      if (self.sensorsAboveCount >= self.yMinSensors
          and self.leftmostAboveIndex is not None
          and self.rightmostAboveIndex is not None
          and (self.rightmostAboveIndex - self.leftmostAboveIndex) >= self.yMinSpan):
        # Centre gap = any of the middle sensors is below threshold
        Y_pattern = any(not self.sensorAboveThreshold[i] for i in self.yMidIndices)
      self.crossingLine = T_pattern or Y_pattern
      # Named state for status prints / overlays
      if self.sensorsAboveCount == 0:
        self.lineState = "no_line"
      elif self.crossingLine:
        self.lineState = "crossing"
      else:
        self.lineState = "line"
      # is line valid (high above threshold)
      self.lineValid = self.high >= self.lineValidThreshold
      # weighted center of mass (analog): sub-sensor resolution, smooth for PID
      # weight_i = intensity above background; position in -3.5..3.5
      floor = min(self.edge_n)
      sumW = 0.0
      sumPosW = 0.0
      for i in range(8):
        w = max(0, self.edge_n[i] - floor)
        sumW += w
        sumPosW += (i - 3.5) * w
      if sumW > 0:
        self.lineCenterWeighted = sumPosW / sumW
      # else sumW == 0 (avoid div by zero only). "No line" is indicated by lineValid elsewhere.
      self.lineCenterPattern = self.lineCenterFromPattern()
      # threshold-based left/right edges (kept for validity, display, crossing)
      if self.lineValid:
        posLeft = -3.5 # max left
        if self.edge_n[0] < self.lineValidThreshold:
          posLeft = -3 # between sensor 1 and 2 or more right
          for i in range(1,8):
            if self.edge_n[i] < self.lineValidThreshold:
              posLeft += 1;
            else:
              break;
        posRight = 3.5 # max right
        if self.edge_n[7] < self.lineValidThreshold:
          posRight = 3 # may be between sensor 8 and 7 or more left
          for i in range(1,8):
            if self.edge_n[7-i] < self.lineValidThreshold:
              posRight -= 1;
            else:
              break;
        self.posLeft = posLeft
        self.posRight = posRight
      else:
        # just keep old value
        pass
      #
      if self.lineValid and self.lineValidCnt < 20:
        self.lineValidCnt += 1
      elif not self.lineValid:
        if self.lineValidCnt > 0:
          self.lineValidCnt -= 1
        else:
          self.lineValidCnt = 0
      if self.crossingLine and self.crossingLineCnt < 20:
        self.crossingLineCnt += 1
      elif not self.crossingLine:
        self.crossingLineCnt -= 1
        if self.crossingLineCnt < 0:
          self.crossingLineCnt = 0
      pass
      # print(f"% Edge (sedge.py):: ({self.edge_n[0]} {self.edge_n[1]} {self.edge_n[2]} {self.edge_n[3]} {self.edge_n[4]} {self.edge_n[5]} {self.edge_n[6]}), high={self.high}, left={self.posLeft:.2f}, right={self.posRight:.2f}.")

    ##########################################################

    def lineCenterFromPattern(self):
      center = self.linePatternCenterTable[self.linePattern]
      if center is not None:
        return center
      active = [i for i, is_on in enumerate(self.sensorAboveThreshold) if is_on]
      if not active:
        return self.lineCenterWeighted
      return sum(self.linePatternFallbackCenters[i] for i in active) / len(active)

    ##########################################################

    def lineControl(self, velocity=None, refPosition=0, params="normal"):
      # Use tunable default speed when velocity not given (so mission can call lineControl() and speed is set here)
      if velocity is None:
        velocity = self.defaultLineVelocity
      self.velocity = velocity
      self.refPosition = refPosition
      self.lineRequestedParams = params
      # Any non-trivial speed (forward or reverse) enables line control.
      # 0 keeps the existing "off" behavior used throughout missions.
      self.lineCtrl = abs(velocity) > 0.001
      if not self.lineCtrl:
        self.lineY = 0.0
        self.lineY1 = 0.0
        self.lineIntegral = 0.0
        self.lineE_prev = None
        self.lineLostStartTime = 0.0
        self.lineReacquireSettleUntil = 0.0
        self.lineSettleActive = False
        self.lineSuppressInitialSettle = False
        self.lineWasValid = False
        self.recoveryMode = "none"
        self.recoveryPhaseStartTime = 0.0
        self.recoveryForwardStartTrip = None
        self.recoveryForwardStartTime = 0.0
        self.recoveryStraightActive = False
      else:
        # Start in the requested profile. Slow settle is only for later
        # line-loss -> reacquire events while line following is already active.
        self.lineWasValid = False
        self.lineReacquireSettleUntil = 0.0
        self.lineSuppressInitialSettle = True
      self.lineLastControlTime = 0.0
      if params in self.PARAM_SETS:
        for k, v in self.PARAM_SETS[params].items():
          setattr(self, k, v)
        self.lineIntegral = 0.0  # reset integral to avoid windup carry-over on mode switch
        self.lineE_prev = None
        self.lineE_filt_prev = None  # cold-start EWMA / avoid bogus D after mode switch

    ##########################################################

    def followLine(self):
      from uservice import service
      # Simple PID line follower. Sensor updates can arrive faster, but motor
      # commands are deliberately sent at a fixed, slower cadence.

      if not self.lineCtrl:
        return

      now = t.time()
      control_period = max(0.0, getattr(self, 'lineControlPeriod', 0.050))
      previous_control_time = self.lineLastControlTime
      if (self.lineLastControlTime > 0.0
          and control_period > 0.0
          and now - self.lineLastControlTime < control_period):
        return
      self.lineLastControlTime = now

      Tsec = control_period if previous_control_time <= 0.0 else max(0.001, now - previous_control_time)
      rawLineValid = self.lineValid
      trackingLineValid = rawLineValid or self.lineValidCnt >= self.lineRecentValidCnt
      # A single weak sample can dip below threshold while the line is still
      # under the sensor. Hold the last good center until confidence decays.
      weightedCenter = self.lineCenterWeighted
      measuredCenter = self.lineCenterPattern if self.lineUsePatternCenter else weightedCenter
      lineCenter = measuredCenter if rawLineValid else self.lastValidLineCenter
      e = self.refPosition - lineCenter

      if trackingLineValid and not self.lineWasValid:
        if self.lineSuppressInitialSettle:
          self.lineReacquireSettleUntil = 0.0
          self.lineSuppressInitialSettle = False
        else:
          self.lineReacquireSettleUntil = now + self.lineReacquireSettleTime
        self.lineIntegral = 0.0
        self.lineE_prev = None

      settle_params = getattr(self, 'lineReacquireSettleParams', 'slow')
      self.lineSettleActive = (
        trackingLineValid
        and self.lineReacquireSettleUntil > 0.0
        and now < self.lineReacquireSettleUntil
        and settle_params in self.PARAM_SETS
      )
      active_params = settle_params if self.lineSettleActive else getattr(self, 'lineRequestedParams', 'normal')
      active_profile = self.PARAM_SETS.get(active_params, {})
      lineKp = active_profile.get("lineKp", self.lineKp)
      lineKi = active_profile.get("lineKi", self.lineKi)
      lineKd = active_profile.get("lineKd", self.lineKd)

      pTerm = lineKp * e
      if rawLineValid:
        self.lineIntegral += e * Tsec
        if self.lineIntegral > self.lineIntegralLimit:
          self.lineIntegral = self.lineIntegralLimit
        elif self.lineIntegral < -self.lineIntegralLimit:
          self.lineIntegral = -self.lineIntegralLimit
      elif not trackingLineValid:
        self.lineIntegral = 0.0

      iTerm = lineKi * self.lineIntegral
      if trackingLineValid and self.lineE_prev is not None:
        dTerm = (e - self.lineE_prev) / Tsec
      else:
        dTerm = 0.0
      dContribution = lineKd * dTerm
      self.lineDerivativeGuardActive = False
      d_limit = getattr(self, 'lineDerivativeTermLimit', 0.0)
      if d_limit > 0.0:
        if dContribution > d_limit:
          dContribution = d_limit
          self.lineDerivativeGuardActive = True
        elif dContribution < -d_limit:
          dContribution = -d_limit
          self.lineDerivativeGuardActive = True

      piTerm = pTerm + iTerm
      self.u = piTerm + dContribution
      # D should damp the steering, not reverse it while the line is clearly
      # off-center. This avoids the back-and-forth kicks visible in the logs.
      no_reverse_error = getattr(self, 'lineNoReverseError', 0.0)
      if (trackingLineValid and abs(e) >= no_reverse_error
          and piTerm != 0.0 and self.u * piTerm < 0.0):
        dContribution = 0.0
        self.u = piTerm
        self.lineDerivativeGuardActive = True
      self.lineMinTurnActive = False
      min_turn_error = getattr(self, 'lineMinTurnError', 0.0)
      min_turn_rate = getattr(self, 'lineMinTurnRate', 0.0)
      if (trackingLineValid and min_turn_rate > 0.0
          and abs(e) >= min_turn_error and abs(self.u) < min_turn_rate):
        self.u = min_turn_rate if e > 0 else -min_turn_rate
        self.lineMinTurnActive = True
      self.lineY = self.u
      if self.lineY > self.lineYMax:
        self.lineY = self.lineYMax
      elif self.lineY < self.lineYMin:
        self.lineY = self.lineYMin
      self.lineY1 = self.lineY
      self.lineE1 = self.u

      if trackingLineValid:
        self.lineE_prev = e
        self.lineLostStartTime = 0.0
        self.recoveryMode = "none"
        self.recoveryPhaseStartTime = 0.0
        self.recoveryForwardStartTrip = None
        self.recoveryForwardStartTime = 0.0
        if rawLineValid:
          self.lastValidLineCenter = lineCenter
        self.recoveryStraightActive = False
        if e > 0:
          self.lastLineSide = 1
        elif e < 0:
          self.lastLineSide = -1
      else:
        self.lineE_prev = None

      if not trackingLineValid:
        if self.lineLostStartTime <= 0.0:
          self.lineLostStartTime = now
          self.recoveryMode = "turn"
          self.recoveryPhaseStartTime = now
          self.recoveryForwardStartTrip = None
          self.recoveryForwardStartTime = 0.0
        if self.recoveryMode in ("none", ""):
          self.recoveryMode = "turn"
          self.recoveryPhaseStartTime = now

        if self.recoveryMode == "turn":
          phase_time = now - self.recoveryPhaseStartTime
          if phase_time >= self.recoveryTurnTime:
            self.recoveryMode = "forward"
            self.recoveryPhaseStartTime = now
            self.recoveryForwardStartTime = now
            try:
              from spose import pose
              self.recoveryForwardStartTrip = pose.tripA
            except Exception:
              self.recoveryForwardStartTrip = None

        if self.recoveryMode == "forward":
          forward_done = False
          try:
            from spose import pose
            if self.recoveryForwardStartTrip is None:
              self.recoveryForwardStartTrip = pose.tripA
            forward_done = abs(pose.tripA - self.recoveryForwardStartTrip) >= self.recoveryForwardDistance
          except Exception:
            forward_time = now - self.recoveryForwardStartTime
            forward_done = forward_time * max(abs(self.recoveryForwardVelocity), 0.001) >= self.recoveryForwardDistance

          if forward_done:
            self.recoveryMode = "turn"
            self.recoveryPhaseStartTime = now
            self.recoveryForwardStartTrip = None
            self.recoveryForwardStartTime = 0.0

        if self.recoveryMode == "forward":
          speed_sign = 1.0 if self.velocity >= 0.0 else -1.0
          sent_velocity = speed_sign * self.recoveryForwardVelocity
          sent_turn = 0.0
          self.recoveryStraightActive = False
        else:
          sent_velocity = self.recoveryVelocity
          sent_turn = self.recoveryTurnRate * (self.lastLineSide if self.lastLineSide != 0 else 1)
          self.recoveryStraightActive = False
        if sent_turn > self.lineYMax:
          sent_turn = self.lineYMax
        elif sent_turn < self.lineYMin:
          sent_turn = self.lineYMin
        self.lineY = sent_turn
      else:
        sent_velocity = self.velocity
        if self.lineSettleActive and "lineVelocity" in active_profile:
          speed_sign = 1.0 if self.velocity >= 0.0 else -1.0
          sent_velocity = speed_sign * min(abs(self.velocity), abs(active_profile["lineVelocity"]))
        sent_turn = self.lineY

      if sent_velocity < 0.0:
        sent_turn = -sent_turn
      par = f"rc {sent_velocity:.3f} {sent_turn:.3f} {now}"
      service.send("robobot/cmd/ti", par)

      if self.print_follow_line_block and (getattr(service.args, 'test', False) or not getattr(service.args, 'silent', True)):
        enabled = set(self.print_follow_line_fields)
        parts = []
        if 'livn' in enabled:
          norm = " ".join(f"{self.edge_n[i]:4d}" for i in range(8))
          parts.append(f"livn [{norm}]")
        if 'pattern' in enabled:
          parts.append(f"pattern={self.linePattern:08b}")
        if 'high' in enabled:
          parts.append(f"high={self.high:4d}")
        if 'valid' in enabled:
          parts.append(f"valid={str(self.lineValid):5}")
        if 'validCnt' in enabled:
          parts.append(f"validCnt={self.lineValidCnt:2d}")
        if 'state' in enabled:
          parts.append(f"state={self.lineState}")
        if 'aboveCnt' in enabled:
          parts.append(f"aboveCnt={self.sensorsAboveCount}")
        if 'crossingCnt' in enabled:
          parts.append(f"crossingCnt={self.mission_crossing_count}")
        if 'leftMost' in enabled:
          parts.append(f"leftMost={self.leftmostAboveIndex if self.leftmostAboveIndex is not None else '-'}")
        if 'rightMost' in enabled:
          parts.append(f"rightMost={self.rightmostAboveIndex if self.rightmostAboveIndex is not None else '-'}")
        if 'center' in enabled:
          parts.append(f"center={lineCenter:5.2f}")
        if 'wCenter' in enabled:
          parts.append(f"wCenter={weightedCenter:5.2f}")
        if any(k in enabled for k in ('e', 'p', 'i', 'd', 'u', 'y', 'dGuard', 'minTurn', 'recMode', 'recStraight', 'rc', 'lastLineSide')):
          parts.append("|")
        if 'e' in enabled:
          parts.append(f"e={e:6.3f}")
        if 'p' in enabled:
          parts.append(f"p={pTerm:6.3f}")
        if 'i' in enabled:
          parts.append(f"i={iTerm:6.3f}")
        if 'd' in enabled:
          parts.append(f"d={dContribution:6.3f}")
        if 'u' in enabled:
          parts.append(f"u={self.u:6.3f}")
        if 'y' in enabled:
          parts.append(f"y={self.lineY:6.3f}")
        if 'dGuard' in enabled:
          parts.append(f"dGuard={str(self.lineDerivativeGuardActive):5}")
        if 'minTurn' in enabled:
          parts.append(f"minTurn={str(self.lineMinTurnActive):5}")
        if 'settle' in enabled:
          parts.append(f"settle={str(self.lineSettleActive):5}")
        if 'recMode' in enabled:
          parts.append(f"recMode={self.recoveryMode}")
        if 'recStraight' in enabled:
          parts.append(f"recStraight={str(self.recoveryStraightActive):5}")
        if 'rc' in enabled:
          parts.append(f"-> rc {sent_velocity:.3f} {sent_turn:.3f}")
        if 'lastLineSide' in enabled:
          parts.append(f"lastSide={self.lastLineSide:+d}")
        if parts:
          print("% line: " + " ".join(parts))
      self.lineWasValid = trackingLineValid
      return  # Legacy PID/filter implementation below is intentionally bypassed.
      # some parameters depend on sample time, adjust
      # print(f"LineCtrl:: sample time {self.edge_nInterval}")
      if abs(self.edge_nInterval - self.edgeIntervalSetup) > 2.0: # ms
        self.PIDrecalculate()
        self.edgeIntervalSetup = self.edge_nInterval
      # use weighted center of mass (analog) for error; keep previous when sum(weight)==0
      lineCenter = self.lineCenterWeighted
      e = self.refPosition - lineCenter
      # line center to the right -> negative e -> negative turn corrects when driving forward
      # Fixed sample period: makes Kd and Ki immune to MQTT arrival jitter and the
      # EWMA warm-up on edge_nInterval. The measured interval is checked once for
      # gross drift from nominal so a firmware cadence change can't silently rescale D/I.
      Tsec = self.TS_NOMINAL
      if (self.edge_nInterval > 0
          and self.edge_nUpdCnt > 100
          and not getattr(self, '_ts_warned', False)
          and abs(self.edge_nInterval / 1000.0 - self.TS_NOMINAL) / self.TS_NOMINAL
              > self.TS_DRIFT_WARN_FRAC):
        if not service.is_quiet():
          print(f"% sedge: measured Ts {self.edge_nInterval:.1f} ms differs "
                f">{self.TS_DRIFT_WARN_FRAC*100:.0f}% from TS_NOMINAL "
                f"{self.TS_NOMINAL*1000:.0f} ms — review TS_NOMINAL")
        self._ts_warned = True

      # Reset integral when line lost (avoid windup during search); reset D filter state.
      if not self.lineValid:
        self.lineIntegral = 0.0
        self.lineE_prev = e
        self.lineE_filt = e
        self.lineE_filt_prev = None
      elif self.lineDerivativeBeta > 0.0:
        # EWMA on e for D only — reduces Kd · noise from raw (e[k]-e[k-1])/dt
        if self.lineE_filt_prev is None:
          self.lineE_filt = e
        else:
          b = self.lineDerivativeBeta
          self.lineE_filt = b * self.lineE_filt + (1.0 - b) * e
      else:
        self.lineE_filt = e

      # Derivative: (e_filt - e_filt_prev) / Tsec; zero when e_filt_prev is None.
      if self.lineE_filt_prev is not None:
        dTerm = (self.lineE_filt - self.lineE_filt_prev) / Tsec
      else:
        dTerm = 0.0
      self.lineE_filt_prev = self.lineE_filt
      self.lineE_prev = e

      # Integral: accumulate with clamp (anti-windup)
      if self.lineValid:
        self.lineIntegral += e * Tsec
        if self.lineIntegral > self.lineIntegralLimit:
          self.lineIntegral = self.lineIntegralLimit
        elif self.lineIntegral < -self.lineIntegralLimit:
          self.lineIntegral = -self.lineIntegralLimit

      # PID output (u = P + I + D), clamp, then low-pass smooth.
      pTerm = self.lineKp * e
      iTerm = self.lineKi * self.lineIntegral
      piTerm = pTerm + iTerm
      dContribution = self.lineKd * dTerm
      dLimit = abs(getattr(self, 'lineDerivativeTermLimit', 0.0))
      if dLimit > 0.0:
        if dContribution > dLimit:
          dContribution = dLimit
        elif dContribution < -dLimit:
          dContribution = -dLimit
      self.lineDerivativeGuardActive = False
      no_reverse_error = abs(getattr(self, 'lineNoReverseError', 0.0))
      d_guarded = dContribution
      if (self.lineValid and no_reverse_error > 0.0 and abs(e) >= no_reverse_error
          and piTerm != 0.0 and piTerm * (piTerm + dContribution) < 0.0):
        d_guarded = 0.0
        self.lineDerivativeGuardActive = True
      self.u = piTerm + d_guarded
      raw_y = self.u
      if raw_y > self.lineYMax:
        raw_y = self.lineYMax
      elif raw_y < self.lineYMin:
        raw_y = self.lineYMin
      turn_limit = abs(getattr(self, 'lineTurnLimit', 0.0))
      if self.lineValid and turn_limit > 0.0:
        if raw_y > turn_limit:
          raw_y = turn_limit
        elif raw_y < -turn_limit:
          raw_y = -turn_limit
      # low-pass filter: blend new clamped output with previous output
      desired_y = self.lineOutputAlpha * raw_y + (1.0 - self.lineOutputAlpha) * self.lineY1
      # If the line is still visible but far from the center, do not allow a
      # derivative spike or output smoothing to nearly cancel the steering command.
      self.lineMinTurnActive = False
      min_turn_error = abs(getattr(self, 'lineMinTurnError', 0.0))
      min_turn_rate = abs(getattr(self, 'lineMinTurnRate', 0.0))
      if self.lineValid and min_turn_error > 0.0 and min_turn_rate > 0.0 and abs(e) >= min_turn_error:
        error_sign = 1.0 if e > 0.0 else -1.0 if e < 0.0 else 0.0
        if error_sign != 0.0 and abs(desired_y) < min_turn_rate:
          desired_y = error_sign * min_turn_rate
          self.lineMinTurnActive = True
      self.lineTurnSlewActive = False
      slew_rate = abs(getattr(self, 'lineTurnSlewRate', 0.0))
      if self.lineValid and slew_rate > 0.0:
        max_step = slew_rate * Tsec
        delta_y = desired_y - self.lineY1
        if delta_y > max_step:
          desired_y = self.lineY1 + max_step
          self.lineTurnSlewActive = True
        elif delta_y < -max_step:
          desired_y = self.lineY1 - max_step
          self.lineTurnSlewActive = True
      if self.lineValid and turn_limit > 0.0:
        if desired_y > turn_limit:
          desired_y = turn_limit
        elif desired_y < -turn_limit:
          desired_y = -turn_limit
      self.lineY = desired_y
      self.lineE1 = self.u
      self.lineY1 = self.lineY
      # remember for recovery: which side the line was on when last valid
      if self.lineValid:
        if e > 0:
          self.lastLineSide = 1   # line was left; positive turn searches/corrects left
        elif e < 0:
          self.lastLineSide = -1  # line was right; negative turn searches/corrects right
        # e == 0: keep previous lastLineSide
      # Recovery: when line lost, turn toward last line side (optional forward creep)
      self.lineVelocityAdjusted = False
      if not self.lineValid and self.lineCtrl:
        recovery_turn = self.recoveryTurnRate * (self.lastLineSide if self.lastLineSide != 0 else 1)
        if recovery_turn > self.lineYMax:
          recovery_turn = self.lineYMax
        elif recovery_turn < self.lineYMin:
          recovery_turn = self.lineYMin
        self.lineY = recovery_turn
        sent_velocity = self.recoveryVelocity
        sent_turn = self.lineY
      else:
        sent_velocity = self.velocity
        sent_turn = self.lineY
        if self.lineValid and self.lineCtrl and abs(sent_velocity) > 0.001:
          slowdown_error = abs(getattr(self, 'lineSlowdownError', 0.0))
          min_velocity = abs(getattr(self, 'lineVelocityMin', abs(sent_velocity)))
          base_velocity = abs(sent_velocity)
          if slowdown_error > 0.0 and min_velocity < base_velocity:
            slowdown = min(1.0, abs(e) / slowdown_error)
            target_velocity = base_velocity - (base_velocity - min_velocity) * slowdown
            if target_velocity < base_velocity - 0.001:
              self.lineVelocityAdjusted = True
            sent_velocity = (1.0 if sent_velocity > 0.0 else -1.0) * target_velocity
      # Reverse line-following: the front-mounted sensor leads when going
      # forward, but trails when going backward — so the same error must
      # produce the opposite ω to keep the sensor over the line.
      if sent_velocity < 0.0:
        sent_turn = -sent_turn
      par = f"rc {sent_velocity:.3f} {sent_turn:.3f} {t.time()}"
      service.send("robobot/cmd/ti", par)
      # test print only when line-following (interval and CLI gated; see print_follow_line_block at top)
      if self.print_follow_line_block and (getattr(service.args, 'test', False) or not getattr(service.args, 'silent', True)) and self.edge_nUpdCnt > 0 and self.edge_nUpdCnt % self.follow_line_print_every_n == 0:
        enabled = set(self.print_follow_line_fields)
        parts = []
        if 'livn' in enabled:
          norm = " ".join(f"{self.edge_n[i]:4d}" for i in range(8))
          parts.append(f"livn [{norm}]")
        if 'avg' in enabled:
          parts.append(f"avg={self.average:6.0f}")
        if 'high' in enabled:
          parts.append(f"high={self.high:4d}")
        if 'valid' in enabled:
          parts.append(f"valid={str(self.lineValid):5}")
        if 'validCnt' in enabled:
          parts.append(f"validCnt={self.lineValidCnt:2d}")
        if 'posL' in enabled:
          parts.append(f"posL={self.posLeft:5.2f}")
        if 'posR' in enabled:
          parts.append(f"posR={self.posRight:5.2f}")
        if 'center' in enabled:
          parts.append(f"center={lineCenter:5.2f}")
        if 'cross' in enabled:
          parts.append(f"cross={str(self.crossingLine):5}")
        if 'state' in enabled:
          parts.append(f"state={self.lineState}")
        if 'aboveCnt' in enabled:
          parts.append(f"aboveCnt={self.sensorsAboveCount}")
        if 'crossingCnt' in enabled:
          parts.append(f"crossingCnt={self.mission_crossing_count}")
        if 'leftMost' in enabled:
          parts.append(f"leftMost={self.leftmostAboveIndex if self.leftmostAboveIndex is not None else '-'}")
        if 'rightMost' in enabled:
          parts.append(f"rightMost={self.rightmostAboveIndex if self.rightmostAboveIndex is not None else '-'}")
        if any(k in enabled for k in ('e', 'p', 'i', 'd', 'dTerm', 'integral', 'u', 'dGuard', 'minTurn', 'slew', 'y', 'vAdj', 'rc', 'lastLineSide')):
          parts.append("|")
        if 'e' in enabled:
          parts.append(f"e={e:6.3f}")
        if 'p' in enabled:
          parts.append(f"p={pTerm:6.3f}")
        if 'i' in enabled:
          parts.append(f"i={iTerm:6.3f}")
        if 'd' in enabled:
          parts.append(f"d={d_guarded:6.3f}")
        if 'dTerm' in enabled:
          parts.append(f"dTerm={dTerm:6.3f}")
        if 'integral' in enabled:
          parts.append(f"integral={self.lineIntegral:6.3f}")
        if 'u' in enabled:
          parts.append(f"u={self.u:6.3f}")
        if 'dGuard' in enabled:
          parts.append(f"dGuard={str(self.lineDerivativeGuardActive):5}")
        if 'minTurn' in enabled:
          parts.append(f"minTurn={str(self.lineMinTurnActive):5}")
        if 'slew' in enabled:
          parts.append(f"slew={str(self.lineTurnSlewActive):5}")
        if 'y' in enabled:
          parts.append(f"y={self.lineY:6.3f}")
        if 'vAdj' in enabled:
          parts.append(f"vAdj={str(self.lineVelocityAdjusted):5}")
        if 'rc' in enabled:
          parts.append(f"-> rc {sent_velocity:.3f} {sent_turn:.3f}")
        if 'lastLineSide' in enabled:
          parts.append(f"lastSide={self.lastLineSide:+d}")
        if parts:
          print("% line: " + " ".join(parts))

    ##########################################################

    def PIDrecalculate(self):
      from uservice import service
      if not service.is_quiet():
        print(f"LineCtrl:: PIDrecalculate: T={self.edgeIntervalSetup:.2f} -> {self.edge_nInterval:.2f} ms")
      Tsec = self.edge_nInterval/1000
      self.tauP2pT = self.lineTauP * 2.0 + Tsec
      self.tauP2mT = self.lineTauP * 2.0 - Tsec
      self.tauZ2pT = self.lineTauZ * 2.0 + Tsec
      self.tauZ2mT = self.lineTauZ * 2.0 - Tsec
      if not service.is_quiet():
        print(f"%% Lead: tauZ {self.lineTauZ:.3f} sec, tauP = {self.lineTauP:.3f} sec, T = {self.edge_nInterval:.3f} ms\n")
        print(f"%%       tauZ2pT = {self.tauZ2pT:.4f}, tauZ2mT = {self.tauZ2mT:.4f}, tauP2pT = {self.tauP2pT:.4f}, tauP2mT = {self.tauP2mT:.4f}")


    ##########################################################

    def terminate(self):
      from uservice import service
      self.need_data = False
      if not service.is_quiet():
        print("% Edge (sedge.py):: turn off line sensor")
      service.send(self.topicCmdT0, "lip 0")
      if not service.is_quiet():
        print("% Edge (sedge.py):: terminated")
      pass

    ##########################################################

    def paint(self, img):
      h, w, ch = img.shape
      pl = int(h - h/4) # base position bottom (most positive y)
      st = int(w/10) # distance between sensors
      gh = int(h/2) # graph height
      x = st # base position left
      y = pl
      dtuGreen = (0x35, 0x88, 0) # BGR
      dtuBlue = (0xea, 0x3e, 0x2f)
      dtuRed = (0x00, 0x00, 0x99)
      dtuPurple = (0x8e, 0x23, 0x77)
      # paint baseline
      cv.line(img, (x,y), (int(x + 7*st), int(y)), dtuGreen, thickness=1, lineType=8)
      # paint calibrated white line (top)
      cv.line(img, (x,int(y-gh)), (int(x + 7*st), int(y-gh)), dtuGreen, thickness=1, lineType=8)
      # paint threshold line for line valid
      cv.line(img, (x,int(y-gh*self.lineValidThreshold/1000.0)), (int(x + 7*st), int(y-gh*self.lineValidThreshold/1000.0)), dtuBlue, thickness=1, lineType=4)
      # draw current sensor readings
      for i in range(8):
        y = int(pl - self.edge_n[i]/1000 * gh)
        cv.drawMarker(img, (x,y), dtuRed, markerType=cv.MARKER_STAR, thickness=2, line_type=8, markerSize = 10)
        x += st
      # paint line position
      from uservice import service
      if not service.is_quiet():
        print(f" Edge::paint: posLeft {self.posLeft}, right {self.posRight}")
      pixP = int((self.posLeft + 4.5)*st)
      cv.line(img, (pixP, int(pl)), (pixP, int(pl-gh)), dtuRed, thickness=3, lineType=4)
      pixP = int((self.posRight + 4.5)*st)
      cv.line(img, (pixP, int(pl)), (pixP, int(pl-gh)), dtuGreen, thickness=3, lineType=4)
      # paint low line position
      pixL = pl - int(gh * 0.0)
      cv.line(img, (st, pixL), (st*8, pixL), dtuRed, thickness=1, lineType=4)
      # some axis marking
      cv.putText(img, "Left", (st,pl - 2), cv.FONT_HERSHEY_PLAIN, 1, dtuPurple, thickness=2)
      cv.putText(img, "Right", (int(st+6*st),pl - 2), cv.FONT_HERSHEY_PLAIN, 1, dtuPurple, thickness=2)
      cv.putText(img, "White (1000)", (int(st),pl - gh - 2), cv.FONT_HERSHEY_PLAIN, 1, dtuPurple, thickness=2)
      if self.crossingLine:
        cv.putText(img, "Crossing", (int(st),int(pl - 20)), cv.FONT_HERSHEY_PLAIN, 1, dtuRed, thickness=2)


# create the data object
edge = SEdge()

