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
    defaultLineVelocity = 0.5   # m/s (e.g. 0.15 = slower, 0.25 = faster)
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
    # PID gains (turn rate rad/s; integral in error·s, derivative in error/s)
    lineKp = 0.4  # was 0.8 — reduced to soften proportional jerk
    lineKi = 0.0
    lineKd = 0.1  # was 0.05 — slightly more damping
    lineIntegralLimit = 2.0    # clamp integral to ±this (error·s) to limit windup
    # Sample period used by the PID (s). T0/livn is published every ~10 ms
    # ("sub livn 3"), so the controller assumes a fixed dt to keep Kd and Ki
    # tuning independent of MQTT arrival jitter / EWMA warm-up. The measured
    # interval (edge_nInterval) is kept only for diagnostics + drift warning.
    TS_NOMINAL = 0.010
    TS_DRIFT_WARN_FRAC = 0.30  # one-shot warn if measured Ts drifts >this fraction from nominal
    # Output saturation (turn rate rad/s)
    # Keep ≤ speed / (half_track_width) so inner wheel never reverses.
    # At 0.25 m/s, ~0.13 m track: max safe ≈ 1.9 rad/s → use 1.5 for margin.
    lineYMax = 3.0
    lineYMin = -3.0
    # Low-pass smoothing on the turn-rate output (0 = no filter, 1 = frozen)
    # Absorbs sensor noise between 30 ms livn updates without adding much lag.
    lineOutputAlpha = 0.4   # new = alpha*raw + (1-alpha)*prev; lower = smoother
    # Named PID parameter sets — select via lineControl(params="slow"|"normal")
    # "slow" starts as a copy of "normal"; tune independently on the robot.
    PARAM_SETS = {
        "normal": dict(lineKp=0.5, lineKi=0.0, lineKd=0.15,
                       lineIntegralLimit=2.0, lineOutputAlpha=0.3),
        "slow":   dict(lineKp=0.25, lineKi=0.0, lineKd=0.2,
                       lineIntegralLimit=2.0, lineOutputAlpha=0.4),
    }
    # Recovery when line lost (A1/A4: turn toward last line side)
    recoveryTurnRate = 0.5   # rad/s when turning to find line during recovery
    recoveryVelocity = 0.2   # m/s forward during recovery (0 = turn in place; small value = creep forward while turning)
    recovery_timeout_s = 5.0 # mission stops after this many seconds without line (recovery runs until then)
    # Legacy lead (unused when using PID; for optional re-enable later)
    lineTauZ = 0.8
    lineTauP = 0.25
    # --- Follow-line block print (gated by --test/--silent). Print + flog next to each other: ---
    print_follow_line_block = True  # set True to re-enable; False = no print in hot path (test responsiveness)
    follow_line_print_every_n = 10  # print one line every Nth livn update when enabled
    flog_write_every_n = 1         # flog.write() every Nth livn update (appends line/sensor log line to file)
    
    print_follow_line_fields = (
        'center', 'state', 'aboveCnt', 'crossingCnt', 'leftMost', 'rightMost', 'e', 'y'
    )

    # Available fields (copy into tuple above; order = order on screen; single line):
    #   livn, avg, high, valid, validCnt, posL, posR, center, cross, state, aboveCnt, crossingCnt, leftMost, rightMost, e, p, i, d, dTerm, integral, u, y, rc, lastLineSide
    #   livn = normalized sensor values [8]; avg = 8-sensor avg 
    #   high = peak; valid = line valid; validCnt = 0..20
    #   posL, posR = line position -3.5..3.5 left/right edge (sensors 1..8); controller uses center for error
    #   center = weighted center (-3.5..3.5); cross = crossing detected 
    #   e = error (ref - center)
    #   p, i, d = P, I, D terms (rad/s);
    #   dTerm = raw (e-e_prev)/dt in error/s; d = Kd*dTerm  
    #   integral = accumulated error·s before Ki; 
    #   u = PID before clamp; y = turn rate sent in rc
    #   rc = command sent
    #   lastLineSide = -1/0/+1 (line was left/unknown/right when last valid); recovery turns this way
    #   state = lineState ("no_line"|"line"|"crossing"); aboveCnt = sensorsAboveCount (0..8)
    #   crossingCnt = mission crossing counter (set by driveToLine; number of crossings left)
    #   leftMost, rightMost = first/last sensor index 0..7 above threshold (- when none)
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
    lineE_prev = None  # previous error for derivative; None on first sample or after line lost
    lineE1 = 0.0
    lineY1 = 0.0
    lineY = 0.0  # control output (rad/s), clamped and smoothed
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
      for i in range(8):
        self.sensorAboveThreshold[i] = self.edge_n[i] >= self.lineValidThreshold
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

    def lineControl(self, velocity=None, refPosition=0, params="normal"):
      # Use tunable default speed when velocity not given (so mission can call lineControl() and speed is set here)
      if velocity is None:
        velocity = self.defaultLineVelocity
      self.velocity = velocity
      self.refPosition = refPosition
      # velocity 0 (or negative) is turning off line control
      self.lineCtrl = velocity > 0.001
      if params in self.PARAM_SETS:
        for k, v in self.PARAM_SETS[params].items():
          setattr(self, k, v)
        self.lineIntegral = 0.0  # reset integral to avoid windup carry-over on mode switch

    ##########################################################

    def followLine(self):
      from uservice import service
      # some parameters depend on sample time, adjust
      # print(f"LineCtrl:: sample time {self.edge_nInterval}")
      if abs(self.edge_nInterval - self.edgeIntervalSetup) > 2.0: # ms
        self.PIDrecalculate()
        self.edgeIntervalSetup = self.edge_nInterval
      # use weighted center of mass (analog) for error; keep previous when sum(weight)==0
      lineCenter = self.lineCenterWeighted
      e = self.refPosition - lineCenter
      # line center to the right -> positive e -> robot too far left -> negative turn corrects
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

      # Reset integral when line lost (avoid windup during search)
      if not self.lineValid:
        self.lineIntegral = 0.0
        self.lineE_prev = e

      # Derivative: (e - e_prev) / Tsec; zero on first sample after line recovery.
      if self.lineE_prev is not None:
        dTerm = (e - self.lineE_prev) / Tsec
      else:
        dTerm = 0.0
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
      self.u = pTerm + iTerm + self.lineKd * dTerm
      raw_y = self.u
      if raw_y > self.lineYMax:
        raw_y = self.lineYMax
      elif raw_y < self.lineYMin:
        raw_y = self.lineYMin
      # low-pass filter: blend new clamped output with previous output
      self.lineY = self.lineOutputAlpha * raw_y + (1.0 - self.lineOutputAlpha) * self.lineY1
      self.lineE1 = self.u
      self.lineY1 = self.lineY
      # remember for recovery: which side the line was on when last valid
      if self.lineValid:
        if e > 0:
          self.lastLineSide = 1   # line was to the right (robot left of line)
        elif e < 0:
          self.lastLineSide = -1  # line was to the left
        # e == 0: keep previous lastLineSide
      # Recovery: when line lost, turn toward last line side (optional forward creep)
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
        if any(k in enabled for k in ('e', 'p', 'i', 'd', 'dTerm', 'integral', 'u', 'y', 'rc', 'lastLineSide')):
          parts.append("|")
        if 'e' in enabled:
          parts.append(f"e={e:6.3f}")
        if 'p' in enabled:
          parts.append(f"p={pTerm:6.3f}")
        if 'i' in enabled:
          parts.append(f"i={iTerm:6.3f}")
        if 'd' in enabled:
          parts.append(f"d={self.lineKd*dTerm:6.3f}")
        if 'dTerm' in enabled:
          parts.append(f"dTerm={dTerm:6.3f}")
        if 'integral' in enabled:
          parts.append(f"integral={self.lineIntegral:6.3f}")
        if 'u' in enabled:
          parts.append(f"u={self.u:6.3f}")
        if 'y' in enabled:
          parts.append(f"y={self.lineY:6.3f}")
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

